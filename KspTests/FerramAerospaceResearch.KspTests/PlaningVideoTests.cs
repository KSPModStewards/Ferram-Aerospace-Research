using System;
using System.Collections;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// A video of a V-bottomed hull coming up onto the plane under FAR's water model: frames/NNNNN.jpg and video.csv in
    /// the results folder, one frame per 1/30 s of game time (Time.captureFramerate), for ffmpeg to join.
    /// </summary>
    public static class PlaningVideoTests
    {
        private const string VHull = "Planing V Hull";

        /// <summary>
        /// The Planing V Hull (crafts/planing_hull.py v: one CustomProceduralParts Spine, a 20 deg V bottom with vertical
        /// sides, beam 2 m, 8 m long, the bow raked) made 4 t with its CG 3 m ahead of the transom and 0.5 m above the
        /// keel, towed through its CG as the planing tow tests are (heave and pitch free), from rest to 20 m/s: 3 s at
        /// rest, 36 s of steady acceleration, 5 s at 20 m/s. The flight camera looks at it from the side, level with the
        /// water, the UI hidden; an overlay gives the speed, beam Froude number, trim, resistance over weight and how
        /// the weight is carried (dynamic lift, buoyancy). Parameters: hydro (false: FAR's old water forces), mass,
        /// lcg, cgHeight, topSpeed, rampSec, camDistance (m), camPitch (deg).
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = @"..\..\..\CustomProceduralParts\GameData\CustomProceduralParts",
                StartsAt = StartState.Flight, Craft = VHull, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Video_VHullPlaning(TestContext ctx)
        {
            const double beam = 2, length = 8, depth = 2;
            double massT = ctx.ParamDouble("mass", 4.0), lcg = ctx.ParamDouble("lcg", 3.0), cgHeight = ctx.ParamDouble("cgHeight", 0.5);
            double top = ctx.ParamDouble("topSpeed", 20), ramp = ctx.ParamDouble("rampSec", 36);
            bool hydroWas = Settings.FARSettings.Hydrodynamics;
            string hydro = ctx.Param("hydro");
            if (!string.IsNullOrEmpty(hydro))
                Settings.FARSettings.Hydrodynamics = bool.Parse(hydro);
            string model = Settings.FARSettings.Hydrodynamics ? "FAR PartHydrodynamics" : "FAR legacy water forces";
            ctx.Record("model", model);

            yield return Wait.Seconds(5);
            Vessel v = FlightGlobals.ActiveVessel;
            Part hull = v.rootPart;
            Assert.True(v.parts.Count == 1, $"the hull is one part ({v.parts.Count})");
            foreach (PartResource r in hull.Resources)
                r.amount = 0;
            yield return Wait.Frames(2);
            PartResource lf = hull.Resources.Get("LiquidFuel");
            double add = (massT - hull.mass) / lf.info.density;
            Assert.True(add >= 0 && add <= lf.maxAmount, $"{massT} t reachable (dry {hull.mass:F3} t)");
            lf.amount = add;
            yield return Wait.Frames(2);

            bool crashWas = CheatOptions.NoCrashDamage;
            GameObject recorderGo = null;
            try
            {
                CheatOptions.NoCrashDamage = true;
                v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                yield return FarFlightProbe.Place(Far.At(0, 600, 0, "video", speedMs: 5));
                v = FlightGlobals.ActiveVessel;
                CelestialBody body = v.mainBody;
                Rigidbody rb = hull.rb;
                Vector3 up = v.upAxis;
                Vector3 h = Vector3.ProjectOnPlane(hull.transform.up, up).normalized;

                // keel down: of the part's +-x and +-z, the side whose extreme is a line (the V's point), not a face
                Vector3 keel = KeelAxis(hull);
                Vector3 yW = h, keelW = -up;
                Vector3 xW, zW;
                if (Mathf.Abs(keel.z) > 0.5f)
                {
                    zW = keel.z > 0 ? keelW : -keelW;
                    xW = Vector3.Cross(yW, zW);
                }
                else
                {
                    xW = keel.x > 0 ? keelW : -keelW;
                    zW = Vector3.Cross(xW, yW);
                }
                v.SetRotation(Quaternion.LookRotation(zW, yW));
                ctx.Record("hull", $"keel along local {keel}, mass {v.GetTotalMass():F3} t, C_Delta {v.GetTotalMass() / (body.oceanDensity * beam * beam * beam):F3}");

                hull.CoMOffset = new Vector3(0, (float)(lcg - length / 2), 0) + keel * (float)(depth / 2 - cgHeight);
                rb.centerOfMass = hull.CoMOffset;
                yield return new WaitForFixedUpdate();
                v.SetPosition(v.vesselTransform.position + (Vector3d)up * (0.6 - v.altitude));
                foreach (Part p in v.parts)
                {
                    p.rb.velocity = Vector3.zero;
                    p.rb.angularVelocity = Vector3.zero;
                }

                // camera: level with the water, abeam; its heading found by trying two (camHdg's zero and sense
                // depend on the camera mode)
                FlightCamera cam = FlightCamera.fetch;
                float camPitch = (float)(ctx.ParamDouble("camPitch", 4) * Mathf.Deg2Rad), camDist = (float)ctx.ParamDouble("camDistance", 24);
                cam.minPitch = -0.5f;
                cam.SetDistanceImmediate(camDist);
                Vector3 side = Vector3.Cross(up, h).normalized;
                Vector3 look = -side;   // the camera on the hull's side, looking across it
                float a0 = 0, a1 = 0;
                for (int pass = 0; pass < 2; pass++)
                {
                    float hdg = pass == 0 ? 0f : 0.5f;
                    for (int i = 0; i < 10; i++)
                    {
                        cam.camHdg = hdg;
                        cam.camPitch = camPitch;
                        yield return null;
                    }
                    Vector3 f = Vector3.ProjectOnPlane(cam.transform.forward, up).normalized;
                    float a = Vector3.SignedAngle(look, f, up) * Mathf.Deg2Rad;
                    if (pass == 0) a0 = a;
                    else a1 = a;
                }
                float slope = Mathf.DeltaAngle(a0 * Mathf.Rad2Deg, a1 * Mathf.Rad2Deg) * Mathf.Deg2Rad / 0.5f;
                float camHdg = Math.Abs(slope) > 0.2f ? -a0 / slope : 0;
                ctx.Record("camera", $"heading {camHdg * Mathf.Rad2Deg:F1} deg (slope {slope:F2}), pitch {camPitch * Mathf.Rad2Deg:F1} deg, {camDist} m");

                GameEvents.onHideUI.Fire();
                string frames = Path.Combine(ctx.ResultsDir, "frames");
                Directory.CreateDirectory(frames);
                recorderGo = new GameObject("PlaningVideoRecorder");
                Recorder rec = recorderGo.AddComponent<Recorder>();
                rec.Dir = frames;
                Time.captureFramerate = 30;

                var csv = new StringBuilder("t,V,C_V,trim_deg,R_over_Delta,lift_share,buoy_share,hydroUp_kN,buoyUp_kN,tow_kN,frame\n");
                var fixedWait = new WaitForFixedUpdate();
                double t0 = Planetarium.GetUniversalTime(), setAlong = 0, rEma = 0, liftEma = 0, buoyEma = 1;
                bool restRecorded = false;
                double weight = v.GetTotalMass() * body.GeeASL * 9.80665, total = 3 + ramp + 5;
                var aero = hull.Modules.GetModule<FARAeroComponents.FARAeroPartModule>();
                while (true)
                {
                    double t = Planetarium.GetUniversalTime() - t0;
                    if (t > total)
                        break;
                    double V = t < 3 ? 0 : Math.Min(top, top * (t - 3) / ramp);
                    double dt = TimeWarp.fixedDeltaTime, m = v.GetTotalMass();
                    Vector3 vel = rb.velocity;
                    double towKN = m * (setAlong - Vector3.Dot(vel, h)) / dt;
                    rb.velocity = up * Vector3.Dot(vel, up) + h * (float)V;
                    setAlong = V;
                    rb.angularVelocity = side * Vector3.Dot(rb.angularVelocity, side);
                    cam.camHdg = camHdg;
                    cam.camPitch = camPitch;

                    double trim = Math.Asin(Mathf.Clamp(Vector3.Dot(hull.transform.up, up), -1, 1)) * Mathf.Rad2Deg;
                    FARAeroComponents.PartHydrodynamics hd = aero?.Hydrodynamics;
                    double hydroUp = hd != null ? Vector3.Dot(hd.LastForce, up) : 0;
                    double buoyUp = (hull.partBuoyancy != null ? Vector3.Dot(hull.partBuoyancy.effectiveForce, up) : 0) +
                                    (hd != null ? Vector3.Dot(hd.LastHydrostaticForce, up) : 0);
                    if (!restRecorded && t > 2.9)
                    {
                        // at rest: the keel's depth below the part's center (1 m below it when level)
                        double centerAlt = FlightGlobals.getAltitudeAtPos(hull.transform.position, body);
                        ctx.Record("rest", $"keel {1 - centerAlt:F3} m deep at the middle, trim {trim:F2} deg, buoyancy {buoyUp:F2} kN of {weight:F2} " +
                                           $"({(hd != null && hd.OwnsBuoyancy ? $"FAR's, mesh volume {hd.MeshVolume:F2} m3" : "stock's")})");
                        restRecorded = true;
                    }
                    double k = Math.Min(1, dt / 0.5);
                    if (t > 0.1)
                        rEma += k * (towKN - rEma);
                    liftEma += k * (hydroUp / weight - liftEma);
                    buoyEma += k * (buoyUp / weight - buoyEma);
                    double cv = V / Math.Sqrt(9.81 * beam);
                    string phase = V < 0.3 ? "at rest" : liftEma > 0.5 ? "PLANING" : cv > 1.2 ? "hump" : "displacement";
                    rec.Lines = new[]
                    {
                        $"{model}  -  V hull, 20 deg deadrise, 2 m x 8 m, 4 t",
                        $"speed {V,5:F1} m/s   C_V {cv:F2}",
                        $"trim {trim,5:F1} deg   resistance/weight {Math.Max(0, rEma) / weight:F3}",
                        $"weight carried: dynamic lift {100 * liftEma,3:F0}%  buoyancy {100 * buoyEma,3:F0}%",
                        phase,
                    };
                    csv.AppendLine(string.Join(",", F(t), F(V), F(cv), F(trim), F(rEma / weight), F(liftEma), F(buoyEma), F(hydroUp), F(buoyUp), F(towKN),
                                               rec.Frame.ToString(CultureInfo.InvariantCulture)));
                    yield return fixedWait;
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "video.csv"), csv.ToString());
                ctx.Record("frames", rec.Frame);
                Assert.True(rec.Frame > 100, $"frames captured ({rec.Frame})");
            }
            finally
            {
                Time.captureFramerate = 0;
                if (recorderGo != null)
                    UnityEngine.Object.Destroy(recorderGo);
                GameEvents.onShowUI.Fire();
                CheatOptions.NoCrashDamage = crashWas;
                Settings.FARSettings.Hydrodynamics = hydroWas;
            }
        }

        // the hull's keel direction in part space: of +-x and +-z, the one whose farthest vertices span the least
        // across (the V's point rather than the flat deck)
        internal static Vector3 KeelAxis(Part p)
        {
            MeshCollider mc = p.GetComponentsInChildren<MeshCollider>().FirstOrDefault(c => c.sharedMesh != null);
            Assert.True(mc != null, "the hull has a mesh collider");
            Vector3[] verts = mc.sharedMesh.vertices.Select(w => p.transform.InverseTransformPoint(mc.transform.TransformPoint(w))).ToArray();
            Vector3[] dirs = { Vector3.forward, Vector3.back, Vector3.right, Vector3.left };
            Vector3 best = Vector3.forward;
            float bestSpread = float.MaxValue;
            foreach (Vector3 d in dirs)
            {
                float max = verts.Max(w => Vector3.Dot(w, d));
                Vector3 across = Mathf.Abs(d.z) > 0.5f ? Vector3.right : Vector3.forward;
                var near = verts.Where(w => Vector3.Dot(w, d) > max - 0.02f).Select(w => Vector3.Dot(w, across)).ToArray();
                float spread = near.Max() - near.Min();
                if (spread < bestSpread)
                {
                    bestSpread = spread;
                    best = d;
                }
            }
            return best;
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);

        /// <summary>Saves the screen at the end of every frame and draws the overlay.</summary>
        internal class Recorder : MonoBehaviour
        {
            public string Dir;
            public string[] Lines = Array.Empty<string>();
            public int Frame;
            private GUIStyle style, shadow;

            private IEnumerator Start()
            {
                var end = new WaitForEndOfFrame();
                while (true)
                {
                    yield return end;
                    Texture2D tex = ScreenCapture.CaptureScreenshotAsTexture();
                    File.WriteAllBytes(Path.Combine(Dir, $"{Frame:D5}.jpg"), tex.EncodeToJPG(92));
                    Destroy(tex);
                    Frame++;
                }
            }

            private void OnGUI()
            {
                if (style == null)
                {
                    style = new GUIStyle(GUI.skin.label) { fontSize = Math.Max(16, Screen.height / 36), fontStyle = FontStyle.Bold };
                    style.normal.textColor = Color.white;
                    shadow = new GUIStyle(style);
                    shadow.normal.textColor = new Color(0, 0, 0, 0.8f);
                }
                float lh = style.fontSize * 1.35f, y = 12;
                foreach (string line in Lines)
                {
                    GUI.Label(new Rect(16 + 2, y + 2, Screen.width, lh), line, shadow);
                    GUI.Label(new Rect(16, y, Screen.width, lh), line, style);
                    y += lh;
                }
            }
        }
    }
}
