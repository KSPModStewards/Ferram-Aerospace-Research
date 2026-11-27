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
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.DemoMods,
                StartsAt = StartState.Flight, Craft = VHull, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Video_VHullPlaning(TestContext ctx)
        {
            const double beam = 2, length = 8, depth = 2;
            double massT = ctx.ParamDouble("mass", 4.0), lcg = ctx.ParamDouble("lcg", 3.0), cgHeight = ctx.ParamDouble("cgHeight", 0.5);
            double top = ctx.ParamDouble("topSpeed", 20), ramp = ctx.ParamDouble("rampSec", 36);
            bool hydroWas = FARDebugValues.hydrodynamics;
            string hydro = ctx.Param("hydro");
            if (!string.IsNullOrEmpty(hydro))
                FARDebugValues.hydrodynamics = bool.Parse(hydro);
            string model = FARDebugValues.hydrodynamics ? "FAR PartHydrodynamics" : "FAR legacy water forces";
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
                FARDebugValues.hydrodynamics = hydroWas;
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

        /// <summary>Saves the screen at the end of every frame (Dir/NNNNN.jpg), except while Paused (nothing written; the
        /// frame count not advanced). With Video set: each frame's raw pixels piped to an ffmpeg process instead (Ffmpeg:
        /// its path, default "ffmpeg" on the PATH), encoded as it goes to that .mp4 at Fps (H.264, Height / Crf / Preset):
        /// a few MB rather than hundreds of MB of JPEGs, and no encode afterwards. With Sound too (default): the game's
        /// audio, rendered by Unity's AudioRenderer in step with the capture frame rate (the speakers silent meanwhile),
        /// each frame's samples kept beside it (those of paused frames dropped with them), muxed in as AAC at the end.
        /// Finished when the recorder is destroyed. Draws the overlay (Lines) over the frames.
        /// (Ported from WaterWakes' KspTests TestTools.Recorder.)</summary>
        internal class Recorder : MonoBehaviour
        {
            public string Dir;
            public string[] Lines = Array.Empty<string>();
            private GUIStyle style, shadow;
            public int Frame;
            public bool Paused;
            public string Video, Ffmpeg = "ffmpeg";
            public int Fps = 30;
            public int Height = 720, Crf = 24;    // (the demo clips': 720p (aspect kept), crf 24, x264 medium; Height 0: as captured)
            public string Preset = "medium";
            public bool Sound = true;
            public float SoundVolume = 0.5f;   // the listener's volume while it records: KspTest mutes the game (MASTER_VOLUME 0),
                                               // and AudioRenderer sends the speakers silence meanwhile anyway
            private float listenerWas = -1;
            public bool ToneTest;    // (diagnostic: a quiet 440 Hz tone played while recording, to tell a silent game from a broken capture)
            private float peak;
            private int peakFrames;
            private AudioSource tone;

            private System.Diagnostics.Process ff;
            private Stream pipe;
            private int vw, vh, bpp;
            private string target, silentVideo, audioRaw;
            private FileStream audio;
            private bool rendering;
            private int channels, rate;

            private IEnumerator Start()
            {
                var end = new WaitForEndOfFrame();
                while (true)
                {
                    yield return end;
                    TakeAudio(!Paused && Video != null);
                    if (Paused)
                        continue;
                    Texture2D tex = ScreenCapture.CaptureScreenshotAsTexture();
                    if (Video != null)
                        Pipe(tex);
                    else
                        File.WriteAllBytes(Path.Combine(Dir, $"{Frame:D5}.jpg"), tex.EncodeToJPG(92));
                    Destroy(tex);
                    Frame++;
                }
            }

            // this frame's audio (GetSampleCountForCaptureFrame: the capture frame rate's share), kept or dropped
            private void TakeAudio(bool keep)
            {
                if (!Sound || Video == null)
                    return;
                if (!rendering)
                {
                    rate = AudioSettings.outputSampleRate;
                    AudioSpeakerMode mode = AudioSettings.GetConfiguration().speakerMode;
                    channels = mode == AudioSpeakerMode.Mono ? 1 : mode == AudioSpeakerMode.Quad ? 4 : mode == AudioSpeakerMode.Surround ? 5
                             : mode == AudioSpeakerMode.Mode5point1 ? 6 : mode == AudioSpeakerMode.Mode7point1 ? 8 : 2;
                    rendering = AudioRenderer.Start();
                    if (ToneTest)
                    {
                        var clip = AudioClip.Create("KspTest tone", rate, 1, rate, false);
                        var d = new float[rate];
                        for (int i = 0; i < rate; i++)
                            d[i] = 0.2f * Mathf.Sin(2 * Mathf.PI * 440 * i / rate);
                        clip.SetData(d, 0);
                        tone = gameObject.AddComponent<AudioSource>();
                        tone.clip = clip;
                        tone.loop = true;
                        tone.spatialBlend = 0;
                        tone.Play();
                    }
                    if (!rendering)
                    {
                        Debug.LogError("[KspTest] AudioRenderer would not start: the video without sound");
                        Sound = false;
                        return;
                    }
                }
                // (each frame: KSP sets the listener's volume from its settings)
                if (listenerWas < 0)
                    listenerWas = AudioListener.volume;
                AudioListener.volume = SoundVolume;
                int n = AudioRenderer.GetSampleCountForCaptureFrame();
                if (n <= 0)
                    return;
                var buf = new Unity.Collections.NativeArray<float>(n * channels, Unity.Collections.Allocator.Temp);
                try
                {
                    AudioRenderer.Render(buf);
                    // (diagnostic, each second: the samples' peak, the listener, the sources playing)
                    for (int i = 0; i < buf.Length; i++)
                        peak = Mathf.Max(peak, Mathf.Abs(buf[i]));
                    if (++peakFrames >= Fps)
                    {
                        int playing = 0;
                        foreach (AudioSource a in FindObjectsOfType<AudioSource>())
                            if (a.isPlaying)
                                playing++;
                        Debug.Log($"[KspTest] sound: peak {peak:F4}, listener volume {AudioListener.volume:F2}{(AudioListener.pause ? " PAUSED" : "")}, {playing} sources playing, {n} samples/frame");
                        peak = 0;
                        peakFrames = 0;
                    }
                    if (!keep)
                        return;
                    if (audio == null)
                    {
                        audioRaw = Video + ".audio.f32";
                        audio = new FileStream(audioRaw, FileMode.Create, FileAccess.Write);
                    }
                    float[] f = buf.ToArray();
                    var bytes = new byte[f.Length * 4];
                    Buffer.BlockCopy(f, 0, bytes, 0, bytes.Length);
                    audio.Write(bytes, 0, bytes.Length);
                }
                finally
                {
                    buf.Dispose();
                }
            }

            private void Pipe(Texture2D tex)
            {
                if (ff == null)
                {
                    // (the capture's rows bottom up: flipped and converted by ffmpeg; even sizes for yuv420p). With sound, the
                    // picture first to a silent file beside it, the sound muxed in at the end
                    vw = tex.width;
                    vh = tex.height;
                    bpp = tex.format == TextureFormat.RGB24 ? 3 : 4;
                    target = Video;
                    silentVideo = Sound ? Path.ChangeExtension(Video, ".silent.mp4") : Video;
                    string pf = bpp == 3 ? "rgb24" : tex.format == TextureFormat.ARGB32 ? "argb" : tex.format == TextureFormat.BGRA32 ? "bgra" : "rgba";
                    string args = $"-y -loglevel error -f rawvideo -pix_fmt {pf} -s {vw}x{vh} -r {Fps} -i - " +
                                  $"-vf \"vflip,{(Height > 0 ? $"scale=-2:{Height}" : "crop=trunc(iw/2)*2:trunc(ih/2)*2")}\" -c:v libx264 -preset {Preset} -crf {Crf} -pix_fmt yuv420p -movflags +faststart \"{silentVideo}\"";
                    try
                    {
                        ff = System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo(Ffmpeg, args)
                        {
                            UseShellExecute = false, RedirectStandardInput = true, CreateNoWindow = true,
                        });
                        pipe = ff.StandardInput.BaseStream;
                        Debug.Log($"[KspTest] recording video to {Video} ({vw}x{vh} at {Fps} fps{(Sound ? ", with sound" : "")}, {Ffmpeg})");
                    }
                    catch (Exception e)
                    {
                        Debug.LogError($"[KspTest] could not start {Ffmpeg} ({e.Message}): saving frames as JPEGs instead");
                        Video = null;
                        return;
                    }
                }
                if (tex.width != vw || tex.height != vh)
                    return;   // (the window resized: those frames dropped)
                byte[] raw = tex.GetRawTextureData();
                if (raw.Length != vw * vh * bpp)
                {
                    Debug.LogError($"[KspTest] video frame {raw.Length} bytes, not {vw}x{vh}x{bpp} ({tex.format}): recording stopped");
                    Video = null;
                    Finish();
                    return;
                }
                try
                {
                    pipe.Write(raw, 0, raw.Length);
                }
                catch (Exception e)
                {
                    Debug.LogError($"[KspTest] video pipe failed ({e.Message}): recording stopped");
                    Video = null;
                    Finish();
                }
            }

            private void Finish()
            {
                if (rendering)
                {
                    AudioRenderer.Stop();
                    rendering = false;
                }
                if (listenerWas >= 0)
                {
                    AudioListener.volume = listenerWas;
                    listenerWas = -1;
                }
                audio?.Close();
                if (ff == null)
                    return;
                try
                {
                    pipe.Close();
                    if (!ff.WaitForExit(60000))
                        Debug.LogError("[KspTest] ffmpeg did not finish the video within 60 s");
                    else
                        Debug.Log($"[KspTest] video written: {silentVideo}, {Frame} frames, ffmpeg exit {ff.ExitCode}");
                    ff = null;
                    if (silentVideo != target)
                        Mux();
                }
                catch (Exception e)
                {
                    Debug.LogError($"[KspTest] finishing the video: {e.Message}");
                }
                ff = null;
            }

            // the sound into the picture (the picture copied as is): Video; the silent file and the raw samples then deleted
            private void Mux()
            {
                if (audio == null || !File.Exists(audioRaw) || new FileInfo(audioRaw).Length == 0)
                {
                    File.Copy(silentVideo, target, true);
                    File.Delete(silentVideo);
                    Debug.Log($"[KspTest] no sound recorded: {target} silent");
                    return;
                }
                string args = $"-y -loglevel error -i \"{silentVideo}\" -f f32le -ar {rate} -ac {channels} -i \"{audioRaw}\" " +
                              $"-c:v copy -c:a aac -b:a 160k -ac 2 -shortest -movflags +faststart \"{target}\"";
                var mux = System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo(Ffmpeg, args) { UseShellExecute = false, CreateNoWindow = true });
                if (mux.WaitForExit(60000) && mux.ExitCode == 0)
                {
                    File.Delete(silentVideo);
                    File.Delete(audioRaw);
                    Debug.Log($"[KspTest] video with sound: {target} ({rate} Hz, {channels} ch)");
                }
                else
                    Debug.LogError($"[KspTest] muxing the sound failed (exit {(mux.HasExited ? mux.ExitCode : -1)}): the silent video and the samples kept ({silentVideo}, {audioRaw})");
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

            private void OnDestroy() => Finish();
        }
    }
}
