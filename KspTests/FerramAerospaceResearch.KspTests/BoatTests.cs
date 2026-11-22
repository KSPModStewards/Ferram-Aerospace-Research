using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The Test Boat (crafts/test_boat.py: the Planing Hull with the FAR test electric motor under its stern, a B9
    /// procedural control surface for its spade rudder, a probe core) driven about the water as a player would, after FarWaterTools' button put it there.
    /// </summary>
    public static class BoatTests
    {
        // the floatplane tests' (one instance): CustomProceduralParts for the hull, FARTestFloatplane for the motor
        private const string Mods = TestWarp.WaterMods;   // the water tests' instance

        /// <summary>
        /// The Test Boat put on the water by FarWaterTools.PutOnWater (the button's own code), after the Set Position
        /// cheat's ease-in has cut its gravity to a tenth: it floats at rest at full gravity. Then full throttle, rudder
        /// centered, for runSec (default 45): it gets going, and the motor's thrust is its curve's at the speed it makes
        /// (10 kN at rest, none at 25 m/s; the motor works in the water). Then slowed to turnThrottle (default 0.3) and
        /// full rudder one way for turnSec (default 8) and the other: the rudder turns it both ways, the boat heeling under
        /// 30 deg. mass (t, default 4) the hull's fuel makes it. Writes boat.csv.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Test Boat", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Boat_MotorRun(TestContext ctx)
        {
            double massT = ctx.ParamDouble("mass", 4), runSec = ctx.ParamDouble("runSec", 45), turnSec = ctx.ParamDouble("turnSec", 8);
            WaterParams.Apply(ctx);
            bool crashWas = CheatOptions.NoCrashDamage;
            try
            {
                yield return Wait.Seconds(5);
                Vessel v = FlightGlobals.ActiveVessel;
                Part hull = v.rootPart;
                foreach (PartResource r in hull.Resources)
                    r.amount = 0;
                yield return Wait.Frames(2);
                PartResource lf = hull.Resources.Get("LiquidFuel");
                double dry = v.parts.Sum(q => q.mass + q.GetResourceMass());
                if (lf != null)
                    lf.amount = Math.Max(0, Math.Min(lf.maxAmount, (massT - dry) / lf.info.density));
                yield return Wait.Frames(2);
                CheatOptions.NoCrashDamage = true;

                // the Set Position cheat's ease-in on (its gravity a tenth), for the button to undo
                typeof(FlightGlobals).GetMethod("ToggleVesselEaseIn", BindingFlags.NonPublic | BindingFlags.Instance)
                    ?.Invoke(FlightGlobals.fetch, new object[] { v, true, 0.1 });
                ctx.Record("ease-in", $"on: gravity x{v.gravityMultiplier:F2}");
                string said = null;
                yield return FarWaterTools.PutOnWater(s => said = s);
                v = FlightGlobals.ActiveVessel;
                yield return Wait.GameSeconds(5);
                Vector3 up = v.upAxis;
                Vector3 fwd = v.ReferenceTransform.up, right = v.ReferenceTransform.right;
                double roll = Math.Asin(Mathf.Clamp(Vector3.Dot(right, up), -1, 1)) * 180 / Math.PI;
                double trim = Math.Asin(Mathf.Clamp(Vector3.Dot(fwd, up), -1, 1)) * 180 / Math.PI;
                ctx.Record("on the water", $"{said}; {v.situation}, gravity x{v.gravityMultiplier:F2}, {v.srfSpeed:F2} m/s, trim {trim:F1} deg, " +
                                           $"roll {roll:F1} deg, mass {v.GetTotalMass():F3} t");
                Assert.True(v.situation == Vessel.Situations.SPLASHED, $"splashed ({v.situation})");
                Assert.True(Math.Abs(v.gravityMultiplier - 1) < 1e-6, $"full gravity (x{v.gravityMultiplier:F2})");
                Assert.True(v.srfSpeed < 0.5 && Math.Abs(roll) < 5, $"at rest, upright ({v.srfSpeed:F2} m/s, roll {roll:F1} deg)");

                Part motorPart = v.parts.First(q => q.partInfo.name == "FARTest.BoatMotor");
                ModuleEngines motor = motorPart.Modules.GetModule<ModuleEngines>();
                motor.Activate();
                Ctrl.Attach();
                Ctrl.SetThrottle(1);
                TestWarp.Hyper(ctx, true);
                var csv = new StringBuilder("t,phase,V_ms,thrust_kN,curve_kN,water_drag_kN,heading_deg,trim_deg,roll_deg,yaw\n");
                var fixedWait = new WaitForFixedUpdate();
                Vector3 h0 = Vector3.ProjectOnPlane(fwd, up).normalized;
                double t0 = Planetarium.GetUniversalTime();
                double Heading() => Vector3.SignedAngle(h0, Vector3.ProjectOnPlane(v.ReferenceTransform.up, up), up);
                var tail = new List<(double V, double thrust, double curve)>();
                double turn1 = double.NaN, turn2 = double.NaN, hStart = 0, heel = 0;
                // the turns at turnThrottle (default 0.3, about 5 m/s): at full speed, hard over, this flat-bottomed box (its
                // CG a meter above the keel) heels out of the turn until its rudder leaves the water
                float turnThrottle = (float)ctx.ParamDouble("turnThrottle", 0.3);
                foreach (var (phase, yaw, sec, throttle) in new[]
                         {
                             ("run", 0f, runSec, 1f), ("slow", 0f, 15, turnThrottle), ("right", 1f, turnSec, turnThrottle),
                             ("left", -1f, 2 * turnSec, turnThrottle)
                         })
                {
                    Ctrl.Yaw = yaw;
                    Ctrl.SetThrottle(throttle);
                    double ts = Planetarium.GetUniversalTime(), hs = Heading();
                    while (Planetarium.GetUniversalTime() - ts < sec)
                    {
                        yield return fixedWait;
                        up = v.upAxis;
                        double V = v.srfSpeed, thrust = motor.finalThrust, curve = 10 * Math.Max(0, 1 - V / 25);
                        Vector3 dir = ((Vector3)v.srf_velocity).normalized;
                        Vector3 water = Vector3.zero;
                        foreach (Part q in v.parts)
                        {
                            var hq = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                            if (hq != null)
                                water += hq.LastForce;
                        }
                        double t = Planetarium.GetUniversalTime() - t0;
                        double tr = Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.up, up), -1, 1)) * 180 / Math.PI;
                        double rl = Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.right, up), -1, 1)) * 180 / Math.PI;
                        csv.AppendLine(string.Join(",", F(t), phase, F(V), F(thrust), F(curve), F(-Vector3.Dot(water, dir)), F(Heading()), F(tr), F(rl),
                                                   F(yaw)));
                        if (phase == "right" || phase == "left")
                            heel = Math.Max(heel, Math.Abs(rl));
                        if (phase == "run" && Planetarium.GetUniversalTime() - ts > sec - 5)
                            tail.Add((V, thrust, curve));
                    }
                    double dh = Mathf.DeltaAngle((float)hs, (float)Heading());
                    if (phase == "right")
                        turn1 = dh;
                    else if (phase == "left")
                        turn2 = dh;
                    else if (phase == "run")
                        hStart = Heading();
                }
                Ctrl.Yaw = 0;
                Ctrl.SetThrottle(0);
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "boat.csv"), csv.ToString());
                double vEnd = tail.Average(x => x.V), thrustEnd = tail.Average(x => x.thrust), curveEnd = tail.Average(x => x.curve);
                ctx.Record("run", $"{vEnd:F2} m/s after {runSec:F0} s at full throttle: thrust {thrustEnd:F2} kN, its curve's {curveEnd:F2} kN");
                ctx.Record("turns", $"full rudder one way for {turnSec:F0} s: heading {turn1:+0.0;-0.0} deg; the other way for {2 * turnSec:F0} s: {turn2:+0.0;-0.0} deg; heel up to {heel:F1} deg");
                Assert.True(vEnd > 5, $"under way ({vEnd:F2} m/s)");
                Assert.True(Math.Abs(thrustEnd - curveEnd) < 0.05 * 10, $"the motor's thrust its curve's ({thrustEnd:F2} against {curveEnd:F2} kN)");
                Assert.True(Math.Abs(turn1) > 10 && Math.Sign(turn2) == -Math.Sign(turn1), $"the rudder turns it both ways ({turn1:F1}, {turn2:F1} deg)");
                Assert.True(heel < 30, $"upright in its turns (heel up to {heel:F1} deg)");
            }
            finally
            {
                TestWarp.Hyper(ctx, false);
                Ctrl.Yaw = 0;
                CheatOptions.NoCrashDamage = crashWas;
                WaterParams.Restore();
            }
        }

        /// <summary>
        /// A player's jet boat (Boaty McTestFace x4: Mk3 stock parts for its hulls, four AJE turbofans, B9 wings) put on
        /// the water by FarWaterTools.PutOnWater and run at full throttle, the stick held at each of pitches (default
        /// 0, -0.5, +0.5: level, nose down, nose up) for phaseSec (default 30) in turn: its speed and trim, and the
        /// water's forces along its course (each part's: the hull model's momentum, friction, impact, crossflow and
        /// hydrostatic forces; the foils') beside its engines' thrust, recorded at each phase's fastest. Recorded, not
        /// asserted. Writes jetboat.csv.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Boaty McTestFace x4", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Boat_JetRun(TestContext ctx) => JetRun(ctx);

        /// <summary>Boat_JetRun's run of the player's next version: one turbofan, cargo ramps at the bows.</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Boaty McTestFace", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Boat_JetRunSingle(TestContext ctx) => JetRun(ctx);

        /// <summary>Boat_JetRun's run of Bonnie: a 70 m Spine V-hull (procedural, ~950 t with its fuel and ore keel), two
        /// gas turbine generators driving two 800 kN propellers (Kerbal Powers Naval, from CKAN with KSPCommunityPartModules
        /// for its turbine's converter; mods/FARTestFloatplane/KerbalPowersNaval.cfg mends its engine type).</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods + ";KerbalPowersNaval;KSPCommunityPartModules", StartsAt = StartState.Flight,
                Craft = "Bonnie", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Boat_RunBonnie(TestContext ctx) => JetRun(ctx, 0.5);   // (straight to the water: a V hull on the runway rolls over)

        private static IEnumerator JetRun(TestContext ctx, double settleSec = 5)
        {
            double phaseSec = ctx.ParamDouble("phaseSec", 30);
            double[] pitches = ctx.ParamList("pitches", "0+-0.5+0.5").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            WaterParams.Apply(ctx);
            bool crashWas = CheatOptions.NoCrashDamage, jointsWas = CheatOptions.UnbreakableJoints;
            // at once: a heavy boat (Bonnie, ~900 t) settling onto the runway can break its joints or burst a part
            CheatOptions.NoCrashDamage = true;
            CheatOptions.UnbreakableJoints = true;
            GameObject recorderGo = null;
            Action<int> setView = null;
            Action<Vector3> aimView = null;
            GameObject anchorGo = null;
            var limits = new List<(ModuleEngines e, float was)>();
            bool hyper = ctx.Param("hyperWarp") != null;
            try
            {
                // each part's offset from the root and its joint, now and after it is put on the water (a part coming loose)
                void Parts(string when)
                {
                    Vessel pv = FlightGlobals.ActiveVessel;
                    ctx.Record($"parts {when}", string.Join("; ", pv.parts.Select(q =>
                        $"{q.partInfo.name} {(q.transform.position - pv.rootPart.transform.position).magnitude:F1} m{(q.attachJoint != null ? $" joint {(q.attachJoint.Joint != null ? "ok" : "none")}" : "")}")));
                }
                // (the cheat's flag alone leaves the joints already made breakable: the cheats menu also sets each one)
                foreach (Part q in FlightGlobals.ActiveVessel.parts)
                    q.attachJoint?.SetUnbreakable(true, q.rigidAttachment);
                Parts("at start");
                // (noon=true: the clock moved on to when the sun stands highest over the water site, hour by hour over a
                // day, the craft held on rails meanwhile; for demos lit from above)
                if (ctx.Param("noon") == "true")
                    yield return Noon(ctx);
                yield return Wait.Seconds(settleSec);
                Parts("after 5 s");
                Vessel v = FlightGlobals.ActiveVessel;
                string said = null;
                yield return FarWaterTools.PutOnWater(s => said = s);
                Parts("placed");
                v = FlightGlobals.ActiveVessel;
                yield return Wait.GameSeconds(5);
                Parts("5 s on the water");
                ctx.Record("on the water", $"{said}; {v.situation}, {v.srfSpeed:F2} m/s, mass {v.GetTotalMass():F2} t, {v.parts.Count} parts");

                // (record=true: a video, frames/NNNNN.jpg at 1/30 s of game time each, the camera camAngle deg aft of
                // abeam, camPitch deg down, camDistance m off, the UI hidden)
                if (ctx.Param("record") == "true")
                {
                    FlightCamera cam = FlightCamera.fetch;
                    cam.setModeImmediate(FlightCamera.Modes.FREE);   // (AUTO turns to chase its velocity as it speeds up, the pitch drifting)
                    cam.minPitch = -0.5f;
                    cam.minDistance = Mathf.Min(cam.minDistance, 5f);   // (a big craft's is ~its size: the close views)
                    cam.SetDistanceImmediate((float)ctx.ParamDouble("camDistance", 40));
                    float camPitch = (float)(ctx.ParamDouble("camPitch", 25) * Mathf.Deg2Rad);
                    Vector3 up0 = v.upAxis, h0 = Vector3.ProjectOnPlane(v.ReferenceTransform.up, up0).normalized;
                    Vector3 look = Quaternion.AngleAxis((float)ctx.ParamDouble("camAngle", 135), up0) * Vector3.Cross(up0, h0).normalized;
                    float a0 = 0, a1 = 0;
                    for (int pass = 0; pass < 2; pass++)
                    {
                        for (int i = 0; i < 10; i++)
                        {
                            cam.camHdg = pass * 0.5f;
                            cam.camPitch = camPitch;
                            yield return null;
                        }
                        float a = Vector3.SignedAngle(look, Vector3.ProjectOnPlane(cam.transform.forward, up0).normalized, up0) * Mathf.Deg2Rad;
                        if (pass == 0) a0 = a;
                        else a1 = a;
                    }
                    float slope = Mathf.DeltaAngle(a0 * Mathf.Rad2Deg, a1 * Mathf.Rad2Deg) * Mathf.Deg2Rad / 0.5f;
                    cam.camHdg = Math.Abs(slope) > 0.2f ? -a0 / slope : 0;
                    cam.camPitch = camPitch;
                    GameEvents.onHideUI.Fire();
                    string frames = Path.Combine(ctx.ResultsDir, "frames");
                    Directory.CreateDirectory(frames);
                    recorderGo = new GameObject("JetRunRecorder");
                    var rec = recorderGo.AddComponent<PlaningVideoTests.Recorder>();
                    rec.Dir = frames;
                    Time.captureFramerate = Math.Max(1, (int)ctx.ParamDouble("fps", 30));   // (fewer: a quicker run, a choppier video)

                    // (views=cycle: the camera cycles every viewSec (default 7) through the first view, the same turned 180 deg
                    // about the craft, high over the stern looking down, and low straight on from the stern, each
                    // aimed at a point that far behind the craft along its course (back x camDistance): the quarter
                    // views put the craft to one side and its wake across the frame, the top view over the stern;
                    // each switch's frame in frames/views.txt)
                    // (views=demo: instead, low from astern at the waterline looking at the stern, low off the starboard
                    // bow looking back at it, and high ahead of the bow looking down along the deck: the waterline's
                    // foam and splash at the hull, the bow wave, the wake behind)
                    string views = ctx.Param("views");
                    // (views=deck: one view, from ~12 m over the stern looking aft along the wake, its pivot on the sea
                    // 1.25 camDistance behind the CoM: a passenger's view of the wake off a ferry's stern; camDistance ~115)
                    if (views == "cycle" || views == "demo" || views == "deck")
                    {
                        float dist0 = (float)ctx.ParamDouble("camDistance", 40), pitch0 = camPitch * Mathf.Rad2Deg;
                        Vector3 bowQuarter = Quaternion.AngleAxis(-30, up0) * -h0;   // (looking back at the bow from ahead, off its starboard side)
                        var cycle = views == "deck"
                            ? new (string name, Vector3 look, float pitch, float dist, float back)[] { ("aft-deck", -h0, 6f, 1f, 1.25f) }
                            : views == "demo"
                            ? new (string name, Vector3 look, float pitch, float dist, float back)[]
                            {
                                ("stern-low", h0, 2f, 0.2f, 0.25f),
                                ("bow-low", bowQuarter, 5f, 0.17f, -0.25f),
                                ("top-bow", -h0, 55f, 0.5f, -0.1f),
                            }
                            : new (string name, Vector3 look, float pitch, float dist, float back)[]
                            {
                                ("quarter", look, pitch0, 1f, 0.3f),
                                ("quarter-opposite", -look, pitch0, 1f, 0.3f),   // (the first view turned 180 deg about the craft)
                                ("top-stern", h0, 62f, 1.1f, 0.3f),
                                ("stern", h0, 9f, 0.9f, 0f),
                            };
                        anchorGo = new GameObject("JetRunCameraAnchor");
                        var anchor = anchorGo.AddComponent<CameraAnchor>();
                        anchor.Vessel = v;
                        anchor.Heading = h0;
                        cam.SetTargetTransform(anchorGo.transform);
                        var log = new StringBuilder();
                        string logPath = Path.Combine(frames, "views.txt");
                        // (each view turns with the course: in a turn it keeps its place about the craft and the wake behind)
                        int current = 0;
                        aimView = course =>
                        {
                            var c = cycle[current % cycle.Length];
                            float turned = course.sqrMagnitude > 0 ? Vector3.SignedAngle(h0, course, up0) : 0;
                            float delta = (Vector3.SignedAngle(look, c.look, up0) + turned) * Mathf.Deg2Rad;
                            cam.camHdg = Math.Abs(slope) > 0.2f ? (delta - a0) / slope : 0;
                        };
                        // (where the last view's camera and its pivot ended up, about the CoM: m ahead, to starboard, up)
                        string Where(Vector3 p)
                        {
                            Vector3 r = p - v.CoM, side = Vector3.Cross(up0, h0).normalized;
                            return $"{Vector3.Dot(r, h0):F0}/{Vector3.Dot(r, -side):F0}/{Vector3.Dot(r, up0):F0}";
                        }
                        setView = i =>
                        {
                            if (i > 0)
                                log.AppendLine($"  (camera at {Where(cam.transform.position)}, pivot {Where(cam.GetPivot().position)}, target {Where(anchorGo.transform.position)}; distance {cam.Distance:F0})");
                            current = i;
                            var c = cycle[i % cycle.Length];
                            Vector3 vel = Vector3.ProjectOnPlane(v.srf_velocity, up0);
                            aimView(vel.magnitude > 1 ? vel.normalized : Vector3.zero);
                            cam.camPitch = c.pitch * Mathf.Deg2Rad;
                            cam.minDistance = Mathf.Min(cam.minDistance, 5f);
                            cam.SetDistanceImmediate(dist0 * c.dist);
                            anchor.Back = c.back * dist0;
                            anchor.Height = views == "deck" ? 0f : views == "demo" && c.pitch < 20 ? 1.5f : float.NaN;   // (the low views: 1.5 m above the sea)
                            log.AppendLine($"{rec.Frame} {c.name}");
                            File.WriteAllText(logPath, log.ToString());
                        };
                        setView(0);
                    }
                }

                // (hyperWarp=N: physics hyper-warp at N x, physics accuracy kept)
                if (hyper)
                    TestWarp.Hyper(ctx, true);
                var fixedWait = new WaitForFixedUpdate();
                double viewSec = ctx.ParamDouble("viewSec", 7), nextView = Planetarium.GetUniversalTime() + viewSec;
                int viewIdx = 1;
                // (restSec=S: first S s adrift with the engines off, the camera cycling: the hull's own heave and roll at the waterline)
                double restSec = ctx.ParamDouble("restSec", 0);
                if (restSec > 0)
                {
                    double tr = Planetarium.GetUniversalTime();
                    ctx.Record("rest", $"{restSec:F0} s adrift from frame {(recorderGo != null ? recorderGo.GetComponent<PlaningVideoTests.Recorder>().Frame : -1)}");
                    // (its heave and rocking once a second: a hull at rest should settle)
                    var motion = new List<string>();
                    double nextMotion = 0;
                    while (Planetarium.GetUniversalTime() - tr < restSec)
                    {
                        yield return fixedWait;
                        double tm = Planetarium.GetUniversalTime() - tr;
                        if (tm >= nextMotion)
                        {
                            motion.Add($"{tm:F0}s {v.verticalSpeed:+0.00;-0.00} m/s {v.angularVelocity.magnitude * Mathf.Rad2Deg:F2} deg/s");
                            nextMotion += 1;
                        }
                        if (setView != null && Planetarium.GetUniversalTime() >= nextView)
                        {
                            setView(viewIdx++);
                            nextView += viewSec;
                        }
                    }
                    ctx.Record("rest motion", string.Join(", ", motion));
                }

                var engines = v.parts.SelectMany(q => q.Modules.GetModules<ModuleEngines>()).ToList();
                foreach (ModuleEngines e in engines)
                    e.Activate();
                // (the generators feeding electric propellers)
                var generators = v.parts.SelectMany(q => q.Modules.GetModules<ModuleResourceConverter>()).ToList();
                foreach (ModuleResourceConverter c in generators)
                    c.StartResourceConverter();
                Ctrl.Attach();
                // (throttle=X: the throttle the runs go at, default 1)
                Ctrl.SetThrottle((float)ctx.ParamDouble("throttle", 1));
                bool reported = false;
                var csv = new StringBuilder("t,pitch_in,V_ms,trim_deg,roll_deg,thrust_kN,water_fwd_kN,momentum_fwd_kN,friction_fwd_kN,impact_fwd_kN,crossflow_fwd_kN,hydrostatic_fwd_kN,foil_fwd_kN,water_up_kN," +
                                            "wing_fwd_kN,voxel_fwd_kN,gravity_fwd_kN,ma_fwd_kN,residual_kN,alt_m,vs_ms,heading_deg,drift_deg\n");
                Vector3 velWas = v.srf_velocity;
                double t0 = Planetarium.GetUniversalTime();
                // (the water model's time per physics step over the run, and the frame time: hydro=false runs it without)
                if (ctx.Param("hydro") != null)
                    FARDebugValues.hydrodynamics = ctx.Param("hydro") != "false";
                FARAeroComponents.PartHydrodynamics.ResetProfile();
                // (dryCheck=true: DryFaces checked against its uncached way every recompute, the faces they differ on counted)
                FARAeroComponents.PartHydrodynamics.DryCheck = ctx.Param("dryCheck") == "true";
                FARAeroComponents.PartHydrodynamics.ProfileDryMismatches = 0;
                int profFrames0 = Time.frameCount;
                float profWall0 = Time.realtimeSinceStartup;
                // (turnAfter=S: after S s, yaw at turnYaw (default 1: right; -1 left) and the engines on the inside of the
                // turn limited to turnThrust % (default 100: the rudder alone) for the rest of the run: its wake bends behind it)
                double turnAfter = ctx.ParamDouble("turnAfter", double.NaN), turnHdg = double.NaN, turned = 0, hdgWas = 0;
                Vector3 hRef = Vector3.ProjectOnPlane(v.ReferenceTransform.up, v.upAxis).normalized;
                double Heading() => Vector3.SignedAngle(hRef, Vector3.ProjectOnPlane(v.ReferenceTransform.up, v.upAxis), v.upAxis);
                // its drift (sideslip): the heading's angle from its course, + nose right of it (none when barely moving)
                double Drift()
                {
                    Vector3 course = Vector3.ProjectOnPlane(v.srf_velocity, v.upAxis);
                    return course.magnitude < 1 ? 0 : Vector3.SignedAngle(course, Vector3.ProjectOnPlane(v.ReferenceTransform.up, v.upAxis), v.upAxis);
                }
                double driftMax = 0;
                foreach (double pin in pitches)
                {
                    Ctrl.Pitch = (float)pin;
                    double ts = Planetarium.GetUniversalTime(), vMax = 0;
                    string atMax = "";
                    while (Planetarium.GetUniversalTime() - ts < phaseSec)
                    {
                        yield return fixedWait;
                        if (!double.IsNaN(turnAfter) && double.IsNaN(turnHdg) && Planetarium.GetUniversalTime() - t0 >= turnAfter)
                        {
                            float yaw = (float)ctx.ParamDouble("turnYaw", 1), inner = (float)ctx.ParamDouble("turnThrust", 100);
                            Ctrl.Yaw = yaw;
                            foreach (ModuleEngines e in engines)
                            {
                                float side = Vector3.Dot(e.part.transform.position - v.CoM, v.ReferenceTransform.right);
                                if (inner < 100 && Math.Abs(side) > 0.2f && Math.Sign(side) == Math.Sign(yaw))
                                {
                                    limits.Add((e, e.thrustPercentage));
                                    e.thrustPercentage = inner;
                                }
                            }
                            turnHdg = hdgWas = Heading();
                            ctx.Record("turn", $"at t {Planetarium.GetUniversalTime() - t0:F1} s, {v.srfSpeed:F1} m/s: yaw {yaw:+0.0;-0.0}, {limits.Count} engines inside the turn at {inner:F0} %");
                        }
                        if (!double.IsNaN(turnHdg))
                        {
                            double hNow = Heading();
                            driftMax = Math.Max(driftMax, Math.Abs(Drift()));
                            turned += Mathf.DeltaAngle((float)hdgWas, (float)hNow);   // (past 180 deg too)
                            hdgWas = hNow;
                        }
                        if (setView != null && Planetarium.GetUniversalTime() >= nextView)
                        {
                            setView(viewIdx++);
                            nextView += viewSec;
                        }
                        else if (aimView != null)
                        {
                            Vector3 vel = Vector3.ProjectOnPlane(v.srf_velocity, v.upAxis);
                            if (vel.magnitude > 1)
                                aimView(vel.normalized);
                        }
                        Vector3 up = v.upAxis, dir = ((Vector3)v.srf_velocity).normalized;
                        double V = v.srfSpeed;
                        double trim = Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.up, up), -1, 1)) * 180 / Math.PI;
                        double roll = Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.right, up), -1, 1)) * 180 / Math.PI;
                        double thrust = engines.Sum(e => e.finalThrust);
                        Vector3 all = Vector3.zero, mom = Vector3.zero, fr = Vector3.zero, imp = Vector3.zero, cf = Vector3.zero, hs = Vector3.zero, foil = Vector3.zero;
                        var parts = new List<string>();
                        foreach (Part q in v.parts)
                        {
                            var hq = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                            var w = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                            Vector3 wf = w != null && q.submergedPortion > 0 ? w.worldSpaceForce : Vector3.zero;   // (its air and water both: FAR weights the water by its submerged share)
                            if (hq != null)
                            {
                                all += hq.LastForce + hq.LastHydrostaticForce;
                                mom += hq.LastMomentumForce; fr += hq.LastFrictionForce; imp += hq.LastImpactForce; cf += hq.LastCrossflowForce;
                                hs += hq.LastHydrostaticForce;
                            }
                            foil += wf;
                            if (hq != null && (hq.LastForce + hq.LastHydrostaticForce).sqrMagnitude > 1e-4 || wf.sqrMagnitude > 1e-4)
                                parts.Add($"{q.partInfo.name} sub {q.submergedPortion:F2}{(hq != null ? $" owns {hq.OwnsBuoyancy} open {hq.OpenShare:F2} cap {hq.CapArea:F2} vol {hq.MeshVolume:F2}" : "")}: fwd {Vector3.Dot((hq?.LastForce ?? Vector3.zero) + (hq?.LastHydrostaticForce ?? Vector3.zero), dir):+0.0;-0.0} " +
                                          $"(mom {Vector3.Dot(hq?.LastMomentumForce ?? Vector3.zero, dir):+0.0;-0.0}, imp {Vector3.Dot(hq?.LastImpactForce ?? Vector3.zero, dir):+0.0;-0.0}, hs {Vector3.Dot(hq?.LastHydrostaticForce ?? Vector3.zero, dir):+0.0;-0.0}" +
                                          $"{(wf != Vector3.zero ? $", foil {Vector3.Dot(wf, dir):+0.0;-0.0}" : "")}) up {Vector3.Dot((hq?.LastForce ?? Vector3.zero) + (hq?.LastHydrostaticForce ?? Vector3.zero) + wf, up):F1}");
                        }
                        double t = Planetarium.GetUniversalTime() - t0;
                        // (once, 8 s in: each engine's thrust, its push along the craft's forward axis (+1: straight
                        // ahead), its thrust's depth; the electric charge left and the generators' state)
                        if (!reported && t >= 8)
                        {
                            reported = true;
                            double ec = v.parts.Sum(q => q.Resources.Contains("ElectricCharge") ? q.Resources["ElectricCharge"].amount : 0);
                            ctx.Record("engines", string.Join("; ", engines.Select(e =>
                            {
                                var tts = e.thrustTransforms.Where(x => x).ToList();
                                Vector3 push = tts.Aggregate(Vector3.zero, (a, x) => a - x.forward);
                                double depth = tts.Count > 0 ? tts.Average(x => FlightGlobals.getAltitudeAtPos(x.position)) : double.NaN;
                                return $"{e.part.partInfo.name} {e.finalThrust:F0} kN fwd {Vector3.Dot(push.normalized, v.ReferenceTransform.up):+0.00;-0.00} (its own up {Vector3.Dot(push.normalized, e.part.transform.up):+0.00;-0.00}) at {depth:F1} m ({e.status})";
                            })) + $"; EC {ec:F0}; generators: {string.Join(", ", generators.Select(c => c.status))}");
                        }
                        // the balance along its course: its mass times its acceleration against every force known
                        Vector3 wing = Vector3.zero, voxel = Vector3.zero;
                        foreach (Part q in v.parts)
                        {
                            var w = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                            if (w != null)
                                wing += w.worldSpaceForce;
                            var am = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>();
                            if (am != null)
                                voxel += am.worldSpaceAeroForce;
                        }
                        Vector3 thrustV = Vector3.zero;
                        foreach (ModuleEngines e in engines)
                            foreach (Transform tt in e.thrustTransforms.Where(x => x))
                                thrustV += -tt.forward * (e.finalThrust / Math.Max(1, e.thrustTransforms.Count));
                        double dt = Math.Max(1e-3, TimeWarp.fixedDeltaTime);
                        Vector3 acc = ((Vector3)v.srf_velocity - velWas) / (float)dt;
                        velWas = v.srf_velocity;
                        double mass = v.GetTotalMass();
                        double g = Vector3.Dot((Vector3)FlightGlobals.getGeeForceAtPosition(v.CoM), dir) * mass;
                        double ma = Vector3.Dot(acc, dir) * mass;
                        double known = Vector3.Dot(thrustV + all + wing + voxel, dir) + g;
                        csv.AppendLine(string.Join(",", F(t), F(pin), F(V), F(trim), F(roll), F(thrust), F(Vector3.Dot(all + foil, dir)), F(Vector3.Dot(mom, dir)),
                                                   F(Vector3.Dot(fr, dir)), F(Vector3.Dot(imp, dir)), F(Vector3.Dot(cf, dir)), F(Vector3.Dot(hs, dir)),
                                                   F(Vector3.Dot(foil, dir)), F(Vector3.Dot(all + foil, up)),
                                                   F(Vector3.Dot(wing, dir)), F(Vector3.Dot(voxel, dir)), F(g), F(ma), F(ma - known), F(v.altitude), F(v.verticalSpeed), F(Heading()), F(Drift())));
                        if (V > vMax)
                        {
                            vMax = V;
                            atMax = $"t {t:F1}: {V:F1} m/s, trim {trim:F1} deg, thrust {thrust:F1} kN; water along its course {Vector3.Dot(all + foil, dir):+0.0;-0.0} kN " +
                                    $"(momentum {Vector3.Dot(mom, dir):+0.0;-0.0}, friction {Vector3.Dot(fr, dir):+0.0;-0.0}, impact {Vector3.Dot(imp, dir):+0.0;-0.0}, " +
                                    $"crossflow {Vector3.Dot(cf, dir):+0.0;-0.0}, hydrostatic {Vector3.Dot(hs, dir):+0.0;-0.0}, foils {Vector3.Dot(foil, dir):+0.0;-0.0}), " +
                                    $"up {Vector3.Dot(all + foil, up):F1} kN; parts: {string.Join("; ", parts)}";
                        }
                    }
                    ctx.Record($"pitch {pin:+0.0;-0.0}", $"{v.srfSpeed:F1} m/s at the end; fastest {atMax}");
                }
                {
                    double tick = 1000.0 / System.Diagnostics.Stopwatch.Frequency;
                    long steps = Math.Max(1, FARAeroComponents.PartHydrodynamics.ProfileSteps);
                    double wall = Time.realtimeSinceStartup - profWall0;
                    ctx.Record("frame", $"{wall * 1000 / Math.Max(1, Time.frameCount - profFrames0):F1} ms a frame ({Time.frameCount - profFrames0} frames, {wall:F1} s); hydrodynamics {FARDebugValues.hydrodynamics}");
                    ctx.Record("water model", $"{(FARAeroComponents.PartHydrodynamics.ProfileGatherTicks + FARAeroComponents.PartHydrodynamics.ProfileSolveTicks) * tick / steps:F2} ms a step: " +
                                              $"gather {FARAeroComponents.PartHydrodynamics.ProfileGatherTicks * tick / steps:F2} ms, " +
                                              $"solve {FARAeroComponents.PartHydrodynamics.ProfileSolveTicks * tick / steps:F2} ms (lee {FARAeroComponents.PartHydrodynamics.ProfileLeeTicks * tick / steps:F2} ms, " +
                                              $"{FARAeroComponents.PartHydrodynamics.ProfileLeeRays / steps} rays; pieces {FARAeroComponents.PartHydrodynamics.ProfilePieceTicks * tick / steps:F2} ms; " +
                                              $"strips and impact {FARAeroComponents.PartHydrodynamics.ProfileImpactTicks * tick / steps:F2} ms), " +
                                              $"contacts {FARAeroComponents.PartHydrodynamics.ProfileContactTicks * tick / steps:F2} ms, mesh {FARAeroComponents.PartHydrodynamics.ProfileMeshTicks * tick / steps:F2} ms; " +
                                              $"{FARAeroComponents.PartHydrodynamics.ProfilePolygons / steps} polygons, {FARAeroComponents.PartHydrodynamics.ProfileTrianglesTested / steps} triangles tested; " +
                                              $"{FARAeroComponents.PartHydrodynamics.ProfileShadowBuilds} closed-in rebuilds ({FARAeroComponents.PartHydrodynamics.ProfileShadowTicks * tick:F0} ms all); {steps} steps");
                    long dryRuns = Math.Max(1, FARAeroComponents.PartHydrodynamics.ProfileDryRuns);
                    ctx.Record("dry faces", $"{FARAeroComponents.PartHydrodynamics.ProfileDryRuns} recomputes ({FARAeroComponents.PartHydrodynamics.ProfileDryRuns / wall:F1} a second), " +
                                            $"{FARAeroComponents.PartHydrodynamics.ProfileDryTicks * tick / dryRuns:F1} ms each, at most {FARAeroComponents.PartHydrodynamics.ProfileDryMaxTicks * tick:F1} ms in a step, " +
                                            $"{FARAeroComponents.PartHydrodynamics.ProfileDryTicks * tick / steps:F2} ms a step; each: {FARAeroComponents.PartHydrodynamics.ProfileDryFaces / dryRuns} faces, " +
                                            $"{FARAeroComponents.PartHydrodynamics.ProfileDryHemiRays / dryRuns} rays over them, {FARAeroComponents.PartHydrodynamics.ProfileDryOpenRays / dryRuns} open-surface rays; FAR's new meshes the same as before {FARAeroComponents.PartHydrodynamics.ProfileMeshSame} times" +
                                            (FARAeroComponents.PartHydrodynamics.DryCheck ? $"; checked against the uncached way: {FARAeroComponents.PartHydrodynamics.ProfileDryMismatches} of {FARAeroComponents.PartHydrodynamics.ProfileDryFaces} faces differ" : ""));
                }
                if (!double.IsNaN(turnHdg))
                    ctx.Record("turned", $"{turned:+0.0;-0.0} deg since the turn began, {v.srfSpeed:F1} m/s at the end; drift {Drift():+0.0;-0.0} deg at the end, up to {driftMax:F1} deg");
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "jetboat.csv"), csv.ToString());

                // (stopSec=S: then the engines cut, the rudder centred, and S s more (the camera still cycling) as it comes
                // to rest and its wake runs on without it; stopBrake=B m/s^2 more deceleration while it still moves (a
                // sea anchor's stand-in: a heavy hull alone coasts on for a minute), default 0)
                double stopSec = ctx.ParamDouble("stopSec", 0);
                if (stopSec > 0)
                {
                    Ctrl.SetThrottle(0);
                    Ctrl.Yaw = 0;
                    Ctrl.Pitch = 0;
                    foreach (ModuleEngines e in engines)
                        e.Shutdown();
                    double brake = ctx.ParamDouble("stopBrake", 0), tStop = Planetarium.GetUniversalTime(), stoppedAt = double.NaN, nextLog = 0;
                    var speeds = new List<string>();
                    // (its heave and pitch as it comes to rest, twice a second for 12 s: what the wave sim takes as its push)
                    var motion = new List<string>();
                    double nextMotion = 0, altWas = v.altitude;
                    ctx.Record("stop", $"at t {tStop - t0:F1} s, {v.srfSpeed:F1} m/s, EC {v.parts.Sum(q => q.Resources.Contains("ElectricCharge") ? q.Resources["ElectricCharge"].amount : 0):F0}, frame {(recorderGo != null ? recorderGo.GetComponent<PlaningVideoTests.Recorder>().Frame : -1)}");
                    while (Planetarium.GetUniversalTime() - tStop < stopSec)
                    {
                        yield return fixedWait;
                        double ts = Planetarium.GetUniversalTime() - tStop;
                        // (the root's rigidbody: srf_velocity is only refreshed once a frame, several physics steps under hyper-warp)
                        Vector3 vel = Vector3.ProjectOnPlane(v.rootPart.Rigidbody.velocity + (Vector3)Krakensbane.GetFrameVelocity(), v.upAxis);
                        if (brake > 0 && vel.magnitude > 0.05f)
                            v.ChangeWorldVelocity(-vel.normalized * Mathf.Min(vel.magnitude, (float)(brake * TimeWarp.fixedDeltaTime)));
                        if (double.IsNaN(stoppedAt) && vel.magnitude < 0.3f)
                            stoppedAt = ts;
                        if (ts >= nextLog)
                        {
                            speeds.Add($"{ts:F0}s {vel.magnitude:F1}");
                            nextLog += 2;
                        }
                        if (ts >= nextMotion && ts < 12)
                        {
                            Vector3 w = v.angularVelocity;
                            motion.Add($"{ts:F1}s {vel.magnitude:F1} m/s vs {v.verticalSpeed:+0.00;-0.00} pitch {w.x * Mathf.Rad2Deg:+0.0;-0.0} roll {w.y * Mathf.Rad2Deg:+0.0;-0.0} deg/s");
                            nextMotion += 0.5;
                        }
                        if (setView != null && Planetarium.GetUniversalTime() >= nextView)
                        {
                            setView(viewIdx++);
                            nextView += viewSec;
                        }
                    }
                    ctx.Record("stop motion", string.Join(", ", motion));
                    ctx.Record("stopped", $"{(double.IsNaN(stoppedAt) ? "still moving" : $"under 0.3 m/s {stoppedAt:F1} s after the stop")}, {v.srfSpeed:F2} m/s at the end; m/s: {string.Join(", ", speeds)}");
                }
            }
            finally
            {
                if (recorderGo != null)
                {
                    UnityEngine.Object.Destroy(recorderGo);
                    Time.captureFramerate = 0;
                    GameEvents.onShowUI.Fire();
                }
                if (anchorGo != null)
                {
                    if (FlightGlobals.ActiveVessel != null)
                        FlightCamera.fetch.SetTargetVessel(FlightGlobals.ActiveVessel);
                    UnityEngine.Object.Destroy(anchorGo);
                }
                if (hyper)
                    TestWarp.Hyper(ctx, false);
                foreach (var (e, was) in limits)
                    if (e != null)
                        e.thrustPercentage = was;
                Ctrl.Pitch = 0;
                Ctrl.Yaw = 0;
                Ctrl.SetThrottle(0);
                CheatOptions.NoCrashDamage = crashWas;
                CheatOptions.UnbreakableJoints = jointsWas;
                WaterParams.Restore();
            }
        }

        /// <summary>The clock moved on to when the sun stands highest over the water site, hour by hour over a day, the
        /// active vessel held on rails meanwhile: for demos lit from above. Or, given sunElevation (deg), to the morning
        /// before that when the sun stands that high (its hour angle taken as 90 deg less the height: the site's on the
        /// equator, the axis untilted).</summary>
        internal static IEnumerator Noon(TestContext ctx)
        {
            Vessel nv = FlightGlobals.ActiveVessel;
            nv.GoOnRails();
            CelestialBody nb = nv.mainBody;
            (double lat, double lon) = FarWaterTools.WaterSite(nb);
            double day = nb.solarDayLength > 0 ? nb.solarDayLength : nb.rotationPeriod;
            double ut0 = Planetarium.GetUniversalTime(), bestUt = ut0, bestEl = -90;
            for (int hh = 0; hh < 48; hh++)
            {
                Planetarium.SetUniversalTime(ut0 + hh * day / 48);
                yield return Wait.Frames(2);
                Vector3d upAt = nb.GetSurfaceNVector(lat, lon);
                double el = 90 - Vector3d.Angle(upAt, (Planetarium.fetch.Sun.position - nb.position).normalized);
                if (el > bestEl)
                {
                    bestEl = el;
                    bestUt = Planetarium.GetUniversalTime();
                }
            }
            double target = ctx.ParamDouble("sunElevation", double.NaN);
            if (!double.IsNaN(target) && target < bestEl)
                bestUt -= (bestEl - target) / 360 * day;
            Planetarium.SetUniversalTime(bestUt);
            yield return Wait.Frames(2);
            Vector3d upNow = nb.GetSurfaceNVector(lat, lon);
            bestEl = 90 - Vector3d.Angle(upNow, (Planetarium.fetch.Sun.position - nb.position).normalized);
            ctx.Record("time of day", $"UT {bestUt:F0} s: the sun {bestEl:F0} deg up at {lat:F2} N {lon:F2} E");
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);

        /// <summary>The recording camera's target: Back m behind the vessel's centre of mass along its course (its
        /// heading at the start until it makes way), kept level with it.</summary>
        internal class CameraAnchor : MonoBehaviour
        {
            public Vessel Vessel;
            public Vector3 Heading;
            public float Back;
            public float Height = float.NaN;   // (m above the sea, if set: a low camera kept out of the water)

            private void Update()   // (after physics, before the camera's LateUpdate)
            {
                if (Vessel == null)
                    return;
                Vector3 up = Vessel.upAxis, vel = Vector3.ProjectOnPlane(Vessel.srf_velocity, up);
                Vector3 h = vel.magnitude > 1 ? vel.normalized : Vector3.ProjectOnPlane(Heading, up).normalized;
                transform.position = Vessel.CoM - h * Back;
                if (!float.IsNaN(Height))
                    transform.position += up * (Height - (float)FlightGlobals.getAltitudeAtPos(transform.position, Vessel.mainBody));
                transform.rotation = Vessel.transform.rotation;
            }
        }
    }
}
