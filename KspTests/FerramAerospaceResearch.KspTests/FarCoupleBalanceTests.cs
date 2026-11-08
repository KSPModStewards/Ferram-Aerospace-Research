using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using KspTest;
using KspTest.AtmosphereAutopilot;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>Whether a vessel takes the pitching moment it is given (explicit, until ported).</summary>
    public static class FarCoupleBalanceTests
    {
        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        // The parts a craft file names that this install has not loaded, space-separated; empty when all are there.
        private static string MissingParts(string craftPath)
        {
            if (!File.Exists(craftPath))
                return "(no craft file)";
            var missing = new HashSet<string>();
            foreach (string raw in File.ReadAllLines(craftPath))
            {
                string line = raw.Trim();
                if (!line.StartsWith("part = ", StringComparison.Ordinal))
                    continue;
                string name = line.Substring(7).Trim();
                int id = name.LastIndexOf('_');
                if (id > 0)
                    name = name.Substring(0, id);
                if (PartLoader.getPartInfoByName(name) == null && PartLoader.getPartInfoByName(name.Replace('.', '_')) == null)
                    missing.Add(name);
            }
            return string.Join(" ", missing);
        }

        /// <summary>
        /// Does the vessel take the pitching moment it is given? Each craft is placed at a condition, wings level at an
        /// angle of attack, reaction wheels off, and held at each fixed pitch input in turn (re-placed for each). Once
        /// the surfaces have moved, the moment the integrator applied over a window of physics steps is set against the
        /// rate of the vessel's angular momentum (QueuedTorque.Balance), and each control surface's couple against its own
        /// inertia (WingLiveVsSim.SurfaceJoints: dw/step is the spin it adds in a step, against the 50 rad/s clamp).
        /// Needs nothing from a FAR fork. Params: crafts ("+"-separated), facility (SPH), point (altM:mach:aoaDeg, default
        /// 1000:0.7:3), inputs (default -1,0,1), settleSteps (25), windowSteps (25), maxAngVel (rad/s on every part's
        /// rigidbody, 0 keeps KSP's).
        /// </summary>
        [UITest("Far_CoupleBalance", Explicit = true, Kind = "Measure", StartsAt = StartState.SpaceCenter, TimeoutSec = 3600)]
        public static IEnumerator Far_CoupleBalance(TestContext ctx)
        {
            ctx.Record("farBuild", FarSim.FarVersion());
            ctx.Record("kjrLoaded", AssemblyLoader.loadedAssemblies.Any(a => a.name == "KerbalJointReinforcement"));
            string facility = (ctx.Param("facility") ?? "SPH").ToUpperInvariant();
            string[] crafts = (ctx.Param("crafts") ?? "Aeris 3A").Split('+');
            double[] pt = (ctx.Param("point") ?? "1000:0.7:3").Split(':').Select(x => double.Parse(x.Trim(), Inv)).ToArray();
            float[] inputs = (ctx.Param("inputs") ?? "-1,0,1").Split(',').Select(x => float.Parse(x.Trim(), Inv)).ToArray();
            int settle = (int)ctx.ParamDouble("settleSteps", 25), window = (int)ctx.ParamDouble("windowSteps", 25);
            float maxAngVel = (float)ctx.ParamDouble("maxAngVel", 0);
            bool relaunch = !(ctx.Param("relaunch", "on") ?? "on").Equals("off", StringComparison.OrdinalIgnoreCase);
            // camber=on|off: a fork FAR's ControlSurfaceCamber switch for the run (upstream has none: always on)
            var sw = Reflect.FindType("ferram4.FARWingAerodynamicModel")?.GetField("ControlSurfaceCamber",
                System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static);
            string camberParam = ctx.Param("camber");
            bool? camberWas = sw != null ? (bool?)(bool)sw.GetValue(null) : null;
            if (sw != null && camberParam != null)
                sw.SetValue(null, camberParam.Equals("on", StringComparison.OrdinalIgnoreCase));
            ctx.Record("controlSurfaceCamber", sw != null ? sw.GetValue(null).ToString() : "no switch (on)");
            // arcs=on: FAR's wing interaction overlay (a fork's FARDebugValues.showWingInteractions), paused at the end of
            // the window at input arcsInput (-1) and shot from above, the side and behind with the UI hidden
            var arcsField = Reflect.FindType("FerramAerospaceResearch.FARDebugValues")?.GetField("showWingInteractions");
            bool arcs = (ctx.Param("arcs", "off") ?? "off").Equals("on", StringComparison.OrdinalIgnoreCase) && arcsField != null;
            float arcsInput = (float)ctx.ParamDouble("arcsInput", -1);
            object arcsWas = arcsField?.GetValue(null);
            if (arcs) arcsField.SetValue(null, true);
            ctx.Record("arcs", arcs);
            QueuedTorque.Arm();
            WingLiveVsSim.Arm();
            var txt = new StringBuilder();
            var csv = new StringBuilder("craft,input,altM,mach,qkPa,aoaDeg,appliedKNm,dLdtKNm,lostKNm,maxDwPerStep\n");
            bool[] trajectories = FarFlightProbe.QuietOtherPredictors();
            var fixedWait = new WaitForFixedUpdate();
            try
            {
                foreach (string craftRaw in crafts)
                {
                    string craft = craftRaw.Trim();
                    string path = ctx.CraftPath(craft, facility);
                    if (!File.Exists(path))
                        path = Path.GetFullPath(Path.Combine(KSPUtil.ApplicationRootPath, "Ships", facility, craft + ".craft"));
                    string missing = MissingParts(path);
                    if (missing.Length > 0)
                    {
                        ctx.Record(craft + ".skipped", "missing parts: " + missing);
                        continue;
                    }
                    int parts = 0;
                    double lon = -60;
                    bool launched = false;
                    foreach (float input in inputs)
                    {
                        string k = craft + ".in" + input.ToString("+0.##;-0.##;0", Inv) + ".";
                        // a fresh vessel for each input (relaunch, default on): re-placing one still pitching from the last
                        // input tore big craft apart as they came off rails
                        if (!launched || relaunch)
                        {
                            ctx.Step($"Launch {craft} for input {input}");
                            if (launched)
                            {
                                Ctrl.Detach();
                                yield return Nav.EnsureState(ctx, facility == "VAB" ? StartState.Vab : StartState.Sph, path);
                            }
                            yield return Nav.EnsureState(ctx, StartState.Flight, path, facility == "VAB" ? "LaunchPad" : "Runway");
                            launched = true;
                            // off the ground at once: a big RP-1 stack on the pad lost wing panels and tanks within 3 s of loading
                            parts = FlightGlobals.ActiveVessel.parts.Count;
                            Flight.IgnoreMaxTemperature(true);
                            // in daylight when the overlay is shot, else where it always was
                            lon = arcs ? DaylightLon(FlightGlobals.ActiveVessel.mainBody, pt[0]) : -60;
                            if (arcs) arcsField.SetValue(null, true);   // FAR reloads its debug settings with the scene
                            yield return FarFlightProbe.Place(new FarFlightProbe.Condition { Name = "off the pad", AltM = pt[0], Mach = pt[1], AoADeg = pt[2], Lat = 0, Lon = lon });
                            yield return Wait.Seconds(5);   // FAR voxelizes the craft after load
                            ctx.Record(k + "partsLostAtLoad", parts - FlightGlobals.ActiveVessel.parts.Count);
                        }
                        ctx.Step($"{craft}: pitch input {input}");
                        Vessel v = FlightGlobals.ActiveVessel;
                        if (v == null || v.parts.Count < parts)
                        {
                            ctx.Record(k + "broken", v == null ? "no vessel" : $"{parts - v.parts.Count} parts lost");
                            break;
                        }
                        foreach (Part p in v.parts)
                            foreach (PartModule m in p.Modules)
                                if (m is ModuleReactionWheel rw)
                                {
                                    rw.wheelState = ModuleReactionWheel.WheelState.Disabled;
                                    rw.authorityLimiter = 0;
                                }
                        Ctrl.Sas(false);
                        Ctrl.Attach(true);
                        Ctrl.Set(pitch: 0, yaw: 0, roll: 0, throttle: 0);
                        var at = new FarFlightProbe.Condition { Name = "couple " + input, AltM = pt[0], Mach = pt[1], AoADeg = pt[2], Lat = 0, Lon = lon };
                        yield return FarFlightProbe.Place(at);
                        v = FlightGlobals.ActiveVessel;
                        if (maxAngVel > 0)
                            foreach (Part p in v.parts)
                                if (p.Rigidbody != null)
                                    p.Rigidbody.maxAngularVelocity = maxAngVel;
                        Ctrl.Set(pitch: input);
                        for (int i = 0; i < settle; ++i)
                            yield return fixedWait;
                        QueuedTorque.ResetWindow();
                        string joints = null;
                        for (int i = 0; i < window; ++i)
                        {
                            yield return fixedWait;
                            if (i == window / 2)
                                joints = WingLiveVsSim.SurfaceJoints(FlightGlobals.ActiveVessel);
                        }
                        v = FlightGlobals.ActiveVessel;
                        QueuedTorque.Balance(out double applied, out double dLdt, out int steps);
                        // the largest spin a control surface's couple adds in a step
                        double maxDw = 0;
                        foreach (System.Text.RegularExpressions.Match mm in System.Text.RegularExpressions.Regex.Matches(joints ?? "", @"dw/step (\S+) rad/s"))
                            if (double.TryParse(mm.Groups[1].Value, NumberStyles.Float, Inv, out double dw))
                                maxDw = Math.Max(maxDw, Math.Abs(dw));
                        ctx.Record(k + "balance", string.Format(Inv, "applied {0:F1} kN m, dL/dt {1:F1} kN m, lost {2:F1} kN m over {3} steps; q {4:F1} kPa mach {5:F2} aoa {6:F1}; max surface dw/step {7:G3} rad/s",
                            applied, dLdt, applied - dLdt, steps, v.dynamicPressurekPa, v.mach, Flight.AoA, maxDw));
                        csv.Append(string.Join(",", craft, input.ToString(Inv), v.altitude.ToString("F0", Inv), v.mach.ToString("F3", Inv),
                            v.dynamicPressurekPa.ToString("F2", Inv), Flight.AoA.ToString("F2", Inv), applied.ToString("F2", Inv),
                            dLdt.ToString("F2", Inv), (applied - dLdt).ToString("F2", Inv), maxDw.ToString("G4", Inv))).Append('\n');
                        txt.Append(craft).Append(" input ").Append(input.ToString(Inv)).Append(": ").Append(QueuedTorque.Report()).Append("\n    ")
                           .Append((joints ?? "").Replace(" || ", "\n    ")).Append("\n\n");
                        if (arcs && input == arcsInput)
                            yield return ArcShots(ctx, v, craft.Replace(' ', '_') + "_in" + input.ToString("+0.##;-0.##;0", Inv));
                        Ctrl.Set(pitch: 0);
                    }
                    Ctrl.Detach();
                }
            }
            finally
            {
                Ctrl.Detach();
                FarFlightProbe.ResumeOtherPredictors(trajectories);
                if (camberWas.HasValue) sw.SetValue(null, camberWas.Value);
                if (arcsField != null && arcsWas != null) arcsField.SetValue(null, arcsWas);
            }
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "couple-balance.csv"), csv.ToString());
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "couple-balance.txt"), txt.ToString());
        }

        // A longitude on the equator in the afternoon (the sun 30 deg west of overhead, so the vessel throws a shadow), the
        // first one east of that whose terrain is at least 1500 m below altM.
        private static double DaylightLon(CelestialBody body, double altM)
        {
            CelestialBody sun = Planetarium.fetch != null ? Planetarium.fetch.Sun : null;
            if (sun == null || sun == body) return -60;
            double lon = body.GetLongitude(sun.position) + 30;
            for (int i = 0; i < 36; ++i, lon += 10)
                if (body.TerrainAltitude(0, lon) < altM - 1500)
                    return ((lon + 540) % 360) - 180;
            return -60;
        }

        // The UI hidden, the vessel shot from above, the side and behind at a distance sized to it:
        // wingarcs_<name>_<view>.png in the results.
        private static IEnumerator ArcShots(TestContext ctx, Vessel v, string name)
        {
            FlightCamera cam = FlightCamera.fetch;
            if (cam == null || v == null) yield break;
            float size = 0;
            foreach (Part p in v.parts)
                size = Math.Max(size, 2 * (p.transform.position - v.CoM).magnitude);
            GameEvents.onHideUI.Fire();
            try
            {
                foreach (var (view, pitch, hdg, dist) in new[] { ("above", 1.35f, 0f, 1.6f), ("side", 0.12f, 1.5708f, 1.5f), ("rear", 0.5f, 3.1416f, 1.5f) })
                {
                    cam.camPitch = pitch;
                    cam.camHdg = hdg;
                    cam.SetDistance(Math.Max(20f, dist * size));
                    for (int i = 0; i < 6; ++i) yield return new WaitForEndOfFrame();
                    ScreenCapture.CaptureScreenshot(Path.Combine(ctx.ResultsDir, $"wingarcs_{name}_{view}.png"));
                    yield return new WaitForEndOfFrame();
                    yield return new WaitForEndOfFrame();
                }
            }
            finally
            {
                GameEvents.onShowUI.Fire();
            }
        }
    }
}
