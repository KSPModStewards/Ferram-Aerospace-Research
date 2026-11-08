using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using KspTest;
using KspTest.AtmosphereAutopilot;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The wing aspect ratio change (FAR's fix/spanwise-aspect-ratio) in flight: the same crafts flown on one FAR build
    /// per run, the runs compared afterwards (upstream against the patch).
    /// </summary>
    public static class FarArTests
    {
        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        /// <summary>
        /// Each craft air-started level at the point and flown on AtmosphereAutopilot's cruise (course 90, the altitude,
        /// the speed from the Mach), then climbed on the same autopilot to climbTo and cruised again there. Each cruise,
        /// once steady (altitude within 60 m, speed within 4 m/s, vertical speed under 3 m/s for steadySec, or
        /// settleTimeoutSec and flagged), is measured over windowSec: AoA, pitch input, FAR's lift and drag (kN, across
        /// and along the airflow; wing models and voxel parts apart), CL and CD on the wing area (the FAR wing parts' S
        /// summed, control surfaces in), L/D, lift over weight and g; then FAR's prediction at the same state with the
        /// airflow turned 1 deg either way gives the lift slope (per radian on the wing area), and the parts' own numbers
        /// go to ar-parts.csv. The climb is measured from the autopilot's new target until within 60 m of it (or
        /// climbTimeoutSec, flagged): its time, mean climb rate, AoA and pitch input (mean and peak), lowest speed and
        /// peak g. After each cruise, FAR's aero visualization (Cl tint, and any other in tints; with arrows, its lift and
        /// drag arrows too) is shot with the UI hidden from above, the side and behind, in daylight. Reaction wheels off.
        /// With ffmpeg given, each craft's flight is recorded (the KSP window, from the air start to the last shot) and
        /// kept only when something went wrong: parts lost, a cruise never steady, its AoA unsteady (SD over 0.5 deg) or
        /// its pitch input near its stop (over 0.9), or the climb not reaching its altitude.
        /// Params: crafts ("+"-separated stock SPH crafts, default every stock FAR plane but the Stearwing), point
        /// (altM:mach, 3000:0.5), climbTo (altM:mach, 7000:0.5; none for one cruise only), steadySec (8), windowSec (10),
        /// settleTimeoutSec (120), climbTimeoutSec (300), tints (cl; also none, cd, stall), clSat (0.25; FAR's default 0.5
        /// leaves cruise lift faint), arrows (on), ffmpeg (path to ffmpeg.exe; without it, no video), hyperWarp (4; 1 for real time;
        /// above 1, TimeControl hyper-warp at that rate from the air start, physics step unchanged, off for the shots;
        /// the speedup got is recorded per craft, and the video plays that much faster).
        /// Writes ar-cruise.csv (every frame, with its phase), ar-parts.csv, ar-summary.csv (one row a craft and phase:
        /// cruise, climb, cruise2), ar_*.png and ar_*.mp4.
        /// </summary>
        [UITest("Far_ArCruise", Explicit = true, Kind = "Measure", Requires = "AtmosphereAutopilot", Mods = "AtmosphereAutopilot;TimeControl", StartsAt = StartState.SpaceCenter, TimeoutSec = 10800)]
        public static IEnumerator Far_ArCruise(TestContext ctx)
        {
            ctx.Record("farBuild", FarSim.FarVersion());
            Assert.True(Aa.Present, "AtmosphereAutopilot is installed");
            // the SkyEye flies autostrutted (unstrutted, its wings break up in held flight on any build); the Firehound with
            // its flaps up (F0: the stock craft is saved with its flaps down, which FAR's editor simulation ignores)
            string[] crafts = (ctx.Param("crafts") ?? "Aeris 3A+Aeris 4A+Albatross 3+Dove+FAR Firehound MS F0+FAR SkyEye AS+Gull+Mallard+" +
                                                      "Ravenspear Mk3+Ravenspear Mk4+Skywinder AE1+Thunderbird+Velociteze").Split('+');
            var points = new List<(string phase, double alt, double mach)>();
            string[] pt = (ctx.Param("point") ?? "3000:0.5").Split(':');
            points.Add(("cruise", double.Parse(pt[0], Inv), double.Parse(pt[1], Inv)));
            string climbTo = ctx.Param("climbTo") ?? "7000:0.5";
            if (!climbTo.Equals("none", StringComparison.OrdinalIgnoreCase))
            {
                string[] p2 = climbTo.Split(':');
                points.Add(("cruise2", double.Parse(p2[0], Inv), double.Parse(p2[1], Inv)));
            }
            double steadySec = ctx.ParamDouble("steadySec", 8), windowSec = ctx.ParamDouble("windowSec", 10);
            double timeout = ctx.ParamDouble("settleTimeoutSec", 120), climbTimeout = ctx.ParamDouble("climbTimeoutSec", 300);
            string[] tints = (ctx.Param("tints") ?? "cl").Split(',').Select(s => s.Trim().ToLowerInvariant()).Where(s => s.Length > 0).ToArray();
            double clSat = ctx.ParamDouble("clSat", 0.25);
            bool arrows = !(ctx.Param("arrows", "on") ?? "on").Equals("off", StringComparison.OrdinalIgnoreCase);
            string ffmpeg = ctx.Param("ffmpeg");
            ctx.Record("video", ffmpeg != null && File.Exists(ffmpeg));
            double hyper = ctx.ParamDouble("hyperWarp", 4);
            void Hyper(bool on)
            {
                if (hyper <= 1) return;
                if (on) Assert.True(Flight.HyperWarp(hyper, 1.0), "TimeControl hyper-warp on");
                else Flight.HyperWarpOff();
            }
            var csv = new StringBuilder("craft,phase,ut,altM,srfSpeed,mach,qKPa,aoaDeg,pitchInput,verticalSpeed,liftKN,dragKN,wingLiftKN,bodyLiftKN,CL,CD,LD,liftOverWeight,g\n");
            var partsCsv = new StringBuilder("craft,phase,part,persistentId,kind,areaM2,effective_AR,Cl,Cd,liftslope,stall,liftKN,dragKN,crossKN,tintCl,shielded\n");
            var summary = new StringBuilder("craft,phase,steady,durationS,altM,srfSpeed,mach,qKPa,massT,wingAreaM2,aoaDeg,aoaSd,aoaMax,pitchInput,pitchInputMax," +
                                            "verticalSpeed,speedMin,liftKN,dragKN,wingLiftKN,bodyLiftKN,CL,CD,LD,liftOverWeight,g,gMax,liftSlopePerRad,predLiftKN,meanAR\n");
            bool[] trajectories = FarFlightProbe.QuietOtherPredictors();
            Recorder rec = null;
            try
            {
                foreach (string craftRaw in crafts)
                {
                    string craft = craftRaw.Trim();
                    string path = ctx.CraftPath(craft, "SPH");
                    if (!File.Exists(path))
                        path = Path.GetFullPath(Path.Combine(KSPUtil.ApplicationRootPath, "Ships", "SPH", craft + ".craft"));
                    string missing = MissingParts(path);
                    if (missing.Length > 0)
                    {
                        ctx.Record(craft + ".skipped", "missing parts: " + missing);
                        continue;
                    }
                    ctx.Step("Air start " + craft);
                    yield return Nav.EnsureState(ctx, StartState.Flight, path, "Runway");
                    yield return Wait.Seconds(5);   // FAR voxelizes the craft after load
                    Vessel v = FlightGlobals.ActiveVessel;
                    if (ffmpeg != null && File.Exists(ffmpeg))
                        rec = Recorder.Start(ffmpeg, Path.Combine(ctx.ResultsDir, $"ar_{craft.Replace(' ', '_')}.mp4"));
                    Flight.IgnoreMaxTemperature(true);
                    foreach (Part p in v.parts)
                        foreach (PartModule m in p.Modules)
                            if (m is ModuleReactionWheel rw)
                            {
                                rw.wheelState = ModuleReactionWheel.WheelState.Disabled;
                                rw.authorityLimiter = 0;
                            }
                    Ctrl.Sas(false);
                    Ctrl.Stage();
                    CelestialBody body = v.mainBody;
                    double SpeedAt(double alt, double m)
                    {
                        double pr = body.GetPressure(alt);
                        return m * body.GetSpeedOfSound(pr, body.GetDensity(pr, body.GetTemperature(alt)));
                    }

                    var start = new FarFlightProbe.Condition { Name = "air start", AltM = points[0].alt, Mach = points[0].mach, AoADeg = 3, Lat = 0, Lon = DaylightLon(body, points[0].alt) };
                    yield return FarFlightProbe.Place(start);
                    Flight.PointAtAoA(start.AoADeg);
                    Ctrl.RetractGear();
                    Ctrl.SetThrottle(1f);
                    int parts = FlightGlobals.ActiveVessel.parts.Count;
                    double area = WingArea(FlightGlobals.ActiveVessel);
                    bool abnormal = false, broken = false;
                    double ut0 = Planetarium.GetUniversalTime(), wall0 = Time.realtimeSinceStartup;
                    Hyper(true);

                    for (int ph = 0; ph < points.Count && !broken; ph++)
                    {
                        var (phase, altM, mach) = points[ph];
                        double speed = SpeedAt(altM, mach);
                        Aa.Cruise(FlightGlobals.ActiveVessel, altM, speed, 90);

                        if (ph > 0)
                        {
                            // the climb: from the new target until within 60 m of it
                            ctx.Step($"{craft}: climb to {altM / 1000:0.#} km");
                            var climb = new Dictionary<string, List<double>>();
                            void AddC(string k, double x) { if (!climb.TryGetValue(k, out var l)) climb[k] = l = new List<double>(); l.Add(x); }
                            double t0 = Planetarium.GetUniversalTime(), climbEnd = t0 + climbTimeout;
                            bool reached = false;
                            var climbWait = new WaitForFixedUpdate();
                            while (Planetarium.GetUniversalTime() < climbEnd)
                            {
                                yield return climbWait;
                                Vessel w = FlightGlobals.ActiveVessel;
                                if (w == null || w.parts.Count < parts) { broken = true; break; }
                                if (Math.Abs(w.altitude - altM) < 60) { reached = true; break; }
                                if (!Forces(w, out Vector3 cw, out Vector3 cb)) continue;
                                LiftDrag(w, cw + cb, w.srf_velocity, out double L, out double D);
                                double q = w.dynamicPressurekPa;
                                AddC("aoa", Flight.AoA); AddC("input", w.ctrlState.pitch); AddC("vs", w.verticalSpeed); AddC("speed", w.srfSpeed);
                                AddC("g", w.geeForce); AddC("alt", w.altitude); AddC("mach", w.mach); AddC("q", q); AddC("L", L); AddC("D", D);
                                AddC("LD", L / D); AddC("mass", w.totalMass);
                                csv.Append(string.Join(",", Csv(craft), "climb", Planetarium.GetUniversalTime().ToString("F3", Inv), F(w.altitude, "F1"),
                                    F(w.srfSpeed, "F2"), F(w.mach, "F4"), F(q, "F4"), F(Flight.AoA, "F4"), F(w.ctrlState.pitch, "F5"), F(w.verticalSpeed, "F2"),
                                    F(L, "G6"), F(D, "G6"), "", "", F(L / (q * area), "F5"), F(D / (q * area), "F5"), F(L / D, "F3"), "", F(w.geeForce, "F3"))).Append('\n');
                            }
                            double C(string k, Func<IEnumerable<double>, double> f) => climb.TryGetValue(k, out var l) && l.Count > 0 ? f(l) : double.NaN;
                            double dur = Planetarium.GetUniversalTime() - t0;
                            ctx.Record($"{craft}.climb.reached", reached);
                            ctx.Record($"{craft}.climb.durationS", Math.Round(dur, 1));
                            ctx.Record($"{craft}.climb.verticalSpeed", Math.Round(C("vs", l => l.Average()), 2));
                            ctx.Record($"{craft}.climb.pitchInputMax", Math.Round(C("input", l => l.Max()), 3));
                            abnormal |= !reached;
                            summary.Append(string.Join(",", Csv(craft), "climb", reached ? "1" : "0", F(dur, "F1"), F(C("alt", l => l.Average()), "F1"),
                                F(C("speed", l => l.Average()), "F2"), F(C("mach", l => l.Average()), "F4"), F(C("q", l => l.Average()), "F3"),
                                F(C("mass", l => l.Average()), "F3"), F(area, "F3"), F(C("aoa", l => l.Average()), "F4"), "", F(C("aoa", l => l.Max()), "F4"),
                                F(C("input", l => l.Average()), "F5"), F(C("input", l => l.Max()), "F5"), F(C("vs", l => l.Average()), "F3"),
                                F(C("speed", l => l.Min()), "F2"), F(C("L", l => l.Average()), "F3"), F(C("D", l => l.Average()), "F3"), "", "", "", "",
                                F(C("LD", l => l.Average()), "F3"), "", F(C("g", l => l.Average()), "F3"), F(C("g", l => l.Max()), "F3"), "", "", "")).Append('\n');
                            if (broken)
                                break;
                        }

                        ctx.Step($"{craft}: {phase} at {altM / 1000:0.#} km, Mach {mach:0.##}: steady then measured");
                        double since = double.NaN, tSettle = Planetarium.GetUniversalTime(), deadline = tSettle + timeout;
                        bool steady = false;
                        while (Planetarium.GetUniversalTime() < deadline)
                        {
                            Vessel w = FlightGlobals.ActiveVessel;
                            if (w == null || w.parts.Count < parts) break;
                            bool now = Math.Abs(w.altitude - altM) < 60 && Math.Abs(w.srfSpeed - speed) < 4 && Math.Abs(w.verticalSpeed) < 3;
                            double ut = Planetarium.GetUniversalTime();
                            if (!now) since = double.NaN;
                            else if (double.IsNaN(since)) since = ut;
                            else if (ut - since >= steadySec) { steady = true; break; }
                            yield return null;
                        }
                        double settle = Planetarium.GetUniversalTime() - tSettle;
                        v = FlightGlobals.ActiveVessel;
                        if (v == null || v.parts.Count < parts)
                        {
                            broken = true;
                            break;
                        }
                        ctx.Record($"{craft}.{phase}.steady", steady);

                        var cols = new Dictionary<string, List<double>>();
                        void Add(string k, double x) { if (!cols.TryGetValue(k, out var l)) cols[k] = l = new List<double>(); l.Add(x); }
                        var fixedWait = new WaitForFixedUpdate();
                        double end = Planetarium.GetUniversalTime() + windowSec;
                        bool partsTaken = false;
                        while (Planetarium.GetUniversalTime() < end)
                        {
                            yield return fixedWait;
                            v = FlightGlobals.ActiveVessel;
                            if (!Forces(v, out Vector3 wing, out Vector3 bodyF)) continue;
                            Vector3 vel = v.srf_velocity;
                            LiftDrag(v, wing + bodyF, vel, out double L, out double D);
                            LiftDrag(v, wing, vel, out double Lw, out _);
                            LiftDrag(v, bodyF, vel, out double Lb, out _);
                            double q = v.dynamicPressurekPa, weight = v.totalMass * FlightGlobals.getGeeForceAtPosition(v.CoM).magnitude;
                            double CL = L / (q * area), CD = D / (q * area);
                            Add("alt", v.altitude); Add("speed", v.srfSpeed); Add("mach", v.mach); Add("q", q); Add("aoa", Flight.AoA);
                            Add("input", v.ctrlState.pitch); Add("vs", v.verticalSpeed); Add("L", L); Add("D", D); Add("Lw", Lw); Add("Lb", Lb);
                            Add("CL", CL); Add("CD", CD); Add("LD", L / D); Add("LW", L / weight); Add("g", v.geeForce); Add("mass", v.totalMass);
                            csv.Append(string.Join(",", Csv(craft), phase, Planetarium.GetUniversalTime().ToString("F3", Inv), F(v.altitude, "F1"), F(v.srfSpeed, "F2"),
                                F(v.mach, "F4"), F(q, "F4"), F(Flight.AoA, "F4"), F(v.ctrlState.pitch, "F5"), F(v.verticalSpeed, "F2"), F(L, "G6"), F(D, "G6"),
                                F(Lw, "G6"), F(Lb, "G6"), F(CL, "F5"), F(CD, "F5"), F(L / D, "F3"), F(L / weight, "F4"), F(v.geeForce, "F3"))).Append('\n');
                            if (!partsTaken && Planetarium.GetUniversalTime() > end - windowSec / 2)
                            {
                                partsTaken = true;
                                PartForces(v, craft, phase, partsCsv);
                            }
                        }
                        double M(string k) => cols.TryGetValue(k, out var l) && l.Count > 0 ? l.Average() : double.NaN;
                        double Mx(string k) => cols.TryGetValue(k, out var l) && l.Count > 0 ? l.Max() : double.NaN;
                        FarFlightProbe.Stats(cols.TryGetValue("aoa", out var al) ? al : new List<double>(), out double aoa, out double aoaSd);

                        // FAR's prediction at this state, the airflow turned 1 deg about the pitch axis either way (after the
                        // window: a prediction leaves state on the wings the next physics frame can read)
                        double slope = double.NaN, predL = double.NaN;
                        {
                            Vector3 vel = v.srf_velocity, axis = FarApi.PitchUpAxis(v);
                            double ut = Planetarium.GetUniversalTime(), q = v.dynamicPressurekPa;
                            var lift = new Dictionary<int, double>();
                            foreach (int d in new[] { -1, 0, 1 })
                            {
                                // turning the airflow toward the roof lowers the AoA
                                Vector3 vd = Quaternion.AngleAxis(d, axis) * vel;
                                FarApi.ClearMemo(v);
                                if (FarApi.PredictAtFull(v, vd, v.latitude, v.longitude, v.altitude, ut, out Vector3 f, out _))
                                {
                                    LiftDrag(v, f, vd, out double Ld, out _);
                                    lift[d] = Ld;
                                }
                            }
                            FarApi.ClearMemo(v);
                            if (lift.Count == 3)
                            {
                                slope = (lift[-1] - lift[1]) / (2 * Mathf.Deg2Rad * q * area);
                                predL = lift[0];
                            }
                        }
                        double meanAr = MeanAr(v);
                        string k0 = $"{craft}.{phase}.";
                        ctx.Record(k0 + "aoaDeg", string.Format(Inv, "{0:F3} ± {1:F3}", aoa, aoaSd));
                        ctx.Record(k0 + "pitchInput", Math.Round(M("input"), 4));
                        ctx.Record(k0 + "CL", Math.Round(M("CL"), 4));
                        ctx.Record(k0 + "LD", Math.Round(M("LD"), 2));
                        ctx.Record(k0 + "liftSlopePerRad", Math.Round(slope, 3));
                        ctx.Record(k0 + "meanAR", Math.Round(meanAr, 3));
                        abnormal |= !steady || aoaSd > 0.5 || Math.Abs(M("input")) > 0.9;
                        summary.Append(string.Join(",", Csv(craft), phase, steady ? "1" : "0", F(settle, "F1"), F(M("alt"), "F1"), F(M("speed"), "F2"),
                            F(M("mach"), "F4"), F(M("q"), "F3"), F(M("mass"), "F3"), F(area, "F3"), F(aoa, "F4"), F(aoaSd, "F4"), F(Mx("aoa"), "F4"),
                            F(M("input"), "F5"), F(Mx("input"), "F5"), F(M("vs"), "F3"), F(cols.TryGetValue("speed", out var sl) && sl.Count > 0 ? sl.Min() : double.NaN, "F2"),
                            F(M("L"), "F3"), F(M("D"), "F3"), F(M("Lw"), "F3"), F(M("Lb"), "F3"), F(M("CL"), "F5"), F(M("CD"), "F5"), F(M("LD"), "F3"),
                            F(M("LW"), "F4"), F(M("g"), "F3"), F(Mx("g"), "F3"), F(slope, "F4"), F(predL, "F3"), F(meanAr, "F3"))).Append('\n');

                        ctx.Step($"{craft}: {phase} aero visualization shots");
                        Hyper(false);
                        yield return VizShots(ctx, FlightGlobals.ActiveVessel, craft.Replace(' ', '_') + (ph > 0 ? "_" + phase : ""), tints, clSat, arrows);
                        if (ph + 1 < points.Count) Hyper(true);
                        File.WriteAllText(Path.Combine(ctx.ResultsDir, "ar-summary.csv"), summary.ToString());
                    }

                    Hyper(false);
                    if (hyper > 1)
                        ctx.Record(craft + ".speedup", Math.Round((Planetarium.GetUniversalTime() - ut0) / (Time.realtimeSinceStartup - wall0), 2));
                    v = FlightGlobals.ActiveVessel;
                    if (broken)
                    {
                        ctx.Record(craft + ".broken", v == null ? "no vessel" : $"{parts - v.parts.Count} parts lost");
                        // the aftermath on the video too
                        yield return Wait.Seconds(3);
                    }
                    if (v != null) Aa.Off(v);
                    rec?.Stop(broken || abnormal, ctx, craft);
                    rec = null;
                    File.WriteAllText(Path.Combine(ctx.ResultsDir, "ar-summary.csv"), summary.ToString());
                }
            }
            finally
            {
                Hyper(false);
                rec?.Stop(true, ctx, "interrupted");
                FarFlightProbe.ResumeOtherPredictors(trajectories);
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "ar-cruise.csv"), csv.ToString());
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "ar-parts.csv"), partsCsv.ToString());
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "ar-summary.csv"), summary.ToString());
            }
        }

        /// <summary>
        /// Flaps and spoilers in FAR's editor sim: each craft's AoA sweep (alphaMin to alphaMax by alphaStep, at the Mach)
        /// in each configuration of configs, fN = flap level N (0 to 3; a flap deflects maxdeflectFlap x N / 3), sN = flap
        /// level N with the spoilers out, dX = every flap deflected X deg (its maxdeflectFlap set to X at level 3; negative
        /// is trailing edge up), put back after the craft. Per configuration: CL at 0 deg, the lift slope (per deg, least
        /// squares over alphaMin..6 deg), the zero-lift AoA, CLmax and its AoA, CD at 0 deg, and the change in CL and CD at
        /// 0 deg from f0 (or d0); per control surface its role, maximum flap deflection, chord fraction, area and effective
        /// aspect ratio. Params: crafts ("+"-separated, default the Real Boeing 737-800 CS), facility (SPH), mach (0.2),
        /// alphaMin (-4), alphaMax (24), alphaStep (1), configs (f0,f1,f2,f3,s0,s3). Writes highlift.csv (every point),
        /// highlift-summary.csv and highlift-surfaces.csv.
        /// </summary>
        [UITest("Far_HighLift", Explicit = true, Kind = "Measure", StartsAt = StartState.SpaceCenter, TimeoutSec = 3600)]
        public static IEnumerator Far_HighLift(TestContext ctx)
        {
            ctx.Record("farBuild", FarSim.FarVersion());
            string facility = (ctx.Param("facility") ?? "SPH").ToUpperInvariant();
            StartState state = facility == "VAB" ? StartState.Vab : StartState.Sph;
            string[] crafts = (ctx.Param("crafts") ?? "Real Boeing 737-800 CS").Split('+');
            double mach = ctx.ParamDouble("mach", 0.2);
            double aMin = ctx.ParamDouble("alphaMin", -4), aMax = ctx.ParamDouble("alphaMax", 24), aStep = ctx.ParamDouble("alphaStep", 1);
            var configs = (ctx.Param("configs") ?? "f0,f1,f2,f3,s0,s3").Split(',').Select(s => s.Trim().ToLowerInvariant())
                .Where(s => s.Length == 2 && (s[0] == 'f' || s[0] == 's') && s[1] >= '0' && s[1] <= '3' ||
                            s.Length > 1 && s[0] == 'd' && double.TryParse(s.Substring(1), NumberStyles.Float, Inv, out _)).ToArray();
            var alphas = new List<double>();
            for (double a = aMin; a <= aMax + 1e-6; a += aStep) alphas.Add(Math.Round(a, 6));
            var points = new StringBuilder("craft,config,flaps,spoilers,alpha,Cl,Cd,Cm\n");
            var summary = new StringBuilder("craft,config,flaps,spoilers,Cl0,ClPerDeg,alphaZeroLift,ClMax,alphaClMax,Cd0,dCl0,dCd0\n");
            var surfaces = new StringBuilder("craft,part,x,role,maxdeflectFlap,maxdeflect,ctrlSurfFrac,areaM2,effective_AR\n");
            foreach (string craftRaw in crafts)
            {
                string craft = craftRaw.Trim();
                ctx.Step("Craft " + craft);
                string path = ctx.CraftPath(craft, facility);
                if (!File.Exists(path))
                    path = Path.GetFullPath(Path.Combine(KSPUtil.ApplicationRootPath, "Ships", facility, craft + ".craft"));
                string missing = MissingParts(path);
                if (missing.Length > 0)
                {
                    ctx.Record(craft + ".skipped", "missing parts: " + missing);
                    continue;
                }
                yield return Nav.EnsureState(ctx, state, path);
                Ugui.DismissPopups();
                yield return Wait.Until(FarSim.Ready, 60, "FAR editor sim ready", shot: false);
                yield return Wait.Seconds(8);   // voxelization settles after the craft loads

                foreach (Part p in EditorLogic.fetch.ship.parts)
                    foreach (PartModule m in p.Modules)
                    {
                        if (!IsFarWing(m)) continue;
                        bool ctrl = m.GetType().Name == "FARControllableSurface";
                        string role = !ctrl ? "wing"
                            : Reflect.Get(m, "isFlap") is bool f && f ? "flap"
                            : Reflect.Get(m, "isSpoiler") is bool sp && sp ? "spoiler" : "control";
                        surfaces.Append(string.Join(",", Csv(craft), Csv(p.partInfo.title), F(p.transform.position.x, "F2"), role,
                            ctrl ? F(Num(m, "maxdeflectFlap"), "F1") : "", ctrl ? F(Num(m, "maxdeflect"), "F1") : "",
                            ctrl ? F(Num(m, "ctrlSurfFrac"), "F3") : "", F(Num(m, "S"), "F3"), F(Num(m, "effective_AR"), "F3"))).Append('\n');
                    }

                // the flaps and their own maximum deflection, put back after the craft
                var flapTravel = new List<(PartModule m, object travel)>();
                foreach (Part p in EditorLogic.fetch.ship.parts)
                    foreach (PartModule m in p.Modules)
                        if (m.GetType().Name == "FARControllableSurface" && Reflect.Get(m, "isFlap") is bool isFlap && isFlap)
                            flapTravel.Add((m, Reflect.Get(m, "maxdeflectFlap")));

                FarSim.Instant(5, 0, 0, mach, 0, 0, false);   // the sim carries state between calls: settle it first
                double cl00 = double.NaN, cd00 = double.NaN;
                foreach (string c in configs)
                {
                    int flaps;
                    bool spoilers = c[0] == 's';
                    if (c[0] == 'd')
                    {
                        float deg = float.Parse(c.Substring(1), NumberStyles.Float, Inv);
                        foreach (var (m, _) in flapTravel)
                            Reflect.Set(m, "maxdeflectFlap", deg);
                        flaps = 3;
                    }
                    else
                    {
                        foreach (var (m, travel) in flapTravel)
                            Reflect.Set(m, "maxdeflectFlap", travel);
                        flaps = c[1] - '0';
                    }
                    var cl = new double[alphas.Count];
                    var cd = new double[alphas.Count];
                    for (int i = 0; i < alphas.Count; ++i)
                    {
                        var o = FarSim.Instant(alphas[i], 0, 0, mach, 0, flaps, spoilers);
                        cl[i] = o["Cl"]; cd[i] = o["Cd"];
                        points.Append(string.Join(",", Csv(craft), c, flaps.ToString(Inv), spoilers ? "1" : "0", F(alphas[i], "G6"),
                            F(cl[i], "G9"), F(cd[i], "G9"), F(o["Cm"], "G9"))).Append('\n');
                    }
                    var lin = Enumerable.Range(0, alphas.Count).Where(i => alphas[i] <= 6).ToArray();
                    double mx = lin.Average(i => alphas[i]), my = lin.Average(i => cl[i]);
                    double slope = lin.Sum(i => (alphas[i] - mx) * (cl[i] - my)) / lin.Sum(i => (alphas[i] - mx) * (alphas[i] - mx));
                    int i0 = alphas.FindIndex(a => Math.Abs(a) < 1e-6);
                    double cl0 = i0 >= 0 ? cl[i0] : my - slope * mx, cd0 = i0 >= 0 ? cd[i0] : double.NaN;
                    int iMax = Enumerable.Range(0, alphas.Count).OrderByDescending(i => cl[i]).First();
                    if (c == "f0" || c == "d0") { cl00 = cl0; cd00 = cd0; }
                    string k = $"{craft}.{c}.";
                    ctx.Record(k + "Cl0", Math.Round(cl0, 4));
                    ctx.Record(k + "ClPerDeg", Math.Round(slope, 5));
                    ctx.Record(k + "ClMax", string.Format(Inv, "{0:F3} at {1:0.#} deg", cl[iMax], alphas[iMax]));
                    summary.Append(string.Join(",", Csv(craft), c, flaps.ToString(Inv), spoilers ? "1" : "0", F(cl0, "F5"), F(slope, "F6"),
                        F(-cl0 / slope, "F3"), F(cl[iMax], "F5"), F(alphas[iMax], "G6"), F(cd0, "F5"), F(cl0 - cl00, "F5"), F(cd0 - cd00, "F5"))).Append('\n');
                }
                foreach (var (m, travel) in flapTravel)
                    Reflect.Set(m, "maxdeflectFlap", travel);
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "highlift.csv"), points.ToString());
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "highlift-summary.csv"), summary.ToString());
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "highlift-surfaces.csv"), surfaces.ToString());
            }
        }

        /// <summary>
        /// Each craft air-started level at the point, settled on AtmosphereAutopilot's cruise (course 90, the altitude, the
        /// speed from the Mach), then flown around a full circle: the autopilot's course turned at turnRateDegS until it
        /// has come round 360 deg. Every sampleSec the flight's FAR force and pitching moment (from the last physics
        /// frame) are set against FAR's prediction API at the same position, velocity and time (the vessel's attitude as
        /// it is), as lift, drag and pitching moment; the next two frames after a prediction are not sampled, since a
        /// prediction leaves state on the wings. Each sample also records heading, bank, AoA, sideslip and the wings'
        /// area-weighted effective aspect ratio, which should not change as the vessel turns. A craft passes when, around
        /// the circle (the middle 90 % of the samples: 5th to 95th percentile), the predicted-over-flown lift and drag
        /// each stay within ratioTol and the aspect ratio within arTol of its mean. The prediction's constant offset from
        /// the flight (median) is recorded, and flagged over offsetWarn, but does not decide the pass: upstream FAR's own
        /// prediction API reads about 11 % more lift than its flight on the Gull, at every heading. Reaction wheels off.
        /// With ffmpeg given, a failing or broken craft's flight is kept on video. Params: crafts ("+"-separated; default
        /// the Gull, steady in every cruise run), point (altM:mach, 3000:0.5), turnRateDegS (3), sampleSec (0.5), steadySec
        /// (8), settleTimeoutSec (120), ratioTol (0.03), arTol (0.005), offsetWarn (0.05), ffmpeg, hyperWarp (4; 1 for real time; above 1,
        /// TimeControl hyper-warp at that rate from the air start to the end of the circle, physics step unchanged; the
        /// speedup got is recorded per craft). Writes ar-circle.csv (every sample) and ar-circle-summary.csv.
        /// </summary>
        [UITest("Far_ArCircle", Explicit = true, Kind = "Check", Requires = "AtmosphereAutopilot", Mods = "AtmosphereAutopilot;TimeControl", StartsAt = StartState.SpaceCenter, TimeoutSec = 10800)]
        public static IEnumerator Far_ArCircle(TestContext ctx)
        {
            ctx.Record("farBuild", FarSim.FarVersion());
            Assert.True(Aa.Present, "AtmosphereAutopilot is installed");
            string[] crafts = (ctx.Param("crafts") ?? "Gull").Split('+');
            string[] pt = (ctx.Param("point") ?? "3000:0.5").Split(':');
            double altM = double.Parse(pt[0], Inv), mach = double.Parse(pt[1], Inv);
            double rate = ctx.ParamDouble("turnRateDegS", 3), sampleSec = ctx.ParamDouble("sampleSec", 0.5);
            double steadySec = ctx.ParamDouble("steadySec", 8), timeout = ctx.ParamDouble("settleTimeoutSec", 120);
            double ratioTol = ctx.ParamDouble("ratioTol", 0.03), arTol = ctx.ParamDouble("arTol", 0.005), offsetWarn = ctx.ParamDouble("offsetWarn", 0.05);
            string ffmpeg = ctx.Param("ffmpeg");
            double hyper = ctx.ParamDouble("hyperWarp", 4);
            ctx.Record("hyperWarp", hyper);
            var csv = new StringBuilder("craft,ut,headingDeg,bankDeg,aoaDeg,sideslipDeg,altM,srfSpeed,mach,qKPa,liftKN,predLiftKN,dragKN,predDragKN," +
                                        "pitchKNm,predPitchKNm,meanAR\n");
            var summary = new StringBuilder("craft,pass,samples,turnedDeg,bankMeanDeg,bankMaxDeg,aoaMeanDeg,liftOffset,liftBand,dragOffset,dragBand," +
                                            "pitchErrKNm,arMean,arBand,arMaxMin\n");
            bool[] trajectories = FarFlightProbe.QuietOtherPredictors();
            Recorder rec = null;
            try
            {
                foreach (string craftRaw in crafts)
                {
                    string craft = craftRaw.Trim();
                    string path = ctx.CraftPath(craft, "SPH");
                    if (!File.Exists(path))
                        path = Path.GetFullPath(Path.Combine(KSPUtil.ApplicationRootPath, "Ships", "SPH", craft + ".craft"));
                    string missing = MissingParts(path);
                    if (missing.Length > 0)
                    {
                        ctx.Record(craft + ".skipped", "missing parts: " + missing);
                        continue;
                    }
                    ctx.Step("Air start " + craft);
                    yield return Nav.EnsureState(ctx, StartState.Flight, path, "Runway");
                    yield return Wait.Seconds(5);   // FAR voxelizes the craft after load
                    Vessel v = FlightGlobals.ActiveVessel;
                    if (ffmpeg != null && File.Exists(ffmpeg))
                        rec = Recorder.Start(ffmpeg, Path.Combine(ctx.ResultsDir, $"circle_{craft.Replace(' ', '_')}.mp4"));
                    Flight.IgnoreMaxTemperature(true);
                    foreach (Part p in v.parts)
                        foreach (PartModule m in p.Modules)
                            if (m is ModuleReactionWheel rw)
                            {
                                rw.wheelState = ModuleReactionWheel.WheelState.Disabled;
                                rw.authorityLimiter = 0;
                            }
                    Ctrl.Sas(false);
                    Ctrl.Stage();
                    CelestialBody body = v.mainBody;
                    double p0 = body.GetPressure(altM);
                    double speed = mach * body.GetSpeedOfSound(p0, body.GetDensity(p0, body.GetTemperature(altM)));
                    var start = new FarFlightProbe.Condition { Name = "air start", AltM = altM, Mach = mach, AoADeg = 3, Lat = 0, Lon = DaylightLon(body, altM) };
                    yield return FarFlightProbe.Place(start);
                    Flight.PointAtAoA(start.AoADeg);
                    Ctrl.RetractGear();
                    Ctrl.SetThrottle(1f);
                    Aa.Cruise(FlightGlobals.ActiveVessel, altM, speed, 90);
                    int parts = FlightGlobals.ActiveVessel.parts.Count;
                    double ut0 = Planetarium.GetUniversalTime(), wall0 = Time.realtimeSinceStartup;
                    if (hyper > 1)
                        Assert.True(Flight.HyperWarp(hyper, 1.0), "TimeControl hyper-warp on");

                    ctx.Step(craft + ": settle on course 90");
                    double since = double.NaN, deadline = Planetarium.GetUniversalTime() + timeout;
                    while (Planetarium.GetUniversalTime() < deadline)
                    {
                        Vessel w = FlightGlobals.ActiveVessel;
                        if (w == null || w.parts.Count < parts) break;
                        bool now = Math.Abs(w.altitude - altM) < 60 && Math.Abs(w.srfSpeed - speed) < 4 && Math.Abs(w.verticalSpeed) < 3;
                        double ut = Planetarium.GetUniversalTime();
                        if (!now) since = double.NaN;
                        else if (double.IsNaN(since)) since = ut;
                        else if (ut - since >= steadySec) break;
                        yield return null;
                    }

                    ctx.Step($"{craft}: circle at {rate:0.#} deg/s, prediction against flight");
                    double area = WingArea(FlightGlobals.ActiveVessel);
                    object course = Aa.Read(FlightGlobals.ActiveVessel, "CruiseController", "desired_course");
                    var valueProp = course?.GetType().GetProperty("Value", System.Reflection.BindingFlags.Instance |
                                                                            System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic);
                    if (valueProp == null)
                        throw new UITestFailure("AtmosphereAutopilot: no desired_course to turn");
                    double t0 = Planetarium.GetUniversalTime(), circleEnd = t0 + 360 / rate + 5, nextSample = t0 + sampleSec;
                    int skip = 0;
                    bool broken = false;
                    var liftErr = new List<double>();
                    var dragErr = new List<double>();
                    var pitchErr = new List<double>();
                    var ars = new List<double>();
                    var banks = new List<double>();
                    var aoas = new List<double>();
                    double lastHdg = double.NaN, turned = 0;
                    var fixedWait = new WaitForFixedUpdate();
                    while (Planetarium.GetUniversalTime() < circleEnd)
                    {
                        yield return fixedWait;
                        v = FlightGlobals.ActiveVessel;
                        if (v == null || v.parts.Count < parts)
                        {
                            broken = true;
                            break;
                        }
                        double ut = Planetarium.GetUniversalTime();
                        valueProp.SetValue(course, (float)((90 + rate * (ut - t0)) % 360), null);
                        // heading of the velocity over the ground, and how far it has come round
                        Vector3d up = (v.CoM - body.position).normalized;
                        Vector3d north = Vector3d.Exclude(up, body.transform.up).normalized;
                        Vector3d east = Vector3d.Cross(up, north);
                        Vector3d hvel = Vector3d.Exclude(up, v.srf_velocity);
                        double hdg = (Math.Atan2(Vector3d.Dot(hvel, east), Vector3d.Dot(hvel, north)) * Mathf.Rad2Deg + 360) % 360;
                        if (!double.IsNaN(lastHdg))
                            turned += ((hdg - lastHdg + 540) % 360) - 180;
                        lastHdg = hdg;
                        if (skip > 0)
                        {
                            skip--;
                            continue;
                        }
                        if (ut < nextSample || !FarApi.FlightForceAndTorque(v, out Vector3 fFlight, out Vector3 tFlight))
                            continue;
                        nextSample = ut + sampleSec;

                        Vector3 vel = v.srf_velocity, axis = FarApi.PitchUpAxis(v);
                        FarApi.ClearMemo(v);
                        if (!FarApi.PredictAtFull(v, vel, v.latitude, v.longitude, v.altitude, ut, out Vector3 fPred, out Vector3 tPred))
                            continue;
                        FarApi.ClearMemo(v);
                        skip = 2;
                        LiftDrag(v, fFlight, vel, out double L, out double D);
                        LiftDrag(v, fPred, vel, out double Lp, out double Dp);
                        double pm = Vector3.Dot(tFlight, axis), pmp = Vector3.Dot(tPred, axis);
                        double bank = Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.right, (Vector3)up), -1, 1)) * Mathf.Rad2Deg;
                        double beta = Math.Asin(Mathf.Clamp(Vector3.Dot(vel.normalized, v.ReferenceTransform.right), -1, 1)) * Mathf.Rad2Deg;
                        double ar = MeanAr(v);
                        liftErr.Add(Lp / L - 1);
                        dragErr.Add(Dp / D - 1);
                        pitchErr.Add(pmp - pm);
                        ars.Add(ar);
                        banks.Add(Math.Abs(bank));
                        aoas.Add(Flight.AoA);
                        csv.Append(string.Join(",", Csv(craft), F(ut, "F3"), F(hdg, "F2"), F(bank, "F2"), F(Flight.AoA, "F3"), F(beta, "F3"), F(v.altitude, "F1"),
                            F(v.srfSpeed, "F2"), F(v.mach, "F4"), F(v.dynamicPressurekPa, "F4"), F(L, "G6"), F(Lp, "G6"), F(D, "G6"), F(Dp, "G6"),
                            F(pm, "G6"), F(pmp, "G6"), F(ar, "F4"))).Append('\n');
                    }

                    if (hyper > 1)
                    {
                        Flight.HyperWarpOff();
                        ctx.Record(craft + ".speedup", Math.Round((Planetarium.GetUniversalTime() - ut0) / (Time.realtimeSinceStartup - wall0), 2));
                    }
                    v = FlightGlobals.ActiveVessel;
                    if (v != null) Aa.Off(v);
                    double Mean(List<double> l, Func<double, double> f) => l.Count > 0 ? l.Select(f).Average() : double.NaN;
                    double Max(List<double> l, Func<double, double> f) => l.Count > 0 ? l.Select(f).Max() : double.NaN;
                    // the middle 90 % of the samples: transients (the roll into the turn, an autopilot wobble) left out
                    double Pct(List<double> l, double q)
                    {
                        if (l.Count == 0) return double.NaN;
                        var sorted = l.OrderBy(x => x).ToList();
                        return sorted[(int)Math.Round(q * (sorted.Count - 1))];
                    }
                    double Band(List<double> l) => Pct(l, 0.95) - Pct(l, 0.05);
                    double lOff = Pct(liftErr, 0.5), dOff = Pct(dragErr, 0.5), lBand = Band(liftErr), dBand = Band(dragErr);
                    double arMean = Mean(ars, x => x), arBand = Band(ars) / arMean, arMaxMin = ars.Count > 0 ? (ars.Max() - ars.Min()) / arMean : double.NaN;
                    bool pass = !broken && liftErr.Count > 10 && Math.Abs(turned) >= 330 && lBand <= ratioTol && dBand <= ratioTol && arBand <= arTol;
                    string k0 = craft + ".";
                    if (broken)
                        ctx.Record(k0 + "broken", v == null ? "no vessel" : $"{parts - v.parts.Count} parts lost");
                    ctx.Record(k0 + "pass", pass);
                    ctx.Record(k0 + "turnedDeg", Math.Round(turned, 0));
                    ctx.Record(k0 + "liftPredicted", string.Format(Inv, "{0:+0.0%;-0.0%} of flight, band {1:P1}", lOff, lBand));
                    ctx.Record(k0 + "dragPredicted", string.Format(Inv, "{0:+0.0%;-0.0%} of flight, band {1:P1}", dOff, dBand));
                    ctx.Record(k0 + "arBand", string.Format(Inv, "{0:P2} of {1:F2} (max-min {2:P2})", arBand, arMean, arMaxMin));
                    if (Math.Abs(lOff) > offsetWarn || Math.Abs(dOff) > offsetWarn)
                        ctx.Record(k0 + "predictionOffset", "over " + offsetWarn.ToString("P0", Inv) + ": FAR's prediction API differs from its flight forces by a constant");
                    summary.Append(string.Join(",", Csv(craft), pass ? "1" : "0", liftErr.Count.ToString(Inv), F(turned, "F0"), F(Mean(banks, x => x), "F1"),
                        F(Max(banks, x => x), "F1"), F(Mean(aoas, x => x), "F2"), F(lOff, "F4"), F(lBand, "F4"), F(dOff, "F4"),
                        F(dBand, "F4"), F(Mean(pitchErr, Math.Abs), "F3"), F(arMean, "F3"), F(arBand, "F5"), F(arMaxMin, "F5"))).Append('\n');
                    if (broken)
                        yield return Wait.Seconds(3);
                    rec?.Stop(!pass, ctx, craft);
                    rec = null;
                    File.WriteAllText(Path.Combine(ctx.ResultsDir, "ar-circle-summary.csv"), summary.ToString());
                }
            }
            finally
            {
                if (hyper > 1)
                    Flight.HyperWarpOff();
                rec?.Stop(true, ctx, "interrupted");
                FarFlightProbe.ResumeOtherPredictors(trajectories);
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "ar-circle.csv"), csv.ToString());
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "ar-circle-summary.csv"), summary.ToString());
            }
        }

        /// <summary>
        /// Whether FAR gives each wing part the same effective aspect ratio in flight as in the editor: per craft, every
        /// FAR wing part's effective_AR in the SPH (after an editor sim at Mach 0.5, AoA 3), then the same craft air-started
        /// at 3 km and Mach 0.5 and held at AoA 3 (wings level, FAR's shielding settled after the placement), read with the
        /// controls neutral and again after 50 frames of full nose-up input (the control surfaces deflected). Parts pair up
        /// by persistentId (else their order). Per part, whether its aspect ratio came from its wing group (1, with a FAR
        /// build that has the ArFromGroup diagnostic) and whether it is shielded (s), in each scene. Params: crafts
        /// ("+"-separated), facility (SPH), tol (0.01, relative). Writes ar-editor-flight.csv; records per craft the parts
        /// compared and how many differ by more than tol, neutral and deflected.
        /// </summary>
        [UITest("Far_ArEditorFlight", Explicit = true, Kind = "Check", StartsAt = StartState.SpaceCenter, TimeoutSec = 7200)]
        public static IEnumerator Far_ArEditorFlight(TestContext ctx)
        {
            ctx.Record("farBuild", FarSim.FarVersion());
            string facility = (ctx.Param("facility") ?? "SPH").ToUpperInvariant();
            string[] crafts = (ctx.Param("crafts") ?? "Mallard").Split('+');
            double tol = ctx.ParamDouble("tol", 0.01);
            var csv = new StringBuilder("craft,index,persistentId,part,kind,areaM2,editorAR,flightAR,deflectedAR,deflectionDeg,editorGroup,flightGroup\n");
            bool[] trajectories = FarFlightProbe.QuietOtherPredictors();
            try
            {
                foreach (string craftRaw in crafts)
                {
                    string craft = craftRaw.Trim();
                    string path = ctx.CraftPath(craft, facility);
                    if (!File.Exists(path))
                        path = Path.GetFullPath(Path.Combine(KSPUtil.ApplicationRootPath, "Ships", facility, craft + ".craft"));
                    string missing = File.Exists(path) ? MissingParts(path) : "no craft file";
                    if (missing.Length > 0)
                    {
                        ctx.Record(craft + ".skipped", missing);
                        continue;
                    }

                    ctx.Step("Editor " + craft);
                    yield return Nav.EnsureState(ctx, facility == "VAB" ? StartState.Vab : StartState.Sph, path);
                    Ugui.DismissPopups();
                    yield return Wait.Until(FarSim.Ready, 60, "FAR editor sim ready", shot: false);
                    yield return Wait.Seconds(8);   // voxelization settles after the craft loads
                    FarSim.Instant(3, 0, 0, 0.5, 0, 0, false);
                    var editor = new List<(uint pid, string title, string kind, double area, double ar, string grp)>();
                    foreach (Part p in EditorLogic.fetch.ship.parts)
                        foreach (PartModule m in p.Modules)
                            if (IsFarWing(m))
                                editor.Add((p.persistentId, p.partInfo.title, m.GetType().Name == "FARControllableSurface" ? "control" : "wing",
                                            Num(m, "S"), Num(m, "effective_AR"), FromGroup(m) + (IsShielded(m) ? "s" : "")));

                    ctx.Step("Flight " + craft);
                    yield return Nav.EnsureState(ctx, StartState.Flight, path, "Runway");
                    yield return Wait.Seconds(5);   // FAR voxelizes the craft after load
                    Vessel v = FlightGlobals.ActiveVessel;
                    Flight.IgnoreMaxTemperature(true);
                    Ctrl.Sas(false);
                    // held at the editor sim's flow (AoA 3, wings level, not rotating), FAR's shielding settled after the
                    // placement, as Far_WingSlots does
                    var start = new FarFlightProbe.Condition { Name = "air start", AltM = 3000, Mach = 0.5, AoADeg = 3, Lat = 0, Lon = DaylightLon(v.mainBody, 3000) };
                    var none = new List<FarFlightProbe.Sample>();
                    Ctrl.Attach(true);
                    Ctrl.Set(pitch: 0, yaw: 0, roll: 0);
                    yield return FarFlightProbe.Place(start);
                    yield return FarFlightProbe.Hold(start, 400, 0, none, predict: false);
                    var settled = new bool[1];
                    yield return FarApi.ResettleShielding(FlightGlobals.ActiveVessel, settled);
                    ctx.Record(craft + ".shieldingSettled", settled[0]);
                    yield return FarFlightProbe.Place(start);
                    yield return FarFlightProbe.Hold(start, 75, 0, none, predict: false);
                    // by persistentId (a launch keeps it); by the parts' order where it does not match
                    Dictionary<uint, (double ar, double defl, string grp)> Read()
                    {
                        var r = new Dictionary<uint, (double ar, double defl, string grp)>();
                        foreach (Part p in FlightGlobals.ActiveVessel.parts)
                            foreach (PartModule m in p.Modules)
                                if (IsFarWing(m))
                                    r[p.persistentId] = (Num(m, "effective_AR"),
                                                         m.GetType().Name == "FARControllableSurface" ? Num(m, "AoAoffset") : double.NaN,
                                                         FromGroup(m) + (IsShielded(m) ? "s" : ""));
                        return r;
                    }
                    var neutralBy = Read();
                    Ctrl.Set(pitch: 1);
                    yield return FarFlightProbe.Hold(start, 50, 0, none, predict: false);
                    var deflectedBy = Read();
                    Ctrl.Set(pitch: 0);
                    Ctrl.Detach();
                    bool byPid = editor.All(e => neutralBy.ContainsKey(e.pid));
                    ctx.Record(craft + ".pairedBy", byPid ? "persistentId" : "order");
                    var order = new List<uint>();
                    foreach (Part p in FlightGlobals.ActiveVessel.parts)
                        foreach (PartModule m in p.Modules)
                            if (IsFarWing(m)) order.Add(p.persistentId);
                    var neutral = new List<double>();
                    var deflected = new List<double>();
                    var deflDeg = new List<double>();
                    var flightGrp = new List<string>();
                    for (int i = 0; i < editor.Count && (byPid || i < order.Count); i++)
                    {
                        uint id = byPid ? editor[i].pid : order[i];
                        flightGrp.Add(neutralBy.TryGetValue(id, out var g) ? g.grp : "");
                        neutral.Add(neutralBy.TryGetValue(id, out var a) ? a.ar : double.NaN);
                        deflected.Add(deflectedBy.TryGetValue(id, out var b) ? b.ar : double.NaN);
                        deflDeg.Add(deflectedBy.TryGetValue(id, out var c) ? c.defl : double.NaN);
                    }

                    int n = neutral.Count, badN = 0, badD = 0;
                    if (editor.Count != neutralBy.Count)
                        ctx.Record(craft + ".partCountMismatch", $"editor {editor.Count}, flight {neutralBy.Count}");
                    bool Differs(double a, double b) => double.IsNaN(a) != double.IsNaN(b) || Math.Abs(a - b) > tol * Math.Max(Math.Abs(a), 1e-6);
                    for (int i = 0; i < n; i++)
                    {
                        var e = editor[i];
                        if (Differs(e.ar, neutral[i])) badN++;
                        if (Differs(e.ar, deflected[i])) badD++;
                        csv.Append(string.Join(",", Csv(craft), i.ToString(Inv), e.pid.ToString(Inv), Csv(e.title), e.kind, F(e.area, "F3"), F(e.ar, "F4"),
                            F(neutral[i], "F4"), F(deflected[i], "F4"), F(deflDeg[i], "F2"), e.grp, flightGrp[i])).Append('\n');
                    }
                    ctx.Record(craft + ".parts", n);
                    ctx.Record(craft + ".differNeutral", badN);
                    ctx.Record(craft + ".differDeflected", badD);
                    File.WriteAllText(Path.Combine(ctx.ResultsDir, "ar-editor-flight.csv"), csv.ToString());
                }
            }
            finally
            {
                FarFlightProbe.ResumeOtherPredictors(trajectories);
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "ar-editor-flight.csv"), csv.ToString());
            }
        }

        // whether a wing part's aspect ratio came from its wing group (1) or its own (0): a diagnostic field some FAR
        // builds have; "" without it
        private static string FromGroup(PartModule m)
        {
            try
            {
                object wi = Reflect.Get(m, "WingInteraction");
                return wi != null && Reflect.Get(wi, "ArFromGroup") is bool b ? (b ? "1" : "0") : "";
            }
            catch (Exception) { return ""; }
        }

        private static string F(double x, string fmt) => x.ToString(fmt, Inv);

        private static string Csv(string s) => s.Contains(',') ? "\"" + s + "\"" : s;

        private static bool IsFarWing(PartModule m)
        {
            for (Type t = m.GetType(); t != null && t != typeof(PartModule); t = t.BaseType)
                if (t.Name == "FARWingAerodynamicModel") return true;
            return false;
        }

        private static bool IsShielded(PartModule m) =>
            m.GetType().GetField("isShielded", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public)?.GetValue(m) is bool b && b;

        private static double Num(PartModule m, string field)
        {
            try { return Convert.ToDouble(Reflect.Get(m, field), Inv); }
            catch (Exception) { }
            // the reentry-tab fork keeps a wing's per-call state (effective_AR, effective_MAC, ...) in its _scratch
            // (WingScratch, one per thread slot; the main thread's holds the flight's numbers)
            try { return Convert.ToDouble(Reflect.Get(Reflect.Get(m, "_scratch"), field), Inv); }
            catch (Exception) { return double.NaN; }
        }

        // FAR's wing parts' area (m^2), control surfaces in: the reference area the CL and lift slope are on
        private static double WingArea(Vessel v)
        {
            double s = 0;
            foreach (Part p in v.parts)
                foreach (PartModule m in p.Modules)
                    if (IsFarWing(m))
                        s += Num(m, "S");
            return s;
        }

        // the unshielded wing parts' effective aspect ratio, area-weighted
        private static double MeanAr(Vessel v)
        {
            double num = 0, den = 0;
            foreach (Part p in v.parts)
                foreach (PartModule m in p.Modules)
                    if (IsFarWing(m) && !IsShielded(m))
                    {
                        double s = Num(m, "S"), ar = Num(m, "effective_AR");
                        if (double.IsNaN(s) || double.IsNaN(ar)) continue;
                        num += s * ar;
                        den += s;
                    }
            return den > 0 ? num / den : double.NaN;
        }

        // every part's FAR force: the wing model's (kind wing or control) and the voxel model's (kind body, on a wing's
        // part too) apart; tintCl is what FAR's Cl tint shows, the force across the airflow over q and the area (the
        // wing's S, a body's projected area)
        private static void PartForces(Vessel v, string craft, string phase, StringBuilder csv)
        {
            Vector3 vel = v.srf_velocity;
            double q = v.dynamicPressurekPa;
            foreach (Part p in v.parts)
                foreach (PartModule m in p.Modules)
                {
                    string kind;
                    Vector3 f;
                    double area;
                    if (IsFarWing(m))
                    {
                        kind = m.GetType().Name == "FARControllableSurface" ? "control" : "wing";
                        f = IsShielded(m) ? Vector3.zero : (Vector3)Reflect.Get(m, "worldSpaceForce");
                        area = Num(m, "S");
                    }
                    else if (m.GetType().Name == "FARAeroPartModule")
                    {
                        kind = "body";
                        f = (Vector3)Reflect.Get(m, "worldSpaceAeroForce");
                        object pa = Reflect.Get(m, "ProjectedAreas");
                        area = pa != null ? Convert.ToDouble(Reflect.Get(pa, "totalArea"), Inv) : double.NaN;
                    }
                    else
                        continue;
                    LiftDrag(v, f, vel, out double lift, out double drag);
                    double cross = Vector3.ProjectOnPlane(f, vel.normalized).magnitude;
                    bool wing = kind != "body";
                    csv.Append(string.Join(",", Csv(craft), phase, Csv(p.partInfo.title), p.persistentId.ToString(Inv), kind, F(area, "F4"),
                        wing ? F(Num(m, "effective_AR"), "F4") : "", wing ? F(Num(m, "Cl"), "F5") : "", wing ? F(Num(m, "Cd"), "F5") : "",
                        wing ? F(Num(m, "liftslope"), "F4") : "", wing ? F(Num(m, "stall"), "F4") : "",
                        F(lift, "F4"), F(drag, "F4"), F(cross, "F4"), F(cross / (q * area), "F4"), wing && IsShielded(m) ? "1" : "0")).Append('\n');
                }
        }

        // FAR's force in the last physics frame (kN, world): the wing models' (shielded ones apply nothing) and the voxel
        // parts' apart
        private static bool Forces(Vessel v, out Vector3 wing, out Vector3 body)
        {
            wing = body = Vector3.zero;
            if (v == null) return false;
            int n = 0;
            foreach (Part p in v.parts)
                foreach (PartModule m in p.Modules)
                {
                    try
                    {
                        if (IsFarWing(m))
                        {
                            if (IsShielded(m)) continue;
                            wing += (Vector3)Reflect.Get(m, "worldSpaceForce");
                            n++;
                        }
                        else if (m.GetType().Name == "FARAeroPartModule")
                        {
                            body += (Vector3)Reflect.Get(m, "worldSpaceAeroForce");
                            n++;
                        }
                    }
                    catch (Exception) { }
                }
            return n > 0;
        }

        // lift across the airflow toward the vessel's roof and drag against it
        private static void LiftDrag(Vessel v, Vector3 f, Vector3 vel, out double lift, out double drag)
        {
            Vector3 velN = vel.normalized;
            Vector3 roof = -v.ReferenceTransform.forward;
            drag = -Vector3.Dot(f, velN);
            lift = Vector3.Dot(f, Vector3.ProjectOnPlane(roof, velN).normalized);
        }

        // FAR's aero visualization for the vessel (its FlightGUI vessel module's AeroVizGUI), null without it
        private static object AeroViz(Vessel v)
        {
            Type t = Reflect.FindType("FerramAerospaceResearch.FARGUI.FARFlightGUI.FlightGUI");
            if (t == null || v == null) return null;
            Component gui = v.GetComponent(t);
            return gui != null ? Reflect.Get(gui, "AeroVizGUI") : null;
        }

        // Each tint in turn (none, or FAR's Cl, Cd or stall tint alone; FAR's lift and drag arrows with arrows), the UI
        // and the harness's overlay hidden, the vessel shot from above, the side and
        // behind at a distance sized to it: ar_<name>_<tint>_<view>.png in the results.
        private static IEnumerator VizShots(TestContext ctx, Vessel v, string name, string[] tints, double clSat, bool arrows)
        {
            FlightCamera cam = FlightCamera.fetch;
            object viz = AeroViz(v);
            if (cam == null || v == null) yield break;
            if (viz == null) ctx.Record(name + ".aeroViz", "FAR's AeroVizGUI not found: shots untinted");
            float size = 0;
            foreach (Part p in v.parts)
                size = Math.Max(size, 2 * (p.transform.position - v.CoM).magnitude);
            GameEvents.onHideUI.Fire();
            TestRunner.HideOverlay = true;
            var gui = ShotGui.HideOthers(imgui: false);     // Trajectories' window; the autopilot keeps running
            bool arrowsWas = PhysicsGlobals.AeroForceDisplay;
            PhysicsGlobals.AeroForceDisplay = arrows;
            try
            {
                if (viz != null) Reflect.Set(viz, "FullySaturatedCl", clSat);
                foreach (string tint in tints)
                {
                    if (viz != null)
                    {
                        Reflect.Set(viz, "TintForCl", tint == "cl");
                        Reflect.Set(viz, "TintForCd", tint == "cd");
                        Reflect.Set(viz, "TintForStall", tint == "stall");
                    }
                    // the camera's heading is from north, and every flight here is on course 90 (east): 3.14 looks from the
                    // south at its right side, 1.57 from behind
                    foreach (var (view, pitch, hdg, dist) in new[] { ("above", 1.35f, 0f, 1.6f), ("side", 0.12f, 3.1416f, 1.5f), ("rear", 0.5f, 1.5708f, 1.5f) })
                    {
                        cam.camPitch = pitch;
                        cam.camHdg = hdg;
                        cam.SetDistance(Math.Max(20f, dist * size));
                        for (int i = 0; i < 10; ++i) yield return new WaitForEndOfFrame();
                        ScreenCapture.CaptureScreenshot(Path.Combine(ctx.ResultsDir, $"ar_{name}_{tint}_{view}.png"));
                        yield return new WaitForEndOfFrame();
                        yield return new WaitForEndOfFrame();
                    }
                }
            }
            finally
            {
                if (viz != null)
                {
                    Reflect.Set(viz, "TintForCl", false);
                    Reflect.Set(viz, "TintForCd", false);
                    Reflect.Set(viz, "TintForStall", false);
                }
                PhysicsGlobals.AeroForceDisplay = arrowsWas;
                TestRunner.HideOverlay = false;
                gui.Dispose();
                GameEvents.onShowUI.Fire();
            }
        }

        /// <summary>
        ///     ffmpeg recording the KSP window (gdigrab) to an mp4 until stopped; Stop keeps the file or deletes it
        /// </summary>
        private sealed class Recorder
        {
            private System.Diagnostics.Process proc;
            private string path;
            private ShotGui gui;     // popup windows (Trajectories') hidden while recording

            [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Sequential)]
            private struct Rect { public int Left, Top, Right, Bottom; }

            [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Sequential)]
            private struct Point { public int X, Y; }

            [System.Runtime.InteropServices.DllImport("user32.dll")]
            private static extern bool GetClientRect(IntPtr hWnd, out Rect rect);

            [System.Runtime.InteropServices.DllImport("user32.dll")]
            private static extern bool ClientToScreen(IntPtr hWnd, ref Point point);

            [System.Runtime.InteropServices.DllImport("user32.dll")]
            private static extern IntPtr GetActiveWindow();

            [System.Runtime.InteropServices.DllImport("user32.dll", CharSet = System.Runtime.InteropServices.CharSet.Unicode)]
            private static extern IntPtr FindWindow(string className, string windowName);

            // KSP's client area on the desktop: "-offset_x X -offset_y Y -video_size WxH " (even), "" if not found.
            // gdigrab's window capture (title=) gets a stale frame of a Direct3D window; the desktop's pixels are current.
            private static string ClientArea()
            {
                try
                {
                    // Unity's window: the main thread's active window, else the player's window class
                    IntPtr hwnd = GetActiveWindow();
                    if (hwnd == IntPtr.Zero)
                        hwnd = FindWindow("UnityWndClass", null);
                    if (hwnd == IntPtr.Zero)
                        hwnd = System.Diagnostics.Process.GetCurrentProcess().MainWindowHandle;
                    if (hwnd == IntPtr.Zero || !GetClientRect(hwnd, out Rect r))
                        return "";
                    var p = new Point();
                    if (!ClientToScreen(hwnd, ref p))
                        return "";
                    int w = (r.Right - r.Left) & ~1, h = (r.Bottom - r.Top) & ~1;
                    return w > 0 && h > 0 ? $"-offset_x {p.X} -offset_y {p.Y} -video_size {w}x{h} " : "";
                }
                catch (Exception)
                {
                    return "";
                }
            }

            public static Recorder Start(string ffmpeg, string path)
            {
                string area = ClientArea();
                Debug.Log("[KspTest] recording " + (area.Length > 0 ? area : "the whole desktop ") + "to " + path);
                var psi = new System.Diagnostics.ProcessStartInfo(ffmpeg,
                    "-y -loglevel error -f gdigrab -framerate 30 -draw_mouse 0 " + area + "-i desktop " +
                    "-vf crop=trunc(iw/2)*2:trunc(ih/2)*2 -c:v libx264 -preset ultrafast -crf 26 -pix_fmt yuv420p \"" + path + "\"")
                {
                    UseShellExecute = false, RedirectStandardInput = true, CreateNoWindow = true
                };
                try
                {
                    return new Recorder { proc = System.Diagnostics.Process.Start(psi), path = path, gui = ShotGui.HideOthers(imgui: false) };
                }
                catch (Exception e)
                {
                    Debug.LogWarning("[KspTest] ffmpeg did not start: " + e.Message);
                    return null;
                }
            }

            public void Stop(bool keep, TestContext ctx, string craft)
            {
                gui?.Dispose();
                try
                {
                    if (!proc.HasExited)
                    {
                        proc.StandardInput.Write("q");
                        proc.StandardInput.Flush();
                        if (!proc.WaitForExit(15000))
                            proc.Kill();
                    }
                }
                catch (Exception e)
                {
                    Debug.LogWarning("[KspTest] ffmpeg stop: " + e.Message);
                }
                bool have = File.Exists(path);
                if (have && !keep)
                    File.Delete(path);
                ctx.Record(craft + ".video", !have ? "not written" : keep ? Path.GetFileName(path) : "normal, deleted");
            }
        }

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

        // A longitude on the equator in the afternoon (the sun 30 deg west of overhead), the first one east of that whose
        // terrain is at least 1500 m below altM.
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
    }
}
