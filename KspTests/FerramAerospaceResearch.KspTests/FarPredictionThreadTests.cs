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
    /// <summary>FAR's prediction API against FAR in flight, the flight taken as the gold standard (see FarFlightProbe).</summary>
    public static class FarPredictionThreadTests
    {
        private static readonly FarFlightProbe.Condition[] NoiseFloorConditions =
        {
            new FarFlightProbe.Condition { Name = "subsonic", Mach = 0.5, AltM = 3000, AoADeg = 5 },
            new FarFlightProbe.Condition { Name = "transonic", Mach = 0.95, AltM = 8000, AoADeg = 4 },
            new FarFlightProbe.Condition { Name = "supersonic", Mach = 2.0, AltM = 14000, AoADeg = 5 },
            new FarFlightProbe.Condition { Name = "highAoA", Mach = 1.5, AltM = 12000, AoADeg = 25 },
            new FarFlightProbe.Condition { Name = "mach4", Mach = 4.0, AltM = 22000, AoADeg = 8 },
            new FarFlightProbe.Condition { Name = "hypersonic", Mach = 7.0, AltM = 38000, AoADeg = 20 },
        };

        /// <summary>
        /// The Stearwing A300 is built for stock aero, every surface on 100 % pitch, yaw and roll; under FAR its wing
        /// elevons trim it by dumping lift. Set it up as a FAR pilot would: the outermost surface on each wing
        /// ailerons only (roll 100), the Big-S tail fin rudder only (yaw 100), the FAT-455 tail fins elevators only
        /// (pitch 100, max deflection 40), every other surface (flaps and spoilers to be) off.
        /// </summary>
        private static void StearwingSurfaces(TestContext ctx, Vessel v)
        {
            Vector3 right = v.ReferenceTransform.right, com = v.CoM;
            var surfaces = v.parts.Select(p => new { Part = p, Module = FarApi.ControlSurface(p) }).Where(s => s.Module != null).ToList();
            double outer = surfaces.Max(s => Math.Abs(Vector3.Dot(s.Part.transform.position - com, right)));
            var log = new List<string>();
            foreach (var s in surfaces)
            {
                double span = Vector3.Dot(s.Part.transform.position - com, right);
                string role;
                if (s.Part.name == "wingShuttleRudder") { FarApi.SetControlAxes(s.Module, 0, 100, 0); role = "rudder"; }
                else if (s.Part.name == "airlinerTailFin") { FarApi.SetControlAxes(s.Module, 100, 0, 0, 40); role = "elevator"; }
                else if (Math.Abs(span) > outer - 0.5) { FarApi.SetControlAxes(s.Module, 0, 0, 100); role = "aileron"; }
                else { FarApi.SetControlAxes(s.Module, 0, 0, 0); role = "off"; }
                log.Add(string.Format(CultureInfo.InvariantCulture, "{0} {1:+0.0;-0.0} m {2}", s.Part.name, span, role));
            }
            ctx.Record("surfaces", string.Join(", ", log));
        }

        // Alive and in one piece: a crash leaves the wreck floating, which Flight.AssertAlive accepts.
        private static void AssertWhole(int parts)
        {
            Flight.AssertAlive();
            int now = FlightGlobals.ActiveVessel.parts.Count;
            if (now < parts)
            {
                FlightTrace.End();
                throw new UITestFailure($"vessel broke up ({now} of {parts} parts) at {Flight.Altitude:F0} m, {Flight.SurfaceSpeed:F0} m/s");
            }
        }

        /// <summary>
        /// FAR's prediction for the vessel as it sits (surfaces as they are, gear as it is) flying level at 500 m over
        /// its position, over AoA -4..16 at 100, 150 and 200 m/s: lift, drag and pitch moment about the CoM (nose-up
        /// positive), with the weight. Where the moment crosses zero is the trim AoA with the surfaces neutral; its
        /// slope says whether the airframe is statically stable (negative) or not. Writes trim_sweep.csv and records
        /// the trim and the AoA at which lift meets the weight for each speed.
        /// </summary>
        private static void TrimSweep(TestContext ctx, Vessel v)
        {
            var ic = CultureInfo.InvariantCulture;
            Vector3 nose = v.ReferenceTransform.up, roof = -v.ReferenceTransform.forward;
            Vector3 pitchAxis = FarApi.PitchUpAxis(v);
            double weight = v.totalMass * FlightGlobals.getGeeForceAtPosition(v.CoMD).magnitude;
            double ut = Planetarium.GetUniversalTime();
            var csv = new StringBuilder("speed,aoaDeg,liftKN,dragKN,pitchKNm,weightKN\n");
            foreach (double speed in new[] { 100.0, 150.0, 200.0 })
            {
                double prevA = double.NaN, prevM = double.NaN, prevL = double.NaN, trim = double.NaN, level = double.NaN, slope = double.NaN;
                for (double a = -4; a <= 16.001; a += 1)
                {
                    double r = a * Math.PI / 180;
                    Vector3 vel = (float)speed * ((float)Math.Cos(r) * nose - (float)Math.Sin(r) * roof);
                    if (!FarApi.PredictAtFull(v, vel, v.latitude, v.longitude, 500, ut, out Vector3 f, out Vector3 t)) return;
                    Vector3 velN = vel.normalized;
                    double drag = -Vector3.Dot(f, velN), lift = Vector3.Dot(f, Vector3.ProjectOnPlane(roof, velN).normalized);
                    double m = Vector3.Dot(t, pitchAxis);
                    csv.Append(string.Format(ic, "{0},{1},{2:F2},{3:F2},{4:F1},{5:F2}\n", speed, a, lift, drag, m, weight));
                    if (!double.IsNaN(prevM) && double.IsNaN(trim) && Math.Sign(m) != Math.Sign(prevM))
                    {
                        trim = prevA + (a - prevA) * prevM / (prevM - m);
                        slope = m - prevM;
                    }
                    if (!double.IsNaN(prevL) && double.IsNaN(level) && prevL < weight && lift >= weight)
                        level = prevA + (a - prevA) * (weight - prevL) / (lift - prevL);
                    prevA = a; prevM = m; prevL = lift;
                }
                ctx.Record($"trim.{speed:F0}ms", string.Format(ic, "trim AoA {0:F1} deg (dM/dAoA {1:F0} kN m/deg), lift = weight at AoA {2:F1} deg", trim, slope, level));
            }
            ctx.Record("trim.weightKN", weight.ToString("F1", ic));
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "trim_sweep.csv"), csv.ToString());
        }

        // Where the gold standard is flown: the same airframe condition at different places (latitude and the far side
        // of the planet change stock Kerbin's temperature) and across the Mach range.
        private static readonly FarFlightProbe.Condition[] CrossTargets =
        {
            new FarFlightProbe.Condition { Name = "subsonic", Mach = 0.5, AltM = 3000, AoADeg = 5, Lat = 0, Lon = -60 },
            new FarFlightProbe.Condition { Name = "subsonic-lat60", Mach = 0.5, AltM = 3000, AoADeg = 5, Lat = 60, Lon = -60 },
            new FarFlightProbe.Condition { Name = "subsonic-farside", Mach = 0.5, AltM = 3000, AoADeg = 5, Lat = 0, Lon = 120 },
            new FarFlightProbe.Condition { Name = "supersonic", Mach = 2.0, AltM = 14000, AoADeg = 5, Lat = 0, Lon = -60 },
            new FarFlightProbe.Condition { Name = "hypersonic", Mach = 7.0, AltM = 38000, AoADeg = 20, Lat = 0, Lon = -60 },
        };

        // The targets as a craft flies them: the listed ones, except that the subsonic three go where the craft can
        // hold level flight - the Stearwing A300 cannot below about 190 m/s (its elevators run out), so it flies them at
        // Mach 0.8, as it cruises. The names and places stay, so the tables line up.
        private static FarFlightProbe.Condition[] CrossTargetsFor(string craft) => CrossTargets.Select(c => craft == "Stearwing A300" && c.Name.StartsWith("subsonic")
            ? new FarFlightProbe.Condition { Name = c.Name, Mach = 0.8, AltM = c.AltM, AoADeg = 0, Lat = c.Lat, Lon = c.Lon, HeadingDeg = c.HeadingDeg }
            : c).ToArray();

        // Where the prediction is asked from: somewhere else, flying something else. Vacuum is where FAR's flight
        // path stops updating the wings, so they keep whatever the last prediction left. Slow-high: slow and nose-high,
        // as on an approach - where a craft that cannot fly level at that speed ends up, its wings near or past stall.
        private static readonly FarFlightProbe.Condition[] CrossVantages =
        {
            new FarFlightProbe.Condition { Name = "slow", Mach = 0.45, AltM = 1500, AoADeg = 5, Lat = -20, Lon = -30 },
            new FarFlightProbe.Condition { Name = "slow-high", Mach = 0.3, AltM = 1500, AoADeg = 20, Lat = -20, Lon = -30 },
            new FarFlightProbe.Condition { Name = "hypersonic", Mach = 5.0, AltM = 30000, AoADeg = 12, Lat = 30, Lon = 150 },
            new FarFlightProbe.Condition { Name = "vacuum", SpeedMs = 2200, AltM = 75000, AoADeg = 0, Lat = -30, Lon = 30 },
        };

        // The vantages as a craft flies them: slow is the slowest it holds level low down (the Stearwing A300 at Mach 0.7;
        // the Aeris 4A at the listed 0.45), slow-high stays below that on purpose - the edge of the envelope.
        private static FarFlightProbe.Condition[] CrossVantagesFor(string craft) => CrossVantages.Select(c => craft == "Stearwing A300" && c.Name == "slow"
            ? new FarFlightProbe.Condition { Name = c.Name, Mach = 0.7, AltM = c.AltM, AoADeg = 0, Lat = c.Lat, Lon = c.Lon, HeadingDeg = c.HeadingDeg }
            : c).ToArray();

        // How each craft flies each condition, as FarFlightProbe.Trim found it, keyed by CrossTrimKey (the craft, the
        // condition as listed, its surface setup, and how many of its wings FAR has shielded - the Stearwing's two root
        // control surfaces come out shielded or not from one run to the next, a FAR knife edge, and it flies
        // differently each way): Far_CrossCondition flies these without finding them again. A
        // condition not here (or trims=fresh) is trimmed in the run, which writes trims.tsv and trims.cs (entries for
        // this table) to its results; each run also reports how steady the flight at each target was.
        private static readonly Dictionary<string, FarFlightProbe.TrimResult> CrossTrims = new Dictionary<string, FarFlightProbe.TrimResult>
        {
            // Aeris 4A (all-fixes FAR, 20260926-154552)
            { "Aeris 4A|vacuum|M0|2200m/s|75000m|-30,30|AoA0|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 0, AoADeg = 0, Pitch = 0, Throttle = 0, EngineThrottle = double.NaN, Converged = true, Mode = "no air: inputs centered, throttle closed" } },
            { "Aeris 4A|slow-high|M0.3|0m/s|1500m|-20,-30|AoA20|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 20, AoADeg = 19.956, Pitch = 1, Throttle = 0.2355, EngineThrottle = 0.2355, Converged = true, Mode = "no level flight within AoA -10 to 25 (at about 25.6): held at AoA 20.0, -0.29 m/s2 vertical; pitch input at +1: +41 kN m left to the hold" } },
            { "Aeris 4A|slow|M0.45|0m/s|1500m|-20,-30|AoA5|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 8.648, AoADeg = 8.615, Pitch = 0.6788, Throttle = 0.1327, EngineThrottle = 0.1327, Converged = true, Mode = "level" } },
            { "Aeris 4A|hypersonic|M5|0m/s|30000m|30,150|AoA12|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 16.037, AoADeg = 16.079, Pitch = 0.5042, Throttle = 0, EngineThrottle = double.NaN, Converged = true, Mode = "no thrust: speed changing -1.46 m/s2, throttle closed" } },
            { "Aeris 4A|subsonic-lat60|M0.5|0m/s|3000m|60,-60|AoA5|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 7.614, AoADeg = 7.586, Pitch = 0.626, Throttle = 0.1186, EngineThrottle = 0.1186, Converged = true, Mode = "level" } },
            { "Aeris 4A|subsonic|M0.5|0m/s|3000m|0,-60|AoA5|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 8.937, AoADeg = 8.907, Pitch = 0.7013, Throttle = 0.1458, EngineThrottle = 0.1458, Converged = true, Mode = "level" } },
            { "Aeris 4A|subsonic-farside|M0.5|0m/s|3000m|0,120|AoA5|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 8.65, AoADeg = 8.619, Pitch = 0.6847, Throttle = 0.1398, EngineThrottle = 0.1398, Converged = true, Mode = "level" } },
            { "Aeris 4A|hypersonic|M7|0m/s|38000m|0,-60|AoA20|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 20, AoADeg = 20.041, Pitch = 0.4092, Throttle = 0, EngineThrottle = double.NaN, Converged = true, Mode = "no thrust: speed changing -1.65 m/s2, throttle closed; no level flight within AoA -10 to 25 (at about -13.2): held at AoA 20.0, +1.30 m/s2 vertical" } },
            { "Aeris 4A|supersonic|M2|0m/s|14000m|0,-60|AoA5|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 3.983, AoADeg = 3.965, Pitch = 0.4199, Throttle = 0.0897, EngineThrottle = 0.0897, Converged = true, Mode = "level" } },
            // Stearwing A300 (all-fixes FAR, 20260926-155733; hypersonic 20260926-161005)
            { "Stearwing A300|vacuum|M0|2200m/s|75000m|-30,30|AoA0|surfaces set|shielded 14", new FarFlightProbe.TrimResult { AoACommanded = 0, AoADeg = 0, Pitch = 0, Throttle = 0, EngineThrottle = double.NaN, Converged = true, Mode = "no air: inputs centered, throttle closed" } },
            { "Stearwing A300|slow-high|M0.3|0m/s|1500m|-20,-30|AoA20|surfaces set|shielded 14", new FarFlightProbe.TrimResult { AoACommanded = 20, AoADeg = 20.147, Pitch = 1, Throttle = 0.5194, EngineThrottle = 0.5194, Converged = true, Mode = "no level flight within AoA -10 to 25 (at about 37.8): held at AoA 20.0, -2.77 m/s2 vertical; pitch input at +1: +1279 kN m left to the hold" } },
            { "Stearwing A300|slow|M0.7|0m/s|1500m|-20,-30|AoA0|surfaces set|shielded 14", new FarFlightProbe.TrimResult { AoACommanded = -0.336, AoADeg = -0.372, Pitch = 0.4319, Throttle = 0.2761, EngineThrottle = 0.2761, Converged = true, Mode = "level" } },
            { "Stearwing A300|hypersonic|M5|0m/s|30000m|30,150|AoA12|surfaces set|shielded 14", new FarFlightProbe.TrimResult { AoACommanded = 13.683, AoADeg = 13.632, Pitch = 1, Throttle = 0, EngineThrottle = double.NaN, Converged = true, Mode = "no thrust: speed changing -2.84 m/s2, throttle closed; pitch input at +1: +737 kN m left to the hold" } },
            { "Stearwing A300|subsonic-lat60|M0.8|0m/s|3000m|60,-60|AoA0|surfaces set|shielded 14", new FarFlightProbe.TrimResult { AoACommanded = -1.085, AoADeg = -1.123, Pitch = 0.2767, Throttle = 0.2963, EngineThrottle = 0.2963, Converged = true, Mode = "level" } },
            { "Stearwing A300|subsonic|M0.8|0m/s|3000m|0,-60|AoA0|surfaces set|shielded 14", new FarFlightProbe.TrimResult { AoACommanded = -0.6, AoADeg = -0.634, Pitch = 0.3996, Throttle = 0.3209, EngineThrottle = 0.3209, Converged = true, Mode = "level" } },
            { "Stearwing A300|subsonic-farside|M0.8|0m/s|3000m|0,120|AoA0|surfaces set|shielded 14", new FarFlightProbe.TrimResult { AoACommanded = -0.7, AoADeg = -0.736, Pitch = 0.3717, Throttle = 0.3154, EngineThrottle = 0.3154, Converged = true, Mode = "level" } },
            { "Stearwing A300|supersonic|M2|0m/s|14000m|0,-60|AoA5|surfaces set|shielded 14", new FarFlightProbe.TrimResult { AoACommanded = 3.553, AoADeg = 3.521, Pitch = 0.8516, Throttle = 1, EngineThrottle = 1, Converged = true, Mode = "throttle at 1: speed changing -4.35 m/s2" } },
            { "Stearwing A300|hypersonic|M7|0m/s|38000m|0,-60|AoA20|surfaces set|shielded 14", new FarFlightProbe.TrimResult { AoACommanded = 20, AoADeg = 19.951, Pitch = 1, Throttle = 0, EngineThrottle = double.NaN, Converged = true, Mode = "no thrust: speed changing -2.77 m/s2, throttle closed; no level flight within AoA -10 to 25 (at about -12.2): held at AoA 20.0, +1.56 m/s2 vertical; pitch input at +1: +592 kN m left to the hold" } },
            // Stearwing A300 PW (all-fixes FAR, 20260926-170309)
            { "Stearwing A300 PW|vacuum|M0|2200m/s|75000m|-30,30|AoA0|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 0, AoADeg = 0, Pitch = 0, Throttle = 0, EngineThrottle = double.NaN, Converged = true, Mode = "no air: inputs centered, throttle closed" } },
            { "Stearwing A300 PW|slow-high|M0.3|0m/s|1500m|-20,-30|AoA20|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 20, AoADeg = 19.271, Pitch = 1, Throttle = 0.4534, EngineThrottle = 0.4534, Converged = true, Mode = "no level flight within AoA -10 to 25 (at about 43.8): held at AoA 20.0, +8.32 m/s2 vertical; pitch input at +1: +2611 kN m left to the hold" } },
            { "Stearwing A300 PW|slow|M0.45|0m/s|1500m|-20,-30|AoA5|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 4.951, AoADeg = 4.917, Pitch = 0.8305, Throttle = 0.1313, EngineThrottle = 0.1313, Converged = true, Mode = "level" } },
            { "Stearwing A300 PW|hypersonic|M5|0m/s|30000m|30,150|AoA12|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 12.387, AoADeg = 12.352, Pitch = 1, Throttle = 0, EngineThrottle = double.NaN, Converged = true, Mode = "no thrust: speed changing -2.57 m/s2, throttle closed; pitch input at +1: +293 kN m left to the hold" } },
            { "Stearwing A300 PW|subsonic-lat60|M0.5|0m/s|3000m|60,-60|AoA5|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 4.427, AoADeg = 4.401, Pitch = 0.7099, Throttle = 0.1405, EngineThrottle = 0.1405, Converged = true, Mode = "level" } },
            { "Stearwing A300 PW|subsonic|M0.5|0m/s|3000m|0,-60|AoA5|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 5.064, AoADeg = 5.033, Pitch = 0.8697, Throttle = 0.1547, EngineThrottle = 0.1547, Converged = true, Mode = "level" } },
            { "Stearwing A300 PW|subsonic-farside|M0.5|0m/s|3000m|0,120|AoA5|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 4.93, AoADeg = 4.897, Pitch = 0.8247, Throttle = 0.1508, EngineThrottle = 0.1508, Converged = true, Mode = "level" } },
            { "Stearwing A300 PW|hypersonic|M7|0m/s|38000m|0,-60|AoA20|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = -6.263, AoADeg = -6.215, Pitch = -1, Throttle = 0, EngineThrottle = double.NaN, Converged = true, Mode = "no thrust: speed changing -2.90 m/s2, throttle closed; pitch input at -1: -50 kN m left to the hold" } },
            { "Stearwing A300 PW|supersonic|M2|0m/s|14000m|0,-60|AoA5|shielded 0", new FarFlightProbe.TrimResult { AoACommanded = 4.093, AoADeg = 4.07, Pitch = 0.3994, Throttle = 1, EngineThrottle = 1, Converged = true, Mode = "throttle at 1: speed changing -3.31 m/s2" } },
        };

        private static string CrossTrimKey(Vessel v, FarFlightProbe.Condition c, bool surfacesSet, int shielded) => string.Format(CultureInfo.InvariantCulture,
            "{0}|{1}|M{2}|{3}m/s|{4}m|{5},{6}|AoA{7}{8}|shielded {9}", v.vesselName, c.Name, c.Mach, c.SpeedMs, c.AltM, c.Lat, c.Lon, c.AoADeg,
            surfacesSet ? "|surfaces set" : "", shielded);

        /// <summary>
        /// Whether a prediction leaves anything behind: FAR's prediction API is called by other mods (Trajectories over a
        /// whole grid of conditions, MechJeb's VesselState) while the vessel flies, so a prediction must neither change
        /// the state the flight reads nor depend on the predictions before it. In one frame, with no physics step in
        /// between: every value field of FAR's per-part state (FarStateSnapshot) is taken; the vessel's own condition
        /// A is predicted; a grid of unrelated conditions (Mach 0.3-8, AoA -10 to 45 deg, 1-50 km, other places) is
        /// predicted; A is predicted again; the state is taken again. Passes only if no field changed and the two
        /// predictions of A are exactly equal. Writes isolation.md (the fields that changed, grouped). Params: craft,
        /// strict (true: fail on any change).
        /// </summary>
        [UITest("Far_PredictionIsolation", Kind = "Check", StartsAt = StartState.Flight, Craft = "Aeris 4A", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 600)]
        public static IEnumerator Far_PredictionIsolation(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            var inv = CultureInfo.InvariantCulture;
            ctx.Record("predictionApi", FarApi.HasPredictAt ? "CalculateVesselAeroForcesAt" : "CalculateVesselAeroForces (altitude only)");
            bool[] trajectories = FarFlightProbe.QuietOtherPredictors();
            Flight.IgnoreMaxTemperature(true);
            Ctrl.SetThrottle(0f);
            Vessel v = FlightGlobals.ActiveVessel;
            v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
            ctx.Record("surfacesLocked", FarApi.SetMaxDeflect(v, 0));

            var c = CrossTargets[0];
            ctx.Step($"Flying at {c}");
            yield return FarFlightProbe.Place(c);
            ctx.Record("gearRetracted", Ctrl.RetractGear());
            yield return Wait.Seconds(8);
            var settleSamples = new List<FarFlightProbe.Sample>();
            yield return FarFlightProbe.Hold(c, 25, 1, settleSamples, predict: false);

            ctx.Step("Snapshot, predictions, snapshot (one frame)");
            v = FlightGlobals.ActiveVessel;
            CelestialBody body = v.mainBody;
            Vector3 nose = v.ReferenceTransform.up, roof = -v.ReferenceTransform.forward;
            double ut = Planetarium.GetUniversalTime();
            Vector3 velA = v.srf_velocity;
            double latA = v.latitude, lonA = v.longitude, altA = v.altitude;

            Dictionary<string, string> before = FarStateSnapshot.Take(v);
            FarApi.ClearMemo(v);
            FarApi.PredictAtFull(v, velA, latA, lonA, altA, ut, out Vector3 fA0, out Vector3 tA0);
            int n = 0;
            foreach (double mach in new[] { 0.3, 0.9, 2.0, 5.0, 8.0 })
                foreach (double aoa in new[] { -10.0, 0.0, 10.0, 25.0, 45.0 })
                    foreach (double altM in new[] { 1000.0, 20000.0, 50000.0 })
                    {
                        double p = body.GetPressure(altM);
                        double speed = mach * body.GetSpeedOfSound(p, body.GetDensity(p, body.GetTemperature(altM)));
                        double a = aoa * Math.PI / 180;
                        Vector3 vel = (float)speed * ((float)Math.Cos(a) * nose - (float)Math.Sin(a) * roof);
                        FarApi.ClearMemo(v);
                        FarApi.PredictAtFull(v, vel, 30 - n % 60, 150 - 3 * n, altM, ut + 600 * n, out _, out _);
                        ++n;
                    }
            FarApi.ClearMemo(v);
            FarApi.PredictAtFull(v, velA, latA, lonA, altA, ut, out Vector3 fA1, out Vector3 tA1);
            Dictionary<string, string> after = FarStateSnapshot.Take(v);

            List<string> changed = FarStateSnapshot.Diff(before, after);
            // Exact: Unity's Vector3 == is equality within about 1e-5.
            bool sameA = fA0.x.Equals(fA1.x) && fA0.y.Equals(fA1.y) && fA0.z.Equals(fA1.z) &&
                         tA0.x.Equals(tA1.x) && tA0.y.Equals(tA1.y) && tA0.z.Equals(tA1.z);
            int parts = changed.Select(FarStateSnapshot.PartOf).Distinct().Count();
            ctx.Record("fieldsWatched", before.Count);
            ctx.Record("predictions", n + 2);
            ctx.Record("fieldsChanged", changed.Count);
            ctx.Record("partsChanged", parts);
            ctx.Record("changedFields", FarStateSnapshot.Summarize(changed));
            ctx.Record("sameAnswerForA", sameA);
            string V(Vector3 x) => $"({x.x.ToString("R", inv)}, {x.y.ToString("R", inv)}, {x.z.ToString("R", inv)})";
            if (!sameA)
                ctx.Record("answerForA", $"force {V(fA0)} then {V(fA1)}; torque {V(tA0)} then {V(tA1)}");

            var md = new System.Text.StringBuilder("# FAR prediction isolation\n\n");
            md.Append($"Vessel {v.vesselName}, flying at {c} with the control surfaces locked. In one frame: {before.Count} value fields of FAR's per-part state ");
            md.Append($"taken, the vessel's own condition predicted, {n} unrelated conditions predicted, the own condition predicted again, the state taken again.\n\n");
            md.Append($"- fields changed: **{changed.Count}** on {parts} parts\n");
            md.Append($"- same answer for the own condition before and after: **{(sameA ? "yes" : "no")}**\n");
            if (!sameA)
                md.Append($"  - force {V(fA0)} then {V(fA1)} kN; torque {V(tA0)} then {V(tA1)} kN m\n");
            md.Append("\n## Changed fields, by name\n\n");
            foreach (var g in changed.Select(FarStateSnapshot.FieldOf).GroupBy(f => f).OrderByDescending(g => g.Count()))
                md.Append($"- {g.Key}: {g.Count()} parts\n");
            md.Append("\n## Every change\n\n```\n").Append(string.Join("\n", changed.Take(2000))).Append("\n```\n");
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "isolation.md"), md.ToString());
            FarFlightProbe.ResumeOtherPredictors(trajectories);

            if (ctx.Param("strict", "true") != "false")
            {
                Assert.True(changed.Count == 0, $"predictions left FAR's flight state as it was ({changed.Count} fields changed: {FarStateSnapshot.Summarize(changed, 8)})");
                Assert.True(sameA, "the own condition predicted the same before and after the others");
            }
        }

        private struct PredictionInput
        {
            public Vector3 Vel;
            public Vector3d LatLonAlt;
            public double UT;
        }

        /// <summary>
        /// FAR's predictions off the main thread (FARAPI.CaptureVesselSnapshot on the main thread, then
        /// CalculateVesselAeroForcesAt(snapshot, ...) from any thread). Flying at a condition with the surfaces locked:
        /// (1) in the capture frame, the snapshot's answer for the vessel's own condition against the live API's, to
        /// 1e-6 relative (the two reach the same inputs by different float arithmetic; everything else is exact);
        /// (2) every input (the own condition and a grid of unrelated conditions and places) on the main thread and on
        /// one worker thread, exactly equal; (3) 8 worker threads at once, each in its own order, the main thread
        /// waiting inside one frame: every answer exactly the main thread's and FAR's state (FarStateSnapshot) unchanged;
        /// (4) 8 workers predicting for `flyFrames` frames while the vessel flies, the main thread predicting live and
        /// capturing new snapshots meanwhile (its call times recorded): every answer still exact, no exceptions;
        /// (5) the live-vessel API called from a worker refuses (zero); (6) after the gear comes down and FAR
        /// re-voxelizes, the old snapshot is refused and a new one works. Writes threads.md. Params: craft, threads (8),
        /// repeats (3), flyFrames (100), strict (true).
        /// </summary>
        [UITest("Far_PredictionThreads", Kind = "Check", StartsAt = StartState.Flight, Craft = "Aeris 4A", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Far_PredictionThreads(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            Assert.True(FarSnapshotApi.Present, "FAR has the snapshot API (FARAPI.CaptureVesselSnapshot)");
            int threads = (int)ctx.ParamDouble("threads", 8), repeats = (int)ctx.ParamDouble("repeats", 3);
            int flyFrames = (int)ctx.ParamDouble("flyFrames", 100);
            bool[] trajectories = FarFlightProbe.QuietOtherPredictors();
            Flight.IgnoreMaxTemperature(true);
            Ctrl.SetThrottle(0f);
            Vessel v = FlightGlobals.ActiveVessel;
            v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
            FarApi.SetMaxDeflect(v, 0);

            var c = CrossTargets[0];
            ctx.Step($"Flying at {c}");
            yield return FarFlightProbe.Place(c);
            Ctrl.RetractGear();
            yield return Wait.Seconds(8);
            var none = new List<FarFlightProbe.Sample>();
            yield return FarFlightProbe.Hold(c, 25, 1, none, predict: false);

            // ---- the capture frame: everything up to the 8-thread batch happens with no physics step in between
            ctx.Step("Capture frame: snapshot vs live, main thread vs one worker, 8 workers");
            v = FlightGlobals.ActiveVessel;
            CelestialBody body = v.mainBody;
            Vector3 nose = v.ReferenceTransform.up, roof = -v.ReferenceTransform.forward;
            double ut = Planetarium.GetUniversalTime();
            var inputs = new List<PredictionInput>
            {
                new PredictionInput { Vel = v.srf_velocity, LatLonAlt = new Vector3d(v.latitude, v.longitude, v.altitude), UT = ut },
            };
            for (int rep = 0; rep < 4; ++rep)
                foreach (double mach in new[] { 0.3, 0.9, 2.0, 5.0, 8.0 })
                    foreach (double aoa in new[] { -10.0, 0.0, 10.0, 25.0, 45.0 })
                        foreach (double altM in new[] { 1000.0, 20000.0, 50000.0 })
                        {
                            int n = inputs.Count;
                            double p = body.GetPressure(altM);
                            double speed = mach * body.GetSpeedOfSound(p, body.GetDensity(p, body.GetTemperature(altM)));
                            double a = (aoa + 2 * rep) * Math.PI / 180;
                            inputs.Add(new PredictionInput
                            {
                                Vel = (float)speed * ((float)Math.Cos(a) * nose - (float)Math.Sin(a) * roof),
                                LatLonAlt = new Vector3d(60 - (7 * n) % 120, 180 - (13 * n) % 360, altM),
                                UT = ut + 900 * n,
                            });
                        }
            int count = inputs.Count;
            ctx.Record("inputs", count);

            object snap = FarSnapshotApi.Capture(v);
            Assert.True(snap != null, "a snapshot of the flying vessel");
            FarApi.ClearMemo(v);
            FarSnapshotApi.PredictLive(v, inputs[0].Vel, inputs[0].LatLonAlt, inputs[0].UT, out Vector3 fLive, out Vector3 tLive);

            var main = new Vector3[count, 2];
            var ok = new bool[count];
            for (int i = 0; i < count; ++i)
            {
                ok[i] = FarSnapshotApi.Predict(snap, inputs[i].Vel, inputs[i].LatLonAlt, inputs[i].UT, out main[i, 0], out main[i, 1]);
            }
            int refused = ok.Count(b => !b);
            // The live API reads Unity's transforms and KSP's atmosphere functions directly, the snapshot its copies of
            // them computed in C#: the same inputs by different arithmetic, so the two agree to float rounding (about
            // one unit in the last place of a float), not bit for bit. Every other comparison here is exact.
            const double LiveTolerance = 1e-6;
            double relForce = (main[0, 0] - fLive).magnitude / Math.Max(1e-9f, fLive.magnitude);
            // A trimmed vessel's torque is near zero, so the torque's rounding is scaled by what it is made of: the
            // force times the vessel's size (a float's rounding in a lever arm shows as force x that rounding).
            double torqueScale = Math.Max(tLive.magnitude, fLive.magnitude * v.vesselSize.magnitude);
            double relTorque = (main[0, 1] - tLive).magnitude / Math.Max(1e-9, torqueScale);
            bool snapMatchesLive = relForce <= LiveTolerance && relTorque <= LiveTolerance;
            ctx.Record("refusedFresh", refused);
            ctx.Record("snapshotMatchesLive", snapMatchesLive);
            ctx.Record("snapshotVsLiveRel", $"force {relForce:E2}, torque {relTorque:E2} (tolerance {LiveTolerance:E0})");

            var errors = new System.Collections.Concurrent.ConcurrentQueue<string>();
            var mismatches = new int[1];
            // Runs every input in the given order `times` times, counting answers that differ from the main thread's.
            void Work(int[] order, int times)
            {
                try
                {
                    for (int r = 0; r < times; ++r)
                        foreach (int i in order)
                        {
                            bool good = FarSnapshotApi.Predict(snap, inputs[i].Vel, inputs[i].LatLonAlt, inputs[i].UT, out Vector3 f, out Vector3 t);
                            if (good != ok[i] || !FarSnapshotApi.Same(f, t, main[i, 0], main[i, 1]))
                                System.Threading.Interlocked.Increment(ref mismatches[0]);
                        }
                }
                catch (Exception e) { errors.Enqueue(e.InnerException?.ToString() ?? e.ToString()); }
            }

            int[] identity = Enumerable.Range(0, count).ToArray();
            var one = new System.Threading.Thread(() => Work(identity, 1));
            one.Start();
            one.Join();
            int oneWorkerMismatches = mismatches[0];
            ctx.Record("oneWorkerMismatches", oneWorkerMismatches);

            Dictionary<string, string> before = FarStateSnapshot.Take(v);
            mismatches[0] = 0;
            var rng = new System.Random(4242);
            var workers = new List<System.Threading.Thread>();
            var sw = System.Diagnostics.Stopwatch.StartNew();
            for (int k = 0; k < threads; ++k)
            {
                int[] order = identity.OrderBy(_ => rng.Next()).ToArray();
                workers.Add(new System.Threading.Thread(() => Work(order, repeats)));
            }
            foreach (var w in workers) w.Start();
            foreach (var w in workers) w.Join();
            double frozenBatchMs = sw.Elapsed.TotalMilliseconds;
            List<string> changed = FarStateSnapshot.Diff(before, FarStateSnapshot.Take(v));
            int frozenMismatches = mismatches[0];
            ctx.Record("frozenBatch", $"{threads} threads x {repeats} x {count} in {frozenBatchMs:F0} ms");
            ctx.Record("frozenMismatches", frozenMismatches);
            ctx.Record("frozenFieldsChanged", changed.Count);
            if (changed.Count > 0) ctx.Record("frozenChangedFields", FarStateSnapshot.Summarize(changed));

            // ---- flying: workers keep predicting across frames while the main thread flies and predicts live
            ctx.Step($"{threads} workers for {flyFrames} frames while flying");
            mismatches[0] = 0;
            var stop = new bool[1];
            int calls = 0;
            workers.Clear();
            for (int k = 0; k < threads; ++k)
            {
                int[] order = identity.OrderBy(_ => rng.Next()).ToArray();
                workers.Add(new System.Threading.Thread(() =>
                {
                    while (!System.Threading.Volatile.Read(ref stop[0]))
                    {
                        Work(order, 1);
                        System.Threading.Interlocked.Add(ref calls, order.Length);
                    }
                }));
            }
            foreach (var w in workers) w.Start();
            double maxLiveMs = 0, maxCaptureMs = 0;
            int captures = 0, capturesRefused = 0;
            var fixedWait = new WaitForFixedUpdate();
            for (int frame = 0; frame < flyFrames; ++frame)
            {
                Flight.PointAtAoA(c.AoADeg);
                foreach (Part part in FlightGlobals.ActiveVessel.parts)
                    if (part.Rigidbody != null) part.Rigidbody.angularVelocity = Vector3.zero;
                yield return fixedWait;
                v = FlightGlobals.ActiveVessel;
                var t0 = System.Diagnostics.Stopwatch.StartNew();
                FarApi.ClearMemo(v);
                FarSnapshotApi.PredictLive(v, v.srf_velocity, new Vector3d(v.latitude, v.longitude, v.altitude), Planetarium.GetUniversalTime(), out _, out _);
                maxLiveMs = Math.Max(maxLiveMs, t0.Elapsed.TotalMilliseconds);
                if (frame % 10 == 0)
                {
                    t0 = System.Diagnostics.Stopwatch.StartNew();
                    object s2 = FarSnapshotApi.Capture(v);
                    maxCaptureMs = Math.Max(maxCaptureMs, t0.Elapsed.TotalMilliseconds);
                    ++captures;
                    if (s2 == null || !FarSnapshotApi.Predict(s2, v.srf_velocity, new Vector3d(v.latitude, v.longitude, v.altitude), Planetarium.GetUniversalTime(), out _, out _))
                        ++capturesRefused;
                }
            }
            System.Threading.Volatile.Write(ref stop[0], true);
            foreach (var w in workers) w.Join();
            int flyingMismatches = mismatches[0];
            ctx.Record("flyingCalls", calls);
            ctx.Record("flyingMismatches", flyingMismatches);
            ctx.Record("mainThreadLiveMaxMs", Math.Round(maxLiveMs, 2));
            ctx.Record("mainThreadCaptureMaxMs", Math.Round(maxCaptureMs, 2));
            ctx.Record("capturesWhileFlying", $"{captures}, refused {capturesRefused}");

            // ---- the live API from a worker must refuse
            ctx.Step("Live API from a worker, stale snapshot");
            Vector3 fOff = Vector3.one, tOff = Vector3.one;
            Vessel vw = FlightGlobals.ActiveVessel;
            Vector3 velW = vw.srf_velocity;
            var lla = new Vector3d(vw.latitude, vw.longitude, vw.altitude);
            double utW = Planetarium.GetUniversalTime();
            var off = new System.Threading.Thread(() =>
            {
                try { FarSnapshotApi.PredictLive(vw, velW, lla, utW, out fOff, out tOff); }
                catch (Exception e) { errors.Enqueue("live API off-thread: " + (e.InnerException?.Message ?? e.Message)); }
            });
            off.Start();
            off.Join();
            bool liveRefusedOffThread = fOff == Vector3.zero && tOff == Vector3.zero;
            ctx.Record("liveApiOffThreadRefused", liveRefusedOffThread);

            // ---- stale: the gear changes the shape, FAR re-voxelizes, the old snapshot must be refused
            Ctrl.Gear(true);
            yield return Wait.Seconds(6);
            v = FlightGlobals.ActiveVessel;
            bool staleRefused = !FarSnapshotApi.Predict(snap, inputs[0].Vel, inputs[0].LatLonAlt, inputs[0].UT, out _, out _);
            object fresh = FarSnapshotApi.Capture(v);
            bool freshWorks = fresh != null && FarSnapshotApi.Predict(fresh, v.srf_velocity, new Vector3d(v.latitude, v.longitude, v.altitude), Planetarium.GetUniversalTime(), out _, out _);
            ctx.Record("staleSnapshotRefused", staleRefused);
            ctx.Record("freshSnapshotWorks", freshWorks);
            ctx.Record("exceptions", errors.Count);
            FarFlightProbe.ResumeOtherPredictors(trajectories);

            var md = new System.Text.StringBuilder("# FAR predictions off the main thread\n\n");
            md.Append($"Vessel {v.vesselName}, flying at {c}, surfaces locked; {count} inputs (the own condition and unrelated conditions, places and times).\n\n");
            md.Append("| check | result |\n|---|---|\n");
            md.Append($"| snapshot answer = live answer to {LiveTolerance:E0} relative, own condition, capture frame | {snapMatchesLive} (force {relForce:E2}, torque {relTorque:E2}) |\n");
            md.Append($"| fresh snapshot refused | {refused} of {count} |\n");
            md.Append($"| one worker = main thread | {count - oneWorkerMismatches} of {count} exact |\n");
            md.Append($"| {threads} workers x {repeats}, main thread waiting in one frame | {frozenMismatches} mismatches in {threads * repeats * count}, {frozenBatchMs:F0} ms |\n");
            md.Append($"| FAR flight state changed by that batch | {changed.Count} fields |\n");
            md.Append($"| {threads} workers for {flyFrames} frames while flying | {flyingMismatches} mismatches in {calls} calls |\n");
            md.Append($"| main thread meanwhile: live prediction / capture, max ms | {maxLiveMs:F2} / {maxCaptureMs:F2} |\n");
            md.Append($"| snapshots captured while flying (refused) | {captures} ({capturesRefused}) |\n");
            md.Append($"| live-vessel API from a worker refused | {liveRefusedOffThread} |\n");
            md.Append($"| old snapshot refused after re-voxelization / new one works | {staleRefused} / {freshWorks} |\n");
            md.Append($"| exceptions on workers | {errors.Count} |\n");
            if (errors.Count > 0)
                md.Append("\n## Exceptions\n\n```\n").Append(string.Join("\n---\n", errors.Take(5))).Append("\n```\n");
            if (changed.Count > 0)
                md.Append("\n## Changed fields\n\n```\n").Append(string.Join("\n", changed.Take(500))).Append("\n```\n");
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "threads.md"), md.ToString());

            if (ctx.Param("strict", "true") != "false")
            {
                Assert.True(errors.Count == 0, $"no exceptions on worker threads ({errors.Count}; first: {errors.FirstOrDefault()})");
                Assert.True(snapMatchesLive, $"the snapshot's answer for the own condition is the live API's to {LiveTolerance:E0} in the capture frame (force {relForce:E2}, torque {relTorque:E2})");
                Assert.True(refused == 0, $"a fresh snapshot is not refused ({refused} of {count})");
                Assert.True(oneWorkerMismatches == 0, $"one worker thread gives the main thread's answers ({oneWorkerMismatches} differ)");
                Assert.True(frozenMismatches == 0, $"{threads} concurrent workers give the main thread's answers ({frozenMismatches} differ)");
                Assert.True(changed.Count == 0, $"concurrent predictions leave FAR's flight state as it was ({changed.Count} fields: {FarStateSnapshot.Summarize(changed, 8)})");
                Assert.True(flyingMismatches == 0, $"workers predicting while the vessel flies give the main thread's answers ({flyingMismatches} of {calls} differ)");
                Assert.True(liveRefusedOffThread, "the live-vessel API refuses a worker thread");
                Assert.True(staleRefused && freshWorks, "a snapshot taken before re-voxelization is refused and a new one works");
            }
        }

        private static double Mean(List<FarFlightProbe.Sample> list, Func<FarFlightProbe.Sample, double> f) => list.Count == 0 ? double.NaN : list.Average(f);

        // Mean error ± its SD, % of the mean total force (moments: of force x length).
        private static string ErrPct(List<FarFlightProbe.Sample> list, int k, double length)
        {
            if (list.Count < 2) return "-";
            double f = list.Average(s => s.ForceKN), scale = k < 3 ? f : f * length;
            FarFlightProbe.Stats(list.Select(s => s.Predicted[k] - s.Flight[k]), out double m, out double sd);
            return string.Format(CultureInfo.InvariantCulture, "{0:+0.00;-0.00} ± {1:0.00}", 100 * m / scale, 100 * sd / scale);
        }

        private static string FlightSdPct(List<FarFlightProbe.Sample> list, int k, double length)
        {
            if (list.Count < 2) return "-";
            double f = list.Average(s => s.ForceKN), scale = k < 3 ? f : f * length;
            FarFlightProbe.Stats(list.Select(s => s.Flight[k]), out _, out double sd);
            return (100 * sd / scale).ToString("0.000", CultureInfo.InvariantCulture);
        }

        // Heading of the surface velocity (deg from north).
        private static double HeadingDeg(Vessel v)
        {
            CelestialBody body = v.mainBody;
            Vector3d pos = body.GetWorldSurfacePosition(v.latitude, v.longitude, v.altitude);
            Vector3d north = (body.GetWorldSurfacePosition(v.latitude + 1e-4, v.longitude, v.altitude) - pos).normalized;
            Vector3d east = (body.GetWorldSurfacePosition(v.latitude, v.longitude + 1e-4, v.altitude) - pos).normalized;
            Vector3d vel = v.srf_velocity;
            return Math.Atan2(Vector3d.Dot(vel, east), Vector3d.Dot(vel, north)) * 180 / Math.PI;
        }

        // Extent of the vessel's parts along its nose axis (m), the length the moment errors are scaled by.
        private static double VesselLength(Vessel v)
        {
            Vector3 nose = v.ReferenceTransform.up;
            float lo = float.MaxValue, hi = float.MinValue;
            foreach (Part p in v.parts)
            {
                float d = Vector3.Dot(p.transform.position, nose);
                lo = Mathf.Min(lo, d);
                hi = Mathf.Max(hi, d);
            }
            return hi > lo ? hi - lo : 1.0;
        }
    }
}
