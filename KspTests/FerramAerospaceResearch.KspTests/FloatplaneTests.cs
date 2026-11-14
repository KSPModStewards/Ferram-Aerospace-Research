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
using static FerramAerospaceResearch.KspTests.Refl;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The Cessna 172M on EDO 89-2000 floats (crafts/c172_floatplane.py), to compare FAR's water and air with the 1973
    /// Cessna floatplane supplement (water takeoff run and distance over 50 ft at 2220/1950/1700 lb, landing run, climb)
    /// and FAA TCDS 3A12 (seaplane static rpm, weights, C.G.).
    /// </summary>
    public static class FloatplaneTests
    {
        // the plane with B9's own hinged control surfaces and a water rudder on each float (crafts/c172_floatplane.py
        // hinged; its plain variant, the controls FAR surfaces in B9 strips, is set aside in crafts/stashed)
        private const string Craft = "Cessna 172M Floatplane Hinged";
        // the same plane with each float three parts (crafts/c172_floatplane.py split hinged): the *Split tests run it as
        // the one-part floats are run; FAR's water should not tell them apart
        private const string SplitCraft = "Cessna 172M Floatplane Split Hinged";
        private const string HingedCraft = Craft;
        // the hinged craft on RO's parts (crafts/c172_floatplane.py ro), for RealismOverhaul / RP-1 installs (KSPDIR)
        private const string HingedRoCraft = "Cessna 172M Floatplane Hinged RO";
        private const string Mods = TestWarp.WaterMods;   // the water tests' instance

        private const double Lb = 0.45359237e-3;          // t
        private const double GrossT = 2220 * Lb;          // TCDS 3A12: 2220 lb on floats
        // crafts/c172_floatplane.py: the CG station (28.4% MAC, the TCDS seaplane range's middle) along the fuselage
        // root's local y (craft z), and the float's step keel in the float's frame (local y forward, z up)
        private const double CgLocalY = 1.499;
        private static readonly Vector3 StepKeel = new Vector3(0, -0.260f, -0.593f);
        // hand statics (scratchpad c172_rest.py): rest trim (deg, bow up) and the step keel's depth (m)
        private const double RestTrim = 3.3, RestStepDraft = 0.384;
        private const double StaticRpmMin = 2445, StaticRpmMax = 2545;     // TCDS 3A12, 172M, 1A175/ATM 8042
        private const float FloatLength = 4.889f;

        private class Plane
        {
            public Vessel V;
            public Part Root, Fuel, Engine;
            public string FuelName;
            public HullParts[] Floats;
            public PartModule Aje;
            public double Get(string field) => Convert.ToDouble(Aje.Fields[field].GetValue(Aje), CultureInfo.InvariantCulture);
        }

        /// <summary>
        /// The setup the comparisons stand on, against the sources: weight (fuel to the 2220 lb gross), C.G. (the fuselage's
        /// CoM moved to 28.4% MAC), FAR's wing, tail and fin areas (172: 174, 21.9, 16.5 sq ft), the attitude and step draft
        /// afloat at rest (hand statics: 3.3 deg, 0.384 m), and full throttle held still on the water: static rpm against
        /// the TCDS band (2445 - 2545), static thrust (the force holding it), shaft power, manifold pressure.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = Craft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Setup_C172Floatplane(TestContext ctx) => Setup(ctx);

        /// <summary>Setup_C172Floatplane with each float three parts.</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = SplitCraft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Setup_C172FloatplaneSplit(TestContext ctx) => Setup(ctx);

        private static IEnumerator Setup(TestContext ctx)
        {
            Hyper(ctx, false);
            StandardDay(ctx, null);
            WaterParams.Restore();
            yield return Wait.Seconds(5);
            Plane p = null;
            foreach (object o in Prepare(ctx, r => p = r))
                yield return o;
            Hyper(ctx, true);
            Vessel v = p.V;
            CelestialBody body = v.mainBody;
            Rigidbody rb = p.Root.rb;
            Vector3 up = v.upAxis;
            Vector3 h = Vector3.ProjectOnPlane(p.Root.transform.up, up).normalized;
            var fixedWait = new WaitForFixedUpdate();

            // afloat at rest, 15 s to settle, 5 s averaged
            ctx.Step("at rest");
            var trim = new List<double>(); var draft = new List<double>();
            double t0 = Planetarium.GetUniversalTime();
            while (Planetarium.GetUniversalTime() - t0 < 20)
            {
                if (Planetarium.GetUniversalTime() - t0 > 15)
                {
                    trim.Add(Math.Asin(Mathf.Clamp(Vector3.Dot(p.Root.transform.up, up), -1, 1)) * 180 / Math.PI);
                    draft.Add(p.Floats.Average(f => -FlightGlobals.getAltitudeAtPos(f.TransformPoint(StepKeel), body)));
                }
                yield return fixedWait;
            }
            ctx.Record("rest", $"trim {trim.Average():F2} deg (hand {RestTrim:F1}), step keel {draft.Average():F3} m deep (hand {RestStepDraft:F3})");

            // full throttle, held still: the tow line's force is the static thrust
            ctx.Step("static run-up");
            p.Engine.Modules.GetModule<ModuleEngines>().Activate();
            Ctrl.Attach();
            Ctrl.SetThrottle(1f);
            var rpm = new List<double>(); var thrust = new List<double>(); var aje = new List<double>(); var hp = new List<double>(); var map = new List<double>();
            double m = v.GetTotalMass();
            t0 = Planetarium.GetUniversalTime();
            while (Planetarium.GetUniversalTime() - t0 < 15)
            {
                double dt = TimeWarp.fixedDeltaTime;
                Vector3 vel = rb.velocity;
                double towKN = m * Vector3.Dot(vel, h) / dt;      // what holding it at rest takes out each step
                foreach (Part q in v.parts.Where(q => q.rb != null))
                    q.rb.velocity -= h * Vector3.Dot(q.rb.velocity, h);
                if (Planetarium.GetUniversalTime() - t0 > 8)
                {
                    rpm.Add(p.Get("propRPM")); thrust.Add(towKN); aje.Add(p.Get("propThrust")); hp.Add(p.Get("brakeShaftPower")); map.Add(p.Get("manifoldPressure"));
                }
                yield return fixedWait;
            }
            Ctrl.SetThrottle(0f);
            double r = rpm.Average();
            ctx.Record("static", $"{r:F0} rpm (TCDS {StaticRpmMin}-{StaticRpmMax}), thrust {thrust.Average() / 4.448e-3:F0} lbf at the tow line " +
                                 $"(AJE's propeller {aje.Average() / 4.448e-3:F0} lbf), {hp.Average():F0} hp, {map.Average():F2} inHg; {Ambient(v)}");
            Hyper(ctx, false);
            StandardDay(ctx, null);
            WaterParams.Restore();
            Assert.True(r >= StaticRpmMin && r <= StaticRpmMax, $"static rpm {r:F0} within the TCDS band {StaticRpmMin}-{StaticRpmMax}");
        }

        // the air: temperature against the standard day's 59 F, pressure, density (the supplement's tables are for a
        // standard day: distances +10% for each 25 F above it)
        private static string Ambient(Vessel v)
        {
            double tF = (v.atmosphericTemperature - 273.15) * 9 / 5 + 32;
            return $"air {v.atmosphericTemperature - 273.15:F1} C ({tF:F0} F, {tF - 59:+0;-0} F from standard), {v.staticPressurekPa:F2} kPa, " +
                   $"{v.atmDensity:F4} kg/m3 (sigma {v.atmDensity / 1.225:F3})";
        }

        // TimeControl's hyper-warp (parameter hyperWarp, default 4; 1 for real time) at the 1x physics step: the same
        // simulation, more physics frames per rendered frame. Off at each test's start (a failed test leaves it on) and end.
        private static void Hyper(TestContext ctx, bool on)
        {
            double rate = ctx.ParamDouble("hyperWarp", 4);
            if (!on || rate <= 1)
            {
                Flight.HyperWarpOff();
                return;
            }
            ctx.Record("hyperWarp", Flight.HyperWarp(rate, 1.0) ? $"{rate:F0}x attempted, time scale {Time.timeScale:F1}" : "refused (real time)");
        }

        // a standard day at sea level for the supplement's tables (parameter standardDay, default on): the body's
        // temperature offsets (latitude, sun, axial tilt, eccentricity: KSC's afternoon is ~41 C) set to none, so the
        // air is its base curve (Kerbin: 15 C, 101.325 kPa, 1.225 kg/m3); null puts the body's own back
        private static FloatCurve[] savedTemperatureCurves;
        private static CelestialBody savedTemperatureBody;

        private static void StandardDay(TestContext ctx, CelestialBody body)
        {
            if (body == null)
            {
                if (savedTemperatureBody != null)
                {
                    CelestialBody b = savedTemperatureBody;
                    b.latitudeTemperatureBiasCurve = savedTemperatureCurves[0];
                    b.latitudeTemperatureSunMultCurve = savedTemperatureCurves[1];
                    b.axialTemperatureSunBiasCurve = savedTemperatureCurves[2];
                    b.axialTemperatureSunMultCurve = savedTemperatureCurves[3];
                    b.eccentricityTemperatureBiasCurve = savedTemperatureCurves[4];
                    savedTemperatureBody = null;
                }
                return;
            }
            if (!(ctx.Param("standardDay", "on") ?? "on").Equals("on", StringComparison.OrdinalIgnoreCase) || savedTemperatureBody != null)
                return;
            savedTemperatureBody = body;
            savedTemperatureCurves = new[]
            {
                body.latitudeTemperatureBiasCurve, body.latitudeTemperatureSunMultCurve, body.axialTemperatureSunBiasCurve,
                body.axialTemperatureSunMultCurve, body.eccentricityTemperatureBiasCurve
            };
            FloatCurve Zero()
            {
                var c = new FloatCurve();
                c.Add(-1000, 0, 0, 0);
                c.Add(1000, 0, 0, 0);
                return c;
            }
            body.latitudeTemperatureBiasCurve = Zero();
            body.latitudeTemperatureSunMultCurve = Zero();
            body.axialTemperatureSunBiasCurve = Zero();
            body.axialTemperatureSunMultCurve = Zero();
            body.eccentricityTemperatureBiasCurve = Zero();
        }

        /// <summary>
        /// The airframe's pitch rigging, in the air by FAR's API (no water): at each speed (mphs, sea level) and flap
        /// setting, the angle of attack at which the wing carries the weight, and the pitching moment about the C.G. there
        /// at elevator settings -0.5, 0, 0.5, 1 (of its 28 deg up travel): the elevator the airplane trims at. A 172 in
        /// cruise trims with the elevator about neutral (its tab near neutral). Writes trim.csv. Parameters: mphs (+
        /// separated), weightLb.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = Craft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Trim_C172Floatplane(TestContext ctx) => Trim(ctx);

        private static IEnumerator Trim(TestContext ctx)
        {
            double[] mphs = ctx.ParamList("mphs", "60+80+110").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            double weightLb = ctx.ParamDouble("weightLb", 2220);
            Hyper(ctx, false);
            StandardDay(ctx, null);
            WaterParams.Restore();
            yield return Wait.Seconds(5);
            Plane p = null;
            foreach (object o in Prepare(ctx, r => p = r, weightLb * Lb, afloat: false))
                yield return o;
            Hyper(ctx, true);
            Vessel v = p.V;
            double weight = v.GetTotalMass() * 9.80665 * 1000;     // N
            var flaps = v.parts.SelectMany(q => q.Modules.OfType<ferram4.FARControllableSurface>()).Where(c => c.isFlap).ToList();
            Ctrl.Attach();
            var csv = new StringBuilder("flaps,mph,elevator,alpha_deg,CL,moment_kNm\n");
            Vector3 fwd = p.Root.transform.up, bodyUp = -p.Root.transform.forward, right = p.Root.transform.right;

            // where FAR takes moments (KSP's vessel.CoM) against the parts' rigidbodies' centre of mass, and the airframe's
            // neutral point (clean, elevator neutral, 110 mph): moment and lift at alpha 0 and 6 deg
            yield return Wait.Seconds(2);
            Vector3 rbCom = WorldCoM(v);
            double comGap = Vector3.Dot((Vector3)v.CoM - rbCom, fwd);
            var np = new List<(double l, double m)>();
            // each surface's lift and moment about the CoM, by kind (the rest of the total is the bodies')
            var byKind = new Dictionary<string, double[]>();
            foreach (double a in new[] { 0.0, 6.0 })
            {
                double ar = a * Math.PI / 180, V0 = 110 * 0.44704;
                Vector3 air = (fwd * (float)Math.Cos(ar) - bodyUp * (float)Math.Sin(ar)) * (float)V0;
                FARAPI.CalculateVesselAeroForces(v, out Vector3 f, out Vector3 tq, air, 0);
                Vector3 liftDir = Vector3.Cross(air.normalized, right).normalized;
                if (Vector3.Dot(liftDir, bodyUp) < 0) liftDir = -liftDir;
                np.Add((Vector3.Dot(f, liftDir), -Vector3.Dot(tq, right)));
                Vector3 com = v.CoM;
                var dummy = new ferram4.FARCenterQuery();
                double wingsM = 0, wingsL = 0;
                // inside a prediction at this airflow, as FARAPI's own: a surface hinged behind a wing reads the wing's
                // angle of attack at it, not at the parked plane's live velocity
                ferram4.SimThreadContext.PredictionCall call = ferram4.SimThreadContext.BeginPrediction(air);
                try
                {
                foreach (Part q in v.parts)
                    foreach (var w in q.Modules.OfType<ferram4.FARWingAerodynamicModel>())
                    {
                        Vector3d wf = w.PrecomputeCenterOfLift(air, V0 / 340.0, 1.2, dummy);
                        Vector3 ac = (Vector3)w.GetAerodynamicCenter();
                        Vector3 lp = p.Root.transform.InverseTransformPoint(q.transform.position);
                        bool upright = Mathf.Abs(p.Root.transform.InverseTransformDirection(q.transform.right).z) > 0.7f;
                        string kind = upright ? "fin" : lp.y < -2 ? "tail" : w is ferram4.FARControllableSurface c && (c.isFlap || c.rollaxis != 0) ? (c.isFlap ? "flaps" : "ailerons") : "wing";
                        double l = Vector3.Dot((Vector3)wf, liftDir), m = -Vector3.Dot(Vector3.Cross(ac - com, (Vector3)wf), right);
                        if (!byKind.TryGetValue(kind, out double[] acc))
                            byKind[kind] = acc = new double[4];
                        acc[a > 0 ? 2 : 0] += l;
                        acc[a > 0 ? 3 : 1] += m;
                        wingsL += l; wingsM += m;
                    }
                }
                finally
                {
                    ferram4.SimThreadContext.EndPrediction(call);
                }
                if (!byKind.TryGetValue("bodies", out double[] bodies))
                    byKind["bodies"] = bodies = new double[4];
                bodies[a > 0 ? 2 : 0] += np[np.Count - 1].l - wingsL;
                bodies[a > 0 ? 3 : 1] += np[np.Count - 1].m - wingsM;
            }
            ctx.Record("breakdown", string.Join("; ", byKind.Select(k => $"{k.Key}: lift {k.Value[0]:F2} / {k.Value[2]:F2} kN, moment {k.Value[1]:F2} / {k.Value[3]:F2} kN m" +
                                                                       (Math.Abs(k.Value[2] - k.Value[0]) > 0.05 ? $" (acts {-(k.Value[3] - k.Value[1]) / (k.Value[2] - k.Value[0]):F2} m aft of the CoM)" : ""))) +
                                    " (alpha 0 / 6 deg, 110 mph, clean, elevator neutral)");
            double npAft = -(np[1].m - np[0].m) / (np[1].l - np[0].l);
            ctx.Record("reference", $"vessel.CoM {comGap:+0.000;-0.000} m ahead of the rigidbodies' CoM; at 110 mph clean, elevator neutral: lift {np[0].l:F2} / {np[1].l:F2} kN " +
                                    $"and moment {np[0].m:F2} / {np[1].m:F2} kN m at alpha 0 / 6 deg: neutral point {npAft:F3} m aft of the CoM " +
                                    $"({npAft / (58.8 * 0.0254):P0} MAC static margin), moment at zero lift {np[0].m + npAft * np[0].l:F2} kN m");
            // each wing panel at 4 deg, 60 mph, controls neutral: its angle of attack, the panels FAR takes as ahead of
            // it (chordwise, either way) and their influence, the angle it sees relative to them, its lift increment
            // from them (ClIncrementFromRear) and its Cl
            {
                double ar = 4 * Math.PI / 180, V0 = 60 * 0.44704;
                Vector3 air = (fwd * (float)Math.Cos(ar) - bodyUp * (float)Math.Sin(ar)) * (float)V0;
                var dq = new ferram4.FARCenterQuery();
                var diag = new StringBuilder();
                string Name(ferram4.FARWingAerodynamicModel w)
                {
                    Vector3 lp = p.Root.transform.InverseTransformPoint(w.part.transform.position);
                    return $"{w.part.partInfo.name.Replace("B9.Aero.Wing.Procedural.", "")}@{lp.x:F2},{lp.y:F2}";
                }
                ferram4.SimThreadContext.PredictionCall call = ferram4.SimThreadContext.BeginPrediction(air);
                try
                {
                foreach (Part q in v.parts)
                    foreach (var w in q.Modules.OfType<ferram4.FARWingAerodynamicModel>())
                    {
                        w.PrecomputeCenterOfLift(air, V0 / 340.0, 1.225, dq);
                        var wi = w.WingInteraction;
                        string Lst(string field, string infl)
                        {
                            var l = Get(wi, field) as System.Collections.IList;
                            var f = Get(wi, infl) as List<double>;
                            if (l == null || l.Count == 0) return "-";
                            return string.Join(" ", Enumerable.Range(0, l.Count).Select(i => $"{Name((ferram4.FARWingAerodynamicModel)l[i])}:{f[i]:F2}"));
                        }
                        diag.Append($"{Name(w)} AoA {w.CalculateAoA(air) * 180 / Math.PI:F2} upstreamAoA {wi.EffectiveUpstreamAoA * 180 / Math.PI:F2} infl {wi.EffectiveUpstreamInfluence:F2} " +
                                    $"ClInc {w._scratch.ClIncrementFromRear:F3} Cl {w._scratch.Cl:F3} fwd[{Lst("nearbyWingModulesForwardList", "nearbyWingModulesForwardInfluence")}] " +
                                    $"back[{Lst("nearbyWingModulesBackwardList", "nearbyWingModulesBackwardInfluence")}]; ");
                    }
                }
                finally
                {
                    ferram4.SimThreadContext.EndPrediction(call);
                }
                ctx.Record("panels at 4 deg", diag.ToString());
                // each control surface's ray geometry (FARWingInteraction's chordwise search): its start, the part's axes,
                // span, MAC, sweep and attach side, in the root frame
                var rays = new StringBuilder();
                Transform rt = p.Root.transform;
                string V3(Vector3 x) => $"({x.x:F2},{x.y:F2},{x.z:F2})";
                foreach (Part q in v.parts)
                    foreach (var w in q.Modules.OfType<ferram4.FARControllableSurface>())
                    {
                        var wi = w.WingInteraction;
                        Vector3 origin = (Vector3)Get(wi, "rootChordMidPt");
                        rays.Append($"{Name(w)}: origin {V3(rt.InverseTransformPoint(origin))} up {V3(rt.InverseTransformDirection(q.transform.up))} " +
                                    $"right {V3(rt.InverseTransformDirection(q.transform.right))} b_2 {w.b_2_actual:F3} MAC {w.MAC_actual:F3} sweep {w.MidChordSweep:F1} " +
                                    $"flip {w.srfAttachNegative} rootMid {V3(w.rootMidChordOffsetFromOrig)} nonSide {w.nonSideAttach}; ");
                    }
                ctx.Record("control rays", rays.ToString());
            }

            foreach (int flap in new[] { 0, 1 })
            {
                foreach (var c in flaps)
                    c.SetDeflection(flap);
                var trimAt = new List<string>();
                foreach (double mph in mphs)
                {
                    double V = mph * 0.44704;
                    var moments = new List<(double e, double m, double a)>();
                    foreach (float e in new[] { -0.5f, 0f, 0.5f, 1f })
                    {
                        Ctrl.Pitch = e;
                        yield return Wait.GameSeconds(2);       // the surfaces move at their own rate
                        // the angle of attack carrying the weight (lift square to the airflow, in the symmetry plane)
                        double lo = -6, hi = 16, m = double.NaN, cl = double.NaN;
                        for (int it = 0; it < 30; it++)
                        {
                            double a = (lo + hi) / 2, ar = a * Math.PI / 180;
                            Vector3 air = (fwd * (float)Math.Cos(ar) - bodyUp * (float)Math.Sin(ar)) * (float)V;
                            FARAPI.CalculateVesselAeroForces(v, out Vector3 f, out Vector3 tq, air, 0);
                            Vector3 liftDir = Vector3.Cross(air.normalized, right).normalized;
                            if (Vector3.Dot(liftDir, bodyUp) < 0) liftDir = -liftDir;
                            double lift = Vector3.Dot(f, liftDir) * 1000;
                            m = -Vector3.Dot(tq, right);
                            cl = lift / (0.5 * 1.225 * V * V * 15.86);
                            if (lift > weight) hi = a; else lo = a;
                        }
                        double alpha = (lo + hi) / 2;
                        moments.Add((e, m, alpha));
                        csv.AppendLine(string.Join(",", flap, F(mph), F(e), F(alpha), F(cl), F(m)));
                    }
                    // the elevator for zero moment, between the settings either side of it
                    double eTrim = double.NaN;
                    for (int i = 0; i + 1 < moments.Count; i++)
                        if (Math.Sign(moments[i].m) != Math.Sign(moments[i + 1].m))
                            eTrim = moments[i].e + (moments[i + 1].e - moments[i].e) * (0 - moments[i].m) / (moments[i + 1].m - moments[i].m);
                    trimAt.Add($"{mph:F0} mph: alpha {moments[1].a:F1} deg, moment at neutral {moments[1].m:F2} kN m, trims at elevator {eTrim:F2} " +
                               $"({eTrim * 28:F1} deg up)");
                }
                ctx.Record($"flaps {flap * 10}", string.Join("; ", trimAt));
            }

            // the drag at lift-off (60 mph, flaps 10) by kind: CD and CL on the 172's 174 sq ft (scratchpad c172_calib.py
            // holds it against JSBSim c172p's tables)
            {
                double V0 = ctx.ParamDouble("dragMph", 60) * 0.44704, qS = 0.5 * 1.225 * V0 * V0 * 174 * 0.092903;
                var drag = new StringBuilder();
                foreach (float e in new[] { 0f, 0.6f })
                {
                    Ctrl.Pitch = e;
                    yield return Wait.GameSeconds(2);
                    foreach (double a in new[] { 0.0, 4.0, 8.0, 10.0 })
                    {
                        double ar = a * Math.PI / 180;
                        Vector3 air = (fwd * (float)Math.Cos(ar) - bodyUp * (float)Math.Sin(ar)) * (float)V0;
                        FARAPI.CalculateVesselAeroForces(v, out Vector3 f, out Vector3 tq, air, 0);
                        Vector3 liftDir = Vector3.Cross(air.normalized, right).normalized;
                        if (Vector3.Dot(liftDir, bodyUp) < 0) liftDir = -liftDir;
                        double total = -Vector3.Dot(f, air.normalized), lift = Vector3.Dot(f, liftDir);
                        var kinds = new Dictionary<string, double>();
                        var dummy = new ferram4.FARCenterQuery();
                        double wings = 0;
                        foreach (Part q in v.parts)
                            foreach (var w in q.Modules.OfType<ferram4.FARWingAerodynamicModel>())
                            {
                                Vector3d wf = w.PrecomputeCenterOfLift(air, V0 / 340.0, 1.225, dummy);
                                Vector3 lp = p.Root.transform.InverseTransformPoint(q.transform.position);
                                bool upright = Mathf.Abs(p.Root.transform.InverseTransformDirection(q.transform.right).z) > 0.7f;
                                string kind = upright ? "fin" : lp.y < -2 ? "tail" : w is ferram4.FARControllableSurface c && (c.isFlap || c.rollaxis != 0) ? (c.isFlap ? "flaps" : "ailerons") : "wing";
                                double d = -Vector3.Dot((Vector3)wf, air.normalized);
                                kinds[kind] = (kinds.TryGetValue(kind, out double acc) ? acc : 0) + d;
                                wings += d;
                            }
                        kinds["bodies"] = total - wings;
                        double cl = lift * 1000 / qS;
                        drag.Append($"elevator {e:F1} alpha {a:F0}: CL {cl:F3} CD {total * 1000 / qS:F4} = " +
                                    string.Join(" + ", kinds.Select(k => $"{k.Key} {k.Value * 1000 / qS:F4}")) + "; ");
                    }
                }
                ctx.Record("drag", drag.ToString());
                // each lifting panel's effective aspect ratio (FAR's whole-wing grouping) at the last of those
                var ars = new StringBuilder();
                foreach (Part q in v.parts)
                    foreach (var w in q.Modules.OfType<ferram4.FARWingAerodynamicModel>())
                    {
                        Vector3 lp = p.Root.transform.InverseTransformPoint(q.transform.position);
                        ars.Append($"{q.partInfo.name} x {lp.x:F2} y {lp.y:F2}: S {w.S:F2} AR {w.EffectiveAR:F2}; ");
                    }
                ctx.Record("effective AR", ars.ToString());
            }
            Ctrl.Pitch = 0;
            Hyper(ctx, false);
            StandardDay(ctx, null);
            WaterParams.Restore();
            File.WriteAllText(Path.Combine(ctx.ResultsDir, $"trim-{FlightGlobals.ActiveVessel.vesselName}.csv"), csv.ToString());
        }

        /// <summary>
        /// The floatplane held at fixed pitch attitudes at a fixed speed on the water (full throttle, 10 deg flaps, heave
        /// free, the speed held at the tow line), to split its pitching moment: a pitch-hold torque on the fuselage (PD with
        /// an integral, averaged once steady) balances the rest, so the water's moment is minus the hold, FAR's
        /// aerodynamic moment (FARAPI at the attitude and control setting) and the thrust's. Each attitude with the
        /// elevator neutral, then full up. Parameters: mph (60), pitches (deg, + separated), weightLb. Writes captive.csv:
        /// per attitude and elevator, the moments (kN m, nose up +), the water's vertical force and drag, the wing's lift,
        /// the step keel's depth.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = Craft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Captive_C172Floatplane(TestContext ctx) => Captive(ctx);

        /// <summary>
        /// The B9 panels' drag cubes against their shape: each panel's cube (its six face areas and size) beside its
        /// prefab's, FAR's voxel areas for it, and the radiative and exposed areas KSP's thermal model was given; the
        /// stock buoyancy's displacement those cube numbers make. On the runway, the voxel model built.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = Craft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 600)]
        public static IEnumerator Measure_B9DragCubes(TestContext ctx)
        {
            yield return Wait.GameSeconds(8);
            Vessel v = FlightGlobals.ActiveVessel;
            string A(float[] a) => a == null ? "-" : string.Join("/", a.Take(6).Select(x => x.ToString("F2", CultureInfo.InvariantCulture)));
            double cubeSum = 0, farSum = 0;
            int n = 0;
            foreach (Part q in v.parts.Where(q => q.partInfo.name.StartsWith("B9.Aero.Wing")).OrderBy(q => q.mass))
            {
                var aero = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>();
                DragCube c = q.DragCubes.Cubes.FirstOrDefault();
                DragCube pc = q.partInfo.partPrefab.DragCubes.Cubes.FirstOrDefault();
                var pa = aero.ProjectedAreas;
                double far = pa.iP + pa.iN + pa.jP + pa.jN + pa.kP + pa.kN, cube = c != null ? c.Area.Take(6).Sum() : 0;
                Vector3 s = q.DragCubes.WeightedSize;
                float[] wa = q.DragCubes.WeightedArea;
                double over = 1;
                if (wa != null && s.x > 0 && s.y > 0 && s.z > 0)
                {
                    double xP = wa[0] / (s.y * s.z), yP = wa[2] / (s.x * s.z), zP = wa[4] / (s.x * s.y);
                    over = (Math.Min(xP, zP) + 2 * xP * zP) / 3 * yP;
                }
                cubeSum += cube;
                farSum += far;
                n++;
                if (n <= 12)
                    ctx.Record($"panel {n}", $"{q.partInfo.name} {q.mass * 1000:F1} kg: {q.DragCubes.Cubes.Count} cube(s) '{c?.Name}' areas {A(c?.Area)} size {c?.Size}; " +
                                             $"prefab's areas {A(pc?.Area)} size {pc?.Size}; FAR voxel areas {pa.iP:F2}/{pa.iN:F2}/{pa.jP:F2}/{pa.jN:F2}/{pa.kP:F2}/{pa.kN:F2}; " +
                                             $"thermal radiative {q.radiativeArea:F2} m2, exposed {q.exposedArea:F2}; stock displacement x{over:F0} its box " +
                                             $"({s.x * s.y * s.z * over:F2} m3 of {s.x * s.y * s.z:F3})");
            }
            ctx.Record("all panels", $"{n} panels: cube faces {cubeSum:F1} m2, FAR's voxel faces {farSum:F1} m2 ({cubeSum / Math.Max(1e-9, farSum):F1}x); " +
                                     $"exposedAreaUsesKSPHack {FerramAerospaceResearch.Settings.FARSettings.ExposedAreaUsesKSPHack}");
        }

        /// <summary>Captive_C172Floatplane with each float three parts.</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = SplitCraft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Captive_C172FloatplaneSplit(TestContext ctx) => Captive(ctx);

        private static IEnumerator Captive(TestContext ctx)
        {
            double V = ctx.ParamDouble("mph", 60) * 0.44704, weightLb = ctx.ParamDouble("weightLb", 2220);
            double[] pitches = ctx.ParamList("pitches", "4+6+8+10+12").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            Hyper(ctx, false);
            StandardDay(ctx, null);
            WaterParams.Restore();
            yield return Wait.Seconds(5);
            Plane p = null;
            foreach (object o in Prepare(ctx, r => p = r, weightLb * Lb))
                yield return o;
            Hyper(ctx, true);
            Vessel v = p.V;
            CelestialBody body = v.mainBody;
            var fixedWait = new WaitForFixedUpdate();
            yield return Wait.GameSeconds(5);
            foreach (ferram4.FARControllableSurface c in v.parts.SelectMany(q => q.Modules.OfType<ferram4.FARControllableSurface>()).Where(c => c.isFlap))
                c.SetDeflection(1);
            ModuleEngines engine = p.Engine.Modules.GetModule<ModuleEngines>();
            engine.Activate();
            Ctrl.Attach();
            Ctrl.SetThrottle(1f);
            Vector3 h = Vector3.ProjectOnPlane(p.Root.transform.up, v.upAxis).normalized;
            var csv = new StringBuilder("pitch_deg,elevator,V_ms,hold_kNm,aero_kNm,thrust_kNm,water_kNm,waterUp_kN,waterDrag_kN,wingLift_kN,weight_kN,stepKeel_m,thrust_kN," +
                                        "hydrostatic_kNm,flow_kNm,hydrostaticUp_kN,flowUp_kN\n");
            ctx.Record("start", $"{weightLb:F0} lb at {V / 0.44704:F0} mph, flaps 10, full throttle; {Ambient(v)}");

            foreach (double pitchTarget in pitches)
                foreach (float elev in new[] { 0f, 1f })
                {
                    ctx.Step($"pitch {pitchTarget} elevator {elev}");
                    double integral = 0, lastPitch = double.NaN, t0 = Planetarium.GetUniversalTime(), lastRoll = 0, lastHdg = 0;
                    var hold = new List<double>(); var aero = new List<double>(); var thrustM = new List<double>(); var wUp = new List<double>();
                    var wDrag = new List<double>(); var lift = new List<double>(); var keel = new List<double>(); var thr = new List<double>(); var speed = new List<double>();
                    // the water's moment about the CoM (nose up +) and its lift, split: its pressure (hydrostatic) and its flow
                    var hsM = new List<double>(); var flowM = new List<double>(); var hsUp = new List<double>(); var flowUp = new List<double>();
                    Vector3 h0 = h;
                    int n = 0;
                    while (Planetarium.GetUniversalTime() - t0 < 10)
                    {
                        double t = Planetarium.GetUniversalTime() - t0, dt = TimeWarp.fixedDeltaTime;
                        Vector3 up = v.upAxis;
                        h = Vector3.ProjectOnPlane(h, up).normalized;
                        Vector3 fwd = p.Root.transform.up, right = p.Root.transform.right;
                        double pitch = Math.Asin(Mathf.Clamp(Vector3.Dot(fwd, up), -1, 1)) * 180 / Math.PI;
                        double roll = Math.Asin(Mathf.Clamp(Vector3.Dot(right, up), -1, 1)) * 180 / Math.PI;
                        double hdg = Vector3.SignedAngle(Vector3.ProjectOnPlane(h0, up), Vector3.ProjectOnPlane(fwd, up), up);
                        double q = double.IsNaN(lastPitch) ? 0 : (pitch - lastPitch) / dt;
                        double pr = (roll - lastRoll) / dt, r = (hdg - lastHdg) / dt;
                        lastPitch = pitch; lastRoll = roll; lastHdg = hdg;

                        // the speed held along the tow line on every part, the heave free
                        // (a physicsless part, the drain valve, has no rigidbody: its parent carries it)
                        foreach (Part part in v.parts.Where(q => q.rb != null))
                            part.rb.velocity += h * (float)(V - Vector3.Dot(part.rb.velocity, h));

                        // the pitch hold: kN m nose up, about the root's right axis (a positive torque about +right puts the nose down)
                        double err = pitchTarget - pitch;
                        integral = Math.Max(-20, Math.Min(20, integral + err * dt));
                        double torque = 0.6 * err + 0.4 * integral - 0.25 * q;
                        p.Root.rb.AddTorque(-right * (float)torque);
                        Ctrl.Pitch = elev;
                        Ctrl.Roll = Mathf.Clamp((float)(0.08 * roll + 0.03 * pr), -1, 1);
                        Ctrl.Yaw = Mathf.Clamp((float)(-0.10 * hdg - 0.05 * r), -1, 1);

                        if (t > 6 && ++n % 5 == 0)
                        {
                            Vector3 com = WorldCoM(v);
                            FARAPI.CalculateVesselAeroForces(v, out Vector3 af, out Vector3 at, (Vector3)v.srf_velocity, Math.Max(0, v.altitude));
                            hold.Add(torque);
                            aero.Add(-Vector3.Dot(at, right));
                            lift.Add(Vector3.Dot(af, up));
                            // the thrust along -thrustTransform.forward at the transform (KSP's engines)
                            Transform tt = engine.thrustTransforms[0];
                            double thrust = p.Get("propThrust");
                            Vector3 tf = -tt.forward * (float)thrust;
                            thrustM.Add(-Vector3.Dot(Vector3.Cross(tt.position - com, tf), right));
                            thr.Add(thrust);
                            Vector3 water = Vector3.zero;
                            foreach (HullParts f in p.Floats)
                                water += f.Sum(x => x.LastForce + x.LastHydrostaticForce);
                            wUp.Add(Vector3.Dot(water, up));
                            Vector3 mh = Vector3.zero, mf = Vector3.zero, fh = Vector3.zero, ff = Vector3.zero;
                            foreach (HullParts f in p.Floats)
                                foreach (FARAeroComponents.PartHydrodynamics x in f.Hydro)
                                {
                                    Vector3 r0 = x.part.rb.worldCenterOfMass - com;
                                    mh += Vector3.Cross(r0, x.LastHydrostaticForce) + x.LastHydrostaticTorque;
                                    mf += Vector3.Cross(r0, x.LastForce) + x.LastTorque;
                                    fh += x.LastHydrostaticForce;
                                    ff += x.LastForce;
                                }
                            hsM.Add(-Vector3.Dot(mh, right)); flowM.Add(-Vector3.Dot(mf, right));
                            hsUp.Add(Vector3.Dot(fh, up)); flowUp.Add(Vector3.Dot(ff, up));
                            wDrag.Add(-Vector3.Dot(water, h));
                            keel.Add(-p.Floats.Average(f => FlightGlobals.getAltitudeAtPos(f.TransformPoint(StepKeel), body)));
                            speed.Add(Vector3.Dot(v.srf_velocity, h));
                        }
                        yield return fixedWait;
                    }
                    double weight = v.GetTotalMass() * 9.80665 * body.GeeASL;
                    // steady: hold + aero + thrust + water = 0
                    double water0 = -(hold.Average() + aero.Average() + thrustM.Average());
                    csv.AppendLine(string.Join(",", F(pitchTarget), F(elev), F(speed.Average()), F(hold.Average()), F(aero.Average()), F(thrustM.Average()), F(water0),
                                               F(wUp.Average()), F(wDrag.Average()), F(lift.Average()), F(weight), F(keel.Average()), F(thr.Average()),
                                               F(hsM.Average()), F(flowM.Average()), F(hsUp.Average()), F(flowUp.Average())));
                    ctx.Record($"pitch {pitchTarget} elevator {elev}", $"hold {hold.Average():F2} kN m (sd {Sd(hold):F2}), aero {aero.Average():F2}, thrust {thrustM.Average():F2}, " +
                                                                    $"water {water0:F2}; water up {wUp.Average():F2} kN drag {wDrag.Average():F2}, wing lift {lift.Average():F2} of {weight:F2}, step keel {keel.Average():F3} m deep");
                }
            Hyper(ctx, false);
            StandardDay(ctx, null);
            WaterParams.Restore();
            File.WriteAllText(Path.Combine(ctx.ResultsDir, $"captive-{FlightGlobals.ActiveVessel.vesselName}.csv"), csv.ToString());
            Ctrl.SetThrottle(0f);
            Ctrl.Neutral();
        }

        private static bool Finite(Vector3 w) => !float.IsNaN(w.x + w.y + w.z) && !float.IsInfinity(w.x + w.y + w.z);

        private static double Sd(List<double> x)
        {
            if (x.Count < 2) return double.NaN;
            double m = x.Average();
            return Math.Sqrt(x.Sum(y => (y - m) * (y - m)) / (x.Count - 1));
        }

        /// <summary>
        /// A water takeoff as the Cessna floatplane supplement flies it (10 deg flaps, full throttle; the yoke full aft until
        /// over the hump, then onto the step at a planing attitude; off the water at 50-60 mph; the climb at the table's
        /// 50 ft speed) at a weight (weightLb: 2220, 1950, 1700), and its water run, time and distance to 50 ft against the
        /// supplement's sea-level, standard-day, no-wind figures. The ailerons hold the wings level and the rudder the
        /// heading. Parameters: weightLb, stepPitch (deg, the planing attitude held), rotateMph (IAS to raise the nose
        /// off the water), liftoffPitch (deg, the attitude then held until it flies), climbMph, hyperWarp, standardDay.
        /// Writes takeoff.csv.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = Craft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Takeoff_C172Floatplane(TestContext ctx) => Takeoff(ctx);

        /// <summary>Takeoff_C172Floatplane with each float three parts.</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = SplitCraft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Takeoff_C172FloatplaneSplit(TestContext ctx) => Takeoff(ctx);


        private static IEnumerator Takeoff(TestContext ctx)
        {
            double weightLb = ctx.ParamDouble("weightLb", 2220);
            // supplement Fig. 4-3, sea level, 59 F, no wind: water run, to clear 50 ft (ft), IAS at 50 ft (mph)
            var table = new Dictionary<int, (double run, double obstacle, double ias50)>
                { [2220] = (1620, 2390, 64), [1950] = (1135, 1715, 60), [1700] = (805, 1260, 58) };
            (double run, double obstacle, double ias50) book = table.TryGetValue((int)Math.Round(weightLb), out var b) ? b : (double.NaN, double.NaN, 64);
            double stepPitch = ctx.ParamDouble("stepPitch", 6), rotateMs = ctx.ParamDouble("rotateMph", 55) * 0.44704;
            // on the step the yoke eased from full aft to stepElevator over 3 s and held there (no feedback: the floats
            // find their own planing attitude, as a pilot lets them)
            double stepElevator = ctx.ParamDouble("stepElevator", 0), stepSince = double.NaN;
            // humpElevator (-1..1): the stick held there from the start, in place of full aft over the hump (as a pilot who
            // leaves it neutral); flaps: FAR's flap setting (0-3, 10 deg each; the supplement's 10 deg by default)
            double humpElevator = ctx.ParamDouble("humpElevator", double.NaN);
            int flapSetting = (int)ctx.ParamDouble("flaps", 1);
            double climbMs = ctx.ParamDouble("climbMph", book.ias50) * 0.44704;
            // the attitude held from rotateMph until it flies: under FAR's stall (flaps 10: ~12 deg) and the floats'
            // heels (~10 deg); full aft stick there pulls it to 11-12 deg, onto the heels, and it plows
            double liftoffPitch = ctx.ParamDouble("liftoffPitch", 9);
            const double Mph = 0.44704, Ft = 0.3048;

            Hyper(ctx, false);
            StandardDay(ctx, null);
            WaterParams.Restore();
            yield return Wait.Seconds(5);
            Plane p = null;
            foreach (object o in Prepare(ctx, r => p = r, weightLb * Lb))
                yield return o;
            Hyper(ctx, true);
            Vessel v = p.V;
            CelestialBody body = v.mainBody;
            var fixedWait = new WaitForFixedUpdate();
            yield return Wait.GameSeconds(10);     // settle afloat

            foreach (ferram4.FARControllableSurface c in v.parts.SelectMany(q => q.Modules.OfType<ferram4.FARControllableSurface>()).Where(c => c.isFlap))
                c.SetDeflection(flapSetting);      // 10 deg each: FAR's three settings of the 30 deg travel
            p.Engine.Modules.GetModule<ModuleEngines>().Activate();
            Ctrl.Attach();
            Ctrl.SetThrottle(1f);
            ctx.Record("start", $"{weightLb:F0} lb, flaps {flapSetting * 10}" +
                                (double.IsNaN(humpElevator) ? "" : $", stick {humpElevator:F2} until the step") + $", {Ambient(v)}");

            Vector3 up = v.upAxis;
            Vector3 h0 = Vector3.ProjectOnPlane(p.Root.transform.up, up).normalized;
            var csv = new StringBuilder("t,x_m,V_ms,IAS_mph,pitch_deg,roll_deg,heading_deg,keel_alt_m,wet,phase,elevator,rpm,thrust_lbf,float_rbDrag,aero_moment_kNm,pitch_rate,aero_lift_kN,aero_drag_kN,water_up_kN,water_drag_kN\n");
            string phase = "hump", lastPhase = "hump";
            double aeroMoment = double.NaN, aeroLift = double.NaN, aeroDrag = double.NaN;
            double t0 = Planetarium.GetUniversalTime(), peak = -90, lastPitch = 0, lastRoll = 0, lastHdg = 0, x = 0, integral = 0;
            int steps = 0;
            double runX = double.NaN, runT = double.NaN, obstacleX = double.NaN, obstacleT = double.NaN, airborneSince = double.NaN;
            // floats of several parts: how far their joints bend them (rigidFloats=true: put back in shape every step)
            bool rigidFloats = ctx.Param("rigidFloats") == "true";
            double bendDeg = 0, bendM = 0;
            while (Planetarium.GetUniversalTime() - t0 < 150 && double.IsNaN(obstacleX) && !(phase == "hump" && Planetarium.GetUniversalTime() - t0 > 40))
            {
                double t = Planetarium.GetUniversalTime() - t0, dt = TimeWarp.fixedDeltaTime;
                // the vessel's state gone NaN: the run ends on a record, not the game's crash a few frames later (a NaN
                // stick did it: FAR deflects the control surfaces by it, and their colliders take the rotation)
                if (!Finite(p.Root.rb.velocity) || !Finite(p.Root.rb.position))
                {
                    ctx.Record("nan", $"t {t:F2} s: the vessel's state NaN ({v.situation}, orbit inclination {v.orbit.inclination})");
                    Assert.True(false, "the vessel's state is finite");
                }
                foreach (HullParts f in p.Floats.Where(f => f.Parts.Length > 1))
                {
                    (double d, double m) fb = f.Bend();
                    bendDeg = Math.Max(bendDeg, fb.d);
                    bendM = Math.Max(bendM, fb.m);
                    if (rigidFloats)
                        f.Snap();
                }
                up = v.upAxis;
                Vector3 fwd = p.Root.transform.up, right = p.Root.transform.right;
                double pitch = Math.Asin(Mathf.Clamp(Vector3.Dot(fwd, up), -1, 1)) * 180 / Math.PI;
                double roll = Math.Asin(Mathf.Clamp(Vector3.Dot(right, up), -1, 1)) * 180 / Math.PI;     // right wing up +
                double hdg = Vector3.SignedAngle(Vector3.ProjectOnPlane(h0, up), Vector3.ProjectOnPlane(fwd, up), up); // nose right +
                double q = (pitch - lastPitch) / dt, pr = (roll - lastRoll) / dt, r = (hdg - lastHdg) / dt;
                lastPitch = pitch; lastRoll = roll; lastHdg = hdg;
                double V = v.srfSpeed, ias = v.indicatedAirSpeed;
                x += Vector3.ProjectOnPlane(v.srf_velocity, up).magnitude * dt;     // the run over the surface (the floating origin moves)
                bool wet = p.Floats.Any(f => f.Parts.Any(q => q.submergedPortion > 0));
                double keelAlt = p.Floats.Min(f => FlightGlobals.getAltitudeAtPos(f.TransformPoint(StepKeel), body));

                // the pilot
                double target;
                switch (phase)
                {
                    case "hump":
                        peak = Math.Max(peak, pitch);
                        target = 90;                                       // the yoke full aft
                        if (V > 6 && (pitch < peak - 0.5 || V > 10))
                            phase = "step";                                // the bow wave aft: onto the step
                        break;
                    case "step":
                        target = double.NaN;                               // the stick held, not an attitude
                        if (double.IsNaN(stepSince))
                            stepSince = t;
                        if (ias >= rotateMs)
                            phase = "rotate";
                        break;
                    case "rotate":
                        target = liftoffPitch;                             // raised to the lift-off attitude, held until it flies
                        break;
                    default:                                               // climbing at the 50 ft speed
                        target = Mathf.Clamp((float)(stepPitch + 3 + 1.0 * (ias - climbMs)), 0, 15);
                        break;
                }
                // attitude hold: PI on the pitch error with rate damping (the integral held while the stick is at a stop)
                double err = Math.Max(-10, Math.Min(10, target - pitch));
                if (phase != lastPhase)
                    integral = 0;
                lastPhase = phase;
                double elevator;
                // (the hump case moves on to the step before the step case sets stepSince: until then, no ramp; a NaN stick
                // here crashed the game)
                double ramp = double.IsNaN(stepSince) ? 0 : Math.Min(1, (t - stepSince) / 3);
                if (!double.IsNaN(humpElevator) && (phase == "hump" || phase == "step"))
                    elevator = phase == "hump" ? humpElevator : humpElevator + (stepElevator - humpElevator) * ramp;
                else if (double.IsNaN(target))
                    elevator = 1 + (stepElevator - 1) * ramp;
                else
                {
                    elevator = 0.15 * err + 0.03 * integral - 0.06 * q;
                    if (Math.Abs(elevator) < 1 || Math.Sign(err) != Math.Sign(elevator))
                        integral = Math.Max(-30, Math.Min(30, integral + err * dt));
                }
                elevator = Mathf.Clamp((float)elevator, -1, 1);
                // (the stick set directly: Ctrl.Set logs every call)
                Ctrl.Pitch = (float)elevator;
                Ctrl.Roll = Mathf.Clamp((float)(0.08 * roll + 0.03 * pr), -1, 1);
                Ctrl.Yaw = Mathf.Clamp((float)(-0.10 * hdg - 0.05 * r), -1, 1);

                // off the water: every float clear for a second; then 50 ft
                if (!wet && phase != "hump")
                {
                    if (double.IsNaN(airborneSince))
                        airborneSince = t;
                    if (double.IsNaN(runX) && t - airborneSince > 1)
                    {
                        runX = x; runT = t;
                        phase = "climb";
                    }
                }
                else if (double.IsNaN(runX))
                    airborneSince = double.NaN;
                if (!double.IsNaN(runX) && keelAlt > 50 * Ft)
                {
                    obstacleX = x; obstacleT = t;
                }
                // FAR's aerodynamic pitching moment about the CG (kN m, nose up +) at this attitude, speed and control setting
                if (++steps % 10 == 0)
                {
                    FARAPI.CalculateVesselAeroForces(v, out Vector3 af, out Vector3 at, (Vector3)v.srf_velocity, Math.Max(0, v.altitude));
                    aeroMoment = -Vector3.Dot(at, right);
                    aeroDrag = -Vector3.Dot(af, ((Vector3)v.srf_velocity).normalized);
                    aeroLift = Vector3.Dot(af, up);
                }
                // the water's force on the floats: up, and its drag along the run
                Vector3 water = Vector3.zero;
                foreach (HullParts f in p.Floats)
                    water += f.Sum(x => x.LastForce + x.LastHydrostaticForce);
                Vector3 runDir = Vector3.ProjectOnPlane(v.srf_velocity, up).normalized;
                csv.AppendLine(string.Join(",", F(t), F(x), F(V), F(ias / Mph), F(pitch), F(roll), F(hdg), F(keelAlt), wet ? "1" : "0", phase, F(elevator),
                                           F(p.Get("propRPM")), F(p.Get("propThrust") / 4.448e-3), F(p.Floats.Max(f => f.Ref.rb.drag)), F(aeroMoment), F(q),
                                           F(aeroLift), F(aeroDrag), F(Vector3.Dot(water, up)), F(-Vector3.Dot(water, runDir))));
                yield return fixedWait;
            }
            Hyper(ctx, false);
            StandardDay(ctx, null);
            WaterParams.Restore();
            File.WriteAllText(Path.Combine(ctx.ResultsDir, $"takeoff-{FlightGlobals.ActiveVessel.vesselName}.csv"), csv.ToString());
            Ctrl.SetThrottle(0f);
            Ctrl.Neutral();
            if (p.Floats.Any(f => f.Parts.Length > 1))
                ctx.Record("float bend", $"the joints bent the floats up to {bendDeg:F3} deg, {bendM * 1000:F1} mm" + (rigidFloats ? " in a step (put back every step)" : ""));
            ctx.Record("takeoff", $"{weightLb:F0} lb: water run {runX / Ft:F0} ft in {runT:F1} s (supplement {book.run:F0} ft), to 50 ft {obstacleX / Ft:F0} ft " +
                                  $"in {obstacleT:F1} s (supplement {book.obstacle:F0} ft at {book.ias50:F0} mph); peak hump pitch {peak:F1} deg");
            Assert.True(!double.IsNaN(runX), "off the water");
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);

        /// <summary>
        /// A video of the hinged craft as it loads (frames/NNNNN.jpg, one per 1/30 s of game time, and demo.csv): the
        /// fuselage full of LFO (crafts/c172_floatplane.py FUSELAGE_TANK), 2618 lb, 398 over the gross and the C.G. aft.
        /// Full throttle on the water with the stick at overStick: it cannot get over the hump. Then the drain valve
        /// dumps the fuselage's fuel (2220 lb, C.G. 28.4% MAC) and the stick goes neutral: over the hump and onto the
        /// step, where the water's drag holds the speed; then full aft stick lifts the bows and it flies. The flight camera
        /// abeam (camAngle deg aft of abeam), the UI hidden, an overlay with weight, C.G., speed, thrust and drag.
        /// Parameters: overStick (-1..1, default 0), overSec (default 30), overLb (the fuselage's fuel, default 398: the
        /// stock craft's full fuselage), pullMph (the IAS at which the stick goes full aft, default 70; 0: at neutral
        /// until the speed stops rising, at most neutralSec, default 60), lat and lon (the site; RSS's Earth: off Cape
        /// Canaveral),
        /// flaps (default 0), record (false: no frames, hyper-warped), camDistance, camPitch, camAngle, hyperWarp,
        /// standardDay, linearFlapLift, hydro (off: FAR's water model off, the water as thick air; default on).
        /// Recording, hyperWarp given hyper-warps it too: a time-lapse.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.DemoMods, StartsAt = StartState.Flight, Craft = HingedCraft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Demo_C172FloatplaneHinged(TestContext ctx) => Demo(ctx);

        /// <summary>
        /// The hinged craft in rough Scatterer waves (TestWarp.RoughMods: wind 5, amplitude 2, as a player saved it), rolled
        /// until a wing's tip is in the water: full throttle from rest, and after taxiSec (default 12 s) a roll torque on the
        /// fuselage (at most torqueKnm, default 30 kN m) holds rollDeg (default 25 deg, the right wing down) for holdSec
        /// (default 15 s), then 10 s more free. Crash damage on. Records the waves under the floats, when a wing panel first
        /// got wet and the deepest any did, the largest force a wing panel's model put out, the fuselage's largest
        /// acceleration (g, gravity out), the parts lost, and the water model's cost (ms per step; its shadow rebuilds, the
        /// slowest); wingdip.csv per step. Parameters: hydro (on/off, default on), hostAny (false: a foil hands its force
        /// only to a hull's part, as before), rollDeg, taxiSec, holdSec, torqueKnm. Asserts, hydro on, that no part is lost.
        /// Needs a GPU: run it without batch mode.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.RoughMods, StartsAt = StartState.Flight, Craft = HingedCraft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Waves_WingDip(TestContext ctx)
        {
            bool hydroWas = FARDebugValues.hydrodynamics, hostWas = FARAeroComponents.PartHydrodynamics.FoilHostAnyBody;
            bool hydro = ctx.Param("hydro") != "off";
            double rollDeg = ctx.ParamDouble("rollDeg", 25), taxiSec = ctx.ParamDouble("taxiSec", 12), holdSec = ctx.ParamDouble("holdSec", 15);
            double torqueMax = ctx.ParamDouble("torqueKnm", 30);
            Hyper(ctx, false);
            StandardDay(ctx, null);
            WaterParams.Restore();
            yield return Wait.Seconds(5);
            Plane p = null;
            foreach (object o in Prepare(ctx, r => p = r))
                yield return o;
            Vessel v = p.V;
            CelestialBody body = v.mainBody;
            bool crashWas = CheatOptions.NoCrashDamage;
            var csv = new StringBuilder("t,phase,V_ms,roll_deg,torque_knm,wing_sub_max,wing_force_max_kn,accel_g,parts,wave_lo,wave_hi,hydro_ms\n");
            try
            {
                FARDebugValues.hydrodynamics = hydro;
                FARAeroComponents.PartHydrodynamics.FoilHostAnyBody = ctx.Param("hostAny") != "false";
                CheatOptions.NoCrashDamage = false;
                ctx.Record("setup", $"hydrodynamics {hydro}, foil host any body {FARAeroComponents.PartHydrodynamics.FoilHostAnyBody}, " +
                                    $"roll {rollDeg:F0} deg from {taxiSec:F0} s for {holdSec:F0} s, torque up to {torqueMax:F0} kN m");
                yield return Wait.GameSeconds(3);
                Part root = p.Root;
                var floats = p.Floats.SelectMany(f => f.Parts).ToList();
                // the wing panels (and tail): FAR surfaces not on a float (the water rudders are, always in the water)
                bool OnFloat(Part q)
                {
                    for (Part a = q.parent; a != null; a = a.parent)
                        if (floats.Contains(a))
                            return true;
                    return false;
                }
                var allFoils = v.parts.Where(q => q.Modules.Contains<ferram4.FARWingAerodynamicModel>()).ToList();
                var wings = allFoils.Where(q => !OnFloat(q)).ToList();
                int parts0 = v.parts.Count;
                ctx.Record("wings", $"{wings.Count} wing and tail panels, {allFoils.Count - wings.Count} on the floats; {parts0} parts");
                var spikes = new List<string>();
                var partVelWas = new Dictionary<Part, Vector3>();
                FARAeroComponents.PartHydrodynamics.ResetProfile();
                p.Engine.Modules.GetModule<ModuleEngines>().Activate();
                Ctrl.Attach();
                Ctrl.SetThrottle(1f);
                var fixedWait = new WaitForFixedUpdate();
                double t0 = Planetarium.GetUniversalTime(), end = taxiSec + holdSec + 10;
                double dipT = double.NaN, subMax = 0, forceMax = 0, accelMax = 0, accelT = 0, waveLo = 0, waveHi = 0, worstStepMs = 0;
                string subPart = "", forcePart = "";
                // stock's buoyancy on a FAR surface in the water (wings and the water rudders), the most against its weight
                double buoyMax = 0;
                string buoyWhat = "none";
                string lost = null;
                double lostT = double.NaN;
                Vector3 lastVel = root.rb.velocity;
                long hTicks = 0;
                double tick = 1000.0 / System.Diagnostics.Stopwatch.Frequency;
                while (Planetarium.GetUniversalTime() - t0 < end && root != null && root.rb != null)
                {
                    double t = Planetarium.GetUniversalTime() - t0, dt = TimeWarp.fixedDeltaTime;
                    Vessel cur = root.vessel;
                    Vector3 up = (Vector3)FlightGlobals.getUpAxis(body, root.rb.worldCenterOfMass);
                    Vector3 fwd = root.transform.up, right = root.transform.right;
                    // bank: the right wing's rise (sin of the roll); its rate from the angular velocity
                    double s = Vector3.Dot(right, up);
                    double sDot = Vector3.Dot(Vector3.Cross(root.rb.angularVelocity, right), up);
                    // a torque about the nose's axis turns the right wing up (g > 0) or down
                    double g = Math.Sign(Vector3.Dot(Vector3.Cross(fwd, right), up));
                    string phase = t < taxiSec ? "taxi" : t < taxiSec + holdSec ? "roll" : "free";
                    double torque = 0;
                    if (phase == "roll" && g != 0)
                    {
                        double target = -Math.Sin(rollDeg * Math.PI / 180);
                        torque = Math.Max(-torqueMax, Math.Min(torqueMax, g * (100 * (target - s) - 30 * sDot)));
                        root.rb.AddTorque(fwd * (float)torque);
                    }
                    // the wing panels: in the water, the force their model put out
                    double subNow = 0, forceNow = 0;
                    foreach (Part q in wings)
                    {
                        if (q == null || q.vessel != cur)
                            continue;
                        var w = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                        double sub = q.submergedPortion, f = ((Vector3)w.worldSpaceForce).magnitude;
                        subNow = Math.Max(subNow, sub);
                        if (sub > 0 && double.IsNaN(dipT))
                            dipT = t;
                        if (sub > subMax) { subMax = sub; subPart = $"{q.partInfo.name} at {t:F1} s"; }
                        if (sub > 0 && f > forceMax) { forceMax = f; forcePart = $"{q.partInfo.name} at {t:F1} s ({sub:P0} in)"; }
                        if (sub > 0) forceNow = Math.Max(forceNow, f);
                    }
                    foreach (Part q in allFoils)
                    {
                        if (q == null || q.vessel != cur || q.submergedPortion <= 0 || q.partBuoyancy == null || q.rb == null)
                            continue;
                        double bf = q.partBuoyancy.lastBuoyantForce.magnitude, ratio = bf / (q.rb.mass * 9.81);
                        if (ratio > buoyMax)
                        {
                            buoyMax = ratio;
                            buoyWhat = $"{q.partInfo.name}{(OnFloat(q) ? " (float)" : "")} {q.rb.mass * 1000:F1} kg at {t:F1} s, {q.submergedPortion:P0} in: " +
                                       $"{bf:F2} kN ({ratio:F0} times its weight); displacement {q.partBuoyancy.displacement:F3} m3, buoyancy {q.buoyancy:F2}, " +
                                       $"drag cubes {(q.DragCubes.None ? "none" : "")} size {q.DragCubes.WeightedSize}";
                        }
                    }
                    // the fuselage's acceleration, gravity out (g)
                    Vector3 vel = root.rb.velocity;
                    double acc = ((vel - lastVel) / (float)dt - (Vector3)FlightGlobals.getGeeForceAtPosition(root.rb.worldCenterOfMass)).magnitude / 9.81;
                    lastVel = vel;
                    if (t > 0.5 && acc > accelMax) { accelMax = acc; accelT = t; }
                    // a jolt: the parts' forces that step, the largest (their models' and the water's), and what was in the water
                    if (t > 0.5 && acc > 3 && spikes.Count < 12)
                    {
                        // each part's momentum change over the step (t m/s = kN s), the largest first, with the forces its
                        // models put out and its rigidbody's drag (stock's water drag on it)
                        var top = cur.parts.Where(q => q.rb != null).Select(q =>
                        {
                            var w = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                            var hd = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                            Vector3 wf = w != null ? (Vector3)w.worldSpaceForce : Vector3.zero;
                            Vector3 hf = hd != null && q.submergedPortion > 0 ? hd.LastForce + hd.LastHydrostaticForce : Vector3.zero;
                            Vector3 was = partVelWas.TryGetValue(q, out Vector3 pv) ? pv : q.rb.velocity;
                            double imp = (q.rb.velocity - was).magnitude * q.rb.mass;
                            return (q, wf, hf, imp, dv: (q.rb.velocity - was).magnitude);
                        }).OrderByDescending(x => x.imp).Take(5);
                        spikes.Add($"{t:F2} s {acc:F1} g: " + string.Join("; ", top.Select(x =>
                            $"{x.q.partInfo.name}{(OnFloat(x.q) ? " (float)" : "")} {x.q.rb.mass * 1000:F0} kg in {x.q.submergedPortion:P0}: " +
                            $"dv {x.dv:F2} m/s ({x.imp:F3} kN s), drag {x.q.rb.drag:F2}/{x.q.rb.angularDrag:F2}, wing {x.wf.magnitude:F1} kN, water {x.hf.magnitude:F1} kN")));
                    }
                    foreach (Part q in cur.parts.Where(q => q.rb != null))
                        partVelWas[q] = q.rb.velocity;
                    foreach (Part q in floats.Where(q => q != null && q.partBuoyancy != null))
                    {
                        waveLo = Math.Min(waveLo, q.partBuoyancy.waterLevel);
                        waveHi = Math.Max(waveHi, q.partBuoyancy.waterLevel);
                    }
                    if (lost == null && wings.Concat(floats).Any(q => q == null || q.vessel != cur))
                    {
                        lostT = t;
                        lost = string.Join(", ", wings.Concat(floats).Where(q => q == null || q.vessel != cur).Select(q => q == null ? "(destroyed)" : q.partInfo.name).Take(8));
                    }
                    long h = FARAeroComponents.PartHydrodynamics.ProfileGatherTicks + FARAeroComponents.PartHydrodynamics.ProfileSolveTicks;
                    double stepMs = (h - hTicks) * tick;
                    hTicks = h;
                    worstStepMs = Math.Max(worstStepMs, stepMs);
                    csv.AppendLine(string.Join(",", F(t), phase, F(cur.srf_velocity.magnitude), F(Math.Asin(Math.Max(-1, Math.Min(1, s))) * 180 / Math.PI),
                                               F(torque), F(subNow), F(forceNow), F(acc), cur.parts.Count.ToString(), F(waveLo), F(waveHi), F(stepMs)));
                    yield return fixedWait;
                }
                Ctrl.SetThrottle(0f);
                Ctrl.Neutral();
                int partsNow = root != null && root.vessel != null ? root.vessel.parts.Count : 0;
                ctx.Record("waves", $"{waveLo:+0.00;-0.00} to {waveHi:+0.00;-0.00} m under the floats");
                ctx.Record("wing in the water", double.IsNaN(dipT) ? "never" : $"first at {dipT:F1} s; deepest {subMax:P0} ({subPart}); largest force in it {forceMax:F1} kN ({forcePart})");
                ctx.Record("fuselage", $"largest acceleration {accelMax:F1} g at {accelT:F1} s");
                ctx.Record("stock buoyancy on a surface", buoyWhat);
                ctx.Record("parts", lost == null ? $"none lost ({partsNow} of {parts0})" : $"lost from {lostT:F1} s: {lost}; {partsNow} of {parts0} left");
                long steps = Math.Max(1, FARAeroComponents.PartHydrodynamics.ProfileSteps);
                for (int i = 0; i < spikes.Count; i++)
                    ctx.Record($"jolt {i + 1}", spikes[i]);
                ctx.Record("water model parts", $"per step: gather {FARAeroComponents.PartHydrodynamics.ProfileGatherTicks * tick / steps:F2} ms, " +
                                                $"solve {FARAeroComponents.PartHydrodynamics.ProfileSolveTicks * tick / steps:F2} ms (of it lee " +
                                                $"{FARAeroComponents.PartHydrodynamics.ProfileLeeTicks * tick / steps:F2} ms, {FARAeroComponents.PartHydrodynamics.ProfileLeeRays / steps} rays; " +
                                                $"pieces {FARAeroComponents.PartHydrodynamics.ProfilePieceTicks * tick / steps:F2} ms, " +
                                                $"strips and impact {FARAeroComponents.PartHydrodynamics.ProfileImpactTicks * tick / steps:F2} ms), " +
                                                $"contacts {FARAeroComponents.PartHydrodynamics.ProfileContactTicks * tick / steps:F2} ms, mesh {FARAeroComponents.PartHydrodynamics.ProfileMeshTicks * tick / steps:F2} ms; " +
                                                $"{FARAeroComponents.PartHydrodynamics.ProfilePolygons / steps} polygons, {FARAeroComponents.PartHydrodynamics.ProfileTrianglesTested / steps} triangles tested; " +
                                                $"{steps} steps");
                ctx.Record("water model cost", $"{(FARAeroComponents.PartHydrodynamics.ProfileGatherTicks + FARAeroComponents.PartHydrodynamics.ProfileSolveTicks) * tick / steps:F2} ms per step " +
                                               $"(worst {worstStepMs:F1}); {FARAeroComponents.PartHydrodynamics.ProfileShadowBuilds} shadow rebuilds, " +
                                               $"{FARAeroComponents.PartHydrodynamics.ProfileShadowTicks * tick:F0} ms all, slowest {FARAeroComponents.PartHydrodynamics.ProfileShadowMaxTicks * tick:F0} ms");
                Assert.True(!hydro || double.IsNaN(dipT) || lost == null, $"no part lost with a wing in the water ({lost})");
                Assert.True(!double.IsNaN(dipT), "a wing's tip got into the water");
            }
            finally
            {
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "wingdip.csv"), csv.ToString());
                FARDebugValues.hydrodynamics = hydroWas;
                FARAeroComponents.PartHydrodynamics.FoilHostAnyBody = hostWas;
                CheatOptions.NoCrashDamage = crashWas;
                Ctrl.SetThrottle(0f);
                StandardDay(ctx, null);
                WaterParams.Restore();
            }
        }

        /// <summary>
        /// Demo_C172FloatplaneHinged on RO's parts, in a RealismOverhaul / RP-1 install (KSPDIR). It starts at the space
        /// center: RP-1's KSCSwitcher binding is mended there (Rp1KscRebind) before the editor and the launch.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.SpaceCenter, Craft = HingedRoCraft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 3600)]
        public static IEnumerator Demo_C172FloatplaneHingedRO(TestContext ctx)
        {
            yield return Rp1KscRebind(ctx);
            // (ctx.Craft is made only for a test that starts in the editor or flight)
            string craft = Path.Combine(KSPUtil.ApplicationRootPath, "saves", HighLogic.SaveFolder, "Ships", "SPH", HingedRoCraft + ".craft");
            Assert.True(File.Exists(craft), $"the craft in the save ({craft})");
            // daylight at the site (a new game starts at night off Cape Canaveral): the clock moved on to when the sun
            // stands highest there, tried hour by hour over a day from the space center
            if (FlightGlobals.GetHomeBody() is CelestialBody home && home.Radius > 1e6 && ctx.Param("noon") != "false")
            {
                double ut0 = Planetarium.GetUniversalTime(), bestUt = ut0, bestEl = -90;
                double lat = ctx.ParamDouble("lat", 28.45), lon = ctx.ParamDouble("lon", -80.45);
                for (int h = 0; h < 48; h++)
                {
                    Planetarium.SetUniversalTime(ut0 + h * home.solarDayLength / 48);
                    yield return Wait.Frames(2);
                    Vector3d upAt = home.GetSurfaceNVector(lat, lon);
                    Vector3d sun = (Planetarium.fetch.Sun.position - home.position).normalized;
                    double el = 90 - Vector3d.Angle(upAt, sun);
                    if (el > bestEl)
                    {
                        bestEl = el;
                        bestUt = Planetarium.GetUniversalTime();
                    }
                }
                Planetarium.SetUniversalTime(bestUt);
                yield return Wait.Frames(2);
                ctx.Record("time of day", $"UT {bestUt:F0} s: the sun {bestEl:F0} deg up at {lat:F2} N {lon:F2} E");
            }
            yield return Nav.EnsureState(ctx, StartState.Flight, craft, ctx.LaunchSite ?? "Runway");
            // Scatterer's ocean pushes craft with its waves (oceanCraftWaveInteractions) where FAR's water is a flat sea:
            // off for this flight (waves=on keeps it), its settings untouched (Scatterer saves them on a scene change)
            ctx.Record("scatterer waves", ctx.Param("waves") == "on" ? "on" : ScattererWavesOff());
            yield return Demo(ctx);
        }

        // Scatterer's wave interaction stopped in this scene: each ocean's GPUWaveInteractionHandler (it sets every part's
        // PartBuoyancy.waterLevel to the wave height under it each frame) cleaned up and dropped, the parts' water levels
        // back to the sea's
        private static string ScattererWavesOff()
        {
            Type t = AssemblyLoader.loadedAssemblies.Select(a => a.assembly.GetType("Scatterer.OceanFFTgpu")).FirstOrDefault(x => x != null);
            if (t == null)
                return "no Scatterer";
            FieldInfo f = t.GetField("waveInteractionHandler", BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Instance);
            int n = 0;
            foreach (UnityEngine.Object o in UnityEngine.Object.FindObjectsOfType(t))
            {
                object h = f?.GetValue(o);
                if (h == null)
                    continue;
                h.GetType().GetMethod("Cleanup", BindingFlags.Public | BindingFlags.Instance)?.Invoke(h, null);
                f.SetValue(o, null);
                n++;
            }
            foreach (Part q in FlightGlobals.ActiveVessel.parts.Where(q => q.partBuoyancy != null))
                q.partBuoyancy.waterLevel = 0;
            return n > 0 ? $"off for the flight ({n} wave handler(s) dropped)" : "none running";
        }

        // RP-1 with KSCSwitcher (RSS): RP-1's ScenarioHandler asks KSCSwitcher for its sites at the main menu, and in a
        // run that comes before KSCSwitcher has loaded them (its KSCLoader logs "loaded N launch sites" after). The
        // binding throws half made and is cached so: every later call throws, and RP-1's space center data fails to load
        // ("Error Loading RP-1 Data", in each scene). Bound again here, the sites loaded by now; its popups closed.
        private static IEnumerator Rp1KscRebind(TestContext ctx)
        {
            Type t = AssemblyLoader.loadedAssemblies.Select(a => a.assembly.GetType("RP0.KSCSwitcherInterop")).FirstOrDefault(x => x != null);
            if (t == null)
                yield break;
            t.GetField("_isKSCSwitcherInstalled", BindingFlags.NonPublic | BindingFlags.Static)?.SetValue(null, null);
            bool bound = (bool)t.GetProperty("IsKSCSwitcherInstalled", BindingFlags.Public | BindingFlags.Static).GetValue(null);
            string site = bound ? t.GetMethod("GetActiveRSSKSC", BindingFlags.Public | BindingFlags.Static).Invoke(null, null) as string : null;
            ctx.Record("rp1 ksc", $"RP-1's KSCSwitcher binding made again: {bound}, site {site}");
            yield return Wait.Seconds(2);
            foreach (PopupDialog d in Ugui.OpenPopups().ToList())
                d.Dismiss();
            yield return Wait.Frames(5);
        }

        private static IEnumerator Demo(TestContext ctx)
        {
            double overStick = ctx.ParamDouble("overStick", 0), overSec = ctx.ParamDouble("overSec", 30);
            double neutralSec = ctx.ParamDouble("neutralSec", 60), pullMph = ctx.ParamDouble("pullMph", 70);
            // after the drain, the yoke at humpStick (full aft) over the hump as the supplement flies it, until the nose
            // starts down (as Takeoff's pilot), then eased to stepStick over 3 s (neutral; forward lowers the nose onto the
            // planing attitude)
            double humpStick = ctx.ParamDouble("humpStick", 1), stepStick = ctx.ParamDouble("stepStick", 0);
            double overT = ctx.ParamDouble("overLb", 398) * Lb;
            int flapSetting = (int)ctx.ParamDouble("flaps", 1);      // the supplement's 10 deg
            bool record = ctx.Param("record") != "false";
            const double Mph = 0.44704, Ft = 0.3048, Lbf = 4.448e-3, MacM = 58.8 * 0.0254;

            Hyper(ctx, false);
            StandardDay(ctx, null);
            WaterParams.Restore();
            yield return Wait.Seconds(5);
            Plane p = null;
            foreach (object o in Prepare(ctx, r => p = r))
                yield return o;
            Vessel v = p.V;
            CelestialBody body = v.mainBody;
            // the overload as the craft loads it (Prepare fills the fuel tank to the gross): overLb more in the fuel tank, at
            // the C.G. station (the C.G. stays put), in its LiquidFuel (stock: the engine's fuel; RO: a resource of its own
            // beside the AvGas), which the drain valve on the tank dumps
            ModuleResourceDrain drain = v.parts.Select(q => q.Modules.GetModule<ModuleResourceDrain>()).FirstOrDefault(m => m != null);
            Assert.True(drain != null && drain.part.parent == p.Fuel, "a drain valve on the fuel tank");
            PartResource over = p.Fuel.Resources.Get("LiquidFuel");
            Assert.True(over != null && over.info.partResourceDrainDefinition.isDrainable, "the fuel tank holds drainable LiquidFuel");
            over.amount += overT / over.info.density;
            Assert.True(over.amount <= over.maxAmount + 1e-6, $"the fuel tank holds {ctx.ParamDouble("overLb", 398):F0} lb more " +
                                                            $"({over.amount * over.info.density / Lb:F0} lb of {over.maxAmount * over.info.density / Lb:F0})");
            double grossT = v.GetTotalMass();
            for (int i = 0; i < 3; i++)
                yield return new WaitForFixedUpdate();
            double MacPct(Vector3 cgw) => 28.4 - (p.Root.transform.InverseTransformPoint(cgw).y - CgLocalY) / MacM * 100;
            ctx.Record("loaded", $"{v.GetTotalMass() / Lb:F0} lb, the fuel tank " +
                                 string.Join(", ", p.Fuel.Resources.Cast<PartResource>().Select(r => $"{r.resourceName} {r.amount:F2}")) +
                                 $"; C.G. {MacPct(WorldCoM(v)):F1}% MAC");

            bool crashWas = CheatOptions.NoCrashDamage, hydroWas = FARDebugValues.hydrodynamics;
            GameObject recorderGo = null;
            try
            {
                CheatOptions.NoCrashDamage = true;
                // FAR's water model (parameter hydro=off: the water as thick air, as before it)
                FARDebugValues.hydrodynamics = ctx.Param("hydro") != "off";
                ctx.Record("hydrodynamics", FARDebugValues.hydrodynamics.ToString());
                yield return Wait.GameSeconds(3);     // settle afloat
                Vector3 up = v.upAxis;
                Vector3 h0 = Vector3.ProjectOnPlane(p.Root.transform.up, up).normalized;
                PlaningVideoTests.Recorder rec = null;
                FlightCamera cam = FlightCamera.fetch;
                float camHdg = 0, camPitch = (float)(ctx.ParamDouble("camPitch", 6) * Mathf.Deg2Rad);
                if (record)
                {
                    // the camera abeam (the right side), camAngle aft of it; its heading found by trying two (camHdg's
                    // zero and sense depend on the camera mode)
                    cam.minPitch = -0.5f;
                    cam.SetDistanceImmediate((float)ctx.ParamDouble("camDistance", 6.8));
                    Vector3 side = Vector3.Cross(up, h0).normalized;
                    Vector3 look = Quaternion.AngleAxis((float)ctx.ParamDouble("camAngle", 25), up) * side;
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
                    camHdg = Math.Abs(slope) > 0.2f ? -a0 / slope : 0;
                    GameEvents.onHideUI.Fire();
                    string frames = Path.Combine(ctx.ResultsDir, "frames");
                    Directory.CreateDirectory(frames);
                    recorderGo = new GameObject("FloatplaneDemoRecorder");
                    rec = recorderGo.AddComponent<PlaningVideoTests.Recorder>();
                    rec.Dir = frames;
                    Time.captureFramerate = 30;
                }
                // (recording, hyper-warped only when hyperWarp is given: each frame then 1/30 s of real time, more of the
                // game's, the video a time-lapse)
                if (!record || ctx.Param("hyperWarp") != null)
                    Hyper(ctx, true);

                foreach (ferram4.FARControllableSurface c in v.parts.SelectMany(q => q.Modules.OfType<ferram4.FARControllableSurface>()).Where(c => c.isFlap))
                    c.SetDeflection(flapSetting);
                p.Engine.Modules.GetModule<ModuleEngines>().Activate();
                Ctrl.Attach();
                Ctrl.SetThrottle(1f);
                ctx.Record("start", $"flaps {flapSetting * 10}, overweight stick {overStick:F2}, {Ambient(v)}");

                var csv = new StringBuilder("t,phase,lb,cg_mac,V_ms,IAS_mph,pitch_deg,elevator,keel_alt_m,wet,thrust_lbf,water_drag_lbf,air_drag_lbf\n");
                // (waterLog=from+to, game s: every step between, the floats' water forces and moments about the CoM and each
                // strip of the first float, for looking into its pitching)
                string[] wl = (ctx.Param("waterLog") ?? "").Split('+');
                double wlFrom = wl.Length == 2 ? double.Parse(wl[0], CultureInfo.InvariantCulture) : double.NaN;
                double wlTo = wl.Length == 2 ? double.Parse(wl[1], CultureInfo.InvariantCulture) : double.NaN;
                var waterCsv = new StringBuilder("t,pitch_deg,q_dps,keel_alt_m,V_ms,hydrostatic_up_kN,flow_up_kN,hydrostatic_kNm,flow_kNm,flow_drag_kN\n");
                var stripCsv = new StringBuilder("t,x_from_step_m,width_m,ma_kg_m,v_ms,jump_N,load_N,separated\n");
                if (!double.IsNaN(wlFrom))
                    FARAeroComponents.PartHydrodynamics.RecordStrips = true;
                var fixedWait = new WaitForFixedUpdate();
                string phase = "overweight", lastPhase = phase;
                double t0 = Planetarium.GetUniversalTime(), since = 0, lastPitch = 0, lastRoll = 0, lastHdg = 0, integral = 0;
                double airDrag = 0, waterEma = 0, airborneSince = double.NaN, liftoffT = double.NaN, capMph = double.NaN, capT = double.NaN;
                double humpPeak = -90, nextLog = 10, overHumpT = double.NaN;
                // the water's level under the floats (Scatterer's waves: PartBuoyancy.waterLevel; 0 on a flat sea): its
                // range over the run and over each log line's 10 s
                double waveLo = 0, waveHi = 0, logLo = 0, logHi = 0;
                bool overHump = false;
                var speeds = new Queue<(double t, double V)>();
                var phases = new StringBuilder();
                int steps = 0;
                while (true)
                {
                    double t = Planetarium.GetUniversalTime() - t0, dt = TimeWarp.fixedDeltaTime;
                    if (!Finite(p.Root.rb.velocity))
                    {
                        ctx.Record("nan", $"t {t:F2} s");
                        Assert.True(false, "the vessel's state is finite");
                    }
                    if (phase != lastPhase)
                    {
                        phases.Append($"{lastPhase} to {phase} at {t:F1} s, {v.indicatedAirSpeed / Mph:F1} mph IAS; ");
                        lastPhase = phase;
                        since = t;
                        integral = 0;
                    }
                    double inPhase = t - since;
                    up = v.upAxis;
                    Vector3 fwd = p.Root.transform.up, right = p.Root.transform.right;
                    double pitch = Math.Asin(Mathf.Clamp(Vector3.Dot(fwd, up), -1, 1)) * 180 / Math.PI;
                    double roll = Math.Asin(Mathf.Clamp(Vector3.Dot(right, up), -1, 1)) * 180 / Math.PI;
                    double hdg = Vector3.SignedAngle(Vector3.ProjectOnPlane(h0, up), Vector3.ProjectOnPlane(fwd, up), up);
                    double q = (pitch - lastPitch) / dt, pr = (roll - lastRoll) / dt, rr = (hdg - lastHdg) / dt;
                    lastPitch = pitch; lastRoll = roll; lastHdg = hdg;
                    double V = v.srfSpeed, ias = v.indicatedAirSpeed;
                    bool wet = p.Floats.Any(f => f.Parts.Any(x => x.submergedPortion > 0));
                    double keelAlt = p.Floats.Min(f => FlightGlobals.getAltitudeAtPos(f.TransformPoint(StepKeel), body));
                    speeds.Enqueue((t, V));
                    while (speeds.Peek().t < t - 8)
                        speeds.Dequeue();
                    double gain8 = V - speeds.Peek().V;     // m/s over the last 8 s
                    foreach (Part fp in p.Floats.SelectMany(f => f.Parts).Where(x => x.partBuoyancy != null))
                    {
                        double lvl = fp.partBuoyancy.waterLevel;
                        waveLo = Math.Min(waveLo, lvl); waveHi = Math.Max(waveHi, lvl);
                        logLo = Math.Min(logLo, lvl); logHi = Math.Max(logHi, lvl);
                    }
                    // (a line in the log now and then: a slow run is not a frozen one to KspTest)
                    if (t >= nextLog)
                    {
                        Debug.Log($"[KspTest] demo t {t:F0} s: {phase}, {ias / Mph:F1} mph IAS, pitch {pitch:F1} deg, " +
                                  $"water level {logLo:+0.000;-0.000} to {logHi:+0.000;-0.000} m");
                        logLo = logHi = 0;
                        nextLog += 10;
                    }

                    // the pilot
                    double elevator;
                    switch (phase)
                    {
                        case "overweight":
                            elevator = overStick;
                            if (inPhase > overSec)
                            {
                                drain.TurnOnDrain();
                                ctx.Record("drain on", $"t {t:F1} s: draining {drain.IsDraining}, {V:F2} m/s");
                                phase = "drain";
                            }
                            break;
                        case "drain":
                            elevator = overStick;
                            // (to the gross: the overload's LiquidFuel gone, or the plane back to its gross weight)
                            if (v.GetTotalMass() <= grossT - overT + 1e-4 || over.amount < 1e-3 || (inPhase > 1 && !drain.IsDraining))
                            {
                                if (drain.IsDraining)
                                    drain.TurnOffDrain();
                                phase = "neutral";
                            }
                            break;
                        case "neutral":
                            humpPeak = Math.Max(humpPeak, pitch);
                            if (!overHump && V > 6 && (pitch < humpPeak - 0.5 || V > 10))
                            {
                                overHump = true;
                                overHumpT = t;
                                ctx.Record("over the hump", $"t {t:F1} s, {ias / Mph:F1} mph IAS, peak pitch {humpPeak:F1} deg");
                            }
                            elevator = overHump ? humpStick + (stepStick - humpStick) * Math.Min(1, (t - overHumpT) / 3) : humpStick;
                            if (pullMph <= 0 && inPhase > 15 && gain8 < 0.4 && double.IsNaN(capMph))
                            {
                                capMph = ias / Mph;
                                capT = t;
                            }
                            if (pullMph > 0 ? ias >= pullMph * Mph : t - capT > 6 || inPhase > neutralSec)
                                phase = "pull";
                            break;
                        case "pull":
                            elevator = 1;
                            break;
                        default:                                       // climbing: 8 deg held
                        {
                            double err = Math.Max(-10, Math.Min(10, 8 - pitch));
                            elevator = 0.15 * err + 0.03 * integral - 0.06 * q;
                            if (Math.Abs(elevator) < 1 || Math.Sign(err) != Math.Sign(elevator))
                                integral = Math.Max(-30, Math.Min(30, integral + err * dt));
                            break;
                        }
                    }
                    elevator = Mathf.Clamp((float)elevator, -1, 1);
                    Ctrl.Pitch = (float)elevator;
                    if (t >= wlFrom && t <= wlTo)
                    {
                        Vector3 com = WorldCoM(v);
                        Vector3 mh = Vector3.zero, mf = Vector3.zero, fh = Vector3.zero, ff = Vector3.zero;
                        foreach (HullParts f in p.Floats)
                            foreach (FARAeroComponents.PartHydrodynamics x in f.Hydro)
                            {
                                Vector3 r0 = x.part.rb.worldCenterOfMass - com;
                                mh += Vector3.Cross(r0, x.LastHydrostaticForce) + x.LastHydrostaticTorque;
                                mf += Vector3.Cross(r0, x.LastForce) + x.LastTorque;
                                fh += x.LastHydrostaticForce;
                                ff += x.LastForce;
                            }
                        waterCsv.AppendLine(string.Join(",", F(t), F(pitch), F(q), F(keelAlt), F(V), F(Vector3.Dot(fh, up)), F(Vector3.Dot(ff, up)),
                                                        F(-Vector3.Dot(mh, right)), F(-Vector3.Dot(mf, right)), F(-Vector3.Dot(ff, fwd))));
                        HullParts f0 = p.Floats[0];
                        foreach (var st in f0.StripData)
                            stripCsv.AppendLine(string.Join(",", F(t), F(f0.InverseTransformPoint(st.point).y - StepKeel.y), F(st.width), F(st.ma),
                                                            F(st.v), F(st.jump), F(st.load), st.separated ? "1" : "0"));
                    }
                    Ctrl.Roll = Mathf.Clamp((float)(0.08 * roll + 0.03 * pr), -1, 1);
                    Ctrl.Yaw = Mathf.Clamp((float)(-0.10 * hdg - 0.05 * rr), -1, 1);

                    // off the water: every float clear for a second (from any phase: the record says which)
                    if (!wet && phase != "overweight" && phase != "drain")
                    {
                        if (double.IsNaN(airborneSince))
                            airborneSince = t;
                        if (phase != "climb" && t - airborneSince > 1)
                        {
                            liftoffT = t;
                            phase = "climb";
                        }
                    }
                    else if (phase != "climb")
                        airborneSince = double.NaN;
                    if (phase == "climb" && (keelAlt > 100 * Ft || t - liftoffT > 25))
                        break;
                    if (t > 400)
                        break;

                    if (++steps % 10 == 0)
                    {
                        FARAPI.CalculateVesselAeroForces(v, out Vector3 af, out Vector3 _, (Vector3)v.srf_velocity, Math.Max(0, v.altitude));
                        airDrag = -Vector3.Dot(af, ((Vector3)v.srf_velocity).normalized);
                    }
                    Vector3 water = Vector3.zero;
                    foreach (HullParts f in p.Floats)
                        water += f.Sum(x => x.LastForce + x.LastHydrostaticForce);
                    Vector3 runDir = Vector3.ProjectOnPlane(v.srf_velocity, up).normalized;
                    double waterDrag = -Vector3.Dot(water, runDir);
                    waterEma += Math.Min(1, dt / 0.5) * (waterDrag - waterEma);
                    double lb = v.GetTotalMass() / Lb, thrust = p.Get("propThrust");
                    double mac = MacPct(WorldCoM(v));
                    if (rec != null)
                    {
                        cam.camHdg = camHdg;
                        cam.camPitch = camPitch;
                        string stick = elevator > 0.98 ? "full aft" : Math.Abs(elevator) < 0.02 ? "neutral" : $"{elevator * 100:+0;-0}%";
                        string caption = phase switch
                        {
                            "overweight" => inPhase < 12
                                ? $"Fuselage tank full of fuel: {overT / Lb:F0} lb over max takeoff weight, C.G. far aft. Full power, stick neutral"
                                : "Overweight: nose-high on the float heels, stuck behind the hump - water drag equals thrust",
                            "drain" => "Dumping the fuselage's fuel through the drain valve...",
                            "neutral" => double.IsNaN(capMph)
                                ? (V < 12 ? "At max takeoff weight: over the hump and onto the step" : "On the step, stick neutral")
                                : $"Stick neutral: too flat to lift off - water and air drag hold it at {capMph:F0} mph",
                            "pull" => "Full aft stick: the bows come up and the floats let go",
                            _ => "Airborne",
                        };
                        rec.Lines = new[]
                        {
                            FARDebugValues.hydrodynamics ? "Cessna 172M floatplane - FAR water model on" : "Cessna 172M floatplane - FAR water model off (old water)",
                            $"weight {lb:F0} lb (max 2220)   C.G. {mac:F1}% MAC",
                            $"IAS {ias / Mph,3:F0} mph   pitch {pitch,5:F1} deg   stick {stick}",
                            FARDebugValues.hydrodynamics
                                ? $"thrust {thrust / Lbf,4:F0} lbf   water drag {Math.Max(0, waterEma) / Lbf,4:F0} lbf   air drag {airDrag / Lbf,3:F0} lbf"
                                : $"thrust {thrust / Lbf,4:F0} lbf   air drag {airDrag / Lbf,3:F0} lbf",
                            caption,
                        };
                    }
                    csv.AppendLine(string.Join(",", F(t), phase, F(lb), F(mac), F(V), F(ias / Mph), F(pitch), F(elevator), F(keelAlt), wet ? "1" : "0",
                                               F(thrust / Lbf), F(waterDrag / Lbf), F(airDrag / Lbf)));
                    yield return fixedWait;
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "demo.csv"), csv.ToString());
                if (!double.IsNaN(wlFrom))
                {
                    FARAeroComponents.PartHydrodynamics.RecordStrips = false;
                    File.WriteAllText(Path.Combine(ctx.ResultsDir, "demo-water.csv"), waterCsv.ToString());
                    File.WriteAllText(Path.Combine(ctx.ResultsDir, "demo-strips.csv"), stripCsv.ToString());
                }
                ctx.Record("phases", phases.ToString());
                ctx.Record("neutral cap", double.IsNaN(capMph) ? "none (still gaining)" : $"{capMph:F1} mph IAS");
                ctx.Record("liftoff", double.IsNaN(liftoffT) ? "no" : $"t {liftoffT:F1} s");
                ctx.Record("water level", $"{waveLo:+0.000;-0.000} to {waveHi:+0.000;-0.000} m under the floats");
                if (rec != null)
                    ctx.Record("frames", rec.Frame);
                Assert.True(!double.IsNaN(liftoffT), "it flew");
            }
            finally
            {
                Time.captureFramerate = 0;
                if (recorderGo != null)
                    UnityEngine.Object.Destroy(recorderGo);
                GameEvents.onShowUI.Fire();
                Ctrl.SetThrottle(0f);
                Ctrl.Neutral();
                Hyper(ctx, false);
                StandardDay(ctx, null);
                WaterParams.Restore();
                CheatOptions.NoCrashDamage = crashWas;
                FARDebugValues.hydrodynamics = hydroWas;
            }
        }

        // the vessel's centre of mass from its parts' rigidbodies (their masses with resources, their CoM offsets)
        private static Vector3 WorldCoM(Vessel v)
        {
            Vector3 sum = Vector3.zero;
            float m = 0;
            foreach (Part q in v.parts)
                if (q.rb != null)
                {
                    sum += q.rb.worldCenterOfMass * q.rb.mass;
                    m += q.rb.mass;
                }
            return sum / m;
        }

        // load the craft's numbers into place and float it: weight, C.G., FAR's surfaces recorded; on the water at rest
        private static IEnumerable Prepare(TestContext ctx, Action<Plane> done, double grossT = GrossT, bool afloat = true)
        {
            Vessel v = FlightGlobals.ActiveVessel;
            StandardDay(ctx, v.mainBody);
            WaterParams.Apply(ctx);
            var p = new Plane { V = v, Root = v.rootPart };
            p.Engine = v.parts.FirstOrDefault(q => q.Modules.Contains("ModuleEnginesAJEPropeller"));
            Assert.True(p.Engine != null, "the O-320 (AJE propeller engine) is on the craft");
            p.Aje = p.Engine.Modules["ModuleEnginesAJEPropeller"];
            // the fuel: the engine's propellant (LiquidFuel; RO's AvGas) in the tank inside the fuselage (B9's wing panels
            // hold fuel too)
            p.FuelName = p.Engine.Modules.GetModule<ModuleEngines>().propellants.Select(r => r.name).First(n => n != "IntakeAir" && n != "IntakeAtm");
            p.Fuel = v.parts.FirstOrDefault(q => q != p.Root && (q.partInfo.name == "proceduralTankLiquid" || q.partInfo.name == "RO-ProcTankSpaceplane") &&
                                                 q.Resources.Contains(p.FuelName));
            Assert.True(p.Fuel != null, "a fuel tank");
            // the floats: the structural Spines along the fuselage (the struts stand across it), one part or several end to
            // end, each as long as the float (crafts/edo_float.py: 16.05 ft)
            p.Floats = HullParts.Find(v, q => q != p.Root && q.partInfo.name == "proceduralStructural" &&
                                              Math.Abs(Vector3.Dot(q.transform.up, p.Root.transform.up)) > 0.99f).ToArray();
            Assert.True(p.Floats.Length == 2, $"two floats ({p.Floats.Length})");
            ctx.Record("floats", string.Join("; ", p.Floats.Select(f => $"{f.Parts.Length} part(s), {f.Length:F3} m")));
            Assert.True(p.Floats.All(f => Math.Abs(f.Length - FloatLength) < 0.05), $"floats {FloatLength} m long");

            // weight: everything as built and emptied (but electric charge and air), the fuel making up the gross
            foreach (PartResource res in v.parts.SelectMany(q => q.Resources.Cast<PartResource>()))
                if (res.resourceName != "ElectricCharge" && res.resourceName != "IntakeAir")
                    res.amount = 0;
            yield return Wait.Frames(2);
            double dry = v.GetTotalMass();
            PartResource lf = p.Fuel.Resources.Get(p.FuelName);
            double add = (grossT - dry) / lf.info.density;
            var parts = new StringBuilder();
            foreach (var g in v.parts.GroupBy(q => q.partInfo.name))
                parts.Append($"{g.Key} x{g.Count()} {g.Sum(q => q.mass) * 1000:F0} kg; ");
            ctx.Record("mass", $"empty {dry * 1000:F0} kg ({dry / Lb:F0} lb), gross {grossT / Lb:F0} lb: {add * lf.info.density * 1000:F0} kg of fuel " +
                               $"({lf.maxAmount * lf.info.density * 1000:F0} kg tank). {parts}");
            Assert.True(add >= 0 && add <= lf.maxAmount, $"the gross reachable with fuel (empty {dry / Lb:F0} lb)");
            lf.amount = add;
            // the rigidbodies' masses take the fuel at the next physics frames
            for (int i = 0; i < 3; i++)
                yield return new WaitForFixedUpdate();

            // C.G.: as the craft has it (crafts/c172_floatplane.py places the fuel tank for CgLocalY at the gross); checked,
            // not set, so a flight from the craft file flies the test's airplane
            Vector3 cg0 = p.Root.transform.InverseTransformPoint(WorldCoM(v));
            var where = new StringBuilder();
            foreach (Part q in v.parts.Where(q => q.rb != null).OrderByDescending(q => q.rb.mass).Take(8))
                where.Append($"{q.partInfo.name} {q.rb.mass * 1000:F0} kg at y {p.Root.transform.InverseTransformPoint(q.rb.worldCenterOfMass).y:F2}; ");
            ctx.Record("cg as built", $"local y {cg0.y:F3} m: {where}");
            // the fuel tank's move that would put it at CgLocalY (for c172_floatplane.py's TANK_SHIFT)
            double tankShift = (CgLocalY - cg0.y) * v.GetTotalMass() / (p.Fuel.mass + p.Fuel.GetResourceMass());
            ctx.Record("cg error", $"{(cg0.y - CgLocalY) * 100:+0.0;-0.0} cm ({(cg0.y - CgLocalY) / (58.8 * 0.0254):+0.0%;-0.0%} MAC); " +
                                   $"the tank {tankShift * 100:+0.0;-0.0} cm to correct it");
            Assert.True(ctx.Param("cgCheck") == "false" || Math.Abs(cg0.y - CgLocalY) < (Math.Abs(grossT - GrossT) < 1e-6 ? 0.01 : 0.03),
                        $"the craft's C.G. {cg0.y:F3} m, CgLocalY {CgLocalY:F3} (crafts/c172_floatplane.py TANK_SHIFT {tankShift:+0.000;-0.000} m)");
            Vector3 cg = cg0;
            ctx.Record("cg", $"local y {cg.y:F3} m (target {CgLocalY:F3}: 28.4% MAC), {-cg.z:F3} m above the fuselage axis; fuel tank CoM offset {p.Fuel.CoMOffset}; " +
                             "steps " + string.Join(", ", p.Floats.Select(f => $"{p.Root.transform.InverseTransformPoint(f.TransformPoint(StepKeel)).y - cg.y:F3}")) +
                             " m ahead of it (generator: -0.234), float origins " + string.Join(", ", p.Floats.Select(f => $"{p.Root.transform.InverseTransformPoint(f.Position)}")));

            // FAR's surfaces: area by kind (the wing high, the tail aft, the fin upright), control surfaces
            double wing = 0, tail = 0, fin = 0;
            var surfaces = new StringBuilder();
            foreach (Part q in v.parts)
            {
                var w = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                if (w == null)
                    continue;
                Vector3 lp = p.Root.transform.InverseTransformPoint(q.transform.position);
                Vector3 span = p.Root.transform.InverseTransformDirection(q.transform.right);
                bool upright = Mathf.Abs(span.z) > 0.7f;
                double s = w.S;
                if (upright) fin += s;
                else if (lp.y < -2) tail += s;
                else wing += s;
                if (w is ferram4.FARControllableSurface c)
                    surfaces.Append($"{(upright ? "fin" : lp.y < -2 ? "tail" : "wing")} at y {lp.y:F2} x {lp.x:F2}: S {s:F2} frac {c.ctrlSurfFrac:F2} " +
                                    $"flap {c.isFlap} pitch {c.pitchaxis} roll {c.rollaxis} yaw {c.yawaxis} max {c.maxdeflect}/{c.maxdeflectFlap}; ");
            }
            const double ft2 = 0.09290304;
            ctx.Record("surfaces", $"wing {wing:F2} m2 (172: {174 * ft2:F2}), tail {tail:F2} ({21.9 * ft2:F2}), fin {fin:F2} ({16.5 * ft2:F2}). {surfaces}");

            // on the water, level, at rest (crash damage off for the test: water landings, the run-up)
            CheatOptions.NoCrashDamage = true;
            if (!afloat)
            {
                done(p);
                yield break;
            }
            v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
            // the site: Far.At's (0 N 60 W, over Kerbin's sea); on RSS's Earth that is the Amazon, so off Cape Canaveral
            FarFlightProbe.Condition site = Far.At(0, 600, 0, "float", speedMs: 0);
            bool earth = v.mainBody.Radius > 1e6;
            site.Lat = ctx.ParamDouble("lat", earth ? 28.45 : site.Lat);
            site.Lon = ctx.ParamDouble("lon", earth ? -80.45 : site.Lon);
            yield return FarFlightProbe.Place(site);
            v = p.V = FlightGlobals.ActiveVessel;
            Vector3 up = v.upAxis;
            Vector3 h = Vector3.ProjectOnPlane(p.Root.transform.up, up).normalized;
            Flight.PointAtAoA(0, h, up);
            yield return new WaitForFixedUpdate();
            double keelAlt = p.Floats.Min(f => FlightGlobals.getAltitudeAtPos(f.TransformPoint(StepKeel), v.mainBody));
            v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-0.30 - keelAlt));
            foreach (Part q in v.parts.Where(q => q.rb != null))
            {
                q.rb.velocity = Vector3.zero;
                q.rb.angularVelocity = Vector3.zero;
            }
            done(p);
        }
    }
}
