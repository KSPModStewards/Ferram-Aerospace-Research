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
        private const string Craft = "Cessna 172M Floatplane";
        // the same plane with each float three parts (crafts/c172_floatplane.py split): the *Split tests run it as the
        // one-part floats are run; FAR's water should not tell them apart
        private const string SplitCraft = "Cessna 172M Floatplane Split";
        // the plane with B9's own hinged control surfaces (crafts/c172_floatplane.py hinged): no FARTest wing parts
        private const string HingedCraft = "Cessna 172M Floatplane Hinged";
        // the hinged craft on RO's parts (crafts/c172_floatplane.py ro), for RealismOverhaul / RP-1 installs (KSPDIR)
        private const string HingedRoCraft = "Cessna 172M Floatplane Hinged RO";
        private const string Mods = @"AdvancedJetEngine;Firespitter;TimeControl;..\..\..\CustomProceduralParts\GameData\CustomProceduralParts;" +
                                    @"..\FerramAerospaceResearch.KspTests.Run\mods\FARTestFloatplane";

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

        /// <summary>Trim_C172Floatplane on the plane with B9's own hinged control surfaces.</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = HingedCraft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Trim_C172FloatplaneHinged(TestContext ctx) => Trim(ctx);

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
            var csv = new StringBuilder("pitch_deg,elevator,V_ms,hold_kNm,aero_kNm,thrust_kNm,water_kNm,waterUp_kN,waterDrag_kN,wingLift_kN,weight_kN,stepKeel_m,thrust_kN\n");
            ctx.Record("start", $"{weightLb:F0} lb at {V / 0.44704:F0} mph, flaps 10, full throttle; {Ambient(v)}");

            foreach (double pitchTarget in pitches)
                foreach (float elev in new[] { 0f, 1f })
                {
                    ctx.Step($"pitch {pitchTarget} elevator {elev}");
                    double integral = 0, lastPitch = double.NaN, t0 = Planetarium.GetUniversalTime(), lastRoll = 0, lastHdg = 0;
                    var hold = new List<double>(); var aero = new List<double>(); var thrustM = new List<double>(); var wUp = new List<double>();
                    var wDrag = new List<double>(); var lift = new List<double>(); var keel = new List<double>(); var thr = new List<double>(); var speed = new List<double>();
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
                        foreach (Part part in v.parts)
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
                                               F(wUp.Average()), F(wDrag.Average()), F(lift.Average()), F(weight), F(keel.Average()), F(thr.Average())));
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

        /// <summary>Takeoff_C172Floatplane with B9's own hinged control surfaces.</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = HingedCraft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Takeoff_C172FloatplaneHinged(TestContext ctx) => Takeoff(ctx);

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
        /// standardDay, linearFlapLift.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = HingedCraft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Demo_C172FloatplaneHinged(TestContext ctx) => Demo(ctx);

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
            double overT = ctx.ParamDouble("overLb", 398) * Lb;
            int flapSetting = (int)ctx.ParamDouble("flaps", 0);
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
            Part fus = p.Root;
            // the fuselage's fuel back, as the craft loads (Prepare empties it): overLb of it, its resources filled in turn
            double left = overT;
            foreach (PartResource r in fus.Resources.Cast<PartResource>().Where(r => r.info.density > 0 && r.info.partResourceDrainDefinition.isDrainable))
            {
                r.amount = Math.Min(r.maxAmount, left / r.info.density);
                left -= r.amount * r.info.density;
            }
            Assert.True(left < 1e-3, $"the fuselage holds {ctx.ParamDouble("overLb", 398):F0} lb ({left / Lb:F0} lb short)");
            ModuleResourceDrain drain = v.parts.Select(q => q.Modules.GetModule<ModuleResourceDrain>()).FirstOrDefault(m => m != null);
            Assert.True(drain != null && drain.part.parent == fus, "a drain valve on the fuselage");
            for (int i = 0; i < 3; i++)
                yield return new WaitForFixedUpdate();
            double MacPct(Vector3 cgw) => 28.4 - (p.Root.transform.InverseTransformPoint(cgw).y - CgLocalY) / MacM * 100;
            ctx.Record("loaded", $"{v.GetTotalMass() / Lb:F0} lb, the fuselage " +
                                 string.Join(", ", fus.Resources.Cast<PartResource>().Select(r => $"{r.resourceName} {r.amount:F2}")) +
                                 $"; C.G. {MacPct(WorldCoM(v)):F1}% MAC");

            bool crashWas = CheatOptions.NoCrashDamage;
            GameObject recorderGo = null;
            try
            {
                CheatOptions.NoCrashDamage = true;
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
                    cam.SetDistanceImmediate((float)ctx.ParamDouble("camDistance", 20));
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
                else
                    Hyper(ctx, true);

                foreach (ferram4.FARControllableSurface c in v.parts.SelectMany(q => q.Modules.OfType<ferram4.FARControllableSurface>()).Where(c => c.isFlap))
                    c.SetDeflection(flapSetting);
                p.Engine.Modules.GetModule<ModuleEngines>().Activate();
                Ctrl.Attach();
                Ctrl.SetThrottle(1f);
                ctx.Record("start", $"flaps {flapSetting * 10}, overweight stick {overStick:F2}, {Ambient(v)}");

                var csv = new StringBuilder("t,phase,lb,cg_mac,V_ms,IAS_mph,pitch_deg,elevator,keel_alt_m,wet,thrust_lbf,water_drag_lbf,air_drag_lbf\n");
                var fixedWait = new WaitForFixedUpdate();
                string phase = "overweight", lastPhase = phase;
                double t0 = Planetarium.GetUniversalTime(), since = 0, lastPitch = 0, lastRoll = 0, lastHdg = 0, integral = 0;
                double airDrag = 0, waterEma = 0, airborneSince = double.NaN, liftoffT = double.NaN, capMph = double.NaN, capT = double.NaN;
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
                            if (fus.Resources.Cast<PartResource>().Sum(r => r.amount) < 1e-3 || (inPhase > 1 && !drain.IsDraining))
                            {
                                if (drain.IsDraining)
                                    drain.TurnOffDrain();
                                phase = "neutral";
                            }
                            break;
                        case "neutral":
                            elevator = 0;
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
                            "Cessna 172M floatplane - FAR water model",
                            $"weight {lb:F0} lb (max 2220)   C.G. {mac:F1}% MAC",
                            $"IAS {ias / Mph,3:F0} mph   pitch {pitch,5:F1} deg   stick {stick}",
                            $"thrust {thrust / Lbf,4:F0} lbf   water drag {Math.Max(0, waterEma) / Lbf,4:F0} lbf   air drag {airDrag / Lbf,3:F0} lbf",
                            caption,
                        };
                    }
                    csv.AppendLine(string.Join(",", F(t), phase, F(lb), F(mac), F(V), F(ias / Mph), F(pitch), F(elevator), F(keelAlt), wet ? "1" : "0",
                                               F(thrust / Lbf), F(waterDrag / Lbf), F(airDrag / Lbf)));
                    yield return fixedWait;
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "demo.csv"), csv.ToString());
                ctx.Record("phases", phases.ToString());
                ctx.Record("neutral cap", double.IsNaN(capMph) ? "none (still gaining)" : $"{capMph:F1} mph IAS");
                ctx.Record("liftoff", double.IsNaN(liftoffT) ? "no" : $"t {liftoffT:F1} s");
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
