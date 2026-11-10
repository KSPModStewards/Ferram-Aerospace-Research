using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using KspTest;
using UnityEngine;

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
            public Part[] Floats;
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
        public static IEnumerator Setup_C172Floatplane(TestContext ctx)
        {
            Hyper(ctx, false);
            StandardDay(ctx, null);
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
                    draft.Add(p.Floats.Average(f => -FlightGlobals.getAltitudeAtPos(f.transform.TransformPoint(StepKeel), body)));
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
                foreach (Part q in v.parts)
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
        public static IEnumerator Trim_C172Floatplane(TestContext ctx)
        {
            double[] mphs = ctx.ParamList("mphs", "60+80+110").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            double weightLb = ctx.ParamDouble("weightLb", 2220);
            Hyper(ctx, false);
            StandardDay(ctx, null);
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
                foreach (Part q in v.parts)
                    foreach (var w in q.Modules.OfType<ferram4.FARWingAerodynamicModel>())
                    {
                        Vector3d wf = w.PrecomputeCenterOfLift(air, V0 / 340.0, 1.2, dummy);
                        Vector3 ac = (Vector3)w.GetAerodynamicCenter();
                        Vector3 lp = p.Root.transform.InverseTransformPoint(q.transform.position);
                        bool upright = Mathf.Abs(p.Root.transform.InverseTransformDirection(q.transform.right).z) > 0.7f;
                        string kind = upright ? "fin" : lp.y < -2 ? "tail" : w is ferram4.FARControllableSurface c ? (c.isFlap ? "flaps" : "ailerons") : "wing";
                        double l = Vector3.Dot((Vector3)wf, liftDir), m = -Vector3.Dot(Vector3.Cross(ac - com, (Vector3)wf), right);
                        if (!byKind.TryGetValue(kind, out double[] acc))
                            byKind[kind] = acc = new double[4];
                        acc[a > 0 ? 2 : 0] += l;
                        acc[a > 0 ? 3 : 1] += m;
                        wingsL += l; wingsM += m;
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
            Ctrl.Pitch = 0;
            Hyper(ctx, false);
            StandardDay(ctx, null);
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "trim.csv"), csv.ToString());
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
        public static IEnumerator Captive_C172Floatplane(TestContext ctx)
        {
            double V = ctx.ParamDouble("mph", 60) * 0.44704, weightLb = ctx.ParamDouble("weightLb", 2220);
            double[] pitches = ctx.ParamList("pitches", "4+6+8+10+12").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            Hyper(ctx, false);
            StandardDay(ctx, null);
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
                            foreach (Part f in p.Floats)
                            {
                                var hd = f.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                                if (hd != null)
                                    water += hd.LastForce + hd.LastHydrostaticForce;
                            }
                            wUp.Add(Vector3.Dot(water, up));
                            wDrag.Add(-Vector3.Dot(water, h));
                            keel.Add(-p.Floats.Average(f => FlightGlobals.getAltitudeAtPos(f.transform.TransformPoint(StepKeel), body)));
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
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "captive.csv"), csv.ToString());
            Ctrl.SetThrottle(0f);
            Ctrl.Neutral();
        }

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
        /// off the water), climbMph. Writes takeoff.csv.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = Craft, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Takeoff_C172Floatplane(TestContext ctx)
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
            double climbMs = ctx.ParamDouble("climbMph", book.ias50) * 0.44704;
            const double Mph = 0.44704, Ft = 0.3048;

            Hyper(ctx, false);
            StandardDay(ctx, null);
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
                c.SetDeflection(1);                // 10 deg: FAR's three settings of the 30 deg travel
            p.Engine.Modules.GetModule<ModuleEngines>().Activate();
            Ctrl.Attach();
            Ctrl.SetThrottle(1f);
            ctx.Record("start", $"{weightLb:F0} lb, flaps 10, {Ambient(v)}");

            Vector3 up = v.upAxis;
            Vector3 h0 = Vector3.ProjectOnPlane(p.Root.transform.up, up).normalized;
            var csv = new StringBuilder("t,x_m,V_ms,IAS_mph,pitch_deg,roll_deg,heading_deg,keel_alt_m,wet,phase,elevator,rpm,thrust_lbf,float_rbDrag,aero_moment_kNm,pitch_rate\n");
            string phase = "hump", lastPhase = "hump";
            double aeroMoment = double.NaN;
            double t0 = Planetarium.GetUniversalTime(), peak = -90, lastPitch = 0, lastRoll = 0, lastHdg = 0, x = 0, integral = 0;
            int steps = 0;
            double runX = double.NaN, runT = double.NaN, obstacleX = double.NaN, obstacleT = double.NaN, airborneSince = double.NaN;
            while (Planetarium.GetUniversalTime() - t0 < 150 && double.IsNaN(obstacleX) && !(phase == "hump" && Planetarium.GetUniversalTime() - t0 > 40))
            {
                double t = Planetarium.GetUniversalTime() - t0, dt = TimeWarp.fixedDeltaTime;
                up = v.upAxis;
                Vector3 fwd = p.Root.transform.up, right = p.Root.transform.right;
                double pitch = Math.Asin(Mathf.Clamp(Vector3.Dot(fwd, up), -1, 1)) * 180 / Math.PI;
                double roll = Math.Asin(Mathf.Clamp(Vector3.Dot(right, up), -1, 1)) * 180 / Math.PI;     // right wing up +
                double hdg = Vector3.SignedAngle(Vector3.ProjectOnPlane(h0, up), Vector3.ProjectOnPlane(fwd, up), up); // nose right +
                double q = (pitch - lastPitch) / dt, pr = (roll - lastRoll) / dt, r = (hdg - lastHdg) / dt;
                lastPitch = pitch; lastRoll = roll; lastHdg = hdg;
                double V = v.srfSpeed, ias = v.indicatedAirSpeed;
                x += Vector3.ProjectOnPlane(v.srf_velocity, up).magnitude * dt;     // the run over the surface (the floating origin moves)
                bool wet = p.Floats.Any(f => f.submergedPortion > 0);
                double keelAlt = p.Floats.Min(f => FlightGlobals.getAltitudeAtPos(f.transform.TransformPoint(StepKeel), body));

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
                        target = 90;                                       // the yoke back until it flies
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
                if (double.IsNaN(target))
                    elevator = 1 + (stepElevator - 1) * Math.Min(1, (t - stepSince) / 3);
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
                }
                csv.AppendLine(string.Join(",", F(t), F(x), F(V), F(ias / Mph), F(pitch), F(roll), F(hdg), F(keelAlt), wet ? "1" : "0", phase, F(elevator),
                                           F(p.Get("propRPM")), F(p.Get("propThrust") / 4.448e-3), F(p.Floats.Max(f => f.rb.drag)), F(aeroMoment), F(q)));
                yield return fixedWait;
            }
            Hyper(ctx, false);
            StandardDay(ctx, null);
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "takeoff.csv"), csv.ToString());
            Ctrl.SetThrottle(0f);
            Ctrl.Neutral();
            ctx.Record("takeoff", $"{weightLb:F0} lb: water run {runX / Ft:F0} ft in {runT:F1} s (supplement {book.run:F0} ft), to 50 ft {obstacleX / Ft:F0} ft " +
                                  $"in {obstacleT:F1} s (supplement {book.obstacle:F0} ft at {book.ias50:F0} mph); peak hump pitch {peak:F1} deg");
            Assert.True(!double.IsNaN(runX), "off the water");
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);

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
            var p = new Plane { V = v, Root = v.rootPart };
            p.Engine = v.parts.FirstOrDefault(q => q.Modules.Contains("ModuleEnginesAJEPropeller"));
            Assert.True(p.Engine != null, "the O-320 (AJE propeller engine) is on the craft");
            p.Aje = p.Engine.Modules["ModuleEnginesAJEPropeller"];
            // the fuel: the tank inside the fuselage (B9's wing panels hold fuel too)
            p.Fuel = v.parts.FirstOrDefault(q => q != p.Root && q.partInfo.name == "proceduralTankLiquid" && q.Resources.Contains("LiquidFuel"));
            Assert.True(p.Fuel != null, "a fuel tank");
            // the floats: the Spines as long as the float (crafts/edo_float.py: 16.05 ft)
            p.Floats = v.parts.Where(q => q.Modules.Contains("ProceduralShapeSpine") &&
                                          Math.Abs(Convert.ToSingle(q.Modules["ProceduralShapeSpine"].Fields.GetValue("length")) - FloatLength) < 0.05f).ToArray();
            Assert.True(p.Floats.Length == 2, $"two floats ({p.Floats.Length})");

            // weight: everything as built and emptied (but electric charge and air), the fuel making up the gross
            foreach (PartResource res in v.parts.SelectMany(q => q.Resources.Cast<PartResource>()))
                if (res.resourceName != "ElectricCharge" && res.resourceName != "IntakeAir")
                    res.amount = 0;
            yield return Wait.Frames(2);
            double dry = v.GetTotalMass();
            PartResource lf = p.Fuel.Resources.Get("LiquidFuel");
            double add = (grossT - dry) / lf.info.density;
            var parts = new StringBuilder();
            foreach (var g in v.parts.GroupBy(q => q.partInfo.name))
                parts.Append($"{g.Key} x{g.Count()} {g.Sum(q => q.mass) * 1000:F0} kg; ");
            ctx.Record("mass", $"empty {dry * 1000:F0} kg ({dry / Lb:F0} lb), gross {grossT / Lb:F0} lb: {add * lf.info.density * 1000:F0} kg of fuel " +
                               $"({lf.maxAmount * lf.info.density * 1000:F0} kg tank). {parts}");
            Assert.True(add >= 0 && add <= lf.maxAmount, $"the gross reachable with fuel (empty {dry / Lb:F0} lb)");
            lf.amount = add;
            yield return Wait.Frames(2);

            // C.G.: the fuel tank's CoM (fuel and the load it stands for) moved along the root's local y (craft z) to put the
            // vessel's at CgLocalY
            Vector3 cg0 = p.Root.transform.InverseTransformPoint(WorldCoM(v));
            var where = new StringBuilder();
            foreach (Part q in v.parts.Where(q => q.rb != null).OrderByDescending(q => q.rb.mass).Take(8))
                where.Append($"{q.partInfo.name} {q.rb.mass * 1000:F0} kg at y {p.Root.transform.InverseTransformPoint(q.rb.worldCenterOfMass).y:F2}; ");
            ctx.Record("cg as built", $"local y {cg0.y:F3} m: {where}");
            for (int i = 0; i < 3; i++)
            {
                Vector3 com = p.Root.transform.InverseTransformPoint(WorldCoM(v));
                double shift = (CgLocalY - com.y) * v.GetTotalMass() / (p.Fuel.mass + p.Fuel.GetResourceMass());
                Vector3 along = p.Fuel.transform.InverseTransformDirection(p.Root.transform.up);
                p.Fuel.CoMOffset += along * (float)shift;
                if (p.Fuel.rb != null)
                    p.Fuel.rb.centerOfMass = p.Fuel.CoMOffset;
                yield return Wait.Frames(2);
            }
            Vector3 cg = p.Root.transform.InverseTransformPoint(WorldCoM(v));
            ctx.Record("cg", $"local y {cg.y:F3} m (target {CgLocalY:F3}: 28.4% MAC), {-cg.z:F3} m above the fuselage axis; fuel tank CoM offset {p.Fuel.CoMOffset}; " +
                             "steps " + string.Join(", ", p.Floats.Select(f => $"{p.Root.transform.InverseTransformPoint(f.transform.TransformPoint(StepKeel)).y - cg.y:F3}")) +
                             " m ahead of it (generator: -0.234), float origins " + string.Join(", ", p.Floats.Select(f => $"{p.Root.transform.InverseTransformPoint(f.transform.position)}")));

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
            yield return FarFlightProbe.Place(Far.At(0, 600, 0, "float", speedMs: 0));
            v = p.V = FlightGlobals.ActiveVessel;
            Vector3 up = v.upAxis;
            Vector3 h = Vector3.ProjectOnPlane(p.Root.transform.up, up).normalized;
            Flight.PointAtAoA(0, h, up);
            yield return new WaitForFixedUpdate();
            double keelAlt = p.Floats.Min(f => FlightGlobals.getAltitudeAtPos(f.transform.TransformPoint(StepKeel), v.mainBody));
            v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-0.30 - keelAlt));
            foreach (Part q in v.parts)
            {
                q.rb.velocity = Vector3.zero;
                q.rb.angularVelocity = Vector3.zero;
            }
            done(p);
        }
    }
}
