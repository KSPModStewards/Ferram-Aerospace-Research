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
    /// The Cessna floatplane's float (NACA model 57-B-5's lines, crafts/naca_57b5.py) towed as NACA towed the model free
    /// to trim (TN 716, Parkinson, Olson and House 1939): the speed held, free to rise and to trim about its centre of
    /// gravity, its load taken off with speed as NACA's hydrofoil lift device took it.
    /// </summary>
    public static class NacaFloatTests
    {
        private const string Float = "NACA 57-B-5";

        // TN 716 Fig. 4, model 57-B-5 free to trim (C_Delta0 1.575, the load zero at C_V 10; C.G. 36.1% of the beam ahead
        // of the step, 24.00 in above the keel at the model's 12.47 in beam): the trim (deg) against C_V, read off the
        // figure (+/- 0.5 deg)
        private static readonly (double cv, double trim)[] Fig4 =
        {
            (1.0, 3.3), (1.25, 4.3), (1.5, 5.4), (2.0, 6.7), (2.5, 8.2), (3.0, 10.2), (3.5, 10.9), (4.0, 10.7), (5.0, 10.5),
        };

        /// <summary>
        /// For each speed coefficient, from rest up (C_V the speed over sqrt(g b), b the beam over the spray strips): the
        /// speed held along the tow line, the float free to heave and to pitch about its C.G. (roll and yaw held level), a
        /// counterweight leaving C_Delta0 (1 - (C_V / 10)^2) of the load on the water; settled, then the trim, its spread,
        /// the step's draft and the resistance averaged. Its trim against Fig. 4's within tolerance (deg, default 2) from
        /// C_V 1 to 5. Writes naca57-free.csv. Parameters: cvs (+ separated), load (C_Delta0), tolerance, settle (s).
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.WaterMods,
                StartsAt = StartState.Flight, Craft = Float, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Tow_Naca57B5FreeToTrim(TestContext ctx)
        {
            // crafts/edo_float.py: model inches at the 172's size; the Spine's middle 47 in from the bow, the step 52
            const double Inch = 2.128 * 12.0 / 12.47 * 0.0254, Beam = 12.47 * Inch;
            double stepY = -(52.0 - 47.0) * Inch, keelAtStep = 11.40 * Inch;
            // (cgAhead: share of the beam ahead of the step; cgHeight: model inches above the keel at the step)
            double cgY = stepY + ctx.ParamDouble("cgAhead", 0.361) * Beam, cgAboveBase = (ctx.ParamDouble("cgHeight", 24.00) - 11.40) * Inch;
            double load0 = ctx.ParamDouble("load", 1.575), tolerance = ctx.ParamDouble("tolerance", 2.0);
            double settle = ctx.ParamDouble("settle", 8);
            double[] cvs = ctx.ParamList("cvs", "0+1.0+1.25+1.5+1.75+2.0+2.25+2.5+3.0+3.5+4.0+4.5+5.0+6.0+7.0")
                              .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            WaterParams.Apply(ctx);

            yield return Wait.Seconds(5);
            Vessel v = FlightGlobals.ActiveVessel;
            foreach (PartResource r in v.parts.SelectMany(q => q.Resources.Cast<PartResource>()))
                r.amount = 0;
            yield return Wait.Frames(2);

            bool crashWas = CheatOptions.NoCrashDamage;
            try
            {
                CheatOptions.NoCrashDamage = true;
                v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                yield return FarFlightProbe.Place(Far.At(0, 600, 0, "naca57", speedMs: 5));
                v = FlightGlobals.ActiveVessel;
                CelestialBody body = v.mainBody;
                HullParts hull = HullParts.Whole(v);
                Vector3 up = v.upAxis;
                Vector3 h = Vector3.ProjectOnPlane(hull.Forward, up).normalized;
                Vector3 bowUp = Vector3.Cross(h, up).normalized;
                Vector3 keel = PlaningVideoTests.KeelAxis(hull.Ref);
                Assert.True(Mathf.Abs(keel.z) > 0.5f, $"the keel is along the part's z (sections' -y): {keel}");
                hull.SetCoM(v, new Vector3(0, (float)cgY, 0) - keel * (float)cgAboveBase);
                Vector3 stepKeel = new Vector3(0, (float)stepY, 0) + keel * (float)keelAtStep;
                Vector3 cgLocal = hull.InverseTransformPoint(RigidVessel.CoM(v));

                double rho = body.oceanDensity * 1000, g0 = 9.80665 * body.GeeASL;
                double sqrtGb = Math.Sqrt(g0 * Beam), wb3 = rho * g0 * Beam * Beam * Beam / 1000;   // kN
                double mass = RigidVessel.Mass(v);
                ctx.Record("float", $"{hull.Length:F3} m long, beam {Beam:F4} m over the spray strips, mesh volume {hull.MeshVolume:F3} m3, " +
                                    $"mass {mass:F3} t; C.G. {(cgLocal.y - stepY) / Beam:P1} of the beam ahead of the step, " +
                                    $"{(cgLocal.y - stepY):F3} m (TN 716: 36.1%), {-Vector3.Dot(cgLocal - stepKeel, keel) / Inch:F2} model in above the keel " +
                                    $"at the step (24.00); load C_Delta0 {load0} ({load0 * wb3:F2} kN)");

                Quaternion Hold(double tau)
                {
                    up = (Vector3)FlightGlobals.getUpAxis(body, RigidVessel.CoM(v));
                    h = Vector3.ProjectOnPlane(h, up).normalized;
                    bowUp = Vector3.Cross(h, up).normalized;
                    Vector3 fwd = Quaternion.AngleAxis((float)tau, bowUp) * h;
                    Vector3 down = Vector3.Cross(fwd, bowUp).normalized;
                    return Quaternion.LookRotation(keel.z > 0 ? down : -down, fwd);
                }
                double Trim() => Math.Asin(Mathf.Clamp(Vector3.Dot(hull.Forward, up), -1, 1)) * 180 / Math.PI;

                // at each collider detail (ColliderDetail): the run from rest, each level's trim and resistance held to the
                // finest's after
                int[] details = ColliderDetail.Levels(ctx, "12+24+48", v);
                double setAlong = 0;
                var wrong = new List<string>();
                var lodRuns = new Dictionary<(int, double), (double trim, double cr)>();
                foreach (int detail in details)
                {
                    yield return ColliderDetail.Set(ctx, v, detail);
                    // the frame where the hull is now: a level's tows carry it a km or more round the planet, its up turned with it
                    up = v.upAxis;
                    h = Vector3.ProjectOnPlane(h, up).normalized;
                    string at = details.Length > 1 ? $" @{detail}" : "", sfx = details.Length > 1 ? $"-{detail}" : "";
                    setAlong = 0;
                    mass = RigidVessel.Mass(v);   // (more section points: a little more volume, a heavier tank)
                    // afloat at 2 deg, the step at about its draft at rest
                    v.SetRotation(Hold(2));
                    yield return new WaitForFixedUpdate();
                    double stepAlt = FlightGlobals.getAltitudeAtPos(hull.TransformPoint(stepKeel), body);
                    v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-0.6 * keelAtStep - stepAlt));
                    RigidVessel.Set(v, Vector3.zero, Vector3.zero);

                    var csv = new StringBuilder("C_V,V_ms,load_C_Delta,trim_deg,trim_sd,trim_fig4,draft_step_m,R_kN,C_R,water_up_kN,hydrostatic_up_kN\n");
                    // each strip at the end of each speed: its station from the step (m, + forward), width, load, separated
                    var strips = new StringBuilder("C_V,trim_deg,x_from_step_m,width_m,ma_kg_m,load_N,separated,jump_N\n");
                    FARAeroComponents.PartHydrodynamics.RecordStrips = true;
                    var fixedWait = new WaitForFixedUpdate();
                    TestWarp.Hyper(ctx, true);
                    foreach (double cv in cvs)
                    {
                        double V = cv * sqrtGb, cDelta = load0 * Math.Max(0, 1 - cv * cv / 100), delta = cDelta * wb3;
                        ctx.Step($"C_V {cv}");
                        double t0 = Planetarium.GetUniversalTime();
                        var trims = new List<double>(); var drafts = new List<double>(); var tows = new List<double>();
                        var wUp = new List<double>(); var hsUp = new List<double>();
                        while (Planetarium.GetUniversalTime() - t0 < settle + 4)
                        {
                            double dt = TimeWarp.fixedDeltaTime;
                            Vector3 vel = RigidVessel.Velocity(v), omega = RigidVessel.AngularVelocity(v);
                            Vector3 hWas = h;
                            double towKN = mass * (setAlong - Vector3.Dot(vel, hWas)) / dt;
                            double tau = Trim();
                            // roll and yaw held level, the pitch and its rate left as the water has them
                            Quaternion hold = Hold(tau);
                            Vector3d com = RigidVessel.CoM(v);
                            double g = -Vector3d.Dot(FlightGlobals.getGeeForceAtPosition(com) + FlightGlobals.getCentrifugalAcc(com, body), (Vector3d)up);
                            double lift = mass * g - delta;               // the counterweight and the hydrofoil, kN
                            double vUp = Vector3.Dot(vel, up) + lift / mass * dt;
                            RigidVessel.Rotate(v, hull.Ref, hold);
                            RigidVessel.Set(v, up * (float)vUp + h * (float)V, bowUp * Vector3.Dot(omega, bowUp));
                            setAlong = V;
                            if (Planetarium.GetUniversalTime() - t0 >= settle)
                            {
                                trims.Add(tau);
                                drafts.Add(-FlightGlobals.getAltitudeAtPos(hull.TransformPoint(stepKeel), body));
                                tows.Add(towKN);
                                wUp.Add(Vector3.Dot(hull.Sum(x => x.LastForce + x.LastHydrostaticForce), up));
                                hsUp.Add(Vector3.Dot(hull.Sum(x => x.LastHydrostaticForce), up));
                            }
                            yield return fixedWait;
                        }
                        foreach (var st in hull.StripData)
                            strips.AppendLine(string.Join(",", F(cv), F(Trim()), F(hull.InverseTransformPoint(st.point).y - stepY), F(st.width), F(st.ma),
                                                          F(st.load), st.separated ? "1" : "0", F(st.jump)));
                        double trim = trims.Average(), sd = Math.Sqrt(trims.Sum(x => (x - trim) * (x - trim)) / Math.Max(1, trims.Count - 1));
                        double fig = Fig4Trim(cv), R = tows.Average();
                        csv.AppendLine(string.Join(",", F(cv), F(V), F(cDelta), F(trim), F(sd), F(fig), F(drafts.Average()), F(R), F(R / wb3),
                                                   F(wUp.Average()), F(hsUp.Average())));
                        lodRuns[(detail, cv)] = (trim, R / wb3);
                        ctx.Record($"C_V {cv}{at}", $"trim {trim:F1} deg (sd {sd:F1}; Fig. 4 {(double.IsNaN(fig) ? "-" : fig.ToString("F1"))}), " +
                                                $"step draft {drafts.Average():F3} m, C_R {R / wb3:F3}, C_Delta {cDelta:F3}");
                        if (!double.IsNaN(fig) && Math.Abs(trim - fig) > tolerance)
                            wrong.Add($"{(at.Length > 0 ? $"{detail}: " : "")}C_V {cv}: {trim:F1} deg against {fig:F1}");
                    }
                    File.WriteAllText(Path.Combine(ctx.ResultsDir, $"naca57-free{sfx}.csv"), csv.ToString());
                    File.WriteAllText(Path.Combine(ctx.ResultsDir, $"naca57-strips{sfx}.csv"), strips.ToString());
                }
                // each level's trim within 1 deg and C_R within 10% of the finest's (the last level's)
                if (details.Length > 1)
                {
                    int finest = details[details.Length - 1];
                    var lodLines = new StringBuilder($"against {finest} points per ring: ");
                    var lodOff = new List<string>();
                    foreach (int d in details.Take(details.Length - 1))
                    {
                        lodLines.Append($"{d}:");
                        foreach (double cv in cvs.Where(c => c > 0))
                        {
                            var x = lodRuns[(d, cv)]; var f = lodRuns[(finest, cv)];
                            bool ok = Math.Abs(x.trim - f.trim) <= 1.0 && Math.Abs(x.cr - f.cr) <= 0.1 * Math.Abs(f.cr) + 0.002;
                            if (!ok) lodOff.Add($"{d} at C_V {cv}: trim {x.trim:F1} deg ({f.trim:F1}), C_R {x.cr:F3} ({f.cr:F3})");
                            lodLines.Append($" C_V {cv} trim {x.trim - f.trim:+0.0;-0.0} C_R {x.cr / f.cr:F2}x{(ok ? "" : " OFF")};");
                        }
                        lodLines.Append(" ");
                    }
                    ctx.Record("details", lodLines.ToString());
                    Assert.True(lodOff.Count == 0, "each collider detail's trim within 1 deg and C_R within 10% of the finest's: " + string.Join("; ", lodOff.Take(8)));
                }
                ctx.Record("off Fig. 4", wrong.Count == 0 ? "none" : string.Join("; ", wrong));
                Assert.True(wrong.Count == 0, $"free to trim within {tolerance} deg of TN 716 Fig. 4 ({string.Join("; ", wrong.Take(5))})");
            }
            finally
            {
                TestWarp.Hyper(ctx, false);
                FARAeroComponents.PartHydrodynamics.RecordStrips = false;
                CheatOptions.NoCrashDamage = crashWas;
                WaterParams.Restore();
            }
        }

        /// <summary>
        /// The float's damping (measure only): at each speed coefficient (cvs) settled free to trim as Tow_Naca57B5FreeToTrim
        /// does, then its mean height and trim held and one motion forced about them, a sine of period (s) and amplitude
        /// (pitch: amp deg about the C.G.; heave: ampHeave m of the C.G.), mode=pitch or heave. The water's moment about
        /// the C.G. (nose up +) and its up force over whole cycles fitted to a + b sin + c cos: the in-phase part the
        /// stiffness, the out-of-phase part the damping (kN m s/rad, kN s/m; positive: damping). Writes naca57-forced.csv.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.WaterMods,
                StartsAt = StartState.Flight, Craft = Float, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Oscillate_Naca57B5(TestContext ctx)
        {
            const double Inch = 2.128 * 12.0 / 12.47 * 0.0254, Beam = 12.47 * Inch;
            double stepY = -(52.0 - 47.0) * Inch, keelAtStep = 11.40 * Inch;
            double cgY = stepY + ctx.ParamDouble("cgAhead", 0.361) * Beam, cgAboveBase = (ctx.ParamDouble("cgHeight", 24.00) - 11.40) * Inch;
            double load0 = ctx.ParamDouble("load", 1.575), settle = ctx.ParamDouble("settle", 8);
            double period = ctx.ParamDouble("period", 2.5), ampDeg = ctx.ParamDouble("amp", 2), ampHeave = ctx.ParamDouble("ampHeave", 0.03);
            int cycles = (int)ctx.ParamDouble("cycles", 4);
            bool heave = ctx.Param("mode") == "heave";
            double[] cvs = ctx.ParamList("cvs", "2+3+4+5").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            WaterParams.Apply(ctx);

            yield return Wait.Seconds(5);
            Vessel v = FlightGlobals.ActiveVessel;
            foreach (PartResource r in v.parts.SelectMany(q => q.Resources.Cast<PartResource>()))
                r.amount = 0;
            yield return Wait.Frames(2);
            bool crashWas = CheatOptions.NoCrashDamage;
            try
            {
                CheatOptions.NoCrashDamage = true;
                v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                yield return FarFlightProbe.Place(Far.At(0, 600, 0, "naca57", speedMs: 5));
                v = FlightGlobals.ActiveVessel;
                CelestialBody body = v.mainBody;
                HullParts hull = HullParts.Whole(v);
                Vector3 up = v.upAxis;
                Vector3 h = Vector3.ProjectOnPlane(hull.Forward, up).normalized;
                Vector3 bowUp = Vector3.Cross(h, up).normalized;
                Vector3 keel = PlaningVideoTests.KeelAxis(hull.Ref);
                hull.SetCoM(v, new Vector3(0, (float)cgY, 0) - keel * (float)cgAboveBase);
                Vector3 stepKeel = new Vector3(0, (float)stepY, 0) + keel * (float)keelAtStep;
                double rho = body.oceanDensity * 1000, g0 = 9.80665 * body.GeeASL;
                double sqrtGb = Math.Sqrt(g0 * Beam), wb3 = rho * g0 * Beam * Beam * Beam / 1000;
                double mass = RigidVessel.Mass(v);
                ctx.Record("forced", $"{(heave ? $"heave {ampHeave} m" : $"pitch {ampDeg} deg")}, period {period} s, {cycles} cycles; mass {mass:F3} t, beam {Beam:F3} m");

                Quaternion Hold(double tau)
                {
                    up = (Vector3)FlightGlobals.getUpAxis(body, RigidVessel.CoM(v));
                    h = Vector3.ProjectOnPlane(h, up).normalized;
                    bowUp = Vector3.Cross(h, up).normalized;
                    Vector3 fwd = Quaternion.AngleAxis((float)tau, bowUp) * h;
                    Vector3 down = Vector3.Cross(fwd, bowUp).normalized;
                    return Quaternion.LookRotation(keel.z > 0 ? down : -down, fwd);
                }
                double Trim() => Math.Asin(Mathf.Clamp(Vector3.Dot(hull.Forward, up), -1, 1)) * 180 / Math.PI;
                double Alt() => FlightGlobals.getAltitudeAtPos((Vector3d)RigidVessel.CoM(v), body);
                // the water's moment about the C.G. (nose up +, kN m) and its force up (kN)
                (double m, double f) Water()
                {
                    Vector3 c = RigidVessel.CoM(v), mm = Vector3.zero, ff = Vector3.zero;
                    foreach (var x in hull.Hydro)
                    {
                        Vector3 r0 = x.part.rb.worldCenterOfMass - c;
                        mm += x.LastTorque + x.LastHydrostaticTorque + Vector3.Cross(r0, x.LastForce + x.LastHydrostaticForce);
                        ff += x.LastForce + x.LastHydrostaticForce;
                    }
                    return (Vector3.Dot(mm, bowUp), Vector3.Dot(ff, up));
                }

                v.SetRotation(Hold(2));
                yield return new WaitForFixedUpdate();
                double stepAlt = FlightGlobals.getAltitudeAtPos(hull.TransformPoint(stepKeel), body);
                v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-0.6 * keelAtStep - stepAlt));
                RigidVessel.Set(v, Vector3.zero, Vector3.zero);

                var csv = new StringBuilder("C_V,V_ms,trim_deg,draft_step_m,mode,K,B,B_theory_hint,a0,fit_r2\n");
                var trace = new StringBuilder("C_V,t,trim_deg,alt_m,moment_kNm,up_kN\n");
                var fixedWait = new WaitForFixedUpdate();
                TestWarp.Hyper(ctx, true);
                double w = 2 * Math.PI / period;
                foreach (double cv in cvs)
                {
                    double V = cv * sqrtGb, delta = load0 * Math.Max(0, 1 - cv * cv / 100) * wb3;
                    ctx.Step($"C_V {cv}: free");
                    double t0 = Planetarium.GetUniversalTime();
                    var trims = new List<double>(); var alts = new List<double>();
                    while (Planetarium.GetUniversalTime() - t0 < settle + 2)
                    {
                        double dt = TimeWarp.fixedDeltaTime;
                        Vector3 vel = RigidVessel.Velocity(v), omega = RigidVessel.AngularVelocity(v);
                        double tau = Trim();
                        Quaternion hold = Hold(tau);
                        Vector3d com = RigidVessel.CoM(v);
                        double g = -Vector3d.Dot(FlightGlobals.getGeeForceAtPosition(com) + FlightGlobals.getCentrifugalAcc(com, body), (Vector3d)up);
                        double vUp = Vector3.Dot(vel, up) + (mass * g - delta) / mass * dt;
                        RigidVessel.Rotate(v, hull.Ref, hold);
                        RigidVessel.Set(v, up * (float)vUp + h * (float)V, bowUp * Vector3.Dot(omega, bowUp));
                        if (Planetarium.GetUniversalTime() - t0 >= settle)
                        {
                            trims.Add(tau);
                            alts.Add(Alt());
                        }
                        yield return fixedWait;
                    }
                    double trim0 = trims.Average(), alt0 = alts.Average();
                    double draft0 = -FlightGlobals.getAltitudeAtPos(hull.TransformPoint(stepKeel), body);
                    // forced: the motion as asked, the other held (trim and height steered back to it each step)
                    ctx.Step($"C_V {cv}: forced");
                    t0 = Planetarium.GetUniversalTime();
                    var ts = new List<double>(); var ms = new List<double>(); var fs = new List<double>();
                    double total = (cycles + 1) * period;
                    while (Planetarium.GetUniversalTime() - t0 < total)
                    {
                        double t = Planetarium.GetUniversalTime() - t0 + TimeWarp.fixedDeltaTime;   // (the state the step will have)
                        double tauT = trim0 + (heave ? 0 : ampDeg * Math.Sin(w * t));
                        double qT = heave ? 0 : ampDeg * w * Math.Cos(w * t) * Math.PI / 180;
                        double altT = alt0 + (heave ? ampHeave * Math.Sin(w * t) : 0);
                        double vzT = heave ? ampHeave * w * Math.Cos(w * t) : 0;
                        double tau = Trim();
                        RigidVessel.Rotate(v, hull.Ref, Hold(tau));
                        double q = qT + 5 * (tauT - tau) * Math.PI / 180;
                        double vUp = vzT + 5 * (altT - Alt());
                        // the water's forces of the state just stepped (before this step's)
                        (double m, double f) wat = Water();
                        double tNow = Planetarium.GetUniversalTime() - t0;
                        if (tNow >= period)
                        {
                            ts.Add(tNow); ms.Add(wat.m); fs.Add(wat.f);
                        }
                        trace.AppendLine(string.Join(",", F(cv), F(tNow), F(tau), F(Alt() - alt0), F(wat.m), F(wat.f)));
                        RigidVessel.Set(v, up * (float)vUp + h * (float)V, bowUp * (float)q);
                        yield return fixedWait;
                    }
                    // fit y = a + b sin(w t') + c cos(w t'), t' the time of the state the forces were of (one step back)
                    double Fit(List<double> y, out double b, out double c, out double r2)
                    {
                        double dt = TimeWarp.fixedDeltaTime;
                        double[,] A = new double[3, 3]; double[] rhs = new double[3];
                        for (int i = 0; i < y.Count; i++)
                        {
                            double tt = ts[i] - dt;
                            double[] row = { 1, Math.Sin(w * tt), Math.Cos(w * tt) };
                            for (int j = 0; j < 3; j++)
                            {
                                rhs[j] += row[j] * y[i];
                                for (int k = 0; k < 3; k++)
                                    A[j, k] += row[j] * row[k];
                            }
                        }
                        double[] x = Solve(A, rhs);
                        double mean = y.Average(), ssT = 0, ssR = 0;
                        for (int i = 0; i < y.Count; i++)
                        {
                            double tt = ts[i] - dt;
                            double e = y[i] - (x[0] + x[1] * Math.Sin(w * tt) + x[2] * Math.Cos(w * tt));
                            ssR += e * e;
                            ssT += (y[i] - mean) * (y[i] - mean);
                        }
                        b = x[1];
                        c = x[2];
                        r2 = ssT > 0 ? 1 - ssR / ssT : 0;
                        return x[0];
                    }
                    double am = Fit(ms, out double bm, out double cm, out double r2m);
                    double af = Fit(fs, out double bf, out double cf, out double r2f);
                    string rec;
                    if (heave)
                    {
                        // heave z = A sin: the force's in-phase part -K z, its out-of-phase -B zdot (zdot = A w cos)
                        double K = -bf / ampHeave, B = -cf / (ampHeave * w), K53 = -bm / ampHeave, B53 = -cm / (ampHeave * w);
                        rec = $"heave: K33 {K:F1} kN/m, B33 {B:F2} kN s/m (R2 {r2f:F2}); moment K53 {K53:F1} kN m/m, B53 {B53:F2} kN m s/m (R2 {r2m:F2})";
                        csv.AppendLine(string.Join(",", F(cv), F(V), F(trim0), F(draft0), "heave", F(K), F(B), "", F(af), F(r2f)));
                        csv.AppendLine(string.Join(",", F(cv), F(V), F(trim0), F(draft0), "heave-moment", F(K53), F(B53), "", F(am), F(r2m)));
                    }
                    else
                    {
                        double A = ampDeg * Math.PI / 180;
                        double K = -bm / A, B = -cm / (A * w), K35 = -bf / A, B35 = -cf / (A * w);
                        rec = $"pitch: K55 {K:F2} kN m/rad, B55 {B:F3} kN m s/rad (R2 {r2m:F2}); force K35 {K35:F2} kN/rad, B35 {B35:F3} kN s/rad (R2 {r2f:F2})";
                        csv.AppendLine(string.Join(",", F(cv), F(V), F(trim0), F(draft0), "pitch", F(K), F(B), "", F(am), F(r2m)));
                        csv.AppendLine(string.Join(",", F(cv), F(V), F(trim0), F(draft0), "pitch-force", F(K35), F(B35), "", F(af), F(r2f)));
                    }
                    ctx.Record($"C_V {cv}", $"trim {trim0:F1} deg, step draft {draft0:F3} m, mean moment {am:F2} kN m, up {af:F2} kN; {rec}");
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "naca57-forced.csv"), csv.ToString());
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "naca57-forced-trace.csv"), trace.ToString());
            }
            finally
            {
                TestWarp.Hyper(ctx, false);
                CheatOptions.NoCrashDamage = crashWas;
                WaterParams.Restore();
            }
        }

        private static double[] Solve(double[,] a, double[] b)
        {
            int n = b.Length;
            var m = (double[,])a.Clone();
            var x = (double[])b.Clone();
            for (int i = 0; i < n; i++)
            {
                int p = i;
                for (int r = i + 1; r < n; r++)
                    if (Math.Abs(m[r, i]) > Math.Abs(m[p, i]))
                        p = r;
                for (int c = 0; c < n; c++)
                    (m[i, c], m[p, c]) = (m[p, c], m[i, c]);
                (x[i], x[p]) = (x[p], x[i]);
                for (int r = i + 1; r < n; r++)
                {
                    double f = m[r, i] / m[i, i];
                    for (int c = i; c < n; c++)
                        m[r, c] -= f * m[i, c];
                    x[r] -= f * x[i];
                }
            }
            for (int i = n - 1; i >= 0; i--)
            {
                for (int c = i + 1; c < n; c++)
                    x[i] -= m[i, c] * x[c];
                x[i] /= m[i, i];
            }
            return x;
        }

        // Fig. 4's trim at a speed coefficient (linear between its points; NaN outside them)
        private static double Fig4Trim(double cv)
        {
            if (cv < Fig4[0].cv - 1e-9 || cv > Fig4[Fig4.Length - 1].cv + 1e-9)
                return double.NaN;
            for (int i = 0; i + 1 < Fig4.Length; i++)
                if (cv <= Fig4[i + 1].cv + 1e-9)
                    return Fig4[i].trim + (cv - Fig4[i].cv) / (Fig4[i + 1].cv - Fig4[i].cv) * (Fig4[i + 1].trim - Fig4[i].trim);
            return Fig4[Fig4.Length - 1].trim;
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);
    }
}
