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

                // afloat at 2 deg, the step at about its draft at rest
                v.SetRotation(Hold(2));
                yield return new WaitForFixedUpdate();
                double stepAlt = FlightGlobals.getAltitudeAtPos(hull.TransformPoint(stepKeel), body);
                v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-0.6 * keelAtStep - stepAlt));
                RigidVessel.Set(v, Vector3.zero, Vector3.zero);

                var csv = new StringBuilder("C_V,V_ms,load_C_Delta,trim_deg,trim_sd,trim_fig4,draft_step_m,R_kN,C_R,water_up_kN,hydrostatic_up_kN\n");
                var wrong = new List<string>();
                // each strip at the end of each speed: its station from the step (m, + forward), width, load, separated
                var strips = new StringBuilder("C_V,trim_deg,x_from_step_m,width_m,ma_kg_m,load_N,separated,jump_N\n");
                FARAeroComponents.PartHydrodynamics.RecordStrips = true;
                var fixedWait = new WaitForFixedUpdate();
                TestWarp.Hyper(ctx, true);
                double setAlong = 0;
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
                    ctx.Record($"C_V {cv}", $"trim {trim:F1} deg (sd {sd:F1}; Fig. 4 {(double.IsNaN(fig) ? "-" : fig.ToString("F1"))}), " +
                                            $"step draft {drafts.Average():F3} m, C_R {R / wb3:F3}, C_Delta {cDelta:F3}");
                    if (!double.IsNaN(fig) && Math.Abs(trim - fig) > tolerance)
                        wrong.Add($"C_V {cv}: {trim:F1} deg against {fig:F1}");
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "naca57-free.csv"), csv.ToString());
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "naca57-strips.csv"), strips.ToString());
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
