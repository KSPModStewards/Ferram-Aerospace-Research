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
    /// A flat-bottomed box towed on the water, for comparing FAR's water forces with planing-hull data (Savitsky's
    /// prismatic-hull method, NACA tank tests).
    /// </summary>
    public static class PlaningTests
    {
        private const string Box = "Planing Box";

        /// <summary>
        /// The Planing Box (one ProceduralParts square prism: beam 2 m, 8 m long, flat bottom) made 4.0 t (the tank alone is 3.5 t) with its CG
        /// 3.0 m ahead of the transom and 0.5 m above the bottom, towed like a towing-tank model: every physics step its
        /// velocity along the tow line is put back to the tow speed (a horizontal force through the CG), its sideways
        /// velocity and roll and yaw rates zeroed; it heaves and pitches freely. At each speed it settles 8 s, then 6 s
        /// are averaged: the tow force (mass times the speed lost per step), the air's share of it (FAR's API on the
        /// same attitude in air), trim, keel depth at the transom, buoyancy and FAR's vertical force.
        /// Writes planing.csv (a row a speed) and planing-steps.csv (every step).
        /// Parameters: mass (t), lcg (m from the transom), cgHeight (m above the bottom), speeds (m/s, + separated).
        /// </summary>
        [UITest(Kind = "Measure", StartsAt = StartState.Flight, Craft = Box, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 1500)]
        public static IEnumerator Tow_PlaningBox(TestContext ctx)
        {
            double massT = ctx.ParamDouble("mass", 4.0), lcg = ctx.ParamDouble("lcg", 3.0), cgHeight = ctx.ParamDouble("cgHeight", 0.5);
            double[] speeds = ctx.ParamList("speeds", "2+3+4+5+6+8+10+12+14+16+18+20").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            const double beam = 2, length = 8;

            yield return Wait.Seconds(5);
            Vessel v = FlightGlobals.ActiveVessel;
            Part hull = v.rootPart;
            Assert.True(v.parts.Count == 1, $"the box is one part ({v.parts.Count})");

            // mass: tank emptied, then filled with liquid fuel to the mass asked for
            foreach (PartResource r in hull.Resources)
                r.amount = 0;
            yield return Wait.Frames(2);
            double dry = hull.mass;
            PartResource lf = hull.Resources.Get("LiquidFuel");
            Assert.True(lf != null, "the box tank holds liquid fuel");
            double add = (massT - dry) / lf.info.density;
            Assert.True(add >= 0 && add <= lf.maxAmount, $"{massT} t reachable: dry {dry:F3} t, {lf.maxAmount:F0} units of {lf.info.density} t");
            lf.amount = add;
            yield return Wait.Frames(2);

            bool crashWas = CheatOptions.NoCrashDamage;
            try
            {
                CheatOptions.NoCrashDamage = true;
                v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                FarFlightProbe.Condition c = Far.At(0, 600, 0, "tow", speedMs: speeds[0]);
                yield return FarFlightProbe.Place(c);
                v = FlightGlobals.ActiveVessel;
                CelestialBody body = v.mainBody;
                Assert.True(body.ocean, "over the sea");
                double rho = body.oceanDensity * 1000;   // t/m3 -> kg/m3
                Rigidbody rb = hull.rb;

                // the hull's axes: bow is the part's +y (its long axis, nose of a craft built along the SPH);
                // its bottom is whichever of +-x, +-z points down
                Vector3 up = v.upAxis;
                Vector3 localDown = Quantize(hull.transform.InverseTransformDirection(-up));
                Vector3 h = Vector3.ProjectOnPlane(hull.transform.up, up).normalized;
                ctx.Record("hull", $"local down {localDown}, ocean density {body.oceanDensity} t/m3, g {body.GeeASL * 9.80665:F3} m/s2, " +
                                   $"mass {v.GetTotalMass():F3} t (dry {dry:F3}), C_Delta {v.GetTotalMass() * 1000 / (rho * beam * beam * beam):F3}");

                // CG: lcg from the transom (local y = -length/2), cgHeight above the bottom
                hull.CoMOffset = new Vector3(0, (float)(lcg - length / 2), 0) + localDown * (float)(beam / 2 - cgHeight);
                rb.centerOfMass = hull.CoMOffset;

                // level on the water at the tow speed, floating at its static draft
                double draft = v.GetTotalMass() * 1000 / (rho * beam * length);
                Flight.PointAtAoA(0, h, up);
                yield return new WaitForFixedUpdate();
                double centerAlt = beam / 2 - draft;
                v.SetPosition(v.vesselTransform.position + (Vector3d)up * (centerAlt - v.altitude));
                foreach (Part p in v.parts)
                {
                    p.rb.velocity = h * (float)speeds[0];
                    p.rb.angularVelocity = Vector3.zero;
                }

                var steps = new StringBuilder("V,t,tow_kN,air_kN,trim_deg,transomDepth_m,bowDepth_m,buoyUp_kN,farUp_kN,farAlong_kN,rbDrag,submerged,comOffsetY\n");
                var rows = new StringBuilder("V_ms,C_V,R_kN,R_sd,air_kN,water_kN,water_over_Delta,trim_deg,trim_sd,transomDepth_m,bowDepth_m,lambda_est,buoyUp_kN,farUp_kN,rbDrag,submerged\n");
                var fixedWait = new WaitForFixedUpdate();
                double g = body.GeeASL * 9.80665, weightKN = v.GetTotalMass() * g;
                double setAlong = speeds[0];
                Vector3 stern = new Vector3(0, (float)(-length / 2), 0) + localDown * (float)(beam / 2);
                Vector3 bow = new Vector3(0, (float)(length / 2), 0) + localDown * (float)(beam / 2);

                foreach (double V in speeds)
                {
                    ctx.Step($"tow at {V} m/s");
                    double t0 = Planetarium.GetUniversalTime();
                    var tow = new List<double>(); var air = new List<double>(); var trim = new List<double>(); var dT = new List<double>(); var dB = new List<double>();
                    var buoy = new List<double>(); var farUp = new List<double>(); var drag = new List<double>(); var sub = new List<double>();
                    int n = 0;
                    while (Planetarium.GetUniversalTime() - t0 < 14)
                    {
                        // after the physics step: what the tow line had to give back, then the tow speed restored
                        double dt = TimeWarp.fixedDeltaTime, m = v.GetTotalMass();
                        Vector3 vel = rb.velocity;
                        double along = Vector3.Dot(vel, h);
                        double towKN = m * (setAlong - along) / dt;
                        Vector3 side = Vector3.Cross(up, h).normalized;
                        Vector3 vert = up * Vector3.Dot(vel, up);
                        rb.velocity = vert + h * (float)V;
                        setAlong = V;
                        rb.angularVelocity = side * Vector3.Dot(rb.angularVelocity, side);

                        double t = Planetarium.GetUniversalTime() - t0;
                        Vector3 fwd = hull.transform.up;
                        double trimDeg = Math.Asin(Mathf.Clamp(Vector3.Dot(fwd, up), -1, 1)) * 180 / Math.PI;
                        double depthT = -FlightGlobals.getAltitudeAtPos(hull.transform.TransformPoint(stern), body);
                        double depthB = -FlightGlobals.getAltitudeAtPos(hull.transform.TransformPoint(bow), body);
                        double buoyUp = Vector3.Dot(hull.partBuoyancy != null ? hull.partBuoyancy.effectiveForce : Vector3.zero, up);
                        var aero = hull.Modules.GetModule<FARAeroComponents.FARAeroPartModule>();
                        double farUpKN = aero != null ? Vector3.Dot(aero.worldSpaceAeroForce, up) : double.NaN;
                        double farAlongKN = aero != null ? -Vector3.Dot(aero.worldSpaceAeroForce, h) : double.NaN;
                        double airKN = double.NaN;
                        if (n % 10 == 0)
                        {
                            FARAPI.CalculateVesselAeroForces(v, out Vector3 f, out _, h * (float)V, Math.Max(0, v.altitude));
                            airKN = -Vector3.Dot(f, h);
                        }

                        steps.AppendLine(string.Join(",", F(V), F(t), F(towKN), F(airKN), F(trimDeg), F(depthT), F(depthB), F(buoyUp), F(farUpKN), F(farAlongKN),
                                                     F(rb.drag), F(hull.submergedPortion), F(hull.CoMOffset.y)));
                        if (t >= 8)
                        {
                            tow.Add(towKN); trim.Add(trimDeg); dT.Add(depthT); dB.Add(depthB); buoy.Add(buoyUp); farUp.Add(farUpKN); drag.Add(rb.drag); sub.Add(hull.submergedPortion);
                            if (!double.IsNaN(airKN)) air.Add(airKN);
                        }
                        n++;
                        yield return fixedWait;
                    }

                    double R = tow.Average(), airMean = air.Count > 0 ? air.Average() : double.NaN, water = R - airMean, tr = trim.Average();
                    double lambda = tr > 0.5 ? Math.Max(0, dT.Average()) / Math.Sin(tr * Math.PI / 180) / beam : double.NaN;
                    rows.AppendLine(string.Join(",", F(V), F(V / Math.Sqrt(g * beam)), F(R), F(Sd(tow)), F(airMean), F(water), F(water / weightKN), F(tr), F(Sd(trim)),
                                                F(dT.Average()), F(dB.Average()), F(lambda), F(buoy.Average()), F(farUp.Average()), F(drag.Average()), F(sub.Average())));
                    ctx.Record($"V{V}", $"R {R:F2} kN (sd {Sd(tow):F2}), air {airMean:F2}, water/Delta {water / weightKN:F4}, trim {tr:F2} deg (sd {Sd(trim):F2}), " +
                                        $"transom {dT.Average():F2} m deep, bow {dB.Average():F2}, buoyancy up {buoy.Average():F1} kN, FAR up {farUp.Average():F1} kN of {weightKN:F1}");
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "planing.csv"), rows.ToString());
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "planing-steps.csv"), steps.ToString());
            }
            finally
            {
                CheatOptions.NoCrashDamage = crashWas;
            }
        }

        private static Vector3 Quantize(Vector3 d)
        {
            Vector3[] axes = { Vector3.right, Vector3.left, Vector3.forward, Vector3.back };
            return axes.OrderByDescending(a => Vector3.Dot(a, d)).First();
        }

        private static double Sd(List<double> x)
        {
            if (x.Count < 2) return double.NaN;
            double m = x.Average();
            return Math.Sqrt(x.Sum(y => (y - m) * (y - m)) / (x.Count - 1));
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);
    }
}
