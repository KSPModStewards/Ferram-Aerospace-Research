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
    /// FAR's water model on what goes under the water rather than over it: a submarine's hull (crafts/underwater.py
    /// body) and a rudder or keel (crafts/underwater.py rudder), each held captive (its velocity and rotation set every
    /// physics step, as the tow tests hold their hulls) at the speeds and angles asked for.
    /// </summary>
    public static class UnderwaterTests
    {
        // the floatplane tests' (one instance for both): CustomProceduralParts for the Spine body
        private const string Mods = @"AdvancedJetEngine;Firespitter;TimeControl;..\..\..\CustomProceduralParts\GameData\CustomProceduralParts;" +
                                    @"..\FerramAerospaceResearch.KspTests.Run\mods\FARTestFloatplane";

        /// <summary>
        /// The Submerged Body (an 8 m by 1.2 m body of revolution) held level and pitched, its center depth m (default 3)
        /// below the water, at each of speeds (m/s, default 0+2+4+6+8) and pitch angles (deg, aoas, default -6+-3+0+3+6).
        /// A hull wholly under the water is no planing hull: its buoyancy stays the displaced water's weight at every
        /// speed, and its forces at +a and -a are mirror images (its lift and pitching moment odd in the angle, its drag
        /// even). Writes submerged.csv; parameter submergedFade sets PartHydrodynamics.SubmergedFade (a huge value: the
        /// water's surface terms everywhere, as before they faded under it).
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Submerged Body", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1800)]
        public static IEnumerator Tow_SubmergedBody(TestContext ctx)
        {
            double[] speeds = ctx.ParamList("speeds", "0+2+4+6+8").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            double[] aoas = ctx.ParamList("aoas", "-6+-3+0+3+6").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            double depth = ctx.ParamDouble("depth", 3), massT = ctx.ParamDouble("mass", 6);
            FieldInfo fadeField = typeof(FARAeroComponents.PartHydrodynamics).GetField("SubmergedFade", BindingFlags.Public | BindingFlags.Static);
            object fadeWas = fadeField?.GetValue(null);
            if (fadeField != null && !double.IsNaN(ctx.ParamDouble("submergedFade", double.NaN)))
                fadeField.SetValue(null, ctx.ParamDouble("submergedFade", 0.5));
            ctx.Record("model", fadeField == null ? "no SubmergedFade (FAR before the underwater terms)" : $"SubmergedFade {fadeField.GetValue(null)}");
            WaterParams.Apply(ctx);
            bool crashWas = CheatOptions.NoCrashDamage;
            try
            {
                yield return Wait.Seconds(5);
                Vessel v = FlightGlobals.ActiveVessel;
                HullParts hull = HullParts.Whole(v);
                foreach (PartResource r in hull.Parts.SelectMany(q => q.Resources.Cast<PartResource>()))
                    r.amount = 0;
                yield return Wait.Frames(2);
                PartResource lf = hull.Ref.Resources.Get("LiquidFuel");
                double dry = hull.Parts.Sum(q => q.mass);
                if (lf != null)
                    lf.amount = Math.Max(0, Math.Min(lf.maxAmount, (massT - dry) / lf.info.density));
                yield return Wait.Frames(2);

                CheatOptions.NoCrashDamage = true;
                v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                yield return FarFlightProbe.Place(Far.At(0, 600, 0, "submerged", speedMs: 0));
                v = FlightGlobals.ActiveVessel;
                hull = HullParts.Whole(v);
                CelestialBody body = v.mainBody;
                double rho = body.oceanDensity * 1000, g = body.GeeASL * 9.80665;
                Vector3 up = v.upAxis;
                Vector3 h = Vector3.ProjectOnPlane(hull.Forward, up).normalized;
                Vector3 side = Vector3.Cross(up, h).normalized;
                hull.SetCoM(v, Vector3.zero);    // its CoM at its middle, where its buoyancy acts (a body of revolution)

                var csv = new StringBuilder("V_ms,aoa_deg,up_kN,lift_kN,drag_kN,pitch_kNm,surface,buoyancy_kN,submerged\n");
                var results = new Dictionary<(double, double), (double up, double drag, double pitch)>();
                double buoyancyKN = double.NaN, surface = double.NaN;
                var fixedWait = new WaitForFixedUpdate();
                foreach (double U in speeds)
                    foreach (double aoa in aoas)
                    {
                        Flight.PointAtAoA(aoa, h, up);
                        yield return fixedWait;
                        v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-depth - v.altitude));
                        RigidVessel.Set(v, h * (float)U, Vector3.zero);
                        var ups = new List<double>(); var drags = new List<double>(); var pitches = new List<double>();
                        double t0 = Planetarium.GetUniversalTime();
                        while (Planetarium.GetUniversalTime() - t0 < 2.5)
                        {
                            yield return fixedWait;
                            if (Planetarium.GetUniversalTime() - t0 > 1)
                            {
                                Vector3 f = hull.Sum(x => x.LastForce + x.LastHydrostaticForce) + hull.StockBuoyancy;
                                Vector3 com = RigidVessel.CoM(v), m = Vector3.zero;
                                foreach (Part q in hull.Parts)
                                {
                                    var hq = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                                    if (hq == null || q.rb == null)
                                        continue;
                                    m += hq.LastTorque + hq.LastHydrostaticTorque +
                                         Vector3.Cross(q.rb.worldCenterOfMass - com, hq.LastForce + hq.LastHydrostaticForce);
                                }
                                ups.Add(Vector3.Dot(f, up));
                                drags.Add(-Vector3.Dot(f, h));
                                pitches.Add(-Vector3.Dot(m, side));     // nose up +
                                if (double.IsNaN(surface))
                                {
                                    object hr = hull.Hydro.FirstOrDefault()?.Hull;
                                    FieldInfo sf = hr?.GetType().GetField("Surface");
                                    surface = sf != null ? (double)sf.GetValue(hr) : -1;
                                }
                            }
                            Flight.PointAtAoA(aoa, h, up);
                            RigidVessel.Set(v, h * (float)U, Vector3.zero);
                        }
                        if (double.IsNaN(buoyancyKN))
                            buoyancyKN = rho * g * hull.MeshVolume / 1000;
                        double upKN = ups.Average(), dragKN = drags.Average(), pitch = pitches.Average();
                        results[(U, aoa)] = (upKN, dragKN, pitch);
                        csv.AppendLine(string.Join(",", F(U), F(aoa), F(upKN), F(upKN - buoyancyKN), F(dragKN), F(pitch), F(surface), F(buoyancyKN),
                                                   F(hull.Submerged)));
                    }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "submerged.csv"), csv.ToString());
                ctx.Record("body", $"volume {hull.MeshVolume:F3} m3: buoyancy {buoyancyKN:F2} kN; surface factor {surface:F2} (0: wholly submerged)");

                // buoyancy kept at speed; lift and pitching moment odd in the angle
                var lines = new StringBuilder();
                bool kept = true, odd = true;
                foreach (double U in speeds)
                {
                    if (results.TryGetValue((U, 0), out var z))
                    {
                        double share = z.up / buoyancyKN;
                        kept &= Math.Abs(share - 1) < 0.03;
                        lines.Append($"{U:F0} m/s: up {share:P1} of the buoyancy at 0 deg, drag {z.drag:F2} kN; ");
                    }
                    // the flow's share: each angle's value at rest (the hydrostatics: the CoM is not at the center of
                    // buoyancy, so pitching it turns the buoyancy's moment, alike either way) taken off
                    if (U == 0)
                        continue;
                    foreach (double a in aoas.Where(a => a > 0))
                        if (results.TryGetValue((U, a), out var p) && results.TryGetValue((U, -a), out var n) && results.TryGetValue((U, 0), out var c) &&
                            results.TryGetValue((0, a), out var p0) && results.TryGetValue((0, -a), out var n0) && results.TryGetValue((0, 0), out var c0))
                        {
                            double lp = (p.up - p0.up) - (c.up - c0.up), ln = (n.up - n0.up) - (c.up - c0.up);
                            double mp = (p.pitch - p0.pitch) - (c.pitch - c0.pitch), mn = (n.pitch - n0.pitch) - (c.pitch - c0.pitch);
                            double tolL = 0.15 * Math.Max(Math.Abs(lp), Math.Abs(ln)) + 0.05, tolM = 0.15 * Math.Max(Math.Abs(mp), Math.Abs(mn)) + 0.05;
                            odd &= Math.Abs(lp + ln) < tolL && Math.Abs(mp + mn) < tolM;
                            lines.Append($"+-{a:F0} deg lift {lp:F2}/{ln:F2} kN, moment {mp:F2}/{mn:F2} kN m; ");
                        }
                }
                ctx.Record("results", lines.ToString());
                Assert.True(kept, "the buoyancy kept at every speed, level (within 3%)");
                Assert.True(odd, "the lift and pitching moment at +a and -a mirror images (within 15%)");
            }
            finally
            {
                CheatOptions.NoCrashDamage = crashWas;
                WaterParams.Restore();
                if (fadeField != null)
                    fadeField.SetValue(null, fadeWas);
            }
        }

        /// <summary>
        /// The Rudder Rig's tail fin (a stock FAR control surface, its span down) held fully under the water at speed
        /// (m/s, default 3), at each yaw angle (yaws, deg, default -10+-5+0+5+10) and then straight with full rudder each
        /// way: its side force as a lift coefficient on FAR's area for it, against FAR's own wing model for the same fin
        /// at the same velocity, angle and control deflection in air (FARWingAerodynamicModel.CalculateForces at sea
        /// level's density). A foil in the water is a foil: its lift slope and its rudder's power the air's. Writes
        /// rudder.csv.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Rudder Rig", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1800)]
        public static IEnumerator Tow_Rudder(TestContext ctx)
        {
            double U = ctx.ParamDouble("speed", 3), depth = ctx.ParamDouble("depth", 4);
            double[] yaws = ctx.ParamList("yaws", "-10+-5+0+5+10").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            WaterParams.Apply(ctx);
            bool crashWas = CheatOptions.NoCrashDamage;
            try
            {
                yield return Wait.Seconds(5);
                Vessel v = FlightGlobals.ActiveVessel;
                Part fin = v.parts.First(q => q.partInfo.name == "tailfin");
                var wing = fin.Modules.GetModule<ferram4.FARControllableSurface>();
                Assert.True(wing != null, "the fin is a FAR control surface");
                CheatOptions.NoCrashDamage = true;
                v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                Ctrl.Attach();

                yield return FarFlightProbe.Place(Far.At(0, 600, 0, "rudder", speedMs: 0));
                v = FlightGlobals.ActiveVessel;
                CelestialBody body = v.mainBody;
                double rhoWater = body.oceanDensity * 1000, rhoAir = body.GetDensity(body.GetPressure(0), body.GetTemperature(0));
                double q = 0.5 * rhoWater * U * U;
                Vector3 up = v.upAxis;
                Vector3 h = Vector3.ProjectOnPlane(v.vesselTransform.up, up).normalized;
                Vector3 sideW = Vector3.Cross(up, h).normalized;
                var fixedWait = new WaitForFixedUpdate();
                Flight.PointAtAoA(0, h, up);
                yield return fixedWait;
                // the fin down (its span: from the probe core to it)
                if (Vector3.Dot((fin.transform.position - v.rootPart.transform.position).normalized, up) > 0)
                {
                    v.SetRotation(Quaternion.AngleAxis(180, h) * v.transform.rotation);
                    yield return fixedWait;
                }

                var csv = new StringBuilder("V_ms,q_kPa,yaw_deg,rudder,side_kN,CL_water,CL_air_model,submerged\n");
                var water = new Dictionary<(double, int), double>();
                var air = new Dictionary<(double, int), double>();
                var cases = yaws.Select(y => (yaw: y, rudder: 0)).Concat(new[] { (yaw: 0.0, rudder: 1), (yaw: 0.0, rudder: -1) }).ToArray();
                foreach (var c in cases)
                {
                    Vector3 dir = Quaternion.AngleAxis((float)c.yaw, up) * h;
                    Flight.PointAtAoA(0, dir, up);
                    yield return fixedWait;
                    v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-depth - v.altitude));
                    double mass = RigidVessel.Mass(v) * 1000;
                    Ctrl.Yaw = c.rudder;
                    RigidVessel.Set(v, h * (float)U, Vector3.zero);
                    var sides = new List<double>();
                    double t0 = Planetarium.GetUniversalTime();
                    while (Planetarium.GetUniversalTime() - t0 < 3)
                    {
                        yield return fixedWait;
                        // the side force: the momentum it put on the rig across the flow since the last step's hold
                        double vs = Vector3.Dot(RigidVessel.Velocity(v), sideW);
                        if (Planetarium.GetUniversalTime() - t0 > 1.5)
                            sides.Add(mass * vs / TimeWarp.fixedDeltaTime / 1000);
                        Flight.PointAtAoA(0, dir, up);
                        RigidVessel.Set(v, h * (float)U, Vector3.zero);
                    }
                    // FAR's wing model for the fin as it is now (its angle, its deflection) in air at sea level
                    Vector3d velocity = (Vector3d)(h * (float)U);
                    Vector3d fAir = wing.CalculateForces(velocity, 0, wing.CalculateAoA(velocity), rhoAir, false);
                    double clAir = Vector3d.Dot(fAir, (Vector3d)sideW) / (0.0005 * rhoAir * U * U * wing.S);
                    Ctrl.Yaw = 0;
                    double side = sides.Average(), clWater = side / (q / 1000 * wing.S);
                    water[(c.yaw, c.rudder)] = clWater;
                    air[(c.yaw, c.rudder)] = clAir;
                    csv.AppendLine(string.Join(",", F(U), F(q / 1000), F(c.yaw), c.rudder.ToString(CultureInfo.InvariantCulture), F(side), F(clWater), F(clAir),
                                               F(fin.submergedPortion)));
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "rudder.csv"), csv.ToString());

                double maxYaw = yaws.Max();
                double Slope(Dictionary<(double, int), double> d) => (d[(maxYaw, 0)] - d[(-maxYaw, 0)]) / (2 * maxYaw);
                double Rudder(Dictionary<(double, int), double> d) => (d[(0, 1)] - d[(0, -1)]) / 2;
                double sw = Slope(water), sa = Slope(air), rw = Rudder(water), ra = Rudder(air);
                ctx.Record("fin", $"FAR's area {wing.S:F3} m2, MAC {wing.MAC:F3} m, semispan {wing.b_2:F3} m, max deflection {wing.maxdeflect} deg, " +
                                  $"submerged {fin.submergedPortion:F2}");
                ctx.Record("lift slope", $"water {sw:F4} / deg, FAR's in air {sa:F4} / deg (water/air {sw / sa:F2})");
                ctx.Record("rudder", $"full rudder CL water {rw:F3}, FAR's in air {ra:F3} (water/air {rw / ra:F2})");
                Assert.True(Math.Abs(sw / sa - 1) < 0.25, $"the fin's lift slope in the water the air's (within 25%): {sw / sa:F2}");
                Assert.True(Math.Abs(rw / ra - 1) < 0.25, $"the rudder's power in the water the air's (within 25%): {rw / ra:F2}");
            }
            finally
            {
                Ctrl.Yaw = 0;
                CheatOptions.NoCrashDamage = crashWas;
                WaterParams.Restore();
            }
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);
    }
}
