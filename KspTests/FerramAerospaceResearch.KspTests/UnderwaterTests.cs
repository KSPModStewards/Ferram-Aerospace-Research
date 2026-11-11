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
        /// even). Then yawed instead (yaws, deg, default -6+-3+3+6): a body of revolution has no up or down under the
        /// water, so its side force and yawing moment at a sideslip are its lift and pitching moment at that angle of
        /// attack (within 15%). Writes submerged.csv and submerged-yaw.csv; parameter submergedFade sets
        /// PartHydrodynamics.SubmergedFade (a huge value: the water's surface terms everywhere, as before they faded under it).
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Submerged Body", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1800)]
        public static IEnumerator Tow_SubmergedBody(TestContext ctx) =>
            TowBody(ctx, "0+2+4+6+8", "-6+-3+0+3+6", "-6+-3+3+6", 3, 6, 0, null, true);

        /// <summary>
        /// The DARPA SUBOFF bare hull at model scale (crafts/suboff.py: DTRC Model 5470 from Groves et al.'s equations)
        /// towed as Roddy towed it (DTRC/SHD-1298-08, 1990): wholly under the water (its middle depth m below, default 2)
        /// at 6.5 knots (3.344 m/s), pitched and yawed (default to 18 deg either way), its moments about his reference point
        /// 2.013 m aft of the nose (0.165 m ahead of its middle). Its derivatives on his basis (L 4.261 m, the length
        /// between perpendiculars; Y' = Y / (1/2 rho L^2 U^2), N' = N / (1/2 rho L^3 U^2), per radian of v/U) against his
        /// bare hull's (configuration 3, Table 4): Y'v -0.005948, N'v -0.012795 (a fit of the angles' Y' and N' to linear
        /// and |v|v terms), its forces at 18.11 deg drift, Y' 7.355e-3 and N' 2.986e-3, and its axial force X' -1.061e-3
        /// (Appendix B), each within 30%; its rotary derivatives (turned at rates, rotating-arm fashion: Y'r 0.001811,
        /// N'r -0.001597) recorded. Then the Tow_SubmergedBody checks (buoyancy kept, mirror images, yaw as pitch).
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "SUBOFF Bare Hull", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1800)]
        public static IEnumerator Tow_SuboffBareHull(TestContext ctx) =>
            TowBody(ctx, "0+3.344", "-18+-10+-4+-2+0+2+4+10+18", "-18+-10+-4+-2+2+4+10+18", 2, 0.6, 0.165,
                    (4.261, -0.005948, -0.012795, -1.061e-3, 18.11, 7.355e-3, 2.986e-3, 0.001811, -0.001597), true);

        /// <summary>
        /// Tow_SuboffBareHull's tow of the SUBOFF hull with its four stern appendages (crafts/suboff.py: B9 control
        /// surfaces held at zero, FAR's wing model in the water), against Roddy's configuration 5 (Table 4): Y'v
        /// -0.010494, N'v -0.011254, each within 30%: what the fins add to the bare hull's (its rotary derivatives, Y'r
        /// 0.006324 and N'r -0.003064, recorded). Its X' -1.158e-3 is the bare
        /// hull's times its tow-tank resistance over the bare hull's (Liu and Huang, DTRC/SHD-1298-09, Table 14: 21.45 lb
        /// against 19.65 at 5.92 knots). No large drift angle to meet.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "SUBOFF Hull Fins", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1800)]
        public static IEnumerator Tow_SuboffHullFins(TestContext ctx) =>
            TowBody(ctx, "0+3.344", "-10+-4+-2+0+2+4+10", "-10+-4+-2+2+4+10", 2, 0.6, 0.165,
                    (4.261, -0.010494, -0.011254, -1.061e-3 * 21.45 / 19.65, double.NaN, double.NaN, double.NaN, 0.006324, -0.003064), true);

        /// <summary>
        /// Tow_SuboffBareHull's tow of the SUBOFF hull with its sail (crafts/suboff.py: a B9 control surface held at
        /// zero), beside Roddy's configuration 4 (Table 4): Y'v -0.023008, N'v -0.015534, Y'r -0.000023, N'r -0.002378,
        /// recorded, not asserted: FAR gives the sail about 45% of the side force Roddy measured (its low aspect ratio's
        /// lift, none carried onto the hull), and its yaw damping is short. No pitch-yaw symmetry (the sail is on top).
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "SUBOFF Hull Sail", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1800)]
        public static IEnumerator Tow_SuboffHullSail(TestContext ctx) =>
            TowBody(ctx, "0+3.344", "-10+-4+-2+0+2+4+10", "-10+-4+-2+2+4+10", 2, 0.6, 0.165,
                    (4.261, -0.023008, -0.015534, double.NaN, double.NaN, double.NaN, double.NaN, -0.000023, -0.002378), false, false);

        // speeds, aoas and yaws (deg) its defaults; its CoM refForward (m) ahead of its middle, the moments' point;
        // reference: a captive-model test's derivatives on its own length (L, Y'v, N'v, X'), its Y' and N' at a large
        // drift angle (deg) and its rotary derivatives (Y'r, N'r: turned at rates, rotating-arm fashion) to meet;
        // symmetric: a body of revolution's yaw its pitch (no sail); assertReference: the reference met (else recorded)
        private static IEnumerator TowBody(TestContext ctx, string speedsDefault, string aoasDefault, string yawsDefault, double depthDefault,
                                           double massDefault, double refForward,
                                           (double L, double yv, double nv, double x, double drift, double yd, double nd, double yr, double nr)? reference,
                                           bool symmetric, bool assertReference = true)
        {
            double[] speeds = ctx.ParamList("speeds", speedsDefault).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            double[] aoas = ctx.ParamList("aoas", aoasDefault).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            double[] yaws = ctx.ParamList("yaws", yawsDefault).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            double depth = ctx.ParamDouble("depth", depthDefault), massT = ctx.ParamDouble("mass", massDefault);
            // nondimensional yaw rates r' = r L / U (turned on its course, its attitude held: a rotating arm's)
            double[] rates = ctx.ParamList("rates", reference != null ? "-0.3+-0.15+0.15+0.3" : "")
                .Where(x => x.Length > 0).Select(x => double.Parse(x, CultureInfo.InvariantCulture)).ToArray();
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
                HullParts hull = HullParts.Find(v, p => p.Modules.GetModule<ferram4.FARWingAerodynamicModel>() == null)[0];   // its foils (fins) not of it
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
                hull = HullParts.Find(v, p => p.Modules.GetModule<ferram4.FARWingAerodynamicModel>() == null)[0];
                CelestialBody body = v.mainBody;
                double rho = body.oceanDensity * 1000, g = body.GeeASL * 9.80665;
                Vector3 up = v.upAxis;
                Vector3 h = Vector3.ProjectOnPlane(hull.Forward, up).normalized;
                Vector3 side = Vector3.Cross(up, h).normalized;
                // its CoM at its middle (where a body of revolution's buoyancy acts), or at a reference's moment point
                hull.SetCoM(v, new Vector3(0, (float)refForward, 0));

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
                                // its foils' (fins'): FAR's wing model, its force at its aerodynamic center
                                foreach (Part q in v.parts)
                                {
                                    var w = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                                    if (w == null)
                                        continue;
                                    f += w.worldSpaceForce;
                                    m += Vector3.Cross((Vector3)w.AerodynamicCenter - com, w.worldSpaceForce);
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

                // yawed: the nose turned toward the right (side) by the angle, the flow along h as before
                var yawCsv = new StringBuilder("V_ms,yaw_deg,side_kN,yaw_kNm\n");
                var yawed = new Dictionary<(double, double), (double side, double yaw)>();
                Vector3 yawAxis = Vector3.Cross(h, side);   // a moment along it turns the nose toward the right
                foreach (double U in speeds)
                    foreach (double b in yaws)
                    {
                        Vector3 nose = h * (float)Math.Cos(b * Math.PI / 180) + side * (float)Math.Sin(b * Math.PI / 180);
                        Flight.PointAtAoA(0, h, up);
                        v.SetRotation(Quaternion.FromToRotation(h, nose) * v.transform.rotation);
                        yield return fixedWait;
                        v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-depth - v.altitude));
                        RigidVessel.Set(v, h * (float)U, Vector3.zero);
                        var sides = new List<double>(); var yms = new List<double>();
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
                                // its foils' (fins'): FAR's wing model, its force at its aerodynamic center
                                foreach (Part q in v.parts)
                                {
                                    var w = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                                    if (w == null)
                                        continue;
                                    f += w.worldSpaceForce;
                                    m += Vector3.Cross((Vector3)w.AerodynamicCenter - com, w.worldSpaceForce);
                                }
                                sides.Add(Vector3.Dot(f, side));
                                yms.Add(Vector3.Dot(m, yawAxis));
                            }
                            Flight.PointAtAoA(0, h, up);
                            v.SetRotation(Quaternion.FromToRotation(h, nose) * v.transform.rotation);
                            RigidVessel.Set(v, h * (float)U, Vector3.zero);
                        }
                        yawed[(U, b)] = (sides.Average(), yms.Average());
                        yawCsv.AppendLine(string.Join(",", F(U), F(b), F(sides.Average()), F(yms.Average())));
                    }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "submerged-yaw.csv"), yawCsv.ToString());

                // turning: straight and level on its course at the fastest speed, turning at r' (r = r' U / L about the
                // vertical, its nose toward the right for r' > 0), its attitude put back every step
                var turning = new Dictionary<double, (double side, double yaw)>();
                if (rates.Length > 0 && reference is { } rRef)
                {
                    double U = speeds.Max();
                    foreach (double rp in rates)
                    {
                        float omega = (float)(rp * U / rRef.L);
                        Flight.PointAtAoA(0, h, up);
                        yield return fixedWait;
                        v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-depth - v.altitude));
                        RigidVessel.Set(v, h * (float)U, up * omega);
                        var sides = new List<double>(); var yms = new List<double>();
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
                                foreach (Part q in v.parts)
                                {
                                    var w = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                                    if (w == null)
                                        continue;
                                    f += w.worldSpaceForce;
                                    m += Vector3.Cross((Vector3)w.AerodynamicCenter - com, w.worldSpaceForce);
                                }
                                sides.Add(Vector3.Dot(f, side));
                                yms.Add(Vector3.Dot(m, yawAxis));
                            }
                            Flight.PointAtAoA(0, h, up);
                            v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-depth - v.altitude));
                            RigidVessel.Set(v, h * (float)U, up * omega);
                        }
                        turning[rp] = (sides.Average(), yms.Average());
                    }
                }
                // its foils as FAR's wing model sees them
                var foils = v.parts.Select(q => q.Modules.GetModule<ferram4.FARWingAerodynamicModel>()).Where(w => w != null).ToList();
                if (foils.Count > 0)
                    ctx.Record("foils", $"wholeWingAspectRatio {FARDebugValues.wholeWingAspectRatio}: " + string.Join("; ", foils.Select(w =>
                        $"{Vector3.Dot(w.part.transform.position - hull.Position, up):+0.00;-0.00} up {Vector3.Dot(w.part.transform.position - hull.Position, side):+0.00;-0.00} right: " +
                        $"S {w.S:F4} m2, b/2 {w.b_2_actual:F3}, MAC {w.MAC_actual:F3}, AR {w.EffectiveAR:F2}")));
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

                // yaw against pitch: the flow's side force and yawing moment (at rest's taken off) beside its lift and
                // pitching moment at the same angle
                var yawLines = new StringBuilder();
                bool alike = true;
                int compared = 0;
                foreach (double U in speeds.Where(u => u > 0))
                    foreach (double b in yaws.Where(b => b > 0))
                        if (yawed.TryGetValue((U, b), out var yp) && yawed.TryGetValue((U, -b), out var yn) &&
                            yawed.TryGetValue((0, b), out var yp0) && yawed.TryGetValue((0, -b), out var yn0) &&
                            results.TryGetValue((U, b), out var p) && results.TryGetValue((U, -b), out var n) && results.TryGetValue((U, 0), out var c) &&
                            results.TryGetValue((0, b), out var p0) && results.TryGetValue((0, -b), out var n0) && results.TryGetValue((0, 0), out var c0))
                        {
                            // odd parts: (+ - -) / 2, rest taken off
                            double sideF = ((yp.side - yp0.side) - (yn.side - yn0.side)) / 2, yawM = ((yp.yaw - yp0.yaw) - (yn.yaw - yn0.yaw)) / 2;
                            double lift = ((p.up - p0.up) - (n.up - n0.up)) / 2, pitchM = ((p.pitch - p0.pitch) - (n.pitch - n0.pitch)) / 2;
                            bool ok = Math.Abs(sideF - lift) < 0.15 * Math.Abs(lift) + 0.05 && Math.Abs(yawM - pitchM) < 0.15 * Math.Abs(pitchM) + 0.05;
                            alike &= ok;
                            compared++;
                            yawLines.Append($"{U:F0} m/s {b:F0} deg: side {sideF:F2} kN (lift {lift:F2}), yaw {yawM:F2} kN m (pitch {pitchM:F2}){(ok ? "" : " OFF")}; ");
                        }
                ctx.Record("yaw", compared > 0 ? yawLines.ToString() : "(no yaw angle also pitched: none compared)");
                Assert.True(kept, "the buoyancy kept at every speed, level (within 3%)");
                Assert.True(odd, "the lift and pitching moment at +a and -a mirror images (within 15%)");

                // against a captive-model test, at the fastest speed: each yaw angle's odd part, fitted as
                // -Y' = Y'v s + Y'v|v| s^2 and -N' = N'v s + N'v|v| s^2 (s = sin of the yaw: v = -U s, the flow from its left
                // with its nose turned right), its linear derivatives and its forces at the test's large drift angle
                bool meets = true;
                if (reference is { } rf)
                {
                    double U = speeds.Max(), q = 0.5 * rho * U * U;
                    var refLines = new StringBuilder();
                    double s2 = 0, s3 = 0, s4 = 0, sy = 0, s2y = 0, sn = 0, s2n = 0;
                    foreach (double b in yaws.Where(b => b > 0))
                        if (yawed.TryGetValue((U, b), out var yp) && yawed.TryGetValue((U, -b), out var yn) &&
                            yawed.TryGetValue((0, b), out var yp0) && yawed.TryGetValue((0, -b), out var yn0))
                        {
                            double sb = Math.Sin(b * Math.PI / 180);
                            double y = -((yp.side - yp0.side) - (yn.side - yn0.side)) / 2 * 1000 / (q * rf.L * rf.L);
                            double nn = -((yp.yaw - yp0.yaw) - (yn.yaw - yn0.yaw)) / 2 * 1000 / (q * rf.L * rf.L * rf.L);
                            s2 += sb * sb; s3 += sb * sb * sb; s4 += sb * sb * sb * sb;
                            sy += sb * y; s2y += sb * sb * y; sn += sb * nn; s2n += sb * sb * nn;
                            refLines.Append($"{b:F0} deg: -Y' {y:E3}, -N' {nn:E3}; ");
                        }
                    double det = s2 * s4 - s3 * s3;
                    double yv = (sy * s4 - s2y * s3) / det, yvv = (s2 * s2y - s3 * sy) / det;
                    double nv = (sn * s4 - s2n * s3) / det, nvv = (s2 * s2n - s3 * sn) / det;
                    double sd = double.IsNaN(rf.drift) ? 0 : Math.Sin(rf.drift * Math.PI / 180);
                    double yd = -(yv * sd + yvv * sd * sd), nd = -(nv * sd + nvv * sd * sd);
                    double xp = results.TryGetValue((U, 0), out var z0) ? -z0.drag * 1000 / (q * rf.L * rf.L) : double.NaN;
                    // rotary: the odd part of each rate's side force and moment, a line through none
                    double yr = double.NaN, nr = double.NaN;
                    if (turning.Count > 0)
                    {
                        double rr = 0, ry = 0, rn = 0;
                        foreach (double rp in rates.Where(x => x > 0))
                            if (turning.TryGetValue(rp, out var tp) && turning.TryGetValue(-rp, out var tn))
                            {
                                double y = (tp.side - tn.side) / 2 * 1000 / (q * rf.L * rf.L), nn = (tp.yaw - tn.yaw) / 2 * 1000 / (q * rf.L * rf.L * rf.L);
                                rr += rp * rp; ry += rp * y; rn += rp * nn;
                                refLines.Append($"r' {rp:F2}: Y'r {y / rp:F6}, N'r {nn / rp:F6}; ");
                            }
                        yr = ry / rr;
                        nr = rn / rr;
                        refLines.Append($"rotary: Y'r {yr:F6} (test {rf.yr:F6}, {yr / rf.yr:F2}x), N'r {nr:F6} (test {rf.nr:F6}, {nr / rf.nr:F2}x); ");
                    }
                    refLines.Append($"fit: Y'v {yv:F6} (test {rf.yv:F6}, {yv / rf.yv:F2}x), Y'v|v| {yvv:F5}, N'v {nv:F6} (test {rf.nv:F6}, {nv / rf.nv:F2}x), " +
                                    $"N'v|v| {nvv:F5}; at {rf.drift:F2} deg Y' {yd:E3} (test {rf.yd:E3}, {yd / rf.yd:F2}x), N' {nd:E3} (test {rf.nd:E3}, {nd / rf.nd:F2}x); " +
                                    $"X' {xp:E3} (test {rf.x:E3}, {xp / rf.x:F2}x)");
                    ctx.Record("reference", refLines.ToString());
                    // (a reference too near none for a ratio is recorded only)
                    double Ratio(double a, double b) => Math.Abs(b) < 2e-4 ? double.NaN : a / b;
                    // the rotary derivatives recorded, not asserted (FAR's yaw damping is short of Roddy's: 0.6 - 0.8 of it)
                    meets = !assertReference || new[] { Ratio(yv, rf.yv), Ratio(nv, rf.nv), Ratio(xp, rf.x), Ratio(yd, rf.yd), Ratio(nd, rf.nd) }
                        .Where(r => !double.IsNaN(r)).All(r => Math.Abs(r - 1) < 0.3);
                }

                Assert.True(alike || !symmetric, "the side force and yawing moment at a sideslip its lift and pitching moment at that angle (within 15%)");
                Assert.True(meets, "Y'v, N'v, X' and the drift angle's Y' and N' within 30% of the captive-model test's");
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
