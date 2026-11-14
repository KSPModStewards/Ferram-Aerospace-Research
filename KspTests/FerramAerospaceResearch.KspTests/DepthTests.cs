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
    /// Each water test craft (the test boat, the C172 floatplane, the test submarine) held level and towed at a speed (m/s,
    /// default 5) at depths from its lowest point just above the water (0.1 m) to its highest 5 m below it, in steps
    /// (default 12), hyper-warped. At each: its drag (the momentum the tow loses to the air and the water in a step), the
    /// air's share of it (FAR's aero forces on the parts, a wing's as far as it is in the air: the wing model's density
    /// is the air's and the water's by its submerged share) and the water's (the rest). Going deeper, more of it is in
    /// the water and less in the air: its drag and the water's never fall, the air's never rises (each within 5% of its
    /// largest), and at the deepest the water's is more and the air's less than at the first. An edge between the air
    /// and the water models, a section crossing the surface or a part going under, shows as a step the wrong way.
    /// Writes depth-&lt;craft&gt;.csv.
    /// </summary>
    public static class DepthTests
    {
        private const string Mods = TestWarp.WaterMods;

        [UITest(Kind = "Check", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Test Boat", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Depth_TestBoat(TestContext ctx) => Sweep(ctx);

        [UITest(Kind = "Check", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Cessna 172M Floatplane Hinged", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Depth_C172Floatplane(TestContext ctx) => Sweep(ctx);

        [UITest(Kind = "Check", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Test Submarine", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Depth_TestSubmarine(TestContext ctx) => Sweep(ctx);

        private static IEnumerator Sweep(TestContext ctx)
        {
            double U = ctx.ParamDouble("speed", 5), below = ctx.ParamDouble("below", 5), above = ctx.ParamDouble("above", 0.1);
            int steps = (int)ctx.ParamDouble("steps", 12);
            double tolerance = ctx.ParamDouble("tolerance", 0.05);
            WaterParams.Apply(ctx);
            bool crashWas = CheatOptions.NoCrashDamage, failuresWas = FARDebugValues.allowStructuralFailures;
            try
            {
                yield return Wait.Seconds(5);
                CheatOptions.NoCrashDamage = true;
                // its drag, not its strength: a wing dipped at speed carries the water's loads, past what FAR lets it
                // (a C172's tore off at 5 m/s)
                FARDebugValues.allowStructuralFailures = false;
                Vessel v = FlightGlobals.ActiveVessel;
                v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                yield return FarFlightProbe.Place(Far.At(0, 600, 0, "depth", speedMs: 0));
                v = FlightGlobals.ActiveVessel;
                CelestialBody body = v.mainBody;
                Vector3 up = v.upAxis;
                Vector3 h = Vector3.ProjectOnPlane(v.ReferenceTransform.up, up).normalized;
                var fixedWait = new WaitForFixedUpdate();

                // its height level: its colliders' lowest and highest points against its own altitude
                Flight.PointAtAoA(0, h, up);
                RigidVessel.Set(v, Vector3.zero, Vector3.zero);
                yield return fixedWait;
                (double lo, double hi) = Extent(v);
                double height = hi - lo, below0 = v.altitude - lo;   // its altitude above its lowest point
                int partsAtStart = v.parts.Count;
                ctx.Record("craft", $"{v.vesselName}: {v.parts.Count} parts, {RigidVessel.Mass(v):F3} t, {height:F2} m high level; " +
                                    $"towed at {U:F1} m/s from its bottom {above:F1} m above the water to its top {below:F1} m below");
                TestWarp.Hyper(ctx, true);

                var csv = new StringBuilder("bottom_depth_m,top_depth_m,drag_kN,air_kN,water_kN,hydro_kN,momentum_kN,crossflow_kN,friction_kN," +
                                            "impact_kN,wings_kN,body_aero_kN,hydrostatic_kN,stock_buoyancy_kN,surface,submerged\n");
                var rows = new List<(double depth, double drag, double air, double water)>();
                var topUnder = new List<bool>();
                var surfaces = new List<double>();
                var unders = new List<double>();     // the hulls' planform under the water (m2): theirs times 1 - their surface factor
                var pressureWrong = new List<string>();
                // where it is held now (its altitude, m): it sinks from one depth to the next at sinkRate (m/s), its
                // velocity down with it, not put there at once (a part moved into the water in a step splashes, to KSP's
                // PartBuoyancy, at the jump over the step: tens of km/s, and is destroyed)
                double sinkRate = ctx.ParamDouble("sinkRate", 0.5), cur = below0 + above;
                for (int i = 0; i < steps; i++)
                {
                    // the depth of its lowest point: from above (negative) to its height and below deeper
                    double depth = -above + (height + below + above) * i / (steps - 1);
                    double alt = below0 - depth, vz = 0;
                    void Hold()
                    {
                        Flight.PointAtAoA(0, h, up);
                        v.SetPosition(v.vesselTransform.position + (Vector3d)up * (cur - v.altitude));
                        RigidVessel.Set(v, h * (float)U + up * (float)vz, Vector3.zero);
                    }
                    double flex = 0;
                    string flexPart = "";
                    // the largest of each force on any part over the step (kN): its water pressure (hydrostatic), the water
                    // model's flow forces, FAR's wing model, FAR's body aero
                    var peak = new (double kN, string part)[4];
                    (double kN, string part) stockPeak = (0, "");
                    void Flex()
                    {
                        foreach (Part q in v.parts)
                        {
                            var aero = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>();
                            var hq = aero?.Hydrodynamics;
                            var wing = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                            double[] f =
                            {
                                hq?.LastHydrostaticForce.magnitude ?? 0, hq?.LastForce.magnitude ?? 0, wing?.worldSpaceForce.magnitude ?? 0,
                                aero != null && !float.IsNaN(aero.worldSpaceAeroForce.x) ? aero.worldSpaceAeroForce.magnitude : 0
                            };
                            for (int k = 0; k < 4; k++)
                                if (f[k] > peak[k].kN)
                                    peak[k] = (f[k], q.partInfo.name);
                            double sb = q.partBuoyancy != null ? q.partBuoyancy.lastBuoyantForce.magnitude : 0;
                            if (sb > stockPeak.kN)
                                stockPeak = (sb, $"{q.partInfo.name} {q.rb?.mass * 1000:F1} kg under {q.submergedPortion:F2} buoyancy {q.buoyancy:F3}, " +
                                                 $"FAR water model {(hq == null ? "none" : hq.Hull == null ? "not solved" : "solving")}, " +
                                                 $"owns {hq?.OwnsBuoyancy}, FAR meshes {(q.Modules.GetModule<FARPartGeometry.GeometryPartModule>()?.meshDataList?.Count ?? -1)}");
                        }
                        foreach (Part q in v.parts)
                            if (q.rb != null)
                            {
                                double s = (q.rb.velocity - (h * (float)U + up * (float)vz)).magnitude;
                                if (s > flex)
                                {
                                    flex = s;
                                    var hq = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                                    var wing = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                                    flexPart = $"{q.partInfo.name} {q.rb.mass * 1000:F2} kg, under {q.submergedPortion:F2}, " +
                                               $"stock buoyancy {(q.partBuoyancy != null ? q.partBuoyancy.lastBuoyantForce.magnitude : 0):F3} kN " +
                                               $"(buoyancy {q.buoyancy:F2}), rb.drag {q.rb.drag:F1}, angularDrag {q.rb.angularDrag:F1}, " +
                                               $"hydrostatic {hq?.LastHydrostaticForce.magnitude ?? 0:F3} kN, flow {hq?.LastForce.magnitude ?? 0:F3}, " +
                                               $"wing {wing?.worldSpaceForce.magnitude ?? 0:F3}, owns buoyancy {hq?.OwnsBuoyancy}";
                                }
                            }
                    }
                    while (Math.Abs(cur - alt) > 1e-4)
                    {
                        double move = Math.Min(Math.Abs(alt - cur), sinkRate * TimeWarp.fixedDeltaTime);
                        vz = Math.Sign(alt - cur) * sinkRate;
                        cur += Math.Sign(alt - cur) * move;
                        Hold();
                        yield return fixedWait;
                        Flex();
                    }
                    vz = 0;
                    Hold();
                    yield return fixedWait;
                    Hold();
                    double mass = RigidVessel.Mass(v) * 1000;
                    var drags = new List<double>(); var airs = new List<double>(); var hydros = new List<double>();
                    var terms = new double[9];   // the drag of: momentum, crossflow, friction, impact; the wings'; the bodies' aero;
                                                 // the water's pressure, stock's buoyancy; the hull's surface factor
                    int n = 0;
                    double t0 = Planetarium.GetUniversalTime();
                    while (Planetarium.GetUniversalTime() - t0 < 2.5)
                    {
                        yield return fixedWait;
                        Flex();
                        if (Planetarium.GetUniversalTime() - t0 > 1.5)
                        {
                            // its drag: the speed along the tow it lost in the step since it was held at U
                            double vh = Vector3.Dot(RigidVessel.Velocity(v), h);
                            drags.Add(-mass * (vh - U) / TimeWarp.fixedDeltaTime / 1000);
                            airs.Add(AirDrag(v, h));
                            hydros.Add(-v.parts.Select(q => q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics)
                                              .Where(hq => hq != null).Sum(hq => Vector3.Dot(hq.LastForce, h)));
                            foreach (Part q in v.parts)
                            {
                                var aero = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>();
                                var hq = aero?.Hydrodynamics;
                                if (hq != null)
                                {
                                    terms[0] -= Vector3.Dot(hq.LastMomentumForce, h);
                                    terms[1] -= Vector3.Dot(hq.LastCrossflowForce, h);
                                    terms[2] -= Vector3.Dot(hq.LastFrictionForce, h);
                                    terms[3] -= Vector3.Dot(hq.LastImpactForce, h);
                                }
                                var wing = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                                if (wing != null)
                                    terms[4] -= Vector3.Dot(wing.worldSpaceForce, h);
                                if (aero != null && !float.IsNaN(aero.worldSpaceAeroForce.x))
                                    terms[5] -= Vector3.Dot(aero.worldSpaceAeroForce, h);
                                if (hq != null)
                                    terms[6] -= Vector3.Dot(hq.LastHydrostaticForce, h);
                                if (q.partBuoyancy != null)
                                    terms[7] -= Vector3.Dot(q.partBuoyancy.lastBuoyantForce, h);
                            }
                            terms[8] += Surface(v);
                            n++;
                        }
                        Hold();
                    }
                    // its wings as FAR's wing model has them: shielded (no force), the vessel's air density (none: no
                    // force), each one's submerged share and force
                    var wings = v.parts.Select(q => q.Modules.GetModule<ferram4.FARWingAerodynamicModel>()).Where(w => w != null).ToList();
                    if (wings.Count > 0)
                    {
                        var shielded = typeof(ferram4.FARWingAerodynamicModel).GetField("isShielded",
                            System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public);
                        ctx.Record($"wings at {depth:F2} m", $"vessel atmDensity {v.atmDensity:G3}, altitude {v.altitude:F2} m; " +
                            string.Join("; ", wings.Take(6).Select(w => $"{w.part.partInfo.name} shielded {shielded?.GetValue(w)}, " +
                                $"under {w.part.submergedPortion:F2}, |F| {w.worldSpaceForce.magnitude:F4} kN")));
                    }
                    if (i == steps - 1)
                        // wholly under: the parts KSP does not have so (their water and air forces both missing)
                        ctx.Record("not under at the deepest", string.Join("; ", v.parts.Where(q => q.submergedPortion < 0.99)
                            .Select(q => $"{q.partInfo.name} {q.submergedPortion:F2} (buoyancy {q.buoyancy:F2}, " +
                                         $"{(q.rb == null ? "physicsless" : "rb")}, {(q.partBuoyancy == null ? "no PartBuoyancy" : "PartBuoyancy")})")
                            .DefaultIfEmpty("none")));
                    if (i == steps - 1)
                        // wholly under, the water's pressure on a closed body is straight up (its buoyancy): along the
                        // tow or across it, no more than 1% of it (an open mesh's pressure pushes through its gap)
                        foreach (Part q in v.parts)
                        {
                            var hq = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                            if (hq == null || !hq.OwnsBuoyancy)
                                continue;
                            Vector3 f = hq.LastHydrostaticForce;
                            double level = Vector3.ProjectOnPlane(f, up).magnitude, lift = Vector3.Dot(f, up);
                            if (level > 0.01 * Math.Abs(lift) + 0.005)
                                pressureWrong.Add($"{q.partInfo.name} {level:F3} kN level on {lift:F3} up");
                        }
                    if (i == steps - 1)
                        // each part's water pressure at the deepest (kN: along the tow, across it, up): wholly under, a
                        // closed body's is straight up
                        ctx.Record("hydrostatic at the deepest", string.Join("; ", v.parts
                            .Select(q => (q, hq: q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics))
                            .Where(x => x.hq != null && x.hq.LastHydrostaticForce.sqrMagnitude > 0)
                            .OrderByDescending(x => Math.Abs(Vector3.Dot(x.hq.LastHydrostaticForce, h)))
                            .Select(x => $"{x.q.partInfo.name} {Vector3.Dot(x.hq.LastHydrostaticForce, h):F4}/" +
                                         $"{Vector3.Dot(x.hq.LastHydrostaticForce, Vector3.Cross(up, h)):F4}/{Vector3.Dot(x.hq.LastHydrostaticForce, up):F4} " +
                                         $"(owns {x.hq.OwnsBuoyancy}, open {x.hq.OpenShare:F4}, capped {x.hq.CapArea:F3} m2, volume {x.hq.MeshVolume:F3}, " +
                                         $"cfg buoyancy {x.q.buoyancy:F3}, foil {x.hq.Foil})")));
                    if (v.parts.Count != partsAtStart)
                        ctx.Record($"parts at {depth:F2} m", $"{v.parts.Count} of {partsAtStart}");
                    // a part moving against the rest (held as one body each step): its joint flexing, or shaking
                    ctx.Record($"flex at {depth:F2} m", $"{flex:F2} m/s ({flexPart}); largest forces: hydrostatic {peak[0].kN:F2} kN ({peak[0].part}), " +
                                                        $"water flow {peak[1].kN:F2} ({peak[1].part}), wing {peak[2].kN:F2} ({peak[2].part}), " +
                                                        $"body aero {peak[3].kN:F2} ({peak[3].part}); stock buoyancy {stockPeak.kN:F2} kN ({stockPeak.part})");
                    double drag = drags.Average(), air = airs.Average(), water = drag - air;
                    rows.Add((depth, drag, air, water));
                    topUnder.Add(depth - height > 0);
                    surfaces.Add(terms[8] / Math.Max(1, n));
                    unders.Add(Under(v));
                    double sub = v.parts.Sum(q => q.submergedPortion * q.mass) / Math.Max(1e-6, v.parts.Sum(q => q.mass));
                    csv.AppendLine(string.Join(",", new[] { depth, depth - height, drag, air, water, hydros.Average() }
                                                        .Concat(terms.Select(x => x / Math.Max(1, n))).Concat(new[] { sub }).Select(F)));
                }
                TestWarp.Hyper(ctx, false);
                File.WriteAllText(Path.Combine(ctx.ResultsDir, $"depth-{v.vesselName}.csv"), csv.ToString());

                // each series' steps the wrong way, against its largest
                var lines = new StringBuilder();
                var wrong = new List<string>();
                // (fallOnceUnder: the series may fall once its top is under, or where its sections go under (the hulls'
                // surface factor falls, or their planform under the water grows): the water's surface terms, planing's lift
                // and momentum and a face's impact, go with the surface; a submarine has more drag awash than deep)
                void Series(string name, Func<(double depth, double drag, double air, double water), double> of, int sign, bool fallOnceUnder)
                {
                    double scale = rows.Max(r => Math.Abs(of(r))), tol = tolerance * scale + 0.005;
                    lines.Append($"{name}: " + string.Join(" ", rows.Select(r => of(r).ToString("F3", CultureInfo.InvariantCulture))) + "; ");
                    for (int i = 1; i < rows.Count; i++)
                    {
                        double step = sign * (of(rows[i]) - of(rows[i - 1]));
                        if (step < -tol && !(fallOnceUnder && (topUnder[i] || surfaces[i] < surfaces[i - 1] - 0.02 || unders[i] > unders[i - 1] + 0.05)))
                            wrong.Add($"{name} {(sign > 0 ? "fell" : "rose")} {Math.Abs(of(rows[i]) - of(rows[i - 1])):F3} kN from bottom " +
                                      $"{rows[i - 1].depth:F2} to {rows[i].depth:F2} m deep");
                    }
                    if (sign * (of(rows[rows.Count - 1]) - of(rows[0])) <= 0)
                        wrong.Add($"{name} not {(sign > 0 ? "more" : "less")} at the deepest than at the first");
                }
                Series("drag", r => r.drag, +1, true);
                Series("air", r => r.air, -1, false);
                Series("water", r => r.water, +1, true);
                ctx.Record("by depth (kN)", lines.ToString());
                ctx.Record("wrong way", wrong.Count == 0 ? "none" : string.Join("; ", wrong));
                Assert.True(v.parts.Count == partsAtStart, $"no part lost on the way down ({partsAtStart} parts, {v.parts.Count} left)");
                ctx.Record("pressure not up at the deepest", pressureWrong.Count == 0 ? "none" : string.Join("; ", pressureWrong));
                Assert.True(pressureWrong.Count == 0, $"the water's pressure on each part wholly under is straight up ({string.Join("; ", pressureWrong.Take(4))})");
                Assert.True(wrong.Count == 0, $"its drag and the water's grow and the air's shrinks going deeper ({string.Join("; ", wrong.Take(4))})");
            }
            finally
            {
                TestWarp.Hyper(ctx, false);
                CheatOptions.NoCrashDamage = crashWas;
                FARDebugValues.allowStructuralFailures = failuresWas;
                WaterParams.Restore();
            }
        }

        // the air's drag on it (kN): FAR's aero force on each part (the dry share of a body part's), and a wing's force as
        // far as its density is the air's
        private static double AirDrag(Vessel v, Vector3 h)
        {
            Vector3 f = Vector3.zero;
            foreach (Part q in v.parts)
            {
                var aero = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>();
                if (aero != null && !float.IsNaN(aero.worldSpaceAeroForce.x))
                    f += aero.worldSpaceAeroForce;
                var wing = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                if (wing == null)
                    continue;
                double air = q.atmDensity * (1 - q.submergedPortion), water = v.mainBody.oceanDensity * 1000 * q.submergedPortion;
                if (air + water > 0)
                    f += wing.worldSpaceForce * (float)(air / (air + water));
            }
            return -Vector3.Dot(f, h);
        }

        // its colliders' lowest and highest altitudes (m): their mesh vertices, or a collider's box corners
        private static (double lo, double hi) Extent(Vessel v)
        {
            double lo = double.MaxValue, hi = double.MinValue;
            void At(Vector3 p)
            {
                double a = FlightGlobals.getAltitudeAtPos((Vector3d)p, v.mainBody);
                lo = Math.Min(lo, a);
                hi = Math.Max(hi, a);
            }
            foreach (Part q in v.parts)
                foreach (Collider c in q.GetComponentsInChildren<Collider>())
                {
                    if (!c.enabled || c.isTrigger)
                        continue;
                    if (c is MeshCollider mc && mc.sharedMesh != null && mc.sharedMesh.isReadable)
                    {
                        Matrix4x4 m = mc.transform.localToWorldMatrix;
                        foreach (Vector3 p in mc.sharedMesh.vertices)
                            At(m.MultiplyPoint3x4(p));
                        continue;
                    }
                    Bounds b = c.bounds;
                    for (int k = 0; k < 8; k++)
                        At(new Vector3((k & 1) == 0 ? b.min.x : b.max.x, (k & 2) == 0 ? b.min.y : b.max.y, (k & 4) == 0 ? b.min.z : b.max.z));
                }
            return (lo, hi);
        }

        // the hulls' planform under the water (m2): each one's (FAR's HullResult.Planform) times 1 - its surface factor. It grows
        // as any hull's sections go under, whatever the others do (a float going under as the fuselage meets the water)
        private static double Under(Vessel v)
        {
            double under = 0;
            var seen = new HashSet<object>();
            foreach (Part q in v.parts)
            {
                object hull = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics?.Hull;
                if (hull != null && seen.Add(hull) && hull.GetType().GetField("Surface")?.GetValue(hull) is double f &&
                    hull.GetType().GetField("Planform")?.GetValue(hull) is double a)
                    under += a * (1 - f);
            }
            return under;
        }

        // the hulls' surface factor (FAR's HullResult.Surface: 1 at the surface, 0 wholly under), weighted by their planforms
        // (a float strut wholly under early is no float going under)
        private static double Surface(Vessel v)
        {
            double sum = 0, weight = 0;
            var seen = new HashSet<object>();
            foreach (Part q in v.parts)
            {
                object hull = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics?.Hull;
                if (hull == null || !seen.Add(hull))
                    continue;
                if (hull.GetType().GetField("Surface")?.GetValue(hull) is double f && hull.GetType().GetField("Planform")?.GetValue(hull) is double a)
                {
                    sum += f * a;
                    weight += a;
                }
            }
            return weight > 0 ? sum / weight : 1;
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);
    }
}
