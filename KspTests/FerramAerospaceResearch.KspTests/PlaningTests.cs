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
        /// Writes planing-{vessel}.csv (a row a speed) and planing-steps-{vessel}.csv (every step).
        /// Parameters: mass (t), lcg (m from the transom), cgHeight (m above the bottom), speeds (m/s, + separated),
        /// rigid (default true: a hull of several parts put back in its built shape every step, so its joints don't bend
        /// it; the bend they would have put in it each step is recorded).
        /// </summary>
        [UITest(Kind = "Measure", StartsAt = StartState.Flight, Craft = Box, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 1500)]
        public static IEnumerator Tow_PlaningBox(TestContext ctx) => Tow(ctx, 2, 8, 2, new Rest(32.0, 0.250, 2.68), false);

        private const string Hull = "Planing Hull";

        /// <summary>
        /// The Planing Hull (crafts/planing_hull.py: one CustomProceduralParts Spine, Rectangle sections, beam 2 m, 8 m
        /// long, 2 m deep to 5.6 m from the transom, 0.5 m deep at the bow: a flat bottom rising about 17 deg at the bow)
        /// towed as Tow_PlaningBox is. Explicit: CustomProceduralParts comes from its checkout beside this repository's.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = @"..\..\..\CustomProceduralParts\GameData\CustomProceduralParts",
                StartsAt = StartState.Flight, Craft = Hull, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 1500)]
        public static IEnumerator Tow_PlaningHull(TestContext ctx) => Tow(ctx, 2, 8, 0.5, new Rest(28.40, 0.323, 0.33), false);

        private const string VHull = "Planing V Hull";

        /// <summary>
        /// The Planing V Hull (crafts/planing_hull.py v: Planing Hull's outline with a 20 deg V bottom, 0.6 m deep at
        /// the bow) towed as Tow_PlaningBox is, its keel turned down. Explicit, as Tow_PlaningHull.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = @"..\..\..\CustomProceduralParts\GameData\CustomProceduralParts",
                StartsAt = StartState.Flight, Craft = VHull, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 1500)]
        public static IEnumerator Tow_PlaningVHull(TestContext ctx) => Tow(ctx, 2, 8, 0.6, new Rest(26.03, 0.493, 0.70), true);

        // the same hulls split into three parts each (crafts/split_hull.py, cut at 35% and 70% of the length from the
        // transom): towed and checked as the one-part hulls are; FAR's water should not tell them apart
        private const string SplitMods = @"..\..\..\CustomProceduralParts\GameData\CustomProceduralParts";

        /// <summary>Tow_PlaningBox on Planing Box Split (three prisms end to end).</summary>
        [UITest(Kind = "Measure", Explicit = true, StartsAt = StartState.Flight, Craft = Box + " Split", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1500)]
        public static IEnumerator Tow_PlaningBoxSplit(TestContext ctx) => Tow(ctx, 2, 8, 2, new Rest(32.0, 0.250, 2.68), false);

        /// <summary>Tow_PlaningHull on Planing Hull Split (three Spines end to end).</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = SplitMods, StartsAt = StartState.Flight, Craft = Hull + " Split", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1500)]
        public static IEnumerator Tow_PlaningHullSplit(TestContext ctx) => Tow(ctx, 2, 8, 0.5, new Rest(28.40, 0.323, 0.33), false);

        /// <summary>Tow_PlaningVHull on Planing V Hull Split (three Spines end to end).</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = SplitMods, StartsAt = StartState.Flight, Craft = VHull + " Split", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1500)]
        public static IEnumerator Tow_PlaningVHullSplit(TestContext ctx) => Tow(ctx, 2, 8, 0.6, new Rest(26.03, 0.493, 0.70), true);

        /// <summary>
        /// A hull at rest at 4 t with its CG 3 m from the transom, by hand (sections integrated along the length): its
        /// volume (m3), the keel's depth below the water at the middle of its length (m) and its trim (deg, bow up).
        /// </summary>
        private readonly struct Rest
        {
            public readonly double Volume, Keel, Trim;
            public Rest(double volume, double keel, double trim) { Volume = volume; Keel = keel; Trim = trim; }
        }

        // beam, length and the hull's depth at the bow (m); the transom is a full beam-deep square. rest: the hand
        // calculation at the default mass and CG, checked before the tow. vKeel: turn the V's point down.
        private static IEnumerator Tow(TestContext ctx, double beam, double length, double bowDepthOfHull, Rest rest, bool vKeel)
        {
            double massT = ctx.ParamDouble("mass", 4.0), lcg = ctx.ParamDouble("lcg", 3.0), cgHeight = ctx.ParamDouble("cgHeight", 0.5);
            bool rigid = ctx.Param("rigid") != "false";
            WaterParams.Apply(ctx);
            double bendDeg = 0, bendM = 0;
            void Hold(Vessel vs, Part reference)
            {
                (double d, double m) b = RigidVessel.Bend(vs, reference);
                bendDeg = Math.Max(bendDeg, b.d);
                bendM = Math.Max(bendM, b.m);
                if (rigid)
                    RigidVessel.Snap(vs, reference);
            }
            double[] speeds = ctx.ParamList("speeds", "2+3+4+5+6+8+10+12+14+16+18+20").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            bool hydroWas = Settings.FARSettings.Hydrodynamics;
            string hydro = ctx.Param("hydro");
            if (!string.IsNullOrEmpty(hydro))
                Settings.FARSettings.Hydrodynamics = bool.Parse(hydro);
            double kaWas = FARAeroComponents.PartHydrodynamics.AddedMassCoefficient, pbWas = FARAeroComponents.PartHydrodynamics.PlaningBuoyancy;
            FARAeroComponents.PartHydrodynamics.AddedMassCoefficient = ctx.ParamDouble("ka", kaWas);
            FARAeroComponents.PartHydrodynamics.PlaningBuoyancy = ctx.ParamDouble("planingBuoyancy", pbWas);
            ctx.Record("model", Settings.FARSettings.Hydrodynamics
                                    ? $"PartHydrodynamics (added mass x{FARAeroComponents.PartHydrodynamics.AddedMassCoefficient}, planing buoyancy x{FARAeroComponents.PartHydrodynamics.PlaningBuoyancy})"
                                    : "legacy water share of the aero force");

            yield return Wait.Seconds(5);
            Vessel v = FlightGlobals.ActiveVessel;
            HullParts hull = HullParts.Whole(v);
            ctx.Record("parts", $"{hull.Parts.Length}: " + string.Join(", ", hull.Parts.Select(q => $"{q.partInfo.name} at {hull.InverseTransformPoint(q.transform.position).y:F3}")) +
                                $", {hull.Length:F3} m long");

            // mass: tanks emptied, then filled with liquid fuel to the mass asked for (each tank its share of the room)
            foreach (PartResource r in hull.Parts.SelectMany(q => q.Resources.Cast<PartResource>()))
                r.amount = 0;
            yield return Wait.Frames(2);
            double dry = hull.Parts.Sum(q => q.mass);
            PartResource[] lfs = hull.Parts.Select(q => q.Resources.Get("LiquidFuel")).ToArray();
            Assert.True(lfs.All(r => r != null), "the hull's tanks hold liquid fuel");
            double room = lfs.Sum(r => r.maxAmount), density = lfs[0].info.density;
            double add = (massT - dry) / density;
            Assert.True(add >= 0 && add <= room, $"{massT} t reachable: dry {dry:F3} t, {room:F0} units of {density} t");
            foreach (PartResource r in lfs)
                r.amount = add * r.maxAmount / room;
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
                hull = HullParts.Whole(v);
                Rigidbody rb = hull.Ref.rb;

                // the hull's axes: bow is the part's +y (its long axis, nose of a craft built along the SPH);
                // its bottom is whichever of +-x, +-z points down
                Vector3 up = v.upAxis;
                Vector3 h = Vector3.ProjectOnPlane(hull.Forward, up).normalized;
                Vector3 localDown;
                if (vKeel)
                    localDown = PlaningVideoTests.KeelAxis(hull.Ref);
                else
                    localDown = Quantize(hull.Ref.transform.InverseTransformDirection(-up));
                ctx.Record("hull", $"local down {localDown}, ocean density {body.oceanDensity} t/m3, g {body.GeeASL * 9.80665:F3} m/s2, " +
                                   $"mass {v.GetTotalMass():F3} t (dry {dry:F3}), C_Delta {v.GetTotalMass() * 1000 / (rho * beam * beam * beam):F3}");

                // CG: lcg from the transom (local y = -length/2), cgHeight above the bottom
                Vector3 cgLocal = new Vector3(0, (float)(lcg - length / 2), 0) + localDown * (float)(beam / 2 - cgHeight);
                hull.SetCoM(v, cgLocal);

                // FAR's voxel model of it: its length and largest cross-section (one hull: 8 m and its section's area)
                var fva = v.GetComponent<FARAeroComponents.FARVesselAero>();
                ctx.Record("voxel", $"FAR's model {fva.Length:F3} m long, largest section {fva.MaxCrossSectionArea:F3} m2");

                // held still in the air first: the moment on it about its CoM (its angular momentum gained each step, then
                // taken out), which with gravity alone is nothing
                {
                    Vector3 sideA = Vector3.Cross(up, h).normalized;
                    var airM = new List<double>();
                    double ta = Planetarium.GetUniversalTime();
                    while (Planetarium.GetUniversalTime() - ta < 3)
                    {
                        Vector3 l = RigidVessel.AngularMomentum(v);
                        if (Planetarium.GetUniversalTime() - ta > 1)
                            airM.Add(Vector3.Dot(l, sideA) / TimeWarp.fixedDeltaTime);
                        Hold(v, hull.Ref);
                        RigidVessel.Set(v, Vector3.zero, Vector3.zero);
                        yield return new WaitForFixedUpdate();
                    }
                    ctx.Record("air", $"held still in the air: moment about the CoM {airM.Average():F4} kN m (sd {Sd(airM):F4}; bow down +)");
                }

                // level on the water at the tow speed, floating at its static draft
                double draft = v.GetTotalMass() * 1000 / (rho * beam * length);
                Flight.PointAtAoA(0, h, up);
                yield return new WaitForFixedUpdate();
                if (vKeel)
                {
                    // PointAtAoA leaves the roll as it finds it: turn the V's point down
                    Vector3 zW = Mathf.Abs(localDown.z) > 0.5f ? (localDown.z > 0 ? -up : up) : Vector3.Cross(localDown.x > 0 ? -up : up, h);
                    v.SetRotation(Quaternion.LookRotation(zW, h));
                    yield return new WaitForFixedUpdate();
                }
                double centerAlt = beam / 2 - draft;
                v.SetPosition(v.vesselTransform.position + (Vector3d)up * (centerAlt - v.altitude));
                RigidVessel.Set(v, Vector3.zero, Vector3.zero);

                var steps = new StringBuilder("V,t,tow_kN,air_kN,trim_deg,transomDepth_m,bowDepth_m,buoyUp_kN,farUp_kN,farAlong_kN,rbDrag,submerged,comOffsetY," +
                                              "hydroUp_kN,hydroDrag_kN,momUp_kN,momDrag_kN,crossUp_kN,friction_kN,impact_kN,wetLength_m,planform_m2,limiter,hydrostaticUp_kN\n");
                var rows = new StringBuilder("V_ms,C_V,R_kN,R_sd,air_kN,water_kN,water_over_Delta,trim_deg,trim_sd,transomDepth_m,bowDepth_m,lambda_est,buoyUp_kN,farUp_kN,rbDrag,submerged," +
                                             "hydroUp_kN,hydroDrag_kN,wetLength_m,limiter\n");
                var fixedWait = new WaitForFixedUpdate();
                double g = body.GeeASL * 9.80665, weightKN = v.GetTotalMass() * g;
                var aeroModules = hull.Parts.Select(q => q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()).Where(a => a != null).ToArray();

                // at rest first: the setup the tow stands on (mass, volume, draft, trim) against the hand calculation
                ctx.Step("at rest");
                double tr0 = Planetarium.GetUniversalTime();
                var restKeel = new List<double>(); var restTrim = new List<double>(); var restBuoy = new List<double>();
                var restCom = new List<double>(); var restCb = new List<double>(); var restDyn = new List<double>();
                var restMeas = new List<double>(); var restPerPart = new List<string>();
                double lSet = double.NaN;
                Vector3 sideAxis = Vector3.Cross(up, h).normalized;
                while (Planetarium.GetUniversalTime() - tr0 < 12)
                {
                    // the moment that acted over the last step: the angular momentum it put on the hull since the hold
                    double lNow = Vector3.Dot(RigidVessel.AngularMomentum(v), sideAxis);
                    double measured = double.IsNaN(lSet) ? double.NaN : (lNow - lSet) / TimeWarp.fixedDeltaTime;
                    bool freeRest = ctx.Param("freeRest") == "true" && Planetarium.GetUniversalTime() - tr0 > 2;   // left alone after 2 s
                    if (!freeRest)
                        Hold(v, hull.Ref);
                    if (!freeRest)
                        RigidVessel.Set(v, up * Vector3.Dot(RigidVessel.Velocity(v), up), sideAxis * Vector3.Dot(RigidVessel.AngularVelocity(v), sideAxis));
                    lSet = Vector3.Dot(RigidVessel.AngularMomentum(v), sideAxis);
                    if (Planetarium.GetUniversalTime() - tr0 > 8)
                    {
                        if (!double.IsNaN(measured))
                            restMeas.Add(measured);
                        restKeel.Add(beam / 2 - FlightGlobals.getAltitudeAtPos(hull.Position, body));
                        restTrim.Add(Math.Asin(Mathf.Clamp(Vector3.Dot(hull.Forward, up), -1, 1)) * 180 / Math.PI);
                        restBuoy.Add(Vector3.Dot(hull.StockBuoyancy + hull.Sum(x => x.LastHydrostaticForce), up));
                        // the CoM along the hull, and where the water's pressure acts (its moment about the CoM over its lift)
                        Vector3 com = RigidVessel.CoM(v), moment = Vector3.zero, lift = Vector3.zero, dyn = Vector3.zero;
                        foreach (Part q in hull.Parts)
                        {
                            var hq = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                            if (hq == null || q.rb == null)
                                continue;
                            moment += hq.LastHydrostaticTorque + Vector3.Cross(q.rb.worldCenterOfMass - com, hq.LastHydrostaticForce);
                            lift += hq.LastHydrostaticForce;
                            dyn += hq.LastTorque + Vector3.Cross(q.rb.worldCenterOfMass - com, hq.LastForce);
                        }
                        restCom.Add(hull.InverseTransformPoint(com).y);
                        restCb.Add(-Vector3.Dot(moment, sideAxis) / Math.Max(1e-6, Vector3.Dot(lift, up)));
                        restDyn.Add(Vector3.Dot(dyn, sideAxis));
                        if (restPerPart.Count == 0)
                            foreach (Part q in hull.Parts)
                            {
                                var hq = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                                Vector3 m = hq == null ? Vector3.zero : hq.LastHydrostaticTorque + Vector3.Cross(q.rb.worldCenterOfMass - com, hq.LastHydrostaticForce);
                                restPerPart.Add($"{q.partInfo.name} at {hull.InverseTransformPoint(q.transform.position).y:F2}: mass {q.rb.mass * 1000:F0} kg, " +
                                                $"CoM {hull.InverseTransformPoint(q.rb.worldCenterOfMass).y:F3}, pressure up {Vector3.Dot(hq?.LastHydrostaticForce ?? Vector3.zero, up):F3} kN, " +
                                                $"its moment about the CoM {Vector3.Dot(m, sideAxis):F3} kN m, stock buoyancy {(q.partBuoyancy != null ? Vector3.Dot(q.partBuoyancy.effectiveForce, up) : 0):F3} kN " +
                                                $"(part.buoyancy {q.buoyancy:F2}), rb drag {q.rb.drag:F3} angular {q.rb.angularDrag:F3}, submerged {q.submergedPortion:F3}");
                            }
                    }
                    yield return fixedWait;
                }
                ctx.Record("rest parts", string.Join("; ", restPerPart));
                bool ownsBuoyancy = Settings.FARSettings.Hydrodynamics && hull.OwnsBuoyancy;
                double meshVolume = hull.MeshVolume;
                bool defaults = Math.Abs(massT - 4.0) < 1e-6 && Math.Abs(lcg - 3.0) < 1e-6;
                ctx.Record("rest", $"keel {restKeel.Average():F3} m deep at the middle (hand {rest.Keel:F3}), trim {restTrim.Average():F2} deg (hand {rest.Trim:F2}), " +
                                   $"buoyancy {restBuoy.Average():F2} kN of {weightKN:F2}, CoM at {restCom.Average():F4} m along the hull " +
                                   $"(set {cgLocal.y:F4}), the pressure's line {restCb.Average():F4} m ahead of it, the other water forces' " +
                                   $"moment about it {restDyn.Average():F3} kN m (bow down +), the moment that acted {restMeas.Average():F3} kN m, " +
                                   (ownsBuoyancy ? $"FAR's, mesh volume {meshVolume:F2} m3 (hand {rest.Volume:F2})" : "stock's"));
                if (ownsBuoyancy)
                {
                    Assert.True(Math.Abs(meshVolume / rest.Volume - 1) < 0.01, $"mesh volume {meshVolume:F2} m3 within 1% of {rest.Volume:F2}");
                    if (defaults)
                    {
                        Assert.True(Math.Abs(restKeel.Average() - rest.Keel) < 0.02, $"rest keel {restKeel.Average():F3} m within 2 cm of {rest.Keel:F3}");
                        Assert.True(Math.Abs(restTrim.Average() - rest.Trim) < 0.3, $"rest trim {restTrim.Average():F2} deg within 0.3 of {rest.Trim:F2}");
                    }
                }
                double setAlong = 0;
                Vector3 stern = new Vector3(0, (float)(-length / 2), 0) + localDown * (float)(beam / 2);
                Vector3 bow = new Vector3(0, (float)(length / 2), 0) + localDown * (float)(bowDepthOfHull / 2);

                foreach (double V in speeds)
                {
                    ctx.Step($"tow at {V} m/s");
                    double t0 = Planetarium.GetUniversalTime();
                    var tow = new List<double>(); var air = new List<double>(); var trim = new List<double>(); var dT = new List<double>(); var dB = new List<double>();
                    var buoy = new List<double>(); var farUp = new List<double>(); var drag = new List<double>(); var sub = new List<double>();
                    var hUp = new List<double>(); var hDrag = new List<double>(); var wetL = new List<double>(); var lim = new List<double>();
                    int n = 0;
                    while (Planetarium.GetUniversalTime() - t0 < 14)
                    {
                        // after the physics step: what the tow line had to give back, then the tow speed restored
                        double dt = TimeWarp.fixedDeltaTime, m = v.GetTotalMass();
                        Vector3 vel = RigidVessel.Velocity(v);
                        double along = Vector3.Dot(vel, h);
                        double towKN = m * (setAlong - along) / dt;
                        Vector3 side = Vector3.Cross(up, h).normalized;
                        Vector3 vert = up * Vector3.Dot(vel, up);
                        Hold(v, hull.Ref);
                        RigidVessel.Set(v, vert + h * (float)V, side * Vector3.Dot(RigidVessel.AngularVelocity(v), side));
                        setAlong = V;

                        double t = Planetarium.GetUniversalTime() - t0;
                        Vector3 fwd = hull.Forward;
                        double trimDeg = Math.Asin(Mathf.Clamp(Vector3.Dot(fwd, up), -1, 1)) * 180 / Math.PI;
                        double depthT = -FlightGlobals.getAltitudeAtPos(hull.TransformPoint(stern), body);
                        double depthB = -FlightGlobals.getAltitudeAtPos(hull.TransformPoint(bow), body);
                        double buoyUp = Vector3.Dot(hull.StockBuoyancy + hull.Sum(x => x.LastHydrostaticForce), up);
                        Vector3 aeroForce = aeroModules.Aggregate(Vector3.zero, (sum, a) => sum + a.worldSpaceAeroForce);
                        double farUpKN = aeroModules.Length > 0 ? Vector3.Dot(aeroForce, up) : double.NaN;
                        double farAlongKN = aeroModules.Length > 0 ? -Vector3.Dot(aeroForce, h) : double.NaN;
                        double airKN = double.NaN;
                        if (n % 10 == 0)
                        {
                            FARAPI.CalculateVesselAeroForces(v, out Vector3 f, out _, h * (float)V, Math.Max(0, v.altitude));
                            airKN = -Vector3.Dot(f, h);
                        }

                        Vector3 hForce = hull.Sum(x => x.LastForce), hMom = hull.Sum(x => x.LastMomentumForce);
                        double hydroUp = Vector3.Dot(hForce, up), hydroDrag = -Vector3.Dot(hForce, h);
                        double sub0 = hull.Submerged;
                        steps.AppendLine(string.Join(",", F(V), F(t), F(towKN), F(airKN), F(trimDeg), F(depthT), F(depthB), F(buoyUp), F(farUpKN), F(farAlongKN),
                                                     F(rb.drag), F(sub0), F(hull.InverseTransformPoint(RigidVessel.CoM(v)).y),
                                                     F(hydroUp), F(hydroDrag),
                                                     F(Vector3.Dot(hMom, up)), F(-Vector3.Dot(hMom, h)),
                                                     F(Vector3.Dot(hull.Sum(x => x.LastCrossflowForce), up)), F(-Vector3.Dot(hull.Sum(x => x.LastFrictionForce), h)),
                                                     F(-Vector3.Dot(hull.Sum(x => x.LastImpactForce), h)), F(hull.WettedLength), F(hull.Planform),
                                                     F(hull.Limiter), F(Vector3.Dot(hull.Sum(x => x.LastHydrostaticForce), up))));
                        if (t >= 8)
                        {
                            tow.Add(towKN); trim.Add(trimDeg); dT.Add(depthT); dB.Add(depthB); buoy.Add(buoyUp); farUp.Add(farUpKN); drag.Add(rb.drag); sub.Add(sub0);
                            hUp.Add(hydroUp); hDrag.Add(hydroDrag); wetL.Add(hull.WettedLength); lim.Add(hull.Limiter);
                            if (!double.IsNaN(airKN)) air.Add(airKN);
                        }
                        n++;
                        yield return fixedWait;
                    }

                    double R = tow.Average(), airMean = air.Count > 0 ? air.Average() : double.NaN, water = R - airMean, tr = trim.Average();
                    double lambda = tr > 0.5 ? Math.Max(0, dT.Average()) / Math.Sin(tr * Math.PI / 180) / beam : double.NaN;
                    rows.AppendLine(string.Join(",", F(V), F(V / Math.Sqrt(g * beam)), F(R), F(Sd(tow)), F(airMean), F(water), F(water / weightKN), F(tr), F(Sd(trim)),
                                                F(dT.Average()), F(dB.Average()), F(lambda), F(buoy.Average()), F(farUp.Average()), F(drag.Average()), F(sub.Average()),
                                                F(hUp.Average()), F(hDrag.Average()), F(wetL.Average()), F(lim.Min())));
                    ctx.Record($"V{V} aero parts", string.Join("; ", aeroModules.Select(a => $"{a.part.partInfo.name} at {hull.InverseTransformPoint(a.part.transform.position).y:F2}: along {-Vector3.Dot(a.worldSpaceAeroForce, h):F3} up {Vector3.Dot(a.worldSpaceAeroForce, up):F3} kN, submerged {a.part.submergedPortion:F3}")));
                    ctx.Record($"V{V}", $"R {R:F2} kN (sd {Sd(tow):F2}), air {airMean:F2}, water/Delta {water / weightKN:F4}, trim {tr:F2} deg (sd {Sd(trim):F2}), " +
                                        $"transom {dT.Average():F2} m deep, bow {dB.Average():F2}, buoyancy up {buoy.Average():F1} kN, FAR up {farUp.Average():F1} kN, " +
                                        $"hydro up {hUp.Average():F1} kN drag {hDrag.Average():F2} kN, wetted {wetL.Average():F2} m, of {weightKN:F1}");
                }
                ctx.Record("bend", $"the joints bent the hull up to {bendDeg:F3} deg, {bendM * 1000:F1} mm " +
                                   (rigid ? "in a step (put back every step)" : "(left free)"));
                File.WriteAllText(Path.Combine(ctx.ResultsDir, $"planing-{v.vesselName}.csv"), rows.ToString());
                File.WriteAllText(Path.Combine(ctx.ResultsDir, $"planing-steps-{v.vesselName}.csv"), steps.ToString());
            }
            finally
            {
                CheatOptions.NoCrashDamage = crashWas;
                Settings.FARSettings.Hydrodynamics = hydroWas;
                FARAeroComponents.PartHydrodynamics.AddedMassCoefficient = kaWas;
                FARAeroComponents.PartHydrodynamics.PlaningBuoyancy = pbWas;
                WaterParams.Restore();
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
