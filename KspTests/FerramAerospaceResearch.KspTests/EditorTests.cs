using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.FARPartGeometry.GeometryModification;
using FerramAerospaceResearch.FARThreading;
using KspTest;
using ProceduralFairings;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR in the editor, on stock crafts (the Aeris 4A, an airliner, a rocket): the voxel model of the craft and the static analysis built on it, checked
    /// against the craft's own size and against what any conventional plane does (lift rises with angle of attack, it
    /// is stable in pitch, a symmetric craft is symmetric in sideslip, drag rises through Mach 1).
    /// </summary>
    public static class EditorTests
    {
        // stock crafts
        private const string Airliner = "Stearwing A300";
        private const string AirlinerCovered = "Stearwing A300 Covered";   // crafts/: a Mk7 nose cone on one Goliath's intake
        private const string Rocket = "Kerbal 1";

        /// <summary>The voxel model's length and largest cross-section against the craft's bounds.</summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Editor_VoxelModelMatchesTheCraft(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            VehicleAerodynamics vehicle = Far.EditorVehicle;
            // FAR measures along the craft's axis, the root part's up for a plane: its length, and the area across it
            Transform root = EditorLogic.RootPart.transform;
            Vector3 extent = CraftExtent(root.up, root.right, root.forward);
            double boundsLength = extent.x, boundsArea = extent.y * extent.z;
            ctx.Record("extent", extent.ToString("F2"));
            ctx.Record("voxelLength", vehicle.Length);
            ctx.Record("maxCrossSection", vehicle.MaxCrossSectionArea);
            Assert.InRange(0.85 * boundsLength, 1.1 * boundsLength, vehicle.Length, "voxel length against the craft's length");
            Assert.InRange(0.02 * boundsArea, boundsArea, vehicle.MaxCrossSectionArea, "largest cross-section inside the craft's bounding box");

            double[] areas = vehicle.GetCrossSectionAreas();
            Assert.True(areas.Length > 10, $"cross-sections along the craft ({areas.Length})");
            Assert.True(areas.All(a => a >= 0 && !double.IsNaN(a)), "every cross-section is a real, non-negative area");
            // MaxCrossSectionArea is the raw voxel's; the sections are smoothed for area ruling after, which lowers peaks
            Assert.InRange(0.7 * vehicle.MaxCrossSectionArea, 1.0001 * vehicle.MaxCrossSectionArea, areas.Max(),
                           "the smoothed sections' largest against the voxel's largest cross-section");
        }

        /// <summary>Lift against angle of attack at Mach 0.4: rising, with a plausible slope, and drag always positive.</summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Editor_LiftRisesWithAngleOfAttack(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            double prevCl = double.NegativeInfinity;
            for (int alpha = -4; alpha <= 10; alpha += 2)
            {
                InstantConditionSimOutput o = Far.Instant(alpha, 0.4);
                ctx.Record($"alpha{alpha}", $"Cl {o.Cl:F4} Cd {o.Cd:F4} Cm {o.Cm:F4}");
                Assert.True(o.Cl > prevCl, $"Cl rises with alpha: {o.Cl:F4} at {alpha} deg after {prevCl:F4}");
                Assert.True(o.Cd > 0, $"Cd is positive at {alpha} deg ({o.Cd:F4})");
                prevCl = o.Cl;
            }

            double slopePerDeg = (Far.Instant(4, 0.4).Cl - Far.Instant(0, 0.4).Cl) / 4;
            ctx.Record("liftSlopePerDeg", slopePerDeg);
            // a wing's 2 pi per radian is 0.11 per degree; a whole plane, on its wing area, has rather less
            Assert.InRange(0.02, 0.12, slopePerDeg, "lift slope per degree");
        }

        /// <summary>Nose-down pitching moment rises with angle of attack: the stock plane is stable in pitch.</summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Editor_StableInPitch(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            double cm0 = Far.Instant(0, 0.4).Cm, cm8 = Far.Instant(8, 0.4).Cm;
            ctx.Record("Cm0", cm0);
            ctx.Record("Cm8", cm8);
            Assert.True(cm8 < cm0, $"Cm falls with alpha (dCm/dalpha < 0): {cm0:F4} at 0 deg, {cm8:F4} at 8 deg");
        }

        /// <summary>
        /// The wings take the whole craft's part list once it is voxelized after a load: not a list FAR's editor made while
        /// the craft was still loading (EditorGUI.ResetEditor throws the last one away, then whatever reads the list first
        /// makes it from the parts under the root by then). A wing whose list lacks a part takes its raycasts' hits on it
        /// for nothing there: after a save reload changed a load's timing, the ProceduralParts Aeris's winglets beside its
        /// tanks were measured with their roots in free air, half their aspect ratio.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 600)]
        public static IEnumerator Editor_WingsTakeTheWholeCraftsPartList(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            yield return Far.Settled("the Aeris 4A");
            List<Part> ship = EditorLogic.fetch.ship.parts;
            var wings = ship.Select(p => p.GetComponent<ferram4.FARWingAerodynamicModel>()).Where(w => w != null).ToList();
            Assert.True(wings.Count > 4, $"the Aeris 4A's wings ({wings.Count})");
            Dictionary<ferram4.FARWingAerodynamicModel, double> ar = wings.ToDictionary(w => w, w => w.EffectiveAR);

            try
            {
                // as a load leaves it when the list is made before the craft's last parts are attached: the root alone
                var partial = new List<Part> { EditorLogic.RootPart };
                FerramAerospaceResearch.FARGUI.FAREditorGUI.EditorGUI.ResetEditor();
                Refl.SetStatic(typeof(FARAeroUtil), "AllEditorPartsCache", partial);
                foreach (ferram4.FARWingAerodynamicModel w in wings)
                    w.VesselPartList = partial;
                yield return Far.Settled("the Aeris 4A, voxelized again");

                foreach (ferram4.FARWingAerodynamicModel w in wings)
                {
                    string name = $"{w.part.partInfo.name} {w.part.craftID}";
                    Assert.Equal(ship.Count, w.VesselPartList?.Count ?? -1, $"{name}'s part list is the craft's");
                    // and its neighbours found among them again: the aspect ratio it had
                    Assert.Near(ar[w], w.EffectiveAR, 1e-6 * ar[w], $"{name}'s aspect ratio");
                }
            }
            finally
            {
                // FAR's own list and the wings' from it, whatever the test left
                FerramAerospaceResearch.FARGUI.FAREditorGUI.EditorGUI.ResetEditor();
            }
            yield return Far.Settled("the Aeris 4A, its wings' lists made again");
        }

        /// <summary>A symmetric craft: no side force, yaw or roll without sideslip, and equal and opposite ones with it.</summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Editor_SymmetricInSideslip(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            InstantConditionSimOutput zero = Far.Instant(3, 0.4, 0), right = Far.Instant(3, 0.4, 5), left = Far.Instant(3, 0.4, -5);
            ctx.Record("beta+5", $"Cy {right.Cy:F5} Cn {right.Cn:F5} Croll {right.C_roll:F5}");
            ctx.Record("beta-5", $"Cy {left.Cy:F5} Cn {left.Cn:F5} Croll {left.C_roll:F5}");
            double scale = Math.Abs(right.Cy) + 1e-3;
            Assert.True(Math.Abs(zero.Cy) < 0.02 * scale && Math.Abs(zero.Cn) < 1e-3 && Math.Abs(zero.C_roll) < 1e-3,
                        $"no lateral force or moment at zero sideslip: Cy {zero.Cy:G3}, Cn {zero.Cn:G3}, Croll {zero.C_roll:G3}");
            Assert.True(Math.Abs(right.Cy) > 1e-3, $"sideslip gives a side force ({right.Cy:G3})");
            Assert.Near(-right.Cy, left.Cy, 0.02 * Math.Abs(right.Cy), "side force is equal and opposite either side");
            Assert.Near(-right.Cn, left.Cn, 0.02 * Math.Abs(right.Cn) + 1e-5, "yawing moment is equal and opposite either side");
            Assert.Near(-right.C_roll, left.C_roll, 0.02 * Math.Abs(right.C_roll) + 1e-5, "rolling moment is equal and opposite either side");
        }

        /// <summary>Drag rises through the transonic: wave drag at Mach 1.1 on top of the subsonic drag at Mach 0.5.</summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Editor_TransonicDragRise(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            double sub = Far.Instant(2, 0.5).Cd, trans = Far.Instant(2, 1.1).Cd;
            ctx.Record("Cd0.5", sub);
            ctx.Record("Cd1.1", trans);
            Assert.True(trans > 1.3 * sub, $"Cd at Mach 1.1 ({trans:F4}) well above Mach 0.5's ({sub:F4})");
        }

        /// <summary>
        /// The stock Stearwing A300 airliner: its Goliath turbofans are engines with an intake in the part, so each gets
        /// one integrated intake-engine adjuster ducting 100 times the intake's area. The ducting takes area out of the
        /// cross-sections the nacelles are in: without the adjusters the voxel model holds more volume and the airliner
        /// more drag at Mach 0.8, and with them back the model is what it was. And the airliner flies like one: lift
        /// rising with angle of attack, stable in pitch.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Airliner, TimeoutSec = 900)]
        public static IEnumerator Editor_AirlinerDuctsItsIntegratedIntakes(TestContext ctx)
        {
            yield return LoadedCraft("the airliner", "turboFanSize2", "airlinerMainWing");
            VehicleAerodynamics vehicle = Far.EditorVehicle;
            Part[] goliaths = EditorLogic.fetch.ship.parts.Where(p => p.partInfo.name == "turboFanSize2").ToArray();
            Assert.Equal(2, goliaths.Length, "the airliner's Goliaths");
            foreach (Part p in goliaths)
            {
                GeometryPartModule g = p.FindModuleImplementing<GeometryPartModule>();
                var adjusters = Refl.Get<List<ICrossSectionAdjuster>>(g, "crossSectionAdjusters");
                Assert.Equal(1, adjusters.Count, "one cross-section adjuster on a Goliath");
                Assert.True(adjusters[0] is IntegratedIntakeEngineCrossSectionAdjuster, $"an integrated intake-engine adjuster ({adjusters[0].GetType().Name})");
                double intake = p.FindModuleImplementing<ModuleResourceIntake>().area;
                Assert.Near(100 * intake, g.MaxCrossSectionAdjusterArea, 1e-6, "the adjuster ducts 100 times the intake's area (m2)");
            }

            Transform root = EditorLogic.RootPart.transform;
            Vector3 extent = CraftExtent(root.up, root.right, root.forward);
            ctx.Record("extent", extent.ToString("F2"));
            Assert.InRange(0.85 * extent.x, 1.1 * extent.x, vehicle.Length, "voxel length against the airliner's length");

            double vol = Far.Volume(vehicle), cd = Far.Instant(2, 0.8).Cd;
            // the most the ducts can take out: their area along the whole nacelle
            double goliathLength = goliaths.Max(p => p.FindModuleImplementing<GeometryPartModule>().localMeshBounds.size.y);
            double ductMax = goliaths.Sum(p => p.FindModuleImplementing<GeometryPartModule>().MaxCrossSectionAdjusterArea) * goliathLength;
            var saved = new Dictionary<GeometryPartModule, List<ICrossSectionAdjuster>>();
            try
            {
                ctx.Step("Without the Goliaths' adjusters");
                foreach (Part p in goliaths)
                {
                    GeometryPartModule g = p.FindModuleImplementing<GeometryPartModule>();
                    saved[g] = Refl.Get<List<ICrossSectionAdjuster>>(g, "crossSectionAdjusters");
                    Refl.Set(g, "crossSectionAdjusters", new List<ICrossSectionAdjuster>());
                }

                yield return Far.Revoxelize("the airliner without ducting");
                double volBare = Far.Volume(vehicle), cdBare = Far.Instant(2, 0.8).Cd;
                ctx.Record("ducting", $"volume {vol:F2} m3 ducted, {volBare:F2} without; Cd at Mach 0.8 {cd:F5} ducted, {cdBare:F5} without; " +
                                      $"at most {ductMax:F2} m3 of duct");
                Assert.True(volBare > vol, $"the ducting takes volume out of the model ({vol:F2} against {volBare:F2} m3)");
                Assert.True(volBare - vol < ductMax, $"no more than the ducts' area along the nacelles ({volBare - vol:F2} m3, at most {ductMax:F2})");
                Assert.True(cdBare > cd, $"more drag without the ducting ({cdBare:F5} against {cd:F5})");
            }
            finally
            {
                foreach (KeyValuePair<GeometryPartModule, List<ICrossSectionAdjuster>> kv in saved)
                    Refl.Set(kv.Key, "crossSectionAdjusters", kv.Value);
            }

            ctx.Step("The adjusters back");
            yield return Far.Revoxelize("the airliner ducted again");
            Assert.Near(vol, Far.Volume(vehicle), 1e-4 * vol, "the ducted volume again (m3)");

            ctx.Step("An airliner's lift and pitch stability");
            InstantConditionSimOutput a0 = Far.Instant(0, 0.5), a6 = Far.Instant(6, 0.5);
            ctx.Record("alpha", $"0 deg Cl {a0.Cl:F4} Cm {a0.Cm:F4}; 6 deg Cl {a6.Cl:F4} Cm {a6.Cm:F4}");
            Assert.InRange(0.02, 0.12, (a6.Cl - a0.Cl) / 6, "lift slope per degree");
            Assert.True(a6.Cm < a0.Cm, $"stable in pitch: Cm falls with alpha ({a0.Cm:F4} to {a6.Cm:F4})");
        }

        /// <summary>
        /// The airliner with a Mk7 nose cone on one Goliath's intake node (its occludeNode, top): the covered Goliath's
        /// adjuster is no longer an integrated intake but an engine exit (its area negated, ducted in the engines' sweep,
        /// its threshold read), the open one's still an intake. Each adjuster taken out in turn: the open Goliath's ducting
        /// takes volume out of the model, the covered one's puts some back. (AJE swaps the Goliath's intake; this runs in
        /// the instance without it.)
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = AirlinerCovered, TimeoutSec = 900)]
        public static IEnumerator Editor_CoveredGoliathIntakeIsAnEngineExit(TestContext ctx)
        {
            yield return LoadedCraft("the airliner with a covered Goliath", "turboFanSize2", "rocketNoseCone.v3");
            VehicleAerodynamics vehicle = Far.EditorVehicle;
            Part[] goliaths = EditorLogic.fetch.ship.parts.Where(p => p.partInfo.name == "turboFanSize2").ToArray();
            Assert.Equal(2, goliaths.Length, "the airliner's Goliaths");
            Part covered = goliaths.FirstOrDefault(p => p.FindModuleImplementing<ModuleResourceIntake>().node?.attachedPart != null);
            Part open = goliaths.FirstOrDefault(p => p != covered);
            Assert.NotNull(covered, "a Goliath with a part on its intake node");
            Assert.Equal("rocketNoseCone.v3", covered.FindModuleImplementing<ModuleResourceIntake>().node.attachedPart.partInfo.name, "the nose cone on it");
            Assert.True(open.FindModuleImplementing<ModuleResourceIntake>().node?.attachedPart == null, "the other Goliath's intake open");

            GeometryPartModule Geo(Part p) => p.FindModuleImplementing<GeometryPartModule>();
            IntegratedIntakeEngineCrossSectionAdjuster Adjuster(Part p)
            {
                var list = Refl.Get<List<ICrossSectionAdjuster>>(Geo(p), "crossSectionAdjusters");
                Assert.Equal(1, list.Count, $"one cross-section adjuster on the {(p == covered ? "covered" : "open")} Goliath");
                return list[0] as IntegratedIntakeEngineCrossSectionAdjuster;
            }

            IntegratedIntakeEngineCrossSectionAdjuster ac = Adjuster(covered), ao = Adjuster(open);
            Assert.NotNull(ac, "the covered Goliath's adjuster an integrated intake-engine one");
            Assert.NotNull(ao, "the open Goliath's adjuster an integrated intake-engine one");
            ctx.Record("adjusters", $"covered: integrated {ac.IntegratedCrossSectionIncreaseDecrease()}, area {ac.AreaRemovedFromCrossSection():F3}, threshold {ac.AreaThreshold()}; " +
                                    $"open: integrated {ao.IntegratedCrossSectionIncreaseDecrease()}, area {ao.AreaRemovedFromCrossSection():F3}");
            Assert.True(ao.IntegratedCrossSectionIncreaseDecrease(), "the open Goliath an integrated intake");
            Assert.False(ac.IntegratedCrossSectionIncreaseDecrease(), "the covered Goliath not one: an engine exit");
            Assert.True(Math.Abs(ao.AreaRemovedFromCrossSection()) > 0, "the open Goliath ducts area");
            Assert.Near(-ao.AreaRemovedFromCrossSection(), ac.AreaRemovedFromCrossSection(), 1e-9, "the covered Goliath's area the open one's negated");
            Assert.Equal(0.0, ac.AreaThreshold(), "an integrated adjuster's threshold");
            Vector3[] axes = { Vector3.right, Vector3.left, Vector3.up, Vector3.down, Vector3.forward, Vector3.back };
            double[] along = axes.Select(x => ao.AreaRemovedFromCrossSection(x)).ToArray();
            Assert.Equal(1, along.Count(x => x > 0), $"the open Goliath ducts along one axis of the six ({string.Join(", ", along.Select(x => x.ToString("F3")))})");

            double vol = Far.Volume(vehicle);
            var saved = new Dictionary<GeometryPartModule, List<ICrossSectionAdjuster>>();
            double volNoCovered = 0, volNoOpen = 0;
            try
            {
                foreach (Part p in new[] { covered, open })
                {
                    ctx.Step($"Without the {(p == covered ? "covered" : "open")} Goliath's adjuster");
                    GeometryPartModule g = Geo(p);
                    saved[g] = Refl.Get<List<ICrossSectionAdjuster>>(g, "crossSectionAdjusters");
                    Refl.Set(g, "crossSectionAdjusters", new List<ICrossSectionAdjuster>());
                    yield return Far.Revoxelize("the airliner without one Goliath's adjuster");
                    if (p == covered) volNoCovered = Far.Volume(vehicle);
                    else volNoOpen = Far.Volume(vehicle);
                    Refl.Set(g, "crossSectionAdjusters", saved[g]);
                    saved.Remove(g);
                }
            }
            finally
            {
                foreach (KeyValuePair<GeometryPartModule, List<ICrossSectionAdjuster>> kv in saved)
                    Refl.Set(kv.Key, "crossSectionAdjusters", kv.Value);
            }

            ctx.Record("volume", $"{vol:F3} m3 with both; {volNoCovered:F3} without the covered one's adjuster, {volNoOpen:F3} without the open one's");
            Assert.True(volNoOpen > vol, $"the open Goliath's ducting takes volume out ({vol:F3} against {volNoOpen:F3} m3)");
            // an exit's area is ducted with the opposite sign: it puts volume back (335.6 m3 with it, 331.3 without)
            Assert.True(volNoCovered < vol, $"the covered Goliath's adjuster puts volume back, as an engine exit ({vol:F3} against {volNoCovered:F3} m3)");

            ctx.Step("Both adjusters back");
            yield return Far.Revoxelize("the airliner with both adjusters");
            Assert.Near(vol, Far.Volume(vehicle), 1e-4 * vol, "the volume again (m3)");
        }

        /// <summary>
        /// The stock Kerbal 1 in the VAB (the Kerbal X has launch clamps, whose encrypted config the test instance does not
        /// load): a rocket, so FAR's axis is the stack's (the root part's up), and the stock tanks
        /// voxelize from their colliders. The voxel model's length and largest cross-section against the rocket's bounds,
        /// at least the core and its two boosters' circles across it; drag at zero angle of attack, rising through Mach 1; equal
        /// and opposite side forces either side.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Vab, Craft = Rocket, TimeoutSec = 900)]
        public static IEnumerator Editor_RocketVoxelModelMatchesTheStack(TestContext ctx)
        {
            yield return LoadedCraft("the rocket", "solidBooster.v2", "liquidEngine3.v2");
            VehicleAerodynamics vehicle = Far.EditorVehicle;
            Transform root = EditorLogic.RootPart.transform;
            Vector3 extent = CraftExtent(root.up, root.right, root.forward);
            ctx.Record("extent", extent.ToString("F2"));
            ctx.Record("voxel", $"length {vehicle.Length:F2} m, max cross-section {vehicle.MaxCrossSectionArea:F3} m2, volume {Far.Volume(vehicle):F2} m3");
            Assert.InRange(0.85 * extent.x, 1.1 * extent.x, vehicle.Length, "voxel length against the rocket's height");
            Assert.InRange(0.02 * extent.y * extent.z, extent.y * extent.z, vehicle.MaxCrossSectionArea, "largest cross-section inside the rocket's footprint");
            // the 1.25 m core and its two 1.25 m solid boosters side by side
            double stack = 3 * Math.PI * 0.625 * 0.625;
            Assert.True(vehicle.MaxCrossSectionArea > 0.8 * stack, $"the core and boosters across the stack ({vehicle.MaxCrossSectionArea:F2} against {stack:F2} m2)");
            double[] areas = vehicle.GetCrossSectionAreas();
            Assert.True(areas.All(a => a >= 0 && !double.IsNaN(a)), "every cross-section is a real, non-negative area");

            double sub = Far.Instant(0, 0.5).Cd, trans = Far.Instant(0, 1.1).Cd;
            ctx.Record("Cd", $"Mach 0.5 {sub:F4}, Mach 1.1 {trans:F4}");
            Assert.True(sub > 0, $"drag at zero angle of attack ({sub:F4})");
            Assert.True(trans > 1.3 * sub, $"Cd at Mach 1.1 ({trans:F4}) well above Mach 0.5's ({sub:F4})");
            InstantConditionSimOutput right = Far.Instant(0, 0.5, 4), left = Far.Instant(0, 0.5, -4);
            ctx.Record("sideslip", $"Cy {right.Cy:F5} / {left.Cy:F5}");
            Assert.True(Math.Abs(right.Cy) > 1e-3, $"sideslip gives a side force ({right.Cy:G3})");
            Assert.Near(-right.Cy, left.Cy, 0.05 * Math.Abs(right.Cy), "side force is equal and opposite either side");
        }

        private const string ComSat = "ComSat Lx Unclamped";      // stock, its launch clamp taken off (crafts/)

        /// <summary>
        /// A stock fairing in the editor is voxelized shut whatever the editor shows of it: the editor draws a fairing being
        /// worked on exploded and see-through, and FAR's fairing updater puts every panel back (unexploded, opaque, its
        /// colliders on) before each voxelization. The ComSat Lx with its panels shown exploded and see-through comes out
        /// the model it is with them shut, the panels put back, and the updater keeps the bounds of every panel.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Vab, Craft = ComSat, TimeoutSec = 900)]
        public static IEnumerator Editor_FairingIsVoxelizedShut(TestContext ctx)
        {
            yield return LoadedCraft("the ComSat Lx", "fairingSize1");
            Part fairingPart = EditorLogic.fetch.ship.parts.First(p => p.Modules.Contains<ModuleProceduralFairing>());
            var fairing = fairingPart.FindModuleImplementing<ModuleProceduralFairing>();
            List<FairingPanel> panels = fairing.Panels;
            Assert.True(panels != null && panels.Count > 0, "the fairing has panels");
            var geo = fairingPart.FindModuleImplementing<GeometryPartModule>();
            object updater = ((IEnumerable)Refl.Get(geo, "geometryUpdaters")).Cast<object>().FirstOrDefault(u => u is StockProcFairingGeoUpdater);
            Assert.NotNull(updater, "FAR's fairing updater on the fairing");
            Assert.Equal(panels.Count, Refl.Get<List<Bounds>>(updater, "prevPanelBounds").Count, "the updater keeps the bounds of every panel");
            VehicleAerodynamics v = Far.EditorVehicle;
            double volume = Far.Volume(v), area = v.MaxCrossSectionArea, cd = Far.Instant(0, 0.5).Cd;
            ctx.Record("shut", $"{panels.Count} panels; volume {volume:F3} m3, max cross-section {area:F3} m2, Cd {cd:F5}");

            ctx.Step("The panels shown exploded and see-through, the craft voxelized again");
            foreach (FairingPanel p in panels)
            {
                p.SetTgtExplodedView(1);
                p.SetExplodedView(1);
                p.SetTgtOpacity(0.2f);
                p.SetOpacity(0.2f);
            }

            yield return Far.Revoxelize("the ComSat Lx, its fairing exploded");
            yield return Far.Settled("the ComSat Lx");
            v = Far.EditorVehicle;
            double volume2 = Far.Volume(v), area2 = v.MaxCrossSectionArea, cd2 = Far.Instant(0, 0.5).Cd;
            ctx.Record("exploded", $"volume {volume2:F3} m3, max cross-section {area2:F3} m2, Cd {cd2:F5}");
            Assert.Near(volume, volume2, 0.01 * volume, "volume with the panels shown exploded, the shut fairing's");
            Assert.Near(area, area2, 0.01 * area, "largest cross-section with the panels shown exploded, the shut fairing's");
            Assert.Near(cd, cd2, 0.01 * cd, "Cd at Mach 0.5 with the panels shown exploded, the shut fairing's");
            Assert.Equal(panels.Count, Refl.Get<List<Bounds>>(updater, "prevPanelBounds").Count, "the updater still keeps every panel");
        }

        /// <summary>
        /// Voxelized on Unity's main thread (as FAR does in a development build: VoxelizationThreadpool.RunInMainThread),
        /// the Aeris is the model the worker threads make of it: the same cross-sections and volume, and the static analysis
        /// on it the same to 0.1% (on master it differs in the fourth figure: Cd 0.025886 against 0.025882 at 4 deg).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Editor_MainThreadVoxelizationIsTheThreadedOne(TestContext ctx)
        {
            bool was = VoxelizationThreadpool.RunInMainThread;
            try
            {
                yield return Far.WaitForEditorVoxels();
                yield return Far.Settled("the Aeris");
                double[] threaded = Far.EditorVehicle.GetCrossSectionAreas();
                double vol = Far.Volume(Far.EditorVehicle);
                InstantConditionSimOutput t = Far.Instant(4, 0.5);

                ctx.Step("Voxelized again on the main thread");
                VoxelizationThreadpool.RunInMainThread = true;
                yield return Far.Revoxelize("the Aeris on the main thread");
                yield return Far.Settled("the Aeris on the main thread");
                double[] main = Far.EditorVehicle.GetCrossSectionAreas();
                double vol2 = Far.Volume(Far.EditorVehicle);
                InstantConditionSimOutput m = Far.Instant(4, 0.5);
                double worst = threaded.Zip(main, (a, b) => Math.Abs(a - b)).DefaultIfEmpty(0).Max();
                ctx.Record("models", $"threaded: volume {vol:F4} m3, Cl {t.Cl:F5}, Cd {t.Cd:F5}, Cm {t.Cm:F5}; " +
                                     $"main thread: volume {vol2:F4} m3, Cl {m.Cl:F5}, Cd {m.Cd:F5}, Cm {m.Cm:F5}; " +
                                     $"{threaded.Length} / {main.Length} sections, the most apart {worst:G3} m2");
                Assert.Equal(threaded.Length, main.Length, "as many cross-sections");
                Assert.True(worst <= 1e-6 * threaded.Max(), $"the same cross-sections (the most apart by {worst:G3} m2)");
                Assert.Near(vol, vol2, 1e-6 * vol, "the same volume");
                Assert.Near(t.Cl, m.Cl, 1e-3 * Math.Abs(t.Cl) + 1e-6, "the same Cl at 4 deg");
                Assert.Near(t.Cd, m.Cd, 1e-3 * t.Cd + 1e-7, "the same Cd at 4 deg");
                Assert.Near(t.Cm, m.Cm, 1e-3 * Math.Abs(t.Cm) + 1e-6, "the same Cm at 4 deg");
            }
            finally
            {
                VoxelizationThreadpool.RunInMainThread = was;
            }
        }

        private const string CubeRoot = "Kerbal 1 Cube Root";   // crafts/cube_root.py

        private sealed class AxisModel
        {
            public string Axis;
            public double Length, MaxArea, Volume, Thickness, CdSub, CdTrans, Cl, Cm, Cy;
            public double[] Areas;
        }

        /// <summary>
        /// FAR voxelizes an editor craft in its root part's frame and cuts its cross-sections across the axis of that
        /// frame the craft lies along, by one of three branches of its section code. The stock Kerbal 1 on a cubic strut
        /// root (crafts/cube_root.py), the strut as the game places it (the stack along the root's y) and turned 90 deg in
        /// place (along its x, along its z): one shape in three frames, the parts all where they are. Then the strut turned
        /// 30 deg and 60 deg (the stack between two of the root's axes, nearer its y, its x, its z): FAR's axis is none of
        /// the root's but the stack's, within 15 deg, and the model is cut at a slant to its grid. Each is cut along its own
        /// axis, as long as the rocket, every section a real area; each is the same rocket to within 15% in volume and 20% in
        /// largest cross-section, with drag at zero angle of attack that rises through Mach 1. (On master the slanted models
        /// come out 8-13% fuller and up to 16% wider than the one along the root's y.)
        /// Editor_VoxelModelIsTheSameAlongEveryAxis holds the first three to the same model.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Vab, Craft = CubeRoot, TimeoutSec = 1200)]
        public static IEnumerator Editor_VoxelModelAlongEachRootAxis(TestContext ctx)
        {
            var models = new List<AxisModel>();
            yield return AxisModels(ctx, models, oblique: true);
            AxisModel y = models[0];
            foreach (AxisModel m in models)
            {
                string along = $"along the root's {m.Axis}";
                Assert.Near(y.Volume, m.Volume, 0.15 * y.Volume, "volume " + along + ", the rocket's");
                Assert.Near(y.MaxArea, m.MaxArea, 0.20 * y.MaxArea, "largest cross-section " + along + ", the rocket's");
                Assert.True(m.CdSub > 0, $"drag at zero angle of attack {along} ({m.CdSub:F5})");
                Assert.True(m.CdTrans > 1.3 * m.CdSub, $"Cd at Mach 1.1 ({m.CdTrans:F5}) well above Mach 0.5's ({m.CdSub:F5}) {along}");
            }
        }

        /// <summary>
        /// As Editor_VoxelModelAlongEachRootAxis, and the three models agree: length, volume, largest cross-section, how the
        /// volume is spread along the rocket, and the static analysis (which flies the craft along the world's axes) at
        /// zero and at 4 deg angle of attack and sideslip. Explicit: on master the models along the root's x and z come out
        /// 5-7% fuller (volume) and 7-9% wider (largest cross-section) than along its y, and at 4 deg they lift half as much
        /// (Cl 0.15 against 0.31) and pitch a sixth as much (Cm -0.04 against -0.25), with half the side force. FAR's voxel
        /// interior fill (VehicleVoxel.SolidifyVoxel) sweeps along the grid's y axis only, the root's y in the editor, which
        /// may be why: across the rocket a sweep can close off gaps (between the boosters and the core, say) that one along
        /// it leaves open.
        /// </summary>
        [UITest(Kind = "Check", Explicit = true, StartsAt = StartState.Vab, Craft = CubeRoot, TimeoutSec = 900)]
        public static IEnumerator Editor_VoxelModelIsTheSameAlongEveryAxis(TestContext ctx)
        {
            var models = new List<AxisModel>();
            yield return AxisModels(ctx, models);

            ctx.Step("The three models against each other");
            AxisModel y = models[0];
            foreach (AxisModel m in models.Skip(1))
            {
                string along = $"along the root's {m.Axis} against its y";
                Assert.Near(y.Length, m.Length, 2 * Math.Max(y.Thickness, m.Thickness) + 0.01 * y.Length, "length " + along);
                Assert.Near(y.Volume, m.Volume, 0.02 * y.Volume, "volume " + along);
                Assert.Near(y.MaxArea, m.MaxArea, 0.03 * y.MaxArea, "largest cross-section " + along);
                double[] py = Profile(y.Areas, 10), pm = Profile(m.Areas, 10);
                double same = py.Zip(pm, (a, b) => Math.Abs(a - b)).Max();
                double reversed = py.Zip(pm.Reverse().Select(p => 1 - p), (a, b) => Math.Abs(a - b)).Max();
                ctx.Record("profile_" + m.Axis, $"{string.Join(" ", pm.Select(p => p.ToString("F3")))} (y: {string.Join(" ", py.Select(p => p.ToString("F3")))}); " +
                                                $"off by {same:F4} the same way, {reversed:F4} reversed");
                Assert.True(Math.Min(same, reversed) < 0.03, $"the volume spread along the rocket {along} (off by {Math.Min(same, reversed):F3} of the volume)");
                Assert.Near(y.CdSub, m.CdSub, 0.03 * y.CdSub, "Cd at Mach 0.5 " + along);
                Assert.Near(y.CdTrans, m.CdTrans, 0.03 * y.CdTrans, "Cd at Mach 1.1 " + along);
                Assert.Near(y.Cl, m.Cl, 0.05 * Math.Abs(y.Cl) + 1e-4, "Cl at 4 deg " + along);
                Assert.Near(y.Cm, m.Cm, 0.05 * Math.Abs(y.Cm) + 1e-4, "Cm at 4 deg " + along);
                Assert.Near(y.Cy, m.Cy, 0.05 * Math.Abs(y.Cy) + 1e-4, "Cy at 4 deg sideslip " + along);
            }
        }

        /// <summary>The three cube-root crafts loaded in turn, each one's model checked along its own axis and kept.</summary>
        private static IEnumerator AxisModels(TestContext ctx, List<AxisModel> models, bool oblique = false)
        {
            // the craft, the root's axis the stack lies along (or lies nearest), its name, and whether it lies between two
            var crafts = new List<(string, Vector3, string, bool)>
            {
                (CubeRoot, Vector3.up, "y", false), (CubeRoot + " X", Vector3.right, "x", false), (CubeRoot + " Z", Vector3.forward, "z", false)
            };
            if (oblique)
                crafts.AddRange(new[]
                {
                    (CubeRoot + " Y30", Vector3.up, "y30", true), (CubeRoot + " X60", Vector3.right, "x60", true),
                    (CubeRoot + " Z60", Vector3.forward, "z60", true)
                });
            foreach ((string craft, Vector3 axis, string name, bool slant) in crafts)
            {
                ctx.Step($"{craft}: the stack along the root's {name}");
                if (craft != CubeRoot)
                    yield return Nav.LoadCraft(ctx, ctx.CraftPath(craft, "VAB"));
                // the stack in the root's frame, and FAR's axis for it: the root's own axis, or within 15 deg of the stack
                Vector3 Stack() => Abs(EditorLogic.RootPart.transform.InverseTransformDirection(Vector3.up));
                bool AxisIsTheCrafts(Vector3 main) =>
                    slant ? Vector3.Angle(main, Stack()) < 15 && main != axis && Largest(main) == axis : main == axis;
                yield return Wait.Until(() => Far.EditorReady && EditorLogic.RootPart != null && EditorLogic.RootPart.partInfo.name == "strutCube" &&
                                              EditorLogic.fetch.ship.shipName == craft &&
                                              AxisIsTheCrafts(Refl.Get<Vector3>(Far.EditorVehicle, "_vehicleMainAxis")),
                                        120, $"FAR's voxel model of {craft} along its root's {name}", shot: false);
                // the static analysis on the new model too (under a coverage recorder it can lag the model)
                yield return Wait.Until(() => Far.EditorReady && Far.Instant(2, 0.5).Cd > 0, 120, $"the static analysis of {craft}", shot: false);
                yield return Far.Settled(craft);

                Vector3 main = Refl.Get<Vector3>(Far.EditorVehicle, "_vehicleMainAxis");
                ctx.Record("axis_" + name, $"stack in the root's frame {Stack():F3}, FAR's axis {main:F3} ({Vector3.Angle(main, Stack()):F1} deg off)");
                if (slant)
                {
                    Assert.True(Vector3.Angle(main, Stack()) < 15, $"FAR's axis is the stack's ({main:F3} against {Stack():F3})");
                    Assert.True(main != Vector3.up && main != Vector3.right && main != Vector3.forward, $"FAR's axis is none of the root's ({main:F3})");
                }
                else
                {
                    Assert.True(Vector3.Dot(Stack(), axis) > 0.999f, $"the root's {name} axis is the stack's ({Stack():F3})");
                    Assert.Equal(axis, main, $"FAR's axis is the root's {name}");
                }

                VehicleAerodynamics v = Far.EditorVehicle;
                double extent = CraftExtent(Vector3.up, Vector3.right, Vector3.forward).x;
                InstantConditionSimOutput aoa = Far.Instant(4, 0.5), beta = Far.Instant(0, 0.5, 4);
                var m = new AxisModel
                {
                    Axis = name, Length = v.Length, MaxArea = v.MaxCrossSectionArea, Volume = Far.Volume(v), Thickness = v.SectionThickness,
                    Areas = Trimmed(v.GetCrossSectionAreas()), CdSub = Far.Instant(0, 0.5).Cd, CdTrans = Far.Instant(0, 1.1).Cd,
                    Cl = aoa.Cl, Cm = aoa.Cm, Cy = beta.Cy
                };
                ctx.Record("model_" + name, $"length {m.Length:F3} m ({m.Areas.Length} sections of {m.Thickness:F4} m), max cross-section {m.MaxArea:F4} m2, " +
                                            $"volume {m.Volume:F3} m3; Cd {m.CdSub:F5} / {m.CdTrans:F5}, 4 deg Cl {m.Cl:F5} Cm {m.Cm:F5}, 4 deg sideslip Cy {m.Cy:F5}");
                Assert.InRange(0.85 * extent, 1.1 * extent, m.Length, $"the model along the root's {name} as long as the rocket ({extent:F2} m)");
                Assert.True(m.Areas.All(a => a >= 0 && !double.IsNaN(a)), "every cross-section is a real, non-negative area");
                models.Add(m);
            }
        }

        private static Vector3 Abs(Vector3 v) => new Vector3(Math.Abs(v.x), Math.Abs(v.y), Math.Abs(v.z));

        /// <summary>The unit axis of a vector's largest component.</summary>
        private static Vector3 Largest(Vector3 v) =>
            Math.Abs(v.y) >= Math.Abs(v.x) && Math.Abs(v.y) >= Math.Abs(v.z) ? Vector3.up : Math.Abs(v.x) > Math.Abs(v.z) ? Vector3.right : Vector3.forward;

        /// <summary>The cross-sections with the empty ones at either end left off.</summary>
        private static double[] Trimmed(double[] areas)
        {
            int a = 0, b = areas.Length - 1;
            while (a < b && areas[a] <= 0) a++;
            while (b > a && areas[b] <= 0) b--;
            return areas.Skip(a).Take(b - a + 1).ToArray();
        }

        /// <summary>The share of the volume ahead of each of n - 1 evenly spaced cuts along the model.</summary>
        private static double[] Profile(double[] areas, int n)
        {
            var cum = new double[areas.Length + 1];
            for (int i = 0; i < areas.Length; i++)
                cum[i + 1] = cum[i] + areas[i];
            var p = new double[n - 1];
            for (int k = 1; k < n; k++)
            {
                double x = k * areas.Length / (double)n;
                int i0 = Math.Min((int)x, areas.Length - 1);
                p[k - 1] = (cum[i0] + (x - i0) * areas[i0]) / cum[areas.Length];
            }

            return p;
        }

        /// <summary>
        /// A craft loaded in place of another: its parts there, FAR's voxel model as long as the craft (the editor can be
        /// ready, and steady, with the last craft's model for a while after a load), and settled.
        /// </summary>
        private static IEnumerator LoadedCraft(string what, params string[] parts)
        {
            yield return Wait.Until(() => Far.EditorReady && parts.All(n => EditorLogic.fetch.ship.parts.Any(p => p.partInfo.name == n)),
                                    120, "FAR's editor on " + what, shot: false);
            Transform root = EditorLogic.RootPart.transform;
            double length = CraftExtent(root.up, root.right, root.forward).x;
            yield return Wait.Until(() => Far.EditorReady && Far.EditorVehicle.Length > 0.85 * length && Far.EditorVehicle.Length < 1.1 * length,
                                    120, $"FAR's voxel model of {what}, {length:F1} m long", shot: false,
                                    detail: () => (Far.EditorReady ? $"FAR's model is {Far.EditorVehicle.Length:F2} m long" : "FAR's editor not ready") +
                                                  "; parts the length leaves out: " + UnmeasuredParts() +
                                                  $"; with the meshes not drawn too the craft is {CraftExtent(root.up, root.right, root.forward, true).x:F2} m long" +
                                                  "; not drawn: " + HiddenMeshes() + "; not readable (their bounds measured): " + UnreadableMeshes() +
                                                  "; with points not finite (left out): " + NonFiniteMeshes() + "; " + Safely(() => FarModelShape(root)) + "; " + Safely(() => FarGeometryBeyond(root.up)));
            yield return Far.Settled(what);
        }

        /// <summary>
        /// The parts none of whose vertices CraftExtent counts, each with why: its mesh filters, those without a mesh,
        /// those not drawn, and those whose mesh has no vertices to read.
        /// </summary>
        private static string UnmeasuredParts()
        {
            var lines = new List<string>();
            foreach (Part p in EditorLogic.fetch.ship.parts)
            {
                MeshFilter[] mfs = p.GetComponentsInChildren<MeshFilter>();
                int noMesh = mfs.Count(mf => mf.sharedMesh == null);
                int hidden = mfs.Count(mf => mf.sharedMesh != null && mf.GetComponent<Renderer>()?.enabled != true);
                int empty = mfs.Count(mf => mf.sharedMesh != null && mf.GetComponent<Renderer>()?.enabled == true && mf.sharedMesh.vertices.Length == 0);
                if (mfs.Length - noMesh - hidden - empty == 0)
                    lines.Add($"{p.partInfo.name} ({mfs.Length} mesh filters: {noMesh} without a mesh, {hidden} not drawn, {empty} without vertices)");
            }

            return lines.Count == 0 ? "none" : string.Join(", ", lines);
        }

        /// <summary>A diagnostic's text, or what it threw: it goes into a failure's message, which it must not replace.</summary>
        private static string Safely(Func<string> diagnostic)
        {
            try { return diagnostic(); }
            catch (Exception e) { return $"({e.GetType().Name}: {e.Message})"; }
        }

        /// <summary>
        /// How FAR measured its model: Length is its sections along its main axis (from the parts' orientations), so an
        /// axis off the craft's own makes it longer; with the voxel's bounds.
        /// </summary>
        private static string FarModelShape(Transform root)
        {
            if (!Far.EditorReady)
                return "FAR's editor not ready";
            VehicleAerodynamics v = Far.EditorVehicle;
            Vector3 axis = Refl.Get<Vector3>(v, "_vehicleMainAxis");
            double thickness = Refl.Get<double>(v, "_sectionThickness");
            return $"FAR's main axis {axis} ({Vector3.Angle(axis, root.up):F1} deg from the root's up), " +
                   $"{(thickness > 0 ? v.Length / thickness : 0):F0} sections of {thickness:F3} m, voxel bounds {v.VoxelBounds.size}";
        }

        /// <summary>
        /// FAR's geometry (each part's GeometryPartModule meshes, its vertices where FAR places them) reaching more than
        /// 0.25 m past the craft's drawn meshes along <paramref name="axis" />: part/mesh and its span from the craft's start.
        /// </summary>
        private static string FarGeometryBeyond(Vector3 axis)
        {
            double lo = double.PositiveInfinity, hi = double.NegativeInfinity;
            foreach (Part p in EditorLogic.fetch.ship.parts)
                foreach (MeshFilter mf in p.GetComponentsInChildren<MeshFilter>())
                {
                    if (mf.sharedMesh == null || mf.GetComponent<Renderer>()?.enabled != true) continue;
                    foreach (Vector3 w in Far.MeshPoints(mf))
                    {
                        double d = Vector3.Dot(w, axis);
                        lo = Math.Min(lo, d);
                        hi = Math.Max(hi, d);
                    }
                }

            var lines = new List<string>();
            foreach (Part p in EditorLogic.fetch.ship.parts)
            {
                GeometryPartModule geo = p.FindModuleImplementing<GeometryPartModule>();
                if (geo?.meshDataList == null) continue;
                foreach (GeometryMesh m in geo.meshDataList)
                {
                    if (!m.valid || m.vertices == null || m.vertices.Length == 0) continue;
                    // FAR keeps the vertices in its vessel frame (thisToVesselMatrix of the mesh's local ones): back to world
                    Matrix4x4 toWorld = m.meshLocalToWorld * m.thisToVesselMatrix.inverse;
                    double a = double.PositiveInfinity, b = double.NegativeInfinity;
                    foreach (Vector3 v in m.vertices)
                    {
                        double d = Vector3.Dot(toWorld.MultiplyPoint3x4(v), axis);
                        a = Math.Min(a, d);
                        b = Math.Max(b, d);
                    }

                    if (a < lo - 0.25 || b > hi + 0.25)
                        lines.Add($"{p.partInfo.name}/{(m.meshTransform != null ? m.meshTransform.name : "?")} {a - lo:F2}..{b - lo:F2} m");
                }
            }

            return $"the craft's drawn meshes 0..{hi - lo:F2} m along the root's up; FAR's geometry past them: " +
                   (lines.Count == 0 ? "none" : string.Join(", ", lines));
        }

        /// <summary>The drawn meshes with points that are not finite (Far.MeshPoints leaves them out), as part/mesh (count).</summary>
        private static string NonFiniteMeshes()
        {
            var lines = new List<string>();
            foreach (Part p in EditorLogic.fetch.ship.parts)
                foreach (MeshFilter mf in p.GetComponentsInChildren<MeshFilter>())
                    if (mf.sharedMesh != null && mf.GetComponent<Renderer>()?.enabled == true && Far.NonFinitePoints(mf) is int n && n > 0)
                        lines.Add($"{p.partInfo.name}/{mf.name} ({n} of {mf.sharedMesh.vertexCount}; at {mf.transform.position})");
            return lines.Count == 0 ? "none" : string.Join(", ", lines);
        }

        /// <summary>The drawn meshes whose vertices cannot be read (Far.MeshPoints measures their bounds), as part/mesh.</summary>
        private static string UnreadableMeshes()
        {
            var lines = new List<string>();
            foreach (Part p in EditorLogic.fetch.ship.parts)
                foreach (MeshFilter mf in p.GetComponentsInChildren<MeshFilter>())
                    if (mf.sharedMesh != null && !mf.sharedMesh.isReadable && mf.GetComponent<Renderer>()?.enabled == true)
                        lines.Add($"{p.partInfo.name}/{mf.name} ({mf.sharedMesh.vertexCount} vertices)");
            return lines.Count == 0 ? "none" : string.Join(", ", lines);
        }

        /// <summary>The meshes CraftExtent leaves out as not drawn (their renderer off), as part/mesh, with vertex counts.</summary>
        private static string HiddenMeshes()
        {
            var lines = new List<string>();
            foreach (Part p in EditorLogic.fetch.ship.parts)
                foreach (MeshFilter mf in p.GetComponentsInChildren<MeshFilter>(true))
                    if (mf.sharedMesh != null && mf.GetComponent<Renderer>()?.enabled != true)
                        lines.Add($"{p.partInfo.name}/{mf.name} ({mf.sharedMesh.vertexCount} vertices{(mf.gameObject.activeInHierarchy ? "" : ", inactive")})");
            return lines.Count == 0 ? "none" : string.Join(", ", lines);
        }

        /// <summary>
        /// The craft's size along three axes: the spread of its meshes' vertices (in world space) along each; those drawn,
        /// or with <paramref name="hiddenToo" /> every mesh of its active objects.
        /// </summary>
        private static Vector3 CraftExtent(Vector3 a, Vector3 b, Vector3 c, bool hiddenToo = false)
        {
            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            foreach (Part p in EditorLogic.fetch.ship.parts)
                foreach (MeshFilter mf in p.GetComponentsInChildren<MeshFilter>())
                {
                    if (mf.sharedMesh == null || !hiddenToo && mf.GetComponent<Renderer>()?.enabled != true) continue;
                    foreach (Vector3 w in Far.MeshPoints(mf))
                    {
                        var d = new Vector3(Vector3.Dot(w, a), Vector3.Dot(w, b), Vector3.Dot(w, c));
                        min = Vector3.Min(min, d);
                        max = Vector3.Max(max, d);
                    }
                }

            return max - min;
        }
    }
}
