using System.Collections;
using System.Linq;
using FerramAerospaceResearch.FARPartGeometry.GeometryModification;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARPartGeometry;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>FAR's geometry of an engine following its shroud: the shroud goes when the stage below it does.</summary>
    public static class StagingGeometryTests
    {
        /// <summary>
        /// The Kerbal 1's upper engine (a Terrier) wears a shroud (ModuleJettison) down to the decoupler under it.
        /// Decoupled from the stage below, the engine drops its shroud, FAR's shroud updater sees it go and rebuilds the
        /// engine's meshes without the shroud's, and the vessel is voxelized again.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Flight_StagingJettisonsTheEngineShroud(TestContext ctx)
        {
            yield return Wait.Seconds(3);
            Vessel v = FlightGlobals.ActiveVessel;
            yield return Wait.Until(() => FARAPI.VesselVoxelizationCompletedEver(v), 90, "the rocket voxelized", shot: false);
            Part engine = v.parts.FirstOrDefault(p => p.partInfo.name == "liquidEngine3.v2");
            Assert.NotNull(engine, "the Kerbal 1's Terrier");
            var shroud = engine.FindModuleImplementing<ModuleJettison>();
            Assert.NotNull(shroud, "the Terrier's shroud");
            Assert.False(shroud.isJettisoned, "the shroud on before staging");
            Part below = engine.children.FirstOrDefault(p => p.FindModuleImplementing<ModuleDecouple>() != null);
            Assert.NotNull(below, "the decoupler under the Terrier");
            var geo = engine.FindModuleImplementing<GeometryPartModule>();
            Assert.NotNull(geo, "FAR's geometry module on the Terrier");
            object updater = ((IEnumerable)Refl.Get(geo, "geometryUpdaters")).Cast<object>().FirstOrDefault(u => u is StockJettisonTransformGeoUpdater);
            Assert.NotNull(updater, "FAR's shroud updater on the Terrier");
            Assert.True((bool)Refl.Get(updater, "fairingVisible"), "the updater sees the shroud on");
            Transform shroudModel = shroud.jettisonTransform;
            int ShroudMeshes() => geo.meshDataList?.Count(m => m.meshTransform != null && m.meshTransform.IsChildOf(shroudModel) && m.gameObjectActiveInHierarchy) ?? -1;
            int meshesBefore = geo.meshDataList?.Count ?? 0, shroudBefore = ShroudMeshes();
            Assert.True(shroudBefore > 0, $"the shroud's meshes in FAR's geometry of the Terrier ({shroudBefore})");
            using var log = new ExceptionLog();

            ctx.Step("the stage below decoupled");
            below.FindModuleImplementing<ModuleDecouple>().Decouple();
            yield return Wait.Until(() => shroud.isJettisoned, 10, "the shroud jettisoned", shot: false);
            yield return Wait.Until(() => !(bool)Refl.Get(updater, "fairingVisible"), 30, "FAR's shroud updater to see the shroud gone", shot: false);
            yield return Wait.Until(() => ShroudMeshes() == 0, 20, "FAR to rebuild the Terrier's meshes without its shroud",
                                    shot: false, detail: () => $"{ShroudMeshes()} shroud meshes of {geo.meshDataList?.Count}");
            Vessel upper = engine.vessel;
            var aero = upper.GetComponent<FARVesselAero>();
            yield return Wait.Until(() => FARAPI.VesselVoxelizationCompletedEver(upper) && aero.isValid, 60, "the upper stage voxelized", shot: false);
            ctx.Record("terrier", $"meshes {meshesBefore} ({shroudBefore} the shroud's) then {geo.meshDataList?.Count}; " +
                                  $"upper stage max cross-section {aero.MaxCrossSectionArea:F3} m2");
            log.AssertNone("jettisoning the shroud");
        }

        /// <summary>
        /// In the VAB, the Kerbal 1's Terrier with its shroud toggled off (its editor tweakable): FAR's shroud updater sees it go at the next voxelization and
        /// rebuilds the engine's meshes without the shroud's; shown again, they are back.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Vab, Craft = "Kerbal 1", TimeoutSec = 600)]
        public static IEnumerator Editor_ShroudHiddenAndShownRebuildsTheEngine(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            Part engine = EditorLogic.fetch.ship.parts.FirstOrDefault(p => p.partInfo.name == "liquidEngine3.v2");
            Assert.NotNull(engine, "the Kerbal 1's Terrier");
            var shroud = engine.FindModuleImplementing<ModuleJettison>();
            Assert.NotNull(shroud, "the Terrier's shroud");
            var geo = engine.FindModuleImplementing<GeometryPartModule>();
            object updater = ((IEnumerable)Refl.Get(geo, "geometryUpdaters")).Cast<object>().FirstOrDefault(u => u is StockJettisonTransformGeoUpdater);
            Assert.NotNull(updater, "FAR's shroud updater on the Terrier");
            Assert.True((bool)Refl.Get(updater, "fairingVisible"), "the updater sees the shroud on");
            Transform shroudModel = shroud.jettisonTransform;
            int ShroudMeshes() => geo.meshDataList?.Count(m => m.meshTransform != null && m.meshTransform.IsChildOf(shroudModel) && m.gameObjectActiveInHierarchy) ?? -1;
            Assert.True(ShroudMeshes() > 0, $"the shroud's meshes in FAR's geometry of the Terrier ({ShroudMeshes()})");
            using var log = new ExceptionLog();
            BaseField hide = shroud.Fields["shroudHideOverride"];
            Assert.NotNull(hide, "the shroud's editor toggle (shroudHideOverride)");
            void Toggle(bool hidden)
            {
                // as its tweakable sets it: the field, then its change callback
                hide.SetValue(hidden, shroud);
                hide.uiControlEditor?.onFieldChanged?.Invoke(hide, !hidden);
            }

            try
            {
                ctx.Step("the shroud hidden");
                // setting the transform inactive alone is undone by the module before the next voxelization
                Toggle(true);
                yield return Wait.Until(() => !shroudModel.gameObject.activeSelf, 10, "the shroud hidden by its toggle", shot: false);
                yield return Far.Revoxelize("the Terrier without its shroud");
                Assert.False((bool)Refl.Get(updater, "fairingVisible"), "the updater sees the shroud gone");
                Assert.Equal(0, ShroudMeshes(), "the Terrier's meshes rebuilt without the shroud's");
            }
            finally
            {
                Toggle(false);
            }

            ctx.Step("the shroud shown again");
            yield return Wait.Until(() => shroudModel.gameObject.activeSelf, 10, "the shroud shown by its toggle", shot: false);
            yield return Far.Revoxelize("the Terrier with its shroud");
            Assert.True((bool)Refl.Get(updater, "fairingVisible"), "the updater sees the shroud back");
            Assert.True(ShroudMeshes() > 0, "the shroud's meshes back in the Terrier's geometry");
            log.AssertNone("hiding and showing the shroud in the editor");
        }
    }
}
