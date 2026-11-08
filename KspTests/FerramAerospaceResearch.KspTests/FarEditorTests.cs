using System;
using System.Collections;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>FAR in the editor: its static analysis and voxel grid on a loaded craft.</summary>
    public static class FarEditorTests
    {
        /// <summary>
        /// FAR's voxel grid on a craft with landing gear and control surfaces, sampled in the editor with the gear
        /// deployed (as loaded), retracted and deployed again: element size, bounds, per-part face areas and Cl/Cd/Cm. Made
        /// for VehicleVoxel's sizing change (element size from the bounds of the parts that are neither control
        /// surfaces nor wheels), where the element and every other part's areas should stop moving with the gear.
        /// Params: craft (default PoofySkirt) or crafts (a "+"-separated list, each loaded in turn and its values
        /// prefixed with the craft name), mach (default 0.5), alphas (default "0,5").
        /// </summary>
        [UITest("Far_VoxelSize_Editor", Explicit = true, Kind = "Check", StartsAt = StartState.Sph, Craft = "PoofySkirt", TimeoutSec = 900)]
        public static IEnumerator Far_VoxelSize_Editor(TestContext ctx)
        {
            double mach = ctx.ParamDouble("mach", 0.5);
            string[] alphas = (ctx.Param("alphas") ?? "0,5").Split(',');
            var rows = new System.Text.StringBuilder(FarApi.PartAreasHeader);
            string[] crafts = ctx.Param("crafts") != null ? ctx.Param("crafts").Split('+') : new[] { (string)null };
            foreach (string craftName in crafts)
            {
                string prefix = "";
                if (craftName != null)
                {
                    ctx.Step("Craft " + craftName.Trim());
                    yield return Nav.EnsureState(ctx, StartState.Sph, ctx.CraftPath(craftName.Trim(), "SPH"));
                    prefix = craftName.Trim() + ".";
                }
                Ugui.DismissPopups();
                yield return Wait.Seconds(8);   // FAR voxelizes the craft after load
                yield return VoxelSizeEditorCraft(ctx, prefix, mach, alphas, rows);
            }
            System.IO.File.WriteAllText(System.IO.Path.Combine(ctx.ResultsDir, "voxelsize-editor.csv"), rows.ToString());
        }

        private static IEnumerator VoxelSizeEditorCraft(TestContext ctx, string prefix, double mach, string[] alphas, System.Text.StringBuilder rows)
        {
            var parts = EditorLogic.fetch.ship.parts;
            var gear = new System.Collections.Generic.List<ModuleWheels.ModuleWheelDeployment>();
            foreach (Part p in parts) gear.AddRange(p.FindModulesImplementing<ModuleWheels.ModuleWheelDeployment>());
            ctx.Record(prefix + "gearParts", gear.Count);

            // the SPH loads the gear deployed; EventToggle propagates to symmetry partners, so toggle one per symmetry group
            var toggles = new System.Collections.Generic.List<ModuleWheels.ModuleWheelDeployment>();
            foreach (var d in gear)
            {
                bool covered = false;
                foreach (var t in toggles) if (t.part == d.part || t.part.symmetryCounterparts.Contains(d.part)) covered = true;
                if (!covered) toggles.Add(d);
            }
            foreach (string stateName in new[] { "gearDown", "gearUp", "gearDown2" })
            {
                string state = prefix + stateName;
                if (stateName != "gearDown")
                {
                    FarApi.PartAreas(parts, null, null, out double before, out _);
                    foreach (var d in toggles) d.Events["EventToggle"].Invoke();
                    yield return Wait.Seconds(3);
                    // the deploy animation re-voxelizes the craft (rate limited); wait for the areas to move, then settle
                    double after = before;
                    yield return Wait.Until(() => { FarApi.PartAreas(parts, null, null, out after, out _); return Math.Abs(after - before) > 1e-6; }, 25, "re-voxelization after the gear toggle", shot: false);
                    yield return Wait.Seconds(5);
                }
                string gs = "";
                foreach (var d in gear) gs += (gs.Length > 0 ? "|" : "") + d.stateString + ":" + d.fsm?.CurrentState?.name;
                ctx.Record(state + ".gearStates", gs);
                Assert.True(FarApi.VoxelInfo(out double el, out Vector3 bs, out double len, out double mcs), "FAR voxel info (" + state + ")");
                FarApi.PartAreas(parts, state, rows, out double areaAll, out double areaFixed);
                ctx.Record(state + ".elementSize", Math.Round(el, 5));
                ctx.Record(state + ".boundsX", Math.Round(bs.x, 3)); ctx.Record(state + ".boundsY", Math.Round(bs.y, 3)); ctx.Record(state + ".boundsZ", Math.Round(bs.z, 3));
                ctx.Record(state + ".boundsVolume", Math.Round(bs.x * bs.y * bs.z, 3));
                ctx.Record(state + ".length", Math.Round(len, 3)); ctx.Record(state + ".maxCrossSection", Math.Round(mcs, 4));
                ctx.Record(state + ".areaAll", Math.Round(areaAll, 3)); ctx.Record(state + ".areaFixed", Math.Round(areaFixed, 3));
                foreach (string a in alphas)
                {
                    double alpha = double.Parse(a.Trim(), System.Globalization.CultureInfo.InvariantCulture);
                    Assert.True(FarApi.EditorClCdCm(alpha, mach, 0.0, out double cl, out double cd, out double cm), "FAR editor sim call");
                    string k = $"{state}.alpha{alpha:0}.";
                    ctx.Record(k + "Cl", Math.Round(cl, 4)); ctx.Record(k + "Cd", Math.Round(cd, 5)); ctx.Record(k + "Cm", Math.Round(cm, 5));
                }
            }
        }
    }
}
