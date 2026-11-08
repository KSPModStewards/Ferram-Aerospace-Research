using System;
using System.Collections;
using System.IO;
using System.Text;
using KspTest;
using KspTest.RP1;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests.RP1
{
    /// <summary>FAR's thermal area modes (KSPModStewards FAR #156, <see cref="ThermalAreaModes"/>) on an RO capsule entry.</summary>
    public static class ThermalModeRp1Tests
    {
        /// <summary>
        /// The same KCT reentry simulation flown under each mode (reverting to the editor between them), holding a fixed
        /// angle of attack to the airflow, under TimeControl's hyper-warp at physics accuracy 1. Records per part the
        /// peak skin and internal temperatures, ablator used, and the areas and occlusion multipliers at the peak shock
        /// temperature; the trajectory is recorded to check the modes flew alike. Params: peKm (30), apKm (250),
        /// entryAltKm (110), incDeg (28.6), aoa (180 = heat shield first), endAltKm (20), modes (Master+Pr156+Hybrid), hyper (10).
        /// </summary>
        [UITest("Far_ThermalEntry_RP1", Explicit = true, Kind = "Measure", NewGame = "career", Requires = "RP-1", StartsAt = StartState.Vab, Craft = "Mk1-3 Test", TimeoutSec = 7200)]
        public static IEnumerator Far_ThermalEntry_RP1(TestContext ctx)
        {
            Ugui.DismissPopups();
            ctx.Record("farBuild", FarSim.FarVersion());
            double peKm = ctx.ParamDouble("peKm", 30), apKm = ctx.ParamDouble("apKm", 250), entryAltKm = ctx.ParamDouble("entryAltKm", 110);
            double aoa = ctx.ParamDouble("aoa", 180), endAltKm = ctx.ParamDouble("endAltKm", 20), hyper = ctx.ParamDouble("hyper", 10);
            // KCT aims the entry at the space center: 0 never passes over it, and the sim falls back to the pad
            double incDeg = ctx.ParamDouble("incDeg", 28.6);
            string[] modes = (ctx.Param("modes") ?? "Master+Pr156+Hybrid").Split('+');
            string tag = ctx.Param("tag", "Mk1-3_Test_RP1");
            ThermalAreaModes.Install();
            try
            {
                ctx.Record("exposedAreaLimited", ThermalAreaModes.ExposedAreaLimited);

                var summary = new StringBuilder(FarThermalModeTests.EntrySummaryHeader);
                foreach (string modeName in modes)
                {
                    var mode = (ThermalAreaModes.Mode)Enum.Parse(typeof(ThermalAreaModes.Mode), modeName, true);
                    ctx.Step($"Entry, mode {mode}");
                    ThermalAreaModes.Current = mode;
                    yield return Rp1.Simulate(ctx, new SimSpec
                    {
                        StartInOrbit = true, Mode = "Reentry", PeKm = peKm, Arrival = "Apoapsis", ApKm = apKm, EntryAltKm = entryAltKm,
                        IncDeg = incDeg, Attitude = "Retrograde",
                    });
                    yield return FarThermalModeTests.WaitVoxels();
                    Vessel v = FlightGlobals.ActiveVessel;
                    if (v.altitude < entryAltKm * 500)
                        throw new UITestFailure($"the sim did not start in orbit (alt {v.altitude / 1000:F1} km, {v.situation}): KCT rejected the entry?");
                    ThermalAreaModes.Set(mode, v);
                    Flight.IgnoreMaxTemperature(true);
                    Ctrl.Sas(false);
                    string Areas() => string.Join("; ", v.parts.ConvertAll(p => $"{p.partInfo.name} {ThermalAreaModes.FarTotal(p):F2} [{string.Join(" ", Array.ConvertAll(ThermalAreaModes.FarFaces(p), x => x.ToString("F2")))}]"));
                    ctx.Record($"{mode}.farAreasPlaced", Areas());
                    Flight.PointAtAoA(aoa);
                    if (ctx.Param("revoxel", "off") == "on")
                    {
                        // voxelize again with the vessel held still at its entry attitude (KCT turned it while FAR voxelized)
                        foreach (Part p in v.parts)
                            if (p.Rigidbody != null) p.Rigidbody.angularVelocity = UnityEngine.Vector3.zero;
                        yield return Wait.Seconds(0.5);
                        FarApi.RequestRevoxel(v);
                        yield return Wait.Seconds(1);
                        yield return FarThermalModeTests.WaitVoxels();
                        ctx.Record($"{mode}.farAreasRevoxel", Areas());
                    }
                    ctx.Record($"{mode}.start", $"UT {Planetarium.GetUniversalTime():F0}, alt {v.altitude / 1000:F1} km, speed {v.srfSpeed:F0} m/s");
                    yield return FarThermalModeTests.FlyEntry(ctx, v, mode, aoa, endAltKm, 0, hyper, tag, summary);
                    Flight.IgnoreMaxTemperature(false);
                    yield return Rp1.RevertToEditorViaWindow();
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, $"entry_{tag}_parts.csv"), summary.ToString());
            }
            finally
            {
                FarThermalModeTests.PutBack();
            }
        }

        /// <summary>
        /// The area each part shows the airflow when the heat shield leads, at a few angles of attack: ray-traced from the
        /// render meshes, against stock's AreaOccluded (what OcclusionData's cross section is built from) and the
        /// AreaOccluded master hands KSP from FAR's voxels, both on the face toward the flow (YN at 0). Params: craft
        /// (Mk1-3 Test), aoas (0,10,20), grid (400).
        /// </summary>
        [UITest("Far_FlowFacingArea_RP1", Explicit = true, Kind = "Measure", Requires = "RP-1", StartsAt = StartState.Flight, Craft = "Mk1-3 Test", TimeoutSec = 900)]
        public static IEnumerator Far_FlowFacingArea_RP1(TestContext ctx)
        {
            yield return FarThermalModeTests.WaitVoxels();
            ThermalAreaModes.Install();
            Vessel v = FlightGlobals.ActiveVessel;
            var ps = new System.Collections.Generic.List<Part>(v.parts);
            int n = ps.Count, grid = (int)ctx.ParamDouble("grid", 400);
            string[] faces = { "XP", "XN", "YP", "YN", "ZP", "ZN" };
            string Occluded(Part p) => string.Join(" ", Array.ConvertAll(p.DragCubes.AreaOccluded, x => x.ToString("F2")));

            ThermalAreaModes.Set(ThermalAreaModes.Mode.Pr156, v);
            yield return Wait.Seconds(1);
            foreach (Part p in ps) ctx.Record($"{p.partInfo.name}.stockAreaOccluded", $"{Occluded(p)} ({string.Join(" ", faces)})");
            ThermalAreaModes.Set(ThermalAreaModes.Mode.Master, v);
            yield return Wait.Seconds(1);
            foreach (Part p in ps) ctx.Record($"{p.partInfo.name}.masterAreaOccluded", Occluded(p));
            ThermalAreaModes.Set(ThermalAreaModes.Mode.Native, v);

            // Trace sends its rays along Fibonacci(1)[0]: turn the vessel so they travel from the shield toward the nose
            Vector3 d = RayAreas.Fibonacci(1)[0], up = v.transform.up, side = v.transform.right, pivot = v.CoM;
            foreach (string s in (ctx.Param("aoas") ?? "0,10,20").Split(','))
            {
                double aoa = double.Parse(s, System.Globalization.CultureInfo.InvariantCulture);
                Vector3 flow = Quaternion.AngleAxis((float)aoa, side) * up;
                Quaternion q = Quaternion.FromToRotation(flow, d);
                var items = RayAreas.PartItems(ps, out int unreadable);
                foreach (RayAreas.Item it in items)
                {
                    it.Pos = pivot + q * (it.Pos - pivot);
                    it.Rot = q * it.Rot;
                }
                var res = new RayAreas.Result();
                yield return RayAreas.Trace(items, n, 1, grid, res);
                double total = 0;
                var sb = new StringBuilder();
                for (int i = 0; i < n; i++)
                {
                    total += res.Projected[i][0];
                    sb.Append($"{ps[i].partInfo.name} {res.Projected[i][0]:F3}; ");
                }
                ctx.Record($"ray.aoa{s}", $"{sb}total {total:F3} m2 (spacing {res.Spacing * 100:F1} cm, {unreadable} unreadable)");
            }
        }
    }
}
