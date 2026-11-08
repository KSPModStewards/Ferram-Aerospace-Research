using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using HarmonyLib;
using KspTest;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The thermal areas FAR hands KSP (KSPModStewards FAR #156), measured and flown under each variant of FAR's
    /// UpdateThermodynamicsPre in one session (<see cref="ThermalAreaModes"/>), against a ray-traced reference.
    /// </summary>
    public static class FarThermalModeTests
    {
        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
        public static string F(double x) => x.ToString("G6", Inv);

        /// <summary>"Name@VAB" or "Name@SPH" (VAB by default).</summary>
        private static (string name, string facility) CraftSpec(string raw)
        {
            string[] s = raw.Trim().Split('@');
            return (s[0], s.Length > 1 ? s[1].ToUpperInvariant() : "VAB");
        }

        private static IEnumerator LaunchCraft(TestContext ctx, string name, string facility)
        {
            string path = ctx.CraftPath(name, facility);
            yield return Nav.EnsureState(ctx, StartState.Flight, path, facility == "SPH" ? "Runway" : "LaunchPad");
            yield return WaitVoxels();
        }

        public static IEnumerator WaitVoxels()
        {
            MethodInfo valid = Reflect.TryFindType("FerramAerospaceResearch.FARAPI")?.GetMethod("VesselVoxelizationCompletedAndValid", BindingFlags.Public | BindingFlags.Static);
            yield return Wait.Until(() => FlightGlobals.ActiveVessel != null && !FlightGlobals.ActiveVessel.packed &&
                                          (valid == null || (bool)valid.Invoke(null, new object[] { FlightGlobals.ActiveVessel })), 120, "FAR voxelization");
            yield return Wait.Seconds(2);
        }

        /// <summary>
        /// Per part, on the pad/runway: FAR's voxel area (ProjectedAreas.totalArea), stock's drag cube area
        /// (PostOcclusionArea, with stock occlusion restored), the radiativeArea each mode hands KSP, the summed triangle
        /// area of the part's render meshes (no occlusion: an upper bound) and a ray-traced radiating area: 4 x the
        /// mean visible projected area over directions, which is the area times the view factor to open space
        /// (Cauchy's formula; exact surface area for a convex vessel). Also the projected area toward each direction
        /// (what sun and body heating use): ray-traced against FAR's ProjectedAreaWorld and stock's GetCubeAreaDir.
        /// A cube, sphere and one-sided quad check the tracer first. Params: crafts ("+"-separated Name@VAB|SPH),
        /// dirs (96), grid (rays across the vessel per direction, 220).
        /// </summary>
        [UITest("Far_ThermalAreaTruth", Explicit = true, Kind = "Measure", StartsAt = StartState.SpaceCenter, TimeoutSec = 1800)]
        public static IEnumerator Far_ThermalAreaTruth(TestContext ctx)
        {
            ctx.Record("farBuild", FarSim.FarVersion());
            int dirs = (int)ctx.ParamDouble("dirs", 96), grid = (int)ctx.ParamDouble("grid", 220);

            ctx.Step("Tracer self-check");
            foreach (PrimitiveType prim in new[] { PrimitiveType.Cube, PrimitiveType.Sphere, PrimitiveType.Quad })
            {
                GameObject go = GameObject.CreatePrimitive(prim);
                Mesh mesh = go.GetComponent<MeshFilter>().sharedMesh;
                UnityEngine.Object.Destroy(go);
                var res = new RayAreas.Result();
                yield return RayAreas.Trace(new List<RayAreas.Item> { new RayAreas.Item(mesh, new Vector3(0, 1000, 0), Quaternion.Euler(10, 20, 30), Vector3.one * 2, 0) }, 1, dirs, grid, res);
                ctx.Record("selfcheck." + prim, $"ray effective {F(res.Effective[0])} m2, mesh {F(res.MeshArea[0])} m2 (scale 2: cube 24, sphere 4 pi = 12.566, two-sided quad 8)");
            }

            ThermalAreaModes.Install();
            var parts = new StringBuilder("craft,part,index,farVoxel,stockPostOcclusion,stockUnoccluded,radMaster,radPr156,radNative,meshArea,rayEffective,expMasterRunway,expPrRunway,skinMassMaster,skinMassPr\n");
            var dirCsv = new StringBuilder("craft,dir,dx,dy,dz,ray,far,stock\n");
            foreach (string raw in (ctx.Param("crafts") ?? "HS Capsule@VAB+Aeris 4A@SPH+Radiator Test@VAB").Split('+'))
            {
                var (craft, facility) = CraftSpec(raw);
                ctx.Step("Launch " + craft);
                yield return LaunchCraft(ctx, craft, facility);
                Vessel v = FlightGlobals.ActiveVessel;
                List<Part> ps = v.parts.ToList();
                int n = ps.Count;

                ThermalAreaModes.Set(ThermalAreaModes.Mode.Pr156, v);
                yield return Wait.Seconds(1);
                double[] stockPost = ps.Select(p => p.DragCubes.None ? double.NaN : (double)p.DragCubes.PostOcclusionArea).ToArray();
                double[] stockUnocc = ps.Select(p => p.DragCubes.None ? double.NaN : (double)Enumerable.Range(0, 6).Sum(i => p.DragCubes.WeightedArea[i])).ToArray();
                double[] radPr = ps.Select(p => p.radiativeArea).ToArray();
                double[] expPr = ps.Select(p => p.exposedArea).ToArray();
                double[] skinPr = ps.Select(p => p.skinThermalMass).ToArray();
                // stock's projected area toward a source in -d, with stock's own occlusion
                Vector3[] dirVecs = RayAreas.Fibonacci(dirs);
                double[,] stockDir = new double[n, dirs];
                for (int i = 0; i < n; i++)
                    for (int k = 0; k < dirs; k++)
                        stockDir[i, k] = ps[i].DragCubes.None ? 0 : ps[i].DragCubes.GetCubeAreaDir(ps[i].partTransform.InverseTransformDirection(-dirVecs[k]));

                ThermalAreaModes.Set(ThermalAreaModes.Mode.Master, v);
                yield return Wait.Seconds(1);
                double[] radMaster = ps.Select(p => p.radiativeArea).ToArray();
                double[] expMaster = ps.Select(p => p.exposedArea).ToArray();
                double[] skinMaster = ps.Select(p => p.skinThermalMass).ToArray();
                ThermalAreaModes.Set(ThermalAreaModes.Mode.Native, v);
                yield return Wait.Seconds(1);
                double[] radNative = ps.Select(p => p.radiativeArea).ToArray();
                double[] far = ps.Select(p => ThermalAreaModes.FarTotal(p)).ToArray();
                double[,] farDir = new double[n, dirs];
                for (int i = 0; i < n; i++)
                    for (int k = 0; k < dirs; k++)
                        farDir[i, k] = ThermalAreaModes.FarWorld(ps[i], -dirVecs[k]);

                ctx.Step("Ray-trace " + craft);
                var res = new RayAreas.Result();
                var items = RayAreas.PartItems(ps, out int unreadable);
                yield return RayAreas.Trace(items, n, dirs, grid, res);
                ctx.Record(craft + ".tracer", $"{items.Count} meshes ({unreadable} unreadable, skipped), {dirs} directions, ray spacing {res.Spacing * 100:F1} cm");

                for (int i = 0; i < n; i++)
                    parts.Append(string.Join(",", craft, ps[i].partInfo.name, i, F(far[i]), F(stockPost[i]), F(stockUnocc[i]), F(radMaster[i]), F(radPr[i]), F(radNative[i]),
                        F(res.MeshArea[i]), F(res.Effective[i]), F(expMaster[i]), F(expPr[i]), F(skinMaster[i]), F(skinPr[i]))).Append('\n');
                double eFar = 0, eStock = 0, bFar = 0, bStock = 0;
                for (int k = 0; k < dirs; k++)
                {
                    double ray = 0, fa = 0, st = 0;
                    for (int i = 0; i < n; i++) { ray += res.Projected[i][k]; fa += farDir[i, k]; st += stockDir[i, k]; }
                    dirCsv.Append(string.Join(",", craft, k, F(dirVecs[k].x), F(dirVecs[k].y), F(dirVecs[k].z), F(ray), F(fa), F(st))).Append('\n');
                    eFar += Math.Abs(fa / ray - 1); eStock += Math.Abs(st / ray - 1); bFar += fa / ray - 1; bStock += st / ray - 1;
                }
                double S(double[] a) => a.Where(x => !double.IsNaN(x)).Sum();
                ctx.Record(craft + ".radiatingArea", $"ray {S(res.Effective):F2}, FAR voxel {S(far):F2}, stock {S(stockPost):F2} (unoccluded {S(stockUnocc):F2}), mesh {S(res.MeshArea):F2} m2");
                ctx.Record(craft + ".radiativeArea", $"master {S(radMaster):F2}, PR {S(radPr):F2}, native {S(radNative):F2} m2; skinThermalMass master {S(skinMaster):F1}, PR {S(skinPr):F1} kJ/K");
                ctx.Record(craft + ".directional", $"vessel projected area vs ray over {dirs} directions: FAR mean |err| {eFar / dirs:P1} (bias {bFar / dirs:P1}), stock {eStock / dirs:P1} (bias {bStock / dirs:P1})");
                // per part: model / ray
                var worst = Enumerable.Range(0, n).Where(i => res.Effective[i] > 0.05)
                    .Select(i => $"{ps[i].partInfo.name} ray {res.Effective[i]:F2} FAR {far[i] / res.Effective[i]:F2}x stock {stockPost[i] / res.Effective[i]:F2}x")
                    .ToList();
                ctx.Record(craft + ".perPart", string.Join("; ", worst.Distinct()));
            }
            ThermalAreaModes.Set(ThermalAreaModes.Mode.Native, FlightGlobals.ActiveVessel);
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "area_parts.csv"), parts.ToString());
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "area_dirs.csv"), dirCsv.ToString());
        }

        /// <summary>
        /// One entry flown under each mode from the same saved state (the save is loaded again for each), holding a
        /// fixed angle of attack to the airflow so every mode flies the same attitude. Records per part the peak skin
        /// and internal temperature against their limits (max temperature is ignored, nothing burns up), ablator used,
        /// and at the peak shock temperature the areas and occlusion multipliers KSP heated it with; the trajectory is
        /// logged to check the modes flew alike. Params: craft (Name@VAB|SPH, default HS Capsule), aoa (deg, nose above
        /// the airflow; 180 = heat shield first), peKm, apKm, startAltKm (75), endAltKm (20), modes ("+"-separated
        /// Master/Pr156/Hybrid/VoxelExposed/Native), physWarp (rate index, 1 = 2x), startTempK (300).
        /// </summary>
        [UITest("Far_ThermalEntry", Explicit = true, Kind = "Measure", StartsAt = StartState.SpaceCenter, TimeoutSec = 7200)]
        public static IEnumerator Far_ThermalEntry(TestContext ctx)
        {
            ctx.Record("farBuild", FarSim.FarVersion());
            var (craft, facility) = CraftSpec(ctx.Param("craft", "HS Capsule"));
            bool capsule = craft == "HS Capsule";
            double aoa = ctx.ParamDouble("aoa", capsule ? 180 : 35);
            double peKm = ctx.ParamDouble("peKm", capsule ? 32 : 30), apKm = ctx.ParamDouble("apKm", capsule ? 12000 : 100);
            double startAltKm = ctx.ParamDouble("startAltKm", 75), endAltKm = ctx.ParamDouble("endAltKm", 20);
            int warp = (int)ctx.ParamDouble("physWarp", 1);
            double startTemp = ctx.ParamDouble("startTempK", 300);
            string[] modes = (ctx.Param("modes") ?? "Master+Pr156+Hybrid").Split('+');
            string tag = craft.Replace(' ', '_');

            ctx.Step("Launch " + craft);
            yield return LaunchCraft(ctx, craft, facility);
            ThermalAreaModes.Install();
            try
            {
                Flight.IgnoreMaxTemperature(true);
                Ctrl.Sas(false);

                ctx.Step($"Place on a {peKm} x {apKm} km orbit descending through {startAltKm} km");
                Vessel v = FlightGlobals.ActiveVessel;
                Flight.SetOrbitDescendingAt(peKm * 1000, apKm * 1000, 0, 0, 0, startAltKm * 1000);
                yield return Wait.Seconds(2);
                yield return Wait.Until(() => !FlightGlobals.ActiveVessel.packed, 60, "off rails");
                Flight.PointAtAoA(aoa);
                ResetThermal(FlightGlobals.ActiveVessel, startTemp);
                yield return Wait.Frames(3);
                GamePersistence.SaveGame("ksptest-entry", HighLogic.SaveFolder, SaveMode.OVERWRITE);
                ctx.Record("start", $"alt {FlightGlobals.ActiveVessel.altitude / 1000:F1} km, speed {FlightGlobals.ActiveVessel.srfSpeed:F0} m/s");

                var summary = new StringBuilder(EntrySummaryHeader);
                foreach (string modeName in modes)
                {
                    var mode = (ThermalAreaModes.Mode)Enum.Parse(typeof(ThermalAreaModes.Mode), modeName, true);
                    ctx.Step($"Entry, mode {mode}");
                    ThermalAreaModes.Current = mode;
                    Game g = GamePersistence.LoadGame("ksptest-entry", HighLogic.SaveFolder, true, false);
                    FlightDriver.StartAndFocusVessel(g, g.flightState.activeVesselIdx);
                    yield return Wait.Seconds(2);
                    yield return Wait.FlightReady(120);
                    yield return WaitVoxels();
                    v = FlightGlobals.ActiveVessel;
                    ThermalAreaModes.Set(mode, v);
                    Flight.IgnoreMaxTemperature(true);
                    Ctrl.Sas(false);

                    yield return FlyEntry(ctx, v, mode, aoa, endAltKm, warp, ctx.ParamDouble("hyper", 0), tag, summary);
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, $"entry_{tag}_parts.csv"), summary.ToString());
            }
            finally
            {
                FarThermalModeTests.PutBack();
            }
        }

        public const string EntrySummaryHeader = "mode,part,index,peakSkinK,skinMaxK,peakSkinFrac,peakIntK,intMaxK,peakIntFrac,ablatorUsed,radArea,expArea,convArea,convAreaMult,convTempMult,skinExpFrac,skinMass,crossSection\n";

        /// <summary>
        /// Flies v down to endAltKm (or below Mach 1.2 under 40 km) holding aoa, under TimeControl's hyper-warp at
        /// physics accuracy 1 when hyper &gt; 0 and it is installed, else physics warp rate index `warp`; writes the
        /// trace CSV, appends each part's peaks to summary and records the hottest parts, trajectory and ablator used.
        /// </summary>
        public static IEnumerator FlyEntry(TestContext ctx, Vessel v, ThermalAreaModes.Mode mode, double aoa, double endAltKm, int warp, double hyper,
                                           string tag, StringBuilder summary)
        {
            var rec = v.gameObject.AddComponent<ThermalEntryRecorder>();
            rec.Aoa = aoa;
            rec.Begin(v);
            bool hyperOn = hyper > 0 && Flight.HyperWarp(hyper, 1.0);
            if (!hyperOn && warp > 0) Flight.PhysicsWarp(warp);
            ctx.Record($"{mode}.warp", hyperOn ? $"hyper {hyper}x" : $"physics index {warp}");
            yield return Wait.Until(() => rec.Vessel == null || rec.Vessel.state == Vessel.State.DEAD || rec.Vessel.altitude < endAltKm * 1000 ||
                                          (rec.Vessel.altitude < 40000 && rec.Vessel.mach < 1.2), 3600, "entry ends");
            if (hyperOn) Flight.HyperWarpOff();
            Flight.PhysicsWarp(0);
            rec.enabled = false;
            File.WriteAllText(Path.Combine(ctx.ResultsDir, $"entry_{tag}_{mode}_trace.csv"), rec.Trace.ToString());
            for (int i = 0; i < rec.Parts.Count; i++)
            {
                ThermalEntryRecorder.PartPeak pk = rec.Peaks[i];
                Part p = rec.Parts[i];
                summary.Append(string.Join(",", mode, p.partInfo.name, i, F(pk.Skin), F(p.skinMaxTemp), F(pk.Skin / p.skinMaxTemp), F(pk.Int), F(p.maxTemp), F(pk.Int / p.maxTemp),
                    F(pk.AblatorStart - Ablator(p)), F(pk.Rad), F(pk.Exp), F(pk.ConvArea), F(pk.ConvAreaMult), F(pk.ConvTempMult), F(pk.Frac), F(pk.SkinMass), F(pk.Cross))).Append('\n');
            }
            var hot = Enumerable.Range(0, rec.Parts.Count).OrderByDescending(i => rec.Peaks[i].Skin / rec.Parts[i].skinMaxTemp).Take(6)
                .Select(i => $"{rec.Parts[i].partInfo.name}#{i} skin {rec.Peaks[i].Skin:F0}/{rec.Parts[i].skinMaxTemp:F0} K int {rec.Peaks[i].Int:F0} K");
            ctx.Record($"{mode}.hottest", string.Join("; ", hot));
            ctx.Record($"{mode}.trajectory", rec.TrajectorySummary());
            double abl = Enumerable.Range(0, rec.Parts.Count).Sum(i => rec.Peaks[i].AblatorStart - Ablator(rec.Parts[i]));
            if (abl > 0) ctx.Record($"{mode}.ablatorUsed", F(abl));
            UnityEngine.Object.Destroy(rec);
        }

        /// <summary>
        /// Skin and internal temperatures in a sunlit orbit under each mode, from the same saved state at a uniform
        /// start temperature, after soakSec of physics time: the radiative balance in vacuum, where radiativeArea is the
        /// emitting area and FAR's voxel sun/body areas the absorbing ones. Params: craft (Name@VAB|SPH, default
        /// Radiator Test), orbitKm (250), soakSec (900), modes (Master+Pr156), physWarp (rate index, 3 = 4x), startTempK (250).
        /// </summary>
        [UITest("Far_ThermalOrbit", Explicit = true, Kind = "Measure", StartsAt = StartState.SpaceCenter, TimeoutSec = 3600)]
        public static IEnumerator Far_ThermalOrbit(TestContext ctx)
        {
            ctx.Record("farBuild", FarSim.FarVersion());
            var (craft, facility) = CraftSpec(ctx.Param("craft", "Radiator Test"));
            double orbitKm = ctx.ParamDouble("orbitKm", 250), soak = ctx.ParamDouble("soakSec", 900), startTemp = ctx.ParamDouble("startTempK", 250);
            int warp = (int)ctx.ParamDouble("physWarp", 3);
            string[] modes = (ctx.Param("modes") ?? "Master+Pr156").Split('+');
            string tag = craft.Replace(' ', '_');

            ctx.Step("Launch " + craft);
            yield return LaunchCraft(ctx, craft, facility);
            ThermalAreaModes.Install();
            try
            {
                Ctrl.Sas(false);

                ctx.Step($"Place in a {orbitKm} km orbit under the sun");
                Flight.SetOrbitAtMeanAnomaly(orbitKm * 1000, orbitKm * 1000, 0, 0, 0, 0);
                yield return Wait.Seconds(2);
                yield return Wait.Until(() => !FlightGlobals.ActiveVessel.packed, 60, "off rails");
                // turn the orbit so the vessel sits under the sun, a quarter orbit before noon
                Vessel v = FlightGlobals.ActiveVessel;
                Vector3d r = (v.CoMD - v.mainBody.position).normalized, s = (Planetarium.fetch.Sun.position - v.mainBody.position).normalized;
                Vector3d nrm = v.orbit.GetOrbitNormal().xzy.normalized;
                Vector3d sIn = (s - Vector3d.Dot(s, nrm) * nrm).normalized;
                double ang = Math.Atan2(Vector3d.Dot(Vector3d.Cross(r, sIn), nrm), Vector3d.Dot(r, sIn));
                double target = ang - ctx.ParamDouble("leadDeg", 30) * Math.PI / 180;
                for (int attempt = 0; attempt < 2; attempt++)
                {
                    Flight.SetOrbitAtMeanAnomaly(orbitKm * 1000, orbitKm * 1000, 0, 0, 0, attempt == 0 ? target : -target);
                    yield return Wait.Seconds(2);
                    yield return Wait.Until(() => !FlightGlobals.ActiveVessel.packed, 60, "off rails");
                    v = FlightGlobals.ActiveVessel;
                    r = (v.CoMD - v.mainBody.position).normalized;
                    s = (Planetarium.fetch.Sun.position - v.mainBody.position).normalized;
                    if (Vector3d.Dot(r, s) > 0.5) break;
                }
                ctx.Record("sunElevationDeg", F(90 - Vector3d.Angle(r, s)));
                foreach (Part p in v.parts)
                    if (p.Rigidbody != null) p.Rigidbody.angularVelocity = Vector3.zero;
                ResetThermal(v, startTemp);
                yield return Wait.Frames(3);
                GamePersistence.SaveGame("ksptest-orbit", HighLogic.SaveFolder, SaveMode.OVERWRITE);

                var csv = new StringBuilder("mode,part,index,t,skinK,unexpSkinK,intK,radArea,sunArea,bodyArea,skinMass,radFlux,sunlit\n");
                foreach (string modeName in modes)
                {
                    var mode = (ThermalAreaModes.Mode)Enum.Parse(typeof(ThermalAreaModes.Mode), modeName, true);
                    ctx.Step($"Soak, mode {mode}");
                    ThermalAreaModes.Current = mode;
                    Game g = GamePersistence.LoadGame("ksptest-orbit", HighLogic.SaveFolder, true, false);
                    FlightDriver.StartAndFocusVessel(g, g.flightState.activeVesselIdx);
                    yield return Wait.Seconds(2);
                    yield return Wait.FlightReady(120);
                    yield return WaitVoxels();
                    v = FlightGlobals.ActiveVessel;
                    ThermalAreaModes.Set(mode, v);
                    foreach (Part p in v.parts)
                        if (p.Rigidbody != null) p.Rigidbody.angularVelocity = Vector3.zero;
                    var fi = v.GetComponent<FlightIntegrator>();
                    double t0 = Planetarium.GetUniversalTime(), next = 0;
                    if (warp > 0) Flight.PhysicsWarp(warp);
                    while (Planetarium.GetUniversalTime() - t0 < soak)
                    {
                        double t = Planetarium.GetUniversalTime() - t0;
                        if (t >= next)
                        {
                            next += 60;
                            for (int i = 0; i < v.parts.Count; i++)
                            {
                                Part p = v.parts[i];
                                PartThermalData ptd = p.ptd;
                                csv.Append(string.Join(",", mode, p.partInfo.name, i, F(t), F(p.skinTemperature), F(p.skinUnexposedTemperature), F(p.temperature), F(p.radiativeArea),
                                    ptd == null ? "" : F(fi.GetSunArea(ptd)), ptd == null ? "" : F(fi.GetBodyArea(ptd)), F(p.skinThermalMass), F(p.thermalRadiationFlux), v.directSunlight)).Append('\n');
                            }
                        }
                        yield return null;
                    }
                    Flight.PhysicsWarp(0);
                    ctx.Record($"{mode}.end", string.Join("; ", v.parts.Select((p, i) => $"{p.partInfo.name}#{i} skin {p.skinTemperature:F0} int {p.temperature:F0} K rad {p.radiativeArea:F2} m2")));
                    ctx.Record($"{mode}.sunlit", v.directSunlight);
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, $"orbit_{tag}.csv"), csv.ToString());
            }
            finally
            {
                FarThermalModeTests.PutBack();
            }
        }

        /// <summary>
        /// What the mode tests change, put back however they end (a failure included), so the session's later tests run
        /// on FAR's own thermal areas: the mode (the Harmony prefix stays, passing straight through on Native), the
        /// max-temperature cheat and any warp.
        /// </summary>
        public static void PutBack()
        {
            ThermalAreaModes.Current = ThermalAreaModes.Mode.Native;
            Flight.IgnoreMaxTemperature(false);
            Flight.HyperWarpOff();
            if (TimeWarp.fetch != null) Flight.PhysicsWarp(0);
        }

        public static void ResetThermal(Vessel v, double k)
        {
            foreach (Part p in v.parts)
            {
                p.temperature = k;
                p.skinTemperature = k;
                p.skinUnexposedTemperature = k;
            }
        }

        public static double Ablator(Part p) => p.Resources.Contains("Ablator") ? p.Resources["Ablator"].amount : 0;

        /// <summary>Holds the angle of attack every physics frame and keeps each part's peak temperatures.</summary>
        public sealed class ThermalEntryRecorder : MonoBehaviour
        {
            public sealed class PartPeak
            {
                public double Skin, Int, AblatorStart, Rad, Exp, ConvArea, ConvAreaMult = 1, ConvTempMult = 1, Frac, SkinMass, Cross;
            }

            public double Aoa;
            // false: hold with forces (Flight.TorqueHoldAoA) instead of setting every part's angular velocity, which
            // Principia (it integrates the parts' motion itself) does not take
            public bool Steer = true;
            // when set, holds the attitude instead (called with Aoa every physics frame)
            public Action<double> Hold;
            public Vessel Vessel;
            public List<Part> Parts;
            public List<PartPeak> Peaks;
            public readonly StringBuilder Trace = new StringBuilder();
            private double t0, nextLog, peakShock, peakQ, peakMach;
            private double tAt60 = double.NaN, speedAt60 = double.NaN, tAt40 = double.NaN, speedAt40 = double.NaN;

            public void Begin(Vessel v)
            {
                Vessel = v;
                Parts = v.parts.ToList();
                Peaks = Parts.Select(p => new PartPeak { Skin = p.skinTemperature, Int = p.temperature, AblatorStart = Ablator(p) }).ToList();
                t0 = Planetarium.GetUniversalTime();
                Trace.Append("t,altKm,speed,mach,qKPa,shockK," + string.Join(",", Parts.Select((p, i) => $"skin_{p.partInfo.name}_{i}")) + "\n");
            }

            private void FixedUpdate()
            {
                if (Vessel == null || Vessel != FlightGlobals.ActiveVessel || Vessel.packed) return;
                if (Hold != null) Hold(Aoa);
                else if (Steer) Flight.SteerToAoA(Aoa);
                else Flight.TorqueHoldAoA(Aoa);
                double t = Planetarium.GetUniversalTime() - t0;
                double shock = Vessel.externalTemperature;
                bool atPeak = shock > peakShock;
                if (atPeak) peakShock = shock;
                peakQ = Math.Max(peakQ, Vessel.dynamicPressurekPa);
                peakMach = Math.Max(peakMach, Vessel.mach);
                if (double.IsNaN(tAt60) && Vessel.altitude < 60000) { tAt60 = t; speedAt60 = Vessel.srfSpeed; }
                if (double.IsNaN(tAt40) && Vessel.altitude < 40000) { tAt40 = t; speedAt40 = Vessel.srfSpeed; }
                for (int i = 0; i < Parts.Count; i++)
                {
                    Part p = Parts[i];
                    if (p == null || p.vessel != Vessel) continue;
                    PartPeak pk = Peaks[i];
                    pk.Skin = Math.Max(pk.Skin, p.skinTemperature);
                    pk.Int = Math.Max(pk.Int, p.temperature);
                    if (atPeak && p.ptd != null)
                    {
                        pk.Rad = p.radiativeArea; pk.Exp = p.exposedArea; pk.ConvArea = p.ptd.convectionArea;
                        pk.ConvAreaMult = p.ptd.convectionAreaMultiplier; pk.ConvTempMult = p.ptd.convectionTempMultiplier;
                        pk.Frac = p.skinExposedAreaFrac; pk.SkinMass = p.skinThermalMass;
                        pk.Cross = p.DragCubes.None ? 0 : p.DragCubes.CrossSectionalArea;
                    }
                }
                if (t >= nextLog)
                {
                    nextLog += 2;
                    Trace.Append(string.Join(",", F(t), F(Vessel.altitude / 1000), F(Vessel.srfSpeed), F(Vessel.mach), F(Vessel.dynamicPressurekPa), F(shock)));
                    foreach (Part p in Parts)
                        Trace.Append(',').Append(p == null ? "" : F(p.skinTemperature));
                    Trace.Append('\n');
                }
            }

            public string TrajectorySummary() =>
                $"peak shock {peakShock:F0} K, q {peakQ:F1} kPa, mach {peakMach:F2}; 60 km at {tAt60:F1} s {speedAt60:F0} m/s, 40 km at {tAt40:F1} s {speedAt40:F0} m/s";
        }
    }

    /// <summary>
    /// Which thermal areas FAR's ModularFlightIntegratorRegisterer.UpdateThermodynamicsPre hands KSP, switched at
    /// runtime by a Harmony prefix, so FAR master and PR #156 can be compared in one session on the same build:
    /// Master: drag cube AreaOccluded overwritten from the voxels, radiativeArea = voxel area, exposedArea = the
    /// (voxel-fed) drag cube ExposedArea. Pr156: none of that, stock radiativeArea and exposedArea. Hybrid: stock
    /// AreaOccluded and exposedArea, voxel radiativeArea. VoxelExposed: Master with exposedAreaUsesKSPHack off
    /// (exposedArea = voxel area projected along the airflow). Native: FAR's own code. Sun and body areas stay FAR's
    /// voxel areas in every mode, as in both master and the PR.
    /// </summary>
    public static class ThermalAreaModes
    {
        public enum Mode { Native, Master, Pr156, Hybrid, VoxelExposed }

        public static Mode Current = Mode.Native;

        private static Harmony harmony;
        private static Type aeroType, vesselAeroType;
        private static PropertyInfo projectedProp;
        private static FieldInfo totalField;
        private static FieldInfo[] faceFields;
        private static int[] faceMap;
        private static MethodInfo hasEverValid, baseRad, baseExp, projectedLocal, projectedWorld;
        private static bool exposedAreaLimited;
        public static bool ExposedAreaLimited => exposedAreaLimited;
        private static readonly Dictionary<Part, PartModule> modules = new Dictionary<Part, PartModule>();
        private static readonly object[] one = new object[1];
        private static bool warned;

        public static void Install()
        {
            if (harmony != null) return;
            aeroType = Reflect.TryFindType("FerramAerospaceResearch.FARAeroComponents.FARAeroPartModule") ?? throw new UITestFailure("FARAeroPartModule not found");
            vesselAeroType = Reflect.TryFindType("FerramAerospaceResearch.FARAeroComponents.FARVesselAero") ?? throw new UITestFailure("FARVesselAero not found");
            projectedProp = aeroType.GetProperty("ProjectedAreas");
            Type pa = projectedProp.PropertyType;
            totalField = pa.GetField("totalArea");
            faceFields = new[] { "iP", "iN", "jP", "jN", "kP", "kN" }.Select(n => pa.GetField(n)).ToArray();
            faceMap = (int[])pa.GetField("FaceMap", BindingFlags.Public | BindingFlags.Static).GetValue(null);
            hasEverValid = vesselAeroType.GetMethod("HasEverValidVoxelization");
            projectedLocal = aeroType.GetMethod("ProjectedAreaLocal");
            projectedWorld = aeroType.GetMethod("ProjectedAreaWorld");
            Type mfi = Reflect.TryFindType("ModularFI.ModularFlightIntegrator") ?? throw new UITestFailure("ModularFlightIntegrator not found");
            baseRad = mfi.GetMethod("BaseFICalculateAreaRadiative");
            baseExp = mfi.GetMethod("BaseFICalculateAreaExposed");
            exposedAreaLimited = (bool)Reflect.TryFindType("FerramAerospaceResearch.Settings.FARSettings").GetField("ExposedAreaLimited").GetValue(null);
            MethodInfo target = Reflect.TryFindType("FerramAerospaceResearch.FARAeroComponents.ModularFlightIntegratorRegisterer")
                                       ?.GetMethod("UpdateThermodynamicsPre", BindingFlags.NonPublic | BindingFlags.Static)
                                ?? throw new UITestFailure("UpdateThermodynamicsPre not found");
            harmony = new Harmony("KspTest.FarThermalAreaModes");
            harmony.Patch(target, new HarmonyMethod(typeof(ThermalAreaModes), nameof(Prefix)));
            Log.Info("thermal area modes installed");
        }

        /// <summary>Switches mode; leaving the voxel AreaOccluded behind, stock occlusion is set up again.</summary>
        public static void Set(Mode mode, Vessel v)
        {
            Current = mode;
            if (mode == Mode.Pr156 || mode == Mode.Hybrid)
                foreach (Part p in v.parts)
                    if (!p.DragCubes.None)
                        p.DragCubes.ForceUpdate(false, true);
            Log.Info("thermal area mode " + mode);
        }

        public static double FarTotal(Part p)
        {
            PartModule m = Module(p);
            return m == null ? double.NaN : (double)totalField.GetValue(projectedProp.GetValue(m));
        }

        public static double[] FarFaces(Part p)
        {
            PartModule m = Module(p);
            if (m == null) return new double[6];
            object pa = projectedProp.GetValue(m);
            return faceFields.Select(f => (double)f.GetValue(pa)).ToArray();
        }

        public static double FarWorld(Part p, Vector3 towardSource)
        {
            PartModule m = Module(p);
            return m == null ? 0 : (double)projectedWorld.Invoke(m, new object[] { towardSource });
        }

        private static PartModule Module(Part p)
        {
            if (!modules.TryGetValue(p, out PartModule m) || m == null)
            {
                m = p.Modules.Cast<PartModule>().FirstOrDefault(x => aeroType.IsInstanceOfType(x));
                modules[p] = m;
            }
            return m;
        }

        private static bool Prefix(object[] __args)
        {
            if (Current == Mode.Native) return true;
            try
            {
                Apply((FlightIntegrator)__args[0]);
            }
            catch (Exception e)
            {
                if (!warned) Log.Warn("thermal area mode failed: " + e);
                warned = true;
            }
            return false;
        }

        private static double Base(MethodInfo m, FlightIntegrator fi, Part p)
        {
            one[0] = p;
            return (double)m.Invoke(fi, one);
        }

        private static void Apply(FlightIntegrator fi)
        {
            Component va = fi.Vessel.GetComponent(vesselAeroType);
            bool voxel = va != null && (bool)hasEverValid.Invoke(va, null);
            foreach (PartThermalData ptd in fi.partThermalDataList)
            {
                Part part = ptd.part;
                PartModule m = Module(part);
                if (m == null) continue;
                double rad, exp;
                if (voxel && (Current == Mode.Master || Current == Mode.VoxelExposed))
                {
                    object pa = projectedProp.GetValue(m);
                    if (!part.DragCubes.None)
                        for (int j = 0; j < 6; j++)
                            part.DragCubes.AreaOccluded[faceMap[j]] = (float)(double)faceFields[j].GetValue(pa);
                    double total = (double)totalField.GetValue(pa);
                    rad = total > 0 ? total : Base(baseRad, fi, part);
                    if (part.machNumber > 0)
                    {
                        double e = Current == Mode.VoxelExposed
                                       ? (double)projectedLocal.Invoke(m, new object[] { -part.dragVectorDirLocal })
                                       : part.DragCubes.ExposedArea;
                        exp = e > 0 ? e : Base(baseExp, fi, part);
                    }
                    else
                        exp = rad;
                }
                else if (voxel && Current == Mode.Hybrid)
                {
                    double total = (double)totalField.GetValue(projectedProp.GetValue(m));
                    rad = total > 0 ? total : Base(baseRad, fi, part);
                    exp = Base(baseExp, fi, part);
                }
                else
                {
                    rad = Base(baseRad, fi, part);
                    exp = Base(baseExp, fi, part);
                }
                part.radiativeArea = rad;
                part.exposedArea = exp;
                if (exposedAreaLimited && part.exposedArea > part.radiativeArea)
                    part.exposedArea = part.radiativeArea;
            }
        }
    }

    /// <summary>
    /// Ray-traced areas of a set of meshes: parallel rays from many directions against MeshColliders built from the
    /// meshes in a physics scene of their own (nothing else in it, never simulated), back faces hit, each ray credited
    /// to the owner of the first mesh it hits.
    /// </summary>
    public static class RayAreas
    {
        public sealed class Item
        {
            public Mesh Mesh; public Vector3 Pos; public Quaternion Rot; public Vector3 Scale; public int Owner;
            public Item(Mesh mesh, Vector3 pos, Quaternion rot, Vector3 scale, int owner) { Mesh = mesh; Pos = pos; Rot = rot; Scale = scale; Owner = owner; }
        }

        public sealed class Result
        {
            public Vector3[] Dirs;
            public double[][] Projected;   // [owner][dir]: area seen from -dir (rays travel along dir), m2
            public double[] MeshArea, Effective;
            public double Spacing;
        }

        private static int sceneSeq;

        /// <summary>Evenly spread unit vectors (Fibonacci sphere).</summary>
        public static Vector3[] Fibonacci(int n)
        {
            var d = new Vector3[n];
            double golden = Math.PI * (3 - Math.Sqrt(5));
            for (int i = 0; i < n; i++)
            {
                double z = 1 - (2.0 * i + 1) / n, r = Math.Sqrt(1 - z * z), phi = i * golden;
                d[i] = new Vector3((float)(r * Math.Cos(phi)), (float)z, (float)(r * Math.Sin(phi)));
            }
            return d;
        }

        /// <summary>The enabled render meshes of each part (skinned ones baked), owner = index in parts.</summary>
        public static List<Item> PartItems(IList<Part> parts, out int unreadable)
        {
            var items = new List<Item>();
            unreadable = 0;
            for (int i = 0; i < parts.Count; i++)
            {
                foreach (MeshFilter mf in parts[i].FindModelComponents<MeshFilter>())
                {
                    var r = mf.GetComponent<MeshRenderer>();
                    if (r == null || !r.enabled || !mf.gameObject.activeInHierarchy || mf.sharedMesh == null) continue;
                    if (!mf.sharedMesh.isReadable) { unreadable++; continue; }
                    items.Add(new Item(mf.sharedMesh, mf.transform.position, mf.transform.rotation, mf.transform.lossyScale, i));
                }
                foreach (SkinnedMeshRenderer smr in parts[i].FindModelComponents<SkinnedMeshRenderer>())
                {
                    if (!smr.enabled || !smr.gameObject.activeInHierarchy || smr.sharedMesh == null) continue;
                    var baked = new Mesh();
                    smr.BakeMesh(baked);
                    items.Add(new Item(baked, smr.transform.position, smr.transform.rotation, Vector3.one, i));
                }
            }
            return items;
        }

        public static IEnumerator Trace(List<Item> items, int owners, int dirCount, int grid, Result res)
        {
            Scene scene = SceneManager.CreateScene("KspTestRays" + ++sceneSeq, new CreateSceneParameters(LocalPhysicsMode.Physics3D));
            PhysicsScene ps = scene.GetPhysicsScene();
            var owner = new Dictionary<Collider, int>();
            res.MeshArea = new double[owners];
            res.Effective = new double[owners];
            res.Projected = new double[owners][];
            Bounds b = default;
            bool first = true;
            bool backfaces = Physics.queriesHitBackfaces;
            try
            {
                foreach (Item it in items)
                {
                    var go = new GameObject("ray");
                    SceneManager.MoveGameObjectToScene(go, scene);
                    go.transform.SetPositionAndRotation(it.Pos, it.Rot);
                    go.transform.localScale = it.Scale;
                    var mc = go.AddComponent<MeshCollider>();
                    mc.sharedMesh = it.Mesh;
                    owner[mc] = it.Owner;
                    Matrix4x4 m = go.transform.localToWorldMatrix;
                    Vector3[] vs = it.Mesh.vertices;
                    int[] tris = it.Mesh.triangles;
                    var w = new Vector3[vs.Length];
                    for (int i = 0; i < vs.Length; i++)
                    {
                        w[i] = m.MultiplyPoint3x4(vs[i]);
                        if (first) { b = new Bounds(w[i], Vector3.zero); first = false; }
                        else b.Encapsulate(w[i]);
                    }
                    double area = 0;
                    for (int t = 0; t + 2 < tris.Length; t += 3)
                        area += 0.5 * Vector3.Cross(w[tris[t + 1]] - w[tris[t]], w[tris[t + 2]] - w[tris[t]]).magnitude;
                    res.MeshArea[it.Owner] += area;
                }
                Physics.SyncTransforms();
                Physics.queriesHitBackfaces = true;

                Vector3 c = b.center;
                float R = b.extents.magnitude + 0.05f;
                double h = 2.0 * R / grid;
                res.Spacing = h;
                res.Dirs = Fibonacci(dirCount);
                for (int p = 0; p < owners; p++) res.Projected[p] = new double[dirCount];
                var rng = new System.Random(12345);
                var counts = new int[owners];
                for (int k = 0; k < dirCount; k++)
                {
                    Vector3 d = res.Dirs[k];
                    Vector3 u = Vector3.Cross(d, Mathf.Abs(d.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
                    Vector3 v = Vector3.Cross(d, u);
                    double ou = rng.NextDouble(), ov = rng.NextDouble();
                    Array.Clear(counts, 0, owners);
                    Vector3 o0 = c - d * (R + 1);
                    for (int i = 0; i < grid; i++)
                    {
                        Vector3 oi = o0 + u * (float)(-R + (i + ou) * h);
                        for (int j = 0; j < grid; j++)
                        {
                            Vector3 o = oi + v * (float)(-R + (j + ov) * h);
                            if (ps.Raycast(o, d, out RaycastHit hit, 2 * R + 2, ~0, QueryTriggerInteraction.Collide) &&
                                owner.TryGetValue(hit.collider, out int p))
                                counts[p]++;
                        }
                    }
                    for (int p = 0; p < owners; p++)
                        res.Projected[p][k] = counts[p] * h * h;
                    if (k % 4 == 3) yield return null;
                }
                for (int p = 0; p < owners; p++)
                    res.Effective[p] = 4 * res.Projected[p].Average();
            }
            finally
            {
                Physics.queriesHitBackfaces = backfaces;
                SceneManager.UnloadSceneAsync(scene);
            }
        }
    }
}
