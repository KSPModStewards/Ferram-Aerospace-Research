using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.Settings;
using HarmonyLib;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>The areas FAR gives the game's heating model, from its voxels, and the game's own aerodynamics.</summary>
    public static class ThermalAreaTests
    {
        /// <summary>
        /// With FAR's exposed-area hack off (the config's exposedAreaUsesKSPHack = false), in cruise at Mach 0.5: each
        /// part's radiative area is the surface FAR's voxels give it, its exposed area the part of it facing the flow
        /// (no more than the whole), and its drag cube's occluded areas are FAR's projected areas
        /// face by face.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_ExposedAreaFromVoxelsWithoutTheKspHack(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0.5, 3000, 4);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            bool hackWas = FARSettings.ExposedAreaUsesKSPHack;
            try
            {
                FARSettings.ExposedAreaUsesKSPHack = false;
                yield return FarFlightProbe.Hold(c, 10, 1, new List<FarFlightProbe.Sample>(), predict: false);

                int checkedParts = 0, facingParts = 0;
                foreach (Part p in v.parts)
                {
                    FARAeroPartModule m = p.Modules.GetModule<FARAeroPartModule>();
                    if (m == null || p.DragCubes.None || m.ProjectedAreas.totalArea <= 0)
                        continue;
                    string name = p.partInfo.name;
                    double total = m.ProjectedAreas.totalArea;
                    Assert.Near(total, p.radiativeArea, 1e-4 * total, $"{name}'s radiative area is FAR's voxel surface");
                    Assert.True(p.exposedArea >= 0 && p.exposedArea <= p.radiativeArea * 1.0001,
                                $"{name}'s exposed area within its surface ({p.exposedArea:F4} of {p.radiativeArea:F4} m2)");
                    double facing = m.ProjectedAreaLocal(-p.dragVectorDirLocal);
                    if (facing > 0)
                        Assert.Near(facing, p.exposedArea, 0.05 * facing + 1e-4, $"{name}'s exposed area faces the flow");
                    for (int j = 0; j < 6; j++)
                        Assert.Near(m.ProjectedAreas[j], p.DragCubes.AreaOccluded[FARAeroPartModule.ProjectedArea.FaceMap[j]],
                                    1e-4 * m.ProjectedAreas[j] + 1e-6, $"{name}'s drag cube face {j} is FAR's projected area");
                    checkedParts++;
                    if (p.exposedArea > 0) facingParts++;
                }

                ctx.Record("parts", $"{checkedParts}, {facingParts} with an area facing the flow");
                Assert.True(checkedParts > 5 && facingParts > 5, $"parts checked ({checkedParts}, {facingParts} facing the flow)");
            }
            finally
            {
                FARSettings.ExposedAreaUsesKSPHack = hackWas;
            }
        }

        /// <summary>
        /// A stock airbrake (ModuleAeroSurface) is left to the game's own aerodynamics (FAR's UpdateAerodynamics hands it
        /// back), which asks ModularFlightIntegrator for its exposed area: FAR's override answers, for the airbrakes and
        /// no other part. With the exposed-area hack off it is the airbrake's projected area facing the flow, from FAR's
        /// voxels; with it on, the drag cube's exposed area; the stock area where FAR's is none. On the stock Mallard
        /// in cruise.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Mallard", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_AirbrakesExposedAreaFromFar(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0.3, 2000, 3);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            List<Part> airbrakes = v.parts.Where(p => p.Modules.Contains<ModuleAeroSurface>()).ToList();
            ctx.Record("airbrakes", string.Join(", ", airbrakes.Select(p => p.partInfo.name)));
            Assert.True(airbrakes.Count > 0, "the Mallard's airbrakes");

            Type mfi = AccessTools.TypeByName("ModularFI.ModularFlightIntegrator") ?? throw new UITestFailure("no ModularFlightIntegrator");
            MethodInfo target = AccessTools.Method(typeof(ModularFlightIntegratorRegisterer), "CalculateAreaExposed", new[] { mfi, typeof(Part) })
                                ?? throw new UITestFailure("no CalculateAreaExposed(ModularFlightIntegrator, Part) in FAR");
            ExposedAreaCalls.Base = AccessTools.Method(mfi, "BaseFICalculateAreaExposed");
            var harmony = new Harmony(HarmonyId);
            bool hackWas = FARSettings.ExposedAreaUsesKSPHack;
            try
            {
                harmony.Patch(target, postfix: new HarmonyMethod(typeof(ExposedAreaCalls), nameof(ExposedAreaCalls.Postfix)));
                foreach (bool hack in new[] { false, true })
                {
                    ctx.Step(hack ? "the exposed-area hack on" : "the exposed-area hack off");
                    FARSettings.ExposedAreaUsesKSPHack = hack;
                    ExposedAreaCalls.Calls.Clear();
                    yield return FarFlightProbe.Hold(c, 5, 1, new List<FarFlightProbe.Sample>(), predict: false);
                    Dictionary<Part, ExposedAreaCalls.Call> calls = ExposedAreaCalls.Calls;
                    ctx.Record(hack ? "hackOn" : "hackOff",
                               string.Join(", ", calls.Select(kv => $"{kv.Key.partInfo.name} x{kv.Value.N}: {kv.Value.Result:F4} (FAR {kv.Value.Far:F4}, cube {kv.Value.Cube:F4}, stock {kv.Value.Stock:F4})")));
                    Assert.True(airbrakes.All(calls.ContainsKey), $"every airbrake's exposed area asked of FAR ({calls.Count} parts)");
                    Assert.True(calls.Keys.All(airbrakes.Contains), "and no other part's");
                    foreach (Part p in airbrakes)
                    {
                        ExposedAreaCalls.Call call = calls[p];
                        double expected = hack ? call.Cube : call.Far;
                        if (expected <= 0)
                            expected = call.Stock;
                        Assert.Near(expected, call.Result, 1e-9 + 1e-6 * expected, $"{p.partInfo.name}'s exposed area ({(hack ? "drag cube" : "FAR's voxels")})");
                        Assert.True(call.Result > 0, $"{p.partInfo.name} has an exposed area");
                    }
                }
            }
            finally
            {
                harmony.UnpatchAll(HarmonyId);
                FARSettings.ExposedAreaUsesKSPHack = hackWas;
            }
        }

        private const string HarmonyId = "FerramAerospaceResearch.KspTests.ThermalArea";

        /// <summary>FAR's CalculateAreaExposed(ModularFlightIntegrator, Part): what it gave each part, and its inputs.</summary>
        private static class ExposedAreaCalls
        {
            public sealed class Call
            {
                public int N;
                public double Result, Far, Cube, Stock;
            }

            public static readonly Dictionary<Part, Call> Calls = new Dictionary<Part, Call>();
            public static MethodInfo Base;

            public static void Postfix(object fi, Part part, double __result)
            {
                FARAeroPartModule m = part.Modules.GetModule<FARAeroPartModule>();
                if (!Calls.TryGetValue(part, out Call call))
                    Calls[part] = call = new Call();
                call.N++;
                call.Result = __result;
                call.Far = m != null ? m.ProjectedAreaLocal(-part.dragVectorDirLocal) : double.NaN;
                call.Cube = part.DragCubes.ExposedArea;
                call.Stock = (double)Base.Invoke(fi, new object[] { part });
            }
        }
    }
}
