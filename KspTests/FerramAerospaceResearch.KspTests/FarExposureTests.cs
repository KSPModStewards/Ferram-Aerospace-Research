using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>FAR's exposure (part area) rendering and its device setting.</summary>
    public static class FarExposureTests
    {
        private const BindingFlags Any = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static;

        /// <summary>
        /// With no saved FAR settings (GameData/Custom_FARConfig.cfg absent), FARConfig.Exposure.Device must be None and
        /// the active vessel's VehicleExposure disabled. Then, as in the debug window: the sun direction is turned on
        /// (FARConfig.Exposure.Sun, the window's Enable toggle), which must still render nothing while the device is None;
        /// the device is set to PreferGPU (VehicleExposure.ComputeDevice, the window's dropdown), which must render; and
        /// back to None, which must stop. Each step watches the exposure renderer for 3 s. The sun flag is restored and a
        /// Custom_FARConfig.cfg written during the test is deleted. Needs a graphics device (not -batchmode).
        /// (KSPModStewards FAR #157 makes None the default.)
        /// </summary>
        [UITest("Far_ExposureDefaultOff", Explicit = true, Kind = "Check", StartsAt = StartState.Flight, Craft = "Aeris 4A", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Far_ExposureDefaultOff(TestContext ctx)
        {
            ctx.Record("farBuild", FarSim.FarVersion());
            string custom = Path.Combine(KSPUtil.ApplicationRootPath, "GameData", "Custom_FARConfig.cfg");
            bool customExisted = File.Exists(custom);
            ctx.Record("customFARConfigPresent", customExisted);

            Type config = Reflect.TryFindType("FerramAerospaceResearch.FARConfig") ?? throw new UITestFailure("FARConfig not found");
            Type vesselAeroType = Reflect.TryFindType("FerramAerospaceResearch.FARAeroComponents.FARVesselAero")
                                  ?? throw new UITestFailure("FARVesselAero not found");
            Type exposureType = Reflect.TryFindType("FerramAerospaceResearch.FARAeroComponents.VehicleExposure")
                                ?? throw new UITestFailure("VehicleExposure not found");
            Type deviceType = Reflect.TryFindType("FerramAerospaceResearch.Resources.Device") ?? throw new UITestFailure("Device not found");
            PropertyInfo executorJobs = Reflect.TryFindType("FerramAerospaceResearch.Geometry.Exposure.Executor")?.GetProperty("JobsCount", Any);

            object exposureConfig = config.GetField("Exposure", Any)?.GetValue(null) ?? throw new UITestFailure("FARConfig.Exposure not found");
            object ConfigValue(string name) => exposureConfig.GetType().GetProperty(name, Any)?.GetValue(exposureConfig)
                                               ?? throw new UITestFailure($"ExposureConfig.{name} not found");
            object Get(object observable) => observable.GetType().GetProperty("Value", Any)?.GetValue(observable);
            void Set(object observable, object value) => observable.GetType().GetMethod("Set", new[] { typeof(object) }).Invoke(observable, new[] { value });
            object sunFlag = ConfigValue("Sun");
            string configDevice = Get(ConfigValue("Device"))?.ToString();
            ctx.Record("configDevice", configDevice);
            ctx.Record("configDirections", $"airstream {Get(ConfigValue("Airstream"))}, sun {Get(sunFlag)}, body {Get(ConfigValue("Body"))}");
            ctx.Record("deviceEnum", string.Join(",", Enum.GetNames(deviceType).Select(n => $"{n}={Convert.ToInt32(Enum.Parse(deviceType, n))}")));
            bool sunWas = (bool)Get(sunFlag);

            yield return Wait.FlightReady();
            Vessel v = FlightGlobals.ActiveVessel;
            Component vesselAero = v.GetComponent(vesselAeroType) ?? throw new UITestFailure("no FARVesselAero on the active vessel");
            object exposure = vesselAeroType.GetProperty("Exposure", Any)?.GetValue(vesselAero) ?? throw new UITestFailure("FARVesselAero.Exposure is null");
            PropertyInfo computeDevice = exposureType.GetProperty("ComputeDevice", Any);
            PropertyInfo enabled = exposureType.GetProperty("Enabled", Any);
            object deviceSelect = exposureType.GetField("deviceSelect", Any)?.GetValue(exposure);
            object renderer = exposureType.GetField("exposureRenderer", Any)?.GetValue(exposure);
            PropertyInfo activeBatches = renderer?.GetType().GetProperty("ActiveBatches", Any);
            List<List<double>> results = new[] { "airstreamRequest", "sunRequest", "bodyRequest" }.Select(name =>
            {
                object request = exposureType.GetField(name, Any)?.GetValue(exposure) ?? throw new UITestFailure($"{name} not found");
                object result = request.GetType().GetField("result", Any)?.GetValue(request) ?? throw new UITestFailure($"{name}.result not found");
                return (List<double>)result.GetType().GetField("Areas", Any).GetValue(result);
            }).ToList();

            // watches the renderer for 3 s from cleared results; rendered = any batch, job or result seen
            bool rendered = false;
            IEnumerator Watch(string label)
            {
                foreach (List<double> r in results)
                    r.Clear();
                int frames = 0, maxBatches = 0, maxJobs = 0;
                float until = Time.realtimeSinceStartup + 3;
                while (Time.realtimeSinceStartup < until)
                {
                    yield return Wait.Frames(1);
                    frames++;
                    maxBatches = Math.Max(maxBatches, activeBatches == null ? -1 : (int)activeBatches.GetValue(renderer));
                    maxJobs = Math.Max(maxJobs, executorJobs == null ? -1 : (int)executorJobs.GetValue(null));
                }
                rendered = maxBatches > 0 || maxJobs > 0 || results.Any(r => r.Count > 0);
                ctx.Record(label, $"ComputeDevice {computeDevice.GetValue(exposure)}, Enabled {enabled.GetValue(exposure)}, sun {Get(sunFlag)}; " +
                                  $"{frames} frames, max batches {maxBatches}, max executor jobs {maxJobs}, " +
                                  $"results airstream/sun/body {string.Join("/", results.Select(r => r.Count == 0 ? "-" : $"{r.Count} parts {r.Sum():F2} m2"))}");
            }

            bool offByDefault, noneWithSun, rendersWithGpu, stopsAgain;
            try
            {
                ctx.Step("Fresh settings");
                yield return Wait.Seconds(3);
                ctx.Record("dropdown", deviceSelect?.GetType().GetProperty("ActiveSelection", Any)?.GetValue(deviceSelect));
                yield return Watch("fresh");
                offByDefault = configDevice == "None" && !(bool)enabled.GetValue(exposure) && !rendered;

                ctx.Step("Sun direction on (the window's Enable toggle), device unchanged");
                Set(sunFlag, true);
                yield return Watch("sunOn");
                noneWithSun = !rendered;

                ctx.Step("Device PreferGPU (the window's dropdown)");
                computeDevice.SetValue(exposure, Enum.Parse(deviceType, "PreferGPU"));
                yield return Watch("preferGPU");
                ctx.Record("actualDevice", renderer?.GetType().GetProperty("ActualDevice", Any)?.GetValue(renderer));
                rendersWithGpu = rendered;

                ctx.Step("Device None again");
                computeDevice.SetValue(exposure, Enum.Parse(deviceType, "None"));
                yield return Wait.Seconds(1);
                yield return Watch("noneAgain");
                stopsAgain = !rendered;
            }
            finally
            {
                Set(sunFlag, sunWas);
                if (!customExisted && File.Exists(custom))
                {
                    ctx.Record("customFARConfigWritten", true);
                    File.Delete(custom);
                }
            }

            Assert.True(!customExisted, "no saved FAR settings (Custom_FARConfig.cfg) in the instance");
            Assert.True(offByDefault, $"exposure device None and nothing rendered with fresh settings (config device {configDevice})");
            Assert.True(noneWithSun, "nothing rendered with the sun direction on while the device is None");
            Assert.True(rendersWithGpu, "exposure renders after selecting PreferGPU");
            Assert.True(stopsAgain, "exposure stops after selecting None again");
        }
    }
}
