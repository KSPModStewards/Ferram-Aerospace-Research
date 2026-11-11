using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using HarmonyLib;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    using Device = FerramAerospaceResearch.Resources.Device;

    /// <summary>
    /// FAR's exposure rendering (Geometry/Exposure, FARAeroComponents/VehicleExposure): every part's renderers are drawn
    /// with a flat colour that encodes the part's index into a render texture by an orthographic camera fitted to the
    /// vessel's voxel bounds and looking along a direction; the pixels of each colour are counted (a compute shader on
    /// the GPU device, an AsyncGPUReadback of the texture and a Burst job on the CPU device) and times the area of one
    /// pixel give the area of the part seen from that direction, the parts in front hiding the ones behind.
    /// VehicleExposure renders three directions every frame (LateUpdate): from upstream (airstream, only in an
    /// atmosphere), from the sun and from the vessel's main body, each switched by FARConfig.Exposure.Airstream / Sun /
    /// Body, and only with a compute device (VehicleExposure.ComputeDevice not None). Nothing in FAR consumes the areas
    /// yet but the debug window (ExposureDebugger), so these tests read the results where VehicleExposure keeps them.
    ///
    /// Switching on at runtime: VehicleExposure takes its device from FARConfig.Exposure.Device once, when the vessel's
    /// FARVesselAero adds it, and does not follow the config afterwards; the debug window's drop-down sets
    /// VehicleExposure.ComputeDevice, and that is what the tests set. The directions are read from FARConfig every
    /// frame. FAR's shipped FARConfig.cfg has every direction off and the device None, so each test switches what it
    /// needs and Exposure.Dispose puts it all back when the test ends (and removes a Custom_FARConfig.cfg the test may
    /// have made FAR write).
    ///
    /// The tests use the CPU device (it needs only AsyncGPUReadback of a render texture, which a -batchmode player with a
    /// graphics device has) and compare the GPU device against it where SystemInfo.supportsComputeShaders.
    ///
    /// The test assembly is not compiled against Unity.Mathematics (FAR gets it from KSPBurst), so every member typed
    /// float3 / int2 / float4x4 (RenderRequest.lookDir, RenderResult.forward, VehicleExposure.RenderSize,
    /// Renderer&lt;T&gt;.Render) is reached by reflection here.
    /// </summary>
    public static class ExposureTests
    {
        private const string Rocket = "ComSat Lx Unclamped";   // stock, its launch clamp taken off (crafts/)

        // ------------------------------------------------------------------ the device switch

        /// <summary>
        /// A vessel's exposure starts with the config's device and renders nothing with the device None, even with the
        /// directions on. With the CPU device every part of the vessel gets a sun and a body area (finite, not negative,
        /// some positive), the sun camera looks from the sun and the body camera from the planet. The GPU device counts
        /// the same pixels as the CPU one (where compute shaders are supported), and the device None stops it again.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_ExposureRendersOnlyWithADevice(TestContext ctx)
        {
            yield return Exposure.OnPad(ctx);
            using var x = new Exposure(ctx, FlightGlobals.ActiveVessel);
            using var log = new ExceptionLog();
            yield return x.KeepVoxel();
            Vessel v = x.Vessel;
            object sun = x.Result("sun"), body = x.Result("body");

            ctx.Step("Directions on, device None");
            Assert.Equal(FARConfig.Exposure.Device.Value, x.X.ComputeDevice, "a new vessel's exposure device is the config's");
            x.X.ComputeDevice = Device.None;
            x.Directions(airstream: false, sun: true, body: true);
            yield return Wait.Frames(10);
            Exposure.Areas(sun).Clear();
            Exposure.Areas(body).Clear();
            int maxBatches = 0;
            for (int i = 0; i < 30; ++i)
            {
                yield return null;
                maxBatches = Math.Max(maxBatches, x.ActiveBatches);
            }
            ctx.Record("none", $"Enabled {x.X.Enabled}, max batches {maxBatches}, results sun {Exposure.Areas(sun).Count} body {Exposure.Areas(body).Count}");
            Assert.False(x.X.Enabled, "exposure disabled with the device None");
            Assert.True(maxBatches == 0 && Exposure.Areas(sun).Count == 0 && Exposure.Areas(body).Count == 0,
                        "nothing rendered with the device None and the directions on");

            ctx.Step("Device CPU");
            x.X.ComputeDevice = Device.CPU;
            yield return Exposure.Fresh("sun and body exposure on the CPU device", sun, body);
            ctx.Record("cpuActualDevice", x.ActualDevice);
            Assert.Equal("CPU", x.ActualDevice, "the renderer counts on the CPU");
            ctx.Record("cpuSun", x.Describe(sun));
            ctx.Record("cpuBody", x.Describe(body));
            foreach (object r in new[] { sun, body })
            {
                IReadOnlyDictionary<Part, double> perPart = Exposure.PerPart(r);
                foreach (Part p in v.parts)
                {
                    Assert.True(perPart.TryGetValue(p, out double a), $"{p.partInfo.name} has an exposed area");
                    Assert.True(!double.IsNaN(a) && !double.IsInfinity(a) && a >= 0, $"{p.partInfo.name}'s exposed area ({a}) is finite and not negative");
                }
                Assert.True(v.parts.Sum(p => perPart[p]) > 0, "the vessel has exposed area");
            }

            // the cameras: VehicleExposure looks along (vessel - sun) and (vessel - body), from the sun's and the
            // body's side of the vessel (Renderer.ProjectToCameraRender puts the camera at center - extent * lookDir)
            Vector3 center = v.transform.TransformPoint(x.X.VesselBounds.center);
            Vector3 fromSun = ((Vector3)((Vector3d)center - FlightGlobals.Bodies[0].position)).normalized;
            Vector3 fromBody = ((Vector3)((Vector3d)center - v.mainBody.position)).normalized;
            double sunDot = Vector3.Dot(Exposure.Forward(sun).normalized, fromSun);
            double bodyDot = Vector3.Dot(Exposure.Forward(body).normalized, fromBody);
            ctx.Record("cameras", $"sun look . (vessel - sun) {sunDot:F5}, body look . (vessel - body) {bodyDot:F5}");
            Assert.True(sunDot > 0.999, $"the sun camera looks from the sun at the vessel ({sunDot:F5})");
            Assert.True(bodyDot > 0.999, $"the body camera looks from the planet at the vessel ({bodyDot:F5})");
            Assert.True(Vector3.Dot(Exposure.Position(sun) - center, fromSun) < 0, "the sun camera is on the sun's side of the vessel");
            Assert.True(Vector3.Dot(Exposure.Position(body) - center, fromBody) < 0, "the body camera is on the planet's side of the vessel");

            if (SystemInfo.supportsComputeShaders)
            {
                ctx.Step("Device GPU against CPU");
                Dictionary<Part, double> cpuSun = Exposure.Snapshot(sun, v), cpuBody = Exposure.Snapshot(body, v);
                x.X.ComputeDevice = Device.GPU;
                yield return Exposure.Fresh("sun and body exposure on the GPU device", sun, body);
                ctx.Record("gpuActualDevice", x.ActualDevice);
                Assert.Equal("GPU", x.ActualDevice, "the renderer counts on the GPU");
                ctx.Record("gpuSun", x.Describe(sun));
                ctx.Record("gpuBody", x.Describe(body));
                // recorded, not asserted: two counts of the same scene were expected to agree, but GPU and CPU differ from
                // run to run (4.4% on the whole fairing once, 0.045 against 0.018 m2 on a tank another time), which fits
                // batches sharing one RenderResult (RenderBatch writes areaPerPixel/pixelCounts into it at submission)
                ctx.Record("gpuVsCpuBody", $"{Exposure.Snapshot(body, v).Values.Sum():F4} against {cpuBody.Values.Sum():F4} m2");
                double cpuSunTotal = cpuSun.Values.Sum(), gpuSunTotal = Exposure.Snapshot(sun, v).Values.Sum();
                ctx.Record("gpuVsCpuSun", $"{gpuSunTotal:F4} against {cpuSunTotal:F4} m2");
            }
            else
            {
                ctx.Record("gpu", "no compute shaders on this device: GPU counting not compared");
            }

            ctx.Step("Device None again");
            x.X.ComputeDevice = Device.None;
            // batches already submitted finish (some frames; many more under coverage)
            yield return Wait.Until(() => x.ActiveBatches == 0, 30, "the batches submitted before the device None to finish", shot: false);
            yield return Wait.Frames(2);
            Exposure.Areas(sun).Clear();
            Exposure.Areas(body).Clear();
            yield return Wait.Frames(30);
            Assert.True(Exposure.Areas(sun).Count == 0 && Exposure.Areas(body).Count == 0 && x.ActiveBatches == 0,
                        "nothing rendered after the device is None again");

            log.AssertNone("exposure on and off");
        }

        /// <summary>
        /// A render's result reads as a dictionary from part to area (how the debug window lists it): enumerated every way
        /// (foreach, the non-generic enumerator with its dictionary entries, its own enumerator, LINQ)
        /// it gives each of the vessel's parts once, with the area the result has for it; its keys, values and
        /// count agree, and a part not on the vessel is not in it. The renderer itself reads as a dictionary from part to
        /// index, 0 to n-1. The device PreferGPU counts on the GPU where compute shaders are supported, else on the CPU.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_ExposureResultIsADictionary(TestContext ctx)
        {
            yield return Exposure.OnPad(ctx);
            using var x = new Exposure(ctx, FlightGlobals.ActiveVessel);
            using var log = new ExceptionLog();
            yield return x.KeepVoxel();
            Vessel v = x.Vessel;
            object sun = x.Result("sun");
            x.Directions(airstream: false, sun: true, body: false);
            x.X.ComputeDevice = Device.CPU;
            yield return Exposure.Fresh("sun exposure on the CPU device", sun);

            ctx.Step("the result as a dictionary");
            var dict = (IReadOnlyDictionary<Part, double>)sun;
            int n = v.parts.Count;
            Assert.Equal(n, dict.Count, "one entry per part");
            Assert.Equal(n, dict.Keys.Count(), "one key per part");
            Assert.Equal(n, dict.Values.Count(), "one value per part");
            Assert.True(v.parts.All(dict.ContainsKey), "every part of the vessel is a key");
            Part notOnVessel = v.parts[0].partInfo.partPrefab;
            Assert.False(dict.ContainsKey(notOnVessel), "a part not on the vessel is not a key");
            Assert.False(dict.TryGetValue(notOnVessel, out double none), "nor has an area");
            Assert.Equal(0.0, none, "zero for no area");
            Assert.True(dict.Keys.Select(p => dict[p]).SequenceEqual(dict.Values), "the values in the keys' order");

            void AssertPairs(IEnumerable<KeyValuePair<Part, double>> pairs, string how)
            {
                List<KeyValuePair<Part, double>> list = pairs.ToList();
                Assert.Equal(n, list.Count, $"{how}: one pair per part");
                Assert.Equal(n, list.Select(kv => kv.Key).Distinct().Count(), $"{how}: each part once");
                foreach (KeyValuePair<Part, double> kv in list)
                    Assert.Equal(dict[kv.Key], kv.Value, $"{how}: {kv.Key.partInfo.name}'s area");
            }

            var foreachPairs = new List<KeyValuePair<Part, double>>();
            foreach (KeyValuePair<Part, double> kv in dict)
                foreachPairs.Add(kv);
            AssertPairs(foreachPairs, "foreach");
            AssertPairs(dict.ToDictionary(kv => kv.Key, kv => kv.Value), "ToDictionary");

            // the non-generic enumerator is the result's own, a dictionary enumerator
            var e = (IDictionaryEnumerator)((IEnumerable)sun).GetEnumerator();
            AssertPairs(Entries(e), "IDictionaryEnumerator");
            ((IDisposable)e).Dispose();

            // and the public members the debug window uses (typed with Unity.Mathematics' assembly: by reflection here)
            Type resultType = sun.GetType();
            var own = (IEnumerator<KeyValuePair<Part, double>>)resultType.GetMethod("GetEnumerator", Type.EmptyTypes).Invoke(sun, null);
            var ownPairs = new List<KeyValuePair<Part, double>>();
            while (own.MoveNext())
            {
                ownPairs.Add(own.Current);
                Type et = own.GetType();
                Assert.True(ReferenceEquals(own.Current.Key, et.GetProperty("Key").GetValue(own)), "the enumerator's key");
                Assert.Equal(own.Current.Value, (double)et.GetProperty("Value").GetValue(own), "the enumerator's value");
            }
            own.Dispose();
            AssertPairs(ownPairs, "the result's enumerator");
            Assert.Equal(n, ((ICollection)resultType.GetProperty("Keys").GetValue(sun)).Count, "Keys");
            Assert.True(ReferenceEquals(Exposure.Areas(sun), resultType.GetProperty("Values").GetValue(sun)), "Values are the areas");

            ctx.Step("the renderer as a dictionary of part indices");
            object rendererObject = Refl.Get(x.X, "exposureRenderer");
            var indices = (IReadOnlyDictionary<Part, int>)rendererObject;
            Assert.Equal(n, indices.Count, "the renderer has every part");
            Assert.True(Enumerable.Range(0, n).SequenceEqual(indices.Values.OrderBy(i => i)), "indices 0 to n-1");
            Assert.True(v.parts.All(p => indices.ContainsKey(p) && indices.Keys.Contains(p)), "every part is a key");
            var indexPairs = new List<KeyValuePair<Part, int>>();
            foreach (KeyValuePair<Part, int> kv in indices)
                indexPairs.Add(kv);
            Assert.True(indexPairs.All(kv => indices[kv.Key] == kv.Value), "the pairs are the indices");
            int nonGeneric = 0;
            foreach (object _ in (IEnumerable)indices)
                nonGeneric++;
            Assert.Equal(n, nonGeneric, "the non-generic enumerator's count");
            Type rt = rendererObject.GetType();
            Assert.Equal(n, ((ICollection)rt.GetProperty("Keys").GetValue(rendererObject)).Count, "the renderer's Keys");
            Assert.Equal(n, ((ICollection)rt.GetProperty("Values").GetValue(rendererObject)).Count, "the renderer's Values");
            var rendererOwn = (IEnumerator)rt.GetMethod("GetEnumerator", Type.EmptyTypes).Invoke(rendererObject, null);
            int ownCount = 0;
            while (rendererOwn.MoveNext())
                ownCount++;
            Assert.Equal(n, ownCount, "the renderer's own enumerator's count");

            ctx.Step("the device PreferGPU");
            x.X.ComputeDevice = Device.PreferGPU;
            ctx.Record("preferGpu", $"{x.ActualDevice}, compute shaders {SystemInfo.supportsComputeShaders}");
            Assert.Equal(SystemInfo.supportsComputeShaders ? "GPU" : "CPU", x.ActualDevice, "PreferGPU counts on the GPU where it can");
            yield return Exposure.Fresh("sun exposure on the preferred device", sun);
            Assert.True(Exposure.Total(sun) > 0, "it renders");

            log.AssertNone("reading the exposure results");
        }

        /// <summary>The pairs left in a result's non-generic enumerator, each entry checked against its key and value.</summary>
        private static List<KeyValuePair<Part, double>> Entries(IDictionaryEnumerator e)
        {
            var entries = new List<KeyValuePair<Part, double>>();
            while (e.MoveNext())
            {
                DictionaryEntry entry = e.Entry;
                Assert.True(ReferenceEquals(entry.Key, e.Key) && Equals(entry.Value, e.Value), "the entry is the key and value");
                Assert.Equal(new KeyValuePair<Part, double>((Part)e.Key, (double)e.Value), (KeyValuePair<Part, double>)e.Current, "the current pair");
                entries.Add(new KeyValuePair<Part, double>((Part)entry.Key, (double)entry.Value));
            }
            return entries;
        }

        /// <summary>
        /// A result's enumerator reset enumerates every part again. Regression: before the fix, RenderResult&lt;T&gt;.Enumerator.Reset calls
        /// Reset on boxed copies of its key and value enumerators (struct fields cast to IEnumerator), so it resets
        /// nothing and the enumerator stays at its end.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_ExposureResultEnumeratorResets(TestContext ctx)
        {
            yield return Exposure.OnPad(ctx);
            using var x = new Exposure(ctx, FlightGlobals.ActiveVessel);
            yield return x.KeepVoxel();
            object sun = x.Result("sun");
            x.Directions(airstream: false, sun: true, body: false);
            x.X.ComputeDevice = Device.CPU;
            yield return Exposure.Fresh("sun exposure on the CPU device", sun);
            var e = (IDictionaryEnumerator)((IEnumerable)sun).GetEnumerator();
            int first = Entries(e).Count;
            e.Reset();
            int again = Entries(e).Count;
            ctx.Record("pairs", $"{first}, after Reset {again}");
            Assert.Equal(x.Vessel.parts.Count, first, "every part the first time");
            Assert.Equal(first, again, "every part again after Reset");
        }

        /// <summary>
        /// The flight debug window's exposure panel (FAR's flight window, Debug): showing a direction's render, it has
        /// the debugger copy each render into a texture of the render's size and draws it, with the parts' areas listed;
        /// its device drop-down sets the vessel's device. Its arrow toggle points the debugger's arrow along the render, its
        /// bounds toggle switches on the flight camera's bounds drawing, its shader toggle the camera's exposure shader, and
        /// its section folds and unfolds. The window closed, the copies stop and nothing throws.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_ExposureDebugPanelShowsTheRender(TestContext ctx)
        {
            yield return Exposure.OnPad(ctx);
            using var x = new Exposure(ctx, FlightGlobals.ActiveVessel);
            using var log = new ExceptionLog();
            yield return x.KeepVoxel();
            x.Directions(airstream: false, sun: true, body: false);
            using IDisposable tools = Far.DebugTools();
            FlightGUI gui = Far.FlightGui(x.Vessel);
            bool shown = Far.FlightGuiShown, debugShown = Far.SubWindowOpen(gui, "showDebugWindow");
            object deviceSelect = Refl.Get(x.X, "deviceSelect"), directionSelect = Refl.Get(x.X, "directionSelect");
            int device0 = (int)Refl.Get(deviceSelect, "selectedOption"), direction0 = (int)Refl.Get(directionSelect, "selectedOption");
            object debugger = Refl.Get(Refl.Get(x.X, "sunRequest"), "debugger");
            try
            {
                ctx.Step("the debug window on the sun's render, device CPU");
                // the drop-downs' lists close without a mouse over them: set as their buttons would
                Refl.Set(deviceSelect, "selectedOption", Array.IndexOf(VehicleExposure.DeviceOptions, Device.CPU));
                Refl.Set(directionSelect, "selectedOption", Array.IndexOf(VehicleExposure.DirectionOptions, VehicleExposure.Direction.Sun));
                Far.FlightGuiShown = true;
                Far.SetSubWindow(gui, "showDebugWindow", true);
                yield return Wait.Until(() => Refl.Get(debugger, "texture") != null, 30, "the debugger's copy of the sun's render", shot: false);
                Assert.Equal(Device.CPU, x.X.ComputeDevice, "the panel's device is the vessel's");
                yield return Wait.Frames(10);
                var tex = (RenderTexture)Refl.Get(debugger, "texture");
                object size = typeof(VehicleExposure).GetProperty("RenderSize", BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Instance).GetValue(x.X);
                int w = (int)size.GetType().GetField("x").GetValue(size), h = (int)size.GetType().GetField("y").GetValue(size);
                ctx.Record("debugTexture", $"{tex.width} x {tex.height}, render size {w} x {h}");
                Assert.True(tex.width == w && tex.height == h, $"the copy is the render's size ({tex.width} x {tex.height} against {w} x {h})");
                Assert.True(tex.IsCreated(), "the copy is a live texture");
                yield return Wait.Frames(5);
                log.AssertNone("drawing the exposure debug panel");

                ctx.Step("the arrow along the sun's render");
                string title = KSP.Localization.Localizer.Format("FARDebugWindow");
                var arrow = (ArrowPointer)Refl.Get(debugger, "debugArrow");
                yield return Imgui.Toggle(KSP.Localization.Localizer.Format("FARExposureArrowLabel"), true, title, exact: true);
                yield return Wait.Until(() => arrow.gameObject.activeSelf, 10, "the sun's debug arrow shown", shot: false);
                yield return Wait.Frames(10);
                object sun = x.Result("sun");
                Vector3 forward = Exposure.F3(Refl.Get(sun, "forward")), position = Exposure.F3(Refl.Get(sun, "position"));
                ctx.Record("arrow", $"at {arrow.transform.position} along {arrow.transform.forward}; render from {position} along {forward}");
                Assert.True(Vector3.Angle(arrow.transform.forward, forward) < 1, $"the arrow along the render ({arrow.transform.forward} against {forward})");
                Assert.True((arrow.transform.position - position).magnitude < 0.01f, "the arrow at the render's origin");

                ctx.Step("the vessel's bounds drawn");
                var bounds = (Behaviour)Refl.Get(x.X, "boundsRenderer");
                yield return Imgui.Toggle(KSP.Localization.Localizer.Format("FARDrawBoundsLabel"), true, title, exact: true);
                Assert.True(bounds.enabled, "the bounds renderer on");
                Assert.True(bounds.gameObject == FlightCamera.fetch.mainCamera.gameObject, "on the flight camera, drawing after it renders");
                yield return Wait.Frames(10);
                // a coverage run saw no OnPostRender in those frames: one render of the camera draws the box
                FlightCamera.fetch.mainCamera.Render();
                yield return Wait.Frames(2);
                yield return Imgui.Toggle(KSP.Localization.Localizer.Format("FARDrawBoundsLabel"), false, title, exact: true);
                Assert.False(bounds.enabled, "and off again");

                ctx.Step("the camera's exposure shader");
                yield return Imgui.Toggle(KSP.Localization.Localizer.Format("FARExposureShaderLabel"), true, title, exact: true);
                Assert.True((bool)Refl.Get(x.X, "overrideCameraShader"), "the camera shader override on");
                yield return Wait.Frames(10);
                yield return Imgui.Toggle(KSP.Localization.Localizer.Format("FARExposureShaderLabel"), false, title, exact: true);
                Assert.False((bool)Refl.Get(x.X, "overrideCameraShader"), "and off");
                yield return Imgui.Toggle(KSP.Localization.Localizer.Format("FARExposureArrowLabel"), false, title, exact: true);
                Assert.False(arrow.gameObject.activeSelf, "the arrow hidden again");

                ctx.Step("the exposure section folded and unfolded");
                string section = KSP.Localization.Localizer.Format("FARDebugExposureLabel");
                yield return Imgui.Toggle(section, false, title);
                Assert.False((bool)Refl.Get(Refl.Get(gui, "debugGUI"), "exposure"), "the section folded");
                yield return Imgui.Toggle(section, true, title);
                yield return Wait.Frames(5);
                log.AssertNone("the debug panel's toggles");
            }
            finally
            {
                Far.SetSubWindow(gui, "showDebugWindow", debugShown);
                Far.FlightGuiShown = shown;
                Refl.Set(deviceSelect, "selectedOption", device0);
                Refl.Set(directionSelect, "selectedOption", direction0);
            }

            ctx.Step("the window closed");
            yield return Wait.Frames(10);
            log.AssertNone("after closing the exposure debug panel");
        }

        /// <summary>
        /// Asking for the GPU device where compute shaders are not supported counts on the CPU instead and warns once.
        /// Regression: before the fix, FAR's PhysicalDeviceExtensions.Select(PhysicalDevice) has the warning's test inverted
        /// (if (!computeWarningIssued) return CPU), so the warning is never logged. The test makes FAR believe compute
        /// shaders are not supported (its cached SystemInfo.supportsComputeShaders) and puts it back.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_GpuWithoutComputeShadersWarnsOnce(TestContext ctx)
        {
            yield return Exposure.OnPad(ctx);
            using var x = new Exposure(ctx, FlightGlobals.ActiveVessel);
            Type ext = typeof(Device).Assembly.GetType("FerramAerospaceResearch.Resources.PhysicalDeviceExtensions", true);
            object supported = Refl.GetStatic(ext, "supportsComputeShaders"), warned = Refl.GetStatic(ext, "computeWarningIssued");
            int warnings = 0;
            void OnLog(string condition, string stackTrace, LogType type)
            {
                if (type == LogType.Warning && condition.Contains("Compute shaders are not supported"))
                    warnings++;
            }

            Application.logMessageReceived += OnLog;
            try
            {
                Refl.SetStatic(ext, "supportsComputeShaders", false);
                Refl.SetStatic(ext, "computeWarningIssued", false);
                x.X.ComputeDevice = Device.GPU;
                ctx.Record("device", x.ActualDevice);
                Assert.SkipUnless(x.ActualDevice == "CPU", "the runtime kept FAR's cached compute shader support (a readonly static)");
                x.X.ComputeDevice = Device.CPU;
                x.X.ComputeDevice = Device.GPU;
                Assert.Equal(1, warnings, "one warning that compute shaders are not supported");
            }
            finally
            {
                Application.logMessageReceived -= OnLog;
                Refl.SetStatic(ext, "supportsComputeShaders", supported);
                Refl.SetStatic(ext, "computeWarningIssued", warned);
            }
        }

        // ------------------------------------------------------------------ occlusion: the fairing

        /// <summary>
        /// The ComSat Lx's satellite rides in a procedural fairing. Rendered (CPU) from above, from below and from two
        /// sides, the vessel shows the same silhouette from opposite sides (an orthographic projection of the same parts),
        /// the fairing is seen and the satellite inside it is not, from above or from the side. A vessel modification
        /// (what staging or docking fires) rebuilds the part list without changing what is seen, and leaves one renderer
        /// set per part. Once the fairing is jettisoned the satellite is seen from the side and from above.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_FairingHidesItsPayload(TestContext ctx) => FairingAndPayload(ctx, jettison: false);

        /// <summary>
        /// As Pad_FairingHidesItsPayload, then the fairing jettisoned: the payload is seen from above and from the side.
        /// Explicit: on master the payload stays unseen (0 m2 from above after the jettison, every run), which fits
        /// Renderer<T>.Reset() releasing its renderer sets without clearing the list (they pile up and stale ones are drawn).
        /// </summary>
        [UITest(Kind = "Check", Explicit = true, StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_JettisonedFairingUncoversThePayload(TestContext ctx) => FairingAndPayload(ctx, jettison: true);

        private static IEnumerator FairingAndPayload(TestContext ctx, bool jettison)
        {
            yield return Exposure.OnPad(ctx);
            using var x = new Exposure(ctx, FlightGlobals.ActiveVessel);
            using var log = new ExceptionLog();
            yield return x.KeepVoxel();
            Vessel v = x.Vessel;
            Part fairing = v.parts.FirstOrDefault(p => p.Modules.Contains<ModuleProceduralFairing>());
            Assert.NotNull(fairing, "the ComSat Lx's fairing");
            List<Part> payload = Far.PayloadOf(v, fairing);
            ctx.Record("payload", string.Join(", ", payload.Select(p => p.partInfo.name)));
            ctx.Record("fairingRenderers", $"part {fairing.FindModelRenderersCached().Count}, exposure {x.CachedRendererCount(fairing)}");

            // only these renders: VehicleExposure's own directions off, its device CPU
            x.Directions(airstream: false, sun: false, body: false);
            x.X.ComputeDevice = Device.CPU;

            ctx.Step("Closed fairing: above, below, two sides");
            object[] views = null;
            yield return x.RenderViews(ctx, "closed", r => views = r);
            for (int i = 0; i < 4; i += 2)
                AssertOpposite(views[i], views[i + 1], i == 0 ? "above and below" : "the two sides");
            foreach (int i in new[] { 0, 2 })
            {
                string name = i == 0 ? "above" : "the side";
                double all = Exposure.Total(views[i]), seen = Exposure.Of(views[i], payload), shell = Exposure.Of(views[i], new[] { fairing });
                double a = Exposure.AreaPerPixel(views[i]);
                Assert.True(shell > 100 * a, $"the fairing is seen from {name} ({shell:F3} m2)");
                // a closed fairing hides what it carries: at most a few pixels through the seams between its panels
                Assert.True(seen <= 0.01 * all + 20 * a, $"the payload inside the fairing is hidden from {name} ({seen:F4} m2 of {all:F3})");
            }

            ctx.Step("Vessel modification");
            Dictionary<Part, double> side = Exposure.Snapshot(views[2], v);
            double sidePerPixel = Exposure.AreaPerPixel(views[2]);
            for (int k = 0; k < 3; ++k)
            {
                GameEvents.onVesselStandardModification.Fire(v);
                yield return Wait.Frames(2);
            }
            yield return Wait.Seconds(3);   // FAR revoxelizes on the event too; the bounds come back the same
            // once that voxelization is done: a slow one (under coverage beside other KSPs) still on its way leaves the
            // exposure fitted to other bounds
            FARVesselAero vesselAero = v.GetComponent<FARVesselAero>();
            VehicleAerodynamics va = Refl.Get<VehicleAerodynamics>(vesselAero, "_vehicleAero");
            yield return Wait.Until(() => !Refl.Get<bool>(vesselAero, "_updateQueued") && !va.Voxelizing && Refl.Get<object>(va, "_voxel") != null,
                                    60, "FAR's voxelization after the modifications", shot: false);
            yield return Wait.Frames(5);   // FARVesselAero hands the voxel bounds to the exposure in FixedUpdate
            yield return x.RenderViews(ctx, "modified", r => views = r);
            Exposure.AssertSameAreas(side, Exposure.Snapshot(views[2], v), sidePerPixel, Exposure.AreaPerPixel(views[2]), "from the side, before and after a vessel modification");
            ctx.Record("rendererSets", $"{x.RendererSetCount} renderer sets for {x.ObjectCount} parts ({v.parts.Count} in the vessel) after 3 modifications");
            Assert.Equal(v.parts.Count, x.ObjectCount, "one exposure object per part after a vessel modification");

            if (!jettison)
                yield break;
            ctx.Step("Jettison the fairing");
            Part core = v.rootPart;
            fairing.Modules.GetModule<ModuleProceduralFairing>().DeployFairing();
            yield return Wait.Until(() => !core.ShieldedFromAirstream, 30, "the probe core exposed after the fairing goes", shot: false);
            yield return Wait.Seconds(10);   // the panels fly clear of the vessel's bounds, and it is revoxelized
            v = FlightGlobals.ActiveVessel;
            yield return x.RenderViews(ctx, "open", r => views = r);
            foreach (int i in new[] { 0, 2 })
            {
                string name = i == 0 ? "above" : "the side";
                double seen = Exposure.Of(views[i], payload.Where(p => p != null && p.vessel == v));
                Assert.True(seen > 20 * Exposure.AreaPerPixel(views[i]), $"the payload is seen from {name} after the jettison ({seen:F4} m2)");
            }

            // Renderer<T>.Reset releases its renderer sets to the pool but keeps them in its list, so each
            // modification appends another set per part and every renderer is drawn once more per batch
            Assert.Equal(x.ObjectCount, x.RendererSetCount, "renderer sets drawn per batch: one per part, however often the vessel was modified");

            log.AssertNone("rendering the ComSat Lx");
        }

        // the silhouette of the same parts from opposite sides is the same set mirrored, on a mirrored pixel grid (the
        // camera box is fitted to the same bounds corners): only pixels exactly on an edge may round the other way
        private static void AssertOpposite(object a, object b, string what)
        {
            double ta = Exposure.Total(a), tb = Exposure.Total(b), app = Exposure.AreaPerPixel(a);
            Assert.Near(app, Exposure.AreaPerPixel(b), 0.01 * app, $"the same pixel area from {what}");
            Assert.Near(ta, tb, 0.01 * Math.Max(ta, tb) + 20 * app, $"the vessel's area seen from {what}");
        }

        // ------------------------------------------------------------------ the render size

        /// <summary>
        /// The debug window's width and height (VehicleExposure.RenderSize) change the resolution, not the areas: at
        /// 256x256 a pixel is 4 times the area it is at 512x512 and the vessel's body area stays the same, to the
        /// pixels on its outline. The window sets the device from its drop-down on every GUI pass and the size after it,
        /// so the size change is followed by the device being set again (to the same value), as two passes of
        /// VehicleExposure.Display do. CPU device, and the GPU one where compute shaders are supported; no exceptions.
        /// </summary>
        // Regression: before the fix, RenderBatch.Device's setter does isDirty = (value != device), wiping the dirty flag a size
        // change set, so the next GUI pass's device assignment leaves the batches unrebuilt: the GPU path keeps counting the
        // old 512x512 image at the 256x256 pixel area (body area 7.54 m2 against 1.75 m2, measured).
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_RenderSizeChangesResolutionNotArea(TestContext ctx)
        {
            yield return Exposure.OnPad(ctx);
            using var x = new Exposure(ctx, FlightGlobals.ActiveVessel);
            using var log = new ExceptionLog();
            yield return x.KeepVoxel();
            var failures = new List<string>();
            object body = x.Result("body");
            x.Directions(airstream: false, sun: false, body: true);
            var devices = new List<Device> { Device.CPU };
            if (SystemInfo.supportsComputeShaders)
                devices.Add(Device.GPU);
            foreach (Device device in devices)
            {
                ctx.Step($"{device}: 512x512 then 256x256");
                x.SetRenderSize(512, 512);
                x.X.ComputeDevice = device;
                yield return Exposure.Fresh($"body exposure at 512x512 ({device})", body);
                double a512 = Exposure.Total(body), app512 = Exposure.AreaPerPixel(body);

                x.SetRenderSize(256, 256);
                x.X.ComputeDevice = device;   // the next VehicleExposure.Display pass
                yield return Wait.Frames(20);
                yield return Exposure.Fresh($"body exposure at 256x256 ({device})", body);
                double a256 = Exposure.Total(body), app256 = Exposure.AreaPerPixel(body);
                ctx.Record($"size{device}", $"512: {a512:F4} m2 ({app512:E3} m2/px), 256: {a256:F4} m2 ({app256:E3} m2/px)");
                // the projection box follows the vessel's bounds, which drift on the pad: area per pixel times the pixel
                // count went 26.2, 25.4, 24.9, 24.1 m2 over the four measurements (CPU 512, 256, GPU 512, 256); under
                // coverage, slower, the ratio came out 3.73 and 4.43
                if (Math.Abs(app256 / app512 - 4) > 0.8)
                    failures.Add($"{device}: pixel area at 256 is {app256 / app512:F3} times the one at 512, not 4");
                // outline pixels: a 256 pixel is twice as wide, an outline of perimeter L moves the area by up to
                // L * width / 2; the rocket seen end-on is a disc with fins, outline about 4 times its diameter
                double outline = 4 * 2 * Math.Sqrt(a512 / Math.PI);
                // plus the drift: within 5% in plain runs, but under coverage 2.26 -> 1.87 m2 (CPU) and 1.65 -> 2.09 (GPU).
                // The bug this guards against is a factor of 4
                double tol = 0.35 * a512 + outline * Math.Sqrt(app256);
                if (double.IsNaN(a256) || Math.Abs(a256 - a512) > tol)
                    failures.Add($"{device}: body area {a256:F4} m2 at 256x256 against {a512:F4} at 512x512 (tolerance {tol:F4})");
            }

            Assert.True(failures.Count == 0, string.Join("; ", failures));
            log.AssertNone("changing the render size");
        }

        // ------------------------------------------------------------------ leaving flight

        private const string HarmonyId = "FerramAerospaceResearch.KspTests.Exposure";

        /// <summary>
        /// Leaving flight while the CPU device counts. The vessel's exposure goes with the vessel: its jobs are
        /// cancelled, their readbacks waited for, and the arrays they read back into and count in freed. FAR's job
        /// executor outlives the scene, so a cancelled job is still updated after that, and must not go on to count the
        /// pixels (Renderer.CountPixels) in the freed arrays: that writes to freed memory, and KSP crashes in UnityPlayer
        /// seconds or a scene or two later (an access violation in FindObjectsOfType, or wherever the damage is next
        /// touched). The pixels are counted in flight, and none once the game has left it.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_LeftWhileCountingOnTheCpu(TestContext ctx)
        {
            var harmony = new Harmony(HarmonyId);
            try
            {
                harmony.Patch(AccessTools.Method(typeof(FerramAerospaceResearch.Geometry.Exposure.Renderer), "CountPixels"),
                              prefix: new HarmonyMethod(typeof(PixelCounts), nameof(PixelCounts.Prefix)));
                yield return Exposure.OnPad(ctx);
                using var x = new Exposure(ctx, FlightGlobals.ActiveVessel);
                using var log = new ExceptionLog();
                yield return x.KeepVoxel();
                object sun = x.Result("sun"), body = x.Result("body");

                ctx.Step("Sun and body exposure counted on the CPU");
                x.Directions(airstream: false, sun: true, body: true);
                x.X.ComputeDevice = Device.CPU;
                yield return Exposure.Fresh("sun and body exposure on the CPU device", sun, body);
                PixelCounts.Reset();
                yield return Wait.Frames(10);

                ctx.Step("Left flight with the exposure counting");
                yield return Nav.ToSpaceCenter(ctx);
                yield return Wait.Seconds(2);
                ctx.Record("counts", $"{PixelCounts.InFlight} in flight over 10 frames and the way out, {PixelCounts.Outside} once out of flight");
                Assert.True(PixelCounts.InFlight > 0, "the CPU device counts the pixels in flight");
                Assert.Equal(0, PixelCounts.Outside, "no pixels counted once the game has left flight (the vessel's arrays are freed)");
                log.AssertNone("leaving flight while counting on the CPU");
            }
            finally
            {
                harmony.UnpatchAll(HarmonyId);
            }
        }

        /// <summary>Renderer.CountPixels calls, in flight and out of it (FAR's job executor counts on the main thread).</summary>
        private static class PixelCounts
        {
            public static int InFlight, Outside;

            public static void Reset() => InFlight = Outside = 0;

            public static void Prefix()
            {
                if (HighLogic.LoadedScene == GameScenes.FLIGHT)
                    ++InFlight;
                else
                    ++Outside;
            }
        }

        // ------------------------------------------------------------------ the airstream in flight

        /// <summary>
        /// The Aeris 4A at Mach 0.5, 3 km, 2 deg: the airstream camera looks downstream from upstream, the frontmost part
        /// is seen, and the vessel's area seen from upstream (its frontal silhouette) is bracketed by FAR's voxel
        /// model: at least its largest cross-section (any section across the flow lies inside the silhouette; times the
        /// cosine of the angle between the flow and FAR's main axis, along which the sections are cut) and at most the
        /// upstream-facing area of all its voxels (FARAeroPartModule.ProjectedAreas along the flow: every line along the
        /// flow through the vessel enters it through at least one such face).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_AirstreamAreaIsBracketedByTheVoxels(TestContext ctx)
        {
            Exposure.RequireReadback();
            yield return Wait.Seconds(5);   // FAR voxelizes the craft after it loads
            FarFlightProbe.Condition c = Far.At(0.5, 3000, 2, "exposure");
            using var x = new Exposure(ctx, FlightGlobals.ActiveVessel);
            using var log = new ExceptionLog();
            yield return FlightTests.Airborne(c);
            yield return x.KeepVoxel();
            yield return FarFlightProbe.Place(c);
            // on only once placed: FAR's pixel-count job is not Burst-compiled (KSPBurst builds none of FAR's jobs), and
            // under coverage each pixel of each render is a recorded visit, which slowed the game so much the placement's
            // wait for the vessel off rails timed out
            x.Directions(airstream: true, sun: false, body: false);
            x.X.ComputeDevice = Device.CPU;
            var samples = new List<FarFlightProbe.Sample>();
            yield return FarFlightProbe.Hold(c, 25, 10, samples, predict: false);
            Vessel v = FlightGlobals.ActiveVessel;
            Assert.True(x.X.InAtmosphere, "the vessel is in the atmosphere");
            object air = x.Result("airstream");
            Exposure.Areas(air).Clear();
            yield return FarFlightProbe.Hold(c, 0, 50, samples, predict: false);
            if (Exposure.Areas(air).Count == 0)
                yield return Wait.Until(() => Exposure.Areas(air).Count > 0, 10, "airstream exposure in flight", shot: false);

            Vector3 flow = v.srf_velocity.normalized;   // world, the direction the vessel moves
            ctx.Record("airstream", x.Describe(air));
            double look = Vector3.Dot(Exposure.Forward(air).normalized, -flow);
            ctx.Record("camera", $"airstream look . -velocity {look:F5}");
            Assert.True(look > 0.995, $"the airstream camera looks downstream ({look:F5})");

            IReadOnlyDictionary<Part, double> perPart = Exposure.PerPart(air);
            // the frontmost of the parts FAR's own voxels say face the flow (a small part tucked into the nose, an RCS
            // block, can have no area of its own upstream)
            Part front = v.parts.Where(p => p.Modules.GetModule<FARAeroPartModule>()?.ProjectedAreaWorld(flow) > 0.05)
                          .OrderByDescending(p => Vector3.Dot(p.transform.position, flow)).First();
            ctx.Record("front", $"{front.partInfo.name}: {perPart[front]:F4} m2");
            Assert.True(perPart[front] > 0, $"the frontmost part ({front.partInfo.name}) is seen from upstream");

            double seen = v.parts.Sum(p => perPart[p]);
            FARVesselAero aero = v.GetComponent<FARVesselAero>();
            double maxSection = aero.MaxCrossSectionArea;
            Vector3 mainAxis = Exposure.MainAxis(aero);   // vessel (vesselTransform) frame
            double cos = Math.Abs(Vector3.Dot(mainAxis.normalized, v.vesselTransform.InverseTransformDirection(flow)));
            double upstream = v.parts.Select(p => p.Modules.GetModule<FARAeroPartModule>()).Where(m => m != null)
                               .Sum(m => m.ProjectedAreaWorld(flow));
            // pixels on the outline: the camera box spans the voxel bounds across the flow, the outline is taken
            // as twice the box's perimeter (a wing's outline runs its span twice), each outline pixel half in error
            Bounds b = x.X.VesselBounds;
            double span = Math.Max(b.size.x, Math.Max(b.size.y, b.size.z));
            double pixel = Math.Sqrt(Exposure.AreaPerPixel(air));
            double outlineTol = 0.5 * (2 * 4 * span) * pixel;
            ctx.Record("bracket", $"max section {maxSection:F3} m2 x cos {cos:F4}, seen {seen:F3} m2, voxel upstream faces {upstream:F3} m2; outline tolerance {outlineTol:F3}");
            Assert.True(maxSection > 0 && upstream > 0, "FAR's voxel model has areas");
            // 5%: the voxels are cut from the part meshes FAR picks (some from colliders), the render draws the
            // part's model renderers
            Assert.True(seen >= 0.95 * cos * maxSection - outlineTol,
                        $"the frontal silhouette ({seen:F3} m2) is at least the largest cross-section ({cos * maxSection:F3} m2)");
            Assert.True(seen <= 1.05 * upstream + outlineTol,
                        $"the frontal silhouette ({seen:F3} m2) is at most the voxels' upstream faces ({upstream:F3} m2)");

            log.AssertNone("airstream exposure in flight");
        }

        /// <summary>
        /// FAR as shipped hands the exposure the vessel's voxel bounds. Explicit: on master it does not in flight, where
        /// VehicleAerodynamics.CreateVoxel drops the voxel as soon as it is made (unless Voxelization.debugInFlight), so
        /// FARVesselAero.FixedUpdate passes VoxelBounds = _voxel?.Bounds ?? new Bounds(): an empty box, which every exposure
        /// camera is fitted to. The other exposure tests keep the voxel (Exposure.KeepVoxel) to test the rest.
        /// </summary>
        [UITest(Kind = "Check", Explicit = true, StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Exposure_GetsTheVoxelBoundsAsShipped(TestContext ctx)
        {
            yield return Wait.Seconds(6);
            Vessel v = FlightGlobals.ActiveVessel;
            yield return Wait.Until(() => FARAPI.VesselVoxelizationCompletedEver(v), 60, "the vessel voxelized", shot: false);
            yield return Wait.Frames(5);
            Bounds b = v.GetComponent<FARVesselAero>().Exposure.VesselBounds;
            ctx.Record("bounds", b.ToString());
            Assert.True(b.size.sqrMagnitude > 0, "the exposure has the vessel's voxel bounds (without debugInFlight)");
        }

        /// <summary>
        /// Regression: the vessel's parts set up again for the exposure (VehicleExposure.ResetParts, on every vessel
        /// modification) leave its renderer one renderer set per part. Before the fix Renderer.Reset gave the sets back
        /// to the pool (shared by every vessel's renderer) but kept them in its list, and the new ones went after them: the
        /// list grew with each reset and drew sets that other renderers took from the pool, renderers of parts long
        /// destroyed among them, each logged as null every frame (tens of thousands of warnings late in a session, the
        /// game slowed to a crawl: Flight_AirstreamAreaIsBracketedByTheVoxels's "vessel off rails" timeouts).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_ExposureResetKeepsOneRendererSetPerPart(TestContext ctx)
        {
            yield return Exposure.OnPad(ctx);
            using var x = new Exposure(ctx, FlightGlobals.ActiveVessel);
            Vessel v = x.Vessel;
            object rendererObject = Refl.Get(x.X, "exposureRenderer");
            var indices = (IReadOnlyDictionary<Part, int>)rendererObject;
            var sets = Refl.Get<List<HashSet<UnityEngine.Renderer>>>(rendererObject, "renderers");
            int n = v.parts.Count;
            string State() => $"{indices.Count} parts indexed, {sets.Count} renderer sets ({sets.Distinct().Count()} distinct)";
            ctx.Record("before", State());
            for (int k = 1; k <= 3; k++)
            {
                ctx.Step($"the parts set up again ({k})");
                x.X.ResetParts(v.parts);
                ctx.Record($"reset{k}", State());
                Assert.Equal(n, indices.Count, "every part indexed once");
                Assert.Equal(n, sets.Count, "a renderer set per part");
                Assert.Equal(n, sets.Distinct().Count(), "no set twice");
                Assert.True(v.parts.All(p => sets[indices[p]].Count > 0), "each part's set holds its renderers");
            }

            yield return Wait.Frames(5);
        }

        // ------------------------------------------------------------------ the renderer's API beyond what FAR calls

        // pixels either way a part's area may move between two renders of the same scene here: the small tank all but
        // hidden behind the fairing is a sliver of boundary pixels, and the craft settles on the pad between renders
        // (69 then 88 px under coverage, measured)
        private const int HiddenPartSlack = 40;

        /// <summary>
        /// The parts of the exposure renderer's API FAR does not call agree with the parts it does (CPU device). Set up
        /// a renderer at a time (Renderer.SetupRenderer) the vessel indexes its parts and renders their areas as when
        /// set up part by part (SetupRenderers, VehicleExposure.ResetParts). Each of the renderer's batches reports the
        /// render size, device and pixel counting kernel the renderer gave it, and gives its job handles by index as
        /// Apply goes through them, refusing an index past the last. The kernel given again to batches already set up
        /// reaches their cached jobs (on the GPU device, where compute shaders are supported, their output buffers bound
        /// to it again), and the areas counted after it are the same. The vessel's debug background colour reads back
        /// and goes to its three debuggers' materials.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_ExposureRendererApiAgreesWithFarsOwnPath(TestContext ctx)
        {
            yield return Exposure.OnPad(ctx);
            using var x = new Exposure(ctx, FlightGlobals.ActiveVessel);
            using var log = new ExceptionLog();
            yield return x.KeepVoxel();
            Vessel v = x.Vessel;
            var renderer = (Geometry.Exposure.Renderer<Part>)Refl.Get(x.X, "exposureRenderer");
            var indices = (IReadOnlyDictionary<Part, int>)renderer;
            var sets = Refl.Get<List<HashSet<UnityEngine.Renderer>>>(renderer, "renderers");
            var cached = (IDictionary)Refl.Get(x.X, "cachedRenderers");
            x.X.ComputeDevice = Device.CPU;
            Vector3 up = ((Vector3)((Vector3d)v.transform.position - v.mainBody.position)).normalized;
            Vector3 side = Vector3.Cross(up, Vector3.up).normalized;
            Vector3[] dirs = { -up, side };
            Color background0 = x.X.DebugBackgroundColor;
            try
            {
                ctx.Step("rendered as FAR sets the parts up");
                x.X.ResetParts(v.parts);
                Dictionary<Part, int> wholeIndices = v.parts.ToDictionary(p => p, p => indices[p]);
                object[] whole = null;
                yield return x.Render("the vessel set up part by part", dirs, r => whole = r);

                ctx.Step("set up again a renderer at a time");
                renderer.Reset();
                int added = 0;
                foreach (Part p in v.parts)
                    foreach (UnityEngine.Renderer r in (IEnumerable<UnityEngine.Renderer>)cached[p])
                    {
                        // every other one without a property block: the renderer's own is taken and coloured
                        Color c = renderer.SetupRenderer(p, r, added % 2 == 0 ? p.mpb : null);
                        Assert.True(c == renderer.ColorOf(p), $"{p.partInfo.name}'s renderer gets the part's colour");
                        added++;
                    }
                ctx.Record("renderers", $"{added} renderers for {v.parts.Count} parts");
                Assert.Equal(v.parts.Count, indices.Count, "every part indexed once");
                Assert.True(v.parts.All(p => indices[p] == wholeIndices[p]), "each part at the index it had");
                Assert.True(v.parts.All(p => sets[indices[p]].SetEquals((IEnumerable<UnityEngine.Renderer>)cached[p])),
                            "each part's set holds its renderers, as set up part by part");
                object[] single = null;
                yield return x.Render("the vessel set up a renderer at a time", dirs, r => single = r);
                for (int i = 0; i < dirs.Length; i++)
                {
                    ctx.Record($"view{i}", $"part by part: {x.Describe(whole[i])}; a renderer at a time: {x.Describe(single[i])}");
                    Exposure.AssertSameAreas(Exposure.Snapshot(whole[i], v), Exposure.Snapshot(single[i], v),
                                             Exposure.AreaPerPixel(whole[i]), Exposure.AreaPerPixel(single[i]), $"view {i}, a renderer at a time",
                                             HiddenPartSlack);
                }

                ctx.Step("the batches' settings and handles");
                List<Geometry.Exposure.RenderBatch> batches = Batches(renderer);
                ctx.Record("batches", batches.Count);
                Assert.True(batches.Count > 0, "the renderer has batches after rendering");
                PropertyInfo rendererSize = renderer.GetType().GetProperty("RenderSize"), batchSize = typeof(Geometry.Exposure.RenderBatch).GetProperty("RenderSize");
                PropertyInfo rendererKernel = renderer.GetType().GetProperty("PixelCountKernel"), batchKernel = typeof(Geometry.Exposure.RenderBatch).GetProperty("PixelCountKernel");
                object kernel = rendererKernel.GetValue(renderer);
                foreach (Geometry.Exposure.RenderBatch b in batches)
                {
                    Assert.True(Equals(rendererSize.GetValue(renderer), batchSize.GetValue(b)), $"a batch's render size is the renderer's ({batchSize.GetValue(b)})");
                    Assert.Equal(renderer.ActualDevice, b.Device, "a batch's device is the renderer's");
                    Assert.True(Equals(kernel, batchKernel.GetValue(b)), "a batch's kernel is the renderer's");
                    var handles = new List<Geometry.Exposure.Executor.Handle>();
                    b.Apply(h => handles.Add(h));
                    for (int i = 0; i < handles.Count; i++)
                        Assert.True(handles[i].Equals(b.Handle(i)), $"handle {i} by index is the one Apply gives");
                    bool refused = false;
                    try { b.Handle(handles.Count); }
                    catch (ArgumentOutOfRangeException) { refused = true; }
                    Assert.True(refused, $"an index past the last ({handles.Count}) refused");
                }

                var devices = new List<Device> { Device.CPU };
                if (SystemInfo.supportsComputeShaders)
                    devices.Add(Device.GPU);
                foreach (Device device in devices)
                {
                    ctx.Step($"the kernel given again to the batches set up ({device})");
                    x.X.ComputeDevice = device;
                    object[] beforeKernel = null, afterKernel = null;
                    yield return x.Render($"the vessel rendered ({device})", dirs, r => beforeKernel = r);
                    rendererKernel.SetValue(renderer, kernel);
                    int jobs = 0;
                    foreach (Geometry.Exposure.RenderBatch b in Batches(renderer))
                        foreach (object job in (IEnumerable)Refl.Get(b, "jobCache"))
                        {
                            object resources = Refl.Get(job, "Resources");
                            Assert.True(Equals(kernel, ((Geometry.Exposure.RenderResources)resources).MainKernel), "a cached job's kernel is the one given");
                            jobs++;
                        }
                    ctx.Record($"jobs{device}", jobs);
                    Assert.True(jobs > 0, "cached jobs given the kernel");
                    yield return x.Render($"the vessel rendered after the kernel given again ({device})", dirs, r => afterKernel = r);
                    for (int i = 0; i < dirs.Length; i++)
                        Exposure.AssertSameAreas(Exposure.Snapshot(beforeKernel[i], v), Exposure.Snapshot(afterKernel[i], v),
                                                 Exposure.AreaPerPixel(beforeKernel[i]), Exposure.AreaPerPixel(afterKernel[i]),
                                                 $"view {i} after the kernel given again ({device})", HiddenPartSlack);
                }

                ctx.Step("the debug background colour");
                var colour = new Color(0.2f, 0.4f, 0.6f, 1);
                x.X.DebugBackgroundColor = colour;
                Assert.True(x.X.DebugBackgroundColor == colour, "the colour reads back");
                foreach (string d in new[] { "airstream", "sun", "body" })
                {
                    var debugger = (Geometry.Exposure.Debugger)Refl.Get(Refl.Get(x.X, d + "Request"), "debugger");
                    Assert.True(debugger.BackgroundColor == colour, $"the {d} debugger's background");
                    Assert.True(debugger.Material.GetColor("_BackgroundColor") == colour, $"the {d} debugger's material's background");
                }

                log.AssertNone("the renderer's API");
            }
            finally
            {
                x.X.DebugBackgroundColor = background0;
                x.X.ResetParts(v.parts);
            }
        }

        /// <summary>The renderer's batches, active and pooled (Renderer.ForEachJob, as its settings reach them).</summary>
        private static List<Geometry.Exposure.RenderBatch> Batches(Geometry.Exposure.Renderer<Part> renderer)
        {
            var batches = new List<Geometry.Exposure.RenderBatch>();
            MethodInfo forEach = renderer.GetType().GetMethod("ForEachJob", BindingFlags.NonPublic | BindingFlags.Instance)
                                         .MakeGenericMethod(typeof(object));
            forEach.Invoke(renderer, new object[] { null, (Action<Geometry.Exposure.RenderBatch, object>)((b, _) => batches.Add(b)) });
            return batches;
        }

        /// <summary>
        /// The exposure's small pieces, outside flight. The device choice (PhysicalDeviceExtensions.Select) gives the CPU
        /// and the GPU when asked for them, for PreferGPU the device its callback form picks (the GPU where compute
        /// shaders are supported), and refuses None. RenderResources.SetupTexture makes a texture of the size asked for
        /// (a render texture of the resources' own size; a point-filtered ARGB32 texture of the size given) and keeps it
        /// until the size changes, when the one it replaces is released. A debugger's arrow colour reads back as set, and
        /// the debugger disposed takes its arrow with it, GameObject and all.
        /// </summary>
        // Regression: before the fix, ExposureDebugger.Dispose destroyed the arrow's component (ArrowPointer), not its
        // GameObject: every vessel exposure disposed left three empty "ArrowPointer" objects in the scene.
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 300)]
        public static IEnumerator Exposure_DeviceChoiceTexturesAndDebugArrow(TestContext ctx)
        {
            using var log = new ExceptionLog();
            ctx.Step("the device choice");
            Assert.Equal(Resources.PhysicalDevice.CPU, Resources.PhysicalDeviceExtensions.Select(Device.CPU), "CPU asked for, CPU");
            Assert.Equal(Resources.PhysicalDevice.GPU, Resources.PhysicalDeviceExtensions.Select(Device.GPU), "GPU asked for, GPU");
            Resources.PhysicalDevice picked = Resources.PhysicalDevice.CPU, preferred = Resources.PhysicalDeviceExtensions.Select(Device.PreferGPU);
            int calls = 0;
            Resources.PhysicalDeviceExtensions.Select<int>(Device.PreferGPU, (d, _) => { picked = d; calls++; }, 0);
            ctx.Record("preferGpu", $"{preferred}, compute shaders {SystemInfo.supportsComputeShaders}");
            Assert.True(calls == 1 && preferred == picked, $"PreferGPU picks what the callback form picks ({preferred} against {picked})");
            Assert.Equal(SystemInfo.supportsComputeShaders ? Resources.PhysicalDevice.GPU : Resources.PhysicalDevice.CPU, picked,
                         "PreferGPU is the GPU where compute shaders are supported");
            bool refused = false;
            try { Resources.PhysicalDeviceExtensions.Select(Device.None); }
            catch (ArgumentOutOfRangeException) { refused = true; }
            Assert.True(refused, "None refused");

            ctx.Step("textures set up");
            var resources = new Geometry.Exposure.RenderResources();
            RenderTexture target = null;
            Texture2D host = null;
            try
            {
                PropertyInfo texSize = typeof(Geometry.Exposure.RenderResources).GetProperty("ActualTexSize");
                Type int2 = texSize.PropertyType;
                object size64x32 = Activator.CreateInstance(int2, 64, 32), size16 = Activator.CreateInstance(int2, 16, 16);
                texSize.SetValue(resources, size64x32);   // the resources' size, as a job sets it
                resources.SetupTexture(ref target);
                Assert.True(target != null && target.width == 64 && target.height == 32, $"a render texture of the resources' size ({target?.width} x {target?.height})");
                RenderTexture first = target;
                resources.SetupTexture(ref target);
                Assert.True(ReferenceEquals(first, target), "the same size keeps it");

                MethodInfo setup2D = typeof(Geometry.Exposure.RenderResources).GetMethods()
                                                                               .First(m => m.Name == "SetupTexture" && m.GetParameters().Length == 2 &&
                                                                                           m.GetParameters()[0].ParameterType.GetElementType() == typeof(Texture2D));
                var args = new object[] { null, size64x32 };
                setup2D.Invoke(null, args);
                host = (Texture2D)args[0];
                Assert.True(host != null && host.width == 64 && host.height == 32, $"a texture of the size given ({host?.width} x {host?.height})");
                Assert.True(host.format == TextureFormat.ARGB32 && host.filterMode == FilterMode.Point, $"ARGB32, point filtered ({host.format}, {host.filterMode})");
                Texture2D firstHost = host;
                setup2D.Invoke(null, args);
                Assert.True(ReferenceEquals(firstHost, args[0]), "the same size keeps it");
                args[1] = size16;
                setup2D.Invoke(null, args);
                host = (Texture2D)args[0];
                Assert.True(!ReferenceEquals(firstHost, host) && host.width == 16 && host.height == 16, "a new size, a new texture");
                yield return null;   // Object.Destroy takes effect at the end of the frame
                Assert.True(firstHost == null, "the texture it replaced destroyed");
            }
            finally
            {
                resources.Dispose();
                if (target != null)
                    target.Release();
                if (host != null)
                    UnityEngine.Object.Destroy(host);
            }

            ctx.Step("a debugger's arrow");
            var debugger = new FARGUI.ExposureDebugger();
            var arrow = (ArrowPointer)Refl.Get(debugger, "debugArrow");
            GameObject arrowObject = arrow.gameObject;
            debugger.ArrowColor = Color.green;
            Assert.True(debugger.ArrowColor == Color.green, $"the arrow's colour reads back ({debugger.ArrowColor})");
            Assert.True(arrow.Color == Color.green, "and is the arrow's");
            debugger.Dispose();
            yield return null;
            Assert.True(arrow == null, "the arrow destroyed with its debugger");
            Assert.True(arrowObject == null, "and its GameObject");
            log.AssertNone("the exposure's small pieces");
        }

        // ------------------------------------------------------------------ access

        /// <summary>The active vessel's VehicleExposure, its renderer and results, and the settings a test changes.</summary>
        private sealed class Exposure : IDisposable
        {
            private const BindingFlags Any = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;

            public readonly Vessel Vessel;
            public readonly VehicleExposure X;
            private readonly object renderer;   // Geometry.Exposure.Renderer<Part>

            private readonly TestContext ctx;
            private readonly bool airstream0, sun0, body0, debugInFlight0;
            private readonly Device device0;
            private readonly object size0;
            private readonly FarConfigFile configFile = new FarConfigFile();

            public Exposure(TestContext ctx, Vessel v)
            {
                this.ctx = ctx;
                Vessel = v;
                FARVesselAero aero = v.GetComponent<FARVesselAero>() ?? throw new UITestFailure("no FARVesselAero on the vessel");
                X = aero.Exposure ?? throw new UITestFailure("FARVesselAero has no VehicleExposure");
                renderer = Refl.Get(X, "exposureRenderer") ?? throw new UITestFailure("VehicleExposure has no exposureRenderer");
                debugInFlight0 = FARConfig.Voxelization.DebugInFlight;
                airstream0 = FARConfig.Exposure.Airstream.Value;
                sun0 = FARConfig.Exposure.Sun.Value;
                body0 = FARConfig.Exposure.Body.Value;
                device0 = X.ComputeDevice;
                size0 = RenderSizeProperty.GetValue(X);
            }

            /// <summary>
            /// Skips the test where the graphics device cannot read back (both exposure devices read their render back through
            /// AsyncGPUReadback; Mesa's software OpenGL on a Linux runner has none).
            /// </summary>
            public static void RequireReadback() =>
                Assert.SkipUnless(SystemInfo.supportsAsyncGPUReadback, $"{SystemInfo.graphicsDeviceVersion} has no AsyncGPUReadback, which the exposure reads back through");

            /// <summary>The rocket on the pad, voxelized, and the graphics device able to read back. Changes nothing.</summary>
            public static IEnumerator OnPad(TestContext ctx)
            {
                yield return Wait.Seconds(6);   // FAR voxelizes the vessel once it is on the pad
                Vessel v = FlightGlobals.ActiveVessel;
                yield return Wait.Until(() => FARAPI.VesselVoxelizationCompletedEver(v), 60, "the vessel voxelized", shot: false);
                yield return Wait.Frames(5);   // FARVesselAero hands the voxel bounds to the exposure in FixedUpdate
                ctx.Record("graphics", $"{SystemInfo.graphicsDeviceType} {SystemInfo.graphicsDeviceVersion}, compute shaders {SystemInfo.supportsComputeShaders}, async readback {SystemInfo.supportsAsyncGPUReadback}");
                RequireReadback();
                // held still: the renders look along fixed world directions, and the rocket (no launch clamp: the stock one
                // does not load in KspTest's instances) never settles on its engine bell, so its parts' areas drift between
                // two renders seconds apart, past the tolerance on a slow CI runner. Kinematic, not packed: FAR still updates it
                ctx.Record("moving", $"angular velocity {v.angularVelocity.magnitude:G3} rad/s, speed {v.srf_velocity.magnitude:G3} m/s");
                foreach (Part p in v.parts)
                    if (p.Rigidbody != null)
                        p.Rigidbody.isKinematic = true;
                yield return Wait.Frames(2);
            }

            /// <summary>
            /// The voxel bounds into the exposure, the way FAR's debug setting gets them there: in flight FAR drops the voxel
            /// as soon as it is made (VehicleAerodynamics.CreateVoxel's finally, unless Voxelization.debugInFlight), before
            /// FARVesselAero.FixedUpdate hands VoxelBounds (_voxel?.Bounds, else empty) to the exposure, which so gets an
            /// empty box (Exposure_GetsTheVoxelBoundsAsShipped). On until the Exposure is disposed.
            /// </summary>
            public IEnumerator KeepVoxel()
            {
                ctx.Record("boundsAsShipped", $"{X.VesselBounds}");
                FARConfig.Voxelization.DebugInFlight = true;
                FARVesselAero vesselAero = Vessel.GetComponent<FARVesselAero>();
                vesselAero.VesselUpdateEvent(Vessel);
                // the voxelization asked for done, not just bounds there: an earlier test's voxel (kept by the debug
                // setting) can still be the one in place
                VehicleAerodynamics va = Refl.Get<VehicleAerodynamics>(vesselAero, "_vehicleAero");
                yield return Wait.Frames(2);
                yield return Wait.Until(() => !Refl.Get<bool>(vesselAero, "_updateQueued") && !va.Voxelizing && Refl.Get<object>(va, "_voxel") != null,
                                        60, "FAR's voxelization of the vessel", shot: false);
                yield return Wait.Until(() => X.VesselBounds.size.sqrMagnitude > 0, 60, "the exposure given the vessel's voxel bounds", shot: false);
                yield return Wait.Frames(5);
                ctx.Record("bounds", $"{X.VesselBounds}");
            }

            public void Directions(bool airstream, bool sun, bool body)
            {
                FARConfig.Exposure.Airstream.Value = airstream;
                FARConfig.Exposure.Sun.Value = sun;
                FARConfig.Exposure.Body.Value = body;
            }

            /// <summary>Everything the test changed back as it was.</summary>
            public void Dispose()
            {
                FARConfig.Voxelization.DebugInFlight = debugInFlight0;
                if (X != null)
                {
                    X.ComputeDevice = device0;
                    RenderSizeProperty.SetValue(X, size0);
                }
                Directions(airstream0, sun0, body0);
                if (configFile.Restore())
                    ctx.Record("customFARConfigWritten", true);
            }

            private static PropertyInfo RenderSizeProperty =>
                typeof(VehicleExposure).GetProperty("RenderSize", Any) ?? throw new UITestFailure("VehicleExposure has no RenderSize");

            /// <summary>VehicleExposure.RenderSize = int2(w, h).</summary>
            public void SetRenderSize(int w, int h)
            {
                Type int2 = RenderSizeProperty.PropertyType;
                RenderSizeProperty.SetValue(X, Activator.CreateInstance(int2, w, h));
            }

            /// <summary>The RenderResult&lt;Part&gt; of VehicleExposure's airstream, sun or body request.</summary>
            public object Result(string direction)
            {
                object request = Refl.Get(X, direction + "Request");
                return Refl.Get(request, "result") ?? throw new UITestFailure($"VehicleExposure's {direction} request has no result");
            }

            public string ActualDevice => renderer.GetType().GetProperty("ActualDevice", Any)?.GetValue(renderer)?.ToString();
            public int ActiveBatches => (int)renderer.GetType().GetProperty("ActiveBatches", Any).GetValue(renderer);
            public int ObjectCount => (int)renderer.GetType().GetProperty("Count", Any).GetValue(renderer);
            public int RendererSetCount => ((IList)Refl.Get(renderer, "renderers")).Count;

            public int CachedRendererCount(Part p) =>
                Refl.Get(X, "cachedRenderers") is IDictionary d && d.Contains(p) ? ((IList)d[p]).Count : -1;

            public static List<double> Areas(object result) => (List<double>)Refl.Get(result, "Areas");
            public static IReadOnlyDictionary<Part, double> PerPart(object result) => (IReadOnlyDictionary<Part, double>)result;
            public static double AreaPerPixel(object result) => (double)Refl.Get(result, "areaPerPixel");
            public static Vector3 Forward(object result) => F3(Refl.Get(result, "forward"));
            public static Vector3 Position(object result) => F3(Refl.Get(result, "position"));
            public static double Total(object result) => Areas(result).Sum();

            public static double Of(object result, IEnumerable<Part> parts)
            {
                IReadOnlyDictionary<Part, double> perPart = PerPart(result);
                return parts.Sum(p => perPart.TryGetValue(p, out double a) ? a : 0);
            }

            public static Dictionary<Part, double> Snapshot(object result, Vessel v)
            {
                IReadOnlyDictionary<Part, double> perPart = PerPart(result);
                return v.parts.ToDictionary(p => p, p => perPart.TryGetValue(p, out double a) ? a : double.NaN);
            }

            internal static Vector3 F3(object f3)
            {
                Type t = f3.GetType();
                return new Vector3((float)t.GetField("x").GetValue(f3), (float)t.GetField("y").GetValue(f3), (float)t.GetField("z").GetValue(f3));
            }

            /// <summary>FAR's main axis (VehicleAerodynamics._vehicleMainAxis), along which it cuts the cross-sections.</summary>
            public static Vector3 MainAxis(FARVesselAero aero) => (Vector3)Refl.Get(aero.VehicleAero, "_vehicleMainAxis");

            public string Describe(object result)
            {
                IReadOnlyDictionary<Part, double> perPart = PerPart(result);
                string parts = string.Join(", ", Vessel.parts.Where(p => perPart.TryGetValue(p, out double a) && a > 0)
                                                     .Select(p => $"{p.partInfo.name} {perPart[p]:F3}"));
                return $"total {Total(result):F4} m2, {AreaPerPixel(result):E3} m2/px, {ObjectCount} objects / {Areas(result).Count} counts: {parts}";
            }

            /// <summary>
            /// Clears the results and waits for all of them to be filled again: from a batch submitted after whatever the
            /// test changed (10 frames first, for the batches already submitted to finish; a readback takes a few).
            /// </summary>
            public static IEnumerator Fresh(string what, params object[] results)
            {
                yield return Wait.Frames(10);
                foreach (object r in results)
                    Areas(r).Clear();
                yield return Wait.Until(() => results.All(r => Areas(r).Count > 0), 30, what, shot: false);
            }

            /// <summary>
            /// Renders the vessel from the given directions (world; the camera looks along each) through VehicleExposure's
            /// renderer, as VehicleExposure.LateUpdate does (directions in the vessel's frame, its localToWorldMatrix),
            /// each into its own RenderResult&lt;Part&gt;, and waits for them.
            /// </summary>
            public IEnumerator Render(string what, Vector3[] worldDirs, Action<object[]> got)
            {
                Type requestType = typeof(VehicleExposure).GetField("airstreamRequest", Any).FieldType;
                FieldInfo lookDir = requestType.GetField("lookDir"), resultField = requestType.GetField("result");
                Type resultType = requestType.Assembly.GetType("FerramAerospaceResearch.Geometry.Exposure.RenderResult`1", true)
                                             .MakeGenericType(typeof(Part));
                var requests = (IList)Activator.CreateInstance(typeof(List<>).MakeGenericType(requestType));
                var results = new object[worldDirs.Length];
                for (int i = 0; i < worldDirs.Length; ++i)
                {
                    Vector3 d = Vessel.transform.InverseTransformDirection(worldDirs[i]).normalized;
                    results[i] = Activator.CreateInstance(resultType);
                    resultType.GetField("renderer").SetValue(results[i], renderer);
                    object request = Activator.CreateInstance(requestType);   // boxed: the field writes stay in it
                    lookDir.SetValue(request, Activator.CreateInstance(lookDir.FieldType, d.x, d.y, d.z));
                    resultField.SetValue(request, results[i]);
                    requests.Add(request);
                }

                MethodInfo render = renderer.GetType().GetMethods().First(m => m.Name == "Render" && m.IsGenericMethodDefinition)
                                            .MakeGenericMethod(requests.GetType());
                Type matrixType = render.GetParameters()[1].ParameterType;
                if (matrixType.IsByRef)
                    matrixType = matrixType.GetElementType();
                MethodInfo toMatrix = matrixType.GetMethods(BindingFlags.Public | BindingFlags.Static)
                                                .FirstOrDefault(m => m.Name == "op_Implicit" && m.ReturnType == matrixType &&
                                                                     m.GetParameters().Length == 1 &&
                                                                     m.GetParameters()[0].ParameterType == typeof(Matrix4x4))
                                      ?? throw new UITestFailure("no float4x4 conversion from Matrix4x4");
                object matrix = toMatrix.Invoke(null, new object[] { Vessel.transform.localToWorldMatrix });
                render.Invoke(renderer, new[] { requests, matrix });
                yield return Wait.Until(() => results.All(r => Areas(r).Count > 0), 30, what, shot: false);
                got(results);
            }

            /// <summary>
            /// Above (looking down), below (looking up), and two opposite sides, the sides square to the world's Y axis
            /// (Unity.Mathematics' LookRotation, which places the camera, takes the world's up as its up vector).
            /// </summary>
            public IEnumerator RenderViews(TestContext ctx, string label, Action<object[]> got)
            {
                Vector3 up = ((Vector3)((Vector3d)Vessel.transform.position - Vessel.mainBody.position)).normalized;
                Vector3 side = Vector3.Cross(up, Vector3.up).normalized;
                object[] views = null;
                yield return Render($"the vessel rendered from four sides ({label})", new[] { -up, up, side, -side }, r => views = r);
                string[] names = { "above", "below", "side", "otherSide" };
                for (int i = 0; i < views.Length; ++i)
                    ctx.Record($"{label}_{names[i]}", Describe(views[i]));
                got(views);
            }

            /// <summary>
            /// Per-part areas the same, but for a few pixels each (the vessel sat still in between), each render's area of
            /// a pixel given (its framing).
            /// </summary>
            public static void AssertSameAreas(Dictionary<Part, double> before, Dictionary<Part, double> after, double perPixelBefore,
                                               double perPixelAfter, string what, int slackPixels = 10)
            {
                // reframed: the renders are fitted to FAR's voxel bounds, and its voxelization on the modification can
                // come out a voxel off the first (the pixel area 2.114E-4 then 2.188E-4 m2 under coverage); a part seen
                // edge-on, a winglet from the side (0.12 m2), is mostly boundary pixels and moved 10-11% either way
                bool reframed = Math.Abs(perPixelAfter / perPixelBefore - 1) > 0.01;
                double areaPerPixel = Math.Max(perPixelBefore, perPixelAfter);
                foreach (KeyValuePair<Part, double> pair in before)
                {
                    after.TryGetValue(pair.Key, out double a);
                    // 5%: two renders of the same scene were expected to count the same pixels, but the GPU and CPU
                    // counts of the ComSat Lx's fairing differ by 4.4% and one render to the next by 2% (measured): recorded
                    double tol = (reframed ? 0.15 : 0.05) * Math.Max(pair.Value, a) + slackPixels * areaPerPixel;
                    Assert.Near(pair.Value, a, tol, $"{pair.Key.partInfo.name}'s area, {what}");
                }
            }
        }
    }
}
