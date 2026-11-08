using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using KSP.Localization;
using KspTest;
using UnityEngine;
using Cell = FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation.PerformanceEnvelopeCalculator.CellResult;
using static FerramAerospaceResearch.KspTests.Refl;
using static FerramAerospaceResearch.KspTests.EditorWindow;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's flight heatmap window on the stock Aeris 4A with AJE, flying, beyond showing the editor's sweep: the
    /// sweep found again on disk, no sweep or another craft's, its controls, and a click on a cell under a
    /// (synthetic) mouse.
    /// </summary>
    public static class EnvelopeFlightMapTests
    {
        private const string AjeMods = "AdvancedJetEngine";
        private const string AjeRequires = "AJE";

        /// <summary>
        /// A 5 x 5 sweep with stability (Mach 0.4-2 x 0-12 km) in the SPH, its history file kept, then the craft
        /// flown at Mach 0.8, 3 km with the flight heatmap window open. With the sweep gone from memory the window
        /// finds it on disk, by the vessel's name and part count, the same maps as swept; with its file gone too it
        /// says there is no map. Flying a vessel of another name it warns that the map is another craft's. Its
        /// stability toggle draws the stability map too; its Color-by toggles recolor the maps by what they say;
        /// "Center on current speed/altitude" zooms the maps around the vessel's cell. A click on the vessel's own
        /// (flyable) cell sends that cell's speed and altitude to AtmosphereAutopilot and says so in the window; a
        /// click on a cell that cannot be flown is refused, the targets left alone.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Envelope_FlightMapFromDiskAndItsControls(TestContext ctx)
        {
            List<HeatmapResult> swept;
            string cachePath = null;
            try
            {
                ctx.Step("Heatmaps: Mach 0.4-2 x 0-12 km, 5 x 5, with stability; its file kept");
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                var sw = new SweepOut();
                yield return Sweep(new[] { "0.4", "2", "0", "12", "5", "5" }, true, true, sw);
                cachePath = sw.CachePath;
                swept = sw.Results;
                Assert.True(cachePath != null && File.Exists(cachePath), "the sweep filed on disk");
            }
            finally
            {
                CloseWindow();
            }

            ctx.Step("Launch to the runway; fly at Mach 0.8, 3 km");
            EditorLogic.fetch.launchSiteName = "Runway";
            yield return Nav.Launch(ctx);
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition cruise = Far.At(0.8, 3000, 2, "cruise M0.8 3km");
            yield return FlightTests.Airborne(cruise);

            Vessel v = FlightGlobals.ActiveVessel;
            FlightGUI gui = Far.FlightGui(v);
            object heatmapGui = Get(gui, "_heatmapGUI");
            bool shown = Far.FlightGuiShown, heatmapOpen = Get<bool>(gui, "showHeatmapWindow");
            HeatmapRenderer.Metric metric = HeatmapRenderer.ColorMetric;
            string vesselName = v.vesselName;
            string title = L("FARFlightHeatmapTitle");
            try
            {
                Far.FlightGuiShown = true;
                yield return Imgui.WaitWindow(Far.MainWindow, 30);
                yield return Imgui.Toggle(L("FARFlightGUIHeatmapBtn"), true, Far.MainWindow, exact: true);
                yield return Imgui.WaitWindow(title);

                ctx.Step("The sweep gone from memory: found on disk");
                HeatmapResult.Cached = null;
                Set(heatmapGui, "_diskLoadTriedFor", null);
                yield return Wait.Until(() => HeatmapResult.Cached != null, 15, "the window to load the sweep from disk", shot: false);
                Assert.False(ReferenceEquals(swept, HeatmapResult.Cached), "the map read from disk, not the one in memory");
                Maps.AssertSameSweep(swept, HeatmapResult.Cached, "the sweep read back by the flight window");
                yield return Wait.Frames(3);
                Assert.False(Imgui.IsVisible(L("FARFlightHeatmapNone").Substring(0, 20), title), "the window has a map");

                ctx.Step("No sweep in memory or on disk");
                File.Delete(cachePath);
                HeatmapResult.Cached = null;
                Set(heatmapGui, "_diskLoadTriedFor", null);
                yield return Imgui.WaitVisible(L("FARFlightHeatmapNone").Substring(0, 20), title);
                Assert.True(HeatmapResult.Cached == null, "nothing found");

                ctx.Step("Another craft's map");
                HeatmapResult.Cached = swept;
                v.vesselName = "FAR test other craft";
                string mismatch = Localizer.Format("FARFlightHeatmapMismatch", swept[0].CraftName);
                yield return Imgui.WaitVisible(mismatch, title);
                v.vesselName = vesselName;
                yield return Wait.Frames(3);
                Assert.False(Imgui.IsVisible(mismatch, title), "the warning gone with the vessel's own name back");

                var display = Get<List<HeatmapResult>>(heatmapGui, "_displayResults");
                HeatmapResult flight = display[display.Count - 1];
                var renderers = Get<List<HeatmapRenderer>>(heatmapGui, "_envRenderers");
                HeatmapRenderer env = renderers[0], stab = Get<HeatmapRenderer>(heatmapGui, "_stabRenderer");

                ctx.Step("The stability map");
                yield return Imgui.Toggle(L("FARHeatmapsIncludeStab"), true, title);
                yield return Wait.Frames(3);
                yield return Wait.Until(() => Imgui.ImageRect(Get<Texture2D>(stab, "_stabTex"), title) != null, 10, "the stability map drawn",
                                        shot: false, detail: Imgui.Dump);

                ctx.Step("Color by");
                foreach ((string label, HeatmapRenderer.Metric m) in new[] { ("AoA", HeatmapRenderer.Metric.Aoa), ("Throttle", HeatmapRenderer.Metric.Throttle),
                                                                             ("Climb", HeatmapRenderer.Metric.Climb) })
                {
                    yield return Imgui.Toggle(label, true, title, exact: true);
                    Assert.Equal(m, HeatmapRenderer.ColorMetric, $"the maps colored by {label}");
                }

                ctx.Step("Center on the vessel");
                yield return FarFlightProbe.Place(cruise);
                // the vessel's Mach (what the window reads) comes a few frames after its speed off rails
                yield return Wait.Until(() => v.speedOfSound > 0 && Math.Abs(v.mach - v.srfSpeed / v.speedOfSound) < 0.01, 10,
                                        "the vessel's Mach caught up with its speed", shot: false);
                yield return Wait.Frames(3);
                object current = typeof(FlightHeatmapGUI).GetMethod("CurrentCell", Members).Invoke(heatmapGui, new object[] { v, flight });
                Assert.NotNull(current, "the vessel inside the swept range");
                var here = ((int ci, int cj))current;
                yield return Imgui.Toggle("Center on current speed/altitude", true, title);
                yield return Wait.Frames(3);
                object zoom = GetStatic(typeof(HeatmapRenderer), "_zoom");
                int i0 = Get<int>(zoom, "IMin"), i1 = Get<int>(zoom, "IMax"), j0 = Get<int>(zoom, "JMin"), j1 = Get<int>(zoom, "JMax");
                ctx.Record("centered", $"vessel's cell [{here.ci},{here.cj}], zoom Mach cells {i0}-{i1}, altitude cells {j0}-{j1}");
                Assert.True((bool)GetStatic(typeof(HeatmapRenderer), "_zoomActive"), "the maps zoomed");
                Assert.True(i0 <= here.ci && here.ci <= i1 && j0 <= here.cj && here.cj <= j1, "the zoom has the vessel's cell");
                Assert.True(i1 - i0 < flight.MachAxis.Length - 1 || j1 - j0 < flight.AltAxis.Length - 1, "a zoom, not the whole map");
                yield return Imgui.Toggle("Center on current speed/altitude", false, title);
                HeatmapRenderer.ResetZoom();
                yield return Wait.Frames(3);

                ctx.Step("A click on the vessel's cell");
                Cell hereCell = flight.Envelope[here.ci, here.cj];
                Assert.True(hereCell.Feasible, $"the vessel's cell is flyable ({Maps.Describe(hereCell)})");
                Assert.True(AtmosphereAutopilotInterop.Available, "FAR found AtmosphereAutopilot");
                AaCreateControllers(v);
                var envTex = Get<Texture2D>(env, "_envTex");
                yield return Wait.Until(() => Imgui.ImageRect(envTex, title) != null, 10, "the envelope map drawn", shot: false, detail: Imgui.Dump);
                Rect map = Imgui.ImageRect(envTex, title).Value;
                int nM = flight.MachAxis.Length, nA = flight.AltAxis.Length;
                var at = new Vector2(map.x + (here.ci + 0.5f) / nM * map.width, map.y + (nA - 1 - here.cj + 0.5f) / nA * map.height);
                yield return Imgui.MouseClick(at);
                yield return Wait.Until(() => (Get<string>(heatmapGui, "_logMessage") ?? "").StartsWith("Set "), 10, "the click's message", shot: false);
                string sent = Get<string>(heatmapGui, "_logMessage");
                ctx.Record("clicked", $"at {at} on {map}: {sent}");
                yield return Wait.Frames(3);
                if (!Imgui.IsVisible(sent, title))
                    // once (coverage, 4 KSPs) every FAR window was gone here: which of FlightGUI's conditions to draw failed
                    ctx.Record("guiGone", $"showGUI {GetStatic(typeof(FlightGUI), "showGUI")}, showAllGUI {FlightGUI.showAllGUI}, " +
                                          $"its vessel active {ReferenceEquals(gui.Vessel, FlightGlobals.ActiveVessel)}, " +
                                          $"drawn GUIs {((System.Collections.ICollection)GetStatic(typeof(FlightGUIDrawer), "activeGUIs")).Count}, " +
                                          $"heatmap window {Get<bool>(gui, "showHeatmapWindow")}, gui enabled {gui.enabled}");
                Assert.True(Imgui.IsVisible(sent, title), "the window says what it sent");
                AaCruiseTargets(v, out float aaAlt, out float aaSpeed);
                Assert.Near(flight.AltAxis[here.cj], aaAlt, 1, "AA's cruise altitude is the cell's (m)");
                Assert.Near(hereCell.SpeedMPerS, aaSpeed, 0.1, "AA's cruise speed is the cell's (m/s)");

                ctx.Step("A cell that cannot be flown");
                (int i, int j) bad = (-1, -1);
                for (int i = 0; i < nM && bad.i < 0; i++)
                    for (int j = 0; j < nA; j++)
                        if (!flight.Envelope[i, j].Feasible)
                        {
                            bad = (i, j);
                            break;
                        }

                Assert.True(bad.i >= 0, "the map has a cell that cannot be flown");
                typeof(FlightHeatmapGUI).GetMethod("HandleCellClick", Members).Invoke(heatmapGui, new object[] { v, flight, bad });
                Assert.Equal(L("FARFlightHeatmapCellInfeasible"), Get<string>(heatmapGui, "_logMessage"), "the window refuses the cell");
                AaCruiseTargets(v, out float alt2, out float speed2);
                Assert.True(alt2 == aaAlt && speed2 == aaSpeed, "AA's targets left alone");
            }
            finally
            {
                Imgui.ReleasePointer();
                HeatmapRenderer.ResetZoom();
                HeatmapRenderer.ColorMetric = metric;
                HeatmapRenderer.InvalidateColors();
                if (v != null)
                    v.vesselName = vesselName;
                try
                {
                    Set(gui, "showHeatmapWindow", heatmapOpen);
                }
                catch (Exception e)
                {
                    KspTest.Log.Warn("EnvelopeFlightMapTests: closing the flight heatmap window: " + e.Message);
                }

                Far.FlightGuiShown = shown;
                DeleteCache(cachePath);
            }
        }

        // ------------------------------------------------------------------------------------------ AtmosphereAutopilot

        private static Type Aa(string name)
        {
            AssemblyLoader.LoadedAssembly aa = AssemblyLoader.loadedAssemblies.FirstOrDefault(a => a.assembly.GetName().Name == "AtmosphereAutopilot");
            Assert.NotNull(aa, "AtmosphereAutopilot loaded");
            return aa.assembly.GetType("AtmosphereAutopilot." + name) ?? throw new UITestFailure("AA has no type " + name);
        }

        private static IDictionary AaModules(Vessel v)
        {
            Type autopilot = Aa("AtmosphereAutopilot");
            object instance = autopilot.GetProperty("Instance", Members)?.GetValue(null, null);
            Assert.NotNull(instance, "AA's instance");
            var modules = (IDictionary)autopilot.GetMethod("getVesselModules", new[] { typeof(Vessel) }).Invoke(instance, new object[] { v });
            Assert.True(modules != null && modules.Contains(Aa("TopModuleManager")), "AA's module manager for the vessel");
            return modules;
        }

        /// <summary>AA's controllers made for the vessel as its master switch first makes them, nothing switched on.</summary>
        private static void AaCreateControllers(Vessel v)
        {
            IDictionary modules = AaModules(v);
            if (!modules.Contains(Aa("CruiseController")))
                Aa("TopModuleManager").GetMethod("create_context", Members).Invoke(modules[Aa("TopModuleManager")], null);
            Assert.True(modules.Contains(Aa("CruiseController")) && modules.Contains(Aa("ProgradeThrustController")),
                        "AA's cruise and thrust controllers on the vessel");
        }

        private static void AaCruiseTargets(Vessel v, out float altitudeM, out float speed)
        {
            IDictionary modules = AaModules(v);
            PropertyInfo value = Aa("DelayedFieldFloat").GetProperty("Value");
            altitudeM = (float)value.GetValue(Get(modules[Aa("CruiseController")], "desired_altitude"), null);
            speed = (float)value.GetValue(Get(modules[Aa("ProgradeThrustController")], "setpoint_field"), null);
        }
    }
}
