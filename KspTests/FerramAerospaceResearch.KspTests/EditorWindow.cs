using System;
using System.Collections;
using System.Collections.Generic;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using KSP.Localization;
using KspTest;
using static FerramAerospaceResearch.KspTests.Refl;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's editor window (FAR Analysis) driven as a player would: shown on a tab, its dropdowns, the Heatmaps tab's
    /// controls and sweep. What a test changes is remembered and put back when the window closes: the fields it
    /// remembered, the shared viscous option, the heatmap cleanup option and the Heatmaps tab's fields in FAR's config.
    /// </summary>
    internal static class EditorWindow
    {
        // the shared "More accurate aerodynamics" toggle (not localized in FAR)
        public const string ViscousToggle = "More accurate aerodynamics";

        public static string L(string key) => Localizer.Format(key).Trim();

        public static string Window => L("FAREditorTitle");

        /// <summary>The editor window shown (EditorGUI.showGUI, what FAR's toolbar button toggles).</summary>
        public static bool WindowShown
        {
            get => (bool)GetStatic(typeof(EditorGUI), "showGUI");
            set => SetStatic(typeof(EditorGUI), "showGUI", value);
        }

        /// <summary>The window shown on a tab (its localization key), once the craft is voxelized.</summary>
        public static IEnumerator OpenWindow(string tabKey)
        {
            yield return Far.WaitForEditorVoxels();
            SelectTab(tabKey);
            if (!WindowShown)
            {
                // what FAR's toolbar button does in the editor; the button shown pressed to match
                EditorGUI.onAppLaunchToggle();
                if (FARDebugAndSettings.FARDebugButtonStock)
                    FARDebugAndSettings.FARDebugButtonStock.SetTrue(false);
            }

            yield return Imgui.WaitWindow(Window);
            yield return Wait.Frames(2);
        }

        /// <summary>Everything the test changed put back (the Heatmaps tab's fields saved back to FAR's config), and the window closed.</summary>
        public static void CloseWindow()
        {
            RestoreRemembered();
            try
            {
                if (viscousAtStart.HasValue && EditorSimManager.UseAccurateViscousCalc != viscousAtStart.Value)
                    EditorSimManager.UseAccurateViscousCalc = viscousAtStart.Value;
                viscousAtStart = null;
                if (cleanupAtStart.HasValue)
                    HeatmapResult.CleanupDisplay = cleanupAtStart.Value;
                cleanupAtStart = null;
                heatmapsUsed?.SaveSettings();
                heatmapsUsed = null;
                if (EditorGUI.Instance != null && WindowShown)
                {
                    EditorGUI.onAppLaunchToggle();
                    if (FARDebugAndSettings.FARDebugButtonStock)
                        FARDebugAndSettings.FARDebugButtonStock.SetFalse(false);
                }
            }
            catch (Exception e)
            {
                Log.Warn("EditorWindow: closing FAR's window: " + e.Message);
            }
        }

        public static void SelectTab(string tabKey)
        {
            object modes = Get(Far.Editor, "modeDropdown");
            Remember(modes, "selectedOption");
            SelectOption(modes, L(tabKey));
        }

        /// <summary>A FAR dropdown's option chosen by its label, as a click in its list would.</summary>
        public static void SelectOption(object dropdown, string label)
        {
            var options = (string[])Get(dropdown, "stringOptions");
            int i = Array.FindIndex(options, o => string.Equals(o.Trim(), label, StringComparison.OrdinalIgnoreCase));
            if (i < 0)
                throw new UITestFailure($"no option '{label}' in FAR's dropdown ({string.Join(", ", options)})");
            Set(dropdown, "selectedOption", i);
        }

        /// <summary>The planet (home body) and flap setting (up) every tab shares.</summary>
        public static void ResetConditions()
        {
            object graphGui = Get(Far.Editor, "_editorGraph");
            object bodies = Get(graphGui, "bodySettingDropdown"), flaps = Get(graphGui, "flapSettingDropdown");
            Remember(bodies, "selectedOption");
            Remember(flaps, "selectedOption");
            SelectOption(bodies, FlightGlobals.GetHomeBody().bodyName);
            SelectOption(flaps, L("FARFlapSetting0"));
        }

        private static bool? viscousAtStart;

        /// <summary>The shared viscous option off (through its toggle on the tab shown), for FAR's default conditions.</summary>
        public static IEnumerator Inviscid()
        {
            if (!viscousAtStart.HasValue)
                viscousAtStart = EditorSimManager.UseAccurateViscousCalc;
            if (EditorSimManager.UseAccurateViscousCalc)
                yield return Imgui.Toggle(ViscousToggle, false, Window);
            Assert.False(EditorSimManager.UseAccurateViscousCalc, "the viscous option is off");
        }

        // ------------------------------------------------------------------------------------------ remembered fields

        private static readonly List<KeyValuePair<object, Dictionary<string, object>>> Remembered =
            new List<KeyValuePair<object, Dictionary<string, object>>>();

        /// <summary>Fields' values kept (the first time in a test) to put back when the window closes.</summary>
        public static void Remember(object o, params string[] fields)
        {
            Dictionary<string, object> saved = null;
            foreach (KeyValuePair<object, Dictionary<string, object>> e in Remembered)
                if (ReferenceEquals(e.Key, o))
                    saved = e.Value;
            if (saved == null)
            {
                saved = new Dictionary<string, object>();
                Remembered.Add(new KeyValuePair<object, Dictionary<string, object>>(o, saved));
            }

            foreach (string f in fields)
                if (!saved.ContainsKey(f))
                {
                    object v = Get(o, f);
                    saved[f] = v is Array a ? a.Clone() : v;
                }
        }

        private static void RestoreRemembered()
        {
            foreach (KeyValuePair<object, Dictionary<string, object>> e in Remembered)
                foreach (KeyValuePair<string, object> f in e.Value)
                    try
                    {
                        // an array field (readonly) gets its elements back
                        if (f.Value is Array saved && Get(e.Key, f.Key) is Array current && current.Length == saved.Length)
                            Array.Copy(saved, current, saved.Length);
                        else
                            Set(e.Key, f.Key, f.Value);
                    }
                    catch (Exception ex)
                    {
                        Log.Warn($"EditorWindow: restoring {e.Key.GetType().Name}.{f.Key}: {ex.Message}");
                    }

            Remembered.Clear();
        }

        // ------------------------------------------------------------------------------------------ Heatmaps tab

        public sealed class SweepOut
        {
            public List<HeatmapResult> Results;
            public string Status, CachePath;
            public int CellsDone, CellsTotal;
            public bool StabilityOnly;
        }

        public static HeatmapsGUI Heatmaps => (HeatmapsGUI)Get(Far.Editor, "_heatmaps");

        private static HeatmapsGUI heatmapsUsed;
        private static bool? cleanupAtStart;

        /// <summary>The Heatmaps tab's state remembered, spoilers in, the fuel slider at full.</summary>
        public static void PrepareHeatmaps(HeatmapsGUI hm)
        {
            heatmapsUsed = hm;
            if (!cleanupAtStart.HasValue)
                cleanupAtStart = HeatmapResult.CleanupDisplay;
            Remember(hm, "_machMin", "_machMax", "_altMinKm", "_altMaxKm", "_nMach", "_nAlt", "_spoilers", "_includeStability",
                     "_autoRecalc", "_speedInMS", "_autoMaxAlt", "_autoMaxSpeed", "_multithread", "_fuelPos");
            Set(hm, "_spoilers", false);   // no control for it on the tab
            Set(hm, "_fuelPos", 2f);       // the slider at full (the last of the 3 mass samples)
        }

        /// <summary>
        /// The x axis in Mach or m/s, auto-fit of both maxima on or off, no auto-recalc, cleanup on, stability and
        /// multithreading as asked, through the tab's controls.
        /// </summary>
        public static IEnumerator SetHeatmapToggles(bool stability, bool multithread, bool autoFit = false, bool speedInMS = false)
        {
            bool inMS = Get<bool>(Heatmaps, "_speedInMS");
            if (inMS != speedInMS)
                yield return Imgui.Click(inMS ? "m/s" : "Mach", Window, exact: true);
            Assert.Equal(speedInMS, Get<bool>(Heatmaps, "_speedInMS"), "the speed unit button");
            yield return Imgui.Toggle("Max altitude", autoFit, Window);
            yield return Imgui.Toggle(speedInMS ? "Max speed" : "Max Mach", autoFit, Window);
            yield return Imgui.Toggle(L("FARHeatmapsAutoRecalc"), false, Window);
            yield return Imgui.Toggle("Cleanup", true, Window);
            yield return Imgui.Toggle("Multithread", multithread, Window);
            yield return Imgui.Toggle(L("FARHeatmapsIncludeStab"), stability, Window);
        }

        /// <summary>
        /// A sweep through the Heatmaps tab: its range fields typed in (Mach min, max, altitude min, max km, cells along
        /// each), Run pressed, the sweep waited out. Records the history file it saved, for the test to delete.
        /// </summary>
        public static IEnumerator Sweep(string[] fields, bool stability, bool multithread, SweepOut o, bool autoFit = false,
                                        bool speedInMS = false)
        {
            SelectTab("FAREditorModeHeatmaps");
            yield return Wait.Frames(2);
            HeatmapsGUI hm = Heatmaps;
            PrepareHeatmaps(hm);
            yield return SetHeatmapToggles(stability, multithread, autoFit, speedInMS);
            for (int i = 0; i < fields.Length; i++)
                yield return Imgui.SetTextNth(i, fields[i], Window);
            Assert.Equal(multithread, Get<bool>(hm, "_multithread"), "the Multithread toggle");
            Assert.Equal(stability, Get<bool>(hm, "_includeStability"), "the Include stability toggle");

            object before = Get(hm, "_results");
            yield return Imgui.Click(L("FARStabHeatmapRun"), Window);
            // an auto-fit probe hands over to the sweep it framed in the same frame it finishes
            yield return Wait.Until(() => !hm.IsSweepRunning && !Get<bool>(hm, "_autoFitProbe"), 900, "the heatmap sweep", shot: false);

            o.Results = Get<List<HeatmapResult>>(hm, "_results");
            o.Status = Get<string>(hm, "_statusMessage");
            o.CellsDone = Get<int>(hm, "_cellsDone");
            o.CellsTotal = Get<int>(hm, "_cellsTotal");
            o.StabilityOnly = Get<bool>(hm, "_stabilityOnly");
            Assert.True(o.Results != null && o.Results.Count > 0 && !ReferenceEquals(o.Results, before),
                        "the Run button swept a new map: " + o.Status);

            var history = Get<List<HeatmapPersistence.CacheEntry>>(hm, "_cachedSweeps");
            int vi = Get<int>(hm, "_viewIndex");
            if (!o.StabilityOnly && history != null && vi >= 0 && vi < history.Count)
                o.CachePath = history[vi].Path;
        }

        /// <summary>A sweep's history file deleted (null: none).</summary>
        public static void DeleteCache(string path)
        {
            if (path == null)
                return;
            try
            {
                HeatmapPersistence.DeletePath(path);
            }
            catch (Exception e)
            {
                Log.Warn("EditorWindow: deleting the sweep's history file: " + e.Message);
            }
        }
    }
}
