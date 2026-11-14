using System;
using System.Collections;
using FerramAerospaceResearch.FARGUI;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using KSP.Localization;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// Screenshots of what FAR's debug tools (FARDebugValues.DebugTools) show and hide, each with the tools off and on:
    /// the flight window (its Debug button, its flight assistance), the flight settings window's group list and its
    /// flight assistance group, the debug window (the exposure renders), the toolbar's put-on-water button, and the
    /// Space Center's debug options window with the toggle. For the eye: they assert only that the windows came up.
    /// Run windowed (KSPTEST_KSP_ARGS empty) for the toolbar to be in them.
    /// </summary>
    public static class DebugToolsShots
    {
        private static string Loc(string tag) => Localizer.Format(tag);

        [UITest(Kind = "Measure", Explicit = true, StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway",
                TimeoutSec = 300)]
        public static IEnumerator Shots_DebugToolsFlight(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FlightGUI gui = Far.FlightGui();
            bool shown = Far.FlightGuiShown;
            bool settings = Far.SubWindowOpen(gui, "showSettingsWindow"), debug = Far.SubWindowOpen(gui, "showDebugWindow");
            int group = Far.SettingsGroup(gui);
            object main = Refl.GetStatic(typeof(FlightGUI), "mainGuiRect"), set = Refl.GetStatic(typeof(FlightGUI), "settingsGuiRect"),
                   dbg = Refl.GetStatic(typeof(FlightGUI), "debugGuiRect");
            using IDisposable tools = Far.DebugTools(false);
            try
            {
                foreach (bool on in new[] { false, true })
                {
                    string tag = on ? "on" : "off";
                    ctx.Step("debug tools " + tag);
                    FARDebugValues.DebugTools = on;
                    // side by side: the flight window, the settings window, the debug window
                    Refl.SetStatic(typeof(FlightGUI), "mainGuiRect", new Rect(20, 80, 230, 0));
                    Refl.SetStatic(typeof(FlightGUI), "settingsGuiRect", new Rect(270, 80, 260, 0));
                    Refl.SetStatic(typeof(FlightGUI), "debugGuiRect", new Rect(550, 80, 300, 0));
                    Far.FlightGuiShown = true;
                    Far.SetSubWindow(gui, "showSettingsWindow", true);
                    Far.SetSubWindow(gui, "showDebugWindow", on);
                    Far.SetSettingsGroup(gui, on ? Far.GroupStabilityAugmentation : Far.GroupAirspeed);
                    yield return Imgui.WaitWindow(Far.MainWindow, 30);
                    yield return Imgui.WaitWindow(Far.SettingsWindow);
                    yield return Wait.Frames(10);
                    ctx.Record("windows-" + tag, $"Debug button {Imgui.IsVisible(Loc("FARFlightGUIFltDebug"), Far.MainWindow)}, " +
                                                 $"Roll toggle {Imgui.IsVisible(Loc("FARFlightStabAugLabel0"), Far.MainWindow)}");
                    yield return Snap.Shot(ctx, "flight-windows-" + tag);

                    // the settings window's group list, open (it closes once the mouse is off it: its button's rect
                    // made the whole screen while it is shot)
                    object dropdown = Refl.Get(gui, "settingsWindow");
                    Refl.Set(dropdown, "toggleBtnState", true);
                    yield return Wait.Until(() => (bool)Refl.Get(dropdown, "isActive"), 5, "the group list open", shot: false);
                    Refl.Set(FARGUIDropDownDisplay.Instance, "btnRect", new Rect(-1e5f, -1e5f, 2e5f, 2e5f));
                    yield return Wait.Frames(5);
                    yield return Snap.Shot(ctx, "settings-groups-" + tag);
                    Refl.Set(dropdown, "toggleBtnState", false);
                    yield return Wait.Frames(5);
                }
            }
            finally
            {
                Refl.SetStatic(typeof(FlightGUI), "mainGuiRect", main);
                Refl.SetStatic(typeof(FlightGUI), "settingsGuiRect", set);
                Refl.SetStatic(typeof(FlightGUI), "debugGuiRect", dbg);
                Far.SetSubWindow(gui, "showSettingsWindow", settings);
                Far.SetSubWindow(gui, "showDebugWindow", debug);
                Far.FlightGuiShown = shown;
            }

            // after the tools' scope: the group can be flight assistance's only with them on
            Far.SetSettingsGroup(gui, group == Far.GroupStabilityAugmentation ? Far.GroupFlightData : group);
        }

        [UITest(Kind = "Measure", Explicit = true, StartsAt = StartState.SpaceCenter, TimeoutSec = 300)]
        public static IEnumerator Shots_DebugToolsSpaceCenter(TestContext ctx)
        {
            const string window = "FAR Debug Options";
            using IDisposable tools = Far.DebugTools(false);
            bool shown = Far.FlightGuiShown, editor = EditorWindow.WindowShown;
            FARDebugAndSettings.Toggle();
            try
            {
                yield return Imgui.WaitWindow(window, 30);
                foreach (bool on in new[] { false, true })
                {
                    ctx.Step("debug tools " + (on ? "on" : "off"));
                    FARDebugValues.DebugTools = on;
                    yield return Wait.Frames(10);
                    yield return Snap.Shot(ctx, "debug-options-" + (on ? "on" : "off"));
                }
            }
            finally
            {
                FARDebugAndSettings.ForceCloseDebugWindow();
                EditorWindow.WindowShown = editor;
                Far.FlightGuiShown = shown;
            }
        }
    }
}
