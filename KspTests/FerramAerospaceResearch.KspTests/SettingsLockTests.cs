using System;
using System.Collections;
using System.Linq;
using System.Reflection;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using KspTest;
using UnityEngine;
using static FerramAerospaceResearch.KspTests.Refl;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's debug window and model options window keep clicks on them from going through to the game under them:
    /// with the (synthetic) mouse over either, the scene under it is locked; off it, or with the window closed, it is
    /// unlocked. The model options window's whole-wing aspect ratio recomputes the editor's Components tab.
    /// </summary>
    public static class SettingsLockTests
    {
        private const string DebugWindow = "FAR Debug Options", ModelWindow = "FAR Model Options";
        private const string DebugLock = "FARDebugLock", ModelLock = "FARModelOptionsLock";

        private static bool Locked(string id) => InputLockManager.lockStack.ContainsKey(id);

        private static bool DebugMenuOpen => (bool)GetStatic(typeof(FARDebugAndSettings), "debugMenu");

        private static bool ModelOptionsOpen => (bool)GetStatic(typeof(FARDebugAndSettings), "modelOptions");

        private static FARDebugAndSettings DebugInstance =>
            UnityEngine.Object.FindObjectOfType<FARDebugAndSettings>() ?? throw new UITestFailure("FAR's FARDebugAndSettings is not there");

        // left of both windows (the debug window from x 50, the model options from x 80) on any screen: on Linux
        // batchmode's 640x480 the 700-wide debug window reaches past the right edge and down to the bottom
        private static Vector2 Away => new Vector2(3, 3);

        /// <summary>
        /// At the Space Center: FAR's stock toolbar button opens the debug window and closes it again. The window
        /// locks the Space Center while the mouse is over it, unlocks it when the mouse leaves, and unlocks it when
        /// closed under the mouse.
        /// The model options window (the button's right click) does the same, closed with its Close button.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.SpaceCenter, TimeoutSec = 600)]
        public static IEnumerator Settings_DebugAndModelOptionsLockTheSpaceCenter(TestContext ctx)
        {
            bool editorShown = EditorWindow.WindowShown, flightShown = Far.FlightGuiShown;
            object tab = Get(DebugInstance, "activeTab");
            try
            {
                Imgui.PointerFeeds(typeof(FARGUI.GUIUtils).GetMethod("GetMousePos", BindingFlags.Public | BindingFlags.Static));
                yield return Imgui.Hover(Away);

                ctx.Step("The debug window, from FAR's stock toolbar button");
                KSP.UI.Screens.ApplicationLauncherButton button = FARDebugAndSettings.FARDebugButtonStock;
                Assert.True(button != null, "FAR's button on the stock toolbar");
                if (DebugMenuOpen)
                    FARDebugAndSettings.ForceCloseDebugWindow();
                // pressed as a click presses it: its toggle on, with its callback
                button.SetTrue(true);
                Assert.True(DebugMenuOpen, "the button opened the debug window");

                yield return Imgui.WaitWindow(DebugWindow);
                Assert.False(Locked(DebugLock), "the Space Center not locked with the mouse away");
                Rect at = Get<Rect>(DebugInstance, "debugWinPos");
                yield return Imgui.Hover(at.center);
                yield return Wait.Until(() => Locked(DebugLock), 10, "the Space Center locked under the debug window", shot: false);
                Assert.Equal((ulong)ControlTypes.KSC_ALL, (ulong)InputLockManager.lockStack[DebugLock], "the lock is on the Space Center's controls");
                yield return Imgui.Hover(Away);
                yield return Wait.Until(() => !Locked(DebugLock), 10, "unlocked with the mouse off the window", shot: false);
                yield return Imgui.Hover(at.center);
                yield return Wait.Until(() => Locked(DebugLock), 10, "locked again", shot: false);
                button.SetFalse(true);
                Assert.False(DebugMenuOpen, "the button pressed again closed the debug window");
                yield return Wait.Until(() => !Locked(DebugLock), 10, "unlocked when the window closes under the mouse", shot: false);

                ctx.Step("The model options window");
                yield return Imgui.Hover(Away);
                if (!ModelOptionsOpen)
                    FARDebugAndSettings.ToggleModelOptions();
                yield return Imgui.WaitWindow(ModelWindow);
                Assert.False(Locked(ModelLock), "not locked with the mouse away");
                Rect model = (Rect)GetStatic(typeof(FARDebugAndSettings), "modelWinPos");
                yield return Imgui.Hover(model.center);
                yield return Wait.Until(() => Locked(ModelLock), 10, "the Space Center locked under the model options", shot: false);
                Assert.Equal((ulong)(ControlTypes.KSC_ALL | ControlTypes.CAMERACONTROLS), (ulong)InputLockManager.lockStack[ModelLock], "the lock is on the Space Center and its camera");
                yield return Imgui.Hover(Away);
                yield return Wait.Until(() => !Locked(ModelLock), 10, "unlocked with the mouse off the window", shot: false);
                yield return Imgui.Hover(model.center);
                yield return Wait.Until(() => Locked(ModelLock), 10, "locked again", shot: false);
                yield return Imgui.Click("Close", ModelWindow, exact: true);
                Assert.False(ModelOptionsOpen, "the Close button closed the window");
                yield return Wait.Until(() => !Locked(ModelLock), 10, "unlocked when the window closes under the mouse", shot: false);
            }
            finally
            {
                Imgui.ReleasePointer();
                if (DebugMenuOpen)
                    FARDebugAndSettings.ForceCloseDebugWindow();
                if (ModelOptionsOpen)
                    FARDebugAndSettings.ToggleModelOptions();
                EditorWindow.WindowShown = editorShown;
                Far.FlightGuiShown = flightShown;
                Set(DebugInstance, "activeTab", tab);
            }
        }

        /// <summary>
        /// The debug window open at the Space Center with the mouse over it (the Space Center locked), and the game
        /// leaves for the main menu: the window closes and its lock goes with it.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.SpaceCenter, TimeoutSec = 600)]
        public static IEnumerator Settings_DebugLockGoesWithTheSpaceCenter(TestContext ctx)
        {
            try
            {
                Imgui.PointerFeeds(typeof(FARGUI.GUIUtils).GetMethod("GetMousePos", BindingFlags.Public | BindingFlags.Static));
                yield return Imgui.Hover(Away);
                if (!DebugMenuOpen)
                    FARDebugAndSettings.FARDebugButtonStock.SetTrue(true);
                yield return Imgui.WaitWindow(DebugWindow);
                Rect at = Get<Rect>(DebugInstance, "debugWinPos");
                yield return Imgui.Hover(at.center);
                yield return Wait.Until(() => Locked(DebugLock), 10, "the Space Center locked under the debug window", shot: false);

                ctx.Step("To the main menu, the mouse still over the window");
                yield return Nav.ToMainMenu(ctx);
                yield return Wait.Frames(5);
                Assert.False(DebugMenuOpen, "the debug window closed away from the Space Center");
                Assert.False(Get<bool>(DebugInstance, "inputLocked"), "FAR holds no lock any more");
                Assert.False(Locked(DebugLock), "and the lock is gone");
            }
            finally
            {
                Imgui.ReleasePointer();
                if (DebugMenuOpen)
                    FARDebugAndSettings.ForceCloseDebugWindow();
            }
        }

        /// <summary>
        /// In the editor: the model options window locks the editor while the mouse is over it and unlocks it off it.
        /// Its whole-wing aspect ratio switched off has the Components tab recompute with the panels' aspect ratios
        /// changed; switched on again, they are what they were.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 600)]
        public static IEnumerator Settings_ModelOptionsLockTheEditorAndRecomputeComponents(TestContext ctx)
        {
            bool whole = FARDebugValues.wholeWingAspectRatio;
            try
            {
                Imgui.PointerFeeds(typeof(FARGUI.GUIUtils).GetMethod("GetMousePos", BindingFlags.Public | BindingFlags.Static));
                yield return Imgui.Hover(Away);
                yield return EditorWindow.OpenWindow("FAREditorModeComponents");
                object comp = Get(Far.Editor, "_components");
                yield return Wait.Until(() => !Get<bool>(comp, "dirty"), 15, "the Components tab computed", shot: false);
                if (!whole)
                {
                    FARDebugValues.wholeWingAspectRatio = true;
                    ferram4.FARWingInteraction.InvalidateWingGroups();
                    FARDebugValues.OnModelChanged();
                    yield return Far.Revoxelize("whole-wing aspect ratio on");
                }

                ctx.Step("The model options window over the editor");
                if (!ModelOptionsOpen)
                    FARDebugAndSettings.ToggleModelOptions();
                yield return Imgui.WaitWindow(ModelWindow);
                Rect model = (Rect)GetStatic(typeof(FARDebugAndSettings), "modelWinPos");
                yield return Imgui.Hover(model.center);
                yield return Wait.Until(() => Locked(ModelLock), 10, "the editor locked under the model options", shot: false);
                yield return Imgui.Hover(Away);
                yield return Wait.Until(() => !Locked(ModelLock), 10, "unlocked with the mouse off the window", shot: false);

                ctx.Step("Whole-wing aspect ratio off, then on");
                yield return Wait.Until(() => !Get<bool>(comp, "dirty"), 15, "the Components tab computed", shot: false);
                // each panel's aspect ratio as the tab shows it, with whole-wing aspect ratio on
                var on = Rows(comp).ToDictionary(r => Get<Part>(r, "Part"), r => Get<double>(r, "EffectiveAR"));
                bool Same(object r) => on.TryGetValue(Get<Part>(r, "Part"), out double ar) && Math.Abs(Get<double>(r, "EffectiveAR") - ar) < 1e-6;
                yield return Imgui.Toggle("Whole-wing aspect ratio", false, ModelWindow);
                Assert.False(FARDebugValues.wholeWingAspectRatio, "the option off");
                yield return Wait.Until(() => !Get<bool>(comp, "dirty") && !Rows(comp).All(Same), 30,
                                        "the Components tab recomputed: the panels' aspect ratios changed", shot: false);
                ctx.Record("aspectRatios", string.Join(", ", Rows(comp).Select(r => $"{Get<double>(r, "EffectiveAR"):F2} (on {on[Get<Part>(r, "Part")]:F2}, own {Get<double>(r, "OwnAR"):F2})")));
                yield return Imgui.Toggle("Whole-wing aspect ratio", true, ModelWindow);
                yield return Wait.Until(() => !Get<bool>(comp, "dirty") && Rows(comp).All(Same), 30,
                                        "the Components tab recomputed: the panels' aspect ratios back", shot: false);
            }
            finally
            {
                Imgui.ReleasePointer();
                if (ModelOptionsOpen)
                    FARDebugAndSettings.ToggleModelOptions();
                if (FARDebugValues.wholeWingAspectRatio != whole)
                {
                    FARDebugValues.wholeWingAspectRatio = whole;
                    ferram4.FARWingInteraction.InvalidateWingGroups();
                    FARDebugValues.OnModelChanged();
                }

                EditorWindow.CloseWindow();
            }
        }

        private static System.Collections.Generic.IEnumerable<object> Rows(object comp) =>
            ((IList)Get(comp, "rows")).Cast<object>().Where(r => Get(r, "Wing") != null);
    }
}
