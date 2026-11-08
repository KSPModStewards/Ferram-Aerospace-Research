using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using ferram4;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.Resources;
using KspTest;
using UnityEngine;
using static FerramAerospaceResearch.KspTests.EditorWindow;
using Object = UnityEngine.Object;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR with the optional mods it talks to, in an instance of their own (InteropMods): Blizzy's toolbar (FAR's
    /// buttons in it when its setting says so, and the wrapper FAR drives it through), TweakScale (a rescaled wing's
    /// geometry and wing model), KSPWheel (its gears raised and lowered by FAR's editor gear button) and the full
    /// RealChute (a chute's size change reaching FAR's geometry). None of these mods is referenced: what the test reads
    /// of them it reads by reflection, so the assembly loads in every instance.
    ///
    /// With RealChute in, the stock chutes are RealChute's (FAR's RealChuteLite stands down), so the chute tests
    /// (ParachuteTests, ChuteStateTests) do not run in this instance; nor does anything else but these tests.
    /// </summary>
    public static class InteropTests
    {
        // CKAN identifiers: Toolbar (blizzy78's, now Toolbar Continued; brings ClickThroughBlocker and ToolbarController),
        // TweakScaleRescaled (TweakScale for KSP 1.12, which provides "TweakScale"), KerbalFoundriesContinued (KSPWheel
        // gears; brings KSPWheel and TexturesUnlimited), RealChute
        private const string InteropMods = "Toolbar;TweakScaleRescaled;KerbalFoundriesContinued;RealChute";

        // a Mk1 pod with RealChute's stack chute on top and KSPWheel's small adjustable gear and aero leg on its sides (crafts/)
        private const string Lander = "FAR Interop Lander";

        private const string FarNamespace = "FerramAerospaceResearch";
        private const string EditorButtonId = FarNamespace + ".FAREditorButtonBlizzy";
        private const string FlightButtonId = FarNamespace + ".FARFlightButtonBlizzy";
        private const string DebugButtonId = FarNamespace + ".FARDebugButtonBlizzy";

        // ------------------------------------------------------------------------------------------ Blizzy's toolbar

        /// <summary>
        /// FAR's setting to use Blizzy's toolbar turned on in its debug window at the Space Center: FAR's debug button is
        /// in the toolbar at once; in the SPH its editor button ("FAR Editor", FAR's small icon), whose left click shows
        /// and hides FAR's editor window and right click its model options, and the debug button is gone; in flight its
        /// flight button ("FAR Flight Sys"), the same for the flight window, and the editor button gone; back at the
        /// Space Center the flight button gone. The clicks are the toolbar's own (its button's click, in an OnGUI pass,
        /// with the mouse button given), through FAR's wrapper to FAR's handlers.
        /// </summary>
        [UITest(Kind = "Check", Requires = "aaa_Toolbar", Mods = InteropMods, StartsAt = StartState.SpaceCenter, TimeoutSec = 1200)]
        public static IEnumerator Toolbar_FarButtonsInBlizzysToolbar(TestContext ctx)
        {
            Assert.True(ToolbarManager.ToolbarAvailable, "FAR sees Blizzy's toolbar");
            bool settingAtStart = FARDebugValues.useBlizzyToolbar;
            bool editorShown = WindowShown, flightShown = Far.FlightGuiShown, modelOptions = ModelOptions;
            using var log = new ExceptionLog();
            try
            {
                ctx.Step("The setting on in FAR's debug window");
                yield return SetToolbarSetting(true);
                object debugButton = DebugButton();
                ctx.Record("debugButton", Describe(debugButton));
                Assert.NotNull(debugButton, $"FAR's debug button in the toolbar ({Ids()})");

                ctx.Step("Into the SPH with the Aeris 4A");
                yield return Nav.EnterEditor(ctx, EditorFacility.SPH, ctx.CraftPath(Far.Plane, "SPH"));
                yield return Far.WaitForEditorVoxels();
                Assert.True(DebugButton() == null, $"the debug button out of the toolbar in the editor ({Ids()})");
                object editorButton = Command(EditorButtonId);
                Assert.NotNull(editorButton, $"FAR's editor button in the toolbar ({Ids()})");
                ctx.Record("editorButton", Describe(editorButton));
                Assert.Equal("FAR Editor", (string)Reflect.Get(editorButton, "ToolTip"), "the editor button's tooltip");
                Assert.Equal(FARAssets.Instance.Textures.IconSmall.Url, (string)Reflect.Get(editorButton, "TexturePath"),
                             "the editor button's icon");

                ctx.Step("The editor button clicked");
                yield return ClickInToolbar(editorButton, 0);
                Assert.Equal(!editorShown, WindowShown, "FAR's editor window after a left click");
                if (WindowShown)
                    yield return Imgui.WaitWindow(Window);
                yield return ClickInToolbar(editorButton, 0);
                Assert.Equal(editorShown, WindowShown, "FAR's editor window after a second left click");
                yield return ClickInToolbar(editorButton, 1);
                Assert.Equal(!modelOptions, ModelOptions, "the model options after a right click");
                yield return ClickInToolbar(editorButton, 1);
                Assert.Equal(modelOptions, ModelOptions, "the model options after a second right click");

                ctx.Step("Launched to the runway");
                EditorLogic.fetch.launchSiteName = "Runway";
                yield return Nav.Launch(ctx);
                yield return Wait.Until(() => Command(FlightButtonId) != null, 30, "FAR's flight button in the toolbar", shot: false, detail: Ids);
                Assert.True(Command(EditorButtonId) == null, $"the editor button out of the toolbar in flight ({Ids()})");
                Assert.True(Get(typeof(EditorGUI), "blizzyEditorGUIButton") == null, "FAR's editor button let go");
                object flightButton = Command(FlightButtonId);
                ctx.Record("flightButton", Describe(flightButton));
                Assert.Equal("FAR Flight Sys", (string)Reflect.Get(flightButton, "ToolTip"), "the flight button's tooltip");

                ctx.Step("The flight button clicked");
                bool shown = Far.FlightGuiShown;
                yield return ClickInToolbar(flightButton, 0);
                Assert.Equal(!shown, Far.FlightGuiShown, "FAR's flight window after a left click");
                yield return ClickInToolbar(flightButton, 0);
                Assert.Equal(shown, Far.FlightGuiShown, "FAR's flight window after a second left click");
                yield return ClickInToolbar(flightButton, 1);
                Assert.Equal(!modelOptions, ModelOptions, "the model options after a right click in flight");
                yield return ClickInToolbar(flightButton, 1);
                Assert.Equal(modelOptions, ModelOptions, "the model options after a second right click in flight");

                ctx.Step("Back to the Space Center");
                yield return Nav.ToSpaceCenter(ctx);
                Assert.True(Command(FlightButtonId) == null, $"the flight button out of the toolbar at the Space Center ({Ids()})");
                Assert.True(Get(typeof(FlightGUI), "blizzyFlightGUIButton") == null, "FAR's flight button let go");

                ctx.Step("The setting off again");
                yield return SetToolbarSetting(false);
                Assert.True(DebugButton() == null, $"the debug button out of the toolbar with the setting off ({Ids()})");
                log.AssertNone("with FAR's buttons in Blizzy's toolbar");
            }
            finally
            {
                RestoreToolbarSetting(settingAtStart);
                ModelOptions = modelOptions;
                Far.FlightGuiShown = flightShown;
                WindowShown = editorShown;
            }
        }

        /// <summary>
        /// FAR's debug button in Blizzy's toolbar (its setting on) at the Space Center every time the game comes back
        /// to it from the editor, and gone in the editor; its click shows and hides FAR's debug window.
        /// </summary>
        /// <remarks>
        /// Fails: FARDebugAndSettings.OnSceneChange destroys the button on leaving the Space Center but keeps the
        /// destroyed button in FARDebugButtonBlizzy, and only makes a button when that is null, so once the Space Center
        /// has been left the button never comes back.
        /// </remarks>
        [UITest(Kind = "Check", Requires = "aaa_Toolbar", Mods = InteropMods, StartsAt = StartState.SpaceCenter, TimeoutSec = 900)]
        public static IEnumerator Toolbar_DebugButtonBackAtTheSpaceCenter(TestContext ctx)
        {
            Assert.True(ToolbarManager.ToolbarAvailable, "FAR sees Blizzy's toolbar");
            bool settingAtStart = FARDebugValues.useBlizzyToolbar;
            try
            {
                ctx.Step("The setting on in FAR's debug window");
                yield return SetToolbarSetting(true);
                Assert.NotNull(DebugButton(), $"FAR's debug button in the toolbar ({Ids()})");
                for (int visit = 1; visit <= 2; visit++)
                {
                    ctx.Step($"To the VAB and back ({visit})");
                    yield return Nav.EnterEditor(ctx, EditorFacility.VAB);
                    Assert.True(DebugButton() == null, $"the debug button out of the toolbar in the editor ({Ids()})");
                    yield return Nav.ToSpaceCenter(ctx);
                    object button = DebugButton();
                    ctx.Record("debugButton" + visit, $"{Describe(button)}; FAR holds a button: {Get(typeof(FARDebugAndSettings), "FARDebugButtonBlizzy") != null}");
                    Assert.True(FARDebugValues.useBlizzyToolbar, "FAR's toolbar setting kept through the scene changes");
                    Assert.NotNull(button, $"FAR's debug button in the toolbar back at the Space Center, visit {visit} ({Ids()})");

                    yield return ClickInToolbar(button, 0);
                    Assert.True(DebugMenuOpen, "FAR's debug window after a click on its button");
                    yield return ClickInToolbar(button, 0);
                    Assert.False(DebugMenuOpen, "FAR's debug window after a second click");
                }

                ctx.Step("The setting off again");
                yield return SetToolbarSetting(false);
            }
            finally
            {
                RestoreToolbarSetting(settingAtStart);
            }
        }

        /// <summary>
        /// FAR's wrapper of Blizzy's toolbar (ToolbarWrapper.cs) against the toolbar itself, on a button of its own made
        /// through it: each property it sets is the toolbar button's and reads back; a visibility of the scenes it is
        /// shown in (a GameScenesVisibility) makes the button visible in the editor and not for flight alone; the
        /// toolbar's mouse enter, leave and clicks reach the wrapper's handlers with the wrapper's button and the mouse
        /// button; a popup menu (a PopupMenuDrawable) takes an option and a separator, draws in an OnGUI pass and,
        /// set as the button's drawable, is what the toolbar updates; destroyed, the button leaves the toolbar. FAR's own
        /// buttons use part of this (Toolbar_FarButtonsInBlizzysToolbar); the rest is the wrapper's to keep working.
        /// </summary>
        [UITest(Kind = "Check", Requires = "aaa_Toolbar", Mods = InteropMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 600)]
        public static IEnumerator Toolbar_WrapperDrivesTheToolbar(TestContext ctx)
        {
            Assert.True(ToolbarManager.ToolbarAvailable, "FAR sees Blizzy's toolbar");
            const string id = "FARInteropTestButton";
            IButton button = null;
            PopupMenuDrawable menu = null;
            try
            {
                ctx.Step("A button made through FAR's wrapper");
                button = ToolbarManager.Instance.add(FarNamespace, id);
                object real = Command(FarNamespace + "." + id);
                Assert.NotNull(real, $"the button in the toolbar ({Ids()})");

                ctx.Step("Its properties");
                string icon = FARAssets.Instance.Textures.IconSmall.Url;
                button.Text = "FAR test";
                button.TextColor = Color.green;
                button.TexturePath = icon;
                button.ToolTip = "FAR test tooltip";
                Assert.Equal("FAR test", (string)Reflect.Get(real, "Text"), "the toolbar button's text");
                Assert.Equal("FAR test", button.Text, "the text read back");
                Assert.True(button.TextColor == Color.green && (Color)Reflect.Get(real, "TextColor") == Color.green, "the text colour");
                Assert.Equal(icon, (string)Reflect.Get(real, "TexturePath"), "the toolbar button's icon");
                Assert.Equal(icon, button.TexturePath, "the icon read back");
                Assert.Equal("FAR test tooltip", (string)Reflect.Get(real, "ToolTip"), "the toolbar button's tooltip");
                Assert.Equal("FAR test tooltip", button.ToolTip, "the tooltip read back");
                foreach (string flag in new[] { "Visible", "Enabled", "Important" })
                {
                    PropertyInfo p = typeof(IButton).GetProperty(flag);
                    bool was = (bool)p.GetValue(button, null);
                    p.SetValue(button, !was, null);
                    Assert.Equal(!was, (bool)Reflect.Get(real, flag), $"the toolbar button's {flag} set");
                    Assert.Equal(!was, (bool)p.GetValue(button, null), $"{flag} read back");
                    p.SetValue(button, was, null);
                    Assert.Equal(was, (bool)Reflect.Get(real, flag), $"the toolbar button's {flag} put back");
                }

                ctx.Step("Its visibility");
                Assert.True(button.Visible && button.EffectivelyVisible, "a visible button with an icon is shown");
                var inEditor = new GameScenesVisibility(GameScenes.EDITOR, GameScenes.FLIGHT);
                var inFlight = new GameScenesVisibility(GameScenes.FLIGHT);
                Assert.True(inEditor.Visible && !inFlight.Visible, "the scene visibilities in the editor");
                button.Visibility = inFlight;
                Assert.True(ReferenceEquals(inFlight, button.Visibility), "the visibility read back");
                Assert.False(button.EffectivelyVisible, "the button not shown in the editor with a flight visibility");
                button.Visibility = inEditor;
                Assert.True(button.EffectivelyVisible, "the button shown in the editor with an editor visibility");
                button.Visibility = null;
                Assert.True(button.EffectivelyVisible && Reflect.Get(real, "Visibility") == null, "the visibility taken off");

                ctx.Step("The toolbar's mouse and clicks");
                var events = new List<string>();
                button.OnMouseEnter += e => events.Add("enter" + (ReferenceEquals(e.button, button) ? "" : " (another button)"));
                button.OnMouseLeave += e => events.Add("leave" + (ReferenceEquals(e.button, button) ? "" : " (another button)"));
                button.OnClick += e => events.Add("click" + e.MouseButton + (ReferenceEquals(e.Button, button) ? "" : " (another button)"));
                Reflect.Call(real, "mouseEnter");
                Reflect.Call(real, "mouseLeave");
                yield return ClickInToolbar(real, 0);
                yield return ClickInToolbar(real, 1);
                ctx.Record("events", string.Join(", ", events));
                Assert.Equal("enter, leave, click0, click1", string.Join(", ", events), "what the button's handlers were told");

                ctx.Step("A popup menu");
                menu = new PopupMenuDrawable();
                IButton option = menu.AddOption("FAR option");
                menu.AddSeparator();
                Assert.Equal("FAR option", option.Text, "the menu option's text");
                Action onAny = () => { };
                menu.OnAnyOptionClicked += onAny;
                menu.OnAnyOptionClicked -= onAny;
                menu.Update();
                Vector2 size = Vector2.zero;
                PopupMenuDrawable m = menu;
                yield return InOnGui(() => size = m.Draw(new Vector2(100, 100)), "the menu drawn", passes: 6);
                ctx.Record("menuSize", size);
                Assert.True(size.x > 0 && size.y > 0, $"the menu drawn has a size ({size})");
                button.Drawable = menu;
                Assert.True(ReferenceEquals(menu, button.Drawable), "the drawable read back");
                object realDrawable = Reflect.Get(real, "Drawable");
                Assert.NotNull(realDrawable, "the toolbar button's drawable");
                Reflect.Call(realDrawable, "Update");
                yield return InOnGui(() => size = (Vector2)Reflect.Call(realDrawable, "Draw", new Vector2(100, 100)), "the toolbar's drawable drawn", passes: 6);
                Assert.True(size.x > 0 && size.y > 0, $"the toolbar's drawable is the menu ({size})");
                button.Drawable = null;
                Assert.True(button.Drawable == null && Reflect.Get(real, "Drawable") == null, "the drawable taken off");

                ctx.Step("The button destroyed");
                button.Destroy();
                button = null;
                Assert.True(Command(FarNamespace + "." + id) == null, $"the button out of the toolbar ({Ids()})");
            }
            finally
            {
                menu?.Destroy();
                button?.Destroy();
            }
        }

        // ------------------------------------------------------------------------------------------ TweakScale

        /// <summary>
        /// One of the Aeris 4A's delta wings doubled in size through TweakScale's scale in its part action window: FAR's
        /// wing model takes the new span and mean chord (twice the prefab's) and area (four times), and the wing mass
        /// FAR gives it follows the area; FAR's geometry of the part is rebuilt (a new mesh list) and spans twice as
        /// much; the craft's voxel model grows. Scaled back to 1, the wing is the prefab's again.
        /// </summary>
        [UITest(Kind = "Check", Requires = "Scale", Mods = InteropMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator TweakScale_RescaledWingScalesFarsModel(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            yield return Far.Settled("the Aeris 4A");
            Part part = EditorLogic.fetch.ship.parts.First(p => p.partInfo.name == "deltaWing");
            var wing = part.FindModuleImplementing<FARWingAerodynamicModel>();
            var geometry = part.FindModuleImplementing<GeometryPartModule>();
            PartModule tweakScale = part.Modules.Contains("TweakScale") ? part.Modules["TweakScale"] : null;
            Assert.NotNull(wing, "the delta wing is a FAR wing");
            Assert.NotNull(geometry, "the delta wing has FAR's geometry");
            Assert.NotNull(tweakScale, "TweakScale on the delta wing");
            using var log = new ExceptionLog();
            double b2 = wing.b_2, mac = wing.MAC, area = wing.S, vol = Far.Volume(Far.EditorVehicle);
            float wingMass = wing.curWingMass;
            Vector3 meshSize = geometry.localMeshBounds.size;
            ctx.Record("before", $"span {wing.b_2_actual:F4} m (prefab {b2:F4}), MAC {wing.MAC_actual:F4} m (prefab {mac:F4}), " +
                                 $"S {area:F4} m2, wing mass {wingMass:F4} t, mesh {meshSize}, craft volume {vol:F3} m3");
            Assert.Near(b2, wing.b_2_actual, 1e-6, "the unscaled wing's span is the prefab's");
            try
            {
                ctx.Step("Scaled to 2 through its part action window");
                List<GeometryMesh> meshes = geometry.meshDataList;
                yield return SetTweakScale(part, tweakScale, 2);
                float factor = ((float)Reflect.Get(tweakScale, "currentScaleFactor"));
                ctx.Record("factor", factor);
                Assert.Near(2, factor, 1e-3, "TweakScale's scale factor");
                yield return Wait.Until(() => geometry.Ready && !ReferenceEquals(meshes, geometry.meshDataList), 30,
                                        "FAR's geometry of the wing rebuilt", shot: false);
                yield return Far.Revoxelize("the Aeris 4A with a wing scaled");
                Vector3 scaledMesh = geometry.localMeshBounds.size;
                double scaledVol = Far.Volume(Far.EditorVehicle);
                ctx.Record("scaled", $"span {wing.b_2_actual:F4} m, MAC {wing.MAC_actual:F4} m, S {wing.S:F4} m2, " +
                                     $"wing mass {wing.curWingMass:F4} t, mesh {scaledMesh}, craft volume {scaledVol:F3} m3");
                Assert.Near(factor * b2, wing.b_2_actual, 1e-4 * b2, "the scaled wing's span (m)");
                Assert.Near(factor * mac, wing.MAC_actual, 1e-4 * mac, "the scaled wing's mean chord (m)");
                Assert.Near(factor * factor * area, wing.S, 1e-3 * area, "the scaled wing's area (m2)");
                Assert.True(wing.curWingMass > 2 * wingMass, $"the wing mass follows the area ({wing.curWingMass:F4} t from {wingMass:F4})");
                Assert.Near(factor * meshSize.magnitude, scaledMesh.magnitude, 0.02 * factor * meshSize.magnitude,
                            $"FAR's geometry of the wing spans {factor} times as much ({scaledMesh} from {meshSize})");
                Assert.True(scaledVol > vol, $"the craft's voxel model grows ({scaledVol:F3} m3 from {vol:F3})");

                ctx.Step("Scaled back to 1");
                yield return SetTweakScale(part, tweakScale, 1);
                Assert.Near(1, ((float)Reflect.Get(tweakScale, "currentScaleFactor")), 1e-4, "TweakScale's scale factor back");
                Assert.Near(b2, wing.b_2_actual, 1e-4 * b2, "the span back to the prefab's (m)");
                Assert.Near(mac, wing.MAC_actual, 1e-4 * mac, "the mean chord back to the prefab's (m)");
                Assert.Near(area, wing.S, 1e-3 * area, "the area back (m2)");
                log.AssertNone("rescaling a wing with TweakScale");
            }
            finally
            {
                // the craft as it was, for the tests that follow in the editor (it keeps it), whatever happened to it here
                EditorLogic.LoadShipFromFile(ctx.Craft);
            }
        }

        // ------------------------------------------------------------------------------------------ KSPWheel

        /// <summary>
        /// FAR's editor gear button on KSPWheel gears (Kerbal Foundries' small adjustable landing gear, a
        /// KSPWheelAdjustableGear, and its aero leg, a KSPWheelDeployment): Raise Gear retracts both, through their
        /// gear actions (deployAction, toggleGearAction), and Lower Gear deploys them again.
        /// </summary>
        [UITest(Kind = "Check", Requires = "KSPWheel", Mods = InteropMods, StartsAt = StartState.Vab, Craft = Lander, TimeoutSec = 900)]
        public static IEnumerator KspWheel_FarsGearButtonRaisesAndLowersTheGear(TestContext ctx)
        {
            Part gear = EditorLogic.fetch.ship.parts.FirstOrDefault(p => p.Modules.Contains("KSPWheelAdjustableGear"));
            Part leg = EditorLogic.fetch.ship.parts.FirstOrDefault(p => p.Modules.Contains("KSPWheelDeployment"));
            Assert.NotNull(gear, "a KSPWheelAdjustableGear on the lander");
            Assert.NotNull(leg, "a KSPWheelDeployment on the lander");
            string State(Part p) => Reflect.Get(p.Modules["KSPWheelBase"], "wheelState").ToString();
            string States() => $"{gear.partInfo.name} {State(gear)}, {leg.partInfo.name} {State(leg)}";
            bool? gearAtStart = null;
            using var log = new ExceptionLog();
            try
            {
                yield return OpenWindow("FAREditorModeStatic");
                gearAtStart = Refl.Get<bool>(Far.Editor, "gearToggle");
                if (gearAtStart.Value)
                    yield return Imgui.Click(L("FARGearToggleLower"), Window);
                yield return Wait.Until(() => State(gear) == "DEPLOYED" && State(leg) == "DEPLOYED", 30, "the gear down", shot: false, detail: States);
                ctx.Record("atStart", States());

                ctx.Step("Raise Gear");
                yield return Imgui.Click(L("FARGearToggleRaise"), Window);
                Assert.True(Refl.Get<bool>(Far.Editor, "gearToggle"), "the window has the gear up");
                yield return Wait.Until(() => State(gear) == "RETRACTED" && State(leg) == "RETRACTED", 30, "the KSPWheel gear retracted", detail: States);
                ctx.Record("raised", States());

                ctx.Step("Lower Gear");
                yield return Imgui.Click(L("FARGearToggleLower"), Window);
                Assert.False(Refl.Get<bool>(Far.Editor, "gearToggle"), "the window has the gear down");
                yield return Wait.Until(() => State(gear) == "DEPLOYED" && State(leg) == "DEPLOYED", 30, "the KSPWheel gear deployed", detail: States);
                ctx.Record("lowered", States());
                log.AssertNone("raising and lowering KSPWheel gear");
            }
            finally
            {
                try
                {
                    if (gearAtStart.HasValue && EditorGUI.Instance != null && Refl.Get<bool>(Far.Editor, "gearToggle") != gearAtStart.Value)
                        Far.Editor.GetType().GetMethod("ToggleGear", Refl.Members)?.Invoke(Far.Editor, null);
                }
                catch (Exception e)
                {
                    Log.Warn("InteropTests: restoring the gear: " + e.Message);
                }

                CloseWindow();
            }
        }

        // ------------------------------------------------------------------------------------------ RealChute

        /// <summary>
        /// RealChute's stack chute on the lander made the next size up (1.25 m to 2.5 m, twice the size) with its
        /// parachute editor's Next size button: RealChute tells the part (RC_Rescale) and FAR rebuilds its geometry of
        /// the chute (a new mesh list), twice the size it was; the craft's voxel model grows.
        /// </summary>
        [UITest(Kind = "Check", Requires = "RealChute", Mods = InteropMods, StartsAt = StartState.Vab, Craft = Lander, TimeoutSec = 900)]
        public static IEnumerator RealChute_ResizedChuteRebuildsFarsGeometry(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            yield return Far.Settled("the lander");
            Part chute = EditorLogic.fetch.ship.parts.FirstOrDefault(p => p.partInfo.name == "RC.stack");
            Assert.NotNull(chute, "RealChute's stack chute on the lander");
            PartModule pChute = chute.Modules["ProceduralChute"];
            Assert.NotNull(pChute, "the chute's ProceduralChute");
            var geometry = chute.FindModuleImplementing<GeometryPartModule>();
            Assert.NotNull(geometry, "the chute has FAR's geometry");
            int sizeAtStart = ((int)Reflect.Get(pChute, "size"));
            Vector3 Scale() => chute.transform.GetChild(0).localScale;
            Vector3 scale = Scale(), meshSize = geometry.localMeshBounds.size;
            double vol = Far.Volume(Far.EditorVehicle);
            ctx.Record("before", $"size {sizeAtStart}, model scale {scale}, mesh {meshSize}, craft volume {vol:F3} m3");
            using var log = new ExceptionLog();
            const string editor = "RealChute Parachute Editor";
            try
            {
                ctx.Step("The parachute editor's Next size");
                yield return Paw.ClickEvent(chute, "Show Parachute Editor");
                yield return Imgui.WaitWindow(editor);
                List<GeometryMesh> meshes = geometry.meshDataList;
                yield return Imgui.Click("Next size", editor, exact: true);
                yield return Wait.Until(() => ((int)Reflect.Get(pChute, "lastSize")) == ((int)Reflect.Get(pChute, "size")) && Scale() != scale, 15,
                                        "RealChute to resize the chute", shot: false);
                float ratio = Scale().x / scale.x;
                ctx.Record("ratio", ratio);
                Assert.Near(2, ratio, 1e-3, "the chute's model twice the size");
                yield return Wait.Until(() => geometry.Ready && !ReferenceEquals(meshes, geometry.meshDataList), 30,
                                        "FAR's geometry of the chute rebuilt", shot: false);
                yield return Far.Revoxelize("the lander with a bigger chute");
                Vector3 resized = geometry.localMeshBounds.size;
                double resizedVol = Far.Volume(Far.EditorVehicle);
                ctx.Record("resized", $"model scale {Scale()}, mesh {resized}, craft volume {resizedVol:F3} m3");
                Assert.Near(ratio * meshSize.magnitude, resized.magnitude, 0.02 * ratio * meshSize.magnitude,
                            $"FAR's geometry of the chute {ratio} times the size ({resized} from {meshSize})");
                Assert.True(resizedVol > vol, $"the craft's voxel model grows ({resizedVol:F3} m3 from {vol:F3})");
                log.AssertNone("resizing a RealChute chute");
            }
            finally
            {
                try
                {
                    Reflect.Call(pChute, "HideEditor");
                }
                catch (Exception e)
                {
                    Log.Warn("InteropTests: closing RealChute's editor: " + e.Message);
                }

                // the craft as it was, for the tests that follow in the editor (it keeps it), whatever happened to it here
                EditorLogic.LoadShipFromFile(ctx.Craft);
            }
        }

        // ------------------------------------------------------------------------------------------ helpers

        private static object Get(Type t, string field) => Refl.GetStatic(t, field);

        private static void Set(Type t, string field, object value) => Refl.SetStatic(t, field, value);

        /// <summary>FAR's model options window shown (what its toolbar buttons' right click toggles).</summary>
        private static bool ModelOptions
        {
            get => (bool)Get(typeof(FARDebugAndSettings), "modelOptions");
            set => Set(typeof(FARDebugAndSettings), "modelOptions", value);
        }

        /// <summary>The toolbar's buttons (its Command objects).</summary>
        private static IEnumerable<object> Commands()
        {
            object manager = Reflect.FindType("Toolbar.ToolbarManager").GetField("InternalInstance", Refl.Members)?.GetValue(null);
            Assert.NotNull(manager, "Blizzy's toolbar running");
            return ((IEnumerable)Reflect.Get(manager, "Commands")).Cast<object>().ToList();
        }

        /// <summary>The toolbar's button of that full id (namespace.id), or null.</summary>
        private static object Command(string fullId) => Commands().FirstOrDefault(c => (string)Reflect.Get(c, "FullId") == fullId);

        private static string Ids() => string.Join(", ", Commands().Select(c => (string)Reflect.Get(c, "FullId")));

        private static string Describe(object command) =>
            command == null
                ? "none"
                : $"{Reflect.Get(command, "FullId")}: '{Reflect.Get(command, "ToolTip")}', {Reflect.Get(command, "TexturePath")}";

        /// <summary>The toolbar's click on its button, as its GUI makes it: in an OnGUI pass, with that mouse button.</summary>
        private static IEnumerator ClickInToolbar(object command, int mouseButton)
        {
            yield return InOnGui(() =>
            {
                Event e = Event.current;
                int was = e.button;
                e.button = mouseButton;
                try
                {
                    Reflect.Call(command, "click");
                }
                finally
                {
                    e.button = was;
                }
            }, $"a click ({mouseButton}) on {Reflect.Get(command, "FullId")}");
            yield return Wait.Frames(2);
        }

        /// <summary>
        /// Runs the action in OnGUI passes (where GUI calls and Event.current are): once, or in that many passes (a
        /// GUILayout window has its size only after its layout and repaint passes).
        /// </summary>
        private static IEnumerator InOnGui(Action action, string what, int passes = 1)
        {
            var go = new GameObject("FarInteropOnGui");
            var runner = go.AddComponent<OnGuiRunner>();
            runner.Action = action;
            runner.Passes = passes;
            try
            {
                yield return Wait.Until(() => runner.Done, 15, what, shot: false);
            }
            finally
            {
                Object.Destroy(go);
            }

            if (runner.Error != null)
                throw new UITestFailure($"{what}: {runner.Error.GetType().Name}: {runner.Error.Message}", runner.Error);
        }

        private sealed class OnGuiRunner : MonoBehaviour
        {
            public Action Action;
            public Exception Error;
            public int Passes = 1;
            public bool Done;

            private void OnGUI()
            {
                if (Done || Action == null)
                    return;
                try
                {
                    Action();
                }
                catch (Exception e)
                {
                    Error = e;
                }

                if (Error != null || --Passes <= 0)
                    Done = true;
            }
        }

        // ------------------------------------------------------------------------------------------ helpers: FAR's toolbar setting

        private const string DebugWindow = "FAR Debug Options";
        private const string ToolbarToggle = "Use Blizzy78 Toolbar instead of Stock AppManager";

        private static bool DebugMenuOpen => (bool)Get(typeof(FARDebugAndSettings), "debugMenu");

        /// <summary>
        /// FAR's debug button in the toolbar: the one the setting's toggle makes (namespace ferram4) or the one made on
        /// coming to the Space Center (namespace FerramAerospaceResearch).
        /// </summary>
        private static object DebugButton() => Command(DebugButtonId) ?? Command("ferram4.FARDebugButtonBlizzy");

        /// <summary>
        /// FAR's setting to use Blizzy's toolbar set through its toggle in FAR's debug window at the Space Center, and the
        /// game saved as the Space Center's buildings save it on the way out: FAR keeps the setting in its scenario
        /// module, which the next scene loads from the game.
        /// </summary>
        private static IEnumerator SetToolbarSetting(bool on)
        {
            Assert.Equal(GameScenes.SPACECENTER, HighLogic.LoadedScene, "the scene FAR's toolbar setting is set in");
            bool editorShown = WindowShown, flightShown = Far.FlightGuiShown;
            try
            {
                if (!DebugMenuOpen)
                    FARDebugAndSettings.Toggle();   // what FAR's stock button calls: the debug window's toggle (and the editor's and flight's)
                yield return Imgui.WaitWindow(DebugWindow);
                yield return Imgui.Toggle(ToolbarToggle, on, DebugWindow);
                Assert.Equal(on, FARDebugValues.useBlizzyToolbar, "FAR's toolbar setting after its toggle");
            }
            finally
            {
                FARDebugAndSettings.ForceCloseDebugWindow();
                WindowShown = editorShown;
                Far.FlightGuiShown = flightShown;
            }

            GamePersistence.SaveGame("persistent", HighLogic.SaveFolder, SaveMode.OVERWRITE);
            yield return Wait.Frames(2);
        }

        /// <summary>FAR's toolbar setting put back as it was and saved with the game, wherever the test stopped.</summary>
        private static void RestoreToolbarSetting(bool setting)
        {
            try
            {
                FARDebugValues.useBlizzyToolbar = setting;
                if (HighLogic.CurrentGame != null && HighLogic.LoadedScene is GameScenes.SPACECENTER or GameScenes.EDITOR or GameScenes.FLIGHT)
                    GamePersistence.SaveGame("persistent", HighLogic.SaveFolder, SaveMode.OVERWRITE);
            }
            catch (Exception e)
            {
                Log.Warn("InteropTests: restoring FAR's toolbar setting: " + e.Message);
            }
        }

        /// <summary>
        /// TweakScale's scale of the part set to that factor through its part action window, as its control sets it:
        /// the free scale's value (in its units, guiDefaultScale per 1) or the nearest of the stepped scale's sizes.
        /// </summary>
        private static IEnumerator SetTweakScale(Part part, PartModule tweakScale, float factor)
        {
            bool free = ((bool)Reflect.Get(tweakScale, "isFreeScale"));
            float unit = ((float)Reflect.Get(tweakScale, "guiDefaultScale"));
            string fieldName = free ? "guiScaleValue" : "guiScaleNameIndex";
            object value = factor * unit;
            if (!free)
            {
                var sizes = (float[])Reflect.Get(tweakScale, "ScaleFactors");
                int nearest = 0;
                for (int i = 1; i < sizes.Length; i++)
                    if (Math.Abs(sizes[i] - factor * unit) < Math.Abs(sizes[nearest] - factor * unit))
                        nearest = i;
                value = nearest;
            }

            yield return Paw.Open(part);
            UIPartActionFieldItem item = null;
            yield return Wait.Until(() => (item = FieldItem(part, tweakScale, fieldName)) != null, 15,
                                    $"TweakScale's {fieldName} in the part action window", shot: false, detail: () => Paw.Dump(part));
            MethodInfo set = item.GetType().GetMethods(Refl.Members)
                                 .FirstOrDefault(m => m.Name == "SetFieldValue" && m.GetParameters().Length == 1 &&
                                                      m.GetParameters()[0].ParameterType.IsInstanceOfType(value));
            Assert.NotNull(set, $"{item.GetType().Name}.SetFieldValue({value.GetType().Name})");
            Log.Info($"TweakScale {fieldName} -> {value} ({item.GetType().Name})");
            set.Invoke(item, new[] { value });
            yield return Wait.Frames(3);
            Paw.Close(part);
        }

        private static UIPartActionFieldItem FieldItem(Part part, PartModule module, string fieldName)
        {
            UIPartActionWindow w = Paw.Window(part);
            if (w == null)
                return null;
            foreach (UIPartActionItem i in w.ListItems)
                if (i is UIPartActionFieldItem f && f.Field != null && f.Field.name == fieldName && ReferenceEquals(f.PartModule, module))
                    return f;
            return null;
        }
    }
}
