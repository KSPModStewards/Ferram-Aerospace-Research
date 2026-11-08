using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.Geometry;
using FerramAerospaceResearch.Settings;
using KspTest;
using UnityEngine;
using static FerramAerospaceResearch.KspTests.Refl;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's settings: the Space Center debug window (FARDebugAndSettings: the Difficulty and Debug, Aerodynamic Failure
    /// and Atm Composition tabs, and the model options window of the toolbar button's right click), the per-save
    /// difficulty and voxel settings it shows (FARSettingsScenarioModule.DisplaySelection, the presets of
    /// FARDifficultyPresets.cfg) and how they survive a quicksave and quickload, and what the voxel and area-ruling
    /// settings do to the editor's voxel model.
    ///
    /// The window is opened with the handler FAR's toolbar button calls, and driven through its own toggles, fields and
    /// buttons; the preset drop-down's selection is set the way a click in its list sets it (the list closes itself
    /// unless the mouse is over it, which it never is in a headless run). Everything a test changes is put back when
    /// it ends: FAR's settings, the files a save writes (the test's .sfs, and any GameData/Custom_*.cfg FAR's config
    /// saver writes when a setting it keeps differs at a save).
    /// </summary>
    public static class SettingsTests
    {

        // the window's title is "FAR Debug Options, " + the version; the model options window's is fixed
        private const string DebugWindow = "FAR Debug Options";
        private const string ModelWindow = "FAR Model Options";

        // FARDebugAndSettings.MenuTab_str, in MenuTab order
        private const string TabDifficulty = "Difficulty and Debug", TabStress = "Aerodynamic Failure", TabAtm = "Atm Composition";

        // the save the quicksave/quickload tests write in the save folder (deleted after)
        private const string SaveName = "far-settings-test";

        // ------------------------------------------------------------------------------------------ Difficulty and Debug tab

        /// <summary>
        /// The Difficulty and Debug tab: its debug toggles (structural failures, failure explosions, torque arrows)
        /// switch FAR's FARDebugValues and back; a colour typed into the Cl line's red field becomes the Cl colour FAR's
        /// graphs use, the other components and lines untouched. The toolbar button's right click opens the model
        /// options window: its whole-wing aspect ratio toggle switches the option, tells the editor and flight views
        /// (FARDebugValues.ModelChanged) and explains the new setting; its Close button closes it.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.SpaceCenter, TimeoutSec = 600)]
        public static IEnumerator Settings_DebugTabTogglesAndColors(TestContext ctx)
        {
            bool failures = FARDebugValues.allowStructuralFailures;
            bool explosions = FARDebugValues.aeroFailureExplosions;
            bool arrows = FARDebugValues.showMomentArrows;
            bool whole = FARDebugValues.wholeWingAspectRatio;
            Color cl = FARConfig.GUIColors.ClColor, cd = FARConfig.GUIColors.CdColor;
            bool modelWindowAtStart = ModelOptionsOpen;
            int modelChanges = 0;
            Action onModelChanged = () => modelChanges++;
            FARDebugValues.ModelChanged += onModelChanged;
            try
            {
                ctx.Step("Open FAR's debug window at the Space Center");
                yield return OpenDebugWindow();
                yield return SelectTab(TabDifficulty, 0);

                ctx.Step("Flip the debug toggles, then flip them back");
                yield return Imgui.Toggle("Allow Aero-structural Failures", !failures, DebugWindow);
                Assert.Equal(!failures, FARDebugValues.allowStructuralFailures, "structural failures after their toggle");
                yield return Imgui.Toggle("Aero Failures Create Explosions", !explosions, DebugWindow);
                Assert.Equal(!explosions, FARDebugValues.aeroFailureExplosions, "failure explosions after their toggle");
                yield return Imgui.Toggle("Show Torque Arrows in Aero Overlay", !arrows, DebugWindow);
                Assert.Equal(!arrows, FARDebugValues.showMomentArrows, "torque arrows after their toggle");

                yield return Imgui.Toggle("Allow Aero-structural Failures", failures, DebugWindow);
                yield return Imgui.Toggle("Aero Failures Create Explosions", explosions, DebugWindow);
                yield return Imgui.Toggle("Show Torque Arrows in Aero Overlay", arrows, DebugWindow);
                Assert.Equal(failures, FARDebugValues.allowStructuralFailures, "structural failures toggled back");
                Assert.Equal(explosions, FARDebugValues.aeroFailureExplosions, "failure explosions toggled back");
                Assert.Equal(arrows, FARDebugValues.showMomentArrows, "torque arrows toggled back");

                ctx.Step("Type a new red for the Cl line");
                string red = cl.r > 0.5f ? "0.25" : "0.75";
                // the r, g and b fields follow the one label "Cl (r,g,b):" (their own labels are empty): r is the first
                yield return Imgui.SetTextAfterLabel("Cl (r,g,b):", red, DebugWindow);
                yield return Wait.Frames(2);
                Color now = FARConfig.GUIColors.ClColor;
                ctx.Record("ClColor", $"{cl} -> {now}");
                Assert.Near(double.Parse(red), now.r, 1e-5, "the Cl colour's red after typing it");
                Assert.Near(cl.g, now.g, 1e-5, "the Cl colour's green untouched");
                Assert.Near(cl.b, now.b, 1e-5, "the Cl colour's blue untouched");
                Assert.True(FARConfig.GUIColors.GetColor(0) == now, "the graphs' Cl colour (GUIColors.GetColor(0)) is the one typed");
                Assert.True(FARConfig.GUIColors.CdColor == cd, $"the Cd colour untouched ({FARConfig.GUIColors.CdColor} against {cd})");

                ctx.Step("The toolbar button's right click: the model options window");
                if (!ModelOptionsOpen)
                    FARDebugAndSettings.ToggleModelOptions();
                yield return Imgui.WaitWindow(ModelWindow);
                yield return Imgui.Toggle("Whole-wing aspect ratio", !whole, ModelWindow);
                Assert.Equal(!whole, FARDebugValues.wholeWingAspectRatio, "the whole-wing aspect ratio option after its toggle");
                Assert.True(modelChanges >= 1, $"the change was announced (FARDebugValues.ModelChanged fired {modelChanges} times)");
                yield return Imgui.WaitVisible(!whole ? "takes the aspect ratio of the whole wing" : "keeps its own aspect ratio",
                                               ModelWindow, 5);
                int changesBefore = modelChanges;
                yield return Imgui.Toggle("Whole-wing aspect ratio", whole, ModelWindow);
                Assert.Equal(whole, FARDebugValues.wholeWingAspectRatio, "the whole-wing aspect ratio option toggled back");
                Assert.True(modelChanges > changesBefore, "toggling it back was announced too");
                yield return Imgui.Click("Close", ModelWindow, exact: true);
                yield return Imgui.WaitWindowClosed(ModelWindow);
                Assert.False(ModelOptionsOpen, "the model options window closed by its Close button");
            }
            finally
            {
                FARDebugValues.ModelChanged -= onModelChanged;
                CloseDebugWindow();
                if (ModelOptionsOpen != modelWindowAtStart)
                    FARDebugAndSettings.ToggleModelOptions();
                FARDebugValues.allowStructuralFailures = failures;
                FARDebugValues.aeroFailureExplosions = explosions;
                FARDebugValues.showMomentArrows = arrows;
                FARConfig.GUIColors.ClColor = cl;
                FARConfig.GUIColors.CdColor = cd;
                if (FARDebugValues.wholeWingAspectRatio != whole)
                {
                    FARDebugValues.wholeWingAspectRatio = whole;
                    FARWingInteraction.InvalidateWingGroups();
                    FARDebugValues.OnModelChanged();
                }
            }
        }

        // ------------------------------------------------------------------------------------------ Aerodynamic Failure tab

        /// <summary>
        /// The Aerodynamic Failure tab: picking the rocketFuelTank template and typing its axial and lateral stress
        /// limits sets the template a stock liquid fuel and oxidizer tank gets (FARAeroStress.DetermineStressTemplate);
        /// "Requires Crew Compartment" takes the uncrewed tank off the template and back; "+" adds a 500/500 "default"
        /// template at the end of the list. Its resource lists take an entry each with their "+", named as typed, and
        /// lose it with its "-"; the template's own "-" removes it, and the pick moves to the one before.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.SpaceCenter, TimeoutSec = 600)]
        public static IEnumerator Settings_AeroStressTabEditsTemplates(TestContext ctx)
        {
            List<FARPartStressTemplate> templates = FARAeroStress.StressTemplates;
            int rocketIndex = templates.FindIndex(t => t.Name == "rocketFuelTank");
            Assert.True(rocketIndex >= 0, "FARAeroStress.cfg's rocketFuelTank template");
            FARPartStressTemplate rocket = templates[rocketIndex];
            double xz = rocket.XZMaxStress, y = rocket.YMaxStress;
            bool crew = rocket.RequiresCrew;
            int count = templates.Count;
            Part tank = PartLoader.LoadedPartsList.Select(ap => ap.partPrefab)
                                  .FirstOrDefault(p => p != null && p.CrewCapacity == 0 && Far.IsRocketTank(p) &&
                                                       ReferenceEquals(FARAeroStress.DetermineStressTemplate(p), rocket));
            Assert.NotNull(tank, "a stock liquid fuel and oxidizer tank on the rocketFuelTank template");
            ctx.Record("tank", tank.partInfo?.name);

            FARDebugAndSettings window = DebugInstance;
            object stressIndexAtStart = Get(window, "aeroStressIndex");
            FARPartStressTemplate added = null;
            try
            {
                ctx.Step("Open the debug window on Aerodynamic Failure, pick rocketFuelTank");
                yield return OpenDebugWindow();
                yield return SelectTab(TabStress, 1);
                yield return Imgui.Toggle("rocketFuelTank", true, DebugWindow, exact: true);
                Assert.Equal(rocketIndex, (int)Get(window, "aeroStressIndex"), "the template picked in the list");

                ctx.Step("Type its stress limits");
                string xzNew = xz > 40 && xz < 50 ? "55" : "45", yNew = y > 60 && y < 70 ? "75" : "65";
                yield return Imgui.SetTextAfterLabel("Lateral (X,Z-axis) Max Stress", xzNew, DebugWindow);
                yield return Imgui.SetTextAfterLabel("Axial (Y-axis) Max Stress", yNew, DebugWindow);
                Assert.Near(double.Parse(xzNew), rocket.XZMaxStress, 1e-9, "rocketFuelTank's lateral limit after typing it");
                Assert.Near(double.Parse(yNew), rocket.YMaxStress, 1e-9, "rocketFuelTank's axial limit after typing it");
                FARPartStressTemplate got = FARAeroStress.DetermineStressTemplate(tank);
                Assert.True(ReferenceEquals(rocket, got) && got.XZMaxStress == rocket.XZMaxStress,
                            $"the tank gets the edited template ({got.Name}, XZ {got.XZMaxStress})");
                Assert.Equal(count, templates.Count, "editing adds no template");

                ctx.Step("Requires Crew Compartment on, then off");
                yield return Imgui.Toggle("Requires Crew Compartment", true, DebugWindow);
                Assert.True(rocket.RequiresCrew, "rocketFuelTank needs crew after its toggle");
                FARPartStressTemplate uncrewed = FARAeroStress.DetermineStressTemplate(tank);
                ctx.Record("tankTemplateWhenCrewRequired", uncrewed.Name);
                Assert.False(ReferenceEquals(rocket, uncrewed), $"an uncrewed tank is off a template that needs crew (it gets {uncrewed.Name})");
                yield return Imgui.Toggle("Requires Crew Compartment", false, DebugWindow);
                Assert.False(rocket.RequiresCrew, "rocketFuelTank's crew requirement toggled back");
                Assert.True(ReferenceEquals(rocket, FARAeroStress.DetermineStressTemplate(tank)), "the tank is back on rocketFuelTank");

                ctx.Step("Add a template with +");
                // the template list's "+" is drawn before the resource lists' ones
                yield return Imgui.Click("+", DebugWindow, exact: true);
                Assert.Equal(count + 1, templates.Count, "templates after +");
                added = templates[count];
                Assert.Equal("default", added.Name, "the new template's name");
                Assert.True(added.XZMaxStress == 500 && added.YMaxStress == 500 && !added.IsSpecial && !added.RequiresCrew,
                            $"the new template: 500/500, not special, no crew (XZ {added.XZMaxStress}, Y {added.YMaxStress})");
                // a catch-all at the end of the list: what the tank gets now
                ctx.Record("tankTemplateWithNewDefault", FARAeroStress.DetermineStressTemplate(tank).Name);

                ctx.Step("The new template's resource lists: + adds an entry, typing names it, - takes it off");
                Set(window, "aeroStressIndex", count);
                yield return Wait.Frames(2);
                // the "+" buttons in order: the template list's, then the required and the excluded resources' lists'
                yield return Imgui.ClickNth("+", 1, DebugWindow);
                yield return Imgui.ClickNth("+", 2, DebugWindow);
                Assert.True(added.Resources.Resources.Count == 1 && added.Resources.Excluded.Count == 1,
                            $"an entry in each list ({added.Resources.Resources.Count} required, {added.Resources.Excluded.Count} excluded)");
                // the text fields: name, the two limits, the resource count, then the lists' entries
                yield return Imgui.SetTextNth(4, "LiquidFuel", DebugWindow);
                yield return Imgui.SetTextNth(5, "Ore", DebugWindow);
                Assert.Equal("LiquidFuel", added.Resources.Resources[0], "the required resource typed");
                Assert.Equal("Ore", added.Resources.Excluded[0], "the excluded resource typed");
                // the "-" buttons: one per template, then one per list entry
                yield return Imgui.ClickNth("-", count + 1, DebugWindow);
                Assert.True(added.Resources.Resources.Count == 0 && added.Resources.Excluded.Count == 1,
                            "the required resource's - takes it, and only it, off");
                yield return Imgui.ClickNth("-", count + 1, DebugWindow);
                Assert.Equal(0, added.Resources.Excluded.Count, "then the excluded one's");

                ctx.Step("Remove the new template, the one picked, with its -");
                yield return Imgui.ClickNth("-", count, DebugWindow);
                Assert.Equal(count, templates.Count, "templates after its -");
                Assert.False(templates.Contains(added), "the new template gone");
                Assert.Equal(count - 1, (int)Get(window, "aeroStressIndex"), "the pick moved to the template before it");
                added = null;
            }
            finally
            {
                CloseDebugWindow();
                if (added != null)
                    templates.Remove(added);
                rocket.XZMaxStress = xz;
                rocket.YMaxStress = y;
                rocket.RequiresCrew = crew;
                Set(window, "aeroStressIndex", stressIndexAtStart);
            }
        }

        // ------------------------------------------------------------------------------------------ Atm Composition tab

        /// <summary>
        /// The Atm Composition tab: the control surface time constants typed in are FARAeroData's (what the control
        /// surfaces read); the home body's viscosity and reference temperature typed in are its FAR atmosphere, no
        /// other body's; showing another body's data without typing leaves that body as it was, though the field shows
        /// it rounded to 5 decimals (GUIUtils.TextEntryForDouble parsed that text back every frame, so just showing the
        /// Sun's 1.7894e-5 made it 2e-5).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.SpaceCenter, TimeoutSec = 600)]
        public static IEnumerator Settings_AtmTabEditsBodyData(TestContext ctx)
        {
            CelestialBody home = FlightGlobals.GetHomeBody();
            var saved = new Dictionary<int, BodySettings>(FARAeroData.AtmosphericConfiguration);
            // the second body: one whose viscosity has more than 5 decimals (stock: Jool's 8.64e-6, Laythe's 1.7894e-5), if any
            List<CelestialBody> others = FlightGlobals.Bodies.Where(b => b.atmosphere && b != home &&
                                                                         saved.ContainsKey(b.flightGlobalsIndex)).ToList();
            CelestialBody other = others.FirstOrDefault(b => NotFiveDecimals(saved[b.flightGlobalsIndex].ReferenceViscosity)) ??
                                  others.FirstOrDefault();
            Assert.NotNull(other, "a second body with an atmosphere");
            double tc = FARControllableSurface.timeConstant;
            double tcFlap = FARControllableSurface.timeConstantFlap;
            double tcSpoiler = FARControllableSurface.timeConstantSpoiler;
            FARDebugAndSettings window = DebugInstance;
            object bodyIndexAtStart = Get(window, "atmBodyIndex");
            try
            {
                ctx.Step("Open the debug window on Atm Composition");
                yield return OpenDebugWindow();
                yield return SelectTab(TabAtm, 2);

                ctx.Step("Type the control surface time constants");
                yield return Imgui.SetTextAfterLabel("Ctrl Surf Time Constant", "0.4", DebugWindow);
                yield return Imgui.SetTextAfterLabel("Flap Time Constant", "12", DebugWindow);
                yield return Imgui.SetTextAfterLabel("Spoiler Time Constant", "0.9", DebugWindow);
                Assert.Near(0.4, FARAeroData.ControlSurfaceTimeConstant, 1e-9, "the control surface time constant");
                Assert.Near(12, FARAeroData.ControlSurfaceTimeConstantFlap, 1e-9, "the flap time constant");
                Assert.Near(0.9, FARAeroData.ControlSurfaceTimeConstantSpoiler, 1e-9, "the spoiler time constant");

                ctx.Step($"Pick {home.bodyName}, type its viscosity and reference temperature");
                yield return Imgui.Toggle(home.GetName(), true, DebugWindow, exact: true);
                Assert.Equal(FlightGlobals.Bodies.IndexOf(home), (int)Get(window, "atmBodyIndex"), "the body picked");
                yield return Imgui.SetTextAfterLabel("Gas Viscosity", "0.00003", DebugWindow);
                yield return Imgui.SetTextAfterLabel("Ref Temp for Viscosity", "300", DebugWindow);
                BodySettings h = FARAeroData.AtmosphericConfiguration[home.flightGlobalsIndex];
                ctx.Record("home", $"viscosity {h.ReferenceViscosity:G5} at {h.ReferenceTemperature} K");
                Assert.Near(3e-5, h.ReferenceViscosity, 1e-12, home.bodyName + "'s viscosity after typing it");
                Assert.Near(300, h.ReferenceTemperature, 1e-9, home.bodyName + "'s reference temperature after typing it");
                Assert.Equal(saved[home.flightGlobalsIndex].Name, h.Name, home.bodyName + "'s settings keep their name");
                BodySettings o = FARAeroData.AtmosphericConfiguration[other.flightGlobalsIndex];
                Assert.True(o.ReferenceViscosity != 3e-5 || saved[other.flightGlobalsIndex].ReferenceViscosity == 3e-5,
                            $"{other.bodyName}'s viscosity is not the one typed for {home.bodyName}");

                ctx.Step($"Show {other.bodyName}'s data without typing in it");
                yield return Imgui.Toggle(other.GetName(), true, DebugWindow, exact: true);
                yield return Wait.Frames(3);
                Assert.Equal(FlightGlobals.Bodies.IndexOf(other), (int)Get(window, "atmBodyIndex"), "the second body picked");
                Assert.Near(3e-5, FARAeroData.AtmosphericConfiguration[home.flightGlobalsIndex].ReferenceViscosity, 1e-12,
                            home.bodyName + "'s viscosity kept after picking another body");
                BodySettings before = saved[other.flightGlobalsIndex];
                BodySettings shown = FARAeroData.AtmosphericConfiguration[other.flightGlobalsIndex];
                ctx.Record("other", $"{other.bodyName}: viscosity {before.ReferenceViscosity:G6} -> {shown.ReferenceViscosity:G6}");
                Assert.Near(before.ReferenceTemperature, shown.ReferenceTemperature, 1e-9,
                            other.bodyName + "'s reference temperature, only shown");
                Assert.Equal(before.ReferenceViscosity, shown.ReferenceViscosity,
                             other.bodyName + "'s viscosity, only shown (the field shows it rounded to 5 decimals)");
            }
            finally
            {
                // closed first: drawing the tab again would rewrite the shown body's data
                CloseDebugWindow();
                foreach (KeyValuePair<int, BodySettings> kv in saved)
                    FARAeroData.AtmosphericConfiguration[kv.Key] = kv.Value;
                FARControllableSurface.timeConstant = tc;
                FARControllableSurface.timeConstantFlap = tcFlap;
                FARControllableSurface.timeConstantSpoiler = tcSpoiler;
                Set(window, "atmBodyIndex", bodyIndexAtStart);
            }
        }

        // ------------------------------------------------------------------------------------------ difficulty dialog + save

        /// <summary>
        /// The difficulty presets are FARDifficultyPresets.cfg's. Picking one in the dialog makes it FAR's settings
        /// (FARSettingsScenarioModule.Settings is that preset, and the dialog shows its name); the voxel counts and
        /// physics ticks typed in, and the torque arrows toggle, are FAR's. A quicksave writes them all into the save's
        /// FARSettingsScenarioModule node, and a quickload brings them back to a new module (the torque arrows from the
        /// file, not memory: they are flipped back before the load), and the dialog shows the preset again.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.SpaceCenter, TimeoutSec = 900)]
        public static IEnumerator Settings_DifficultyPresetSurvivesQuickload(TestContext ctx)
        {
            ctx.Step("The presets are FARDifficultyPresets.cfg's");
            List<FARDifficultyAndExactnessSettings> presets = FARSettingsScenarioModule.FARDifficultyPresetsConfig.Values;
            AssertPresetsFromConfig(ctx, presets);

            FARSettingsScenarioModule m = SettingsModule;
            ConfigNode saved = SnapshotScenario();
            FARDifficultyAndExactnessSettings customAtStart = Copy(m.customSettings);
            var files = new FileGuard();
            bool arrows = FARDebugValues.showMomentArrows;
            try
            {
                ctx.Step("Open the debug window; pick a preset in the difficulty dialog");
                yield return OpenDebugWindow();
                yield return SelectTab(TabDifficulty, 0);
                if (m.currentIndex < 0)
                {
                    yield return Imgui.Click("Switch Back To Presets", DebugWindow);
                    yield return Wait.Frames(3);
                    Assert.True(m.currentIndex >= 0, "the dialog back on its presets");
                }

                int target = m.currentIndex == presets.Count - 1 ? 0 : presets.Count - 1;
                FARDifficultyAndExactnessSettings preset = presets[target];
                ctx.Record("preset", $"{m.currentIndex} -> {target} ({preset.name})");
                Set(Get(m, "dropdown"), "selectedOption", target);
                yield return Wait.Frames(3);
                Assert.Equal(target, m.currentIndex, "the module's preset index after picking it");
                Assert.True(ReferenceEquals(preset, FARSettingsScenarioModule.Settings), "FAR's settings are the preset picked");
                yield return Imgui.WaitVisible(preset.name, DebugWindow, 5);

                ctx.Step("Type the voxel settings; flip the torque arrows");
                FARVoxelSettings vs = m.voxelSettings;
                int voxels = vs.numVoxelsControllableVessel + 50000, debris = vs.numVoxelsDebrisVessel + 1000;
                int ticks = vs.minPhysTicksPerUpdate + 7;
                yield return Imgui.SetTextAfterLabel("Voxels Controllable Vessel", voxels.ToString(), DebugWindow);
                yield return Imgui.SetTextAfterLabel("Voxels Debris", debris.ToString(), DebugWindow);
                yield return Imgui.SetTextAfterLabel("Min Phys Ticks per Voxel Update", ticks.ToString(), DebugWindow);
                Assert.Equal(voxels, FARSettingsScenarioModule.VoxelSettings.numVoxelsControllableVessel, "voxels for a controllable vessel");
                Assert.Equal(debris, FARSettingsScenarioModule.VoxelSettings.numVoxelsDebrisVessel, "voxels for debris");
                Assert.Equal(ticks, FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate, "physics ticks per voxel update");
                yield return Imgui.Toggle("Show Torque Arrows in Aero Overlay", !arrows, DebugWindow);
                Assert.Equal(!arrows, FARDebugValues.showMomentArrows, "torque arrows after their toggle");
                CloseDebugWindow();

                ctx.Step("Quicksave");
                yield return QuickSave();
                ConfigNode node = ScenarioNodeInSave();
                ctx.Record("savedNode", node.ToString().Replace("\n", " ").Replace("\t", ""));
                Assert.Equal(target.ToString(), node.GetValue("index"), "the preset index in the save");
                Assert.Equal(voxels.ToString(), node.GetValue("numVoxelsControllableVessel"), "the voxel count in the save");
                Assert.Equal(debris.ToString(), node.GetValue("numVoxelsDebrisVessel"), "the debris voxel count in the save");
                Assert.Equal(ticks.ToString(), node.GetValue("minPhysTicksPerUpdate"), "the physics ticks in the save");
                Assert.Equal(!arrows, bool.Parse(node.GetValue("showMomentArrows")), "the torque arrows in the save");

                ctx.Step("Flip the torque arrows back in memory, then quickload");
                FARDebugValues.showMomentArrows = arrows;
                yield return QuickLoad(m);
                FARSettingsScenarioModule loaded = SettingsModule;
                Assert.False(ReferenceEquals(m, loaded), "the quickload made a new settings module");
                Assert.Equal(target, loaded.currentIndex, "the preset index after the quickload");
                Assert.True(ReferenceEquals(preset, FARSettingsScenarioModule.Settings), "FAR's settings are the preset again after the quickload");
                Assert.Equal(voxels, FARSettingsScenarioModule.VoxelSettings.numVoxelsControllableVessel, "the voxel count after the quickload");
                Assert.Equal(debris, FARSettingsScenarioModule.VoxelSettings.numVoxelsDebrisVessel, "the debris voxel count after the quickload");
                Assert.Equal(ticks, FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate, "the physics ticks after the quickload");
                Assert.Equal(!arrows, FARDebugValues.showMomentArrows, "the torque arrows from the save after the quickload");

                ctx.Step("The dialog shows the loaded preset");
                yield return OpenDebugWindow();
                yield return SelectTab(TabDifficulty, 0);
                yield return Imgui.WaitVisible(preset.name, DebugWindow, 5);
                Assert.Equal(target, loaded.currentIndex, "drawing the dialog keeps the loaded preset");
            }
            finally
            {
                CloseDebugWindow();
                RestoreScenario(saved, customAtStart);
                RestorePersistent();
                DeleteSave();
                files.Restore();
            }
        }

        /// <summary>
        /// The difficulty dialog's custom settings: "Choose Custom Settings" leaves the presets (index -1, FAR's settings
        /// the custom ones); the transonic drag fraction, smoothing length and passes typed in are FAR's, a negative
        /// pass count is taken as 0, and no preset changes. A quicksave and quickload bring the custom settings back;
        /// "Switch Back To Presets" then puts FAR on one of the presets.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.SpaceCenter, TimeoutSec = 900)]
        public static IEnumerator Settings_CustomDifficultySurvivesQuickload(TestContext ctx)
        {
            List<FARDifficultyAndExactnessSettings> presets = FARSettingsScenarioModule.FARDifficultyPresetsConfig.Values;
            List<FARDifficultyAndExactnessSettings> presetsAtStart = presets.Select(p => Copy(p)).ToList();
            FARSettingsScenarioModule m = SettingsModule;
            ConfigNode saved = SnapshotScenario();
            FARDifficultyAndExactnessSettings customAtStart = Copy(m.customSettings);
            var files = new FileGuard();
            try
            {
                ctx.Step("Open the debug window; Choose Custom Settings");
                yield return OpenDebugWindow();
                yield return SelectTab(TabDifficulty, 0);
                if (m.currentIndex >= 0)
                    yield return Imgui.Click("Choose Custom Settings", DebugWindow);
                yield return Wait.Frames(3);
                Assert.Equal(-1, m.currentIndex, "the module's index on custom settings");
                Assert.True(ReferenceEquals(m.customSettings, FARSettingsScenarioModule.Settings), "FAR's settings are the custom ones");
                yield return Imgui.WaitVisible("Switch Back To Presets", DebugWindow, 5);

                ctx.Step("Type the custom settings");
                yield return Imgui.SetTextAfterLabel("Frac Mach 1 Drag", "0.55", DebugWindow);
                yield return Imgui.SetTextAfterLabel("% Vehicle Length for Smoothing", "0.02", DebugWindow);
                yield return Imgui.SetTextAfterLabel("Smoothing Passes, Cross-Sectional Area", "-4", DebugWindow);
                yield return Wait.Frames(2);
                Assert.Equal(0, FARSettingsScenarioModule.Settings.numAreaSmoothingPasses, "a negative area smoothing pass count taken as 0");
                yield return Imgui.SetTextAfterLabel("Smoothing Passes, Cross-Sectional Area", "3", DebugWindow);
                yield return Imgui.SetTextAfterLabel("Smoothing Passes, area 2nd deriv", "2", DebugWindow);
                yield return Wait.Frames(2);
                AssertCustom(FARSettingsScenarioModule.Settings, "typed");
                Assert.Equal(-1, FARSettingsScenarioModule.Settings.index, "the custom settings' index");
                for (int i = 0; i < presets.Count; i++)
                    Assert.True(SameValues(presetsAtStart[i], presets[i]), $"preset {presets[i].name} untouched by the custom settings");
                CloseDebugWindow();

                ctx.Step("Quicksave");
                yield return QuickSave();
                ConfigNode node = ScenarioNodeInSave();
                ctx.Record("savedNode", node.ToString().Replace("\n", " ").Replace("\t", ""));
                Assert.Equal("-1", node.GetValue("index"), "the custom index in the save");
                Assert.Equal("3", node.GetValue("numAreaSmoothingPasses"), "the area smoothing passes in the save");

                ctx.Step("Quickload");
                yield return QuickLoad(m);
                FARSettingsScenarioModule loaded = SettingsModule;
                Assert.False(ReferenceEquals(m, loaded), "the quickload made a new settings module");
                Assert.Equal(-1, loaded.currentIndex, "custom settings after the quickload");
                Assert.True(ReferenceEquals(loaded.customSettings, FARSettingsScenarioModule.Settings), "FAR's settings are the custom ones after the quickload");
                AssertCustom(FARSettingsScenarioModule.Settings, "after the quickload");

                ctx.Step("Switch Back To Presets");
                yield return OpenDebugWindow();
                yield return SelectTab(TabDifficulty, 0);
                yield return Imgui.Click("Switch Back To Presets", DebugWindow);
                yield return Wait.Frames(3);
                ctx.Record("presetAfterSwitchBack", $"{loaded.currentIndex} (the config's default is {FARSettingsScenarioModule.FARDifficultyPresetsConfig.Index})");
                Assert.True(loaded.currentIndex >= 0 && loaded.currentIndex < presets.Count, $"back on a preset (index {loaded.currentIndex})");
                Assert.True(ReferenceEquals(presets[loaded.currentIndex], FARSettingsScenarioModule.Settings), "FAR's settings are that preset");
                yield return Imgui.WaitVisible("Choose Custom Settings", DebugWindow, 5);
            }
            finally
            {
                CloseDebugWindow();
                RestoreScenario(saved, customAtStart);
                RestorePersistent();
                for (int i = 0; i < presets.Count && i < presetsAtStart.Count; i++)
                    CopyValues(presetsAtStart[i], presets[i]);
                DeleteSave();
                files.Restore();
            }
        }

        // ------------------------------------------------------------------------------------------ editor: voxel settings

        /// <summary>
        /// The voxel count for a controllable vessel sets the editor's voxel size: an eighth of the voxels doubles the
        /// voxel's edge (the craft's bounds over the count, cube-rooted), so the cross-sections are twice as thick and
        /// half as many, while the craft's length and largest cross-section stay about what they were. Putting the count
        /// back gives the voxel size it had.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Settings_VoxelCountSetsEditorResolution(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            FARVoxelSettings vs = FARSettingsScenarioModule.VoxelSettings;
            int count = vs.numVoxelsControllableVessel;
            VehicleAerodynamics v = Far.EditorVehicle;
            double t0 = v.SectionThickness, len0 = v.Length, max0 = v.MaxCrossSectionArea;
            int n0 = v.GetCrossSectionAreas().Length;
            ctx.Record("fine", $"{count} voxels: section {t0:F4} m, {n0} sections, length {len0:F2} m, max area {max0:F3} m2");
            bool restored = false;
            try
            {
                ctx.Step("An eighth of the voxels");
                vs.numVoxelsControllableVessel = count / 8;
                yield return Far.Revoxelize("an eighth of the voxels");
                double t1 = v.SectionThickness, len1 = v.Length, max1 = v.MaxCrossSectionArea;
                int n1 = v.GetCrossSectionAreas().Length;
                ctx.Record("coarse", $"{count / 8} voxels: section {t1:F4} m, {n1} sections, length {len1:F2} m, max area {max1:F3} m2");
                Assert.Near(2, t1 / t0, 0.02, "the section thickness (voxel size) with an eighth of the voxels, against before");
                Assert.InRange(1.6, 2.4, n0 / (double)n1, "how many times fewer cross-sections");
                Assert.Near(len0, len1, 2 * t1 + 0.05 * len0, "the voxel model's length, coarse against fine");
                Assert.InRange(0.75 * max0, 1.25 * max0, max1, "the largest cross-section, coarse against fine");
                ctx.Record("Cd0.5", $"fine (before) / coarse {Far.Instant(2, 0.5).Cd:F5}");

                ctx.Step("The voxel count put back");
                vs.numVoxelsControllableVessel = count;
                restored = true;
                yield return Far.Revoxelize("the voxel count put back");
                Assert.Near(t0, v.SectionThickness, 1e-3 * t0, "the section thickness with the voxel count put back");
                Assert.Equal(n0, v.GetCrossSectionAreas().Length, "the cross-sections with the voxel count put back");
            }
            finally
            {
                vs.numVoxelsControllableVessel = count;
                if (!restored && EditorGUI.Instance != null)
                    EditorGUI.RequestUpdateVoxel();
            }
        }

        /// <summary>
        /// The subvoxel resolution ("Use Higher Res SubVoxels"; FAR takes it up at the next scene's voxel setup, which
        /// the test calls itself): low-res subvoxels (4 bits a plane) give the craft about the shape the high-res ones
        /// (8 bits) do. Then high-res again at the same voxel count, so the chunk pool is not rebuilt: the shape is the
        /// high-res one, not the low-res chunks read at the high-res scale. The setting and count put back give the
        /// first shape.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Settings_LowResSubvoxelsKeepTheShape(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            FARVoxelSettings vs = FARSettingsScenarioModule.VoxelSettings;
            int count = vs.numVoxelsControllableVessel;
            bool highRes = vs.useHigherResVoxelPoints;
            VehicleAerodynamics v = Far.EditorVehicle;

            (double Volume, double MaxArea, double Length, double Cd) Shape(string what)
            {
                var s = (Far.Volume(v), v.MaxCrossSectionArea, v.Length, Far.Instant(2, 0.5).Cd);
                ctx.Record(what, $"volume {s.Item1:F3} m3, max area {s.Item2:F4} m2, length {s.Item3:F3} m, Cd {s.Item4:F5}");
                return s;
            }

            void AssertNear((double Volume, double MaxArea, double Length, double Cd) e,
                            (double Volume, double MaxArea, double Length, double Cd) a, double tol, string what)
            {
                Assert.Near(e.Volume, a.Volume, tol * e.Volume, $"the volume, {what}");
                Assert.Near(e.MaxArea, a.MaxArea, tol * e.MaxArea, $"the largest cross-section, {what}");
                Assert.Near(e.Length, a.Length, tol * e.Length + 2 * v.SectionThickness, $"the length, {what}");
                Assert.Near(e.Cd, a.Cd, 2 * tol * e.Cd, $"Cd at Mach 0.5, 2 deg, {what}");
            }

            // a few hundred voxels more: the chunk pool is rebuilt (its size follows the count), the voxel barely smaller
            int more = count + count / 20;
            bool restored = false;
            try
            {
                ctx.Step($"High-res subvoxels at {count} voxels");
                vs.useHigherResVoxelPoints = true;
                VehicleVoxel.VoxelSetup();
                yield return Far.Revoxelize("high-res subvoxels");
                var high = Shape("high");

                ctx.Step($"Low-res subvoxels at {more} voxels (the pool rebuilt with 4-bit chunks)");
                vs.useHigherResVoxelPoints = false;
                vs.numVoxelsControllableVessel = more;
                VehicleVoxel.VoxelSetup();
                yield return Far.Revoxelize("low-res subvoxels");
                var low = Shape("low");
                AssertNear(high, low, 0.05, "low-res subvoxels against high-res");

                ctx.Step($"High-res subvoxels again at {more} voxels (the pool not rebuilt)");
                vs.useHigherResVoxelPoints = true;
                VehicleVoxel.VoxelSetup();
                yield return Far.Revoxelize("high-res subvoxels, same count");
                var again = Shape("highAgain");
                AssertNear(high, again, 0.03, "high-res subvoxels after low-res chunks");

                ctx.Step("The setting and the count put back");
                vs.useHigherResVoxelPoints = highRes;
                vs.numVoxelsControllableVessel = count;
                VehicleVoxel.VoxelSetup();
                restored = true;
                yield return Far.Revoxelize("the subvoxel setting put back");
                if (highRes)
                    AssertNear(high, Shape("restored"), 1e-3, "with the setting put back");
            }
            finally
            {
                vs.useHigherResVoxelPoints = highRes;
                vs.numVoxelsControllableVessel = count;
                if (!restored)
                {
                    VehicleVoxel.VoxelSetup();
                    if (EditorGUI.Instance != null)
                        EditorGUI.RequestUpdateVoxel();
                }
            }
        }

        /// <summary>
        /// The area-ruling side of the difficulty presets: on the most lenient preset (the longest smoothing length,
        /// the most passes) the craft's cross-sections come out smoother than on the strictest - less total area
        /// curvature (the sum of |A''|) - and its Mach 1 drag area and transonic drag are lower, over the same voxel
        /// model (same length, about the same volume).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Settings_LenientAreaRulingSmoothsCrossSections(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            List<FARDifficultyAndExactnessSettings> presets = FARSettingsScenarioModule.FARDifficultyPresetsConfig.Values;
            FARDifficultyAndExactnessSettings lenient = presets
                                                        .OrderByDescending(p => p.gaussianVehicleLengthFractionForSmoothing)
                                                        .ThenByDescending(p => p.numAreaSmoothingPasses + p.numDerivSmoothingPasses)
                                                        .First();
            FARDifficultyAndExactnessSettings strict = presets
                                                       .OrderBy(p => p.gaussianVehicleLengthFractionForSmoothing)
                                                       .ThenBy(p => p.numAreaSmoothingPasses + p.numDerivSmoothingPasses)
                                                       .First();
            Assert.True(lenient.gaussianVehicleLengthFractionForSmoothing > strict.gaussianVehicleLengthFractionForSmoothing,
                        $"a lenient preset ({lenient.name}) smooths over more of the craft than a strict one ({strict.name})");
            FARSettingsScenarioModule m = SettingsModule;
            FARDifficultyAndExactnessSettings settingsAtStart = m.settings;
            int indexAtStart = m.currentIndex;
            VehicleAerodynamics v = Far.EditorVehicle;
            bool restored = false;
            try
            {
                ctx.Step("Strict: " + strict.name);
                m.settings = strict;
                m.currentIndex = strict.index;
                yield return Far.Revoxelize("the strict preset");
                double roughS = Curvature(v), sonicS = v.SonicDragArea, cdS = Far.Instant(0, 1.05).Cd;
                double lenS = v.Length, volS = Far.Volume(v);
                ctx.Record("strict", $"sum|A''| {roughS:G5}, Mach 1 drag area {sonicS:G5}, Cd(M1.05) {cdS:F5}, volume {volS:F3}");

                ctx.Step("Lenient: " + lenient.name);
                m.settings = lenient;
                m.currentIndex = lenient.index;
                yield return Far.Revoxelize("the lenient preset");
                double roughL = Curvature(v), sonicL = v.SonicDragArea, cdL = Far.Instant(0, 1.05).Cd;
                double lenL = v.Length, volL = Far.Volume(v);
                ctx.Record("lenient", $"sum|A''| {roughL:G5}, Mach 1 drag area {sonicL:G5}, Cd(M1.05) {cdL:F5}, volume {volL:F3}");

                Assert.Near(lenS, lenL, 2 * v.SectionThickness, "the same voxel model's length under both presets");
                Assert.Near(volS, volL, 0.15 * volS, "about the same volume under both presets (smoothing moves area, it does not make it)");
                Assert.True(roughL < roughS, $"smoother cross-sections when lenient: sum|A''| {roughL:G5} against {roughS:G5}");
                Assert.True(sonicL < sonicS, $"a lower Mach 1 drag area when lenient: {sonicL:G5} against {sonicS:G5}");
                Assert.True(cdL < cdS, $"less transonic drag when lenient: Cd at Mach 1.05 {cdL:F5} against {cdS:F5}");

                ctx.Step("The settings put back");
                m.settings = settingsAtStart;
                m.currentIndex = indexAtStart;
                restored = true;
                yield return Far.Revoxelize("the settings put back");
            }
            finally
            {
                m.settings = settingsAtStart;
                m.currentIndex = indexAtStart;
                if (!restored && EditorGUI.Instance != null)
                    EditorGUI.RequestUpdateVoxel();
            }
        }

        // ------------------------------------------------------------------------------------------ helpers: window

        private static FARSettingsScenarioModule SettingsModule =>
            FARSettingsScenarioModule.Instance ?? throw new UITestFailure("FAR's settings module (FARSettingsScenarioModule) is not there");

        private static FARDebugAndSettings DebugInstance =>
            UnityEngine.Object.FindObjectOfType<FARDebugAndSettings>() ?? throw new UITestFailure("FAR's FARDebugAndSettings is not there");

        private static bool DebugMenuOpen => (bool)StaticField(typeof(FARDebugAndSettings), "debugMenu").GetValue(null);

        private static bool ModelOptionsOpen => (bool)StaticField(typeof(FARDebugAndSettings), "modelOptions").GetValue(null);

        // what the toolbar button's toggle also flips (the editor's and flight's windows), and the tab, put back on close
        private static bool? editorShownAtStart, flightShownAtStart;
        private static object tabAtStart;

        /// <summary>The debug window shown, the way FAR's toolbar button shows it at the Space Center.</summary>
        private static IEnumerator OpenDebugWindow()
        {
            if (tabAtStart == null)
                tabAtStart = Get(DebugInstance, "activeTab");
            if (!DebugMenuOpen)
            {
                editorShownAtStart = EditorWindow.WindowShown;
                flightShownAtStart = Far.FlightGuiShown;
                // what the button's click calls (ToggleGUI): the debug window's toggle, and the editor's and flight's
                FARDebugAndSettings.Toggle();
                if (FARDebugAndSettings.FARDebugButtonStock)
                    FARDebugAndSettings.FARDebugButtonStock.SetTrue(false);
            }

            Assert.True(DebugMenuOpen, "FAR's toolbar button handler opened the debug window");
            yield return Imgui.WaitWindow(DebugWindow);
            yield return Wait.Frames(2);
        }

        private static void CloseDebugWindow()
        {
            try
            {
                if (DebugMenuOpen)
                    FARDebugAndSettings.ForceCloseDebugWindow();
                if (editorShownAtStart.HasValue)
                    EditorWindow.WindowShown = editorShownAtStart.Value;
                if (flightShownAtStart.HasValue)
                    Far.FlightGuiShown = flightShownAtStart.Value;
                editorShownAtStart = flightShownAtStart = null;
                if (tabAtStart != null)
                    Set(DebugInstance, "activeTab", tabAtStart);
                tabAtStart = null;
            }
            catch (Exception e)
            {
                Log.Warn("SettingsTests: closing FAR's debug window: " + e.Message);
            }
        }

        /// <summary>A tab picked through the window's tab grid.</summary>
        private static IEnumerator SelectTab(string label, int index)
        {
            if (Convert.ToInt32(Get(DebugInstance, "activeTab")) != index)
                yield return Imgui.Select(label, DebugWindow);
            yield return Wait.Frames(2);
            Assert.Equal(index, Convert.ToInt32(Get(DebugInstance, "activeTab")), $"the debug window on its {label} tab");
        }

        // ------------------------------------------------------------------------------------------ helpers: settings

        private static bool NotFiveDecimals(double v) => Math.Abs(Math.Round(v, 5) - v) > 1e-6 * Math.Abs(v);

        /// <summary>FARDifficultyPresets.cfg as shipped: name, transonic drag fraction, smoothing length, area and 2nd-derivative passes.</summary>
        private static readonly object[][] ShippedPresets =
        {
            new object[] { "Low Drag, Lenient Area Ruling", 0.6, 0.03, 2, 2 },
            new object[] { "Moderate Drag, Lenient Area Ruling", 0.7, 0.03, 2, 2 },
            new object[] { "Moderate Drag, Moderate Area Ruling", 0.7, 0.015, 2, 1 },
            new object[] { "High Drag, Moderate Area Ruling", 0.85, 0.015, 2, 1 },
            new object[] { "Full Drag, Moderate Area Ruling", 1.0, 0.015, 2, 1 },
            new object[] { "Full Drag, Strict Area Ruling", 1.0, 0.010, 1, 1 },
        };

        private static void AssertPresetsFromConfig(TestContext ctx, List<FARDifficultyAndExactnessSettings> presets)
        {
            ctx.Record("presets", string.Join("; ", presets.Select(p => $"{p.index} {p.name}")));
            Assert.Equal(ShippedPresets.Length, presets.Count, "the presets in FARDifficultyPresets.cfg");
            Assert.Equal(4, FARSettingsScenarioModule.FARDifficultyPresetsConfig.Index, "the new-game preset (FARDifficultyPresets.cfg index)");
            for (int i = 0; i < presets.Count; i++)
            {
                FARDifficultyAndExactnessSettings p = presets[i];
                object[] e = ShippedPresets[i];
                Assert.Equal((string)e[0], p.name, $"preset {i}'s name");
                Assert.Equal(i, p.index, $"{p.name}'s index");
                Assert.Near((double)e[1], p.fractionTransonicDrag, 1e-9, p.name + "'s transonic drag fraction");
                Assert.Near((double)e[2], p.gaussianVehicleLengthFractionForSmoothing, 1e-9, p.name + "'s smoothing length");
                Assert.Equal((int)e[3], p.numAreaSmoothingPasses, p.name + "'s area smoothing passes");
                Assert.Equal((int)e[4], p.numDerivSmoothingPasses, p.name + "'s 2nd derivative smoothing passes");
                Assert.Equal(p.name, FARSettingsScenarioModule.FARDifficultyPresetsConfig.Names[i], $"the dialog's name for preset {i}");
            }
        }

        private static void AssertCustom(FARDifficultyAndExactnessSettings s, string when)
        {
            Assert.Near(0.55, s.fractionTransonicDrag, 1e-9, "the custom transonic drag fraction " + when);
            Assert.Near(0.02, s.gaussianVehicleLengthFractionForSmoothing, 1e-9, "the custom smoothing length " + when);
            Assert.Equal(3, s.numAreaSmoothingPasses, "the custom area smoothing passes " + when);
            Assert.Equal(2, s.numDerivSmoothingPasses, "the custom 2nd derivative smoothing passes " + when);
        }

        private static FARDifficultyAndExactnessSettings Copy(FARDifficultyAndExactnessSettings s) =>
            s == null
                ? null
                : new FARDifficultyAndExactnessSettings(s.fractionTransonicDrag, s.gaussianVehicleLengthFractionForSmoothing,
                                                        s.numAreaSmoothingPasses, s.numDerivSmoothingPasses, s.index, s.name);

        private static void CopyValues(FARDifficultyAndExactnessSettings from, FARDifficultyAndExactnessSettings to)
        {
            if (from == null || to == null)
                return;
            to.fractionTransonicDrag = from.fractionTransonicDrag;
            to.gaussianVehicleLengthFractionForSmoothing = from.gaussianVehicleLengthFractionForSmoothing;
            to.numAreaSmoothingPasses = from.numAreaSmoothingPasses;
            to.numDerivSmoothingPasses = from.numDerivSmoothingPasses;
            to.name = from.name;
        }

        private static bool SameValues(FARDifficultyAndExactnessSettings a, FARDifficultyAndExactnessSettings b) =>
            a.fractionTransonicDrag == b.fractionTransonicDrag &&
            a.gaussianVehicleLengthFractionForSmoothing == b.gaussianVehicleLengthFractionForSmoothing &&
            a.numAreaSmoothingPasses == b.numAreaSmoothingPasses && a.numDerivSmoothingPasses == b.numDerivSmoothingPasses &&
            a.name == b.name;

        /// <summary>
        /// The settings module's state as its OnSave writes it (without OnSave's side effects: the flight GUI's data,
        /// FAR's config files), with the debug values its OnLoad reads, to put back with OnLoad.
        /// </summary>
        private static ConfigNode SnapshotScenario()
        {
            FARSettingsScenarioModule m = SettingsModule;
            var n = new ConfigNode("SCENARIO");
            n.AddValue("newGame", m.newGame);
            n.AddValue("fractionTransonicDrag", m.settings.fractionTransonicDrag.ToString("R"));
            n.AddValue("gaussianVehicleLengthFractionForSmoothing", m.settings.gaussianVehicleLengthFractionForSmoothing.ToString("R"));
            n.AddValue("numAreaSmoothingPasses", m.settings.numAreaSmoothingPasses);
            n.AddValue("numDerivSmoothingPasses", m.settings.numDerivSmoothingPasses);
            n.AddValue("numVoxelsControllableVessel", m.voxelSettings.numVoxelsControllableVessel);
            n.AddValue("numVoxelsDebrisVessel", m.voxelSettings.numVoxelsDebrisVessel);
            n.AddValue("minPhysTicksPerUpdate", m.voxelSettings.minPhysTicksPerUpdate);
            n.AddValue("useHigherResVoxelPoints", m.voxelSettings.useHigherResVoxelPoints);
            n.AddValue("use32BitIndices", DebugVoxelMesh.Use32BitIndices);
            n.AddValue("index", m.settings.index);
            n.AddValue("allowStructuralFailures", FARDebugValues.allowStructuralFailures);
            n.AddValue("showMomentArrows", FARDebugValues.showMomentArrows);
            n.AddValue("useBlizzyToolbar", FARDebugValues.useBlizzyToolbar);
            n.AddValue("aeroFailureExplosions", FARDebugValues.aeroFailureExplosions);
            n.AddValue("wholeWingAspectRatio", FARDebugValues.wholeWingAspectRatio);
            return n;
        }

        /// <summary>The snapshot loaded into the settings module there is now (a quickload makes a new one).</summary>
        private static void RestoreScenario(ConfigNode saved, FARDifficultyAndExactnessSettings custom)
        {
            try
            {
                FARSettingsScenarioModule m = FARSettingsScenarioModule.Instance;
                if (m == null)
                    return;
                List<ConfigNode> flightGui = m.flightGUISettings;
                m.OnLoad(saved);
                m.flightGUISettings = flightGui;
                // on a preset, OnLoad starts the custom settings afresh: the ones there were
                if (m.currentIndex >= 0 && custom != null)
                {
                    m.customSettings = Copy(custom);
                    m.customSettings.index = -1;
                }
            }
            catch (Exception e)
            {
                Log.Warn("SettingsTests: restoring FAR's settings: " + e.Message);
            }
        }

        private static string SaveDir => Path.Combine(KSPUtil.ApplicationRootPath, "saves", HighLogic.SaveFolder);

        private static string SavePath => Path.Combine(SaveDir, SaveName + ".sfs");

        /// <summary>The game saved under the test's name, as a quicksave saves it; its config saver given time to finish.</summary>
        private static IEnumerator QuickSave()
        {
            GamePersistence.SaveGame(SaveName, HighLogic.SaveFolder, SaveMode.OVERWRITE);
            // FARDebugAndSettings.SaveConfigs saves FAR's configs in a coroutine (ConfigAdapter.SaveAll, threaded)
            yield return Wait.Seconds(3);
            Assert.True(File.Exists(SavePath), "the quicksave " + SavePath);
        }

        /// <summary>
        ///     The test's save loaded at the Space Center as KSP's quickload loads one that does not start in flight
        ///     (QuickSaveLoad.onQuickloadPipelineFinished): the game read, its scenario modules brought up to date, written
        ///     as the persistent save, and the Space Center loaded, which loads the persistent save. (Starting the loaded
        ///     game in place does not do it: the Space Center still loads the persistent save, and FAR's settings come
        ///     from whatever it last held.) Waits for the new settings module.
        /// </summary>
        private static IEnumerator QuickLoad(FARSettingsScenarioModule old)
        {
            string folder = HighLogic.SaveFolder;
            ConfigNode node = GamePersistence.LoadSFSFile(SaveName, folder);
            Assert.NotNull(node, "the quicksave read back");
            Game game = GamePersistence.LoadGameCfg(node, SaveName, true, false);
            Assert.NotNull(game, "the quicksave loaded as a game");
            GamePersistence.UpdateScenarioModules(game);
            GameEvents.onGameStatePostLoad.Fire(node);
            string persistent = Path.Combine(SaveDir, "persistent.sfs");
            if (persistentBefore == null && File.Exists(persistent))
                persistentBefore = File.ReadAllBytes(persistent);
            GamePersistence.SaveGame(game, "persistent", folder, SaveMode.OVERWRITE);
            HighLogic.LoadScene(GameScenes.SPACECENTER);
            InputLockManager.ClearControlLocks();
            yield return Wait.Until(() => HighLogic.LoadedScene == GameScenes.SPACECENTER &&
                                          FARSettingsScenarioModule.Instance != null &&
                                          !ReferenceEquals(FARSettingsScenarioModule.Instance, old) &&
                                          FARSettingsScenarioModule.Instance.settings != null &&
                                          FARSettingsScenarioModule.Instance.voxelSettings != null,
                                    300, "the Space Center reloaded from the quicksave, with a new FAR settings module", shot: false);
            yield return Wait.Seconds(2);
            Ugui.DismissPopups();
        }

        // the persistent save as it was before QuickLoad wrote the test's save over it
        private static byte[] persistentBefore;

        /// <summary>
        ///     The persistent save put back as it was before QuickLoad, from its bytes: saving the game again instead would
        ///     have FAR's OnSave start its config saver on a thread, racing the config files' restore that follows.
        /// </summary>
        private static void RestorePersistent()
        {
            if (persistentBefore == null)
                return;
            try
            {
                File.WriteAllBytes(Path.Combine(SaveDir, "persistent.sfs"), persistentBefore);
            }
            catch (Exception e)
            {
                Log.Warn("SettingsTests: putting the persistent save back: " + e.Message);
            }
            finally
            {
                persistentBefore = null;
            }
        }

        /// <summary>The FARSettingsScenarioModule node of the test's save, read from the file.</summary>
        private static ConfigNode ScenarioNodeInSave()
        {
            ConfigNode root = ConfigNode.Load(SavePath);
            Assert.NotNull(root, "the quicksave's file read");
            ConfigNode game = root.GetNode("GAME") ?? root;
            foreach (ConfigNode s in game.GetNodes("SCENARIO"))
                if (s.GetValue("name") == nameof(FARSettingsScenarioModule))
                    return s;
            throw new UITestFailure("the quicksave has no FARSettingsScenarioModule SCENARIO node");
        }

        private static void DeleteSave()
        {
            foreach (string ext in new[] { ".sfs", ".loadmeta" })
                try
                {
                    string path = Path.Combine(SaveDir, SaveName + ext);
                    if (File.Exists(path))
                        File.Delete(path);
                }
                catch (Exception e)
                {
                    Log.Warn($"SettingsTests: deleting the test save ({ext}): {e.Message}");
                }
        }

        /// <summary>
        /// FAR's config saver writes GameData/Custom_&lt;config&gt;.cfg at a game save when a setting it keeps (the GUI
        /// colours, FARAeroData, the stress templates) differs from what it last saw. The files as they were, and what
        /// the saver last saw, kept to be put back: new files deleted, changed ones rewritten.
        /// </summary>
        private sealed class FileGuard
        {
            private readonly Dictionary<string, byte[]> files = new Dictionary<string, byte[]>();
            private readonly Dictionary<string, string> lastSeen;

            public FileGuard()
            {
                foreach (string f in Custom())
                    files[f] = File.ReadAllBytes(f);
                lastSeen = LastConfigs is Dictionary<string, string> d ? new Dictionary<string, string>(d) : null;
            }

            private static Dictionary<string, string> LastConfigs =>
                StaticField(typeof(ConfigAdapter), "lastConfigs").GetValue(null) as Dictionary<string, string>;

            private static string[] Custom() =>
                Directory.Exists(KSPUtils.GameDataPath) ? Directory.GetFiles(KSPUtils.GameDataPath, "Custom_*.cfg") : new string[0];

            public void Restore()
            {
                try
                {
                    foreach (string f in Custom())
                        if (!files.TryGetValue(f, out byte[] before))
                        {
                            Log.Info("SettingsTests: deleting " + f + ", written during the test");
                            File.Delete(f);
                        }
                        else if (!before.SequenceEqual(File.ReadAllBytes(f)))
                        {
                            Log.Info("SettingsTests: putting back " + f);
                            File.WriteAllBytes(f, before);
                        }

                    Dictionary<string, string> last = LastConfigs;
                    if (lastSeen != null && last != null)
                    {
                        last.Clear();
                        foreach (KeyValuePair<string, string> kv in lastSeen)
                            last[kv.Key] = kv.Value;
                    }
                }
                catch (Exception e)
                {
                    Log.Warn("SettingsTests: putting FAR's config files back: " + e.Message);
                }
            }
        }

        // ------------------------------------------------------------------------------------------ helpers: editor

        /// <summary>A fresh voxelization of the craft in the editor, and its result taken up by the editor's sims.</summary>
        /// <summary>The cross-sections' total curvature: the sum of |A''| over the craft, times the section thickness.</summary>
        private static double Curvature(VehicleAerodynamics v) =>
            v.GetCrossSection2ndAreaDerivs().Sum(d => Math.Abs(d)) * v.SectionThickness;


    }
}
