/*
Ferram Aerospace Research v0.16.2.0 "Maskell"
=========================
Aerodynamics model for Kerbal Space Program

Copyright 2022, Michael Ferrara, aka Ferram4

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Ferram Aerospace Research is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Ferram Aerospace Research.  If not, see <http://www.gnu.org/licenses/>.

   Serious thanks:		a.g., for tons of bugfixes and code-refactorings
				stupid_chris, for the RealChuteLite implementation
            			Taverius, for correcting a ton of incorrect values
				Tetryds, for finding lots of bugs and issues and not letting me get away with them, and work on example crafts
            			sarbian, for refactoring code for working with MechJeb, and the Module Manager updates
            			ialdabaoth (who is awesome), who originally created Module Manager
                        	Regex, for adding RPM support
				DaMichel, for some ferramGraph updates and some control surface-related features
            			Duxwing, for copy editing the readme

   CompatibilityChecker by Majiir, BSD 2-clause http://opensource.org/licenses/BSD-2-Clause

   Part.cfg changes powered by sarbian & ialdabaoth's ModuleManager plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/55219

   ModularFLightIntegrator by Sarbian, Starwaster and Ferram4, MIT: http://opensource.org/licenses/MIT
	http://forum.kerbalspaceprogram.com/threads/118088

   Toolbar integration powered by blizzy78's Toolbar plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/60863
 */

using System.Collections.Generic;
using System.Text;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.Resources;
using FerramAerospaceResearch.Settings;
using KSP.IO;
using KSP.Localization;
using StringLeakTest;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FARFlightGUI
{
    [DefaultExecutionOrder(1)]
    public class FlightGUI : VesselModule
    {
        private static bool showGUI;
        public static bool showAllGUI = true;
        public static bool savedShowGUI = true;
        private static Rect mainGuiRect;
        private static Rect dataGuiRect;
        private static Rect settingsGuiRect;
        private static Rect debugGuiRect;
        private static IButton blizzyFlightGUIButton;
        private static int activeFlightGUICount;
        private static int frameCountForSaving;
        public static Dictionary<Vessel, FlightGUI> vesselFlightGUI;

        internal static GUIStyle boxStyle;
        internal static GUIStyle buttonStyle;

        private readonly StringBuilder _strBuilder = new StringBuilder();
        private FARVesselAero _vesselAero;

        private PhysicsCalcs _physicsCalcs;

        private FlightStatusGUI _flightStatusGUI;
        private StabilityAugmentation _stabilityAugmentation;
        private FlightDataGUI _flightDataGUI;
        private FlightDataLogger flightDataLogger;
        private DebugGUI debugGUI;

        private bool showFlightDataWindow;
        private bool showSettingsWindow;
        private bool showDebugWindow;
        private bool started;

        private GUIDropDown<int> settingsWindow;
        public VesselFlightInfo InfoParameters { get; private set; }
        public AeroVisualizationGUI AeroVizGUI { get; private set; }

        public AirspeedSettingsGUI airSpeedGUI { get; private set; }

        protected override void OnAwake()
        {
            if (vesselFlightGUI == null)
                vesselFlightGUI = new Dictionary<Vessel, FlightGUI>();
        }

        protected override void OnStart()
        {
            base.OnStart();

            showGUI = savedShowGUI;
            //since we're sharing the button, we need these shenanigans now
            if (FARDebugAndSettings.FARDebugButtonStock && HighLogic.LoadedSceneIsFlight)
                if (showGUI)
                    FARDebugAndSettings.FARDebugButtonStock.SetTrue(false);
                else
                    FARDebugAndSettings.FARDebugButtonStock.SetFalse(false);

            _vesselAero = GetComponent<FARVesselAero>();
            _physicsCalcs = new PhysicsCalcs(vessel, _vesselAero);
            _flightStatusGUI = new FlightStatusGUI();
            _stabilityAugmentation = new StabilityAugmentation(vessel);
            _flightDataGUI = new FlightDataGUI();
            AeroVizGUI = new AeroVisualizationGUI();
            debugGUI = new DebugGUI();

            settingsWindow = new GUIDropDown<int>(new[]
                                                  {
                                                      LocalizerExtensions.Get("FARFlightGUIWindowSelect0"),
                                                      LocalizerExtensions.Get("FARFlightGUIWindowSelect1"),
                                                      LocalizerExtensions.Get("FARFlightGUIWindowSelect2"),
                                                      LocalizerExtensions.Get("FARFlightGUIWindowSelect3")
                                                  },
                                                  new[] { 0, 1, 2, 3 });

            if (vesselFlightGUI.ContainsKey(vessel))
                vesselFlightGUI[vessel] = this;
            else
                vesselFlightGUI.Add(vessel, this);
            flightDataLogger = FlightDataLogger.CreateLogger(vessel);

            enabled = true;

            if (FARDebugValues.useBlizzyToolbar)
                GenerateBlizzyToolbarButton();

            activeFlightGUICount++;
            started = true;

            if (vessel == FlightGlobals.ActiveVessel || FlightGlobals.ActiveVessel == null)
                LoadConfigs();

            GameEvents.onShowUI.Add(ShowUI);
            GameEvents.onHideUI.Add(HideUI);
        }

        public override Activation GetActivation() => Activation.FlightScene | Activation.LoadedVessels;

        private void OnDestroy()
        {
            FlightGUIDrawer.SetGUIActive(this, false);
            GameEvents.onShowUI.Remove(ShowUI);
            GameEvents.onHideUI.Remove(HideUI);
            SaveConfigs();
            if (vessel)
                vesselFlightGUI.Remove(vessel);
            _physicsCalcs = null;

            _flightDataGUI?.SaveSettings();
            _flightDataGUI = null;

            _stabilityAugmentation?.SaveAndDestroy();
            _stabilityAugmentation = null;

            airSpeedGUI?.SaveSettings();
            airSpeedGUI = null;

            AeroVizGUI?.SaveSettings();

            if (flightDataLogger)
            {
                flightDataLogger.StopLogging();
                flightDataLogger = null;
            }

            _flightStatusGUI = null;
            settingsWindow = null;

            // OnStart only runs once the vessel has been loaded
            if (started)
                activeFlightGUICount--;

            if (activeFlightGUICount <= 0)
            {
                activeFlightGUICount = 0;
                if (blizzyFlightGUIButton != null)
                    ClearBlizzyToolbarButton();
            }

            savedShowGUI = showGUI;
        }

        public void SaveData()
        {
            if (vessel != FlightGlobals.ActiveVessel)
                return;
            SaveConfigs();
            airSpeedGUI?.SaveSettings();
            _stabilityAugmentation?.SaveSettings();
            _flightDataGUI?.SaveSettings();
            AeroVizGUI?.SaveSettings();
        }

        public static void SaveActiveData()
        {
            if (!FlightGlobals.ready ||
                FlightGlobals.ActiveVessel == null ||
                vesselFlightGUI == null ||
                !vesselFlightGUI.TryGetValue(FlightGlobals.ActiveVessel, out FlightGUI gui))
                return;
            if (gui != null)
                gui.SaveData();
        }

        //Receives message from FARVesselAero through _vessel on the recalc being completed
        public void UpdateAeroModules(
            List<FARAeroPartModule> newAeroModules,
            List<FARWingAerodynamicModel> legacyWingModels
        )
        {
            _physicsCalcs.UpdateAeroModules(newAeroModules, legacyWingModels);
        }

        // ReSharper disable once UnusedMember.Local
        //Receives a message from any FARWingAerodynamicModel or FARAeroPartModule that has failed to update the GUI
        private void AerodynamicFailureStatus()
        {
            _flightStatusGUI?.AerodynamicFailureStatus();
        }

        private void FixedUpdate()
        {
            if (_physicsCalcs == null)
                return;

            InfoParameters = _physicsCalcs.UpdatePhysicsParameters();

            _stabilityAugmentation.UpdatePhysicsInfo(InfoParameters);
            _flightStatusGUI.UpdateInfoParameters(InfoParameters);
            _flightDataGUI.UpdateInfoParameters(InfoParameters);
        }

        private void Update()
        {
            FlightGUIDrawer.SetGUIActive(this, vessel == FlightGlobals.ActiveVessel && showGUI && showAllGUI);
            if (frameCountForSaving >= 120)
            {
                SaveActiveData();
                frameCountForSaving = 0;
            }
            else
            {
                frameCountForSaving++;
            }
        }

        private void LateUpdate()
        {
            if (airSpeedGUI != null)
                airSpeedGUI.ChangeSurfVelocity();
            else if (vessel != null)
                airSpeedGUI = new AirspeedSettingsGUI(vessel);
        }

        public void DrawGUI()
        {
            GUI.skin = HighLogic.Skin;
            if (boxStyle == null)
            {
                boxStyle = new GUIStyle(GUI.skin.box);
                boxStyle.normal.textColor = boxStyle.focused.textColor = Color.white;
                boxStyle.hover.textColor = boxStyle.active.textColor = Color.yellow;
                boxStyle.onNormal.textColor = boxStyle.onFocused.textColor =
                                                  boxStyle.onHover.textColor =
                                                      boxStyle.onActive.textColor = Color.green;
                boxStyle.padding = new RectOffset(2, 2, 2, 2);
            }

            if (buttonStyle == null)
            {
                buttonStyle = new GUIStyle(GUI.skin.button);
                buttonStyle.normal.textColor = buttonStyle.focused.textColor = Color.white;
                buttonStyle.hover.textColor =
                    buttonStyle.active.textColor = buttonStyle.onActive.textColor = Color.yellow;
                buttonStyle.onNormal.textColor =
                    buttonStyle.onFocused.textColor = buttonStyle.onHover.textColor = Color.green;
                buttonStyle.padding = new RectOffset(2, 2, 2, 2);
            }

            if (vessel != FlightGlobals.ActiveVessel || !showGUI || !showAllGUI)
                return;
            mainGuiRect = GUILayout.Window(GetHashCode(),
                                           mainGuiRect,
                                           MainFlightGUIWindow,
                                           "FAR, " + Version.LongString,
                                           GUILayout.MinWidth(230));
            GUIUtils.ClampToScreen(mainGuiRect);

            if (showFlightDataWindow)
            {
                dataGuiRect = GUILayout.Window(GetHashCode() + 1,
                                               dataGuiRect,
                                               FlightDataWindow,
                                               LocalizerExtensions.Get("FARFlightDataTitle"),
                                               GUILayout.MinWidth(150));
                GUIUtils.ClampToScreen(dataGuiRect);
            }

            if (showSettingsWindow)
            {
                settingsGuiRect = GUILayout.Window(GetHashCode() + 2,
                                                   settingsGuiRect,
                                                   SettingsWindow,
                                                   LocalizerExtensions.Get("FARFlightSettings"),
                                                   GUILayout.MinWidth(200));
                GUIUtils.ClampToScreen(settingsGuiRect);
            }

            // ReSharper disable once InvertIf
            if (showDebugWindow)
            {
                debugGuiRect = GUILayout.Window(GetHashCode() + 3,
                                                debugGuiRect,
                                                DebugWindow,
                                                LocalizerExtensions.Get("FARDebugWindow"),
                                                GUILayout.MinWidth(200));
                GUIUtils.ClampToScreen(debugGuiRect);
            }
        }

        private void MainFlightGUIWindow(int windowId)
        {
            GUILayout.BeginVertical(GUILayout.Height(100));
            GUILayout.BeginHorizontal();
            _strBuilder.Length = 0;
            _strBuilder.Append(LocalizerExtensions.Get("FARAbbrevMach"));
            _strBuilder.Append(": ");
            _strBuilder.Concat((float)_vesselAero.MachNumber, 3).AppendLine();
            _strBuilder.AppendFormat(LocalizerExtensions.Get("FARFlightGUIReynolds"), _vesselAero.ReynoldsNumber);
            GUILayout.Box(_strBuilder.ToString(), boxStyle, GUILayout.ExpandWidth(true));
            GUILayout.EndHorizontal();

            _strBuilder.Length = 0;
            _strBuilder.Append(LocalizerExtensions.Get("FARFlightGUIAtmDens"));
            _strBuilder.Concat((float)vessel.atmDensity, 3);

            GUILayout.Box(_strBuilder.ToString(), boxStyle, GUILayout.ExpandWidth(true));

            _flightStatusGUI.Display();
            showFlightDataWindow = GUILayout.Toggle(showFlightDataWindow,
                                                    LocalizerExtensions.Get("FARFlightGUIFltDataBtn"),
                                                    buttonStyle,
                                                    GUILayout.ExpandWidth(true));
            showSettingsWindow = GUILayout.Toggle(showSettingsWindow,
                                                  LocalizerExtensions.Get("FARFlightGUIFltSettings"),
                                                  buttonStyle,
                                                  GUILayout.ExpandWidth(true));
            showDebugWindow = GUILayout.Toggle(showDebugWindow,
                                               LocalizerExtensions.Get("FARFlightGUIFltDebug"),
                                               buttonStyle,
                                               GUILayout.ExpandWidth(true));

            bool logging = GUILayout.Toggle(flightDataLogger.IsActive,
                                            LocalizerExtensions.Get("FARFlightGUIFltLogging"),
                                            buttonStyle,
                                            GUILayout.ExpandWidth(true));
            if (logging != flightDataLogger.IsActive)
            {
                if (!flightDataLogger.IsActive)
                    flightDataLogger.StartLogging();
                else
                    flightDataLogger.StopLogging();
            }

            flightDataLogger.Period =
                GUIUtils.TextEntryForInt(LocalizerExtensions.Get("FARFlightGUIFltLogPeriod"),
                                         150,
                                         flightDataLogger.Period);
            flightDataLogger.FlushPeriod =
                GUIUtils.TextEntryForInt(LocalizerExtensions.Get("FARFlightGUIFltLogFlushPeriod"),
                                         150,
                                         flightDataLogger.FlushPeriod);
            DebugVisualizationGUI();

            GUILayout.Label(LocalizerExtensions.Get("FARFlightGUIFltAssistance"));

            _stabilityAugmentation.Display();

            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        private void DebugVisualizationGUI()
        {
            if (!FARConfig.Voxelization.DebugInFlight)
                return;
            GUILayout.BeginHorizontal();
            GUI.enabled = !_vesselAero.VehicleAero.Voxelizing;
            if (GUILayout.Button(LocalizerExtensions.Get("FARDebugVoxels")))
            {
                _vesselAero.VehicleAero.DebugVisualizeVoxels(vessel.transform.localToWorldMatrix);
            }

            GUI.enabled = true;
            GUILayout.EndHorizontal();
        }

        private void FlightDataWindow(int windowId)
        {
            _flightDataGUI.DataDisplay();
            GUI.DragWindow();
        }

        private void SettingsWindow(int windowId)
        {
            GUILayout.Label(LocalizerExtensions.Get("FARFlightSettingsLabel"));
            settingsWindow.GUIDropDownDisplay();
            int selection = settingsWindow.ActiveSelection;
            switch (selection)
            {
                case 0:
                    if (_flightDataGUI.SettingsDisplay())
                        dataGuiRect.height = 0;
                    break;
                case 1:
                    _stabilityAugmentation.SettingsDisplay();
                    break;
                case 2:
                    airSpeedGUI.AirSpeedSettings();
                    break;
                case 3:
                    AeroVizGUI.SettingsDisplay();
                    break;
            }

            GUI.DragWindow();
        }

        private void DebugWindow(int windowId)
        {
            GUILayout.Label(LocalizerExtensions.Get("FARDebugWindowLabel"));
            debugGUI.Display(ref debugGuiRect);
            GUI.DragWindow();
        }

        private static void ClearBlizzyToolbarButton()
        {
            blizzyFlightGUIButton.Destroy();
            blizzyFlightGUIButton = null;
        }

        private static void GenerateBlizzyToolbarButton()
        {
            if (blizzyFlightGUIButton != null)
                return;
            blizzyFlightGUIButton = ToolbarManager.Instance.add("FerramAerospaceResearch", "FARFlightButtonBlizzy");
            blizzyFlightGUIButton.TexturePath = FARAssets.Instance.Textures.IconSmall.Url;
            blizzyFlightGUIButton.ToolTip = "FAR Flight Sys";
            blizzyFlightGUIButton.OnClick += e => showGUI = !showGUI;
        }

        public static void onAppLaunchToggle()
        {
            showGUI = !showGUI;
        }

        // ReSharper disable once MemberCanBeMadeStatic.Local -> static does not work with GameEvents
        private void HideUI()
        {
            showAllGUI = false;
        }

        // ReSharper disable once MemberCanBeMadeStatic.Local -> static does not work with GameEvents
        private void ShowUI()
        {
            showAllGUI = true;
        }

        private static void SaveConfigs()
        {
            if (FARDebugAndSettings.config == null)
                return;
            PluginConfiguration config = FARDebugAndSettings.config;
            config.SetValue("flight_mainGuiRect", mainGuiRect);
            config.SetValue("flight_dataGuiRect", dataGuiRect);
            config.SetValue("flight_settingsGuiRect", settingsGuiRect);
            config.SetValue("flight_debugGuiRect", debugGuiRect);
        }

        private static void LoadConfigs()
        {
            PluginConfiguration config = FARDebugAndSettings.config;
            mainGuiRect = config.GetValue("flight_mainGuiRect", new Rect());
            dataGuiRect = config.GetValue("flight_dataGuiRect", new Rect());
            settingsGuiRect = config.GetValue("flight_settingsGuiRect", new Rect());
            debugGuiRect = config.GetValue("flight_debugGuiRect", new Rect());
        }
    }
}
