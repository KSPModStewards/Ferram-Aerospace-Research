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

using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using FerramAerospaceResearch.Resources;
using FerramAerospaceResearch.Settings;
using FerramAerospaceResearch.Threading;
using KSP.IO;
using KSP.UI.Screens;
using UnityEngine;

namespace FerramAerospaceResearch
{
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public class FARDebugAndSettings : MonoBehaviour
    {
        public static PluginConfiguration config;
        private static bool hasScenarioChanged = true;
        private static IButton FARDebugButtonBlizzy;
        public static ApplicationLauncherButton FARDebugButtonStock;
        public static Callback Toggle = delegate { };
        private static bool debugMenu;
        private static bool inputLocked;

        private static readonly string[] MenuTab_str =
        {
            "Difficulty and Debug", "Aerodynamic Failure", "Atm Composition"
        };

        public static readonly string[] FlowMode_str =
        {
            "NO_FLOW", "ALL_VESSEL", "STAGE_PRIORITY_FLOW", "STACK_PRIORITY_SEARCH"
        };

        private Rect debugWinPos = new Rect(50, 50, 700, 250);
        private Texture2D cLTexture;
        private Texture2D cDTexture;
        private Texture2D cMTexture;
        private Texture2D l_DTexture;

        private MenuTab activeTab = MenuTab.DebugAndData;

        private int aeroStressIndex;
        private int atmBodyIndex = 1;

        private void Awake()
        {
            cLTexture = new Texture2D(25, 15);
            cDTexture = new Texture2D(25, 15);
            cMTexture = new Texture2D(25, 15);
            l_DTexture = new Texture2D(25, 15);
            GameEvents.onGameSceneSwitchRequested.Add(OnSceneChange);
            GameEvents.onGUIApplicationLauncherReady.Add(OnAppLauncherReadySetup);
        }

        private void Start()
        {
            FARAeroSection.GenerateCrossFlowDragCurve();
            FARAeroUtil.LoadAeroDataFromConfig();
            DontDestroyOnLoad(this);

            debugMenu = false;
        }

        private void Update()
        {
            if (!hasScenarioChanged)
                return;
            OnScenarioChanged();
            hasScenarioChanged = false;
        }

        // ReSharper disable once MemberCanBeMadeStatic.Local -> static does not work with GameEvents
        private void OnSceneChange(GameEvents.FromToAction<GameScenes, GameScenes> fromToScenes)
        {
            FARLogger.Info("check scene");
            if (fromToScenes.to == GameScenes.SPACECENTER)
            {
                if (!FARDebugValues.useBlizzyToolbar)
                    return;
                if (FARDebugButtonBlizzy != null)
                    return;
                FARDebugButtonBlizzy = ToolbarManager.Instance.add("FerramAerospaceResearch", "FARDebugButtonBlizzy");
                FARDebugButtonBlizzy.TexturePath = "FerramAerospaceResearch/Textures/icon_button_blizzy";
                FARDebugButtonBlizzy.ToolTip = "FAR Debug Options";
                FARDebugButtonBlizzy.OnClick += e => debugMenu = !debugMenu;
            }
            else
            {
                FARDebugButtonBlizzy?.Destroy();
            }
        }

        // ReSharper disable once MemberCanBeMadeStatic.Local -> static does not work with GameEvents
        private void OnAppLauncherReadySetup()
        {
            OnGUIAppLauncherReady();
            GameEvents.onGUIApplicationLauncherReady.Remove(OnAppLauncherReadySetup);
            Toggle += onAppLaunchToggle;
            Toggle += EditorGUI.onAppLaunchToggle;
            Toggle += FlightGUI.onAppLaunchToggle;
        }

        private static void ToggleGUI()
        {
            Toggle();
        }

        private static void OnGUIAppLauncherReady()
        {
            FARLogger.Info("Adding Debug Button");
            FARDebugButtonStock = ApplicationLauncher.Instance.AddModApplication(ToggleGUI,
                                                                                 ToggleGUI,
                                                                                 null,
                                                                                 null,
                                                                                 null,
                                                                                 null,
                                                                                 ApplicationLauncher
                                                                                     .AppScenes.SPACECENTER |
                                                                                 ApplicationLauncher.AppScenes.VAB |
                                                                                 ApplicationLauncher.AppScenes.SPH |
                                                                                 ApplicationLauncher.AppScenes.FLIGHT,
                                                                                 FARAssets.Instance.Textures.IconLarge);
        }

        private static void onAppLaunchToggle()
        {
            debugMenu = !debugMenu;
        }

        public static void ForceCloseDebugWindow()
        {
            if (FARDebugButtonStock)
                FARDebugButtonStock.SetFalse(false);
            debugMenu = false;
        }

        public void OnGUI()
        {
            if (HighLogic.LoadedScene != GameScenes.SPACECENTER)
            {
                debugMenu = false;
                if (!inputLocked)
                    return;
                InputLockManager.RemoveControlLock("FARDebugLock");
                inputLocked = false;
                return;
            }

            GUI.skin = HighLogic.Skin;
            if (debugMenu)
            {
                debugWinPos = GUILayout.Window("FARDebug".GetHashCode(),
                                               debugWinPos,
                                               debugWindow,
                                               "FAR Debug Options, " + Version.LongString,
                                               GUILayout.ExpandWidth(true),
                                               GUILayout.ExpandHeight(true));
                if (!inputLocked && debugWinPos.Contains(GUIUtils.GetMousePos()))
                {
                    InputLockManager.SetControlLock(ControlTypes.KSC_ALL, "FARDebugLock");
                    inputLocked = true;
                }
                else if (inputLocked && !debugWinPos.Contains(GUIUtils.GetMousePos()))
                {
                    InputLockManager.RemoveControlLock("FARDebugLock");
                    inputLocked = false;
                }
            }
            else if (inputLocked)
            {
                InputLockManager.RemoveControlLock("FARDebugLock");
                inputLocked = false;
            }
        }

        private void debugWindow(int windowID)
        {
            var thisStyle = new GUIStyle(GUI.skin.toggle)
            {
                stretchHeight = true,
                stretchWidth = true,
                padding = new RectOffset(4, 4, 4, 4),
                margin = new RectOffset(4, 4, 4, 4)
            };

            var buttonStyle = new GUIStyle(GUI.skin.button)
            {
                stretchHeight = true,
                stretchWidth = true,
                padding = new RectOffset(4, 4, 4, 4),
                margin = new RectOffset(4, 4, 4, 4)
            };

            var boxStyle = new GUIStyle(GUI.skin.box)
            {
                stretchHeight = true,
                stretchWidth = true,
                padding = new RectOffset(4, 4, 4, 4),
                margin = new RectOffset(4, 4, 4, 4)
            };

            activeTab = (MenuTab)GUILayout.SelectionGrid((int)activeTab, MenuTab_str, 3);

            switch (activeTab)
            {
                case MenuTab.DebugAndData:
                    DebugAndDataTab(thisStyle);
                    break;
                case MenuTab.AeroStress:
                    AeroStressTab(buttonStyle, boxStyle);
                    break;
                default:
                    AeroDataTab(buttonStyle, boxStyle);
                    break;
            }

            GUI.DragWindow();
            debugWinPos = GUIUtils.ClampToScreen(debugWinPos);
        }

        private void AeroDataTab(GUIStyle buttonStyle, GUIStyle boxStyle)
        {
            GUILayout.BeginVertical(boxStyle);

            FARControllableSurface.timeConstant =
                GUIUtils.TextEntryForDouble("Ctrl Surf Time Constant:", 160, FARControllableSurface.timeConstant);
            FARControllableSurface.timeConstantFlap =
                GUIUtils.TextEntryForDouble("Flap Time Constant:", 160, FARControllableSurface.timeConstantFlap);
            FARControllableSurface.timeConstantSpoiler =
                GUIUtils.TextEntryForDouble("Spoiler Time Constant:", 160, FARControllableSurface.timeConstantSpoiler);


            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            GUILayout.Label("Celestial Body Atmospheric Properties");

            GUILayout.BeginHorizontal();

            GUILayout.BeginVertical(boxStyle);

            GUILayout.BeginHorizontal();
            int j = 0;
            for (int i = 0; i < FlightGlobals.Bodies.Count; i++)
            {
                CelestialBody body = FlightGlobals.Bodies[i];

                if (!body.atmosphere)
                    continue;

                bool active = GUILayout.Toggle(i == atmBodyIndex,
                                               body.GetName(),
                                               buttonStyle,
                                               GUILayout.Width(150),
                                               GUILayout.Height(40));
                if (active)
                    atmBodyIndex = i;
                if ((j + 1) % 4 == 0)
                {
                    GUILayout.EndHorizontal();
                    GUILayout.BeginHorizontal();
                }

                j++;
            }

            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
            GUILayout.BeginVertical(boxStyle);

            int flightGlobalsIndex = FlightGlobals.Bodies[atmBodyIndex].flightGlobalsIndex;

            BodySettings atmProperties = FARAeroData.AtmosphericConfiguration[flightGlobalsIndex];

            atmProperties.ReferenceViscosity =
                GUIUtils.TextEntryForDouble("Gas Viscosity:", 80, atmProperties.ReferenceViscosity);
            atmProperties.ReferenceTemperature =
                GUIUtils.TextEntryForDouble("Ref Temp for Viscosity:", 80, atmProperties.ReferenceTemperature);

            FARAeroData.AtmosphericConfiguration[flightGlobalsIndex] = atmProperties;

            GUILayout.EndVertical();
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
        }

        private void AeroStressTab(GUIStyle buttonStyle, GUIStyle boxStyle)
        {
            int removeIndex = -1;
            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical(boxStyle);

            for (int i = 0; i < FARAeroStress.StressTemplates.Count; i++)
            {
                GUILayout.BeginHorizontal();
                bool active = GUILayout.Toggle(i == aeroStressIndex,
                                               FARAeroStress.StressTemplates[i].Name,
                                               buttonStyle,
                                               GUILayout.Width(150));
                if (GUILayout.Button("-", buttonStyle, GUILayout.Width(30), GUILayout.Height(30)))
                    removeIndex = i;
                GUILayout.EndHorizontal();
                if (active)
                    aeroStressIndex = i;
            }

            if (removeIndex >= 0)
            {
                FARAeroStress.StressTemplates.RemoveAt(removeIndex);
                if (aeroStressIndex == removeIndex && removeIndex > 0)
                    aeroStressIndex--;
            }

            if (GUILayout.Button("+", buttonStyle, GUILayout.Width(30), GUILayout.Height(30)))
            {
                var newTemplate = new FARPartStressTemplate
                {
                    XZMaxStress = 500,
                    YMaxStress = 500,
                    Name = "default",
                    IsSpecial = false,
                    Resources =
                    {
                        NumRequired = 0,
                        RejectUnlisted = false,
                        FlowMode = ResourceFlowMode.NO_FLOW
                    },
                    RequiresCrew = false,
                    FlowModeNeeded = false
                };

                FARAeroStress.StressTemplates.Add(newTemplate);
            }

            GUILayout.EndVertical();
            GUILayout.BeginVertical(boxStyle);

            FARPartStressTemplate activeTemplate = FARAeroStress.StressTemplates[aeroStressIndex];

            GUIUtils.TextEntryField("Name:", 80, ref activeTemplate.Name);

            activeTemplate.YMaxStress =
                GUIUtils.TextEntryForDouble("Axial (Y-axis) Max Stress:", 240, activeTemplate.YMaxStress);
            activeTemplate.XZMaxStress =
                GUIUtils.TextEntryForDouble("Lateral (X,Z-axis) Max Stress:", 240, activeTemplate.XZMaxStress);

            activeTemplate.RequiresCrew = GUILayout.Toggle(activeTemplate.RequiresCrew, "Requires Crew Compartment");

            string tmp = activeTemplate.Resources.NumRequired.ToString();
            GUIUtils.TextEntryField("Min Num Resources:", 80, ref tmp);
            tmp = Regex.Replace(tmp, @"[^\d]", "");
            activeTemplate.Resources.NumRequired = Convert.ToInt32(tmp);

            GUILayout.Label("Req Resources:");
            StringListUpdateGUI(activeTemplate.Resources.Resources, buttonStyle, boxStyle);

            GUILayout.Label("Exclude Resources:");
            StringListUpdateGUI(activeTemplate.Resources.Excluded, buttonStyle, boxStyle);

            activeTemplate.Resources.RejectUnlisted =
                GUILayout.Toggle(activeTemplate.Resources.RejectUnlisted, "Reject Unlisted Res");

            activeTemplate.FlowModeNeeded =
                GUILayout.Toggle(activeTemplate.FlowModeNeeded, "Requires Specific Flow Mode");
            if (activeTemplate.FlowModeNeeded)
                activeTemplate.Resources.FlowMode =
                    (ResourceFlowMode)GUILayout.SelectionGrid((int)activeTemplate.Resources.FlowMode, FlowMode_str, 1);

            activeTemplate.IsSpecial = GUILayout.Toggle(activeTemplate.IsSpecial, "Special Hardcoded Usage");

            GUILayout.EndVertical();
            GUILayout.EndHorizontal();
        }

        private static void StringListUpdateGUI(List<string> stringList, GUIStyle thisStyle, GUIStyle boxStyle)
        {
            int removeIndex = -1;
            GUILayout.BeginVertical(boxStyle);
            for (int i = 0; i < stringList.Count; i++)
            {
                string tmp = stringList[i];
                GUILayout.BeginHorizontal();
                tmp = GUILayout.TextField(tmp, GUILayout.Height(30));
                if (GUILayout.Button("-", thisStyle, GUILayout.Width(30), GUILayout.Height(30)))
                    removeIndex = i;
                GUILayout.EndHorizontal();
                if (removeIndex >= 0)
                    break;

                stringList[i] = tmp;
            }

            if (removeIndex >= 0)
                stringList.RemoveAt(removeIndex);
            if (GUILayout.Button("+", thisStyle, GUILayout.Width(30), GUILayout.Height(30)))
                stringList.Add("");

            GUILayout.EndVertical();
        }

        private void DebugAndDataTab(GUIStyle thisStyle)
        {
            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical(GUILayout.Width(250));
            GUILayout.Label("Debug / Cheat Options");
            FARDebugValues.allowStructuralFailures =
                GUILayout.Toggle(FARDebugValues.allowStructuralFailures, "Allow Aero-structural Failures", thisStyle);

            GUILayout.Label("Editor GUI Graph Colors");


            Color tmpColor = FARConfig.GUIColors.ClColor;
            ChangeColor("Cl", ref tmpColor, ref cLTexture);
            FARConfig.GUIColors.ClColor = tmpColor;

            tmpColor = FARConfig.GUIColors.CdColor;
            ChangeColor("Cd", ref tmpColor, ref cDTexture);
            FARConfig.GUIColors.CdColor = tmpColor;

            tmpColor = FARConfig.GUIColors.CmColor;
            ChangeColor("Cm", ref tmpColor, ref cMTexture);
            FARConfig.GUIColors.CmColor = tmpColor;

            tmpColor = FARConfig.GUIColors.LdColor;
            ChangeColor("L_D", ref tmpColor, ref l_DTexture);
            FARConfig.GUIColors.LdColor = tmpColor;

            FARActionGroupConfiguration.DrawGUI();
            GUILayout.Label("Other Options"); // DaMichel: put it above the toolbar toggle
            FARDebugValues.aeroFailureExplosions =
                GUILayout.Toggle(FARDebugValues.aeroFailureExplosions, "Aero Failures Create Explosions", thisStyle);
            FARDebugValues.showMomentArrows =
                GUILayout.Toggle(FARDebugValues.showMomentArrows, "Show Torque Arrows in Aero Overlay", thisStyle);
            if (ToolbarManager.ToolbarAvailable)
            {
                bool tmp = FARDebugValues.useBlizzyToolbar;
                FARDebugValues.useBlizzyToolbar = GUILayout.Toggle(FARDebugValues.useBlizzyToolbar,
                                                                   "Use Blizzy78 Toolbar instead of Stock AppManager",
                                                                   thisStyle);

                if (tmp != FARDebugValues.useBlizzyToolbar)
                {
                    FARDebugButtonBlizzy?.Destroy();

                    if (FARDebugValues.useBlizzyToolbar)
                    {
                        FARDebugButtonBlizzy = ToolbarManager.Instance.add("ferram4", "FARDebugButtonBlizzy");
                        FARDebugButtonBlizzy.TexturePath = "FerramAerospaceResearch/Textures/icon_button_blizzy";
                        FARDebugButtonBlizzy.ToolTip = "FAR Debug Options";
                        FARDebugButtonBlizzy.OnClick += e => debugMenu = !debugMenu;
                    }
                }
            }

            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            FARSettingsScenarioModule.Instance.DisplaySelection();

            GUILayout.EndVertical();
            GUILayout.EndHorizontal();
        }

        private static void ChangeColor(string colorTitle, ref Color input, ref Texture2D texture)
        {
            GUILayout.BeginHorizontal();

            GUILayout.Label(colorTitle + " (r,g,b):", GUILayout.Width(80));

            bool updateTexture = false;

            GUILayout.BeginHorizontal(GUILayout.Width(100));
            float tmp = input.r;
            input.r = (float)GUIUtils.TextEntryForDouble("", 0, input.r);
            updateTexture |= !tmp.NearlyEqual(input.r);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal(GUILayout.Width(100));
            tmp = input.g;
            input.g = (float)GUIUtils.TextEntryForDouble("", 0, input.g);
            updateTexture |= !tmp.NearlyEqual(input.g);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal(GUILayout.Width(100));
            tmp = input.b;
            input.b = (float)GUIUtils.TextEntryForDouble("", 0, input.b);
            updateTexture |= !tmp.NearlyEqual(input.b);
            GUILayout.EndHorizontal();

            if (updateTexture)
                ReColorTexture(input, ref texture);

            Rect textRect = GUILayoutUtility.GetRect(25, 15);
            textRect.Set(textRect.x, textRect.y + 10, textRect.width, textRect.height);
            GUI.DrawTexture(textRect, texture);
            GUILayout.EndHorizontal();
        }

        private static void ReColorTexture(Color color, ref Texture2D texture)
        {
            for (int i = 0; i < texture.width; i++)
                for (int j = 0; j < texture.height; j++)
                    texture.SetPixel(i, j, color);

            texture.Apply();
        }

        public static void LoadConfigs(ConfigNode node)
        {
            config = PluginConfiguration.CreateForType<FARSettingsScenarioModule>();
            config.load();

            if (node.HasValue("allowStructuralFailures") &&
                bool.TryParse(node.GetValue("allowStructuralFailures"), out bool tmp))
                FARDebugValues.allowStructuralFailures = tmp;
            else
                FARDebugValues.allowStructuralFailures = true;

            if (node.HasValue("showMomentArrows") && bool.TryParse(node.GetValue("showMomentArrows"), out tmp))
                FARDebugValues.showMomentArrows = tmp;
            else
                FARDebugValues.showMomentArrows = false;

            if (node.HasValue("useBlizzyToolbar") && bool.TryParse(node.GetValue("useBlizzyToolbar"), out tmp))
                FARDebugValues.useBlizzyToolbar = tmp;
            else
                FARDebugValues.useBlizzyToolbar = false;

            if (node.HasValue("aeroFailureExplosions") &&
                bool.TryParse(node.GetValue("aeroFailureExplosions"), out tmp))
                FARDebugValues.aeroFailureExplosions = tmp;
            else
                FARDebugValues.aeroFailureExplosions = true;

            FARAeroUtil.LoadAeroDataFromConfig();
            FARActionGroupConfiguration.LoadConfiguration();

            hasScenarioChanged = true;
        }

        /// <summary> Update GUI after a new scenario was loaded. </summary>
        private void OnScenarioChanged()
        {
            ReColorTexture(FARConfig.GUIColors.ClColor, ref cLTexture);
            ReColorTexture(FARConfig.GUIColors.CdColor, ref cDTexture);
            ReColorTexture(FARConfig.GUIColors.CmColor, ref cMTexture);
            ReColorTexture(FARConfig.GUIColors.LdColor, ref l_DTexture);
        }

        public static void SaveConfigs(ConfigNode node)
        {
            node.AddValue("allowStructuralFailures", FARDebugValues.allowStructuralFailures);
            node.AddValue("showMomentArrows", FARDebugValues.showMomentArrows);
            node.AddValue("useBlizzyToolbar", FARDebugValues.useBlizzyToolbar & ToolbarManager.ToolbarAvailable);
            node.AddValue("aeroFailureExplosions", FARDebugValues.aeroFailureExplosions);

            FARActionGroupConfiguration.SaveConfiguration();
            // save in off thread so that it doesn't delay scene changes
            MainThread.StartCoroutine(ConfigAdapter.SaveAll);
            config.save();
        }

        private void OnDestroy()
        {
            GameEvents.onGameSceneSwitchRequested.Remove(OnSceneChange);

            FARDebugButtonBlizzy?.Destroy();
        }

        // ReSharper disable once UnusedMember.Local -> AtmComposition
        private enum MenuTab
        {
            DebugAndData,
            AeroStress,
            AtmComposition
        }
    }

    public static class FARDebugValues
    {
        //Right-click menu options
        public static bool allowStructuralFailures = true;
        public static bool showMomentArrows;

        public static bool useBlizzyToolbar;
        public static bool aeroFailureExplosions = true;
    }
}
