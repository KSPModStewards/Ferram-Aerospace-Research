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
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.Geometry;
using FerramAerospaceResearch.Reflection;
using UniLinq;
using UnityEngine;

namespace FerramAerospaceResearch
{
    [KSPScenario(ScenarioCreationOptions.AddToAllGames, GameScenes.SPACECENTER, GameScenes.EDITOR, GameScenes.FLIGHT)]
    public class FARSettingsScenarioModule : ScenarioModule
    {
        [ConfigNode("FARDifficultyPresets", shouldSave: false, isRoot: true)]
        public static class FARDifficultyPresetsConfig
        {
            // reflection shows lots of false positives...

            // ReSharper disable once FieldCanBeMadeReadOnly.Global
            // ReSharper disable once ConvertToConstant.Global
            [ConfigValue("index")] public static int Index = 4;

            // ReSharper disable once CollectionNeverUpdated.Global
            [ConfigValue] public static readonly List<FARDifficultyAndExactnessSettings> Values = new();

            [ConfigValueIgnore] public static string[] Names { get; private set; }

            public static void AfterLoaded()
            {
                Names = Values.Select(preset => preset.name).ToArray();
                for (int i = 0; i < Values.Count; ++i)
                    Values[i].index = i;
            }
        }

        public bool newGame;

        public FARDifficultyAndExactnessSettings settings;
        public FARDifficultyAndExactnessSettings customSettings;
        public FARVoxelSettings voxelSettings;

        public List<ConfigNode> flightGUISettings;

        public int currentIndex;

        private GUIDropDown<FARDifficultyAndExactnessSettings> dropdown;


        private FARSettingsScenarioModule()
        {
            Instance = this;
        }

        public static FARDifficultyAndExactnessSettings Settings
        {
            get { return Instance.settings; }
        }

        public static FARVoxelSettings VoxelSettings
        {
            get { return Instance.voxelSettings; }
        }

        public static List<ConfigNode> FlightGUISettings
        {
            get { return Instance.flightGUISettings; }
        }

        public static FARSettingsScenarioModule Instance { get; private set; }

        public static void MainMenuBuildDefaultScenarioModule()
        {
            if (Instance != null)
                return;
            Instance = new GameObject().AddComponent<FARSettingsScenarioModule>();
            DontDestroyOnLoad(Instance);
            FARLogger.Info("Creating new setting module for tutorial/scenario");
            Instance.OnLoad(new ConfigNode());
        }

        private void Start()
        {
            Instance = this;

            FARLogger.Info("Vehicle Voxel Setup started");
            FARAeroSection.GenerateCrossFlowDragCurve();
            VehicleVoxel.VoxelSetup();
            PhysicsGlobals.DragCubeMultiplier = 0;
            FARLogger.Info("Vehicle Voxel Setup complete");

            newGame = false;
        }

        public override void OnSave(ConfigNode node)
        {
            FARLogger.Info("saved");
            node.AddValue("newGame", newGame);
            node.AddValue("fractionTransonicDrag", settings.fractionTransonicDrag);
            node.AddValue("gaussianVehicleLengthFractionForSmoothing",
                          settings.gaussianVehicleLengthFractionForSmoothing);
            node.AddValue("numAreaSmoothingPasses", settings.numAreaSmoothingPasses);
            node.AddValue("numDerivSmoothingPasses", settings.numDerivSmoothingPasses);
            node.AddValue("numVoxelsControllableVessel", voxelSettings.numVoxelsControllableVessel);
            node.AddValue("numVoxelsDebrisVessel", voxelSettings.numVoxelsDebrisVessel);
            node.AddValue("minPhysTicksPerUpdate", voxelSettings.minPhysTicksPerUpdate);
            node.AddValue("useHigherResVoxelPoints", voxelSettings.useHigherResVoxelPoints);
            node.AddValue("use32BitIndices", DebugVoxelMesh.Use32BitIndices);
            node.AddValue("index", settings.index);

            FlightGUI.SaveActiveData();
            var flightGUINode = new ConfigNode("FlightGUISettings");
            FARLogger.Info("Saving FAR Data");
            foreach (ConfigNode configNode in flightGUISettings)
                flightGUINode.AddNode(configNode);
            node.AddNode(flightGUINode);

            FARDebugAndSettings.SaveConfigs(node);
        }

        public override void OnLoad(ConfigNode node)
        {
            Instance = this;
            int index = FARDifficultyPresetsConfig.Index;
            if (node.HasValue("newGame"))
                newGame = bool.Parse(node.GetValue("newGame"));

            if (node.HasValue("index"))
                index = int.Parse(node.GetValue("index"));

            dropdown = new GUIDropDown<FARDifficultyAndExactnessSettings>(FARDifficultyPresetsConfig.Names,
                                                                          FARDifficultyPresetsConfig.Values.ToArray(),
                                                                          index < 0 ? 2 : index);
            voxelSettings = new FARVoxelSettings();

            if (node.HasValue("numVoxelsControllableVessel"))
                voxelSettings.numVoxelsControllableVessel = int.Parse(node.GetValue("numVoxelsControllableVessel"));
            if (node.HasValue("numVoxelsDebrisVessel"))
                voxelSettings.numVoxelsDebrisVessel = int.Parse(node.GetValue("numVoxelsDebrisVessel"));
            if (node.HasValue("minPhysTicksPerUpdate"))
                voxelSettings.minPhysTicksPerUpdate = int.Parse(node.GetValue("minPhysTicksPerUpdate"));
            if (node.HasValue("useHigherResVoxelPoints"))
                voxelSettings.useHigherResVoxelPoints = bool.Parse(node.GetValue("useHigherResVoxelPoints"));
            if (node.HasValue("use32BitIndices"))
                DebugVoxelMesh.Use32BitIndices = bool.Parse(node.GetValue("use32BitIndices"));

            if (index == -1)
            {
                settings = new FARDifficultyAndExactnessSettings(index);

                if (node.HasValue("fractionTransonicDrag"))
                    settings.fractionTransonicDrag = double.Parse(node.GetValue("fractionTransonicDrag"));
                if (node.HasValue("gaussianVehicleLengthFractionForSmoothing"))
                    settings.gaussianVehicleLengthFractionForSmoothing =
                        double.Parse(node.GetValue("gaussianVehicleLengthFractionForSmoothing"));

                if (node.HasValue("numAreaSmoothingPasses"))
                    settings.numAreaSmoothingPasses = int.Parse(node.GetValue("numAreaSmoothingPasses"));
                if (node.HasValue("numDerivSmoothingPasses"))
                    settings.numDerivSmoothingPasses = int.Parse(node.GetValue("numDerivSmoothingPasses"));

                customSettings = settings;
            }
            else
            {
                settings = FARDifficultyPresetsConfig.Values[index];
                customSettings = new FARDifficultyAndExactnessSettings(-1);
            }

            currentIndex = index;

            FARLogger.Info("Loading FAR Data");
            flightGUISettings = new List<ConfigNode>();
            if (node.HasNode("FlightGUISettings"))
                foreach (ConfigNode flightGUINode in node.GetNode("FlightGUISettings").nodes)
                    flightGUISettings.Add(flightGUINode);

            FARDebugAndSettings.LoadConfigs(node);
        }

        public void DisplaySelection()
        {
            GUILayout.BeginVertical();
            GUILayout.Label("Transonic Drag Settings");
            GUILayout.Label("Absolute magnitude of drag can be scaled, as can how lenient FAR is about enforcing proper area ruling.");

            GUILayout.BeginHorizontal();
            if (currentIndex >= 0)
            {
                dropdown.GUIDropDownDisplay(GUILayout.Width(300));
                settings = dropdown.ActiveSelection;
                currentIndex = settings.index;
            }
            else
            {
                GUILayout.BeginVertical();
                settings = customSettings;
                settings.fractionTransonicDrag =
                    GUIUtils.TextEntryForDouble("Frac Mach 1 Drag: ", 150, settings.fractionTransonicDrag);
                GUILayout.Label("The below are used in controlling leniency of design.  Higher values for all will result in more leniency");
                settings.gaussianVehicleLengthFractionForSmoothing =
                    GUIUtils.TextEntryForDouble("% Vehicle Length for Smoothing",
                                                250,
                                                settings.gaussianVehicleLengthFractionForSmoothing);
                settings.numAreaSmoothingPasses =
                    GUIUtils.TextEntryForInt("Smoothing Passes, Cross-Sectional Area",
                                             250,
                                             settings.numAreaSmoothingPasses);
                if (settings.numAreaSmoothingPasses < 0)
                    settings.numAreaSmoothingPasses = 0;
                settings.numDerivSmoothingPasses =
                    GUIUtils.TextEntryForInt("Smoothing Passes, area 2nd deriv", 250, settings.numDerivSmoothingPasses);
                if (settings.numDerivSmoothingPasses < 0)
                    settings.numDerivSmoothingPasses = 0;

                customSettings = settings;
                GUILayout.EndVertical();
            }

            if (GUILayout.Button(currentIndex < 0 ? "Switch Back To Presets" : "Choose Custom Settings"))
            {
                if (currentIndex >= 0)
                    currentIndex = -1;
                else
                    currentIndex = FARDifficultyPresetsConfig.Index;
            }

            GUILayout.EndHorizontal();
            GUILayout.Label("Voxel Detail Settings; increasing these will improve accuracy at the cost of performance");

            voxelSettings.numVoxelsControllableVessel =
                GUIUtils.TextEntryForInt("Voxels Controllable Vessel: ",
                                         200,
                                         voxelSettings.numVoxelsControllableVessel);
            if (voxelSettings.numVoxelsControllableVessel < 0)
                voxelSettings.numVoxelsControllableVessel = 100000;

            voxelSettings.numVoxelsDebrisVessel =
                GUIUtils.TextEntryForInt("Voxels Debris: ", 200, voxelSettings.numVoxelsDebrisVessel);
            if (voxelSettings.numVoxelsDebrisVessel < 0)
                voxelSettings.numVoxelsDebrisVessel = 5000;

            voxelSettings.minPhysTicksPerUpdate =
                GUIUtils.TextEntryForInt("Min Phys Ticks per Voxel Update: ", 200, voxelSettings.minPhysTicksPerUpdate);
            if (voxelSettings.minPhysTicksPerUpdate < 0)
                voxelSettings.minPhysTicksPerUpdate = 80;

            GUILayout.BeginHorizontal();
            GUILayout.Label("Use Higher Res SubVoxels: ");
            voxelSettings.useHigherResVoxelPoints = GUILayout.Toggle(voxelSettings.useHigherResVoxelPoints,
                                                                     voxelSettings.useHigherResVoxelPoints
                                                                         ? "High Res SubVoxels"
                                                                         : "Low Res SubVoxels");
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Mesh: use 32 bit indices: ");
            DebugVoxelMesh.Use32BitIndices = GUILayout.Toggle(DebugVoxelMesh.Use32BitIndices,
                                                              DebugVoxelMesh.Use32BitIndices
                                                                  ? "32 bit indices"
                                                                  : "16 bit indices");
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
        }
    }

    [ConfigNode("FARDifficultyAndExactnessSettings", shouldSave: false, isRoot: false, allowMultiple: true)]
    public class FARDifficultyAndExactnessSettings
    {
        // TODO: index should be the index in the list, allow multiple custom settings
        [ConfigValueIgnore] public int index = -1;
        [ConfigValue] public double fractionTransonicDrag = 0.7;
        [ConfigValue] public double gaussianVehicleLengthFractionForSmoothing = 0.015;
        [ConfigValue] public int numAreaSmoothingPasses = 2;
        [ConfigValue] public int numDerivSmoothingPasses = 1;
        [ConfigValue] public string name = string.Empty;

        public FARDifficultyAndExactnessSettings()
        {
        }

        public FARDifficultyAndExactnessSettings(int index)
        {
            this.index = index;
        }

        public FARDifficultyAndExactnessSettings(
            double transDrag,
            double gaussianLength,
            int areaPass,
            int derivPass,
            int index,
            string name = ""
        )
        {
            this.index = index;
            fractionTransonicDrag = transDrag;
            gaussianVehicleLengthFractionForSmoothing = gaussianLength;
            numAreaSmoothingPasses = areaPass;
            numDerivSmoothingPasses = derivPass;
            this.name = name;
        }
    }

    public class FARVoxelSettings
    {
        public int numVoxelsControllableVessel;
        public int numVoxelsDebrisVessel;
        public int minPhysTicksPerUpdate;
        public bool useHigherResVoxelPoints;

        public FARVoxelSettings() : this(250000, 20000, 80, true)
        {
        }

        public FARVoxelSettings(int vesselCount, int debrisCount, int minPhysTicks, bool higherResVoxPoints)
        {
            numVoxelsControllableVessel = vesselCount;
            numVoxelsDebrisVessel = debrisCount;
            minPhysTicksPerUpdate = minPhysTicks;
            useHigherResVoxelPoints = higherResVoxPoints;
        }
    }
}
