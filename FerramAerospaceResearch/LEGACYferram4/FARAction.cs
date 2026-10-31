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
using FerramAerospaceResearch;
using FerramAerospaceResearch.FARGUI;
using KSP.IO;
using KSP.Localization;
using UnityEngine;

// ReSharper disable once CheckNamespace
namespace ferram4
{
    //Added by DaMichel:
    public class FARAction : KSPAction
    {
        // Is constructed each time a part module instance is created.
        // The current AG seems to be stored elsewhere so base.actionGroup
        // is only used for the initial assignment.
        public FARAction(string guiName, int actionIdentifier) : base(guiName)
        {
            actionGroup = FARActionGroupConfiguration.map(actionIdentifier);
        }
    }


    public static class FARActionGroupConfiguration
    {
        public const int ID_SPOILER = 0;
        public const int ID_INCREASE_FLAP_DEFLECTION = 1;
        public const int ID_DECREASE_FLAP_DEFLECTION = 2;
        public const int ACTION_COUNT = 3;

        // private lookup tables
        private static readonly KSPActionGroup[] id2actionGroup =
        {
            KSPActionGroup.Brakes, KSPActionGroup.None, KSPActionGroup.None
        };

        // keys in the configuration file
        private static readonly string[] configKeys =
        {
            "actionGroupSpoiler", "actionGroupIncreaseFlapDeflection", "actionGroupDecreaseFlapDeflection"
        };

        // for the gui
        private static readonly string[] guiLabels =
        {
            Localizer.Format("FARActionSpoilers"),
            Localizer.Format("FARActionIncreaseFlap"),
            Localizer.Format("FARActionDecreaseFlap")
        };

        private static readonly string[] currentGuiStrings =
        {
            id2actionGroup[0].ToString(), id2actionGroup[1].ToString(), id2actionGroup[2].ToString()
        };

        private static GUIDropDown<KSPActionGroup>[] actionGroupDropDown;

        public static KSPActionGroup map(int id)
        {
            return id2actionGroup[id];
        }

        public static void LoadConfiguration()
        {
            string[] namesTmp = Enum.GetNames(typeof(KSPActionGroup));
            var names = new string[namesTmp.Length - 1];

            for (int i = 0; i < namesTmp.Length - 1; ++i)
                names[i] = namesTmp[i];
            var agTypes = new KSPActionGroup[names.Length];
            actionGroupDropDown = new GUIDropDown<KSPActionGroup>[3];

            for (int i = 0; i < agTypes.Length; i++)
                agTypes[i] = (KSPActionGroup)Enum.Parse(typeof(KSPActionGroup), names[i]);
            // straight forward, reading the (action name, action group) tuples
            PluginConfiguration config = FARDebugAndSettings.config;
            for (int i = 0; i < ACTION_COUNT; ++i)
            {
                try
                {
                    // don't forget to initialize the gui
                    string currentGuiString = currentGuiStrings[i] = id2actionGroup[i].ToString();
                    id2actionGroup[i] =
                        (KSPActionGroup)Enum.Parse(typeof(KSPActionGroup),
                                                   config.GetValue(configKeys[i], currentGuiString));
                    FARLogger.Info($"Loaded AG {configKeys[i]} as {currentGuiString}");
                }
                catch (Exception e)
                {
                    FARLogger.Warning("Error reading config key '" +
                                      configKeys[i] +
                                      "' with value '" +
                                      config.GetValue(configKeys[i], "n/a") +
                                      "' gave " +
                                      e);
                }

                int initIndex = 0;
                for (int j = 0; j < agTypes.Length; j++)
                {
                    if (id2actionGroup[i] != agTypes[j])
                        continue;
                    initIndex = j;
                    break;
                }

                var dropDown = new GUIDropDown<KSPActionGroup>(names, agTypes, initIndex);
                actionGroupDropDown[i] = dropDown;
            }
        }

        public static void SaveConfiguration()
        {
            PluginConfiguration config = FARDebugAndSettings.config;
            for (int i = 0; i < ACTION_COUNT; ++i)
            {
                FARLogger.Info($"Save AG {configKeys[i]} as {id2actionGroup[i].ToString()}");
                config.SetValue(configKeys[i], id2actionGroup[i].ToString());
            }
        }

        public static void DrawGUI()
        {
            var label = new GUIStyle(GUI.skin.label) {normal = {textColor = GUI.skin.toggle.normal.textColor}};
            GUILayout.Label(Localizer.Format("FARActionDefaultLabel"));
            // left column: label, right column: text field
            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical();
            for (int i = 0; i < ACTION_COUNT; ++i)
                GUILayout.Label(guiLabels[i], label);
            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            for (int i = 0; i < ACTION_COUNT; ++i)
            {
                actionGroupDropDown[i].GUIDropDownDisplay(GUILayout.Width(150));
                id2actionGroup[i] = actionGroupDropDown[i].ActiveSelection;
            }

            GUILayout.EndVertical();
            GUILayout.EndHorizontal(); // end of columns
        }
    }
}
