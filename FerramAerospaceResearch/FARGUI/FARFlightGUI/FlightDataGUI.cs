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
using System.Linq;
using System.Text;
using KSP.Localization;
using StringLeakTest;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FARFlightGUI
{
    internal class FlightDataGUI
    {
        private readonly bool[] activeFlightDataSections = {true, true, true, true, true, true, true, true, true};
        private readonly bool[] oldFlightDataSections = {false, false, false, false, false, false, false, false, false};

        private readonly string[] flightDataOptionLabels =
        {
            Localizer.Format("FARFlightDataOption0"),
            Localizer.Format("FARFlightDataOption1"),
            Localizer.Format("FARFlightDataOption2"),
            Localizer.Format("FARFlightDataOption3"),
            Localizer.Format("FARFlightDataOption4"),
            Localizer.Format("FARFlightDataOption5"),
            Localizer.Format("FARFlightDataOption6"),
            Localizer.Format("FARFlightDataOption7"),
            Localizer.Format("FARFlightDataOption8")
        };

        private readonly StringBuilder dataStringBuilder = new StringBuilder();
        private readonly StringBuilder labelStringBuilder = new StringBuilder();

        private VesselFlightInfo infoParameters;

        private GUIStyle buttonStyle;
        private GUIStyle boxStyle;

        private int thisFrame;

        public FlightDataGUI()
        {
            LoadSettings();
        }

        public void UpdateInfoParameters(VesselFlightInfo info)
        {
            infoParameters = info;
        }


        private void CreateLabelString()
        {
            bool change = false;
            for (int i = 0; i < activeFlightDataSections.Length; ++i)
                change |= oldFlightDataSections[i] == activeFlightDataSections[i];
            //no need to recreate string, we still have one, and the settings have not been changed.
            if (!change && labelStringBuilder.Length != 0)
                return;
            for (int i = 0; i < activeFlightDataSections.Length; ++i)
                oldFlightDataSections[i] = activeFlightDataSections[i];

            labelStringBuilder.Length = 0;
            labelStringBuilder.AppendLine();
            //PYR angles
            if (activeFlightDataSections[0])
            {
                labelStringBuilder.AppendLine(Localizer.Format("FARFlightData0"));
                labelStringBuilder.AppendLine();
            }

            //AoA and sideslip
            if (activeFlightDataSections[1])
            {
                labelStringBuilder.AppendLine(Localizer.Format("FARFlightData1"));
                labelStringBuilder.AppendLine();
            }

            //Dyn pres
            if (activeFlightDataSections[2])
            {
                labelStringBuilder.AppendLine(Localizer.Format("FARFlightData2"));
                labelStringBuilder.AppendLine();
            }

            //Raw Forces
            if (activeFlightDataSections[3])
            {
                labelStringBuilder.AppendLine(Localizer.Format("FARFlightData3"));
                labelStringBuilder.AppendLine();
            }

            //Coeffs + refArea
            if (activeFlightDataSections[4])
            {
                labelStringBuilder.AppendLine(Localizer.Format("FARFlightData4"));
                labelStringBuilder.AppendLine();
            }

            //L/D and VL/D
            if (activeFlightDataSections[5])
            {
                labelStringBuilder.AppendLine(Localizer.Format("FARFlightData5"));
                labelStringBuilder.AppendLine();
            }

            //Engine and intake data; Thrust Specific Fuel Consumption and Specific Excess Power
            if (activeFlightDataSections[6])
            {
                labelStringBuilder.AppendLine(Localizer.Format("FARFlightData6"));
                labelStringBuilder.AppendLine();
            }

            //Range, Endurance est
            if (activeFlightDataSections[7])
            {
                labelStringBuilder.AppendLine(Localizer.Format("FARFlightData7"));
                labelStringBuilder.AppendLine();
            }

            // ReSharper disable once InvertIf
            //Ballistic Coeff and Term Vel
            if (activeFlightDataSections[8])
            {
                labelStringBuilder.AppendLine(Localizer.Format("FARFlightData8"));
                labelStringBuilder.AppendLine();
            }
        }

        private void CreateDataString()
        {
            dataStringBuilder.Length = 0;
            dataStringBuilder.AppendLine();
            //PYR angles
            if (activeFlightDataSections[0])
            {
                dataStringBuilder.Concat((float)infoParameters.pitchAngle, 1);
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitDeg"));
                dataStringBuilder.Concat((float)infoParameters.headingAngle, 1);
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitDeg"));
                dataStringBuilder.Concat((float)infoParameters.rollAngle, 1);
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitDeg"));
                dataStringBuilder.AppendLine();
            }

            //AoA and sideslip
            if (activeFlightDataSections[1])
            {
                dataStringBuilder.Concat((float)infoParameters.aoA, 1);
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitDeg"));
                dataStringBuilder.Concat((float)infoParameters.sideslipAngle, 1);
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitDeg"));
                dataStringBuilder.AppendLine();
            }

            //Dyn pres
            if (activeFlightDataSections[2])
            {
                dataStringBuilder.Concat((float)infoParameters.dynPres, 3);
                dataStringBuilder.Append(" ");
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitKPa"));
                dataStringBuilder.AppendLine();
            }

            //Raw Forces
            if (activeFlightDataSections[3])
            {
                dataStringBuilder.Concat((float)infoParameters.liftForce, 3);
                dataStringBuilder.Append(" ");
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitKN"));
                dataStringBuilder.Concat((float)infoParameters.dragForce, 3);
                dataStringBuilder.Append(" ");
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitKN"));
                dataStringBuilder.Concat((float)infoParameters.sideForce, 3);
                dataStringBuilder.Append(" ");
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitKN"));
                dataStringBuilder.AppendLine();
            }

            //Coeffs + refArea
            if (activeFlightDataSections[4])
            {
                dataStringBuilder.Concat((float)infoParameters.liftCoeff, 4).AppendLine();
                dataStringBuilder.Concat((float)infoParameters.dragCoeff, 4).AppendLine();
                dataStringBuilder.Concat((float)infoParameters.sideCoeff, 4).AppendLine();
                dataStringBuilder.Concat((float)infoParameters.refArea, 3);
                dataStringBuilder.Append(" ");
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitMSq"));
                dataStringBuilder.AppendLine();
            }

            //L/D and VL/D
            if (activeFlightDataSections[5])
            {
                dataStringBuilder.Concat((float)infoParameters.liftToDragRatio, 3).AppendLine();
                dataStringBuilder.Concat((float)infoParameters.velocityLiftToDragRatio, 3).AppendLine();
                dataStringBuilder.AppendLine();
            }

            //Engine and intake data
            if (activeFlightDataSections[6])
            {
                dataStringBuilder
                    .Concat((float)((infoParameters.fullMass - infoParameters.dryMass) / infoParameters.fullMass), 2)
                    .AppendLine();
                dataStringBuilder.Concat((float)infoParameters.tSFC, 3);
                dataStringBuilder.Append(" ");
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitInvHr"));
                if (double.IsInfinity(infoParameters.intakeAirFrac))
                {
                    dataStringBuilder.AppendLine("Infinity");
                }
                else
                {
                    //Note: Originally this was output using P1 format, leading to an effective factor of 100*100.
                    dataStringBuilder.Concat((float)(infoParameters.intakeAirFrac * 100), 1);
                    dataStringBuilder.AppendLine(Localizer.Format("FARUnitPercent"));
                }

                //this is a noticeable change to original code: Here N2 format was used...
                dataStringBuilder.Concat((float)infoParameters.specExcessPower, 2);
                dataStringBuilder.Append(" ");
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitSpecPower"));
                dataStringBuilder.AppendLine();
            }

            //Range, Endurance est
            if (activeFlightDataSections[7])
            {
                dataStringBuilder.Concat((float)infoParameters.endurance, 2);
                dataStringBuilder.Append(" ");
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitHr"));
                //also here: original N2 format.
                dataStringBuilder.Concat((float)infoParameters.range, 2);
                dataStringBuilder.Append(" ");
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitkM"));
                dataStringBuilder.AppendLine();
            }

            // ReSharper disable once InvertIf
            //Ballistic Coeff and Term Vel
            if (activeFlightDataSections[8])
            {
                dataStringBuilder.Concat((float)infoParameters.ballisticCoeff, 2);
                dataStringBuilder.Append(" ");
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitBC"));
                dataStringBuilder.Concat((float)infoParameters.termVelEst, 2);
                dataStringBuilder.Append(" ");
                dataStringBuilder.AppendLine(Localizer.Format("FARUnitMPerSec"));
                dataStringBuilder.AppendLine();
            }
        }

        public void DataDisplay()
        {
            if (boxStyle == null)
                boxStyle = FlightGUI.boxStyle;
            if (Time.frameCount != thisFrame)
            {
                thisFrame = Time.frameCount;
                CreateLabelString();
                CreateDataString();
            }

            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical();
            GUILayout.Box(labelStringBuilder.ToString(), boxStyle, GUILayout.Width(140));
            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            GUILayout.Box(dataStringBuilder.ToString(), boxStyle, GUILayout.Width(140));
            GUILayout.EndVertical();
            GUILayout.EndHorizontal();
        }

        //Returns true on a setting change
        public bool SettingsDisplay()
        {
            if (buttonStyle == null)
                buttonStyle = FlightGUI.buttonStyle;

            GUILayout.Label(Localizer.Format("FARFlightDataOptionLabel"));
            GUILayout.BeginVertical();
            bool change = false;
            for (int i = 0; i < activeFlightDataSections.Length; i++)
            {
                bool currentVal = activeFlightDataSections[i];
                bool newVal = GUILayout.Toggle(currentVal, flightDataOptionLabels[i], GUILayout.Width(100));
                activeFlightDataSections[i] = newVal;

                change |= newVal != currentVal;
            }

            GUILayout.EndVertical();

            if (!change)
                return false;
            CreateDataString();
            CreateLabelString();

            return true;
        }

        public void SaveSettings()
        {
            List<ConfigNode> flightGUISettings = FARSettingsScenarioModule.FlightGUISettings;
            if (flightGUISettings == null)
            {
                FARLogger.Error("Could not save Flight Data Settings because settings config list was null");
                return;
            }

            ConfigNode node = flightGUISettings.FirstOrDefault(t => t.name == "FlightDataSettings");

            if (node == null)
            {
                node = new ConfigNode("FlightDataSettings");
                flightGUISettings.Add(node);
            }

            node.ClearData();

            for (int i = 0; i < activeFlightDataSections.Length; i++)
                node.AddValue("section" + i + "active", activeFlightDataSections[i]);
        }

        private void LoadSettings()
        {
            List<ConfigNode> flightGUISettings = FARSettingsScenarioModule.FlightGUISettings;

            ConfigNode node = flightGUISettings.FirstOrDefault(t => t.name == "FlightDataSettings");

            if (node == null)
                return;
            for (int i = 0; i < activeFlightDataSections.Length; i++)
                if (bool.TryParse(node.GetValue("section" + i + "active"), out bool tmp))
                    activeFlightDataSections[i] = tmp;
        }
    }
}
