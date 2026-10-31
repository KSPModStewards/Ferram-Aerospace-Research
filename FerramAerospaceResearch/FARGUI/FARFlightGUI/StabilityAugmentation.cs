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
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FARFlightGUI
{
    internal class StabilityAugmentation
    {
        private static readonly string[] systemLabel =
        {
            Localizer.Format("FARFlightStabAugLabel0"),
            Localizer.Format("FARFlightStabAugLabel1"),
            Localizer.Format("FARFlightStabAugLabel2"),
            Localizer.Format("FARFlightStabAugLabel3"),
            Localizer.Format("FARFlightStabAugLabel4")
        };

        // ReSharper disable once UnusedMember.Local
        private static string[] systemLabelLong =
        {
            Localizer.Format("FARFlightStabAugLabelLong0"),
            Localizer.Format("FARFlightStabAugLabelLong1"),
            Localizer.Format("FARFlightStabAugLabelLong2"),
            Localizer.Format("FARFlightStabAugLabelLong3"),
            Localizer.Format("FARFlightStabAugLabelLong4")
        };

        private static ControlSystem[] systemTemplates;

        private static double aoALowLim, aoAHighLim;
        private static double scalingDynPres;
        private readonly Vessel _vessel;
        private readonly ControlSystem[] systemInstances;
        private readonly GUIDropDown<int> systemDropdown;
        private VesselFlightInfo info;

        private GUIStyle buttonStyle;
        private GUIStyle boxStyle;

        public StabilityAugmentation(Vessel vessel)
        {
            _vessel = vessel;
            systemDropdown = new GUIDropDown<int>(systemLabel, new[] {0, 1, 2, 3, 4, 5});
            LoadSettings();
            systemInstances = new ControlSystem[systemTemplates.Length];

            for (int i = 0; i < systemInstances.Length; i++)
                systemInstances[i] = new ControlSystem(systemTemplates[i]);
            _vessel.OnAutopilotUpdate += OnAutoPilotUpdate;
        }

        public void SaveAndDestroy()
        {
            // ReSharper disable once DelegateSubtraction
            if (_vessel != null)
                _vessel.OnAutopilotUpdate -= OnAutoPilotUpdate;
            SaveSettings();
        }

        public void UpdatePhysicsInfo(VesselFlightInfo flightInfo)
        {
            info = flightInfo;
        }

        public void Display()
        {
            if (buttonStyle == null)
                buttonStyle = FlightGUI.buttonStyle;
            GUILayout.BeginHorizontal(GUILayout.ExpandWidth(true));
            for (int i = 0; i < systemInstances.Length; i++)
                systemInstances[i].active = GUILayout.Toggle(systemInstances[i].active,
                                                             systemLabel[i],
                                                             buttonStyle,
                                                             GUILayout.MinWidth(30),
                                                             GUILayout.ExpandWidth(true));
            GUILayout.EndHorizontal();
        }

        public void SettingsDisplay()
        {
            if (boxStyle == null)
                boxStyle = FlightGUI.boxStyle;

            GUILayout.Label(Localizer.Format("FARFlightStabLabel"));
            systemDropdown.GUIDropDownDisplay(GUILayout.Width(120));
            int selectedItem = systemDropdown.ActiveSelection;

            ControlSystem sys = systemInstances[selectedItem];
            GUILayout.BeginVertical(boxStyle);
            if (selectedItem != 4)
            {
                sys.kP = GUIUtils.TextEntryForDouble(Localizer.Format("FARFlightStabPropGain"), 120, sys.kP);
                sys.kD = GUIUtils.TextEntryForDouble(Localizer.Format("FARFlightStabDerivGain"), 120, sys.kD);
                sys.kI = GUIUtils.TextEntryForDouble(Localizer.Format("FARFlightStabIntGain"), 120, sys.kI);

                if (selectedItem == 3)
                {
                    aoALowLim = GUIUtils.TextEntryForDouble(Localizer.Format("FARFlightStabAoALow"), 120, aoALowLim);
                    aoAHighLim = GUIUtils.TextEntryForDouble(Localizer.Format("FARFlightStabAoAHigh"), 120, aoAHighLim);
                }
                else
                {
                    sys.zeroPoint =
                        GUIUtils.TextEntryForDouble(Localizer.Format("FARFlightStabOffset"), 120, sys.zeroPoint);
                }
            }
            else
            {
                scalingDynPres =
                    GUIUtils.TextEntryForDouble(Localizer.Format("FARFlightStabQScaling"), 150, scalingDynPres);
            }

            GUILayout.EndVertical();
        }

        private void OnAutoPilotUpdate(FlightCtrlState state)
        {
            if (_vessel.srfSpeed < 5)
                return;

            ControlSystem sys = systemInstances[0]; //wing leveler
            if (sys.active)
            {
                double phi = info.rollAngle - sys.zeroPoint;
                if (sys.kP < 0)
                {
                    phi += 180;
                    if (phi > 180)
                        phi -= 360;
                }
                else
                {
                    phi = -phi;
                }

                phi *= -FARMathUtil.deg2rad;


                // If there's any input, assume the pilot is in control
                if (Math.Abs(state.roll - state.rollTrim) < 0.01)
                {
                    double output = ControlStateChange(sys, phi);
                    if (output > 1)
                        output = 1;
                    else if (output < -1)
                        output = -1;

                    state.roll = (float)output + state.rollTrim;
                } else {
                    sys.errorIntegral = 0;
                }
            }
            else
            {
                sys.errorIntegral = 0;
            }

            sys = systemInstances[1];
            if (sys.active)
            {
                double beta = -(info.sideslipAngle - sys.zeroPoint) * FARMathUtil.deg2rad;

                double output = ControlStateChange(sys, beta);

                if (Math.Abs(state.yaw - state.yawTrim) < 0.01)
                {
                    if (output > 1)
                        output = 1;
                    else if (output < -1)
                        output = -1;

                    state.yaw = (float)output + state.yawTrim;
                }
            }
            else
            {
                sys.errorIntegral = 0;
            }

            sys = systemInstances[2];
            if (sys.active)
            {
                double pitch = (info.aoA - sys.zeroPoint) * FARMathUtil.deg2rad;

                double output = ControlStateChange(sys, pitch);

                if (Math.Abs(state.pitch - state.pitchTrim) < 0.01)
                {
                    if (output > 1)
                        output = 1;
                    else if (output < -1)
                        output = -1;

                    state.pitch = (float)output + state.pitchTrim;
                }
            }
            else
            {
                sys.errorIntegral = 0;
            }

            sys = systemInstances[3];
            if (sys.active)
            {
                if (info.aoA > aoAHighLim)
                    state.pitch = (float)ControlStateChange(sys, info.aoA - aoAHighLim).Clamp(-1, 1) + state.pitchTrim;
                else if (info.aoA < aoALowLim)
                    state.pitch = (float)ControlStateChange(sys, info.aoA - aoALowLim).Clamp(-1, 1) + state.pitchTrim;
            }
            else
            {
                sys.errorIntegral = 0;
            }

            sys = systemInstances[4];
            if (!sys.active)
                return;
            double scalingFactor = scalingDynPres / info.dynPres;

            if (scalingFactor > 1)
                scalingFactor = 1;

            state.pitch = state.pitchTrim + (state.pitch - state.pitchTrim) * (float)scalingFactor;
            state.yaw = state.yawTrim + (state.yaw - state.yawTrim) * (float)scalingFactor;
            state.roll = state.rollTrim + (state.roll - state.rollTrim) * (float)scalingFactor;
        }

        private static double ControlStateChange(ControlSystem system, double error)
        {
            double state = 0;
            double dt = TimeWarp.fixedDeltaTime;

            double dError_dt = error - system.lastError;
            dError_dt /= dt;

            system.errorIntegral += error * dt;

            state -= system.kP * error + system.kD * dError_dt + system.kI * system.errorIntegral;

            system.lastError = error;

            return state;
        }

        public static void LoadSettings()
        {
            List<ConfigNode> flightGUISettings = FARSettingsScenarioModule.FlightGUISettings;

            ConfigNode node = flightGUISettings.FirstOrDefault(t => t.name == "StabilityAugmentationSettings");

            if (systemTemplates != null)
                return;
            systemTemplates = new ControlSystem[5];
            for (int i = 0; i < systemTemplates.Length; i++)
                systemTemplates[i] = new ControlSystem();

            if (node != null)
            {
                for (int i = 0; i < systemTemplates.Length; i++)
                {
                    string nodeName = "ControlSys" + i;
                    if (node.HasNode(nodeName))
                        TryLoadSystem(node.GetNode(nodeName), i);
                }

                if (node.HasValue("aoALowLim"))
                    double.TryParse(node.GetValue("aoALowLim"), out aoALowLim);
                if (node.HasValue("aoAHighLim"))
                    double.TryParse(node.GetValue("aoAHighLim"), out aoAHighLim);
                if (node.HasValue("scalingDynPres"))
                    double.TryParse(node.GetValue("scalingDynPres"), out scalingDynPres);
            }
            else
            {
                BuildDefaultSystems();
            }
        }

        public static void BuildDefaultSystems()
        {
            //Roll system
            var sys = new ControlSystem
            {
                kP = 0.5,
                kD = 1,
                kI = 0.5
            };

            systemTemplates[0] = sys;

            //Yaw system
            sys = new ControlSystem
            {
                kP = 0,
                kD = 1,
                kI = 0
            };

            systemTemplates[1] = sys;

            //Pitch system
            sys = new ControlSystem
            {
                kP = 0,
                kD = 1,
                kI = 0
            };

            systemTemplates[2] = sys;

            //AoA system
            sys = new ControlSystem
            {
                kP = 0.25,
                kD = 0,
                kI = 0
            };

            systemTemplates[3] = sys;

            aoALowLim = -10;
            aoAHighLim = 20;

            scalingDynPres = 20;
        }

        // ReSharper disable once UnusedMethodReturnValue.Global -> try method
        public static bool TryLoadSystem(ConfigNode systemNode, int index)
        {
            bool sysExists = false;
            ControlSystem sys = systemTemplates[index];

            if (systemNode.HasValue("active"))
            {
                bool.TryParse(systemNode.GetValue("active"), out sys.active);
                sysExists = true;
            }

            if (systemNode.HasValue("zeroPoint"))
                double.TryParse(systemNode.GetValue("zeroPoint"), out sys.zeroPoint);

            if (systemNode.HasValue("kP"))
                double.TryParse(systemNode.GetValue("kP"), out sys.kP);
            if (systemNode.HasValue("kD"))
                double.TryParse(systemNode.GetValue("kD"), out sys.kD);
            if (systemNode.HasValue("kI"))
                double.TryParse(systemNode.GetValue("kI"), out sys.kI);

            systemTemplates[index] = sys;
            return sysExists;
        }

        public void SaveSettings()
        {
            List<ConfigNode> flightGUISettings = FARSettingsScenarioModule.FlightGUISettings;
            if (flightGUISettings == null)
            {
                FARLogger.Error("Could not save Stability Augmentation Settings because settings config list was null");
                return;
            }

            ConfigNode node = flightGUISettings.FirstOrDefault(t => t.name == "StabilityAugmentationSettings");

            Vessel active_vessel;

            try
            {
                active_vessel = FlightGlobals.ActiveVessel;
            }
            catch (NullReferenceException)
            {
                FARLogger.Error("Could not save Stability Augmentation Settings because 'FlightGlobals.ActiveVessel' is null.");
                return;
            }

            if (_vessel != active_vessel)
                return;
            systemTemplates = systemInstances;

            if (node == null)
            {
                node = new ConfigNode("StabilityAugmentationSettings");
                flightGUISettings.Add(node);
            }
            else
            {
                node.ClearData();
            }

            for (int i = 0; i < systemTemplates.Length; i++)
                node.AddNode(BuildSystemNode(i));
            node.AddValue("aoALowLim", aoALowLim);
            node.AddValue("aoAHighLim", aoAHighLim);
            node.AddValue("scalingDynPres", scalingDynPres);
        }

        public static ConfigNode BuildSystemNode(int index)
        {
            ControlSystem sys = systemTemplates[index];

            var node = new ConfigNode("ControlSys" + index);
            node.AddValue("active", sys.active);
            node.AddValue("zeroPoint", sys.zeroPoint);

            node.AddValue("kP", sys.kP);
            node.AddValue("kD", sys.kD);
            node.AddValue("kI", sys.kI);

            return node;
        }

        private class ControlSystem
        {
            public bool active;

            public double zeroPoint;

            public double kP;
            public double kD;
            public double kI;

            public double lastError;
            public double errorIntegral;

            public ControlSystem(ControlSystem sys)
            {
                active = sys.active;
                zeroPoint = sys.zeroPoint;
                kP = sys.kP;
                kD = sys.kD;
                kI = sys.kI;

                lastError = 0;
                errorIntegral = 0;
            }

            public ControlSystem()
            {
            }
        }
    }
}
