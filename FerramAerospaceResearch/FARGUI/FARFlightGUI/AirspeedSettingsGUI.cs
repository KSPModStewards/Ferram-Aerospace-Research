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
using KSP.UI.Screens.Flight;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FARFlightGUI
{
    public class AirspeedSettingsGUI
    {
        // ReSharper disable once UnusedMember.Global -> MACH
        public enum SurfaceVelMode
        {
            TAS,
            IAS,
            EAS,
            MACH
        }

        public enum SurfaceVelUnit
        {
            M_S,
            KNOTS,
            MPH,
            KM_H
        }

        public static bool allEnabled = true;
        private readonly Vessel _vessel;

        private readonly string[] surfModel_str =
        {
            Localizer.Format("FARFlightAirspeedGroundspeed"),
            Localizer.Format("FARFlightAirspeedIndicated"),
            Localizer.Format("FARFlightAirspeedEquivalent"),
            Localizer.Format("FARAbbrevMach")
        };

        private readonly string[] surfUnit_str =
        {
            Localizer.Format("FARFlightAirspeedMeterPerSec"),
            Localizer.Format("FARFlightAirspeedKnots"),
            Localizer.Format("FARFlightAirspeedMPH"),
            Localizer.Format("FARFlightAirspeedKMH")
        };

        private GUIStyle buttonStyle;

        private SurfaceVelMode velMode = SurfaceVelMode.TAS;

        private SurfaceVelUnit unitMode = SurfaceVelUnit.M_S;

        // DaMichel: cache the velocity display string for retrieval in GetVelocityDisplayString
        private string velString;
        private bool active; // Have we actually generated the string?

        public AirspeedSettingsGUI(Vessel vessel, bool enabled = true)
        {
            _vessel = vessel;
            this.enabled = enabled;
            LoadSettings();
        }

        public bool enabled { get; set; }

        public void AirSpeedSettings()
        {
            if (buttonStyle == null)
                buttonStyle = FlightGUI.buttonStyle;

            GUILayout.BeginVertical();
            Localizer.Format("FARFlightAirspeedLabel");
            GUILayout.Space(10);
            GUILayout.EndVertical();
            GUILayout.BeginHorizontal();
            velMode = (SurfaceVelMode)GUILayout.SelectionGrid((int)velMode, surfModel_str, 1, buttonStyle);
            unitMode = (SurfaceVelUnit)GUILayout.SelectionGrid((int)unitMode, surfUnit_str, 1, buttonStyle);
            GUILayout.EndHorizontal();
        }

        public bool GetVelocityDisplayString(out string value_out, out SurfaceVelMode mode_out)
        {
            value_out = velString;
            mode_out = velMode;
            return allEnabled && enabled && active;
        }

        public double CalculateIAS()
        {
            //stag pressure at pitot tube face / ambient pressure
            double pressureRatio = FARAeroUtil.RayleighPitotTubeStagPressure(_vessel.mach);

            double velocity = pressureRatio - 1;
            velocity *= FARAtmosphere.GetPressure(_vessel) * 2;
            velocity /= 1.225;
            velocity = Math.Sqrt(velocity);

            return velocity;
        }

        public double CalculateEAS()
        {
            double densityRatio = FARAeroUtil.GetCurrentDensity(_vessel) / 1.225;
            return _vessel.srfSpeed * Math.Sqrt(densityRatio);
        }

        public void ChangeSurfVelocity()
        {
            // No need to build the string
            if (!(allEnabled && enabled))
                return;

            active = false;
            //DaMichel: Avoid conflict between multiple vessels in physics range. We only want to show the speed of the active vessel.
            if (FlightGlobals.ActiveVessel != _vessel)
                return;
            //DaMichel: Keep our fingers off of this also if there is no atmosphere (staticPressure <= 0)
            if (FlightGlobals.speedDisplayMode != FlightGlobals.SpeedDisplayModes.Surface || _vessel.atmDensity <= 0)
                return;

            if (SpeedDisplay.Instance == null)
                return;

            double unitConversion = 1;
            string unitString;
            string caption;
            switch (unitMode)
            {
                case SurfaceVelUnit.KNOTS:
                    unitConversion = 1.943844492440604768413343347219;
                    unitString = surfUnit_str[1];
                    break;
                case SurfaceVelUnit.KM_H:
                    unitConversion = 3.6;
                    unitString = surfUnit_str[3];
                    break;
                case SurfaceVelUnit.MPH:
                    unitConversion = 2.236936;
                    unitString = surfUnit_str[2];
                    break;
                default:
                    unitString = surfUnit_str[0];
                    break;
            }

            switch (velMode)
            {
                case SurfaceVelMode.TAS:
                    caption = surfModel_str[0];
                    velString = (_vessel.srfSpeed * unitConversion).ToString("F1") + unitString;
                    break;
                case SurfaceVelMode.IAS:
                    caption = surfModel_str[1];
                    velString = (CalculateIAS() * unitConversion).ToString("F1") + unitString;
                    break;
                case SurfaceVelMode.EAS:
                    caption = surfModel_str[2];
                    velString = (CalculateEAS() * unitConversion).ToString("F1") + unitString;
                    break;
                default:
                    caption = surfModel_str[3];
                    velString = _vessel.mach.ToString("F3");
                    break;
            }

            active = true;

            SpeedDisplay UI = SpeedDisplay.Instance;
            if (UI.textSpeed == null || UI.textTitle == null)
                return;

            UI.textTitle.text = caption;
            UI.textSpeed.text = velString;
        }

        public void SaveSettings()
        {
            List<ConfigNode> flightGUISettings = FARSettingsScenarioModule.FlightGUISettings;
            if (flightGUISettings == null)
            {
                FARLogger.Error("Could not save Airspeed Settings because settings config list was null");
                return;
            }

            ConfigNode node = flightGUISettings.FirstOrDefault(t => t.name == "AirSpeedSettings");

            if (node == null)
            {
                node = new ConfigNode("AirSpeedSettings");
                flightGUISettings.Add(node);
            }

            node.ClearData();

            node.AddValue("unitTypeIndex", (int)unitMode);
            node.AddValue("velTypeIndex", (int)velMode);
        }

        private void LoadSettings()
        {
            List<ConfigNode> flightGUISettings = FARSettingsScenarioModule.FlightGUISettings;

            ConfigNode node = flightGUISettings.FirstOrDefault(t => t.name == "AirSpeedSettings");

            if (node == null)
            {
                unitMode = 0;
                velMode = 0;
            }
            else
            {
                if (int.TryParse(node.GetValue("unitTypeIndex"), out int tmp))
                    unitMode = (SurfaceVelUnit)tmp;
                else
                    unitMode = 0;

                if (int.TryParse(node.GetValue("velTypeIndex"), out tmp))
                    velMode = (SurfaceVelMode)tmp;
                else
                    velMode = 0;
            }
        }
    }
}
