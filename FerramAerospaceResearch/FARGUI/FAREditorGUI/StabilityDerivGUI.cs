/*
Ferram Aerospace Research v0.16.1.2 "Marangoni"
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
using System.Text.RegularExpressions;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using KSP.Localization;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI
{
    internal class StabilityDerivGUI
    {
        private readonly GUIDropDown<int> _flapSettingDropdown;
        private readonly GUIDropDown<CelestialBody> _bodySettingDropdown;

        private readonly EditorSimManager simManager;

        private StabilityDerivOutput stabDerivOutput;

        private string altitude = "0";
        private string machNumber = "0.35";
        private bool spoilersDeployed;

        private Vector3 aoAVec;

        public StabilityDerivGUI(
            EditorSimManager simManager,
            GUIDropDown<int> flapSettingDropDown,
            GUIDropDown<CelestialBody> bodySettingDropdown
        )
        {
            this.simManager = simManager;
            _flapSettingDropdown = flapSettingDropDown;
            _bodySettingDropdown = bodySettingDropdown;

            stabDerivOutput = new StabilityDerivOutput();
        }

        public void ArrowAnim(ArrowPointer velArrow)
        {
            velArrow.Direction = -aoAVec;
        }

        private void SetAngleVectors(double aoA)
        {
            if (double.IsNaN(aoA))
                aoA = 0;
            aoA *= FARMathUtil.deg2rad;

            aoAVec = EditorDriver.editorFacility == EditorFacility.SPH
                         ? new Vector3d(0, -Math.Sin(aoA), Math.Cos(aoA))
                         : new Vector3d(0, Math.Cos(aoA), Math.Sin(aoA));
        }

        public void Display()
        {
            const int W160 = 160 + 20; // Rodhern: A width originally designed to be of size 160.

            GUILayout.Label(Localizer.Format("FAREditorStabDerivFlightCond"));
            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FAREditorStabDerivPlanet"));
            _bodySettingDropdown.GUIDropDownDisplay();
            GUILayout.Label(Localizer.Format("FAREditorStabDerivAlt"));
            altitude = GUILayout.TextField(altitude, GUILayout.ExpandWidth(true));
            GUILayout.Label(Localizer.Format("FAREditorStabDerivMach"));
            machNumber = GUILayout.TextField(machNumber, GUILayout.ExpandWidth(true));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FAREditorStabDerivFlap"));
            _flapSettingDropdown.GUIDropDownDisplay();
            GUILayout.Label(Localizer.Format("FAREditorStabDerivSpoiler"));
            spoilersDeployed = GUILayout.Toggle(spoilersDeployed,
                                                spoilersDeployed
                                                    ? Localizer.Format("FAREditorStabDerivSDeploy")
                                                    : Localizer.Format("FAREditorStabDerivSRetract"),
                                                GUILayout.Width(100));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            bool viscous = GUILayout.Toggle(EditorSimManager.UseAccurateViscousCalc, "More accurate aerodynamics");
            if (viscous != EditorSimManager.UseAccurateViscousCalc)
                EditorSimManager.UseAccurateViscousCalc = viscous;
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUI.enabled = !EditorGUI.Instance.VoxelizationUpdateQueued;
            if (GUILayout.Button(Localizer.Format("FAREditorStabDerivCalcButton"),
                                 GUILayout.Width(250.0F),
                                 GUILayout.Height(25.0F)))
                StabDerivCalcButtonAction();
            GUI.enabled = true;
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FAREditorStabDerivAirProp"), GUILayout.Width(180));
            GUILayout.Label(Localizer.Format("FAREditorStabDerivMoI"), GUILayout.Width(W160));
            GUILayout.Label(Localizer.Format("FAREditorStabDerivPoI"), GUILayout.Width(W160));
            GUILayout.Label(Localizer.Format("FAREditorStabDerivLvlFl"), GUILayout.Width(140));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical(GUILayout.Width(180));
            GUILayout.Label(Localizer.Format("FAREditorStabDerivRefArea") +
                            stabDerivOutput.area.ToString("G3") +
                            " " +
                            Localizer.Format("FARUnitMSq"));
            GUILayout.Label(Localizer.Format("FAREditorStabDerivScaledChord") +
                            stabDerivOutput.MAC.ToString("G3") +
                            " " +
                            Localizer.Format("FARUnitM"));
            GUILayout.Label(Localizer.Format("FAREditorStabDerivScaledSpan") +
                            stabDerivOutput.b.ToString("G3") +
                            " " +
                            Localizer.Format("FARUnitM"));
            GUILayout.EndVertical();

            GUILayout.BeginVertical(GUILayout.Width(W160));
            GUILayout.Label(new GUIContent(Localizer.Format("FAREditorStabDerivIxx") +
                                           stabDerivOutput.stabDerivs[0].ToString("G6") +
                                           " " +
                                           Localizer.Format("FARUnitKgMSq"),
                                           Localizer.Format("FAREditorStabDerivIxxExp")));
            GUILayout.Label(new GUIContent(Localizer.Format("FAREditorStabDerivIyy") +
                                           stabDerivOutput.stabDerivs[1].ToString("G6") +
                                           " " +
                                           Localizer.Format("FARUnitKgMSq"),
                                           Localizer.Format("FAREditorStabDerivIyyExp")));
            GUILayout.Label(new GUIContent(Localizer.Format("FAREditorStabDerivIzz") +
                                           stabDerivOutput.stabDerivs[2].ToString("G6") +
                                           " " +
                                           Localizer.Format("FARUnitKgMSq"),
                                           Localizer.Format("FAREditorStabDerivIzzExp")));
            GUILayout.EndVertical();

            GUILayout.BeginVertical(GUILayout.Width(W160));
            GUILayout.Label(new GUIContent(Localizer.Format("FAREditorStabDerivIxy") +
                                           stabDerivOutput.stabDerivs[24].ToString("G6") +
                                           " " +
                                           Localizer.Format("FARUnitKgMSq"),
                                           Localizer.Format("FAREditorStabDerivIxyExp")));
            GUILayout.Label(new GUIContent(Localizer.Format("FAREditorStabDerivIyz") +
                                           stabDerivOutput.stabDerivs[25].ToString("G6") +
                                           " " +
                                           Localizer.Format("FARUnitKgMSq"),
                                           Localizer.Format("FAREditorStabDerivIyzExp")));
            GUILayout.Label(new GUIContent(Localizer.Format("FAREditorStabDerivIxz") +
                                           stabDerivOutput.stabDerivs[26].ToString("G6") +
                                           " " +
                                           Localizer.Format("FARUnitKgMSq"),
                                           Localizer.Format("FAREditorStabDerivIxzExp")));
            GUILayout.EndVertical();

            GUILayout.BeginVertical(GUILayout.Width(140));
            GUILayout.Label(new GUIContent(Localizer.Format("FAREditorStabDerivu0") +
                                           stabDerivOutput.nominalVelocity.ToString("G6") +
                                           " " +
                                           Localizer.Format("FARUnitMPerSec"),
                                           Localizer.Format("FAREditorStabDerivu0Exp")));
            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent(Localizer.Format("FARAbbrevCl") +
                                           ": " +
                                           stabDerivOutput.stableCl.ToString("G3"),
                                           Localizer.Format("FAREditorStabDerivClExp")));
            GUILayout.Label(new GUIContent(Localizer.Format("FARAbbrevCd") +
                                           ": " +
                                           stabDerivOutput.stableCd.ToString("G3"),
                                           Localizer.Format("FAREditorStabDerivCdExp")));
            GUILayout.EndHorizontal();
            GUILayout.Label(new GUIContent(Localizer.Format("FARAbbrevAoA") +
                                           ": " +
                                           stabDerivOutput.stableAoAState +
                                           stabDerivOutput.stableAoA.ToString("G6") +
                                           " " +
                                           Localizer.Format("FARUnitDeg"),
                                           Localizer.Format("FAREditorStabDerivAoAExp")));
            GUILayout.EndVertical();
            GUILayout.EndHorizontal();

            var BackgroundStyle = new GUIStyle(GUI.skin.box);
            BackgroundStyle.hover = BackgroundStyle.active = BackgroundStyle.normal;

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FAREditorLongDeriv"), GUILayout.Width(W160));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FAREditorDownVelDeriv"), GUILayout.Width(W160));
            GUILayout.Label(Localizer.Format("FAREditorFwdVelDeriv"), GUILayout.Width(W160));
            GUILayout.Label(Localizer.Format("FAREditorPitchRateDeriv"), GUILayout.Width(W160));
            GUILayout.Label(Localizer.Format("FAREditorPitchCtrlDeriv"), GUILayout.Width(W160));
            GUILayout.EndHorizontal();
            GUILayout.BeginVertical(BackgroundStyle);
            GUILayout.BeginHorizontal();
            StabilityLabel(Localizer.Format("FAREditorZw"),
                           stabDerivOutput.stabDerivs[3],
                           " " + Localizer.Format("FARUnitInvSec"),
                           Localizer.Format("FAREditorZwExp"),
                           W160,
                           -1);
            StabilityLabel(Localizer.Format("FAREditorZu"),
                           stabDerivOutput.stabDerivs[6],
                           " " + Localizer.Format("FARUnitInvSec"),
                           Localizer.Format("FAREditorZuExp"),
                           W160,
                           -1);
            StabilityLabel(Localizer.Format("FAREditorZq"),
                           stabDerivOutput.stabDerivs[9],
                           " " + Localizer.Format("FARUnitMPerSec"),
                           Localizer.Format("FAREditorZqExp"),
                           W160,
                           0);
            StabilityLabel(Localizer.Format("FAREditorZDeltae"),
                           stabDerivOutput.stabDerivs[12],
                           " " + Localizer.Format("FARUnitMPerSecSq"),
                           Localizer.Format("FAREditorZDeltaeExp"),
                           W160,
                           0);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            StabilityLabel(Localizer.Format("FAREditorXw"),
                           stabDerivOutput.stabDerivs[4],
                           " " + Localizer.Format("FARUnitInvSec"),
                           Localizer.Format("FAREditorXwExp"),
                           W160,
                           0);
            StabilityLabel(Localizer.Format("FAREditorXu"),
                           stabDerivOutput.stabDerivs[7],
                           " " + Localizer.Format("FARUnitInvSec"),
                           Localizer.Format("FAREditorXuExp"),
                           W160,
                           -1);
            StabilityLabel(Localizer.Format("FAREditorXq"),
                           stabDerivOutput.stabDerivs[10],
                           " " + Localizer.Format("FARUnitMPerSec"),
                           Localizer.Format("FAREditorXqExp"),
                           W160,
                           0);
            StabilityLabel(Localizer.Format("FAREditorXDeltae"),
                           stabDerivOutput.stabDerivs[13],
                           " " + Localizer.Format("FARUnitMPerSecSq"),
                           Localizer.Format("FAREditorXDeltaeExp"),
                           W160,
                           0);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            StabilityLabel(Localizer.Format("FAREditorMw"),
                           stabDerivOutput.stabDerivs[5],
                           " " + Localizer.Format("FARUnitInvMSec"),
                           Localizer.Format("FAREditorMwExp"),
                           W160,
                           -1);
            StabilityLabel(Localizer.Format("FAREditorMu"),
                           stabDerivOutput.stabDerivs[8],
                           " " + Localizer.Format("FARUnitInvMSec"),
                           Localizer.Format("FAREditorMuExp"),
                           W160,
                           0);
            StabilityLabel(Localizer.Format("FAREditorMq"),
                           stabDerivOutput.stabDerivs[11],
                           " " + Localizer.Format("FARUnitInvSec"),
                           Localizer.Format("FAREditorMqExp"),
                           W160,
                           -1);
            StabilityLabel(Localizer.Format("FAREditorMDeltae"),
                           stabDerivOutput.stabDerivs[14],
                           " " + Localizer.Format("FARUnitInvSecSq"),
                           Localizer.Format("FAREditorMDeltaeExp"),
                           W160,
                           1);
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FAREditorLatDeriv"), GUILayout.Width(W160));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FAREditorSideslipDeriv"), GUILayout.Width(W160));
            GUILayout.Label(Localizer.Format("FAREditorRollRateDeriv"), GUILayout.Width(W160));
            GUILayout.Label(Localizer.Format("FAREditorYawRateDeriv"), GUILayout.Width(W160));
            GUILayout.EndHorizontal();
            GUILayout.BeginVertical(BackgroundStyle);
            GUILayout.BeginHorizontal();
            StabilityLabel(Localizer.Format("FAREditorYβ"),
                           stabDerivOutput.stabDerivs[15],
                           " " + Localizer.Format("FARUnitMPerSecSq"),
                           Localizer.Format("FAREditorYβExp"),
                           W160,
                           -1);
            StabilityLabel(Localizer.Format("FAREditorYp"),
                           stabDerivOutput.stabDerivs[18],
                           " " + Localizer.Format("FARUnitMPerSec"),
                           Localizer.Format("FAREditorYpExp"),
                           W160,
                           0);
            StabilityLabel(Localizer.Format("FAREditorYr"),
                           stabDerivOutput.stabDerivs[21],
                           " " + Localizer.Format("FARUnitMPerSec"),
                           Localizer.Format("FAREditorYrExp"),
                           W160,
                           1);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            StabilityLabel(Localizer.Format("FAREditorLβ"),
                           stabDerivOutput.stabDerivs[16],
                           " " + Localizer.Format("FARUnitInvSecSq"),
                           Localizer.Format("FAREditorLβExp"),
                           W160,
                           -1);
            StabilityLabel(Localizer.Format("FAREditorLp"),
                           stabDerivOutput.stabDerivs[19],
                           " " + Localizer.Format("FARUnitInvSec"),
                           Localizer.Format("FAREditorLpExp"),
                           W160,
                           -1);
            StabilityLabel(Localizer.Format("FAREditorLr"),
                           stabDerivOutput.stabDerivs[22],
                           " " + Localizer.Format("FARUnitInvSec"),
                           Localizer.Format("FAREditorLrExp"),
                           W160,
                           1);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            StabilityLabel(Localizer.Format("FAREditorNβ"),
                           stabDerivOutput.stabDerivs[17],
                           " " + Localizer.Format("FARUnitInvSecSq"),
                           Localizer.Format("FAREditorNβExp"),
                           W160,
                           1);
            StabilityLabel(Localizer.Format("FAREditorNp"),
                           stabDerivOutput.stabDerivs[20],
                           " " + Localizer.Format("FARUnitInvSec"),
                           Localizer.Format("FAREditorNpExp"),
                           W160,
                           0);
            StabilityLabel(Localizer.Format("FAREditorNr"),
                           stabDerivOutput.stabDerivs[23],
                           " " + Localizer.Format("FARUnitInvSec"),
                           Localizer.Format("FAREditorNrExp"),
                           W160,
                           -1);
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();

            DrawTooltip();
        }

        private void StabDerivCalcButtonAction()
        {
            CelestialBody body = _bodySettingDropdown.ActiveSelection;
            FARAeroUtil.UpdateCurrentActiveBody(body);
            altitude = Regex.Replace(altitude, @"[^-?[0-9]*(\.[0-9]*)?]", "");
            double altitudeDouble = Convert.ToDouble(altitude) * 1000;
            machNumber = Regex.Replace(machNumber, @"[^-?[0-9]*(\.[0-9]*)?]", "");
            double machDouble = FARMathUtil.Clamp(Convert.ToSingle(machNumber), 0.001, float.PositiveInfinity);
            int flapsettingInt = _flapSettingDropdown.ActiveSelection;
            bool spoilersDeployedBool = spoilersDeployed;

            if (FARAtmosphere.GetPressure(body, new Vector3d(0, 0, altitudeDouble), Planetarium.GetUniversalTime()) > 0)
            {
                try
                {
                    // When the shared viscous toggle is on, solve at the real flight-point skin
                    // friction / Reynolds; always cleared below so other tabs stay on the default path.
                    if (EditorSimManager.UseAccurateViscousCalc)
                        simManager.SetAeroCondition(simManager.MakeAeroCondition(body, altitudeDouble, machDouble));

                    var stabProps =
                        simManager.StabDerivCalculator.ComputeVehicleProperties(flapsettingInt, spoilersDeployedBool);
                    GasProperties stabGas =
                        FARAtmosphere.GetGasProperties(body, new Vector3d(0, 0, altitudeDouble), Planetarium.GetUniversalTime());
                    stabDerivOutput =
                        simManager.StabDerivCalculator.CalculateStabilityDerivs(stabProps,
                                                                                body,
                                                                                altitudeDouble,
                                                                                machDouble,
                                                                                stabGas.Density,
                                                                                stabGas.SpeedOfSound,
                                                                                flapsettingInt,
                                                                                spoilersDeployedBool,
                                                                                0,
                                                                                0,
                                                                                0);
                    simManager.vehicleData = stabDerivOutput;
                    SetAngleVectors(stabDerivOutput.stableAoA);
                }
                finally
                {
                    simManager.SetAeroCondition(null);
                }
            }
            else
            {
                PopupDialog.SpawnPopupDialog(new Vector2(0, 0),
                                             new Vector2(0, 0),
                                             "FARStabDerivError",
                                             Localizer.Format("FAREditorStabDerivError"),
                                             Localizer.Format("FAREditorStabDerivErrorExp"),
                                             Localizer.Format("FARGUIOKButton"),
                                             true,
                                             HighLogic.UISkin);
            }
        }

        private static void StabilityLabel(string text1, double val, string text2, string tooltip, int width, int sign)
        {
            Color color = Color.white;
            if (sign != 0)
                color = !double.IsNaN(val) && Math.Sign(val) == sign ? Color.green : Color.red;

            var style = new GUIStyle(GUI.skin.label);
            style.normal.textColor = style.hover.textColor = color;

            GUILayout.Label(new GUIContent(text1 + val.ToString("G6") + text2, tooltip), style, GUILayout.Width(width));
        }

        private static void DrawTooltip()
        {
            if (GUI.tooltip == "")
                return;

            var BackgroundStyle = new GUIStyle(GUI.skin.box);
            BackgroundStyle.hover = BackgroundStyle.active = BackgroundStyle.normal;

            Vector3 mousePos = GUIUtils.GetMousePos();
            Rect windowRect = EditorGUI.GUIRect;

            var tooltipRect = new Rect(Mathf.Clamp(mousePos.x - windowRect.x, 0, windowRect.width - 300),
                                       Mathf.Clamp(mousePos.y - windowRect.y, 0, windowRect.height - 80),
                                       300,
                                       80);

            GUIStyle toolTipStyle = BackgroundStyle;
            toolTipStyle.normal.textColor = toolTipStyle.active.textColor =
                                                toolTipStyle.hover.textColor =
                                                    toolTipStyle.focused.textColor =
                                                        toolTipStyle.onNormal.textColor =
                                                            toolTipStyle.onHover.textColor =
                                                                toolTipStyle.onActive.textColor =
                                                                    toolTipStyle.onFocused.textColor =
                                                                        new Color(1, 0.75f, 0);

            GUI.Box(tooltipRect, GUI.tooltip, toolTipStyle);
        }
    }
}
