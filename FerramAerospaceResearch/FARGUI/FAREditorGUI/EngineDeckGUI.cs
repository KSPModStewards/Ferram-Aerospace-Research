/*
Ferram Aerospace Research
=========================
Engine deck readout — thrust and fuel flow over a (Mach × altitude) grid.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Globalization;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using KSP.Localization;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI
{
    /// <summary>
    /// Tabulates thrust and fuel flow from the ship's AJE/SolverEngines engines across a
    /// (Mach × altitude) grid, at a fixed throttle.
    ///
    /// This is the thrust half of a level-flight performance envelope; the aero half (trim for
    /// L=W, solve T=D for top speed / ceiling / endurance) is not wired up yet. It stands alone as
    /// a check that the engine sampling is sound: the M 0.00 / 0 km cell should reproduce the
    /// part tooltip's static thrust, since AJE generates that number with the same UpdateSolver
    /// call this tab makes.
    ///
    /// Sampling is synchronous — no aero runs here, so a grid this size costs well under a frame.
    /// </summary>
    internal class EngineDeckGUI
    {
        private readonly EditorEngineDeck _deck = new EditorEngineDeck();
        private readonly GUIDropDown<CelestialBody> _bodySettingDropdown;

        private string _machMin = "0";
        private string _machMax = "2.0";
        private string _altMinKm = "0";
        private string _altMaxKm = "15";
        private string _nMach = "6";
        private string _nAlt = "6";
        private string _throttle = "1.0";

        private double[] _machAxis;
        private double[] _altAxis;
        private EditorEngineDeck.Sample[,] _samples;
        private string _statusMessage = "";
        private bool _showFuelFlow;

        public EngineDeckGUI(GUIDropDown<CelestialBody> bodySettingDropdown)
        {
            _bodySettingDropdown = bodySettingDropdown;
        }

        public void Display()
        {
            GUILayout.Label(Localizer.Format("FAREngineDeckTitle"));
            GUILayout.Label(Localizer.Format("FAREngineDeckDesc"), GUI.skin.box);

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FAREditorStabDerivPlanet"));
            _bodySettingDropdown.GUIDropDownDisplay();
            GUILayout.Space(20);
            GUILayout.Label(Localizer.Format("FAREngineDeckThrottle"), GUILayout.Width(70));
            _throttle = GUILayout.TextField(_throttle, GUILayout.Width(50));
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FARStabHeatmapMachRange"), GUILayout.Width(120));
            _machMin = GUILayout.TextField(_machMin, GUILayout.Width(60));
            GUILayout.Label("→", GUILayout.Width(20));
            _machMax = GUILayout.TextField(_machMax, GUILayout.Width(60));
            GUILayout.Space(20);
            GUILayout.Label(Localizer.Format("FARStabHeatmapAltRange"), GUILayout.Width(140));
            _altMinKm = GUILayout.TextField(_altMinKm, GUILayout.Width(60));
            GUILayout.Label("→", GUILayout.Width(20));
            _altMaxKm = GUILayout.TextField(_altMaxKm, GUILayout.Width(60));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FARStabHeatmapCells"), GUILayout.Width(120));
            _nMach = GUILayout.TextField(_nMach, GUILayout.Width(40));
            GUILayout.Label("×", GUILayout.Width(20));
            _nAlt = GUILayout.TextField(_nAlt, GUILayout.Width(40));
            GUILayout.Space(20);
            _showFuelFlow = GUILayout.Toggle(_showFuelFlow,
                                             Localizer.Format("FAREngineDeckShowFlow"),
                                             GUILayout.Width(110));
            GUILayout.FlexibleSpace();

            if (GUILayout.Button(Localizer.Format("FAREngineDeckRun"), GUILayout.Width(120), GUILayout.Height(25)))
                RunSamples();

            GUILayout.EndHorizontal();

            if (!string.IsNullOrEmpty(_statusMessage))
                GUILayout.Label(_statusMessage);

            DrawTable();
        }

        private void RunSamples()
        {
            _samples = null;

            SolverEnginesInterop.Initialize();
            if (!SolverEnginesInterop.Available)
            {
                _statusMessage = Localizer.Format("FAREngineDeckNoSolver");
                return;
            }

            if (!TryParseRanges(out double machMin,
                                out double machMax,
                                out double altMin,
                                out double altMax,
                                out int nMach,
                                out int nAlt,
                                out double throttle))
                return;

            CelestialBody body = _bodySettingDropdown.ActiveSelection;
            FARAeroUtil.UpdateCurrentActiveBody(body);

            _deck.UpdateShip();
            if (!_deck.Ready)
            {
                _statusMessage = Localizer.Format("FAREngineDeckNoEngines");
                return;
            }

            _machAxis = LinSpace(machMin, machMax, nMach);
            _altAxis = LinSpace(altMin, altMax, nAlt);
            _samples = new EditorEngineDeck.Sample[nMach, nAlt];

            double ut = Planetarium.GetUniversalTime();

            // This runs inside a GUI render. Anything escaping here tears down the whole FAR window
            // for the frame, and would skip the throttle reset below.
            try
            {
                for (int j = 0; j < nAlt; j++)
                {
                    double alt = _altAxis[j];

                    // Above the atmosphere an air-breather has nothing to work with, and
                    // EngineThermodynamics would be handed a zero pressure.
                    if (FARAtmosphere.GetPressure(body, new Vector3d(0, 0, alt), ut) <= 0)
                        continue;

                    for (int i = 0; i < nMach; i++)
                        _samples[i, j] = _deck.Evaluate(body, alt, _machAxis[i], throttle);
                }
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "while sampling the engine deck");
                _statusMessage = "Engine sampling failed — see KSP.log.";
                _samples = null;
                return;
            }
            finally
            {
                _deck.Restore();
            }

            _statusMessage = Localizer.Format("FAREngineDeckStatus",
                                              _deck.EngineCount.ToString(CultureInfo.InvariantCulture),
                                              _deck.InletCount.ToString(CultureInfo.InvariantCulture));
        }

        private void DrawTable()
        {
            if (_samples == null || _machAxis == null || _altAxis == null)
                return;

            GUILayout.Label(_showFuelFlow
                                ? Localizer.Format("FAREngineDeckFlowHeader")
                                : Localizer.Format("FAREngineDeckThrustHeader"));

            GUILayout.BeginHorizontal();
            GUILayout.Label("alt \\ M", GUI.skin.box, GUILayout.Width(60));
            foreach (double mach in _machAxis)
                GUILayout.Label(mach.ToString("F2", CultureInfo.InvariantCulture),
                                GUI.skin.box,
                                GUILayout.Width(60));
            GUILayout.EndHorizontal();

            // Descending so altitude increases up the table, matching the heatmap tab.
            for (int j = _altAxis.Length - 1; j >= 0; j--)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label((_altAxis[j] * 0.001).ToString("F1", CultureInfo.InvariantCulture) + " km",
                                GUI.skin.box,
                                GUILayout.Width(60));

                for (int i = 0; i < _machAxis.Length; i++)
                {
                    EditorEngineDeck.Sample s = _samples[i, j];
                    string text = _showFuelFlow
                                      ? s.FuelFlow.ToString("F3", CultureInfo.InvariantCulture)
                                      : (s.ThrustMagnitude * 0.001).ToString("F1", CultureInfo.InvariantCulture);
                    GUILayout.Label(text, GUI.skin.box, GUILayout.Width(60));
                }

                GUILayout.EndHorizontal();
            }
        }

        private bool TryParseRanges(
            out double machMin,
            out double machMax,
            out double altMin,
            out double altMax,
            out int nMach,
            out int nAlt,
            out double throttle
        )
        {
            machMin = machMax = altMin = altMax = throttle = 0;
            nMach = nAlt = 0;

            if (!double.TryParse(_machMin, NumberStyles.Float, CultureInfo.InvariantCulture, out machMin) ||
                !double.TryParse(_machMax, NumberStyles.Float, CultureInfo.InvariantCulture, out machMax))
            {
                _statusMessage = "Mach range must be numeric.";
                return false;
            }

            if (machMax <= machMin || machMin < 0)
            {
                _statusMessage = "Mach range must be increasing and non-negative.";
                return false;
            }

            if (!double.TryParse(_altMinKm, NumberStyles.Float, CultureInfo.InvariantCulture, out double aMin) ||
                !double.TryParse(_altMaxKm, NumberStyles.Float, CultureInfo.InvariantCulture, out double aMax))
            {
                _statusMessage = "Altitude range must be numeric.";
                return false;
            }

            if (aMax <= aMin || aMin < 0)
            {
                _statusMessage = "Altitude range must be increasing and non-negative.";
                return false;
            }

            altMin = aMin * 1000;
            altMax = aMax * 1000;

            if (!int.TryParse(_nMach, out nMach) || nMach < 2 || nMach > 12 ||
                !int.TryParse(_nAlt, out nAlt) || nAlt < 2 || nAlt > 12)
            {
                _statusMessage = "Cell counts must be integers between 2 and 12.";
                return false;
            }

            if (!double.TryParse(_throttle, NumberStyles.Float, CultureInfo.InvariantCulture, out throttle) ||
                throttle < 0 ||
                throttle > 1)
            {
                _statusMessage = "Throttle must be between 0 and 1.";
                return false;
            }

            return true;
        }

        private static double[] LinSpace(double min, double max, int n)
        {
            var values = new double[n];
            if (n == 1)
            {
                values[0] = min;
                return values;
            }

            double step = (max - min) / (n - 1);
            for (int i = 0; i < n; i++)
                values[i] = min + step * i;
            return values;
        }
    }
}
