/*
Ferram Aerospace Research
=========================
Combined (Mach × altitude) heatmaps: level-flight performance envelope and stability derivatives.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using KSP.IO;
using KSP.Localization;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI
{
    /// <summary>
    /// Two (Mach × altitude) maps over one shared grid: can the aircraft fly here, and is it stable
    /// here. They answer different halves of the same design question and are worth reading against
    /// each other, so they share one set of sweep parameters and one Run.
    ///
    /// Stacked rather than side by side because the FAR window is 650 px wide; two maps abreast
    /// would leave cells too small to hover, and hovering is how these maps are read.
    ///
    /// The finished sweep is published to <see cref="HeatmapResult.Cached" /> so the flight scene can
    /// display it. Flight cannot re-run it — see HeatmapResult's remarks.
    /// </summary>
    internal class HeatmapsGUI
    {
        // Stability-derivative indices in StabilityDerivOutput.stabDerivs[] paired with the sign
        // expected for dynamic stability. Mirrors the (text, value, …, sign) tuples in
        // StabilityDerivGUI.StabilityLabel call sites where sign != 0.
        private static readonly (int index, int expectedSign, string name)[] CheckedDerivs =
        {
            (3, -1, "Zw"),
            (6, -1, "Zu"),
            (5, -1, "Mw"),
            (11, -1, "Mq"),
            (14, 1, "MΔe"),
            (15, -1, "Yβ"),
            (16, -1, "Lβ"),
            (19, -1, "Lp"),
            (22, 1, "Lr"),
            (17, 1, "Nβ"),
            (23, -1, "Nr")
        };

        private const float MapWidth = 470f;
        private const float MapHeight = 190f;

        // Masses the envelope is swept at, from dry to wet, so the flight scene can interpolate the
        // map as fuel burns. Three (dry, mid, wet) keeps the quadratic weight→drag curve honest
        // without a straight two-point underestimate.
        private const int MassSamples = 3;

        private readonly EditorSimManager _simManager;
        private readonly EditorEngineDeck _deck = new EditorEngineDeck();

        // One renderer per engine-type envelope map, plus one for the shared stability map.
        private readonly List<HeatmapRenderer> _envRenderers = new List<HeatmapRenderer>();
        private readonly HeatmapRenderer _stabRenderer = new HeatmapRenderer();

        private readonly GUIDropDown<int> _flapSettingDropdown;
        private readonly GUIDropDown<CelestialBody> _bodySettingDropdown;

        private string _machMin = "0.15";
        private string _machMax = "2.5";
        private string _altMinKm = "0";
        private string _altMaxKm = "20";
        private string _nMach = "12";
        private string _nAlt = "10";
        private bool _spoilers;
        private bool _includeStability = true;
        private bool _autoRecalc;
        private bool _lastVoxQueued;

        // Speed axis in true airspeed (m/s) rather than Mach. The x-range fields and the map x-axis
        // both follow this; each cell's Mach is then derived per altitude from the local sound speed.
        private bool _speedInMS;

        private List<HeatmapResult> _results;
        private List<EditorEngineDeck.EngineGroup> _sweepGroups; // engines per map; null = combined
        private Coroutine _sweep;
        private bool _cancelRequested;
        private int _cellsDone;
        private int _cellsTotal;
        private string _statusMessage = "";
        private CelestialBody _lastBody;

        public HeatmapsGUI(
            EditorSimManager simManager,
            GUIDropDown<int> flapSettingDropdown,
            GUIDropDown<CelestialBody> bodySettingDropdown
        )
        {
            _simManager = simManager;
            _flapSettingDropdown = flapSettingDropdown;
            _bodySettingDropdown = bodySettingDropdown;
            LoadSettings();
        }

        /// <summary>
        /// Restores the sweep parameters from FAR's plugin config. Stored as the raw strings the
        /// text fields hold, so a partially-typed value round-trips unchanged rather than being
        /// silently reformatted.
        /// </summary>
        private void LoadSettings()
        {
            PluginConfiguration config = FARDebugAndSettings.config;
            if (config == null)
                return;

            _machMin = config.GetValue("heatmap_machMin", _machMin);
            _machMax = config.GetValue("heatmap_machMax", _machMax);
            _altMinKm = config.GetValue("heatmap_altMinKm", _altMinKm);
            _altMaxKm = config.GetValue("heatmap_altMaxKm", _altMaxKm);
            _nMach = config.GetValue("heatmap_nMach", _nMach);
            _nAlt = config.GetValue("heatmap_nAlt", _nAlt);
            _spoilers = config.GetValue("heatmap_spoilers", _spoilers ? 1 : 0) != 0;
            _includeStability = config.GetValue("heatmap_includeStability", _includeStability ? 1 : 0) != 0;
            _speedInMS = config.GetValue("heatmap_speedInMS", _speedInMS ? 1 : 0) != 0;

            // The altitude range is normally re-derived whenever the body changes. Seed _lastBody
            // with the current selection so that first draw counts as "no change" and leaves the
            // restored range alone — otherwise persistence would be pointless, overwritten before
            // the user ever saw it.
            _lastBody = _bodySettingDropdown.ActiveSelection;
        }

        public void SaveSettings()
        {
            PluginConfiguration config = FARDebugAndSettings.config;
            if (config == null)
                return;

            config.SetValue("heatmap_machMin", _machMin);
            config.SetValue("heatmap_machMax", _machMax);
            config.SetValue("heatmap_altMinKm", _altMinKm);
            config.SetValue("heatmap_altMaxKm", _altMaxKm);
            config.SetValue("heatmap_nMach", _nMach);
            config.SetValue("heatmap_nAlt", _nAlt);
            config.SetValue("heatmap_spoilers", _spoilers ? 1 : 0);
            config.SetValue("heatmap_includeStability", _includeStability ? 1 : 0);
            config.SetValue("heatmap_speedInMS", _speedInMS ? 1 : 0);
            config.save();
        }

        public bool IsSweepRunning => _sweep != null;

        public void Display()
        {
            SyncAltitudeDefaultsToBody();
            CheckAutoRecalc();

            GUILayout.Label(Localizer.Format("FARHeatmapsTitle"));

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FAREditorStabDerivPlanet"));
            _bodySettingDropdown.GUIDropDownDisplay();
            GUILayout.Label(Localizer.Format("FAREditorStabDerivFlap"));
            _flapSettingDropdown.GUIDropDownDisplay();
            _spoilers = GUILayout.Toggle(_spoilers,
                                         _spoilers
                                             ? Localizer.Format("FAREditorStabDerivSDeploy")
                                             : Localizer.Format("FAREditorStabDerivSRetract"),
                                         GUILayout.Width(100));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(_speedInMS ? "Speed (m/s)" : Localizer.Format("FARStabHeatmapMachRange"),
                            GUILayout.Width(90));
            if (GUILayout.Button(_speedInMS ? "m/s" : "Mach", GUILayout.Width(46)))
                ToggleSpeedUnit();
            _machMin = GUILayout.TextField(_machMin, GUILayout.Width(50));
            GUILayout.Label("→", GUILayout.Width(18));
            _machMax = GUILayout.TextField(_machMax, GUILayout.Width(50));
            GUILayout.Space(14);
            GUILayout.Label(Localizer.Format("FARStabHeatmapAltRange"), GUILayout.Width(90));
            _altMinKm = GUILayout.TextField(_altMinKm, GUILayout.Width(50));
            GUILayout.Label("→", GUILayout.Width(18));
            _altMaxKm = GUILayout.TextField(_altMaxKm, GUILayout.Width(50));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FARStabHeatmapCells"), GUILayout.Width(110));
            _nMach = GUILayout.TextField(_nMach, GUILayout.Width(40));
            GUILayout.Label("×", GUILayout.Width(18));
            _nAlt = GUILayout.TextField(_nAlt, GUILayout.Width(40));
            GUILayout.Space(14);
            _includeStability = GUILayout.Toggle(_includeStability,
                                                 Localizer.Format("FARHeatmapsIncludeStab"),
                                                 GUILayout.Width(150));
            GUILayout.FlexibleSpace();

            if (IsSweepRunning)
            {
                if (GUILayout.Button(Localizer.Format("FARStabHeatmapCancel"),
                                     GUILayout.Width(110),
                                     GUILayout.Height(24)))
                    _cancelRequested = true;
            }
            else
            {
                GUI.enabled = !EditorGUI.Instance.VoxelizationUpdateQueued;
                if (GUILayout.Button(Localizer.Format("FARStabHeatmapRun"),
                                     GUILayout.Width(110),
                                     GUILayout.Height(24)))
                    StartSweep();
                GUI.enabled = true;
            }

            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            DrawMetricToggle();
            GUILayout.Space(14);
            bool prevAuto = _autoRecalc;
            _autoRecalc = GUILayout.Toggle(_autoRecalc,
                                           Localizer.Format("FARHeatmapsAutoRecalc"),
                                           GUILayout.Width(200));
            if (_autoRecalc && !prevAuto)
                _lastVoxQueued = true; // arm: recalc once the current/next voxelization completes
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();

            if (IsSweepRunning)
                GUILayout.Label(Localizer.Format("FARStabHeatmapProgress",
                                                 _cellsDone.ToString(CultureInfo.InvariantCulture),
                                                 _cellsTotal.ToString(CultureInfo.InvariantCulture)));
            else if (!string.IsNullOrEmpty(_statusMessage))
                GUILayout.Label(_statusMessage);

            if (_results == null || _results.Count == 0)
                return;

            // Up to two maps fit across the 650 px window. With the combined map that makes three,
            // so wrap into rows of two.
            bool multi = _results.Count >= 2;
            float w = multi ? 250f : MapWidth;
            const int perRow = 2;

            // Ordered map cells: one per engine-type envelope, then the shared stability map. Adding
            // stability to the flow (rather than drawing it full-width beneath) lets it fill the last
            // partial row — for a two-engine craft that is the row the combined map sits alone on.
            var cells = new List<MapCell>();
            foreach (HeatmapRenderer r in _envRenderers)
                cells.Add(new MapCell(r, false));
            bool stabInFlow = _includeStability && multi;
            if (stabInFlow)
                cells.Add(new MapCell(_stabRenderer, true));

            for (int start = 0; start < cells.Count; start += perRow)
            {
                int end = Math.Min(start + perRow, cells.Count);
                DrawMapRow(cells, start, end, w, multi);
            }

            // Single-type craft: the lone envelope map is full-width, so the stability map reads
            // better stacked beneath it than squeezed into a half-width column alongside.
            if (_includeStability && !stabInFlow)
                _stabRenderer.DrawStabilityMap(MapWidth, MapHeight);

            // Tooltips last, over everything. Only the hovered renderer has a live hover.
            foreach (HeatmapRenderer r in _envRenderers)
                r.DrawTooltips();
            _stabRenderer.DrawTooltips();
        }

        /// <summary>One map's placement in the row flow: an envelope renderer, or the stability map.</summary>
        private readonly struct MapCell
        {
            public readonly HeatmapRenderer Renderer;
            public readonly bool IsStability;

            public MapCell(HeatmapRenderer renderer, bool isStability)
            {
                Renderer = renderer;
                IsStability = isStability;
            }
        }

        /// <summary>
        /// Draws cells [start, end) as a text row over an aligned map row. Splitting text from maps
        /// keeps the maps level even when one column's summary is taller than another's — including
        /// the stability column, whose header is one line against an envelope column's summary block.
        /// </summary>
        private void DrawMapRow(List<MapCell> cells, int start, int end, float w, bool multi)
        {
            float col = w + 60;

            if (!IsSweepRunning)
            {
                GUILayout.BeginHorizontal();
                for (int t = start; t < end; t++)
                {
                    GUILayout.BeginVertical(GUILayout.Width(col));
                    MapCell cell = cells[t];
                    if (cell.IsStability)
                    {
                        GUILayout.Label(Localizer.Format("FARHeatmapsStabLabel"));
                    }
                    else
                    {
                        if (multi)
                            GUILayout.Label(cell.Renderer.EngineTypeLabel);
                        GUILayout.Label(cell.Renderer.SummaryText(out bool ceilingClipped, out bool topClipped));
                        if (ceilingClipped)
                            GUILayout.Label(Localizer.Format("FAREnvelopeCeilingClipped"));
                        if (topClipped)
                            GUILayout.Label(Localizer.Format("FAREnvelopeTopSpeedClipped"));
                    }

                    GUILayout.EndVertical();
                }

                GUILayout.EndHorizontal();
            }

            GUILayout.BeginHorizontal();
            for (int t = start; t < end; t++)
            {
                GUILayout.BeginVertical(GUILayout.Width(col));
                MapCell cell = cells[t];
                if (cell.IsStability)
                    cell.Renderer.DrawStabilityMap(w, MapHeight, null, false);
                else
                    cell.Renderer.DrawEnvelopeMap(w, MapHeight, null, !multi);
                GUILayout.EndVertical();
            }

            GUILayout.EndHorizontal();
        }

        /// <summary>
        /// Endurance/Range colour toggle. On change, repaints every envelope texture (colour only —
        /// no re-sweep). Shared static so all maps switch together.
        /// </summary>
        private void DrawMetricToggle()
        {
            GUILayout.Label(Localizer.Format("FARHeatmapsColorBy"), GUILayout.Width(60));
            bool endurance = GUILayout.Toggle(HeatmapRenderer.ColorMetric == HeatmapRenderer.Metric.Endurance,
                                              Localizer.Format("FARHeatmapsEndurance"),
                                              GUILayout.Width(90));
            bool range = GUILayout.Toggle(HeatmapRenderer.ColorMetric == HeatmapRenderer.Metric.Range,
                                          Localizer.Format("FARHeatmapsRange"),
                                          GUILayout.Width(80));

            HeatmapRenderer.Metric want = HeatmapRenderer.ColorMetric;
            if (range && HeatmapRenderer.ColorMetric != HeatmapRenderer.Metric.Range)
                want = HeatmapRenderer.Metric.Range;
            else if (endurance && HeatmapRenderer.ColorMetric != HeatmapRenderer.Metric.Endurance)
                want = HeatmapRenderer.Metric.Endurance;

            if (want != HeatmapRenderer.ColorMetric)
            {
                HeatmapRenderer.ColorMetric = want;
                if (!IsSweepRunning)
                    foreach (HeatmapRenderer r in _envRenderers)
                        r.RepaintEnvelope();
            }
        }

        /// <summary>
        /// Kicks off a fresh sweep whenever the voxel model finishes rebuilding, so shape changes
        /// re-map without pressing Run. Only while the tab is visible and auto-recalc is on.
        /// </summary>
        private void CheckAutoRecalc()
        {
            if (!_autoRecalc)
            {
                _lastVoxQueued = false;
                return;
            }

            bool queued = EditorGUI.Instance != null && EditorGUI.Instance.VoxelizationUpdateQueued;

            // Fire on the queued -> done edge: a rebuild just finished and fresh aero data is ready.
            if (_lastVoxQueued && !queued && !IsSweepRunning && _simManager.SweepSim.IsReady())
                StartSweep();

            _lastVoxQueued = queued;
        }

        /// <summary>
        /// Repopulates the altitude range when the selected body changes. Scale height varies by
        /// body — Kerbin's is roughly 5.6 km against Earth's 8.5 km — so a range that frames an
        /// envelope on one badly misframes it on the other. Only fires on an actual change, so typed
        /// values survive.
        /// </summary>
        private void SyncAltitudeDefaultsToBody()
        {
            CelestialBody body = _bodySettingDropdown.ActiveSelection;
            if (body == null || ReferenceEquals(body, _lastBody))
                return;

            _lastBody = body;

            double suggested = PerformanceEnvelopeCalculator.SuggestedMaxAltitude(body);
            if (suggested <= 0)
                return;

            _altMinKm = "0";
            _altMaxKm = Math.Ceiling(suggested / 1000.0).ToString(CultureInfo.InvariantCulture);
        }

        private void StartSweep()
        {
            if (!TryParseRanges(out double machMin,
                                out double machMax,
                                out double altMin,
                                out double altMax,
                                out int nMach,
                                out int nAlt))
            {
                FARLogger.Info("Heatmap sweep aborted: bad ranges — " + _statusMessage);
                return;
            }

            if (!_simManager.SweepSim.IsReady())
            {
                _statusMessage = Localizer.Format("FAREnvelopeNotReady");
                FARLogger.Info("Heatmap sweep aborted: sim not ready (voxelization/aero data pending).");
                return;
            }

            SolverEnginesInterop.Initialize();
            if (!SolverEnginesInterop.Available)
            {
                _statusMessage = Localizer.Format("FAREngineDeckNoSolver");
                FARLogger.Info("Heatmap sweep aborted: SolverEngines/AJE interop unavailable.");
                return;
            }

            _deck.UpdateShip();
            if (!_deck.Ready)
            {
                _statusMessage = Localizer.Format("FAREngineDeckNoEngines");
                FARLogger.Info("Heatmap sweep aborted: engine deck not ready (no usable engines found).");
                return;
            }

            FARLogger.Info("Heatmap sweep starting: " + _deck.Groups.Count + " engine group(s), " +
                           nMach + "x" + nAlt + " cells.");

            CelestialBody body = _bodySettingDropdown.ActiveSelection;

            double[] machAxis = LinSpace(machMin, machMax, nMach);
            double[] altAxis = LinSpace(altMin, altMax, nAlt);
            var stability = new HeatmapResult.StabilityCell[nMach, nAlt]; // shared across types
            string craftName = EditorLogic.fetch != null && EditorLogic.fetch.ship != null
                                   ? EditorLogic.fetch.ship.shipName
                                   : "";

            // One envelope result + renderer per engine type, plus a combined all-engines map when
            // there is more than one type. Axes and the stability grid are shared by reference — only
            // the Envelope differs. _sweepGroups holds which engines each map uses; null = combined.
            //
            // Groups are ordered compressor engines (turbojets) first so the maps read turbojet →
            // ramjet → combined left to right.
            bool multiType = _deck.Groups.Count > 1;

            var ordered = new List<EditorEngineDeck.EngineGroup>();
            foreach (EditorEngineDeck.EngineGroup g in _deck.Groups)
                if (!g.Compressorless)
                    ordered.Add(g);
            foreach (EditorEngineDeck.EngineGroup g in _deck.Groups)
                if (g.Compressorless)
                    ordered.Add(g);

            _results = new List<HeatmapResult>();
            _sweepGroups = new List<EditorEngineDeck.EngineGroup>();
            _envRenderers.Clear();

            foreach (EditorEngineDeck.EngineGroup group in ordered)
            {
                AddResult(machAxis, altAxis, nMach, nAlt, stability, body, craftName,
                          multiType ? group.Label : "", group);
            }

            if (multiType)
                AddResult(machAxis, altAxis, nMach, nAlt, stability, body, craftName,
                          Localizer.Format("FARHeatmapsCombined"), null);

            _stabRenderer.SetResult(_results[0]); // shares the stability grid

            _cellsDone = 0;
            _cellsTotal = nMach * nAlt * (_results.Count * MassSamples + (_includeStability ? 1 : 0));
            _cancelRequested = false;

            _statusMessage = "";
            _sweep = EditorGUI.Instance.StartCoroutine(SweepCoroutine(body));
        }

        private void AddResult(
            double[] machAxis,
            double[] altAxis,
            int nMach,
            int nAlt,
            HeatmapResult.StabilityCell[,] stability,
            CelestialBody body,
            string craftName,
            string label,
            EditorEngineDeck.EngineGroup group
        )
        {
            // One envelope grid per mass sample; the displayed grid is the wet (last) one until flight
            // interpolates it. Filled in during the sweep.
            var samples = new PerformanceEnvelopeCalculator.CellResult[MassSamples][,];
            for (int k = 0; k < MassSamples; k++)
                samples[k] = new PerformanceEnvelopeCalculator.CellResult[nMach, nAlt];

            var result = new HeatmapResult
            {
                MachAxis = machAxis,
                AltAxis = altAxis,
                SpeedAxisMode = _speedInMS,
                EnvelopeSamples = samples,
                Envelope = samples[MassSamples - 1],
                Stability = stability,
                HasStability = _includeStability,
                EngineTypeLabel = label,
                BodyName = body != null ? body.bodyName : "",
                CraftName = craftName
            };
            _results.Add(result);
            _sweepGroups.Add(group);

            var renderer = new HeatmapRenderer();
            renderer.SetResult(result);
            _envRenderers.Add(renderer);
        }

        private IEnumerator SweepCoroutine(CelestialBody body)
        {
            FARAeroUtil.UpdateCurrentActiveBody(body);
            FARAeroUtil.ResetEditorParts();

            int flapSetting = _flapSettingDropdown.ActiveSelection;
            bool spoilers = _spoilers;
            bool includeStab = _includeStability;
            double ut = Planetarium.GetUniversalTime();

            PerformanceEnvelopeCalculator calc = _simManager.EnvelopeCalculator;
            PerformanceEnvelopeCalculator.VehicleProperties[] massProps = calc.ComputeMassSamples(_deck, MassSamples);
            PerformanceEnvelopeCalculator.VehicleProperties wetProps = massProps[massProps.Length - 1];

            if (!wetProps.Valid)
            {
                _statusMessage = Localizer.Format("FAREnvelopeNoVehicle");
                FARLogger.Info("Heatmap sweep aborted: vehicle properties invalid (mass/reference area).");
                _sweep = null;
                yield break;
            }

            double[] machAxis = _results[0].MachAxis;
            double[] altAxis = _results[0].AltAxis;

            double dryMass = massProps[0].MassKg; // sample 0 is dry (no usable fuel)
            int partCount = CountPhysicalParts();
            foreach (HeatmapResult r in _results)
            {
                r.MassKg = wetProps.MassKg; // displayed grid is the wet sample until flight interpolates
                r.RefAreaM2 = wetProps.Area;
                r.PartCount = partCount;
                r.DryMassKg = dryMass;
                r.SampleMassesKg = new double[massProps.Length];
                for (int k = 0; k < massProps.Length; k++)
                    r.SampleMassesKg[k] = massProps[k].MassKg;
            }

            // Stability is engine-independent, so sweep it once into the shared grid.
            if (includeStab)
                for (int j = 0; j < altAxis.Length; j++)
                {
                    if (_cancelRequested)
                    {
                        Cancelled();
                        yield break;
                    }

                    double alt = altAxis[j];
                    if (FARAtmosphere.GetPressure(body, new Vector3d(0, 0, alt), ut) <= 0)
                    {
                        _cellsDone += machAxis.Length;
                        continue;
                    }

                    double sos = SpeedOfSoundAt(body, alt);

                    for (int i = 0; i < machAxis.Length; i++)
                    {
                        double mach = ColumnMach(machAxis[i], sos);
                        var cell = new HeatmapResult.StabilityCell();
                        try
                        {
                            StabilityDerivOutput output =
                                _simManager.StabDerivCalculator.CalculateStabilityDerivs(body,
                                                                                         alt,
                                                                                         mach,
                                                                                         flapSetting,
                                                                                         spoilers,
                                                                                         0,
                                                                                         0,
                                                                                         0);
                            CollectOutOfRange(output, cell);
                            cell.Score = cell.OutOfRange.Count;
                        }
                        catch (Exception e)
                        {
                            FARLogger.Exception(e, $"while evaluating stability cell M {mach}, alt {alt}");
                            cell.Score = -1;
                        }

                        _results[0].Stability[i, j] = cell;
                        _stabRenderer.PaintStabilityCell(i, j);
                        _cellsDone++;
                    }

                    _stabRenderer.ApplyStability();
                    yield return null;
                }

            // Envelope: cell-outer, so every engine configuration at a cell shares one aerodynamic
            // solve — the expensive InstantConditionSim trim is done once per (cell, mass) by BeginCell
            // and reused for each engine type and the combined map (EvaluateWithAero). The combined map
            // is a real summed-thrust evaluation; where running every engine would overheat, it sheds
            // the turbojets and shows the compressor-less engines alone (ReducedEngines, dimmed). The
            // whole grid is swept at each mass sample so the flight scene can interpolate it as fuel
            // burns — the mass loop is inside the cell so it shares nothing, since trim depends on mass.
            int mapCount = _results.Count;
            int massCount = massProps.Length;

            for (int j = 0; j < altAxis.Length; j++)
            {
                if (_cancelRequested)
                {
                    Cancelled();
                    yield break;
                }

                double alt = altAxis[j];
                if (FARAtmosphere.GetPressure(body, new Vector3d(0, 0, alt), ut) <= 0)
                {
                    _cellsDone += machAxis.Length * mapCount * massCount;
                    continue;
                }

                double sos = SpeedOfSoundAt(body, alt);

                for (int i = 0; i < machAxis.Length; i++)
                {
                    double mach = ColumnMach(machAxis[i], sos);

                    for (int k = 0; k < massCount; k++)
                    {
                        PerformanceEnvelopeCalculator.VehicleProperties props = massProps[k];

                        // BeginCell sets the shared AeroCondition; it must be inside the try so EndCell
                        // still clears it if BeginCell (the trim solve) throws — a leaked AeroCondition
                        // silently corrupts the stability/static tabs that expect it null.
                        PerformanceEnvelopeCalculator.AeroContext ctx = default;
                        try
                        {
                            ctx = calc.BeginCell(body, alt, mach, flapSetting, spoilers, props);
                            for (int t = 0; t < mapCount; t++)
                            {
                                bool isCombined = _sweepGroups[t] == null;
                                _deck.SetActiveGroup(_sweepGroups[t]); // null => combined (all engines)

                                PerformanceEnvelopeCalculator.CellResult cell;
                                try
                                {
                                    cell = calc.EvaluateWithAero(ctx, body, alt, mach, _deck, props);

                                    // Combined map: running every engine here would melt the turbojets,
                                    // so the honest result is the aircraft on ram power alone. Use it
                                    // whether or not it sustains level flight — you would shed the
                                    // melting turbojets regardless. Keeping the ram result even when it
                                    // is infeasible also keeps the top-speed edge continuous: the cell
                                    // just past the ramjet's reach then carries ram's negative excess
                                    // thrust, so the thrust=drag crossing interpolates instead of
                                    // snapping to the grid (an all-engines overheat cell has positive
                                    // excess and would break that, reading as a lower top speed).
                                    if (isCombined && cell.Overheats && _deck.CanShedToCompressorless)
                                    {
                                        _deck.SetActiveCompressorless();
                                        PerformanceEnvelopeCalculator.CellResult ram =
                                            calc.EvaluateWithAero(ctx, body, alt, mach, _deck, props);
                                        ram.ReducedEngines = true;
                                        cell = ram;
                                    }
                                }
                                catch (Exception e)
                                {
                                    FARLogger.Exception(e, $"while evaluating envelope cell M {mach}, alt {alt}");
                                    cell = default;
                                }

                                _results[t].EnvelopeSamples[k][i, j] = cell;
                                _cellsDone++;
                            }
                        }
                        finally
                        {
                            calc.EndCell();
                        }
                    }
                }

                // The displayed grid is the wet sample (Envelope references EnvelopeSamples[last]).
                for (int t = 0; t < mapCount; t++)
                    _envRenderers[t].PaintEnvelopeRow(j);

                yield return null;
            }

            for (int t = 0; t < mapCount; t++)
            {
                _results[t].Recompute();
                _envRenderers[t].RepaintEnvelope(); // endurance colours scale to each map's own best cell
            }

            _deck.Restore();

            HeatmapResult.Cached = _results;
            _statusMessage = Localizer.Format("FAREnvelopeDone", _cellsDone.ToString(CultureInfo.InvariantCulture));
            FARLogger.Info("Heatmap sweep complete: " + _cellsDone + " cells across " + _results.Count + " map(s).");
            _sweep = null;
        }

        private void Cancelled()
        {
            _statusMessage = Localizer.Format("FAREnvelopeCancelled");
            _deck.Restore();
            _sweep = null;
        }

        private static int CountPhysicalParts()
        {
            int n = 0;
            List<Part> parts = EditorLogic.SortedShipList;
            if (parts == null)
                return 0;
            foreach (Part p in parts)
                if (!FARAeroUtil.IsNonphysical(p))
                    n++;
            return n;
        }

        private static void CollectOutOfRange(StabilityDerivOutput output, HeatmapResult.StabilityCell into)
        {
            foreach ((int idx, int sign, string name) in CheckedDerivs)
            {
                double v = output.stabDerivs[idx];
                if (double.IsNaN(v) || Math.Sign(v) != sign)
                    into.OutOfRange.Add((name, v, sign));
            }
        }

        private bool TryParseRanges(
            out double machMin,
            out double machMax,
            out double altMin,
            out double altMax,
            out int nMach,
            out int nAlt
        )
        {
            machMin = machMax = altMin = altMax = 0;
            nMach = nAlt = 0;

            if (!double.TryParse(_machMin, NumberStyles.Float, CultureInfo.InvariantCulture, out machMin) ||
                !double.TryParse(_machMax, NumberStyles.Float, CultureInfo.InvariantCulture, out machMax))
            {
                _statusMessage = "Mach range must be numeric.";
                return false;
            }

            if (machMax <= machMin || machMin <= 0)
            {
                _statusMessage = "Mach range must be increasing and above zero.";
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

            // Capped low: every cell costs at least one aero trim solve, and with stability enabled
            // a full derivative set on top. The maps are meant to be hovered, not zoomed.
            if (!int.TryParse(_nMach, out nMach) || nMach < 2 || nMach > 30 ||
                !int.TryParse(_nAlt, out nAlt) || nAlt < 2 || nAlt > 30)
            {
                _statusMessage = "Cell counts must be integers between 2 and 30.";
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

        private static double SpeedOfSoundAt(CelestialBody body, double alt)
        {
            return EditorAtmosphere.GetGasProperties(body, alt).SpeedOfSound;
        }

        /// <summary>
        /// The Mach for an x-axis column at one altitude. In Mach mode the column value is already the
        /// Mach; in speed mode it is a true airspeed that maps to a different Mach at each altitude via
        /// the local sound speed.
        /// </summary>
        private double ColumnMach(double axisValue, double speedOfSound)
        {
            return _speedInMS && speedOfSound > 0 ? axisValue / speedOfSound : axisValue;
        }

        /// <summary>
        /// Flips the x-axis between Mach and true airspeed, converting the entered range through the
        /// body's sea-level sound speed so the swept span keeps roughly the same physical speeds.
        /// </summary>
        private void ToggleSpeedUnit()
        {
            CelestialBody body = _bodySettingDropdown.ActiveSelection;
            double sos = body != null ? SpeedOfSoundAt(body, 0) : 0;
            bool toSpeed = !_speedInMS;

            if (sos > 0 &&
                double.TryParse(_machMin, NumberStyles.Float, CultureInfo.InvariantCulture, out double lo) &&
                double.TryParse(_machMax, NumberStyles.Float, CultureInfo.InvariantCulture, out double hi))
            {
                double f = toSpeed ? sos : 1.0 / sos;
                _machMin = (lo * f).ToString(toSpeed ? "F0" : "F2", CultureInfo.InvariantCulture);
                _machMax = (hi * f).ToString(toSpeed ? "F0" : "F2", CultureInfo.InvariantCulture);
            }

            _speedInMS = toSpeed;
        }

        public void Cleanup()
        {
            SaveSettings();

            if (_sweep != null && EditorGUI.Instance != null)
            {
                EditorGUI.Instance.StopCoroutine(_sweep);
                _sweep = null;
            }

            // Textures are owned by the renderers; the results themselves stay alive in
            // HeatmapResult.Cached for the flight scene.
            foreach (HeatmapRenderer r in _envRenderers)
                r.Cleanup();
            _stabRenderer.Cleanup();
        }
    }
}
