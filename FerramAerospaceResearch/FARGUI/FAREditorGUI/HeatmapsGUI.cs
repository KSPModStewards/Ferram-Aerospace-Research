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

        // Auto-recalc debounce: each settled shape change (a finished voxel rebuild) resets a short
        // timer, and the regen only fires once no further change has arrived for this long — so a burst
        // of edits coalesces into a single sweep instead of one per edit. A change also cancels any
        // in-flight sweep immediately, since it is computing an already-stale shape.
        private const float RecalcDebounceSeconds = 0.5f;
        private bool _recalcPending;
        private float _recalcDueTime;

        // Multithread the envelope sweep across altitude rows. On by default; the serial path is kept
        // as a fallback for comparing results and timings.
        private bool _multithread = true;

        // Speed axis in true airspeed (m/s) rather than Mach. The x-range fields and the map x-axis
        // both follow this; each cell's Mach is then derived per altitude from the local sound speed.
        private bool _speedInMS;

        private List<HeatmapResult> _results;
        private List<EditorEngineDeck.EngineGroup> _sweepGroups; // engines per map; null = combined

        // Completed-sweep history so the user can flip back through previous generations and compare
        // (e.g. serial vs multithreaded, or before/after a design tweak). Only the map data is kept per
        // generation — the textures are rebuilt on demand when a generation is shown, so N generations
        // cost N sets of result grids, not N sets of GPU textures.
        private readonly List<Generation> _history = new List<Generation>();
        private int _viewIndex = -1;
        private int _genCounter;
        private const int MaxHistory = 12;

        /// <summary>One completed sweep kept for the history navigator: the map data and a label.</summary>
        private sealed class Generation
        {
            public readonly List<HeatmapResult> Results;
            public readonly string Label;

            public Generation(List<HeatmapResult> results, string label)
            {
                Results = results;
                Label = label;
            }
        }
        private Coroutine _sweep;

        // The running parallel worker task (Phase A trim, or the merged trim+engine sweep), held so a
        // cancel can join it synchronously — see CancelAndJoin.
        private System.Threading.Tasks.Task _sweepTask;

        // Written on the main thread (Display/Cancel/auto-recalc), read by the Parallel.For workers'
        // cancel checks — volatile so an in-flight worker is guaranteed to observe a mid-sweep cancel.
        private volatile bool _cancelRequested;

        // Diagnostics for the parallel trim: worker exceptions blank a cell silently, so count them and
        // log the first with its location.
        private volatile bool _parallelErrorLogged;
        private int _parallelErrorCount;
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

            // The parallel sweep holds live references to the ship's aero part-modules; any ship edit
            // (move, delete, undo, redo, load) can destroy or rebuild those on the main thread while the
            // worker threads read them — a native crash. These editor events fire synchronously in
            // Update, at the moment of the edit and earlier in the frame than the GUI, so joining the
            // workers here stops them before the parts are torn down. Mirrors the event set FAR's own
            // EditorGUI hooks for re-voxelization.
            GameEvents.onEditorShipModified.Add(OnEditorShipEvent);
            GameEvents.onEditorUndo.Add(OnEditorShipEvent);
            GameEvents.onEditorRedo.Add(OnEditorShipEvent);
            GameEvents.onEditorPartEvent.Add(OnEditorPartEvent);
        }

        private void OnEditorShipEvent(ShipConstruct ship)
        {
            if (IsSweepRunning)
                CancelAndJoin();
        }

        private void OnEditorPartEvent(ConstructionEventType type, Part part)
        {
            if (IsSweepRunning)
                CancelAndJoin();
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
            _multithread = config.GetValue("heatmap_multithread", _multithread ? 1 : 0) != 0;

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
            config.SetValue("heatmap_multithread", _multithread ? 1 : 0);
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
            GUILayout.Space(14);
            _multithread = GUILayout.Toggle(_multithread, "Multithread", GUILayout.Width(110));
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

            DrawHistoryNav();

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
        /// Requests cancellation and synchronously joins the running worker task, so once this returns
        /// no sweep worker thread is executing. Called on the main thread when an edit is about to
        /// re-voxelize the shared aero model the workers read — they must be stopped first or the
        /// re-voxelization races them (a native crash). Workers observe the volatile cancel flag and
        /// bail within a Mach column, so the join is brief; the coroutine tears down on its next step.
        /// </summary>
        private void CancelAndJoin()
        {
            _cancelRequested = true;

            System.Threading.Tasks.Task t = _sweepTask;
            if (t == null || t.IsCompleted)
                return;

            try
            {
                t.Wait();
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "while joining a cancelled envelope sweep");
            }
        }

        /// <summary>
        /// Kicks off a fresh sweep whenever the voxel model finishes rebuilding, so shape changes
        /// re-map without pressing Run. Only while the tab is visible and auto-recalc is on.
        /// </summary>
        private void CheckAutoRecalc()
        {
            bool queued = EditorGUI.Instance != null && EditorGUI.Instance.VoxelizationUpdateQueued;

            // Safety, independent of auto-recalc: a pending re-voxelization while a sweep runs means the
            // editor is about to mutate the shared aero model under the worker threads — a native crash.
            // Cancel AND join them now, in this same frame, before the editor gets the main thread back
            // to apply the new voxel model. Applies to manual sweeps too, not just auto-recalc ones.
            if (queued && IsSweepRunning)
                CancelAndJoin();

            if (!_autoRecalc)
            {
                _lastVoxQueued = queued;
                _recalcPending = false;
                return;
            }

            // On the queued -> done edge the shape has settled to a new state; arm (or re-arm) the
            // debounce so the regen waits until edits stop before firing.
            if (_lastVoxQueued && !queued)
            {
                _recalcPending = true;
                _recalcDueTime = Time.realtimeSinceStartup + RecalcDebounceSeconds;
            }

            _lastVoxQueued = queued;

            // Fire once the debounce has elapsed with no newer change, the stale sweep has torn down, and
            // fresh aero data is ready.
            if (_recalcPending &&
                Time.realtimeSinceStartup >= _recalcDueTime &&
                !IsSweepRunning &&
                _simManager.SweepSim.IsReady())
            {
                _recalcPending = false;
                StartSweep();
            }
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
            foreach (HeatmapRenderer r in _envRenderers)
                r.Cleanup(); // release textures before rebuilding; navigation can leave live ones here
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
            int nMach = machAxis.Length;
            int nAlt = altAxis.Length;

            if (_multithread)
            {
                // Parallel sweep. When the interop can drive per-worker engine solver clones, the trim
                // and engine passes run merged in ONE parallel loop over altitude rows, so each row's
                // cells paint the moment that row finishes (progressive fill) instead of the whole map
                // appearing only at the end. Without clones it falls back to a parallel trim pass then a
                // serial engine pass. Both consume the atmosphere/inlet state sampled here up front.
                var rowValid = new bool[nAlt];
                var machCell = new double[nMach, nAlt];
                var gasCell = new GasProperties[nMach, nAlt];
                var aeroCondCell = new SimAeroCondition[nMach, nAlt];
                var envGrid = new EditorEngineDeck.CellEnv[nMach, nAlt];

                // Bootstrap = all the main-thread prep before any parallel compute (atmosphere/inlet
                // sampling, clone minting, warm-up). Timed so we know how much of the pre-fill delay is
                // setup versus the sweep itself.
                var bootstrapTimer = System.Diagnostics.Stopwatch.StartNew();

                // Sample the atmosphere and inlet geometry on the main thread — CelestialBody/Planetarium
                // and Unity transforms are main-thread only. The trim and the engine drive then consume
                // these plain values, so both can run on worker threads.
                for (int j = 0; j < nAlt; j++)
                {
                    double altPre = altAxis[j];
                    if (FARAtmosphere.GetPressure(body, new Vector3d(0, 0, altPre), ut) <= 0)
                        continue;

                    rowValid[j] = true;
                    double sos = SpeedOfSoundAt(body, altPre);
                    for (int i = 0; i < nMach; i++)
                    {
                        double mach = ColumnMach(machAxis[i], sos);
                        machCell[i, j] = mach;
                        calc.GetCellAtmosphere(body, altPre, mach, out GasProperties g, out SimAeroCondition ac);
                        gasCell[i, j] = g;
                        aeroCondCell[i, j] = ac;
                        envGrid[i, j] = _deck.BuildCellEnv(g, altPre, mach);
                    }
                }

                _parallelErrorLogged = false;
                _parallelErrorCount = 0;

                // The slot pool (SimThreadContext) recycles a slot per row, so it only needs one per
                // concurrent worker plus a margin: one engine solver clone per slot a sweep worker can hold
                // (see SimThreadContext.SweepPoolSize). This is bounded by concurrency, not by how many threads
                // the pool churns through over the sweep, so it can't run out and force two workers to share.
                int slotCount = ferram4.SimThreadContext.SweepPoolSize + 1;

                // Mint the per-worker engine solver clones (main thread). If unavailable — an older
                // SolverEngines whose internals the interop cannot drive — Phase B falls back to the
                // serial engine pass below and the sweep still completes.
                bool engineParallel = _deck.BeginParallel(slotCount);
                EditorEngineDeck.ParallelGroupContext[] groupCtx = null;
                EditorEngineDeck.ParallelGroupContext compressorlessCtx = null;
                if (engineParallel)
                {
                    groupCtx = new EditorEngineDeck.ParallelGroupContext[mapCount];
                    for (int t = 0; t < mapCount; t++)
                        groupCtx[t] = _deck.BuildGroupContext(_sweepGroups[t]);
                    if (_deck.CanShedToCompressorless)
                        compressorlessCtx = _deck.CompressorlessContext();
                }

                // Leave a core for the main thread so KSP keeps rendering; never exceed the slot budget.
                var parallelOptions = new System.Threading.Tasks.ParallelOptions
                {
                    MaxDegreeOfParallelism = Math.Max(1, Math.Min(slotCount - 1, Environment.ProcessorCount - 1))
                };

                calc.BeginParallelSweep();
                try
                {
                    // Warm any lazily-initialised static caches in the solve path on the main thread
                    // first, so the workers never race to initialise them. Run both the lowest- and
                    // highest-Mach columns of the first valid row so the subsonic and supersonic paths
                    // (and their distinct lazy curves) are both built here.
                    for (int jw = 0; jw < nAlt; jw++)
                    {
                        if (!rowValid[jw])
                            continue;
                        calc.BeginCellPrecomputed(body, altAxis[jw], machCell[0, jw], flapSetting, spoilers,
                                                  massProps[0], 0, gasCell[0, jw], aeroCondCell[0, jw]);
                        calc.BeginCellPrecomputed(body, altAxis[jw], machCell[nMach - 1, jw], flapSetting, spoilers,
                                                  massProps[0], 0, gasCell[nMach - 1, jw], aeroCondCell[nMach - 1, jw]);
                        break;
                    }

                    bootstrapTimer.Stop();

                    if (engineParallel)
                    {
                        // Merged trim+engine, one worker per altitude row. A row is marked done when all
                        // its cells (every Mach, mass and map) are written; the main thread paints those
                        // rows as they land, so the map fills progressively. Trim and engine time are
                        // accumulated per thread-slot (no contention) to keep the split visible even
                        // though the two are now interleaved.
                        var rowDone = new bool[nAlt];
                        var rowPainted = new bool[nAlt];
                        var trimTicks = new long[ferram4.SimThreadContext.MaxSlots];
                        var engineTicks = new long[ferram4.SimThreadContext.MaxSlots];
                        var sweepTimer = System.Diagnostics.Stopwatch.StartNew();

                        System.Threading.Tasks.Task task = System.Threading.Tasks.Task.Run(() =>
                            System.Threading.Tasks.Parallel.For(0, nAlt, parallelOptions, j =>
                            {
                                if (_cancelRequested)
                                {
                                    System.Threading.Volatile.Write(ref rowDone[j], true);
                                    return;
                                }

                                if (!rowValid[j])
                                {
                                    System.Threading.Interlocked.Add(ref _cellsDone, nMach * mapCount * massCount);
                                    System.Threading.Volatile.Write(ref rowDone[j], true);
                                    return;
                                }

                                // Claim a pooled slot for this row's per-thread scratch/clones, returned
                                // below. The row body catches its own exceptions, so this always pairs.
                                ferram4.SimThreadContext.AcquireSlot();
                                int slot = ferram4.SimThreadContext.Slot;
                                double altRow = altAxis[j];
                                for (int i = 0; i < nMach; i++)
                                {
                                    // Bail promptly on cancel (a design change mid-sweep) rather than
                                    // finishing the whole row — keeps auto-recalc responsive.
                                    if (_cancelRequested)
                                        break;

                                    double mach = machCell[i, j];
                                    for (int k = 0; k < massCount; k++)
                                    {
                                        PerformanceEnvelopeCalculator.AeroContext ctx;
                                        long ts0 = System.Diagnostics.Stopwatch.GetTimestamp();
                                        try
                                        {
                                            ctx = calc.BeginCellPrecomputed(body, altRow, mach, flapSetting, spoilers,
                                                                            massProps[k], 0,
                                                                            gasCell[i, j], aeroCondCell[i, j]);
                                        }
                                        catch (Exception e)
                                        {
                                            System.Threading.Interlocked.Increment(ref _parallelErrorCount);
                                            if (!_parallelErrorLogged)
                                            {
                                                _parallelErrorLogged = true;
                                                FARLogger.Exception(e,
                                                                    $"parallel trim threw at i{i} j{j} k{k} (M {mach:F2}, alt {altRow:F0})");
                                            }

                                            System.Threading.Interlocked.Add(ref _cellsDone, mapCount);
                                            continue;
                                        }

                                        long ts1 = System.Diagnostics.Stopwatch.GetTimestamp();

                                        // The feasibility trim reads the sim's per-thread viscous condition.
                                        calc.SetAeroCondition(aeroCondCell[i, j]);
                                        try
                                        {
                                            EvaluateCellEnginesParallel(calc, ctx, envGrid[i, j],
                                                                        groupCtx, compressorlessCtx, slot,
                                                                        massProps[k], i, j, k, mapCount);
                                        }
                                        finally
                                        {
                                            calc.SetAeroCondition(null);
                                        }

                                        long ts2 = System.Diagnostics.Stopwatch.GetTimestamp();
                                        trimTicks[slot] += ts1 - ts0;
                                        engineTicks[slot] += ts2 - ts1;
                                    }
                                }

                                ferram4.SimThreadContext.ReleaseSlot();
                                System.Threading.Volatile.Write(ref rowDone[j], true);
                            }));
                        _sweepTask = task;

                        while (!task.IsCompleted)
                        {
                            // On cancel, stop yielding and block-join immediately (below). Continuing to
                            // yield would hand the main thread back to the editor, which — when the cancel
                            // came from an edit — is about to re-voxelize the shared aero model while the
                            // worker threads are still reading it. That race is a native crash, so the
                            // workers must be joined before any more main-thread work runs.
                            if (_cancelRequested)
                                break;

                            PaintFinishedRows(rowValid, rowDone, rowPainted, mapCount);
                            yield return null;
                        }

                        try
                        {
                            task.Wait();
                        }
                        catch (Exception e)
                        {
                            FARLogger.Exception(e, "while joining the parallel envelope sweep");
                        }

                        _sweepTask = null;

                        PaintFinishedRows(rowValid, rowDone, rowPainted, mapCount);
                        sweepTimer.Stop();

                        if (task.IsFaulted)
                            FARLogger.Exception(task.Exception, "while running the parallel envelope sweep");

                        if (_cancelRequested)
                        {
                            Cancelled();
                            yield break;
                        }

                        long freq = System.Diagnostics.Stopwatch.Frequency;
                        long trimSum = 0, engineSum = 0;
                        for (int s = 0; s < trimTicks.Length; s++)
                        {
                            trimSum += trimTicks[s];
                            engineSum += engineTicks[s];
                        }

                        if (_parallelErrorCount > 0)
                            FARLogger.Info("parallel sweep: " + _parallelErrorCount +
                                           " cell(s) threw and were blanked (see the first exception above).");
                        FARLogger.Info("Heatmap envelope timings: bootstrap " + bootstrapTimer.ElapsedMilliseconds +
                                       " ms, sweep " + sweepTimer.ElapsedMilliseconds + " ms (parallel); of which trim ~" +
                                       trimSum * 1000 / freq + " ms + engine ~" + engineSum * 1000 / freq +
                                       " ms CPU-summed across " + parallelOptions.MaxDegreeOfParallelism + " threads.");
                    }
                    else
                    {
                        // Fallback (older SolverEngines): parallel trim, then serial engine pass.
                        var ctxGrid = new PerformanceEnvelopeCalculator.AeroContext[nMach, nAlt, massCount];
                        var trimTimer = System.Diagnostics.Stopwatch.StartNew();
                        System.Threading.Tasks.Task task = System.Threading.Tasks.Task.Run(() =>
                            System.Threading.Tasks.Parallel.For(0, nAlt, parallelOptions, j =>
                            {
                                if (!rowValid[j] || _cancelRequested)
                                    return;

                                double altRow = altAxis[j];
                                for (int i = 0; i < nMach; i++)
                                {
                                    if (_cancelRequested)
                                        break;

                                    double mach = machCell[i, j];
                                    for (int k = 0; k < massCount; k++)
                                    {
                                        try
                                        {
                                            ctxGrid[i, j, k] =
                                                calc.BeginCellPrecomputed(body, altRow, mach, flapSetting, spoilers,
                                                                          massProps[k], 0,
                                                                          gasCell[i, j], aeroCondCell[i, j]);
                                        }
                                        catch (Exception e)
                                        {
                                            System.Threading.Interlocked.Increment(ref _parallelErrorCount);
                                            if (!_parallelErrorLogged)
                                            {
                                                _parallelErrorLogged = true;
                                                FARLogger.Exception(e,
                                                                    $"parallel trim threw at i{i} j{j} k{k} (M {mach:F2}, alt {altRow:F0})");
                                            }

                                            ctxGrid[i, j, k] = default;
                                        }
                                    }
                                }
                            }));
                        _sweepTask = task;

                        while (!task.IsCompleted)
                        {
                            if (_cancelRequested)
                                break;
                            yield return null;
                        }

                        try
                        {
                            task.Wait();
                        }
                        catch (Exception e)
                        {
                            FARLogger.Exception(e, "while joining the parallel envelope trim");
                        }

                        _sweepTask = null;

                        trimTimer.Stop();
                        if (task.IsFaulted)
                            FARLogger.Exception(task.Exception, "while running the parallel envelope trim");

                        var engineTimer = System.Diagnostics.Stopwatch.StartNew();
                        for (int j = 0; j < nAlt; j++)
                        {
                            if (_cancelRequested)
                            {
                                Cancelled();
                                yield break;
                            }

                            if (!rowValid[j])
                            {
                                _cellsDone += nMach * mapCount * massCount;
                                continue;
                            }

                            double altRow = altAxis[j];
                            for (int i = 0; i < nMach; i++)
                            {
                                double mach = machCell[i, j];
                                for (int k = 0; k < massCount; k++)
                                {
                                    calc.SetAeroCondition(aeroCondCell[i, j]);
                                    try
                                    {
                                        EvaluateCellEngines(calc, ctxGrid[i, j, k], body, altRow, mach,
                                                            massProps[k], i, j, k, mapCount);
                                    }
                                    finally
                                    {
                                        calc.SetAeroCondition(null);
                                    }
                                }
                            }

                            for (int t = 0; t < mapCount; t++)
                                _envRenderers[t].PaintEnvelopeRow(j);

                            yield return null;
                        }

                        engineTimer.Stop();
                        if (_parallelErrorCount > 0)
                            FARLogger.Info("parallel sweep: " + _parallelErrorCount +
                                           " cell(s) threw and were blanked (see the first exception above).");
                        FARLogger.Info("Heatmap envelope timings: bootstrap " + bootstrapTimer.ElapsedMilliseconds +
                                       " ms, trim " + trimTimer.ElapsedMilliseconds + " ms (parallel), engine " +
                                       engineTimer.ElapsedMilliseconds + " ms (serial).");
                    }
                }
                finally
                {
                    calc.EndParallelSweep();
                    _deck.EndParallel();
                }
            }
            else
            {
                // Serial fallback (no snapshot): the original single-threaded path, kept for comparing
                // results and timings against the multithreaded sweep.
                var serialTimer = System.Diagnostics.Stopwatch.StartNew();

                for (int j = 0; j < nAlt; j++)
                {
                    if (_cancelRequested)
                    {
                        Cancelled();
                        yield break;
                    }

                    double alt = altAxis[j];
                    if (FARAtmosphere.GetPressure(body, new Vector3d(0, 0, alt), ut) <= 0)
                    {
                        _cellsDone += nMach * mapCount * massCount;
                        continue;
                    }

                    double sos = SpeedOfSoundAt(body, alt);

                    for (int i = 0; i < nMach; i++)
                    {
                        double mach = ColumnMach(machAxis[i], sos);

                        for (int k = 0; k < massCount; k++)
                        {
                            PerformanceEnvelopeCalculator.VehicleProperties props = massProps[k];

                            // BeginCell sets the AeroCondition; it must be inside the try so EndCell still
                            // clears it if BeginCell (the trim solve) throws — a leaked AeroCondition
                            // silently corrupts the stability/static tabs that expect it null.
                            PerformanceEnvelopeCalculator.AeroContext ctx = default;
                            try
                            {
                                ctx = calc.BeginCell(body, alt, mach, flapSetting, spoilers, props);
                                EvaluateCellEngines(calc, ctx, body, alt, mach, props, i, j, k, mapCount);
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

                serialTimer.Stop();
                FARLogger.Info("Heatmap envelope timings: " + serialTimer.ElapsedMilliseconds + " ms (serial).");
            }

            for (int t = 0; t < mapCount; t++)
            {
                foreach (PerformanceEnvelopeCalculator.CellResult[,] grid in _results[t].EnvelopeSamples)
                    CleanIsolatedGreyDots(grid, nMach, nAlt);
                _results[t].Recompute();
                _envRenderers[t].RepaintEnvelope(); // endurance colours scale to each map's own best cell
            }

            _deck.Restore();

            HeatmapResult.Cached = _results;
            AppendGeneration();
            _statusMessage = Localizer.Format("FAREnvelopeDone", _cellsDone.ToString(CultureInfo.InvariantCulture));
            FARLogger.Info("Heatmap sweep complete: " + _cellsDone + " cells across " + _results.Count + " map(s).");
            _sweep = null;
        }

        /// <summary>Records the just-completed sweep in the history and selects it. Oldest drops off.</summary>
        private void AppendGeneration()
        {
            _genCounter++;
            _history.Add(new Generation(_results, BuildGenerationLabel()));
            while (_history.Count > MaxHistory)
                _history.RemoveAt(0);
            _viewIndex = _history.Count - 1;
        }

        private string BuildGenerationLabel()
        {
            HeatmapResult r = _results[0];
            int nMach = r.MachAxis?.Length ?? 0;
            int nAlt = r.AltAxis?.Length ?? 0;
            return string.Format(CultureInfo.InvariantCulture,
                                 "#{0} · {1}×{2} · {3} · {4}",
                                 _genCounter,
                                 nMach,
                                 nAlt,
                                 _multithread ? "MT" : "serial",
                                 DateTime.Now.ToString("HH:mm:ss", CultureInfo.InvariantCulture));
        }

        /// <summary>
        /// Shows a stored generation: repoints the current data and rebuilds the renderer textures from
        /// it. Cheap enough for a button press; not called per frame.
        /// </summary>
        private void ShowGeneration(int index)
        {
            if (index < 0 || index >= _history.Count)
                return;

            _viewIndex = index;
            _results = _history[index].Results;

            foreach (HeatmapRenderer r in _envRenderers)
                r.Cleanup();
            _envRenderers.Clear();

            foreach (HeatmapResult res in _results)
            {
                var renderer = new HeatmapRenderer();
                renderer.SetResult(res);
                _envRenderers.Add(renderer);
            }

            _stabRenderer.SetResult(_results[0]);
        }

        private void ClearHistory()
        {
            _history.Clear();
            _viewIndex = -1;
        }

        /// <summary>
        /// Prev/Next navigator across the sweep history, with the selected generation's label and a
        /// clear button. Hidden while a sweep is running (the maps are being painted live then).
        /// </summary>
        private void DrawHistoryNav()
        {
            if (IsSweepRunning || _history.Count == 0 || _viewIndex < 0)
                return;

            GUILayout.BeginHorizontal();

            GUI.enabled = _viewIndex > 0;
            if (GUILayout.Button("◄", GUILayout.Width(28)))
                ShowGeneration(_viewIndex - 1);
            GUI.enabled = true;

            string tag = _viewIndex == _history.Count - 1 ? "" : "   (history)";
            GUILayout.Label(string.Format(CultureInfo.InvariantCulture,
                                          "Gen {0}/{1}:  {2}{3}",
                                          _viewIndex + 1,
                                          _history.Count,
                                          _history[_viewIndex].Label,
                                          tag),
                            GUILayout.Width(470));

            GUI.enabled = _viewIndex < _history.Count - 1;
            if (GUILayout.Button("►", GUILayout.Width(28)))
                ShowGeneration(_viewIndex + 1);
            GUI.enabled = true;

            GUILayout.FlexibleSpace();
            if (GUILayout.Button("Clear", GUILayout.Width(55)))
                ClearHistory();

            GUILayout.EndHorizontal();
        }

        /// <summary>
        /// The engine-dependent half of a cell: evaluate every engine-type map (and the combined map,
        /// with its turbojet-shedding fallback) against the already-solved cruise aerodynamics, storing
        /// each result. Runs on the main thread — the engine deck mutates AJE state and reads Unity.
        /// </summary>
        private void EvaluateCellEngines(
            PerformanceEnvelopeCalculator calc,
            PerformanceEnvelopeCalculator.AeroContext ctx,
            CelestialBody body,
            double alt,
            double mach,
            PerformanceEnvelopeCalculator.VehicleProperties props,
            int i,
            int j,
            int k,
            int mapCount
        )
        {
            for (int t = 0; t < mapCount; t++)
            {
                bool isCombined = _sweepGroups[t] == null;
                _deck.SetActiveGroup(_sweepGroups[t]); // null => combined (all engines)

                PerformanceEnvelopeCalculator.CellResult cell;
                try
                {
                    cell = calc.EvaluateWithAero(ctx, body, alt, mach, _deck, props);

                    // Combined map: running every engine here would melt the turbojets, so the honest
                    // result is the aircraft on ram power alone. Use it whether or not it sustains level
                    // flight — you would shed the melting turbojets regardless. Keeping the ram result
                    // even when it is infeasible also keeps the top-speed edge continuous: the cell just
                    // past the ramjet's reach then carries ram's negative excess thrust, so the
                    // thrust=drag crossing interpolates instead of snapping to the grid.
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

        /// <summary>
        /// The worker-thread counterpart of <see cref="EvaluateCellEngines" />: evaluates every map for
        /// one cell against the slot's solver clones and the pre-built engine environment. Writes only
        /// its own (i, j, k) cells, so distinct rows never collide; <see cref="_cellsDone" /> is bumped
        /// atomically. Runs under the parallel Phase B.
        /// </summary>
        private void EvaluateCellEnginesParallel(
            PerformanceEnvelopeCalculator calc,
            PerformanceEnvelopeCalculator.AeroContext ctx,
            in EditorEngineDeck.CellEnv env,
            EditorEngineDeck.ParallelGroupContext[] groupCtx,
            EditorEngineDeck.ParallelGroupContext compressorlessCtx,
            int slot,
            PerformanceEnvelopeCalculator.VehicleProperties props,
            int i,
            int j,
            int k,
            int mapCount
        )
        {
            for (int t = 0; t < mapCount; t++)
            {
                bool isCombined = _sweepGroups[t] == null;

                PerformanceEnvelopeCalculator.CellResult cell;
                try
                {
                    cell = calc.EvaluateWithAeroParallel(ctx, _deck, env, groupCtx[t], slot, props);

                    // Combined map: where running every engine would melt the turbojets, shed them and
                    // fly on the compressor-less engines alone (dimmed). Same fallback the serial path uses.
                    if (isCombined && cell.Overheats && compressorlessCtx != null)
                    {
                        PerformanceEnvelopeCalculator.CellResult ram =
                            calc.EvaluateWithAeroParallel(ctx, _deck, env, compressorlessCtx, slot, props);
                        ram.ReducedEngines = true;
                        cell = ram;
                    }
                }
                catch (Exception e)
                {
                    System.Threading.Interlocked.Increment(ref _parallelErrorCount);
                    if (!_parallelErrorLogged)
                    {
                        _parallelErrorLogged = true;
                        FARLogger.Exception(e, $"parallel engine pass threw at i{i} j{j} k{k}");
                    }

                    cell = default;
                }

                _results[t].EnvelopeSamples[k][i, j] = cell;
                System.Threading.Interlocked.Increment(ref _cellsDone);
            }
        }

        /// <summary>
        /// Paints any altitude rows whose parallel workers have finished but which the main thread has
        /// not yet drawn, giving the map a progressive row-by-row fill while the sweep runs. Texture
        /// writes are main-thread only, so this is called from the coroutine, never a worker. The
        /// Volatile read pairs with the worker's Volatile write of <paramref name="rowDone" /> so the
        /// cell data for that row is guaranteed visible before it is read here.
        /// </summary>
        private void PaintFinishedRows(bool[] rowValid, bool[] rowDone, bool[] rowPainted, int mapCount)
        {
            for (int j = 0; j < rowDone.Length; j++)
            {
                if (rowPainted[j] || !System.Threading.Volatile.Read(ref rowDone[j]))
                    continue;

                rowPainted[j] = true;
                if (!rowValid[j])
                    continue;

                for (int t = 0; t < mapCount; t++)
                    _envRenderers[t].PaintEnvelopeRow(j);
            }
        }

        /// <summary>
        /// Blacks out grey specks and blobs — cells the trim solved but the engines cannot sustain
        /// (rendered grey), stranded away from the real flyable envelope. These are spurious high-AoA
        /// "trims" the root-find finds past the stall; grey that only borders black or other grey is
        /// not a real flyable island, so it reads as an artifact. A grey cell is kept only if it
        /// touches a "real" cell — one that is feasible or on the thermal wall (light grey), both
        /// genuine boundaries. Decided on a snapshot of the original cells so the removal does not
        /// cascade through a connected grey region in one pass.
        /// </summary>
        private static void CleanIsolatedGreyDots(
            PerformanceEnvelopeCalculator.CellResult[,] grid,
            int nMach,
            int nAlt
        )
        {
            // "Real" = a cell that anchors a grey neighbour as legitimate: it holds level flight
            // (feasible) or melts trying to (overheats). Everything else — black (no trim) or plain
            // grey (thrust-limited) — does not anchor.
            var real = new bool[nMach, nAlt];
            for (int i = 0; i < nMach; i++)
                for (int j = 0; j < nAlt; j++)
                {
                    PerformanceEnvelopeCalculator.CellResult c = grid[i, j];
                    real[i, j] = c.Trimmed && (c.Feasible || c.Overheats);
                }

            for (int i = 0; i < nMach; i++)
                for (int j = 0; j < nAlt; j++)
                {
                    PerformanceEnvelopeCalculator.CellResult c = grid[i, j];
                    if (!c.Trimmed || c.Feasible || c.Overheats)
                        continue;

                    // Keep the grey cell only if some orthogonal neighbour is real; otherwise it is
                    // surrounded by black and/or grey, so hide it. Off-grid neighbours are not real.
                    bool anyRealNeighbour = (i > 0 && real[i - 1, j]) ||
                                            (i < nMach - 1 && real[i + 1, j]) ||
                                            (j > 0 && real[i, j - 1]) ||
                                            (j < nAlt - 1 && real[i, j + 1]);
                    if (anyRealNeighbour)
                        continue;

                    c.Trimmed = false;
                    grid[i, j] = c;
                }
        }

        private void Cancelled()
        {
            _statusMessage = Localizer.Format("FAREnvelopeCancelled");
            _deck.Restore();
            _sweepTask = null;
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

            // Capped: every cell costs at least one aero trim solve, and with stability enabled a full
            // derivative set on top. 100 is a lot of cells (up to 10k per map) but the parallel sweep
            // handles it; the maps are still meant to be hovered, not zoomed.
            if (!int.TryParse(_nMach, out nMach) || nMach < 2 || nMach > 100 ||
                !int.TryParse(_nAlt, out nAlt) || nAlt < 2 || nAlt > 100)
            {
                _statusMessage = "Cell counts must be integers between 2 and 100.";
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

            GameEvents.onEditorShipModified.Remove(OnEditorShipEvent);
            GameEvents.onEditorUndo.Remove(OnEditorShipEvent);
            GameEvents.onEditorRedo.Remove(OnEditorShipEvent);
            GameEvents.onEditorPartEvent.Remove(OnEditorPartEvent);

            if (_sweep != null && EditorGUI.Instance != null)
            {
                // Join any running worker threads first — the snapshot and clone pool released below are
                // exactly what they read, so tearing those down under live workers is a native crash.
                CancelAndJoin();

                EditorGUI.Instance.StopCoroutine(_sweep);
                _sweep = null;

                // Stopping the coroutine mid-sweep skips its finally, so release the geometry snapshot
                // and the engine clone pool here — otherwise the editor tabs would keep reading frozen
                // transforms and the clones would leak.
                _simManager.EnvelopeCalculator?.EndParallelSweep();
                _deck.EndParallel();
            }

            // Textures are owned by the renderers; the results themselves stay alive in
            // HeatmapResult.Cached for the flight scene.
            foreach (HeatmapRenderer r in _envRenderers)
                r.Cleanup();
            _stabRenderer.Cleanup();
        }
    }
}
