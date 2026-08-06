/*
Ferram Aerospace Research
=========================
Flight-scene display of the heatmaps computed in the editor.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using ferram4;
using KSP.Localization;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FARFlightGUI
{
    /// <summary>
    /// Read-only view of the last editor sweep, plus a marker for where the aircraft actually is.
    ///
    /// Display only, by design. Re-running the sweep here would drive ComputeForceEditor on the same
    /// FARWingAerodynamicModel instances the flight path uses every physics frame — they share one
    /// WingScratch, and stall carries hysteresis across calls — so a live sweep would inject stall
    /// state from hypothetical conditions into the wings currently holding the aircraft up.
    ///
    /// The map is therefore frozen at the mass the sweep assumed. That is a real limitation and is
    /// stated in the window rather than hidden: as fuel burns the true ceiling rises above what is
    /// drawn here.
    /// </summary>
    internal class FlightHeatmapGUI
    {
        private const float MapWidth = 420f;
        private const float MapHeight = 170f;
        private const float MultiMapWidth = 250f;

        private readonly List<HeatmapRenderer> _envRenderers = new List<HeatmapRenderer>();
        private readonly HeatmapRenderer _stabRenderer = new HeatmapRenderer();
        private List<HeatmapResult> _boundResults;

        // Per-craft display copies of the cached editor results: when the sweep carries mass samples,
        // each is interpolated to the vessel's live mass here rather than mutating the shared cached
        // grid. Falls back to the cached result itself for a single-mass sweep.
        private List<HeatmapResult> _displayResults;
        private double _lastInterpMassKg = double.NaN;

        private bool _showStability = true;
        private string _logMessage = "";

        private Vessel _diskLoadTriedFor; // vessel the on-disk heatmap cache was last attempted for

        public void Display(Vessel vessel)
        {
            List<HeatmapResult> results = HeatmapResult.Cached;

            // Nothing swept this session, or the in-memory sweep is for a different craft: try the on-disk
            // cache for the vessel being flown, so a heatmap swept in an earlier session still shows. Gate on
            // the vessel first so this whole probe (and its per-part Matches scan) runs once per vessel, not
            // every frame (Display is per-frame).
            if (vessel != null && !ReferenceEquals(_diskLoadTriedFor, vessel))
            {
                _diskLoadTriedFor = vessel;

                if (results == null || results.Count == 0 || !results[0].Matches(vessel))
                {
                    List<HeatmapResult> loaded =
                        HeatmapPersistence.Load(vessel.vesselName, HeatmapResult.CountPhysicalParts(vessel));
                    if (loaded != null && loaded.Count > 0)
                    {
                        HeatmapResult.Cached = loaded;
                        results = loaded;
                    }
                }
            }

            if (results == null || results.Count == 0)
            {
                GUILayout.Label(Localizer.Format("FARFlightHeatmapNone"), FlightGUI.boxStyle);
                return;
            }

            SyncRenderers(results);
            UpdateMassInterpolation(vessel);
            AtmosphereAutopilotInterop.Initialize();

            HeatmapResult first = _displayResults[0];

            // The prediction readout and the log compare against the combined (all-engines) map when
            // there is one — that is what the whole aircraft can actually do, so it matches flight even
            // when a single engine type (e.g. a turbojet past its wall) could not hold the condition.
            HeatmapResult predMap = _displayResults[_displayResults.Count - 1];

            // A name and part-count check is weak, but sweeping one craft and flying another is the
            // easy mistake to make, and a map silently describing a different aircraft is worse than
            // no map at all.
            if (!first.Matches(vessel))
                GUILayout.Label(Localizer.Format("FARFlightHeatmapMismatch", first.CraftName), FlightGUI.boxStyle);

            GUILayout.BeginHorizontal();
            if (first.HasMassSamples)
                GUILayout.Label("Map at current mass " +
                                (first.MassKg / 1000).ToString("F2", CultureInfo.InvariantCulture) +
                                " t (interpolated dry↔wet as fuel burns)");
            else
                GUILayout.Label(Localizer.Format("FARFlightHeatmapMass",
                                                 (first.MassKg / 1000).ToString("F2", CultureInfo.InvariantCulture)));
            GUILayout.FlexibleSpace();
            _showStability = GUILayout.Toggle(_showStability,
                                              Localizer.Format("FARHeatmapsIncludeStab"),
                                              GUILayout.Width(150));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            DrawMetricToggle();
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();

            HeatmapRenderer.DrawZoomControls();

            if (AtmosphereAutopilotInterop.Available)
                GUILayout.Label(Localizer.Format("FARFlightHeatmapClickHint"), FlightGUI.boxStyle);

            GUILayout.BeginHorizontal();
            if (GUILayout.Button(Localizer.Format("FARFlightHeatmapLogBtn"), GUILayout.Height(24)))
                // This runs inside a GUI render: anything escaping tears down the window for the
                // frame. A diagnostic must never be able to break the thing it is diagnosing.
                try
                {
                    _logMessage = LogPredictionCheck(vessel, predMap);
                }
                catch (Exception e)
                {
                    FARLogger.Exception(e, "while logging the heatmap prediction check");
                    _logMessage = "Prediction check failed — see KSP.log.";
                }

            GUILayout.EndHorizontal();

            if (!string.IsNullOrEmpty(_logMessage))
                GUILayout.Label(_logMessage);

            DrawCurrentState(vessel, predMap);

            (int i, int j)? here = CurrentCell(vessel, predMap);
            bool multi = results.Count >= 2;
            float w = multi ? MultiMapWidth : MapWidth;
            const int perRow = 2;

            // Ordered map cells: one per engine-type envelope, then the shared stability map, so it
            // fills the last partial row alongside the combined map instead of dropping full-width
            // beneath. Mirrors the editor tab (see HeatmapsGUI) — the two must not drift.
            var cells = new List<MapCell>();
            foreach (HeatmapRenderer r in _envRenderers)
                cells.Add(new MapCell(r, false));
            bool stabInFlow = _showStability && multi;
            if (stabInFlow)
                cells.Add(new MapCell(_stabRenderer, true));

            for (int start = 0; start < cells.Count; start += perRow)
            {
                int end = Math.Min(start + perRow, cells.Count);
                DrawMapRow(vessel, _displayResults, cells, start, end, w, multi, here);
            }

            if (_showStability && !stabInFlow)
                _stabRenderer.DrawStabilityMap(MapWidth, MapHeight, here);

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

        private void DrawMapRow(
            Vessel vessel,
            List<HeatmapResult> results,
            List<MapCell> cells,
            int start,
            int end,
            float w,
            bool multi,
            (int i, int j)? here
        )
        {
            float col = w + 60;

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
                    GUILayout.Label(cell.Renderer.SummaryText(out bool _, out bool _2));
                }

                GUILayout.EndVertical();
            }

            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            for (int t = start; t < end; t++)
            {
                GUILayout.BeginVertical(GUILayout.Width(col));
                MapCell cell = cells[t];
                if (cell.IsStability)
                {
                    cell.Renderer.DrawStabilityMap(w, MapHeight, here, false);
                    GUILayout.EndVertical();
                }
                else
                {
                    cell.Renderer.DrawEnvelopeMap(w, MapHeight, here, !multi);
                    GUILayout.EndVertical();

                    // Clicking a cell sets it as AA's cruise target.
                    if (cell.Renderer.ClickedEnvelopeCell != null)
                        HandleCellClick(vessel, results[t], cell.Renderer.ClickedEnvelopeCell.Value);
                }
            }

            GUILayout.EndHorizontal();
        }

        /// <summary>Endurance/Range/AoA/Throttle colour toggle, repainting all maps on change. Colour
        /// only. Mirrors the editor tab (HeatmapsGUI.DrawMetricToggle) — the two must not drift.</summary>
        private void DrawMetricToggle()
        {
            GUILayout.Label(Localizer.Format("FARHeatmapsColorBy"), GUILayout.Width(60));

            HeatmapRenderer.Metric want = HeatmapRenderer.ColorMetric;
            want = HeatmapRenderer.MetricToggle(want, HeatmapRenderer.Metric.Endurance, Localizer.Format("FARHeatmapsEndurance"), 90);
            want = HeatmapRenderer.MetricToggle(want, HeatmapRenderer.Metric.Range, Localizer.Format("FARHeatmapsRange"), 80);
            want = HeatmapRenderer.MetricToggle(want, HeatmapRenderer.Metric.Aoa, "AoA", 60);
            want = HeatmapRenderer.MetricToggle(want, HeatmapRenderer.Metric.Throttle, "Throttle", 85);
            want = HeatmapRenderer.MetricToggle(want, HeatmapRenderer.Metric.Climb, "Climb", 60);

            if (want == HeatmapRenderer.ColorMetric)
                return;

            HeatmapRenderer.ColorMetric = want;
            HeatmapRenderer.InvalidateColors(); // each map recolours on its next draw
        }


        /// <summary>Rebuilds the renderer list when a new sweep set is cached.</summary>
        private void SyncRenderers(List<HeatmapResult> results)
        {
            if (ReferenceEquals(results, _boundResults) && _envRenderers.Count == results.Count)
                return;

            foreach (HeatmapRenderer r in _envRenderers)
                r.Cleanup();
            _envRenderers.Clear();

            // Interpolate into per-craft copies so the shared cached (editor) grids are never mutated.
            // A single-mass sweep has nothing to interpolate, so it is displayed directly.
            _displayResults = new List<HeatmapResult>(results.Count);
            foreach (HeatmapResult result in results)
            {
                HeatmapResult disp = result.HasMassSamples ? result.FlightView() : result;

                // The cache is raw now, so derive the shown grid (cleaned per CleanupDisplay). A mass-sampled
                // map is refreshed every frame by the mass interpolation (which cleans); a single-mass map is
                // not, so clean its display once here.
                if (!disp.HasMassSamples)
                    disp.SetDisplaySample(0);

                _displayResults.Add(disp);

                var r = new HeatmapRenderer();
                r.SetResult(disp);
                _envRenderers.Add(r);
            }

            _stabRenderer.SetResult(_displayResults[0]);
            _boundResults = results;
            _lastInterpMassKg = double.NaN; // force a fill on the next frame
        }

        /// <summary>
        /// Interpolates each mass-sampled display map to the vessel's live mass, repainting when the
        /// mass has moved enough to matter. Cheap arithmetic on cached numbers — no sim is re-run.
        /// </summary>
        private void UpdateMassInterpolation(Vessel vessel)
        {
            if (_displayResults == null || vessel == null)
                return;

            double massKg = vessel.totalMass * 1000.0;
            if (!double.IsNaN(_lastInterpMassKg) &&
                Math.Abs(massKg - _lastInterpMassKg) < Math.Max(20.0, 0.001 * massKg))
                return;

            _lastInterpMassKg = massKg;
            for (int t = 0; t < _displayResults.Count; t++)
            {
                if (!_displayResults[t].HasMassSamples)
                    continue;
                _displayResults[t].FillForMass(massKg);
                _envRenderers[t].RepaintEnvelope();
            }

            // The perf map's black/grey cells shift as the interpolated mass changes, so re-mask and repaint
            // the stability map against the combined (or only) map at the new mass.
            HeatmapResult.ApplyStabilityPerfMask(_displayResults);
            _stabRenderer.RepaintStability();
        }

        private void HandleCellClick(Vessel vessel, HeatmapResult result, (int i, int j) cell)
        {
            AtmosphereAutopilotInterop.Initialize();
            if (!AtmosphereAutopilotInterop.Available)
            {
                _logMessage = Localizer.Format("FARFlightHeatmapNoAA");
                return;
            }

            PerformanceEnvelopeCalculator.CellResult c = result.Envelope[cell.i, cell.j];
            if (!c.Feasible)
            {
                _logMessage = Localizer.Format("FARFlightHeatmapCellInfeasible");
                return;
            }

            _logMessage = AtmosphereAutopilotInterop.SetCruiseTarget(vessel, c.SpeedMPerS, result.AltAxis[cell.j]);
        }

        /// <summary>
        /// Where the aircraft actually is, against the cell the map predicted for that condition.
        /// This is the only comparison in the mod between a prediction and reality, so it is worth
        /// showing plainly.
        /// </summary>
        private void DrawCurrentState(Vessel vessel, HeatmapResult result)
        {
            if (vessel == null || result.MachAxis == null || result.AltAxis == null)
                return;

            double mach = vessel.mach;
            double alt = vessel.altitude;
            double xNow = VesselAxisValue(vessel, result);

            GUILayout.Label(Localizer.Format("FARFlightHeatmapNow",
                                             mach.ToString("F2", CultureInfo.InvariantCulture),
                                             (alt / 1000).ToString("F1", CultureInfo.InvariantCulture)));

            int i = NearestIndex(result.MachAxis, xNow);
            int j = NearestIndex(result.AltAxis, alt);
            if (i < 0 || j < 0)
                return;

            // Only meaningful if the aircraft is actually inside the swept box; otherwise the
            // "nearest" cell is an extrapolation dressed up as a reading.
            if (xNow < result.MachAxis[0] ||
                xNow > result.MachAxis[result.MachAxis.Length - 1] ||
                alt < result.AltAxis[0] ||
                alt > result.AltAxis[result.AltAxis.Length - 1])
            {
                GUILayout.Label(Localizer.Format("FARFlightHeatmapOutside"));
                return;
            }

            PerformanceEnvelopeCalculator.CellResult c = result.Envelope[i, j];
            string verdict = !c.Trimmed
                                 ? Localizer.Format("FARFlightHeatmapPredLift")
                                 : c.Feasible
                                     ? Localizer.Format("FARFlightHeatmapPredOk")
                                     : Localizer.Format("FARFlightHeatmapPredThrust");

            GUILayout.Label(Localizer.Format("FARFlightHeatmapPred",
                                             c.Mach.ToString("F2", CultureInfo.InvariantCulture),
                                             (result.AltAxis[j] / 1000).ToString("F1", CultureInfo.InvariantCulture),
                                             verdict));
        }

        /// <summary>
        /// Dumps measured flight state beside the cell the editor predicted for it, so the two can
        /// be compared after the fact. This is the only check in the feature that tests the model
        /// against reality rather than against itself.
        ///
        /// Which comparisons are honest matters, so the log says so rather than leaving it to be
        /// worked out later:
        ///   - Drag is directly comparable. Flight measures -dot(force, velocity) and the sim
        ///     computes Cd*q*S; both are the force opposing the relative wind.
        ///   - Throttle is comparable in steady level flight, where thrust must equal drag.
        ///   - L/D is NOT. Flight resolves lift along vessel.ReferenceTransform.forward
        ///     (PhysicsCalcs), while the sim uses a wind-axis Cl. Different decompositions.
        ///   - Everything is only meaningful near level flight at the swept mass, hence the notes.
        /// </summary>
        /// <summary>
        /// Sums the live flight wings' induced, profile and total drag in kN, from the coefficients
        /// each wing left after the last physics frame. Lets the prediction check compare flight's
        /// drag split against the sweep's to see whether a gap is in induced, profile or the body.
        /// q is kPa, so Cd*S*q is kN.
        /// </summary>
        private static void FlightWingDragSplit(
            Vessel vessel,
            double qKPa,
            out double inducedKN,
            out double profileKN,
            out double totalKN
        )
        {
            inducedKN = 0;
            profileKN = 0;
            totalKN = 0;
            if (vessel?.parts == null)
                return;

            foreach (Part p in vessel.parts)
            {
                FARWingAerodynamicModel w = p.GetComponent<FARWingAerodynamicModel>();
                if (w == null || w.isShielded)
                    continue;

                inducedKN += w.CdInduced * w.S * qKPa;
                profileKN += w.CdProfile * w.S * qKPa;
                // Full wing Cd, so control-surface and stall drag (the sweep's "other") are in the
                // total. Without it, body-implied is inflated wherever a control surface is working.
                totalKN += w.Cd * w.S * qKPa;
            }
        }

        /// <summary>
        /// Live control-surface state for the prediction check: the largest deflection (degrees) and
        /// the control surfaces' share of "other" drag (Cd minus induced and profile, i.e. stall and
        /// deflection drag), in kN. Compares against the sweep's trim to see if it over- or
        /// under-deflects the elevons at high AoA. q is kPa.
        /// </summary>
        private static void FlightControlSurfaceState(Vessel vessel, double qKPa, out double maxDeflectDeg, out double otherKN)
        {
            maxDeflectDeg = 0;
            otherKN = 0;
            if (vessel?.parts == null)
                return;

            foreach (Part p in vessel.parts)
            {
                FARControllableSurface cs = p.GetComponent<FARControllableSurface>();
                if (cs == null || cs.isShielded)
                    continue;

                double d = Math.Abs(cs.ControlDeflection);
                if (d > maxDeflectDeg)
                    maxDeflectDeg = d;

                otherKN += (cs.Cd - cs.CdInduced - cs.CdProfile) * cs.S * qKPa;
            }
        }

        private string LogPredictionCheck(Vessel vessel, HeatmapResult result)
        {
            if (vessel == null)
                return "No vessel.";

            FlightGUI gui = null;
            if (FlightGUI.vesselFlightGUI != null)
                FlightGUI.vesselFlightGUI.TryGetValue(vessel, out gui);

            if (gui == null)
                return "No FAR flight data for this vessel yet.";

            // VesselFlightInfo is a struct — this is a copy taken at the moment of the click, which
            // is what we want: the log should be one coherent instant, not fields sampled as the
            // physics loop moves under us.
            VesselFlightInfo info = gui.InfoParameters;

            double mach = vessel.mach;
            double alt = vessel.altitude;
            double xNow = VesselAxisValue(vessel, result);
            double vertSpeed = vessel.verticalSpeed;
            double massT = vessel.totalMass;

            SumEngines(vessel, out double thrustKN, out double fuelFlowKgS, out double ispEff);

            var sb = new StringBuilder();
            sb.AppendLine("=== FAR heatmap prediction check ===");
            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "Vessel: {0}   UT: {1:F1}\n",
                            vessel.vesselName,
                            Planetarium.GetUniversalTime());

            sb.AppendLine("-- MEASURED --");
            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "  M {0:F3}  TAS {1:F1} m/s  alt {2:F0} m  mass {3:F2} t  vert {4:+0.0;-0.0} m/s\n",
                            mach,
                            vessel.srfSpeed,
                            alt,
                            massT,
                            vertSpeed);
            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "  Thrust {0:F2} kN   Drag {1:F2} kN   Lift {2:F2} kN\n",
                            thrustKN,
                            info.dragForce,
                            info.liftForce);
            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "  AoA {0:F2} deg   L/D {1:F2}   Cl {2:F4}   Cd {3:F4}   refArea {4:F2} m^2\n",
                            info.aoA,
                            info.liftToDragRatio,
                            info.liftCoeff,
                            info.dragCoeff,
                            info.refArea);
            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "  q {0:F3} kPa   throttle {1:F0}%   fuel flow {2:F4} kg/s   Isp_eff {3:F0} s   stall {4:F2}\n",
                            info.dynPres,
                            vessel.ctrlState.mainThrottle * 100,
                            fuelFlowKgS,
                            ispEff,
                            info.stallFraction);

            int i = NearestIndex(result.MachAxis, xNow);
            int j = NearestIndex(result.AltAxis, alt);

            bool outside = i < 0 ||
                           j < 0 ||
                           xNow < result.MachAxis[0] ||
                           xNow > result.MachAxis[result.MachAxis.Length - 1] ||
                           alt < result.AltAxis[0] ||
                           alt > result.AltAxis[result.AltAxis.Length - 1];

            if (outside)
            {
                sb.AppendLine("-- PREDICTED --");
                sb.AppendLine("  Outside the swept range; no comparable cell.");
                FARLogger.Info(sb.ToString());
                return "Logged (outside swept range).";
            }

            PerformanceEnvelopeCalculator.CellResult c = result.Envelope[i, j];

            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "-- PREDICTED (cell M {0:F3}, alt {1:F0} m; swept at {2:F2} t) --\n",
                            c.Mach,
                            result.AltAxis[j],
                            result.MassKg / 1000);

            if (!c.Trimmed)
            {
                sb.AppendLine("  Cell: NOT TRIMMED — predicted no AoA produces enough lift here.");
            }
            else
            {
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "  Thrust avail {0:F2} kN (sustainable, thermally capped)   Drag {1:F2} kN\n",
                                c.ThrustAvailableN / 1000,
                                c.DragN / 1000);
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "  Drag split : wing {0:F2} + body {1:F2} + ram {2:F2} kN  (flight total {3:F2} kN, no split available)\n",
                                c.WingDragN / 1000,
                                c.BodyDragN / 1000,
                                c.RamDragN / 1000,
                                info.dragForce);
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "  Wing drag  : induced {0:F2} + profile {1:F2} + other {2:F2} kN (profile = zero-lift + skin friction; other = control-surface + stall drag)\n",
                                c.WingInducedDragN / 1000,
                                c.WingProfileDragN / 1000,
                                (c.WingDragN - c.WingInducedDragN - c.WingProfileDragN) / 1000);
                FlightWingDragSplit(vessel, info.dynPres, out double flInducedKN, out double flProfileKN, out double flWingKN);
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "  FLIGHT wing: induced {0:F2} + profile {1:F2} + other {2:F2} = {3:F2} kN  -> body implied {4:F2} kN  (sim: wing {5:F2}, body {6:F2}, ram {7:F2})\n",
                                flInducedKN,
                                flProfileKN,
                                flWingKN - flInducedKN - flProfileKN,
                                flWingKN,
                                info.dragForce - flWingKN,
                                c.WingDragN / 1000,
                                c.BodyDragN / 1000,
                                c.RamDragN / 1000);
                FlightControlSurfaceState(vessel, info.dynPres, out double flDeflectDeg, out double flCtrlOtherKN);
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "  Trim       : sim pitch {0:+0.00;-0.00}{1} (Cm {2:F4})  |  flight max elevon {3:F1} deg, ctrl-surf other {4:F2} kN\n",
                                c.CruisePitch,
                                Math.Abs(c.CruisePitch) >= 0.99 ? " SAT" : "",
                                c.CmCruise,
                                flDeflectDeg,
                                flCtrlOtherKN);
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "  Lift split : wing {0:F2} kN + body {1:F2} kN  (flight total {2:F2} kN)\n",
                                c.WingLiftN / 1000,
                                c.BodyLiftN / 1000,
                                info.liftForce);
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "  Cm at cruise: {0:F4}  (cruise solves lift only, not pitch; large => untrimmed, missing trim drag)\n",
                                c.CmCruise);
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "  AoA {0:F2} deg   L/D {1:F2}   TAS {2:F1} m/s\n",
                                c.Alpha,
                                c.LiftToDrag,
                                c.SpeedMPerS);
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "  Feasible {0}   cruise state {1}   overheats {2}\n",
                                c.Feasible,
                                c.Cruise,
                                c.Overheats);
                // Predicted temp is the CAPPED running temp (an engine held at its limiter sits at the
                // limit, not the uncapped ratio it would reach wide open, which is shown in parentheses).
                double cappedRatio = Math.Min(c.ThermalRatio, EditorEngineDeck.ThermalLimit);
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "  Engine temp: predicted {0:F0}% of limit{1}   flight {2:F0}% of limit\n",
                                cappedRatio * 100,
                                c.ThermalRatio > EditorEngineDeck.ThermalLimit
                                    ? string.Format(CultureInfo.InvariantCulture, " ({0:F0}% uncapped)", c.ThermalRatio * 100)
                                    : "",
                                FlightMaxTempRatio(vessel) * 100);
                if (c.Cruise == PerformanceEnvelopeCalculator.CruiseState.Ok)
                    sb.AppendFormat(CultureInfo.InvariantCulture,
                                    "  Throttle req {0:F0}%   fuel flow {1:F4} kg/s   endurance {2:F2} hr\n",
                                    c.ThrottleRequired * 100,
                                    c.FuelFlowKgPerS,
                                    c.EnduranceHours);
            }

            // The inlet accounting is the part of the engine deck reconstructed from
            // SolverFlightSys without being able to run it, so compare the two term by term.
            // Thrust is over-predicted increasingly with dynamic pressure, and one of these four is
            // where that comes from.
            if (SolverEnginesInterop.TryGetFlightInletState(vessel,
                                                            out double fInletArea,
                                                            out double fEngineArea,
                                                            out double fAreaRatio,
                                                            out double fOverallTpr))
            {
                sb.AppendLine("-- INLET (sim vs flight SolverFlightSys) --");
                AppendDelta(sb, "InletArea", c.InletArea, fInletArea);
                AppendDelta(sb, "EngineArea", c.EngineArea, fEngineArea);
                AppendDelta(sb, "AreaRatio", c.AreaRatio, fAreaRatio);
                AppendDelta(sb, "OverallTPR", c.OverallTpr, fOverallTpr);
                sb.AppendLine("  (sim column is at the cell's Mach, flight at the current Mach)");
            }

            // The conditions handed to the solver. The inlet bookkeeping matches exactly, so if
            // thrust still diverges the difference has to be in here — the editor builds these from
            // FARAtmosphere, flight from vessel.staticPressurekPa / vessel.atmosphericTemperature.
            if (SolverEnginesInterop.TryGetFlightThermo(vessel,
                                                        out double fAmbP,
                                                        out double fAmbT,
                                                        out double fInP,
                                                        out double fInT))
            {
                sb.AppendLine("-- THERMO (sim vs flight) --");
                AppendDelta(sb, "Ambient P(Pa)", c.AmbientP, fAmbP);
                AppendDelta(sb, "Ambient T(K)", c.AmbientT, fAmbT);
                AppendDelta(sb, "Inlet P(Pa)", c.InletP, fInP);
                AppendDelta(sb, "Inlet T(K)", c.InletT, fInT);
            }

            if (c.Trimmed)
            {
                sb.AppendLine("-- DELTA (measured vs predicted) --");
                AppendDelta(sb, "Drag  (kN)", info.dragForce, c.DragN / 1000);
                AppendDelta(sb, "AoA   (deg)", info.aoA, c.Alpha);
                if (c.Cruise == PerformanceEnvelopeCalculator.CruiseState.Ok)
                {
                    // Compare the SOLVER throttle, not main-throttle: the sim's ThrottleRequired is the
                    // solver throttle (at 100% limiter), and with a thrust limiter set the flight main
                    // throttle is much higher because AJE squares the limiter before the solver sees it.
                    if (SolverEnginesInterop.TryGetFlightSolverThrottle(vessel, out double solverThrottle))
                    {
                        AppendDelta(sb, "Throttle (%)", solverThrottle * 100, c.ThrottleRequired * 100);
                        sb.AppendFormat(CultureInfo.InvariantCulture,
                                        "  (flight main-throttle {0:F0}%; solver throttle differs when a thrust limiter is set — AJE squares it)\n",
                                        vessel.ctrlState.mainThrottle * 100);
                    }
                    else
                    {
                        AppendDelta(sb, "Throttle (%)", vessel.ctrlState.mainThrottle * 100, c.ThrottleRequired * 100);
                    }

                    AppendDelta(sb, "Fuel (kg/s)", fuelFlowKgS, c.FuelFlowKgPerS);
                }
            }

            sb.AppendLine("-- VALIDITY --");

            double machOff = Math.Abs(xNow - result.MachAxis[i]);
            double altOff = Math.Abs(alt - result.AltAxis[j]);
            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "  Cell offset: dM {0:F3}, dAlt {1:F0} m (grid is coarse; expect some of the delta here)\n",
                            machOff,
                            altOff);

            // Forces are Cd * q * S. If the area the sweep normalised against differs from the one
            // the flight scene uses, every predicted force is wrong by exactly that ratio — and
            // lift and drag would be wrong by the SAME ratio, leaving L/D looking correct.
            if (result.RefAreaM2 > 0 && info.refArea > 0)
            {
                double areaRatio = result.RefAreaM2 / info.refArea;
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "  Ref area: sim {0:F2} m^2 vs flight {1:F2} m^2  (ratio {2:F3}){3}\n",
                                result.RefAreaM2,
                                info.refArea,
                                areaRatio,
                                Math.Abs(areaRatio - 1) > 0.05
                                    ? "  <-- MISMATCH; predicted forces scale by this"
                                    : "");
            }

            double massErr = (massT - result.MassKg / 1000) / (result.MassKg / 1000) * 100;
            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "  Mass: {0:F2} t vs swept {1:F2} t ({2:+0.0;-0.0}%){3}\n",
                            massT,
                            result.MassKg / 1000,
                            massErr,
                            Math.Abs(massErr) > 5 ? "  <-- predicted drag assumes the swept mass" : "");

            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "  Vertical speed {0:+0.0;-0.0} m/s{1}\n",
                            vertSpeed,
                            Math.Abs(vertSpeed) > 2
                                ? "  <-- climbing/descending; prediction assumes L = W"
                                : "  (near level)");

            // Being level is not the same as being in equilibrium: an aircraft can hold altitude
            // while accelerating hard, and then every cruise figure below is answering a different
            // question than the one being asked. Thrust = drag is the assumption the whole
            // prediction rests on, so it gets checked explicitly.
            double netAxialKN = thrustKN - info.dragForce;
            double axialAccel = massT > 0 ? netAxialKN / massT : 0; // kN/t == m/s^2
            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "  Thrust - drag: {0:+0.00;-0.00} kN  ->  {1:+0.00;-0.00} m/s^2 along track{2}\n",
                            netAxialKN,
                            axialAccel,
                            Math.Abs(axialAccel) > 0.5
                                ? "  <-- NOT in equilibrium; throttle/fuel/endurance deltas are meaningless"
                                : "  (near equilibrium)");

            if (Math.Abs(axialAccel) > 0.5)
            {
                sb.AppendLine("        For a valid throttle comparison, trim for hands-off level");
                sb.AppendLine("        cruise and let the speed settle before logging.");
            }

            sb.AppendLine("  NOTE: L/D is not directly comparable — flight resolves lift along");
            sb.AppendLine("        ReferenceTransform.forward, the sim uses a wind-axis Cl.");
            sb.AppendLine("        Drag and throttle ARE comparable.");

            FARLogger.Info(sb.ToString());
            return $"Logged to KSP.log (cell M {c.Mach:F2}, {result.AltAxis[j] / 1000:F1} km).";
        }

        private static void AppendDelta(StringBuilder sb, string label, double actual, double predicted)
        {
            double diff = actual - predicted;
            double pct = Math.Abs(predicted) > 1e-9 ? diff / Math.Abs(predicted) * 100 : double.NaN;

            // The sign belongs in the format section ("+0.000;-0.000"), not the alignment slot —
            // alignment takes a plain integer and throws FormatException on anything else.
            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "  {0,-13}: {1,9:F3}  vs {2,9:F3}   diff {3,9:+0.000;-0.000}  {4}\n",
                            label,
                            actual,
                            predicted,
                            diff,
                            double.IsNaN(pct)
                                ? ""
                                : "(" + pct.ToString("+0.0;-0.0", CultureInfo.InvariantCulture) + "%)");
        }

        /// <summary>
        /// Totals live engine output. Isp comes from the engines' own realIsp rather than being
        /// derived, so it is directly comparable with the solver's number.
        /// </summary>
        /// <summary>
        /// Highest engine-temperature-to-limit ratio across the vessel's running engines, as the flight
        /// scene sees it right now — the actual-vs-predicted counterpart to the sweep's ThermalRatio, so
        /// the prediction check can show whether the sim's overheat flag matches reality.
        /// </summary>
        private static double FlightMaxTempRatio(Vessel vessel)
        {
            SolverEnginesInterop.Initialize();
            double max = 0;
            foreach (Part p in vessel.Parts)
                foreach (PartModule m in p.Modules)
                {
                    if (!(m is ModuleEngines e) || !e.EngineIgnited || e.engineShutdown)
                        continue;
                    double r = SolverEnginesInterop.GetThermalRatio(m);
                    if (r > max)
                        max = r;
                }

            return max;
        }

        private static void SumEngines(Vessel vessel, out double thrustKN, out double fuelFlowKgS, out double ispEff)
        {
            thrustKN = 0;
            fuelFlowKgS = 0;
            ispEff = 0;

            double thrustIspSum = 0;

            foreach (Part p in vessel.Parts)
                foreach (PartModule m in p.Modules)
                {
                    if (!(m is ModuleEngines e) || !e.EngineIgnited || e.engineShutdown)
                        continue;

                    thrustKN += e.finalThrust;
                    thrustIspSum += e.finalThrust * e.realIsp;

                    if (e.realIsp > 0)
                        fuelFlowKgS += e.finalThrust * 1000 / (e.realIsp * 9.80665);
                }

            if (thrustKN > 0 && thrustIspSum > 0)
                ispEff = thrustIspSum / thrustKN;
        }

        /// <summary>
        /// The grid cell nearest the aircraft's current Mach and altitude, for the white highlight.
        /// Returns the nearest cell even when the aircraft is outside the swept box — "closest" is
        /// what the marker means, and the tooltip/summary already say when it is out of range.
        /// </summary>
        private static (int i, int j)? CurrentCell(Vessel vessel, HeatmapResult result)
        {
            if (vessel == null || result.MachAxis == null || result.AltAxis == null)
                return null;

            int i = NearestIndex(result.MachAxis, VesselAxisValue(vessel, result));
            int j = NearestIndex(result.AltAxis, vessel.altitude);
            if (i < 0 || j < 0)
                return null;

            return (i, j);
        }

        /// <summary>
        /// The vessel's position along the map's x-axis in that axis's unit — true airspeed when the
        /// sweep was run in m/s mode, Mach otherwise.
        /// </summary>
        private static double VesselAxisValue(Vessel vessel, HeatmapResult result)
        {
            return result.SpeedAxisMode ? vessel.srfSpeed : vessel.mach;
        }

        private static int NearestIndex(double[] axis, double value)
        {
            if (axis == null || axis.Length == 0)
                return -1;

            int best = 0;
            double bestDist = double.MaxValue;
            for (int i = 0; i < axis.Length; i++)
            {
                double d = System.Math.Abs(axis[i] - value);
                if (d >= bestDist)
                    continue;
                bestDist = d;
                best = i;
            }

            return best;
        }

        public void Cleanup()
        {
            foreach (HeatmapRenderer r in _envRenderers)
                r.Cleanup();
            _stabRenderer.Cleanup();
        }
    }
}
