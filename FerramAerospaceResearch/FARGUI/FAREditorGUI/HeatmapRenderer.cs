/*
Ferram Aerospace Research
=========================
Renders a completed heatmap sweep. Shared by the editor tab and the flight readout.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Globalization;
using System.Text;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using KSP.Localization;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI
{
    /// <summary>
    /// Turns a <see cref="HeatmapResult" /> into two maps with axes and hover tooltips.
    ///
    /// Split out of the editor tab so the flight scene draws the identical thing from the same data,
    /// rather than growing a second copy that can drift out of agreement — which is exactly how the
    /// texture-orientation bug survived: two maps, two copies of the row-mapping logic, both wrong.
    /// </summary>
    internal class HeatmapRenderer
    {
        private const int MaxScore = 11;

        // Stability cells outside the performance envelope keep their hue but are darkened to this
        // fraction, so the reading stays visible yet clearly reads as unflyable.
        private const float OutsideEnvelopeDim = 0.6f;

        /// <summary>Which quantity the colour ramp encodes. Range/Endurance/Climb colour high=best (cool
        /// end); AoA/Throttle colour low=best (cool end), since a lower AoA or throttle is the better cell.</summary>
        public enum Metric
        {
            Endurance,
            Range,
            Aoa,
            Throttle,
            Climb
        }

        /// <summary>
        /// Shared across every map so the toggle applies to all at once. Colouring only — both
        /// numbers are always in the tooltip.
        /// </summary>
        public static Metric ColorMetric = Metric.Endurance;

        /// <summary>Cruise range for a cell, km: endurance (hr) × true airspeed (m/s).</summary>
        public static double RangeKm(PerformanceEnvelopeCalculator.CellResult c)
        {
            return PerformanceEnvelopeCalculator.RangeKm(c);
        }

        /// <summary>One radio-style metric toggle: returns <paramref name="metric" /> if it was just
        /// selected, else the unchanged <paramref name="want" />. Shared by the editor and flight metric
        /// rows so they cannot drift.</summary>
        public static Metric MetricToggle(Metric want, Metric metric, string label, float width)
        {
            bool on = GUILayout.Toggle(ColorMetric == metric, label, GUILayout.Width(width));
            return on && ColorMetric != metric ? metric : want;
        }

        private HeatmapResult _result;

        /// <summary>Engine-type label of the bound result, or empty for a single-type craft.</summary>
        public string EngineTypeLabel
        {
            get { return _result != null ? _result.EngineTypeLabel : ""; }
        }
        private Texture2D _envTex;
        private Texture2D _stabTex;

        private (int i, int j)? _hoverEnv;
        private (int i, int j)? _hoverStab;

        /// <summary>Rebuilds the textures for a sweep. Safe to call with the same result repeatedly.</summary>
        public void SetResult(HeatmapResult result)
        {
            if (ReferenceEquals(result, _result) && _envTex != null)
                return;

            Cleanup();
            _result = result;
            if (result?.MachAxis == null || result.AltAxis == null)
                return;

            int w = result.MachAxis.Length;
            int h = result.AltAxis.Length;

            // A stability-only result (no SolverEngines/AJE) carries no envelope grid; skip its texture
            // entirely — the envelope draw/paint methods all no-op when _envTex is null.
            bool hasEnvelope = result.Envelope != null;
            if (hasEnvelope)
                _envTex = NewTexture(w, h);
            _stabTex = NewTexture(w, h);

            for (int j = 0; j < h; j++)
                for (int i = 0; i < w; i++)
                {
                    if (hasEnvelope)
                        _envTex.SetPixel(i, HeatmapAxes.TexRow(j), EnvelopeColor(i, j));
                    if (result.HasStability)
                        _stabTex.SetPixel(i, HeatmapAxes.TexRow(j), StabilityColor(i, j));
                }

            if (hasEnvelope)
                _envTex.Apply();
            _stabTex.Apply();
        }

        private static Texture2D NewTexture(int w, int h)
        {
            var tex = new Texture2D(w, h, TextureFormat.RGBA32, false)
            {
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp
            };

            var blank = new Color32[w * h];
            for (int i = 0; i < blank.Length; i++)
                blank[i] = new Color32(28, 28, 30, 255);
            tex.SetPixels32(blank);
            tex.Apply();
            return tex;
        }

        // Porkchop-plot ramp for the endurance/range value: red (lowest) → orange → yellow → green →
        // blue → indigo (highest). Best performance is the cool end, worst is the warm end. The top
        // stop is a bright indigo rather than a dark violet, which read too close to black.
        private static readonly Color[] PorkchopStops =
        {
            new Color(0.85f, 0.10f, 0.10f), // red
            new Color(0.95f, 0.55f, 0.10f), // orange
            new Color(0.95f, 0.90f, 0.15f), // yellow
            new Color(0.20f, 0.80f, 0.25f), // green
            new Color(0.15f, 0.45f, 0.90f), // blue
            new Color(0.48f, 0.35f, 0.95f)  // indigo
        };

        private static Color Porkchop(float t)
        {
            t = Mathf.Clamp01(t);
            float scaled = t * (PorkchopStops.Length - 1);
            int lo = Mathf.Min((int)scaled, PorkchopStops.Length - 2);
            return Color.Lerp(PorkchopStops[lo], PorkchopStops[lo + 1], scaled - lo);
        }

        public Color EnvelopeColor(int i, int j)
        {
            PerformanceEnvelopeCalculator.CellResult c = _result.Envelope[i, j];

            if (!c.Trimmed || c.Suppressed)
                return Color.black; // no trim here, or hidden by cleanup (isolated dot / disconnected)
            if (!c.Feasible)
                // A heat wall (an engine at its temperature limit, and even the thermally-capped thrust
                // cannot hold this speed) reads off-white; a plain thrust shortfall reads grey.
                return c.Overheats
                           ? new Color(0.867f, 0.867f, 0.867f)
                           : new Color(0.5f, 0.5f, 0.5f);

            // Feasible but with no cruise-endurance figure by nature — nothing to burn, or idle thrust
            // exceeds drag: a distinct teal rather than a point on the porkchop ramp. The climb map is the
            // exception — these cells have a real climb rate (an idle-exceeds-drag cell climbs hard), so
            // let them fall through and colour by it.
            if (ColorMetric != Metric.Climb &&
                (c.Cruise == PerformanceEnvelopeCalculator.CruiseState.NoBurnableFuel ||
                 c.Cruise == PerformanceEnvelopeCalculator.CruiseState.IdleExceedsDrag))
                return new Color(0.35f, 0.6f, 0.65f);

            double value, max;
            bool lowerIsBetter;
            switch (ColorMetric)
            {
                case Metric.Range:
                    value = RangeKm(c); max = _result.MaxRangeKm; lowerIsBetter = false;
                    break;
                case Metric.Aoa:
                    value = c.Alpha; max = _result.MaxAlphaDeg; lowerIsBetter = true;
                    break;
                case Metric.Throttle:
                    value = c.ThrottleRequired; max = _result.MaxThrottle; lowerIsBetter = true;
                    break;
                case Metric.Climb:
                    value = c.ClimbRateMPerS; max = _result.MaxClimbRateMPerS; lowerIsBetter = false;
                    break;
                default:
                    value = c.EnduranceHours; max = _result.MaxEnduranceHours; lowerIsBetter = false;
                    break;
            }

            // Normalise to 0..1 of the map's own best cell. A cell whose value is unresolved (e.g. a
            // near-empty cruise with Breguet endurance → 0) sits at the low-performance end rather than
            // blanking to teal. For AoA/throttle low is best, so invert onto the cool end of the ramp.
            float n = !FARMathUtil.IsFinite(value) || max <= 0
                          ? (lowerIsBetter ? 1f : 0f)
                          : Mathf.Clamp01((float)(value / max));
            Color feasible = Porkchop(lowerIsBetter ? 1f - n : n);

            // Combined-map cells flown on ramjet power alone (turbojets shed to avoid melting) are
            // dimmed so the handoff region reads as distinct from full-combined-power cells.
            if (c.ReducedEngines)
                feasible = new Color(feasible.r * 0.775f, feasible.g * 0.775f, feasible.b * 0.775f);

            // Thermally limited: flyable, but an engine is held at a reduced thrust limiter here to stay
            // under temperature. Warm-tint the porkchop colour so the heat-limited band reads distinctly.
            if (c.Overheats)
                feasible = new Color(Mathf.Lerp(feasible.r, 1f, 0.25f), feasible.g * 0.8f, feasible.b * 0.7f);

            return feasible;
        }

        private Color StabilityColor(int i, int j)
        {
            HeatmapResult.StabilityCell c = _result.Stability?[i, j];
            if (c == null || c.Score < 0)
                return new Color(0.5f, 0.2f, 0.6f); // errored — distinct from any score colour

            // Same porkchop ramp as the performance maps, fewer-out-of-range-is-better: a clean cell (score 0)
            // reads cool (indigo), the worst (most derivatives out of range) reads warm (red).
            Color color = Porkchop(1f - Mathf.Clamp01((float)c.Score / MaxScore));

            // Outside the level-flight envelope (the perf map blacks or greys it) the craft can still fly
            // here transiently — just not hold level flight — so keep the stability hue but darken it rather
            // than hide the reading.
            if (c.PerfMask != 0)
                color = new Color(color.r * OutsideEnvelopeDim, color.g * OutsideEnvelopeDim, color.b * OutsideEnvelopeDim);

            return color;
        }

        /// <summary>Repaints a single envelope row in place, for a sweep still in progress.</summary>
        public void PaintEnvelopeRow(int j)
        {
            if (_envTex == null)
                return;
            for (int i = 0; i < _result.MachAxis.Length; i++)
                _envTex.SetPixel(i, HeatmapAxes.TexRow(j), EnvelopeColor(i, j));
            _envTex.Apply();
        }

        public void PaintStabilityCell(int i, int j)
        {
            if (_stabTex == null)
                return;
            _stabTex.SetPixel(i, HeatmapAxes.TexRow(j), StabilityColor(i, j));
        }

        /// <summary>Repaints the whole stability texture, e.g. after the perf mask is recomputed.</summary>
        public void RepaintStability()
        {
            if (_stabTex == null)
                return;
            for (int j = 0; j < _result.AltAxis.Length; j++)
                for (int i = 0; i < _result.MachAxis.Length; i++)
                    _stabTex.SetPixel(i, HeatmapAxes.TexRow(j), StabilityColor(i, j));
            _stabTex.Apply();
        }

        public void ApplyStability()
        {
            if (_stabTex != null)
                _stabTex.Apply();
        }

        /// <summary>Full repaint — needed once the sweep ends, since endurance colours scale to its best cell.</summary>
        public void RepaintEnvelope()
        {
            if (_envTex == null)
                return;
            for (int j = 0; j < _result.AltAxis.Length; j++)
                for (int i = 0; i < _result.MachAxis.Length; i++)
                    _envTex.SetPixel(i, HeatmapAxes.TexRow(j), EnvelopeColor(i, j));
            _envTex.Apply();
        }

        /// <summary>Envelope cell clicked this frame, consumed by the caller. Null if none.</summary>
        public (int i, int j)? ClickedEnvelopeCell { get; private set; }

        /// <summary>Cell the user middle-clicked this frame — a hidden diagnostic dump gesture.</summary>
        public (int i, int j)? DiagClickedEnvelopeCell { get; private set; }

        /// <summary>
        /// Draws the maps. <paramref name="highlight" /> marks one cell with a white border on both
        /// maps — used in flight to show the aircraft's current (Mach, altitude) against the grid.
        /// </summary>
        /// <summary>
        /// Draws just the envelope map. <paramref name="showLabel" /> is false when the caller has
        /// already headed the column (e.g. side-by-side maps whose titles sit in a separate text
        /// row above), so the maps themselves stay the same height and line up.
        /// </summary>
        public void DrawEnvelopeMap(float mapWidth, float mapHeight, (int i, int j)? highlight = null, bool showLabel = true)
        {
            _hoverEnv = null;
            ClickedEnvelopeCell = null;
            DiagClickedEnvelopeCell = null;
            if (_envTex == null)
                return;

            if (showLabel)
                GUILayout.Label(!string.IsNullOrEmpty(_result.EngineTypeLabel)
                                    ? _result.EngineTypeLabel
                                    : Localizer.Format("FARHeatmapsEnvLabel"));

            Rect map = HeatmapAxes.ReserveRect(_envTex, mapWidth, mapHeight);
            GUI.DrawTexture(map, _envTex, ScaleMode.StretchToFill, false);
            HeatmapAxes.Draw(map, _result.MachAxis, _result.AltAxis, _result.SpeedAxisMode);
            if (highlight != null)
                HeatmapAxes.DrawCellBorder(map, _result.MachAxis, _result.AltAxis, highlight.Value, Color.white);

            _hoverEnv = HeatmapAxes.HitTest(map, _result.MachAxis, _result.AltAxis);

            if (_hoverEnv != null)
            {
                if (HeatmapAxes.ClickedIn(map))
                    ClickedEnvelopeCell = _hoverEnv;
                else if (HeatmapAxes.MiddleClickedIn(map))
                    DiagClickedEnvelopeCell = _hoverEnv;
            }
        }

        /// <summary>
        /// Draws just the stability map, which is engine-independent and shown once.
        /// <paramref name="showLabel" /> is false when the caller has already headed the column in a
        /// separate text row above (as with side-by-side envelope maps), so the map itself stays the
        /// same height as its neighbours and lines up.
        /// </summary>
        public void DrawStabilityMap(float mapWidth, float mapHeight, (int i, int j)? highlight = null, bool showLabel = true)
        {
            _hoverStab = null;
            if (!_result.HasStability || _stabTex == null)
                return;

            if (showLabel)
                GUILayout.Label(Localizer.Format("FARHeatmapsStabLabel"));
            Rect smap = HeatmapAxes.ReserveRect(_stabTex, mapWidth, mapHeight);
            GUI.DrawTexture(smap, _stabTex, ScaleMode.StretchToFill, false);
            HeatmapAxes.Draw(smap, _result.MachAxis, _result.AltAxis, _result.SpeedAxisMode);
            if (highlight != null)
                HeatmapAxes.DrawCellBorder(smap, _result.MachAxis, _result.AltAxis, highlight.Value, Color.white);
            _hoverStab = HeatmapAxes.HitTest(smap, _result.MachAxis, _result.AltAxis);
        }

        /// <summary>Call last, so tooltips draw over everything rather than behind it.</summary>
        public void DrawTooltips()
        {
            if (_hoverEnv != null)
            {
                HeatmapAxes.DrawTooltip(EnvelopeTooltip(_hoverEnv.Value.i, _hoverEnv.Value.j));
                return;
            }

            if (_hoverStab != null)
                HeatmapAxes.DrawTooltip(StabilityTooltip(_hoverStab.Value.i, _hoverStab.Value.j));
        }

        public string EnvelopeTooltip(int i, int j)
        {
            PerformanceEnvelopeCalculator.CellResult c = _result.Envelope[i, j];
            var sb = new StringBuilder();

            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "M {0:F2}  ·  {1:F0} m/s  ·  {2:F1} km",
                            c.Mach,
                            c.SpeedMPerS,
                            _result.AltAxis[j] / 1000.0);

            if (c.ReducedEngines)
            {
                sb.Append("\n\nTurbojets shut off here — they would");
                sb.Append("\noverheat. Flying on ramjet power alone;");
                sb.Append("\nfigures below are ramjet-only.");
            }

            if (!c.Trimmed)
            {
                sb.Append("\n\nCannot hold level flight here —");
                sb.Append("\nno angle of attack produces enough lift.");
            }
            else
            {
                if (c.Suppressed)
                    sb.Append("\n\nHidden — isolated from the main envelope;\nshown for reference only.");

                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "\n\nThrust avail : {0:F1} kN (sustainable)",
                                c.ThrustAvailableN / 1000);
                sb.AppendFormat(CultureInfo.InvariantCulture, "\nDrag         : {0:F1} kN", c.DragN / 1000);
                sb.AppendFormat(CultureInfo.InvariantCulture, "\nAoA          : {0:F1}°", c.Alpha);
                sb.AppendFormat(CultureInfo.InvariantCulture, "\nL/D          : {0:F2}", c.LiftToDrag);
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "\nClimb rate   : {0:+0;-0} m/s (max, at speed)",
                                c.ClimbRateMPerS);

                if (c.ThermalRatio > 0)
                    sb.AppendFormat(CultureInfo.InvariantCulture,
                                    "\nEngine temp  : {0:F0}% of limit at cruise{1}",
                                    Math.Min(c.ThermalRatio, EditorEngineDeck.ThermalLimit) * 100,
                                    c.ThermalRatio > EditorEngineDeck.ThermalLimit
                                        ? string.Format(CultureInfo.InvariantCulture, " (capped from {0:F0}%)", c.ThermalRatio * 100)
                                        : "");

                // Overheats now means an engine is held at a reduced thrust limiter to stay under
                // temperature: flyable-but-heat-limited when feasible, a genuine heat wall when not.
                if (c.Overheats)
                    sb.Append(c.Feasible
                                  ? "\nHeat-limited : an engine runs a reduced thrust\nlimiter here to stay under temperature."
                                  : "\n\nHeat wall: even at the thermal limit the\nengines cannot hold this speed.");

                if (c.Feasible)
                    switch (c.Cruise)
                    {
                        case PerformanceEnvelopeCalculator.CruiseState.Ok:
                            sb.AppendFormat(CultureInfo.InvariantCulture,
                                            "\nThrottle req : {0:F0}%",
                                            c.ThrottleRequired * 100);
                            sb.AppendFormat(CultureInfo.InvariantCulture,
                                            "\nFuel flow    : {0:F3} kg/s",
                                            c.FuelFlowKgPerS);
                            sb.AppendFormat(CultureInfo.InvariantCulture,
                                            "\nEndurance    : {0:F2} hr",
                                            c.EnduranceHours);
                            sb.AppendFormat(CultureInfo.InvariantCulture,
                                            "\nRange        : {0:F0} km",
                                            RangeKm(c));
                            break;
                        case PerformanceEnvelopeCalculator.CruiseState.IdleExceedsDrag:
                            sb.Append("\n\nNo steady cruise at this speed —");
                            sb.Append("\nidle thrust still exceeds drag, so the");
                            sb.Append("\naircraft would accelerate out of it.");
                            break;
                        case PerformanceEnvelopeCalculator.CruiseState.NoBurnableFuel:
                            sb.Append("\n\nNo endurance: nothing aboard that");
                            sb.Append("\nthese engines can burn.");
                            break;
                        default:
                            sb.Append("\n\nCruise throttle did not converge.");
                            break;
                    }
                else if (!c.Overheats)
                {
                    sb.AppendFormat(CultureInfo.InvariantCulture,
                                    "\n\nDrag exceeds thrust by {0:F1} kN —",
                                    -c.ExcessThrustN / 1000);
                    sb.Append("\nlevel flight not sustainable here.");
                }
            }

            AppendRowEnvelope(sb, j);
            return sb.ToString();
        }

        private string StabilityTooltip(int i, int j)
        {
            HeatmapResult.StabilityCell c = _result.Stability?[i, j];
            var sb = new StringBuilder();

            if (_result.SpeedAxisMode)
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "{0:F0} m/s  ·  {1:F1} km",
                                _result.MachAxis[i],
                                _result.AltAxis[j] / 1000.0);
            else
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "M {0:F2}  ·  {1:F1} km",
                                _result.MachAxis[i],
                                _result.AltAxis[j] / 1000.0);

            if (c == null)
            {
                sb.Append("\n\nNot evaluated.");
                return sb.ToString();
            }

            if (c.Score < 0)
            {
                sb.Append("\n\nErrored — see KSP.log.");
                return sb.ToString();
            }

            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "\nScore: {0}/{1} derivatives out of range",
                            c.Score,
                            MaxScore);

            if (c.OutOfRange.Count == 0)
            {
                sb.Append("\n\nAll derivatives within expected sign.");
                return sb.ToString();
            }

            sb.Append("\n\nOut of range:");
            foreach ((string name, double value, int expectedSign) in c.OutOfRange)
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "\n  {0} = {1} ({2})",
                                name,
                                double.IsNaN(value) ? "NaN" : value.ToString("G3", CultureInfo.InvariantCulture),
                                expectedSign < 0 ? "exp <0" : "exp >0");

            return sb.ToString();
        }

        public struct EnvelopeEdge
        {
            public bool Valid;
            public double SpeedMPerS;
            public double Mach;
            public bool Clipped;
            public bool LiftLimited;
            public bool TrimFailedBeyond;
        }

        /// <summary>
        /// Interpolates the thrust = drag crossing between a feasible cell and the infeasible one
        /// beyond it. Excess thrust is signed and changes sign across the boundary, so a linear
        /// crossing lands far closer than the last feasible cell's speed, which would quantise the
        /// answer to the grid spacing — tens of m/s wide at useful Mach ranges.
        ///
        /// Only valid against a thrust-limited neighbour. A lift-limited one has no thrust crossing:
        /// the aircraft is out of lift, not out of power.
        /// </summary>
        private bool TryInterpolateCrossing(int iFeasible, int iOther, int j, out double speed, out double mach)
        {
            speed = mach = 0;

            PerformanceEnvelopeCalculator.CellResult a = _result.Envelope[iFeasible, j];
            PerformanceEnvelopeCalculator.CellResult b = _result.Envelope[iOther, j];

            if (!b.Trimmed ||
                b.Suppressed ||
                !FARMathUtil.IsFinite(a.ExcessThrustN) ||
                !FARMathUtil.IsFinite(b.ExcessThrustN) ||
                a.ExcessThrustN <= 0 ||
                b.ExcessThrustN >= 0)
                return false;

            double t = a.ExcessThrustN / (a.ExcessThrustN - b.ExcessThrustN);
            speed = a.SpeedMPerS + t * (b.SpeedMPerS - a.SpeedMPerS);
            mach = a.Mach + t * (b.Mach - a.Mach);
            return true;
        }

        public EnvelopeEdge TopSpeedAtRow(int j)
        {
            var edge = new EnvelopeEdge();

            int last = -1;
            for (int i = 0; i < _result.MachAxis.Length; i++)
                if (_result.Envelope[i, j].Feasible)
                    last = i;

            if (last < 0)
                return edge;

            edge.Valid = true;
            edge.SpeedMPerS = _result.Envelope[last, j].SpeedMPerS;
            edge.Mach = _result.Envelope[last, j].Mach;

            if (last == _result.MachAxis.Length - 1)
            {
                edge.Clipped = true;
                return edge;
            }

            if (TryInterpolateCrossing(last, last + 1, j, out double speed, out double mach))
            {
                edge.SpeedMPerS = speed;
                edge.Mach = mach;
            }
            else if (!_result.Envelope[last + 1, j].Trimmed || _result.Envelope[last + 1, j].Suppressed)
            {
                // Anomalous, not physical. Needed Cl scales as 1/V^2, so trimming only gets easier
                // with speed — the fast edge of a level-flight envelope is always thrust-limited. A
                // failure to trim just beyond it means the AoA solve gave up where a solution
                // exists, so report the solver problem rather than dress it up as an aero limit.
                edge.TrimFailedBeyond = true;
            }

            return edge;
        }

        public EnvelopeEdge MinLevelSpeedAtRow(int j)
        {
            var edge = new EnvelopeEdge();

            int first = -1;
            for (int i = 0; i < _result.MachAxis.Length; i++)
                if (_result.Envelope[i, j].Feasible)
                {
                    first = i;
                    break;
                }

            if (first < 0)
                return edge;

            edge.Valid = true;
            edge.SpeedMPerS = _result.Envelope[first, j].SpeedMPerS;
            edge.Mach = _result.Envelope[first, j].Mach;

            if (first == 0)
            {
                edge.Clipped = true;
                return edge;
            }

            if (TryInterpolateCrossing(first, first - 1, j, out double speed, out double mach))
            {
                edge.SpeedMPerS = speed;
                edge.Mach = mach;
            }
            else if (!_result.Envelope[first - 1, j].Trimmed || _result.Envelope[first - 1, j].Suppressed)
            {
                // The usual slow edge: the wing runs out of lift before the engine runs out of
                // thrust. This is the stall boundary.
                edge.LiftLimited = true;
            }

            return edge;
        }

        private void AppendRowEnvelope(StringBuilder sb, int j)
        {
            EnvelopeEdge top = TopSpeedAtRow(j);
            EnvelopeEdge min = MinLevelSpeedAtRow(j);

            sb.AppendFormat(CultureInfo.InvariantCulture, "\n\n─ At {0:F1} km ─", _result.AltAxis[j] / 1000.0);

            if (!top.Valid)
            {
                sb.Append("\nLevel flight not possible at any");
                sb.Append("\nspeed in the swept range.");
                return;
            }

            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "\nTop speed    : {0:F0} m/s (M {1:F2}){2}",
                            top.SpeedMPerS,
                            top.Mach,
                            top.Clipped ? " +" : top.TrimFailedBeyond ? " (!)" : "");

            sb.AppendFormat(CultureInfo.InvariantCulture,
                            "\nMin level spd: {0:F0} m/s (M {1:F2}){2}",
                            min.SpeedMPerS,
                            min.Mach,
                            min.Clipped ? " −" : min.LiftLimited ? " (stall)" : "");

            if (top.Clipped)
                sb.Append("\n  + still flying at the fastest Mach swept");
            if (min.Clipped)
                sb.Append("\n  − still flying at the slowest Mach swept");
            if (top.TrimFailedBeyond)
                sb.Append("\n  ! trim solve failed past this point —\n    top speed is a lower bound (see KSP.log)");
        }

        /// <summary>Ceiling / top speed / best endurance line shared by both scenes.</summary>
        public string SummaryText(out bool ceilingClipped, out bool topClipped)
        {
            ceilingClipped = false;
            topClipped = false;

            string ceiling = double.IsNaN(_result.CeilingM)
                                 ? Localizer.Format("FAREnvelopeNone")
                                 : (_result.CeilingM / 1000).ToString("F1", CultureInfo.InvariantCulture) + " km";

            string endurance = double.IsNaN(_result.BestEnduranceHours)
                                   ? Localizer.Format("FAREnvelopeNone")
                                   : _result.BestEnduranceHours.ToString("F2", CultureInfo.InvariantCulture) + " hr";

            double topSpeed = double.NaN;
            for (int j = 0; j < _result.AltAxis.Length; j++)
            {
                EnvelopeEdge e = TopSpeedAtRow(j);
                if (!e.Valid)
                    continue;
                if (double.IsNaN(topSpeed) || e.SpeedMPerS > topSpeed)
                {
                    topSpeed = e.SpeedMPerS;
                    topClipped = e.Clipped;
                }
            }

            string top = double.IsNaN(topSpeed)
                             ? Localizer.Format("FAREnvelopeNone")
                             : topSpeed.ToString("F0", CultureInfo.InvariantCulture) + " m/s";

            ceilingClipped = !double.IsNaN(_result.CeilingM) &&
                             _result.CeilingM >= _result.AltAxis[_result.AltAxis.Length - 1];

            string range = double.IsNaN(_result.BestRangeKm)
                               ? Localizer.Format("FAREnvelopeNone")
                               : _result.BestRangeKm.ToString("F0", CultureInfo.InvariantCulture) + " km";

            return Localizer.Format("FAREnvelopeSummary", ceiling, top, endurance, range);
        }

        public void Cleanup()
        {
            if (_envTex != null)
            {
                UnityEngine.Object.Destroy(_envTex);
                _envTex = null;
            }

            if (_stabTex == null)
                return;

            UnityEngine.Object.Destroy(_stabTex);
            _stabTex = null;
        }
    }
}
