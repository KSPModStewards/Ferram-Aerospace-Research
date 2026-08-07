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
using System.Collections.Generic;
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

        // Stability-map caption: title on its own line, then the colour key. Built as a literal (not the
        // one-line FARHeatmapsStabLabel loc template) so the newline and the corrected blue/red wording
        // survive a DLL-only deploy. Shared by the editor and flight tabs so the two cannot drift.
        public const string StabilityCaption =
            "Stability derivatives\nBlue = no derivatives unstable, Red = all unstable" +
            "\nDimmed = outside the level-flight envelope";

        // Per-derivative explanations shown in the stability tooltip's out-of-range list — the same
        // FAREditor*Exp strings the Stability tab uses (present in FAR's base Localization.cfg, so they
        // resolve without deploying the cfg). Keyed by the short name CheckedDerivs stores on each cell.
        private static readonly Dictionary<string, string> StabDerivExpKeys = new Dictionary<string, string>
        {
            { "Zw", "FAREditorZwExp" },
            { "Zu", "FAREditorZuExp" },
            { "Mw", "FAREditorMwExp" },
            { "Mq", "FAREditorMqExp" },
            { "MΔe", "FAREditorMDeltaeExp" },
            { "Yβ", "FAREditorYβExp" },
            { "Lβ", "FAREditorLβExp" },
            { "Lp", "FAREditorLpExp" },
            { "Lr", "FAREditorLrExp" },
            { "Nβ", "FAREditorNβExp" },
            { "Nr", "FAREditorNrExp" }
        };

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

        // --- Zoom -------------------------------------------------------------------------------------
        // A visible cell window shared across every map (all share the same axes, so they zoom together)
        // and both scenes, like ColorMetric. Held in cell-index space and reset whenever the grid it was
        // set against changes size. Any change bumps _colorEpoch; each renderer recolours + repaints its
        // own texture the next frame it draws, so a zoom on one map recolours them all without the
        // renderers needing to know about each other.

        internal struct ZoomWindow
        {
            public int IMin, IMax, JMin, JMax;
        }

        /// <summary>When on, a zoomed view re-spreads the colour ramp across the visible cells' own
        /// min..max (full contrast within the window) instead of the whole-map scale.</summary>
        public static bool RescaleColorsToView = true;

        private static ZoomWindow _zoom;
        private static bool _zoomActive;
        private static int _zoomMach = -1, _zoomAlt = -1; // grid dims _zoom is valid for
        private static int _colorEpoch;                   // bumped on any zoom / metric / rescale change

        public static bool ZoomActive => _zoomActive;

        /// <summary>Bumps the colour epoch so every renderer recolours on its next draw. Call after a
        /// metric or rescale-toggle change.</summary>
        public static void InvalidateColors()
        {
            _colorEpoch++;
        }

        public static void ResetZoom()
        {
            if (!_zoomActive)
                return;
            _zoomActive = false;
            InvalidateColors();
        }

        /// <summary>The zoom controls, shared by the editor and flight tabs so they cannot drift: the
        /// recolour-to-view toggle, a Reset button (live only while zoomed), and a drag hint. With
        /// <paramref name="ownRow" /> false the caller supplies the horizontal group (and the trailing
        /// flexible space), so more can share the row — the editor puts the fuel slider to the right.</summary>
        public static void DrawZoomControls(bool ownRow = true)
        {
            if (ownRow)
                GUILayout.BeginHorizontal();
            GUILayout.Label("Zoom:", GUILayout.Width(50));

            bool prev = RescaleColorsToView;
            RescaleColorsToView = GUILayout.Toggle(prev, "Recolor to view", GUILayout.Width(150));
            if (RescaleColorsToView != prev)
                InvalidateColors();

            bool prevEnabled = GUI.enabled;
            GUI.enabled = prevEnabled && _zoomActive;
            if (GUILayout.Button("Reset", GUILayout.Width(55)))
                ResetZoom();
            GUI.enabled = prevEnabled;

            GUILayout.Label(_zoomActive ? "" : "drag a box to zoom", GUILayout.Width(140));

            if (ownRow)
            {
                GUILayout.FlexibleSpace();
                GUILayout.EndHorizontal();
            }
        }

        // Drag-to-zoom state. The owner is the renderer the drag started on; only it draws the rubber-band
        // and applies the window on release, so a drag that runs across a neighbouring map is unambiguous.
        private static HeatmapRenderer _dragOwner;
        private static Vector2 _dragStartMouse;
        private static Vector2 _dragCurMouse;
        private const float DragThresholdPx = 4f; // below this it is a click, not a box

        private void VisibleBounds(out int i0, out int i1, out int j0, out int j1)
        {
            int nM = _result.MachAxis.Length, nA = _result.AltAxis.Length;
            if (_zoomActive && _zoomMach == nM && _zoomAlt == nA)
            {
                i0 = Mathf.Clamp(_zoom.IMin, 0, nM - 1);
                i1 = Mathf.Clamp(_zoom.IMax, i0, nM - 1);
                j0 = Mathf.Clamp(_zoom.JMin, 0, nA - 1);
                j1 = Mathf.Clamp(_zoom.JMax, j0, nA - 1);
                return;
            }
            i0 = 0; i1 = nM - 1; j0 = 0; j1 = nA - 1;
        }

        private static void SetZoom(int i0, int i1, int j0, int j1, int nM, int nA)
        {
            _zoom = new ZoomWindow { IMin = i0, IMax = i1, JMin = j0, JMax = j1 };
            _zoomMach = nM;
            _zoomAlt = nA;
            _zoomActive = !(i0 <= 0 && j0 <= 0 && i1 >= nM - 1 && j1 >= nA - 1); // a full box is just "no zoom"
            InvalidateColors();
        }

        /// <summary>
        /// Recentres the shared zoom window on cell (ci, cj), keeping the current window's span (or a
        /// default quarter-grid span when not yet zoomed) and shifting it to stay in bounds. A no-op when
        /// the window is already positioned there, so the flight tab can call it every frame to follow the
        /// aircraft without re-invalidating every map's colours each frame.
        /// </summary>
        public static void CenterZoomOn(int ci, int cj, int nM, int nA)
        {
            if (nM <= 0 || nA <= 0)
                return;

            int halfI, halfJ;
            if (_zoomActive && _zoomMach == nM && _zoomAlt == nA)
            {
                halfI = (_zoom.IMax - _zoom.IMin) / 2;
                halfJ = (_zoom.JMax - _zoom.JMin) / 2;
            }
            else
            {
                halfI = Math.Max(1, nM / 4);
                halfJ = Math.Max(1, nA / 4);
            }

            int spanI = 2 * halfI;
            int spanJ = 2 * halfJ;
            int i0 = Mathf.Clamp(ci - halfI, 0, Math.Max(0, nM - 1 - spanI));
            int i1 = Math.Min(i0 + spanI, nM - 1);
            int j0 = Mathf.Clamp(cj - halfJ, 0, Math.Max(0, nA - 1 - spanJ));
            int j1 = Math.Min(j0 + spanJ, nA - 1);

            if (_zoomActive && _zoomMach == nM && _zoomAlt == nA &&
                _zoom.IMin == i0 && _zoom.IMax == i1 && _zoom.JMin == j0 && _zoom.JMax == j1)
                return; // already centred here — don't re-invalidate colours

            SetZoom(i0, i1, j0, j1, nM, nA);
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

            ResetZoom(); // a freshly bound sweep starts un-zoomed

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
            _paintedEpoch = _colorEpoch; // freshly painted at the current epoch
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

        // The stability ramp drops the indigo top stop: on that map it read too close to the blue below
        // it to tell a clean cell from a nearly-clean one, so clean tops out at blue.
        private static readonly Color[] StabilityStops =
        {
            new Color(0.85f, 0.10f, 0.10f), // red
            new Color(0.95f, 0.55f, 0.10f), // orange
            new Color(0.95f, 0.90f, 0.15f), // yellow
            new Color(0.20f, 0.80f, 0.25f), // green
            new Color(0.15f, 0.45f, 0.90f)  // blue
        };

        private static Color Ramp(Color[] stops, float t)
        {
            t = Mathf.Clamp01(t);
            float scaled = t * (stops.Length - 1);
            int lo = Mathf.Min((int)scaled, stops.Length - 2);
            return Color.Lerp(stops[lo], stops[lo + 1], scaled - lo);
        }

        private static Color Porkchop(float t)
        {
            return Ramp(PorkchopStops, t);
        }

        /// <summary>True if a cell is coloured on the porkchop ramp (a feasible cruise with a metric
        /// value), rather than a fixed swatch (black/grey/heat-wall/teal). Shared by the colour path and
        /// the view-range scan so the two cannot disagree about which cells count.</summary>
        private bool IsRampColored(in PerformanceEnvelopeCalculator.CellResult c)
        {
            if (!c.Trimmed || c.Suppressed || !c.Feasible)
                return false;
            // No-cruise-endurance cells (nothing to burn, or idle exceeds drag) are a fixed teal on every
            // map but the climb map, where their real climb rate puts them on the ramp.
            if (ColorMetric != Metric.Climb &&
                (c.Cruise == PerformanceEnvelopeCalculator.CruiseState.NoBurnableFuel ||
                 c.Cruise == PerformanceEnvelopeCalculator.CruiseState.IdleExceedsDrag))
                return false;
            return true;
        }

        private static Color NonRampColor(in PerformanceEnvelopeCalculator.CellResult c)
        {
            if (!c.Trimmed || c.Suppressed)
                return Color.black; // no trim here, or hidden by cleanup (isolated dot / disconnected)
            if (!c.Feasible)
                // A heat wall (an engine at its temperature limit, and even the thermally-capped thrust
                // cannot hold this speed) reads off-white; a plain thrust shortfall reads grey.
                return c.Overheats
                           ? new Color(0.867f, 0.867f, 0.867f)
                           : new Color(0.5f, 0.5f, 0.5f);
            return new Color(0.35f, 0.6f, 0.65f); // feasible but no cruise-endurance figure: teal
        }

        /// <summary>The value the current metric colours by, plus whether low is the better (cooler) end.</summary>
        private void MetricValue(in PerformanceEnvelopeCalculator.CellResult c, out double value, out bool lowerIsBetter)
        {
            switch (ColorMetric)
            {
                case Metric.Range: value = RangeKm(c); lowerIsBetter = false; break;
                case Metric.Aoa: value = c.Alpha; lowerIsBetter = true; break;
                case Metric.Throttle: value = c.ThrottleRequired; lowerIsBetter = true; break;
                case Metric.Climb: value = c.ClimbRateMPerS; lowerIsBetter = false; break;
                default: value = c.EnduranceHours; lowerIsBetter = false; break;
            }
        }

        /// <summary>Whole-map best for the current metric — the top of the ramp when not rescaling to view.</summary>
        private double MetricWholeMax()
        {
            switch (ColorMetric)
            {
                case Metric.Range: return _result.MaxRangeKm;
                case Metric.Aoa: return _result.MaxAlphaDeg;
                case Metric.Throttle: return _result.MaxThrottle;
                case Metric.Climb: return _result.MaxClimbRateMPerS;
                default: return _result.MaxEnduranceHours;
            }
        }

        /// <summary>
        /// Maps a raw metric value into the space the colour ramp is normalised over. Endurance and range
        /// span orders of magnitude and collapse toward zero at the feasibility edge, which linear
        /// normalisation crams into a razor-thin, numerically-noisy warm strip (the salt-and-pepper edge);
        /// log-compressing them spreads the warm->cool gradient smoothly across several cells. AoA, throttle
        /// and climb are bounded (climb can be negative), so they stay linear.
        /// </summary>
        private static double RampSpace(double value)
        {
            if (ColorMetric == Metric.Endurance || ColorMetric == Metric.Range)
                return value > 0 ? Math.Log(1.0 + value) : 0.0;
            return value;
        }

        public Color EnvelopeColor(int i, int j)
        {
            PerformanceEnvelopeCalculator.CellResult c = _result.Envelope[i, j];
            if (!IsRampColored(c))
                return NonRampColor(c);

            MetricValue(c, out double value, out bool lowerIsBetter);

            // Normalise onto the ramp. Zoomed with rescale on, the window's own min..max is re-spread for
            // full local contrast; otherwise the floor is 0 and the top is the map's best cell. A cell
            // whose value is unresolved (e.g. a near-empty cruise, Breguet -> 0) sits at the low end. For
            // AoA/throttle low is best, so invert onto the cool end.
            // Normalise over the robust value range computed for the shown cells (the whole map, or the
            // visible window when zoomed + rescaling). Using the data's own min..max rather than a fixed 0
            // floor keeps the ramp spread across the real spread of values, so an all-high map (endurance
            // especially, once log-compressed) doesn't read uniformly blue. Before the range is computed
            // (initial paint / a mid-sweep progressive row) fall back to 0..whole-map max.
            bool haveRange = _viewMax > _viewMin;
            double lo = RampSpace(haveRange ? _viewMin : 0);
            double hi = RampSpace(haveRange ? _viewMax : MetricWholeMax());
            double v = RampSpace(value);
            float n = !FARMathUtil.IsFinite(value) || hi <= lo
                          ? (lowerIsBetter ? 1f : 0f)
                          : Mathf.Clamp01((float)((v - lo) / (hi - lo)));
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

        // Current metric's value range across the visible window's ramp-coloured cells, for the rescale.
        private double _viewMin, _viewMax;
        private int _paintedEpoch = -1;
        private readonly List<double> _viewSamples = new List<double>(); // reused scratch for the percentile range

        /// <summary>Recomputes the visible-window value range for the current metric, over the same cells
        /// EnvelopeColor ramps. Only meaningful while rescaling a zoomed view.</summary>
        private void ComputeViewRange()
        {
            _viewMin = 0;
            _viewMax = 0;
            if (_result?.Envelope == null)
                return;

            // Robust 5th..95th percentile range over the ramp-coloured cells: near the feasibility edge a
            // handful of near-zero cells would otherwise stretch the ramp and blow out the contrast. Over
            // the visible window when zoomed + rescaling (local contrast), else the whole map.
            int nM = _result.MachAxis.Length, nA = _result.AltAxis.Length;
            if (RescaleColorsToView && _zoomActive)
            {
                // Zoomed local contrast: the visible window of the live (current-fuel) grid.
                VisibleBounds(out int i0, out int i1, out int j0, out int j1);
                CollectViewSamples(_result.Envelope, i0, i1, j0, j1);
            }
            else
            {
                // Whole map at the current fuel level — each level scales to its own data range so it keeps
                // full contrast (anchoring the scale to the wet sample instead made a near-empty map read
                // uniformly red, since every cell's endurance is tiny against the full-fuel range).
                CollectViewSamples(_result.Envelope, 0, nM - 1, 0, nA - 1);
            }

            if (_viewSamples.Count == 0)
                return;

            _viewSamples.Sort();
            _viewMin = Percentile(_viewSamples, 0.05);
            _viewMax = Percentile(_viewSamples, 0.95);

            // Degenerate window (few cells, or all-equal after clipping): fall back to raw min..max so the
            // ramp still spans something.
            if (_viewMax <= _viewMin)
            {
                _viewMin = _viewSamples[0];
                _viewMax = _viewSamples[_viewSamples.Count - 1];
            }
        }

        /// <summary>Fills <see cref="_viewSamples" /> with the current metric's finite values over the
        /// ramp-coloured cells of <paramref name="grid" /> in [i0..i1] x [j0..j1].</summary>
        private void CollectViewSamples(PerformanceEnvelopeCalculator.CellResult[,] grid, int i0, int i1, int j0, int j1)
        {
            _viewSamples.Clear();
            for (int j = j0; j <= j1; j++)
                for (int i = i0; i <= i1; i++)
                {
                    PerformanceEnvelopeCalculator.CellResult c = grid[i, j];
                    if (!IsRampColored(c))
                        continue;
                    MetricValue(c, out double v, out bool _);
                    if (FARMathUtil.IsFinite(v))
                        _viewSamples.Add(v);
                }
        }

        /// <summary>Linear-interpolated percentile of an already-sorted list (p in [0,1]).</summary>
        private static double Percentile(List<double> sorted, double p)
        {
            if (sorted.Count == 1)
                return sorted[0];
            double idx = p * (sorted.Count - 1);
            int lo = (int)idx;
            int hi = Math.Min(lo + 1, sorted.Count - 1);
            return sorted[lo] + (sorted[hi] - sorted[lo]) * (idx - lo);
        }

        /// <summary>Recolours the envelope texture if the shared colour epoch (zoom / metric / rescale)
        /// has moved since it was last painted. Cheap and idempotent; called each frame before drawing.</summary>
        private void SyncColors()
        {
            if (_paintedEpoch == _colorEpoch || _envTex == null)
                return;
            _paintedEpoch = _colorEpoch;
            RepaintEnvelope(); // recomputes the view range itself when zoomed + rescaling
        }

        private Color StabilityColor(int i, int j)
        {
            HeatmapResult.StabilityCell c = _result.Stability?[i, j];
            if (c == null || c.Score < 0)
                return new Color(0.5f, 0.2f, 0.6f); // errored — distinct from any score colour

            // The performance porkchop ramp minus its indigo top stop, fewer-out-of-range-is-better: a
            // clean cell (score 0) reads cool (blue), the worst (most derivatives out of range) reads red.
            Color color = Ramp(StabilityStops, 1f - Mathf.Clamp01((float)c.Score / MaxScore));

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

        /// <summary>Full repaint — needed once the sweep ends (colours scale to the best cell) and after
        /// the fuel slider / mass interpolation moves. Refreshes the visible-window range first when
        /// rescaling a zoomed view, so every repaint path keeps the ramp in step with the shown cells.</summary>
        public void RepaintEnvelope()
        {
            if (_envTex == null)
                return;
            ComputeViewRange(); // whole-map (or windowed) robust range, so the ramp always spans the data
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
        /// <summary>Texture UV rect for the visible cell window. Texture y is the altitude index and UV v
        /// runs bottom-up, so the lowest visible altitude sits at v0 and draws at the bottom of the rect.</summary>
        private Rect TexCoords(int i0, int i1, int j0, int j1)
        {
            int nM = _result.MachAxis.Length, nA = _result.AltAxis.Length;
            return new Rect((float)i0 / nM, (float)j0 / nA, (float)(i1 - i0 + 1) / nM, (float)(j1 - j0 + 1) / nA);
        }

        /// <summary>
        /// Handles the drag-to-zoom gesture on <paramref name="map" />: press starts a box, release sets
        /// the shared zoom window to the dragged cell range (a full-map box just clears the zoom). A press
        /// and release with no real drag is a click — the cell is returned so the caller can act on it.
        /// </summary>
        private (int i, int j)? HandleBoxZoom(Rect map)
        {
            Event e = Event.current;
            if (e == null)
                return null;

            int nM = _result.MachAxis.Length, nA = _result.AltAxis.Length;
            VisibleBounds(out int i0, out int i1, out int j0, out int j1);

            if (e.type == EventType.MouseDown && e.button == 0 && map.Contains(e.mousePosition))
            {
                _dragOwner = this;
                _dragStartMouse = e.mousePosition;
                _dragCurMouse = e.mousePosition;
                e.Use();
                return null;
            }

            if (_dragOwner != this)
                return null;

            if (e.type == EventType.MouseDrag && e.button == 0)
            {
                _dragCurMouse = e.mousePosition;
                e.Use();
            }
            else if (e.type == EventType.MouseUp && e.button == 0)
            {
                _dragCurMouse = e.mousePosition;
                _dragOwner = null;
                e.Use();

                if ((_dragCurMouse - _dragStartMouse).magnitude < DragThresholdPx)
                    return HeatmapAxes.CellAt(map, _dragStartMouse, i0, i1, j0, j1); // a click, not a box

                (int i, int j) a = HeatmapAxes.CellAt(map, _dragStartMouse, i0, i1, j0, j1);
                (int i, int j) b = HeatmapAxes.CellAt(map, _dragCurMouse, i0, i1, j0, j1);
                int zi0 = Mathf.Min(a.i, b.i), zi1 = Mathf.Max(a.i, b.i);
                int zj0 = Mathf.Min(a.j, b.j), zj1 = Mathf.Max(a.j, b.j);

                // Keep at least a 2x2 window so a nearly-still drag does not collapse to one cell.
                if (zi1 == zi0) { if (zi1 < nM - 1) zi1++; else if (zi0 > 0) zi0--; }
                if (zj1 == zj0) { if (zj1 < nA - 1) zj1++; else if (zj0 > 0) zj0--; }
                SetZoom(zi0, zi1, zj0, zj1, nM, nA);
            }

            return null;
        }

        public void DrawEnvelopeMap(float mapWidth, float mapHeight, (int i, int j)? highlight = null, bool showLabel = true)
        {
            _hoverEnv = null;
            ClickedEnvelopeCell = null;
            DiagClickedEnvelopeCell = null;
            if (_envTex == null)
                return;

            SyncColors(); // recolour if zoom / metric / rescale moved since last frame

            if (showLabel)
                GUILayout.Label(!string.IsNullOrEmpty(_result.EngineTypeLabel)
                                    ? _result.EngineTypeLabel
                                    : Localizer.Format("FARHeatmapsEnvLabel"));

            Rect map = HeatmapAxes.ReserveRect(_envTex, mapWidth, mapHeight);
            VisibleBounds(out int i0, out int i1, out int j0, out int j1);
            GUI.DrawTextureWithTexCoords(map, _envTex, TexCoords(i0, i1, j0, j1));
            HeatmapAxes.Draw(map, _result.MachAxis, _result.AltAxis, _result.SpeedAxisMode, i0, i1, j0, j1);
            if (highlight != null)
                HeatmapAxes.DrawCellBorder(map, _result.MachAxis, _result.AltAxis, highlight.Value, Color.white, i0, i1, j0, j1);

            _hoverEnv = HeatmapAxes.HitTest(map, _result.MachAxis, _result.AltAxis, i0, i1, j0, j1);

            (int i, int j)? click = HandleBoxZoom(map);
            if (click != null)
                ClickedEnvelopeCell = click;
            else if (_hoverEnv != null && HeatmapAxes.MiddleClickedIn(map))
                DiagClickedEnvelopeCell = _hoverEnv;

            if (_dragOwner == this)
                HeatmapAxes.DrawSelectionBox(map, _dragStartMouse, _dragCurMouse);
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
                GUILayout.Label(StabilityCaption);
            Rect smap = HeatmapAxes.ReserveRect(_stabTex, mapWidth, mapHeight);
            VisibleBounds(out int i0, out int i1, out int j0, out int j1);
            GUI.DrawTextureWithTexCoords(smap, _stabTex, TexCoords(i0, i1, j0, j1));
            HeatmapAxes.Draw(smap, _result.MachAxis, _result.AltAxis, _result.SpeedAxisMode, i0, i1, j0, j1);
            if (highlight != null)
                HeatmapAxes.DrawCellBorder(smap, _result.MachAxis, _result.AltAxis, highlight.Value, Color.white, i0, i1, j0, j1);
            _hoverStab = HeatmapAxes.HitTest(smap, _result.MachAxis, _result.AltAxis, i0, i1, j0, j1);

            // Stability shares the zoom, so a drag here reframes every map too; its own colours are a fixed
            // score ramp (not view-rescaled), so no recolour is needed — only the window changes.
            HandleBoxZoom(smap);
            if (_dragOwner == this)
                HeatmapAxes.DrawSelectionBox(smap, _dragStartMouse, _dragCurMouse);
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

            // Assumed, not solved: past several heat-walled cells in this row the sweep stopped computing
            // the high-Mach tail and filled the rest as heat wall, so it carries no measured numbers.
            if (c.Extrapolated)
            {
                sb.Append("\n\nHeat wall (assumed) — past several cells the\nengines could not hold, so this was not computed.");
                AppendRowEnvelope(sb, j);
                return sb.ToString();
            }

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
            {
                sb.AppendFormat(CultureInfo.InvariantCulture,
                                "\n  {0} = {1} ({2})",
                                name,
                                double.IsNaN(value) ? "NaN" : value.ToString("G3", CultureInfo.InvariantCulture),
                                expectedSign < 0 ? "exp <0" : "exp >0");
                // The Stability tab's own explanation for this derivative, so the map tooltip is
                // self-explanatory rather than just naming the derivative.
                if (StabDerivExpKeys.TryGetValue(name, out string expKey))
                    sb.Append("\n    " + Localizer.Format(expKey));
            }

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

            // Two lines: ceiling + top speed, then endurance + range. Built here (not via the one-line
            // FAREnvelopeSummary template) so the newline survives without touching the localization cfg.
            return string.Format(CultureInfo.InvariantCulture,
                                 "Ceiling: {0}    Top speed: {1}\nBest endurance: {2}    Best range: {3}",
                                 ceiling, top, endurance, range);
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
