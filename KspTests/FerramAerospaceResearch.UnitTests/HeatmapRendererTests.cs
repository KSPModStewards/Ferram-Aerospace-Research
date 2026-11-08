using System;
using System.Reflection;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using UnityEngine;
using Xunit;
using Cell = FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation.PerformanceEnvelopeCalculator.CellResult;
using Cruise = FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation.PerformanceEnvelopeCalculator.CruiseState;
using Metric = FerramAerospaceResearch.FARGUI.FAREditorGUI.HeatmapRenderer.Metric;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// HeatmapRenderer's text and colours without drawing anything: the hover tooltips (cell, row envelope, stability),
    /// the row's top and minimum level speeds, the colour of each kind of cell per colour metric, the robust value range
    /// the ramp spreads over, and the shared zoom window's recentring. The renderer is bound to a hand-built
    /// HeatmapResult (its private _result: SetResult makes textures, which need the engine).
    /// </summary>
    [Collection(HeatmapStatics.Collection)]
    public class HeatmapRendererTests : IDisposable
    {
        private const BindingFlags Instance = BindingFlags.NonPublic | BindingFlags.Instance;
        private const BindingFlags Static = BindingFlags.NonPublic | BindingFlags.Static;

        // Porkchop ramp ends: worst (red) and best (indigo)
        private static readonly float[] Red = { 0.85f, 0.10f, 0.10f };
        private static readonly float[] Indigo = { 0.48f, 0.35f, 0.95f };
        private static readonly float[] Teal = { 0.35f, 0.6f, 0.65f };

        private readonly Metric metric;
        private readonly bool rescale;

        public HeatmapRendererTests()
        {
            if (!UnityRuntime.Available)
                return;
            metric = HeatmapRenderer.ColorMetric;
            rescale = HeatmapRenderer.RescaleColorsToView;
            HeatmapRenderer.ColorMetric = Metric.Endurance;
            HeatmapRenderer.RescaleColorsToView = true;
            HeatmapRenderer.ResetZoom();
        }

        public void Dispose()
        {
            if (!UnityRuntime.Available)
                return;
            HeatmapRenderer.ColorMetric = metric;
            HeatmapRenderer.RescaleColorsToView = rescale;
            HeatmapRenderer.ResetZoom();
        }

        private static HeatmapRenderer Bound(HeatmapResult result)
        {
            var renderer = new HeatmapRenderer();
            typeof(HeatmapRenderer).GetField("_result", Instance).SetValue(renderer, result);
            return renderer;
        }

        /// <summary>A one-altitude map (5 km) of these cells, Mach axis from the cells, the summary recomputed.</summary>
        private static HeatmapRenderer Row(params Cell[] cells)
        {
            var mach = new double[cells.Length];
            for (int i = 0; i < cells.Length; i++)
                mach[i] = cells[i].Mach;
            var result = new HeatmapResult { MachAxis = mach, AltAxis = new[] { 5000.0 }, Envelope = Cells.Row(cells) };
            result.Recompute();
            return Bound(result);
        }

        private static HeatmapResult ResultOf(HeatmapRenderer r) =>
            (HeatmapResult)typeof(HeatmapRenderer).GetField("_result", Instance).GetValue(r);

        private static object Call(HeatmapRenderer r, string name, params object[] args) =>
            typeof(HeatmapRenderer).GetMethod(name, Instance).Invoke(r, args);

        private static T StaticField<T>(string name) => (T)typeof(HeatmapRenderer).GetField(name, Static).GetValue(null);

        private static int Epoch => StaticField<int>("_colorEpoch");
        private static bool ZoomActive => StaticField<bool>("_zoomActive");

        private static (int, int, int, int) Zoom
        {
            get
            {
                var z = StaticField<HeatmapRenderer.ZoomWindow>("_zoom");
                return (z.IMin, z.IMax, z.JMin, z.JMax);
            }
        }

        private static void AssertColor(float[] rgb, Color actual)
        {
            Assert.Equal(rgb[0], actual.r, 4);
            Assert.Equal(rgb[1], actual.g, 4);
            Assert.Equal(rgb[2], actual.b, 4);
        }

        private static Cell With(Cell c, Action<Cell[]> edit)
        {
            var box = new[] { c };
            edit(box);
            return box[0];
        }

        // --- envelope tooltip ------------------------------------------------------------------------

        /// <summary>The middle cell of a slow-cruise-fast row, so the row envelope below it is ordinary.</summary>
        private static string Tooltip(Cell middle) =>
            Row(Cells.Cruise(0.5), middle, Cells.Grey(0.9)).EnvelopeTooltip(1, 0);

        [SkippableFact]
        public void ExtrapolatedCellSaysItWasNotComputed()
        {
            UnityRuntime.Require();
            Cell c = Cells.Heat(0.7);
            c.Extrapolated = true;
            string tip = Tooltip(c);
            Assert.StartsWith("M 0.70", tip);
            Assert.Contains("210 m/s", tip);
            Assert.Contains("5.0 km", tip);
            Assert.Contains("Heat wall (assumed)", tip);
            Assert.Contains("so this was not computed.", tip);
            Assert.DoesNotContain("Thrust avail", tip);   // no measured numbers
            Assert.Contains("Top speed", tip);            // but the row's envelope
        }

        [SkippableFact]
        public void ReducedEngineCellSaysRamjetOnly()
        {
            UnityRuntime.Require();
            Cell c = Cells.Cruise(0.7);
            c.ReducedEngines = true;
            string tip = Tooltip(c);
            Assert.Contains("Turbojets shut off here", tip);
            Assert.Contains("figures below are ramjet-only.", tip);
            Assert.Contains("Thrust avail : 3.0 kN (sustainable)", tip);
        }

        [SkippableFact]
        public void UntrimmedCellHasNoFigures()
        {
            UnityRuntime.Require();
            string tip = Tooltip(Cells.Black(0.7));
            Assert.Contains("Cannot hold level flight here", tip);
            Assert.Contains("no angle of attack produces enough lift.", tip);
            Assert.DoesNotContain("Thrust avail", tip);
            Assert.DoesNotContain("Drag exceeds thrust", tip);
        }

        [SkippableFact]
        public void SuppressedCellIsHiddenButKeepsItsFigures()
        {
            UnityRuntime.Require();
            Cell c = Cells.Grey(0.7, -2500);
            c.Suppressed = true;
            string tip = Tooltip(c);
            Assert.Contains("Hidden", tip);
            Assert.Contains("shown for reference only.", tip);
            Assert.Contains("Thrust avail : 0.5 kN", tip);
            Assert.Contains("Drag         : 3.0 kN", tip);
            Assert.Contains("Drag exceeds thrust by 2.5 kN", tip);
        }

        [SkippableFact]
        public void ThrustShortCellSaysByHowMuch()
        {
            UnityRuntime.Require();
            string tip = Tooltip(Cells.Grey(0.7, -1234));
            Assert.Contains("Drag exceeds thrust by 1.2 kN", tip);
            Assert.Contains("level flight not sustainable here.", tip);
            Assert.DoesNotContain("Heat", tip);
            Assert.DoesNotContain("Throttle req", tip);
        }

        [SkippableFact]
        public void HeatLimitedCellsTellAWallFromALimiter()
        {
            UnityRuntime.Require();
            Cell limited = Cells.Cruise(0.7);
            limited.Overheats = true;
            string tip = Tooltip(limited);
            Assert.Contains("Heat-limited : an engine runs a reduced thrust", tip);
            Assert.Contains("Throttle req : 60%", tip);   // still a cruise

            string wall = Tooltip(Cells.Heat(0.7));
            Assert.Contains("Heat wall: even at the thermal limit the", wall);
            Assert.DoesNotContain("Heat-limited", wall);
            Assert.DoesNotContain("Drag exceeds thrust", wall);   // a heat wall, not a plain shortfall
        }

        [SkippableFact]
        public void CruiseCellShowsItsEnduranceAndRange()
        {
            UnityRuntime.Require();
            Cell c = Cells.Cruise(0.7);
            c.ThermalRatio = 0.5;
            string tip = Tooltip(c);
            Assert.Contains("Throttle req : 60%", tip);
            Assert.Contains("Fuel flow    : 0.200 kg/s", tip);
            Assert.Contains("Endurance    : 1.50 hr", tip);
            Assert.Contains("Range        : 1134 km", tip);   // 1.5 hr at 210 m/s
            Assert.Contains("L/D          : 10.00", tip);
            Assert.Contains("Climb rate   : +20 m/s", tip);
            Assert.Contains("Engine temp  : 50% of limit at cruise", tip);
            Assert.DoesNotContain("capped", tip);

            c.ThermalRatio = 1.2;
            Assert.Contains("Engine temp  : 98% of limit at cruise (capped from 120%)", Tooltip(c));
        }

        [SkippableTheory]
        [InlineData((int)Cruise.IdleExceedsDrag, "idle thrust still exceeds drag, so the")]
        [InlineData((int)Cruise.NoBurnableFuel, "No endurance: nothing aboard that")]
        [InlineData((int)Cruise.Unsolved, "Cruise throttle did not converge.")]
        public void CellsWithoutACruiseSayWhy(int cruise, string text)
        {
            UnityRuntime.Require();
            Cell c = Cells.Cruise(0.7);
            c.Cruise = (Cruise)cruise;
            string tip = Tooltip(c);
            Assert.Contains(text, tip);
            Assert.DoesNotContain("Throttle req", tip);
            Assert.DoesNotContain("Endurance    :", tip);
        }

        // --- row envelope ----------------------------------------------------------------------------

        [SkippableFact]
        public void RowWithoutLevelFlightSaysSo()
        {
            UnityRuntime.Require();
            HeatmapRenderer r = Row(Cells.Black(0.5), Cells.Grey(0.7), Cells.Heat(0.9));
            Assert.False(r.TopSpeedAtRow(0).Valid);
            Assert.False(r.MinLevelSpeedAtRow(0).Valid);
            string tip = r.EnvelopeTooltip(1, 0);
            Assert.Contains("Level flight not possible at any", tip);
            Assert.Contains("speed in the swept range.", tip);
            Assert.DoesNotContain("Top speed", tip);
        }

        /// <summary>Thrust-limited both ends: the edges are where the excess thrust crosses zero, linearly between
        /// the last feasible cell and the next (excess +1000 N to -3000 N: a quarter of the way).</summary>
        [SkippableFact]
        public void EdgesInterpolateTheThrustCrossing()
        {
            UnityRuntime.Require();
            HeatmapRenderer r = Row(Cells.Grey(0.5, -1000), Cells.Cruise(0.7, 1000), Cells.Grey(0.9, -3000));

            HeatmapRenderer.EnvelopeEdge top = r.TopSpeedAtRow(0);
            Assert.True(top.Valid);
            Assert.Equal(225, top.SpeedMPerS, 9);
            Assert.Equal(0.75, top.Mach, 9);
            Assert.False(top.Clipped || top.TrimFailedBeyond);

            HeatmapRenderer.EnvelopeEdge min = r.MinLevelSpeedAtRow(0);
            Assert.True(min.Valid);
            Assert.Equal(180, min.SpeedMPerS, 9);
            Assert.Equal(0.6, min.Mach, 9);
            Assert.False(min.Clipped || min.LiftLimited);

            string tip = r.EnvelopeTooltip(1, 0);
            Assert.Contains("Top speed    : 225 m/s (M 0.75)\n", tip);
            Assert.Contains("Min level spd: 180 m/s (M 0.60)", tip);
            Assert.DoesNotContain("still flying", tip);
        }

        /// <summary>Feasible at both ends of the swept range: the edges are the range's, marked + and -.</summary>
        [SkippableFact]
        public void EdgesClippedByTheSweptRange()
        {
            UnityRuntime.Require();
            HeatmapRenderer r = Row(Cells.Cruise(0.5), Cells.Cruise(0.7), Cells.Cruise(0.9));
            HeatmapRenderer.EnvelopeEdge min = r.MinLevelSpeedAtRow(0);
            Assert.True(min.Valid && min.Clipped);
            Assert.Equal(150, min.SpeedMPerS, 9);
            Assert.Equal(0.5, min.Mach, 9);
            Assert.True(r.TopSpeedAtRow(0).Clipped);

            string tip = r.EnvelopeTooltip(0, 0);
            Assert.Contains("Top speed    : 270 m/s (M 0.90) +", tip);
            Assert.Contains("Min level spd: 150 m/s (M 0.50) \u2212", tip);
            Assert.Contains("+ still flying at the fastest Mach swept", tip);
            Assert.Contains("still flying at the slowest Mach swept", tip);
        }

        /// <summary>Untrimmed below the slowest feasible cell: a stall edge. Untrimmed (or suppressed) above the
        /// fastest: the trim solve failed, the top speed a lower bound.</summary>
        [SkippableFact]
        public void StallBelowAndTrimFailureAbove()
        {
            UnityRuntime.Require();
            HeatmapRenderer r = Row(Cells.Black(0.5), Cells.Cruise(0.7), Cells.Black(0.9));
            HeatmapRenderer.EnvelopeEdge min = r.MinLevelSpeedAtRow(0);
            Assert.True(min.LiftLimited);
            Assert.False(min.Clipped);
            Assert.Equal(210, min.SpeedMPerS, 9);
            HeatmapRenderer.EnvelopeEdge top = r.TopSpeedAtRow(0);
            Assert.True(top.TrimFailedBeyond);
            Assert.Equal(210, top.SpeedMPerS, 9);

            string tip = r.EnvelopeTooltip(1, 0);
            Assert.Contains("Top speed    : 210 m/s (M 0.70) (!)", tip);
            Assert.Contains("Min level spd: 210 m/s (M 0.70) (stall)", tip);
            Assert.Contains("! trim solve failed past this point", tip);

            Cell hidden = Cells.Grey(0.9);
            hidden.Suppressed = true;
            Assert.True(Row(Cells.Black(0.5), Cells.Cruise(0.7), hidden).TopSpeedAtRow(0).TrimFailedBeyond);
            Assert.True(Row(hidden, Cells.Cruise(0.7), Cells.Cruise(0.9)).MinLevelSpeedAtRow(0).LiftLimited);
        }

        /// <summary>A trimmed neighbour whose excess thrust is not negative (a heat wall) has no crossing: the edge
        /// is the feasible cell itself, with no flag.</summary>
        [SkippableFact]
        public void HeatWallNeighbourGivesTheCellsOwnSpeed()
        {
            UnityRuntime.Require();
            Cell wall = Cells.Heat(0.9);
            wall.ExcessThrustN = 500;
            HeatmapRenderer.EnvelopeEdge top = Row(Cells.Cruise(0.5), Cells.Cruise(0.7), wall).TopSpeedAtRow(0);
            Assert.True(top.Valid);
            Assert.Equal(210, top.SpeedMPerS, 9);
            Assert.False(top.Clipped || top.TrimFailedBeyond);
        }

        // --- stability tooltip -----------------------------------------------------------------------

        private static string StabilityTooltip(HeatmapRenderer r, int i, int j) => (string)Call(r, "StabilityTooltip", i, j);

        [SkippableFact]
        public void StabilityTooltipHeadersAndStates()
        {
            UnityRuntime.Require();
            var result = new HeatmapResult { MachAxis = new[] { 0.5, 250.0 }, AltAxis = new[] { 3000.0 } };
            HeatmapRenderer r = Bound(result);

            Assert.Equal("M 0.50  \u00b7  3.0 km\n\nNot evaluated.", StabilityTooltip(r, 0, 0));   // no stability grid

            result.Stability = new HeatmapResult.StabilityCell[2, 1];
            result.Stability[1, 0] = new HeatmapResult.StabilityCell { Score = -1 };
            result.SpeedAxisMode = true;
            string errored = StabilityTooltip(r, 1, 0);
            Assert.StartsWith("250 m/s  \u00b7  3.0 km", errored);
            Assert.Contains("Errored", errored);
            Assert.Contains("see KSP.log.", errored);
            Assert.DoesNotContain("Score", errored);

            Assert.EndsWith("Not evaluated.", StabilityTooltip(r, 0, 0));   // a null cell in the grid

            result.Stability[0, 0] = new HeatmapResult.StabilityCell { Score = 0 };
            string clean = StabilityTooltip(r, 0, 0);
            Assert.Contains("Score: 0/11 derivatives out of range", clean);
            Assert.Contains("All derivatives within expected sign.", clean);
        }

        // --- colours ---------------------------------------------------------------------------------

        [SkippableFact]
        public void CellsOffTheRampHaveFixedColours()
        {
            UnityRuntime.Require();
            Cell suppressed = Cells.Cruise(0.6);
            suppressed.Suppressed = true;
            Cell noFuel = Cells.Cruise(0.9);
            noFuel.Cruise = Cruise.NoBurnableFuel;
            Cell idle = Cells.Cruise(1.0);
            idle.Cruise = Cruise.IdleExceedsDrag;
            HeatmapRenderer r = Row(Cells.Black(0.5), suppressed, Cells.Grey(0.7), Cells.Heat(0.8), noFuel, idle);

            AssertColor(new[] { 0f, 0f, 0f }, r.EnvelopeColor(0, 0));
            AssertColor(new[] { 0f, 0f, 0f }, r.EnvelopeColor(1, 0));
            AssertColor(new[] { 0.5f, 0.5f, 0.5f }, r.EnvelopeColor(2, 0));
            AssertColor(new[] { 0.867f, 0.867f, 0.867f }, r.EnvelopeColor(3, 0));
            AssertColor(Teal, r.EnvelopeColor(4, 0));
            AssertColor(Teal, r.EnvelopeColor(5, 0));
        }

        /// <summary>No-cruise cells still have a climb rate: on the climb map they are on the ramp.</summary>
        [SkippableFact]
        public void NoCruiseCellsAreRampedOnTheClimbMap()
        {
            UnityRuntime.Require();
            Cell noFuel = Cells.Cruise(0.9);
            noFuel.Cruise = Cruise.NoBurnableFuel;
            noFuel.ClimbRateMPerS = 40;
            Cell idle = Cells.Cruise(1.0);
            idle.Cruise = Cruise.IdleExceedsDrag;
            idle.ClimbRateMPerS = 0;
            HeatmapRenderer r = Row(noFuel, idle);
            HeatmapRenderer.ColorMetric = Metric.Climb;
            AssertColor(Indigo, r.EnvelopeColor(0, 0));
            AssertColor(Red, r.EnvelopeColor(1, 0));
        }

        private static Cell High(double mach) =>
            With(Cells.Cruise(mach), b =>
            {
                b[0].EnduranceHours = 2;
                b[0].Alpha = 8;
                b[0].ThrottleRequired = 0.9;
                b[0].ClimbRateMPerS = 30;
            });

        private static Cell Low(double mach) =>
            With(Cells.Cruise(mach), b =>
            {
                b[0].EnduranceHours = 0;
                b[0].Alpha = 0;
                b[0].ThrottleRequired = 0;
                b[0].ClimbRateMPerS = -5;   // below the ramp's 0 floor: clamped
            });

        /// <summary>Range, endurance and climb colour high = best (indigo), AoA and throttle low = best; before a view
        /// range is computed the ramp runs from 0 to the whole map's best.</summary>
        [SkippableTheory]
        [InlineData((int)Metric.Endurance, true)]
        [InlineData((int)Metric.Range, true)]
        [InlineData((int)Metric.Climb, true)]
        [InlineData((int)Metric.Aoa, false)]
        [InlineData((int)Metric.Throttle, false)]
        public void EachMetricPutsItsBestCellAtTheCoolEnd(int metricIndex, bool highIsBest)
        {
            UnityRuntime.Require();
            HeatmapRenderer r = Row(High(0.5), Low(0.7));
            HeatmapRenderer.ColorMetric = (Metric)metricIndex;
            AssertColor(highIsBest ? Indigo : Red, r.EnvelopeColor(0, 0));
            AssertColor(highIsBest ? Red : Indigo, r.EnvelopeColor(1, 0));
        }

        /// <summary>AoA is normalised linearly (4 of 8 deg: mid-ramp, between yellow and green); endurance in log space,
        /// so mid-ramp is at ln(1 + e) = ln(3) / 2 of the best 2 hr, e = sqrt(3) - 1.</summary>
        [SkippableFact]
        public void RampSpaceIsLinearForAoaAndLogForEndurance()
        {
            UnityRuntime.Require();
            float[] mid = { 0.575f, 0.85f, 0.20f };
            Cell half = With(High(0.6), b =>
            {
                b[0].Alpha = 4;
                b[0].EnduranceHours = Math.Sqrt(3) - 1;
            });
            HeatmapRenderer r = Row(High(0.5), half, Low(0.7));
            HeatmapRenderer.ColorMetric = Metric.Aoa;
            AssertColor(mid, r.EnvelopeColor(1, 0));
            HeatmapRenderer.ColorMetric = Metric.Endurance;
            AssertColor(mid, r.EnvelopeColor(1, 0));
        }

        [SkippableTheory]
        [InlineData((int)Metric.Endurance, 1.0, 0.69314718055994530)]
        [InlineData((int)Metric.Range, 1.7182818284590451, 1.0)]
        [InlineData((int)Metric.Endurance, 0.0, 0.0)]
        [InlineData((int)Metric.Range, -4.0, 0.0)]
        [InlineData((int)Metric.Aoa, -3.0, -3.0)]
        [InlineData((int)Metric.Throttle, 0.5, 0.5)]
        [InlineData((int)Metric.Climb, -12.0, -12.0)]
        public void RampSpace(int metricIndex, double value, double space)
        {
            UnityRuntime.Require();
            HeatmapRenderer.ColorMetric = (Metric)metricIndex;
            Assert.Equal(space, (double)typeof(HeatmapRenderer).GetMethod("RampSpace", Static).Invoke(null, new object[] { value }), 12);
        }

        [SkippableTheory]
        [InlineData((int)Metric.Endurance, 1.5, false)]
        [InlineData((int)Metric.Range, 1134.0, false)]
        [InlineData((int)Metric.Aoa, 2.0, true)]
        [InlineData((int)Metric.Throttle, 0.6, true)]
        [InlineData((int)Metric.Climb, 20.0, false)]
        public void MetricValueAndWholeMapMax(int metricIndex, double value, bool lowerIsBetter)
        {
            UnityRuntime.Require();
            HeatmapRenderer r = Row(Cells.Cruise(0.7));
            HeatmapRenderer.ColorMetric = (Metric)metricIndex;
            var args = new object[] { Cells.Cruise(0.7), null, null };
            Call(r, "MetricValue", args);
            Assert.Equal(value, (double)args[1], 9);
            Assert.Equal(lowerIsBetter, (bool)args[2]);
            // the one-cell map's best is that cell
            Assert.Equal(value, (double)Call(r, "MetricWholeMax"), 9);

            HeatmapResult result = ResultOf(r);
            result.MaxEnduranceHours = 11;
            result.MaxRangeKm = 12;
            result.MaxAlphaDeg = 13;
            result.MaxThrottle = 14;
            result.MaxClimbRateMPerS = 15;
            Assert.Equal(11.0 + metricIndex, (double)Call(r, "MetricWholeMax"));
        }

        /// <summary>An unresolved value sits at the warm end whichever way the metric runs.</summary>
        [SkippableTheory]
        [InlineData((int)Metric.Endurance)]
        [InlineData((int)Metric.Aoa)]
        public void UnresolvedValueIsWorst(int metricIndex)
        {
            UnityRuntime.Require();
            Cell nan = With(High(0.7), b =>
            {
                b[0].EnduranceHours = double.NaN;
                b[0].Alpha = double.NaN;
            });
            HeatmapRenderer r = Row(High(0.5), nan);
            HeatmapRenderer.ColorMetric = (Metric)metricIndex;
            AssertColor(Red, r.EnvelopeColor(1, 0));
        }

        [SkippableFact]
        public void ReducedEngineCellsAreDimmedAndHeatLimitedOnesWarmed()
        {
            UnityRuntime.Require();
            Cell reduced = High(0.6);
            reduced.ReducedEngines = true;
            Cell hot = High(0.7);
            hot.Overheats = true;
            HeatmapRenderer r = Row(High(0.5), reduced, hot);

            AssertColor(Indigo, r.EnvelopeColor(0, 0));
            AssertColor(new[] { Indigo[0] * 0.775f, Indigo[1] * 0.775f, Indigo[2] * 0.775f }, r.EnvelopeColor(1, 0));
            AssertColor(new[] { Indigo[0] + (1 - Indigo[0]) * 0.25f, Indigo[1] * 0.8f, Indigo[2] * 0.7f }, r.EnvelopeColor(2, 0));
        }

        // --- view range ------------------------------------------------------------------------------

        private static (double min, double max) ViewRange(HeatmapRenderer r)
        {
            Call(r, "ComputeViewRange");
            return ((double)typeof(HeatmapRenderer).GetField("_viewMin", Instance).GetValue(r),
                    (double)typeof(HeatmapRenderer).GetField("_viewMax", Instance).GetValue(r));
        }

        private static HeatmapRenderer EnduranceRow(int n)
        {
            var cells = new Cell[n];
            for (int i = 0; i < n; i++)
            {
                cells[i] = Cells.Cruise(0.5 + 0.1 * i);
                cells[i].EnduranceHours = i;
            }
            return Row(cells);
        }

        [SkippableFact]
        public void ViewRangeIsTheFifthToNinetyFifthPercentile()
        {
            UnityRuntime.Require();
            (double min, double max) = ViewRange(EnduranceRow(21));
            Assert.Equal(1, min, 9);
            Assert.Equal(19, max, 9);
        }

        [SkippableFact]
        public void ViewRangeWithoutRampCellsIsEmpty()
        {
            UnityRuntime.Require();
            Assert.Equal((0.0, 0.0), ViewRange(Row(Cells.Grey(0.5), Cells.Black(0.6), Cells.Heat(0.7))));
            Assert.Equal((0.0, 0.0), ViewRange(Bound(new HeatmapResult { MachAxis = new[] { 0.5 }, AltAxis = new[] { 0.0 } })));

            Cell nan = Cells.Cruise(0.8);
            nan.EnduranceHours = double.NaN;   // ramped, but no finite value to sample
            Assert.Equal((0.0, 0.0), ViewRange(Row(nan)));
        }

        /// <summary>One sample, or all equal: the percentiles collapse, and the range falls back to the values' own
        /// min..max (here one value) rather than failing.</summary>
        [SkippableFact]
        public void DegenerateViewRangeFallsBackToMinMax()
        {
            UnityRuntime.Require();
            Assert.Equal((1.5, 1.5), ViewRange(Row(Cells.Cruise(0.5), Cells.Grey(0.6))));
            Assert.Equal((1.5, 1.5), ViewRange(Row(Cells.Cruise(0.5), Cells.Cruise(0.6), Cells.Cruise(0.7))));
        }

        /// <summary>Zoomed with recolour-to-view on, the range is the visible window's; off, the whole map's.</summary>
        [SkippableFact]
        public void ZoomedViewRangeIsTheWindows()
        {
            UnityRuntime.Require();
            HeatmapRenderer r = EnduranceRow(21);
            HeatmapRenderer.CenterZoomOn(10, 0, 21, 1);   // columns 5..15
            Assert.True(ZoomActive);
            (double min, double max) = ViewRange(r);
            Assert.Equal(5.5, min, 9);
            Assert.Equal(14.5, max, 9);

            HeatmapRenderer.RescaleColorsToView = false;
            (min, max) = ViewRange(r);
            Assert.Equal(1, min, 9);
            Assert.Equal(19, max, 9);
        }

        // --- zoom ------------------------------------------------------------------------------------

        [SkippableFact]
        public void CenterZoomOnAnEmptyGridDoesNothing()
        {
            UnityRuntime.Require();
            int epoch = Epoch;
            HeatmapRenderer.CenterZoomOn(3, 3, 0, 10);
            HeatmapRenderer.CenterZoomOn(3, 3, 10, -1);
            Assert.Equal(epoch, Epoch);
            Assert.False(ZoomActive);
        }

        /// <summary>
        /// Unzoomed, the window is a quarter grid each side of the cell (half-spans nM / 4, nA / 4); zoomed, it keeps
        /// the window's span; either way shifted to stay on the grid. Recentring where it already is changes nothing
        /// (the flight scene calls it every frame), anywhere else bumps the colour epoch.
        /// </summary>
        [SkippableFact]
        public void CenterZoomOnKeepsTheSpanAndStaysOnTheGrid()
        {
            UnityRuntime.Require();
            int epoch = Epoch;
            HeatmapRenderer.CenterZoomOn(10, 6, 20, 12);
            Assert.True(ZoomActive);
            Assert.Equal((5, 15, 3, 9), Zoom);
            Assert.Equal(epoch + 1, Epoch);

            HeatmapRenderer.CenterZoomOn(10, 6, 20, 12);   // already there
            Assert.Equal(epoch + 1, Epoch);

            HeatmapRenderer.CenterZoomOn(0, 0, 20, 12);    // the low corner: shifted up to 0
            Assert.Equal((0, 10, 0, 6), Zoom);
            Assert.Equal(epoch + 2, Epoch);

            HeatmapRenderer.CenterZoomOn(19, 11, 20, 12);  // the high corner: shifted down, span kept
            Assert.Equal((9, 19, 5, 11), Zoom);

            HeatmapRenderer.CenterZoomOn(12, 7, 20, 12);   // in the middle again: 12 +- 5, 7 +- 3
            Assert.Equal((7, 17, 4, 10), Zoom);
            Assert.True(ZoomActive);
        }

        [SkippableFact]
        public void CenterZoomOnAnotherGridStartsFromAQuarterSpan()
        {
            UnityRuntime.Require();
            HeatmapRenderer.CenterZoomOn(10, 6, 20, 12);
            HeatmapRenderer.CenterZoomOn(5, 5, 10, 10);    // new grid size: the 20 x 12 span does not carry over
            Assert.Equal((3, 7, 3, 7), Zoom);
            Assert.True(ZoomActive);
        }

        /// <summary>On a grid too small to zoom the window covers it all: no zoom (but the colours are refreshed).</summary>
        [SkippableFact]
        public void CenterZoomOnATinyGridIsNoZoom()
        {
            UnityRuntime.Require();
            int epoch = Epoch;
            HeatmapRenderer.CenterZoomOn(1, 0, 2, 2);
            Assert.False(ZoomActive);
            Assert.Equal((0, 1, 0, 1), Zoom);
            Assert.Equal(epoch + 1, Epoch);
        }

        [SkippableFact]
        public void EngineTypeLabelIsTheResults()
        {
            UnityRuntime.Require();
            Assert.Equal("", new HeatmapRenderer().EngineTypeLabel);
            Assert.Equal("Ramjet", Bound(new HeatmapResult { EngineTypeLabel = "Ramjet" }).EngineTypeLabel);
        }
    }
}
