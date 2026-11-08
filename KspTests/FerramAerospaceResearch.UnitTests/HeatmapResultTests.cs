using System;
using System.Collections.Generic;
using System.Reflection;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using Xunit;
using Cell = FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation.PerformanceEnvelopeCalculator.CellResult;
using Cruise = FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation.PerformanceEnvelopeCalculator.CruiseState;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// The tests that set the heatmap display's statics (HeatmapResult.CleanupDisplay, HeatmapRenderer's colour
    /// metric and zoom): one collection, so xunit does not run them side by side.
    /// </summary>
    internal static class HeatmapStatics
    {
        public const string Collection = "FAR heatmap statics";
    }

    /// <summary>Hand-built envelope cells for the heatmap tests.</summary>
    internal static class Cells
    {
        /// <summary>A feasible cruise cell (porkchop-coloured).</summary>
        public static Cell Cruise(double mach, double excessN = 1000, double alpha = 2, double dragN = 2000) =>
            new Cell
            {
                Trimmed = true,
                Feasible = true,
                Cruise = PerformanceEnvelopeCalculator.CruiseState.Ok,
                Mach = mach,
                SpeedMPerS = mach * 300,
                Alpha = alpha,
                DragN = dragN,
                ThrustAvailableN = dragN + excessN,
                ExcessThrustN = excessN,
                LiftToDrag = 10,
                FuelFlowKgPerS = 0.2,
                ThrottleRequired = 0.6,
                EnduranceHours = 1.5,
                ClimbRateMPerS = 20
            };

        /// <summary>Trimmed but thrust-short (grey).</summary>
        public static Cell Grey(double mach, double excessN = -1000) =>
            new Cell
            {
                Trimmed = true,
                Mach = mach,
                SpeedMPerS = mach * 300,
                DragN = 3000,
                ThrustAvailableN = 3000 + excessN,
                ExcessThrustN = excessN,
                EnduranceHours = double.NaN,
                ThrottleRequired = double.NaN
            };

        /// <summary>Trimmed, infeasible, overheating: the heat wall.</summary>
        public static Cell Heat(double mach)
        {
            Cell c = Grey(mach);
            c.Overheats = true;
            return c;
        }

        /// <summary>No trim (black).</summary>
        public static Cell Black(double mach) => new Cell { Mach = mach, SpeedMPerS = mach * 300 };

        /// <summary>One row (altitude) of cells, as the [mach, alt] grid the sweep fills.</summary>
        public static Cell[,] Row(params Cell[] cells)
        {
            var grid = new Cell[cells.Length, 1];
            for (int i = 0; i < cells.Length; i++)
                grid[i, 0] = cells[i];
            return grid;
        }
    }

    /// <summary>
    /// HeatmapResult's display side: the stability map's performance mask, picking and interpolating the mass samples
    /// (the editor's fuel slider, the flight scene's live mass), and the cleanup passes over the displayed grid.
    /// </summary>
    [Collection(HeatmapStatics.Collection)]
    public class HeatmapResultTests : IDisposable
    {
        private readonly bool cleanup = HeatmapResult.CleanupDisplay;

        public void Dispose() => HeatmapResult.CleanupDisplay = cleanup;

        private const BindingFlags Private = BindingFlags.NonPublic | BindingFlags.Static;

        private static double LerpFinite(double a, double b, double f) =>
            (double)typeof(HeatmapResult).GetMethod("LerpFinite", Private).Invoke(null, new object[] { a, b, f });

        private static double LerpPositive(double a, double b, double f) =>
            (double)typeof(HeatmapResult).GetMethod("LerpPositive", Private).Invoke(null, new object[] { a, b, f });

        private static HeatmapResult.StabilityCell Stab(int mask) => new HeatmapResult.StabilityCell { PerfMask = mask };

        /// <summary>A 1 x 1 map swept at two masses, 1000 kg (dry) and 2000 kg (wet).</summary>
        private static HeatmapResult TwoMasses(Cell dry, Cell wet) =>
            new HeatmapResult
            {
                MachAxis = new[] { 0.8 },
                AltAxis = new[] { 5000.0 },
                EnvelopeSamples = new[] { Cells.Row(dry), Cells.Row(wet) },
                SampleMassesKg = new[] { 1000.0, 2000.0 },
                DryMassKg = 1000,
                Envelope = new Cell[1, 1]
            };

        [Fact]
        public void PerfMaskWithoutResultsIsANoOp()
        {
            HeatmapResult.ApplyStabilityPerfMask(null);
            HeatmapResult.ApplyStabilityPerfMask(new List<HeatmapResult>());
            var noStab = new HeatmapResult { Envelope = Cells.Row(Cells.Cruise(1)) };
            HeatmapResult.ApplyStabilityPerfMask(new List<HeatmapResult> { noStab });
            Assert.Null(noStab.Stability);
        }

        [Fact]
        public void PerfMaskSkipsAGridOfAnotherSize()
        {
            var r = new HeatmapResult
            {
                Stability = new[,] { { Stab(7) }, { Stab(7) } },
                Envelope = Cells.Row(Cells.Black(0.5), Cells.Black(0.6), Cells.Black(0.7))
            };
            HeatmapResult.ApplyStabilityPerfMask(new List<HeatmapResult> { r });
            Assert.Equal(7, r.Stability[0, 0].PerfMask);
            Assert.Equal(7, r.Stability[1, 0].PerfMask);

            r.Envelope = new Cell[2, 2];   // same width, other height
            HeatmapResult.ApplyStabilityPerfMask(new List<HeatmapResult> { r });
            Assert.Equal(7, r.Stability[0, 0].PerfMask);
        }

        /// <summary>The first result's stability grid is masked by the last result's (the combined map's) envelope:
        /// black (no trim, or suppressed) 1, grey (thrust-short or heat wall) 2, flyable 0.</summary>
        [Fact]
        public void PerfMaskFollowsTheCombinedMap()
        {
            var stab = new[,] { { Stab(9) }, { Stab(9) }, { Stab(9) }, { Stab(9) }, { Stab(9) }, { null } };
            Cell suppressed = Cells.Cruise(1.0);
            suppressed.Suppressed = true;
            suppressed.Feasible = false;
            var first = new HeatmapResult
            {
                Stability = stab,
                Envelope = Cells.Row(Cells.Cruise(0.5), Cells.Cruise(0.6), Cells.Cruise(0.7), Cells.Cruise(0.8), Cells.Cruise(0.9),
                                     Cells.Cruise(1.0))
            };
            var combined = new HeatmapResult
            {
                Envelope = Cells.Row(Cells.Black(0.5), Cells.Grey(0.6), Cells.Heat(0.7), Cells.Cruise(0.8), suppressed, Cells.Black(1.0))
            };
            HeatmapResult.ApplyStabilityPerfMask(new List<HeatmapResult> { first, combined });
            Assert.Equal(new[] { 1, 2, 2, 0, 1 }, new[] { stab[0, 0].PerfMask, stab[1, 0].PerfMask, stab[2, 0].PerfMask, stab[3, 0].PerfMask, stab[4, 0].PerfMask });
            Assert.Null(stab[5, 0]);
        }

        [Fact]
        public void DisplaySampleWithoutSamplesIsANoOp()
        {
            Cell[,] shown = Cells.Row(Cells.Cruise(0.8));
            var r = new HeatmapResult { MachAxis = new[] { 0.8 }, AltAxis = new[] { 0.0 }, Envelope = shown, MassKg = 1234 };
            r.SetDisplaySample(0);
            Assert.Same(shown, r.Envelope);
            r.EnvelopeSamples = new Cell[0][,];
            r.SetDisplaySample(0);
            Assert.Same(shown, r.Envelope);
            Assert.Equal(1234, r.MassKg);
            Assert.True(double.IsNaN(r.CeilingM));   // not recomputed either
        }

        [Fact]
        public void DisplaySampleIndexIsClampedToTheSamples()
        {
            HeatmapResult.CleanupDisplay = false;
            Cell[][,] samples = { Cells.Row(Cells.Cruise(0.8, alpha: 2)), Cells.Row(Cells.Cruise(0.8, alpha: 3)), Cells.Row(Cells.Cruise(0.8, alpha: 4)) };
            var r = new HeatmapResult
            {
                MachAxis = new[] { 0.8 },
                AltAxis = new[] { 3000.0 },
                EnvelopeSamples = samples,
                SampleMassesKg = new[] { 1000.0, 1500.0, 2000.0 }
            };

            r.SetDisplaySample(-3);
            Assert.Same(samples[0], r.Envelope);
            Assert.Equal(1000, r.MassKg);
            Assert.Equal(2, r.MaxAlphaDeg);   // the summary recomputed for it
            Assert.Equal(3000, r.CeilingM);

            r.SetDisplaySample(99);
            Assert.Same(samples[2], r.Envelope);
            Assert.Equal(2000, r.MassKg);
            Assert.Equal(4, r.MaxAlphaDeg);
        }

        /// <summary>With cleanup on the displayed grid is a cleaned copy; the raw sample keeps its isolated speck.</summary>
        [Fact]
        public void DisplaySampleCleansACopy()
        {
            HeatmapResult.CleanupDisplay = true;
            Cell[,] raw = Cells.Row(Cells.Cruise(0.5), Cells.Black(0.6), Cells.Grey(0.7));
            var r = new HeatmapResult
            {
                MachAxis = new[] { 0.5, 0.6, 0.7 },
                AltAxis = new[] { 0.0 },
                EnvelopeSamples = new[] { raw },
                SampleMassesKg = new[] { 1000.0 }
            };
            r.SetDisplaySample(0);
            Assert.NotSame(raw, r.Envelope);
            Assert.True(r.Envelope[2, 0].Suppressed);
            Assert.False(raw[2, 0].Suppressed);
        }

        [Fact]
        public void DisplayMassWithoutMassSamplesIsANoOp()
        {
            var r = new HeatmapResult { MachAxis = new[] { 0.8 }, AltAxis = new[] { 0.0 } };
            r.SetDisplayMass(1500);
            Assert.Null(r.Envelope);

            r.EnvelopeSamples = new[] { Cells.Row(Cells.Cruise(0.8)) };   // one mass: nothing to interpolate
            r.SampleMassesKg = new[] { 1000.0 };
            r.SetDisplayMass(1500);
            Assert.Null(r.Envelope);
            Assert.Equal(0, r.MassKg);
        }

        [Fact]
        public void DisplayMassInterpolatesIntoItsOwnBuffer()
        {
            HeatmapResult.CleanupDisplay = false;
            HeatmapResult r = TwoMasses(Cells.Cruise(0.8, alpha: 2), Cells.Cruise(0.8, alpha: 4));
            r.SetDisplayMass(1500);
            Assert.NotSame(r.EnvelopeSamples[0], r.Envelope);
            Assert.NotSame(r.EnvelopeSamples[1], r.Envelope);
            Assert.Equal(3, r.Envelope[0, 0].Alpha, 12);
            Assert.Equal(1500, r.MassKg);
            Assert.Equal(2, r.EnvelopeSamples[0][0, 0].Alpha);   // samples untouched
        }

        [Fact]
        public void FillForMassWithoutSamplesIsANoOp()
        {
            Cell[,] shown = Cells.Row(Cells.Cruise(0.8, alpha: 7));
            var r = new HeatmapResult { MachAxis = new[] { 0.8 }, AltAxis = new[] { 0.0 }, Envelope = shown, MassKg = 1 };
            r.FillForMass(1500);
            Assert.Equal(7, shown[0, 0].Alpha);
            Assert.Equal(1, r.MassKg);

            HeatmapResult noBuffer = TwoMasses(Cells.Cruise(0.8), Cells.Cruise(0.8));
            noBuffer.Envelope = null;
            noBuffer.FillForMass(1500);
            Assert.Null(noBuffer.Envelope);
        }

        /// <summary>Endurance is re-derived from the blended aerodynamics against the mass clamped to the swept
        /// dry..wet range: 0 at (or below) dry, Breguet from the wet mass above it.</summary>
        [Fact]
        public void FillForMassClampsTheMassToTheSamples()
        {
            HeatmapResult.CleanupDisplay = false;
            HeatmapResult r = TwoMasses(Cells.Cruise(0.8, alpha: 2, dragN: 1000), Cells.Cruise(0.8, alpha: 4, dragN: 3000));

            r.FillForMass(500);
            Cell c = r.Envelope[0, 0];
            Assert.True(c.Feasible);
            Assert.Equal(2, c.Alpha, 12);
            Assert.Equal(0, c.EnduranceHours);
            Assert.Equal(500, r.MassKg);   // the live mass, as given

            r.FillForMass(5000);
            c = r.Envelope[0, 0];
            Assert.Equal(4, c.Alpha, 12);
            Assert.Equal(PerformanceEnvelopeCalculator.EnduranceHours(3000, 0.2, 10, 2000, 1000), c.EnduranceHours, 12);

            r.FillForMass(1500);
            c = r.Envelope[0, 0];
            Assert.Equal(3, c.Alpha, 12);
            Assert.Equal(2000, c.DragN, 12);
            Assert.Equal(PerformanceEnvelopeCalculator.EnduranceHours(2000, 0.2, 10, 1500, 1000), c.EnduranceHours, 12);
            Assert.Equal(Cruise.Ok, c.Cruise);
        }

        /// <summary>
        /// A feasible dry sample and a thrust-short wet one are not blended: the cell is one of the two, switching where
        /// the excess thrust (linear in the mass fraction) crosses zero. Excess +1000 N dry, -3000 N wet: at f = 0.25.
        /// </summary>
        [Fact]
        public void FeasibleAndThrustShortSwitchAtTheExcessThrustZero()
        {
            HeatmapResult.CleanupDisplay = false;
            HeatmapResult r = TwoMasses(Cells.Cruise(0.8, excessN: 1000, alpha: 2), Cells.Grey(0.8, excessN: -3000));

            r.FillForMass(1240);
            Assert.True(r.Envelope[0, 0].Feasible);
            Assert.Equal(1000, r.Envelope[0, 0].ExcessThrustN);   // the dry cell whole, not blended
            Assert.Equal(1.5, r.Envelope[0, 0].EnduranceHours);
            Assert.Equal(2, r.Envelope[0, 0].Alpha);

            r.FillForMass(1260);
            Assert.False(r.Envelope[0, 0].Feasible);
            Assert.True(r.Envelope[0, 0].Trimmed);
            Assert.Equal(-3000, r.Envelope[0, 0].ExcessThrustN);
        }

        [Theory]
        [InlineData(-1000, -500, 1999, -1000)]   // zero crossing past the wet sample (f0 2 -> 1): dry cell until the wet mass
        [InlineData(-1000, -500, 2000, -500)]
        [InlineData(1000, 3000, 1000, 3000)]     // crossing before the dry sample (f0 -0.5 -> 0): the wet cell throughout
        [InlineData(-800, -800.0000000001, 1490, -800)]   // no slope to speak of: halfway
        [InlineData(-800, -800.0000000001, 1510, -800.0000000001)]
        public void TwoInfeasibleSamplesSwitchAtTheClampedCrossing(double dryExcess, double wetExcess, double mass, double excess)
        {
            HeatmapResult.CleanupDisplay = false;
            HeatmapResult r = TwoMasses(Cells.Grey(0.8, dryExcess), Cells.Grey(0.8, wetExcess));
            r.FillForMass(mass);
            Assert.Equal(excess, r.Envelope[0, 0].ExcessThrustN);
            Assert.False(r.Envelope[0, 0].Feasible);
        }

        /// <summary>An untrimmed sample has no excess thrust to cross: the cell snaps at the midpoint.</summary>
        [Fact]
        public void StallSampleSwitchesAtTheMidpoint()
        {
            HeatmapResult.CleanupDisplay = false;
            HeatmapResult r = TwoMasses(Cells.Black(0.8), Cells.Cruise(0.8));
            r.FillForMass(1490);
            Assert.False(r.Envelope[0, 0].Trimmed);
            r.FillForMass(1510);
            Assert.True(r.Envelope[0, 0].Feasible);
        }

        /// <summary>Two feasible samples whose blended excess thrust is negative make a wall, not a cruise; a heat
        /// limit on either sample makes it a heat wall.</summary>
        [Theory]
        [InlineData(false, false)]
        [InlineData(true, true)]
        public void BlendedNegativeExcessIsInfeasible(bool wetOverheats, bool overheats)
        {
            HeatmapResult.CleanupDisplay = false;
            Cell wet = Cells.Cruise(0.8, excessN: -3000);
            wet.Overheats = wetOverheats;
            HeatmapResult r = TwoMasses(Cells.Cruise(0.8, excessN: 1000), wet);
            r.FillForMass(1500);
            Cell c = r.Envelope[0, 0];
            Assert.Equal(-1000, c.ExcessThrustN, 9);
            Assert.True(c.Trimmed);
            Assert.False(c.Feasible);
            Assert.Equal(overheats, c.Overheats);
            Assert.True(double.IsNaN(c.EnduranceHours));
            Assert.True(double.IsNaN(r.CeilingM));   // nothing feasible left
        }

        [Theory]
        [InlineData(2, 4, 0.25, 2.5)]
        [InlineData(2, double.NaN, 0.25, 2)]
        [InlineData(double.PositiveInfinity, 4, 0.25, 4)]
        [InlineData(double.NaN, double.NegativeInfinity, 0.25, double.NaN)]
        [InlineData(-2, 0, 0.5, -1)]   // finite non-positive values blend
        public void LerpFiniteDropsTheMissingSide(double a, double b, double f, double expected) =>
            Assert.Equal(expected, LerpFinite(a, b, f), 12);

        [Theory]
        [InlineData(2, 4, 0.25, 2.5)]
        [InlineData(2, 0, 0.75, 2)]         // a zero is a sample with no cruise, not a real zero
        [InlineData(-1, 4, 0.25, 4)]
        [InlineData(double.NaN, 4, 0.25, 4)]
        [InlineData(2, double.PositiveInfinity, 0.25, 2)]
        [InlineData(0, -3, 0.5, double.NaN)]
        [InlineData(double.NaN, double.NaN, 0.5, double.NaN)]
        public void LerpPositiveDropsTheMissingSide(double a, double b, double f, double expected) =>
            Assert.Equal(expected, LerpPositive(a, b, f), 12);

        /// <summary>A lone grey speck between two cruise cells is healed to their midpoint, keeping its own Mach and
        /// speed and taking the render tints (heat-limited, reduced engines) of the nearer endpoint (ties: the faster).</summary>
        [Fact]
        public void LoneGreySpeckInsideTheEnvelopeIsHealed()
        {
            Cell slow = Cells.Cruise(0.5, excessN: 400, alpha: 2, dragN: 1000);
            slow.Overheats = true;
            Cell fast = Cells.Cruise(0.7, excessN: 800, alpha: 4, dragN: 3000);
            fast.ReducedEngines = true;
            fast.EnduranceHours = 2.5;
            Cell[,] grid = Cells.Row(slow, Cells.Grey(0.6), fast);

            HeatmapResult.CleanGrid(grid, 3, 1);

            Cell c = grid[1, 0];
            Assert.True(c.Trimmed && c.Feasible && !c.Suppressed);
            Assert.Equal(Cruise.Ok, c.Cruise);
            Assert.Equal(0.6, c.Mach);
            Assert.Equal(180, c.SpeedMPerS, 9);
            Assert.Equal(3, c.Alpha, 12);
            Assert.Equal(2000, c.DragN, 12);
            Assert.Equal(600, c.ExcessThrustN, 12);
            Assert.Equal(2, c.EnduranceHours, 12);
            Assert.False(c.Overheats);
            Assert.True(c.ReducedEngines);
            // the endpoints are left as they were
            Assert.True(grid[0, 0].Overheats);
            Assert.Equal(0.7, grid[2, 0].Mach);
        }

        /// <summary>Two infeasible cells in a row may be a real pinch (transonic drag rise): not healed.</summary>
        [Fact]
        public void TwoCellGapIsLeftAlone()
        {
            Cell[,] grid = Cells.Row(Cells.Cruise(0.5), Cells.Grey(0.6), Cells.Grey(0.7), Cells.Cruise(0.8));
            HeatmapResult.CleanGrid(grid, 4, 1);
            for (int i = 1; i <= 2; i++)
            {
                Assert.False(grid[i, 0].Feasible);
                Assert.False(grid[i, 0].Suppressed);   // next to the envelope: not a stranded speck either
                Assert.Equal(-1000, grid[i, 0].ExcessThrustN);
            }
            Assert.True(grid[0, 0].Feasible && grid[3, 0].Feasible);
        }

        /// <summary>Without a feasible cell there is no envelope to keep: the trimmed cells stay as swept.</summary>
        [Fact]
        public void NoFeasibleCellsLeavesTheGridAlone()
        {
            Cell[,] grid = Cells.Row(Cells.Grey(0.4), Cells.Heat(0.5), Cells.Heat(0.6), Cells.Black(0.7), Cells.Heat(0.8), Cells.Heat(0.9));
            HeatmapResult.CleanGrid(grid, 6, 1);
            for (int i = 0; i < 6; i++)
                Assert.False(grid[i, 0].Suppressed, $"cell {i}");
            Assert.True(grid[0, 0].Trimmed);
        }

        /// <summary>
        /// Only the group with the most feasible cells is the envelope, whatever its size: a detached feasible island
        /// (heat-limited or not) is suppressed but keeps its measurements, a detached heat wall stays visible.
        /// </summary>
        [Fact]
        public void DetachedIslandsAreSuppressedButNotTheHeatWall()
        {
            Cell heatLimited = Cells.Cruise(1.4);
            heatLimited.Overheats = true;
            Cell[,] grid = Cells.Row(Cells.Cruise(0.4), Cells.Cruise(0.5), Cells.Black(0.6), Cells.Cruise(0.7, dragN: 4321),
                                     Cells.Black(0.8), Cells.Heat(0.9), Cells.Heat(1.0), Cells.Heat(1.1), Cells.Heat(1.2),
                                     Cells.Cruise(1.3), Cells.Black(1.35), heatLimited);
            HeatmapResult.CleanGrid(grid, 12, 1);

            Assert.True(grid[0, 0].Feasible && !grid[0, 0].Suppressed);
            Assert.True(grid[1, 0].Feasible && !grid[1, 0].Suppressed);

            Cell island = grid[3, 0];
            Assert.True(island.Suppressed);
            Assert.False(island.Feasible);
            Assert.True(island.Trimmed);
            Assert.Equal(4321, island.DragN);

            // the heat wall group (5..9) outweighs the envelope in size but has one feasible cell to its two
            for (int i = 5; i <= 8; i++)
                Assert.False(grid[i, 0].Suppressed, $"heat wall cell {i}");
            Assert.True(grid[9, 0].Suppressed);
            Assert.False(grid[9, 0].Feasible);
            Assert.True(grid[11, 0].Suppressed);
            Assert.False(grid[11, 0].Feasible);
        }

        /// <summary>The envelope connects across altitude rows too (4-neighbour).</summary>
        [Fact]
        public void EnvelopeConnectsAcrossRows()
        {
            var grid = new Cell[3, 2];
            grid[0, 0] = Cells.Cruise(0.5);
            grid[1, 0] = Cells.Black(0.6);
            grid[2, 0] = Cells.Cruise(0.7);
            grid[0, 1] = Cells.Cruise(0.5);
            grid[1, 1] = Cells.Cruise(0.6);
            grid[2, 1] = Cells.Cruise(0.7);
            HeatmapResult.CleanGrid(grid, 3, 2);
            foreach (Cell c in grid)
                if (c.Trimmed)
                    Assert.True(c.Feasible && !c.Suppressed);

            grid[1, 1] = Cells.Black(0.6);   // now [2, 1] is cut off, and the smaller group
            grid[2, 0] = Cells.Black(0.7);
            grid[2, 1] = Cells.Cruise(0.7);
            HeatmapResult.CleanGrid(grid, 3, 2);
            Assert.True(grid[0, 0].Feasible && grid[0, 1].Feasible);
            Assert.True(grid[2, 1].Suppressed);
            Assert.False(grid[2, 1].Feasible);
        }
    }
}
