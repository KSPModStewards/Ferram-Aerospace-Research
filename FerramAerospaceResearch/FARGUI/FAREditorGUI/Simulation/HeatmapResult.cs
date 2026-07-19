/*
Ferram Aerospace Research
=========================
Completed heatmap sweep data, and the hand-off from the editor to the flight scene.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System.Collections.Generic;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    /// <summary>
    /// A finished (Mach × altitude) sweep, decoupled from the GUI that produced it so the flight
    /// scene can render one without re-running anything.
    ///
    /// Re-running in flight is not an option: the editor sim drives ComputeForceEditor on the very
    /// FARWingAerodynamicModel instances the flight path uses every physics frame, and they share a
    /// single WingScratch. Stall carries hysteresis across calls (DetermineStall reads the previous
    /// call's value), so a sweep would inject stall history from hypothetical conditions into the
    /// wings of the aircraft being flown. Computing in the editor and only displaying in flight
    /// sidesteps that entirely.
    /// </summary>
    internal class HeatmapResult
    {
        /// <summary>
        /// X-axis sample values. Mach numbers when <see cref="SpeedAxisMode" /> is false, true
        /// airspeeds in m/s when it is true. Per-cell Mach lives on the CellResult either way.
        /// </summary>
        public double[] MachAxis;
        public double[] AltAxis;

        /// <summary>True when the x-axis is true airspeed (m/s) rather than Mach.</summary>
        public bool SpeedAxisMode;

        /// <summary>The envelope currently displayed. In the editor this is the wet-mass sample; in
        /// flight it is <see cref="EnvelopeSamples" /> interpolated to the live mass.</summary>
        public PerformanceEnvelopeCalculator.CellResult[,] Envelope;

        /// <summary>
        /// The envelope swept at several masses, ascending from dry (empty) to wet (full), so the
        /// flight scene can interpolate it as fuel burns. <see cref="SampleMassesKg" /> holds the mass
        /// (kg) each grid was swept at. Null when only a single mass was swept.
        /// </summary>
        public PerformanceEnvelopeCalculator.CellResult[][,] EnvelopeSamples;
        public double[] SampleMassesKg;

        /// <summary>Empty (no usable fuel) mass, kg — the m1 in the Breguet endurance recompute.</summary>
        public double DryMassKg;

        public bool HasMassSamples => EnvelopeSamples != null && SampleMassesKg != null && SampleMassesKg.Length > 1;
        public StabilityCell[,] Stability;
        public bool HasStability;

        /// <summary>
        /// Which engine kind this envelope was swept for (e.g. "Turbojet/fan", "Ramjet"). One
        /// HeatmapResult is produced per engine type on the craft; empty when the craft has a single
        /// type. Stability, axes and vehicle identity are shared across a craft's set.
        /// </summary>
        public string EngineTypeLabel = "";

        /// <summary>Best endurance in the sweep; the endurance colour ramp is relative to it.</summary>
        public double MaxEnduranceHours;

        /// <summary>Best range in the sweep; the range colour ramp is relative to it.</summary>
        public double MaxRangeKm;

        public double CeilingM = double.NaN;
        public double BestEnduranceHours = double.NaN;
        public double BestRangeKm = double.NaN;

        /// <summary>Body the sweep was run against, for the flight scene to sanity-check.</summary>
        public string BodyName;

        /// <summary>Craft name at sweep time — see <see cref="Matches" />.</summary>
        public string CraftName;

        /// <summary>Physical part count at sweep time — see <see cref="Matches" />.</summary>
        public int PartCount;

        /// <summary>Vehicle mass the sweep assumed, kg. The map is only valid at this mass.</summary>
        public double MassKg;

        /// <summary>
        /// Reference area the sweep normalised against, m^2. Logged so it can be checked against the
        /// flight scene's refArea: forces are Cd * q * S, so if this disagrees with the area the
        /// coefficients were actually normalised by, every force is wrong by exactly that ratio.
        /// </summary>
        public double RefAreaM2;

        public class StabilityCell
        {
            public int Score;
            public readonly List<(string name, double value, int expectedSign)> OutOfRange =
                new List<(string name, double value, int expectedSign)>();
        }

        /// <summary>
        /// A display copy for the flight scene: shares the axes, stability grid, mass samples and
        /// identity with this result, but gets its own <see cref="Envelope" /> buffer so interpolating
        /// to the live mass does not overwrite the cached editor grid (whose Envelope is the wet sample).
        /// </summary>
        public HeatmapResult FlightView()
        {
            return new HeatmapResult
            {
                MachAxis = MachAxis,
                AltAxis = AltAxis,
                SpeedAxisMode = SpeedAxisMode,
                Stability = Stability,
                HasStability = HasStability,
                EnvelopeSamples = EnvelopeSamples,
                SampleMassesKg = SampleMassesKg,
                DryMassKg = DryMassKg,
                EngineTypeLabel = EngineTypeLabel,
                BodyName = BodyName,
                CraftName = CraftName,
                PartCount = PartCount,
                MassKg = MassKg,
                RefAreaM2 = RefAreaM2,
                Envelope = new PerformanceEnvelopeCalculator.CellResult[MachAxis.Length, AltAxis.Length]
            };
        }

        /// <summary>
        /// Recomputes the ceiling and the best/max endurance and range from the current
        /// <see cref="Envelope" />. The colour ramps scale to the max values, so this must run whenever
        /// the displayed grid changes (after a sweep, or after interpolating to a new mass).
        /// </summary>
        public void Recompute()
        {
            CeilingM = double.NaN;
            BestEnduranceHours = double.NaN;
            BestRangeKm = double.NaN;
            MaxEnduranceHours = 0;
            MaxRangeKm = 0;

            for (int j = 0; j < AltAxis.Length; j++)
                for (int i = 0; i < MachAxis.Length; i++)
                {
                    PerformanceEnvelopeCalculator.CellResult c = Envelope[i, j];
                    if (!c.Feasible)
                        continue;

                    if (double.IsNaN(CeilingM) || AltAxis[j] > CeilingM)
                        CeilingM = AltAxis[j];

                    if (!Finite(c.EnduranceHours))
                        continue;

                    if (c.EnduranceHours > MaxEnduranceHours)
                        MaxEnduranceHours = c.EnduranceHours;
                    if (double.IsNaN(BestEnduranceHours) || c.EnduranceHours > BestEnduranceHours)
                        BestEnduranceHours = c.EnduranceHours;

                    double range = PerformanceEnvelopeCalculator.RangeKm(c);
                    if (!Finite(range))
                        continue;
                    if (range > MaxRangeKm)
                        MaxRangeKm = range;
                    if (double.IsNaN(BestRangeKm) || range > BestRangeKm)
                        BestRangeKm = range;
                }
        }

        /// <summary>
        /// Fills <see cref="Envelope" /> by interpolating the mass samples to <paramref name="massKg" />,
        /// then recomputes the summary. Continuous quantities (drag, excess thrust, thermal, fuel flow)
        /// are blended and feasibility/endurance are re-derived from them against the live mass — so the
        /// ceiling and top-speed edges expand correctly as fuel burns. A no-op without mass samples.
        /// </summary>
        public void FillForMass(double massKg)
        {
            if (!HasMassSamples || Envelope == null)
                return;

            int n = SampleMassesKg.Length;
            double m = massKg;
            if (m < SampleMassesKg[0])
                m = SampleMassesKg[0];
            if (m > SampleMassesKg[n - 1])
                m = SampleMassesKg[n - 1];

            int hi = 1;
            while (hi < n - 1 && SampleMassesKg[hi] < m)
                hi++;
            int lo = hi - 1;

            double denom = SampleMassesKg[hi] - SampleMassesKg[lo];
            double f = denom > 0 ? (m - SampleMassesKg[lo]) / denom : 0;

            PerformanceEnvelopeCalculator.CellResult[,] a = EnvelopeSamples[lo];
            PerformanceEnvelopeCalculator.CellResult[,] b = EnvelopeSamples[hi];

            // Endurance uses the clamped mass m, not the raw live mass: the map is only valid across
            // the swept dry↔wet range, and letting ln(mass/dry) run past the wet sample would inflate
            // endurance without bound (flight-at-full must reproduce the editor's wet map exactly).
            for (int j = 0; j < AltAxis.Length; j++)
                for (int i = 0; i < MachAxis.Length; i++)
                    Envelope[i, j] = InterpCell(a[i, j], b[i, j], f, m);

            MassKg = massKg;
            Recompute();
        }

        private PerformanceEnvelopeCalculator.CellResult InterpCell(
            PerformanceEnvelopeCalculator.CellResult a,
            PerformanceEnvelopeCalculator.CellResult b,
            double f,
            double massKg
        )
        {
            // Across a trim boundary (one sample stalls, the other flies) the untrimmed sample carries
            // zero drag, so blending would corrupt the flying one — snap to the nearer sample instead.
            if (!(a.Trimmed && b.Trimmed))
                return f < 0.5 ? a : b;

            var c = new PerformanceEnvelopeCalculator.CellResult
            {
                Mach = a.Mach,
                SpeedMPerS = a.SpeedMPerS,
                Trimmed = true,
                // Inlet and thermo bookkeeping is a function of the flight condition, not the mass, so
                // it is the same across samples — carry it through for the flight prediction-check log.
                InletArea = a.InletArea,
                EngineArea = a.EngineArea,
                AreaRatio = a.AreaRatio,
                OverallTpr = a.OverallTpr,
                AmbientP = a.AmbientP,
                AmbientT = a.AmbientT,
                InletP = a.InletP,
                InletT = a.InletT,
                Alpha = Lerp(a.Alpha, b.Alpha, f),
                DragN = Lerp(a.DragN, b.DragN, f),
                WingDragN = Lerp(a.WingDragN, b.WingDragN, f),
                BodyDragN = Lerp(a.BodyDragN, b.BodyDragN, f),
                RamDragN = Lerp(a.RamDragN, b.RamDragN, f),
                WingInducedDragN = Lerp(a.WingInducedDragN, b.WingInducedDragN, f),
                WingProfileDragN = Lerp(a.WingProfileDragN, b.WingProfileDragN, f),
                WingLiftN = Lerp(a.WingLiftN, b.WingLiftN, f),
                BodyLiftN = Lerp(a.BodyLiftN, b.BodyLiftN, f),
                CmCruise = Lerp(a.CmCruise, b.CmCruise, f),
                CruisePitch = Lerp(a.CruisePitch, b.CruisePitch, f),
                LiftToDrag = Lerp(a.LiftToDrag, b.LiftToDrag, f),
                ThrustAvailableN = Lerp(a.ThrustAvailableN, b.ThrustAvailableN, f),
                ExcessThrustN = Lerp(a.ExcessThrustN, b.ExcessThrustN, f),
                ThermalRatio = Lerp(a.ThermalRatio, b.ThermalRatio, f),
                ThrottleRequired = LerpFinite(a.ThrottleRequired, b.ThrottleRequired, f),
                // Fuel flow of 0 means the sample never solved a cruise (infeasible/overheating), so it
                // is missing data, not a real zero — blending toward it would inflate Isp and endurance.
                FuelFlowKgPerS = LerpPositive(a.FuelFlowKgPerS, b.FuelFlowKgPerS, f),
                ReducedEngines = a.ReducedEngines || b.ReducedEngines,
                EnduranceHours = double.NaN
            };

            if (c.ExcessThrustN < 0)
                return c; // thrust-limited

            if (c.ThermalRatio >= EditorEngineDeck.ThermalLimit)
            {
                c.Overheats = true;
                return c;
            }

            c.Feasible = true;
            c.Cruise = PerformanceEnvelopeCalculator.CruiseState.Ok;
            // At or below the swept dry mass the tank is empty, so range/endurance is 0, not the NaN
            // that ln(mass/dry) = ln(1) produces. FillForMass clamps live mass to the dry sample, so
            // once the aircraft burns down to it the whole map would otherwise read NaN.
            c.EnduranceHours = massKg <= DryMassKg
                                   ? 0
                                   : PerformanceEnvelopeCalculator.EnduranceHours(c.DragN,
                                                                                  c.FuelFlowKgPerS,
                                                                                  c.LiftToDrag,
                                                                                  massKg,
                                                                                  DryMassKg);
            return c;
        }

        private static double Lerp(double a, double b, double f)
        {
            return a + (b - a) * f;
        }

        private static double LerpFinite(double a, double b, double f)
        {
            bool fa = Finite(a);
            bool fb = Finite(b);
            if (fa && fb)
                return a + (b - a) * f;
            if (fa)
                return a;
            if (fb)
                return b;
            return double.NaN;
        }

        // Like LerpFinite but treats non-positive operands as missing, so a zero (a sample that never
        // solved a cruise) does not drag the blend toward zero.
        private static double LerpPositive(double a, double b, double f)
        {
            bool fa = Finite(a) && a > 0;
            bool fb = Finite(b) && b > 0;
            if (fa && fb)
                return a + (b - a) * f;
            if (fa)
                return a;
            if (fb)
                return b;
            return double.NaN;
        }

        private static bool Finite(double x)
        {
            return !double.IsNaN(x) && !double.IsInfinity(x);
        }

        /// <summary>
        /// Last sweep run this session — one entry per engine type, sharing stability/axes/identity.
        /// Deliberately not persisted: it lives only in memory, so it is gone on game restart and is
        /// not attached to any particular vessel. <see cref="Matches" /> exists because of that — the
        /// flight scene must not present a map for a different aircraft as though it belonged to the
        /// one being flown.
        /// </summary>
        public static List<HeatmapResult> Cached { get; set; }

        /// <summary>
        /// Whether this sweep plausibly describes <paramref name="vessel" />. A name and part-count
        /// check is weak, but it catches the common mistake — sweeping one craft and launching
        /// another — and a weak check that is reported honestly beats none.
        /// </summary>
        public bool Matches(Vessel vessel)
        {
            if (vessel == null)
                return false;

            int parts = 0;
            foreach (Part p in vessel.Parts)
                if (!FARAeroUtil.IsNonphysical(p))
                    parts++;

            return parts == PartCount && vessel.vesselName == CraftName;
        }
    }
}
