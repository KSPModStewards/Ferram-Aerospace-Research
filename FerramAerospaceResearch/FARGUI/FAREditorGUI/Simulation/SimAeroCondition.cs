/*
Ferram Aerospace Research
=========================
Real Reynolds / skin-friction conditions for an editor-side simulation point.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    /// <summary>
    /// The viscous flow parameters for a specific (altitude, Mach) being simulated.
    ///
    /// The editor sim evaluates forces with a unit velocity vector and density 2, so that
    /// q = 0.5 * 2 * 1^2 = 1 and the resulting forces read directly as coefficients. That trick is
    /// sound for pressure forces, but it destroys the Reynolds number — so both the wing model and
    /// the aero sections fall back to a hardcoded skin friction of 0.005 whenever they are not in
    /// flight.
    ///
    /// Measured against flight, that constant is roughly double the true coefficient at sea-level
    /// Reynolds numbers and below it at low ones, which shows up as drag being over-predicted by
    /// ~2x near the ground and under-predicted at altitude. Supplying the real values fixes that.
    ///
    /// This is opt-in: callers that pass nothing get the historical behaviour, so Static Analysis
    /// and the stability derivative tabs are unaffected.
    /// </summary>
    internal readonly struct SimAeroCondition
    {
        /// <summary>Reynolds number per unit length, 1/m.</summary>
        public readonly double ReynoldsPerLength;

        /// <summary>Skin friction coefficient at this condition.</summary>
        public readonly double SkinFriction;

        /// <summary>Rarefaction proxy used by the aero sections.</summary>
        public readonly double PseudoKnudsen;

        private SimAeroCondition(double reynoldsPerLength, double skinFriction, double pseudoKnudsen)
        {
            ReynoldsPerLength = reynoldsPerLength;
            SkinFriction = skinFriction;
            PseudoKnudsen = pseudoKnudsen;
        }

        /// <summary>
        /// Derives the viscous parameters for a flight condition. Mirrors what
        /// FARVesselAero.SimulateAeroProperties does before calling the same force routines, so the
        /// editor and flight see the same physics.
        /// </summary>
        /// <param name="body">Body whose atmosphere to sample.</param>
        /// <param name="alt">Altitude ASL, m.</param>
        /// <param name="mach">Free-stream Mach number.</param>
        /// <param name="lengthScale">Vehicle reference length, m.</param>
        public static SimAeroCondition ForFlightPoint(CelestialBody body, double alt, double mach, double lengthScale)
        {
            // Same atmosphere the rest of the sweep uses — Reynolds depends on density and the
            // speed of sound, both of which move with the solar temperature term.
            GasProperties gas = EditorAtmosphere.GetGasProperties(body, alt);

            double speed = mach * gas.SpeedOfSound;

            double reynolds = FARAeroUtil.CalculateReynoldsNumber(gas.Density,
                                                                  lengthScale,
                                                                  speed,
                                                                  mach,
                                                                  gas.Temperature,
                                                                  gas.AdiabaticIndex);

            double skinFriction = FARAeroUtil.SkinFrictionDrag(reynolds, mach);

            // A flight point with no meaningful atmosphere — near-zero density at or above the top of the
            // atmosphere, which the Static Analysis tab lets the user dial in — drives Reynolds to zero
            // and the skin-friction correlation to NaN/Inf. Fall back to the historical constant there so
            // the viscous path degrades to the original behaviour instead of poisoning the solve with a
            // NaN (which the graph and stability code cannot handle). (!(x > 0) also catches NaN.)
            if (!(gas.Density > 0) || !(reynolds > 0) || double.IsNaN(skinFriction) || double.IsInfinity(skinFriction))
            {
                reynolds = 0;
                skinFriction = 0.005;
            }

            // Matches FARVesselAero's formulation exactly rather than inventing one.
            double pseudoKnudsen = reynolds > 0 ? mach / (reynolds + mach) : 0;

            double reynoldsPerLength = lengthScale > 0 ? reynolds / lengthScale : 0;

            return new SimAeroCondition(reynoldsPerLength, skinFriction, pseudoKnudsen);
        }
    }
}
