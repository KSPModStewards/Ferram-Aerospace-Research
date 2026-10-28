/*
Ferram Aerospace Research
=========================
Per-call mutable state extracted from FARWingInteraction for thread safety.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

// ReSharper disable once CheckNamespace
namespace ferram4
{
    /// <summary>
    /// Per-call mutable state of <see cref="FARWingInteraction"/>.
    /// One instance is allocated per (thread x wing) on the parallel path.
    ///
    /// The neighbour-wing lists (<c>nearbyWingModulesForwardList</c> etc.) remain on the
    /// <see cref="FARWingInteraction"/> instance - they are populated by
    /// <c>UpdateWingInteraction</c> outside the per-call hot path and are read-only here.
    /// </summary>
    public sealed class WingInteractionScratch
    {
        // Set by UpdateOrientationForInteractionCore
        public double ARFactor = 1.0;
        public bool HasWingsUpstream;

        // Accumulated by CalculateEffectsOfUpstreamWingCore
        public double EffectiveUpstreamMAC;
        public double EffectiveUpstreamb_2;
        public double EffectiveUpstreamArea;
        public double EffectiveUpstreamLiftSlope;
        public double EffectiveUpstreamStall;
        public double EffectiveUpstreamCosSweepAngle;
        public double EffectiveUpstreamAoAMax;
        public double EffectiveUpstreamAoA;
        public double EffectiveUpstreamCd0;
        public double EffectiveUpstreamInfluence;
    }
}
