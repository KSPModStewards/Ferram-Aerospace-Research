/*
Ferram Aerospace Research
=========================
Per-call mutable state for FARAeroSection parallel evaluation.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using ferram4;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    /// <summary>
    /// Per-thread substitute for <see cref="FARAeroSection"/>'s instance
    /// <c>simContext</c> field. Holds the per-call inputs (freestream velocity, density)
    /// plus the <see cref="FARCenterQuery"/> the parallel evaluator accumulates forces
    /// into.
    ///
    /// Allocated once per worker thread (not per call) — <see cref="Center"/> is reset
    /// before each cell evaluation by the caller. One scratch is shared across all
    /// sections within a single per-thread evaluation since
    /// <see cref="FARAeroSection.CalculateAeroForces"/> only reads instance-immutable
    /// data and writes exclusively through the force context.
    /// </summary>
    public sealed class SectionScratch
    {
        public Vector3 WorldVel;
        public float AtmDensity;

        /// <summary>
        /// Accumulator passed in by the caller; force application calls
        /// <c>AddForce</c>/<c>AddTorque</c> on this. Reset by the caller before each
        /// evaluation.
        /// </summary>
        public FARCenterQuery Center = new FARCenterQuery();
    }
}
