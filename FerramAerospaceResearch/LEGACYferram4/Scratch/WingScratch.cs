/*
Ferram Aerospace Research
=========================
Per-call mutable state extracted from FARWingAerodynamicModel for thread safety.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using UnityEngine;

// ReSharper disable once CheckNamespace
namespace ferram4
{
    /// <summary>
    /// Per-call mutable state of <see cref="FARWingAerodynamicModel"/> aerodynamic math.
    /// One instance is allocated per (thread x wing) on the parallel evaluation path; the
    /// single-threaded path uses one instance per wing stored in the wing module itself.
    ///
    /// Invariant: when a downstream wing reads upstream-wing state during
    /// <c>CalculateEffectsOfUpstreamWing</c>, it reads from the same per-thread
    /// <see cref="WingScratch"/> array indexed by wing position. The parallel sweep
    /// therefore must iterate wings sequentially per cell in <c>_wingAerodynamicModel</c>
    /// order, matching <c>InstantConditionSim.GetClCdCmSteady</c>.
    /// </summary>
    public sealed class WingScratch
    {
        // Inputs threaded through from caller
        public Vector3d velocityEditor;
        public double rho;

        // Lift/drag coefficient outputs (also persisted on instance as KSPField-observable)
        public double Cl;
        public double Cd;

        // Stall state (also KSPField-observable on instance)
        public double stall;
        public double minStall;

        // Stall-angle bounds. rawAoAmax defaults to 15 to match the wing module's
        // instance-field default, so a worker thread's freshly-created scratch sees the
        // same value before CalculateAoAmax first populates it.
        public double rawAoAmax = 15;
        public double AoAmax;

        // Lift slope (working copy; upstream interactions can modify)
        public double rawLiftSlope;
        public double liftslope;

        // Sweep angle (working copy; CalculateWingCamberInteractions overwrites this
        // with upstream-influenced value, SetSweepAngle re-derives it).
        public double cosSweepAngle;

        // Effective span/chord/aspect - recomputed by CalculateSubsonicLiftSlope
        // each call from the velocity's projection on the wing plane. These are
        // mutable working geometry, not static like b_2_actual / MAC_actual.
        // Initial values match the legacy instance-field defaults so the very
        // first call (before CalculateSubsonicLiftSlope populates them) sees the
        // same numbers - CalculateAerodynamicCenter reads effective_MAC before
        // CalculateCoefficients runs.
        public double effective_b_2 = 1;
        public double effective_MAC = 1;
        public double effective_AR = 4;
        public double transformed_AR = 4;

        // Camber/AC shift carry-over between camber-interaction and coefficient calc
        public double ClIncrementFromRear;

        // Oswald efficiency (also KSPField-observable on instance) and induced-drag factor.
        // piARe initial of 1 matches the legacy private-field default.
        public double e;
        public double piARe = 1;

        // Zero-lift Cd increment (compressibility). Read-modify-written across the
        // compressibility calc and read by downstream wings via GetCd0, so it must be
        // per-thread rather than a shared instance field.
        public double zeroLiftCdIncrement;

        // Final lift slope reported via property
        public double FinalLiftSlope;

        // Working geometry vectors (in world frame for the call)
        public Vector3d ParallelInPlane;
        public Vector3d perp;
        public Vector3d liftDirection;
        public Vector3d ParallelInPlaneLocal;

        // Aerodynamic centre (instance field is read by other parts after the call)
        public Vector3d AerodynamicCenter;

        // Working wing centroid in world space
        public Vector3d CurWingCentroid;
    }
}
