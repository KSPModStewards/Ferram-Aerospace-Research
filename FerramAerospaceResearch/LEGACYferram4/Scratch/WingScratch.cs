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

        // Drag split for the editor prediction-check diagnostic: CdInduced (lift-dependent, Cl^2/piARe)
        // + CdProfile (zero-lift + skin friction) is the pre-stall Cd. Captured before the stall/flat-
        // plate adjustments, so it is only meaningful in attached flow (i.e. cruise).
        public double CdInduced;
        public double CdProfile;

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

        // The wing's live state captured with a vessel snapshot, installed on the slot for one snapshot prediction:
        // valid while CaptureStamp is that prediction's stamp (SimThreadContext.SnapshotStamp).
        public long CaptureStamp;
        public FrozenPartTransform CapturedGeom;
        public double CapturedRbMass;
        public double CapturedFixedDeltaTime;
        public bool CapturedShielded;
        public double CapturedSubmergedPortion;
        public double CapturedSubmergedLiftScalar;
        public double CapturedSubmergedDragScalar;

        /// <summary>
        /// Every field back to the value a new scratch has, with the wing's geometric aspect ratio: the fixed state a
        /// flight prediction starts from, so its answer depends only on its inputs.
        /// </summary>
        public void Reset(double geometricTransformedAR)
        {
            velocityEditor = Vector3d.zero;
            rho = 0;
            Cl = Cd = 0;
            stall = minStall = 0;
            rawAoAmax = 15;
            AoAmax = 0;
            rawLiftSlope = liftslope = 0;
            cosSweepAngle = 0;
            effective_b_2 = 1;
            effective_MAC = 1;
            effective_AR = 4;
            transformed_AR = geometricTransformedAR;
            ClIncrementFromRear = 0;
            e = 0;
            piARe = 1;
            zeroLiftCdIncrement = 0;
            FinalLiftSlope = 0;
            ParallelInPlane = perp = liftDirection = ParallelInPlaneLocal = Vector3d.zero;
            AerodynamicCenter = CurWingCentroid = Vector3d.zero;
            CaptureStamp = 0;
        }
    }
}
