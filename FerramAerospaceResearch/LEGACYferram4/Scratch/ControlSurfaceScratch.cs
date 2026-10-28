/*
Ferram Aerospace Research
=========================
Per-call mutable state extracted from FARControllableSurface for thread safety.

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
    /// Per-call mutable state of <see cref="FARControllableSurface"/>: the control-surface
    /// specific scratch fields (the base wing state is the wing's own <see cref="WingScratch"/>).
    /// One instance is allocated per (slot, control surface).
    ///
    /// The original <see cref="FARControllableSurface"/> animates
    /// <c>movableSection.localRotation</c> (a Unity Transform) inside
    /// <c>DeflectionAnimation</c>. The parallel path uses
    /// <see cref="ControlSurfaceScratch.deflectedNormal"/> directly without touching the
    /// Transform - the animation is purely cosmetic and unsafe to run off the main
    /// thread.
    /// </summary>
    public sealed class ControlSurfaceScratch
    {
        // Blended deflection state (linear/exponential ramps in flight; forced to
        // desired value in editor sim via SetControlStateEditor).
        public double AoAcurrentControl;
        public double AoAcurrentFlap;
        public double AoAoffset;
        public double AoAdesiredControl;
        public double AoAdesiredFlap;

        // Deflected surface normal, used by FARControllableSurface.CalculateAoA in place
        // of part_transform.forward. Vector3d and initialised to forward to match the
        // instance-field default it backs.
        public Vector3d deflectedNormal = Vector3d.forward;

        // Cache of previous AoAoffset to skip Transform animation on tiny changes
        public double lastAoAoffset;

        // Control-effectiveness factors recomputed from part geometry each call
        public double PitchLocation;
        public double YawLocation;
        public double RollLocation;
        public double BrakeRudderLocation;
        public double BrakeRudderSide;
        public double AoAsign = 1.0;

        // Flap/spoiler control mapping
        public int flapLocation;
        public int spoilerLocation;

        // Shielded state: CheckShielded rewrites it from the deflection every trim step, and the solve
        // reads it (SetState, wing interaction) to decide whether to skip the surface. It must be
        // per-thread on the parallel path or those reads race. Seeded from the instance value.
        public bool IsShielded;

        /// <summary>The deflection, control mapping and shielding of another slot (the flight's), for a prediction.</summary>
        public void CopyControlStateFrom(ControlSurfaceScratch src)
        {
            AoAcurrentControl = src.AoAcurrentControl;
            AoAcurrentFlap = src.AoAcurrentFlap;
            AoAoffset = src.AoAoffset;
            AoAdesiredControl = src.AoAdesiredControl;
            AoAdesiredFlap = src.AoAdesiredFlap;
            deflectedNormal = src.deflectedNormal;
            lastAoAoffset = src.lastAoAoffset;
            PitchLocation = src.PitchLocation;
            YawLocation = src.YawLocation;
            RollLocation = src.RollLocation;
            BrakeRudderLocation = src.BrakeRudderLocation;
            BrakeRudderSide = src.BrakeRudderSide;
            AoAsign = src.AoAsign;
            flapLocation = src.flapLocation;
            spoilerLocation = src.spoilerLocation;
            IsShielded = src.IsShielded;
        }
    }
}
