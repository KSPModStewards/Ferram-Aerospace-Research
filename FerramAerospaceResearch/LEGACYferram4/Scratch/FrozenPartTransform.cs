/*
Ferram Aerospace Research
=========================
Thread-safe snapshot of a part's Transform for the parallel editor sweep.

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
    /// A plain-data copy of the geometry a part's <see cref="Transform"/> exposes, captured on the
    /// main thread so the editor aero solve can run on worker threads. Unity <c>Transform</c> is
    /// main-thread-only, but <c>Vector3</c>/<c>Quaternion</c>/<c>Matrix4x4</c> are pure value types
    /// whose math is thread-safe, so the sweep snapshots each part once (transforms do not move
    /// during a sweep) and routes its transform reads through this struct.
    ///
    /// Each method mirrors the exact Unity <c>Transform</c> semantics of the call it replaces:
    /// direction transforms use rotation only (no scale), point/vector transforms use the full
    /// matrices (scale included).
    /// </summary>
    public readonly struct FrozenPartTransform
    {
        private readonly Quaternion rotation;
        private readonly Matrix4x4 localToWorld;
        private readonly Matrix4x4 worldToLocal;

        public readonly Vector3 Position;

        public FrozenPartTransform(Transform t)
        {
            rotation = t.rotation;
            Position = t.position;
            localToWorld = t.localToWorldMatrix;
            worldToLocal = t.worldToLocalMatrix;
        }

        // Transform.forward/up/right are defined as rotation * the corresponding axis.
        public Vector3 Forward => rotation * Vector3.forward;
        public Vector3 Up => rotation * Vector3.up;
        public Vector3 Right => rotation * Vector3.right;

        // Direction transforms: rotation only, not affected by scale or position.
        public Vector3 TransformDirection(Vector3 d)
        {
            return rotation * d;
        }

        public Vector3 InverseTransformDirection(Vector3 d)
        {
            return Quaternion.Inverse(rotation) * d;
        }

        // Point transform: rotation + scale + translation.
        public Vector3 TransformPoint(Vector3 p)
        {
            return localToWorld.MultiplyPoint3x4(p);
        }

        // Vector transform: rotation + scale, no translation.
        public Vector3 InverseTransformVector(Vector3 v)
        {
            return worldToLocal.MultiplyVector(v);
        }
    }
}
