/*
Ferram Aerospace Research
=========================
Thread-safe exact evaluation of a KSP FloatCurve / Unity AnimationCurve.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using UnityEngine;

// ReSharper disable once CheckNamespace
namespace ferram4
{
    /// <summary>
    /// An immutable copy of a <see cref="FloatCurve"/> / <see cref="AnimationCurve"/> evaluated the way Unity's
    /// runtime evaluates it, so any thread can evaluate it. <c>AnimationCurve.Evaluate</c> keeps a segment cache in
    /// the curve object, so concurrent evaluation of one curve is not safe; this takes the keys once (on the thread
    /// that owns the curve) and repeats Unity's arithmetic without the shared cache:
    ///
    /// - outside the key range the curve clamps to the first or last value (ClampForever/Clamp/Once/Default);
    /// - inside, the segment [lhs, rhs] is a cubic in u = t - lhs.time with the coefficients Unity's segment cache
    ///   holds: dx = max(rhs.time - lhs.time, 1e-4), d1 = lhs.outTangent * dx, d2 = rhs.inTangent * dx,
    ///   dy = rhs.value - lhs.value, c0 = (d1 + d2 - dy - dy) / dx^3, c1 = (3 dy - 2 d1 - d2) / dx^2,
    ///   c2 = lhs.outTangent, c3 = lhs.value, evaluated as ((c0 u + c1) u + c2) u + c3, in single precision;
    /// - a segment with an infinite tangent (stepped) is the left key's value.
    ///
    /// Every intermediate is rounded to float as Unity's single-precision code does. Weighted tangents and the
    /// Loop/PingPong wrap modes are not reproduced exactly (<see cref="IsExact"/> is false for such a curve):
    /// weighted segments are evaluated as unweighted Hermite segments and wrapped times are evaluated at the wrapped
    /// time. None of FAR's curves or KSP's stock physics and atmosphere curves use either.
    /// </summary>
    public sealed class ExactCurve
    {
        private readonly float[] times;
        private readonly float[] values;
        private readonly float[] c0;
        private readonly float[] c1;
        private readonly float[] c2;
        private readonly WrapMode preWrap;
        private readonly WrapMode postWrap;

        // The source keys, to tell whether a curve still has them (Matches).
        private readonly Keyframe[] sourceKeys;

        /// <summary>False when the curve uses weighted tangents or a Loop/PingPong wrap mode.</summary>
        public bool IsExact { get; }

        public ExactCurve(FloatCurve curve) : this(curve.Curve)
        {
        }

        public ExactCurve(AnimationCurve curve)
        {
            Keyframe[] keys = curve.keys;
            sourceKeys = keys;
            int n = keys.Length;
            times = new float[n];
            values = new float[n];
            var inSlope = new float[n];
            var outSlope = new float[n];
            bool exact = true;
            for (int i = 0; i < n; i++)
            {
                times[i] = keys[i].time;
                values[i] = keys[i].value;
                inSlope[i] = keys[i].inTangent;
                outSlope[i] = keys[i].outTangent;
                if (keys[i].weightedMode != WeightedMode.None)
                    exact = false;
            }

            preWrap = curve.preWrapMode;
            postWrap = curve.postWrapMode;
            if (!IsClamp(preWrap) || !IsClamp(postWrap))
                exact = false;
            IsExact = exact;

            int segments = Math.Max(n - 1, 0);
            c0 = new float[segments];
            c1 = new float[segments];
            c2 = new float[segments];
            for (int i = 0; i < segments; i++)
            {
                float m1 = outSlope[i];
                float m2 = inSlope[i + 1];
                if (float.IsInfinity(m1) || float.IsInfinity(m2))
                    continue; // stepped: the left key's value, c0 = c1 = c2 = 0

                float dx = times[i + 1] - times[i];
                dx = Math.Max(dx, 0.0001f);
                float dy = values[i + 1] - values[i];
                float length = 1.0f / (float)(dx * dx);
                float d1 = (float)(m1 * dx);
                float d2 = (float)(m2 * dx);
                c0[i] = (float)((float)((float)((float)((float)(d1 + d2) - dy) - dy) * length) / dx);
                c1[i] = (float)((float)((float)((float)((float)((float)(dy + dy) + dy) - d1) - d1) - d2) * length);
                c2[i] = m1;
            }
        }

        /// <summary>Whether a curve has the keys and wrap modes this was copied from. Main thread (reads the curve).</summary>
        public bool Matches(AnimationCurve curve)
        {
            if (curve == null || curve.preWrapMode != preWrap || curve.postWrapMode != postWrap)
                return false;
            Keyframe[] keys = curve.keys;
            if (keys.Length != sourceKeys.Length)
                return false;
            for (int i = 0; i < keys.Length; i++)
            {
                Keyframe a = keys[i], b = sourceKeys[i];
                if (a.time != b.time || a.value != b.value || a.inTangent != b.inTangent || a.outTangent != b.outTangent ||
                    a.weightedMode != b.weightedMode || a.inWeight != b.inWeight || a.outWeight != b.outWeight)
                    return false;
            }

            return true;
        }

        private static bool IsClamp(WrapMode mode)
        {
            return mode != WrapMode.Loop && mode != WrapMode.PingPong;
        }

        public float Evaluate(float t)
        {
            // Unity's evaluation carries a NaN through the cubic.
            if (float.IsNaN(t))
                return t;
            int n = times.Length;
            if (n == 0)
                return 0;
            if (n == 1)
                return values[0];

            float begin = times[0];
            float end = times[n - 1];
            if (t >= end)
            {
                if (IsClamp(postWrap))
                    return values[n - 1];
                t = Wrap(t, begin, end, postWrap);
            }
            else if (t < begin)
            {
                if (IsClamp(preWrap))
                    return values[0];
                t = Wrap(t, begin, end, preWrap);
            }

            // Last key with time <= t (Unity's upper-bound search, lhs = first key after t, minus one).
            int lo = 0;
            int len = n;
            while (len > 0)
            {
                int half = len >> 1;
                int middle = lo + half;
                if (t < times[middle])
                {
                    len = half;
                }
                else
                {
                    lo = middle + 1;
                    len = len - half - 1;
                }
            }

            int lhs = lo - 1;
            if (lhs < 0)
                return values[0];
            if (lhs >= n - 1)
                return values[n - 1];

            float u = t - times[lhs];
            return (float)((float)(u * (float)((float)(u * (float)((float)(u * c0[lhs]) + c1[lhs])) + c2[lhs])) + values[lhs]);
        }

        private static float Wrap(float t, float begin, float end, WrapMode mode)
        {
            float range = end - begin;
            if (range <= 0)
                return begin;
            if (mode == WrapMode.PingPong)
                return begin + Mathf.PingPong(t - begin, range);
            return begin + Mathf.Repeat(t - begin, range);
        }
    }
}
