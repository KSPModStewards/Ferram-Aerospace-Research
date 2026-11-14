using System;
using System.Collections.Generic;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    /// <summary>
    /// Triangles in a bounding volume hierarchy, for "does this ray meet any of them" (PartHydrodynamics' enclosed and
    /// shadowed faces: a face's rays against its part's mesh and its neighbours'). Built by splitting the triangles'
    /// centroids at the middle of their longest axis, a leaf at a few triangles; a ray visits only the boxes it passes
    /// through, the nearest child first, and stops at its first hit.
    /// </summary>
    internal sealed class TriangleBvh
    {
        private const int LeafSize = 4;

        private readonly Vector3[] a, e1, e2;
        private readonly int[] order;   // triangle indices, each leaf's a run of them
        // the nodes: their boxes, and for an inner node its children (left, right), for a leaf its run (first, count)
        private readonly List<Vector3> min = new List<Vector3>(), max = new List<Vector3>();
        private readonly List<int> left = new List<int>(), right = new List<int>(), first = new List<int>(), count = new List<int>();

        /// <summary>The triangles (corner a, edges e1 and e2), indexed as given.</summary>
        public TriangleBvh(Vector3[] a, Vector3[] e1, Vector3[] e2)
        {
            this.a = a;
            this.e1 = e1;
            this.e2 = e2;
            int n = a.Length;
            order = new int[n];
            var centroid = new Vector3[n];
            for (int i = 0; i < n; i++)
            {
                order[i] = i;
                centroid[i] = a[i] + (e1[i] + e2[i]) / 3;
            }
            if (n > 0)
                Build(0, n, centroid);
        }

        private int Build(int from, int to, Vector3[] centroid)
        {
            Vector3 lo = Vector3.positiveInfinity, hi = Vector3.negativeInfinity, clo = lo, chi = hi;
            for (int i = from; i < to; i++)
            {
                int t = order[i];
                Vector3 p0 = a[t], p1 = p0 + e1[t], p2 = p0 + e2[t];
                lo = Vector3.Min(lo, Vector3.Min(p0, Vector3.Min(p1, p2)));
                hi = Vector3.Max(hi, Vector3.Max(p0, Vector3.Max(p1, p2)));
                clo = Vector3.Min(clo, centroid[t]);
                chi = Vector3.Max(chi, centroid[t]);
            }
            int node = min.Count;
            min.Add(lo);
            max.Add(hi);
            left.Add(-1);
            right.Add(-1);
            first.Add(from);
            count.Add(to - from);
            if (to - from <= LeafSize)
                return node;
            Vector3 ext = chi - clo;
            int axis = ext.x >= ext.y && ext.x >= ext.z ? 0 : ext.y >= ext.z ? 1 : 2;
            float split = 0.5f * (clo[axis] + chi[axis]);
            int mid = from;
            for (int i = from; i < to; i++)
                if (centroid[order[i]][axis] < split)
                {
                    (order[i], order[mid]) = (order[mid], order[i]);
                    mid++;
                }
            if (mid == from || mid == to)
                mid = (from + to) / 2;   // (every centroid alike along it: halved as they are)
            int l = Build(from, mid, centroid), r = Build(mid, to, centroid);
            left[node] = l;
            right[node] = r;
            count[node] = 0;
            return node;
        }

        // the ray's entry into a box (its distance; +infinity: it misses it)
        private float Enter(int node, Vector3 o, Vector3 inv)
        {
            Vector3 lo = min[node], hi = max[node];
            float t0 = 0, t1 = float.PositiveInfinity;
            for (int k = 0; k < 3; k++)
            {
                float ta = (lo[k] - o[k]) * inv[k], tb = (hi[k] - o[k]) * inv[k];
                if (float.IsNaN(ta) || float.IsNaN(tb))
                {
                    // parallel to these faces: in the slab or not
                    if (o[k] < lo[k] || o[k] > hi[k])
                        return float.PositiveInfinity;
                    continue;
                }
                if (ta > tb)
                    (ta, tb) = (tb, ta);
                t0 = Math.Max(t0, ta);
                t1 = Math.Min(t1, tb);
                if (t0 > t1)
                    return float.PositiveInfinity;
            }
            return t0;
        }

        private readonly Stack<int> stack = new Stack<int>(64);

        /// <summary>
        /// Moller and Trumbore's: where the line from o along d meets the triangle (corner a, edges e1 and e2), t along
        /// d (false: it misses it, or runs along its plane).
        /// </summary>
        public static bool Ray(Vector3 o, Vector3 d, Vector3 a, Vector3 e1, Vector3 e2, out float t)
        {
            t = 0;
            Vector3 p = Vector3.Cross(d, e2);
            float det = Vector3.Dot(e1, p);
            if (Math.Abs(det) < 1e-12f)
                return false;
            float invDet = 1 / det;
            Vector3 s = o - a;
            float u = Vector3.Dot(s, p) * invDet;
            if (u < 0 || u > 1)
                return false;
            Vector3 q = Vector3.Cross(s, e1);
            float v = Vector3.Dot(d, q) * invDet;
            if (v < 0 || u + v > 1)
                return false;
            t = Vector3.Dot(e2, q) * invDet;
            return true;
        }

        /// <summary>Whether the ray from o along d meets a triangle (not skip) beyond 0.1 mm.</summary>
        public bool AnyHit(Vector3 o, Vector3 d, int skip)
        {
            if (a.Length == 0)
                return false;
            var inv = new Vector3(1 / d.x, 1 / d.y, 1 / d.z);
            stack.Clear();
            if (Enter(0, o, inv) < float.PositiveInfinity)
                stack.Push(0);
            while (stack.Count > 0)
            {
                int node = stack.Pop();
                if (count[node] > 0)
                {
                    for (int i = first[node], end = first[node] + count[node]; i < end; i++)
                    {
                        int j = order[i];
                        if (j != skip && Ray(o, d, a[j], e1[j], e2[j], out float t) && t > 1e-4f)
                            return true;
                    }
                    continue;
                }
                int l = left[node], r = right[node];
                float tl = Enter(l, o, inv), tr = Enter(r, o, inv);
                // the nearer one popped first
                if (tl <= tr)
                {
                    if (tr < float.PositiveInfinity)
                        stack.Push(r);
                    if (tl < float.PositiveInfinity)
                        stack.Push(l);
                }
                else
                {
                    if (tl < float.PositiveInfinity)
                        stack.Push(l);
                    stack.Push(r);
                }
            }
            return false;
        }
    }
}
