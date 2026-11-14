using System;
using System.Collections.Generic;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    /// <summary>
    /// A triangle mesh simplified within a distance of its shape (Garland and Heckbert's quadric error metric): its
    /// vertices welded where they meet, each holding the planes of the triangles around it; the edge whose collapse
    /// moves the surface least (the sum of the squared distances from the merged vertex to those planes) collapsed first,
    /// to the point that minimises it, and on until the cheapest would move it more than the tolerance (its square). A
    /// flat panel or a gentle curve falls to a few large triangles; a crease (a step, a chine, an edge of a wing) stays,
    /// its planes' distances growing at once off it. A closed surface stays closed: an edge collapses only where its
    /// two ends share exactly two neighbours (the link condition), and not where a triangle would turn over; an open
    /// edge's or a non-manifold one's vertices are left where they are.
    /// </summary>
    internal static class MeshSimplifier
    {
        private struct Candidate
        {
            public double Cost;
            public int A, B, StampA, StampB;
        }

        // vertices this close (m) welded before simplifying
        private const double Weld = 1e-5;

        /// <summary>
        /// Simplifies the mesh (part space) in place within tolerance (m). Returns how many triangles it took off.
        /// </summary>
        public static int Simplify(List<Vector3> verts, List<int> tris, double tolerance)
        {
            int nf = tris.Count / 3;
            if (tolerance <= 0 || nf == 0)
                return 0;

            // weld
            var index = new Dictionary<(long, long, long), int>();
            var p = new List<Vector3d>();
            var map = new int[verts.Count];
            for (int i = 0; i < verts.Count; i++)
            {
                Vector3d v = verts[i];
                var key = ((long)Math.Round(v.x / Weld), (long)Math.Round(v.y / Weld), (long)Math.Round(v.z / Weld));
                if (!index.TryGetValue(key, out int at))
                {
                    at = p.Count;
                    index[key] = at;
                    p.Add(v);
                }
                map[i] = at;
            }
            int nv = p.Count;
            var f = new int[nf * 3];
            var alive = new bool[nf];
            var vf = new List<int>[nv];
            for (int i = 0; i < nv; i++)
                vf[i] = new List<int>(6);
            for (int k = 0; k < nf; k++)
            {
                int a = map[tris[3 * k]], b = map[tris[3 * k + 1]], c = map[tris[3 * k + 2]];
                f[3 * k] = a;
                f[3 * k + 1] = b;
                f[3 * k + 2] = c;
                alive[k] = a != b && b != c && a != c;
                if (!alive[k])
                    continue;
                vf[a].Add(k);
                vf[b].Add(k);
                vf[c].Add(k);
            }

            // each vertex's planes; open and non-manifold edges' vertices frozen
            var q = new double[nv * 10];
            var frozen = new bool[nv];
            var edges = new Dictionary<(int, int), int>();
            for (int k = 0; k < nf; k++)
            {
                if (!alive[k])
                    continue;
                int a = f[3 * k], b = f[3 * k + 1], c = f[3 * k + 2];
                Vector3d n = Vector3d.Cross(p[b] - p[a], p[c] - p[a]);
                double len = n.magnitude;
                if (len > 1e-18)
                {
                    n /= len;
                    double d = -Vector3d.Dot(n, p[a]);
                    AddPlane(q, a, n, d);
                    AddPlane(q, b, n, d);
                    AddPlane(q, c, n, d);
                }
                CountEdge(edges, a, b);
                CountEdge(edges, b, c);
                CountEdge(edges, c, a);
            }
            foreach (var e in edges)
                if (e.Value != 2)
                {
                    frozen[e.Key.Item1] = true;
                    frozen[e.Key.Item2] = true;
                }

            double tol2 = tolerance * tolerance;
            var stamp = new int[nv];
            var dead = new bool[nv];
            var heap = new List<Candidate>(edges.Count);
            foreach (var e in edges)
                if (e.Value == 2)
                    Push(heap, Evaluate(p, q, stamp, e.Key.Item1, e.Key.Item2, out _));

            var ringA = new HashSet<int>();
            var ringB = new HashSet<int>();
            int removed = 0;
            while (heap.Count > 0)
            {
                Candidate c = Pop(heap);
                if (c.Cost > tol2)
                    break;
                int a = c.A, b = c.B;
                if (dead[a] || dead[b] || stamp[a] != c.StampA || stamp[b] != c.StampB || frozen[a] || frozen[b])
                    continue;
                Evaluate(p, q, stamp, a, b, out Vector3d v);
                // the link condition: their rings share exactly the two vertices across the edge
                Ring(f, alive, vf[a], a, ringA);
                Ring(f, alive, vf[b], b, ringB);
                if (!ringA.Contains(b))
                    continue;
                int shared = 0;
                foreach (int x in ringA)
                    if (ringB.Contains(x))
                        shared++;
                if (shared != 2)
                    continue;
                // no triangle turned over or crushed
                if (Flips(p, f, alive, vf[a], a, b, v) || Flips(p, f, alive, vf[b], b, a, v))
                    continue;

                // collapse b into a, at v
                p[a] = v;
                for (int i = 0; i < 10; i++)
                    q[10 * a + i] += q[10 * b + i];
                foreach (int k in vf[b])
                {
                    if (!alive[k])
                        continue;
                    bool hasA = f[3 * k] == a || f[3 * k + 1] == a || f[3 * k + 2] == a;
                    if (hasA)
                    {
                        alive[k] = false;
                        removed++;
                        continue;
                    }
                    for (int j = 0; j < 3; j++)
                        if (f[3 * k + j] == b)
                            f[3 * k + j] = a;
                    vf[a].Add(k);
                }
                vf[b].Clear();
                vf[a].RemoveAll(k => !alive[k]);
                dead[b] = true;
                stamp[a]++;
                stamp[b]++;
                Ring(f, alive, vf[a], a, ringA);
                foreach (int x in ringA)
                    if (!frozen[x])
                        Push(heap, Evaluate(p, q, stamp, a, x, out _));
            }

            if (removed == 0)
                return 0;
            // the mesh back, its vertices in use only
            var remap = new int[nv];
            for (int i = 0; i < nv; i++)
                remap[i] = -1;
            verts.Clear();
            tris.Clear();
            for (int k = 0; k < nf; k++)
            {
                if (!alive[k])
                    continue;
                for (int j = 0; j < 3; j++)
                {
                    int x = f[3 * k + j];
                    if (remap[x] < 0)
                    {
                        remap[x] = verts.Count;
                        verts.Add((Vector3)p[x]);
                    }
                    tris.Add(remap[x]);
                }
            }
            return removed;
        }

        private static void CountEdge(Dictionary<(int, int), int> edges, int a, int b)
        {
            var key = a < b ? (a, b) : (b, a);
            edges.TryGetValue(key, out int n);
            edges[key] = n + 1;
        }

        // the plane n.x + d = 0 into vertex i's quadric: A += n n^T, b += d n, c += d^2
        private static void AddPlane(double[] q, int i, Vector3d n, double d)
        {
            int o = 10 * i;
            q[o] += n.x * n.x;
            q[o + 1] += n.x * n.y;
            q[o + 2] += n.x * n.z;
            q[o + 3] += n.y * n.y;
            q[o + 4] += n.y * n.z;
            q[o + 5] += n.z * n.z;
            q[o + 6] += d * n.x;
            q[o + 7] += d * n.y;
            q[o + 8] += d * n.z;
            q[o + 9] += d * d;
        }

        private static double Error(double[] s, Vector3d v) =>
            s[0] * v.x * v.x + 2 * s[1] * v.x * v.y + 2 * s[2] * v.x * v.z + s[3] * v.y * v.y + 2 * s[4] * v.y * v.z + s[5] * v.z * v.z +
            2 * (s[6] * v.x + s[7] * v.y + s[8] * v.z) + s[9];

        // an edge's collapse: the two quadrics' sum, its minimum (solved; held near the edge) or the best of its ends
        // and middle
        private static Candidate Evaluate(List<Vector3d> p, double[] q, int[] stamp, int a, int b, out Vector3d best)
        {
            var s = new double[10];
            for (int i = 0; i < 10; i++)
                s[i] = q[10 * a + i] + q[10 * b + i];
            Vector3d pa = p[a], pb = p[b], mid = (pa + pb) * 0.5;
            best = mid;
            double cost = Error(s, mid);
            double ea = Error(s, pa), eb = Error(s, pb);
            if (ea < cost) { cost = ea; best = pa; }
            if (eb < cost) { cost = eb; best = pb; }
            // A v = -b
            double a00 = s[0], a01 = s[1], a02 = s[2], a11 = s[3], a12 = s[4], a22 = s[5];
            double det = a00 * (a11 * a22 - a12 * a12) - a01 * (a01 * a22 - a12 * a02) + a02 * (a01 * a12 - a11 * a02);
            double scale = Math.Max(1e-12, Math.Abs(a00 * a11 * a22));
            if (Math.Abs(det) > 1e-6 * scale)
            {
                double b0 = -s[6], b1 = -s[7], b2 = -s[8];
                double x = (b0 * (a11 * a22 - a12 * a12) - a01 * (b1 * a22 - a12 * b2) + a02 * (b1 * a12 - a11 * b2)) / det;
                double y = (a00 * (b1 * a22 - a12 * b2) - b0 * (a01 * a22 - a12 * a02) + a02 * (a01 * b2 - b1 * a02)) / det;
                double z = (a00 * (a11 * b2 - b1 * a12) - a01 * (a01 * b2 - b1 * a02) + b0 * (a01 * a12 - a11 * a02)) / det;
                var v = new Vector3d(x, y, z);
                double e = Error(s, v);
                if (e < cost && (v - mid).magnitude <= (pb - pa).magnitude)
                {
                    cost = e;
                    best = v;
                }
            }
            return new Candidate { Cost = Math.Max(0, cost), A = a, B = b, StampA = stamp[a], StampB = stamp[b] };
        }

        private static void Ring(int[] f, bool[] alive, List<int> faces, int v, HashSet<int> ring)
        {
            ring.Clear();
            foreach (int k in faces)
            {
                if (!alive[k])
                    continue;
                for (int j = 0; j < 3; j++)
                    if (f[3 * k + j] != v)
                        ring.Add(f[3 * k + j]);
            }
        }

        // whether moving v (its faces not on the edge to other) to to turns a face over or crushes it
        private static bool Flips(List<Vector3d> p, int[] f, bool[] alive, List<int> faces, int v, int other, Vector3d to)
        {
            foreach (int k in faces)
            {
                if (!alive[k])
                    continue;
                int a = f[3 * k], b = f[3 * k + 1], c = f[3 * k + 2];
                if (a == other || b == other || c == other)
                    continue;
                Vector3d pa = p[a], pb = p[b], pc = p[c];
                Vector3d n0 = Vector3d.Cross(pb - pa, pc - pa);
                if (a == v) pa = to;
                else if (b == v) pb = to;
                else pc = to;
                Vector3d n1 = Vector3d.Cross(pb - pa, pc - pa);
                double l0 = n0.magnitude, l1 = n1.magnitude;
                if (l1 < 1e-12 || l0 < 1e-12 || Vector3d.Dot(n0, n1) < 0.5 * l0 * l1)
                    return true;
            }
            return false;
        }

        private static void Push(List<Candidate> heap, Candidate c)
        {
            heap.Add(c);
            int i = heap.Count - 1;
            while (i > 0)
            {
                int parent = (i - 1) / 2;
                if (heap[parent].Cost <= heap[i].Cost)
                    break;
                (heap[parent], heap[i]) = (heap[i], heap[parent]);
                i = parent;
            }
        }

        private static Candidate Pop(List<Candidate> heap)
        {
            Candidate top = heap[0];
            int last = heap.Count - 1;
            heap[0] = heap[last];
            heap.RemoveAt(last);
            int i = 0;
            while (true)
            {
                int l = 2 * i + 1, r = l + 1, m = i;
                if (l < heap.Count && heap[l].Cost < heap[m].Cost) m = l;
                if (r < heap.Count && heap[r].Cost < heap[m].Cost) m = r;
                if (m == i)
                    break;
                (heap[m], heap[i]) = (heap[i], heap[m]);
                i = m;
            }
            return top;
        }
    }
}
