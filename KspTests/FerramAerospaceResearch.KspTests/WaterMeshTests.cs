using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using FerramAerospaceResearch.FARAeroComponents;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The meshes FAR's water model could read for a part (PartHydrodynamics.MeshSource): its colliders, its visible
    /// meshes, or FAR's own choice of the two (what its voxels are made from), compared part by part.
    /// </summary>
    public static class WaterMeshTests
    {
        /// <summary>
        /// For every part of each craft (parameter crafts: name@SPH or name@VAB, + separated), each source's meshes as the
        /// water model joins them, in part space: triangles, pieces, how many pieces are closed (what the model takes
        /// buoyancy from), the volume of the closed pieces summed (what the model's pressure adds up to: overlapping
        /// pieces count twice) and their union (sampled on a grid), the bounds; and between colliders and visible meshes,
        /// how far each surface lies from the other (visible vertices to the collider surface and back, sampled), over
        /// the part's size. Writes water-meshes.csv (a row a part) and records a summary a craft.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, StartsAt = StartState.SpaceCenter, TimeoutSec = 3600)]
        public static IEnumerator Measure_ColliderVsVisual(TestContext ctx)
        {
            string[] crafts = (ctx.Param("crafts") ?? "FAR SkyEye AS@SPH+Aeris 4A@SPH+FAR Firehound MS F0@SPH").Split('+');
            var csv = new StringBuilder("craft,part,size_m," +
                                        "col_tris,col_pieces,col_closed,col_volume_sum,col_volume_union,col_bounds," +
                                        "vis_tris,vis_pieces,vis_closed,vis_volume_sum,vis_volume_union,vis_bounds," +
                                        "far_source,far_tris,vis_to_col_mean,vis_to_col_p95,vis_to_col_max,col_to_vis_mean,col_to_vis_p95,col_to_vis_max\n");
            foreach (string raw in crafts)
            {
                string[] nd = raw.Trim().Split('@');
                string craft = nd[0], dir = nd.Length > 1 ? nd[1] : "SPH";
                string path = ctx.CraftPath(craft, dir);
                if (!File.Exists(path))
                    path = Path.GetFullPath(Path.Combine(KSPUtil.ApplicationRootPath, "Ships", dir, craft + ".craft"));
                if (!File.Exists(path))
                {
                    ctx.Record(craft, "craft not found");
                    continue;
                }
                ctx.Step(craft);
                yield return Nav.EnsureState(ctx, StartState.Flight, path, "Runway");
                yield return Wait.Seconds(5);
                Vessel v = FlightGlobals.ActiveVessel;

                int n = 0, visClosed = 0, colClosed = 0, bothClosed = 0, colOverlap = 0, farColliders = 0;
                long colTris = 0, visTris = 0, farTris = 0;
                double colVol = 0, visVol = 0;
                var rel95 = new List<double>();
                var relMean = new List<double>();
                foreach (Part p in v.parts)
                {
                    Mesh3 col = Mesh3.Of(p, PartHydrodynamics.MeshSourceKind.Colliders);
                    Mesh3 vis = Mesh3.Of(p, PartHydrodynamics.MeshSourceKind.Visual);
                    Mesh3 far = Mesh3.Of(p, PartHydrodynamics.MeshSourceKind.Far);
                    if (col.Tris == 0 && vis.Tris == 0)
                        continue;
                    n++;
                    Bounds all = col.Bounds;
                    if (vis.Tris > 0)
                    {
                        if (col.Tris == 0) all = vis.Bounds;
                        else all.Encapsulate(vis.Bounds);
                    }
                    double size = all.size.magnitude;
                    double colUnion = col.UnionVolume(all), visUnion = vis.UnionVolume(all);
                    (double mean, double p95, double max) vc = vis.DistanceTo(col), cv = col.DistanceTo(vis);
                    string farSource = far.Tris == 0 ? "none" : far.Tris == col.Tris ? "colliders" : far.Tris == vis.Tris ? "visual" : "mixed";
                    colTris += col.Tris;
                    visTris += vis.Tris;
                    farTris += far.Tris;
                    if (farSource == "colliders") farColliders++;
                    if (col.Closed > 0) colClosed++;
                    if (vis.Closed > 0 && vis.Closed == vis.Pieces.Count) visClosed++;
                    if (col.Closed == col.Pieces.Count && vis.Closed == vis.Pieces.Count && col.Tris > 0 && vis.Tris > 0)
                    {
                        bothClosed++;
                        colVol += colUnion;
                        visVol += visUnion;
                    }
                    if (col.VolumeSum > colUnion * 1.05 + 1e-4)
                        colOverlap++;
                    if (!double.IsNaN(vc.mean) && size > 0)
                    {
                        relMean.Add(Math.Max(vc.mean, cv.mean) / size);
                        rel95.Add(Math.Max(vc.p95, cv.p95) / size);
                    }
                    csv.AppendLine(string.Join(",", craft, p.partInfo.name, F(size),
                                               col.Tris.ToString(), col.Pieces.Count.ToString(), col.Closed.ToString(), F(col.VolumeSum), F(colUnion), V(col.Bounds.size),
                                               vis.Tris.ToString(), vis.Pieces.Count.ToString(), vis.Closed.ToString(), F(vis.VolumeSum), F(visUnion), V(vis.Bounds.size),
                                               farSource, far.Tris.ToString(), F(vc.mean), F(vc.p95), F(vc.max), F(cv.mean), F(cv.p95), F(cv.max)));
                    yield return null;   // keep the game alive between parts
                }
                rel95.Sort();
                relMean.Sort();
                double Med(List<double> x) => x.Count > 0 ? x[x.Count / 2] : double.NaN;
                double P90(List<double> x) => x.Count > 0 ? x[(int)(x.Count * 0.9)] : double.NaN;
                ctx.Record(craft, $"{n} parts: triangles colliders {colTris}, visible {visTris}, FAR's {farTris} (FAR uses the colliders for {farColliders}); " +
                                  $"closed: colliders {colClosed}, visible {visClosed} (all pieces); both closed {bothClosed}: volume colliders {colVol:F2} m3, " +
                                  $"visible {visVol:F2}; collider pieces overlapping (summed volume over 1.05 x union) {colOverlap}; " +
                                  $"surface apart over the part's size: mean median {Med(relMean):P1} (90th {P90(relMean):P1}), " +
                                  $"95th-percentile median {Med(rel95):P1} (90th {P90(rel95):P1})");
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "water-meshes.csv"), csv.ToString());
            }
        }

        private static string F(double x) => double.IsNaN(x) ? "" : x.ToString("G5", CultureInfo.InvariantCulture);
        private static string V(Vector3 v) => $"{F(v.x)} x {F(v.y)} x {F(v.z)}";

        /// <summary>A part's meshes from one source, in part space, piece by piece.</summary>
        private sealed class Mesh3
        {
            public readonly List<(Vector3[] v, int[] t, bool closed, double volume)> Pieces = new List<(Vector3[], int[], bool, double)>();
            public int Tris, Closed;
            public double VolumeSum;
            public Bounds Bounds;

            public static Mesh3 Of(Part part, PartHydrodynamics.MeshSourceKind kind)
            {
                var m = new Mesh3();
                bool first = true;
                PartHydrodynamics.Pieces(part, kind, (name, mv, mt, toPart) =>
                {
                    bool flip = toPart.determinant < 0;
                    var v = mv.Select(x => toPart.MultiplyPoint3x4(x)).ToArray();
                    var t = new int[mt.Length - mt.Length % 3];
                    for (int i = 0; i + 2 < mt.Length; i += 3)
                    {
                        t[i] = mt[i];
                        t[i + 1] = flip ? mt[i + 2] : mt[i + 1];
                        t[i + 2] = flip ? mt[i + 1] : mt[i + 2];
                    }
                    Vector3 areaSum = Vector3.zero;
                    double area = 0, volume = 0;
                    for (int i = 0; i + 2 < t.Length; i += 3)
                    {
                        Vector3 a = v[t[i]], b = v[t[i + 1]], c = v[t[i + 2]];
                        Vector3 n2 = Vector3.Cross(b - a, c - a);
                        areaSum += n2 * 0.5f;
                        area += n2.magnitude * 0.5;
                        volume += Vector3.Dot(a, Vector3.Cross(b, c)) / 6.0;
                    }
                    bool closed = area > 0 && volume > 0 && areaSum.magnitude < PartHydrodynamics.ClosureTolerance * area;
                    m.Pieces.Add((v, t, closed, volume));
                    m.Tris += t.Length / 3;
                    if (closed)
                    {
                        m.Closed++;
                        m.VolumeSum += volume;
                    }
                    foreach (Vector3 x in v)
                    {
                        if (first)
                        {
                            m.Bounds = new Bounds(x, Vector3.zero);
                            first = false;
                        }
                        else
                            m.Bounds.Encapsulate(x);
                    }
                });
                return m;
            }

            // the closed pieces' union, sampled on a 16^3 grid over the bounds (a point inside any closed piece)
            public double UnionVolume(Bounds over)
            {
                if (Closed == 0)
                    return 0;
                const int N = 16;
                Vector3 step = over.size / N;
                int inside = 0;
                for (int i = 0; i < N; i++)
                for (int j = 0; j < N; j++)
                for (int k = 0; k < N; k++)
                {
                    Vector3 q = over.min + Vector3.Scale(step, new Vector3(i + 0.5f, j + 0.5f, k + 0.5f));
                    foreach (var piece in Pieces)
                        if (piece.closed && Inside(piece.v, piece.t, q))
                        {
                            inside++;
                            break;
                        }
                }
                return inside * (double)step.x * step.y * step.z;
            }

            private static bool Inside(Vector3[] v, int[] t, Vector3 q)
            {
                var dir = new Vector3(0.3713907f, 0.5570860f, 0.7427813f);
                int crossings = 0;
                for (int i = 0; i + 2 < t.Length; i += 3)
                {
                    Vector3 a = v[t[i]], e1 = v[t[i + 1]] - a, e2 = v[t[i + 2]] - a;
                    Vector3 pv = Vector3.Cross(dir, e2);
                    float det = Vector3.Dot(e1, pv);
                    if (Math.Abs(det) < 1e-12f)
                        continue;
                    float inv = 1 / det;
                    Vector3 s = q - a;
                    float u = Vector3.Dot(s, pv) * inv;
                    if (u < 0 || u > 1)
                        continue;
                    Vector3 qv = Vector3.Cross(s, e1);
                    float w = Vector3.Dot(dir, qv) * inv;
                    if (w < 0 || u + w > 1)
                        continue;
                    if (Vector3.Dot(e2, qv) * inv > 0)
                        crossings++;
                }
                return (crossings & 1) == 1;
            }

            // this mesh's vertices (up to 800, evenly picked) to the other's surface: mean, 95th percentile, largest (m)
            public (double mean, double p95, double max) DistanceTo(Mesh3 other)
            {
                if (Tris == 0 || other.Tris == 0)
                    return (double.NaN, double.NaN, double.NaN);
                var pts = Pieces.SelectMany(pc => pc.v).ToArray();
                int stride = Math.Max(1, pts.Length / 800);
                var d = new List<double>();
                for (int i = 0; i < pts.Length; i += stride)
                {
                    double best = double.MaxValue;
                    foreach (var pc in other.Pieces)
                        for (int k = 0; k + 2 < pc.t.Length; k += 3)
                            best = Math.Min(best, PointTriangle(pts[i], pc.v[pc.t[k]], pc.v[pc.t[k + 1]], pc.v[pc.t[k + 2]]));
                    d.Add(best);
                }
                d.Sort();
                return (d.Average(), d[(int)(d.Count * 0.95)], d[d.Count - 1]);
            }

            // distance from p to the triangle abc (Ericson, Real-Time Collision Detection 5.1.5)
            private static double PointTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
            {
                Vector3 ab = b - a, ac = c - a, ap = p - a;
                float d1 = Vector3.Dot(ab, ap), d2 = Vector3.Dot(ac, ap);
                if (d1 <= 0 && d2 <= 0) return (p - a).magnitude;
                Vector3 bp = p - b;
                float d3 = Vector3.Dot(ab, bp), d4 = Vector3.Dot(ac, bp);
                if (d3 >= 0 && d4 <= d3) return (p - b).magnitude;
                float vc = d1 * d4 - d3 * d2;
                if (vc <= 0 && d1 >= 0 && d3 <= 0)
                    return (p - (a + ab * (d1 / (d1 - d3)))).magnitude;
                Vector3 cp = p - c;
                float d5 = Vector3.Dot(ab, cp), d6 = Vector3.Dot(ac, cp);
                if (d6 >= 0 && d5 <= d6) return (p - c).magnitude;
                float vb = d5 * d2 - d1 * d6;
                if (vb <= 0 && d2 >= 0 && d6 <= 0)
                    return (p - (a + ac * (d2 / (d2 - d6)))).magnitude;
                float va = d3 * d6 - d5 * d4;
                if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0)
                    return (p - (b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))))).magnitude;
                float denom = 1 / (va + vb + vc);
                return (p - (a + ab * (vb * denom) + ac * (vc * denom))).magnitude;
            }
        }
    }
}
