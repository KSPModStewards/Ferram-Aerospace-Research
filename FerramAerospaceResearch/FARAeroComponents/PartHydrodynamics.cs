using System;
using System.Collections.Generic;
using FerramAerospaceResearch.FARPartGeometry;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    /// <summary>
    /// Water forces on a part from its own geometry (FAR's meshes for the part), for the part of it under the water
    /// plane, by strip (added-mass momentum) theory as used for planing hulls (Zarnick 1978):
    /// - the submerged surface is cut into strips across the horizontal flow; each strip's downward-facing area gives its
    ///   wetted beam w, bottom normal n and the velocity V of the bottom into the water (from the point velocity, so heave
    ///   and pitch rates count);
    /// - the water a strip passes takes up momentum m_a V, m_a = kA rho pi/2 (w/2)^2 per unit length; the hull feels
    ///   U d(m_a V)/ds (U the flow speed, s aft along the flow), pushing along -n: lift linear in trim, its drag share
    ///   (N sin tau), and damping of heave and pitch. Each strip's momentum change is spread aft over the rest of the
    ///   wetted run with the flat-plate loading sqrt((1-x)/x), so a chine-wetted plate's center of pressure is a quarter
    ///   of its wetted length behind the spray root (Savitsky's high-speed limit) instead of at it;
    /// - crossflow drag 1/2 Cd rho w V|V| per strip (damping when U is small);
    /// - impact pressure 1/2 Cp rho vn^2 on steep faces moving into the water (a blunt bow);
    /// - skin friction (ITTC-57) on the wetted area.
    /// Buoyancy stays stock's (PartBuoyancy).
    /// </summary>
    internal sealed class PartHydrodynamics
    {
        public static double AddedMassCoefficient = 1.0;
        public static double CrossflowDragIn = 1.0;
        public static double CrossflowDragOut = 0.5;
        public static double ImpactPressureCoefficient = 1.0;
        public static double StripLength = 0.25;
        public static double PlaningBuoyancy = 0.5;   // share of stock's buoyancy kept once planing (1: all of it)
        public const int MinStrips = 4, MaxStrips = 40;
        private const double KinematicViscosity = 1.19e-6;

        private readonly Part part;
        private object meshSource;
        private Vector3[] localVerts = Array.Empty<Vector3>();
        private int[] tris = Array.Empty<int>();
        private Vector3[] worldVerts = Array.Empty<Vector3>();
        private float[] depths = Array.Empty<float>();

        // clipped submerged polygons: their vertices, counts, and source triangle normals
        private readonly List<Vector3> polyVerts = new List<Vector3>();
        private readonly List<int> polyCounts = new List<int>();
        private readonly List<Vector3> polyNormals = new List<Vector3>();
        private readonly List<Vector3> scratchA = new List<Vector3>(8), scratchB = new List<Vector3>(8);

        private Strip[] strips = new Strip[MaxStrips];

        private struct Strip
        {
            public List<Vector2> Spans;  // lateral extents of its downward-facing pieces, merged into wetted widths
            public Vector3 Friction, FrictionPoint;
            public double FrictionWeight, WettedArea;
            public double Planform;      // downward-facing area projected on the water plane
            public Vector3 NormalSum;    // area-weighted outward normals of the downward-facing pieces
            public Vector3 CentroidSum;  // planform-weighted centroids
            public double Jump;          // U * d(m_a V) at this strip (N)
            public double Load;          // the normal force it ends up carrying (N)
            public Vector3 Normal, Point;
            public double V, Width;
        }

        // what the last step did, for tests and display (kN, m2, m)
        public Vector3 LastForce, LastMomentumForce, LastFrictionForce, LastImpactForce, LastCrossflowForce, LastBuoyancyCorrection;
        public double LastWettedArea, LastPlanform, LastWettedLength, LastLimiter = 1;
        public int LastStrips;

        public PartHydrodynamics(Part part)
        {
            this.part = part;
        }

        private bool EnsureMesh()
        {
            GeometryPartModule g = part.GetComponent<GeometryPartModule>();
            List<GeometryMesh> meshes = g != null ? g.meshDataList : null;
            if (meshes == null || meshes.Count == 0)
                return false;
            if (ReferenceEquals(meshes, meshSource))
                return tris.Length > 0;

            var verts = new List<Vector3>();
            var t = new List<int>();
            var seen = new List<(int verts, int tris, Bounds bounds)>();
            var log = new System.Text.StringBuilder();
            foreach (GeometryMesh m in meshes)
            {
                if (m == null || !m.valid || !m.gameObjectActiveInHierarchy || m.MeshLocalVerts == null)
                    continue;
                Matrix4x4 toPart = m.meshLocalToPart;
                // the same mesh twice (a copy for another purpose, LODs) would count its surface twice
                Bounds pb = GeometryMesh.TransformBounds(m.meshLocalBounds, toPart);
                bool copy = seen.Exists(s => s.verts == m.MeshLocalVerts.Length && s.tris == m.triangles.Length &&
                                             (s.bounds.center - pb.center).sqrMagnitude < 1e-6f &&
                                             (s.bounds.size - pb.size).sqrMagnitude < 1e-6f);
                log.Append($" {m.meshTransform.name}({m.triangles.Length / 3}{(copy ? ", copy skipped" : "")})");
                if (copy)
                    continue;
                seen.Add((m.MeshLocalVerts.Length, m.triangles.Length, pb));
                bool flip = toPart.determinant < 0;
                int offset = verts.Count;
                foreach (Vector3 v in m.MeshLocalVerts)
                    verts.Add(toPart.MultiplyPoint3x4(v));
                int[] mt = m.triangles;
                for (int i = 0; i + 2 < mt.Length; i += 3)
                {
                    t.Add(offset + mt[i]);
                    t.Add(offset + (flip ? mt[i + 2] : mt[i + 1]));
                    t.Add(offset + (flip ? mt[i + 1] : mt[i + 2]));
                }
            }

            FARLogger.Info($"Hydrodynamics of {part.partInfo?.name}: {t.Count / 3} triangles from meshes{log}");
            localVerts = verts.ToArray();
            tris = t.ToArray();
            worldVerts = new Vector3[localVerts.Length];
            depths = new float[localVerts.Length];
            meshSource = meshes;
            return tris.Length > 0;
        }

        private void Clear()
        {
            LastForce = LastMomentumForce = LastFrictionForce = LastImpactForce = LastCrossflowForce = LastBuoyancyCorrection = Vector3.zero;
            LastWettedArea = LastPlanform = LastWettedLength = 0;
            LastStrips = 0;
            LastLimiter = 1;
        }

        /// <summary>Computes and applies (Part.AddForceAtPosition, kN) the water forces of this physics step.</summary>
        public void Apply()
        {
            Clear();
            Rigidbody rb = part.rb;
            Vessel vessel = part.vessel;
            if (rb == null || vessel == null || part.submergedPortion <= 0)
                return;
            CelestialBody body = vessel.mainBody;
            if (body == null || !body.ocean || !EnsureMesh())
                return;

            double rho = body.oceanDensity * 1000;
            Transform tf = part.partTransform;
            Vector3 p0 = tf.position;
            Vector3 up = (Vector3)(((Vector3d)p0 - body.position).normalized);
            double depth0 = vessel.waterOffset - FlightGlobals.getAltitudeAtPos((Vector3d)p0, body);

            Matrix4x4 l2w = tf.localToWorldMatrix;
            bool any = false;
            for (int i = 0; i < localVerts.Length; i++)
            {
                Vector3 w = l2w.MultiplyPoint3x4(localVerts[i]);
                worldVerts[i] = w;
                float d = (float)(depth0 - Vector3.Dot(w - p0, up));
                depths[i] = d;
                any |= d > 0;
            }
            if (!any)
                return;

            Vector3 frame = Krakensbane.GetFrameVelocity();
            Vector3 vCom = rb.velocity + frame;
            Vector3 vh = vCom - up * Vector3.Dot(vCom, up);
            float U = vh.magnitude;
            Vector3 xHat = U > 0.05f ? vh / U : Vector3.ProjectOnPlane(tf.up, up).normalized;
            if (xHat.sqrMagnitude < 0.5f)
                xHat = Vector3.ProjectOnPlane(tf.forward, up).normalized;

            // clip every triangle to the part of it under the water
            polyVerts.Clear();
            polyCounts.Clear();
            polyNormals.Clear();
            float xMin = float.MaxValue, xMax = float.MinValue;
            for (int i = 0; i + 2 < tris.Length; i += 3)
            {
                int a = tris[i], b = tris[i + 1], c = tris[i + 2];
                if (depths[a] <= 0 && depths[b] <= 0 && depths[c] <= 0)
                    continue;
                Vector3 n = Vector3.Cross(worldVerts[b] - worldVerts[a], worldVerts[c] - worldVerts[a]);
                float n2 = n.sqrMagnitude;
                if (n2 < 1e-12f)
                    continue;
                n /= Mathf.Sqrt(n2);
                scratchA.Clear();
                ClipUnderwater(worldVerts[a], depths[a], worldVerts[b], depths[b], worldVerts[c], depths[c], scratchA);
                if (scratchA.Count < 3)
                    continue;
                foreach (Vector3 v in scratchA)
                {
                    float x = Vector3.Dot(v - p0, xHat);
                    xMin = Math.Min(xMin, x);
                    xMax = Math.Max(xMax, x);
                    polyVerts.Add(v);
                }
                polyCounts.Add(scratchA.Count);
                polyNormals.Add(n);
            }
            if (polyCounts.Count == 0 || xMax - xMin < 1e-4f)
                return;

            double wettedLength = xMax - xMin;
            int nStrips = Mathf.Clamp(Mathf.CeilToInt((float)(wettedLength / StripLength)), MinStrips, MaxStrips);
            float dx = (float)(wettedLength / nStrips);
            Vector3 yHat = Vector3.Cross(up, xHat);
            for (int k = 0; k < nStrips; k++)
            {
                List<Vector2> spans = strips[k].Spans ?? new List<Vector2>();
                spans.Clear();
                strips[k] = default;
                strips[k].Spans = spans;
            }

            double re = Math.Max(1e5, U * wettedLength / KinematicViscosity);
            double lg = Math.Log10(re) - 2;
            double cf = 0.075 / (lg * lg);

            Vector3 friction = Vector3.zero, impact = Vector3.zero;
            double wetted = 0;
            int at = 0;
            for (int pi = 0; pi < polyCounts.Count; pi++)
            {
                int count = polyCounts[pi];
                Vector3 n = polyNormals[pi];
                float down = -Vector3.Dot(n, up);
                // the strips this polygon spans
                float pxMin = float.MaxValue, pxMax = float.MinValue;
                for (int j = 0; j < count; j++)
                {
                    float x = Vector3.Dot(polyVerts[at + j] - p0, xHat);
                    pxMin = Math.Min(pxMin, x);
                    pxMax = Math.Max(pxMax, x);
                }
                int k0 = Mathf.Clamp((int)((pxMin - xMin) / dx), 0, nStrips - 1);
                int k1 = Mathf.Clamp((int)((pxMax - xMin) / dx), 0, nStrips - 1);
                for (int k = k0; k <= k1; k++)
                {
                    scratchA.Clear();
                    for (int j = 0; j < count; j++)
                        scratchA.Add(polyVerts[at + j]);
                    float lo = xMin + k * dx, hi = lo + dx;
                    ClipSlab(scratchA, scratchB, p0, xHat, lo, hi);
                    if (scratchB.Count < 3)
                        continue;
                    PolygonAreaCentroid(scratchB, n, out float area, out Vector3 centroid);
                    if (area <= 0)
                        continue;
                    wetted += area;

                    Vector3 v = rb.GetPointVelocity(centroid) + frame;
                    float vn = Vector3.Dot(v, n);
                    // skin friction against the surface's own slip through the water
                    Vector3 vt = v - n * vn;
                    float vtm = vt.magnitude;
                    ref Strip st = ref strips[k];
                    st.WettedArea += area;
                    st.FrictionWeight += area;
                    st.FrictionPoint += centroid * area;
                    if (vtm > 1e-3f)
                        st.Friction += -vt * (float)(0.5 * rho * vtm * cf * area);

                    if (down > 0.3f)
                    {
                        double planform = area * down;
                        st.Planform += planform;
                        st.NormalSum += n * (float)planform;
                        st.CentroidSum += centroid * (float)planform;
                        float yLo = float.MaxValue, yHi = float.MinValue;
                        foreach (Vector3 q in scratchB)
                        {
                            float y = Vector3.Dot(q - p0, yHat);
                            yLo = Math.Min(yLo, y);
                            yHi = Math.Max(yHi, y);
                        }
                        st.Spans.Add(new Vector2(yLo, yHi));
                    }
                    else if (vn > 0)
                    {
                        // a steep face pushed into the water
                        Vector3 f = -n * (float)(0.5 * ImpactPressureCoefficient * rho * vn * vn * area);
                        impact += f;
                        part.AddForceAtPosition((Vector3d)f * 0.001, (Vector3d)centroid);
                    }
                }
                at += count;
            }

            // momentum: bow (largest x) to stern; a dry strip ends a wetted run (the flow leaves the bottom)
            double maPrevV = 0, stiffness = 0;
            Vector3 crossflow = Vector3.zero;
            double ka = AddedMassCoefficient * rho * Math.PI / 2;
            for (int k = nStrips - 1; k >= 0; k--)
            {
                ref Strip s = ref strips[k];
                if (s.Planform <= 0)
                {
                    maPrevV = 0;
                    continue;
                }
                s.Normal = s.NormalSum.normalized;
                s.Point = s.CentroidSum / (float)s.Planform;
                // wetted widths: the merged lateral spans (copies and inner faces overlap, separate hulls don't)
                MergeSpans(s.Spans, out double width, out double sumHalfWidth2);
                s.Width = width;
                s.V = Vector3.Dot(rb.GetPointVelocity(s.Point) + frame, s.Normal);
                double ma = ka * sumHalfWidth2;
                double maV = ma * s.V;
                s.Jump = U * (maV - maPrevV);
                maPrevV = maV;
                stiffness = Math.Max(stiffness, U * ma);
                double cd = s.V > 0 ? CrossflowDragIn : CrossflowDragOut;
                double fcf = 0.5 * cd * rho * s.Width * dx * s.V * Math.Abs(s.V);
                stiffness += cd * rho * s.Width * dx * Math.Abs(s.V);
                s.Load = fcf;
                crossflow += -s.Normal * (float)fcf;
            }

            // spread each jump aft over the rest of its wetted run with the flat-plate loading
            for (int k = nStrips - 1; k >= 0; k--)
            {
                if (strips[k].Planform <= 0 || strips[k].Jump == 0)
                    continue;
                int end = k;
                while (end - 1 >= 0 && strips[end - 1].Planform > 0)
                    end--;
                int span = k - end + 1;
                for (int j = k; j >= end; j--)
                {
                    double x0 = (double)(k - j) / span, x1 = (double)(k - j + 1) / span;
                    strips[j].Load += strips[k].Jump * (PlateLoad(x1) - PlateLoad(x0)) / (Math.PI / 2);
                }
            }

            // explicit-step safety: the damping these forces put on the vessel stays well inside one step
            double mass = Math.Max(vessel.totalMass, rb.mass) * 1000;
            double limiter = Math.Min(1, 0.5 * mass / Math.Max(1e-9, stiffness * TimeWarp.fixedDeltaTime));

            Vector3 momentum = Vector3.zero;
            int wetStrips = 0;
            for (int k = 0; k < nStrips; k++)
            {
                ref Strip s = ref strips[k];
                if (s.Planform <= 0)
                    continue;
                wetStrips++;
                Vector3 f = -s.Normal * (float)(s.Load * limiter);
                momentum += f;
                part.AddForceAtPosition((Vector3d)f * 0.001, (Vector3d)s.Point);
            }

            // friction on the wetted area, as much of it as the bottom's merged width says is not a copy
            double planformTotal = 0;
            wetted = 0;
            for (int k = 0; k < nStrips; k++)
            {
                ref Strip s = ref strips[k];
                double dup = s.Planform > 0 ? Math.Min(1, s.Width * dx / s.Planform) : 1;
                planformTotal += s.Planform * dup;
                wetted += s.WettedArea * dup;
                if (s.FrictionWeight <= 0)
                    continue;
                Vector3 f = s.Friction * (float)dup;
                friction += f;
                part.AddForceAtPosition((Vector3d)f * 0.001, (Vector3d)(s.FrictionPoint / (float)s.FrictionWeight));
            }

            // hydrostatics at speed: once the flow leaves the transom dry, the bottom near it carries no hydrostatic
            // pressure (Savitsky's buoyancy term is about half the displacement's; Zarnick scales it by 0.5). Stock's
            // buoyancy stays; this takes back its share, fading in from C_V 1 to 3 on the mean wetted beam.
            Vector3 hydrostatic = Vector3.zero;
            PartBuoyancy pb = part.partBuoyancy;
            if (PlaningBuoyancy < 1 && pb != null && planformTotal > 0 && wettedLength > 0)
            {
                double beam = planformTotal / wettedLength;
                double cv = U / Math.Sqrt(9.81 * Math.Max(0.05, beam));
                double x = Math.Max(0, Math.Min(1, (cv - 1) / 2));
                double take = (1 - PlaningBuoyancy) * x * x * (3 - 2 * x);
                if (take > 0)
                {
                    hydrostatic = -pb.lastBuoyantForce * (float)take;
                    part.AddForceAtPosition((Vector3d)hydrostatic, (Vector3d)pb.lastForcePosition);
                }
            }
            LastBuoyancyCorrection = hydrostatic;

            LastMomentumForce = (momentum - crossflow * (float)limiter) * 0.001f;
            LastCrossflowForce = crossflow * (float)(limiter * 0.001);
            LastFrictionForce = friction * 0.001f;
            LastImpactForce = impact * 0.001f;
            LastForce = (momentum + friction + impact) * 0.001f;
            LastWettedArea = wetted;
            LastPlanform = planformTotal;
            LastWettedLength = wettedLength;
            LastStrips = wetStrips;
            LastLimiter = limiter;
        }

        // total width of the union of spans, and the sum of each merged run's half-width squared (added mass per length)
        private static void MergeSpans(List<Vector2> spans, out double width, out double sumHalfWidth2)
        {
            width = sumHalfWidth2 = 0;
            if (spans.Count == 0)
                return;
            spans.Sort((a, b) => a.x.CompareTo(b.x));
            float lo = spans[0].x, hi = spans[0].y;
            for (int i = 1; i <= spans.Count; i++)
            {
                if (i < spans.Count && spans[i].x <= hi + 0.01f)
                {
                    hi = Math.Max(hi, spans[i].y);
                    continue;
                }
                double w = hi - lo;
                width += w;
                sumHalfWidth2 += w * w / 4;
                if (i < spans.Count)
                {
                    lo = spans[i].x;
                    hi = spans[i].y;
                }
            }
        }

        // integral of sqrt((1-x)/x) from 0 to x
        private static double PlateLoad(double x)
        {
            x = Math.Max(0, Math.Min(1, x));
            return Math.Sqrt(x * (1 - x)) + Math.Asin(Math.Sqrt(x));
        }

        private static void ClipUnderwater(Vector3 a, float da, Vector3 b, float db, Vector3 c, float dc, List<Vector3> into)
        {
            Edge(a, da, b, db, into);
            Edge(b, db, c, dc, into);
            Edge(c, dc, a, da, into);
        }

        // Sutherland-Hodgman against depth > 0: emits the start vertex if under, and the crossing
        private static void Edge(Vector3 p, float dp, Vector3 q, float dq, List<Vector3> into)
        {
            if (dp > 0)
                into.Add(p);
            if (dp > 0 != dq > 0)
                into.Add(Vector3.Lerp(p, q, dp / (dp - dq)));
        }

        private void ClipSlab(List<Vector3> poly, List<Vector3> into, Vector3 origin, Vector3 axis, float lo, float hi)
        {
            // keep x >= lo, then x <= hi
            into.Clear();
            ClipPlane(poly, into, origin, axis, lo, 1);
            poly.Clear();
            poly.AddRange(into);
            into.Clear();
            ClipPlane(poly, into, origin, axis, hi, -1);
        }

        private static void ClipPlane(List<Vector3> poly, List<Vector3> into, Vector3 origin, Vector3 axis, float at, float sign)
        {
            int n = poly.Count;
            for (int i = 0; i < n; i++)
            {
                Vector3 p = poly[i], q = poly[(i + 1) % n];
                float dp = sign * (Vector3.Dot(p - origin, axis) - at), dq = sign * (Vector3.Dot(q - origin, axis) - at);
                if (dp >= 0)
                    into.Add(p);
                if (dp >= 0 != dq >= 0)
                    into.Add(Vector3.Lerp(p, q, dp / (dp - dq)));
            }
        }

        private static void PolygonAreaCentroid(List<Vector3> poly, Vector3 normal, out float area, out Vector3 centroid)
        {
            area = 0;
            centroid = Vector3.zero;
            Vector3 o = poly[0];
            for (int i = 1; i + 1 < poly.Count; i++)
            {
                float a = 0.5f * Vector3.Dot(Vector3.Cross(poly[i] - o, poly[i + 1] - o), normal);
                area += a;
                centroid += (o + poly[i] + poly[i + 1]) * (a / 3);
            }
            if (area > 1e-9f)
                centroid /= area;
            else
                area = 0;
        }
    }
}
