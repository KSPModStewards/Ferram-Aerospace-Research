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
    /// - the flow leaves a bottom that falls away from it going aft (a step, a transom, a rising afterbody): a section
    ///   whose momentum m_a V is more than SeparationDrop below the run's peak (narrower, or its bottom less steep to the
    ///   flow) is in the separated, ventilated wake. It takes no momentum (the water keeps what it was given; no
    ///   suction) and no crossflow pull, and the run's pressure ends ahead of it. It planes again only where it meets
    ///   the water harder than the run ahead did;
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
        public static double PlaningBuoyancy = 0.5;   // share of the hydrostatic force kept once planing (1: all of it)
        public static double SeparationDrop = 0.1;    // a section's m_a V this share below the run's peak: flow separated
        // a mesh whose area-weighted normals sum to more than this share of its area is not closed: its hydrostatic
        // pressure would not add up to a buoyancy, and stock's stays
        public const double ClosureTolerance = 0.02;
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
            public bool Separated;       // its m_a V well below the run's peak: the flow has left the bottom
        }

        // the mesh is closed: its hydrostatic pressure is the part's buoyancy, which replaces stock's (part.buoyancy 0)
        public bool OwnsBuoyancy { get; private set; }
        public double MeshVolume { get; private set; }
        private float stockBuoyancy = -1;

        // what the last step did, for tests and display (kN, m2, m). LastHydrostaticForce: the part's buoyancy when the
        // mesh is closed, else the planing cut to stock's
        public Vector3 LastForce, LastMomentumForce, LastFrictionForce, LastImpactForce, LastCrossflowForce, LastHydrostaticForce;
        public double LastWettedArea, LastPlanform, LastWettedLength, LastLimiter = 1;
        public int LastStrips;

        // every strip of the last step, bow first, when RecordStrips is set (tests): its position along the flow from the
        // wetted bow (m), wetted width (m), added mass per length (kg/m), normal velocity (m/s), momentum jump and the
        // load it carries (N), whether separated, and the load's point (world)
        public static bool RecordStrips;
        public readonly List<(double x, double width, double ma, double v, double jump, double load, bool separated, Vector3 point)> LastStripData =
            new List<(double, double, double, double, double, double, bool, Vector3)>();

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

            // closed and outward: the area vectors cancel and the enclosed volume is positive
            Vector3 areaSum = Vector3.zero;
            double area = 0, volume = 0;
            for (int i = 0; i + 2 < t.Count; i += 3)
            {
                Vector3 a = verts[t[i]], b = verts[t[i + 1]], c = verts[t[i + 2]];
                Vector3 n2 = Vector3.Cross(b - a, c - a);   // twice the area, outward
                areaSum += n2 * 0.5f;
                area += n2.magnitude * 0.5;
                volume += Vector3.Dot(a, Vector3.Cross(b, c)) / 6.0;
            }
            MeshVolume = volume;
            var bounds = new Bounds(verts.Count > 0 ? verts[0] : Vector3.zero, Vector3.zero);
            foreach (Vector3 w in verts)
                bounds.Encapsulate(w);
            log.Append($"; part-space bounds {bounds.min:F3} to {bounds.max:F3}");
            OwnsBuoyancy = area > 0 && volume > 0 && areaSum.magnitude < ClosureTolerance * area;
            if (!OwnsBuoyancy)
                RestoreStockBuoyancy();
            FARLogger.Info($"Hydrodynamics of {part.partInfo?.name}: {t.Count / 3} triangles from meshes{log}; " +
                           $"volume {volume:F3} m3, open {(area > 0 ? areaSum.magnitude / area : 1):F4} of its area: " +
                           (OwnsBuoyancy ? "buoyancy from its pressure" : "stock buoyancy"));
            localVerts = verts.ToArray();
            tris = t.ToArray();
            worldVerts = new Vector3[localVerts.Length];
            depths = new float[localVerts.Length];
            meshSource = meshes;
            return tris.Length > 0;
        }

        /// <summary>Gives the part its stock buoyancy back (the model off, or the mesh not closed).</summary>
        public void RestoreStockBuoyancy()
        {
            if (stockBuoyancy >= 0)
                part.buoyancy = stockBuoyancy;
            stockBuoyancy = -1;
        }

        private void Clear()
        {
            LastForce = LastMomentumForce = LastFrictionForce = LastImpactForce = LastCrossflowForce = LastHydrostaticForce = Vector3.zero;
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

            if (OwnsBuoyancy && stockBuoyancy < 0)
            {
                stockBuoyancy = part.buoyancy;
                part.buoyancy = 0;
            }
            double g = FlightGlobals.getGeeForceAtPosition((Vector3d)p0).magnitude;
            Vector3 com = rb.worldCenterOfMass, hsForce = Vector3.zero, hsTorque = Vector3.zero;

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
                if (OwnsBuoyancy)
                {
                    // hydrostatic pressure rho g depth on the wet piece, integrated exactly: linear over each fan
                    // triangle, so its force is A mean(p) and its first moment A/12 (sum p_i r_i + sum p_i sum r_i)
                    // (the centroid would put a long, sloping piece's force too far up it); on a closed surface it
                    // sums to the displaced water's weight at the center of buoyancy
                    Vector3 o = scratchA[0];
                    float po = (float)Math.Max(0, depth0 - Vector3.Dot(o - p0, up));
                    for (int j = 1; j + 1 < scratchA.Count; j++)
                    {
                        Vector3 q1 = scratchA[j], q2 = scratchA[j + 1];
                        float ta = 0.5f * Vector3.Dot(Vector3.Cross(q1 - o, q2 - o), n);
                        if (ta <= 0)
                            continue;
                        float p1 = (float)Math.Max(0, depth0 - Vector3.Dot(q1 - p0, up));
                        float p2 = (float)Math.Max(0, depth0 - Vector3.Dot(q2 - p0, up));
                        float ps = po + p1 + p2;
                        if (ps <= 0)
                            continue;
                        Vector3 ro = o - com, r1 = q1 - com, r2 = q2 - com;
                        Vector3 moment = (po * ro + p1 * r1 + p2 * r2 + ps * (ro + r1 + r2)) * (ta / 12f);  // integral of depth * r
                        float scale = (float)(rho * g);
                        hsForce += -n * (scale * ta * ps / 3f);
                        hsTorque += Vector3.Cross(moment * scale, -n);
                    }
                }
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
            double maPrevV = 0, maVPeak = 0, stiffness = 0;
            Vector3 crossflow = Vector3.zero;
            double ka = AddedMassCoefficient * rho * Math.PI / 2;
            for (int k = nStrips - 1; k >= 0; k--)
            {
                ref Strip s = ref strips[k];
                if (s.Planform <= 0)
                {
                    maPrevV = maVPeak = 0;
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
                maVPeak = Math.Max(maVPeak, maV);
                s.Separated = maVPeak > 0 && maV < (1 - SeparationDrop) * maVPeak;
                if (s.Separated)
                {
                    // in the wake: friction and hydrostatics only, and a push into the water
                    if (s.V > 0)
                    {
                        double fin = 0.5 * CrossflowDragIn * rho * s.Width * dx * s.V * s.V;
                        stiffness += CrossflowDragIn * rho * s.Width * dx * s.V;
                        s.Load = fin;
                        crossflow += -s.Normal * (float)fin;
                    }
                    continue;
                }
                s.Jump = U * (maV - maPrevV);
                maPrevV = maV;
                stiffness = Math.Max(stiffness, U * ma);
                double cd = s.V > 0 ? CrossflowDragIn : CrossflowDragOut;
                double fcf = 0.5 * cd * rho * s.Width * dx * s.V * Math.Abs(s.V);
                stiffness += cd * rho * s.Width * dx * Math.Abs(s.V);
                s.Load = fcf;
                crossflow += -s.Normal * (float)fcf;
            }

            // spread each jump aft over the rest of its wetted run with the flat-plate loading, to where the flow leaves
            for (int k = nStrips - 1; k >= 0; k--)
            {
                if (strips[k].Planform <= 0 || strips[k].Jump == 0)
                    continue;
                int end = k;
                while (end - 1 >= 0 && strips[end - 1].Planform > 0 && !strips[end - 1].Separated)
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
            if (RecordStrips)
            {
                LastStripData.Clear();
                for (int k = nStrips - 1; k >= 0; k--)
                {
                    Strip s = strips[k];
                    if (s.Planform > 0)
                        LastStripData.Add(((nStrips - 1 - k + 0.5) * dx, s.Width, ka * s.Width * s.Width / 4, s.V, s.Jump, s.Load * limiter, s.Separated, s.Point));
                }
            }
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
            // pressure (Savitsky's buoyancy term is about half the displacement's; Zarnick scales it by 0.5): the
            // share kept fades from all of it to PlaningBuoyancy from C_V 1 to 3 on the mean wetted beam
            double take = 0;
            if (PlaningBuoyancy < 1 && planformTotal > 0 && wettedLength > 0)
            {
                double beam = planformTotal / wettedLength;
                double cv = U / Math.Sqrt(g * Math.Max(0.05, beam));
                double x = Math.Max(0, Math.Min(1, (cv - 1) / 2));
                take = (1 - PlaningBuoyancy) * x * x * (3 - 2 * x);
            }
            Vector3 hydrostatic = Vector3.zero;
            PartBuoyancy pb = part.partBuoyancy;
            if (OwnsBuoyancy)
            {
                // the part's own buoyancy: its pressure, times its cfg buoyancy (stock's buoyancyScalar left out)
                float keep = (float)((1 - take) * (stockBuoyancy >= 0 ? stockBuoyancy : 1));
                hydrostatic = hsForce * (keep * 0.001f);
                part.AddForce((Vector3d)hydrostatic);
                part.AddTorque((Vector3d)(hsTorque * (keep * 0.001f)));
            }
            else if (take > 0 && pb != null)
            {
                hydrostatic = -pb.lastBuoyantForce * (float)take;
                part.AddForceAtPosition((Vector3d)hydrostatic, (Vector3d)pb.lastForcePosition);
            }
            LastHydrostaticForce = hydrostatic;

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
