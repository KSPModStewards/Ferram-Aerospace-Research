using System;
using System.Collections.Generic;
using System.Diagnostics;
using FerramAerospaceResearch.FARPartGeometry;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    /// <summary>
    /// Water forces on a vessel's wet parts from their geometry, for the parts of them under the water plane, by strip
    /// (added-mass momentum) theory as used for planing hulls (Zarnick 1978). Each part clips its own mesh at the water
    /// (Gather); the parts that make one hull (wet bottoms in the same lane of water, touching along the flow: HullSolver)
    /// share one set of strips, so a hull built of several parts runs as the same hull in one part does:
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
    ///   the water harder than the run ahead did. Once the flow ventilates the step, past the hump (from C_V
    ///   VentStartCv to VentFullCv), it carries no hydrostatic pressure either: it is in the hollow the bottom ahead
    ///   leaves, not under the level water;
    /// - crossflow drag 1/2 Cd rho w V|V| per strip (damping when U is small);
    /// - impact pressure 1/2 Cp rho vn^2 on steep faces moving into the water (a blunt bow);
    /// - skin friction (ITTC-57) on the wetted area.
    /// A part whose mesh is closed takes its buoyancy from the water pressure on it; others keep stock's. Faces where
    /// two parts meet inside the hull (a face whose outside lies in another wet part) are not wetted: they take no
    /// momentum, impact or friction, only their hydrostatic pressure, which the other part's face cancels.
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
        // the step ventilates past the hump: in the volume Froude number V / sqrt(g vol^1/3), which takes out the load,
        // NACA TN 470's Model 11-A humps at 2.7 - 3.0 and TM 826's float planes clear at 4.1; on the mean wetted beam
        // (C_V = F_vol C_Delta^1/6, C_Delta 0.5 - 1.8 for floats and hulls) 3.5 to 5
        public static double VentStartCv = 3.5, VentFullCv = 5.0;
        /// <summary>
        /// Stock's water drag and angular drag on the wet parts' rigidbodies (PartBuoyancy, FlightIntegrator), on top of
        /// this model's own damping: as stock applies them (Unity's rigidbody damping, which on a vessel of several parts
        /// puts a moment on it: it damps each part's share of the forces the joints pass on), as forces from each part's
        /// motion (Explicit: the same damping, none at rest), or none (the rigidbodies keep the air's angular drag).
        /// </summary>
        public enum WaterDampingMode { Stock, Explicit, None }
        public static WaterDampingMode WaterDamping = WaterDampingMode.Stock;
        // a mesh whose area-weighted normals sum to more than this share of its area is not closed: its hydrostatic
        // pressure would not add up to a buoyancy, and stock's stays
        public const double ClosureTolerance = 0.02;
        public const int MinStrips = 4, MaxStrips = 40;
        // a face this far (m) out along its normal inside another part: the two parts meet there
        public const float ContactProbe = 0.005f;

        /// <summary>The meshes the water sees: FAR's (what its voxels are made from: colliders or visible meshes, by
        /// FAR's own choice per part), the part's colliders, or its visible meshes.</summary>
        public enum MeshSourceKind { Far, Colliders, Visual }
        public static MeshSourceKind MeshSource = MeshSourceKind.Far;

        // time spent (Stopwatch ticks) and work done since the counters were last reset (tests, benchmarks)
        public static long ProfileGatherTicks, ProfileSolveTicks, ProfileContactTicks, ProfileMeshTicks;
        public static long ProfileSteps, ProfileTrianglesTested, ProfilePolygons, ProfileContactPairs;
        public static void ResetProfile() =>
            ProfileGatherTicks = ProfileSolveTicks = ProfileContactTicks = ProfileMeshTicks =
                ProfileSteps = ProfileTrianglesTested = ProfilePolygons = ProfileContactPairs = 0;

        internal readonly Part part;
        private object meshSource;
        private MeshSourceKind meshKind;
        private int meshVersion;
        private Vector3[] localVerts = Array.Empty<Vector3>();
        private int[] tris = Array.Empty<int>();
        private Bounds localBounds;
        private Vector3[] worldVerts = Array.Empty<Vector3>();
        private float[] depths = Array.Empty<float>();

        // the triangles that meet another part, per other part (indices; null: none), and the stamp marking this
        // step's union of them
        private readonly Dictionary<PartHydrodynamics, (int version, int otherVersion, int[] tris)> contacts =
            new Dictionary<PartHydrodynamics, (int, int, int[])>();
        private int[] contactStamp = Array.Empty<int>();
        private int stamp;

        // clipped submerged polygons (not contact faces): their vertices, counts, and source triangle normals
        internal readonly List<Vector3> PolyVerts = new List<Vector3>();
        internal readonly List<int> PolyCounts = new List<int>();
        internal readonly List<Vector3> PolyNormals = new List<Vector3>();
        private readonly List<Vector3> scratchA = new List<Vector3>(8);
        // each wet fan triangle's hydrostatic force and moment about the part's CoM, and its centroid
        internal readonly List<Vector3> HsForces = new List<Vector3>(), HsTorques = new List<Vector3>(), HsPoints = new List<Vector3>();
        internal double Gravity;
        internal Vector3 Origin;

        // the mesh is closed: its hydrostatic pressure is the part's buoyancy, which replaces stock's (part.buoyancy 0)
        public bool OwnsBuoyancy { get; private set; }
        public double MeshVolume { get; private set; }
        public int MeshTriangles => tris.Length / 3;
        internal float StockBuoyancy = -1;
        // stock's water drag and angular drag taken off the rigidbody for the hull to apply (WaterDampingMode.Explicit; -1: none)
        internal float DampDrag = -1, DampAngular;

        // what the last step did, for tests and display (kN, m2, m): this part's share of its hull's forces.
        // LastHydrostaticForce: the part's buoyancy when the mesh is closed, else the planing cut to stock's.
        // LastPlanform, LastWettedLength, LastLimiter, LastStrips: its hull's
        public Vector3 LastForce, LastMomentumForce, LastFrictionForce, LastImpactForce, LastCrossflowForce, LastHydrostaticForce;
        public Vector3 LastHydrostaticTorque;   // kN m, about the part's CoM (the closed mesh's pressure)
        public Vector3 LastTorque;              // kN m, LastForce's about the part's CoM
        public double LastWettedArea, LastPlanform, LastWettedLength, LastLimiter = 1;
        public int LastStrips;
        /// <summary>The hull this part was part of in the last step (null: dry).</summary>
        public HullResult Hull { get; internal set; }

        // every strip of the last step, bow first, when RecordStrips is set (tests): its position along the flow from the
        // wetted bow (m), wetted width (m), added mass per length (kg/m), normal velocity (m/s), momentum jump and the
        // load it carries (N), whether separated, and the load's point (world)
        public static bool RecordStrips;
        private static readonly List<(double x, double width, double ma, double v, double jump, double load, bool separated, Vector3 point)> NoStrips =
            new List<(double, double, double, double, double, double, bool, Vector3)>();
        public List<(double x, double width, double ma, double v, double jump, double load, bool separated, Vector3 point)> LastStripData =>
            Hull?.StripData ?? NoStrips;

        public PartHydrodynamics(Part part)
        {
            this.part = part;
        }

        private bool EnsureMesh()
        {
            GeometryPartModule g = part.GetComponent<GeometryPartModule>();
            List<GeometryMesh> meshes = g != null ? g.meshDataList : null;
            object source = MeshSource == MeshSourceKind.Far ? (object)meshes : part.partTransform;
            if (source == null || MeshSource == MeshSourceKind.Far && meshes.Count == 0)
                return false;
            if (ReferenceEquals(source, meshSource) && meshKind == MeshSource)
                return tris.Length > 0;

            long t0 = Stopwatch.GetTimestamp();
            var verts = new List<Vector3>();
            var t = new List<int>();
            var log = new System.Text.StringBuilder();
            if (MeshSource == MeshSourceKind.Far)
                FarMeshes(meshes, verts, t, log);
            else
                OwnMeshes(MeshSource == MeshSourceKind.Colliders, verts, t, log);

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
            localBounds = bounds;
            log.Append($"; part-space bounds {bounds.min:F3} to {bounds.max:F3}");
            OwnsBuoyancy = area > 0 && volume > 0 && areaSum.magnitude < ClosureTolerance * area;
            if (!OwnsBuoyancy)
                RestoreStockBuoyancy();
            FARLogger.Info($"Hydrodynamics of {part.partInfo?.name}: {t.Count / 3} triangles from {MeshSource} meshes{log}; " +
                           $"volume {volume:F3} m3, open {(area > 0 ? areaSum.magnitude / area : 1):F4} of its area: " +
                           (OwnsBuoyancy ? "buoyancy from its pressure" : "stock buoyancy"));
            localVerts = verts.ToArray();
            tris = t.ToArray();
            worldVerts = new Vector3[localVerts.Length];
            depths = new float[localVerts.Length];
            contactStamp = new int[tris.Length / 3];
            contacts.Clear();
            meshSource = source;
            meshKind = MeshSource;
            meshVersion++;
            ProfileMeshTicks += Stopwatch.GetTimestamp() - t0;
            return tris.Length > 0;
        }

        private static void FarMeshes(List<GeometryMesh> meshes, List<Vector3> verts, List<int> t, System.Text.StringBuilder log)
        {
            var seen = new List<(int verts, int tris, Bounds bounds)>();
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
                AddMesh(m.MeshLocalVerts, m.triangles, toPart, verts, t);
            }
        }

        // the part's own colliders (mesh and box) or visible meshes (active, rendered, readable), in part space
        private void OwnMeshes(bool colliders, List<Vector3> verts, List<int> t, System.Text.StringBuilder log)
        {
            Matrix4x4 toPart = part.partTransform.worldToLocalMatrix;
            if (colliders)
            {
                foreach (Collider c in part.GetComponentsInChildren<Collider>(false))
                {
                    if (!c.enabled || c.isTrigger || c is WheelCollider)
                        continue;
                    Matrix4x4 m = toPart * c.transform.localToWorldMatrix;
                    if (c is MeshCollider mc && mc.sharedMesh != null && mc.sharedMesh.isReadable)
                    {
                        AddMesh(mc.sharedMesh.vertices, mc.sharedMesh.triangles, m, verts, t);
                        log.Append($" {c.name}({mc.sharedMesh.triangles.Length / 3})");
                    }
                    else if (c is BoxCollider bc)
                    {
                        MeshData box = GeometryPartModule.CreateBoxMeshFromBoxCollider(bc.size, bc.center);
                        AddMesh(box.vertices, box.triangles, m, verts, t);
                        log.Append($" {c.name}(box)");
                    }
                    else
                        log.Append($" {c.name}({c.GetType().Name} skipped)");
                }
                return;
            }
            foreach (MeshFilter mf in part.GetComponentsInChildren<MeshFilter>(false))
            {
                MeshRenderer mr = mf.GetComponent<MeshRenderer>();
                if (mr == null || !mr.enabled || mf.sharedMesh == null || !mf.sharedMesh.isReadable)
                    continue;
                AddMesh(mf.sharedMesh.vertices, mf.sharedMesh.triangles, toPart * mf.transform.localToWorldMatrix, verts, t);
                log.Append($" {mf.name}({mf.sharedMesh.triangles.Length / 3})");
            }
        }

        private static void AddMesh(Vector3[] mv, int[] mt, Matrix4x4 toPart, List<Vector3> verts, List<int> t)
        {
            bool flip = toPart.determinant < 0;
            int offset = verts.Count;
            foreach (Vector3 v in mv)
                verts.Add(toPart.MultiplyPoint3x4(v));
            for (int i = 0; i + 2 < mt.Length; i += 3)
            {
                t.Add(offset + mt[i]);
                t.Add(offset + (flip ? mt[i + 2] : mt[i + 1]));
                t.Add(offset + (flip ? mt[i + 1] : mt[i + 2]));
            }
        }

        /// <summary>Gives the part its stock buoyancy back (the model off, or the mesh not closed).</summary>
        public void RestoreStockBuoyancy()
        {
            if (StockBuoyancy >= 0)
                part.buoyancy = StockBuoyancy;
            StockBuoyancy = -1;
        }

        internal void Clear()
        {
            LastForce = LastMomentumForce = LastFrictionForce = LastImpactForce = LastCrossflowForce = LastHydrostaticForce = LastHydrostaticTorque = LastTorque = Vector3.zero;
            LastWettedArea = LastPlanform = LastWettedLength = 0;
            LastStrips = 0;
            LastLimiter = 1;
            Hull = null;
            PolyVerts.Clear();
            PolyCounts.Clear();
            PolyNormals.Clear();
            HsForces.Clear();
            HsTorques.Clear();
            HsPoints.Clear();
        }

        /// <summary>Clears the last step's results, and says whether the part can be wet this step.</summary>
        internal bool Prepare()
        {
            Clear();
            Vessel vessel = part.vessel;
            if (part.rb == null || vessel == null || part.submergedPortion <= 0)
                return false;
            CelestialBody body = vessel.mainBody;
            return body != null && body.ocean && EnsureMesh();
        }

        // the triangles of this part whose outside lies in the other part (they meet there), computed once per pair:
        // the parts are joined, so they keep their places
        private int[] ContactsWith(PartHydrodynamics other)
        {
            if (contacts.TryGetValue(other, out var c) && c.version == meshVersion && c.otherVersion == other.meshVersion)
                return c.tris;
            long t0 = Stopwatch.GetTimestamp();
            int[] found = null;
            if (other.OwnsBuoyancy)
            {
                Matrix4x4 toOther = other.part.partTransform.worldToLocalMatrix * part.partTransform.localToWorldMatrix;
                Bounds ob = other.localBounds;
                ob.Expand(2 * ContactProbe);
                if (ob.Intersects(GeometryMesh.TransformBounds(localBounds, toOther)))
                {
                    var list = new List<int>();
                    for (int i = 0; i + 2 < tris.Length; i += 3)
                    {
                        Vector3 a = localVerts[tris[i]], b = localVerts[tris[i + 1]], cc = localVerts[tris[i + 2]];
                        Vector3 n = Vector3.Cross(b - a, cc - a);
                        if (n.sqrMagnitude < 1e-12f)
                            continue;
                        Vector3 q = toOther.MultiplyPoint3x4((a + b + cc) / 3 + n.normalized * ContactProbe);
                        if (ob.Contains(q) && other.Inside(q))
                            list.Add(i / 3);
                    }
                    if (list.Count > 0)
                        found = list.ToArray();
                }
            }
            contacts[other] = (meshVersion, other.meshVersion, found);
            ProfileContactPairs++;
            ProfileContactTicks += Stopwatch.GetTimestamp() - t0;
            return found;
        }

        // a point in part space inside the closed mesh: a ray's crossings are odd
        private bool Inside(Vector3 q)
        {
            var dir = new Vector3(0.3713907f, 0.5570860f, 0.7427813f);
            int crossings = 0;
            for (int i = 0; i + 2 < tris.Length; i += 3)
            {
                Vector3 a = localVerts[tris[i]], e1 = localVerts[tris[i + 1]] - a, e2 = localVerts[tris[i + 2]] - a;
                Vector3 p = Vector3.Cross(dir, e2);
                float det = Vector3.Dot(e1, p);
                if (Math.Abs(det) < 1e-12f)
                    continue;
                float inv = 1 / det;
                Vector3 s = q - a;
                float u = Vector3.Dot(s, p) * inv;
                if (u < 0 || u > 1)
                    continue;
                Vector3 qv = Vector3.Cross(s, e1);
                float v = Vector3.Dot(dir, qv) * inv;
                if (v < 0 || u + v > 1)
                    continue;
                if (Vector3.Dot(e2, qv) * inv > 0)
                    crossings++;
            }
            return (crossings & 1) == 1;
        }

        /// <summary>
        /// Clips the part's mesh at the water: its wet polygons (contact faces left out) for the hull's strips, and the
        /// hydrostatic pressure on every wet piece (contact faces too: the other part's face cancels them). wet: the
        /// other wet parts of the vessel. Returns whether anything is under the water.
        /// </summary>
        internal bool Gather(List<PartHydrodynamics> wet)
        {
            long t0 = Stopwatch.GetTimestamp();
            Vessel vessel = part.vessel;
            CelestialBody body = vessel.mainBody;
            double rho = body.oceanDensity * 1000;
            Transform tf = part.partTransform;
            Vector3 p0 = tf.position;
            Origin = p0;
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
            {
                ProfileGatherTicks += Stopwatch.GetTimestamp() - t0;
                return false;
            }

            // the faces that meet another wet part this step
            stamp++;
            bool anyContact = false;
            foreach (PartHydrodynamics o in wet)
            {
                if (ReferenceEquals(o, this))
                    continue;
                int[] c = ContactsWith(o);
                if (c == null)
                    continue;
                anyContact = true;
                foreach (int k in c)
                    contactStamp[k] = stamp;
            }

            if (OwnsBuoyancy && StockBuoyancy < 0)
            {
                StockBuoyancy = part.buoyancy;
                part.buoyancy = 0;
            }
            Gravity = FlightGlobals.getGeeForceAtPosition((Vector3d)p0).magnitude;
            Vector3 com = part.rb.worldCenterOfMass;
            float scale = (float)(rho * Gravity);

            for (int i = 0; i + 2 < tris.Length; i += 3)
            {
                int a = tris[i], b = tris[i + 1], c = tris[i + 2];
                if (depths[a] <= 0 && depths[b] <= 0 && depths[c] <= 0)
                    continue;
                ProfileTrianglesTested++;
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
                        HsForces.Add(-n * (scale * ta * ps / 3f));
                        HsTorques.Add(Vector3.Cross(moment * scale, -n));
                        HsPoints.Add((o + q1 + q2) / 3f);
                    }
                }
                if (anyContact && contactStamp[i / 3] == stamp)
                    continue;
                PolyVerts.AddRange(scratchA);
                PolyCounts.Add(scratchA.Count);
                PolyNormals.Add(n);
            }
            ProfilePolygons += PolyCounts.Count;
            ProfileGatherTicks += Stopwatch.GetTimestamp() - t0;
            return PolyCounts.Count > 0 || HsForces.Count > 0;
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
    }

    /// <summary>A hull's last step: the parts of it, its wetted length and planform, the limiter, its strips.</summary>
    internal sealed class HullResult
    {
        public readonly List<PartHydrodynamics> Parts = new List<PartHydrodynamics>();
        public double WettedLength, Planform, Limiter = 1;
        public int Strips;
        public readonly List<(double x, double width, double ma, double v, double jump, double load, bool separated, Vector3 point)> StripData =
            new List<(double, double, double, double, double, double, bool, Vector3)>();
    }

    /// <summary>
    /// A vessel's water forces for one physics step: its wet parts gathered (FARAeroPartModule.ApplyForces), grouped
    /// into hulls, and each hull's strips solved and its forces put on its parts.
    /// </summary>
    internal sealed class VesselHydrodynamics
    {
        private const double KinematicViscosity = 1.19e-6;
        private readonly List<PartHydrodynamics> pending = new List<PartHydrodynamics>();
        private readonly List<PartHydrodynamics> wet = new List<PartHydrodynamics>();
        private readonly List<HullResult> hulls = new List<HullResult>();
        private int[] groupOf = Array.Empty<int>();
        private float[] range = Array.Empty<float>();   // per wet part: x min, x max, y min, y max in the vessel's flow frame

        private Strip[] strips = new Strip[PartHydrodynamics.MaxStrips];
        private readonly List<Vector3> scratchA = new List<Vector3>(8), scratchB = new List<Vector3>(8);

        private struct Share
        {
            public int Part;
            public double Planform;
            public Vector3 CentroidSum;
            public Vector3 Friction, FrictionPoint;
            public double FrictionWeight, WettedArea;
        }

        private struct Strip
        {
            public List<Vector2> Spans;  // lateral extents of its downward-facing pieces, merged into wetted widths
            public Share[] Shares;       // each part's piece of it
            public int ShareCount;
            public double Planform;      // downward-facing area projected on the water plane
            public Vector3 NormalSum;    // area-weighted outward normals of the downward-facing pieces
            public Vector3 CentroidSum;  // planform-weighted centroids
            public Vector3 VelocitySum;  // planform-weighted point velocities
            public double Jump;          // U * d(m_a V) at this strip (N)
            public double Load;          // the normal force it ends up carrying (N)
            public double Crossflow;     // the crossflow share of Load (N)
            public Vector3 Normal, Point;
            public double V, Width;
            public bool Separated;       // its m_a V well below the run's peak: the flow has left the bottom
        }

        public void Add(PartHydrodynamics hd) => pending.Add(hd);

        /// <summary>Solves and applies this step's water forces on the parts added since the last call.</summary>
        public void Solve(Vessel vessel)
        {
            if (pending.Count == 0)
                return;
            long t0 = Stopwatch.GetTimestamp();
            wet.Clear();
            foreach (PartHydrodynamics hd in pending)
                if (hd.Prepare())
                    wet.Add(hd);
            pending.Clear();
            if (wet.Count == 0)
                return;
            PartHydrodynamics.ProfileSteps++;
            long g0 = PartHydrodynamics.ProfileGatherTicks;
            for (int i = wet.Count - 1; i >= 0; i--)
                if (!wet[i].Gather(wet))
                    wet.RemoveAt(i);
            long gathered = PartHydrodynamics.ProfileGatherTicks - g0;
            if (wet.Count > 0)
            {
                Group(vessel);
                foreach (HullResult h in hulls)
                {
                    SolveHull(h, vessel);
                    ApplyHullDamping(h);
                }
            }
            PartHydrodynamics.ProfileSolveTicks += Stopwatch.GetTimestamp() - t0 - gathered;
        }

        // hulls: wet parts whose wet surfaces share a lane of water (overlap across the flow) and touch along it
        private void Group(Vessel vessel)
        {
            int n = wet.Count;
            if (groupOf.Length < n)
            {
                groupOf = new int[n];
                range = new float[4 * n];
            }
            Vector3 frame = Krakensbane.GetFrameVelocity();
            Vector3 up = (Vector3)vessel.upAxis;
            Vector3 vel = Vector3.zero;
            double m = 0;
            foreach (PartHydrodynamics hd in wet)
            {
                vel += hd.part.rb.velocity * hd.part.rb.mass;
                m += hd.part.rb.mass;
            }
            vel = m > 0 ? vel / (float)m + frame : frame;
            Vector3 xHat = Vector3.ProjectOnPlane(vel, up);
            xHat = xHat.sqrMagnitude > 0.0025f ? xHat.normalized : Vector3.ProjectOnPlane(vessel.transform.up, up).normalized;
            if (xHat.sqrMagnitude < 0.5f)
                xHat = Vector3.ProjectOnPlane(vessel.transform.forward, up).normalized;
            Vector3 yHat = Vector3.Cross(up, xHat);
            Vector3 o = (Vector3)vessel.CoMD;
            for (int i = 0; i < n; i++)
            {
                groupOf[i] = i;
                float x0 = float.MaxValue, x1 = float.MinValue, y0 = float.MaxValue, y1 = float.MinValue;
                foreach (Vector3 p in wet[i].PolyVerts)
                {
                    float x = Vector3.Dot(p - o, xHat), y = Vector3.Dot(p - o, yHat);
                    x0 = Math.Min(x0, x);
                    x1 = Math.Max(x1, x);
                    y0 = Math.Min(y0, y);
                    y1 = Math.Max(y1, y);
                }
                range[4 * i] = x0;
                range[4 * i + 1] = x1;
                range[4 * i + 2] = y0;
                range[4 * i + 3] = y1;
            }
            float xTol = (float)PartHydrodynamics.StripLength, yTol = 0.02f;
            for (int i = 0; i < n; i++)
            {
                if (wet[i].PolyCounts.Count == 0)
                    continue;
                for (int j = i + 1; j < n; j++)
                {
                    if (wet[j].PolyCounts.Count == 0)
                        continue;
                    if (range[4 * i] > range[4 * j + 1] + xTol || range[4 * j] > range[4 * i + 1] + xTol ||
                        range[4 * i + 2] > range[4 * j + 3] + yTol || range[4 * j + 2] > range[4 * i + 3] + yTol)
                        continue;
                    int a = Root(i), b = Root(j);
                    if (a != b)
                        groupOf[Math.Max(a, b)] = Math.Min(a, b);
                }
            }
            int used = 0;
            for (int i = 0; i < n; i++)
            {
                int r = Root(i);
                if (r != i)
                    continue;
                if (hulls.Count <= used)
                    hulls.Add(new HullResult());
                HullResult h = hulls[used++];
                h.Parts.Clear();
                for (int j = i; j < n; j++)
                    if (Root(j) == i)
                        h.Parts.Add(wet[j]);
            }
            hulls.RemoveRange(used, hulls.Count - used);
        }

        private int Root(int i)
        {
            while (groupOf[i] != i)
                i = groupOf[i] = groupOf[groupOf[i]];
            return i;
        }

        private void SolveHull(HullResult hull, Vessel vessel)
        {
            List<PartHydrodynamics> parts = hull.Parts;
            PartHydrodynamics first = parts[0];
            foreach (PartHydrodynamics hd in parts)
                hd.Hull = hull;
            hull.WettedLength = hull.Planform = 0;
            hull.Limiter = 1;
            hull.Strips = 0;
            hull.StripData.Clear();

            CelestialBody body = vessel.mainBody;
            double rho = body.oceanDensity * 1000;
            Vector3 p0 = first.Origin;
            Vector3 up = (Vector3)(((Vector3d)p0 - body.position).normalized);
            Vector3 frame = Krakensbane.GetFrameVelocity();
            // the flow: the hull's own (its parts' mass-weighted velocity)
            Vector3 vCom = Vector3.zero;
            float hullMass = 0;
            foreach (PartHydrodynamics hd in parts)
            {
                vCom += hd.part.rb.velocity * hd.part.rb.mass;
                hullMass += hd.part.rb.mass;
            }
            vCom = (hullMass > 0 ? vCom / hullMass : first.part.rb.velocity) + frame;
            Vector3 vh = vCom - up * Vector3.Dot(vCom, up);
            float U = vh.magnitude;
            Transform tf = first.part.partTransform;
            Vector3 xHat = U > 0.05f ? vh / U : Vector3.ProjectOnPlane(tf.up, up).normalized;
            if (xHat.sqrMagnitude < 0.5f)
                xHat = Vector3.ProjectOnPlane(tf.forward, up).normalized;

            float xMin = float.MaxValue, xMax = float.MinValue;
            foreach (PartHydrodynamics hd in parts)
                foreach (Vector3 v in hd.PolyVerts)
                {
                    float x = Vector3.Dot(v - p0, xHat);
                    xMin = Math.Min(xMin, x);
                    xMax = Math.Max(xMax, x);
                }
            if (xMax - xMin < 1e-4f)
            {
                // no wetted bottom to strip (only contact faces, or a sliver): the parts' buoyancy alone
                foreach (PartHydrodynamics hd in parts)
                    if (hd.OwnsBuoyancy && hd.HsForces.Count > 0)
                    {
                        Vector3 hsF = Vector3.zero, hsT = Vector3.zero;
                        for (int i = 0; i < hd.HsForces.Count; i++)
                        {
                            hsF += hd.HsForces[i];
                            hsT += hd.HsTorques[i];
                        }
                        float keep = (hd.StockBuoyancy >= 0 ? hd.StockBuoyancy : 1) * 0.001f;
                        hd.LastHydrostaticForce = hsF * keep;
                        hd.part.AddForce((Vector3d)hd.LastHydrostaticForce);
                        hd.LastHydrostaticTorque = hsT * keep;
                        hd.part.AddTorque((Vector3d)hd.LastHydrostaticTorque);
                    }
                return;
            }

            double wettedLength = xMax - xMin;
            int nStrips = Mathf.Clamp(Mathf.CeilToInt((float)(wettedLength / PartHydrodynamics.StripLength)),
                                      PartHydrodynamics.MinStrips, PartHydrodynamics.MaxStrips);
            float dx = (float)(wettedLength / nStrips);
            Vector3 yHat = Vector3.Cross(up, xHat);
            for (int k = 0; k < nStrips; k++)
            {
                List<Vector2> spans = strips[k].Spans ?? new List<Vector2>();
                Share[] shares = strips[k].Shares ?? new Share[2];
                spans.Clear();
                strips[k] = default;
                strips[k].Spans = spans;
                strips[k].Shares = shares;
            }

            double re = Math.Max(1e5, U * wettedLength / KinematicViscosity);
            double lg = Math.Log10(re) - 2;
            double cf = 0.075 / (lg * lg);

            var impactOf = new Vector3[parts.Count];
            var torqueOf = new Vector3[parts.Count];   // of LastForce, about each part's CoM (N m)
            for (int pi = 0; pi < parts.Count; pi++)
            {
                PartHydrodynamics hd = parts[pi];
                Rigidbody rb = hd.part.rb;
                int at = 0;
                for (int poly = 0; poly < hd.PolyCounts.Count; poly++)
                {
                    int count = hd.PolyCounts[poly];
                    Vector3 n = hd.PolyNormals[poly];
                    float down = -Vector3.Dot(n, up);
                    // the strips this polygon spans
                    float pxMin = float.MaxValue, pxMax = float.MinValue;
                    for (int j = 0; j < count; j++)
                    {
                        float x = Vector3.Dot(hd.PolyVerts[at + j] - p0, xHat);
                        pxMin = Math.Min(pxMin, x);
                        pxMax = Math.Max(pxMax, x);
                    }
                    int k0 = Mathf.Clamp((int)((pxMin - xMin) / dx), 0, nStrips - 1);
                    int k1 = Mathf.Clamp((int)((pxMax - xMin) / dx), 0, nStrips - 1);
                    for (int k = k0; k <= k1; k++)
                    {
                        scratchA.Clear();
                        for (int j = 0; j < count; j++)
                            scratchA.Add(hd.PolyVerts[at + j]);
                        float lo = xMin + k * dx, hi = lo + dx;
                        ClipSlab(scratchA, scratchB, p0, xHat, lo, hi);
                        if (scratchB.Count < 3)
                            continue;
                        PolygonAreaCentroid(scratchB, n, out float area, out Vector3 centroid);
                        if (area <= 0)
                            continue;

                        Vector3 v = rb.GetPointVelocity(centroid) + frame;
                        float vn = Vector3.Dot(v, n);
                        // skin friction against the surface's own slip through the water
                        Vector3 vt = v - n * vn;
                        float vtm = vt.magnitude;
                        ref Strip st = ref strips[k];
                        ref Share sh = ref ShareOf(ref st, pi);
                        sh.WettedArea += area;
                        sh.FrictionWeight += area;
                        sh.FrictionPoint += centroid * area;
                        if (vtm > 1e-3f)
                            sh.Friction += -vt * (float)(0.5 * rho * vtm * cf * area);

                        if (down > 0.3f)
                        {
                            double planform = area * down;
                            st.Planform += planform;
                            st.NormalSum += n * (float)planform;
                            st.CentroidSum += centroid * (float)planform;
                            st.VelocitySum += v * (float)planform;
                            sh.Planform += planform;
                            sh.CentroidSum += centroid * (float)planform;
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
                            Vector3 f = -n * (float)(0.5 * PartHydrodynamics.ImpactPressureCoefficient * rho * vn * vn * area);
                            impactOf[pi] += f;
                            torqueOf[pi] += Vector3.Cross(centroid - rb.worldCenterOfMass, f);
                            hd.part.AddForceAtPosition((Vector3d)f * 0.001, (Vector3d)centroid);
                        }
                    }
                    at += count;
                }
            }

            // momentum: bow (largest x) to stern; a dry strip ends a wetted run (the flow leaves the bottom)
            double maPrevV = 0, maVPeak = 0, stiffness = 0;
            double ka = PartHydrodynamics.AddedMassCoefficient * rho * Math.PI / 2;
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
                s.V = Vector3.Dot(s.VelocitySum / (float)s.Planform, s.Normal);
                double ma = ka * sumHalfWidth2;
                double maV = ma * s.V;
                maVPeak = Math.Max(maVPeak, maV);
                s.Separated = maVPeak > 0 && maV < (1 - PartHydrodynamics.SeparationDrop) * maVPeak;
                if (s.Separated)
                {
                    // in the wake: friction and hydrostatics only, and a push into the water
                    if (s.V > 0)
                    {
                        double fin = 0.5 * PartHydrodynamics.CrossflowDragIn * rho * s.Width * dx * s.V * s.V;
                        stiffness += PartHydrodynamics.CrossflowDragIn * rho * s.Width * dx * s.V;
                        s.Load = s.Crossflow = fin;
                    }
                    continue;
                }
                s.Jump = U * (maV - maPrevV);
                maPrevV = maV;
                stiffness = Math.Max(stiffness, U * ma);
                double cd = s.V > 0 ? PartHydrodynamics.CrossflowDragIn : PartHydrodynamics.CrossflowDragOut;
                double fcf = 0.5 * cd * rho * s.Width * dx * s.V * Math.Abs(s.V);
                stiffness += cd * rho * s.Width * dx * Math.Abs(s.V);
                s.Load = s.Crossflow = fcf;
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
            double mass = Math.Max(vessel.totalMass, first.part.rb.mass) * 1000;
            double limiter = Math.Min(1, 0.5 * mass / Math.Max(1e-9, stiffness * TimeWarp.fixedDeltaTime));

            if (PartHydrodynamics.RecordStrips)
                for (int k = nStrips - 1; k >= 0; k--)
                {
                    Strip s = strips[k];
                    if (s.Planform > 0)
                        hull.StripData.Add(((nStrips - 1 - k + 0.5) * dx, s.Width, ka * s.Width * s.Width / 4, s.V, s.Jump,
                                            s.Load * limiter, s.Separated, s.Point));
                }

            // each strip's load on the parts in it, each its planform's share at its own centroid there (together the
            // strip's load at its point); friction on the wetted area, as much of it as the bottom's merged width says
            // is not a copy
            int wetStrips = 0;
            double planformTotal = 0;
            var momentumOf = new Vector3[parts.Count];
            var crossflowOf = new Vector3[parts.Count];
            var frictionOf = new Vector3[parts.Count];
            var wettedOf = new double[parts.Count];
            for (int k = 0; k < nStrips; k++)
            {
                ref Strip s = ref strips[k];
                if (s.Planform > 0)
                    wetStrips++;
                double dup = s.Planform > 0 ? Math.Min(1, s.Width * dx / s.Planform) : 1;
                planformTotal += s.Planform * dup;
                for (int i = 0; i < s.ShareCount; i++)
                {
                    ref Share sh = ref s.Shares[i];
                    PartHydrodynamics hd = parts[sh.Part];
                    if (s.Planform > 0 && sh.Planform > 0)
                    {
                        float w = (float)(sh.Planform / s.Planform);
                        Vector3 f = -s.Normal * (float)(s.Load * limiter) * w;
                        momentumOf[sh.Part] += f;
                        crossflowOf[sh.Part] += -s.Normal * (float)(s.Crossflow * limiter) * w;
                        Vector3 at = sh.CentroidSum / (float)sh.Planform;
                        torqueOf[sh.Part] += Vector3.Cross(at - hd.part.rb.worldCenterOfMass, f);
                        hd.part.AddForceAtPosition((Vector3d)f * 0.001, (Vector3d)at);
                    }
                    wettedOf[sh.Part] += sh.WettedArea * dup;
                    if (sh.FrictionWeight <= 0)
                        continue;
                    Vector3 ff = sh.Friction * (float)dup;
                    frictionOf[sh.Part] += ff;
                    Vector3 fat = sh.FrictionPoint / (float)sh.FrictionWeight;
                    torqueOf[sh.Part] += Vector3.Cross(fat - hd.part.rb.worldCenterOfMass, ff);
                    hd.part.AddForceAtPosition((Vector3d)ff * 0.001, (Vector3d)fat);
                }
            }

            // hydrostatics at speed: once the flow leaves the transom dry, the bottom near it carries no hydrostatic
            // pressure (Savitsky's buoyancy term is about half the displacement's; Zarnick scales it by 0.5): the
            // share kept fades from all of it to PlaningBuoyancy from C_V 1 to 3 on the mean wetted beam
            double take = 0, vented = 0;
            if (planformTotal > 0 && wettedLength > 0)
            {
                double beam = planformTotal / wettedLength;
                double cv = U / Math.Sqrt(first.Gravity * Math.Max(0.05, beam));
                double x = Math.Max(0, Math.Min(1, (cv - 1) / 2));
                take = (1 - PartHydrodynamics.PlaningBuoyancy) * x * x * (3 - 2 * x);
                double xv = Math.Max(0, Math.Min(1, (cv - PartHydrodynamics.VentStartCv) /
                                                    (PartHydrodynamics.VentFullCv - PartHydrodynamics.VentStartCv)));
                vented = xv * xv * (3 - 2 * xv);
            }
            for (int pi = 0; pi < parts.Count; pi++)
            {
                PartHydrodynamics hd = parts[pi];
                Vector3 hydrostatic = Vector3.zero;
                PartBuoyancy pb = hd.part.partBuoyancy;
                if (hd.OwnsBuoyancy)
                {
                    // the part's own buoyancy: its pressure, times its cfg buoyancy (stock's buoyancyScalar left out); the
                    // separated strips' pressure gone as the step ventilates
                    Vector3 hsForce = Vector3.zero, hsTorque = Vector3.zero;
                    for (int i = 0; i < hd.HsForces.Count; i++)
                    {
                        int k = Mathf.Clamp((int)((Vector3.Dot(hd.HsPoints[i] - p0, xHat) - xMin) / dx), 0, nStrips - 1);
                        float w = strips[k].Separated ? (float)(1 - vented) : 1f;
                        hsForce += hd.HsForces[i] * w;
                        hsTorque += hd.HsTorques[i] * w;
                    }
                    float keep = (float)((1 - take) * (hd.StockBuoyancy >= 0 ? hd.StockBuoyancy : 1));
                    hydrostatic = hsForce * (keep * 0.001f);
                    hd.LastHydrostaticTorque = hsTorque * (keep * 0.001f);
                    hd.part.AddForce((Vector3d)hydrostatic);
                    hd.part.AddTorque((Vector3d)hd.LastHydrostaticTorque);
                }
                else if (take > 0 && pb != null)
                {
                    hydrostatic = -pb.lastBuoyantForce * (float)take;
                    hd.part.AddForceAtPosition((Vector3d)hydrostatic, (Vector3d)pb.lastForcePosition);
                }
                hd.LastHydrostaticForce = hydrostatic;
                hd.LastMomentumForce = (momentumOf[pi] - crossflowOf[pi]) * 0.001f;
                hd.LastCrossflowForce = crossflowOf[pi] * 0.001f;
                hd.LastFrictionForce = frictionOf[pi] * 0.001f;
                hd.LastImpactForce = impactOf[pi] * 0.001f;
                hd.LastTorque = torqueOf[pi] * 0.001f;
                hd.LastForce = (momentumOf[pi] + frictionOf[pi] + impactOf[pi]) * 0.001f;
                hd.LastWettedArea = wettedOf[pi];
                hd.LastPlanform = planformTotal;
                hd.LastWettedLength = wettedLength;
                hd.LastStrips = wetStrips;
                hd.LastLimiter = limiter;
            }
            hull.WettedLength = wettedLength;
            hull.Planform = planformTotal;
            hull.Limiter = limiter;
            hull.Strips = wetStrips;
        }

        // stock's water drag and angular drag (WaterDampingMode.Explicit: the parts' rigidbody values, taken off them) on
        // the hull as one body: its parts' mass-weighted coefficients, the linear one on the hull's CoM velocity, the
        // angular one on its turn about its CoM with its whole inertia there (each part: its own inertia and its mass at
        // its distance), so a hull of several parts is damped as the same hull in one part is
        private static void ApplyHullDamping(HullResult hull)
        {
            if (PartHydrodynamics.WaterDamping != PartHydrodynamics.WaterDampingMode.Explicit)
                return;
            double m = 0, kLin = 0, kAng = 0;
            Vector3 c = Vector3.zero, v = Vector3.zero, w = Vector3.zero;
            foreach (PartHydrodynamics hd in hull.Parts)
            {
                Rigidbody rb = hd.part.rb;
                if (rb == null || hd.DampDrag < 0)
                    continue;
                m += rb.mass;
                kLin += rb.mass * hd.DampDrag;
                kAng += rb.mass * hd.DampAngular;
                c += rb.worldCenterOfMass * rb.mass;
                v += rb.velocity * rb.mass;
                w += rb.angularVelocity * rb.mass;
            }
            if (m <= 0)
                return;
            float kMax = 0.5f / Math.Max(1e-4f, TimeWarp.fixedDeltaTime);
            float fm = (float)m;
            c /= fm;
            v /= fm;
            w /= fm;
            float lin = Math.Min((float)(kLin / m), kMax), ang = Math.Min((float)(kAng / m), kMax);
            foreach (PartHydrodynamics hd in hull.Parts)
            {
                Rigidbody rb = hd.part.rb;
                if (rb == null || hd.DampDrag < 0)
                    continue;
                Vector3 r = rb.worldCenterOfMass - c;
                Quaternion pr = rb.rotation * rb.inertiaTensorRotation;
                Vector3 own = pr * Vector3.Scale(Quaternion.Inverse(pr) * w, rb.inertiaTensor);
                hd.part.AddForce((Vector3d)(-(v * lin + Vector3.Cross(w, r) * ang) * rb.mass));
                hd.part.AddTorque((Vector3d)(-own * ang));
            }
        }

        private static ref Share ShareOf(ref Strip s, int part)
        {
            for (int i = 0; i < s.ShareCount; i++)
                if (s.Shares[i].Part == part)
                    return ref s.Shares[i];
            if (s.ShareCount == s.Shares.Length)
                Array.Resize(ref s.Shares, s.Shares.Length * 2);
            s.Shares[s.ShareCount] = new Share { Part = part };
            return ref s.Shares[s.ShareCount++];
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

        private static void ClipSlab(List<Vector3> poly, List<Vector3> into, Vector3 origin, Vector3 axis, float lo, float hi)
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
