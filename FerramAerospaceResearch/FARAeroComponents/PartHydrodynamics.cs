using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
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
    /// A part whose mesh is closed takes its buoyancy from the water pressure on it; others keep stock's. A mesh's open
    /// edges (a B9 wing's pieces leave its root, tip and joins open) are capped for the pressure alone: the pressure on
    /// a closed surface in still water is straight up, on an open one it pushes through the gap. Faces where
    /// two parts meet inside the hull (a face whose outside lies in another wet part) are not wetted: they take no
    /// momentum, impact or friction, only their hydrostatic pressure, which the other part's face cancels.
    /// Foils (FAR wing parts: rudders, keels, dive planes, hydrofoils) are no part of a hull: FAR's wing model gives them
    /// their lift and drag in the water (its density on their wetted share), and they take only their buoyancy here.
    /// A section wholly under the water (a submarine's, or the ends of one awash) is no planing hull's: the water is on
    /// its top as on its bottom, and on its sides. The upward-facing faces make a mirrored set of strips (the same
    /// momentum, separation and crossflow, the flow pushing them down), and so do the faces looking to either side (the
    /// crossflow of a sideslip: their spans up and down, the flow pushing them sideways, in place of the impact pressure
    /// they take at the surface); the planing-only terms (the hydrostatic fade at speed, the ventilated step, impact on
    /// the top and sides) go: section by section, by how far under its top is, from the surface to SubmergedFade of the
    /// hull's deepest section below it. A symmetric body's two sides then cancel level (no lift from its thickness) and
    /// add up to the whole section's added mass at an angle, and a body of revolution yaws as it pitches. Under the water
    /// the flow does not leave the body as it leaves a planing bottom: each strip carries its own U d(m_a V) where it
    /// is (slender-body theory: no separated wake, no spreading aft), so a body of revolution feels Munk's couple, and
    /// where its section shrinks (the afterbody, its boundary layer thickening) only SubmergedRecovery of it comes
    /// back: the lift and the shortfall of the couple a real hull shows (DARPA SUBOFF's bare hull, Roddy 1990). The
    /// strips' loads push across the flow only (their share along it taken off as the section is under): the flow
    /// closes behind the body, and its thickness's pressures are no drag (d'Alembert); its drag is its friction. A
    /// section between the surface and SubmergedFade below it takes both, the planing's as it is at the surface.
    /// </summary>
    internal sealed class PartHydrodynamics
    {
        public static double AddedMassCoefficient = 1.0;
        public static double CrossflowDragIn = 1.0;
        public static double CrossflowDragOut = 0.5;
        public static double ImpactPressureCoefficient = 1.0;
        public static double StripLength = 0.25;
        public static double PlaningBuoyancy = 0.75;  // share of the hydrostatic force kept once planing (1: all of it)
        // the C_V (on the mean wetted beam) over which it fades to PlaningBuoyancy (from 1 to 3: TN 470's 11-A 0.3 in low
        // over its hump, with LowSpeedLiftFullCv 3)
        public static double PlaningBuoyancyStartCv = 1.5, PlaningBuoyancyFullCv = 3.5;
        // the planing run's load: each strip's momentum jump spread aft from its own station with the flat-plate loading
        // (0), or the run's whole jump spread so from its spray root (1; Savitsky's centre of pressure, three quarters of
        // the wetted length ahead of where the flow leaves, on a V bottom whose wetted width grows along the run), between
        public static double RunLoading = 1;
        public static double SeparationDrop = 0.1;    // a section's m_a V this share below the run's peak: flow separated
        // the wake behind a run (where the flow leaves a step or a transom) falls aft from the run's deepest point there at
        // this share of the run's keel slope: a bottom behind it reaching below that line is wetted again by the wake's
        // water (an afterbody at a trim past its own keel angle), and carries its pressure below the line
        public static double WakeSlope = 0.25;
        // under the water: the share of a shrinking section's momentum change the flow gives back (1: potential flow,
        // Munk's pure couple; 0: separated at the largest section, its lift and none of the couple aft of it)
        public static double SubmergedRecovery = 0.65;
        // under the water: crossflow drag coefficient of each side's strips on the flow across the body, either way (a
        // body's crossflow drag, Allen and Perkins: about 0.6 on its whole width, both sides). 0.45: DARPA SUBOFF's yaw
        // damping (N'r 0.9 of Roddy's, bare and with fins) with its side force at 18 deg drift within 25% (0.4: the
        // test submarine still turns away on its own; 0.5: the side force at large drift 35% high)
        public static double SubmergedCrossflowDrag = 0.45;
        // the step ventilates past the hump: on the hull's beam (its widest wetted strip, the chine's), from C_V 3 to 4.5.
        // NACA TN 716's float 57-B-5 free to trim then within 1.9 deg of its Fig. 4 (1.2 rms; from 2.5: 0.9 rms, but the
        // Cessna floatplane, heavier and its C.G. higher, sticks at its hump, its stern unsupported; from 2: the float too
        // runs away to 20-25 deg). Not on the mean wetted beam: that moves with the trim, and the afterbody's support with
        // it, which drove the floatplane on its step into porpoising as it pitched
        public static double VentStartCv = 3.0, VentFullCv = 4.5;
        // a section whose top is this share of the hull's deepest section's height below the water is wholly submerged
        // (no free surface)
        public static double SubmergedFade = 0.5;
        // below planing (the hull's beam Froude number C_V, on its widest strip): the planing (momentum) lift's share at
        // C_V 1 and below, all of it from LowSpeedLiftFullCv on (smoothly between); and the suction of the flow under a
        // hull not yet planing (its squat: TN 470's 11-A sinks below its rest draft to C_V ~2), LowSpeedSuction times
        // q times the planform pulling it down, fading out as the lift comes in (fitted to TN 470's Table II at 40 lb,
        // NacaHullTests: its 11-A rides 0.5 - 0.75 in too high below C_V 2.5 at every trim without them). Full at 3.5,
        // the planing buoyancy fading later with it: the hump's step draft within 0.1 in of TN 470's on average (at 3:
        // 0.3 in low, 7 deg C_V 2.49 0.65 in)
        public static double LowSpeedLift = 0.5, LowSpeedLiftFullCv = 3.5, LowSpeedSuction = 0.03;
        // the waves a hull not yet planing makes: a drag of WaveDrag times its displacement (the water its hydrostatics
        // hold up), from C_V 1 fading out as the planing lift comes in (LowSpeedLiftFullCv), as C_V^2 below 1 (TN 470's
        // 11-A at C_V 1 to 1.5 has about twice the resistance its planing terms give, at every trim)
        public static double WaveDrag = 0.045;
        // a foil on a hull wholly under the water: the share (1: all of it) of the hull's upwash it feels as the hull
        // turns (FARWingAerodynamicModel.HullUpwash)
        public static double FoilHullUpwash = 1;
        // a part's mesh of at least MeshSimplifyMinTriangles simplified within MeshSimplifyTolerance of its shape (m; at
        // most MeshSimplifyShare of its smallest size: the 11-A model hull 1.1 mm, its beam 17 in, where 2.5 mm moved
        // its draft 1.5 mm; 0: never; MeshSimplifier): the water model's cost goes with the wet triangles (a float's
        // Spine collider of 6260: 13 ms a step on a floatplane in waves, 1.7 ms simplified), and a grid's clustering
        // to a count rounded off the step and chines (TN 470's 11-A 1.4 - 1.5x at 5 - 7 deg)
        public static double MeshSimplifyTolerance = 0.003, MeshSimplifyShare = 0.0025, MeshSimplifyMinTriangles = 500;
        // a foil in the water hands its force to the first of its parents that is not a foil, whether or not that part is
        // in a hull (FARWingAerodynamicModel.WaterHost; false: only to a hull's part, as before: tests compare)
        public static bool FoilHostAnyBody = true;
        // a face is a strip's (the bottom's, the top's, a side's) as much as it looks that way (its area times its
        // normal's share in that direction), from this share on at the surface (the planing model's), from none under
        // the water (each section's sec between): there every face that looks that way at all is, so the strips do not
        // jump as a faceted section turns (a 12-sided collider rolled 15 deg moved a whole facet in or out of the
        // bottom's width)
        public static double FaceThreshold = 0.3;
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
        // nor one whose open edges take caps of more than this share of its area: a cap fanned across a large gap (a
        // missing face) need not follow the body
        public const double CapTolerance = 0.05;
        public const int MinStrips = 4, MaxStrips = 40;
        // a face this far (m) out along its normal inside another part: the two parts meet there
        public const float ContactProbe = 0.005f;
        // a contact face with the other part's face against it, facing it, within this (m) along its normal: the two
        // faces meet and their pressures cancel. Any other contact face lies inside the other part, dry; a part all of
        // whose faces are dry is sealed inside others (a ballast in a hull, a tank in a fuselage) and takes no water
        // pressure. One only partly inside (a wing's root in a fuselage) keeps its pressure all round, the overlap
        // counted twice: each part's force its own volume's, straight up, not pushing on its joints
        public const float ContactTouch = 0.05f;

        /// <summary>The meshes the water sees: FAR's (what its voxels are made from: colliders or visible meshes, by
        /// FAR's own choice per part), the part's colliders, or its visible meshes.</summary>
        public enum MeshSourceKind { Far, Colliders, Visual }
        public static MeshSourceKind MeshSource = MeshSourceKind.Far;

        // time spent (Stopwatch ticks) and work done since the counters were last reset (tests, benchmarks)
        public static long ProfileGatherTicks, ProfileSolveTicks, ProfileContactTicks, ProfileMeshTicks;
        public static long ProfileSteps, ProfileTrianglesTested, ProfilePolygons, ProfileContactPairs;
        public static long ProfileShadowBuilds, ProfileShadowTicks, ProfileShadowMaxTicks, ProfileLeeRays, ProfileLeeTicks,
            ProfilePieceTicks, ProfileImpactTicks;
        public static void ResetProfile() =>
            ProfileGatherTicks = ProfileSolveTicks = ProfileContactTicks = ProfileMeshTicks =
                ProfileSteps = ProfileTrianglesTested = ProfilePolygons = ProfileContactPairs =
                    ProfileShadowBuilds = ProfileShadowTicks = ProfileShadowMaxTicks = ProfileLeeRays = ProfileLeeTicks =
                        ProfilePieceTicks = ProfileImpactTicks = 0;

        internal readonly Part part;
        private object meshSource;
        private MeshSourceKind meshKind;
        private int meshVersion;
        private Vector3[] localVerts = Array.Empty<Vector3>();
        private int[] tris = Array.Empty<int>();
        private int capStart;   // the triangles from this one on cap the mesh's open edges: pressure only, no wetted face
        private Bounds localBounds;
        // a closed mesh's leak: its area vectors' sum (part space; zero when it is truly closed), the cap it lacks
        private Vector3 leakArea;
        private Vector3[] worldVerts = Array.Empty<Vector3>();
        private float[] depths = Array.Empty<float>();

        // the triangles that meet another part, per other part (indices; null: none), those of them inside it with no
        // face of its against them (dry), and the stamps marking this step's union of each
        private readonly Dictionary<PartHydrodynamics, (int version, int otherVersion, int[] tris, int[] dry)> contacts =
            new Dictionary<PartHydrodynamics, (int, int, int[], int[])>();
        private int[] contactStamp = Array.Empty<int>(), dryStamp = Array.Empty<int>();
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
        internal float OriginDepth;   // Origin's depth below the water (m)
        // a FAR wing part: a foil, FAR's wing model gives it its water forces (its buoyancy only here)
        internal readonly bool Foil;

        // the mesh is closed: its hydrostatic pressure is the part's buoyancy, which replaces stock's (part.buoyancy 0)
        public bool OwnsBuoyancy { get; private set; }
        public double MeshVolume { get; private set; }
        public double OpenShare { get; private set; }   // the mesh's area vectors' sum over its area (0: closed), uncapped
        public double CapArea { get; private set; }     // the area of the caps on its open edges (m2)
        public int MeshTriangles => tris.Length / 3;
        internal float StockBuoyancy = -1;
        // stock's water drag and angular drag taken off the rigidbody for the hull to apply (WaterDampingMode.Explicit; -1: none)
        internal float DampDrag = -1, DampAngular;

        // what the last step did, for tests and display (kN, m2, m): this part's share of its hull's forces.
        // LastHydrostaticForce: the part's buoyancy when the mesh is closed, else the planing cut to stock's.
        // LastPlanform, LastWettedLength, LastLimiter, LastStrips: its hull's
        public Vector3 LastForce, LastMomentumForce, LastFrictionForce, LastImpactForce, LastCrossflowForce, LastHydrostaticForce, LastWaveForce;
        public Vector3 LastHydrostaticTorque;   // kN m, about the part's CoM (the closed mesh's pressure)
        public Vector3 LastTorque;              // kN m, LastForce's about the part's CoM
        public double LastWettedArea, LastPlanform, LastWettedLength, LastLimiter = 1;
        public int LastStrips;
        /// <summary>The hull this part was part of in the last step (null: dry).</summary>
        public HullResult Hull { get; internal set; }
        /// <summary>The hull this part was last part of, kept through the next step's start (Hull is cleared at it, and
        /// another part's FixedUpdate, a foil's, may run before or after the step's solve); null once it is dry.</summary>
        internal HullResult LastHull { get; set; }

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
            foreach (PartModule m in part.Modules)
                if (m is ferram4.FARWingAerodynamicModel)
                    Foil = true;
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
            int pieceCount = 0;
            void Sink(string name, Vector3[] mv, int[] mt, Matrix4x4 toPart)
            {
                log.Append($" {name}({mt.Length / 3})");
                AddMesh(mv, mt, toPart, verts, t);
                pieceCount++;
            }
            Pieces(part, MeshSource, Sink, log);
            int unsimplified = t.Count / 3;
            if (MeshSimplifyTolerance > 0 && unsimplified >= MeshSimplifyMinTriangles && verts.Count > 0)
            {
                var b0 = new Bounds(verts[0], Vector3.zero);
                foreach (Vector3 w in verts)
                    b0.Encapsulate(w);
                double smallest = Math.Min(b0.size.x, Math.Min(b0.size.y, b0.size.z));
                double tol = Math.Min(MeshSimplifyTolerance, MeshSimplifyShare * smallest);
                if (MeshSimplifier.Simplify(verts, t, tol) > 0)
                    log.Append($"; simplified from {unsimplified} within {tol * 1000:F1} mm");
            }

            // closed and outward: the area vectors cancel and the enclosed volume (capped) is positive
            Vector3 openSum = Vector3.zero;
            double area = 0;
            for (int i = 0; i + 2 < t.Count; i += 3)
            {
                Vector3 n2 = Vector3.Cross(verts[t[i + 1]] - verts[t[i]], verts[t[i + 2]] - verts[t[i]]);   // twice the area, outward
                openSum += n2 * 0.5f;
                area += n2.magnitude * 0.5;
            }
            capStart = t.Count / 3;
            CapArea = CapOpenEdges(verts, t, out int loops);
            if (loops > 0)
                log.Append($"; {loops} open edge loops capped, {CapArea:F4} m2");
            // (pieces together sealing a space in: their faces on it dropped, when that closes a hollow in, its volume
            // grown; pieces only overlapping, a B9 wing's, are left as they are: their buried faces' pressures cancel,
            // and dropping them left its skin's seams a little open)
            if (pieceCount > 1)
            {
                double before = SignedVolume(verts, t);
                var trial = new List<int>(t);
                int trialCap = capStart;
                int dropped = DropEnclosed(verts, trial, ref trialCap);
                double after = dropped > 0 ? SignedVolume(verts, trial) : before;
                if (dropped > 0 && after > 1.05 * before)
                {
                    t.Clear();
                    t.AddRange(trial);
                    capStart = trialCap;
                    log.Append($"; {dropped} faces closing a hollow in dropped ({before:F3} to {after:F3} m3)");
                }
            }
            Vector3 areaSum = Vector3.zero;
            double volume = 0;
            for (int i = 0; i + 2 < t.Count; i += 3)
            {
                Vector3 a = verts[t[i]], b = verts[t[i + 1]], c = verts[t[i + 2]];
                areaSum += Vector3.Cross(b - a, c - a) * 0.5f;
                volume += Vector3.Dot(a, Vector3.Cross(b, c)) / 6.0;
            }
            MeshVolume = volume;
            var bounds = new Bounds(verts.Count > 0 ? verts[0] : Vector3.zero, Vector3.zero);
            foreach (Vector3 w in verts)
                bounds.Encapsulate(w);
            localBounds = bounds;
            log.Append($"; part-space bounds {bounds.min:F3} to {bounds.max:F3}");
            OwnsBuoyancy = area > 0 && volume > 0 && openSum.magnitude < ClosureTolerance * area && CapArea < CapTolerance * area;
            leakArea = areaSum;
            OpenShare = area > 0 ? openSum.magnitude / area : 1;
            if (!OwnsBuoyancy)
                RestoreStockBuoyancy();
            FARLogger.Info($"Hydrodynamics of {part.partInfo?.name}: {t.Count / 3} triangles from {MeshSource} meshes{log}; " +
                           $"volume {volume:F3} m3, open {OpenShare:F4} of its area: " +
                           (OwnsBuoyancy ? "buoyancy from its pressure" : "stock buoyancy"));
            localVerts = verts.ToArray();
            tris = t.ToArray();
            worldVerts = new Vector3[localVerts.Length];
            depths = new float[localVerts.Length];
            contactStamp = new int[tris.Length / 3];
            dryStamp = new int[tris.Length / 3];
            contacts.Clear();
            meshSource = source;
            meshKind = MeshSource;
            meshVersion++;
            ProfileMeshTicks += Stopwatch.GetTimestamp() - t0;
            return tris.Length > 0;
        }

        /// <summary>
        /// The meshes the water model reads for a part from a source, piece by piece: each one's name, its vertices and
        /// triangles, and its transform into part space (what the model joins into the part's mesh).
        /// </summary>
        internal static void Pieces(Part part, MeshSourceKind kind, Action<string, Vector3[], int[], Matrix4x4> sink,
                                    System.Text.StringBuilder log = null)
        {
            log ??= new System.Text.StringBuilder();
            if (kind == MeshSourceKind.Far)
            {
                GeometryPartModule g = part.GetComponent<GeometryPartModule>();
                if (g != null && g.meshDataList != null)
                    FarMeshes(g.meshDataList, sink, log);
            }
            else
                OwnMeshes(part, kind == MeshSourceKind.Colliders, sink, log);
        }

        private static void FarMeshes(List<GeometryMesh> meshes, Action<string, Vector3[], int[], Matrix4x4> sink, System.Text.StringBuilder log)
        {
            var seen = new List<(int verts, int tris, Bounds bounds, Vector3 area)>();
            foreach (GeometryMesh m in meshes)
            {
                if (m == null || !m.valid || !m.gameObjectActiveInHierarchy || m.MeshLocalVerts == null)
                    continue;
                Matrix4x4 toPart = m.meshLocalToPart;
                // the same mesh twice (a copy for another purpose, LODs) would count its surface twice: the same counts,
                // bounds and area facing each way (a B9 wing's surface, its top and bottom, and its section, its root and
                // tip, share the first three: the same 8 corners)
                Bounds pb = GeometryMesh.TransformBounds(m.meshLocalBounds, toPart);
                Vector3 area = FacingArea(m.MeshLocalVerts, m.triangles, toPart);
                bool copy = seen.Exists(s => s.verts == m.MeshLocalVerts.Length && s.tris == m.triangles.Length &&
                                             (s.bounds.center - pb.center).sqrMagnitude < 1e-6f &&
                                             (s.bounds.size - pb.size).sqrMagnitude < 1e-6f &&
                                             (s.area - area).sqrMagnitude < 1e-6f * Math.Max(1, area.sqrMagnitude));
                if (copy)
                {
                    log.Append($" {m.meshTransform.name}(copy skipped)");
                    continue;
                }
                seen.Add((m.MeshLocalVerts.Length, m.triangles.Length, pb, area));
                sink(m.meshTransform.name, m.MeshLocalVerts, m.triangles, toPart);
            }
        }

        // a mesh's area facing along each axis of the part, either way (m2)
        private static Vector3 FacingArea(Vector3[] mv, int[] mt, Matrix4x4 toPart)
        {
            Vector3 sum = Vector3.zero;
            for (int i = 0; i + 2 < mt.Length; i += 3)
            {
                Vector3 a = toPart.MultiplyPoint3x4(mv[mt[i]]);
                Vector3 n = Vector3.Cross(toPart.MultiplyPoint3x4(mv[mt[i + 1]]) - a, toPart.MultiplyPoint3x4(mv[mt[i + 2]]) - a);
                sum += new Vector3(Math.Abs(n.x), Math.Abs(n.y), Math.Abs(n.z)) * 0.5f;
            }
            return sum;
        }

        // the part's own colliders (mesh and box) or visible meshes (active, rendered, readable), in part space
        private static void OwnMeshes(Part part, bool colliders, Action<string, Vector3[], int[], Matrix4x4> sink, System.Text.StringBuilder log)
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
                        sink(c.name, mc.sharedMesh.vertices, mc.sharedMesh.triangles, m);
                    else if (c is BoxCollider bc)
                    {
                        MeshData box = GeometryPartModule.CreateBoxMeshFromBoxCollider(bc.size, bc.center);
                        sink(c.name + " box", box.vertices, box.triangles, m);
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
                sink(mf.name, mf.sharedMesh.vertices, mf.sharedMesh.triangles, toPart * mf.transform.localToWorldMatrix);
            }
        }

        // vertices this close (m, part space) are one where the mesh's pieces meet
        private const float WeldTolerance = 1e-4f;

        // Shadowed's last result and what it was computed from (this mesh's version and its neighbours'), and its
        // triangles (this part's and its neighbours', in this part's space) for Lee
        private bool[] shadowed;
        private long shadowKey = -1;
        private TriangleBvh shadowBvh;
        // the neighbours' places in this part's space it was computed at (a hinge, a servo moves them)
        private readonly List<Matrix4x4> shadowPoses = new List<Matrix4x4>(8);
        // its neighbours two joints out (their modules: a neighbour's hydrodynamics comes once it is wet), found again
        // when a vessel's parts change (structureVersion)
        private readonly List<FARAeroPartModule> nearModules = new List<FARAeroPartModule>(16);
        private readonly List<PartHydrodynamics> nb = new List<PartHydrodynamics>(8);
        private int nearVersion = -1;
        private Vessel nearVessel;
        private static int structureVersion;
        private static bool structureHooked;
        // a neighbour moved this far (m) or turned this far (its axes' change, ~10 deg) from its place: computed again, at
        // most once in PoseEvery s (a hinge's, a servo's travel; a joint's flex under the water's load, a few cm on a big
        // boat's keel foils every step, is not: rebuilt each step, 17 ms each, it took the frame rate)
        private const float PoseMove = 0.2f, PoseTurn = 0.17f, PoseEvery = 1f;
        private float shadowBuiltAt = float.NegativeInfinity;
        private int shadowLogged = -1;

        private void FindNear()
        {
            if (!structureHooked)
            {
                structureHooked = true;
                GameEvents.onVesselWasModified.Add(v => structureVersion++);
            }
            if (nearVersion == structureVersion && ReferenceEquals(nearVessel, part.vessel))
                return;
            nearVersion = structureVersion;
            nearVessel = part.vessel;
            nearModules.Clear();
            var near = new HashSet<Part> { part };
            var ring = new List<Part> { part };
            for (int hop = 0; hop < 2; hop++)
            {
                var next = new List<Part>();
                foreach (Part p in ring)
                {
                    foreach (Part q in p.children)
                        if (q != null && near.Add(q))
                            next.Add(q);
                    if (p.parent != null && near.Add(p.parent))
                        next.Add(p.parent);
                }
                ring = next;
            }
            foreach (Part q in near)
                if (q != part && q.Modules.GetModule<FARAeroPartModule>() is FARAeroPartModule m)
                    nearModules.Add(m);
        }

        private bool PosesMoved()
        {
            if (shadowPoses.Count != nb.Count)
                return true;
            Matrix4x4 toMe = part.partTransform.worldToLocalMatrix;
            for (int i = 0; i < nb.Count; i++)
            {
                Matrix4x4 now = toMe * nb[i].part.partTransform.localToWorldMatrix, was = shadowPoses[i];
                if ((now.GetColumn(3) - was.GetColumn(3)).sqrMagnitude > PoseMove * PoseMove)
                    return true;
                for (int c = 0; c < 3; c++)
                {
                    Vector4 a = now.GetColumn(c), b = was.GetColumn(c);
                    if ((a - b).sqrMagnitude > PoseTurn * PoseTurn * b.sqrMagnitude)
                        return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Whether a face's point (world, just off it) is in the lee of this part's or its neighbours' bodies along a
        /// direction (world: upstream, the way the face moves through the water): a ray from it that way meets them.
        /// </summary>
        internal bool Lee(Vector3 point, Vector3 dir)
        {
            if (shadowBvh == null)
                return false;
            Transform tf = part.partTransform;
            return shadowBvh.AnyHit(tf.InverseTransformPoint(point), tf.InverseTransformDirection(dir).normalized, -1);
        }

        /// <summary>
        /// This part's faces that the water cannot reach for the parts joined to it (two joints out, those with meshes):
        /// every ray from the face over the half it looks into (DropEnclosed's) meets this part's mesh or
        /// one of theirs. A Mk3 cargo bay is open at its ends, its hollow closed by the parts there: its inside, and their
        /// end caps facing into it, no part sees closed on its own (DropEnclosed), so their flow terms took the water in
        /// it (a planing floor in the bay, a stagnation face on each cap). Its own mesh alone too: a face folded into
        /// the body (a loft pleated where its sections change in a short way, the 11-A's sternpost: faces there looking
        /// forward inside it, a stagnation drag doubling its resistance at 9 deg trim) sees only its own part's inside.
        /// Null: none. Recomputed when this mesh or a
        /// neighbour's changes, or a neighbour moves from its place (a hinge, a servo).
        /// </summary>
        private bool[] Shadowed()
        {
            // its neighbours two joints out (a cargo bay open at both ends: the part beyond it closes what this one's
            // cap faces into)
            FindNear();
            nb.Clear();
            foreach (FARAeroPartModule m in nearModules)
            {
                PartHydrodynamics hq = m != null ? m.Hydrodynamics : null;
                if (hq != null && hq.tris.Length > 0)
                    nb.Add(hq);
            }
            long key = meshVersion;
            foreach (PartHydrodynamics hq in nb)
                key = key * 1000003 + hq.meshVersion * 7919 + hq.part.GetInstanceID();
            if (key == shadowKey && (Time.time - shadowBuiltAt < PoseEvery || !PosesMoved()))
                return shadowed;
            shadowBuiltAt = Time.time;
            shadowKey = key;
            shadowed = null;
            shadowBvh = null;
            shadowPoses.Clear();
            if (tris.Length == 0)
                return null;

            long t0 = Stopwatch.GetTimestamp();
            // every triangle in this part's space, its own first (their indices its faces'), then its neighbours'
            int total = tris.Length / 3;
            foreach (PartHydrodynamics hq in nb)
                total += hq.tris.Length / 3;
            var ta = new Vector3[total]; var te1 = new Vector3[total]; var te2 = new Vector3[total];
            int at = 0;
            void Mesh(Vector3[] lv, int[] lt, Matrix4x4 m)
            {
                for (int i = 0; i + 2 < lt.Length; i += 3)
                {
                    Vector3 p0 = m.MultiplyPoint3x4(lv[lt[i]]), p1 = m.MultiplyPoint3x4(lv[lt[i + 1]]), p2 = m.MultiplyPoint3x4(lv[lt[i + 2]]);
                    ta[at] = p0; te1[at] = p1 - p0; te2[at] = p2 - p0;
                    at++;
                }
            }
            Mesh(localVerts, tris, Matrix4x4.identity);
            Matrix4x4 toMe = part.partTransform.worldToLocalMatrix;
            foreach (PartHydrodynamics hq in nb)
            {
                Matrix4x4 pose = toMe * hq.part.partTransform.localToWorldMatrix;
                shadowPoses.Add(pose);
                Mesh(hq.localVerts, hq.tris, pose);
            }
            var bvh = new TriangleBvh(ta, te1, te2);
            shadowBvh = bvh;

            int ownCount = tris.Length / 3;
            var result = new bool[ownCount];
            int count = 0;
            for (int i = 0; i < ownCount && i < capStart; i++)
            {
                if (Enclosed(bvh, ta[i], te1[i], te2[i], i))
                {
                    result[i] = true;
                    count++;
                }
            }
            if (count > 0)
                shadowed = result;
            long ticks = Stopwatch.GetTimestamp() - t0;
            ProfileShadowBuilds++;
            ProfileShadowTicks += ticks;
            ProfileShadowMaxTicks = Math.Max(ProfileShadowMaxTicks, ticks);
            // (logged when it changes what is closed in, or is slow)
            double ms = ticks * 1000.0 / Stopwatch.Frequency;
            if (count != shadowLogged || ms > 20)
                FARLogger.Info($"Hydrodynamics of {part.partInfo?.name}: {count} of {capStart} faces closed in with {nb.Count} neighbours ({ms:F0} ms)");
            shadowLogged = count;
            return shadowed;
        }

        private static double SignedVolume(List<Vector3> verts, List<int> t)
        {
            double v = 0;
            for (int i = 0; i + 2 < t.Count; i += 3)
                v += Vector3.Dot(verts[t[i]], Vector3.Cross(verts[t[i + 1]], verts[t[i + 2]])) / 6.0;
            return v;
        }

        private static readonly float Cos50 = Mathf.Cos(50 * Mathf.Deg2Rad), Sin50 = Mathf.Sin(50 * Mathf.Deg2Rad);

        // a face (corner a, edges e1 and e2; self, its index in bvh) closed in by bvh's triangles: rays from just off it
        // along its normal and six around that 50 deg off all meet them. One with no area: not
        private static bool Enclosed(TriangleBvh bvh, Vector3 a, Vector3 e1, Vector3 e2, int self)
        {
            Vector3 nn = Vector3.Cross(e1, e2);
            if (nn.sqrMagnitude < 1e-12f)
                return false;
            nn.Normalize();
            Vector3 o = a + (e1 + e2) / 3 + nn * 1e-3f;
            Vector3 b1 = Vector3.Cross(nn, Math.Abs(nn.x) < 0.9f ? Vector3.right : Vector3.up).normalized, b2 = Vector3.Cross(nn, b1);
            if (!bvh.AnyHit(o, nn, self))
                return false;
            for (int k = 0; k < 6; k++)
            {
                float ang = k * Mathf.PI / 3;
                if (!bvh.AnyHit(o, nn * Cos50 + (b1 * Mathf.Cos(ang) + b2 * Mathf.Sin(ang)) * Sin50, self))
                    return false;
            }
            return true;
        }

        /// <summary>
        /// The faces of a part's mesh that no water reaches: a cargo bay's inside (its base and its doors, each a closed
        /// thin shell, together closing in its bay), a piece's faces inside another's. A face is enclosed when rays
        /// from it over the half it looks into (its normal, and six around it 50 degrees off) all meet the mesh again:
        /// one on the outside sees out along its normal, or in a hollow (a boat's well, a corner) out of its open side.
        /// Dropped (the caps' start moved down by the cap faces dropped), the rest the part's outer skin: its volume the
        /// space it closes, the bay's air with it, and its neighbours' faces against it in contact with it (a point
        /// just off them inside it, not in its hollow). Returns how many were dropped.
        /// </summary>
        internal static int DropEnclosed(List<Vector3> verts, List<int> t, ref int capStart)
        {
            int n = t.Count / 3;
            var a = new Vector3[n];
            var e1 = new Vector3[n];
            var e2 = new Vector3[n];
            for (int i = 0; i < n; i++)
            {
                a[i] = verts[t[3 * i]];
                e1[i] = verts[t[3 * i + 1]] - a[i];
                e2[i] = verts[t[3 * i + 2]] - a[i];
            }
            var bvh = new TriangleBvh(a, e1, e2);
            var keep = new bool[n];
            int dropped = 0, droppedCaps = 0;
            for (int i = 0; i < n; i++)
            {
                bool enclosed = Enclosed(bvh, a[i], e1[i], e2[i], i);
                keep[i] = !enclosed;
                if (enclosed)
                {
                    dropped++;
                    if (i >= capStart)
                        droppedCaps++;
                }
            }
            if (dropped == 0)
                return 0;
            var kept = new List<int>(t.Count - 3 * dropped);
            for (int i = 0; i < n; i++)
                if (keep[i])
                    kept.AddRange(new[] { t[3 * i], t[3 * i + 1], t[3 * i + 2] });
            capStart -= dropped - droppedCaps;
            t.Clear();
            t.AddRange(kept);
            return dropped;
        }

        /// <summary>
        /// Caps the mesh's open edges: its boundary (each edge's triangles' sides one way less those the other, the
        /// vertices welded where its pieces meet) walked into loops, each fanned from its middle. The mesh with them is
        /// closed: its pressure integrates to the displaced water's weight, straight up (exact for a flat gap, as a
        /// wing's root or tip; a T-join's sliver caps to nothing). Returns the caps' area; loops: how many.
        /// </summary>
        private static double CapOpenEdges(List<Vector3> verts, List<int> t, out int loops)
        {
            loops = 0;
            int nv = verts.Count;
            var weld = new int[nv];
            var at = new Dictionary<(int, int, int), int>(nv);
            for (int i = 0; i < nv; i++)
            {
                Vector3 v = verts[i] / WeldTolerance;
                var key = (Mathf.RoundToInt(v.x), Mathf.RoundToInt(v.y), Mathf.RoundToInt(v.z));
                if (!at.TryGetValue(key, out int w))
                    at[key] = w = i;
                weld[i] = w;
            }
            // each edge's net count, lower vertex to higher
            var net = new Dictionary<(int, int), int>();
            void Side(int a, int b)
            {
                if (a == b)
                    return;
                var key = a < b ? (a, b) : (b, a);
                net.TryGetValue(key, out int n);
                net[key] = n + (a < b ? 1 : -1);
            }
            int nt = t.Count;
            for (int i = 0; i + 2 < nt; i += 3)
            {
                int a = weld[t[i]], b = weld[t[i + 1]], c = weld[t[i + 2]];
                Side(a, b);
                Side(b, c);
                Side(c, a);
            }
            var outOf = new Dictionary<int, List<int>>();
            int open = 0;
            foreach (var kv in net)
                for (int k = 0; k < Math.Abs(kv.Value); k++)
                {
                    int a = kv.Value > 0 ? kv.Key.Item1 : kv.Key.Item2, b = kv.Value > 0 ? kv.Key.Item2 : kv.Key.Item1;
                    if (!outOf.TryGetValue(a, out var list))
                        outOf[a] = list = new List<int>(2);
                    list.Add(b);
                    open++;
                }
            if (open == 0)
                return 0;

            // the boundary of a set of triangles is closed (as many sides into each vertex as out): walked from any
            // vertex, a loop comes back to it
            double capArea = 0;
            var loop = new List<int>();
            foreach (int start in new List<int>(outOf.Keys))
                while (outOf.TryGetValue(start, out var first) && first.Count > 0)
                {
                    loop.Clear();
                    int cur = start;
                    while (loop.Count <= open && outOf.TryGetValue(cur, out var nexts) && nexts.Count > 0)
                    {
                        loop.Add(cur);
                        int next = nexts[nexts.Count - 1];
                        nexts.RemoveAt(nexts.Count - 1);
                        cur = next;
                        if (cur == start)
                            break;
                    }
                    if (cur != start || loop.Count < 3)
                        continue;
                    Vector3 mid = Vector3.zero;
                    foreach (int k in loop)
                        mid += verts[k];
                    mid /= loop.Count;
                    int c = verts.Count;
                    verts.Add(mid);
                    // the surface's boundary runs a to b: the cap's runs b to a
                    for (int k = 0; k < loop.Count; k++)
                    {
                        int a = loop[k], b = loop[(k + 1) % loop.Count];
                        t.Add(c);
                        t.Add(b);
                        t.Add(a);
                        capArea += 0.5 * Vector3.Cross(verts[b] - mid, verts[a] - mid).magnitude;
                    }
                    loops++;
                }
            return capArea;
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

        // stock's buoyancy force per unit of part.buoyancy, as last seen (N)
        private double stockPerUnit = -1;

        // the share of the mesh's area (its caps aside) inside the wet parts it is joined to, as the last step found it:
        // an engine's casing sunk into a hull, its open ends with it
        private double buriedShare;
        private float[] faceArea;

        /// <summary>
        ///     A part whose mesh is not closed keeps stock's buoyancy, held to what it could displace: the smaller of its
        ///     mesh's bounding box and the mesh its open edges capped close (rho g V times its submerged share), less the
        ///     share of it buried in the parts it is joined to. Stock's for a procedural part can be its prefab's (a 2 kg
        ///     B9 wing panel was given 3.6 MN, 360 m3 of water, and blew off as it went under); a part's box is its whole
        ///     extent (a propeller's, its spinner and blade roots 1.2 m across: 1.7 m3 on a submarine's tail, standing it
        ///     on its nose), where its capped mesh is its body; and a part sunk into a hull (an engine's casing) displaces
        ///     only what sticks out. Neither is ever more than stock's own: an ordinary part's is not touched. Through
        ///     part.buoyancy (stock applies it): its force per unit of that from the step before.
        /// </summary>
        private void HoldStockBuoyancy(double rho)
        {
            if (StockBuoyancy < 0)
                StockBuoyancy = part.buoyancy;
            double stock = part.partBuoyancy.lastBuoyantForce.magnitude * 1000;
            if (part.buoyancy > 1e-4 && stock > 0)
                stockPerUnit = stock / part.buoyancy;
            if (stockPerUnit <= 0)
                return;
            Vector3 scale = part.partTransform.lossyScale;
            Vector3 size = Vector3.Scale(localBounds.size, scale);
            double volume = Math.Abs(size.x * size.y * size.z);
            double meshVolume = MeshVolume * Math.Abs(scale.x * scale.y * scale.z);
            if (meshVolume > 0)
                volume = Math.Min(volume, meshVolume);
            volume *= 1 - buriedShare;
            double cap = rho * Gravity * volume * part.submergedPortion;
            part.buoyancy = (float)(StockBuoyancy * Math.Min(1, cap / Math.Max(1e-9, stockPerUnit * StockBuoyancy)));
        }

        /// <summary>Gives the part its stock buoyancy back (the model off, or the mesh not closed).</summary>
        public void RestoreStockBuoyancy()
        {
            if (StockBuoyancy >= 0)
                part.buoyancy = StockBuoyancy;
            StockBuoyancy = -1;
        }

        internal void Clear(bool keepLastHull = false)
        {
            if (!keepLastHull)
                LastHull = null;
            LastForce = LastMomentumForce = LastFrictionForce = LastImpactForce = LastCrossflowForce = LastHydrostaticForce = LastWaveForce = LastHydrostaticTorque = LastTorque = Vector3.zero;
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
            Clear(true);
            Vessel vessel = part.vessel;
            if (part.rb == null || vessel == null || part.submergedPortion <= 0)
            {
                LastHull = null;
                return false;
            }
            CelestialBody body = vessel.mainBody;
            return body != null && body.ocean && EnsureMesh();
        }

        // the triangles of this part whose outside lies in the other part (they meet there), computed once per pair:
        // the parts are joined, so they keep their places
        private (int[] tris, int[] dry) ContactsWith(PartHydrodynamics other)
        {
            if (contacts.TryGetValue(other, out var c) && c.version == meshVersion && c.otherVersion == other.meshVersion)
                return (c.tris, c.dry);
            long t0 = Stopwatch.GetTimestamp();
            int[] found = null, dry = null;
            if (other.OwnsBuoyancy)
            {
                Matrix4x4 toOther = other.part.partTransform.worldToLocalMatrix * part.partTransform.localToWorldMatrix;
                Bounds ob = other.localBounds;
                ob.Expand(2 * ContactProbe);
                if (ob.Intersects(GeometryMesh.TransformBounds(localBounds, toOther)))
                {
                    var list = new List<int>();
                    var dryList = new List<int>();
                    for (int i = 0; i + 2 < tris.Length; i += 3)
                    {
                        Vector3 a = localVerts[tris[i]], b = localVerts[tris[i + 1]], cc = localVerts[tris[i + 2]];
                        Vector3 n = Vector3.Cross(b - a, cc - a);
                        if (n.sqrMagnitude < 1e-12f)
                        {
                            dryList.Add(i / 3);     // (no area, no pressure: it does not keep a sealed part wet)
                            continue;
                        }
                        Vector3 mid = (a + b + cc) / 3;
                        Vector3 q = toOther.MultiplyPoint3x4(mid + n.normalized * ContactProbe);
                        if (!ob.Contains(q) || !other.Inside(q))
                            continue;
                        list.Add(i / 3);
                        if (!other.FacesAgainst(toOther.MultiplyPoint3x4(mid), toOther.MultiplyVector(n).normalized))
                            dryList.Add(i / 3);
                    }
                    if (list.Count > 0)
                        found = list.ToArray();
                    if (dryList.Count > 0)
                        dry = dryList.ToArray();
                }
            }
            contacts[other] = (meshVersion, other.meshVersion, found, dry);
            ProfileContactPairs++;
            ProfileContactTicks += Stopwatch.GetTimestamp() - t0;
            return (found, dry);
        }

        // a face of this mesh against a point q (part space) of another's face with normal n: facing it (normals
        // opposed) and crossing the line through q along n within ContactTouch of it
        private bool FacesAgainst(Vector3 q, Vector3 n)
        {
            for (int i = 0; i + 2 < tris.Length; i += 3)
            {
                Vector3 a = localVerts[tris[i]], e1 = localVerts[tris[i + 1]] - a, e2 = localVerts[tris[i + 2]] - a;
                Vector3 fn = Vector3.Cross(e1, e2);
                if (Vector3.Dot(fn, n) > -0.5f * fn.magnitude)
                    continue;
                if (TriangleBvh.Ray(q, n, a, e1, e2, out float d) && Math.Abs(d) < ContactTouch)
                    return true;
            }
            return false;
        }

        // a point in part space inside the closed mesh: a ray's crossings are odd
        private bool Inside(Vector3 q)
        {
            var dir = new Vector3(0.3713907f, 0.5570860f, 0.7427813f);
            int crossings = 0;
            for (int i = 0; i + 2 < tris.Length; i += 3)
            {
                Vector3 a = localVerts[tris[i]], e1 = localVerts[tris[i + 1]] - a, e2 = localVerts[tris[i + 2]] - a;
                if (TriangleBvh.Ray(q, dir, a, e1, e2, out float d) && d > 0)
                    crossings++;
            }
            return (crossings & 1) == 1;
        }

        /// <summary>
        /// Clips the part's mesh at the water: its wet polygons (contact faces left out) for the hull's strips, and the
        /// hydrostatic pressure on every wet piece (contact faces too: the other part's face cancels them, or the
        /// overlap counts twice), unless the part is sealed inside others (a ballast in a hull adds no buoyancy). wet: the
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
            // the water's surface as stock's buoyancy has it: the vessel's offset plus the part's own water level (0 on a
            // flat sea; Scatterer's wave interactions set it to the wave's height under the part's origin)
            double level = vessel.waterOffset + (part.partBuoyancy != null ? part.partBuoyancy.waterLevel : 0);
            double depth0 = level - FlightGlobals.getAltitudeAtPos((Vector3d)p0, body);
            OriginDepth = (float)depth0;

            Matrix4x4 l2w = tf.localToWorldMatrix;
            bool any = false;
            float top = float.MaxValue, bottom = float.MinValue;
            for (int i = 0; i < localVerts.Length; i++)
            {
                Vector3 w = l2w.MultiplyPoint3x4(localVerts[i]);
                worldVerts[i] = w;
                float d = (float)(depth0 - Vector3.Dot(w - p0, up));
                depths[i] = d;
                any |= d > 0;
                top = Math.Min(top, d);
                bottom = Math.Max(bottom, d);
            }
            // the part's buoyancy, whether its mesh finds water or not (KSP may have the part under where its mesh is
            // not): stock's off where its pressure gives it, else held to its mesh's box
            if (OwnsBuoyancy && StockBuoyancy < 0)
            {
                StockBuoyancy = part.buoyancy;
                part.buoyancy = 0;
            }
            Gravity = FlightGlobals.getGeeForceAtPosition((Vector3d)p0).magnitude;
            if (!OwnsBuoyancy && part.partBuoyancy != null)
                HoldStockBuoyancy(rho);
            if (!any)
            {
                ProfileGatherTicks += Stopwatch.GetTimestamp() - t0;
                return false;
            }

            // the faces closed in by the part and its neighbours (a cargo bay's inside, an end cap facing into it): no
            // flow on them (their pressure kept: each part's buoyancy its own)
            bool[] shadow = Foil ? null : Shadowed();

            // the faces that meet another wet part this step
            stamp++;
            bool anyContact = false;
            foreach (PartHydrodynamics o in wet)
            {
                if (ReferenceEquals(o, this))
                    continue;
                (int[] c, int[] dry) = ContactsWith(o);
                if (c == null)
                    continue;
                anyContact = true;
                foreach (int k in c)
                    contactStamp[k] = stamp;
                if (dry != null)
                    foreach (int k in dry)
                        dryStamp[k] = stamp;
            }
            // sealed inside other parts: every face (its caps aside) dry
            bool sealedIn = anyContact;
            for (int k = 0; sealedIn && k < capStart; k++)
                sealedIn = dryStamp[k] == stamp;
            // (a mesh not closed: how much of it is buried, for its stock buoyancy's hold)
            if (!OwnsBuoyancy)
            {
                if (faceArea == null || faceArea.Length != capStart)
                {
                    faceArea = new float[capStart];
                    for (int k = 0; k < capStart; k++)
                    {
                        Vector3 a = localVerts[tris[3 * k]];
                        faceArea[k] = 0.5f * Vector3.Cross(localVerts[tris[3 * k + 1]] - a, localVerts[tris[3 * k + 2]] - a).magnitude;
                    }
                }
                double all = 0, buried = 0;
                for (int k = 0; k < capStart; k++)
                {
                    all += faceArea[k];
                    if (anyContact && dryStamp[k] == stamp)
                        buried += faceArea[k];
                }
                buriedShare = all > 0 ? buried / all : 0;
            }

            Vector3 com = part.rb.worldCenterOfMass;
            float scale = (float)(rho * Gravity);
            // (what is left open once its edges are capped: a weld missed, a loop not closed; nothing, usually)
            // the cap a nearly closed mesh lacks (its area vector -leakArea), at the mesh's middle: without it the pressure
            // on the rest pushes by depth times the leak (kN at a submarine's depth from a few cm2). With all of the mesh
            // under (the gap may be out of the water otherwise, where it has no pressure, and near the surface the leak's
            // error is small), faded in as the last 5% of its height goes under
            float leakIn = Mathf.Clamp01(1 + top / Math.Max(1e-3f, 0.05f * (bottom - top)));
            if (OwnsBuoyancy && !sealedIn && leakIn > 0 && leakArea.sqrMagnitude > 0)
            {
                Vector3 at = l2w.MultiplyPoint3x4(localBounds.center);
                float d = (float)Math.Max(0, depth0 - Vector3.Dot(at - p0, up));
                if (d > 0)
                {
                    Vector3 f = l2w.MultiplyVector(leakArea) * (scale * d * leakIn);
                    HsForces.Add(f);
                    HsTorques.Add(Vector3.Cross(at - com, f));
                    HsPoints.Add(at);
                }
            }

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
                // (a part sealed inside others is dry: no pressure)
                if (OwnsBuoyancy && !sealedIn)
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
                if (Foil || i / 3 >= capStart || anyContact && contactStamp[i / 3] == stamp || shadow != null && shadow[i / 3])
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
        public double Surface = 1;   // at the water's surface (1) or wholly under it (0): its sections', planform-weighted
        public int Strips;
        public readonly List<(double x, double width, double ma, double v, double jump, double load, bool separated, Vector3 point)> StripData =
            new List<(double, double, double, double, double, double, bool, Vector3)>();
        // its first wake's line as the afterbody meets it (where it starts along the flow from the first part's origin,
        // its depth there, its fall aft; m), lagged by the water's time along the hull, and when; NaN: none
        internal double WakeX = double.NaN, WakeD, WakeS, WakeTime = double.NaN;
        internal PartHydrodynamics WakeOf;   // the hull it was (its first part)
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
        private Strip[] tops = new Strip[PartHydrodynamics.MaxStrips];   // under the water: the top's mirrored strips
        // under the water: the sides' (faces looking left, right across the flow), their spans up and down
        private Strip[] ports = new Strip[PartHydrodynamics.MaxStrips], stbds = new Strip[PartHydrodynamics.MaxStrips];
        private Strip[][] mirrored;
        private Strip[][] Mirrored => mirrored ??= new[] { tops, ports, stbds };
        // each strip's section: its shallowest and deepest wet point (m below the water), and at the surface (1) or under it (0)
        private readonly float[] secTop = new float[PartHydrodynamics.MaxStrips], secBottom = new float[PartHydrodynamics.MaxStrips];
        private readonly double[] sec = new double[PartHydrodynamics.MaxStrips];
        // each section's area across the flow (m2; its wet surface's r.n over twice its length: a closed section's)
        private readonly double[] secArea = new double[PartHydrodynamics.MaxStrips];
        // the wet pieces, put in their strips once their sections' sec is known
        private struct Piece
        {
            public int Part, Strip;
            public float Area, Down, Lat, Vn, Axial;
            public Vector3 N, Centroid, V;
            public Vector2 SpanAcross, SpanUp;
        }
        private readonly List<Piece> pieces = new List<Piece>();
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
            public double JumpSteady;    // its share from the hull's speed alone (V from U along the flow; not heave or pitch rate)
            public double Load;          // the normal force it ends up carrying (N)
            public double Crossflow;     // the crossflow share of Load (N)
            public Vector3 Normal, Point;
            public double V, Width;
            public double Ma;            // its added mass per length (kg/m)
            public double SubLoad;       // under the water: its own U d(m_a V) (the afterbody's recovered share) and crossflow (N)
            public double SubCrossflow;  // the crossflow share of SubLoad (N)
            public bool Separated;       // its m_a V well below the run's peak: the flow has left the bottom
            public float WakeDepth;      // behind a run: the depth of its wake's surface here (m; -1: none, the flow on it)
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
                {
                    wet[i].LastHull = null;
                    wet.RemoveAt(i);
                }
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

        private static void ProfilePiece(long t0) => PartHydrodynamics.ProfilePieceTicks += Stopwatch.GetTimestamp() - t0;

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
            // (a foil's own group, or parts with no bottom to strip, host no foil: LastHull set once strips are found)
            foreach (PartHydrodynamics hd in parts)
            {
                hd.Hull = hull;
                hd.LastHull = null;
            }
            hull.WettedLength = hull.Planform = 0;
            hull.Surface = 1;
            hull.Limiter = 1;
            if (!ReferenceEquals(hull.WakeOf, first))
            {
                hull.WakeX = double.NaN;
                hull.WakeOf = first;
            }
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

            foreach (PartHydrodynamics hd in parts)
                if (!hd.Foil)
                    hd.LastHull = hull;
            double wettedLength = xMax - xMin;
            int nStrips = Mathf.Clamp(Mathf.CeilToInt((float)(wettedLength / PartHydrodynamics.StripLength)),
                                      PartHydrodynamics.MinStrips, PartHydrodynamics.MaxStrips);
            float dx = (float)(wettedLength / nStrips);
            Vector3 yHat = Vector3.Cross(up, xHat);
            for (int k = 0; k < nStrips; k++)
            {
                ResetStrip(ref strips[k]);
                ResetStrip(ref tops[k]);
                ResetStrip(ref ports[k]);
                ResetStrip(ref stbds[k]);
                secTop[k] = float.MaxValue;
                secBottom[k] = float.MinValue;
                secArea[k] = 0;
            }
            pieces.Clear();

            double re = Math.Max(1e5, U * wettedLength / KinematicViscosity);
            double lg = Math.Log10(re) - 2;
            double cf = 0.075 / (lg * lg);

            // under the water the top faces are a bottom too (each section's 1 - sec: how far under); at the surface a top
            // face pushed into the water takes impact pressure instead
            var impactOf = new Vector3[parts.Count];
            var torqueOf = new Vector3[parts.Count];   // of LastForce, about each part's CoM (N m)
            var impactTorqueOf = new Vector3[parts.Count];
            long piecesT0 = Stopwatch.GetTimestamp();
            for (int pi = 0; pi < parts.Count; pi++)
            {
                PartHydrodynamics hd = parts[pi];
                Rigidbody rb = hd.part.rb;
                // (its point velocities from its state: GetPointVelocity's, without a call into the engine per piece)
                Vector3 rbVel = rb.velocity + frame, rbOmega = rb.angularVelocity, rbCom = rb.worldCenterOfMass;
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
                        if (k0 == k1)
                        {
                            // (wholly in its strip: as it is)
                            scratchB.Clear();
                            for (int j = 0; j < count; j++)
                                scratchB.Add(hd.PolyVerts[at + j]);
                        }
                        else
                        {
                            scratchA.Clear();
                            for (int j = 0; j < count; j++)
                                scratchA.Add(hd.PolyVerts[at + j]);
                            float lo = xMin + k * dx, hi = lo + dx;
                            ClipSlab(scratchA, scratchB, p0, xHat, lo, hi);
                        }
                        if (scratchB.Count < 3)
                            continue;
                        PolygonAreaCentroid(scratchB, n, out float area, out Vector3 centroid);
                        if (area <= 0)
                            continue;
                        foreach (Vector3 q in scratchB)
                        {
                            float d = hd.OriginDepth - Vector3.Dot(q - hd.Origin, up);   // (its own water level's)
                            secTop[k] = Math.Min(secTop[k], d);
                            secBottom[k] = Math.Max(secBottom[k], d);
                        }

                        Vector3 v = rbVel + Vector3.Cross(rbOmega, centroid - rbCom);
                        float vn = Vector3.Dot(v, n);
                        // skin friction against the surface's own slip through the water
                        Vector3 vt = v - n * vn;
                        float vtm = vt.magnitude;
                        ref Share sh = ref ShareOf(ref strips[k], pi);
                        sh.WettedArea += area;
                        sh.FrictionWeight += area;
                        sh.FrictionPoint += centroid * area;
                        if (vtm > 1e-3f)
                            sh.Friction += -vt * (float)(0.5 * rho * vtm * cf * area);

                        // the piece, for its strips once its section's depth is known: its spans across (the bottom's and
                        // top's) and up and down (a side's); its section's area
                        float yLo = float.MaxValue, yHi = float.MinValue, uLo = float.MaxValue, uHi = float.MinValue;
                        foreach (Vector3 q in scratchB)
                        {
                            float y = Vector3.Dot(q - p0, yHat), u = Vector3.Dot(q - p0, up);
                            yLo = Math.Min(yLo, y);
                            yHi = Math.Max(yHi, y);
                            uLo = Math.Min(uLo, u);
                            uHi = Math.Max(uHi, u);
                        }
                        pieces.Add(new Piece
                        {
                            Part = pi, Strip = k, Area = area, Down = down, Lat = Vector3.Dot(n, yHat), Vn = vn,
                            Axial = Math.Abs(Vector3.Dot(n, xHat)), N = n, Centroid = centroid, V = v,
                            SpanAcross = new Vector2(yLo, yHi), SpanUp = new Vector2(uLo, uHi)
                        });
                        Vector3 r = centroid - p0;
                        secArea[k] += area * Vector3.Dot(r - xHat * Vector3.Dot(r, xHat), n - xHat * Vector3.Dot(n, xHat));
                    }
                    at += count;
                }
            }

            // each section at the surface (1: it breaks the water) or under it, fading to none at SubmergedFade of the
            // hull's deepest section's height below the surface
            float height = 0.1f;
            for (int k = 0; k < nStrips; k++)
                if (secBottom[k] > secTop[k])
                    height = Math.Max(height, secBottom[k] - secTop[k]);
            for (int k = 0; k < nStrips; k++)
                sec[k] = secBottom[k] < secTop[k] ? 1 : Math.Max(0, 1 - Math.Max(0, secTop[k]) / (PartHydrodynamics.SubmergedFade * height));

            ProfilePiece(piecesT0);
            long impactT0 = Stopwatch.GetTimestamp();
            // the pieces in their strips: a side's (looking across the flow, under the water), the bottom's or the top's,
            // from FaceThreshold on as far as their section is at the surface, ramped in over a band of 0.1 as far as it
            // is under (at the surface a step, as the planing model has it; under the water a face sinking through the
            // fade comes in by degrees, its area and its span, not all at once); a face exactly on edge in none
            foreach (Piece pc in pieces)
            {
                int k = pc.Strip;
                float t = Math.Max(1e-3f, (float)(PartHydrodynamics.FaceThreshold * sec[k])), band = (float)(0.1 * (1 - sec[k]));
                float Share(float x) => band < 1e-4f ? (x > t ? 1 : 0) : Mathf.Clamp01((x - t) / band);
                float side = Share(Math.Abs(pc.Lat)), bottom = Share(pc.Down), top = Share(-pc.Down);
                if (side > 0)
                    AddPiece(ref (pc.Lat > 0 ? ref stbds[k] : ref ports[k]), pc.Part, pc.Area * Math.Abs(pc.Lat) * side, pc.N, pc.Centroid, pc.V,
                             Shrunk(pc.SpanUp, side));
                if (bottom > 0)
                    AddPiece(ref strips[k], pc.Part, pc.Area * pc.Down * bottom, pc.N, pc.Centroid, pc.V, Shrunk(pc.SpanAcross, bottom));
                else if (top > 0)
                    AddPiece(ref tops[k], pc.Part, pc.Area * -pc.Down * top, pc.N, pc.Centroid, pc.V, Shrunk(pc.SpanAcross, top));

                // a face pushed into the water, as much as the bottom's strip does not take it (and the flow reaches it,
                // Lee): at the surface as much as
                // its section is there; under the water only a bluff one (looking along the flow, |n.x| 0.9 to 1: a flat
                // end's stagnation pressure, its wake not giving it back), a slender body's nose and sides none (their
                // strips take the flow, the pressure on them recovered further aft)
                if (pc.Vn <= 0)
                    continue;
                float w = (1 - bottom) * (float)(sec[k] + (1 - sec[k]) * Mathf.Clamp01((pc.Axial - 0.9f) / 0.1f));
                if (w <= 0)
                    continue;
                // (the flow must reach it: a face in the lee of the hull, the body right ahead of it upstream, takes no
                // stagnation pressure: a lip behind the 11-A's sternpost, an end cap facing into a cargo bay)
                long leeT0 = Stopwatch.GetTimestamp();
                bool lee = parts[pc.Part].Lee(pc.Centroid + pc.N * 0.002f, pc.V);
                PartHydrodynamics.ProfileLeeTicks += Stopwatch.GetTimestamp() - leeT0;
                PartHydrodynamics.ProfileLeeRays++;
                if (lee)
                    continue;
                Vector3 f = -pc.N * (float)(0.5 * PartHydrodynamics.ImpactPressureCoefficient * rho * pc.Vn * pc.Vn * pc.Area * w);
                Rigidbody rb = parts[pc.Part].part.rb;
                impactOf[pc.Part] += f;
                impactTorqueOf[pc.Part] += Vector3.Cross(pc.Centroid - rb.worldCenterOfMass, f);
            }
            // (each part's impact once: its force and its moment about its CoM, as the pieces' at their points)
            for (int pi = 0; pi < parts.Count; pi++)
                if (impactOf[pi] != Vector3.zero)
                {
                    torqueOf[pi] += impactTorqueOf[pi];
                    parts[pi].part.AddForce((Vector3d)impactOf[pi] * 0.001);
                    parts[pi].part.AddTorque((Vector3d)impactTorqueOf[pi] * 0.001);
                }
            PartHydrodynamics.ProfileImpactTicks += Stopwatch.GetTimestamp() - impactT0;

            // (under: the most any wet section is under; each section's own sec blends its terms)
            double under = 0, secSum = 0, secWeight = 0;
            for (int k = 0; k < nStrips; k++)
            {
                if (strips[k].Planform > 0 || tops[k].Planform > 0 || ports[k].Planform > 0 || stbds[k].Planform > 0)
                    under = Math.Max(under, 1 - sec[k]);
                secSum += sec[k] * strips[k].Planform;
                secWeight += strips[k].Planform;
            }
            double surface = secWeight > 0 ? secSum / secWeight : 1;
            hull.Surface = surface;

            // momentum, bottom and (under the water) top and sides
            double ka = PartHydrodynamics.AddedMassCoefficient * rho * Math.PI / 2;
            // (the steady U d(m_a V)/dx alone: strip theory's time derivative, each section's V dm_a/dt as it wets or dries
            // with the hull's heave and pitch (2D+t, Zarnick's), applied explicitly launched a light hull (NACA 11-A's
            // 23 kg tank model, its added mass near its own) out of the water as it was set in it, and did not damp the
            // floatplane's porpoising: the step's ventilation on the trim-dependent wetted beam drove that)
            double stiffness = Momentum(strips, nStrips, ka, rho, U, dx, secTop, secBottom, xHat, hull, xMin, wettedLength);
            if (under > 0)
            {
                foreach (Strip[] set in Mirrored)
                    Sections(set, nStrips, ka);
                // a section under the water: its added mass from its widths (half-width a across, b up and down) times
                // its area over an ellipse's of them, A / (pi a b): one for an ellipse, and for a faceted one (a 12-sided
                // collider) the same at any roll (its widths change as it turns, its area does not); fading to none at
                // the surface. Its area once however many copies of it the part's meshes hold (the bottom's planform over
                // its merged width, as friction counts them); held within 0.6 - 1.4 (a square's 2/pi rolled 45 deg, a
                // rectangle's 4/pi; beyond: a section not closed, several hulls side by side)
                for (int k = 0; k < nStrips; k++)
                {
                    double a = strips[k].Width / 2, b = (secBottom[k] - secTop[k]) / 2, area = secArea[k] / (2 * dx);
                    if (sec[k] >= 1 || a < 1e-4 || b < 1e-4 || area <= 0)
                        continue;
                    area /= Math.Max(1, strips[k].Planform / (strips[k].Width * dx));
                    double f = Math.Max(0.6, Math.Min(1.4, area / (Math.PI * a * b)));
                    double g = 1 + (f - 1) * (1 - sec[k]);
                    strips[k].Ma *= g;
                    tops[k].Ma *= g;
                    ports[k].Ma *= g;
                    stbds[k].Ma *= g;
                }
                stiffness += under * (MomentumSubmerged(strips, tops, nStrips, rho, U, dx) + MomentumSubmerged(tops, strips, nStrips, rho, U, dx) +
                                            MomentumSubmerged(ports, stbds, nStrips, rho, U, dx) + MomentumSubmerged(stbds, ports, nStrips, rho, U, dx));
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
            // below planing: the momentum loads' share and the squat's suction (LowSpeedLift), by the widest strip's C_V
            double beamWide = 0;
            for (int k = 0; k < nStrips; k++)
                beamWide = Math.Max(beamWide, strips[k].Width);
            double cvWide = U / Math.Sqrt(first.Gravity * Math.Max(0.05, beamWide));
            double rampX = Math.Max(0, Math.Min(1, (cvWide - 1) / Math.Max(0.1, PartHydrodynamics.LowSpeedLiftFullCv - 1)));
            double ramp = rampX * rampX * (3 - 2 * rampX);
            double liftShare = PartHydrodynamics.LowSpeedLift + (1 - PartHydrodynamics.LowSpeedLift) * ramp;
            double suction = PartHydrodynamics.LowSpeedSuction * (1 - ramp) * 0.5 * rho * U * U;   // N/m2 of planform
            double waveShare = PartHydrodynamics.WaveDrag * (1 - ramp) * Math.Min(1, cvWide * cvWide) * surface;   // (at the surface: a hull under the water makes no waves)

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
                        // (liftShare on the momentum's share only: the crossflow, the water's damping, kept)
                        double planing = (s.Load - s.Crossflow) * liftShare + s.Crossflow;
                        double load = under > 0 ? sec[k] * planing + (1 - sec[k]) * s.SubLoad : planing;
                        double cross = under > 0 ? sec[k] * s.Crossflow + (1 - sec[k]) * s.SubCrossflow : s.Crossflow;
                        Vector3 f = AcrossUnder(-s.Normal * (float)(load * limiter) * w, xHat, sec[k]);
                        // (the squat: the strip's planform pulled down, as much as its section is at the surface)
                        if (suction > 0)
                            f -= up * (float)(suction * s.Planform * sec[k] * limiter) * w;
                        momentumOf[sh.Part] += f;
                        crossflowOf[sh.Part] += AcrossUnder(-s.Normal * (float)(cross * limiter) * w, xHat, sec[k]);
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

            // the top's and sides' strips, under the water: their loads on the parts (no friction: the bottom's shares
            // carry it all)
            if (under > 0)
                foreach (Strip[] set in Mirrored)
                {
                    for (int k = 0; k < nStrips; k++)
                    {
                        ref Strip s = ref set[k];
                        double topShare = 1 - sec[k];
                        if (s.Planform <= 0 || topShare <= 0)
                            continue;
                        for (int i = 0; i < s.ShareCount; i++)
                        {
                            ref Share sh = ref s.Shares[i];
                            if (sh.Planform <= 0)
                                continue;
                            PartHydrodynamics hd = parts[sh.Part];
                            float w = (float)(sh.Planform / s.Planform * topShare);
                            Vector3 f = AcrossUnder(-s.Normal * (float)(s.SubLoad * limiter) * w, xHat, sec[k]);
                            momentumOf[sh.Part] += f;
                            crossflowOf[sh.Part] += AcrossUnder(-s.Normal * (float)(s.SubCrossflow * limiter) * w, xHat, sec[k]);
                            Vector3 at = sh.CentroidSum / (float)sh.Planform;
                            torqueOf[sh.Part] += Vector3.Cross(at - hd.part.rb.worldCenterOfMass, f);
                            hd.part.AddForceAtPosition((Vector3d)f * 0.001, (Vector3d)at);
                        }
                    }
                }

            // hydrostatics at speed: once the flow leaves the transom dry, the bottom near it carries no hydrostatic
            // pressure (Savitsky's buoyancy term is about half the displacement's; Zarnick scales it by 0.5): the
            // share kept fades from all of it to PlaningBuoyancy from C_V PlaningBuoyancyStartCv to PlaningBuoyancyFullCv on
            // the mean wetted beam
            double take = 0, vented = 0;
            if (planformTotal > 0 && wettedLength > 0)
            {
                double beam = planformTotal / wettedLength;
                double cv = U / Math.Sqrt(first.Gravity * Math.Max(0.05, beam));
                double x = Math.Max(0, Math.Min(1, (cv - PartHydrodynamics.PlaningBuoyancyStartCv) /
                                                   Math.Max(0.1, PartHydrodynamics.PlaningBuoyancyFullCv - PartHydrodynamics.PlaningBuoyancyStartCv)));
                // (planing only: a section under the water keeps all of its buoyancy, and no step there ventilates)
                take = (1 - PartHydrodynamics.PlaningBuoyancy) * x * x * (3 - 2 * x);
                double xv = Math.Max(0, Math.Min(1, (cvWide - PartHydrodynamics.VentStartCv) /
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
                        // in a wake (behind a run), the share of the pressure the hollow above its surface takes, as the step
                        // ventilates: all of it above the surface, the wake's depth's share of it below
                        float wake = 0;
                        if (strips[k].WakeDepth >= 0)
                        {
                            float depth = hd.OriginDepth - Vector3.Dot(hd.HsPoints[i] - hd.Origin, up);
                            wake = depth <= strips[k].WakeDepth ? 1 : strips[k].WakeDepth / depth;
                        }
                        else if (strips[k].Separated)
                            wake = 1;
                        float w = (float)((1 - take * sec[k]) * (1 - vented * sec[k] * wake));
                        hsForce += hd.HsForces[i] * w;
                        hsTorque += hd.HsTorques[i] * w;
                    }
                    float keep = (float)(hd.StockBuoyancy >= 0 ? hd.StockBuoyancy : 1);
                    hydrostatic = hsForce * (keep * 0.001f);
                    hd.LastHydrostaticTorque = hsTorque * (keep * 0.001f);
                    hd.part.AddForce((Vector3d)hydrostatic);
                    hd.part.AddTorque((Vector3d)hd.LastHydrostaticTorque);
                }
                else if (take > 0 && pb != null)
                {
                    hydrostatic = -pb.lastBuoyantForce * (float)(take * surface);
                    hd.part.AddForceAtPosition((Vector3d)hydrostatic, (Vector3d)pb.lastForcePosition);
                }
                hd.LastHydrostaticForce = hydrostatic;
                // the waves' drag: its share of the displacement (as its hydrostatics hold it up), at its CoM
                Vector3 wave = Vector3.zero;
                if (waveShare > 0)
                {
                    float held = Math.Max(0, Vector3.Dot(hydrostatic, up));
                    wave = -xHat * (float)(waveShare * held);
                    if (held > 0)
                        hd.part.AddForce((Vector3d)wave);
                }
                hd.LastWaveForce = wave;
                hd.LastMomentumForce = (momentumOf[pi] - crossflowOf[pi]) * 0.001f;
                hd.LastCrossflowForce = crossflowOf[pi] * 0.001f;
                hd.LastFrictionForce = frictionOf[pi] * 0.001f;
                hd.LastImpactForce = impactOf[pi] * 0.001f;
                hd.LastTorque = torqueOf[pi] * 0.001f;
                hd.LastForce = (momentumOf[pi] + frictionOf[pi] + impactOf[pi]) * 0.001f + wave;
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

        // a strip's force with its share along the flow (xHat) taken off as far as its section is under the water
        // (sec 0: all of it; at the surface, 1: none)
        private static Vector3 AcrossUnder(Vector3 f, Vector3 xHat, double sec)
        {
            float under = (float)(1 - sec);
            return under > 0 ? f - xHat * (Vector3.Dot(f, xHat) * under) : f;
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

        // under the water (after Momentum, which sets each strip's normal, width, V and m_a): bow to stern, each strip's
        // own U d(m_a V) where it is, a shrinking section's times SubmergedRecovery, and crossflow drag on the flow
        // across the body (its SubLoad): half the difference of its V and its mirror's (the strip on the body's other
        // side, the top's for the bottom's, the port's for the starboard's: the body's own taper moves both alike, the
        // flow across it them oppositely; a tapering end's V is no crossflow). Returns the damping these forces put on
        // the vessel (N s/m)
        private static double MomentumSubmerged(Strip[] ss, Strip[] mirror, int nStrips, double rho, double U, float dx)
        {
            double maPrevV = 0, maPrev = 0, maMax = 0, crossflow = 0;
            for (int k = nStrips - 1; k >= 0; k--)
            {
                ref Strip s = ref ss[k];
                s.SubLoad = s.SubCrossflow = 0;
                if (s.Planform <= 0)
                {
                    maPrevV = maPrev = 0;
                    continue;
                }
                double maV = s.Ma * s.V;
                double jump = U * (maV - maPrevV);
                if (s.Ma < maPrev)
                    jump *= PartHydrodynamics.SubmergedRecovery;
                maPrevV = maV;
                maPrev = s.Ma;
                maMax = Math.Max(maMax, s.Ma);
                double vc = mirror[k].Planform > 0 ? (s.V - mirror[k].V) / 2 : s.V;
                double cd = PartHydrodynamics.SubmergedCrossflowDrag;
                double fcf = 0.5 * cd * rho * s.Width * dx * vc * Math.Abs(vc);
                crossflow += cd * rho * s.Width * dx * Math.Abs(vc);
                s.SubCrossflow = fcf;
                s.SubLoad = jump + fcf;
            }
            return U * maMax + crossflow;
        }

        // bow (largest x) to stern: each strip's momentum jump U d(m_a V) and crossflow drag (its Load), a dry strip ending
        // a wetted run (the flow leaves the bottom), a section well below the run's peak m_a V in the separated wake;
        // then each jump spread aft over the rest of its wetted run with the flat-plate loading. Returns the damping
        // these forces put on the vessel (N s/m), for the explicit step's limiter
        private static double Momentum(Strip[] ss, int nStrips, double ka, double rho, double U, float dx, float[] top, float[] bottom,
                                       Vector3 xHat, HullResult hull, float xMin, double wettedLength)
        {
            // the first wake's line, lagged: the water reaching the afterbody left the step the time it takes to run along
            // the hull ago (and the strips' binning moves where a run ends by a strip from step to step)
            double now = Planetarium.GetUniversalTime(), dtWake = now - hull.WakeTime;
            double lag = 1 - Math.Exp(-Math.Max(0, dtWake) / Math.Max(0.02, wettedLength / Math.Max(1, U)));
            bool firstWake = true, wakeSeen = false;
            // (each jump's steady share: from the speed along the flow alone, V's U n.x)
            double maPrevV = 0, maVPeak = 0, stiffness = 0, maPrevVs = 0, maPrevWakeS = 0;
            // the run on the bottom (its first strip; -1: none) and, once the flow has left it, its wake: from the strip it
            // left (its deepest point's depth there), falling aft at a slope
            int runStart = -1, wakeFrom = -1;
            float wakeDepth = 0, wakeSlope = 0;
            double maPrevWake = 0;
            for (int k = nStrips - 1; k >= 0; k--)
            {
                ref Strip s = ref ss[k];
                s.WakeDepth = -1;
                if (s.Planform <= 0)
                {
                    maPrevV = maVPeak = maPrevVs = 0;
                    runStart = wakeFrom = -1;
                    continue;
                }
                Section(ref s, ka);
                double ma = s.Ma;
                double maV = ma * s.V;
                maVPeak = Math.Max(maVPeak, maV);
                s.Separated = maVPeak > 0 && maV < (1 - PartHydrodynamics.SeparationDrop) * maVPeak;
                if (!s.Separated)
                {
                    if (runStart < 0 || wakeFrom >= 0)
                        runStart = k;
                    wakeFrom = -1;
                }
                else if (wakeFrom < 0 && runStart > k + 1)
                {
                    // the flow leaves the run at the strip ahead: its wake starts at that strip's deepest point, falling
                    // at WakeSlope of the run's keel slope (its deepest points' fall along it)
                    wakeFrom = k + 1;
                    wakeDepth = bottom[k + 1];
                    wakeSlope = (float)(PartHydrodynamics.WakeSlope *
                                        Math.Max(0, (bottom[k + 1] - bottom[runStart]) / ((runStart - k - 1) * dx)));
                    maPrevWake = maPrevWakeS = 0;
                    if (firstWake)
                    {
                        firstWake = false;
                        wakeSeen = true;
                        double x = xMin + (k + 1.5) * dx;
                        if (double.IsNaN(hull.WakeX) || !(dtWake > 0 && dtWake < 0.5))
                        {
                            hull.WakeX = x;
                            hull.WakeD = wakeDepth;
                            hull.WakeS = wakeSlope;
                        }
                        else
                        {
                            hull.WakeX += (x - hull.WakeX) * lag;
                            hull.WakeD += (wakeDepth - hull.WakeD) * lag;
                            hull.WakeS += (wakeSlope - hull.WakeS) * lag;
                        }
                        hull.WakeTime = now;
                        // the line from the lagged origin, in the strips' terms
                        wakeFrom = k + 1;
                        wakeDepth = (float)(hull.WakeD + (hull.WakeX - (xMin + (k + 1.5) * dx)) * hull.WakeS);
                        wakeSlope = (float)hull.WakeS;
                    }
                }
                double vSteady = U * Vector3.Dot(xHat, s.Normal);
                if (s.Separated && wakeFrom >= 0)
                {
                    float zw = wakeDepth + (wakeFrom - k) * dx * wakeSlope;
                    s.WakeDepth = Math.Max(0, zw);
                    // reaching below the wake: wetted again by its water, which falls at the wake's slope; as much of the
                    // section as is below the line (its wetted width that share, its added mass the square)
                    float reach = Mathf.Clamp01((bottom[k] - zw) / Math.Max(1e-3f, bottom[k] - top[k]));
                    double vRel = s.V - U * wakeSlope;
                    if (reach > 0 && vRel > 0)
                    {
                        s.Separated = false;
                        double maW = ma * reach * reach;
                        s.Jump = U * (maW * vRel - maPrevWake);
                        maPrevWake = maW * vRel;
                        double vRelS = Math.Max(0, vSteady - U * wakeSlope);
                        s.JumpSteady = U * (maW * vRelS - maPrevWakeS);
                        maPrevWakeS = maW * vRelS;
                        stiffness = Math.Max(stiffness, U * maW);
                        double fw = 0.5 * PartHydrodynamics.CrossflowDragIn * rho * s.Width * reach * dx * vRel * vRel;
                        stiffness += PartHydrodynamics.CrossflowDragIn * rho * s.Width * reach * dx * vRel;
                        s.Load = s.Crossflow = fw;
                        continue;
                    }
                    maPrevWake = maPrevWakeS = 0;
                }
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
                s.JumpSteady = U * (ma * vSteady - maPrevVs);
                maPrevVs = ma * vSteady;
                stiffness = Math.Max(stiffness, U * ma);
                double cd = s.V > 0 ? PartHydrodynamics.CrossflowDragIn : PartHydrodynamics.CrossflowDragOut;
                double fcf = 0.5 * cd * rho * s.Width * dx * s.V * Math.Abs(s.V);
                stiffness += cd * rho * s.Width * dx * Math.Abs(s.V);
                s.Load = s.Crossflow = fcf;
            }

            if (!wakeSeen)
                hull.WakeX = double.NaN;

            // each wetted run (from its spray root aft to where the flow leaves): each jump spread aft over the rest of the
            // run with the flat-plate loading, and (RunLoading) of the steady jumps the run's whole spread so from its spray
            // root instead
            double w = Math.Max(0, Math.Min(1, PartHydrodynamics.RunLoading));
            for (int k = nStrips - 1; k >= 0; k--)
            {
                if (ss[k].Planform <= 0 || ss[k].Separated || k + 1 < nStrips && ss[k + 1].Planform > 0 && !ss[k + 1].Separated)
                    continue;
                int end = k;
                while (end - 1 >= 0 && ss[end - 1].Planform > 0 && !ss[end - 1].Separated)
                    end--;
                // (the jumps from the hull's heave and pitch rate stay where they are: the water's damping)
                double total = 0;
                for (int i = k; i >= end; i--)
                {
                    total += ss[i].JumpSteady;
                    double jump = ss[i].Jump - w * ss[i].JumpSteady;
                    if (jump == 0)
                        continue;
                    int span = i - end + 1;
                    for (int j = i; j >= end; j--)
                    {
                        double x0 = (double)(i - j) / span, x1 = (double)(i - j + 1) / span;
                        ss[j].Load += jump * (PlateLoad(x1) - PlateLoad(x0)) / (Math.PI / 2);
                    }
                }
                if (w <= 0 || total == 0)
                    continue;
                int runSpan = k - end + 1;
                for (int j = k; j >= end; j--)
                {
                    double x0 = (double)(k - j) / runSpan, x1 = (double)(k - j + 1) / runSpan;
                    ss[j].Load += w * total * (PlateLoad(x1) - PlateLoad(x0)) / (Math.PI / 2);
                }
            }
            return stiffness;
        }

        // a wet strip's section: its normal, point, wetted width, velocity into the water and added mass
        private static void Section(ref Strip s, double ka)
        {
            s.Normal = s.NormalSum.normalized;
            s.Point = s.CentroidSum / (float)s.Planform;
            // wetted widths: the merged lateral spans (copies and inner faces overlap, separate hulls don't)
            MergeSpans(s.Spans, out double width, out double sumHalfWidth2);
            s.Width = width;
            s.V = Vector3.Dot(s.VelocitySum / (float)s.Planform, s.Normal);
            s.Ma = ka * sumHalfWidth2;
        }

        private static void Sections(Strip[] ss, int nStrips, double ka)
        {
            for (int k = 0; k < nStrips; k++)
                if (ss[k].Planform > 0)
                    Section(ref ss[k], ka);
        }

        private static void ResetStrip(ref Strip s)
        {
            List<Vector2> spans = s.Spans ?? new List<Vector2>();
            Share[] shares = s.Shares ?? new Share[2];
            spans.Clear();
            s = default;
            s.Spans = spans;
            s.Shares = shares;
        }

        // a facing piece of a strip (planform: its area projected on the water plane), and its part's share of it
        private static void AddPiece(ref Strip st, int part, double planform, Vector3 n, Vector3 centroid, Vector3 v, Vector2 span)
        {
            st.Planform += planform;
            st.NormalSum += n * (float)planform;
            st.CentroidSum += centroid * (float)planform;
            st.VelocitySum += v * (float)planform;
            ref Share sh = ref ShareOf(ref st, part);
            sh.Planform += planform;
            sh.CentroidSum += centroid * (float)planform;
            st.Spans.Add(span);
        }

        // a span shrunk about its middle to a share of it (a face part way into a strip)
        private static Vector2 Shrunk(Vector2 span, float share)
        {
            if (share >= 1)
                return span;
            float mid = (span.x + span.y) / 2, half = (span.y - span.x) / 2 * share;
            return new Vector2(mid - half, mid + half);
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
