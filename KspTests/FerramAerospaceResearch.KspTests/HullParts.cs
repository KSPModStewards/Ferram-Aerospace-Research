using System;
using System.Collections.Generic;
using System.Linq;
using FerramAerospaceResearch.FARAeroComponents;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// A hull as one part or several (crafts/split_hull.py: the pieces of one Spine or prism, end to end along their
    /// common axis): its frame is the one-part hull's (the reference piece's rotation, the whole length's middle on the
    /// axis), its water forces the sum of its parts', so a test reads a split hull as it reads the one-part hull.
    /// </summary>
    internal sealed class HullParts
    {
        public readonly Part Ref;
        public readonly Part[] Parts;
        public readonly Vector3 Offset;     // the whole hull's middle in Ref's frame
        public readonly double Length;

        private HullParts(Part reference, Part[] parts, Vector3 offset, double length)
        {
            Ref = reference;
            Parts = parts;
            Offset = offset;
            Length = length;
        }

        public Vector3 TransformPoint(Vector3 local) => Ref.transform.TransformPoint(local + Offset);
        public Vector3 InverseTransformPoint(Vector3 world) => Ref.transform.InverseTransformPoint(world) - Offset;
        public Vector3 Position => TransformPoint(Vector3.zero);
        public Vector3 Forward => Ref.transform.up;
        public Quaternion Rotation => Ref.transform.rotation;

        // the part's extent along its own y (its spine or prism axis), from its collider mesh
        private static (float min, float max) Extent(Part p)
        {
            float lo = float.MaxValue, hi = float.MinValue;
            foreach (MeshCollider mc in p.GetComponentsInChildren<MeshCollider>())
            {
                if (mc.sharedMesh == null)
                    continue;
                foreach (Vector3 w in mc.sharedMesh.vertices)
                {
                    float y = p.transform.InverseTransformPoint(mc.transform.TransformPoint(w)).y;
                    lo = Math.Min(lo, y);
                    hi = Math.Max(hi, y);
                }
            }
            Assert.True(lo <= hi, $"{p.partInfo.name} has a mesh collider");
            return (lo, hi);
        }

        /// <summary>The hulls among the parts: pieces on one axis (parallel, origins in line), end to end.</summary>
        public static List<HullParts> Find(Vessel v, Func<Part, bool> isHull)
        {
            Part[] cand = v.parts.Where(isHull).ToArray();
            var ext = cand.Select(Extent).ToArray();
            int[] root = Enumerable.Range(0, cand.Length).ToArray();
            int Root(int i) => root[i] == i ? i : root[i] = Root(root[i]);
            for (int i = 0; i < cand.Length; i++)
                for (int j = i + 1; j < cand.Length; j++)
                {
                    Transform a = cand[i].transform, b = cand[j].transform;
                    if (Vector3.Dot(a.up, b.up) < 0.9999f)
                        continue;
                    Vector3 d = b.position - a.position;
                    float along = Vector3.Dot(d, a.up);
                    if ((d - a.up * along).magnitude > 0.01f)
                        continue;
                    // end to end: b's interval [along + min, along + max] touches a's
                    float bLo = along + ext[j].min, bHi = along + ext[j].max;
                    if (Math.Abs(bLo - ext[i].max) > 0.01f && Math.Abs(ext[i].min - bHi) > 0.01f)
                        continue;
                    root[Root(j)] = Root(i);
                }
            var hulls = new List<HullParts>();
            foreach (var g in Enumerable.Range(0, cand.Length).GroupBy(Root))
            {
                Part[] parts = g.Select(i => cand[i]).ToArray();
                Part reference = parts.Contains(v.rootPart) ? v.rootPart : parts[0];
                Transform rt = reference.transform;
                float lo = float.MaxValue, hi = float.MinValue;
                foreach (int i in g)
                {
                    float c = Vector3.Dot(cand[i].transform.position - rt.position, rt.up);
                    lo = Math.Min(lo, c + ext[i].min);
                    hi = Math.Max(hi, c + ext[i].max);
                }
                hulls.Add(new HullParts(reference, parts, new Vector3(0, (lo + hi) / 2, 0), hi - lo));
            }
            return hulls;
        }

        /// <summary>The vessel as one hull: every part of it a piece of the hull.</summary>
        public static HullParts Whole(Vessel v)
        {
            List<HullParts> hulls = Find(v, p => true);
            Assert.True(hulls.Count == 1, $"the vessel's {v.parts.Count} parts are one hull ({hulls.Count} found)");
            return hulls[0];
        }

        /// <summary>
        /// The vessel as one hull of any parts (a skin of plates, not pieces on one axis) in its root part's frame, the
        /// hull's frame origin at offset in it: the root turned as a one-part hull is (crafts/naca_11a_pw.py's strut).
        /// Its length along the root's y from every part's collider.
        /// </summary>
        public static HullParts Framed(Vessel v, Vector3 offset)
        {
            Part root = v.rootPart;
            Transform rt = root.transform;
            float lo = float.MaxValue, hi = float.MinValue;
            foreach (Part p in v.parts)
                foreach (MeshCollider mc in p.GetComponentsInChildren<MeshCollider>())
                    if (mc.sharedMesh != null)
                        foreach (Vector3 w in mc.sharedMesh.vertices)
                        {
                            float y = rt.InverseTransformPoint(mc.transform.TransformPoint(w)).y;
                            lo = Math.Min(lo, y);
                            hi = Math.Max(hi, y);
                        }
            return new HullParts(root, v.parts.ToArray(), offset, hi - lo);
        }

        // the water on its parts (FAR's PartHydrodynamics; zero for a part with none)
        public IEnumerable<PartHydrodynamics> Hydro =>
            Parts.Select(p => p.Modules.GetModule<FARAeroPartModule>()?.Hydrodynamics).Where(h => h != null);
        public Vector3 Sum(Func<PartHydrodynamics, Vector3> f) => Hydro.Aggregate(Vector3.zero, (s, h) => s + f(h));
        public double WettedLength => Hydro.Select(h => h.LastWettedLength).DefaultIfEmpty(0).Max();
        public double Planform => Hydro.Select(h => h.LastPlanform).DefaultIfEmpty(0).Max();
        public double Limiter => Hydro.Select(h => h.LastLimiter).DefaultIfEmpty(1).Min();
        public bool OwnsBuoyancy => Parts.All(p => p.Modules.GetModule<FARAeroPartModule>()?.Hydrodynamics?.OwnsBuoyancy == true);
        public double MeshVolume => Hydro.Sum(h => h.MeshVolume);
        public double Submerged => Parts.Sum(p => p.submergedPortion * p.mass) / Math.Max(1e-9, Parts.Sum(p => p.mass));
        public Vector3 StockBuoyancy => Parts.Aggregate(Vector3.zero, (s, p) => s + (p.partBuoyancy != null ? p.partBuoyancy.effectiveForce : Vector3.zero));
        public List<(double x, double width, double ma, double v, double jump, double load, bool separated, Vector3 point)> StripData =>
            Hydro.Select(h => h.Hull).FirstOrDefault(h => h != null)?.StripData ??
            new List<(double, double, double, double, double, double, bool, Vector3)>();

        /// <summary>How far (deg, m) the hull's parts are from their places relative to Ref as built.</summary>
        public (double deg, double m) Bend()
        {
            double deg = 0, m = 0;
            Quaternion toWorld = Ref.transform.rotation * Quaternion.Inverse(Ref.orgRot);
            foreach (Part p in Parts)
            {
                Quaternion want = toWorld * p.orgRot;
                Vector3 wantPos = Ref.transform.position + toWorld * (p.orgPos - Ref.orgPos);
                deg = Math.Max(deg, Quaternion.Angle(want, p.transform.rotation));
                m = Math.Max(m, (wantPos - p.transform.position).magnitude);
            }
            return (deg, m);
        }

        /// <summary>The hull's parts put back in their places relative to Ref as built, moving with it.</summary>
        public void Snap()
        {
            Quaternion toWorld = Ref.transform.rotation * Quaternion.Inverse(Ref.orgRot);
            Rigidbody rr = Ref.rb;
            foreach (Part p in Parts)
            {
                if (p == Ref || p.rb == null)
                    continue;
                Quaternion r = toWorld * p.orgRot;
                Vector3 pos = Ref.transform.position + toWorld * (p.orgPos - Ref.orgPos);
                p.rb.rotation = r;
                p.rb.position = pos;
                p.transform.rotation = r;
                p.transform.position = pos;
                p.rb.angularVelocity = rr.angularVelocity;
                p.rb.velocity = rr.GetPointVelocity(p.rb.worldCenterOfMass);
            }
        }

        /// <summary>The vessel's CoM put at a point of the hull's frame: every part's CoM moved by the same amount.</summary>
        public void SetCoM(Vessel v, Vector3 local)
        {
            Vector3 d = TransformPoint(local) - RigidVessel.CoM(v);
            foreach (Part p in v.parts)
            {
                p.CoMOffset += p.transform.InverseTransformDirection(d);
                if (p.rb != null)
                    p.rb.centerOfMass = p.CoMOffset;
            }
        }
    }

    /// <summary>The vessel moved as one rigid body (its parts' rigidbodies kept consistent with it).</summary>
    internal static class RigidVessel
    {
        public static double Mass(Vessel v) => v.parts.Where(p => p.rb != null).Sum(p => (double)p.rb.mass);

        public static Vector3 CoM(Vessel v)
        {
            Vector3 s = Vector3.zero;
            float m = 0;
            foreach (Part p in v.parts)
                if (p.rb != null)
                {
                    s += p.rb.worldCenterOfMass * p.rb.mass;
                    m += p.rb.mass;
                }
            return s / m;
        }

        public static Vector3 Velocity(Vessel v)
        {
            Vector3 s = Vector3.zero;
            float m = 0;
            foreach (Part p in v.parts)
                if (p.rb != null)
                {
                    s += p.rb.velocity * p.rb.mass;
                    m += p.rb.mass;
                }
            return s / m;
        }

        // the angular velocity of the whole: its angular momentum about the CoM over its inertia there (one part: that
        // part's own). Not the parts' rates averaged: the joints pass moments between the parts in equal and opposite
        // pairs, which an average by mass does not cancel, and resetting the vessel to it each step puts a moment on it.
        public static Vector3 AngularVelocity(Vessel v)
        {
            if (v.parts.Count(p => p.rb != null) == 1)
                return v.parts.First(p => p.rb != null).rb.angularVelocity;
            Vector3 c = CoM(v);
            double[,] inertia = new double[3, 3];
            foreach (Part p in v.parts)
            {
                Rigidbody rb = p.rb;
                if (rb == null)
                    continue;
                // the part's own inertia in world axes, and its mass's about the vessel's CoM (parallel axes)
                Quaternion pr = rb.rotation * rb.inertiaTensorRotation;
                Vector3 r = rb.worldCenterOfMass - c;
                for (int i = 0; i < 3; i++)
                {
                    Vector3 ei = Vector3.zero;
                    ei[i] = 1;
                    Vector3 local = Quaternion.Inverse(pr) * ei;
                    Vector3 col = pr * Vector3.Scale(local, rb.inertiaTensor);
                    for (int j = 0; j < 3; j++)
                        inertia[j, i] += col[j] + rb.mass * ((i == j ? r.sqrMagnitude : 0) - r[i] * r[j]);
                }
            }
            return Solve3(inertia, AngularMomentum(v));
        }

        private static Vector3 Solve3(double[,] a, Vector3 b)
        {
            double det = a[0, 0] * (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1]) - a[0, 1] * (a[1, 0] * a[2, 2] - a[1, 2] * a[2, 0]) +
                         a[0, 2] * (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0]);
            Vector3 x = Vector3.zero;
            for (int k = 0; k < 3; k++)
            {
                var m = (double[,])a.Clone();
                for (int i = 0; i < 3; i++)
                    m[i, k] = b[i];
                double dk = m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0]) +
                            m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
                x[k] = (float)(dk / det);
            }
            return x;
        }

        /// <summary>Every part moving with the whole: its CoM at velocity, turning at omega.</summary>
        public static void Set(Vessel v, Vector3 velocity, Vector3 omega)
        {
            Vector3 c = CoM(v);
            foreach (Part p in v.parts)
                if (p.rb != null)
                {
                    p.rb.velocity = velocity + Vector3.Cross(omega, p.rb.worldCenterOfMass - c);
                    p.rb.angularVelocity = omega;
                }
        }

        /// <summary>The whole's angular momentum about its CoM (kN m s with masses in t).</summary>
        public static Vector3 AngularMomentum(Vessel v)
        {
            Vector3 c = CoM(v), vc = Velocity(v), l = Vector3.zero;
            foreach (Part p in v.parts)
            {
                Rigidbody rb = p.rb;
                if (rb == null)
                    continue;
                Quaternion pr = rb.rotation * rb.inertiaTensorRotation;
                Vector3 wP = Quaternion.Inverse(pr) * rb.angularVelocity;
                l += pr * Vector3.Scale(wP, rb.inertiaTensor);
                l += Vector3.Cross(rb.worldCenterOfMass - c, rb.velocity - vc) * rb.mass;
            }
            return l;
        }

        /// <summary>
        /// The largest angle (deg) and offset (m) of any part from its place in the craft as built, relative to the
        /// reference part: how far the joints have let the vessel bend.
        /// </summary>
        public static (double deg, double m) Bend(Vessel v, Part reference)
        {
            double deg = 0, m = 0;
            Quaternion toWorld = reference.transform.rotation * Quaternion.Inverse(reference.orgRot);
            foreach (Part p in v.parts)
            {
                Quaternion want = toWorld * p.orgRot;
                Vector3 wantPos = reference.transform.position + toWorld * (p.orgPos - reference.orgPos);
                deg = Math.Max(deg, Quaternion.Angle(want, p.transform.rotation));
                m = Math.Max(m, (wantPos - p.transform.position).magnitude);
            }
            return (deg, m);
        }

        /// <summary>Every part put back in its place in the craft as built, relative to the reference part.</summary>
        public static void Snap(Vessel v, Part reference)
        {
            Quaternion toWorld = reference.transform.rotation * Quaternion.Inverse(reference.orgRot);
            foreach (Part p in v.parts)
            {
                if (p == reference || p.rb == null)
                    continue;
                Quaternion r = toWorld * p.orgRot;
                Vector3 pos = reference.transform.position + toWorld * (p.orgPos - reference.orgPos);
                p.rb.rotation = r;
                p.rb.position = pos;
                p.transform.rotation = r;
                p.transform.position = pos;
            }
        }

        /// <summary>The whole turned so the reference part has rotation, about the reference part's position.</summary>
        public static void Rotate(Vessel v, Part reference, Quaternion rotation)
        {
            Quaternion q = rotation * Quaternion.Inverse(reference.rb.rotation);
            Vector3 pivot = reference.rb.position;
            foreach (Part p in v.parts)
            {
                if (p.rb == null)
                    continue;
                Quaternion r = q * p.rb.rotation;
                Vector3 pos = pivot + q * (p.rb.position - pivot);
                p.rb.rotation = r;
                p.rb.position = pos;
                p.transform.rotation = r;
                p.transform.position = pos;
            }
        }
    }
}

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// TimeControl's hyper-warp for a test's long runs (parameter hyperWarp, default 4; 1 for real time): more physics
    /// steps a frame at the same step (physics accuracy 1), so the run is the same and only sooner. Needs TimeControl in
    /// the test's mods.
    /// </summary>
    internal static class TestWarp
    {
        /// <summary>
        /// The water tests' instance: AJE and Firespitter for the C172's engine and prop, TimeControl for the hyper-warp,
        /// CustomProceduralParts for the Spine hulls (CustomProceduralPartsPinned: its Spine collider of convex pieces, up
        /// colliderPoints a ring, 24 unless a test sets it (ColliderDetail), and a concave section's convex parts each, which
        /// a hull's two walls (NACA 11-A Shell) need), FARTestFloatplane for the test parts. To test a CustomProceduralParts
        /// checkout, put its GameData's path (..\..\..\CustomProceduralParts\GameData\CustomProceduralParts, beside this
        /// repository) in its place.
        /// </summary>
        public const string WaterMods = "AdvancedJetEngine;Firespitter;TimeControl;" + CustomProceduralPartsPinned + ";" +
                                        @"..\FerramAerospaceResearch.KspTests.Run\mods\FARTestFloatplane";

        /// <summary>CustomProceduralParts' repository at a commit, built from its source.</summary>
        public const string CustomProceduralPartsPinned =
            "https://github.com/chambem42/CustomProceduralParts@b03d4398aba8c1c176b2cd401d9d0b671cc275da/GameData/CustomProceduralParts" +
            "?build=Source/CustomProceduralParts.csproj";

        /// <summary>
        /// The demos' and videos' instance: the water tests' and Scatterer's ocean, made calm with its waves moving craft
        /// (mods/FARTestScatterer), and the 40 m submarine's motor (mods/FARTestSubMotor). Not the water tests': they run
        /// without a GPU (batch mode), and their tows compare FAR's flat sea with tank data.
        /// </summary>
        public const string DemoMods = WaterMods + @";Scatterer;Scatterer-config;..\FerramAerospaceResearch.KspTests.Run\mods\FARTestScatterer;" +
                                       @"..\FerramAerospaceResearch.KspTests.Run\mods\FARTestSubMotor";

        /// <summary>
        /// Rough water: the water tests' and Scatterer's ocean with a metre and more of waves (mods/FARTestScattererRough:
        /// wind 5, amplitude 2), its waves moving craft. Needs a GPU (no batch mode): Scatterer reads its waves back from it.
        /// </summary>
        public const string RoughMods = WaterMods + @";Scatterer;Scatterer-config;..\FerramAerospaceResearch.KspTests.Run\mods\FARTestScattererRough";

        /// <summary>
        /// Hyper-warp on at parameter hyperWarp (default defaultRate), or off. A tow that holds its hull every physics step
        /// (its attitude and speed put back each step, measured over game time) runs the same steps at any rate: 16. One
        /// whose craft flies itself (an autopilot that may act once a frame) changes with it: 4.
        /// </summary>
        public static void Hyper(TestContext ctx, bool on, double defaultRate = 4)
        {
            double rate = ctx.ParamDouble("hyperWarp", defaultRate);
            rearmAt = 0;
            Rearmed = 0;
            if (!on || rate <= 1)
            {
                Flight.HyperWarpOff();
                return;
            }
            ctx.Record("hyperWarp", Flight.HyperWarp(rate, 1.0) ? $"{rate:F0}x attempted, time scale {Time.timeScale:F1}" : "refused (real time)");
        }

        /// <summary>
        /// Hyper-warp (as <see cref="Hyper" /> turned it on) back on when TimeControl has dropped it: it stops when a part
        /// of the active vessel is destroyed or the vessel is (a crash's breakup). At most every half second of game time;
        /// called each physics step of a run that wants it. Rearmed: the times it did since Hyper.
        /// </summary>
        public static void Keep(TestContext ctx, double defaultRate = 4)
        {
            double rate = ctx.ParamDouble("hyperWarp", defaultRate);
            if (rate <= 1 || Time.timeScale > 1.01f)
                return;
            double now = Planetarium.GetUniversalTime();
            if (now < rearmAt)
                return;
            rearmAt = now + 0.5;
            if (Flight.HyperWarp(rate, 1.0))
                Rearmed++;
        }

        public static int Rearmed;
        private static double rearmAt;
    }

    /// <summary>
    /// The water model's switches a test can set (parameter waterDamping: stock, explicit or none, the wet parts' stock
    /// water drag and angular drag: PartHydrodynamics.WaterDamping; byDefault: the test's own when not given), put back
    /// after it.
    /// </summary>
    internal static class WaterParams
    {
        private static PartHydrodynamics.WaterDampingMode? dampingWas;
        private static bool? linearFlapWas;
        private static double? recoveryWas, crossflowWas, stripWas, fadeWas, faceWas;
        private static bool? wholeWingWas, propwashWas, shapeBuoyancyWas;
        // any other of the model's constants (parameter hd.<name>, a public static double of PartHydrodynamics): as they were
        private static readonly Dictionary<System.Reflection.FieldInfo, double> constantsWas = new Dictionary<System.Reflection.FieldInfo, double>();

        public static void Apply(TestContext ctx, PartHydrodynamics.WaterDampingMode? byDefault = null)
        {
            dampingWas ??= PartHydrodynamics.WaterDamping;
            string p = ctx.Param("waterDamping");
            if (!string.IsNullOrEmpty(p))
                PartHydrodynamics.WaterDamping = (PartHydrodynamics.WaterDampingMode)Enum.Parse(typeof(PartHydrodynamics.WaterDampingMode), p, true);
            else if (byDefault.HasValue)
                PartHydrodynamics.WaterDamping = byDefault.Value;
            ctx.Record("waterDamping", PartHydrodynamics.WaterDamping.ToString());
            // FAR's linear flap lift (FARDebugValues.linearFlapLift; parameter linearFlapLift=true/false)
            linearFlapWas ??= FARDebugValues.linearFlapLift;
            string lf = ctx.Param("linearFlapLift");
            if (!string.IsNullOrEmpty(lf))
                if (FARDebugValues.linearFlapLift != bool.Parse(lf))
                {
                    FARDebugValues.linearFlapLift = bool.Parse(lf);
                    FARDebugValues.UpdateWingInteractions();
                }
            ctx.Record("linearFlapLift", FARDebugValues.linearFlapLift.ToString());
            // under the water: the afterbody's recovered share of its momentum (parameter submergedRecovery)
            recoveryWas ??= PartHydrodynamics.SubmergedRecovery;
            if (!double.IsNaN(ctx.ParamDouble("submergedRecovery", double.NaN)))
                PartHydrodynamics.SubmergedRecovery = ctx.ParamDouble("submergedRecovery", PartHydrodynamics.SubmergedRecovery);
            ctx.Record("submergedRecovery", PartHydrodynamics.SubmergedRecovery.ToString(System.Globalization.CultureInfo.InvariantCulture));
            crossflowWas ??= PartHydrodynamics.SubmergedCrossflowDrag;
            if (!double.IsNaN(ctx.ParamDouble("submergedCrossflow", double.NaN)))
                PartHydrodynamics.SubmergedCrossflowDrag = ctx.ParamDouble("submergedCrossflow", PartHydrodynamics.SubmergedCrossflowDrag);
            ctx.Record("submergedCrossflow", PartHydrodynamics.SubmergedCrossflowDrag.ToString(System.Globalization.CultureInfo.InvariantCulture));
            // how far under a section's top must be to be wholly submerged (parameter waterFade: PartHydrodynamics.
            // SubmergedFade; huge: every section at the surface, as the planing model had it)
            fadeWas ??= PartHydrodynamics.SubmergedFade;
            if (!double.IsNaN(ctx.ParamDouble("waterFade", double.NaN)))
                PartHydrodynamics.SubmergedFade = ctx.ParamDouble("waterFade", PartHydrodynamics.SubmergedFade);
            // the share of a face's normal from which it is a strip's at the surface (parameter faceThreshold:
            // PartHydrodynamics.FaceThreshold; none under the water)
            faceWas ??= PartHydrodynamics.FaceThreshold;
            if (!double.IsNaN(ctx.ParamDouble("faceThreshold", double.NaN)))
                PartHydrodynamics.FaceThreshold = ctx.ParamDouble("faceThreshold", PartHydrodynamics.FaceThreshold);
            ctx.Record("faceThreshold", PartHydrodynamics.FaceThreshold.ToString(System.Globalization.CultureInfo.InvariantCulture));
            // the hull's strip length (m; parameter stripLength)
            stripWas ??= PartHydrodynamics.StripLength;
            if (!double.IsNaN(ctx.ParamDouble("stripLength", double.NaN)))
                PartHydrodynamics.StripLength = ctx.ParamDouble("stripLength", PartHydrodynamics.StripLength);
            ctx.Record("stripLength", PartHydrodynamics.StripLength.ToString(System.Globalization.CultureInfo.InvariantCulture));
            foreach (System.Reflection.FieldInfo f in typeof(PartHydrodynamics).GetFields(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static))
            {
                if (f.FieldType != typeof(double) || f.IsLiteral || f.IsInitOnly)
                    continue;
                double x = ctx.ParamDouble("hd." + f.Name, double.NaN);
                if (double.IsNaN(x))
                    continue;
                if (!constantsWas.ContainsKey(f))
                    constantsWas[f] = (double)f.GetValue(null);
                f.SetValue(null, x);
                ctx.Record("hd." + f.Name, x.ToString(System.Globalization.CultureInfo.InvariantCulture));
            }
            // FAR's whole-wing aspect ratio (FARDebugValues.wholeWingAspectRatio; parameter wholeWingAspectRatio=true/false)
            // the propellers' slipstream on the wing panels (parameter propwash=true/false)
            propwashWas ??= FARDebugValues.propwash;
            string pw = ctx.Param("propwash");
            if (!string.IsNullOrEmpty(pw))
                FARDebugValues.propwash = bool.Parse(pw);
            ctx.Record("propwash", FARDebugValues.propwash.ToString());
            // buoyancy from the parts' shapes (parameter shapeBuoyancy=true/false; false: stock's as it is)
            shapeBuoyancyWas ??= FARDebugValues.shapeBuoyancy;
            string sb = ctx.Param("shapeBuoyancy");
            if (!string.IsNullOrEmpty(sb))
                FARDebugValues.shapeBuoyancy = bool.Parse(sb);
            ctx.Record("shapeBuoyancy", FARDebugValues.shapeBuoyancy.ToString());
            wholeWingWas ??= FARDebugValues.wholeWingAspectRatio;
            string ww = ctx.Param("wholeWingAspectRatio");
            if (!string.IsNullOrEmpty(ww) && FARDebugValues.wholeWingAspectRatio != bool.Parse(ww))
            {
                FARDebugValues.wholeWingAspectRatio = bool.Parse(ww);
                FARDebugValues.UpdateWingInteractions();
            }
        }

        /// <summary>The crush cap's worst on the loaded vessels (hd.CrushG; each vessel's least scale and its flooding).</summary>
        public static void RecordCrush(TestContext ctx)
        {
            if (PartHydrodynamics.CrushG <= 0)
                return;
            var lines = new List<string>();
            foreach (Vessel v in FlightGlobals.VesselsLoaded)
                if (v.GetComponent<FARVesselAero>()?.Hydrodynamics is VesselHydrodynamics vh && vh.CrushLeast < 1)
                    lines.Add($"{v.vesselName} ({v.parts.Count} parts, {v.totalMass:F1} t): least scale {vh.CrushLeast:F2}, taken off {vh.CrushDv:F1} m/s, flooded {vh.Flood:P0}");
            ctx.Record("crush", lines.Count > 0 ? string.Join("; ", lines) : "never capped");
        }

        public static void Restore()
        {
            if (propwashWas.HasValue)
                FARDebugValues.propwash = propwashWas.Value;
            propwashWas = null;
            if (shapeBuoyancyWas.HasValue)
                FARDebugValues.shapeBuoyancy = shapeBuoyancyWas.Value;
            shapeBuoyancyWas = null;
            foreach (var kv in constantsWas)
                kv.Key.SetValue(null, kv.Value);
            constantsWas.Clear();
            if (dampingWas.HasValue)
                PartHydrodynamics.WaterDamping = dampingWas.Value;
            dampingWas = null;
            if (recoveryWas.HasValue)
                PartHydrodynamics.SubmergedRecovery = recoveryWas.Value;
            recoveryWas = null;
            if (crossflowWas.HasValue)
                PartHydrodynamics.SubmergedCrossflowDrag = crossflowWas.Value;
            crossflowWas = null;
            if (fadeWas.HasValue)
                PartHydrodynamics.SubmergedFade = fadeWas.Value;
            fadeWas = null;
            if (faceWas.HasValue)
                PartHydrodynamics.FaceThreshold = faceWas.Value;
            faceWas = null;
            if (stripWas.HasValue)
                PartHydrodynamics.StripLength = stripWas.Value;
            stripWas = null;
            if (wholeWingWas.HasValue && FARDebugValues.wholeWingAspectRatio != wholeWingWas.Value)
            {
                FARDebugValues.wholeWingAspectRatio = wholeWingWas.Value;
                FARDebugValues.UpdateWingInteractions();
            }
            wholeWingWas = null;
            if (linearFlapWas.HasValue)
                if (FARDebugValues.linearFlapLift != linearFlapWas.Value)
                {
                    FARDebugValues.linearFlapLift = linearFlapWas.Value;
                    FARDebugValues.UpdateWingInteractions();
                }
            linearFlapWas = null;
        }
    }
}
