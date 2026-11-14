using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Reflection;
using ferram4;
using KspTest;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's whole-wing aspect ratio, on planecraft's B-2 Spirit's B9 panels (its RO parts dropped, a probe core for a
    /// root), three ways (crafts/, made by derive_b2*.py): "B-2 Spirit FAR", a flying wing, its two centre panels lifting
    /// and meeting on the centerline, a ProceduralParts nacelle over each side's second panel; "B-2 Spirit Fuselage", a
    /// ProceduralParts fuselage over the centre panels instead, no nacelles; and "B-2 Spirit Belly", that with two exposed
    /// panels under the fuselage. A wing spans the centerline when its lifting panels reach across it.
    /// </summary>
    public static class FlyingWingTests
    {
        private static readonly Type GroupType = typeof(FARWingInteraction).GetNestedType("WingGroup", BindingFlags.NonPublic);

        private static object GroupOf(FARWingAerodynamicModel w) => w.WingInteraction == null ? null : Refl.Get(w.WingInteraction, "wingGroup");

        // a WingGroup's field or parameterless method
        private static T Member<T>(object group, string name)
        {
            FieldInfo f = Refl.Find(GroupType, name);
            if (f != null) return (T)f.GetValue(group);
            MethodInfo m = GroupType.GetMethod(name, Refl.Members, null, Type.EmptyTypes, null) ?? throw new UITestFailure($"WingGroup has no {name}");
            return (T)m.Invoke(group, null);
        }

        private static List<FARWingAerodynamicModel> Members(object group) => Member<List<FARWingAerodynamicModel>>(group, "Members");
        private static int Bridges(object group) => Refl.Get<IList>(group, "bridges").Count;

        private static string Where(Part p) =>
            string.Format(CultureInfo.InvariantCulture, "{0} {1:+0.0;-0.0} m", p.partInfo.name.Split('.').Last(), p.transform.position.x - EditorLogic.RootPart.transform.position.x);

        private static string What(FARWingAerodynamicModel w) => w is FARControllableSurface ? "control surface" : "panel";

        private sealed class B2
        {
            public List<Part> Parts;
            public List<(Part Part, FARWingAerodynamicModel Wing)> Wings, Shielded, Lifting;
        }

        /// <summary>The craft in the SPH with the whole-wing aspect ratio on, FAR's sim settled; every wing recorded.</summary>
        private static IEnumerator Load(TestContext ctx, B2 into)
        {
            FARDebugValues.wholeWingAspectRatio = true;
            FARWingInteraction.InvalidateWingGroups();
            yield return Wait.Until(() => Far.EditorReady && EditorLogic.fetch.ship.parts.Count > 30 && Far.Instant(2, 0.3).Cl > 0.01,
                                    120, "FAR's editor sim on the B-2", shot: false);
            yield return Far.Settled("the B-2");
            into.Parts = EditorLogic.fetch.ship.parts;
            into.Wings = into.Parts.Select(p => (Part: p, Wing: p.GetComponent<FARWingAerodynamicModel>()))
                             .Where(t => t.Wing != null)
                             .OrderBy(t => t.Part.transform.position.x)
                             .ToList();
            foreach (var (p, w) in into.Wings)
                ctx.Record(Where(p) + " " + p.craftID, string.Format(CultureInfo.InvariantCulture, "{0}; shielded {1}; S {2:F2} m2; AR {3:F3}",
                                                                    What(w), w.ShieldedAtRest, w.S, w.EffectiveAR));
            var main = into.Wings.Where(t => !(t.Wing is FARControllableSurface)).ToList();
            into.Shielded = main.Where(t => t.Wing.ShieldedAtRest).ToList();
            into.Lifting = main.Where(t => !t.Wing.ShieldedAtRest).ToList();
        }

        private static void RecordWing(TestContext ctx, string name, object g) =>
            ctx.Record(name, string.Format(CultureInfo.InvariantCulture,
                                           "{0} panels, {1} shielded between, straddles {2}, root on body {3}, b {4:F2} m, tip to tip {5:F2} m, S {6:F2} + {7:F2} m2 of control surfaces, AR {8:F3}",
                                           Members(g).Count, Bridges(g), Member<bool>(g, "Straddles"), Member<bool>(g, "RootOnBody"), Member<double>(g, "B2"),
                                           Member<double>(g, "TipToTip"), Member<double>(g, "Area"), Member<double>(g, "ControlArea"), Member<double>(g, "AspectRatio")));

        /// <summary>The wing group FAR found for each panel: one each, and a panel's group holds it.</summary>
        private static object GroupOfEach(IEnumerable<(Part Part, FARWingAerodynamicModel Wing)> panels, string what)
        {
            object g = null;
            foreach (var (p, w) in panels)
            {
                object pg = GroupOf(w);
                Assert.NotNull(pg, $"the panel at {Where(p)} has a wing group");
                Assert.True(Members(pg).Contains(w), $"the panel at {Where(p)} is in its wing group");
                g = g ?? pg;
                Assert.True(ReferenceEquals(g, pg), $"the panel at {Where(p)} is in {what}");
            }
            return g;
        }

        /// <summary>Builds the group from each lifting panel in turn (WingGroup.Build), checks it, and has FAR find them again.</summary>
        private static IEnumerator BuiltFromEachPanel(B2 b2, Action<Part, object> check)
        {
            MethodInfo build = GroupType.GetMethod("Build", Refl.Members) ?? throw new UITestFailure("WingGroup has no Build");
            foreach (var (p, w) in b2.Lifting)
            {
                build.Invoke(null, new object[] { w, b2.Parts });
                check(p, GroupOf(w));
            }
            FARWingInteraction.InvalidateWingGroups();
            yield return Far.Settled("the B-2, its wing groups found again");
        }

        // the option as it was, and FAR's own groups again: none a test built (BuiltFromEachPanel) stays in use
        private static void Restore(bool whole)
        {
            FARDebugValues.wholeWingAspectRatio = whole;
            FARWingInteraction.InvalidateWingGroups();
        }

        /// <summary>
        /// The flying B-2's wing is one wing. With the whole-wing aspect ratio on: the panels under the nacelles are
        /// shielded; every other main panel, both sides of the centerline, is in one wing group, joined across the shielded
        /// panels (WingGroup.Bridged) and spanning the centerline (Straddles) through the lifting centre panels, with no
        /// body at its root. The group is the same whichever panel it is built from. Its span is the craft's, its aspect
        /// ratio its span squared over its area with the control surfaces, and every panel and control surface on either
        /// side flies with that aspect ratio: before, a wing reached across the shielded panels refused every panel whose
        /// mirror it held, so the wing built from one centre panel took the other side's panels but not its own side's
        /// beyond the nacelle, and the two halves of the craft got different aspect ratios (5.41 and 3.45).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = "B-2 Spirit FAR", CraftDir = "SPH", TimeoutSec = 600)]
        public static IEnumerator Editor_FlyingWingIsOneWing(TestContext ctx)
        {
            bool whole = FARDebugValues.wholeWingAspectRatio;
            try
            {
                var b2 = new B2();
                yield return Load(ctx, b2);

                ctx.Step("The panels under the nacelles are shielded, the rest one wing");
                Assert.Equal(2, b2.Shielded.Count, $"two shielded panels ({string.Join(", ", b2.Shielded.Select(t => Where(t.Part)))})");
                foreach (var (p, _) in b2.Shielded)
                    Assert.True(p.children.Any(c => c.partInfo.name == "proceduralTankLiquid"), $"the shielded panel at {Where(p)} carries a nacelle");
                Assert.Equal(18, b2.Lifting.Count, "eighteen lifting main panels");
                object g = GroupOfEach(b2.Lifting, "the one wing");
                RecordWing(ctx, "wing", g);
                Assert.Equal(18, Members(g).Count, "the wing's panels");
                Assert.Equal(2, Bridges(g), "the wing reaches across both shielded panels");
                Assert.True(Member<bool>(g, "Straddles"), "the wing spans the centerline");
                Assert.False(Member<bool>(g, "RootOnBody"), "no body holds its root");

                ctx.Step("The same wing whichever panel it is built from");
                yield return BuiltFromEachPanel(b2, (p, built) =>
                {
                    Assert.Equal(18, Members(built).Count, $"the wing built from the panel at {Where(p)}: its panels");
                    Assert.True(Member<bool>(built, "Straddles"), $"the wing built from the panel at {Where(p)} spans the centerline");
                });

                ctx.Step("Its span and aspect ratio, on both sides");
                g = GroupOfEach(b2.Lifting, "the one wing");
                double b = Member<double>(g, "B2"), area = Member<double>(g, "Area") + Member<double>(g, "ControlArea");
                double ar = Member<double>(g, "AspectRatio");
                double width = ShipConstruction.CalculateCraftSize(EditorLogic.fetch.ship).x;
                ctx.Record("craftWidth", width.ToString("F2", CultureInfo.InvariantCulture));
                Assert.Near(width, b, 0.01 * width, "the wing's span is the craft's width (m)");
                // with FAR's endplate factor for covered tips (none here, but the voxels decide)
                double covered = Math.Max(0, Math.Min(1, 1 - Member<double>(g, "ExposureMin"))) + Math.Max(0, Math.Min(1, 1 - Member<double>(g, "ExposureMax")));
                ctx.Record("tipsCovered", covered.ToString("F3", CultureInfo.InvariantCulture));
                Assert.Near(b * b / area * (1 + 0.25 * covered), ar, 1e-3 * ar, "its aspect ratio is its span squared over its area");
                Assert.True(ar > 4 && ar < 7, $"a B-2's aspect ratio ({ar:F2}; the real one's is about 5.7)");
                foreach (var (p, w) in b2.Wings.Where(t => !t.Wing.ShieldedAtRest))
                    Assert.Near(ar, w.EffectiveAR, 1e-3 * ar, $"the {What(w)} at {Where(p)} flies with the wing's aspect ratio");
                yield return Snap.Shot(ctx, "b2");
            }
            finally
            {
                Restore(whole);
            }
        }

        /// <summary>
        /// Wings on a fuselage, joined only through the shielded panels inside it, are two wings: "B-2 Spirit Fuselage",
        /// the B-2's panels with a ProceduralParts fuselage over the two centre panels, which shields them. Each side's
        /// lifting panels are one wing, which reaches the shielded centre panels but not across the centerline to the
        /// other side, whichever panel it is built from; neither spans the centerline, each has its root on the body there,
        /// and both fly with the same aspect ratio, measured through the body.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = "B-2 Spirit Fuselage", CraftDir = "SPH", TimeoutSec = 600)]
        public static IEnumerator Editor_WingsThroughAFuselageAreTwo(TestContext ctx)
        {
            bool whole = FARDebugValues.wholeWingAspectRatio;
            try
            {
                var b2 = new B2();
                yield return Load(ctx, b2);

                ctx.Step("The centre panels are shielded, each side one wing on the body");
                Assert.Equal(2, b2.Shielded.Count, $"two shielded panels ({string.Join(", ", b2.Shielded.Select(t => Where(t.Part)))})");
                foreach (var (p, _) in b2.Shielded)
                    Assert.True(p.parent == EditorLogic.RootPart, $"the shielded panel at {Where(p)} is a centre panel");
                double rootX = EditorLogic.RootPart.transform.position.x;
                var left = b2.Lifting.Where(t => t.Part.transform.position.x < rootX).ToList();
                var right = b2.Lifting.Where(t => t.Part.transform.position.x > rootX).ToList();
                Assert.Equal(9, left.Count, "nine lifting panels on the left");
                Assert.Equal(9, right.Count, "nine on the right");
                object gl = GroupOfEach(left, "the left wing"), gr = GroupOfEach(right, "the right wing");
                RecordWing(ctx, "leftWing", gl);
                RecordWing(ctx, "rightWing", gr);
                Assert.False(ReferenceEquals(gl, gr), "the two sides are two wings");
                foreach (var (g, side) in new[] { (gl, "left"), (gr, "right") })
                {
                    Assert.Equal(9, Members(g).Count, $"the {side} wing's panels");
                    Assert.True(Bridges(g) > 0, $"the {side} wing reaches the shielded centre panels");
                    Assert.False(Member<bool>(g, "Straddles"), $"the {side} wing does not span the centerline");
                    Assert.True(Member<bool>(g, "RootOnBody") && Member<bool>(g, "RootOnCenter"), $"the {side} wing's root is on the body on the centerline");
                }

                ctx.Step("The same wings whichever panel they are built from");
                var leftWings = left.Select(t => t.Wing).ToList();
                yield return BuiltFromEachPanel(b2, (p, built) =>
                {
                    List<FARWingAerodynamicModel> members = Members(built);
                    int l = members.Count(leftWings.Contains);
                    Assert.Equal(9, members.Count, $"the wing built from the panel at {Where(p)}: its panels");
                    Assert.True(l == 0 || l == 9, $"the wing built from the panel at {Where(p)} holds one side ({l} of 9 on the left)");
                    Assert.False(Member<bool>(built, "Straddles"), $"the wing built from the panel at {Where(p)} does not span the centerline");
                });

                ctx.Step("Both sides fly with one aspect ratio");
                double ar = Member<double>(GroupOfEach(left, "the left wing"), "AspectRatio");
                Assert.Near(ar, Member<double>(GroupOfEach(right, "the right wing"), "AspectRatio"), 1e-3 * ar, "both sides' aspect ratio");
                Assert.True(ar > 4 && ar < 7, $"a B-2's aspect ratio ({ar:F2})");
                foreach (var (p, w) in b2.Wings.Where(t => !t.Wing.ShieldedAtRest))
                    Assert.Near(ar, w.EffectiveAR, 1e-3 * ar, $"the {What(w)} at {Where(p)} flies with its wing's aspect ratio");
                yield return Snap.Shot(ctx, "b2fuselage");
            }
            finally
            {
                Restore(whole);
            }
        }

        /// <summary>
        /// Lifting panels under a fuselage carry the wing across it: "B-2 Spirit Belly", the fuselage B-2 with two exposed
        /// panels under the body, independent parts (not mirror images): the left one past the left centre panel and short
        /// of the left wing, the right one touching it and the right wing. The left wing reaches the left belly panel across
        /// the shielded centre panel, the right belly panel from it, and the right wing from that; the right wing reaches
        /// them directly. So every lifting panel is one wing spanning the centerline, as a single-part centre section's
        /// would be, whichever panel it is built from, and every panel and control surface flies with its aspect ratio.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = "B-2 Spirit Belly", CraftDir = "SPH", TimeoutSec = 600)]
        public static IEnumerator Editor_BellyPanelsCarryTheWingAcrossTheFuselage(TestContext ctx)
        {
            bool whole = FARDebugValues.wholeWingAspectRatio;
            try
            {
                var b2 = new B2();
                yield return Load(ctx, b2);
                Part fuselage = EditorLogic.RootPart.children.FirstOrDefault(c => c.partInfo.name == "proceduralTankLiquid");
                Assert.NotNull(fuselage, "the fuselage");

                ctx.Step("The centre panels are shielded, the belly panels lift");
                Assert.Equal(2, b2.Shielded.Count, $"two shielded panels ({string.Join(", ", b2.Shielded.Select(t => Where(t.Part)))})");
                foreach (var (p, _) in b2.Shielded)
                    Assert.True(p.parent == EditorLogic.RootPart, $"the shielded panel at {Where(p)} is a centre panel");
                var belly = b2.Lifting.Where(t => t.Part.parent == fuselage).ToList();
                Assert.Equal(2, belly.Count, "the two belly panels lift");
                Assert.True(belly.All(t => t.Part.symmetryCounterparts.Count == 0), "the belly panels are not mirror images");
                Assert.Equal(20, b2.Lifting.Count, "twenty lifting main panels: nine a side and the belly");

                ctx.Step("One wing across the fuselage");
                object g = GroupOfEach(b2.Lifting, "the one wing");
                RecordWing(ctx, "wing", g);
                Assert.Equal(20, Members(g).Count, "the wing's panels");
                Assert.True(Member<bool>(g, "Straddles"), "the wing spans the centerline");

                ctx.Step("The same wing whichever panel it is built from");
                yield return BuiltFromEachPanel(b2, (p, built) =>
                {
                    Assert.Equal(20, Members(built).Count, $"the wing built from the panel at {Where(p)}: its panels");
                    Assert.True(Member<bool>(built, "Straddles"), $"the wing built from the panel at {Where(p)} spans the centerline");
                });

                ctx.Step("Both sides fly with its aspect ratio");
                double ar = Member<double>(GroupOfEach(b2.Lifting, "the one wing"), "AspectRatio");
                Assert.True(ar > 4 && ar < 7, $"a B-2's aspect ratio ({ar:F2})");
                foreach (var (p, w) in b2.Wings.Where(t => !t.Wing.ShieldedAtRest))
                    Assert.Near(ar, w.EffectiveAR, 1e-3 * ar, $"the {What(w)} at {Where(p)} flies with the wing's aspect ratio");
                yield return Snap.Shot(ctx, "b2belly");
            }
            finally
            {
                Restore(whole);
            }
        }
    }
}
