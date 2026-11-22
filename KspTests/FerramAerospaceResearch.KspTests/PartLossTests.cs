using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARPartGeometry;
using KSP.UI.Screens;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>Parts lost in flight: failed under aero stress, or destroyed, and what FAR keeps of them.</summary>
    public static class PartLossTests
    {
        /// <summary>
        /// A fairing base that fails under aero stress (the ComSat Lx's, on the pad) opens its fairing with a soft
        /// ejection, and everything on it comes off with it: its children decouple from it and it from the rocket. The
        /// staging list drops what came off (the fairing, the lower stage's engine), and no stage is left empty.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "ComSat Lx Unclamped", CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_OverstressedFairingOpensAndShedsWhatIsOnIt(TestContext ctx)
        {
            yield return Wait.Seconds(3);
            Vessel v = FlightGlobals.ActiveVessel;
            Part fairing = v.parts.FirstOrDefault(p => p.Modules.Contains<ModuleProceduralFairing>());
            Assert.NotNull(fairing, "the ComSat Lx's fairing base");
            var fm = fairing.Modules.GetModule<ModuleProceduralFairing>();
            List<Part> children = fairing.children.ToList();
            Assert.True(children.Count > 0 && fairing.parent != null, $"the fairing base between the rocket and {children.Count} part(s) on it");
            FARAeroPartModule aero = fairing.Modules.GetModule<FARAeroPartModule>();
            Assert.NotNull(aero, "FAR's aero module on the fairing base");
            MethodInfo fail = typeof(FARAeroPartModule).GetMethod("ApplyAeroStressFailure", BindingFlags.Instance | BindingFlags.NonPublic);
            using var exceptions = new ExceptionLog();
            ctx.Record("before", $"ejection force {fm.ejectionForce}, children {string.Join(", ", children.Select(c => c.partInfo.name))}");
            HashSet<Part> lost = Staging.Subtree(fairing);
            List<string> lostStaged = Staging.Parts().Where(lost.Contains).Select(p => p.partInfo.name).ToList();
            ctx.Record("lost staged", $"{string.Join(", ", lostStaged)}; {StageManager.StageCount} stages");
            Assert.True(lostStaged.Count > 1, "the fairing and a part below it are in the staging list");

            ctx.Step("The fairing base failed under aero stress");
            fail.Invoke(aero, null);
            Assert.Near(0.5, fm.ejectionForce, 1e-6, "the fairing ejected softly");
            yield return Wait.Frames(5);
            ctx.Record("after", $"the base on {fairing.vessel?.vesselName}");
            Assert.True(fairing.vessel != v, "the fairing base came off the rocket");
            foreach (Part c in children)
                Assert.True(c.vessel != fairing.vessel && c.parent != fairing, $"{c.partInfo.name} came off the fairing base");
            List<string> stray = Staging.Stray();
            ctx.Record("staging after", $"stray {string.Join(", ", stray)}; {StageManager.StageCount} stages, {Staging.EmptyStages()} empty");
            Assert.Equal(0, stray.Count, "the staging list keeps only the rocket's parts");
            Assert.Equal(0, Staging.EmptyStages(), "no stage left empty");
            exceptions.AssertNone("failing a fairing base under aero stress");
        }

        /// <summary>
        /// The wing panel carrying an engine (one of the Stearwing A300 PW's Goliaths, on a B9 procedural wing; the stock
        /// A300's delta wing is shielded) past its own wing limit (FARWingAerodynamicModel's) at Mach 0.5: it tears off
        /// with the engine, and the staging list drops the engine and keeps the other one. The two are a symmetry pair
        /// with one icon between them; the engine torn off is the one that holds it.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Stearwing A300 PW", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_WingTornOffTakesItsEngineOutOfStaging(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            yield return FlightTests.Airborne(Far.At(0.5, 3000, 4));
            Vessel v = FlightGlobals.ActiveVessel;
            using var exceptions = new ExceptionLog();
            Part engine = Staging.Parts().FirstOrDefault(p => p.Modules.Contains<ModuleEngines>());
            Assert.NotNull(engine, "a staged engine");
            var wings = new List<FARWingAerodynamicModel>();
            for (Part p = engine.parent; p != null && p.parent != null; p = p.parent)
                wings.AddRange(p.Modules.OfType<FARWingAerodynamicModel>());
            ctx.Record("wings", string.Join(", ", wings.Select(w => $"{w.part.partInfo.name}{(w.isShielded ? " (shielded)" : "")}")));
            wings = wings.Where(w => !w.isShielded).ToList();
            Assert.True(wings.Count > 0, $"an unshielded wing panel between {engine.partInfo.name} and the fuselage");
            Part other = v.parts.FirstOrDefault(p => p != engine && p.Modules.Contains<ModuleEngines>());
            Assert.NotNull(other, "a second engine");
            ctx.Record("before", $"other {other.partInfo.name} icon {other.stackIcon?.StageIcon != null}, listed {Staging.Parts().Contains(other)}; " +
                                 $"{StageManager.StageCount} stages, {Staging.Parts().Count} listed icons");

            ctx.Step("the panels past their wing limit");
            foreach (FARWingAerodynamicModel w in wings)
            {
                w.YmaxForce = 0;
                w.XZmaxForce = 0;
            }
            yield return Wait.Until(() => engine.vessel != v, 10, "a panel past its wing limit to come off with the engine", shot: false);
            yield return Wait.Frames(3);
            List<string> stray = Staging.Stray();
            ctx.Record("staging after", $"stray {string.Join(", ", stray)}; other on the plane {other.vessel == v}, icon {other.stackIcon?.StageIcon != null}, " +
                                        $"listed {Staging.Parts().Contains(other)}; {StageManager.StageCount} stages, " +
                                        $"{Staging.Parts().Count} listed icons, {Staging.EmptyStages()} empty");
            Assert.True(other.vessel == v, "the other engine stays on the plane");
            Assert.Equal(0, stray.Count, "the staging list keeps only the plane's parts");
            Assert.True(Staging.Parts().Contains(other), "the other engine stays in the staging list");
            Assert.Equal(0, Staging.EmptyStages(), "no stage left empty");
            exceptions.AssertNone("tearing off a wing with an engine on it");
        }

        private static readonly string[] Directions = { "Forward", "Backward", "Leftward", "Rightward" };

        private static List<FARWingAerodynamicModel> Neighbours(FARWingInteraction wi, string dir) =>
            Refl.Get<List<FARWingAerodynamicModel>>(wi, $"nearbyWingModules{dir}List");

        private static List<double> Influences(FARWingInteraction wi, string dir) =>
            Refl.Get<List<double>>(wi, $"nearbyWingModules{dir}Influence");

        /// <summary>
        /// A wing panel destroyed in flight (the plane's, on the runway): FAR's vessel drops its geometry module, and
        /// the panels that had it as a neighbour drop it with its influence. The neighbour's influences before and after
        /// are recorded: FAR scales what is left to sum to 1 when it prunes a lost neighbour (HandleNullPart), and as
        /// built they need not sum to 1.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 600)]
        public static IEnumerator Runway_DestroyedWingPanelLeavesItsNeighbours(TestContext ctx)
        {
            yield return Wait.Seconds(3);
            Vessel v = FlightGlobals.ActiveVessel;
            List<FARWingAerodynamicModel> wings = v.parts.Select(p => p.Modules.GetModule<FARWingAerodynamicModel>()).Where(w => w != null).ToList();
            FARWingAerodynamicModel lost = null, neighbour = null;
            string dir = null;
            foreach (FARWingAerodynamicModel w in wings)
                foreach (string d in Directions)
                {
                    if (w.WingInteraction == null || lost != null)
                        continue;
                    FARWingAerodynamicModel n = Neighbours(w.WingInteraction, d)?.FirstOrDefault(x => x != null && x != w && x.part.children.Count == 0 &&
                                                                                                    x.part != v.rootPart);
                    if (n == null)
                        continue;
                    lost = n;
                    neighbour = w;
                    dir = d;
                }

            Assert.NotNull(lost, "a leaf wing panel that another panel has as a neighbour");
            FARWingInteraction wi = neighbour.WingInteraction;
            List<double> before = Influences(wi, dir).ToList();
            ctx.Record("before", $"{neighbour.part.partInfo.name}'s {dir.ToLowerInvariant()} neighbours: " +
                                 $"{string.Join(", ", Neighbours(wi, dir).Select(x => x.part.partInfo.name))}; influences {string.Join(", ", before.Select(x => x.ToString("F4")))} " +
                                 $"(sum {before.Sum():F4}); {lost.part.partInfo.name} goes");
            FARVesselAero aero = v.GetComponent<FARVesselAero>();
            GeometryPartModule lostGeo = lost.part.Modules.GetModule<GeometryPartModule>();
            using var exceptions = new ExceptionLog();

            // FAR prunes a lost neighbour when it computes the wing's interactions, and it does not below 0.1 m/s
            // airspeed: the parked plane gets a light breeze, or it prunes only while it still settles
            Vector3 breeze = -2 * v.transform.up;
            FARAtmosphere.SetWindFunction((b, p, pos) => breeze);
            try
            {
                ctx.Step($"{lost.part.partInfo.name} destroyed");
                lost.part.explode();
                // 2 s was enough in plain runs, not under coverage
                yield return Wait.Until(() => !Refl.Get<List<GeometryPartModule>>(aero, "_currentGeoModules").Any(g => g == null || ReferenceEquals(g, lostGeo)),
                                        30, "FAR's vessel to drop the destroyed part's geometry module", shot: false);
                yield return Wait.Until(() => !Neighbours(wi, dir).Any(x => x == null || ReferenceEquals(x, lost)), 30,
                                        $"{neighbour.part.partInfo.name} to drop the destroyed panel as a neighbour", shot: false);
            }
            finally
            {
                FARAtmosphere.SetWindFunction(null);
            }

            List<FARWingAerodynamicModel> left = Neighbours(wi, dir);
            List<double> after = Influences(wi, dir);
            ctx.Record("after", $"{string.Join(", ", left.Select(x => x == null ? "(destroyed)" : x.part.partInfo.name))}; influences " +
                                $"{string.Join(", ", after.Select(x => x.ToString("F4")))} (sum {after.Sum():F4})");
            Assert.False(left.Any(x => x == null || ReferenceEquals(x, lost)), $"{neighbour.part.partInfo.name} lost the destroyed panel as a neighbour");
            Assert.Equal(left.Count, after.Count, "an influence per neighbour");
            exceptions.AssertNone("a wing panel destroyed in flight");
        }
    }
}
