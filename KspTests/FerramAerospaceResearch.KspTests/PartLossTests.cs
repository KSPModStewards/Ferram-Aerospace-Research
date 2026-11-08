using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARPartGeometry;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>Parts lost in flight: failed under aero stress, or destroyed, and what FAR keeps of them.</summary>
    public static class PartLossTests
    {
        /// <summary>
        /// A fairing base that fails under aero stress (the ComSat Lx's, on the pad) opens its fairing with a soft
        /// ejection, and everything on it comes off with it: its children decouple from it and it from the rocket.
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

            ctx.Step("The fairing base failed under aero stress");
            fail.Invoke(aero, null);
            Assert.Near(0.5, fm.ejectionForce, 1e-6, "the fairing ejected softly");
            yield return Wait.Frames(5);
            ctx.Record("after", $"the base on {fairing.vessel?.vesselName}");
            Assert.True(fairing.vessel != v, "the fairing base came off the rocket");
            foreach (Part c in children)
                Assert.True(c.vessel != fairing.vessel && c.parent != fairing, $"{c.partInfo.name} came off the fairing base");
            exceptions.AssertNone("failing a fairing base under aero stress");
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
