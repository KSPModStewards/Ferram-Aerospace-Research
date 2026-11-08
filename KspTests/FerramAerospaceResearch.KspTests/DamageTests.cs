using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>FAR's model of a vessel losing parts in flight: what it drops and what it keeps working with.</summary>
    public static class DamageTests
    {
        private static readonly string[] Directions = { "Forward", "Backward", "Leftward", "Rightward" };

        private static List<FARWingAerodynamicModel> Neighbours(FARWingInteraction wi, string direction) =>
            Refl.Get<List<FARWingAerodynamicModel>>(wi, $"nearbyWingModules{direction}List");

        private static List<double> Influences(FARWingInteraction wi, string direction) =>
            Refl.Get<List<double>>(wi, $"nearbyWingModules{direction}Influence");

        /// <summary>Whether a part hangs off another (is it, or below it in the part tree).</summary>
        private static bool Below(Part p, Part ancestor)
        {
            for (; p != null; p = p.parent)
                if (p == ancestor)
                    return true;
            return false;
        }

        /// <summary>Where a wing's neighbour lists hold a destroyed wing; empty if none.</summary>
        private static string BadLists(IEnumerable<FARWingAerodynamicModel> wings, params string[] directions)
        {
            var bad = new List<string>();
            foreach (FARWingAerodynamicModel w in wings)
            {
                if (w == null || w.WingInteraction == null) continue;
                foreach (string d in directions.Length > 0 ? directions : Directions)
                {
                    List<FARWingAerodynamicModel> list = Neighbours(w.WingInteraction, d);
                    List<double> influence = Influences(w.WingInteraction, d);
                    if (list.Count == 0) continue;
                    if (list.Any(n => n == null))
                        bad.Add($"{w.part.partInfo.name} {d}: a destroyed neighbour");
                    if (list.Count != influence.Count)
                        bad.Add($"{w.part.partInfo.name} {d}: {list.Count} neighbours, {influence.Count} influences");
                }
            }

            return string.Join("; ", bad);
        }

        /// <summary>
        /// A wing panel that another wing takes its upstream flow from, and an air intake, destroyed together in
        /// cruise at Mach 0.5: FAR throws nothing; the downstream wing drops the destroyed one from its neighbours
        /// (no wing keeps a destroyed upstream neighbour); FAR's vessel lists no destroyed part's module; and the plane flies on
        /// with a real force on it.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_DestroyedPartsLeaveTheModel(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0.5, 3000, 4);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            FARVesselAero vesselAero = v.GetComponent<FARVesselAero>();
            List<FARWingAerodynamicModel> wings = v.parts.SelectMany(p => p.Modules.OfType<FARWingAerodynamicModel>()).ToList();
            Assert.True(BadLists(wings) == "", $"the wings' neighbour lists sound before ({BadLists(wings)})");
            string Sums(FARWingAerodynamicModel w) => $"{Influences(w.WingInteraction, "Forward").Count} upstream, influence {Influences(w.WingInteraction, "Forward").Sum():F4}";

            // a wing with an upstream neighbour, the neighbour as few parts on it as there are
            var candidates = wings.SelectMany(w => Neighbours(w.WingInteraction, "Forward").Select(u => (down: w, up: u)))
                                  .Where(x => x.up != null && x.up.part != x.down.part && x.up.part.parent != null && !Below(x.down.part, x.up.part))
                                  .OrderBy(x => x.up.part.children.Count).ToList();
            Assert.True(candidates.Count > 0, "a wing with an upstream wing to take away");
            (FARWingAerodynamicModel downstream, FARWingAerodynamicModel upstream) = candidates[0];
            Part intake = v.parts.FirstOrDefault(p => p.Modules.Contains<ModuleResourceIntake>() && p.children.Count == 0 && p.parent != null);
            ctx.Record("lost", $"{upstream.part.partInfo.name} (upstream of {downstream.part.partInfo.name}: {Sums(downstream)}), intake {intake?.partInfo.name ?? "none"}");
            double influenceBefore = Influences(downstream.WingInteraction, "Forward").Sum();
            double lostInfluence = Influences(downstream.WingInteraction, "Forward")[Neighbours(downstream.WingInteraction, "Forward").IndexOf(upstream)];

            using var exceptions = new ExceptionLog();
            int partsBefore = v.parts.Count;
            var lostParts = new List<Part> { upstream.part };
            if (intake != null && intake != upstream.part) lostParts.Add(intake);
            var lostAero = lostParts.Select(p => p.Modules.GetModule<FARAeroPartModule>()).ToList();
            ctx.Step("the parts destroyed");
            foreach (Part p in lostParts)
                p.Die();

            var samples = new List<FarFlightProbe.Sample>();
            yield return FarFlightProbe.Hold(c, 10, 20, samples, predict: false);
            ctx.Record("after", $"{v.parts.Count} parts (from {partsBefore}); {BadLists(wings)}");
            // the upstream (forward) lists are the ones a wing reads in level flight, and prunes as it reads them; the
            // others keep a destroyed wing until FAR next rebuilds the interactions (after the vessel is voxelized again)
            Assert.True(v.parts.Count < partsBefore, "the parts gone from the vessel");
            ctx.Record("downstream", $"{Sums(downstream)} (before {influenceBefore:F4}, the lost wing's {lostInfluence:F4})");
            Assert.False(Neighbours(downstream.WingInteraction, "Forward").Any(n => n == null || n == upstream),
                         "the downstream wing dropped its destroyed neighbour");
            Assert.True(BadLists(wings, "Forward") == "", $"no wing keeps a destroyed upstream neighbour ({BadLists(wings, "Forward")})");
            var aeroModules = Refl.Get<List<FARAeroPartModule>>(vesselAero, "_currentAeroModules");
            Assert.False(aeroModules.Any(m => m == null || lostAero.Contains(m)), "FAR's vessel dropped the destroyed parts' aero modules");
            Assert.True(samples.All(s => s.Flight.All(x => !double.IsNaN(x) && !double.IsInfinity(x))), "a real force on the plane after the loss");
            Assert.True(samples.Average(s => s.Flight[1]) > 0, "the plane still makes lift");
            exceptions.AssertNone("losing parts in flight");
        }

        /// <summary>
        /// The two stress limits a wing has, each set below its cruise load at Mach 0.5. A wing panel's own wing limit
        /// (FARWingAerodynamicModel's, from its area and the stress template): the panel tears off and the flight log
        /// records it. A panel with another part on it, its part limit (FARAeroPartModule's): it tears off and takes
        /// nothing with it, the part on it coming off on its own; the plane keeps the rest.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_OverstressTearsOffWingsAndWhatIsOnThem(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0.5, 3000, 4);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            using var exceptions = new ExceptionLog();

            ctx.Step("a panel past its wing limit");
            FARWingAerodynamicModel outboard = v.parts.Where(p => p.parent != null && p.children.Count == 0)
                                                .SelectMany(p => p.Modules.OfType<FARWingAerodynamicModel>())
                                                .OrderByDescending(w => w.S).FirstOrDefault();
            Assert.NotNull(outboard, "an outboard wing panel");
            Part panel = outboard.part;
            int logBefore = FlightLogger.eventLog.Count, partsBefore = v.parts.Count;
            outboard.YmaxForce = 0;
            outboard.XZmaxForce = 0;
            yield return Wait.Until(() => panel.vessel != v, 10, "the panel past its wing limit to come off", shot: false);
            List<string> logged = FlightLogger.eventLog.Skip(logBefore).ToList();
            ctx.Record("wingLimit", $"{panel.partInfo.name}: {v.parts.Count} parts (from {partsBefore}); log {string.Join(" | ", logged)}");
            Assert.Equal(partsBefore - 1, v.parts.Count, "only the panel came off");
            Assert.True(logged.Any(m => m.Contains(panel.partInfo.title)), "the flight log records the panel's failure");

            ctx.Step("a panel with a part on it past its part limit");
            Part carrier = v.parts.Where(p => p.parent != null && p.children.Count > 0 && p.Modules.Contains<FARWingAerodynamicModel>())
                            .OrderBy(p => p.children.Count).FirstOrDefault();
            Assert.NotNull(carrier, "a wing panel with a part on it");
            List<Part> carried = carrier.children.ToList();
            ctx.Record("carrier", $"{carrier.partInfo.name} carrying {string.Join(", ", carried.Select(p => p.partInfo.name))}");
            FARAeroPartModule aero = carrier.Modules.GetModule<FARAeroPartModule>();
            partsBefore = v.parts.Count;
            int subtree = v.parts.Count(p => Below(p, carrier));
            Refl.Set(aero, "partStressMaxXZ", 0.0);
            Refl.Set(aero, "partForceMaxXZ", 0.0);
            Refl.Set(aero, "partStressMaxY", 0.0);
            Refl.Set(aero, "partForceMaxY", 0.0);
            yield return Wait.Until(() => carrier.vessel != v, 10, "the overstressed panel to come off", shot: false);
            yield return Wait.Frames(3);
            ctx.Record("partLimit", $"{v.parts.Count} parts (from {partsBefore}); the panel on {carrier.vessel?.vesselName}, " +
                                    string.Join(", ", carried.Select(p => $"{p.partInfo.name} on {p.vessel?.vesselName}")));
            Assert.Equal(partsBefore - subtree, v.parts.Count, "the panel and what was on it came off, nothing else");
            foreach (Part p in carried)
                Assert.True(p.vessel != v && p.parent != carrier, $"{p.partInfo.name} came off the panel too, on its own");
            Assert.True(carrier.children.Count == 0, "the torn-off panel carries nothing");
            exceptions.AssertNone("tearing off wings");
        }
    }
}
