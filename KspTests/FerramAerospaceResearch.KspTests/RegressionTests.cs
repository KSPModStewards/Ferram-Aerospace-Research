using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using KSP.Localization;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>Bugs FAR had and no longer has, each held to the fixed behaviour.</summary>
    public static class RegressionTests
    {
        /// <summary>
        /// FAR's current body in flight is the active vessel's, whatever body the editor's analysis windows last set
        /// (they set the body they analyse; flight kept it, and kept Kerbin past an SOI change). Set to Duna on the
        /// runway, it is Kerbin again within a few physics frames, and the adiabatic index FAR's shock math takes is
        /// Kerbin's.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 600)]
        public static IEnumerator Regression_FlightBodyIsTheActiveVessels(TestContext ctx)
        {
            yield return Wait.Seconds(2);
            Vessel v = FlightGlobals.ActiveVessel;
            CelestialBody home = v.mainBody;
            CelestialBody other = FlightGlobals.Bodies.FirstOrDefault(b => b.bodyName == "Duna") ??
                                  FlightGlobals.Bodies.FirstOrDefault(b => b != home && b.atmosphere);
            Assert.NotNull(other, "an atmospheric body other than the vessel's");
            double ut = Planetarium.fetch.time;
            var at = new Vector3d(v.latitude, v.longitude, v.altitude);
            double homeGamma = FARAtmosphere.GetAdiabaticIndex(home, at, ut);
            double otherGamma = FARAtmosphere.GetAdiabaticIndex(other, at, ut);
            ctx.Record("bodies", $"{home.bodyName} gamma {homeGamma:F4}, {other.bodyName} gamma {otherGamma:F4}");
            Assert.True(System.Math.Abs(homeGamma - otherGamma) > 1e-3, "the two bodies' adiabatic indices differ");
            try
            {
                ctx.Step($"FAR's body set to {other.bodyName}, as the editor's analysis would");
                FARAeroUtil.UpdateCurrentActiveBody(other);
                Assert.True(FARAeroUtil.CurrentBody == other, $"FAR's body set to {other.bodyName}");
                var fixedWait = new WaitForFixedUpdate();
                for (int i = 0; i < 5; i++)
                    yield return fixedWait;
                ctx.Record("after", $"{FARAeroUtil.CurrentBody?.bodyName}, gamma {FARAeroUtil.CurrentAdiabaticIndex:F4}");
                Assert.True(FARAeroUtil.CurrentBody == home,
                            $"FAR's body is the active vessel's ({home.bodyName}), not {FARAeroUtil.CurrentBody?.bodyName}");
                Assert.Near(homeGamma, FARAeroUtil.CurrentAdiabaticIndex, 1e-6, $"FAR's adiabatic index is {home.bodyName}'s");
            }
            finally
            {
                // tests share the game: leave FAR on the vessel's body
                FARAeroUtil.UpdateCurrentActiveBody(home);
            }
        }

        /// <summary>
        /// A part that fails under aero stress shows the failure on the flight status of the vessel it failed on, not
        /// the debris it became (FAR sent it to the part's vessel after decoupling it). A leaf part of the plane failed
        /// on the runway: it comes off, the plane's status reads the aerodynamic failure, and the flight log has it.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 600)]
        public static IEnumerator Regression_AeroFailureStatusOnTheVesselItFailedOn(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            Vessel v = FlightGlobals.ActiveVessel;
            FlightGUI gui = Far.FlightGui(v);
            FlightStatusGUI status = Refl.Get<FlightStatusGUI>(gui, "_flightStatusGUI");
            Assert.NotNull(status, "the plane's flight status");
            Part leaf = v.parts.Where(p => p.parent != null && p.children.Count == 0 && p.Modules.Contains<FARAeroPartModule>() &&
                                           !p.Modules.Contains("ModuleWheelBase"))
                         .OrderBy(p => p.mass).FirstOrDefault();
            Assert.NotNull(leaf, "a leaf part that is no wheel");
            FARAeroPartModule aero = leaf.Modules.GetModule<FARAeroPartModule>();
            MethodInfo fail = typeof(FARAeroPartModule).GetMethod("ApplyAeroStressFailure", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fail, "FARAeroPartModule.ApplyAeroStressFailure");
            string failure = Localizer.Format("FARFlightStatus5");
            using var exceptions = new ExceptionLog();
            int logBefore = FlightLogger.eventLog.Count;
            Refl.Set(status, "statusOverrideTimer", 0.0);
            Refl.Set(status, "statusString", "");

            ctx.Step($"{leaf.partInfo.name} failed under aero stress");
            fail.Invoke(aero, null);
            string shown = Refl.Get<string>(status, "statusString");
            double timer = Refl.Get<double>(status, "statusOverrideTimer");
            ctx.Record("status", $"'{shown}', override {timer:F2} s; {leaf.partInfo.name} on {leaf.vessel?.vesselName}");
            Assert.True(leaf.vessel != v, $"{leaf.partInfo.name} came off the plane");
            Assert.True(FlightGlobals.ActiveVessel == v, "the plane is still the active vessel");
            Assert.Equal(failure, shown, "the plane's flight status reads the aerodynamic failure");
            Assert.True(timer > 0, $"the failure holds the plane's status ({timer:F2} s)");
            Assert.True(FlightLogger.eventLog.Skip(logBefore).Any(m => m.Contains(leaf.partInfo.title)), "the flight log records the failure");
            yield return Wait.Frames(3);
            exceptions.AssertNone("failing a part under aero stress");
        }

        /// <summary>
        /// A part with several parts on it that fails under aero stress sheds all of them: each decouples from it.
        /// (The loop over its children counted up while each decouple took a child out of the list, so every second
        /// one stayed on.) On the plane on the runway, the part with the most children.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 600)]
        public static IEnumerator Regression_OverstressedPartShedsEveryChild(TestContext ctx)
        {
            yield return Wait.Seconds(3);
            Vessel v = FlightGlobals.ActiveVessel;
            Part hub = v.parts.Where(p => p.Modules.Contains<FARAeroPartModule>()).OrderByDescending(p => p.children.Count).First();
            List<Part> children = hub.children.ToList();
            ctx.Record("hub", $"{hub.partInfo.name}: {string.Join(", ", children.Select(c => c.partInfo.name))}");
            Assert.True(children.Count >= 2, $"a part with several on it ({hub.partInfo.name}, {children.Count})");
            MethodInfo fail = typeof(FARAeroPartModule).GetMethod("ApplyAeroStressFailure", BindingFlags.Instance | BindingFlags.NonPublic);
            using var exceptions = new ExceptionLog();

            ctx.Step($"{hub.partInfo.name} failed under aero stress");
            fail.Invoke(hub.Modules.GetModule<FARAeroPartModule>(), null);
            yield return Wait.Frames(5);
            List<Part> kept = children.Where(c => c.parent == hub).ToList();
            ctx.Record("kept", string.Join(", ", kept.Select(c => c.partInfo.name)));
            Assert.Equal(0, kept.Count, $"every child came off {hub.partInfo.name} (still on it: {string.Join(", ", kept.Select(c => c.partInfo.name))})");
            exceptions.AssertNone("failing a part with several children under aero stress");
        }

        /// <summary>
        /// FAR's install check runs: the loader keeps the InstallChecker (it made it and destroyed it in the same frame,
        /// before Unity called its Start, so a bad install went unreported). At the main menu it is alive and enabled on
        /// an active object, frames after the load, so Unity has run its Start.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 300)]
        public static IEnumerator Regression_InstallCheckerRuns(TestContext ctx)
        {
            yield return Wait.Frames(3);
            FARAddonLoader loader = Object.FindObjectOfType<FARAddonLoader>();
            Assert.NotNull(loader, "FAR's addon loader");
            InstallChecker kept = loader.AddonObjects.OfType<InstallChecker>().FirstOrDefault();
            InstallChecker found = Object.FindObjectOfType<InstallChecker>();
            ctx.Record("checker", $"kept by the loader {kept != null}, in the scene {found != null}, enabled {found != null && found.isActiveAndEnabled}");
            Assert.True(kept != null, "the loader keeps the InstallChecker");
            Assert.True(found != null && found.isActiveAndEnabled, "the InstallChecker is alive and enabled (Unity ran its Start)");
        }
    }
}
