using System;
using System.Collections;
using System.IO;
using System.Linq;
using FerramAerospaceResearch.RealChuteLite;
using KSP.Localization;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's parachute (RealChuteFAR) beyond one chute coming down: chutes in symmetry sharing their settings from the
    /// info window, a staged chute that finds it cannot open, and an open chute through a save and a reload.
    /// </summary>
    public static class ChuteStateTests
    {
        private static string Window => Localizer.Format("RCLGUITitle");

        private const string Vtol = "Rocket-power VTOL";     // stock SPH: two Mk2-R radial chutes in mirror symmetry

        private static bool Visible(RealChuteFAR c) => (bool)Refl.Get(c, "visible");

        private static Rect WindowRect(RealChuteFAR c) => (Rect)Refl.Get(c, "window");

        /// <summary>Kerbal 1's capsule off its rocket, as its own vessel and the active one; its chute in into[0].</summary>
        private static IEnumerator CapsuleOff(TestContext ctx, RealChuteFAR[] into)
        {
            yield return Wait.Seconds(5);
            Flight.IgnoreMaxTemperature(true);
            Ctrl.SetThrottle(0f);
            Vessel v = FlightGlobals.ActiveVessel;
            v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
            ctx.Step("the capsule off its rocket");
            int partsBefore = v.parts.Count;
            ModuleDecouple decoupler = v.parts.SelectMany(p => p.Modules.OfType<ModuleDecouple>()).FirstOrDefault();
            Assert.NotNull(decoupler, "Kerbal 1's decoupler");
            decoupler.Decouple();
            yield return Wait.Seconds(2);
            v = FlightGlobals.ActiveVessel;
            into[0] = v.FindPartModulesImplementing<RealChuteFAR>().FirstOrDefault();
            Assert.NotNull(into[0], "the capsule's chute is FAR's RealChute");
            Assert.True(v.parts.Count < partsBefore, $"the capsule is its own vessel ({v.parts.Count} parts of {partsBefore})");
        }

        /// <summary>
        /// The stock Rocket-power VTOL's two radial chutes, in mirror symmetry, on the runway. Opening one chute's info
        /// window while the other's is open closes the other and puts the new window where the old one was. Its settings
        /// changed (deployment altitude, minimum pressure), the window's copy button gives them to the other chute; its
        /// close button closes it.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Vtol, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 600)]
        public static IEnumerator Behaviour_SymmetricChutesShareTheirSettings(TestContext ctx)
        {
            yield return Wait.Seconds(3);
            Vessel v = FlightGlobals.ActiveVessel;
            var chutes = v.FindPartModulesImplementing<RealChuteFAR>();
            Assert.Equal(2, chutes.Count, "the VTOL's chutes");
            RealChuteFAR a = chutes[0];
            Assert.Equal(1, a.part.symmetryCounterparts.Count, "the chutes are in symmetry");
            RealChuteFAR b = a.part.symmetryCounterparts[0].FindModuleImplementing<RealChuteFAR>();
            Assert.True(b != null && b != a, "the other chute is the counterpart");
            float altA = a.deployAltitude, altB = b.deployAltitude, presA = a.minAirPressureToOpen, presB = b.minAirPressureToOpen;
            try
            {
                ctx.Step("one window, then the other's");
                a.GUIToggleWindow();
                Assert.True(Visible(a), "the first chute's window open");
                yield return Imgui.WaitWindow(Window);
                Refl.Set(a, "window", new Rect(320, 140, 350, 400));
                b.GUIToggleWindow();
                Assert.True(Visible(b) && !Visible(a), $"the second chute's window open and the first's closed ({Visible(a)}, {Visible(b)})");
                Rect rb = WindowRect(b);
                Assert.True(Math.Abs(rb.x - 320) < 1 && Math.Abs(rb.y - 140) < 1, $"the second window where the first was ({rb.x}, {rb.y})");
                yield return Imgui.WaitWindow(Window);

                ctx.Step("the second chute's settings copied to the first");
                b.deployAltitude = 2345;
                b.minAirPressureToOpen = 0.125f;
                yield return Imgui.WaitVisible(Localizer.Format("RCLGUI14", 2345f), Window);
                Assert.True(Math.Abs(a.deployAltitude - 2345) > 1, "the first chute's altitude not yet the second's");
                yield return Imgui.Click(Localizer.Format("RCLGUICopy"), Window);
                yield return Wait.Frames(2);
                Assert.Near(2345, a.deployAltitude, 0.5, "the first chute's deployment altitude after the copy (m)");
                Assert.Near(0.125, a.minAirPressureToOpen, 1e-4, "the first chute's minimum pressure after the copy (atm)");

                ctx.Step("closed");
                yield return Imgui.Click(Localizer.Format("RCLGUIClose"), Window);
                yield return Imgui.WaitWindowClosed(Window);
                Assert.False(Visible(a) || Visible(b), "both windows closed");
            }
            finally
            {
                a.deployAltitude = altA;
                b.deployAltitude = altB;
                a.minAirPressureToOpen = presA;
                b.minAirPressureToOpen = presB;
                Refl.Set(a, "visible", false);
                Refl.Set(b, "visible", false);
            }
        }

        /// <summary>
        /// A chute staged and no longer armed (as one is once it has found air to open in, while it waits out its random
        /// delay) that can then not open fails: unstaged, its staging icon blinking red with a screen message giving the
        /// reason, stopped on the ground for the Kerbal 1 on the pad, in space for its capsule at 75 km; the blinking
        /// stops after 2.5 s.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 900)]
        public static IEnumerator Behaviour_StagedChuteThatCannotOpenFails(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            Vessel v = FlightGlobals.ActiveVessel;
            RealChuteFAR chute = v.FindPartModulesImplementing<RealChuteFAR>().FirstOrDefault();
            Assert.NotNull(chute, "the Kerbal 1's chute");
            Assert.True(chute.GroundStop, "the Kerbal 1 standing on the pad");
            yield return FailsWith(ctx, chute, "on the pad", "RCLFailGround");

            var holder = new RealChuteFAR[1];
            yield return CapsuleOff(ctx, holder);
            chute = holder[0];
            yield return FarFlightProbe.Place(Far.At(0, 75000, 0, "vacuum", speedMs: 200));
            yield return Wait.GameSeconds(1);
            yield return FailsWith(ctx, chute, "in space", "RCLFailPres");
        }

        private static IEnumerator FailsWith(TestContext ctx, RealChuteFAR chute, string where, string reasonTag)
        {
            ctx.Step($"staged {where}");
            Assert.False(chute.CanDeploy, $"the chute cannot open {where}");
            string failed = Localizer.Format("RCLFailDeploy"), reason = Localizer.Format(reasonTag);
            chute.staged = true;
            chute.armed = false;
            yield return Wait.Until(() => !chute.staged, 10, $"the chute to fail {where}", shot: false);
            yield return Wait.Until(() => (bool)Refl.Get(chute, "displayed"), 10, "the failure shown", shot: false);
            bool Posted(string text) => ScreenMessages.Instance != null && ScreenMessages.Instance.ActiveMessages.Any(m => m.message == text);
            Assert.True(Posted(failed), $"the screen says the deployment failed ({failed})");
            Assert.True(Posted(reason), $"and why: {reason}");
            Assert.False(chute.armed || chute.staged, "the failed chute unstaged and unarmed");
            Assert.Equal(RealChuteFAR.DeploymentStates.STOWED, chute.DeploymentState, "still stowed");
            var timer = (PhysicsWatch)Refl.Get(chute, "failedTimer");
            yield return Wait.Until(() => !timer.IsRunning && !(bool)Refl.Get(chute, "displayed"), 30, "the failure's blinking to stop", shot: false);
            Assert.True(chute.Events["GUIDeploy"].active, $"the deploy event offered again after failing {where}");
        }

        private const string SaveName = "far-test-chute-reload";

        /// <summary>
        /// The capsule under its chute saved and loaded again, as a quicksave and a quickload do: saved predeployed (its
        /// deployment altitude below it) the chute comes back predeployed, staged, its cap off and its canopy out,
        /// the deployed drag cube in place of the packed one, its canopy's age kept; deployed further down and saved
        /// again it comes back deployed, at its full area from its first frame.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 1200)]
        public static IEnumerator Behaviour_OpenChuteSurvivesAReload(TestContext ctx) => ReloadOpenChute(ctx, false);

        /// <summary>
        /// As Behaviour_OpenChuteSurvivesAReload, and the capsule sinks as fast under the reloaded canopy as before, FAR's
        /// voxel model of it the same: the canopy's drag is RealChuteFAR's own, not the voxel model's. Regression: before the fix, deployed
        /// in flight the canopy stays out of the voxel model (FAR does not voxelize the vessel again as it opens), but
        /// loaded deployed it is voxelized with the rest, and its FAR drag comes on top of the chute's: the capsule sinks
        /// at about half the rate (4.6 against 8.8 m/s at 3.5 km; FAR's largest
        /// cross-section 53 m2 against 5 m2 before the save). FAR's meshes of the chute part hold the canopy either
        /// way: its transform exceptions name the stock ModuleParachute's canopy (FARPartModuleTransformExceptions.cfg),
        /// not that of RealChuteFAR, which FAR puts in its place.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 1200)]
        public static IEnumerator Behaviour_ReloadedOpenChuteIsNotVoxelized(TestContext ctx) => ReloadOpenChute(ctx, true);

        /// <summary>FAR's meshes of the chute's canopy that are out (active).</summary>
        private static int CanopyMeshes(RealChuteFAR chute)
        {
            var canopy = (Transform)Refl.Get(chute, "parachute");
            var geo = chute.part.FindModuleImplementing<FerramAerospaceResearch.FARPartGeometry.GeometryPartModule>();
            return geo?.meshDataList?.Count(m => m.meshTransform != null && m.meshTransform.IsChildOf(canopy) && m.gameObjectActiveInHierarchy) ?? -1;
        }

        private static IEnumerator ReloadOpenChute(TestContext ctx, bool sinkRate)
        {
            var holder = new RealChuteFAR[1];
            yield return CapsuleOff(ctx, holder);
            RealChuteFAR chute = holder[0];
            try
            {
                ctx.Step("predeployed");
                yield return FarFlightProbe.Place(Far.At(0, 4000, 0, "drop", speedMs: 40));
                chute.deployAltitude = 50;
                chute.GUIDeploy();
                yield return Wait.Until(() => chute.DeploymentState == RealChuteFAR.DeploymentStates.PREDEPLOYED, 30, "the chute predeployed", shot: false);
                yield return Wait.GameSeconds(1 / chute.semiDeploymentSpeed + 1);
                float age = chute.time;
                yield return SaveAndReload(ctx, "predeployed", holder);
                chute = holder[0];
                AssertOpen(chute, RealChuteFAR.DeploymentStates.PREDEPLOYED, "predeployed");
                Assert.True(chute.time >= age - 0.01, $"the canopy's age kept ({chute.time:F2} s, {age:F2} saved)");
                yield return Wait.Until(() => Math.Abs(chute.currentArea - chute.PreDeployedArea) < 0.01 * chute.PreDeployedArea, 10,
                                        "the predeployed canopy at its area", shot: false, detail: () => $"{chute.currentArea:F3} m2 of {chute.PreDeployedArea:F3}");

                ctx.Step("deployed");
                chute.deployAltitude = (float)(FlightGlobals.ActiveVessel.altitude + 500);
                yield return Wait.Until(() => chute.DeploymentState == RealChuteFAR.DeploymentStates.DEPLOYED, 30, "the chute deployed", shot: false);
                yield return Wait.GameSeconds(1 / chute.deploymentSpeed + 1);
                var sinks = new double[1];
                double sink = 0, areaBefore = 0;
                if (sinkRate)
                {
                    yield return SteadySink(sinks);
                    sink = sinks[0];
                    // under coverage FAR was still rebuilding the model here (a cross-section of 0)
                    var before = FlightGlobals.ActiveVessel.GetComponent<FerramAerospaceResearch.FARAeroComponents.FARVesselAero>();
                    yield return Wait.Until(() => before.isValid && before.MaxCrossSectionArea > 0, 120, "FAR's model of the capsule under its canopy", shot: false);
                    areaBefore = before.MaxCrossSectionArea;
                    ctx.Record("sinkBefore", $"{sink:F2} m/s at {FlightGlobals.ActiveVessel.altitude:F0} m, {CanopyMeshes(chute)} canopy meshes in FAR's geometry, " +
                                             $"max cross-section {areaBefore:F2} m2");
                }

                yield return SaveAndReload(ctx, "deployed", holder);
                chute = holder[0];
                AssertOpen(chute, RealChuteFAR.DeploymentStates.DEPLOYED, "deployed");
                RealChuteFAR c = chute;
                var fixedWait = new WaitForFixedUpdate();
                yield return fixedWait;
                yield return fixedWait;
                Assert.Near(c.DeployedArea, c.currentArea, 0.01 * c.DeployedArea, "the reloaded canopy's area at once (m2)");
                if (sinkRate)
                {
                    var reloaded = FlightGlobals.ActiveVessel.GetComponent<FerramAerospaceResearch.FARAeroComponents.FARVesselAero>();
                    yield return Wait.Until(() => reloaded.isValid && reloaded.MaxCrossSectionArea > 0, 120, "FAR's model of the reloaded capsule", shot: false);
                    yield return SteadySink(sinks);
                    double sinkAfter = sinks[0];
                    double areaAfter = FlightGlobals.ActiveVessel.GetComponent<FerramAerospaceResearch.FARAeroComponents.FARVesselAero>().MaxCrossSectionArea;
                    ctx.Record("sinkAfter", $"{sinkAfter:F2} m/s at {FlightGlobals.ActiveVessel.altitude:F0} m, {CanopyMeshes(c)} canopy meshes in FAR's geometry, " +
                                            $"max cross-section {areaAfter:F2} m2");
                    Assert.Near(areaBefore, areaAfter, 0.1 * areaBefore, "FAR's largest cross-section of the capsule under its reloaded canopy, as before (m2)");
                    Assert.Near(sink, sinkAfter, 0.2 * sink, "the capsule's sink rate under the canopy after the reload (m/s)");
                }
            }
            finally
            {
                foreach (string ext in new[] { ".sfs", ".loadmeta" })
                    try
                    {
                        File.Delete(Path.Combine(KSPUtil.ApplicationRootPath, "saves", HighLogic.SaveFolder, SaveName + ext));
                    }
                    catch (Exception e)
                    {
                        Log.Warn("ChuteStateTests: deleting the test's save: " + e.Message);
                    }
            }
        }

        /// <summary>The capsule's sink rate once steady: the same to 0.1 m/s over a game second, three times running.</summary>
        private static IEnumerator SteadySink(double[] into)
        {
            double last = double.NaN;
            int steady = 0;
            for (int i = 0; i < 60 && steady < 3; i++)
            {
                yield return Wait.GameSeconds(1);
                double s = -FlightGlobals.ActiveVessel.verticalSpeed;
                steady = Math.Abs(s - last) < 0.1 ? steady + 1 : 0;
                last = s;
            }

            Assert.True(steady >= 3, $"the capsule's sink rate steady under the canopy ({last:F2} m/s)");
            into[0] = last;
        }

        private static void AssertOpen(RealChuteFAR chute, RealChuteFAR.DeploymentStates state, string what)
        {
            Assert.Equal(state, chute.DeploymentState, $"the chute {what} after the reload");
            Assert.True(chute.staged, $"the {what} chute staged after the reload");
            var cap = (Transform)Refl.Get(chute, "cap");
            var canopy = (Transform)Refl.Get(chute, "parachute");
            Assert.True(cap == null || !cap.gameObject.activeSelf, "the cap off");
            Assert.True(canopy != null && canopy.gameObject.activeSelf, "the canopy out");
            float Weight(string cube) => chute.part.DragCubes.Cubes.Where(d => d.Name == cube).Select(d => d.Weight).FirstOrDefault();
            Assert.Near(1, Weight("RCDEPLOYED"), 1e-3, "the deployed drag cube's weight");
            Assert.Near(0, Weight("PACKED"), 1e-3, "the packed drag cube's weight");
        }

        /// <summary>The game saved and loaded into flight again, as a quicksave and a quickload; the chute's new module in chute[0].</summary>
        private static IEnumerator SaveAndReload(TestContext ctx, string what, RealChuteFAR[] chute)
        {
            ctx.Step($"saved and reloaded {what}");
            Vessel before = FlightGlobals.ActiveVessel;
            uint id = chute[0].part.persistentId;
            string saved = GamePersistence.SaveGame(SaveName, HighLogic.SaveFolder, SaveMode.OVERWRITE);
            Assert.False(string.IsNullOrEmpty(saved), "the game saved");
            Game game = GamePersistence.LoadGame(SaveName, HighLogic.SaveFolder, true, false);
            Assert.NotNull(game, "the save loaded back");
            FlightDriver.StartAndFocusVessel(game, game.flightState.activeVesselIdx);
            yield return Wait.Until(() => HighLogic.LoadedSceneIsFlight && FlightGlobals.ready && FlightGlobals.ActiveVessel != null &&
                                          FlightGlobals.ActiveVessel != before && !FlightGlobals.ActiveVessel.packed, 300,
                                    $"the flight reloaded ({what})", shot: false);
            Flight.IgnoreMaxTemperature(true);
            Vessel v = FlightGlobals.ActiveVessel;
            chute[0] = v.FindPartModulesImplementing<RealChuteFAR>().FirstOrDefault(c => c.part.persistentId == id) ??
                       v.FindPartModulesImplementing<RealChuteFAR>().FirstOrDefault();
            Assert.NotNull(chute[0], "the reloaded capsule's chute");
        }

        /// <summary>
        /// RealChuteLite's timer (PhysicsWatch, what times a chute's deployment and failure) runs on the game's clock:
        /// started new, it counts the game time that passes; stopped, it holds what it counted while the clock goes on,
        /// and stopping it again changes nothing; started again it counts on from there, and started while running it
        /// keeps the time since it was last read. At the Space Center, where the clock runs.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.SpaceCenter, TimeoutSec = 300)]
        public static IEnumerator Behaviour_ChuteTimerRunsOnGameTime(TestContext ctx)
        {
            double ut0 = Planetarium.GetUniversalTime();
            PhysicsWatch watch = PhysicsWatch.StartNew();
            Assert.True(watch.IsRunning, "started new, it runs");
            // game time, not the wall clock's: a loaded machine runs the Space Center's clock slow (0.48 s of game
            // time in 2 s with four KSPs side by side)
            yield return Wait.Until(() => Planetarium.GetUniversalTime() - ut0 > 1, 60, "a second of game time", shot: false);
            double ran = Planetarium.GetUniversalTime() - ut0;
            watch.Stop();
            long held = watch.ElapsedMilliseconds;
            ctx.Record("first", $"{ran:F3} s of game time, {held} ms counted");
            Assert.Near(ran * 1000, held, 1, "it counted the game time that passed");
            Assert.False(watch.IsRunning, "stopped");

            double stopped = Planetarium.GetUniversalTime();
            yield return Wait.Until(() => Planetarium.GetUniversalTime() - stopped > 0.5, 60, "half a second of game time", shot: false);
            Assert.Equal(held, watch.ElapsedMilliseconds, "stopped, it holds what it counted");
            watch.Stop();
            Assert.Equal(held, watch.ElapsedMilliseconds, "stopped again, still");

            double ut1 = Planetarium.GetUniversalTime();
            watch.Start();
            yield return Wait.Until(() => Planetarium.GetUniversalTime() - ut1 > 0.5, 60, "half a second more of game time", shot: false);
            double more = Planetarium.GetUniversalTime() - ut1;
            long counted = watch.ElapsedMilliseconds;
            ctx.Record("again", $"{more:F3} s more, {counted} ms counted");
            Assert.True(watch.IsRunning, "started again, it runs");
            Assert.Near(held + more * 1000, counted, 2, "it counted on from what it held");

            double ut2 = Planetarium.GetUniversalTime();
            yield return Wait.Until(() => Planetarium.GetUniversalTime() - ut2 > 0.5, 60, "half a second more, unread", shot: false);
            double since = Planetarium.GetUniversalTime() - ut1;
            watch.Start();
            Assert.True(watch.IsRunning, "started while running, it runs");
            Assert.Near(held + since * 1000, watch.ElapsedMilliseconds, 2, "started while running, it kept the time since it was last read");
        }
    }
}
