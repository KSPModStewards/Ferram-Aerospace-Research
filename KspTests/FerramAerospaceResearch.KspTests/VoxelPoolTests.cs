using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.FARThreading;
using HarmonyLib;
using KspTest;
using static FerramAerospaceResearch.KspTests.Refl;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's pool of voxel chunks, shared by every voxel model in the game. Each model built takes its chunks from the
    /// pool and gives them back when it is done with. A chunk given back twice, or given back while a model still uses
    /// it, ends up in two places of a model at once, and from then on every voxelization throws (IndexOutOfRange in
    /// VoxelChunk) until the game restarts.
    /// </summary>
    public static class VoxelPoolTests
    {
        private const string Airliner = "Stearwing A300";
        private const string HarmonyId = "FerramAerospaceResearch.KspTests.VoxelPool";

        /// <summary>
        /// The editor gives the old model's chunks back when it queues a new voxelization. Leaving the editor while
        /// the new model is being built (launching right after a change, say) must not give them back again: by then
        /// the new model has them. The new model is held on FAR's voxelization thread as it comes out of
        /// VehicleVoxel.CreateNewVoxel while the test leaves the editor, so the voxelization is always still running
        /// when the editor closes. Then no free chunk is one the held model uses, none is free twice, and once it is
        /// let go both models' chunks are given back, each once, with no FAR exception.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Airliner, TimeoutSec = 900)]
        public static IEnumerator Editor_LeftWhileVoxelizing(TestContext ctx)
        {
            bool wasMain = VoxelizationThreadpool.RunInMainThread;
            var harmony = new Harmony(HarmonyId);
            var log = new ExceptionLog();
            var pool = (Stack<VoxelChunk>)GetStatic(typeof(VehicleVoxel), "clearedChunks");
            try
            {
                // held on the main thread, the model would hold the game
                VoxelizationThreadpool.RunInMainThread = false;
                harmony.Patch(AccessTools.Method(typeof(VehicleVoxel), nameof(VehicleVoxel.CreateNewVoxel)),
                              postfix: new HarmonyMethod(typeof(Hold), nameof(Hold.Postfix)));

                yield return Far.WaitForEditorVoxels();
                yield return Far.Settled("the airliner");
                VehicleAerodynamics aero = Far.EditorVehicle;
                var old = Get<VehicleVoxel>(aero, "_voxel");
                Assert.True(old != null, "the editor keeps the airliner's voxel model");
                int oldChunks = ChunkCount(old), inUse = InUse();

                ctx.Step("Voxelized again, the new model held on FAR's voxelization thread");
                Hold.Arm();
                EditorGUI.RequestUpdateVoxel();
                yield return Wait.Until(() => Hold.Reached.WaitOne(0), 120, "FAR's voxelization thread to build the new model", shot: false);
                VehicleVoxel built = Hold.Built;
                var usedByBuilt = new HashSet<VoxelChunk>(Chunks(built));

                ctx.Step("Left the editor with the new model still held");
                yield return Nav.ToSpaceCenter(ctx);
                Assert.True(!Hold.TimedOut, "the new model still held as the scene closed (the hold gave up first)");
                VoxelChunk[] free;
                lock (pool)
                    free = pool.ToArray();
                int shared = free.Count(usedByBuilt.Contains), twice = free.Length - new HashSet<VoxelChunk>(free).Count;
                ctx.Record("pool", $"{free.Length} free chunks, {twice} of them free twice, {shared} of them in the held model " +
                                   $"({usedByBuilt.Count} chunks); the old model had {oldChunks}");
                Assert.Equal(0, shared, "no free chunk is one the held model uses");
                Assert.Equal(0, twice, "no chunk free twice");

                ctx.Step("The held model let go");
                Hold.Release.Set();
                yield return Wait.Until(() => !aero.Voxelizing, 120, "FAR's voxelization to end", shot: false);
                // FAR logs a voxelization thread's exception from the main thread
                yield return Wait.Seconds(2);
                int after = InUse();
                ctx.Record("chunks in use", $"{inUse} with the old model, {after} once both are given back (the old model had {oldChunks})");
                Assert.Equal(inUse - oldChunks, after, "both models' chunks given back, each once");
                log.AssertNone("leaving the editor while it voxelizes");
            }
            finally
            {
                Hold.Disarm();
                log.Dispose();
                harmony.UnpatchAll(HarmonyId);
                VoxelizationThreadpool.RunInMainThread = wasMain;
            }
        }

        /// <summary>
        /// With FAR's debugInFlight on, a flight vessel keeps its voxel model (the exposure tests turn it on to get the
        /// voxel bounds) until the vessel next voxelizes. A vessel that goes (left behind with the scene, destroyed,
        /// recovered) must give the model's chunks back: nothing else ever will, and once the pool's chunks are all in
        /// use every voxelization waits for good.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_LeftWithTheVoxelModelKept(TestContext ctx)
        {
            var debug = new DebugInFlight();
            var log = new ExceptionLog();
            try
            {
                Vessel v = FlightGlobals.ActiveVessel;
                yield return Wait.Until(() => FARAPI.VesselVoxelizationCompletedEver(v), 120, "the plane voxelized", shot: false);

                ctx.Step("Voxelized again with the model kept (debugInFlight)");
                FARConfig.Voxelization.DebugInFlight = true;
                var vesselAero = v.GetComponent<FARVesselAero>();
                VehicleAerodynamics aero = Get<VehicleAerodynamics>(vesselAero, "_vehicleAero");
                vesselAero.VesselUpdateEvent(v);
                yield return Wait.Until(() => !aero.Voxelizing && Get<VehicleVoxel>(aero, "_voxel") != null, 120, "the plane's voxel model kept", shot: false);
                yield return Wait.Until(() => FlightVessels().All(a => !a.Voxelizing), 120, "every flight vessel's voxelization to end", shot: false);
                int kept = FlightVessels().Select(a => Get<VehicleVoxel>(a, "_voxel")).Where(x => x != null).Sum(ChunkCount);
                int inUse = InUse();

                ctx.Step("Left flight");
                yield return Nav.ToSpaceCenter(ctx);
                yield return Wait.Seconds(2);
                int after = InUse();
                ctx.Record("chunks in use", $"{inUse} in flight, {kept} of them in kept models; {after} in the Space Center");
                Assert.Equal(inUse - kept, after, "the kept models' chunks given back when their vessels went");
                log.AssertNone("leaving flight with the voxel model kept");
            }
            finally
            {
                log.Dispose();
                debug.Dispose();
            }
        }

        /// <summary>
        /// In flight a vessel's voxel model is given back as soon as its aerodynamics are worked out, at the end of the
        /// voxelization, but only while the game is still in flight. Leaving flight while a vessel voxelizes must not keep
        /// the model's chunks, nor leave the voxelization waiting for good on main-thread work no scene runs any more.
        /// The model is held on FAR's voxelization thread as it comes out of VehicleVoxel.CreateNewVoxel until the game
        /// is in the Space Center.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_LeftWhileVoxelizing(TestContext ctx)
        {
            bool wasMain = VoxelizationThreadpool.RunInMainThread;
            var debug = new DebugInFlight();
            var harmony = new Harmony(HarmonyId);
            var log = new ExceptionLog();
            try
            {
                // the model given back at the end of the voxelization, the way FAR ships
                FARConfig.Voxelization.DebugInFlight = false;
                VoxelizationThreadpool.RunInMainThread = false;
                harmony.Patch(AccessTools.Method(typeof(VehicleVoxel), nameof(VehicleVoxel.CreateNewVoxel)),
                              postfix: new HarmonyMethod(typeof(Hold), nameof(Hold.Postfix)));

                Vessel v = FlightGlobals.ActiveVessel;
                yield return Wait.Until(() => FARAPI.VesselVoxelizationCompletedEver(v), 120, "the plane voxelized", shot: false);
                var vesselAero = v.GetComponent<FARVesselAero>();
                VehicleAerodynamics aero = Get<VehicleAerodynamics>(vesselAero, "_vehicleAero");
                yield return Wait.Until(() => FlightVessels().All(a => !a.Voxelizing), 120, "every flight vessel's voxelization to end", shot: false);
                // a model kept from before (debugInFlight on as the vessel voxelized) goes when the new one is queued
                int inUse = InUse() - FlightVessels().Select(a => Get<VehicleVoxel>(a, "_voxel")).Where(x => x != null).Sum(ChunkCount);

                ctx.Step("Voxelized again, the new model held on FAR's voxelization thread");
                Hold.Arm();
                vesselAero.VesselUpdateEvent(v);
                yield return Wait.Until(() => Hold.Reached.WaitOne(0), 120, "FAR's voxelization thread to build the new model", shot: false);
                int built = ChunkCount(Hold.Built);

                ctx.Step("Left flight with the new model still held");
                yield return Nav.ToSpaceCenter(ctx);
                Assert.True(!Hold.TimedOut, "the new model still held as the scene closed (the hold gave up first)");

                ctx.Step("The held model let go");
                Hold.Release.Set();
                yield return Wait.Until(() => !aero.Voxelizing, 120, "FAR's voxelization to end", shot: false);
                yield return Wait.Seconds(2);
                int after = InUse();
                ctx.Record("chunks in use", $"{inUse} before the held model ({built} chunks), {after} once it is let go in the Space Center");
                Assert.Equal(inUse, after, "the held model's chunks given back");
                log.AssertNone("leaving flight while it voxelizes");
            }
            finally
            {
                Hold.Disarm();
                log.Dispose();
                harmony.UnpatchAll(HarmonyId);
                VoxelizationThreadpool.RunInMainThread = wasMain;
                debug.Dispose();
            }
        }

        private static IEnumerable<VehicleAerodynamics> FlightVessels() =>
            FlightGlobals.Vessels.Where(v => v != null).Select(v => v.GetComponent<FARVesselAero>()).Where(a => a != null)
                         .Select(a => Get<VehicleAerodynamics>(a, "_vehicleAero")).Where(a => a != null);

        /// <summary>FAR's debugInFlight put back as it was when a test ends, and FAR's config file with it (FarConfigFile).</summary>
        private sealed class DebugInFlight : IDisposable
        {
            private readonly bool was = FARConfig.Voxelization.DebugInFlight;
            private readonly FarConfigFile file = new FarConfigFile();

            public void Dispose()
            {
                FARConfig.Voxelization.DebugInFlight = was;
                file.Restore();
            }
        }

        private static int InUse() => (int)GetStatic(typeof(VehicleVoxel), "chunksInUse");

        private static int ChunkCount(VehicleVoxel v) => Get<int>(v, "xLength") * Get<int>(v, "yLength") * Get<int>(v, "zLength");

        private static IEnumerable<VoxelChunk> Chunks(VehicleVoxel v) =>
            Get<VoxelChunk[,,]>(v, "voxelChunks").Cast<VoxelChunk>().Where(c => c != null);

        /// <summary>The next model VehicleVoxel.CreateNewVoxel returns, held on its thread until released.</summary>
        private static class Hold
        {
            private static int armed;
            public static volatile VehicleVoxel Built;
            /// <summary>The hold gave up before the test let it go: what the test checked did not happen during it.</summary>
            public static volatile bool TimedOut;
            public static readonly ManualResetEvent Reached = new ManualResetEvent(false);
            public static readonly ManualResetEvent Release = new ManualResetEvent(false);

            public static void Arm()
            {
                Built = null;
                TimedOut = false;
                Reached.Reset();
                Release.Reset();
                Interlocked.Exchange(ref armed, 1);
            }

            public static void Disarm()
            {
                Interlocked.Exchange(ref armed, 0);
                Release.Set();
            }

            public static void Postfix(VehicleVoxel __result)
            {
                if (Interlocked.Exchange(ref armed, 0) == 0)
                    return;
                Built = __result;
                Reached.Set();
                // never for good: the test lets it go when it ends however it ends (the harness disposes a test it
                // stops), and so does this, well before a voxelization the next test waits on gives up; stretched on a
                // run known to be slow, as the test's own waits are
                if (!Release.WaitOne(TimeSpan.FromSeconds(60 * Math.Max(1, Wait.TimeoutScale))))
                    TimedOut = true;
            }
        }
    }
}
