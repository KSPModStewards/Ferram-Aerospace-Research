using System;
using System.Collections.Generic;
using System.Threading;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.FARThreading;

namespace FerramAerospaceResearch.FARAeroComponents
{
    // --- made ahead: a part's water model readied on a worker thread before the part is wet ---
    //
    // A part's first time in the water made its mesh (~85 ms for a Mk3 cargo ramp), its closed-in faces (~45 ms, again
    // as each neighbour came in) and its dry candidates (~250 ms) in the steps it touched: a boat lowered in stalled for
    // seconds. None of them needs the water. A vessel low over a sea (PrewarmAltitude, the ground under it below
    // PrewarmGroundBelow) has them made on the voxel threads beforehand: each part's FAR meshes and its neighbours'
    // places taken on the main thread, the work done on a worker, the result put in on the main thread when done if
    // what it was made of is still the part's. One needed before it's done is waited for, or made on the main thread if
    // no worker has started it. A part whose job failed is made ahead no more (made as it's needed, as before).
    internal sealed partial class PartHydrodynamics
    {
        public static double PrewarmAltitude = 1000, PrewarmGroundBelow = 0;
        private const int PrewarmJobsMax = 4;
        // jobs queued and not yet run (any thread's count: a job whose part is gone or never asks for it again still
        // runs on its worker, so it still frees its place)
        private static int prewarmJobs;
        private bool prewarmFailed;

        private sealed class Job<T> where T : class
        {
            private int state;   // 0 queued, 1 running, 2 done
            private readonly Func<T> work;
            private readonly ManualResetEventSlim finished = new ManualResetEventSlim(false);
            public T Result;
            public Exception Error;

            public Job(Func<T> work) => this.work = work;

            public bool Done => Volatile.Read(ref state) == 2;

            // (a worker's, or the main thread's when it needs it first: whichever starts it does it)
            public void Run()
            {
                if (Interlocked.CompareExchange(ref state, 1, 0) != 0)
                    return;
                try
                {
                    Result = work();
                }
                catch (Exception e)
                {
                    Error = e;
                }
                finally
                {
                    Volatile.Write(ref state, 2);
                    finished.Set();
                    Interlocked.Decrement(ref prewarmJobs);
                }
            }

            // its result now (main thread): made here if no worker has started it, else waited for
            public T Get()
            {
                Run();
                finished.Wait();
                return Result;
            }
        }

        private Job<MeshBuild> meshJob;
        private List<GeometryMesh> meshJobSource;
        private Job<ShadowBuild> shadowJob;
        private long shadowJobKey;

        private static Job<T> Queue<T>(Func<T> work) where T : class
        {
            var j = new Job<T>(work);
            Interlocked.Increment(ref prewarmJobs);
            VoxelizationThreadpool.Instance.QueueVoxelization(j.Run);
            return j;
        }

        private T Finish<T>(ref Job<T> job) where T : class
        {
            Job<T> j = job;
            job = null;
            T r = j.Get();
            if (j.Error == null)
                return r;
            prewarmFailed = true;
            FARLogger.Exception(j.Error, $"Hydrodynamics of {part.partInfo?.name}: made ahead (no more for it)");
            return null;
        }

        // EnsureMesh's: the job made for these meshes (null: none for them)
        private MeshBuild TakeMeshJob(List<GeometryMesh> meshes)
        {
            if (meshJob == null || !ReferenceEquals(meshJobSource, meshes))
                return null;
            MeshBuild b = Finish(ref meshJob);
            return b != null && !PosesMoved(meshes, b.Poses, b.Shown) ? b : null;
        }

        // Shadowed's: the job made for this key, of this mesh (null: none)
        private ShadowBuild TakeShadowJob(long key)
        {
            if (shadowJob == null || shadowJobKey != key)
                return null;
            ShadowBuild b = Finish(ref shadowJob);
            return b != null && b.Version == meshVersion ? b : null;
        }

        private List<GeometryMesh> CurrentMeshes()
        {
            GeometryPartModule g = part.GetComponent<GeometryPartModule>();
            return g != null ? g.meshDataList : null;
        }

        // FAR has meshes for it the water model hasn't taken yet
        private bool MeshPending()
        {
            List<GeometryMesh> meshes = CurrentMeshes();
            return meshJob != null || meshes != null && meshes.Count > 0 && !MeshesCurrent(meshes);
        }

        // a step of readying this part: a finished job's result put in, else the next job queued (its mesh, then its
        // closed-in faces once its neighbours' meshes are in)
        private void PrewarmStep()
        {
            if (meshJob != null && meshJob.Done)
            {
                List<GeometryMesh> source = meshJobSource;
                MeshBuild b = Finish(ref meshJob);
                if (b != null && ReferenceEquals(source, CurrentMeshes()) && !PosesMoved(source, b.Poses, b.Shown) && !MeshesCurrent(source))
                    InstallMesh(b);
            }
            if (shadowJob != null && shadowJob.Done)
            {
                long key = shadowJobKey;
                ShadowBuild b = Finish(ref shadowJob);
                if (b != null && b.Version == meshVersion && key != shadowKey && key == ShadowKey())
                {
                    poseWhy = "made ahead";
                    InstallShadow(b);
                }
            }
            if (prewarmFailed || meshJob != null || shadowJob != null || Volatile.Read(ref prewarmJobs) >= PrewarmJobsMax)
                return;
            List<GeometryMesh> meshes = CurrentMeshes();
            if (meshes != null && meshes.Count > 0 && !MeshesCurrent(meshes))
            {
                MeshCapture c = CaptureMesh(meshes);
                if (SameMesh(c.Verts, c.Tris))
                {
                    meshSource = meshes;
                    meshPoses = c.Poses;
                    meshShown = c.Shown;
                    ProfileMeshSame++;
                }
                else
                {
                    bool dry = !IsHullPlate;
                    meshJobSource = meshes;
                    meshJob = Queue(() => BuildMesh(c, dry));
                    return;
                }
            }
            if (Foil || tris.Length == 0)
                return;
            long shadow = ShadowKey();
            if (shadow == shadowKey)
                return;
            foreach (FARAeroPartModule m in nearModules)
                if (m != null && (m.Hydrodynamics == null || m.Hydrodynamics.MeshPending()))
                    return;
            ShadowCapture sc = CaptureShadow(shadow);
            shadowJobKey = shadow;
            shadowJob = Queue(() => BuildShadow(sc));
        }

        /// <summary>
        /// FARVesselAero's: a vessel low over a sea (below PrewarmAltitude over a body with an ocean, the ground under it
        /// below PrewarmGroundBelow or it in the water, loaded and in physics) has its parts' water model readied
        /// (PrewarmStep each). Not over land: a rocket on its pad or a plane on its runway, near sea level, would have
        /// all of it made for nothing.
        /// </summary>
        internal static void PrewarmVessel(Vessel v)
        {
            if (!FARDebugValues.hydrodynamics || v == null || !v.loaded || v.packed || v.mainBody == null || !v.mainBody.ocean ||
                v.altitude > PrewarmAltitude ||
                !v.Splashed && v.mainBody.TerrainAltitude(v.latitude, v.longitude, true) >= PrewarmGroundBelow)
                return;
            foreach (Part p in v.parts)
            {
                if (p == null)
                    continue;
                FARAeroPartModule m = p.Modules.GetModule<FARAeroPartModule>();
                if (m != null)
                    m.EnsureHydrodynamics().PrewarmStep();
            }
        }
    }
}
