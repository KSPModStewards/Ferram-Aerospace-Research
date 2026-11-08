using System;
using System.Collections.Generic;
using FerramAerospaceResearch.Resources;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Profiling;
using UnityEngine.Rendering;

namespace FerramAerospaceResearch.Geometry.Exposure;

/// <summary>
/// A custom renderer job executor replacement for coroutines, avoiding costly GC allocations
/// </summary>
[DefaultExecutionOrder(-1)]
public class Executor : MonoSingleton<Executor>
{
    public struct JobInfo
    {
        public AsyncGPUReadbackRequest readbackRequest;
        public NativeSlice<uint> texture;
        public NativeSlice<int> pixelCounts;
        public PhysicalDevice device;

        public Action<JobInfo, object> callback;

        // additional data to pass to the callback to avoid closure allocations
        public object userData;
    }

    /// <summary>
    /// A readonly handle to the renderer job
    /// </summary>
    public readonly struct Handle
    {
        private readonly JobData job;

        internal Handle(JobData job)
        {
            this.job = job;
        }

        public bool IsValid
        {
            get { return job is not null; }
        }

        public bool IsDone
        {
            get { return job.done; }
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public State CurrentState
        // {
        //     get { return job.state; }
        // }

        /// <summary>
        /// Cancel the associated job, may not have an immediate effect as jobs are executed asynchronously
        /// </summary>
        public void Cancel()
        {
            job?.Cancel();
        }

        public void WaitForUpdate()
        {
            job?.WaitForAnyCompletion();
        }
    }

    public enum State
    {
        Readback,
        CountJob,
    }

    internal class JobData
    {
        public State state;
        public JobInfo info;
        public JobHandle countHandle;
        public bool done;
        public bool cancelled;

        public void Cancel()
        {
            // the job's owner frees its arrays once its readback is in (RenderBatch's JobData.Dispose): a cancelled CPU
            // job must not go on to count pixels in them
            cancelled = true;

            // cancelled, disable callback as well
            info.callback = null;
        }

        public bool Update()
        {
            if (done)
                return true;

            // even if this job is cancelled we have to wait for the async jobs to finish before we can release the resources

            switch (state)
            {
                case State.Readback:
                    if (!info.readbackRequest.done)
                        return false;
                    if (info.readbackRequest.hasError)
                    {
                        FARLogger.Warning("Error in readback request");
                        return true;
                    }

                    info.readbackRequest.WaitForCompletion();
                    if (info.device is PhysicalDevice.GPU)
                    {
                        info.pixelCounts = info.readbackRequest.GetData<int>();
                        return true;
                    }

                    // don't submit a new job if we're cancelled
                    if (cancelled)
                        return true;

                    info.texture = info.readbackRequest.GetData<uint>();
                    countHandle = Renderer.CountPixels(info.texture, info.pixelCounts);
                    state = State.CountJob;
                    return false;

                case State.CountJob:
                    if (!countHandle.IsCompleted)
                        return false;
                    countHandle.Complete();
                    return true;

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        public bool Next()
        {
            return done = Update();
        }

        public void WaitForAnyCompletion()
        {
            if (done)
                return;

            switch (state)
            {
                case State.Readback:
                    info.readbackRequest.WaitForCompletion();
                    break;
                case State.CountJob:
                    countHandle.Complete();
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }
    }

    private static readonly List<JobData> activeJobs = new();
    private static readonly Pool<JobData> jobDataPool = new(() => new JobData(), OnJobDataRelease);

    private static void OnJobDataRelease(JobData job)
    {
        try
        {
            job.info.callback?.Invoke(job.info, job.info.userData);
        }
        catch (Exception e)
        {
            FARLogger.Exception(e, "Caught exception while executing job callback");
        }

        job.info = default;
        job.countHandle = default;
    }

    private static bool UpdateJob(JobData job)
    {
        bool completed = job.Next();
        if (completed)
            jobDataPool.Release(job);

        return completed;
    }

    private static readonly Predicate<JobData> updateJobPredicate = UpdateJob;

    public static void UpdateOnce()
    {
        Profiler.BeginSample("Exposure.Executor.UpdateOnce");
        activeJobs.RemoveAll(updateJobPredicate);
        Profiler.EndSample();
    }

    private void Update()
    {
        UpdateOnce();
    }

    public static void CancelAll()
    {
        foreach (JobData job in activeJobs)
            job.Cancel();
    }

    public static int JobsCount
    {
        get { return activeJobs.Count; }
    }

    // ReSharper disable once MemberCanBeMadeStatic.Global need to make sure the instance exists to have updates
    public Handle Execute(in JobInfo info)
    {
        JobData job = jobDataPool.Acquire();
        job.done = false;
        job.state = State.Readback;
        job.info = info;
        job.cancelled = false;
        activeJobs.Add(job);

        return new Handle(job);
    }

    protected override void OnDestruct()
    {
        CancelAll();
        foreach (JobData job in activeJobs)
            job.WaitForAnyCompletion();
        activeJobs.Clear();
        jobDataPool.Clear();
    }
}
