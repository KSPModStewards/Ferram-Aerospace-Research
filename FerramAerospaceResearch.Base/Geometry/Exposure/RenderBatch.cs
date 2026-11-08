using System;
using System.Collections.Generic;
using System.Linq;
using FerramAerospaceResearch.Resources;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using UnityEngine.Rendering;

namespace FerramAerospaceResearch.Geometry.Exposure;

public class RenderBatch : IDisposable
{
    private class JobData : IDisposable
    {
        public readonly RenderResult DefaultResult = new();
        public readonly RenderResources Resources = new();
        public RenderRequest request;
        public Executor.Handle handle;
        public RenderResult result;

        private void ReleaseUnmanagedResources()
        {
            if (handle.IsValid)
            {
                handle.Cancel();
                handle.WaitForUpdate();
            }

            Resources.Dispose();
        }

        private void Dispose(bool disposing)
        {
            ReleaseUnmanagedResources();
            if (disposing)
            {
            }
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        ~JobData()
        {
            Dispose(false);
        }
    }

    private bool isDirty = true;

    // command buffer used to render all requested directions in current/last job
    private readonly CommandBuffer commandBuffer;

    // per request data
    private readonly List<JobData> jobCache = new();

    // number of request from last time
    private int JobsCount { get; set; }

    public int ActiveJobs { get; private set; }

    // material used to render the objects
    private Material material;

    public Material Material
    {
        get { return material; }
        set
        {
            if (!ReferenceEquals(material, value))
            {
                isDirty = true;
                jobCache.ForEach(job => job.Resources.ReleaseMaterial());
            }

            material = value;
        }
    }

    // compute shader to count pixels
    private ComputeShader pixelCountShader;

    public ComputeShader PixelCountShader
    {
        get { return pixelCountShader; }
        set
        {
            if (!ReferenceEquals(pixelCountShader, value))
            {
                isDirty = true;
                jobCache.ForEach(job => job.Resources.ReleaseComputeShader());
            }

            pixelCountShader = value;
        }
    }

    private Kernel pixelCountKernel;

    public Kernel PixelCountKernel
    {
        get { return pixelCountKernel; }
        set
        {
            pixelCountKernel = value;
            isDirty = true;
            foreach (JobData job in jobCache)
                job.Resources.MainKernel = value;
        }
    }

    private int2 renderSize = 512;

    public int2 RenderSize
    {
        get { return renderSize; }
        set
        {
            if (math.any(value != renderSize))
            {
                isDirty = true;
                jobCache.ForEach(job => job.Resources.ReleaseTextures());
            }

            renderSize = value;
        }
    }

    private PhysicalDevice device = PhysicalDevice.GPU;

    public PhysicalDevice Device
    {
        get { return device; }
        set
        {
            // set, not cleared: the same device again must not drop a rebuild another change (size, material) asked for
            isDirty |= value != device;
            device = value;
        }
    }

    public bool IsDone
    {
        get { return ActiveJobs == 0; }
    }

    private Action<RenderBatch, object> callback;
    private object userData;
    private Action<Executor.JobInfo, object> onJobCompletedAction;

    public RenderBatch()
    {
        commandBuffer = new CommandBuffer { name = "ExposedSurface" };
        onJobCompletedAction = OnJobCompleted; // circular reference so clean up in Dispose
    }

    private void ReleaseUnmanagedResources()
    {
        foreach (JobData data in jobCache)
            data.Dispose();
        jobCache.Clear();
    }

    private void Dispose(bool disposing)
    {
        onJobCompletedAction = default;
        ReleaseUnmanagedResources();
        if (disposing)
        {
            commandBuffer?.Dispose();
        }
    }

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    ~RenderBatch()
    {
        Dispose(false);
    }

    /// <summary>
    /// Mark CommandBuffer for reconstruction on the next request
    /// </summary>
    public void ReconstructCommandBuffer()
    {
        isDirty = true;
    }

    public unsafe bool Execute<T>(
        List<HashSet<UnityEngine.Renderer>> renderers,
        T requests,
        in Bounds bounds,
        in float4x4 transform,
        Action<RenderBatch, object> onCompleted = null,
        object callbackData = null
    ) where T : IReadOnlyList<RenderRequest>
    {
        bool success = true;
        Profiler.BeginSample("Exposure.Batch.Execute");
        FARLogger.Assert(IsDone, "Cannot request a new job while the old is not done yet!");

        isDirty |= requests.Count != JobsCount;
        if (isDirty)
            commandBuffer.Clear();

        float3 center = math.transform(transform, bounds.center);

        float3* cornersBuffer = stackalloc float3[8];
        NativeSlice<float3> corners =
            NativeSliceUnsafeUtility.ConvertExistingDataToNativeSlice<float3>(cornersBuffer, sizeof(float3), 8);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        NativeSliceUnsafeUtility.SetAtomicSafetyHandle(ref corners, AtomicSafetyHandle.GetTempUnsafePtrSliceHandle());
#endif
        Renderer.TransformBoundsCorners(corners, bounds, transform);

        for (int i = 0; i < requests.Count; ++i)
        {
            JobData data;
            if (i < jobCache.Count)
                data = jobCache[i];
            else
            {
                data = new JobData();
                jobCache.Add(data);
            }

            data.request = requests[i];
            data.result = data.request.result ?? data.DefaultResult;
            float3 lookDir = math.rotate(transform, data.request.lookDir);
            success &= CreateJob(ref data, renderers, center, lookDir, corners);
        }

        isDirty = false;
        JobsCount = requests.Count;

        Profiler.BeginSample("Exposure.Batch.ExecuteCommandBuffer");
        Graphics.ExecuteCommandBuffer(commandBuffer);
        Profiler.EndSample();

        callback = onCompleted;
        userData = callbackData;

        Profiler.BeginSample("Exposure.Batch.Submit");
        for (int i = 0; i < JobsCount; ++i)
            Submit(jobCache[i]);
        Profiler.EndSample();

        Profiler.EndSample();

        return success;
    }

    private bool CreateJob(
        ref JobData data,
        List<HashSet<UnityEngine.Renderer>> renderers,
        in float3 center,
        in float3 lookDir,
        in NativeSlice<float3> corners
    )
    {
        bool success = true;
        Profiler.BeginSample("Exposure.Batch.CreateJob");
        if (isDirty)
        {
            data.Resources.PrepareForNextJob(texSize: renderSize,
                                             count: renderers.Count,
                                             compute: device is PhysicalDevice.GPU,
                                             jobMaterial: Material,
                                             shader: PixelCountShader);
            success = AppendCommands(data.Resources, renderers);
        }
        else
        {
            data.Resources.PrepareForNextJob(renderers.Count, device is PhysicalDevice.GPU);
        }

        CameraInfo proj = Renderer.ProjectToCameraRender(center, lookDir, corners);
        data.Resources.material.SetMatrix(ShaderPropertyIds._VPMatrix, proj.vpMatrix);

        data.result.position = proj.position;
        data.result.forward = proj.forward;
        data.result.centerDistance = proj.centerDistance;
        data.result.areaPerPixel = proj.projectedArea / (renderSize.x * renderSize.y);
        data.result.pixelCounts = data.Resources.ActualPixelCounts;
        Profiler.EndSample();

        return success;
    }

    private bool AppendCommands(in RenderResources data, List<HashSet<UnityEngine.Renderer>> renderers)
    {
        bool success = true;
        Profiler.BeginSample("Exposure.Batch.AppendCommands");

        // draw all renderers to the target
        commandBuffer.BeginSample("Exposure.Batch.Render");
        commandBuffer.SetRenderTarget(data.target);
        commandBuffer.ClearRenderTarget(true, true, Color.white);
        int i = -1;
        foreach (HashSet<UnityEngine.Renderer> objectRenderers in renderers)
        {
            ++i;
            int j = -1;
            foreach (UnityEngine.Renderer renderer in objectRenderers)
            {
                ++j;
                if (renderer == null)
                {
                    FARLogger.Warning($"Renderer {renderer} at ({i}, {j}) is null!");
                    success = false;
                    continue;
                }

                commandBuffer.DrawRenderer(renderer, data.material);
            }
        }

        commandBuffer.EndSample("Exposure.Batch.Render");

        // also dispatch the compute shader in the same command buffer
        if (device is PhysicalDevice.GPU)
        {
            commandBuffer.BeginSample("Exposure.Batch.Compute");
            commandBuffer.SetComputeTextureParam(data.computeShader,
                                                 pixelCountKernel.index,
                                                 ShaderPropertyIds.InputTexture,
                                                 data.target,
                                                 0,
                                                 RenderTextureSubElement.Color);
            commandBuffer.DispatchCompute(data.computeShader,
                                          pixelCountKernel.index,
                                          (data.target.width + (int)pixelCountKernel.threadGroupSizes.x - 1) /
                                          (int)pixelCountKernel.threadGroupSizes.x,
                                          (data.target.height + (int)pixelCountKernel.threadGroupSizes.y - 1) /
                                          (int)pixelCountKernel.threadGroupSizes.y,
                                          1);
            commandBuffer.EndSample("Exposure.Batch.Compute");
        }

        Profiler.EndSample();

        return success;
    }

    private void Submit(JobData data)
    {
        data.handle = Executor.Instance.Execute(new Executor.JobInfo
        {
            device = device,
            pixelCounts = data.result.pixelCounts,
            readbackRequest = RequestReadback(data.Resources),
            texture = data.Resources.ActualTexture,
            userData = data,
            callback = onJobCompletedAction
        });

        ActiveJobs += 1;
    }

    private AsyncGPUReadbackRequest RequestReadback(in RenderResources data)
    {
        if (device is PhysicalDevice.GPU)
        {
            NativeArray<int> pixels = data.ActualPixelCounts;
            return AsyncGPUReadback.RequestIntoNativeArray(ref pixels,
                                                           data.outputBuffer,
                                                           sizeof(int) * pixels.Length,
                                                           0);
        }

        NativeArray<uint> tex = data.ActualTexture;
        return AsyncGPUReadback.RequestIntoNativeArray(ref tex, data.target);
    }

    public Executor.Handle Handle(int index)
    {
        if (index >= JobsCount)
            throw new ArgumentOutOfRangeException($"{index} out of range for size {JobsCount}");

        return jobCache[index].handle;
    }

    public void Apply(Action<Executor.Handle> func)
    {
        for (int i = 0; i < JobsCount; ++i)
            func(jobCache[i].handle);
    }

    private void OnJobCompleted(Executor.JobInfo info, object o)
    {
        var data = o as JobData;
        FARLogger.Assert(data != null, nameof(data) + " != null");

        // ReSharper disable once PossibleNullReferenceException
        data.result.pixelCounts = info.pixelCounts;

        int count = data.result.pixelCounts.Length;
        int listCount = data.result.Areas.Count;
        if (listCount != count)
        {
            if (listCount < count)
                data.result.Areas.AddRange(Enumerable.Repeat<double>(0, count - listCount));
            else
                data.result.Areas.RemoveRange(count, listCount - count);
        }

        for (int i = 0; i < count; ++i)
            data.result.Areas[i] = info.pixelCounts[i] * data.result.areaPerPixel;

        data.request.debugger?.ReceiveResult(data.request, data.result, data.Resources.target);

        try
        {
            data.request.callback?.Invoke(data.result, data.request.userData);
        }
        catch (Exception e)
        {
            FARLogger.Exception(e, "Caught exception while executing request callback");
        }

        data.handle = default;
        data.request = default;
        DecrementActiveJobs();
    }

    private void DecrementActiveJobs()
    {
        ActiveJobs -= 1;
        if (!IsDone)
            return;

        try
        {
            callback?.Invoke(this, userData);
        }
        catch (Exception e)
        {
            FARLogger.Exception(e, "Caught exception while executing batch callback");
        }

        callback = default;
        userData = default;
    }
}
