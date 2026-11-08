using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using FerramAerospaceResearch.Resources;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Object = UnityEngine.Object;

namespace FerramAerospaceResearch.Geometry.Exposure;

[StructLayout(LayoutKind.Explicit)]
public struct ColorUintConverter
{
    [FieldOffset(0)] private uint u;
    [FieldOffset(0)] private Color32 c;

    public static Color32 AsColor(uint value)
    {
        return new ColorUintConverter { u = value }.c;
    }

    // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
    // public static uint AsUint(Color32 color)
    // {
    //     return new ColorUintConverter { c = color }.u;
    // }
}

[BurstCompile]
public struct PixelCountJob : IJobParallelFor
{
    [ReadOnly] public NativeSlice<uint> texture;
    public NativeSlice<int> pixels;

    public void Execute(int index)
    {
        uint data = texture[index];
        if (data >= pixels.Length)
            return;

        unsafe
        {
            // [] returns a copy so cannot use ref with it, instead access from the buffer pointer directly
            int* pixelBegin = (int*)pixels.GetUnsafePtr();
            Interlocked.Increment(ref *(pixelBegin + data));
        }
    }
}

public static class Renderer
{
    private static readonly float3 cameraScale;

    static Renderer()
    {
        cameraScale = new float3(1, 1, SystemInfo.usesReversedZBuffer ? -1 : 1);
    }

    public static void TransformBoundsCorners(NativeSlice<float3> corners, in Bounds bounds, in float4x4 transform)
    {
        if (corners.Length != 8)
            throw new ArgumentException($"Invalid number of corners allocated: expected 8 but got {corners.Length}");

        Vector3 min = bounds.min;
        Vector3 max = bounds.max;

        // NativeSlice throws errors accessing its items if it wasn't created from a valid native array such as from stackalloc
        corners[0] = math.transform(transform, new float3(min.x, min.y, min.z));
        corners[1] = math.transform(transform, new float3(min.x, min.y, max.z));
        corners[2] = math.transform(transform, new float3(min.x, max.y, min.z));
        corners[3] = math.transform(transform, new float3(min.x, max.y, max.z));
        corners[4] = math.transform(transform, new float3(max.x, min.y, min.z));
        corners[5] = math.transform(transform, new float3(max.x, min.y, max.z));
        corners[6] = math.transform(transform, new float3(max.x, max.y, min.z));
        corners[7] = math.transform(transform, new float3(max.x, max.y, max.z));
    }

    public static CameraInfo ProjectToCameraRender(in float3 center, in float3 lookDir, in NativeSlice<float3> corners)
    {
        // first pass to compute camera position and its view matrix
        var nearFar = new float2(math.INFINITY, -math.INFINITY);

        float3 dir = math.normalize(lookDir);
        foreach (float3 corner in corners)
        {
            float depth = math.dot(corner, dir);
            nearFar.x = math.min(nearFar.x, depth);
            nearFar.y = math.max(nearFar.y, depth);
        }

        float centerDepth = math.dot(center, dir);
        float extent = 1.1f * math.max(centerDepth - nearFar.x, nearFar.y - centerDepth);
        float3 camPos = center - extent * lookDir;
        var viewMatrix = float4x4.TRS(camPos, quaternion.LookRotation(dir, Vector3.up), cameraScale);
        viewMatrix = math.inverse(viewMatrix);

        // second pass to fit the corners into the camera view and compute the projection matrix
        var min = new float3(math.INFINITY);
        var max = new float3(-math.INFINITY);

        foreach (float3 corner in corners)
        {
            float3 pos = math.transform(viewMatrix, corner);
            min = math.min(min, pos);
            max = math.max(max, pos);
        }

        // slightly expand the bounds so that no pixel falls on the boundary
        max += 0.1f;
        min -= 0.1f;

        // take care of inverted depth
        if (cameraScale.z < 0)
            (min.z, max.z) = (-max.z, -min.z);

        float4x4 projectionMatrix =
            GL.GetGPUProjectionMatrix(Matrix4x4.Ortho(min.x, max.x, min.y, max.y, min.z, max.z), true);

        float3 size = max - min;

        return new CameraInfo
        {
            forward = lookDir,
            projectedArea = (double)size.x * size.y,
            position = camPos,
            centerDistance = extent,
            vpMatrix = math.mul(projectionMatrix, viewMatrix),
        };
    }

    public static JobHandle CountPixels(NativeSlice<uint> texture, NativeSlice<int> pixelCounts)
    {
        // TODO: use multiple sequential counter jobs working on texture slices and separate count arrays since
        // atomic increment on shared cache lines is slow
        var job = new PixelCountJob
        {
            pixels = pixelCounts,
            texture = texture,
        };
        return job.Schedule(texture.Length, 256);
    }
}

public class Renderer<T> : IDisposable, IReadOnlyDictionary<T, int> where T : Object
{
    // ReSharper disable once StaticMemberInGenericType
    private static readonly Pool<HashSet<UnityEngine.Renderer>> renderersPool =
        new(() => new HashSet<UnityEngine.Renderer>(ObjectReferenceEqualityComparer<UnityEngine.Renderer>.Default),
            set => set.Clear());

    // rendered object cache
    private readonly Dictionary<T, int> objects = new(ObjectReferenceEqualityComparer<T>.Default);
    private readonly List<HashSet<UnityEngine.Renderer>> renderers = new();

    // currently executing jobs
    private readonly HashSet<RenderBatch> activeBatches = new(ObjectReferenceEqualityComparer<RenderBatch>.Default);

    // pool of inactive jobs, reusing old ones to reduce the overhead of instantiating Unity resources and GC allocations
    private Pool<RenderBatch> batchPool;

    // material used to render the objects
    private Material material;

    public Material Material
    {
        get { return material; }
        set
        {
            if (ReferenceEquals(material, value))
                return;
            material = value;
            ForEachJob(value, (job, mat) => job.Material = mat);
        }
    }

    // number of requests executed last time
    public int ActiveBatches
    {
        get { return activeBatches.Count; }
    }

    // temporary MPB to avoid GC
    private MaterialPropertyBlock mpb;

    // texture size, greater size -> better resolution
    private int2 renderSize = 512;

    public int2 RenderSize
    {
        get { return renderSize; }
        set
        {
            if (math.all(renderSize == value))
                return;
            renderSize = value;
            ForEachJob(value, (job, v) => job.RenderSize = v);
        }
    }

    // compute shader to count pixels
    private ComputeShader pixelCountShader;

    public ComputeShader PixelCountShader
    {
        get { return pixelCountShader; }
        set
        {
            if (ReferenceEquals(pixelCountShader, value))
                return;
            pixelCountShader = value;
            ForEachJob(value, (job, v) => job.PixelCountShader = v);
        }
    }

    private Kernel pixelCountKernel;

    public Kernel PixelCountKernel
    {
        get { return pixelCountKernel; }
        set
        {
            pixelCountKernel = value;
            ForEachJob(value, (job, v) => job.PixelCountKernel = v);
        }
    }

    // rendering volume
    public Bounds Bounds { get; set; }

    // flag for any changes in objects/their renderers
    private bool isDirty = true;

    // request device to count pixels on
    private PhysicalDevice device = PhysicalDevice.GPU;

    public PhysicalDevice Device
    {
        get { return device; }
        set
        {
            device = value;
            ActualDevice = value.Select();
            ForEachJob(ActualDevice, (job, v) => job.Device = v);
        }
    }

    // actual device pixels are counted on
    public PhysicalDevice ActualDevice { get; private set; }

    public float AspectRatio
    {
        get { return (float)renderSize.x / renderSize.y; }
    }

    public Renderer()
    {
        // circular reference in closure, clean up in Dispose
        batchPool = new Pool<RenderBatch>(() => new RenderBatch
                                          {
                                              Device = ActualDevice,
                                              Material = Material,
                                              PixelCountKernel = PixelCountKernel,
                                              PixelCountShader = PixelCountShader,
                                              RenderSize = RenderSize
                                          },
                                          null);
    }

    /// <summary>
    /// Mark CommandBuffer for reconstruction on the next request
    /// </summary>
    public void ReconstructCommandBuffers()
    {
        ForEachJob(true, (job, _) => job.ReconstructCommandBuffer());
    }

    public Dictionary<T, int>.KeyCollection Keys
    {
        get { return objects.Keys; }
    }

    public Dictionary<T, int>.ValueCollection Values
    {
        get { return objects.Values; }
    }

    public Dictionary<T, int>.Enumerator GetEnumerator()
    {
        return objects.GetEnumerator();
    }

    IEnumerator<KeyValuePair<T, int>> IEnumerable<KeyValuePair<T, int>>.GetEnumerator()
    {
        return objects.GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return ((IEnumerable)objects).GetEnumerator();
    }

    public int Count
    {
        get { return objects.Count; }
    }

    public bool ContainsKey(T key)
    {
        return objects.ContainsKey(key);
    }

    public bool TryGetValue(T key, out int value)
    {
        return objects.TryGetValue(key, out value);
    }

    public int this[T key]
    {
        get { return objects[key]; }
    }

    IEnumerable<T> IReadOnlyDictionary<T, int>.Keys
    {
        get { return objects.Keys; }
    }

    IEnumerable<int> IReadOnlyDictionary<T, int>.Values
    {
        get { return objects.Values; }
    }

    private void ReleaseUnmanagedResources()
    {
        // cancel all jobs first before waiting
        CancelPendingJobs();

        batchPool.Dispose();

        // need to wait before releasing resources
        foreach (RenderBatch job in activeBatches)
            job.Dispose();
        activeBatches.Clear();
    }

    private void Dispose(bool disposing)
    {
        ReleaseUnmanagedResources();
        if (disposing)
        {
            batchPool = null;
        }
    }

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    // coverage: finalizer: runs only at GC time when Dispose was missed
    [ExcludeFromCodeCoverage]
    ~Renderer()
    {
        Dispose(false);
    }

    private void ForEachJob<V>(V value, Action<RenderBatch, V> func)
    {
        foreach (RenderBatch job in activeBatches)
            func(job, value);
        foreach (RenderBatch job in batchPool)
            func(job, value);
    }

    public Color ColorOf(T obj)
    {
        return ColorUintConverter.AsColor((uint)objects[obj]);
    }

    private Color GetObjColor(T obj)
    {
        // unique per object, stability doesn't matter as renderers will have to be rebuilt when anything changes
        if (!objects.TryGetValue(obj, out int index))
        {
            index = objects.Count;
            renderers.Add(renderersPool.Acquire());
            objects.Add(obj, index);
        }

        return ColorUintConverter.AsColor((uint)index);
    }

    public void CancelPendingJobs()
    {
        foreach (RenderBatch job in activeBatches)
            job.Apply(h => h.Cancel());
    }

    public Color SetupRenderers<R>(T obj, R objRenderers, MaterialPropertyBlock propertyBlock = null)
        where R : IEnumerable<UnityEngine.Renderer>
    {
        Color color = GetObjColor(obj);
        int index = objects[obj];
        foreach (UnityEngine.Renderer renderer in objRenderers)
        {
            SetupRenderer(renderer, color, propertyBlock);
            renderers[index].Add(renderer);
        }

        return color;
    }

    public Color SetupRenderer(T obj, UnityEngine.Renderer renderer, MaterialPropertyBlock propertyBlock = null)
    {
        Color color = GetObjColor(obj);
        SetupRenderer(renderer, color, propertyBlock);
        renderers[objects[obj]].Add(renderer);
        isDirty = true;
        return color;
    }

    private void SetupRenderer(UnityEngine.Renderer renderer, Color color, MaterialPropertyBlock propertyBlock = null)
    {
        if (propertyBlock == null)
        {
            mpb ??= new MaterialPropertyBlock();
            renderer.GetPropertyBlock(mpb); // always overwrites block in any case
            propertyBlock = mpb;
        }

        propertyBlock.SetColor(ShaderPropertyIds._ExposedColor, color);
        renderer.SetPropertyBlock(propertyBlock);
    }

    public void Reset()
    {
        foreach (HashSet<UnityEngine.Renderer> set in renderers)
            renderersPool.Release(set);
        // the sets are the pool's again (shared by every renderer): kept here, new objects' sets would go after them
        renderers.Clear();
        objects.Clear();
        isDirty = true;
    }

    public void Render<R>(R requests, in float4x4 transform) where R : IReadOnlyList<RenderRequest>
    {
        if (isDirty)
        {
            ReconstructCommandBuffers();
            isDirty = false;
        }

        RenderBatch batch = batchPool.Acquire();
        if (!batch.Execute(renderers, requests, Bounds, transform, onRenderBatchCompletedAction, this))
        {
            StringBuilder sb =
                BuildRendererMessage("There were errors in setting up exposure computations, renderers in use:");

            FARLogger.Warning(sb.ToString());
        }

        activeBatches.Add(batch);
    }

    public StringBuilder BuildRendererMessage(string message, StringBuilder sb = null)
    {
        sb ??= new StringBuilder(1024);
        if (!string.IsNullOrEmpty(message))
            sb.AppendLine(message);
        for (int i = 0; i < renderers.Count; ++i)
            sb.Append(i).Append(": ").AppendLine(string.Join(", ", renderers[i]));

        sb.AppendLine("With objects:");
        foreach (KeyValuePair<T, int> pair in objects)
            sb.Append(pair.Value).Append(": ").AppendLine(pair.Key.ToString());

        return sb;
    }

    // convert only once to avoid GC allocations
    // ReSharper disable once StaticMemberInGenericType
    private static readonly Action<RenderBatch, object> onRenderBatchCompletedAction = OnRenderBatchCompleted;

    private static void OnRenderBatchCompleted(RenderBatch batch, object o)
    {
        var self = o as Renderer<T>;
        FARLogger.Assert(self != null, nameof(self) + " != null");

        // ReSharper disable once PossibleNullReferenceException
        self.activeBatches.Remove(batch);
        self.batchPool.Release(batch);
    }
}
