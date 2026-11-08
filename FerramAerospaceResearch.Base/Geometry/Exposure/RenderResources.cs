using System;
using System.Diagnostics.CodeAnalysis;
using FerramAerospaceResearch.Resources;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using Object = UnityEngine.Object;

// ReSharper disable Unity.NoNullPropagation

namespace FerramAerospaceResearch.Geometry.Exposure;

public class RenderResources : IDisposable
{
    public Material material;
    public ComputeShader computeShader;
    public ComputeBuffer outputBuffer;
    public Kernel mainKernel;

    public Kernel MainKernel
    {
        get { return mainKernel; }
        set
        {
            mainKernel = value;
            if (computeShader is not null && outputBuffer is not null)
                computeShader.SetBuffer(value.index, ShaderPropertyIds.OutputBuffer, outputBuffer);
        }
    }

    public RenderTexture target;
    public NativeArray<uint> hostTexture;
    public NativeArray<int> pixelCounts;

    // native arrays may be larger so store actual sizes
    public int2 ActualTexSize { get; private set; }
    public int ActualCount { get; private set; }

    public NativeArray<uint> ActualTexture
    {
        get { return hostTexture.GetSubArray(0, math.min(hostTexture.Length, ActualTexSize.x * ActualTexSize.y)); }
    }

    public NativeArray<int> ActualPixelCounts
    {
        get { return pixelCounts.GetSubArray(0, ActualCount); }
    }

    public void ReleaseTextures()
    {
        target?.Release();
        target = null;
    }

    public void ReleaseMaterial()
    {
        if (material is null)
            return;
        Object.Destroy(material);
        material = null;
    }

    public void ReleaseComputeShader()
    {
        if (computeShader is null)
            return;
        Object.Destroy(computeShader);
        computeShader = null;
    }

    public void ReleaseNative()
    {
        if (hostTexture.IsCreated)
            hostTexture.Dispose();
        if (pixelCounts.IsCreated)
            pixelCounts.Dispose();
    }

    public void SetComputeShader(ComputeShader shader)
    {
        if (shader is null || computeShader is not null)
            return;
        computeShader = Object.Instantiate(shader);
        if (outputBuffer is not null)
            computeShader.SetBuffer(mainKernel.index, ShaderPropertyIds.OutputBuffer, outputBuffer);
    }

    public void SetMaterial(Material newMaterial)
    {
        material ??= Object.Instantiate(newMaterial);
    }

    private void PrepareComputeBuffer(int count)
    {
        if (outputBuffer is not null && outputBuffer.count >= count)
            return;
        outputBuffer?.Release();
        outputBuffer = new ComputeBuffer(math.max(count * 2, 1024), sizeof(int));
        computeShader?.SetBuffer(mainKernel.index, ShaderPropertyIds.OutputBuffer, outputBuffer);
    }

    public void PrepareForNextJob(int count, bool compute)
    {
        if (!pixelCounts.IsCreated || pixelCounts.Length < count)
        {
            if (pixelCounts.IsCreated)
                pixelCounts.Dispose();
            // should never be big so allocate slightly larger buffer to reduce the number of reallocations needed
            pixelCounts = new NativeArray<int>(count * 2, Allocator.Persistent);
        }
        else
            unsafe
            {
                // faster than reallocation, texture is always overwritten so doesn't need clearing, this one increments its values
                UnsafeUtility.MemClear(pixelCounts.GetUnsafePtr(), count * sizeof(int));
            }

        ActualCount = count;

        if (!compute)
            return;
        PrepareComputeBuffer(count);
        outputBuffer.SetData(pixelCounts, 0, 0, count);
    }

    public void PrepareForNextJob(int2 texSize, int count, bool compute, Material jobMaterial, ComputeShader shader)
    {
        SetMaterial(jobMaterial);
        ActualTexSize = texSize;
        SetupTexture(ref target, texSize);

        if (compute)
            SetComputeShader(shader);
        else if (!hostTexture.IsCreated || hostTexture.Length < texSize.x * texSize.y)
        {
            if (hostTexture.IsCreated)
                hostTexture.Dispose();
            hostTexture = new NativeArray<uint>(texSize.x * texSize.y, Allocator.Persistent);
        }

        PrepareForNextJob(count, compute);
    }

    public void SetupTexture(ref RenderTexture tex)
    {
        SetupTexture(ref tex, ActualTexSize);
    }

    public static void SetupTexture(ref RenderTexture tex, int2 size)
    {
        if (tex is not null && tex.width == size.x && tex.height == size.y)
            return;
        tex?.Release();
        tex = new RenderTexture(size.x, size.y, 24, RenderTextureFormat.Default, RenderTextureReadWrite.Linear)
        {
            antiAliasing = 1,
            filterMode = FilterMode.Point,
            autoGenerateMips = false
        };
    }

    public static void SetupTexture(ref Texture2D tex, int2 size)
    {
        if (tex is not null && tex.width == size.x && tex.height == size.y)
            return;
        if (tex is not null) Object.Destroy(tex);
        tex = new Texture2D(size.x, size.y, TextureFormat.ARGB32, false)
        {
            filterMode = FilterMode.Point,
        };
    }

    private void ReleaseUnmanagedResources()
    {
        ReleaseMaterial();
        ReleaseComputeShader();
        ReleaseNative();
        ReleaseTextures();
    }

    private void Dispose(bool disposing)
    {
        ReleaseUnmanagedResources();
        if (!disposing)
            return;

        outputBuffer?.Dispose();
        outputBuffer = null;
    }

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    // coverage: finalizer: runs only at GC time when Dispose was missed
    [ExcludeFromCodeCoverage]
    ~RenderResources()
    {
        Dispose(false);
    }
}
