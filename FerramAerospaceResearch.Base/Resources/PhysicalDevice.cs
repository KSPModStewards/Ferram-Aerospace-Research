using System;
using System.Diagnostics.CodeAnalysis;
using JetBrains.Annotations;
using UnityEngine;

namespace FerramAerospaceResearch.Resources;

public enum PhysicalDevice
{
    CPU,
    GPU,
}

public static class PhysicalDeviceExtensions
{
    private static readonly bool supportsComputeShaders;
    private static bool computeWarningIssued;

    static PhysicalDeviceExtensions()
    {
        supportsComputeShaders = SystemInfo.supportsComputeShaders;
    }

    public static PhysicalDevice Select(this Device device)
    {
        switch (device)
        {
            case Device.PreferGPU:
                return supportsComputeShaders ? PhysicalDevice.GPU : PhysicalDevice.CPU;
            case Device.CPU:
                return PhysicalDevice.CPU;
            case Device.GPU:
                return PhysicalDevice.GPU;
            case Device.None:
            default:
                throw new ArgumentOutOfRangeException(nameof(device), device, null);
        }
    }

    public static void Select<T>(this Device device, [NotNull] Action<PhysicalDevice, T> onSelect, T data)
    {
        if (onSelect is null)
            throw new ArgumentNullException(nameof(onSelect));

        switch (device)
        {
            case Device.PreferGPU:
                onSelect(supportsComputeShaders ? PhysicalDevice.GPU : PhysicalDevice.CPU, data);
                return;
            case Device.CPU:
                onSelect(PhysicalDevice.CPU, data);
                return;
            case Device.GPU:
                onSelect(PhysicalDevice.GPU, data);
                return;
            case Device.None:
                return;
            default:
                throw new ArgumentOutOfRangeException(nameof(device), device, null);
        }
    }

    public static PhysicalDevice Select(this PhysicalDevice device)
    {
        if (device is not PhysicalDevice.GPU || supportsComputeShaders)
            return device;

        if (computeWarningIssued)
            return PhysicalDevice.CPU;

        FARLogger.Warning("Compute shaders are not supported on your system!");
        computeWarningIssued = true;

        return PhysicalDevice.CPU;
    }
}
