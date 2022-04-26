using FerramAerospaceResearch.Reflection;

namespace FerramAerospaceResearch.Settings
{
    [ConfigNode("Occlusion", shouldSave: false, parent: typeof(FARConfig))]
    public static class OcclusionSettings
    {
        [ConfigValue("useRaycaster")] public static bool UseRaycaster { get; set; } = false;
        [ConfigValue("jobsPerThread")] public static int JobsPerThread { get; set; } = 3;
        [ConfigValue("maxJobs")] public static int MaxJobs { get; set; } = 10;
        [ConfigValue("fibonacciLatticeSize")] public static int FibonacciLatticeSize { get; set; } = 2000;
        // 1000 points produces typical misses 2-3 degrees when optimized for average miss distance.
        [ConfigValue("maxRaycastDimension")] public static int MaxRaycastDimension { get; set; } = 100;
        [ConfigValue("raycastResolution")] public static float RaycastResolution { get; set; } = 0.1f;
        [ConfigValue("resetInterval")] public static float ResetInterval { get; set; } = 180;
        [ConfigValue("velocityThreshold")] public static float VelocityThreshold { get; set; } = 0.01f;
    }
}
