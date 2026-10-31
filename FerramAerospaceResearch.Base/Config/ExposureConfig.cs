using FerramAerospaceResearch.Reflection;
using FerramAerospaceResearch.Resources;
using UnityEngine;

namespace FerramAerospaceResearch.Config;

[ConfigNode("Exposure")]
public class ExposureConfig
{
    [ConfigValue("airstream")] public Observable<bool> Airstream { get; } = new(true);
    [ConfigValue("sun")] public Observable<bool> Sun { get; } = new(true);
    [ConfigValue("body")] public Observable<bool> Body { get; } = new(true);
    [ConfigValue("device")] public Observable<Device> Device { get; } = new(Resources.Device.None);
    [ConfigValue("debugBackgroundColor")] public Observable<Color> DebugBackgroundColor { get; } = new(Color.black);
    [ConfigValue("width")] public Observable<int> Width { get; } = new(512);
    [ConfigValue("height")] public Observable<int> Height { get; } = new(512);
}
