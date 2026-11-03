using ferram4;
using FerramAerospaceResearch.FARPartGeometry;
using TweakScale;

namespace FerramAerospaceResearch.TweakScale
{
    // TweakScale discovers IRescalable<T> implementations by reflection and constructs one per module of type T
    // (or a subclass of T) on each scaled part.

    public class GeometryPartModuleRescaler : IRescalable<GeometryPartModule>
    {
        private readonly GeometryPartModule module;

        public GeometryPartModuleRescaler(GeometryPartModule module)
        {
            this.module = module;
        }

        public void OnRescale(ScalingFactor factor)
        {
            module.OnRescale(factor.absolute.linear);
        }
    }

    public class FARWingAerodynamicModelRescaler : IRescalable<FARWingAerodynamicModel>
    {
        private readonly FARWingAerodynamicModel module;

        public FARWingAerodynamicModelRescaler(FARWingAerodynamicModel module)
        {
            this.module = module;
        }

        public void OnRescale(ScalingFactor factor)
        {
            module.OnRescale(factor.absolute.linear);
        }
    }
}
