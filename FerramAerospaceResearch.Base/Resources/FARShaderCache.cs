using FerramAerospaceResearch.Config;
using UnityEngine;

namespace FerramAerospaceResearch.Resources
{
    public class FARShaderCache : FARAssetDictionary<Shader>
    {
        public FARShaderCache()
        {
            LineRenderer = MakeAsset(FARConfig.Shaders.LineRenderer, "line_renderer");
            DebugVoxel = MakeAsset(FARConfig.Shaders.DebugVoxel, "debug_voxel");
            DebugVoxelFallback = MakeAsset(FARConfig.Shaders.DebugVoxelFallback, "debug_voxel_fallback");
            ExposedSurface = MakeAsset(FARConfig.Shaders.ExposedSurface, "exposed_surface");
            ExposedSurfaceDebug = MakeAsset(FARConfig.Shaders.ExposedSurfaceDebug, "exposed_surface_debug");
            ExposedSurfaceCamera = MakeAsset(FARConfig.Shaders.ExposedSurfaceCamera, "exposed_surface_camera");
        }

        public ShaderAssetRequest LineRenderer { get; }
        public ShaderAssetRequest DebugVoxel { get; }
        public ShaderAssetRequest DebugVoxelFallback { get; }
        public ShaderAssetRequest ExposedSurface { get; }
        public ShaderAssetRequest ExposedSurfaceDebug { get; }
        public ShaderAssetRequest ExposedSurfaceCamera { get; }

        private ShaderAssetRequest MakeAsset(ResourceNode node, string name)
        {
            var asset = new ShaderAssetRequest
            {
                AssetLoaders = FARAssets.Instance.Loaders.Shaders,
                Key = name,
                Node = node
            };
            SetupAsset(asset);
            return asset;
        }

        public class ShaderAssetRequest : LoadableAsset<Shader>
        {
            private ShaderMaterialPair Items { get; } = new();

            // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
            // public bool IsSupported
            // {
            //     get { return Asset != null && Asset.isSupported; }
            // }

            public Material Material
            {
                get { return Items.Material; }
            }

            /// <inheritdoc />
            protected override void AssetLoaded()
            {
                base.AssetLoaded();
                Items.Shader = Asset;
            }


            public static implicit operator ShaderMaterialPair(ShaderAssetRequest rb)
            {
                return rb.Items;
            }

            public static implicit operator Shader(ShaderAssetRequest rb)
            {
                return rb.Items.Shader;
            }

            public static implicit operator Material(ShaderAssetRequest rb)
            {
                return rb.Items.Material;
            }
        }

        public class ShaderMaterialPair
        {
            private Shader shader;

            public Shader Shader
            {
                get { return shader; }
                set
                {
                    shader = value;
                    if (value == null)
                        return;
                    if (Material == null)
                        Material = new Material(value);
                    else
                        Material.shader = value;
                }
            }

            public Material Material { get; private set; }

            public static implicit operator Material(ShaderMaterialPair sm)
            {
                return sm.Material;
            }

            public static implicit operator Shader(ShaderMaterialPair sm)
            {
                return sm.shader;
            }
        }
    }
}
