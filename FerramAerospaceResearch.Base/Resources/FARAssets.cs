using System.Collections;
using FerramAerospaceResearch.Interfaces;
using FerramAerospaceResearch.Resources.Loading;
using FerramAerospaceResearch.Threading;
using UnityEngine;

namespace FerramAerospaceResearch.Resources
{
    [FARAddon(0, true)]
    public class FARAssets : MonoSingleton<FARAssets>, IReloadable
    {
        private bool loaded;
        public FARShaderCache Shaders { get; private set; }
        public FARComputeShaderCache ComputeShaders { get; private set; }
        public FARTextureCache Textures { get; private set; }
        public LoaderCache Loaders { get; private set; }

        public ShaderBundleLoader ShaderLoader { get; private set; }
        public ComputeShaderBundleLoader ComputeShaderLoader { get; private set; }

        public int Priority { get; set; }
        public bool Completed { get; set; }

        public void DoReload()
        {
            if (!loaded)
                StartCoroutine(LoadAssets());
            else
                Completed = true;
        }

        protected override void OnAwake()
        {
            // setup loader caches and add loaders
            Loaders = new LoaderCache();
            Loaders.Textures.Add(new TextureLoader());
            Loaders.Shaders.Add(new ShaderLoader());
            Loaders.ComputeShaders.Add(new ComputeShaderLoader());

            // setup bundle loaders
            if (!Application.isEditor)
            {
                // ignore asset bundles in editor
                ComputeShaderLoader = new ComputeShaderBundleLoader();
                ShaderLoader = new ShaderBundleLoader();
                Loaders.Shaders.Add(new AssetBundleAssetLoader<Shader> { BundleLoader = ShaderLoader });
                Loaders.ComputeShaders.Add(new AssetBundleAssetLoader<ComputeShader>
                {
                    BundleLoader = ComputeShaderLoader
                });
            }

            // setup asset caches
            Shaders = new FARShaderCache();
            Textures = new FARTextureCache();
            ComputeShaders = new FARComputeShaderCache();
        }

        protected override void OnDestruct()
        {
            Textures.Unsubscribe();
            Shaders.Unsubscribe();

            if (Application.isEditor)
                return;
            ShaderLoader.Unsubscribe();
            ComputeShaderLoader.Unsubscribe();
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public void ReloadAssets()
        // {
        //     MainThread.StartCoroutine(DoReloadAssets);
        // }

        private IEnumerator LoadAssets()
        {
            FARLogger.Debug("Loading all assets");
            if (ShaderLoader.NeedsReload)
                yield return ShaderLoader.Load();

            while (ShaderLoader.State == Progress.InProgress)
                yield return null;

            yield return DoReloadAssets();

            Completed = true;
            loaded = true;
        }

        private IEnumerator DoReloadAssets()
        {
            yield return Shaders.Load();
            yield return ComputeShaders.Load();
            yield return Textures.Load();
        }
    }
}
