using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using FerramAerospaceResearch.Resources.Loading;
using UnityEngine;

namespace FerramAerospaceResearch.Resources
{
    public class FARAssetDictionary<T> : Dictionary<string, T>
    {
        private readonly List<LoadableAsset<T>> requests = new List<LoadableAsset<T>>();

        public bool Contains(string name)
        {
            return ContainsKey(name);
        }

        public void Unsubscribe()
        {
            foreach (LoadableAsset<T> request in requests)
            {
                request.OnAssetLoaded -= OnAssetLoaded;
                request.OnAssetLoadError -= OnAssetError;
            }
        }

        public IEnumerator Load()
        {
            foreach (LoadableAsset<T> request in requests)
            {
                request.Load();
            }

            foreach (LoadableAsset<T> request in requests)
            {
                if (FinishedLoading(request))
                    continue;
                yield return new WaitUntil(() => FinishedLoading(request));
            }
        }

        private static bool FinishedLoading(IAssetRequest request)
        {
            return request.State == Progress.Completed || request.State == Progress.Error;
        }

        protected void SetupAsset(LoadableAsset<T> asset)
        {
            Add(asset.Key, asset.Asset);
            requests.Add(asset);

            asset.OnAssetLoaded += OnAssetLoaded;
            asset.OnAssetLoadError += OnAssetError;
        }

        private void OnAssetLoaded(string key, AssetRequest<T> request)
        {
            this[key] = request.Asset;
        }

        // coverage: empty handler for a load error the tests cannot cause
        [ExcludeFromCodeCoverage]
        private static void OnAssetError(string key, AssetRequest<T> request)
        {
        }
    }
}
