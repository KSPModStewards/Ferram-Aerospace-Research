using System.Diagnostics.CodeAnalysis;

namespace FerramAerospaceResearch.Resources.Loading
{
    public class AssetRequest<T> : IAssetRequest<T>
    {
        /// <inheritdoc />
        public void OnLoad(T resource)
        {
            FARLogger.DebugFormat("Loaded {0} from {1}", resource, Url);
            Asset = resource;
            AssetLoaded();
        }

        public T Asset { get; private set; }

        /// <inheritdoc />
        public Progress State { get; set; }

        /// <inheritdoc />
        public string Url { get; set; }

        /// <inheritdoc />
        public void OnError()
        {
            FARLogger.ErrorFormat("Failed to load asset from {0}", Url);
            AssetError();
        }

        // coverage: empty base method; every subclass overrides it
        [ExcludeFromCodeCoverage]
        protected virtual void AssetLoaded()
        {
        }

        // coverage: empty base method; every subclass overrides it
        [ExcludeFromCodeCoverage]
        protected virtual void AssetError()
        {
        }

        public static implicit operator T(AssetRequest<T> request)
        {
            return request.Asset;
        }
    }
}
