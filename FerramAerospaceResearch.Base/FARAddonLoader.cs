using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using FerramAerospaceResearch.Interfaces;
using FerramAerospaceResearch.Reflection;
using UnityEngine;

namespace FerramAerospaceResearch
{
    /// <summary>
    /// Class to handle instantiation of marked addons, specific implementation needs some entry point to start this
    /// </summary>
    public class FARAddonLoader : MonoSingleton<FARAddonLoader>
    {
        /// <summary>
        /// Dictionary of all instantiated (addons, reloadable) objects
        /// </summary>
        private readonly Dictionary<Type, object> instantiatedTypes = new Dictionary<Type, object>();

        /// <summary>
        /// Dictionary of found addons and their corresponding types
        /// </summary>
        public List<Pair<FARAddonAttribute, Type>> AddonTypes { get; } = new List<Pair<FARAddonAttribute, Type>>();

        /// <summary>
        /// Dictionary of types found that implement <see cref="IReloadable"/> interface/>
        /// </summary>
        public List<Type> ReloadableTypes { get; } = new List<Type>();

        /// <summary>
        /// List of persistant addons
        /// </summary>
        public List<object> AddonObjects { get; } = new List<object>();

        /// <summary>
        /// List of all instantiated objects that implement <see cref="IReloadable"/> interface/>
        /// </summary>
        public List<IReloadable> ReloadableObjects { get; } = new List<IReloadable>();

        /// <summary>
        /// Start the loading
        /// </summary>
        /// <param name="callback">callback function after loading has successfully finished</param>
        public void Load(Action callback = null)
        {
            StartCoroutine(DoLoad(callback));
        }

        private void InitTask()
        {
            Assembly[] assemblies = ReflectionUtils.LoadedAssemblies;
            var types = new List<Type>(ReflectionUtils.GetTypes(assemblies));

            AddonTypes.AddRange(ReflectionUtils.FindAttribute<FARAddonAttribute>(types));
            ReloadableTypes.AddRange(ReflectionUtils.FindTypes<IReloadable>(types, true));

            if (AddonTypes.Count == 0)
                FARLogger.Trace("No FARAddon types found");
            if (ReloadableTypes.Count == 0)
                FARLogger.Trace("No IReloadable types found");

            // sort by descending priority order
            AddonTypes.Sort((x, y) => y.First.Priority.CompareTo(x.First.Priority));

            // output some debug information
            FARLogger.TraceFormat(TraceMessage());
        }

        // coverage: only evaluated as a FARLogger.TraceFormat argument, [Conditional("LOG_TRACE")]
        [ExcludeFromCodeCoverage]
        private string TraceMessage()
        {
            var sb = new StringBuilder("FARAddonLoader found addons: \n");
            foreach (Pair<FARAddonAttribute, Type> pair in AddonTypes)
                // ReSharper disable once UseFormatSpecifierInInterpolation
                sb.AppendLine($"  {pair.First.Priority.ToString("D4")}: {pair.Second.Name}");
            sb.AppendLine("IReloadable types found:");
            foreach (Type type in ReloadableTypes)
                sb.AppendLine($"  {type.Name}");
            return sb.ToString();
        }

        private IEnumerator DoLoad(Action callback)
        {
#pragma warning disable CS0162
            // ReSharper disable once ConditionIsAlwaysTrueOrFalse, updated from CI
            if (Version.GitSha != null)
                FARLogger.Info($"FerramAerospaceResearch CI build from commit {Version.GitSha}");
#pragma warning restore CS0162

            // offload to another thread and wait for it to complete
            Task task = Task.Factory.StartNew(InitTask);
            yield return new WaitUntil(() => task.IsCompleted);

            if (task.Exception != null)
            {
                FARLogger.Exception(task.Exception, "Exception while loading FAR addons:");
                yield break;
            }

            // do instantiation in the main thread in case any of the types are Unity objects
            FARLogger.Debug("Instantiating FAR addons");
            foreach (Pair<FARAddonAttribute, Type> pair in AddonTypes)
            {
                yield return SetupType(pair.Second, pair.First.Persistant, AddonObjects);
                yield return null;
            }

            FARLogger.Debug("Instantiating Reloadable types");
            foreach (Type type in ReloadableTypes)
            {
                yield return SetupType(type, true, ReloadableObjects);
                yield return null;
            }

            callback?.Invoke();
        }

        private IEnumerator SetupType<T>(Type type, bool persistant, List<T> objects) where T : class
        {
            // skip invalid types
            if (type.IsAbstract || type.IsInterface)
                yield break;

            FARLogger.DebugFormat("FARAddonLoader: instantiating {0}", type);

            bool contains = true;

            // if not yet instantiated, try to find singleton instance or create a new one
            if (!instantiatedTypes.TryGetValue(type, out object o))
            {
                o = ReflectionUtils.FindInstance(type);
                if (o == null)
                    o = typeof(Component).IsBaseOf(type)
                            ? ReflectionUtils.Create(type, transform, persistant)
                            : Activator.CreateInstance(type, true);
                else
                    FARLogger.DebugFormat("Found an instance of {0}", type);
                contains = false;
            }

            // enable behaviour so that their Awake methods run
            if (o is Behaviour behaviour && behaviour != null)
                behaviour.enabled = true;

            // wait for the addon to finish its setup
            if (o is IWaitForAddon waitForAddon)
                yield return new WaitUntil(() => waitForAddon.Completed);

            // store persistant objects
            if (persistant)
            {
                objects.Add(o as T);
                if (!contains)
                    instantiatedTypes.Add(type, o);
            }
            else if (o is Component component)
            {
                Destroy(component);
            }

            FARLogger.DebugFormat("FARAddonLoader: {0} finished", type);
        }

        public IEnumerator Reload()
        {
            FARLogger.Debug("Reloading IReloadable objects");

            // sort again in case priorities have changed
            ReloadableObjects.Sort((x, y) => y.Priority.CompareTo(x.Priority));

            foreach (IReloadable reloadable in ReloadableObjects)
            {
                reloadable.Completed = false;
                reloadable.DoReload();
                yield return new WaitUntil(() => reloadable.Completed);
            }
        }
    }
}
