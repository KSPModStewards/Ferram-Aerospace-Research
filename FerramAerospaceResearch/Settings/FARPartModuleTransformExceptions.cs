using System.Collections.Generic;
using System.Linq;
using FerramAerospaceResearch.Reflection;
using UnityEngine;

// ReSharper disable ClassNeverInstantiated.Global
// ReSharper disable UnassignedField.Global
// ReSharper disable UnusedMember.Global
// ReSharper disable CollectionNeverUpdated.Global

namespace FerramAerospaceResearch.Settings
{
    [ConfigNode("FARPartModuleTransformExceptions", true, shouldSave: false)]
    public class FARPartModuleTransformExceptions : Singleton<FARPartModuleTransformExceptions>, Interfaces.IConfigNode
    {
        [ConfigValue]
        private static readonly List<FARPartModuleException> exceptions = new List<FARPartModuleException>();

        [ConfigValueIgnore]
        public static readonly Dictionary<string, FARPartModuleException> Exceptions =
            new Dictionary<string, FARPartModuleException>();

        public void BeforeLoaded()
        {
        }

        public void AfterLoaded()
        {
            Exceptions.Clear();
            foreach (FARPartModuleException exception in exceptions)
                if (!string.IsNullOrEmpty(exception.PartModuleName))
                    Exceptions.Add(exception.PartModuleName, exception);
        }

        public void BeforeSaved()
        {
            exceptions.Clear();
            exceptions.AddRange(Exceptions.Select(pair => pair.Value));
        }

        public void AfterSaved()
        {
        }

        public static List<Transform> IgnoreModelTransformList(Part p)
        {
            var Transform = new List<Transform>();

            // ReSharper disable once PossibleNullReferenceException
            foreach (KeyValuePair<string, FARPartModuleException> pair in Exceptions)
            {
                PartModule module = p.Modules.GetModule(pair.Value.PartModuleName);
                if (module is null)
                    continue;

                // from 0: the module name has its own field now, it no longer leads the list
                foreach (string name in pair.Value.Transforms)
                {
                    // a module field holding the transform's name, or the transform's own name
                    string transformString = module.GetType().GetField(name)?.GetValue(module) as string;
                    Transform.AddRange(p.FindModelComponents<Transform>(string.IsNullOrEmpty(transformString)
                                                                           ? name
                                                                           : transformString));
                }
            }

            foreach (Transform t in p.FindModelComponents<Transform>())
            {
                if (Transform.Contains(t))
                    continue;
                if (!t.gameObject.activeInHierarchy)
                {
                    Transform.Add(t);
                    continue;
                }

                string tag = t.tag.ToLowerInvariant();
                if (tag == "ladder" || tag == "airlock")
                    Transform.Add(t);
            }

            return Transform;
        }

        [ConfigNode("FARPartModuleException")]
        public class FARPartModuleException
        {
            [ConfigValue("TransformException")]
            public readonly List<string> Transforms = new List<string>();

            [ConfigValue("PartModuleName")]
            public string PartModuleName;
        }
    }
}
