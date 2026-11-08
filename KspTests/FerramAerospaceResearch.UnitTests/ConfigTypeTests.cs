using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using FerramAerospaceResearch.Config;
using FerramAerospaceResearch.Settings;
using UnityEngine;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// The config types' members FAR does not use yet: a resource node as its url, a color map's indexer, the voxel
    /// config's default color map, and the save hooks of the animation overrides and the transform exceptions, which
    /// turn the lookups FAR uses back into the lists their nodes load from.
    /// </summary>
    [Collection(TestLog.Collection)]
    public class ConfigTypeTests
    {
        private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;

        [Fact]
        public void AResourceNodeConvertsToItsUrl()
        {
            string url = new TextureNode("FerramAerospaceResearch/Textures/icon", "gamedata");
            Assert.Equal("FerramAerospaceResearch/Textures/icon", url);

            var node = new ShaderNode();
            node.Url.Value = "changed";
            string changed = node;
            Assert.Equal("changed", changed);
        }

        /// <summary>The indexer reads and writes the color list directly: unlike Get it does not wrap around.</summary>
        [SkippableFact]
        public void ColorMapIndexerReadsAndWritesItsColors()
        {
            UnityRuntime.Require();   // Color
            var map = new ColorMap { Name = "test", Colors = { Color.red, Color.green } };
            Assert.Equal(Color.green, map[1]);
            map[0] = Color.blue;
            Assert.Equal(Color.blue, map.Colors[0]);
            Assert.Equal(Color.blue, map.Get(2));
            Assert.Throws<ArgumentOutOfRangeException>(() => map[2]);
        }

        /// <summary>FirstOrDefault() is the map the config's default names, else the built-in default map.</summary>
        [SkippableFact]
        public void VoxelConfigFirstOrDefaultIsTheNamedDefault()
        {
            UnityRuntime.Require();   // ColorMap.Default's color
            var a = new ColorMap { Name = "a" };
            var b = new ColorMap { Name = "b" };
            var config = new VoxelizationConfig { Default = "b", ColorMaps = { a, b } };
            Assert.Same(b, config.FirstOrDefault());

            config.Default = "missing";
            Assert.Same(ColorMap.Default, config.FirstOrDefault());
            config.Default = null;
            Assert.Same(ColorMap.Default, config.FirstOrDefault());
        }

        /// <summary>
        /// FARAnimOverrides' BeforeSaved writes the module-to-field lookup back into the list its node loads, so loading
        /// that list again (AfterLoaded) gives the same lookup; AfterSaved changes nothing.
        /// </summary>
        [SkippableFact]
        public void AnimOverridesSaveTheirLookupAsTheirList()
        {
            UnityRuntime.RequireLog();   // the singleton logs that it was created
            var list = (List<FARAnimOverrides.Override>)typeof(FARAnimOverrides).GetField("overrideList", PrivateStatic).GetValue(null);
            var savedLookup = new Dictionary<string, string>(FARAnimOverrides.Overrides);
            var savedList = list.ToList();
            try
            {
                FARAnimOverrides.Overrides.Clear();
                FARAnimOverrides.Overrides.Add("ModuleA", "animA");
                FARAnimOverrides.Overrides.Add("ModuleB", "animB");

                FARAnimOverrides.Instance.BeforeSaved();
                Assert.Equal(new[] { ("ModuleA", "animA"), ("ModuleB", "animB") },
                             list.Select(o => (o.ModuleName, o.FieldName)).OrderBy(p => p.ModuleName));

                FARAnimOverrides.Instance.AfterSaved();
                Assert.Equal(2, list.Count);
                Assert.Equal(2, FARAnimOverrides.Overrides.Count);

                FARAnimOverrides.Overrides.Clear();
                FARAnimOverrides.Instance.AfterLoaded();
                Assert.Equal("animA", FARAnimOverrides.FieldNameForModule("ModuleA"));
                Assert.Equal("animB", FARAnimOverrides.FieldNameForModule("ModuleB"));
            }
            finally
            {
                FARAnimOverrides.Overrides.Clear();
                foreach (KeyValuePair<string, string> pair in savedLookup)
                    FARAnimOverrides.Overrides.Add(pair.Key, pair.Value);
                list.Clear();
                list.AddRange(savedList);
            }
        }

        /// <summary>
        /// FARPartModuleTransformExceptions' BeforeSaved writes the exceptions it looks up by module back into its list,
        /// which loads (AfterLoaded) back into the same lookup; AfterSaved changes nothing.
        /// </summary>
        [SkippableFact]
        public void TransformExceptionsSaveTheirLookupAsTheirList()
        {
            UnityRuntime.RequireLog();
            Dictionary<string, FARPartModuleTransformExceptions.FARPartModuleException> lookup =
                FARPartModuleTransformExceptions.Exceptions;
            var list = (List<FARPartModuleTransformExceptions.FARPartModuleException>)typeof(FARPartModuleTransformExceptions)
                                                                                      .GetField("exceptions", PrivateStatic)
                                                                                      .GetValue(null);
            var savedLookup = lookup.ToList();
            var savedList = list.ToList();
            try
            {
                var intake = new FARPartModuleTransformExceptions.FARPartModuleException
                {
                    PartModuleName = "ModuleResourceIntake",
                    Transforms = { "intakeTransformName" }
                };
                var engine = new FARPartModuleTransformExceptions.FARPartModuleException { PartModuleName = "ModuleEngines" };
                lookup.Clear();
                lookup.Add(intake.PartModuleName, intake);
                lookup.Add(engine.PartModuleName, engine);

                FARPartModuleTransformExceptions.Instance.BeforeSaved();
                Assert.Equal(new[] { engine, intake }, list.OrderBy(e => e.PartModuleName));

                FARPartModuleTransformExceptions.Instance.AfterSaved();
                Assert.Equal(2, list.Count);
                Assert.Equal(2, lookup.Count);

                lookup.Clear();
                FARPartModuleTransformExceptions.Instance.AfterLoaded();
                Assert.Same(intake, lookup["ModuleResourceIntake"]);
                Assert.Same(engine, lookup["ModuleEngines"]);
                Assert.Equal(new[] { "intakeTransformName" }, lookup["ModuleResourceIntake"].Transforms);
            }
            finally
            {
                lookup.Clear();
                foreach (KeyValuePair<string, FARPartModuleTransformExceptions.FARPartModuleException> pair in savedLookup)
                    lookup.Add(pair.Key, pair.Value);
                list.Clear();
                list.AddRange(savedList);
            }
        }
    }
}
