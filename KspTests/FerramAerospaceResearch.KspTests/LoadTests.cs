using System.Collections;
using System.Collections.Generic;
using System.Linq;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.Settings;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>What FAR has set up once the game has loaded: its part modules, its stock wing conversions, its atmospheres.</summary>
    public static class LoadTests
    {
        /// <summary>
        /// Every part but a launch clamp gets FAR's geometry and aero modules (_FARPartModule.cfg), and no part keeps a
        /// stock lifting surface beside a FAR wing (its lift would count twice).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 600)]
        public static IEnumerator Load_EveryPartHasFarModules(TestContext ctx)
        {
            var missing = new List<string>();
            var doubled = new List<string>();
            int parts = 0, wings = 0;
            foreach (AvailablePart ap in PartLoader.LoadedPartsList)
            {
                Part p = ap.partPrefab;
                // launch clamps are left out by FAR's patch; EVA kerbals are made by the game, not from a part config
                if (p == null || p.Modules.Contains<LaunchClamp>() || ap.name.StartsWith("kerbalEVA"))
                    continue;
                parts++;
                if (!p.Modules.Contains<GeometryPartModule>() || !p.Modules.Contains<FARAeroPartModule>())
                    missing.Add(ap.name);
                if (p.Modules.Contains<FARWingAerodynamicModel>())
                {
                    wings++;
                    if (p.Modules.Contains<ModuleLiftingSurface>())
                        doubled.Add(ap.name);
                }
            }

            ctx.Record("parts", parts);
            ctx.Record("farWings", wings);
            Assert.True(parts > 100, $"the stock parts loaded ({parts})");
            Assert.True(missing.Count == 0, $"every part has GeometryPartModule and FARAeroPartModule; without: {string.Join(", ", missing.Take(20))}");
            Assert.True(doubled.Count == 0, $"no part has a stock lifting surface beside a FAR wing: {string.Join(", ", doubled.Take(20))}");
            yield break;
        }

        /// <summary>FAR's conversions of stock wings and control surfaces (FerramAerospaceResearch.cfg).</summary>
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 600)]
        public static IEnumerator Load_StockWingsAreFarWings(TestContext ctx)
        {
            // part name -> a control surface (true) or a fixed wing (false)
            var expected = new Dictionary<string, bool>
            {
                ["wingConnector"] = false, ["deltaWing"] = false, ["sweptWing1"] = false, ["wingStrake"] = false,
                ["StandardCtrlSrf"] = true, ["elevon2"] = true,
            };
            foreach (KeyValuePair<string, bool> e in expected)
            {
                AvailablePart ap = PartLoader.getPartInfoByName(e.Key);
                Assert.NotNull(ap, "stock part " + e.Key);
                FARWingAerodynamicModel wing = ap.partPrefab.Modules.GetModule<FARWingAerodynamicModel>();
                Assert.NotNull(wing, e.Key + "'s FAR wing module");
                Assert.Equal(e.Value, wing is FARControllableSurface, e.Key + " is a control surface");
                Assert.True(wing.b_2 > 0 && wing.MAC > 0, $"{e.Key} has a span and chord (b_2 {wing.b_2}, MAC {wing.MAC})");
                Assert.False(ap.partPrefab.Modules.Contains<ModuleLiftingSurface>(), e.Key + " has no stock lifting surface left");
                ctx.Record(e.Key, $"b_2 {wing.b_2}, MAC {wing.MAC}, taper {wing.TaperRatio}, sweep {wing.MidChordSweep}");
            }

            yield break;
        }

        /// <summary>
        /// FARAeroStress.cfg's templates are loaded, and each stock part gets the one its crew and resources call for:
        /// uncrewed and empty, structural; only solid fuel, SRB; only liquid fuel, aircraft tank; liquid fuel and oxidizer,
        /// rocket tank; crewed, crewed; with an ablator, the first (structural) whatever else it has.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 600)]
        public static IEnumerator Load_StressTemplatesForStockParts(TestContext ctx)
        {
            string[] names = FARAeroStress.StressTemplates.Select(t => t.Name).ToArray();
            ctx.Record("templates", string.Join(", ", names));
            foreach (string n in new[] { "structural", "crewed", "rocketFuelTank", "aircraftFuelTank", "SRB", "wingStress", "ctrlSurfStress" })
                Assert.True(names.Contains(n), "stress template " + n);
            FARPartStressTemplate rocket = FARAeroStress.StressTemplates.First(t => t.Name == "rocketFuelTank");
            Assert.Near(30, rocket.XZMaxStress, 1e-9, "rocketFuelTank's XZmaxStress (FARAeroStress.cfg)");
            Assert.True(rocket.FlowModeNeeded, "rocketFuelTank counts only stack-priority resources");

            var wrong = new List<string>();
            int checkedParts = 0;
            foreach (AvailablePart ap in PartLoader.LoadedPartsList)
            {
                Part p = ap.partPrefab;
                if (p == null || ap.name.StartsWith("kerbalEVA") || p.Modules.Contains<LaunchClamp>())
                    continue;
                string[] res = Far.Resources(p);
                string expected = null;
                if (p.HasModuleImplementing<ModuleAblator>() || res.Contains("Ablator")) expected = "structural";
                else if (p.CrewCapacity > 0) expected = "crewed";
                else if (res.Length == 0) expected = "structural";
                else if (res.All(r => r == "SolidFuel")) expected = "SRB";
                else if (res.All(r => r == "LiquidFuel")) expected = "aircraftFuelTank";
                else if (Far.IsRocketTank(p)) expected = "rocketFuelTank";
                if (expected == null) continue;
                checkedParts++;
                string got = FARAeroStress.DetermineStressTemplate(p).Name;
                if (got != expected)
                    wrong.Add($"{ap.name} ({string.Join("+", res)}{(p.CrewCapacity > 0 ? ", crewed" : "")}): {got}, not {expected}");
            }

            ctx.Record("partsChecked", checkedParts);
            Assert.True(checkedParts > 100, $"stock parts checked ({checkedParts})");
            Assert.True(wrong.Count == 0, $"{wrong.Count} parts with the wrong stress template: {string.Join("; ", wrong.Take(15))}");
            yield break;
        }

        /// <summary>Every body with an atmosphere has FAR's settings for it (FARAeroData.cfg, or the defaults).</summary>
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 600)]
        public static IEnumerator Load_AtmosphereSettingsForEveryBody(TestContext ctx)
        {
            int withAtmosphere = 0;
            foreach (CelestialBody body in FlightGlobals.Bodies.Where(b => b.atmosphere))
            {
                withAtmosphere++;
                Assert.True(FARAeroData.AtmosphericConfiguration.TryGetValue(body.flightGlobalsIndex, out BodySettings s),
                            body.bodyName + "'s FAR settings");
                Assert.True(s.ReferenceViscosity > 0 && s.ReferenceTemperature > 0,
                            $"{body.bodyName}: viscosity {s.ReferenceViscosity} at {s.ReferenceTemperature} K");
                ctx.Record(body.bodyName, $"viscosity {s.ReferenceViscosity:G4} at {s.ReferenceTemperature} K");
            }

            Assert.True(withAtmosphere >= 5, $"the stock bodies with an atmosphere ({withAtmosphere}: Kerbin, Eve, Duna, Jool, Laythe)");
            CelestialBody home = FlightGlobals.GetHomeBody();
            Assert.Near(home.atmosphereAdiabaticIndex,
                        FARAtmosphere.GetAdiabaticIndex(home, new Vector3d(0, 0, 0), Planetarium.GetUniversalTime()), 1e-9,
                        "FAR's adiabatic index at Kerbin's surface is the body's");
            yield break;
        }

        /// <summary>
        /// A shader whose config url names no shader (a typo in FARConfig.cfg, or a shader another mod removed): the
        /// default loader (Shader.Find) reports it, the asset keeps the shader it had and nothing throws; the url put
        /// back, it loads again. A loader name no loader has, or none, falls back to the default loader.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 300)]
        public static IEnumerator Load_ABadShaderUrlIsReported(TestContext ctx)
        {
            var configFile = new FarConfigFile();
            Config.ShaderNode node = FARConfig.Shaders.LineRenderer;
            Resources.FARShaderCache.ShaderAssetRequest asset = Resources.FARAssets.Instance.Shaders.LineRenderer;
            string url = node.Url.Value, loader = node.Loader.Value;
            Shader shader = asset.Asset;
            Assert.True(shader != null && shader.name == url, $"the line renderer's shader as shipped ({url})");

            var errors = new List<string>();
            void OnError(string key, Resources.Loading.AssetRequest<Shader> request) => errors.Add($"{key} {request.Url}");
            var logged = new List<string>();
            void OnLog(string condition, string stackTrace, LogType type)
            {
                if (type == LogType.Error || type == LogType.Exception)
                    logged.Add($"{type}: {condition}");
            }

            asset.OnAssetLoadError += OnError;
            Application.logMessageReceived += OnLog;
            try
            {
                ctx.Step("a url that names no shader");
                const string bad = "FAR Tests/No Such Shader";
                node.Url.Value = bad;
                yield return Wait.Until(() => asset.State != Resources.Loading.Progress.InProgress && asset.Url == bad, 10, "the bad url tried", shot: false);
                yield return Wait.Frames(2);
                ctx.Record("badUrl", $"{asset.State}; errors {string.Join(", ", errors)}; log {string.Join(" | ", logged)}");
                Assert.Equal(Resources.Loading.Progress.Error, asset.State, "the load failed");
                Assert.Equal(1, errors.Count, "one load error raised");
                Assert.True(errors[0] == $"line_renderer {bad}", $"for the line renderer's url ({errors[0]})");
                Assert.True(logged.Any(l => l.StartsWith("Error") && l.Contains("Failed to load asset from " + bad)), "the failure is logged");
                Assert.False(logged.Any(l => l.StartsWith("Exception")), "no exception");
                Assert.True(asset.Asset == shader, "the shader it had is kept");

                ctx.Step("the url put back");
                node.Url.Value = url;
                yield return Wait.Until(() => asset.State == Resources.Loading.Progress.Completed, 10, "the shader loaded again", shot: false);
                Assert.True(asset.Asset == shader, "the same shader");

                ctx.Step("an unknown loader, and none");
                node.Loader.Value = "no such loader";
                yield return Wait.Frames(2);
                Assert.Equal("default", asset.Loader.Name, "an unknown loader name falls back to the default loader");
                Assert.Equal(Resources.Loading.Progress.Completed, asset.State, "which loads it");
                node.Loader.Value = "";
                yield return Wait.Frames(2);
                Assert.Equal("default", asset.Loader.Name, "no loader name is the default loader");
                Assert.True(asset.Asset == shader && errors.Count == 1, "still the same shader, no more errors");
            }
            finally
            {
                asset.OnAssetLoadError -= OnError;
                Application.logMessageReceived -= OnLog;
                node.Loader.Value = loader;
                node.Url.Value = url;
                configFile.Restore();
            }
        }

        /// <summary>
        /// ModuleManager's reload (its "reload database", which calls FAR's ModuleManagerPostLoad) has FAR reload every
        /// reloadable part of it: its assets, already loaded, report done at once; its configs load again, and
        /// FARAeroData drops the atmosphere settings it had and makes them again (stopping the load it started the
        /// first time). Afterwards every body has the same settings and FAR's config is as it was, with no errors.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 300)]
        public static IEnumerator Load_AReloadKeepsFarsSettings(TestContext ctx)
        {
            var configFile = new FarConfigFile();
            Dictionary<int, (double, double)> before =
                FARAeroData.AtmosphericConfiguration.ToDictionary(p => p.Key,
                                                                  p => ((double)p.Value.ReferenceViscosity, (double)p.Value.ReferenceTemperature));
            int period = FARConfig.FlightLog.LogPeriod.Value;
            Color cl = FARConfig.GUIColors.ClColor;
            var logged = new List<string>();
            void OnLog(string condition, string stackTrace, LogType type)
            {
                if (type == LogType.Error || type == LogType.Exception)
                    lock (logged)
                        logged.Add($"{type}: {condition} {stackTrace?.Split('\n').FirstOrDefault()}");
            }

            Application.logMessageReceivedThreaded += OnLog;
            try
            {
                ctx.Step("reload");
                Assert.True(before.Count >= 5, $"the bodies' settings before ({before.Count})");
                List<Interfaces.IReloadable> reloadables = FARAddonLoader.Instance.ReloadableObjects;
                ctx.Record("reloadables", string.Join(", ", reloadables.Select(r => $"{r.GetType().Name} ({r.Priority})")));
                yield return FARAddonLoader.Instance.Reload();
                Assert.True(reloadables.All(r => r.Completed), "every reloadable done");
                Assert.True(Resources.FARAssets.Instance.Completed, "the assets done");
                // the atmosphere settings come back on the main thread, once the reloaded config is in
                yield return Wait.Until(() => FARAeroData.AtmosphericConfiguration.Count == before.Count, 30,
                                        "the bodies' atmosphere settings made again", shot: false);
                yield return Wait.Frames(5);
                lock (logged)
                    ctx.Record("logged", string.Join(" | ", logged));
                foreach (KeyValuePair<int, (double, double)> b in before)
                {
                    Assert.True(FARAeroData.AtmosphericConfiguration.TryGetValue(b.Key, out BodySettings s), $"body {b.Key}'s settings");
                    Assert.Equal(b.Value, ((double)s.ReferenceViscosity, (double)s.ReferenceTemperature), $"body {b.Key}'s settings as before");
                }
                Assert.Equal(period, FARConfig.FlightLog.LogPeriod.Value, "FAR's config as before");
                Assert.Equal(cl, FARConfig.GUIColors.ClColor, "FAR's colours as before");
                lock (logged)
                    Assert.True(logged.Count == 0, $"no errors or exceptions in the reload ({string.Join(" | ", logged.Take(3))})");
            }
            finally
            {
                Application.logMessageReceivedThreaded -= OnLog;
                configFile.Restore();
            }
        }

        /// <summary>
        /// Each of FAR's shader assets converts to its shader and to the material made of it, directly and through its
        /// shader-material pair, as the material FAR's renderers take from it does.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 300)]
        public static IEnumerator Load_ShaderAssetsGiveTheirShaderAndMaterial(TestContext ctx)
        {
            Resources.FARShaderCache shaders = Resources.FARAssets.Instance.Shaders;
            var assets = new (string, Resources.FARShaderCache.ShaderAssetRequest)[]
            {
                ("line renderer", shaders.LineRenderer), ("debug voxel", shaders.DebugVoxel),
                ("debug voxel fallback", shaders.DebugVoxelFallback), ("exposed surface", shaders.ExposedSurface),
                ("exposed surface debug", shaders.ExposedSurfaceDebug), ("exposed surface camera", shaders.ExposedSurfaceCamera),
            };
            foreach ((string name, Resources.FARShaderCache.ShaderAssetRequest asset) in assets)
            {
                Shader shader = asset;
                Material material = asset;
                Resources.FARShaderCache.ShaderMaterialPair pair = asset;
                ctx.Record(name, $"{asset.Asset?.name}, material {material?.name}");
                Assert.True(asset.Asset != null && shader == asset.Asset, $"the {name} asset converts to its shader");
                Assert.True(material != null && material == asset.Material && material.shader == shader, $"the {name} asset converts to the material of its shader");
                Assert.True(pair != null && pair.Shader == shader && (Shader)pair == shader, $"the {name} asset's pair holds its shader");
                Assert.True((Material)pair == material && pair.Material == material, $"the {name} asset's pair holds its material");
            }

            yield break;
        }

        /// <summary>
        /// The default loaders, which FAR's shipped config does not use (it loads textures through the GameDatabase and
        /// shaders from its bundle). A texture asset switched to the default loader (a file read from GameData) cannot
        /// load its GameDatabase url, which has no extension: the error is raised and the texture it had kept. Given the
        /// file's name, it loads the same image (its size); asked for it again, the loader gives the texture it loaded
        /// before. The pixel counting compute shader switched to the default loader (Resources.Load) is not found, the
        /// error raised and the shader and its kernel kept. Switched back, both load as shipped; nothing throws.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 300)]
        public static IEnumerator Load_DefaultLoadersForTexturesAndComputeShaders(TestContext ctx)
        {
            var configFile = new FarConfigFile();
            Config.ResourceNode texNode = FARConfig.Textures.IconButtonStock;
            Resources.LoadableAsset<Texture2D> tex = Resources.FARAssets.Instance.Textures.IconLarge;
            Config.ComputeShaderNode csNode = FARConfig.Shaders.CountColors;
            Resources.FARComputeShaderCache.ComputeShaderAssetRequest cs = Resources.FARAssets.Instance.ComputeShaders.CountPixels;
            string texUrl = texNode.Url.Value, texLoader = texNode.Loader.Value, csLoader = csNode.Loader.Value;
            Texture2D shipped = tex.Asset;
            ComputeShader shippedCs = cs.Asset;
            string kernel = cs.Kernel.name;
            int kernelIndex = cs.Kernel.index;
            Assert.True(shipped != null && shippedCs != null, $"the stock button's texture and the pixel counting shader as shipped ({texLoader}, {csLoader})");

            var errors = new List<string>();
            void OnTexError(string key, Resources.Loading.AssetRequest<Texture2D> request) => errors.Add($"{key} {request.Url}");
            void OnCsError(string key, Resources.Loading.AssetRequest<ComputeShader> request) => errors.Add($"{key} {request.Url}");
            var logged = new List<string>();
            void OnLog(string condition, string stackTrace, LogType type)
            {
                if (type == LogType.Exception)
                    logged.Add($"{type}: {condition}");
            }

            tex.OnAssetLoadError += OnTexError;
            cs.OnAssetLoadError += OnCsError;
            Application.logMessageReceived += OnLog;
            try
            {
                ctx.Step("a texture through the default loader, its GameDatabase url");
                texNode.Loader.Value = "default";
                yield return Wait.Until(() => tex.State == Resources.Loading.Progress.Error, 15, "the texture's load to fail", shot: false);
                ctx.Record("noExtension", $"{tex.Loader.Name}: {tex.State}; errors {string.Join(", ", errors)}");
                Assert.Equal("default", tex.Loader.Name, "the default loader");
                Assert.True(errors.Count == 1 && errors[0] == $"stock_button {texUrl}", $"the error raised for the url ({string.Join(", ", errors)})");
                Assert.True(tex.Asset == shipped, "the texture it had kept");

                ctx.Step("the file's name");
                string file = texUrl + ".png";
                texNode.Url.Value = file;
                yield return Wait.Until(() => tex.State == Resources.Loading.Progress.Completed && tex.Asset != shipped, 15, "the texture read from its file", shot: false);
                Texture2D read = tex.Asset;
                ctx.Record("file", $"{read.width} x {read.height} {read.format}, shipped {shipped.width} x {shipped.height} {shipped.format}");
                Assert.True(read.width == shipped.width && read.height == shipped.height, "the same image as the GameDatabase's");

                ctx.Step("the same file again");
                texNode.Url.Value = texUrl;
                yield return Wait.Until(() => tex.State == Resources.Loading.Progress.Error, 15, "the url without extension failing again", shot: false);
                texNode.Url.Value = file;
                yield return Wait.Until(() => tex.State == Resources.Loading.Progress.Completed, 15, "the file's texture", shot: false);
                Assert.True(tex.Asset == read, "the texture the loader read before");

                ctx.Step("the compute shader through the default loader");
                errors.Clear();
                csNode.Loader.Value = "default";
                yield return Wait.Until(() => cs.State == Resources.Loading.Progress.Error, 15, "the compute shader's load to fail", shot: false);
                ctx.Record("computeShader", $"{cs.Loader.Name}: {cs.State}; errors {string.Join(", ", errors)}");
                Assert.True(errors.Count == 1 && errors[0].StartsWith("countPixels "), $"the error raised ({string.Join(", ", errors)})");
                Assert.True(cs.Asset == shippedCs && cs.Kernel.name == kernel && cs.Kernel.index == kernelIndex, "the shader and its kernel kept");
            }
            finally
            {
                tex.OnAssetLoadError -= OnTexError;
                cs.OnAssetLoadError -= OnCsError;
                texNode.Url.Value = texUrl;
                texNode.Loader.Value = texLoader;
                csNode.Loader.Value = csLoader;
            }

            ctx.Step("both switched back");
            yield return Wait.Until(() => tex.State == Resources.Loading.Progress.Completed && cs.State == Resources.Loading.Progress.Completed, 15,
                                    "the texture and the compute shader loaded as shipped", shot: false);
            Application.logMessageReceived -= OnLog;
            configFile.Restore();
            Assert.True(tex.Asset == shipped && cs.Asset == shippedCs, "the shipped texture and compute shader back");
            Assert.True(logged.Count == 0, $"nothing thrown ({string.Join(" | ", logged)})");
        }

        /// <summary>
        /// The shader bundle's path changed while the game runs (FAR's config reloaded by ModuleManager's database
        /// reload, with a new bundle path): both bundle loaders, shaders and compute shaders, load the new bundle. A path
        /// to no bundle is reported and leaves both with the assets they had; the path put back, they load the bundle
        /// again (from the bundles already open) with the same assets.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 300)]
        public static IEnumerator Load_ShaderBundlePathChangedAtRuntime(TestContext ctx)
        {
            var configFile = new FarConfigFile();
            Resources.Loading.ShaderBundleLoader shaders = Resources.FARAssets.Instance.ShaderLoader;
            Resources.Loading.ComputeShaderBundleLoader compute = Resources.FARAssets.Instance.ComputeShaderLoader;
            Observable<string> path = new[] { FARConfig.Shaders.BundleWindows, FARConfig.Shaders.BundleLinux, FARConfig.Shaders.BundleMac }
                .FirstOrDefault(o => o.Value == shaders.Url);
            Assert.NotNull(path, $"the config's bundle path the loaders use ({shaders.Url})");
            string url = path.Value;
            string[] shaderNames = shaders.LoadedAssets.Keys.OrderBy(k => k).ToArray(), computeNames = compute.LoadedAssets.Keys.OrderBy(k => k).ToArray();
            ctx.Record("bundle", $"{url}: {shaderNames.Length} shaders, {computeNames.Length} compute shaders");
            Assert.True(shaderNames.Length > 0 && computeNames.Length > 0, "the bundle's shaders and compute shaders loaded");

            var logged = new List<string>();
            void OnLog(string condition, string stackTrace, LogType type)
            {
                if (condition.Contains("bundle"))
                    logged.Add($"{type}: {condition}");
            }

            Application.logMessageReceived += OnLog;
            try
            {
                ctx.Step("a path to no bundle");
                path.Value = "FerramAerospaceResearch/Assets/no_such_bundle.far";   // relative to GameData, as in the config
                string bad = path.Value;
                ctx.Record("badPath", bad);
                Assert.True(System.IO.Path.IsPathRooted(bad) && bad.EndsWith("no_such_bundle.far"), $"the config's path made absolute ({bad})");
                Assert.True(shaders.Url == bad && compute.Url == bad, $"both loaders given the new path ({shaders.Url}, {compute.Url})");
                yield return Wait.Until(() => shaders.State == Resources.Loading.Progress.Error && compute.State == Resources.Loading.Progress.Error, 15,
                                        "both loaders to fail", shot: false);
                ctx.Record("bad", string.Join(" | ", logged));
                Assert.True(logged.Any(l => l.Contains("Loading shaders bundle")) && logged.Any(l => l.Contains("Loading compute shaders bundle")),
                            "each loader's reload logged");
                Assert.True(logged.Any(l => l.StartsWith("Error") && l.Contains("Could not load asset bundle from " + bad)), "the missing bundle reported");
                Assert.True(shaders.LoadedAssets.Keys.OrderBy(k => k).SequenceEqual(shaderNames) &&
                            compute.LoadedAssets.Keys.OrderBy(k => k).SequenceEqual(computeNames), "the assets they had kept");

                ctx.Step("the path put back");
                logged.Clear();
                path.Value = url;
                yield return Wait.Until(() => shaders.State == Resources.Loading.Progress.Completed && compute.State == Resources.Loading.Progress.Completed, 15,
                                        "both loaders to load the bundle again", shot: false);
                ctx.Record("back", string.Join(" | ", logged));
                Assert.True(shaders.LoadedAssets.Keys.OrderBy(k => k).SequenceEqual(shaderNames) &&
                            compute.LoadedAssets.Keys.OrderBy(k => k).SequenceEqual(computeNames), "the same assets");
                Assert.False(logged.Any(l => l.StartsWith("Error") || l.StartsWith("Exception")), "no error");
            }
            finally
            {
                Application.logMessageReceived -= OnLog;
                if (path.Value != url)
                    path.Value = url;
                configFile.Restore();
            }
        }
    }
}
