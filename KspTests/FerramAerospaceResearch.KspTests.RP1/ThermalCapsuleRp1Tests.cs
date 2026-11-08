using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using KspTest;
using KspTest.RP1;
using UnityEngine;
using static FerramAerospaceResearch.KspTests.FarThermalModeTests;

namespace FerramAerospaceResearch.KspTests.RP1
{
    /// <summary>
    /// RO capsules on their own heat shields (Mercury, Gemini, Apollo) through a KCT reentry sim, under each
    /// combination of FAR's "Voxel thermal areas" model option (on = before KSPModStewards FAR #156) and KSPCF's
    /// OcclusionVehicleFrame (KSPCommunityFixes #436), all in one session.
    /// </summary>
    public static class ThermalCapsuleRp1Tests
    {
        private sealed class Capsule
        {
            public string Key, Pod, Shield;
            public double ApKm;
            public Capsule(string key, string pod, string shield, double apKm) { Key = key; Pod = pod; Shield = shield; ApKm = apKm; }
        }

        private static readonly Capsule[] Capsules =
        {
            new Capsule("Mercury", "ROC-MercuryCMBDB", "ROC-MercuryHS", 250),
            new Capsule("Gemini", "ROC-GeminiCMBDB", "ROC-GeminiHSBDB", 250),
            new Capsule("Apollo", "ROC-ApolloCMBDB", "ROC-ApolloHSBDB", 250),
        };

        private sealed class Combo
        {
            public string Name;
            public bool? Voxel;   // null: FAR's areas left as the build has them (a FAR without the model option)
            public bool Occl;
            public Combo(string name, bool? voxel, bool occl) { Name = name; Voxel = voxel; Occl = occl; }
        }

        // what FAR's model options window sets, and what KSPCF reads, switched by reflection
        private static FieldInfo voxelField;
        private static MethodInfo restoreStock;
        private static PropertyInfo occlEnabled;

        private static void Bind()
        {
            Type dv = Reflect.TryFindType("FerramAerospaceResearch.FARDebugValues") ?? throw new UITestFailure("FARDebugValues not found");
            // absent in FAR releases without the model option: only the AsIs combos run there
            voxelField = dv.GetField("voxelThermalAreas", BindingFlags.Public | BindingFlags.Static);
            restoreStock = Reflect.TryFindType("FerramAerospaceResearch.FARAeroComponents.ModularFlightIntegratorRegisterer")
                ?.GetMethod("RestoreStockOcclusion", BindingFlags.Public | BindingFlags.Static);
            // absent in KSPCF releases before #436: only the Stock combos run there
            occlEnabled = Reflect.TryFindType("KSPCommunityFixes.BugFixes.OcclusionVehicleFrame")
                ?.GetProperty("IsEnabled", BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Static);
        }

        private static bool? Voxel
        {
            get => voxelField == null ? (bool?)null : (bool)voxelField.GetValue(null);
            set
            {
                if (value == null) return;
                if (voxelField == null) throw new UITestFailure("this FAR has no voxelThermalAreas model option: use the AsIs combos");
                voxelField.SetValue(null, value.Value);
            }
        }

        // null: a KSPCF without OcclusionVehicleFrame, which is stock occlusion
        private static bool? Occl
        {
            get => occlEnabled == null ? (bool?)null : (bool)occlEnabled.GetValue(null, null);
            set
            {
                if (value == null) return;
                if (occlEnabled == null)
                {
                    if (value.Value) throw new UITestFailure("this KSPCF has no OcclusionVehicleFrame (#436): use the Stock combos");
                    return;
                }
                occlEnabled.SetValue(null, value.Value, null);
            }
        }

        private static void Apply(Combo c)
        {
            bool? wasVoxel = Voxel;
            Voxel = c.Voxel;
            Occl = c.Occl;
            if (wasVoxel == true && c.Voxel == false && HighLogic.LoadedSceneIsFlight)
                restoreStock?.Invoke(null, null);
            Log.Info($"thermal combo {c.Name}: voxelThermalAreas {Voxel}, OcclusionVehicleFrame {Occl}");
        }

        /// <summary>
        /// Params: capsules (Mercury+Gemini+Apollo), craft (absolute path of a craft to fly instead, as "User"), combos (Pr156Occl+Pr156Stock+VoxelOccl+VoxelStock; AsIsOccl/AsIsStock leave FAR alone, for a FAR without the option; only the *Stock ones run with a KSPCF before #436), peKm (30), pes ("+"-separated perigees instead), apKm (250; the sim starts in a 250 km circular orbit), lanDeg (0), argPeDeg (0), orients ("lan:argPe+..." instead of those two),
        /// entryAltKm (110), incDeg (28.6), aoa (180 = heat shield first), aoas ("+"-separated instead), hold (steer: set angular velocity; torque: rigidbody forces; part: Part.AddForce/AddTorque, for Principia; torque and part add an integral trim), holdWn (15), place (teleport|sim), editorOnly, shots, endAltKm (20), hyper (10).
        /// </summary>
        [UITest("Far_ThermalCapsules_RP1", Explicit = true, Kind = "Measure", NewGame = "career", Requires = "RP-1", StartsAt = StartState.SpaceCenter, TimeoutSec = 10800)]
        public static IEnumerator Far_ThermalCapsules_RP1(TestContext ctx)
        {
            Ugui.DismissPopups();
            ctx.Record("farBuild", FarSim.FarVersion());
            double peKm = ctx.ParamDouble("peKm", 30), entryAltKm = ctx.ParamDouble("entryAltKm", 110), incDeg = ctx.ParamDouble("incDeg", 28.6);
            double aoa = ctx.ParamDouble("aoa", 180), endAltKm = ctx.ParamDouble("endAltKm", 20), hyper = ctx.ParamDouble("hyper", 10);
            // angles of attack ("+"-separated), each flown under every perigee, orientation and combo
            string[] aoas = (ctx.Param("aoas") ?? aoa.ToString(System.Globalization.CultureInfo.InvariantCulture)).Split('+');
            // "lan:argPe" orbit orientations ("+"-separated), each flown under every combo
            string[] orients = (ctx.Param("orients") ?? $"{ctx.ParamDouble("lanDeg", 0)}:{ctx.ParamDouble("argPeDeg", 0)}").Split('+');
            double apOverride = ctx.ParamDouble("apKm", double.NaN);
            // "teleport": circular sim, then SetShipOrbit onto the entry orbit; "sim": RP-1's elliptical sim starts there
            // (needed under Principia, which ignores the teleport)
            string place = ctx.Param("place", "teleport");
            var allCombos = new[]
            {
                new Combo("Pr156Occl", false, true), new Combo("Pr156Stock", false, false),
                new Combo("VoxelOccl", true, true), new Combo("VoxelStock", true, false),
                new Combo("AsIsOccl", null, true), new Combo("AsIsStock", null, false),
            };
            // perigees (km, "+"-separated), each flown under every orientation and combo
            string[] pes = (ctx.Param("pes") ?? peKm.ToString(System.Globalization.CultureInfo.InvariantCulture)).Split('+');
            string[] comboNames = (ctx.Param("combos") ?? "Pr156Occl+Pr156Stock+VoxelOccl+VoxelStock").Split('+');
            Combo[] combos = comboNames.Select(n => allCombos.First(c => c.Name.Equals(n, StringComparison.OrdinalIgnoreCase))).ToArray();
            // a supplied craft (absolute path) instead of the generated capsules, flown as capsule "User"
            string userCraft = ctx.Param("craft");
            string[] capsuleNames = userCraft != null ? new[] { "User" } : (ctx.Param("capsules") ?? "Mercury+Gemini+Apollo").Split('+');

            Bind();
            ctx.Record("kspcfPatches", KspcfPatchState());
            bool? startVoxel = Voxel;
            bool? startOccl = Occl;
            ctx.Record("startOptions", $"voxelThermalAreas {startVoxel}, OcclusionVehicleFrame {startOccl}");

            var summary = new StringBuilder("capsule,combo,part,index,peakSkinK,skinMaxK,peakSkinFrac,peakIntK,intMaxK,peakIntFrac,ablatorUsed,radArea,expArea,convArea,convAreaMult,convTempMult,skinExpFrac,skinMass,crossSection\n");
            try
            {
                foreach (string capName in capsuleNames)
                {
                    Capsule cap;
                    string craft;
                    if (userCraft != null)
                    {
                        cap = new Capsule("User", null, null, 250);
                        ctx.Step("Load " + Path.GetFileName(userCraft));
                        string dir = Path.Combine(KSPUtil.ApplicationRootPath, "saves", HighLogic.SaveFolder, "Ships", "VAB");
                        Directory.CreateDirectory(dir);
                        craft = Path.Combine(dir, Path.GetFileName(userCraft));
                        File.Copy(userCraft, craft, true);
                    }
                    else
                    {
                        cap = Capsules.First(c => c.Key.Equals(capName, StringComparison.OrdinalIgnoreCase));
                        ctx.Step($"Build {cap.Key}");
                        craft = WriteCraft(ctx, cap);
                    }
                    if (HighLogic.LoadedScene == GameScenes.SPACECENTER)
                        yield return Nav.EnterEditor(ctx, EditorFacility.VAB, craft);
                    else
                    {
                        EditorLogic.LoadShipFromFile(craft);
                        yield return Wait.Seconds(2);
                        yield return Wait.EditorReady(true, 120);
                    }
                    ctx.Record($"{cap.Key}.craft", string.Join(", ", EditorLogic.fetch.ship.parts.Select(p => $"{p.partInfo.name} {p.transform.position.y:F3}")));
                    ctx.Record($"{cap.Key}.editorJoint", JointGap(EditorLogic.fetch.ship.parts, out double eg, out double el)
                        ? $"pod-shield node gap {eg * 100:F2} cm, lateral {el * 100:F2} cm" : "no parent/child node pair");
                    // where the meshes are, along the pod's axis (world y in the editor): a visible gap shows here
                    foreach (Part p in EditorLogic.fetch.ship.parts)
                    {
                        var rs = p.GetComponentsInChildren<Renderer>().Where(r => r.enabled && r.gameObject.activeInHierarchy && (r is MeshRenderer || r is SkinnedMeshRenderer)).ToList();
                        if (rs.Count == 0) continue;
                        float lo = rs.Min(r => r.bounds.min.y), hi = rs.Max(r => r.bounds.max.y);
                        float wide = rs.Max(r => Mathf.Max(r.bounds.size.x, r.bounds.size.z));
                        ctx.Record($"{cap.Key}.mesh.{p.partInfo.name}", $"y {lo:F3} to {hi:F3} m, widest {wide:F3} m, origin {p.transform.position.y:F3}");
                    }
                    if (ctx.Param("shots", "off") == "on")
                        yield return EditorShots.Views(ctx, cap.Key, new[] { "side", "front" }, true);
                    if (ctx.Param("editorOnly", "off") == "on")
                        continue;

                    foreach (string aoaStr in aoas)
                    foreach (string peStr in pes)
                    foreach (string orient in orients)
                    foreach (Combo combo in combos)
                    {
                        peKm = double.Parse(peStr, System.Globalization.CultureInfo.InvariantCulture);
                        aoa = double.Parse(aoaStr, System.Globalization.CultureInfo.InvariantCulture);
                        string[] lanArg = orient.Split(':');
                        double lanDeg = double.Parse(lanArg[0], System.Globalization.CultureInfo.InvariantCulture);
                        double argPeDeg = double.Parse(lanArg[1], System.Globalization.CultureInfo.InvariantCulture);
                        string where = (aoas.Length > 1 ? $"@AoA{aoa:F0}" : "") + (pes.Length > 1 ? $"@P{peKm:F0}" : "") + (orients.Length > 1 ? $"@L{lanDeg:F0}A{argPeDeg:F0}" : "");
                        string label = $"{cap.Key}{where}.{combo.Name}";
                        ctx.Step($"Entry {label}");
                        Apply(combo);
                        // release RP-1's sim has no reentry mode: start in a circular orbit, then put the capsule on the
                        // entry orbit (same LAN and ArgPe every time, so every combo flies the same path in the same frame)
                        double entryApKm = double.IsNaN(apOverride) ? cap.ApKm : apOverride;
                        if (place == "sim")
                        {
                            // RP-1 itself starts the sim on the entry orbit, at the point descending through entryAltKm:
                            // a placement Principia accepts (it ignores a later SetShipOrbit)
                            yield return Rp1.ConfigureSim(new SimSpec { StartInOrbit = true, Mode = "Elliptical", PeKm = peKm, ApKm = entryApKm, IncDeg = incDeg });
                            Type gui = Reflect.FindType("RP0.KCT_GUI");
                            var inv = System.Globalization.CultureInfo.InvariantCulture;
                            Reflect.SetStatic(gui, "_sOrbitLAN", lanDeg.ToString("R", inv));
                            Reflect.SetStatic(gui, "_sOrbitArgPe", argPeDeg.ToString("R", inv));
                            Reflect.SetStatic(gui, "_sOrbitMNA", MeanAnomalyDescendingAt(Planetarium.fetch.Home, peKm * 1000, entryApKm * 1000, entryAltKm * 1000).ToString("R", inv));
                            yield return Rp1.StartSim(waitPlaced: true);
                            yield return WaitVoxels();
                        }
                        else
                        {
                            yield return Rp1.Simulate(ctx, SimSpec.Circular(cap.ApKm, incDeg));
                            yield return WaitVoxels();
                            Flight.SetOrbitDescendingAt(peKm * 1000, entryApKm * 1000, incDeg, lanDeg, argPeDeg, entryAltKm * 1000);
                            yield return Wait.Seconds(2);
                            yield return Wait.Until(() => !FlightGlobals.ActiveVessel.packed, 60, "off rails after placing");
                        }
                        Flight.PointAtAoA(aoa);
                        yield return Wait.Seconds(1);
                        Vessel v = FlightGlobals.ActiveVessel;
                        // FAR's voxel areas per part as voxelized, and once more after a fresh voxelization
                        ThermalAreaModes.Install();   // reflection only here: the Native mode leaves FAR's code as it is
                        ctx.Record($"{label}.farVoxel", FarAreas(v));
                        if (ctx.Param("revoxel", "off") == "on")
                        {
                            FarApi.RequestRevoxel(v);
                            yield return Wait.Seconds(1);
                            yield return WaitVoxels();
                            ctx.Record($"{label}.farVoxelAgain", FarAreas(v));
                        }
                        if (v.altitude < entryAltKm * 500)
                            throw new UITestFailure($"the sim did not start in orbit (alt {v.altitude / 1000:F1} km, {v.situation})");
                        Apply(combo);   // the scene change kept the statics; this also restores stock occlusion when leaving voxels
                        Flight.IgnoreMaxTemperature(true);
                        Ctrl.Sas(false);
                        Flight.PointAtAoA(aoa);
                        ctx.Record($"{label}.start", $"UT {Planetarium.GetUniversalTime():F0}, alt {v.altitude / 1000:F1} km, speed {v.srfSpeed:F0} m/s, options {Voxel}/{Occl}");
                        if (combo == combos[0] && capName == capsuleNames[0] && orient == orients[0] && peStr == pes[0] && aoaStr == aoas[0])
                        {
                            // the window the options live in, once
                            Reflect.TryFindType("FerramAerospaceResearch.FARDebugAndSettings")?.GetMethod("ToggleModelOptions")?.Invoke(null, null);
                            yield return Wait.Seconds(1);
                            yield return Snap.Shot(ctx, "model-options");
                            Reflect.TryFindType("FerramAerospaceResearch.FARDebugAndSettings")?.GetMethod("ToggleModelOptions")?.Invoke(null, null);
                        }
                        yield return Fly(ctx, v, label, aoa, endAltKm, hyper, summary);
                        Flight.IgnoreMaxTemperature(false);
                        yield return Rp1.RevertToEditorViaWindow();
                    }
                }
            }
            finally
            {
                Voxel = startVoxel;
                Occl = startOccl;
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "capsules_parts.csv"), summary.ToString());
            }
        }

        /// <summary>
        /// A player's own save (saves/user-gemini: their quicksave, as usersave.sfs) quickloaded as KSP's F9 does, once per
        /// entry: warp to sepAltKm on the way down, jettison the equipment section and then the retro section (their own
        /// decouplers), and fly the rest of the way under each combo. Params: combos (Pr156Stock+VoxelStock), aoas
        /// ("+"-separated; "free" = no attitude hold), repeats (2), sepAltKm (150), endAltKm (20), hyper (10), hold (part),
        /// holdWn (15), trimKi (2).
        /// </summary>
        [UITest("Far_UserSaveEntry_RP1", Explicit = true, Kind = "Measure", Save = "user-gemini", Requires = "RP-1", StartsAt = StartState.SpaceCenter, TimeoutSec = 14400)]
        public static IEnumerator Far_UserSaveEntry_RP1(TestContext ctx)
        {
            Ugui.DismissPopups();
            Bind();
            bool? startVoxel = Voxel;
            bool? startOccl = Occl;
            var inv = System.Globalization.CultureInfo.InvariantCulture;
            string[] comboNames = ctx.Param("combos", "Pr156Stock+VoxelStock").Split('+');
            var all = new[]
            {
                new Combo("Pr156Occl", false, true), new Combo("Pr156Stock", false, false),
                new Combo("VoxelOccl", true, true), new Combo("VoxelStock", true, false),
                new Combo("AsIsOccl", null, true), new Combo("AsIsStock", null, false),
            };
            Combo[] combos = comboNames.Select(n => all.First(c => c.Name.Equals(n, StringComparison.OrdinalIgnoreCase))).ToArray();
            string[] aoas = ctx.Param("aoas", "180+170").Split('+');
            int repeats = ctx.ParamInt("repeats", 2);
            double sepAltKm = ctx.ParamDouble("sepAltKm", 150), endAltKm = ctx.ParamDouble("endAltKm", 20), hyper = ctx.ParamDouble("hyper", 10);
            ctx.Record("kspcf", KspcfPatchState());
            ctx.Record("principia", Reflect.TryFindType("principia.ksp_plugin_adapter.PrincipiaPluginAdapter") != null ? "loaded" : "absent");
            ctx.Record("startOptions", $"voxelThermalAreas {startVoxel}, OcclusionVehicleFrame {startOccl}");

            var summary = new StringBuilder("capsule,combo,part,index,peakSkinK,skinMaxK,peakSkinFrac,peakIntK,intMaxK,peakIntFrac,ablatorUsed,radArea,expArea,convArea,convAreaMult,convTempMult,skinExpFrac,skinMass,crossSection\n");
            try
            {
                for (int rep = 1; rep <= repeats; rep++)
                foreach (string aoaStr in aoas)
                foreach (Combo combo in combos)
                {
                    bool free = aoaStr == "free";
                    double aoa = free ? 180 : double.Parse(aoaStr, inv);
                    string label = $"Save@{(free ? "Free" : "AoA" + aoaStr)}r{rep}.{combo.Name}";
                    ctx.Step($"Entry {label}");

                    // F9: the player's quicksave, as KSP's quickload does it
                    Game game = GamePersistence.LoadGame("usersave", HighLogic.SaveFolder, true, false)
                                ?? throw new UITestFailure("could not load usersave.sfs");
                    FlightDriver.StartAndFocusVessel(game, game.flightState.activeVesselIdx);
                    yield return Wait.Seconds(2);
                    yield return Wait.Scene(GameScenes.FLIGHT, 600);
                    yield return Wait.Until(() => FlightGlobals.ready && FlightGlobals.ActiveVessel != null && !FlightGlobals.ActiveVessel.packed, 300, "the quickloaded vessel off rails");
                    yield return WaitVoxels();
                    Apply(combo);
                    Vessel v = FlightGlobals.ActiveVessel;
                    ctx.Record($"{label}.loaded", $"{v.vesselName}: UT {Planetarium.GetUniversalTime():F0}, alt {v.altitude / 1000:F1} km, Ap {v.orbit.ApA / 1000:F1} km, Pe {v.orbit.PeA / 1000:F1} km, {v.parts.Count} parts");
                    ThermalAreaModes.Install();   // reflection only: binds FAR's types for FarAreas
                    ctx.Record($"{label}.farVoxelLoad", FarAreas(v));

                    // down to the separation altitude on rails: osculating orbit, twice, since Principia's path is not KSP's conic
                    for (int pass = 0; pass < 3 && v.altitude > sepAltKm * 1000 + 5000; pass++)
                    {
                        double ut = UtDescendingThrough(v.orbit, sepAltKm * 1000 + 2000);
                        yield return Flight.WarpTo(ut - (pass == 0 ? 120 : 20), 1800);
                        yield return Wait.Until(() => !FlightGlobals.ActiveVessel.packed, 60, "off rails after warp");
                        v = FlightGlobals.ActiveVessel;
                    }
                    yield return Wait.Until(() => FlightGlobals.ActiveVessel.altitude < sepAltKm * 1000, 600, $"down to {sepAltKm} km");
                    v = FlightGlobals.ActiveVessel;
                    Part hs = v.parts.FirstOrDefault(p => p.partInfo.name == "ROC-GeminiHSBDB") ?? throw new UITestFailure("no Gemini heat shield on the active vessel");
                    Part cm = v.parts.FirstOrDefault(p => p.partInfo.name == "ROC-GeminiCMBDB") ?? throw new UITestFailure("no Gemini capsule on the active vessel");
                    ctx.Record($"{label}.cubesBeforeSep", CubeState(hs) + "; " + CubeState(cm));

                    // the player's sequence: equipment section off, then the retro section from the heat shield
                    foreach (string sep in new[] { "ROC-GeminiEquipmentSectionBDB", "ROC-RetrogradeDecouplerBDB" })
                    {
                        Part sp = FlightGlobals.ActiveVessel.parts.FirstOrDefault(p => p.partInfo.name == sep);
                        ModuleDecouple d = sp?.FindModuleImplementing<ModuleDecouple>();
                        if (d == null || d.isDecoupled) { ctx.Record($"{label}.sep.{sep}", "not there"); continue; }
                        d.Decouple();
                        yield return Wait.Seconds(3);
                    }
                    v = FlightGlobals.ActiveVessel;
                    ctx.Record($"{label}.separated", $"{v.vesselName}: {string.Join(", ", v.parts.Select(p => p.partInfo.name))}, alt {v.altitude / 1000:F1} km, Pe {v.orbit.PeA / 1000:F1} km");
                    yield return WaitVoxels();
                    ctx.Record($"{label}.cubesAfterSep", CubeState(hs) + "; " + CubeState(cm));
                    ctx.Record($"{label}.farVoxelSep", FarAreas(v));

                    Apply(combo);
                    Flight.IgnoreMaxTemperature(true);
                    Ctrl.Sas(false);
                    if (!free) Flight.PointAtAoA(aoa);
                    ctx.Record($"{label}.start", $"UT {Planetarium.GetUniversalTime():F0}, alt {v.altitude / 1000:F1} km, speed {v.srfSpeed:F0} m/s, options {Voxel}/{Occl}");
                    yield return Fly(ctx, v, label, aoa, endAltKm, hyper, summary, free ? "none" : ctx.Param("hold", "part"));
                    ctx.Record($"{label}.cubesEnd", CubeState(hs) + "; " + CubeState(cm));
                    Flight.IgnoreMaxTemperature(false);
                }
            }
            finally
            {
                Voxel = startVoxel;
                Occl = startOccl;
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "capsules_parts.csv"), summary.ToString());
            }
        }

        /// <summary>A part's stock drag cube occlusion (AreaOccluded per face, +X -X +Y -Y +Z -Z) and thermal areas.</summary>
        private static string CubeState(Part p)
        {
            if (p == null) return "gone";
            string faces = p.DragCubes.None ? "none" : string.Join(" ", p.DragCubes.AreaOccluded.Select(a => a.ToString("F2")));
            return $"{p.partInfo.name}: occluded [{faces}] exposed {p.exposedArea:F2} radiative {p.radiativeArea:F2} vessel {(p.vessel == null ? "-" : p.vessel.vesselName)}";
        }

        /// <summary>UT at which the orbit next passes radius body.Radius + altM going down.</summary>
        private static double UtDescendingThrough(Orbit o, double altM)
        {
            double r = o.referenceBody.Radius + altM, e = o.eccentricity, p = o.semiMajorAxis * (1 - e * e);
            double cosNu = Math.Max(-1, Math.Min(1, (p / r - 1) / e));
            double nu = 2 * Math.PI - Math.Acos(cosNu);
            double E = 2 * Math.Atan(Math.Sqrt((1 - e) / (1 + e)) * Math.Tan(nu / 2));
            double M = E - e * Math.Sin(E);
            if (M < 0) M += 2 * Math.PI;
            double now = Planetarium.GetUniversalTime();
            double obtNow = o.getObtAtUT(now);
            double dt = M / o.meanMotion - obtNow;
            double period = o.period;
            while (dt < 0) dt += period;
            while (dt > period) dt -= period;
            return now + dt;
        }

        /// <summary>Mean anomaly (rad) on the orbit with these apsis altitudes where it descends through altM.</summary>
        private static double MeanAnomalyDescendingAt(CelestialBody body, double peAltM, double apAltM, double altM)
        {
            double rp = body.Radius + peAltM, ra = body.Radius + apAltM, r = body.Radius + altM;
            double sma = 0.5 * (rp + ra), ecc = (ra - rp) / (ra + rp);
            double cosNu = Math.Max(-1, Math.Min(1, (sma * (1 - ecc * ecc) / r - 1) / ecc));
            double nu = 2 * Math.PI - Math.Acos(cosNu);
            double E = 2 * Math.Atan(Math.Sqrt((1 - ecc) / (1 + ecc)) * Math.Tan(nu / 2));
            double M = E - ecc * Math.Sin(E);
            return M < 0 ? M + 2 * Math.PI : M;
        }

        private static string KspcfPatchState()
        {
            var names = new[] { "OcclusionVehicleFrame", "FlightIntegratorPerf" };
            return string.Join(", ", names.Select(n =>
            {
                Type t = Reflect.TryFindType("KSPCommunityFixes.BugFixes." + n) ?? Reflect.TryFindType("KSPCommunityFixes.Performance." + n);
                PropertyInfo en = t?.GetProperty("IsEnabled", BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Static);
                return $"{n} {(t == null ? "missing" : en == null ? "present" : en.GetValue(null, null).ToString())}";
            }));
        }

        /// <summary>The capsule and its own shield, the shield's top node on the pod's bottom node, as a VAB craft in the save.</summary>
        private static string WriteCraft(TestContext ctx, Capsule cap)
        {
            AvailablePart podInfo = PartLoader.getPartInfoByName(cap.Pod) ?? throw new UITestFailure($"no part {cap.Pod}");
            AvailablePart hsInfo = PartLoader.getPartInfoByName(cap.Shield) ?? throw new UITestFailure($"no part {cap.Shield}");
            AttachNode podBottom = StackNode(podInfo.partPrefab, "bottom", lowest: true);
            AttachNode hsTop = StackNode(hsInfo.partPrefab, "top", lowest: false);
            Vector3 podPos = new Vector3(0, 10, 0);
            Vector3 hsPos = podPos + podBottom.position - hsTop.position;
            ctx.Record($"{cap.Key}.nodes", $"{cap.Pod}.{podBottom.id} {podBottom.position}, {cap.Shield}.{hsTop.id} {hsTop.position}");

            const uint podId = 4000000001, hsId = 4000000002;
            var ship = new ConfigNode();
            ship.AddValue("ship", cap.Key + " Thermal");
            ship.AddValue("version", "1.12.5");
            ship.AddValue("description", "");
            ship.AddValue("type", "VAB");
            ship.AddValue("size", "3,3,3");
            ship.AddValue("persistentId", "4000000000");
            ship.AddNode(PartNode(cap.Pod, podId, podPos, link: $"{cap.Shield}_{hsId}", attN: $"{podBottom.id},{cap.Shield}_{hsId}", stage: 0));
            ship.AddNode(PartNode(cap.Shield, hsId, hsPos, link: null, attN: $"{hsTop.id},{cap.Pod}_{podId}", stage: 0));
            string dir = Path.Combine(KSPUtil.ApplicationRootPath, "saves", HighLogic.SaveFolder, "Ships", "VAB");
            Directory.CreateDirectory(dir);
            string path = Path.Combine(dir, cap.Key + " Thermal.craft");
            ship.Save(path);
            File.Copy(path, Path.Combine(ctx.ResultsDir, cap.Key + " Thermal.craft"), true);
            return path;
        }

        private static AttachNode StackNode(Part prefab, string id, bool lowest)
        {
            AttachNode n = prefab.FindAttachNode(id);
            if (n != null) return n;
            List<AttachNode> stack = prefab.attachNodes.Where(a => a.nodeType == AttachNode.NodeType.Stack).ToList();
            if (stack.Count == 0) throw new UITestFailure($"{prefab.partInfo.name} has no stack nodes");
            return lowest ? stack.OrderBy(a => a.position.y).First() : stack.OrderByDescending(a => a.position.y).First();
        }

        private static ConfigNode PartNode(string name, uint id, Vector3 pos, string link, string attN, int stage)
        {
            var n = new ConfigNode("PART");
            n.AddValue("part", $"{name}_{id}");
            n.AddValue("partName", "Part");
            n.AddValue("persistentId", id);
            n.AddValue("pos", $"{pos.x:R},{pos.y:R},{pos.z:R}");
            n.AddValue("attPos", "0,0,0");
            n.AddValue("attPos0", $"{pos.x:R},{pos.y:R},{pos.z:R}");
            n.AddValue("rot", "0,0,0,1");
            n.AddValue("attRot", "0,0,0,1");
            n.AddValue("attRot0", "0,0,0,1");
            n.AddValue("mir", "1,1,1");
            n.AddValue("symMethod", "Radial");
            n.AddValue("autostrutMode", "Off");
            n.AddValue("rigidAttachment", "False");
            n.AddValue("istg", stage);
            n.AddValue("resPri", 0);
            n.AddValue("dstg", 0);
            n.AddValue("sidx", -1);
            n.AddValue("sqor", -1);
            n.AddValue("sepI", 0);
            n.AddValue("attm", 0);
            n.AddValue("modCost", 0);
            n.AddValue("modMass", 0);
            n.AddValue("modSize", "0,0,0");
            if (link != null) n.AddValue("link", link);
            n.AddValue("attN", attN);
            return n;
        }

        /// <summary>
        /// The joint between the root (pod) and its child (shield): distance between the two attach nodes in world space
        /// and its part across the pod's axis. False when there is no such pair.
        /// </summary>
        private static bool JointGap(List<Part> parts, out double gap, out double lateral)
        {
            gap = lateral = 0;
            Part child = parts.FirstOrDefault(p => p != null && p.parent != null);
            Part parent = child?.parent;
            if (child == null || parent == null) return false;
            AttachNode pn = parent.FindAttachNodeByPart(child), cn = child.FindAttachNodeByPart(parent);
            if (pn == null || cn == null) return false;
            Vector3 a = parent.transform.TransformPoint(pn.position), b = child.transform.TransformPoint(cn.position);
            Vector3 d = b - a, axis = parent.transform.up;
            gap = d.magnitude;
            lateral = (d - Vector3.Dot(d, axis) * axis).magnitude;
            return true;
        }

        private static string FarAreas(Vessel v)
        {
            string gap = JointGap(v.parts, out double g, out double l) ? $"; node gap {g * 100:F1} cm, lateral {l * 100:F1} cm" : "";
            return string.Join("; ", v.parts.Select(p =>
                $"{p.partInfo.name} total {ThermalAreaModes.FarTotal(p):F2} faces [{string.Join(" ", ThermalAreaModes.FarFaces(p).Select(x => x.ToString("F2")))}]")) + gap;
        }

        private static readonly MethodInfo aoaRotation = typeof(Flight).GetMethod("AoARotation", BindingFlags.NonPublic | BindingFlags.Static, null,
                                                                                    new[] { typeof(Vessel), typeof(double) }, null);

        /// <summary>
        /// Flight.TorqueHoldAoA through KSP's Part.AddForce/AddTorque instead of the rigidbodies: Principia integrates
        /// the vessel's rotation itself and only takes the forces and torques parts carry (Part.force/torque), read at
        /// FashionablyLate. Same damped angular acceleration, given to each part as m (alpha x r) about the vessel's
        /// mass center plus its own I alpha, so the joints carry none of it.
        /// </summary>
        private static void PartApiHold(double aoaDeg, double wn)
        {
            Vessel v = FlightGlobals.ActiveVessel;
            Quaternion target = (Quaternion)aoaRotation.Invoke(null, new object[] { v, aoaDeg });
            Quaternion error = target * Quaternion.Inverse(v.transform.rotation);
            error.ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180f) angle -= 360f;
            if (float.IsNaN(axis.x) || Mathf.Abs(angle) < 1e-6f) axis = Vector3.zero;
            Vector3 theta = axis * (angle * Mathf.Deg2Rad);
            Vector3 omega = Vector3.zero, com = Vector3.zero;
            float mass = 0;
            foreach (Part p in v.parts)
                if (p.Rigidbody != null)
                {
                    omega += p.Rigidbody.angularVelocity * p.Rigidbody.mass;
                    com += p.Rigidbody.worldCenterOfMass * p.Rigidbody.mass;
                    mass += p.Rigidbody.mass;
                }
            if (mass <= 0) return;
            omega /= mass;
            com /= mass;
            Vector3 alpha = (float)(wn * wn) * theta - (float)(2 * wn) * omega;
            foreach (Part p in v.parts)
            {
                Rigidbody rb = p.Rigidbody;
                if (rb == null) continue;
                Vector3 r = rb.worldCenterOfMass - com;
                p.AddForce(Vector3.Cross(alpha, r) * rb.mass);
                Quaternion q = rb.rotation * rb.inertiaTensorRotation;
                p.AddTorque(q * Vector3.Scale(rb.inertiaTensor, Quaternion.Inverse(q) * alpha));
            }
        }

        /// <summary>Integral trim on the recorder's commanded angle of attack, so the flown angle (pod axis to airflow) reaches Target.</summary>
        private sealed class AoaTrim : MonoBehaviour
        {
            public ThermalEntryRecorder Rec;
            public double Target, Ki = 2;
            public double MaxOffset;

            private void FixedUpdate()
            {
                Vessel v = Rec == null ? null : Rec.Vessel;
                if (v == null || v.packed || !Rec.enabled || v.dynamicPressurekPa < 0.05) return;
                double flown = Vector3.Angle(v.ReferenceTransform.up, v.srf_velocity);
                double cmd = Rec.Aoa + Ki * (Target - flown) * TimeWarp.fixedDeltaTime;
                Rec.Aoa = Math.Max(Target - 45, Math.Min(Math.Min(179.9, Target + 45), cmd));
                MaxOffset = Math.Max(MaxOffset, Math.Abs(Rec.Aoa - Target));
            }
        }

        // FarThermalModeTests.FlyEntry with a free-form label
        private static IEnumerator Fly(TestContext ctx, Vessel v, string label, double aoa, double endAltKm, double hyper, StringBuilder summary, string holdOverride = null)
        {
            var rec = v.gameObject.AddComponent<ThermalEntryRecorder>();
            rec.Aoa = aoa;
            string hold = holdOverride ?? ctx.Param("hold", "steer");
            rec.Steer = hold == "steer";
            if (hold == "part")
            {
                double wn = ctx.ParamDouble("holdWn", 15);
                rec.Hold = a => PartApiHold(a, wn);
            }
            // none: the capsule flies whatever attitude its own aerodynamics give it
            if (hold == "none")
                rec.Hold = a => { };
            if (!rec.Steer && hold != "none")
            {
                // the force hold has no integral term, and the capsule's own trim pulls it toward heat shield first: walk
                // the commanded angle until the angle flown is the one asked for
                var trim = v.gameObject.AddComponent<AoaTrim>();
                trim.Rec = rec;
                trim.Target = aoa;
                trim.Ki = ctx.ParamDouble("trimKi", 2);
            }
            rec.Begin(v);
            bool hyperOn = hyper > 0 && Flight.HyperWarp(hyper, 1.0);
            ctx.Record($"{label}.warp", hyperOn ? $"hyper {hyper}x" : "no hyper-warp");
            // the joint between pod and shield (gap, lateral offset) and the attitude actually flown, every frame
            double maxGap = 0, maxLat = 0, minAoa = 180, maxAoa = 0, sumAoa = 0, sumQ = 0;
            float started = Time.realtimeSinceStartup;
            while (!(rec.Vessel == null || rec.Vessel.state == Vessel.State.DEAD || rec.Vessel.altitude < endAltKm * 1000 ||
                     (rec.Vessel.altitude < 40000 && rec.Vessel.mach < 1.2)))
            {
                if (Time.realtimeSinceStartup - started > 3600) throw new UITestFailure("timeout waiting for the entry to end");
                if (JointGap(rec.Vessel.parts, out double gap, out double lat))
                {
                    maxGap = Math.Max(maxGap, gap);
                    maxLat = Math.Max(maxLat, lat);
                }
                if (rec.Vessel.dynamicPressurekPa > 1)
                {
                    double flown = Vector3.Angle(rec.Vessel.ReferenceTransform.up, rec.Vessel.srf_velocity);
                    minAoa = Math.Min(minAoa, flown);
                    // q-weighted, so the heating part of the entry counts most
                    sumAoa += flown * rec.Vessel.dynamicPressurekPa;
                    sumQ += rec.Vessel.dynamicPressurekPa;
                    maxAoa = Math.Max(maxAoa, flown);
                }
                yield return null;
            }
            ctx.Record($"{label}.joint", $"pod-shield node gap max {maxGap * 100:F1} cm, lateral offset max {maxLat * 100:F1} cm");
            ctx.Record($"{label}.attitude", $"angle between the pod's axis and the airflow above 1 kPa: {minAoa:F1} to {maxAoa:F1} deg, q-weighted mean {(sumQ > 0 ? sumAoa / sumQ : double.NaN):F2} (target {aoa:F0})");
            AoaTrim trimmer = v.GetComponent<AoaTrim>();
            if (trimmer != null)
            {
                ctx.Record($"{label}.trim", $"commanded angle moved up to {trimmer.MaxOffset:F1} deg off the target");
                UnityEngine.Object.Destroy(trimmer);
            }
            if (hyperOn) Flight.HyperWarpOff();
            rec.enabled = false;
            File.WriteAllText(Path.Combine(ctx.ResultsDir, $"entry_{label}_trace.csv"), rec.Trace.ToString());
            string[] lbl = label.Split('.');
            for (int i = 0; i < rec.Parts.Count; i++)
            {
                ThermalEntryRecorder.PartPeak pk = rec.Peaks[i];
                Part p = rec.Parts[i];
                summary.Append(string.Join(",", lbl[0], lbl[1], p.partInfo.name, i, F(pk.Skin), F(p.skinMaxTemp), F(pk.Skin / p.skinMaxTemp), F(pk.Int), F(p.maxTemp), F(pk.Int / p.maxTemp),
                    F(pk.AblatorStart - Ablator(p)), F(pk.Rad), F(pk.Exp), F(pk.ConvArea), F(pk.ConvAreaMult), F(pk.ConvTempMult), F(pk.Frac), F(pk.SkinMass), F(pk.Cross))).Append('\n');
            }
            ctx.Record($"{label}.parts", string.Join("; ", Enumerable.Range(0, rec.Parts.Count).Select(i =>
                $"{rec.Parts[i].partInfo.name} skin {rec.Peaks[i].Skin:F0}/{rec.Parts[i].skinMaxTemp:F0} K int {rec.Peaks[i].Int:F0}/{rec.Parts[i].maxTemp:F0} K abl {rec.Peaks[i].AblatorStart - Ablator(rec.Parts[i]):F2} exp {rec.Peaks[i].Exp:F2} rad {rec.Peaks[i].Rad:F2} convMult {rec.Peaks[i].ConvAreaMult:F2}")));
            ctx.Record($"{label}.trajectory", rec.TrajectorySummary());
            UnityEngine.Object.Destroy(rec);
        }
    }
}
