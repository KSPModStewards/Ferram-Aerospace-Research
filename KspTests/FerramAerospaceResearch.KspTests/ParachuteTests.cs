using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Reflection;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.RealChuteLite;
using KSP.Localization;
using KSP.UI.Screens;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's parachute (RealChuteLite's RealChuteFAR, which takes the place of the stock chute module) on the stock
    /// Kerbal 1 capsule: how it waits for air thick enough to open, its action groups and staging, its cut on landing
    /// and its repack from EVA, its canopy burning away in a hypersonic flow, and the drag cubes the game renders for it.
    /// </summary>
    public static class ParachuteTests
    {
        private static readonly KSPActionParam Activate = new KSPActionParam(KSPActionGroup.None, KSPActionType.Activate);

        private static string Window => Localizer.Format("RCLGUITitle");

        /// <summary>The first line of a localized text (the window shows each line as its own label).</summary>
        private static string FirstLine(string text)
        {
            int cut = text.IndexOfAny(new[] { '\n', '\\' });
            return (cut > 0 ? text.Substring(0, cut) : text).Trim();
        }

        /// <summary>Kerbal 1's capsule off its rocket, as its own vessel and the active one; its chute in into[0].</summary>
        private static IEnumerator CapsuleOff(TestContext ctx, RealChuteFAR[] into)
        {
            yield return Wait.Seconds(5);
            Flight.IgnoreMaxTemperature(true);
            Ctrl.SetThrottle(0f);
            Vessel v = FlightGlobals.ActiveVessel;
            v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
            ctx.Step("the capsule off its rocket");
            int partsBefore = v.parts.Count;
            ModuleDecouple decoupler = v.parts.SelectMany(p => p.Modules.OfType<ModuleDecouple>()).FirstOrDefault();
            Assert.NotNull(decoupler, "Kerbal 1's decoupler");
            decoupler.Decouple();
            yield return Wait.Seconds(2);
            v = FlightGlobals.ActiveVessel;
            into[0] = v.FindPartModulesImplementing<RealChuteFAR>().FirstOrDefault();
            Assert.NotNull(into[0], "the capsule's chute is FAR's RealChute");
            Assert.True(v.parts.Count < partsBefore, $"the capsule is its own vessel ({v.parts.Count} parts of {partsBefore})");
        }

        /// <summary>
        /// The chute armed where it cannot open waits for the air. Above the atmosphere (75 km, where FAR's Mach number
        /// is nil), staging arms and stages it and it stays stowed; hiding the game's UI (F2) hides its info window and
        /// showing the UI brings it back; disarmed from its action group it is neither armed nor staged and can be
        /// deployed again. Armed again from its action group at 30 km, with its minimum pressure set above the air's,
        /// it still waits; at 3 km with the pressure back where it was it disarms itself and predeploys, and the cut
        /// action group cuts it.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 900)]
        public static IEnumerator Behaviour_ChuteArmedAboveTheAirWaitsForIt(TestContext ctx)
        {
            var holder = new RealChuteFAR[1];
            yield return CapsuleOff(ctx, holder);
            RealChuteFAR chute = holder[0];
            Vessel v = FlightGlobals.ActiveVessel;
            float savedPressure = chute.minAirPressureToOpen;
            try
            {
                ctx.Step("above the atmosphere");
                yield return FarFlightProbe.Place(Far.At(0, 75000, 0, "vacuum", speedMs: 200));
                yield return Wait.GameSeconds(1);
                v = FlightGlobals.ActiveVessel;
                FARVesselAero aero = v.GetComponent<FARVesselAero>();
                Assert.NotNull(aero, "the capsule's FARVesselAero");
                Assert.Equal(0.0, aero.MachNumber, "FAR's Mach number above the atmosphere");
                Assert.False(chute.CanDeploy || chute.PressureCheck, "no air to open in");

                ctx.Step("staged");
                for (int i = 0; i < 6 && !chute.staged; i++)
                {
                    StageManager.ActivateNextStage();
                    yield return Wait.Frames(5);
                }
                Assert.True(chute.staged && chute.armed, $"staging arms and stages the chute (staged {chute.staged}, armed {chute.armed})");
                yield return Wait.GameSeconds(1);
                Assert.True(chute.armed, "still armed with no air");
                Assert.Equal(RealChuteFAR.DeploymentStates.STOWED, chute.DeploymentState, "still stowed with no air");

                ctx.Step("F2 with the info window open");
                chute.GUIToggleWindow();
                yield return Imgui.WaitWindow(Window);
                GameEvents.onHideUI.Fire();
                Assert.True((bool)Refl.Get(chute, "hid"), "the chute's window hidden with the UI");
                yield return Imgui.WaitWindowClosed(Window);
                GameEvents.onShowUI.Fire();
                Assert.False((bool)Refl.Get(chute, "hid"), "the chute's window back with the UI");
                yield return Imgui.WaitVisible(FirstLine(Localizer.Format("RCLGUISafe")), Window);
                Refl.Set(chute, "visible", false);

                ctx.Step("disarmed");
                chute.ActionDisarm(Activate);
                Assert.False(chute.armed || chute.staged, "disarmed: neither armed nor staged");
                yield return Wait.Frames(3);
                Assert.True(chute.Events["GUIDeploy"].active, "the deploy event offered again");
                Assert.Equal(RealChuteFAR.DeploymentStates.STOWED, chute.DeploymentState, "stowed");

                ctx.Step("armed again, in air too thin for its setting");
                chute.minAirPressureToOpen = 1;
                chute.ActionDeploy(Activate);
                Assert.True(chute.staged && chute.armed, "the deploy action group arms and stages the chute");
                yield return FarFlightProbe.Place(Far.At(0, 30000, 0, "thin", speedMs: 200));
                yield return Wait.GameSeconds(2);
                ctx.Record("thin", $"{((double)Refl.Get(chute, "atmPressure")):G3} atm at {FlightGlobals.ActiveVessel.altitude:F0} m");
                Assert.True(((double)Refl.Get(chute, "atmPressure")) > 0 && !chute.PressureCheck, $"some air, thinner than the chute's setting ({((double)Refl.Get(chute, "atmPressure")):G3} atm)");
                Assert.False(chute.CanDeploy, "a stowed chute cannot open below its minimum pressure");
                Assert.True(chute.armed, "still armed below its minimum pressure");
                Assert.Equal(RealChuteFAR.DeploymentStates.STOWED, chute.DeploymentState, "still stowed below its minimum pressure");

                ctx.Step("thick air");
                chute.minAirPressureToOpen = savedPressure;
                yield return FarFlightProbe.Place(Far.At(0, 3000, 0, "thick", speedMs: 80));
                yield return Wait.Until(() => !chute.armed, 10, "the chute to disarm itself in thick air", shot: false);
                yield return Wait.Until(() => chute.DeploymentState == RealChuteFAR.DeploymentStates.PREDEPLOYED, 10, "the chute predeployed", shot: false);
                Assert.True(chute.CanDeploy, "a predeployed chute can stay open");

                ctx.Step("cut from its action group");
                chute.ActionCut(Activate);
                Assert.Equal(RealChuteFAR.DeploymentStates.CUT, chute.DeploymentState, "the cut action group cuts the chute");
                Assert.False(chute.CanDeploy, "a cut chute cannot deploy");
            }
            finally
            {
                chute.minAirPressureToOpen = savedPressure;
                Refl.Find(typeof(RealChuteFAR), "visible")?.SetValue(chute, false);
                if ((bool)Refl.Get(chute, "hid"))
                    GameEvents.onShowUI.Fire();
            }
        }

        /// <summary>
        /// The capsule comes down in the sea under its chute: once it floats nearly still the chute cuts itself (no cut
        /// asked for) and cannot deploy again. Its kerbal out on EVA in the water (FAR's model of the kerbal starts
        /// splashed), repacks it: stowed again, one spare canopy fewer, the packed drag cube back in place of the
        /// deployed one. In a career a kerbal who is not an engineer of level 1 is refused; a stowed chute is not repacked.
        /// FAR's start state for the modules it adds to a kerbal is the vessel's situation, for each situation.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 900)]
        public static IEnumerator Behaviour_ChuteCutsItselfOnSplashdownAndIsRepacked(TestContext ctx)
        {
            var holder = new RealChuteFAR[1];
            yield return CapsuleOff(ctx, holder);
            RealChuteFAR chute = holder[0];
            Vessel v = FlightGlobals.ActiveVessel;
            float savedAltitude = chute.deployAltitude;
            try
            {
                ctx.Step("under the canopy over the sea");
                yield return FarFlightProbe.Place(Far.At(0, 1000, 0, "drop", speedMs: 30));
                v = FlightGlobals.ActiveVessel;
                chute.deployAltitude = (float)(v.altitude + 500);
                chute.GUIDeploy();
                yield return Wait.Until(() => chute.DeploymentState == RealChuteFAR.DeploymentStates.DEPLOYED, 60, "the chute deployed", shot: false);
                yield return Wait.GameSeconds(1 / chute.deploymentSpeed + 2);
                // set down just above the water under the open canopy, rather than wait out the descent
                v.SetPosition(v.vesselTransform.position - (Vector3d)v.upAxis * (v.altitude - 15));
                yield return Wait.Until(() => v.Splashed, 30, "the capsule in the water", shot: false);
                ctx.Record("splash", $"{v.horizontalSrfSpeed:F2} m/s horizontal");
                yield return Wait.Until(() => chute.DeploymentState == RealChuteFAR.DeploymentStates.CUT, 60, "the chute to cut itself in the water",
                                        shot: false, detail: () => $"{chute.DeploymentState}, {v.horizontalSrfSpeed:F2} m/s, splashed {v.Splashed}");
                Assert.True(chute.GroundStop, $"the capsule floating still ({v.horizontalSrfSpeed:F2} m/s)");
                Assert.False(chute.CanDeploy, "a cut chute cannot deploy");
                Assert.True(!chute.staged && !chute.armed && chute.currentArea == 0, "the cut chute unstaged, with no canopy");

                ctx.Step("the kerbal out on EVA");
                Part pod = v.parts.FirstOrDefault(p => p.protoModuleCrew.Count > 0);
                Assert.NotNull(pod, "the crewed capsule");
                ProtoCrewMember crew = pod.protoModuleCrew[0];
                Assert.True(FlightEVA.fetch.spawnEVA(crew, pod, pod.airlock, true), "the kerbal went out on EVA");
                yield return Wait.Until(() => FlightGlobals.ActiveVessel != null && FlightGlobals.ActiveVessel.isEVA, 30, "the kerbal on EVA", shot: false);
                Vessel eva = FlightGlobals.ActiveVessel;
                FARVesselAero evaAero = eva.GetComponent<FARVesselAero>();
                Assert.NotNull(evaAero, "FAR's vessel module on the kerbal");
                yield return Wait.Until(() => eva.Splashed, 20, "the kerbal in the water", shot: false);

                // the start state FAR gives the geometry and aero modules it adds to a kerbal (or a flag): the vessel's
                // situation, read as each would be when its vessel is loaded in it
                MethodInfo startState = typeof(FARVesselAero).GetMethod("StartState", BindingFlags.NonPublic | BindingFlags.Instance);
                Vessel.Situations situation = eva.situation;
                var states = new Dictionary<Vessel.Situations, PartModule.StartState>();
                try
                {
                    foreach (Vessel.Situations s in Enum.GetValues(typeof(Vessel.Situations)))
                    {
                        eva.situation = s;
                        states[s] = (PartModule.StartState)startState.Invoke(evaAero, null);
                    }
                }
                finally
                {
                    eva.situation = situation;
                }

                ctx.Record("startStates", string.Join(", ", states.Select(kv => $"{kv.Key}: {kv.Value}")));
                Assert.Equal(PartModule.StartState.PreLaunch | PartModule.StartState.Landed, states[Vessel.Situations.PRELAUNCH], "on the pad");
                Assert.Equal(PartModule.StartState.Landed, states[Vessel.Situations.LANDED], "landed");
                Assert.Equal(PartModule.StartState.Splashed, states[Vessel.Situations.SPLASHED], "splashed");
                Assert.Equal(PartModule.StartState.Flying, states[Vessel.Situations.FLYING], "flying");
                Assert.Equal(PartModule.StartState.SubOrbital, states[Vessel.Situations.SUB_ORBITAL], "sub-orbital");
                Assert.Equal(PartModule.StartState.Orbital, states[Vessel.Situations.ORBITING], "orbiting");
                Assert.Equal(PartModule.StartState.Orbital, states[Vessel.Situations.ESCAPING], "escaping");
                Assert.Equal(PartModule.StartState.Docked, states[Vessel.Situations.DOCKED], "docked");

                ctx.Step("repacked");
                int spares = chute.chuteCount;
                yield return Wait.Until(() => chute.CanRepack, 20, "the chute repackable from EVA", shot: false,
                                        detail: () => $"capsule {chute.vessel.horizontalSrfSpeed:F2} m/s, splashed {chute.vessel.Splashed}");
                Assert.True(RealChuteFAR.CanRepackCareer, "a sandbox kerbal may repack");
                // in a career only an engineer of level 1 or more may
                ProtoCrewMember kerbal = eva.GetVesselCrew()[0];
                if (kerbal.experienceTrait.Title == "Engineer" && kerbal.experienceLevel >= 1)
                    ctx.Record("careerRefusal", $"skipped: {kerbal.name} is a level {kerbal.experienceLevel} engineer");
                else
                {
                    Game.Modes mode = HighLogic.CurrentGame.Mode;
                    try
                    {
                        HighLogic.CurrentGame.Mode = Game.Modes.CAREER;
                        Assert.False(RealChuteFAR.CanRepackCareer, $"a career {kerbal.experienceTrait.Title} of level {kerbal.experienceLevel} may not repack");
                        chute.GUIRepack();
                    }
                    finally
                    {
                        HighLogic.CurrentGame.Mode = mode;
                    }

                    Assert.Equal(RealChuteFAR.DeploymentStates.CUT, chute.DeploymentState, "the chute refused, still cut");
                    Assert.Equal(spares, chute.chuteCount, "and no spare used");
                }

                chute.GUIRepack();
                Assert.Equal(RealChuteFAR.DeploymentStates.STOWED, chute.DeploymentState, "the repacked chute stowed");
                Assert.Equal(spares - 1, chute.chuteCount, "one spare canopy used");
                float Weight(string cube) => chute.part.DragCubes.Cubes.Where(c => c.Name == cube).Select(c => c.Weight).FirstOrDefault();
                Assert.Near(1, Weight("PACKED"), 1e-3, "the packed drag cube's weight");
                Assert.Near(0, Weight("RCDEPLOYED"), 1e-3, "the deployed drag cube's weight");
                Assert.False(chute.CanRepack, "a stowed chute is not repacked again");
                chute.GUIRepack();
                Assert.Equal(spares - 1, chute.chuteCount, "repacking it anyway uses no spare");
            }
            finally
            {
                chute.deployAltitude = savedAltitude;
            }
        }

        /// <summary>
        /// The chute's deployment safety in a hot flow, at 35 km: safe at Mach 1.5, where the flow is cooler than the
        /// canopy's limit, and risky then dangerous as the flow heats, the info window saying so in yellow and in red.
        /// Deployed at Mach 3 the canopy heats past its limit and burns away: the chute cuts itself.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 900)]
        public static IEnumerator Behaviour_ChuteOpenedTooFastBurnsAway(TestContext ctx)
        {
            var holder = new RealChuteFAR[1];
            yield return CapsuleOff(ctx, holder);
            RealChuteFAR chute = holder[0];
            float savedPressure = chute.minAirPressureToOpen;
            try
            {
                // the air at 35 km is thinner than the chute's default minimum
                chute.minAirPressureToOpen = 0.001f;
                chute.GUIToggleWindow();
                yield return Imgui.WaitWindow(Window);

                double[] machs = { 1.5, 1.65, 3, 8 };
                var states = new int[machs.Length];
                for (int i = 0; i < machs.Length; i++)
                {
                    ctx.Step($"Mach {machs[i]}");
                    yield return FarFlightProbe.Place(Far.At(machs[i], 35000, 0));
                    yield return Wait.GameSeconds(0.3);
                    Vessel v = FlightGlobals.ActiveVessel;
                    object state = Refl.Get(chute, "safeState");
                    states[i] = Convert.ToInt32(state);
                    ctx.Record($"mach{machs[i]}", $"{state}: flow {v.externalTemperature:F0} K, Mach {v.mach:F2}");
                    string label = state.ToString() == "RISKY" ? "RCLGUIRisky" : state.ToString() == "DANGEROUS" ? "RCLGUIDang" : "RCLGUISafe";
                    yield return Imgui.WaitVisible(FirstLine(Localizer.Format(label)), Window);
                }

                Assert.Equal("SAFE", Enum.GetName(Refl.Get(chute, "safeState").GetType(), states[0]), "safe to deploy at Mach 1.5");
                Assert.True(states[1] > states[0], "no longer safe at Mach 1.65, the flow just hotter than the canopy's limit");
                Assert.Equal("DANGEROUS", Enum.GetName(Refl.Get(chute, "safeState").GetType(), states[machs.Length - 1]), "dangerous to deploy at Mach 8");
                for (int i = 1; i < machs.Length; i++)
                    Assert.True(states[i] >= states[i - 1], $"no safer at Mach {machs[i]} than at Mach {machs[i - 1]}");
                Refl.Set(chute, "visible", false);

                // at Mach 3 the canopy heats over some frames (at Mach 8 within one)
                ctx.Step("deployed at Mach 3");
                yield return FarFlightProbe.Place(Far.At(3, 35000, 0));
                chute.GUIDeploy();
                double hottest = 0;
                bool sawOpen = false;
                int openFrames = 0;
                var fixedWait = new WaitForFixedUpdate();
                float deadline = Time.realtimeSinceStartup + 120;
                while (chute.DeploymentState != RealChuteFAR.DeploymentStates.CUT && Time.realtimeSinceStartup < deadline)
                {
                    yield return fixedWait;
                    if (chute.IsDeployed)
                    {
                        sawOpen = true;
                        openFrames++;
                        hottest = Math.Max(hottest, chute.chuteTemperature);
                    }
                }

                ctx.Record("burn", $"{chute.DeploymentState}, hottest {hottest:F0} K (limit {RealChuteFAR.maxTemp} K), {openFrames} frames open, Mach {FlightGlobals.ActiveVessel.mach:F2}");
                Assert.True(sawOpen, "the chute opened");
                Assert.Equal(RealChuteFAR.DeploymentStates.CUT, chute.DeploymentState, "the canopy burnt away");
                Assert.True(hottest > RealChuteFAR.startTemp + 50, $"the canopy heated before it went ({hottest:F0} K)");
                Assert.False(chute.staged, "the burnt chute unstaged");
            }
            finally
            {
                chute.minAirPressureToOpen = savedPressure;
                Refl.Find(typeof(RealChuteFAR), "visible")?.SetValue(chute, false);
            }
        }

        /// <summary>
        /// The game rendering the Mk16 chute's drag cubes from the part (as it does for a part with none in its
        /// database): FAR's chute module names its five cubes and poses the part for each, so the deployed canopy's
        /// cube has by far the most area, the predeployed one less, the packed and stowed ones least, and the cube with
        /// the cap gone and no canopy out no more than the packed one.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.MainMenu, TimeoutSec = 600)]
        public static IEnumerator Load_ChuteDragCubesRenderFromItsCanopy(TestContext ctx)
        {
            Part prefab = PartLoader.getPartInfoByName("parachuteSingle")?.partPrefab;
            Assert.NotNull(prefab, "the Mk16 parachute");
            RealChuteFAR module = prefab.FindModuleImplementing<RealChuteFAR>();
            Assert.NotNull(module, "the Mk16's RealChuteFAR");
            Assert.True(module.IsMultipleCubesActive && !module.UsesProceduralDragCubes(), "fixed multiple drag cubes");
            string[] names = module.GetDragCubeNames();
            Assert.Equal("DEPLOYED,PACKED,RCDEPLOYED,SEMIDEPLOYED,STOWED", string.Join(",", names.OrderBy(n => n)), "the chute's cube names");

            // the prefab's cubes as they were, put back after (the render loads its result onto the part)
            List<DragCube> saved = prefab.DragCubes.Cubes.ToList();
            var weights = saved.Select(c => c.Weight).ToList();
            var node = new ConfigNode("DRAG_CUBE");
            using var exceptions = new ExceptionLog();
            try
            {
                yield return DragCubeSystem.Instance.SetupDragCubeCoroutine(prefab, node);
            }
            finally
            {
                prefab.DragCubes.Cubes.Clear();
                prefab.DragCubes.Cubes.AddRange(saved);
                for (int i = 0; i < saved.Count; i++)
                    saved[i].Weight = weights[i];
            }

            var area = new Dictionary<string, double>();
            foreach (string line in node.GetValues("cube"))
            {
                string[] f = line.Split(',');
                // name, then area, drag coefficient and depth for each of the six faces, then the centre and size
                double total = 0;
                for (int face = 0; face < 6; face++)
                    total += double.Parse(f[1 + 3 * face], CultureInfo.InvariantCulture);
                area[f[0].Trim()] = total;
            }

            ctx.Record("areas", string.Join(", ", area.Select(a => $"{a.Key} {a.Value:F3}")));
            exceptions.AssertNone("rendering the chute's drag cubes");
            foreach (string n in names)
                Assert.True(area.ContainsKey(n), $"a {n} cube rendered");
            Assert.True(area["DEPLOYED"] > 3 * area["SEMIDEPLOYED"], $"the open canopy's cube far larger than the predeployed one ({area["DEPLOYED"]:F2}, {area["SEMIDEPLOYED"]:F2} m2)");
            Assert.True(area["SEMIDEPLOYED"] > 1.2 * area["PACKED"], $"the predeployed canopy's cube larger than the packed one ({area["SEMIDEPLOYED"]:F2}, {area["PACKED"]:F2} m2)");
            Assert.Near(area["PACKED"], area["STOWED"], 0.01 * area["PACKED"], "stowed is packed");
            Assert.True(area["RCDEPLOYED"] <= area["PACKED"] * 1.01, $"no cap and no canopy out: no larger than packed ({area["RCDEPLOYED"]:F3}, {area["PACKED"]:F3} m2)");
        }
    }
}
