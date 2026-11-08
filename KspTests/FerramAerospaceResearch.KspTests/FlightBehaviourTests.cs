using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using FerramAerospaceResearch.RealChuteLite;
using FerramAerospaceResearch.Settings;
using KSP.Localization;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's aerodynamics in flight on stock crafts, against what real aircraft do: wings stall past their maximum angle
    /// and lose lift, the stick moves the elevons and the pitching moment with them, flaps add lift, spoilers dump it and
    /// add drag, drag rises and the lift slope falls supersonic, and a parachute lowers a capsule at the sink rate its
    /// canopy gives.
    /// </summary>
    public static class FlightBehaviourTests
    {

        /// <summary>The aero force held at a condition, as areas: lift / q and drag / q (m^2), pitch moment / q (m^3).</summary>
        private sealed class Aero
        {
            public double LiftS, DragS, PitchS, Lift, Drag, Q;
            public int N;

            public override string ToString() =>
                $"L/q {LiftS:F4} m2, D/q {DragS:F4} m2, M/q {PitchS:F4} m3 (lift {Lift:F2} kN, drag {Drag:F2} kN, q {Q:F3} kPa, {N} frames)";
        }

        /// <summary>Holds the vessel at c (placed there first unless place is off) and averages the force FAR applies.</summary>
        private static IEnumerator Measure(FarFlightProbe.Condition c, Aero into, bool place = true, int settle = 25, int frames = 50)
        {
            if (place)
                yield return FarFlightProbe.Place(c);
            var samples = new List<FarFlightProbe.Sample>();
            yield return FarFlightProbe.Hold(c, settle, frames, samples, predict: false);
            Assert.True(samples.Count >= frames / 2, $"{c}: frames measured ({samples.Count})");
            foreach (FarFlightProbe.Sample s in samples)
                Assert.True(s.Flight.All(x => !double.IsNaN(x) && !double.IsInfinity(x)) && s.QkPa > 0,
                            $"{c}: a real force and moment every frame (drag {s.Flight[0]}, lift {s.Flight[1]}, pitch {s.Flight[3]}, q {s.QkPa})");
            into.N = samples.Count;
            into.LiftS = samples.Average(s => s.Flight[1] / s.QkPa);
            into.DragS = samples.Average(s => s.Flight[0] / s.QkPa);
            into.PitchS = samples.Average(s => s.Flight[3] / s.QkPa);
            into.Lift = samples.Average(s => s.Flight[1]);
            into.Drag = samples.Average(s => s.Flight[0]);
            into.Q = samples.Average(s => s.QkPa);
        }

        /// <summary>The vessel's horizontal control surfaces: the elevons, flaps and canards (not the rudder).</summary>
        private static List<FARControllableSurface> HorizontalSurfaces(Vessel v)
        {
            Vector3 belly = v.ReferenceTransform.forward;
            return v.parts.SelectMany(p => p.Modules.OfType<FARControllableSurface>())
                    .Where(s => Math.Abs(Vector3.Dot(s.part.partTransform.forward, belly)) > 0.7)
                    .ToList();
        }

        /// <summary>A control surface's flap and spoiler settings, to put back.</summary>
        private sealed class SurfaceState
        {
            private readonly FARControllableSurface s;
            private readonly bool isFlap, isSpoiler, brake;
            private readonly int level;

            public SurfaceState(FARControllableSurface s)
            {
                this.s = s;
                isFlap = s.isFlap;
                isSpoiler = s.isSpoiler;
                brake = s.brake;
                level = s.flapDeflectionLevel;
            }

            public void Restore()
            {
                s.isFlap = isFlap;
                s.isSpoiler = isSpoiler;
                s.brake = brake;
                s.SetDeflection(level);
            }
        }

        private static string Deflections(IEnumerable<FARControllableSurface> surfaces) =>
            string.Join(", ", surfaces.Select(s => $"{s.part.partInfo.name} {s.ControlDeflection:F2}"));

        /// <summary>
        /// An angle-of-attack sweep at 80 m/s, 4 to 48 deg: lift rises, peaks once the wings pass their maximum angle,
        /// then falls away with the stall; FAR's stall fraction is nil in cruise and most of the wing past the peak, and
        /// the flight window's status goes from Nominal to Large-Scale Stall.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_WingsStallPastTheirMaxAngle(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            yield return FlightTests.Airborne(Far.At(0.25, 3000, 4));
            Vessel v = FlightGlobals.ActiveVessel;
            FlightGUI gui = Far.FlightGui(v);
            List<FARWingAerodynamicModel> wings = v.parts.SelectMany(p => p.Modules.OfType<FARWingAerodynamicModel>()).ToList();

            int[] alphas = { 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48 };
            var liftS = new double[alphas.Length];
            var stall = new double[alphas.Length];
            var status = new string[alphas.Length];
            for (int i = 0; i < alphas.Length; ++i)
            {
                ctx.Step($"held at {alphas[i]} deg");
                var a = new Aero();
                yield return Measure(Far.At(0.25, 3000, alphas[i]), a);
                liftS[i] = a.LiftS;
                stall[i] = FARAPI.VesselStallFrac(v);
                status[i] = Far.FlightStatus(gui);
                ctx.Record($"alpha{alphas[i]}", $"{a}; stall {stall[i]:F3} (wing mean {FarApi.WingStallMean(v):F3}), status '{status[i]}'");
            }

            double aoaMaxMin = wings.Min(w => w.GetRawAoAmax()) * 180 / Math.PI, aoaMaxMax = wings.Max(w => w.GetRawAoAmax()) * 180 / Math.PI;
            ctx.Record("wingMaxAoA", $"{aoaMaxMin:F1} to {aoaMaxMax:F1} deg");
            int last = alphas.Length - 1;
            int peak = Array.IndexOf(liftS, liftS.Max());
            int onset = Array.FindIndex(stall, s => s > 0.05);
            ctx.Record("peak", $"{alphas[peak]} deg, L/q {liftS[peak]:F3}; stall from {(onset < 0 ? "never" : alphas[onset] + " deg")}");

            Assert.True(stall[0] < 0.005, $"no stall at 4 deg ({stall[0]:F4})");
            Assert.Equal(Localizer.Format("FARFlightStatus0"), status[0], "the status at 4 deg");
            Assert.True(liftS[1] > liftS[0], $"lift rises with the angle of attack below the stall ({liftS[0]:F3} at 4 deg, {liftS[1]:F3} at 8)");
            Assert.True(onset > 1, $"the wings stall only past 8 deg (from {(onset < 0 ? "never" : alphas[onset] + " deg")})");
            Assert.True(peak > 0 && peak < last, $"lift peaks inside the sweep (at {alphas[peak]} deg)");
            Assert.True(stall[last] > 0.5, $"most of the wing stalled at {alphas[last]} deg ({stall[last]:F3})");
            Assert.True(liftS[last] < 0.8 * liftS[peak], $"lift falls past the stall: L/q {liftS[last]:F3} at {alphas[last]} deg, {liftS[peak]:F3} at the peak");
            Assert.Equal(Localizer.Format("FARFlightStatus1"), status[last], $"the status at {alphas[last]} deg");
        }

        /// <summary>
        /// Full nose-up and nose-down stick, held at 3 deg at Mach 0.5: every elevon deflects most of its travel, opposite
        /// ways for the two, and the pitching moment FAR applies moves nose-up and nose-down of the neutral one.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_PitchInputMovesTheElevons(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0.5, 3000, 3);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            List<FARControllableSurface> elevons = HorizontalSurfaces(v).Where(s => s.pitchaxis > 0).ToList();
            Assert.True(elevons.Count >= 2, $"the plane's elevons ({elevons.Count})");

            float[] inputs = { 0, 1, -1 };
            var moment = new double[inputs.Length];
            var deflection = new double[inputs.Length][];
            try
            {
                Ctrl.Attach();
                for (int i = 0; i < inputs.Length; ++i)
                {
                    ctx.Step($"pitch input {inputs[i]}");
                    Ctrl.Set(pitch: inputs[i], yaw: 0, roll: 0);
                    yield return FarFlightProbe.Place(c);
                    yield return FarFlightProbe.SettleSurfaces(c);
                    var a = new Aero();
                    yield return Measure(c, a, place: false);
                    moment[i] = a.PitchS;
                    deflection[i] = elevons.Select(s => s.ControlDeflection).ToArray();
                    ctx.Record($"pitch{inputs[i]}", $"{a}; {Deflections(elevons)}");
                }
            }
            finally
            {
                Ctrl.Detach();
            }

            for (int k = 0; k < elevons.Count; ++k)
            {
                double max = Math.Abs(elevons[k].maxdeflect);
                string name = elevons[k].part.partInfo.name;
                Assert.True(Math.Abs(deflection[0][k]) < 0.5, $"{name} centred with the stick centred ({deflection[0][k]:F2} deg)");
                Assert.True(Math.Abs(deflection[1][k]) > 0.3 * max && Math.Abs(deflection[2][k]) > 0.3 * max,
                            $"{name} deflects with full stick ({deflection[1][k]:F2} and {deflection[2][k]:F2} deg of {max})");
                Assert.True(Math.Sign(deflection[1][k]) == -Math.Sign(deflection[2][k]),
                            $"{name} deflects opposite ways for nose up and nose down ({deflection[1][k]:F2}, {deflection[2][k]:F2})");
            }

            Assert.True(moment[1] > moment[0], $"nose-up stick adds nose-up moment (M/q {moment[1]:F4} against {moment[0]:F4} m3)");
            Assert.True(moment[2] < moment[0], $"nose-down stick adds nose-down moment (M/q {moment[2]:F4} against {moment[0]:F4} m3)");
            double up = moment[1] - moment[0], down = moment[0] - moment[2];
            Assert.InRange(0.5, 2, up / down, $"the two pitch authorities are alike ({up:F4} up, {down:F4} down)");
        }

        /// <summary>
        /// Full flap (FARAPI's flap setting, which stops at 3) at 4 deg and Mach 0.3: the flaps go down their full travel
        /// and the plane makes more lift, and more drag, at the same angle of attack; flaps up again puts the setting back.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_FlapsAddLift(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0.3, 2000, 4);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            List<FARControllableSurface> flaps = HorizontalSurfaces(v);
            Assert.True(flaps.Count >= 2, $"horizontal control surfaces to use as flaps ({flaps.Count})");
            List<SurfaceState> saved = flaps.Select(s => new SurfaceState(s)).ToList();
            double savedTime = FARAeroData.ControlSurfaceTimeConstantFlap;
            try
            {
                ctx.Step("flaps up");
                var clean = new Aero();
                yield return Measure(c, clean);
                ctx.Record("clean", clean);

                ctx.Step("flaps to full");
                foreach (FARControllableSurface s in flaps)
                {
                    s.isSpoiler = false;
                    s.isFlap = true;
                }
                for (int i = 0; i < 4; ++i)
                    FARAPI.VesselIncreaseFlapDeflection(v);
                Assert.Equal(3, FARAPI.VesselFlapSetting(v), "the flap setting after four steps up (it stops at 3)");
                // flaps run at a fixed rate, their full travel in ctrlSurfTimeConstantFlap (10 s); quicker for the test
                FARAeroData.ControlSurfaceTimeConstantFlap = 0.5;
                yield return FarFlightProbe.Place(c);
                yield return FarFlightProbe.SettleSurfaces(c);
                ctx.Record("deflections", Deflections(flaps));
                foreach (FARControllableSurface s in flaps)
                    Assert.Near(Math.Abs(s.maxdeflectFlap), Math.Abs(s.ControlDeflection), 0.1 * Math.Abs(s.maxdeflectFlap) + 0.2,
                                $"{s.part.partInfo.name} at full flap");
                var down = new Aero();
                yield return Measure(c, down);
                ctx.Record("flaps", down);
                Assert.True(down.LiftS > 1.05 * clean.LiftS, $"more lift with flaps at the same angle of attack ({down.LiftS:F3} m2 against {clean.LiftS:F3})");
                Assert.True(down.DragS > clean.DragS, $"more drag with flaps ({down.DragS:F4} m2 against {clean.DragS:F4})");

                ctx.Step("flaps up again");
                for (int i = 0; i < 4; ++i)
                    FARAPI.VesselDecreaseFlapDeflection(v);
                Assert.Equal(0, FARAPI.VesselFlapSetting(v), "the flap setting after four steps down");
            }
            finally
            {
                FARAeroData.ControlSurfaceTimeConstantFlap = savedTime;
                foreach (SurfaceState s in saved)
                    s.Restore();
            }
        }

        /// <summary>
        /// Spoilers (FARAPI's spoiler switch) at 4 deg and Mach 0.3: the surfaces go trailing edge up, the plane loses
        /// lift and gains drag at the same angle of attack, and the switch reads back.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_SpoilersDumpLift(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0.3, 2000, 4);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            List<FARControllableSurface> spoilers = HorizontalSurfaces(v);
            Assert.True(spoilers.Count >= 2, $"horizontal control surfaces to use as spoilers ({spoilers.Count})");
            List<SurfaceState> saved = spoilers.Select(s => new SurfaceState(s)).ToList();
            try
            {
                ctx.Step("spoilers in");
                foreach (FARControllableSurface s in spoilers)
                {
                    s.isFlap = false;
                    s.isSpoiler = true;
                }
                FARAPI.VesselSetSpoilers(v, false);
                yield return FarFlightProbe.Place(c);
                yield return FarFlightProbe.SettleSurfaces(c);
                var clean = new Aero();
                yield return Measure(c, clean, place: false);
                ctx.Record("clean", $"{clean}; {Deflections(spoilers)}");
                Assert.False(FARAPI.VesselSpoilerSetting(v), "the spoiler switch reads off");

                ctx.Step("spoilers out");
                FARAPI.VesselSetSpoilers(v, true);
                Assert.True(FARAPI.VesselSpoilerSetting(v), "the spoiler switch reads on");
                yield return FarFlightProbe.Place(c);
                yield return FarFlightProbe.SettleSurfaces(c);
                ctx.Record("deflections", Deflections(spoilers));
                foreach (FARControllableSurface s in spoilers)
                    Assert.Near(Math.Abs(s.maxdeflectFlap), Math.Abs(s.ControlDeflection), 0.1 * Math.Abs(s.maxdeflectFlap) + 0.2,
                                $"{s.part.partInfo.name} spoiler out");
                var dumped = new Aero();
                yield return Measure(c, dumped, place: false);
                ctx.Record("spoilers", dumped);
                Assert.True(dumped.LiftS < 0.95 * clean.LiftS, $"spoilers dump lift ({dumped.LiftS:F3} m2 against {clean.LiftS:F3})");
                Assert.True(dumped.DragS > 1.02 * clean.DragS, $"spoilers add drag ({dumped.DragS:F4} m2 against {clean.DragS:F4})");
            }
            finally
            {
                foreach (SurfaceState s in saved)
                    s.Restore();
            }
        }

        /// <summary>
        /// The same plane at Mach 0.5 (3 km) and Mach 2 (12 km), at 0 and 4 deg: wave drag raises the drag area well
        /// above the subsonic one, the lift slope falls (as 1 / sqrt(M^2 - 1) does), and FAR's Mach number is the game's.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_SupersonicDragRiseAndLiftSlope(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            yield return FlightTests.Airborne(Far.At(0.5, 3000, 0));
            Vessel v = FlightGlobals.ActiveVessel;
            FARVesselAero vesselAero = v.GetComponent<FARVesselAero>();
            Assert.NotNull(vesselAero, "the vessel's FARVesselAero");

            var sub0 = new Aero();
            var sub4 = new Aero();
            var sup0 = new Aero();
            var sup4 = new Aero();
            ctx.Step("Mach 0.5");
            yield return Measure(Far.At(0.5, 3000, 0), sub0);
            yield return Measure(Far.At(0.5, 3000, 4), sub4);
            double subMachFar = vesselAero.MachNumber, subMach = v.mach;
            ctx.Step("Mach 2");
            yield return Measure(Far.At(2, 12000, 0), sup0);
            yield return Measure(Far.At(2, 12000, 4), sup4);
            double supMachFar = vesselAero.MachNumber, supMach = v.mach;
            ctx.Record("sub", $"0 deg {sub0}; 4 deg {sub4}; Mach {subMach:F3} (FAR {subMachFar:F3})");
            ctx.Record("sup", $"0 deg {sup0}; 4 deg {sup4}; Mach {supMach:F3} (FAR {supMachFar:F3})");

            Assert.Near(subMach, subMachFar, 0.02 * subMach, "FAR's Mach number subsonic");
            Assert.Near(supMach, supMachFar, 0.02 * supMach, "FAR's Mach number supersonic");
            Assert.True(sup0.DragS > 1.3 * sub0.DragS, $"drag area at Mach 2 ({sup0.DragS:F4} m2) well above Mach 0.5's ({sub0.DragS:F4})");
            double slopeSub = (sub4.LiftS - sub0.LiftS) / 4, slopeSup = (sup4.LiftS - sup0.LiftS) / 4;
            ctx.Record("liftSlope", $"Mach 0.5 {slopeSub:F4} m2/deg, Mach 2 {slopeSup:F4}");
            Assert.True(slopeSub > 0 && slopeSup > 0, "lift rises with the angle of attack at both speeds");
            Assert.True(slopeSup < slopeSub, $"the lift slope falls supersonic ({slopeSup:F4} against {slopeSub:F4} m2/deg)");
        }

        /// <summary>
        /// A wing past its stress limit tears off. An outboard wing panel's limit is set below the load it carries in
        /// cruise (the per-part limit FAR takes from the part's stress template at start, as a weaker template would
        /// give it): within a few physics frames FAR decouples it from the plane, the flight log records the
        /// aerodynamic failure, and the parts still within their limits stay on.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_OverstressedWingTearsOff(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            yield return FlightTests.Airborne(Far.At(0.5, 3000, 4));
            Vessel v = FlightGlobals.ActiveVessel;
            Assert.True(FARDebugValues.allowStructuralFailures, "structural failures on (FAR's default)");

            // an outboard panel: a wing with nothing attached to it, the largest such
            Part wing = v.parts.Where(p => p.parent != null && p.children.Count == 0 && p.Modules.Contains<FARWingAerodynamicModel>())
                           .OrderByDescending(p => p.Modules.GetModule<FARWingAerodynamicModel>().S).FirstOrDefault();
            Assert.NotNull(wing, "an outboard wing panel");
            FARAeroPartModule aero = wing.Modules.GetModule<FARAeroPartModule>();
            int partsBefore = v.parts.Count, logBefore = FlightLogger.eventLog.Count;
            ctx.Record("wing", $"{wing.partInfo.title} ({wing.partInfo.name}), limits Y {Refl.Get(aero, "partForceMaxY")}, XZ {Refl.Get(aero, "partForceMaxXZ")} kN");

            ctx.Step("The panel's stress limit below its cruise load");
            Refl.Set(aero, "partStressMaxXZ", 0.0);
            Refl.Set(aero, "partForceMaxXZ", 0.0);
            yield return Wait.Until(() => wing.vessel != v, 10, "the overstressed panel to come off", shot: false);

            ctx.Record("after", $"{v.parts.Count} parts (from {partsBefore}); the panel on {wing.vessel?.vesselName}");
            Assert.Equal(partsBefore - 1, v.parts.Count, "only the panel came off");
            List<string> logged = FlightLogger.eventLog.Skip(logBefore).ToList();
            ctx.Record("flightLog", string.Join(" | ", logged));
            Assert.True(logged.Any(m => m.Contains(wing.partInfo.title)), $"the flight log records the panel's failure ({string.Join(" | ", logged)})");
        }

        /// <summary>
        /// A stock capsule (Kerbal 1's, off its rocket) dropped at 3 km: falling stowed it picks up speed; its chute,
        /// deployed from the part menu, predeploys, then opens fully at the altitude set, to the full canopy area, and
        /// the capsule settles at the sink rate where the canopy's drag (Cd 1) carries its weight; the chute's info
        /// window shows the canopy; cut, the capsule falls fast again.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 900)]
        public static IEnumerator Behaviour_ParachuteLowersTheCapsule(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            Flight.IgnoreMaxTemperature(true);
            Ctrl.SetThrottle(0f);
            Vessel v = FlightGlobals.ActiveVessel;
            v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);

            ctx.Step("the capsule off its rocket");
            int partsBefore = v.parts.Count;
            Part decoupler = v.parts.FirstOrDefault(p => p.Modules.OfType<ModuleDecouple>().Any());
            Assert.NotNull(decoupler, "Kerbal 1's decoupler");
            decoupler.Modules.OfType<ModuleDecouple>().First().Decouple();
            yield return Wait.Seconds(2);
            v = FlightGlobals.ActiveVessel;
            RealChuteFAR chute = v.FindPartModulesImplementing<RealChuteFAR>().FirstOrDefault();
            Assert.NotNull(chute, "the capsule's chute is FAR's RealChute");
            Assert.True(v.parts.Count < partsBefore, $"the capsule is its own vessel ({v.parts.Count} parts of {partsBefore})");
            ctx.Record("capsule", $"{v.parts.Count} parts, {v.GetTotalMass():F3} t");

            ctx.Step("falling stowed");
            FarFlightProbe.Condition c = Far.At(0, 3000, 0, "drop", speedMs: 60);
            yield return FarFlightProbe.Place(c);
            yield return Wait.GameSeconds(6);
            v = FlightGlobals.ActiveVessel;
            Assert.Equal(RealChuteFAR.DeploymentStates.STOWED, chute.DeploymentState, "the chute stowed until deployed");
            Assert.True(chute.PressureCheck && chute.CanDeploy, "enough air to deploy");
            double stowedSpeed = v.srfSpeed;
            ctx.Record("stowed", $"{stowedSpeed:F1} m/s, falling {-v.verticalSpeed:F1} m/s at {v.altitude:F0} m");
            Assert.True(-v.verticalSpeed > 30, $"falling stowed ({-v.verticalSpeed:F1} m/s down)");

            float savedAltitude = chute.deployAltitude;
            string window = Localizer.Format("RCLGUITitle");
            try
            {
                ctx.Step("deploy: predeployed, then open");
                // open fully soon after predeploying (the predeployment takes 1 / semiDeploymentSpeed s at least)
                chute.deployAltitude = (float)(v.altitude - 150);
                chute.GUIDeploy();
                Assert.True(chute.staged && chute.armed, "deploying arms and stages the chute");
                yield return Wait.Until(() => chute.DeploymentState == RealChuteFAR.DeploymentStates.PREDEPLOYED, 10, "the chute predeployed", shot: false);
                yield return Wait.GameSeconds(0.5 / chute.semiDeploymentSpeed);
                ctx.Record("predeployed", $"area {chute.currentArea:F3} m2 of {chute.PreDeployedArea:F3}");
                Assert.True(chute.currentArea > 0 && chute.currentArea <= chute.PreDeployedArea + 1e-3,
                            $"predeployed area grows to the predeployed canopy ({chute.currentArea:F3} of {chute.PreDeployedArea:F3} m2)");
                yield return Wait.Until(() => chute.DeploymentState == RealChuteFAR.DeploymentStates.DEPLOYED, 60, "the chute deployed", shot: false);
                ctx.Record("deployedAt", $"{v.altitude:F0} m (set {chute.deployAltitude:F0} m), {v.srfSpeed:F1} m/s");
                Assert.True(v.altitude <= chute.deployAltitude + 50, $"the chute opened at the altitude set ({v.altitude:F0} m, set {chute.deployAltitude:F0})");
                yield return Wait.GameSeconds(1 / chute.deploymentSpeed + 3);

                ctx.Step("under the canopy");
                v = FlightGlobals.ActiveVessel;
                Assert.Near(chute.DeployedArea, chute.currentArea, 0.01 * chute.DeployedArea, "the canopy fully open");
                var sinks = new List<double>();
                var fixedWait = new WaitForFixedUpdate();
                for (int i = 0; i < 100; ++i)
                {
                    yield return fixedWait;
                    sinks.Add(-v.verticalSpeed);
                }

                double sink = sinks.Average();
                double g = FlightGlobals.getGeeForceAtPosition(v.CoM).magnitude;
                // the canopy's drag 1/2 rho v^2 Cd A (RealChuteFAR.DragCalculation, kN) carrying the weight m g (t, so kN)
                double expected = Math.Sqrt(2000 * v.GetTotalMass() * g / (v.atmDensity * RealChuteFAR.staticCd * chute.DeployedArea));
                ctx.Record("sink", $"{sink:F2} m/s (min {sinks.Min():F2}, max {sinks.Max():F2}); canopy alone {expected:F2}; at {v.altitude:F0} m");
                ctx.Record("chuteTemperature", chute.chuteTemperature);
                Assert.Equal(RealChuteFAR.DeploymentStates.DEPLOYED, chute.DeploymentState, "the chute still open");
                Assert.True(sink < 0.2 * stowedSpeed, $"the chute slows the fall ({sink:F1} m/s down, {stowedSpeed:F1} m/s before)");
                // the capsule's own drag adds to the canopy's, so a little slower than the canopy alone
                Assert.InRange(0.6 * expected, 1.1 * expected, sink, "sink rate against the canopy's drag carrying the weight");
                Assert.True(chute.chuteTemperature > 200 && chute.chuteTemperature < RealChuteFAR.maxTemp,
                            $"the canopy's temperature ({chute.chuteTemperature:F1} K) below its limit");

                yield return Snap.Shot(ctx, "under-canopy");
                ctx.Step("the chute's info window");
                chute.GUIToggleWindow();
                yield return Imgui.WaitWindow(window);
                string diameter = Localizer.Format("RCLGUI10", chute.deployedDiameter, chute.DeployedArea.ToString("0.###"));
                int cut = diameter.IndexOfAny(new[] { '\n', '\\' });
                if (cut > 0) diameter = diameter.Substring(0, cut);
                yield return Imgui.WaitVisible(diameter.Trim(), window);
                ctx.Record("symmetryLine", Imgui.IsVisible(Localizer.Format("RCLGUI1", chute.part.symmetryCounterparts.Count), window));
                yield return Imgui.Click(Localizer.Format("RCLGUIClose"), window, exact: true);
                yield return Imgui.WaitWindowClosed(window);

                ctx.Step("cut");
                chute.GUICut();
                Assert.Equal(RealChuteFAR.DeploymentStates.CUT, chute.DeploymentState, "the chute cut");
                Assert.True(chute.currentArea == 0 && !chute.staged && !chute.armed, "a cut chute has no canopy and is unstaged");
                yield return Wait.GameSeconds(4);
                v = FlightGlobals.ActiveVessel;
                ctx.Record("cutFall", $"{-v.verticalSpeed:F1} m/s down");
                Assert.True(-v.verticalSpeed > 3 * sink, $"cut, the capsule falls fast again ({-v.verticalSpeed:F1} m/s, {sink:F1} under the canopy)");
            }
            finally
            {
                chute.deployAltitude = savedAltitude;
                // close the info window if a failure left it open
                Refl.Find(typeof(RealChuteFAR), "visible")?.SetValue(chute, false);
            }
        }
    }
}
