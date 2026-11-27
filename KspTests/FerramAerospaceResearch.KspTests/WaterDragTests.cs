using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using FerramAerospaceResearch.FARAeroComponents;
using HarmonyLib;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The drag that slows a vessel floating in the water (#102: ditched planes and capsules bounce and keep turning).
    /// FAR turns each wet part's water drag into a linear drag on its rigidbody, which stock's PartBuoyancy sets from
    /// PhysicsGlobals.BuoyancyWaterDragSlow; stock also sets a floating part's angular drag
    /// (BuoyancyWaterAngularDragSlow, with no control input) and hands it back to the flight integrator with
    /// part.angularDragByFI.
    /// </summary>
    public static class WaterDragTests
    {
        private const string HarmonyId = "FerramAerospaceResearch.KspTests.WaterDrag";

        /// <summary>
        /// A stock plane ditching at 60 m/s, 2 m/s down (crash damage off, joints unbreakable), left alone for 40 s, then
        /// spun on the water: a yaw turn and a roll rock of 0.5 rad/s each, timed down to 0.05 rad/s.
        /// Every physics step in the water, each wet part's rigidbody drag is at least the water drag FAR worked out
        /// for that part (the value FAR's water drag hook reads, before stock's extra splashdown drag), and a floating
        /// part whose angular drag stock took over keeps stock's value. The bounce, settling time and spin decay are
        /// recorded (water.csv: every step).
        /// </summary>
        [UITest(Kind = "Check", Mods = "TimeControl", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_DitchedPlaneSettlesOnTheWater(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0, 600, 2, "glide", speedMs: 60);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            Assert.True(v.mainBody.ocean, "the plane flies over the sea");

            bool crashWas = CheatOptions.NoCrashDamage, jointsWas = CheatOptions.UnbreakableJoints;
            float waterDragWas = PhysicsGlobals.BuoyancyWaterDragSlow;
            // (stock's water drag on the wet parts is what it checks: not the water model's default)
            PartHydrodynamics.WaterDampingMode dampingWas = PartHydrodynamics.WaterDamping;
            PartHydrodynamics.WaterDamping = PartHydrodynamics.WaterDampingMode.Stock;
            var harmony = new Harmony(HarmonyId);
            using var exceptions = new ExceptionLog();
            try
            {
                CheatOptions.NoCrashDamage = true;
                CheatOptions.UnbreakableJoints = true;
                // (the model knobs: hd.<name> and the rest, WaterParams)
                WaterParams.Apply(ctx);
                string hook = Probe.Install(harmony);
                ctx.Record("hook", $"FARAeroPartModule.{hook}; stock BuoyancyWaterDragSlow {waterDragWas:G4}, BuoyancyWaterAngularDragSlow {PhysicsGlobals.BuoyancyWaterAngularDragSlow:G4}");
                int partsBefore = v.parts.Count;

                ctx.Step("ditch");
                // (its lowest point 0.3 m over the water: by its CoM a meter up, its belly started under the water)
                v.SetPosition(v.vesselTransform.position - (Vector3d)v.upAxis * (GearDitchTests.Lowest(v) - 0.3));
                v.ChangeWorldVelocity((Vector3d)v.upAxis * (-v.verticalSpeed - 2));
                var fixedWait = new WaitForFixedUpdate();
                yield return Wait.Until(() => v.parts.Any(p => p.submergedPortion > 0), 15, "the plane in the water", shot: false);
                Probe.Reset();
                // hyper-warped from the splash (parameter hyperWarp, default 4; 1 for real time): the same physics steps,
                // sooner; nothing flies it, and every check is per step or in game time
                TestWarp.Hyper(ctx, true);
                var s = new Stats();
                var csv = new StringBuilder("t,alt,vs,hs,omega,wetParts,dragMin,dragMax,angDragMin,angDragMax,globalDragSlow,parts,crushScale,jointOverLimit,flood,thrust,waterYaw,hsYaw\n");
                double t0 = Planetarium.GetUniversalTime();
                ctx.Record("splash", $"{v.horizontalSrfSpeed:F1} m/s horizontal, {-v.verticalSpeed:F1} m/s down at {v.altitude:F2} m");

                double quietSince = double.NaN, settledAt = double.NaN, minAlt = double.MaxValue, maxRise = 0, maxAltAfter = double.MinValue;
                double dryTime = 0, dryLongest = 0, dryRun = 0;
                double sideways = 0;
                string sidewaysPart = "none";
                while (Planetarium.GetUniversalTime() - t0 < 40)
                {
                    yield return fixedWait;
                    double t = Planetarium.GetUniversalTime() - t0, dt = TimeWarp.fixedDeltaTime;
                    double alt = v.altitude, omega = v.rootPart.rb.angularVelocity.magnitude;
                    minAlt = Math.Min(minAlt, alt);
                    maxRise = Math.Max(maxRise, alt - minAlt);
                    maxAltAfter = Math.Max(maxAltAfter, alt);
                    bool wet = v.parts.Any(p => p.submergedPortion > 0);
                    dryRun = wet ? 0 : dryRun + dt;
                    if (!wet) dryTime += dt;
                    dryLongest = Math.Max(dryLongest, dryRun);

                    bool quiet = Math.Abs(v.verticalSpeed) < 0.3 && omega < 0.05;
                    if (!quiet) quietSince = double.NaN;
                    else if (double.IsNaN(quietSince)) quietSince = t;
                    if (double.IsNaN(settledAt) && quiet && t - quietSince >= 2) settledAt = quietSince;

                    Probe.Step(v, s);
                    TestWarp.Keep(ctx);
                    if (t >= 20)
                        Sideways(v, ref sideways, ref sidewaysPart);
                    csv.AppendLine(string.Join(",", F(t), F(alt), F(v.verticalSpeed), F(v.horizontalSrfSpeed), F(omega), s.StepWet.ToString(),
                                               F(s.StepDragMin), F(s.StepDragMax), F(s.StepAngMin), F(s.StepAngMax), F(PhysicsGlobals.BuoyancyWaterDragSlow),
                                               SettleDiag(v)));
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "water.csv"), csv.ToString());

                ctx.Record("bounce", $"rose {maxRise:F2} m at most above its lowest so far; highest {maxAltAfter:F2} m; out of the water {dryTime:F2} s (longest {dryLongest:F2} s)");
                ctx.Record("settled", double.IsNaN(settledAt) ? "not in 40 s" : $"{settledAt:F2} s after the splash (|vs| < 0.3 m/s, |w| < 0.05 rad/s, 2 s)");
                ctx.Record("sidewaysBuoyancy", $"from 20 s, the most horizontal share of a floating part's own buoyancy {sideways:P2} ({sidewaysPart})");
                ctx.Record("hyperWarpRearmed", $"{TestWarp.Rearmed} times (TimeControl stops it as the crash breaks parts off)");
                ctx.Record("order", $"FAR's water drag hook ran before the part's PartBuoyancy {Probe.ConsumerFirst} times, after it {Probe.BuoyancyFirst} times");
                ctx.Record("linearDrag", $"{s.PartSteps} wet part-steps: {s.Short} short of FAR's own value (worst {s.WorstShort}), {s.Mismatch} off it by more than 5%; " +
                                         $"rb.drag {s.DragMin:G4} to {s.DragMax:G4}");
                ctx.Record("angularDrag", $"{s.AngSteps} part-steps with stock's angular drag (angularDragByFI off): rb.angularDrag {s.AngMin:G4} to {s.AngMax:G4}; " +
                                          $"{s.AngInputSteps} steps with control input");
                ctx.Record("globalDragSlow", $"PhysicsGlobals.BuoyancyWaterDragSlow {waterDragWas:G4} before, {PhysicsGlobals.BuoyancyWaterDragSlow:G4} after; {s.GlobalMin:G4} to {s.GlobalMax:G4} in the water");

                ctx.Step("yaw turn on the water");
                var spin = new double[2];
                yield return SpinDecay(v, (Vector3)v.upAxis, 0.5, spin, 0);
                ctx.Record("yawDecay", $"0.5 to 0.05 rad/s in {Sec(spin[0])}");
                ctx.Step("roll rock on the water");
                yield return SpinDecay(v, v.ReferenceTransform.up, 0.5, spin, 1);
                ctx.Record("rollDecay", $"0.5 to 0.05 rad/s in {Sec(spin[1])}");
                ctx.Record("parts", $"{v.parts.Count} of {partsBefore}");

                Assert.True(s.PartSteps > 100, $"wet parts with FAR's water drag over the 40 s ({s.PartSteps} part-steps)");
                Assert.True(s.Short == 0, $"each wet part gets at least its own water drag: {s.Short} of {s.PartSteps} part-steps short (worst {s.WorstShort})");
                Assert.True(s.AngSteps > 0, "stock took over the angular drag of a floating part");
                Assert.True(s.AngMin >= PhysicsGlobals.BuoyancyWaterAngularDragSlow * 0.999,
                            $"a floating part keeps stock's angular drag ({s.AngMin:G4} to {s.AngMax:G4}, stock {PhysicsGlobals.BuoyancyWaterAngularDragSlow:G4})");
                // (still water's pressure on a part is straight up: a sideways share pushes a floating wreck round)
                Assert.True(sideways < 0.02, $"a floating part's buoyancy is straight up: {sideways:P2} of it horizontal ({sidewaysPart})");
                exceptions.AssertNone("ditching");
            }
            finally
            {
                TestWarp.Hyper(ctx, false);
                harmony.UnpatchAll(HarmonyId);
                WaterParams.RecordCrush(ctx);
                WaterParams.Restore();
                CheatOptions.NoCrashDamage = crashWas;
                CheatOptions.UnbreakableJoints = jointsWas;
                PhysicsGlobals.BuoyancyWaterDragSlow = waterDragWas;
                PartHydrodynamics.WaterDamping = dampingWas;
            }
        }

        /// <summary>
        /// The stock plane floating still (put on the water: no crash), spun on it as Behaviour_DitchedPlaneSettlesOnTheWater
        /// spins it (a yaw turn and a roll rock of 0.5 rad/s, timed down to 0.05), at real time and then hyper-warped
        /// (parameter hyperWarp, default 4: the same physics steps, more of them a frame). The water's damping is per
        /// physics step, so the decays match. Parameter damping: the WaterDampingMode (Stock, Explicit, None; default the
        /// model's). spin.csv: every step. Explicit: run by hand after a change to the damping or to the warp (the
        /// hyperwarp-is-step-equivalent proof, 2026-11-25)
        /// </summary>
        [UITest(Kind = "Check", Explicit = true, Mods = "TimeControl", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_FloatingSpinDampsTheSameHyperWarped(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            bool crashWas = CheatOptions.NoCrashDamage;
            PartHydrodynamics.WaterDampingMode dampingWas = PartHydrodynamics.WaterDamping;
            using var exceptions = new ExceptionLog();
            try
            {
                CheatOptions.NoCrashDamage = true;
                string mode = ctx.Param("damping");
                if (mode != null)
                    PartHydrodynamics.WaterDamping = (PartHydrodynamics.WaterDampingMode)Enum.Parse(typeof(PartHydrodynamics.WaterDampingMode), mode, true);
                WaterParams.Apply(ctx);
                string said = null;
                yield return FarWaterTools.PutOnWater(s => said = s);
                Vessel v = FlightGlobals.ActiveVessel;
                yield return Wait.GameSeconds(5);
                ctx.Record("on the water", $"{said}; {v.situation}, {v.srfSpeed:F2} m/s, damping {PartHydrodynamics.WaterDamping}");
                Assert.True(v.situation == Vessel.Situations.SPLASHED, $"splashed ({v.situation})");

                double rate = ctx.ParamDouble("hyperWarp", 4);
                var csv = new StringBuilder("warp,axis,t,omega,yawRate,rollRate,timeScale,rootAngDrag,angDragByFI,ctrlSqr,wetParts\n");
                var decay = new double[4];
                for (int run = 0; run < 2; run++)
                {
                    string warp = run == 0 ? "real" : $"{rate:F0}x";
                    if (run == 1)
                        ctx.Record("hyperWarp", Flight.HyperWarp(rate, 1.0) ? $"{rate:F0}x attempted, time scale {Time.timeScale:F1}" : "refused");
                    yield return Wait.GameSeconds(1);
                    ctx.Step($"yaw turn, {warp}");
                    yield return SpinDecay(v, (Vector3)v.upAxis, 0.5, decay, 2 * run, csv, warp + ",yaw");
                    yield return Wait.GameSeconds(5);
                    ctx.Step($"roll rock, {warp}");
                    yield return SpinDecay(v, v.ReferenceTransform.up, 0.5, decay, 2 * run + 1, csv, warp + ",roll");
                    Flight.HyperWarpOff();
                    yield return Wait.GameSeconds(5);
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "spin.csv"), csv.ToString());
                ctx.Record("yawDecay", $"real {Sec(decay[0])}, hyper-warped {Sec(decay[2])}");
                ctx.Record("rollDecay", $"real {Sec(decay[1])}, hyper-warped {Sec(decay[3])}");

                for (int a = 0; a < 2; a++)
                {
                    string name = a == 0 ? "yaw" : "roll";
                    Assert.True(!double.IsNaN(decay[a]) && !double.IsNaN(decay[2 + a]), $"{name}: damped to 0.05 rad/s in 25 s both times");
                    double real = decay[a], warped = decay[2 + a];
                    Assert.True(Math.Abs(warped - real) <= Math.Max(0.15 * real, 0.1),
                                $"{name}: the same decay hyper-warped ({warped:F2} s) as at real time ({real:F2} s)");
                }
                exceptions.AssertNone("spinning on the water");
            }
            finally
            {
                Flight.HyperWarpOff();
                WaterParams.Restore();
                CheatOptions.NoCrashDamage = crashWas;
                PartHydrodynamics.WaterDamping = dampingWas;
            }
        }

        /// <summary>
        /// The stock plane's cockpit end on its own (Mark1Cockpit, its SAS module, docking port and gear bay: a ditching's
        /// wreck piece, 1.7 t), put on still water upright and left 90 s (parameter sec): it rolls over (upright it is not
        /// stable) and comes to rest. Its gear bay's and cockpit's recesses, rolled over with their rims at the water,
        /// flooded and drained at once as it rocked (PartHydrodynamics.DryFaces), a jump in its buoyancy each time, and kept
        /// it rocking for good: a face's dryness fades with the level (DryWeights). piece.csv: every step,
        /// with the work FAR's hydrostatic and dynamic forces have done on it and its mechanical energy (kJ).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_CockpitPieceComesToRest(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            bool crashWas = CheatOptions.NoCrashDamage;
            using var exceptions = new ExceptionLog();
            try
            {
                CheatOptions.NoCrashDamage = true;
                WaterParams.Apply(ctx);
                Vessel v = FlightGlobals.ActiveVessel;
                Part root = v.rootPart;
                Part sas = root.children.First(p => p.partInfo.name == "advSasModule");
                Part bay = root.children.First(p => p.partInfo.name == "SmallGearBay");
                var keep = new HashSet<Part> { root, sas, bay, sas.children.First(p => p.partInfo.name == "dockingPortLateral") };
                foreach (Part p in v.parts.Where(p => !keep.Contains(p)).OrderByDescending(Depth).ToList())
                    p.Die();
                yield return Wait.Frames(10);
                v = FlightGlobals.ActiveVessel;
                ctx.Record("piece", $"{string.Join(", ", v.parts.Select(p => p.partInfo.name))}: {v.GetTotalMass():F2} t");
                Assert.True(v.parts.Count == keep.Count, $"the cockpit end alone ({v.parts.Count} parts)");
                string said = null;
                yield return FarWaterTools.PutOnWater(s => said = s);
                v = FlightGlobals.ActiveVessel;
                ctx.Record("on the water", $"{said}; {v.situation}");
                Assert.True(v.situation == Vessel.Situations.SPLASHED, $"splashed ({v.situation})");

                double sec = ctx.ParamDouble("sec", 90);
                var csv = new StringBuilder("t,alt,vs,hs,omega,wNose,wRight,wUp,dryRuns,Whs,Wdyn,E\n");
                var fixedWait = new WaitForFixedUpdate();
                double t0 = Planetarium.GetUniversalTime(), vsMax = 0, wMax = 0, workHs = 0, workDyn = 0, e0 = double.NaN;
                long runs = PartHydrodynamics.ProfileDryRuns;
                while (true)
                {
                    yield return fixedWait;
                    double t = Planetarium.GetUniversalTime() - t0;
                    if (t > sec)
                        break;
                    long ran = PartHydrodynamics.ProfileDryRuns - runs;
                    runs = PartHydrodynamics.ProfileDryRuns;
                    Vector3 w = v.rootPart.rb.angularVelocity, up = (Vector3)v.upAxis, frame = (Vector3)Krakensbane.GetFrameVelocity();
                    if (t >= sec - 15)
                    {
                        vsMax = Math.Max(vsMax, Math.Abs(v.verticalSpeed));
                        wMax = Math.Max(wMax, w.magnitude);
                    }
                    double e = 0, g = FlightGlobals.getGeeForceAtPosition(v.CoM).magnitude;
                    foreach (Part p in v.parts)
                    {
                        Rigidbody rb = p.rb;
                        if (rb == null) continue;
                        Vector3 pv = rb.velocity + frame;
                        Vector3 wl = Quaternion.Inverse(rb.rotation * rb.inertiaTensorRotation) * rb.angularVelocity;
                        e += 0.5 * rb.mass * pv.sqrMagnitude + 0.5 * Vector3.Dot(wl, Vector3.Scale(rb.inertiaTensor, wl)) +
                             rb.mass * g * FlightGlobals.getAltitudeAtPos((Vector3d)rb.worldCenterOfMass, v.mainBody);
                        PartHydrodynamics hd = p.FindModuleImplementing<FARAeroPartModule>()?.Hydrodynamics;
                        if (hd == null) continue;
                        workHs += (Vector3.Dot(hd.LastHydrostaticForce, pv) + Vector3.Dot(hd.LastHydrostaticTorque, rb.angularVelocity)) * TimeWarp.fixedDeltaTime;
                        workDyn += (Vector3.Dot(hd.LastForce, pv) + Vector3.Dot(hd.LastTorque, rb.angularVelocity)) * TimeWarp.fixedDeltaTime;
                    }
                    if (double.IsNaN(e0)) e0 = e;
                    Transform rt = v.ReferenceTransform;
                    csv.AppendLine(string.Join(",", F(t), F(v.altitude), F(v.verticalSpeed), F(v.horizontalSrfSpeed), F(w.magnitude),
                                               F(Vector3.Dot(w, rt.up)), F(Vector3.Dot(w, rt.right)), F(Vector3.Dot(w, up)), ran.ToString(),
                                               F(workHs), F(workDyn), F(e - e0)));
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "piece.csv"), csv.ToString());
                ctx.Record("lateMotion", $"last 15 s: |vs| up to {vsMax:F4} m/s, |w| up to {wMax:F4} rad/s");
                Assert.True(vsMax < 0.01 && wMax < 0.02, $"at rest in the last 15 s (|vs| up to {vsMax:F4} m/s, |w| up to {wMax:F4} rad/s)");
                exceptions.AssertNone("floating");
            }
            finally
            {
                WaterParams.Restore();
                CheatOptions.NoCrashDamage = crashWas;
            }
        }

        private static int Depth(Part p)
        {
            int d = 0;
            for (Part q = p.parent; q != null; q = q.parent)
                d++;
            return d;
        }

        // (diagnostics per step: the parts, the crush cap's scale, its joint load over what the joints hold, the
        // flooding, the engines' thrust (kN), and the water's yaw torque about the vessel's CoM, dynamic and hydrostatic (kN m))
        private static string SettleDiag(Vessel v)
        {
            VesselHydrodynamics vh = v.GetComponent<FARVesselAero>()?.Hydrodynamics;
            double thrust = v.parts.SelectMany(p => p.Modules.OfType<ModuleEngines>()).Sum(e => (double)e.finalThrust);
            Vector3 up = (Vector3)v.upAxis, com = (Vector3)v.CoM;
            double yaw = 0, hsYaw = 0;
            foreach (Part p in v.parts)
            {
                PartHydrodynamics hd = p.FindModuleImplementing<FARAeroPartModule>()?.Hydrodynamics;
                if (hd == null || p.rb == null) continue;
                Vector3 r = p.rb.worldCenterOfMass - com;
                yaw += Vector3.Dot(hd.LastTorque + Vector3.Cross(r, hd.LastForce), up);
                hsYaw += Vector3.Dot(hd.LastHydrostaticTorque + Vector3.Cross(r, hd.LastHydrostaticForce), up);
            }
            return string.Join(",", v.parts.Count.ToString(), F(vh?.CrushScale ?? 1),
                               F(vh != null ? vh.LastCrushTotal / Math.Max(1e-6, vh.LastCrushLimit) : 0), F(vh?.Flood ?? 0), F(thrust), F(yaw), F(hsYaw));
        }

        // the most horizontal share of a wet part's own (mesh) buoyancy so far, of those holding up 0.1 kN or more
        private static void Sideways(Vessel v, ref double most, ref string which)
        {
            Vector3 up = (Vector3)v.upAxis;
            foreach (Part p in v.parts)
            {
                PartHydrodynamics hd = p.FindModuleImplementing<FARAeroPartModule>()?.Hydrodynamics;
                if (hd == null || !hd.OwnsBuoyancy || p.submergedPortion <= 0) continue;
                Vector3 f = hd.LastHydrostaticForce;
                double vert = Vector3.Dot(f, up);
                if (vert < 0.1) continue;
                double share = (f - up * (float)vert).magnitude / vert;
                if (share > most)
                {
                    most = share;
                    which = p.partInfo.name;
                }
            }
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);
        private static string Sec(double s) => double.IsNaN(s) ? "more than 25 s" : $"{s:F2} s";

        /// <summary>
        /// Sets the vessel turning at rate rad/s about axis (each rigidbody's velocity matching the turn about the
        /// vessel's center of mass), then times how long its root part's angular velocity takes to stay under a tenth of
        /// that for 1 s (NaN: not in 25 s).
        /// </summary>
        private static IEnumerator SpinDecay(Vessel v, Vector3 axis, double rate, double[] into, int index, StringBuilder log = null,
                                              string label = null)
        {
            var fixedWait = new WaitForFixedUpdate();
            yield return fixedWait;
            Vector3 omega = axis.normalized * (float)rate;
            Vector3 com = Vector3.zero, vcom = Vector3.zero;
            float mass = 0;
            foreach (Part p in v.parts)
            {
                if (p.rb == null) continue;
                com += p.rb.worldCenterOfMass * p.rb.mass;
                vcom += p.rb.velocity * p.rb.mass;
                mass += p.rb.mass;
            }
            com /= mass;
            vcom /= mass;
            foreach (Part p in v.parts)
            {
                if (p.rb == null) continue;
                p.rb.angularVelocity = omega;
                p.rb.velocity = vcom + Vector3.Cross(omega, p.rb.worldCenterOfMass - com);
            }

            double t0 = Planetarium.GetUniversalTime(), below = double.NaN;
            into[index] = double.NaN;
            while (Planetarium.GetUniversalTime() - t0 < 25)
            {
                yield return fixedWait;
                double t = Planetarium.GetUniversalTime() - t0;
                if (log != null)
                {
                    Rigidbody rb = v.rootPart.rb;
                    FlightCtrlState cs = v.ctrlState;
                    log.AppendLine(string.Join(",", label, F(t), F(rb.angularVelocity.magnitude), F(Vector3.Dot(rb.angularVelocity, (Vector3)v.upAxis)),
                                               F(Vector3.Dot(rb.angularVelocity, v.ReferenceTransform.up)), F(Time.timeScale), F(rb.angularDrag),
                                               v.rootPart.angularDragByFI ? "1" : "0", F(cs.pitch * cs.pitch + cs.yaw * cs.yaw + cs.roll * cs.roll),
                                               v.parts.Count(p => p.submergedPortion > 0).ToString()));
                }
                if (v.rootPart.rb.angularVelocity.magnitude >= 0.1 * rate) below = double.NaN;
                else if (double.IsNaN(below)) below = t;
                if (!double.IsNaN(below) && t - below >= 1)
                {
                    into[index] = below;
                    yield break;
                }
            }
        }

        private class Stats
        {
            public int PartSteps, Short, Mismatch, AngSteps, AngInputSteps, StepWet;
            public string WorstShort = "none";
            public double WorstShortRatio = 1, DragMin = double.MaxValue, DragMax, AngMin = double.MaxValue, AngMax;
            public double GlobalMin = double.MaxValue, GlobalMax;
            public double StepDragMin, StepDragMax, StepAngMin, StepAngMax;
        }

        /// <summary>
        /// Watches FAR's water drag hook (master: FARAeroPartModule.FixedUpdate, which writes
        /// PhysicsGlobals.BuoyancyWaterDragSlow; #178: ApplyWaterDrag, from the flight integrator, which sets the part's
        /// own rigidbody) and stock's PartBuoyancy.FixedUpdate: the drag FAR worked out for each part this step, and
        /// which of the two ran first for it.
        /// </summary>
        private static class Probe
        {
            private static readonly Dictionary<Part, float> intended = new Dictionary<Part, float>();
            private static readonly Dictionary<Part, int> consSeq = new Dictionary<Part, int>(), buoySeq = new Dictionary<Part, int>();
            private static readonly AccessTools.FieldRef<PartBuoyancy, Part> buoyPart = AccessTools.FieldRefAccess<PartBuoyancy, Part>("part");
            private static int seq;
            public static int ConsumerFirst, BuoyancyFirst;

            public static string Install(Harmony h)
            {
                MethodInfo hook = AccessTools.DeclaredMethod(typeof(FARAeroPartModule), "ApplyWaterDrag") ??
                                  AccessTools.DeclaredMethod(typeof(FARAeroPartModule), "FixedUpdate");
                h.Patch(hook, prefix: new HarmonyMethod(typeof(Probe), nameof(HookPrefix)));
                h.Patch(AccessTools.Method(typeof(PartBuoyancy), "FixedUpdate"), postfix: new HarmonyMethod(typeof(Probe), nameof(BuoyancyPostfix)));
                return hook.Name;
            }

            public static void Reset()
            {
                intended.Clear();
                consSeq.Clear();
                buoySeq.Clear();
                ConsumerFirst = BuoyancyFirst = 0;
            }

            // the same test and sum as the hook's own
            public static void HookPrefix(FARAeroPartModule __instance)
            {
                Part p = __instance.part;
                // (a FAR wing panel takes its water drag from its wing model, as a force: its rigidbody's is none)
                if (FARAeroPartModule.waterSlowDragNew <= 0 || !__instance.vessel || p.submergedPortion <= 0 || __instance.LegacyWingModel != null)
                    return;
                float own = Math.Max(__instance.hackWaterDragVal, 0f);
                float vel = __instance.partLocalVel.magnitude;
                if (vel < PhysicsGlobals.BuoyancyWaterDragMinVel || vel > __instance.vessel.srfSpeed * FARAeroPartModule.minVelVesselMultNew)
                    own += FARAeroPartModule.waterSlowDragNew;
                intended[p] = own;
                consSeq[p] = ++seq;
            }

            public static void BuoyancyPostfix(PartBuoyancy __instance)
            {
                Part p = buoyPart(__instance);
                if (p != null)
                    buoySeq[p] = ++seq;
            }

            /// <summary>After a physics step: each wet part's rigidbody drag and angular drag against FAR's and stock's.</summary>
            public static void Step(Vessel v, Stats s)
            {
                s.StepWet = 0;
                s.StepDragMin = s.StepAngMin = double.NaN;
                s.StepDragMax = s.StepAngMax = double.NaN;
                bool input = v.ctrlState.pitch * v.ctrlState.pitch + v.ctrlState.yaw * v.ctrlState.yaw + v.ctrlState.roll * v.ctrlState.roll >=
                             PhysicsGlobals.BuoyancyAngularDragMinControlSqrMag;
                if (v.parts.Any(p => p.submergedPortion > 0))
                {
                    s.GlobalMin = Math.Min(s.GlobalMin, PhysicsGlobals.BuoyancyWaterDragSlow);
                    s.GlobalMax = Math.Max(s.GlobalMax, PhysicsGlobals.BuoyancyWaterDragSlow);
                }
                foreach (KeyValuePair<Part, float> kv in intended)
                {
                    Part p = kv.Key;
                    if (p == null || p.rb == null || p.vessel != v) continue;
                    double drag = p.rb.drag, own = kv.Value;
                    s.PartSteps++;
                    s.StepWet++;
                    s.DragMin = Math.Min(s.DragMin, drag);
                    s.DragMax = Math.Max(s.DragMax, drag);
                    s.StepDragMin = double.IsNaN(s.StepDragMin) ? drag : Math.Min(s.StepDragMin, drag);
                    s.StepDragMax = double.IsNaN(s.StepDragMax) ? drag : Math.Max(s.StepDragMax, drag);
                    if (Math.Abs(drag - own) > 0.05 * own + 1e-3) s.Mismatch++;
                    if (drag < 0.95 * own - 1e-3)
                    {
                        s.Short++;
                        double ratio = drag / own;
                        if (ratio < s.WorstShortRatio)
                        {
                            s.WorstShortRatio = ratio;
                            s.WorstShort = $"{p.partInfo.name}: {drag:G4} of its {own:G4}";
                        }
                    }
                }
                foreach (Part p in v.parts)
                {
                    if (p.rb == null || p.submergedPortion <= 0 || p.angularDragByFI) continue;
                    double ang = p.rb.angularDrag;
                    s.AngSteps++;
                    if (input) s.AngInputSteps++;
                    s.AngMin = Math.Min(s.AngMin, ang);
                    s.AngMax = Math.Max(s.AngMax, ang);
                    s.StepAngMin = double.IsNaN(s.StepAngMin) ? ang : Math.Min(s.StepAngMin, ang);
                    s.StepAngMax = double.IsNaN(s.StepAngMax) ? ang : Math.Max(s.StepAngMax, ang);
                }
                foreach (KeyValuePair<Part, int> kv in consSeq)
                    if (buoySeq.TryGetValue(kv.Key, out int b))
                    {
                        if (b < kv.Value) BuoyancyFirst++;
                        else ConsumerFirst++;
                    }
                intended.Clear();
                consSeq.Clear();
                buoySeq.Clear();
            }
        }
    }
}
