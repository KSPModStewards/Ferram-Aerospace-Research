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
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_DitchedPlaneSettlesOnTheWater(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0, 600, 2, "glide", speedMs: 60);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            Assert.True(v.mainBody.ocean, "the plane flies over the sea");

            bool crashWas = CheatOptions.NoCrashDamage, jointsWas = CheatOptions.UnbreakableJoints;
            float waterDragWas = PhysicsGlobals.BuoyancyWaterDragSlow;
            var harmony = new Harmony(HarmonyId);
            using var exceptions = new ExceptionLog();
            try
            {
                CheatOptions.NoCrashDamage = true;
                CheatOptions.UnbreakableJoints = true;
                string hook = Probe.Install(harmony);
                ctx.Record("hook", $"FARAeroPartModule.{hook}; stock BuoyancyWaterDragSlow {waterDragWas:G4}, BuoyancyWaterAngularDragSlow {PhysicsGlobals.BuoyancyWaterAngularDragSlow:G4}");
                int partsBefore = v.parts.Count;

                ctx.Step("ditch");
                v.SetPosition(v.vesselTransform.position - (Vector3d)v.upAxis * (v.altitude - 1));
                v.ChangeWorldVelocity((Vector3d)v.upAxis * (-v.verticalSpeed - 2));
                var fixedWait = new WaitForFixedUpdate();
                yield return Wait.Until(() => v.parts.Any(p => p.submergedPortion > 0), 15, "the plane in the water", shot: false);
                Probe.Reset();
                var s = new Stats();
                var csv = new StringBuilder("t,alt,vs,hs,omega,wetParts,dragMin,dragMax,angDragMin,angDragMax,globalDragSlow\n");
                double t0 = Planetarium.GetUniversalTime();
                ctx.Record("splash", $"{v.horizontalSrfSpeed:F1} m/s horizontal, {-v.verticalSpeed:F1} m/s down at {v.altitude:F2} m");

                double quietSince = double.NaN, settledAt = double.NaN, minAlt = double.MaxValue, maxRise = 0, maxAltAfter = double.MinValue;
                double dryTime = 0, dryLongest = 0, dryRun = 0;
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
                    csv.AppendLine(string.Join(",", F(t), F(alt), F(v.verticalSpeed), F(v.horizontalSrfSpeed), F(omega), s.StepWet.ToString(),
                                               F(s.StepDragMin), F(s.StepDragMax), F(s.StepAngMin), F(s.StepAngMax), F(PhysicsGlobals.BuoyancyWaterDragSlow)));
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "water.csv"), csv.ToString());

                ctx.Record("bounce", $"rose {maxRise:F2} m at most above its lowest so far; highest {maxAltAfter:F2} m; out of the water {dryTime:F2} s (longest {dryLongest:F2} s)");
                ctx.Record("settled", double.IsNaN(settledAt) ? "not in 40 s" : $"{settledAt:F2} s after the splash (|vs| < 0.3 m/s, |w| < 0.05 rad/s, 2 s)");
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
                exceptions.AssertNone("ditching");
            }
            finally
            {
                harmony.UnpatchAll(HarmonyId);
                CheatOptions.NoCrashDamage = crashWas;
                CheatOptions.UnbreakableJoints = jointsWas;
                PhysicsGlobals.BuoyancyWaterDragSlow = waterDragWas;
            }
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);
        private static string Sec(double s) => double.IsNaN(s) ? "more than 25 s" : $"{s:F2} s";

        /// <summary>
        /// Sets the vessel turning at rate rad/s about axis (each rigidbody's velocity matching the turn about the
        /// vessel's center of mass), then times how long its root part's angular velocity takes to stay under a tenth of
        /// that for 1 s (NaN: not in 25 s).
        /// </summary>
        private static IEnumerator SpinDecay(Vessel v, Vector3 axis, double rate, double[] into, int index)
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
                if (FARAeroPartModule.waterSlowDragNew <= 0 || !__instance.vessel || p.submergedPortion <= 0)
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
