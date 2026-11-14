using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The Test Boat (crafts/test_boat.py: the Planing Hull with the FAR test electric motor under its stern, a B9
    /// procedural control surface for its spade rudder, a probe core) driven about the water as a player would, after FarWaterTools' button put it there.
    /// </summary>
    public static class BoatTests
    {
        // the floatplane tests' (one instance): CustomProceduralParts for the hull, FARTestFloatplane for the motor
        private const string Mods = TestWarp.WaterMods;   // the water tests' instance

        /// <summary>
        /// The Test Boat put on the water by FarWaterTools.PutOnWater (the button's own code), after the Set Position
        /// cheat's ease-in has cut its gravity to a tenth: it floats at rest at full gravity. Then full throttle, rudder
        /// centered, for runSec (default 45): it gets going, and the motor's thrust is its curve's at the speed it makes
        /// (10 kN at rest, none at 25 m/s; the motor works in the water). Then slowed to turnThrottle (default 0.3) and
        /// full rudder one way for turnSec (default 8) and the other: the rudder turns it both ways, the boat heeling under
        /// 30 deg. mass (t, default 4) the hull's fuel makes it. Writes boat.csv.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Test Boat", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Boat_MotorRun(TestContext ctx)
        {
            double massT = ctx.ParamDouble("mass", 4), runSec = ctx.ParamDouble("runSec", 45), turnSec = ctx.ParamDouble("turnSec", 8);
            WaterParams.Apply(ctx);
            bool crashWas = CheatOptions.NoCrashDamage;
            try
            {
                yield return Wait.Seconds(5);
                Vessel v = FlightGlobals.ActiveVessel;
                Part hull = v.rootPart;
                foreach (PartResource r in hull.Resources)
                    r.amount = 0;
                yield return Wait.Frames(2);
                PartResource lf = hull.Resources.Get("LiquidFuel");
                double dry = v.parts.Sum(q => q.mass + q.GetResourceMass());
                if (lf != null)
                    lf.amount = Math.Max(0, Math.Min(lf.maxAmount, (massT - dry) / lf.info.density));
                yield return Wait.Frames(2);
                CheatOptions.NoCrashDamage = true;

                // the Set Position cheat's ease-in on (its gravity a tenth), for the button to undo
                typeof(FlightGlobals).GetMethod("ToggleVesselEaseIn", BindingFlags.NonPublic | BindingFlags.Instance)
                    ?.Invoke(FlightGlobals.fetch, new object[] { v, true, 0.1 });
                ctx.Record("ease-in", $"on: gravity x{v.gravityMultiplier:F2}");
                string said = null;
                yield return FarWaterTools.PutOnWater(s => said = s);
                v = FlightGlobals.ActiveVessel;
                yield return Wait.GameSeconds(5);
                Vector3 up = v.upAxis;
                Vector3 fwd = v.ReferenceTransform.up, right = v.ReferenceTransform.right;
                double roll = Math.Asin(Mathf.Clamp(Vector3.Dot(right, up), -1, 1)) * 180 / Math.PI;
                double trim = Math.Asin(Mathf.Clamp(Vector3.Dot(fwd, up), -1, 1)) * 180 / Math.PI;
                ctx.Record("on the water", $"{said}; {v.situation}, gravity x{v.gravityMultiplier:F2}, {v.srfSpeed:F2} m/s, trim {trim:F1} deg, " +
                                           $"roll {roll:F1} deg, mass {v.GetTotalMass():F3} t");
                Assert.True(v.situation == Vessel.Situations.SPLASHED, $"splashed ({v.situation})");
                Assert.True(Math.Abs(v.gravityMultiplier - 1) < 1e-6, $"full gravity (x{v.gravityMultiplier:F2})");
                Assert.True(v.srfSpeed < 0.5 && Math.Abs(roll) < 5, $"at rest, upright ({v.srfSpeed:F2} m/s, roll {roll:F1} deg)");

                Part motorPart = v.parts.First(q => q.partInfo.name == "FARTest.BoatMotor");
                ModuleEngines motor = motorPart.Modules.GetModule<ModuleEngines>();
                motor.Activate();
                Ctrl.Attach();
                Ctrl.SetThrottle(1);
                TestWarp.Hyper(ctx, true);
                var csv = new StringBuilder("t,phase,V_ms,thrust_kN,curve_kN,water_drag_kN,heading_deg,trim_deg,roll_deg,yaw\n");
                var fixedWait = new WaitForFixedUpdate();
                Vector3 h0 = Vector3.ProjectOnPlane(fwd, up).normalized;
                double t0 = Planetarium.GetUniversalTime();
                double Heading() => Vector3.SignedAngle(h0, Vector3.ProjectOnPlane(v.ReferenceTransform.up, up), up);
                var tail = new List<(double V, double thrust, double curve)>();
                double turn1 = double.NaN, turn2 = double.NaN, hStart = 0, heel = 0;
                // the turns at turnThrottle (default 0.3, about 5 m/s): at full speed, hard over, this flat-bottomed box (its
                // CG a meter above the keel) heels out of the turn until its rudder leaves the water
                float turnThrottle = (float)ctx.ParamDouble("turnThrottle", 0.3);
                foreach (var (phase, yaw, sec, throttle) in new[]
                         {
                             ("run", 0f, runSec, 1f), ("slow", 0f, 15, turnThrottle), ("right", 1f, turnSec, turnThrottle),
                             ("left", -1f, 2 * turnSec, turnThrottle)
                         })
                {
                    Ctrl.Yaw = yaw;
                    Ctrl.SetThrottle(throttle);
                    double ts = Planetarium.GetUniversalTime(), hs = Heading();
                    while (Planetarium.GetUniversalTime() - ts < sec)
                    {
                        yield return fixedWait;
                        up = v.upAxis;
                        double V = v.srfSpeed, thrust = motor.finalThrust, curve = 10 * Math.Max(0, 1 - V / 25);
                        Vector3 dir = ((Vector3)v.srf_velocity).normalized;
                        Vector3 water = Vector3.zero;
                        foreach (Part q in v.parts)
                        {
                            var hq = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                            if (hq != null)
                                water += hq.LastForce;
                        }
                        double t = Planetarium.GetUniversalTime() - t0;
                        double tr = Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.up, up), -1, 1)) * 180 / Math.PI;
                        double rl = Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.right, up), -1, 1)) * 180 / Math.PI;
                        csv.AppendLine(string.Join(",", F(t), phase, F(V), F(thrust), F(curve), F(-Vector3.Dot(water, dir)), F(Heading()), F(tr), F(rl),
                                                   F(yaw)));
                        if (phase == "right" || phase == "left")
                            heel = Math.Max(heel, Math.Abs(rl));
                        if (phase == "run" && Planetarium.GetUniversalTime() - ts > sec - 5)
                            tail.Add((V, thrust, curve));
                    }
                    double dh = Mathf.DeltaAngle((float)hs, (float)Heading());
                    if (phase == "right")
                        turn1 = dh;
                    else if (phase == "left")
                        turn2 = dh;
                    else if (phase == "run")
                        hStart = Heading();
                }
                Ctrl.Yaw = 0;
                Ctrl.SetThrottle(0);
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "boat.csv"), csv.ToString());
                double vEnd = tail.Average(x => x.V), thrustEnd = tail.Average(x => x.thrust), curveEnd = tail.Average(x => x.curve);
                ctx.Record("run", $"{vEnd:F2} m/s after {runSec:F0} s at full throttle: thrust {thrustEnd:F2} kN, its curve's {curveEnd:F2} kN");
                ctx.Record("turns", $"full rudder one way for {turnSec:F0} s: heading {turn1:+0.0;-0.0} deg; the other way for {2 * turnSec:F0} s: {turn2:+0.0;-0.0} deg; heel up to {heel:F1} deg");
                Assert.True(vEnd > 5, $"under way ({vEnd:F2} m/s)");
                Assert.True(Math.Abs(thrustEnd - curveEnd) < 0.05 * 10, $"the motor's thrust its curve's ({thrustEnd:F2} against {curveEnd:F2} kN)");
                Assert.True(Math.Abs(turn1) > 10 && Math.Sign(turn2) == -Math.Sign(turn1), $"the rudder turns it both ways ({turn1:F1}, {turn2:F1} deg)");
                Assert.True(heel < 30, $"upright in its turns (heel up to {heel:F1} deg)");
            }
            finally
            {
                TestWarp.Hyper(ctx, false);
                Ctrl.Yaw = 0;
                CheatOptions.NoCrashDamage = crashWas;
                WaterParams.Restore();
            }
        }

        /// <summary>
        /// A player's jet boat (Boaty McTestFace x4: Mk3 stock parts for its hulls, four AJE turbofans, B9 wings) put on
        /// the water by FarWaterTools.PutOnWater and run at full throttle, the stick held at each of pitches (default
        /// 0, -0.5, +0.5: level, nose down, nose up) for phaseSec (default 30) in turn: its speed and trim, and the
        /// water's forces along its course (each part's: the hull model's momentum, friction, impact, crossflow and
        /// hydrostatic forces; the foils') beside its engines' thrust, recorded at each phase's fastest. Recorded, not
        /// asserted. Writes jetboat.csv.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Boaty McTestFace x4", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Boat_JetRun(TestContext ctx) => JetRun(ctx);

        /// <summary>Boat_JetRun's run of the player's next version: one turbofan, cargo ramps at the bows.</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Boaty McTestFace", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Boat_JetRunSingle(TestContext ctx) => JetRun(ctx);

        private static IEnumerator JetRun(TestContext ctx)
        {
            double phaseSec = ctx.ParamDouble("phaseSec", 30);
            double[] pitches = ctx.ParamList("pitches", "0+-0.5+0.5").Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            WaterParams.Apply(ctx);
            bool crashWas = CheatOptions.NoCrashDamage;
            try
            {
                yield return Wait.Seconds(5);
                Vessel v = FlightGlobals.ActiveVessel;
                CheatOptions.NoCrashDamage = true;
                string said = null;
                yield return FarWaterTools.PutOnWater(s => said = s);
                v = FlightGlobals.ActiveVessel;
                yield return Wait.GameSeconds(5);
                ctx.Record("on the water", $"{said}; {v.situation}, {v.srfSpeed:F2} m/s, mass {v.GetTotalMass():F2} t, {v.parts.Count} parts");

                var engines = v.parts.SelectMany(q => q.Modules.GetModules<ModuleEngines>()).ToList();
                foreach (ModuleEngines e in engines)
                    e.Activate();
                Ctrl.Attach();
                Ctrl.SetThrottle(1);
                var csv = new StringBuilder("t,pitch_in,V_ms,trim_deg,roll_deg,thrust_kN,water_fwd_kN,momentum_fwd_kN,friction_fwd_kN,impact_fwd_kN,crossflow_fwd_kN,hydrostatic_fwd_kN,foil_fwd_kN,water_up_kN," +
                                            "wing_fwd_kN,voxel_fwd_kN,gravity_fwd_kN,ma_fwd_kN,residual_kN,alt_m,vs_ms\n");
                Vector3 velWas = v.srf_velocity;
                var fixedWait = new WaitForFixedUpdate();
                double t0 = Planetarium.GetUniversalTime();
                foreach (double pin in pitches)
                {
                    Ctrl.Pitch = (float)pin;
                    double ts = Planetarium.GetUniversalTime(), vMax = 0;
                    string atMax = "";
                    while (Planetarium.GetUniversalTime() - ts < phaseSec)
                    {
                        yield return fixedWait;
                        Vector3 up = v.upAxis, dir = ((Vector3)v.srf_velocity).normalized;
                        double V = v.srfSpeed;
                        double trim = Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.up, up), -1, 1)) * 180 / Math.PI;
                        double roll = Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.right, up), -1, 1)) * 180 / Math.PI;
                        double thrust = engines.Sum(e => e.finalThrust);
                        Vector3 all = Vector3.zero, mom = Vector3.zero, fr = Vector3.zero, imp = Vector3.zero, cf = Vector3.zero, hs = Vector3.zero, foil = Vector3.zero;
                        var parts = new List<string>();
                        foreach (Part q in v.parts)
                        {
                            var hq = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                            var w = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                            Vector3 wf = w != null && q.submergedPortion > 0 ? w.worldSpaceForce : Vector3.zero;   // (its air and water both: FAR weights the water by its submerged share)
                            if (hq != null)
                            {
                                all += hq.LastForce + hq.LastHydrostaticForce;
                                mom += hq.LastMomentumForce; fr += hq.LastFrictionForce; imp += hq.LastImpactForce; cf += hq.LastCrossflowForce;
                                hs += hq.LastHydrostaticForce;
                            }
                            foil += wf;
                            if (hq != null && (hq.LastForce + hq.LastHydrostaticForce).sqrMagnitude > 1e-4 || wf.sqrMagnitude > 1e-4)
                                parts.Add($"{q.partInfo.name} sub {q.submergedPortion:F2}{(hq != null ? $" owns {hq.OwnsBuoyancy} open {hq.OpenShare:F2} cap {hq.CapArea:F2} vol {hq.MeshVolume:F2}" : "")}: fwd {Vector3.Dot((hq?.LastForce ?? Vector3.zero) + (hq?.LastHydrostaticForce ?? Vector3.zero), dir):+0.0;-0.0} " +
                                          $"(mom {Vector3.Dot(hq?.LastMomentumForce ?? Vector3.zero, dir):+0.0;-0.0}, imp {Vector3.Dot(hq?.LastImpactForce ?? Vector3.zero, dir):+0.0;-0.0}, hs {Vector3.Dot(hq?.LastHydrostaticForce ?? Vector3.zero, dir):+0.0;-0.0}" +
                                          $"{(wf != Vector3.zero ? $", foil {Vector3.Dot(wf, dir):+0.0;-0.0}" : "")}) up {Vector3.Dot((hq?.LastForce ?? Vector3.zero) + (hq?.LastHydrostaticForce ?? Vector3.zero) + wf, up):F1}");
                        }
                        double t = Planetarium.GetUniversalTime() - t0;
                        // the balance along its course: its mass times its acceleration against every force known
                        Vector3 wing = Vector3.zero, voxel = Vector3.zero;
                        foreach (Part q in v.parts)
                        {
                            var w = q.Modules.GetModule<ferram4.FARWingAerodynamicModel>();
                            if (w != null)
                                wing += w.worldSpaceForce;
                            var am = q.Modules.GetModule<FARAeroComponents.FARAeroPartModule>();
                            if (am != null)
                                voxel += am.worldSpaceAeroForce;
                        }
                        Vector3 thrustV = Vector3.zero;
                        foreach (ModuleEngines e in engines)
                            foreach (Transform tt in e.thrustTransforms.Where(x => x))
                                thrustV += -tt.forward * (e.finalThrust / Math.Max(1, e.thrustTransforms.Count));
                        double dt = Math.Max(1e-3, TimeWarp.fixedDeltaTime);
                        Vector3 acc = ((Vector3)v.srf_velocity - velWas) / (float)dt;
                        velWas = v.srf_velocity;
                        double mass = v.GetTotalMass();
                        double g = Vector3.Dot((Vector3)FlightGlobals.getGeeForceAtPosition(v.CoM), dir) * mass;
                        double ma = Vector3.Dot(acc, dir) * mass;
                        double known = Vector3.Dot(thrustV + all + wing + voxel, dir) + g;
                        csv.AppendLine(string.Join(",", F(t), F(pin), F(V), F(trim), F(roll), F(thrust), F(Vector3.Dot(all + foil, dir)), F(Vector3.Dot(mom, dir)),
                                                   F(Vector3.Dot(fr, dir)), F(Vector3.Dot(imp, dir)), F(Vector3.Dot(cf, dir)), F(Vector3.Dot(hs, dir)),
                                                   F(Vector3.Dot(foil, dir)), F(Vector3.Dot(all + foil, up)),
                                                   F(Vector3.Dot(wing, dir)), F(Vector3.Dot(voxel, dir)), F(g), F(ma), F(ma - known), F(v.altitude), F(v.verticalSpeed)));
                        if (V > vMax)
                        {
                            vMax = V;
                            atMax = $"t {t:F1}: {V:F1} m/s, trim {trim:F1} deg, thrust {thrust:F1} kN; water along its course {Vector3.Dot(all + foil, dir):+0.0;-0.0} kN " +
                                    $"(momentum {Vector3.Dot(mom, dir):+0.0;-0.0}, friction {Vector3.Dot(fr, dir):+0.0;-0.0}, impact {Vector3.Dot(imp, dir):+0.0;-0.0}, " +
                                    $"crossflow {Vector3.Dot(cf, dir):+0.0;-0.0}, hydrostatic {Vector3.Dot(hs, dir):+0.0;-0.0}, foils {Vector3.Dot(foil, dir):+0.0;-0.0}), " +
                                    $"up {Vector3.Dot(all + foil, up):F1} kN; parts: {string.Join("; ", parts)}";
                        }
                    }
                    ctx.Record($"pitch {pin:+0.0;-0.0}", $"{v.srfSpeed:F1} m/s at the end; fastest {atMax}");
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "jetboat.csv"), csv.ToString());
            }
            finally
            {
                Ctrl.Pitch = 0;
                Ctrl.SetThrottle(0);
                CheatOptions.NoCrashDamage = crashWas;
                WaterParams.Restore();
            }
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);
    }
}
