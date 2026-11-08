using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's forces on parts in the water: a wing dipped in the sea loses nearly all its lift, and water drag slows the
    /// craft far faster than the air does.
    /// </summary>
    public static class WaterTests
    {
        /// <summary>
        /// A stock plane ditching: gliding level at 60 m/s, engine off and gear up, it is set down a few meters above the
        /// sea (crash damage off). In the water, the wings that are partly under keep about a hundredth of the lift FAR
        /// works out for them (FAR's lift collapse for a part-submerged wing), the plane loses speed many times faster
        /// than it did in the air at the same speed, FAR's water drag setting (BuoyancyWaterDragSlow) is set and
        /// real, every force stays finite, and FAR throws nothing.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_PlaneDitchesInTheSea(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0, 600, 2, "glide", speedMs: 60);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            Assert.True(v.mainBody.ocean, "the plane flies over the sea");

            bool dataWas = PhysicsGlobals.AeroDataDisplay, crashWas = CheatOptions.NoCrashDamage;
            double waterDragWas = PhysicsGlobals.BuoyancyWaterDragSlow;
            using var exceptions = new ExceptionLog();
            try
            {
                // the wings' lift before the water split is what the part menu's aero data shows
                PhysicsGlobals.AeroDataDisplay = true;
                CheatOptions.NoCrashDamage = true;

                ctx.Step("gliding at 60 m/s");
                var samples = new List<FarFlightProbe.Sample>();
                yield return FarFlightProbe.Hold(c, 15, 20, samples, predict: false);
                double mass = v.GetTotalMass();
                double airDecel = samples.Average(s => s.Flight[0]) / mass;
                double airSpeed = v.srfSpeed;
                ctx.Record("air", $"{airSpeed:F1} m/s, drag {samples.Average(s => s.Flight[0]):F2} kN, {airDecel:F3} m/s2, {mass:F2} t");
                Assert.True(airDecel > 0, $"drag slows the plane in the air ({airDecel:F3} m/s2)");

                ctx.Step("set down just above the sea");
                // a meter above the water and sinking at 1 m/s: dropped from 3 m it hit at 9 m/s, a wing crossed the
                // surface inside a physics frame, and a plane that stopped dead had no wing part-submerged two frames
                // running (the check below needs that)
                v.SetPosition(v.vesselTransform.position - (Vector3d)v.upAxis * (v.altitude - 1));
                v.ChangeWorldVelocity((Vector3d)v.upAxis * (-v.verticalSpeed - 1));
                var fixedWait = new WaitForFixedUpdate();
                yield return Wait.Until(() => v.Splashed || v.parts.Any(p => p.submergedPortion > 0), 15, "the plane in the water", shot: false);
                double t0 = Planetarium.GetUniversalTime(), speed0 = v.horizontalSrfSpeed;
                ctx.Record("splash", $"{speed0:F1} m/s horizontal, {-v.verticalSpeed:F1} m/s down at {v.altitude:F1} m");

                List<FARWingAerodynamicModel> wings = v.parts.SelectMany(p => p.Modules.OfType<FARWingAerodynamicModel>()).ToList();
                double worstRatio = 0, dragSlowMin = double.MaxValue, dragSlowMax = 0;
                int dipped = 0, frames = 0, wetFrames = 0;
                string worstWing = "";
                // the submerged portion the wing's last update read: this frame's or the one before (the game sets it
                // around FAR's update), so a wing counts only when both were part-submerged
                var lastSub = new Dictionary<FARWingAerodynamicModel, double>();
                var seen = new List<string>();
                var partFrames = new Dictionary<string, (int n, double maxLift)>();   // any frame part-submerged, per wing part
                // 3 s: at 9 m/s down a wing crosses the surface inside a frame, and only the winglets, bobbing, sat
                // part-submerged two frames running: 2 to 8 such frames in 1.5 s, once none
                while (Planetarium.GetUniversalTime() - t0 < 3 && frames < 400)
                {
                    yield return fixedWait;
                    frames++;
                    foreach (Part p in v.parts)
                    {
                        FARAeroPartModule m = p.Modules.GetModule<FARAeroPartModule>();
                        if (m == null) continue;
                        Assert.True(!float.IsNaN(m.worldSpaceAeroForce.sqrMagnitude) && !float.IsInfinity(m.worldSpaceAeroForce.sqrMagnitude),
                                    $"{p.partInfo.name}'s aero force is real in the water");
                    }

                    if (v.parts.Any(p => p.submergedPortion > 0))
                    {
                        wetFrames++;
                        dragSlowMin = Math.Min(dragSlowMin, PhysicsGlobals.BuoyancyWaterDragSlow);
                        dragSlowMax = Math.Max(dragSlowMax, PhysicsGlobals.BuoyancyWaterDragSlow);
                    }

                    foreach (FARWingAerodynamicModel w in wings)
                    {
                        if (w.part == null || w.part.vessel != v || w.isShielded) continue;
                        Assert.True(!float.IsNaN(w.worldSpaceForce.sqrMagnitude) && !float.IsInfinity(w.worldSpaceForce.sqrMagnitude),
                                    $"{w.part.partInfo.name}'s wing force is real in the water");
                        double sub = w.part.submergedPortion;
                        bool before = lastSub.TryGetValue(w, out double prev) && prev > 0.06 && prev < 0.94;
                        lastSub[w] = sub;
                        if (sub > 0.06 && sub < 0.94)
                        {
                            string wn = w.part.partInfo.name;
                            partFrames.TryGetValue(wn, out (int n, double maxLift) pf);
                            partFrames[wn] = (pf.n + 1, Math.Max(pf.maxLift, w.liftForceWing));
                        }
                        if (!before || sub < 0.06 || sub > 0.94 || w.liftForceWing < 0.05)
                            continue;
                        // the force along FAR's own lift direction this frame (perpendicular to the flow at the wing's
                        // centroid): the water drag, a thousand times the air's, drops out of it exactly. Taken across
                        // the part's velocity after the physics step instead, a few degrees of a bouncing plane's turn
                        // read 15% of the lift kept (22.6 of 148 kN, at 0.23 under)
                        Vector3 liftDir = ((Vector3)w.GetLiftDirection()).normalized;
                        double lift = Math.Abs(Vector3.Dot(w.worldSpaceForce, liftDir));
                        double ratio = lift / w.liftForceWing;
                        dipped++;
                        if (seen.Count < 12)
                            seen.Add($"{w.part.partInfo.name} {prev:F2}/{sub:F2} {ratio:F3}");
                        if (ratio > worstRatio)
                        {
                            worstRatio = ratio;
                            worstWing = $"{w.part.partInfo.name} {sub:F2} under, lift {lift:F3} of {w.liftForceWing:F2} kN";
                        }
                    }
                }

                double dt = Planetarium.GetUniversalTime() - t0, speed1 = v.horizontalSrfSpeed;
                double waterDecel = (speed0 - speed1) / dt;
                ctx.Record("water", $"{speed1:F1} m/s after {dt:F2} s: {waterDecel:F2} m/s2 (air {airDecel:F3}); splashed {v.Splashed}");
                ctx.Record("wings", $"{dipped} part-submerged wing frames, the most lift kept {worstRatio:F4} ({worstWing})");
                ctx.Record("dippedWings", string.Join("; ", seen));
                ctx.Record("partSubmerged", string.Join(", ", partFrames.Select(kv => $"{kv.Key} {kv.Value.n} frames (lift up to {kv.Value.maxLift:F2} kN)")));
                ctx.Record("waterDragSlow", $"{dragSlowMin:G4} to {dragSlowMax:G4} over {wetFrames} wet frames (stock {waterDragWas:G4})");

                Assert.True(dipped > 0, "a wing partly in the water at some frame");
                Assert.True(worstRatio < 0.05, $"a part-submerged wing keeps only a sliver of its lift ({worstWing})");
                Assert.True(waterDecel > 5 * airDecel, $"the water slows the plane far faster than the air ({waterDecel:F2} against {airDecel:F3} m/s2)");
                Assert.True(wetFrames > 0 && dragSlowMin >= 0 && !double.IsInfinity(dragSlowMax) && dragSlowMax > 0,
                            $"FAR's water drag setting real while in the water ({dragSlowMin:G4} to {dragSlowMax:G4})");
                exceptions.AssertNone("ditching");
            }
            finally
            {
                PhysicsGlobals.AeroDataDisplay = dataWas;
                CheatOptions.NoCrashDamage = crashWas;
                PhysicsGlobals.BuoyancyWaterDragSlow = (float)waterDragWas;
            }
        }
    }
}
