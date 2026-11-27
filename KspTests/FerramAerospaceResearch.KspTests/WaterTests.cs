using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
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
        /// A stock plane ditching: gliding level at 60 m/s, engine off and gear up, it is set down nose up (entryPitch,
        /// default 5 deg), not turning, its lowest point 0.3 m above the sea (crash damage off). In the water, the wings
        /// that are partly under keep about a hundredth of the lift FAR works out for them (FAR's lift collapse for a
        /// part-submerged wing; with FAR's hydrodynamics, at most their dry share's), the plane loses energy many times
        /// faster than it did in the air at the same speed and gains none (it and any pieces it breaks into), FAR's water
        /// drag setting (BuoyancyWaterDragSlow) is set and real, every force stays finite, and FAR throws nothing. Every
        /// step is traced in ditch.csv (the forces along the travel, each force's power, the plane's energy); videoSec > 0
        /// films it from the side (ditch.mp4, StartVideo) that long.
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
            PlaningVideoTests.Recorder rec = null;
            try
            {
                // (the model knobs: hd.<name> and the rest, WaterParams)
                WaterParams.Apply(ctx);
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

                // videoSec > 0: filmed from the set-down on (StartVideo)
                double videoSec = ctx.ParamDouble("videoSec", 0);
                if (videoSec > 0)
                {
                    yield return SideCamera(v, (float)ctx.ParamDouble("camDistance", 40), (float)ctx.ParamDouble("camPitch", 8));
                    rec = StartVideo(ctx, "DitchVideoRecorder");
                }

                ctx.Step("set down just above the sea");
                // as a pilot puts it down: nose up (entryPitch, default 5 deg; the glide holds it 4 deg down, pitching
                // down at 6 deg/s, which dug the nose in) and not turning, wings level
                double entryPitch = ctx.ParamDouble("entryPitch", 5);
                {
                    Vector3 nose = v.ReferenceTransform.up, axis = v.ReferenceTransform.right;
                    float d = (float)(entryPitch - Pitch(v));
                    // (the sense of a turn about the wing's axis: whichever brings the nose to the pitch)
                    float PitchAfter(float a) => Mathf.Asin(Mathf.Clamp(Vector3.Dot(Quaternion.AngleAxis(a, axis) * nose, v.upAxis), -1, 1)) * Mathf.Rad2Deg;
                    float turn = Math.Abs(PitchAfter(d) - entryPitch) < Math.Abs(PitchAfter(-d) - entryPitch) ? d : -d;
                    v.SetRotation(Quaternion.AngleAxis(turn, axis) * v.vesselTransform.rotation);
                    foreach (Part p in v.parts)
                        if (p.rb != null)
                            p.rb.angularVelocity = Vector3.zero;
                }
                // its lowest point 0.3 m above the water and sinking at 1 m/s: dropped from 3 m it hit at 9 m/s, a wing
                // crossed the surface inside a physics frame, and a plane that stopped dead had no wing part-submerged two
                // frames running (the check below needs that). Set by its CoM a meter up instead, its belly started under
                // the water, its whole run wetted in the first step
                Physics.SyncTransforms();
                v.SetPosition(v.vesselTransform.position - (Vector3d)v.upAxis * (GearDitchTests.Lowest(v) - 0.3));
                v.ChangeWorldVelocity((Vector3d)v.upAxis * (-v.verticalSpeed - 1));
                var fixedWait = new WaitForFixedUpdate();
                yield return Wait.Until(() => v.Splashed || v.parts.Any(p => p.submergedPortion > 0), 15, "the plane in the water", shot: false);
                double t0 = Planetarium.GetUniversalTime(), speed0 = v.horizontalSrfSpeed, entrySpeed = v.srfSpeed;
                ctx.Record("splash", $"{speed0:F1} m/s horizontal, {-v.verticalSpeed:F1} m/s down at {v.altitude:F1} m");
                int parts0 = v.parts.Count;
                List<Part> pieces = v.parts.ToList();
                Vessel near = FlightGlobals.VesselsLoaded.Where(o => o != v).OrderBy(o => (o.CoMD - v.CoMD).magnitude).FirstOrDefault();
                ctx.Record("entry", $"pitch {Pitch(v):F1} deg, roll {Roll(v):F1} deg, turning {v.angularVelocityD.magnitude * Mathf.Rad2Deg:F1} deg/s, " +
                                    $"{parts0} parts; nearest other vessel {(near == null ? "none" : $"{near.vesselName} {(near.CoMD - v.CoMD).magnitude:F0} m")}");
                // every step in the water (ditch.csv): a bounce, a part lost or a speed gained shows where
                // (forces along the travel, kN: positive pushes the plane on. water = PartHydrodynamics' total, then its
                // terms; body = FAR's aero force on the parts, wings = the wing model's)
                var trace = new StringBuilder("t,alt,vs,hs,speed,pitch,roll,turnDegS,parts,wetParts,splashed," +
                                              "water,momentum,friction,impact,crossflow,hydrostatic,wave,radiation,body,wings," +
                                              "waterPowerKW,wingPowerKW,bodyPowerKW,buoyPowerKW,energyMJ,onPlane," +
                                              "pMomentumKW,pImpactKW,pCrossflowKW,pFrictionKW,pRadiationKW,pWaveKW,pTorqueKW,pHydrostaticKW\n");

                List<FARWingAerodynamicModel> wings = v.parts.SelectMany(p => p.Modules.OfType<FARWingAerodynamicModel>()).ToList();
                // the fastest and deepest it went, the fewest parts it had, and the work (MJ) the wings did pushing it on
                double topSpeed = entrySpeed, deepest = v.altitude, wingPush = 0, entryEnergy = 0, peakEnergy = double.MinValue;
                int partsLeast = parts0;
                // its energy (J/kg, every piece) lost over the steps it was in the water, and the most it gained in one: a
                // plane at flying speed may skip and fly on, the air's steps no part of the water's deceleration
                double massT = pieces.Where(p => p.rb != null).Sum(p => (double)p.rb.mass), ePrev = double.NaN, eWetLoss = 0, wetTime = 0, eWetGainMax = 0;
                string gainAt = "";
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
                    trace.AppendLine(TraceLine(v, pieces, Planetarium.GetUniversalTime() - t0, out double wingKW, out double energyMJ));
                    if (frames == 1)
                        entryEnergy = energyMJ;
                    peakEnergy = Math.Max(peakEnergy, energyMJ);
                    double eNow = energyMJ * 1000 / massT;
                    if (!double.IsNaN(ePrev) && v.parts.Any(p => p.submergedPortion > 0))
                    {
                        wetTime += TimeWarp.fixedDeltaTime;
                        eWetLoss += ePrev - eNow;
                        if (eNow - ePrev > eWetGainMax)
                        {
                            eWetGainMax = eNow - ePrev;
                            gainAt = $"{Planetarium.GetUniversalTime() - t0:F2} s, {v.srfSpeed:F1} m/s, pitch {Pitch(v):F0} deg, {v.parts.Count} parts";
                        }
                    }
                    ePrev = eNow;
                    if (rec != null)
                        rec.Lines = Overlay(v, Planetarium.GetUniversalTime() - t0, parts0);
                    topSpeed = Math.Max(topSpeed, v.srfSpeed);
                    deepest = Math.Min(deepest, v.altitude);
                    partsLeast = Math.Min(partsLeast, v.parts.Count);
                    if (wingKW > 0)
                        wingPush += wingKW * TimeWarp.fixedDeltaTime / 1000;
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
                // (the energy lost in the water as a deceleration at the speed it came in at: loss / (time x speed))
                double waterDecel = wetTime > 0 ? eWetLoss / (wetTime * Math.Max(1, speed0)) : 0;
                ctx.Record("water", $"{speed1:F1} m/s after {dt:F2} s ({(speed0 - speed1) / dt:F2} m/s2 over it); in the water {wetTime:F2} s of it, " +
                                    $"energy {eWetLoss:F0} J/kg lost there ({waterDecel:F2} m/s2 at {speed0:F0} m/s), most gained in a step " +
                                    $"{eWetGainMax:F1} J/kg ({gainAt}) (air {airDecel:F3}); splashed {v.Splashed}");
                ctx.Record("ride", $"top speed {topSpeed:F1} m/s (entry {entrySpeed:F1}), deepest {deepest:F1} m, parts {partsLeast} of {parts0}, " +
                                   $"energy {entryEnergy:F2} MJ at entry, {peakEnergy:F2} at most, {v.srfSpeed:F1} m/s now; " +
                                   $"the wings' positive work {wingPush:F2} MJ");
                // the video on past the measured 3 s, still traced
                while (rec != null && Planetarium.GetUniversalTime() - t0 < videoSec)
                {
                    yield return fixedWait;
                    trace.AppendLine(TraceLine(v, pieces, Planetarium.GetUniversalTime() - t0, out _, out _));
                    rec.Lines = Overlay(v, Planetarium.GetUniversalTime() - t0, parts0);
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "ditch.csv"), trace.ToString());
                ctx.Record("wings", $"{dipped} part-submerged wing frames, the most lift kept {worstRatio:F4} ({worstWing})");
                ctx.Record("dippedWings", string.Join("; ", seen));
                ctx.Record("partSubmerged", string.Join(", ", partFrames.Select(kv => $"{kv.Key} {kv.Value.n} frames (lift up to {kv.Value.maxLift:F2} kN)")));
                ctx.Record("waterDragSlow", $"{dragSlowMin:G4} to {dragSlowMax:G4} over {wetFrames} wet frames (stock {waterDragWas:G4})");

                Assert.True(dipped > 0, "a wing partly in the water at some frame");
                if (FARDebugValues.hydrodynamics)
                    // the wing's aero force is its dry share's (PartHydrodynamics has the water): no lift collapse on top
                    Assert.True(worstRatio < 1.05, $"a part-submerged wing's air lift is at most its dry share's ({worstWing})");
                else
                    Assert.True(worstRatio < 0.05, $"a part-submerged wing keeps only a sliver of its lift ({worstWing})");
                Assert.True(waterDecel > 5 * airDecel, $"the water slows the plane far faster than the air ({waterDecel:F2} against {airDecel:F3} m/s2)");
                // the water and the wings in it only take energy from the plane (and the pieces it breaks into): a wing's
                // lift worked out from a flow lagging its motion once drove it 20-40 m under at up to 145 m/s, 2.4 times
                // its entry speed (it still passed the check above, slow by the end). Not its speed: a piece torn off
                // can be flung faster by its joint letting go
                Assert.True(peakEnergy < entryEnergy + 1, $"the plane gains no energy in the water ({peakEnergy:F2} MJ at most, {entryEnergy:F2} at entry)");
                Assert.True(wetFrames > 0 && dragSlowMin >= 0 && !double.IsInfinity(dragSlowMax) && dragSlowMax > 0,
                            $"FAR's water drag setting real while in the water ({dragSlowMin:G4} to {dragSlowMax:G4})");
                exceptions.AssertNone("ditching");
            }
            finally
            {
                StopVideo(rec);
                WaterParams.RecordCrush(ctx);
                WaterParams.Restore();
                PhysicsGlobals.AeroDataDisplay = dataWas;
                CheatOptions.NoCrashDamage = crashWas;
                PhysicsGlobals.BuoyancyWaterDragSlow = (float)waterDragWas;
            }
        }

        // a ditch.csv line; wingKW: the wings' power on the motion they act on (kW; positive: pushing the plane on)
        private static string TraceLine(Vessel v, List<Part> pieces, double t, out double wingKW, out double energyMJ)
        {
            Vector3 along = ((Vector3)v.srf_velocity).normalized;
            double water = 0, momentum = 0, friction = 0, impact = 0, crossflow = 0, hydrostatic = 0, wave = 0, radiation = 0, body = 0, wing = 0;
            Vector3 frame = Krakensbane.GetFrameVelocity();
            foreach (Part p in v.parts)
            {
                FARAeroPartModule m = p.Modules.GetModule<FARAeroPartModule>();
                if (m != null)
                {
                    body += Vector3.Dot(m.worldSpaceAeroForce, along);
                    PartHydrodynamics hd = m.Hydrodynamics;
                    if (hd != null)
                    {
                        water += Vector3.Dot(hd.LastForce, along);
                        momentum += Vector3.Dot(hd.LastMomentumForce, along);
                        friction += Vector3.Dot(hd.LastFrictionForce, along);
                        impact += Vector3.Dot(hd.LastImpactForce, along);
                        crossflow += Vector3.Dot(hd.LastCrossflowForce, along);
                        hydrostatic += Vector3.Dot(hd.LastHydrostaticForce, along);
                        wave += Vector3.Dot(hd.LastWaveForce, along);
                        radiation += Vector3.Dot(hd.LastRadiationForce, along);
                    }
                }
                foreach (FARWingAerodynamicModel w in p.Modules.OfType<FARWingAerodynamicModel>())
                    wing += Vector3.Dot(w.worldSpaceForce, along);
            }
            // power (kW) on every part the plane had at the splash, broken off or not: each force on the motion of the
            // point it acts at (the water's at its part's CoM with its torque, a wing's at its centroid, FAR's body aero
            // and stock's buoyancy at the part's CoM)
            double waterPower = 0, wingPower = 0, bodyPower = 0, buoyPower = 0;
            // (and the water's terms': momentum, impact, crossflow, friction, radiation, wave, its torque, its hydrostatics)
            var terms = new double[8];
            foreach (Part p in pieces)
            {
                if (p == null || p.rb == null)
                    continue;
                Vector3 pv = p.rb.velocity + frame;
                FARAeroPartModule m = p.Modules.GetModule<FARAeroPartModule>();
                if (m != null)
                {
                    bodyPower += Vector3.Dot(m.worldSpaceAeroForce, pv);
                    PartHydrodynamics hd = m.Hydrodynamics;
                    if (hd != null)
                    {
                        waterPower += Vector3.Dot(hd.LastForce, pv) + Vector3.Dot(hd.LastTorque, p.rb.angularVelocity);
                        terms[0] += Vector3.Dot(hd.LastMomentumForce, pv);
                        terms[1] += Vector3.Dot(hd.LastImpactForce, pv);
                        terms[2] += Vector3.Dot(hd.LastCrossflowForce, pv);
                        terms[3] += Vector3.Dot(hd.LastFrictionForce, pv);
                        terms[4] += Vector3.Dot(hd.LastRadiationForce, pv);
                        terms[5] += Vector3.Dot(hd.LastWaveForce, pv);
                        terms[6] += Vector3.Dot(hd.LastTorque, p.rb.angularVelocity);
                        terms[7] += Vector3.Dot(hd.LastHydrostaticForce, pv) + Vector3.Dot(hd.LastHydrostaticTorque, p.rb.angularVelocity);
                    }
                }
                if (p.partBuoyancy != null)
                    buoyPower += Vector3.Dot(p.partBuoyancy.effectiveForce, pv);
                foreach (FARWingAerodynamicModel w in p.Modules.OfType<FARWingAerodynamicModel>())
                    wingPower += Vector3.Dot(w.worldSpaceForce, p.rb.GetPointVelocity(w.LastCentroid) + frame);
            }
            // the plane's mechanical energy (MJ), every part it had at the splash wherever it is now (a part broken off
            // still counts): kinetic (each part's motion and spin; low down the physics frame turns with the planet, so a
            // rigidbody's velocity with Krakensbane's is the surface's) and potential (g at the surface, above sea level)
            double ke = 0, pe = 0, g = v.mainBody.GeeASL * 9.80665;
            foreach (Part p in pieces)
                if (p != null && p.rb != null)
                {
                    Vector3 vel = p.rb.velocity + frame;
                    Vector3 w = Quaternion.Inverse(p.rb.rotation * p.rb.inertiaTensorRotation) * p.rb.angularVelocity;
                    ke += 0.5 * p.rb.mass * vel.sqrMagnitude + 0.5 * Vector3.Dot(p.rb.inertiaTensor, Vector3.Scale(w, w));
                    pe += p.rb.mass * g * FlightGlobals.getAltitudeAtPos(p.rb.worldCenterOfMass, v.mainBody);
                }
            string line = string.Join(",", new[]
            {
                t, v.altitude, v.verticalSpeed, v.horizontalSrfSpeed, v.srfSpeed, Pitch(v), Roll(v), v.angularVelocityD.magnitude * Mathf.Rad2Deg,
                v.parts.Count, v.parts.Count(p => p.submergedPortion > 0), v.Splashed ? 1 : 0,
                water, momentum, friction, impact, crossflow, hydrostatic, wave, radiation, body, wing,
                waterPower, wingPower, bodyPower, buoyPower, (ke + pe) / 1000, pieces.Count(p => p != null && p.vessel == v)
            }.Concat(terms).Select(x => x.ToString("G6", CultureInfo.InvariantCulture)));
            wingKW = wingPower;
            energyMJ = (ke + pe) / 1000;
            return line;
        }

        private static string[] Overlay(Vessel v, double t, int parts0)
        {
            Vector3 along = ((Vector3)v.srf_velocity).normalized;
            double water = 0, wing = 0;
            foreach (Part p in v.parts)
            {
                PartHydrodynamics hd = p.Modules.GetModule<FARAeroPartModule>()?.Hydrodynamics;
                if (hd != null)
                    water += Vector3.Dot(hd.LastForce, along);
                foreach (FARWingAerodynamicModel w in p.Modules.OfType<FARWingAerodynamicModel>())
                    wing += Vector3.Dot(w.worldSpaceForce, along);
            }
            return new[]
            {
                $"Aeris 4A ditching at 60 m/s  -  FAR {(FARDebugValues.hydrodynamics ? "PartHydrodynamics" : "legacy water forces")}",
                $"t {t,5:F2} s   speed {v.srfSpeed,6:F1} m/s   height {v.altitude,6:F1} m",
                $"pitch {Pitch(v),5:F0} deg   roll {Roll(v),5:F0} deg   parts {v.parts.Count}/{parts0}   wet {v.parts.Count(p => p.submergedPortion > 0)}",
                $"along travel: water {water,8:F0} kN   wings {wing,8:F0} kN"
            };
        }

        // a ditching filmed (the camera set first, SideCamera): ditch.mp4 in the results folder, 30 frames a game second,
        // piped to ffmpeg (param ffmpeg: its path) with the game's sound (soundVolume); where ffmpeg will not start,
        // frames/NNNNN.jpg instead
        internal static PlaningVideoTests.Recorder StartVideo(TestContext ctx, string name)
        {
            GameEvents.onHideUI.Fire();
            string frameDir = Path.Combine(ctx.ResultsDir, "frames");
            Directory.CreateDirectory(frameDir);
            PlaningVideoTests.Recorder rec = new GameObject(name).AddComponent<PlaningVideoTests.Recorder>();
            rec.Dir = frameDir;
            rec.Video = Path.Combine(ctx.ResultsDir, "ditch.mp4");
            if (!string.IsNullOrEmpty(ctx.Param("ffmpeg")))
                rec.Ffmpeg = ctx.Param("ffmpeg");
            rec.SoundVolume = (float)ctx.ParamDouble("soundVolume", 0.5);
            Time.captureFramerate = 30;
            return rec;
        }

        internal static void StopVideo(PlaningVideoTests.Recorder rec)
        {
            Time.captureFramerate = 0;
            if (rec == null)
                return;
            UnityEngine.Object.Destroy(rec.gameObject);
            GameEvents.onShowUI.Fire();
        }

        // the camera abeam the plane, level-ish (pitch deg above the horizon); its heading found by trying two
        // (camHdg's zero and sense depend on the camera mode), as PlaningVideoTests does
        internal static IEnumerator SideCamera(Vessel v, float dist, float pitchDeg)
        {
            FlightCamera cam = FlightCamera.fetch;
            Vector3 up = v.upAxis, h = Vector3.ProjectOnPlane(v.srf_velocity, up).normalized;
            Vector3 look = -Vector3.Cross(up, h).normalized;
            float camPitch = pitchDeg * Mathf.Deg2Rad, a0 = 0, a1 = 0;
            cam.minPitch = -0.5f;
            cam.SetDistanceImmediate(dist);
            for (int pass = 0; pass < 2; pass++)
            {
                float hdg = pass == 0 ? 0f : 0.5f;
                for (int i = 0; i < 10; i++)
                {
                    cam.camHdg = hdg;
                    cam.camPitch = camPitch;
                    yield return null;
                }
                Vector3 f = Vector3.ProjectOnPlane(cam.transform.forward, up).normalized;
                float a = Vector3.SignedAngle(look, f, up) * Mathf.Deg2Rad;
                if (pass == 0) a0 = a;
                else a1 = a;
            }
            float slope = Mathf.DeltaAngle(a0 * Mathf.Rad2Deg, a1 * Mathf.Rad2Deg) * Mathf.Deg2Rad / 0.5f;
            cam.camHdg = Math.Abs(slope) > 0.2f ? -a0 / slope : 0;
            cam.camPitch = camPitch;
        }

        // the nose above the horizon and the right wing above it (deg): an SPH craft's up is its nose, its right its right wing
        private static double Pitch(Vessel v) => Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.up, v.upAxis), -1, 1)) * Mathf.Rad2Deg;
        private static double Roll(Vessel v) => Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.right, v.upAxis), -1, 1)) * Mathf.Rad2Deg;
    }
}
