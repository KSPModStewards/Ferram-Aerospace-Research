using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using ferram4;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's control surfaces on the stock plane beyond the plain stick inputs: the part menu's tweakables (pitch due
    /// to angle of attack, brake rudder, dynamic deflection) in flight and in FAR's snapshot predictions, the flap and
    /// spoiler buttons and action groups, and the torque each surface reports to the game's steering (ITorqueProvider).
    /// </summary>
    public static class ControlSurfaceTests
    {
        /// <summary>The vessel's horizontal control surfaces: the elevons, flaps and canards (not the rudder).</summary>
        private static List<FARControllableSurface> HorizontalSurfaces(Vessel v)
        {
            Vector3 belly = v.ReferenceTransform.forward;
            return v.parts.SelectMany(p => p.Modules.OfType<FARControllableSurface>())
                    .Where(s => Math.Abs(Vector3.Dot(s.part.partTransform.forward, belly)) > 0.7)
                    .ToList();
        }

        /// <summary>A control surface's part-menu settings, to put back.</summary>
        private sealed class Tweakables
        {
            private readonly FARControllableSurface s;
            private readonly float pitch, yaw, roll, pitchDueToAoA, brakeRudder, startSpeed;
            private readonly bool dynamic, isFlap, isSpoiler, brake;
            private readonly int level;

            public Tweakables(FARControllableSurface s)
            {
                this.s = s;
                pitch = s.pitchaxis;
                yaw = s.yawaxis;
                roll = s.rollaxis;
                pitchDueToAoA = s.pitchaxisDueToAoA;
                brakeRudder = s.brakeRudder;
                startSpeed = s.dynamicControlStartSpeed;
                dynamic = s.isDynamicDeflection;
                isFlap = s.isFlap;
                isSpoiler = s.isSpoiler;
                brake = s.brake;
                level = s.flapDeflectionLevel;
            }

            public void Restore()
            {
                s.pitchaxis = pitch;
                s.yawaxis = yaw;
                s.rollaxis = roll;
                s.pitchaxisDueToAoA = pitchDueToAoA;
                s.brakeRudder = brakeRudder;
                s.dynamicControlStartSpeed = startSpeed;
                s.isDynamicDeflection = dynamic;
                s.isFlap = isFlap;
                s.isSpoiler = isSpoiler;
                s.brake = brake;
                s.SetDeflection(level);
            }
        }

        private static string Deflections(IEnumerable<FARControllableSurface> surfaces) =>
            string.Join(", ", surfaces.Select(s => $"{s.part.partInfo.name} {s.ControlDeflection:F2}"));

        /// <summary>Holds the condition until the surfaces stop moving; their deflections then.</summary>
        private static IEnumerator Deflect(FarFlightProbe.Condition c, List<FARControllableSurface> surfaces, double[] into)
        {
            yield return FarFlightProbe.Place(c);
            yield return FarFlightProbe.SettleSurfaces(c);
            for (int i = 0; i < surfaces.Count; i++)
                into[i] = surfaces[i].ControlDeflection;
        }

        /// <summary>The snapshot's torque at the vessel's state, with the control inputs given.</summary>
        private static Vector3 SnapshotTorque(FARVesselSnapshot snap, Vessel v, float pitch, float yaw, float roll)
        {
            var where = new Vector3d(v.latitude, v.longitude, v.altitude);
            Assert.True(FARAPI.CalculateSnapshotAeroForcesWithControlsAt(snap, out _, out Vector3 tq, v.srf_velocity, where,
                                                                          Planetarium.GetUniversalTime(), pitch, yaw, roll),
                        $"a snapshot prediction with inputs {pitch}, {yaw}, {roll}");
            return tq;
        }

        /// <summary>The deflection the main thread's last snapshot prediction gave a surface (its prediction slot's scratch).</summary>
        private static double SnapshotDeflection(FARControllableSurface s) =>
            Refl.Get<ControlSurfaceScratch[]>(s, "_ctrlScratchSlots")[SimThreadContext.MainThreadPredictionSlot]?.AoAoffset ?? double.NaN;

        /// <summary>The snapshot's deflections against flight's, surface by surface.</summary>
        private static void SameDeflections(string what, List<FARControllableSurface> surfaces, double[] flight, double[] snapshot)
        {
            for (int k = 0; k < surfaces.Count; k++)
                Assert.Near(flight[k], snapshot[k], 0.05 * Math.Abs(flight[k]) + 0.15,
                            $"{what}: {surfaces[k].part.partInfo.name}'s deflection in the snapshot ({snapshot[k]:F2}) as in flight ({flight[k]:F2} deg)");
        }

        /// <summary>
        /// The elevons' tweakables at Mach 0.5 and 6 deg. Pitch due to angle of attack (50% either way, stick centred)
        /// deflects them by about half the angle they meet the air at, opposite ways for the two signs, and FAR's
        /// snapshot prediction deflects them as flight does. As brake rudders (their stick axes
        /// off), yaw one way deflects only the elevons on one side, the other way only the others, in flight and in
        /// the snapshot. With dynamic deflection from 60 m/s, full stick deflects them
        /// by FAR's speed and density factor of their full travel, in flight and in the snapshot, whose stick authority
        /// falls with it.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_SurfaceTweakablesDeflect(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0.5, 3000, 6);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            List<FARControllableSurface> all = v.parts.SelectMany(p => p.Modules.OfType<FARControllableSurface>()).ToList();
            List<FARControllableSurface> elevons = HorizontalSurfaces(v).Where(s => s.pitchaxis > 0).ToList();
            Assert.True(elevons.Count >= 2, $"the plane's elevons ({elevons.Count})");
            List<Tweakables> saved = all.Select(s => new Tweakables(s)).ToList();
            int n = elevons.Count;
            Vector3 pitchAxis = Flight.PitchUpAxis(v), rollAxis = v.ReferenceTransform.up;
            try
            {
                Ctrl.Attach();
                Ctrl.Set(pitch: 0, yaw: 0, roll: 0);

                ctx.Step("pitch due to angle of attack");
                var d0 = new double[n];
                var dUp = new double[n];
                var dDown = new double[n];
                yield return Deflect(c, elevons, d0);
                FARVesselSnapshot snap0 = FARAPI.CaptureVesselSnapshot(v);
                double m0 = Vector3.Dot(SnapshotTorque(snap0, v, 0, 0, 0), pitchAxis);
                var samples = new List<FarFlightProbe.Sample>();
                yield return FarFlightProbe.Hold(c, 5, 20, samples, predict: false);
                double flight0 = samples.Average(s => s.Flight[3]);
                foreach (FARControllableSurface s in elevons)
                    s.pitchaxisDueToAoA = 50;
                yield return Deflect(c, elevons, dUp);
                FARVesselSnapshot snapUp = FARAPI.CaptureVesselSnapshot(v);
                double mUp = Vector3.Dot(SnapshotTorque(snapUp, v, 0, 0, 0), pitchAxis);
                double[] snapDefl = elevons.Select(SnapshotDeflection).ToArray();
                samples.Clear();
                yield return FarFlightProbe.Hold(c, 5, 20, samples, predict: false);
                double flightUp = samples.Average(s => s.Flight[3]);
                foreach (FARControllableSurface s in elevons)
                    s.pitchaxisDueToAoA = -50;
                yield return Deflect(c, elevons, dDown);
                ctx.Record("dueToAoA", $"0: {string.Join(", ", d0.Select(x => x.ToString("F2")))}; +50: {string.Join(", ", dUp.Select(x => x.ToString("F2")))}; " +
                                       $"-50: {string.Join(", ", dDown.Select(x => x.ToString("F2")))}");
                ctx.Record("dueToAoASnapshot", string.Join(", ", snapDefl.Select(x => x.ToString("F2"))));
                ctx.Record("dueToAoAMoment", $"flight {flight0:F2} -> {flightUp:F2} kN m, snapshot {m0:F2} -> {mUp:F2} kN m");
                for (int k = 0; k < n; k++)
                {
                    string name = elevons[k].part.partInfo.name;
                    double up = dUp[k] - d0[k], down = dDown[k] - d0[k];
                    Assert.InRange(1, 6, Math.Abs(up), $"{name} deflects by about half its angle of attack ({up:F2} deg at 6 deg)");
                    Assert.Near(-up, down, 0.25 * Math.Abs(up) + 0.1, $"{name} deflects the other way for -50% ({down:F2} against {up:F2} deg)");
                }
                SameDeflections("pitch due to angle of attack", elevons, dUp, snapDefl);
                foreach (FARControllableSurface s in elevons)
                    s.pitchaxisDueToAoA = 0;

                ctx.Step("brake rudders");
                foreach (FARControllableSurface s in all)
                {
                    s.yawaxis = 0;
                    if (!elevons.Contains(s)) continue;
                    s.pitchaxis = s.rollaxis = 0;
                    s.brakeRudder = 100;
                }
                var dRight = new double[n];
                var dLeft = new double[n];
                Ctrl.Set(pitch: 0, yaw: 1, roll: 0);
                yield return Deflect(c, elevons, dRight);
                FARVesselSnapshot snapBrake = FARAPI.CaptureVesselSnapshot(v);
                Ctrl.Set(pitch: 0, yaw: -1, roll: 0);
                yield return Deflect(c, elevons, dLeft);
                Ctrl.Set(pitch: 0, yaw: 0, roll: 0);
                ctx.Record("brakeRudder", $"yaw +1: {string.Join(", ", dRight.Select(x => x.ToString("F2")))}; yaw -1: {string.Join(", ", dLeft.Select(x => x.ToString("F2")))}");
                int rightSide = 0, leftSide = 0;
                for (int k = 0; k < n; k++)
                {
                    string name = elevons[k].part.partInfo.name;
                    double max = Math.Abs(elevons[k].maxdeflect);
                    bool right = Math.Abs(dRight[k]) > 0.3 * max, left = Math.Abs(dLeft[k]) > 0.3 * max;
                    Assert.True(right != left, $"{name} brakes for yaw one way only ({dRight[k]:F2} for +1, {dLeft[k]:F2} for -1 deg of {max})");
                    Assert.True(Math.Abs(right ? dLeft[k] : dRight[k]) < 0.5, $"{name} stays put for yaw the other way");
                    if (right) rightSide++;
                    else leftSide++;
                }
                Assert.True(rightSide > 0 && leftSide > 0, $"brake rudders on both sides ({rightSide} for +1, {leftSide} for -1)");
                double r0 = Vector3.Dot(SnapshotTorque(snapBrake, v, 0, 0, 0), rollAxis);
                double rRight = Vector3.Dot(SnapshotTorque(snapBrake, v, 0, 1, 0), rollAxis) - r0;
                SameDeflections("brake rudders, yaw +1", elevons, dRight, elevons.Select(SnapshotDeflection).ToArray());
                double rLeft = Vector3.Dot(SnapshotTorque(snapBrake, v, 0, -1, 0), rollAxis) - r0;
                SameDeflections("brake rudders, yaw -1", elevons, dLeft, elevons.Select(SnapshotDeflection).ToArray());
                ctx.Record("brakeRudderSnapshotRoll", $"yaw +1 {rRight:F2}, yaw -1 {rLeft:F2} kN m");
                foreach (Tweakables t in saved)
                    t.Restore();

                ctx.Step("dynamic deflection");
                var dFull = new double[n];
                var dDyn = new double[n];
                Ctrl.Set(pitch: 1, yaw: 0, roll: 0);
                yield return Deflect(c, elevons, dFull);
                FARVesselSnapshot snapFull = FARAPI.CaptureVesselSnapshot(v);
                foreach (FARControllableSurface s in elevons)
                {
                    s.isDynamicDeflection = true;
                    s.dynamicControlStartSpeed = 60;
                }
                yield return Deflect(c, elevons, dDyn);
                FARVesselSnapshot snapDyn = FARAPI.CaptureVesselSnapshot(v);
                Ctrl.Set(pitch: 0, yaw: 0, roll: 0);
                FARControllableSurface e = elevons[0];
                double factor = Math.Pow(e.dynamicControlStartSpeed / v.srfSpeed, e.exponent) / (v.atmDensity / v.lastBody.atmDensityASL);
                factor = Math.Max(e.minControl, Math.Min(1, factor));
                ctx.Record("dynamic", $"factor {factor:F3}; full {string.Join(", ", dFull.Select(x => x.ToString("F2")))}; dynamic {string.Join(", ", dDyn.Select(x => x.ToString("F2")))}");
                Assert.True(factor < 0.9, $"FAR's factor cuts the deflection at {v.srfSpeed:F0} m/s ({factor:F3})");
                for (int k = 0; k < n; k++)
                    if (Math.Abs(dFull[k]) > 1)
                        Assert.Near(factor * dFull[k], dDyn[k], 0.1 * Math.Abs(dFull[k]) + 0.1,
                                    $"{elevons[k].part.partInfo.name}'s deflection scaled by the dynamic factor ({dDyn[k]:F2} of {dFull[k]:F2} deg)");
                double fullStick = Vector3.Dot(SnapshotTorque(snapFull, v, 1, 0, 0) - SnapshotTorque(snapFull, v, 0, 0, 0), pitchAxis);
                Vector3 dynTorque = SnapshotTorque(snapDyn, v, 1, 0, 0);
                SameDeflections("dynamic deflection, full stick", elevons, dDyn, elevons.Select(SnapshotDeflection).ToArray());
                double dynStick = Vector3.Dot(dynTorque - SnapshotTorque(snapDyn, v, 0, 0, 0), pitchAxis);
                ctx.Record("dynamicSnapshot", $"full stick moment {fullStick:F2}, with dynamic deflection {dynStick:F2} kN m");
                Assert.True(Math.Abs(dynStick) < 0.9 * Math.Abs(fullStick) && Math.Sign(dynStick) == Math.Sign(fullStick),
                            $"the snapshot's stick authority cut by dynamic deflection too ({dynStick:F2} against {fullStick:F2} kN m)");
            }
            finally
            {
                Ctrl.Detach();
                foreach (Tweakables t in saved)
                    t.Restore();
            }
        }

        /// <summary>
        /// The flap buttons in the part menu step a flap and its mirror twin together; the flap action groups' actions
        /// step a flap up to full and back; the brakes action group puts every surface's spoiler out and back in.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Behaviour_FlapsAndSpoilersFromTheirControls(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            Vessel v = FlightGlobals.ActiveVessel;
            List<FARControllableSurface> all = v.parts.SelectMany(p => p.Modules.OfType<FARControllableSurface>()).ToList();
            List<Tweakables> saved = all.Select(s => new Tweakables(s)).ToList();
            try
            {
                FARControllableSurface flap = HorizontalSurfaces(v).FirstOrDefault(s => s.part.symmetryCounterparts.Count > 0);
                Assert.NotNull(flap, "a horizontal surface with a mirror twin");
                FARControllableSurface twin = flap.part.symmetryCounterparts[0].Modules.OfType<FARControllableSurface>().First();
                foreach (FARControllableSurface s in new[] { flap, twin })
                {
                    s.isFlap = true;
                    s.SetDeflection(0);
                }

                ctx.Step("part menu buttons");
                Assert.True(flap.Events["DeflectMore"].active && !flap.Events["DeflectLess"].active, "flaps up: only the down button offered");
                flap.DeflectMore();
                Assert.Equal(1, flap.flapDeflectionLevel, "the flap one step down");
                Assert.Equal(1, twin.flapDeflectionLevel, "its twin one step down with it");
                flap.DeflectMore();
                flap.DeflectLess();
                Assert.Equal(1, twin.flapDeflectionLevel, "the twin follows the flap back up");
                flap.DeflectLess();
                Assert.Equal(0, flap.flapDeflectionLevel, "the flap up again");
                Assert.Equal(0, twin.flapDeflectionLevel, "its twin up again");

                ctx.Step("flap action groups");
                var param = new KSPActionParam(KSPActionGroup.None, KSPActionType.Activate);
                for (int i = 0; i < 5; i++)
                    flap.Actions["IncreaseDeflect"].Invoke(param);
                Assert.Equal(3, flap.flapDeflectionLevel, "the increase action steps the flap down to full and stops there");
                Assert.False(flap.Events["DeflectMore"].active, "no down button at full flap");
                flap.Actions["DecreaseDeflect"].Invoke(param);
                Assert.Equal(2, flap.flapDeflectionLevel, "the decrease action steps it up one");
                for (int i = 0; i < 4; i++)
                    flap.Actions["DecreaseDeflect"].Invoke(param);
                Assert.Equal(0, flap.flapDeflectionLevel, "and up to nothing");

                ctx.Step("brakes action group");
                Assert.Equal(KSPActionGroup.Brakes, FARActionGroupConfiguration.map(FARActionGroupConfiguration.ID_SPOILER), "spoilers on the brakes group (FAR's default)");
                Ctrl.Brakes(true);
                yield return Wait.Frames(3);
                Assert.True(all.All(s => s.brake), "brakes on: every surface's spoiler out");
                Ctrl.Brakes(false);
                yield return Wait.Frames(3);
                Assert.True(all.All(s => !s.brake), "brakes off: spoilers in");
            }
            finally
            {
                Ctrl.Brakes(false);
                foreach (Tweakables t in saved)
                    t.Restore();
            }
        }

        /// <summary>
        /// The torque each control surface reports to the game's steering (stock SAS, MechJeb and kOS read it): the
        /// elevons pitch, the rudder yaws and the ailerons roll, the negative torque the positive one reversed; at the
        /// same Mach number lower down, in denser air, it grows with the dynamic pressure.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_ControlSurfacesReportTheirTorque(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition high = Far.At(0.5, 6000, 3), low = Far.At(0.5, 1000, 3);
            yield return FlightTests.Airborne(high);
            Vessel v = FlightGlobals.ActiveVessel;
            List<FARControllableSurface> surfaces = v.parts.SelectMany(p => p.Modules.OfType<FARControllableSurface>()).ToList();
            Assert.True(surfaces.Count >= 3, $"the plane's control surfaces ({surfaces.Count})");

            var torque = new Vector3[2][];
            var q = new double[2];
            FarFlightProbe.Condition[] conditions = { high, low };
            for (int i = 0; i < 2; i++)
            {
                ctx.Step(conditions[i].Name);
                yield return FarFlightProbe.Place(conditions[i]);
                yield return FarFlightProbe.Hold(conditions[i], 10, 1, new List<FarFlightProbe.Sample>(), predict: false);
                q[i] = v.dynamicPressurekPa;
                torque[i] = new Vector3[surfaces.Count];
                for (int k = 0; k < surfaces.Count; k++)
                {
                    surfaces[k].GetPotentialTorque(out Vector3 pos, out Vector3 neg);
                    Assert.Near(0, (pos + neg).magnitude, 1e-4 * pos.magnitude + 1e-6, $"{surfaces[k].part.partInfo.name}'s negative torque is its positive one reversed");
                    torque[i][k] = pos;
                }
                ctx.Record($"torque{i}", $"q {q[i]:F2} kPa; " + string.Join(", ", surfaces.Select((s, k) => $"{s.part.partInfo.name} {torque[i][k]}")));
            }

            Vector3 sum = torque[0].Aggregate(Vector3.zero, (a, b) => a + b);
            Assert.True(surfaces.Where(s => s.pitchaxis > 0).All(s => torque[0][surfaces.IndexOf(s)].x > 0), "every pitch surface reports pitch torque");
            Assert.True(sum.x > 0 && sum.y > 0 && sum.z > 0, $"the plane's surfaces give pitch, roll and yaw torque ({sum})");
            Assert.True(surfaces.Where(s => s.pitchaxis.NearlyEqual(0)).All(s => torque[0][surfaces.IndexOf(s)].x.NearlyEqual(0)),
                        "a surface off the pitch axis reports no pitch torque");
            Vector3 sumLow = torque[1].Aggregate(Vector3.zero, (a, b) => a + b);
            double qRatio = q[1] / q[0];
            ctx.Record("ratio", $"q x{qRatio:F2}, pitch torque x{sumLow.x / sum.x:F2}, yaw x{sumLow.z / sum.z:F2}");
            Assert.True(qRatio > 1.4, $"denser air lower down ({qRatio:F2} x the dynamic pressure)");
            Assert.Near(qRatio, sumLow.x / sum.x, 0.15 * qRatio, "the pitch torque grows with the dynamic pressure");
            // the one rudder's torque moves with the sideslip the hold leaves it in: x2.16 to x2.56 over runs at x2.21
            Assert.Near(qRatio, sumLow.z / sum.z, 0.3 * qRatio, "the yaw torque grows with the dynamic pressure");
        }

        /// <summary>
        /// Two of the wing modules' helpers FAR does not call, against the state FAR does use, in cruise with the
        /// elevons deflected (Mach 0.4, 6 deg, their pitch due to angle of attack at 50%). A control surface's angle of attack at a deflection offset
        /// (CalculateAoA(velocity, offset)) is, at the surface's own deflection, the angle FAR works its forces out from
        /// (CalculateAoA(velocity), off the deflected normal), and at no offset the angle off the undeflected surface
        /// (its part's forward axis). A wing part's velocity at a point (GetVelocity(point)) is, at the
        /// part's centre of mass, the velocity FAR gives the part (GetVelocity()), and elsewhere on it that plus the
        /// part's rotation about it (the rigid body turning, given a spin for the check).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_SurfaceAoAAtAnOffsetAndPointVelocity(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0.4, 3000, 6);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            List<FARControllableSurface> surfaces = v.parts.SelectMany(p => p.Modules.OfType<FARControllableSurface>()).ToList();
            List<Tweakables> saved = surfaces.Select(s => new Tweakables(s)).ToList();
            try
            {
                Ctrl.Attach();
                Ctrl.Set(pitch: 0, yaw: 0, roll: 0);
                // pitch due to angle of attack: the elevons deflect by about half the angle they meet the air at
                foreach (FARControllableSurface s in HorizontalSurfaces(v).Where(s => s.pitchaxis > 0))
                    s.pitchaxisDueToAoA = 50;
                yield return FarFlightProbe.Place(c);
                yield return FarFlightProbe.SettleSurfaces(c);
                yield return new WaitForFixedUpdate();
                ctx.Record("deflections", Deflections(surfaces));
                Assert.True(surfaces.Any(s => Math.Abs(s.ControlDeflection) > 0.5), $"some surface deflected ({Deflections(surfaces)})");

                ctx.Step("a control surface's angle of attack at an offset");
                int compared = 0;
                foreach (FARControllableSurface s in surfaces)
                {
                    Vector3d vel = s.GetVelocity();
                    double live = s.CalculateAoA(vel), atOwn = s.CalculateAoA(vel, s.ControlDeflection), atZero = s.CalculateAoA(vel, 0);
                    // undeflected, the surface's normal is its part's forward axis
                    Vector3 normal = Refl.Get<Transform>(s, "part_transform").forward;
                    double undeflected = Math.Asin(Math.Max(-1, Math.Min(1, Vector3d.Dot(normal, vel.normalized))));
                    string name = s.part.partInfo.name;
                    ctx.Record($"aoa_{compared}", $"{name} deflected {s.ControlDeflection:F3}: live {live * FARMathUtil.rad2deg:F4}, at it {atOwn * FARMathUtil.rad2deg:F4}, at 0 {atZero * FARMathUtil.rad2deg:F4}, undeflected {undeflected * FARMathUtil.rad2deg:F4} deg (ctrlSurfFrac {s.ctrlSurfFrac})");
                    // not exact: FAR's own angle comes off the normal of its last deflection step, in the solve's
                    // frozen geometry (0.005 deg off at a 3.5 deg deflection, measured)
                    Assert.Near(live, atOwn, 0.02 * Math.Abs(atOwn - atZero) + 1e-4, $"{name}: at its own deflection, the angle FAR's forces take");
                    Assert.Near(undeflected, atZero, 1e-5, $"{name}: at no offset, the angle off the undeflected surface");
                    if (Math.Abs(s.ControlDeflection) > 0.5)
                        Assert.True(Math.Abs(atOwn - atZero) > 1e-4, $"{name}: its deflection changes the angle");
                    compared++;
                }
                Assert.True(compared >= 3, $"control surfaces compared ({compared})");

                ctx.Step("a wing part's velocity at a point");
                var wings = v.parts.SelectMany(p => p.Modules.OfType<FARWingAerodynamicModel>()).Where(w => w.part.Rigidbody != null).ToList();
                foreach (FARWingAerodynamicModel w in wings.Take(4))
                {
                    Rigidbody rb = w.part.Rigidbody;
                    Vector3 com = rb.worldCenterOfMass, offset = w.part.partTransform.right * 2;
                    Vector3 spin0 = rb.angularVelocity;
                    Vector3d atCom, atPoint, whole;
                    Vector3 spin = new Vector3(0.3f, -0.2f, 0.5f);
                    try
                    {
                        rb.angularVelocity = spin;
                        whole = w.GetVelocity();
                        atCom = w.GetVelocity(com);
                        atPoint = w.GetVelocity(com + offset);
                        spin = rb.angularVelocity;   // as the rigid body keeps it (its maximum angular velocity)
                    }
                    finally
                    {
                        rb.angularVelocity = spin0;
                    }

                    string name = w.part.partInfo.name;
                    Vector3d turning = atPoint - atCom, expected = Vector3.Cross(spin, offset);
                    ctx.Record($"velocity_{name}", $"whole {whole}, at the CoM {atCom}, 2 m out {turning} (spin x r {expected})");
                    Assert.Near(0, (atCom - whole).magnitude, 1e-3 * whole.magnitude + 1e-3, $"{name}: at its centre of mass, the part's velocity");
                    Assert.Near(0, (turning - expected).magnitude, 1e-3 * expected.magnitude + 1e-4, $"{name}: 2 m out, its rotation added");
                    Assert.True(expected.magnitude > 0.5, $"{name}: the spin moves that point ({expected.magnitude:F2} m/s)");
                }
            }
            finally
            {
                foreach (Tweakables t in saved)
                    t.Restore();
            }
        }
    }
}
