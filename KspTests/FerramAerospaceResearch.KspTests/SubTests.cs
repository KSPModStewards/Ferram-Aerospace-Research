using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The Test Submarine (crafts/test_sub.py: DARPA SUBOFF's fully appended configuration scaled to 8 m, the FAR test
    /// electric motor on its tail, its stern appendages B9 control surfaces, a probe core) driven under the water as a
    /// player would: trimmed, dived on its planes, held at depth, turned, surfaced.
    /// </summary>
    public static class SubTests
    {
        // the floatplane tests' (one instance): CustomProceduralParts for the hull, FARTestFloatplane for the motor
        private const string Mods = @"AdvancedJetEngine;Firespitter;TimeControl;..\..\..\CustomProceduralParts\GameData\CustomProceduralParts;" +
                                    @"..\FerramAerospaceResearch.KspTests.Run\mods\FARTestFloatplane";

        /// <summary>
        /// KSP has no ballast tanks, so its fuel is its ballast. Put on the water empty by FarWaterTools.PutOnWater (the
        /// button's code); its buoyancy wholly under the water measured, held captive and still at 15 m; fuelled to
        /// surfaced (a share of it, default 0.88) and put back on the water: it floats, some of it out of the water.
        /// Flooded to awash (its weight its buoyancy less reserve, default 0.005): it settles at the surface and does not
        /// sink. Then under way at speed (m/s, default 4.07: SUBOFF's 3 m/s turning runs at model scale, Froude-scaled
        /// to 8 m; 0: full throttle) with a depth autopilot (pitch to the depth, the diving planes to the pitch, as a
        /// player flies it) to depth (m, default 10, its middle's) for diveSec (default 60) and held there holdSec (30):
        /// it holds its depth without porpoising, at a steady speed. A heading autopilot (the rudders for a turn rate,
        /// as a player steers: SUBOFF keeps no course of its own) holds its heading, then turns it gently (gentleRate,
        /// deg/s, default 3, for 40 s): it turns at that rate, keeping its depth (within 2.5 m) and upright (heel under
        /// 30 deg). Then up to the surface and stopped: it comes up through the surface and floats there. Its CG gb (m,
        /// default 0.1) below its hull's axis (a keel's: a submarine's righting moment). Its turning circles (rudders,
        /// none here): Sub_TurningCircles. Writes sub.csv.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Test Submarine", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1500)]
        public static IEnumerator Sub_Dive(TestContext ctx) => Mission(ctx, "", false);

        /// <summary>
        /// Sub_Dive's start (trimmed, dived, held, turned gently), then its turning circles, recorded, not asserted: each
        /// rudder angle (deg, rudders, default 10+20+35) held for turnSec (30), straightened by the heading autopilot after
        /// it; its steady turning diameter (twice its speed over its turn rate, over the turn's second half) in hull
        /// lengths beside SUBOFF's at that rudder, the mean of two CFD studies of the self-propelled fully appended model
        /// (Delen and Kinaci 2023 at 2.75 m/s, Wang, Wang and Wan 2026 at 3 m/s: 3.08, 2.35 and 1.99 lengths at 10, 20 and
        /// 35 deg). FAR's turns are tighter (its yaw damping N'r and its sail's lift short of Roddy's captive tests:
        /// UnderwaterTests.Tow_Suboff*), and a hard turn rolls it out of its depth past its autopilots' recovery: nothing
        /// after the circles is asserted.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Flight, Craft = "Test Submarine", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1500)]
        public static IEnumerator Sub_TurningCircles(TestContext ctx) => Mission(ctx, "10+20+35", true);

        // ruddersDefault: the turning circles' rudder angles (deg; none: Sub_Dive); circles: what follows them (the
        // surfacing) recorded, not asserted
        private static IEnumerator Mission(TestContext ctx, string ruddersDefault, bool circles)
        {
            double surfacedShare = ctx.ParamDouble("surfaced", 0.88), reserve = ctx.ParamDouble("reserve", 0.005);
            double depthM = ctx.ParamDouble("depth", 10), gb = ctx.ParamDouble("gb", 0.1);
            double diveSec = ctx.ParamDouble("diveSec", 60), holdSec = ctx.ParamDouble("holdSec", 30), turnSec = ctx.ParamDouble("turnSec", 30);
            double speed = ctx.ParamDouble("speed", 4.07), hullLength = ctx.ParamDouble("length", 8.0), gentleRate = ctx.ParamDouble("gentleRate", 3);
            double[] rudders = ctx.ParamList("rudders", ruddersDefault).Where(x => x.Length > 0)
                .Select(x => double.Parse(x, CultureInfo.InvariantCulture)).ToArray();
            // SUBOFF's steady turning diameter (hull lengths) at a rudder angle (deg): two CFD studies' mean
            var suboffStd = new Dictionary<double, double> { { 10, (3.201 + 2.964) / 2 }, { 20, (2.317 + 2.386) / 2 }, { 35, (1.841 + 2.129) / 2 } };
            double fadeWas = FARAeroComponents.PartHydrodynamics.SubmergedFade;
            if (!double.IsNaN(ctx.ParamDouble("submergedFade", double.NaN)))
                FARAeroComponents.PartHydrodynamics.SubmergedFade = ctx.ParamDouble("submergedFade", fadeWas);
            ctx.Record("model", $"SubmergedFade {FARAeroComponents.PartHydrodynamics.SubmergedFade}");
            WaterParams.Apply(ctx);
            bool crashWas = CheatOptions.NoCrashDamage;
            var fixedWait = new WaitForFixedUpdate();
            try
            {
                yield return Wait.Seconds(5);
                CheatOptions.NoCrashDamage = true;
                Vessel v = FlightGlobals.ActiveVessel;
                v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                double dry = SetMass(v, 0);
                yield return Wait.Frames(2);
                string said = null;
                yield return FarWaterTools.PutOnWater(s => said = s);
                v = FlightGlobals.ActiveVessel;
                CelestialBody body = v.mainBody;
                double g = body.GeeASL * 9.80665;
                HullParts hull = HullParts.Find(v, p => p == v.rootPart)[0];
                ctx.Record("empty", $"{said}; {v.situation}, {dry:F3} t dry");

                // its buoyancy wholly submerged: held still and level at 15 m
                Vector3 up = v.upAxis;
                Vector3 h = Vector3.ProjectOnPlane(hull.Forward, up).normalized;
                double buoyancy = 0;
                Vector3 cb = Vector3.zero;     // its center of buoyancy off its middle, across the vertical, in the hull's frame
                // its down in its own frame, held level: the CG goes gb below the CB along it (not along the vertical when it
                // is set, which off a heeled hull puts the CG to one side and heels it further)
                Vector3 downLocal = hull.Ref.transform.InverseTransformDirection(-up);
                {
                    var ups = new List<double>();
                    var cbs = new List<Vector3>();
                    double t0 = Planetarium.GetUniversalTime();
                    while (Planetarium.GetUniversalTime() - t0 < 2.5)
                    {
                        Flight.PointAtAoA(0, h, up);
                        yield return fixedWait;
                        v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-15 - Depthless(hull, body)));
                        RigidVessel.Set(v, Vector3.zero, Vector3.zero);
                        if (Planetarium.GetUniversalTime() - t0 > 1.5)
                        {
                            Vector3 f = WaterForce(v), m = WaterMoment(v, hull.Position);
                            ups.Add(Vector3.Dot(f, up));
                            cbs.Add(hull.Ref.transform.InverseTransformDirection(Vector3.Cross(f, m) / f.sqrMagnitude));
                        }
                    }
                    // each part's share: its water force up and its moment about the hull's axis (roll, + right side down)
                    Vector3 right = Vector3.Cross(up, h);
                    ctx.Record("buoyancyParts", string.Join("; ", v.parts.Select(p =>
                    {
                        var hq = p.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                        Vector3 pf = Vector3.zero, pm = Vector3.zero;
                        if (hq != null && p.rb != null)
                        {
                            pf += hq.LastForce + hq.LastHydrostaticForce;
                            pm += hq.LastTorque + hq.LastHydrostaticTorque + Vector3.Cross(p.rb.worldCenterOfMass - hull.Position, hq.LastForce + hq.LastHydrostaticForce);
                        }
                        if (p.partBuoyancy != null)
                        {
                            pf += p.partBuoyancy.effectiveForce;
                            pm += Vector3.Cross(p.transform.TransformPoint(p.CenterOfBuoyancy) - hull.Position, p.partBuoyancy.effectiveForce);
                        }
                        return $"{p.partInfo.name}@{Vector3.Dot(p.transform.position - hull.Position, right):+0.00;-0.00} " +
                               $"{Vector3.Dot(pf, up):F3} kN roll {Vector3.Dot(pm, h):+0.000;-0.000} kNm" +
                               (hq == null ? "" : $" (owns {hq.OwnsBuoyancy}, mesh {hq.MeshVolume:F4} m3 {hq.MeshTriangles} tris, " +
                                                  $"hs {Vector3.Dot(hq.LastHydrostaticForce, up):F3} side {Vector3.Dot(hq.LastHydrostaticForce, right):F3}, " +
                                                  $"stock {(p.partBuoyancy != null ? Vector3.Dot(p.partBuoyancy.effectiveForce, up) : 0):F3})");
                    })));
                    buoyancy = ups.Average();
                    cb = cbs.Aggregate(Vector3.zero, (a, x) => a + x) / cbs.Count;
                    downLocal = hull.Ref.transform.InverseTransformDirection(-up);
                }
                double neutralT = buoyancy / g;
                Vector3 cbCraft = hull.Ref.transform.TransformDirection(cb);
                ctx.Record("buoyancy", $"{buoyancy:F2} kN wholly submerged: neutral at {neutralT:F3} t (dry {dry:F3} t); its center " +
                                       $"{Vector3.Dot(cbCraft, h):+0.000;-0.000} m ahead of its middle, {Vector3.Dot(cbCraft, Vector3.Cross(up, h)):+0.000;-0.000} m to the right");
                Assert.True(neutralT > dry * 1.2, $"it floats empty ({neutralT:F3} t of buoyancy, {dry:F3} t dry)");

                // surfaced
                SetMass(v, surfacedShare * neutralT);
                yield return Wait.Frames(2);
                yield return FarWaterTools.PutOnWater(s => said = s);
                v = FlightGlobals.ActiveVessel;
                hull = HullParts.Find(v, p => p == v.rootPart)[0];
                hull.SetCoM(v, cb + downLocal * (float)gb);
                // the rest of it sped up (the last trip on rails done)
                TestWarp.Hyper(ctx, true);
                yield return Wait.GameSeconds(10);
                double DepthNow() => -FlightGlobals.getAltitudeAtPos((Vector3d)hull.Position, body);
                double Vz() => Vector3.Dot((Vector3)v.srf_velocity, v.upAxis);
                double Pitch() => Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.up, v.upAxis), -1, 1)) * 180 / Math.PI;
                double Roll() => Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.right, v.upAxis), -1, 1)) * 180 / Math.PI;
                double sub0 = v.parts.Sum(q => q.submergedPortion * q.mass) / v.parts.Sum(q => q.mass);
                ctx.Record("surfaced", $"{v.situation} at {RigidVessel.Mass(v):F3} t: its middle {DepthNow():F2} m deep (0.60: its top at the water), " +
                                       $"{sub0:P0} under, pitch {Pitch():F1} deg, roll {Roll():F1} deg, {Vz():+0.00;-0.00} m/s up");
                Assert.True(v.situation == Vessel.Situations.SPLASHED && DepthNow() < 0.55, $"surfaced (its middle {DepthNow():F2} m deep)");

                // flooded: awash
                SetMass(v, (1 - reserve) * neutralT);
                yield return Wait.Frames(2);
                hull.SetCoM(v, cb + downLocal * (float)gb);
                yield return Wait.GameSeconds(20);
                ctx.Record("awash", $"at {RigidVessel.Mass(v):F3} t: its middle {DepthNow():F2} m deep, pitch {Pitch():F1} deg, roll {Roll():F1} deg, " +
                                    $"{Vz():+0.00;-0.00} m/s up");
                Assert.True(DepthNow() < 1.5 && Math.Abs(Vz()) < 0.2, $"awash, not sinking (its middle {DepthNow():F2} m deep, {Vz():F2} m/s up)");

                // startDepth (m, default 0: from awash): let go there instead, level and still, after a moment held
                double startDepth = ctx.ParamDouble("startDepth", 0);
                if (startDepth > 0)
                {
                    up = v.upAxis;
                    h = Vector3.ProjectOnPlane(hull.Forward, up).normalized;
                    double t0 = Planetarium.GetUniversalTime();
                    while (Planetarium.GetUniversalTime() - t0 < 1.5)
                    {
                        Flight.PointAtAoA(0, h, up);
                        yield return fixedWait;
                        v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-startDepth - Depthless(hull, body)));
                        RigidVessel.Set(v, Vector3.zero, Vector3.zero);
                    }
                    ctx.Record("start", $"let go {DepthNow():F2} m deep");
                }

                // under way
                Part motorPart = v.parts.First(q => q.partInfo.name == "FARTest.BoatMotor");
                ModuleEngines motor = motorPart.Modules.GetModule<ModuleEngines>();
                motor.Activate();
                Ctrl.Attach();
                var csv = new StringBuilder("t,phase,target_m,depth_m,vz_ms,V_ms,thrust_kN,pitch_deg,pitch_target_deg,roll_deg,heading_deg,ctrl_pitch,ctrl_roll,ctrl_yaw,water_up_kN,v_fwd_ms,defl_top,defl_bottom,defl_right,defl_left\n");
                // the control surfaces: the rudders (top, bottom: those that yaw) and the planes (right, left: those that
                // pitch), by which way they stand off the hull's axis (the sail, held, is neither)
                ferram4.FARControllableSurface Surface(Func<ferram4.FARControllableSurface, bool> what, Func<Vector3, bool> where) => v.parts
                    .Select(q => q.Modules.GetModule<ferram4.FARControllableSurface>()).Where(c => c != null && what(c))
                    .First(c => where(Vector3.ProjectOnPlane(c.part.transform.position - hull.Position, hull.Forward).normalized));
                Func<ferram4.FARControllableSurface, bool> rudderOf = c => c.yawaxis != 0, planeOf = c => c.pitchaxis != 0;
                var surfaces = new[]
                {
                    Surface(rudderOf, o => Vector3.Dot(o, v.upAxis) > 0.7f), Surface(rudderOf, o => Vector3.Dot(o, v.upAxis) < -0.7f),
                    Surface(planeOf, o => Vector3.Dot(o, v.ReferenceTransform.right) > 0.7f), Surface(planeOf, o => Vector3.Dot(o, v.ReferenceTransform.right) < -0.7f)
                };
                double maxDeflect = surfaces[0].maxdeflect;
                Vector3 h0 = Vector3.ProjectOnPlane(v.ReferenceTransform.up, v.upAxis).normalized;
                double Heading() => Vector3.SignedAngle(h0, Vector3.ProjectOnPlane(v.ReferenceTransform.up, v.upAxis), v.upAxis);
                double tStart = Planetarium.GetUniversalTime(), pitchWas = Pitch(), rollWas = Roll(), depthI = 0, pitchI = 0, throttleI = 0;
                double headingWas = Heading(), yawI = 0;
                var stats = new Dictionary<string, List<(double t, double depth, double V, double thrust, double pitch, double roll, double heading, double vz, double waterUp)>>();
                double surfacedAt = double.NaN;
                // the phases: under way (throttle 1: held at the speed) or stopped (0); the rudders a share of full, or
                // (NaN) the heading autopilot's at a turn rate (deg/s; 0: its heading held); the gentle turn, then each
                // turning circle and a straight run after it
                var phases = new List<(string phase, double target, float yaw, double rate, float throttle, double sec)>
                {
                    ("dive", depthM, float.NaN, 0, 1f, diveSec), ("hold", depthM, float.NaN, 0, 1f, holdSec),
                    ("gentle", depthM, float.NaN, gentleRate, 1f, 40.0), ("straight", depthM, float.NaN, 0, 1f, 15.0)
                };
                foreach (double r in rudders)
                {
                    phases.Add(("turn" + r.ToString(CultureInfo.InvariantCulture), depthM, (float)(r / maxDeflect), 0, 1f, turnSec));
                    phases.Add(("level" + r.ToString(CultureInfo.InvariantCulture), depthM, float.NaN, 0, 1f, 20.0));
                }
                phases.Add(("surface", -3.0, float.NaN, 0, 1f, 40.0));
                phases.Add(("float", -3.0, 0f, 0, 0f, 30.0));
                foreach (var (phase, target, yaw, rate, throttle, sec) in phases)
                {
                    var list = stats[phase] = new List<(double, double, double, double, double, double, double, double, double)>();
                    double ts = Planetarium.GetUniversalTime(), headingTarget = Heading();
                    Ctrl.SetThrottle(throttle);
                    while (Planetarium.GetUniversalTime() - ts < sec)
                    {
                        yield return fixedWait;
                        double dt = Math.Max(1e-3, TimeWarp.fixedDeltaTime);
                        // the speed held (a PI on the throttle), or full throttle
                        if (throttle > 0 && speed > 0)
                        {
                            double err = speed - v.srfSpeed;
                            throttleI = Math.Max(0, Math.Min(1, throttleI + 0.05 * err * dt));
                            Ctrl.SetThrottle((float)Math.Max(0, Math.Min(1, throttleI + 0.2 * err)));
                        }
                        double d = DepthNow(), vz = Vz(), pitch = Pitch(), roll = Roll();
                        double q = (pitch - pitchWas) / dt, p = (roll - rollWas) / dt;
                        pitchWas = pitch;
                        rollWas = roll;
                        // the depth autopilot: the pitch for the depth (nose down to go deeper), the planes for the pitch;
                        // the planes against each other, and the probe core's wheel, against roll; stopped, nothing
                        // (each with an integral for its steady error: its reserve buoyancy, its trim)
                        if (throttle > 0)
                            depthI = Math.Max(-30, Math.Min(30, depthI + (target - d) * dt));
                        double pitchTarget = -Mathf.Clamp((float)(2 * (target - d) + 6 * vz + 0.15 * depthI), -12, 12);
                        if (throttle > 0)
                            pitchI = Math.Max(-0.5, Math.Min(0.5, pitchI + 0.02 * (pitchTarget - pitch) * dt));
                        float cp = throttle > 0 ? Mathf.Clamp((float)(0.08 * (pitchTarget - pitch) - 0.08 * q + pitchI), -1, 1) : 0;
                        float cr = Mathf.Clamp((float)(0.05 * roll + 0.05 * p), -1, 1);
                        // the heading autopilot: the rudders for the turn rate (the heading's error's, held), an
                        // integral for a steady turn's
                        double heading = Heading(), r = Mathf.DeltaAngle((float)headingWas, (float)heading) / dt;
                        headingWas = heading;
                        float cy = yaw;
                        if (float.IsNaN(yaw))
                        {
                            double rateTarget = rate != 0 ? rate : Mathf.Clamp(0.3f * Mathf.DeltaAngle((float)heading, (float)headingTarget), -3, 3);
                            if (throttle > 0)
                                yawI = Math.Max(-0.2, Math.Min(0.2, yawI + 0.005 * (rateTarget - r) * dt));
                            cy = throttle > 0 ? Mathf.Clamp((float)(0.03 * (rateTarget - r) + yawI), -0.5f, 0.5f) : 0;
                        }
                        Ctrl.Set(pitch: cp, yaw: cy, roll: cr);
                        double V = v.srfSpeed, thrust = motor.finalThrust;
                        double waterUp = Vector3.Dot(WaterForce(v), v.upAxis);
                        double t = Planetarium.GetUniversalTime() - tStart;
                        list.Add((t, d, V, thrust, pitch, roll, Heading(), vz, waterUp));
                        csv.AppendLine(string.Join(",", F(t), phase, F(target), F(d), F(vz), F(V), F(thrust), F(pitch), F(pitchTarget), F(roll),
                                                   F(Heading()), F(cp), F(cr), F(cy), F(waterUp),
                                                   F(Vector3.Dot((Vector3)v.srf_velocity, v.ReferenceTransform.up)),
                                                   string.Join(",", surfaces.Select(c => F(c.ControlDeflection)))));
                        if (phase == "surface" && double.IsNaN(surfacedAt) && d < 0.6)
                            surfacedAt = Planetarium.GetUniversalTime() - ts;
                    }
                }
                Ctrl.Neutral();
                Ctrl.SetThrottle(0);
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "sub.csv"), csv.ToString());

                var dive = stats["dive"];
                double reached = dive.FirstOrDefault(x => Math.Abs(x.depth - depthM) < 1).t - dive[0].t;
                var hold = stats["hold"];
                double holdMean = hold.Average(x => x.depth), holdStd = Math.Sqrt(hold.Average(x => (x.depth - holdMean) * (x.depth - holdMean)));
                double pitchStd = Math.Sqrt(hold.Average(x => x.pitch * x.pitch) - Math.Pow(hold.Average(x => x.pitch), 2));
                double vA = hold.Take(hold.Count / 4).Average(x => x.V), vB = hold.Skip(3 * hold.Count / 4).Average(x => x.V);
                double thrustHold = hold.Average(x => x.thrust);
                ctx.Record("dive", $"to {depthM:F0} m (within 1 m) in {reached:F1} s; deepest {dive.Max(x => x.depth):F2} m, pitch down to {dive.Min(x => x.pitch):F1} deg");
                ctx.Record("hold", $"depth {holdMean:F2} m (sd {holdStd:F3} m), pitch {hold.Average(x => x.pitch):F2} deg (sd {pitchStd:F2}), " +
                                   $"{vA:F2} then {vB:F2} m/s, thrust {thrustHold:F2} kN (its curve's {10 * Math.Max(0, 1 - hold.Average(x => x.V) / 25):F2}), " +
                                   $"roll {hold.Average(x => x.roll):F1} deg");
                // a phase's turn rate over its second half (the heading's change summed sample to sample)
                double RateOf(List<(double t, double depth, double V, double thrust, double pitch, double roll, double heading, double vz, double waterUp)> ph)
                {
                    var half = ph.Skip(ph.Count / 2).ToList();
                    double turned = 0;
                    for (int i = 1; i < half.Count; i++)
                        turned += Mathf.DeltaAngle((float)half[i - 1].heading, (float)half[i].heading);
                    return turned / Math.Max(1e-3, half.Last().t - half[0].t);
                }
                var gentle = stats["gentle"];
                double gentleGot = RateOf(gentle), gentleDev = gentle.Max(x => Math.Abs(x.depth - depthM)), gentleHeel = gentle.Max(x => Math.Abs(x.roll));
                ctx.Record("gentle", $"turning at {gentleRate:F1} deg/s asked: {gentleGot:+0.00;-0.00} deg/s at {gentle.Average(x => x.V):F2} m/s, " +
                                     $"depth off up to {gentleDev:F2} m, heel up to {gentleHeel:F1} deg");
                // each turning circle: its steady rate and diameter over its second half, its depth kept and heel (recorded)
                var turnLines = new StringBuilder();
                foreach (double r in rudders)
                {
                    var turn = stats["turn" + r.ToString(CultureInfo.InvariantCulture)];
                    var tail = turn.Skip(turn.Count / 2).ToList();
                    double rate = RateOf(turn);
                    double vTurn = tail.Average(x => x.V), std = 2 * vTurn / Math.Max(1e-6, Math.Abs(rate) * Math.PI / 180) / hullLength;
                    double dev = turn.Max(x => Math.Abs(x.depth - depthM)), heel = turn.Max(x => Math.Abs(x.roll));
                    bool known = suboffStd.TryGetValue(r, out double refStd);
                    turnLines.Append($"{r:F0} deg: {rate:+0.0;-0.0} deg/s at {vTurn:F2} m/s (from {turn[0].V:F2}), steady diameter {std:F2} lengths" +
                                     (known ? $" (SUBOFF {refStd:F2}, {std / refStd:F2}x)" : "") +
                                     $", depth off up to {dev:F2} m, heel up to {heel:F1} deg; ");
                }
                ctx.Record("turns", turnLines.ToString());
                var surf = stats["surface"].Concat(stats["float"]).ToList();
                var fl = stats["float"];
                var flTail = fl.Skip(fl.Count / 2).ToList();
                ctx.Record("surface", $"up through the surface after {surfacedAt:F1} s at up to {surf.Max(x => x.vz):F2} m/s; its middle {-surf.Min(x => x.depth):F2} m above " +
                                      $"the water at most; the water's lift up to {surf.Max(x => x.waterUp) / (RigidVessel.Mass(v) * g):F2} of its weight; " +
                                      $"stopped: its middle {flTail.Average(x => x.depth):F2} m deep, {flTail.Average(x => x.vz):+0.00;-0.00} m/s up, " +
                                      $"pitch {flTail.Average(x => x.pitch):F1} deg, roll {flTail.Average(x => x.roll):F1} deg, {v.situation}");

                Assert.True(Math.Abs(holdMean - depthM) < 1 && holdStd < 0.5, $"holds its depth ({holdMean:F2} m, sd {holdStd:F3})");
                Assert.True(pitchStd < 2, $"no porpoising (pitch sd {pitchStd:F2} deg)");
                Assert.True(Math.Abs(vB - vA) < 0.3 && vB > 2, $"a steady speed ({vA:F2}, {vB:F2} m/s)");
                Assert.True(Math.Abs(gentleGot - gentleRate) < 1 && gentleDev < 2.5 && gentleHeel < 30,
                            $"turns gently at depth ({gentleGot:F2} deg/s, depth off {gentleDev:F2} m, heel {gentleHeel:F1} deg)");
                if (!circles)
                {
                    Assert.True(!double.IsNaN(surfacedAt), "surfaces");
                    Assert.True(flTail.Average(x => x.depth) < 1.0 && Math.Abs(flTail.Average(x => x.vz)) < 0.2 && v.situation == Vessel.Situations.SPLASHED,
                                "floats at the surface, stopped");
                }
            }
            finally
            {
                TestWarp.Hyper(ctx, false);
                Ctrl.Neutral();
                CheatOptions.NoCrashDamage = crashWas;
                WaterParams.Restore();
                FARAeroComponents.PartHydrodynamics.SubmergedFade = fadeWas;
            }
        }

        // the hull's fuel (LiquidFuel first, then Oxidizer) for a total mass (t); the dry mass
        private static double SetMass(Vessel v, double massT)
        {
            foreach (Part p in v.parts)
                foreach (PartResource r in p.Resources)
                    if (r.info.density > 0)
                        r.amount = 0;
            double dry = v.parts.Sum(q => q.mass + q.GetResourceMass());
            double need = massT - dry;
            foreach (string name in new[] { "LiquidFuel", "Oxidizer" })
            {
                PartResource r = v.rootPart.Resources.Get(name);
                if (r == null || need <= 0)
                    continue;
                r.amount = Math.Min(r.maxAmount, need / r.info.density);
                need -= r.amount * r.info.density;
            }
            Assert.True(massT <= 0 || need < 1e-3, $"room for {massT:F3} t ({need:F3} t short)");
            foreach (Part p in v.parts)
                p.UpdateMass();
            return dry;
        }

        // the water's moment on the whole vessel about a point (a part's water force acting at its CoM, stock buoyancy at
        // its center of buoyancy)
        private static Vector3 WaterMoment(Vessel v, Vector3 about)
        {
            Vector3 m = Vector3.zero;
            foreach (Part p in v.parts)
            {
                var hq = p.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                if (hq != null && p.rb != null)
                    m += hq.LastTorque + hq.LastHydrostaticTorque + Vector3.Cross(p.rb.worldCenterOfMass - about, hq.LastForce + hq.LastHydrostaticForce);
                if (p.partBuoyancy != null)
                    m += Vector3.Cross(p.transform.TransformPoint(p.CenterOfBuoyancy) - about, p.partBuoyancy.effectiveForce);
            }
            return m;
        }

        // the hull's middle's altitude (its depth, negated)
        private static double Depthless(HullParts hull, CelestialBody body) => FlightGlobals.getAltitudeAtPos((Vector3d)hull.Position, body);

        // the water's force on the whole vessel: FAR's on its parts, stock buoyancy on those FAR leaves to it
        private static Vector3 WaterForce(Vessel v)
        {
            Vector3 f = Vector3.zero;
            foreach (Part p in v.parts)
            {
                var hq = p.Modules.GetModule<FARAeroComponents.FARAeroPartModule>()?.Hydrodynamics;
                if (hq != null)
                    f += hq.LastForce + hq.LastHydrostaticForce;
                if (p.partBuoyancy != null)
                    f += p.partBuoyancy.effectiveForce;
            }
            return f;
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);
    }
}
