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
        private const string Mods = TestWarp.WaterMods;   // the water tests' instance

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
        /// 30 deg). Then up to the surface and stopped: it comes up through the surface and floats there. Flown as built:
        /// its trim (its CG over its center of buoyancy, from its ballast in the nose) asserted within 5 cm, as a player
        /// gets it; trim 1 moves its CG there instead, gb (m, default 0.1) below its hull's axis (a keel's: a
        /// submarine's righting moment). Its turning circles (rudders, none here): Sub_TurningCircles. Writes sub.csv.
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

                // its trim as built, flooded (its parts' masses where they are): its CG ahead of its center of buoyancy and
                // below its hull's axis (the center of buoyancy's height is the axis's: measured level, its line of action
                // only), each part's mass and place
                bool trim = ctx.ParamDouble("trim", 0) != 0;
                SetMass(v, (1 - reserve) * neutralT);
                yield return Wait.Frames(2);
                yield return fixedWait;
                {
                    Vector3 fwd = hull.Forward, upHull = -hull.Ref.transform.TransformDirection(downLocal);
                    Vector3 cg = RigidVessel.CoM(v) - hull.Position;
                    double ahead = Vector3.Dot(cg - cbCraft, fwd), below = -Vector3.Dot(cg, upHull);
                    ctx.Record("trim as built", $"at {RigidVessel.Mass(v):F3} t its CG {ahead:+0.000;-0.000} m ahead of its center of " +
                                                $"buoyancy, {below:+0.000;-0.000} m below its axis ({buoyancy * ahead:+0.00;-0.00} kNm nose down)");
                    ctx.Record("part masses", string.Join("; ", v.parts.Where(q => q.rb != null).Select(q =>
                        $"{q.partInfo.name} {q.rb.mass:F3} t at {Vector3.Dot(q.rb.worldCenterOfMass - hull.Position, fwd):+0.000;-0.000} m " +
                        $"{Vector3.Dot(q.rb.worldCenterOfMass - hull.Position, upHull):+0.000;-0.000} m up")));
                    if (!trim)
                        Assert.True(Math.Abs(ahead) < 0.05 && below > 0, $"trimmed as built: its CG over its center of buoyancy (" +
                                                                         $"{ahead:+0.000;-0.000} m ahead) and under its axis ({below:+0.000;-0.000} m)");
                }

                // surfaced
                SetMass(v, surfacedShare * neutralT);
                yield return Wait.Frames(2);
                yield return FarWaterTools.PutOnWater(s => said = s);
                v = FlightGlobals.ActiveVessel;
                hull = HullParts.Find(v, p => p == v.rootPart)[0];
                if (trim)
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
                if (trim)
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
                };
                // freeSec (s, default 0: none): hands off after the hold (its stability, a player's let go), then the
                // stick at jab (default +1: full nose up, a key held) for jabSec (default 1), then hands off again;
                // recorded, not asserted, and the autopilots take it back after
                double freeSec = ctx.ParamDouble("freeSec", 0), jabSec = ctx.ParamDouble("jabSec", 1);
                float jab = (float)ctx.ParamDouble("jab", 1);
                // sas 1: KSP's SAS on (stability assist) through them, as a player flies it
                bool sas = ctx.ParamDouble("sas", 0) != 0;
                if (freeSec > 0)
                {
                    phases.Add(("free", depthM, 0f, 0, 1f, freeSec));
                    phases.Add(("jab", depthM, 0f, 0, 1f, jabSec));
                    phases.Add(("after", depthM, 0f, 0, 1f, freeSec));
                    phases.Add(("recover", depthM, float.NaN, 0, 1f, 30.0));
                }
                phases.Add(("gentle", depthM, float.NaN, gentleRate, 1f, 40.0));
                phases.Add(("straight", depthM, float.NaN, 0, 1f, 15.0));
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
                    bool sasOn = sas && (phase == "free" || phase == "jab" || phase == "after");
                    v.ActionGroups.SetGroup(KSPActionGroup.SAS, sasOn);
                    if (sasOn)
                        v.Autopilot.SetMode(VesselAutopilot.AutopilotMode.StabilityAssist);
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
                        if (phase == "free" || phase == "after")
                            cp = cr = cy = 0;
                        else if (phase == "jab")
                        {
                            cp = jab;
                            cr = cy = 0;
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
                foreach (string ph in new[] { "free", "jab", "after", "recover" }.Where(stats.ContainsKey))
                {
                    var s = stats[ph];
                    ctx.Record(ph, $"pitch {s[0].pitch:+0.0;-0.0} to {s[s.Count - 1].pitch:+0.0;-0.0} deg (extremes {s.Min(x => x.pitch):+0.0;-0.0}, " +
                                   $"{s.Max(x => x.pitch):+0.0;-0.0}), depth {s[0].depth:F2} to {s[s.Count - 1].depth:F2} m, " +
                                   $"{s[0].V:F2} to {s[s.Count - 1].V:F2} m/s, roll up to {s.Max(x => Math.Abs(x.roll)):F1} deg");
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

        /// <summary>
        /// A video of the test submarine (frames/NNNNN.jpg, one per 1/30 s, and subdemo.csv) on Scatterer's calm sea:
        /// trimmed awash (its reserve buoyancy, its deck at the water) it cruises at speed for awashSec (default 25), dives
        /// to depth (default 12 m) on its planes and holds it (diveSec, default 40), then blows its ballast (its mass to
        /// surfaced's share of neutral) and pulls up at breachPitch (default 30 deg) at full speed, breaching and then
        /// level on the surface (breachSec, default 25), and floats stopped (floatSec, default 10). The camera abeam
        /// (camAngle deg aft of it, default 20; camDistance, default 7.8 m: it half the frame's width; camPitch, default
        /// 12 deg: above the water awash, under it dived), the UI hidden, an overlay with its depth, speed, pitch and
        /// what it is doing. Parameters: hydro (off: FAR's water model off, the water as thick air), depth, speed
        /// (default 8 m/s), reserve, surfaced, hyperWarp (given: hyper-warped, a time-lapse), record (false: no frames).
        /// It ends 5 s after it tumbles (pitch or roll past 75 deg), recorded.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.DemoMods, StartsAt = StartState.Flight, Craft = "Test Submarine", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Demo_Submarine(TestContext ctx) => SubDemo(ctx);

        /// <summary>
        /// Demo_Submarine with the submarine a player would build with an electric propeller (crafts/test_sub.py prop:
        /// Firespitter's FS1PRE, a pusher on its tail, batteries and its probe core in its nose as trim weight, its
        /// ballast solved again): the propeller as it comes, its thrust Firespitter's (20 kN, the air's density's, not
        /// the water's), its blades in the water to FAR's water model as any part's.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.DemoMods, StartsAt = StartState.Flight, Craft = "Test Submarine Prop", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Demo_SubmarineProp(TestContext ctx) => SubDemo(ctx);

        /// <summary>
        /// Demo_Submarine with a submarine at a real one's size (crafts/test_sub_big.py: 40 m, a coastal boat's motor
        /// (mods/FARTestSubMotor) sunk into its stern, trim tanks fore and aft that the demo pumps between to trim it
        /// level). Its parameters scaled: give camDistance ~39, speed ~5, depth ~25, awashDepth ~2.2, surfaceDepth ~1.5,
        /// hover ~0.01, blowTrim ~1, blowDelay ~10, breachDepth ~3; and waterDamping none (stock's water drag below
        /// 0.4 m/s, 2/s whatever its mass, holds a real boat's 0.2 m/s2 still).
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.DemoMods, StartsAt = StartState.Flight, Craft = "Test Submarine Big", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 3600)]
        public static IEnumerator Demo_SubmarineBig(TestContext ctx) => SubDemo(ctx);

        /// <summary>
        /// The propeller's geometry on the prop submarine, for fairing its stern (crafts/test_sub.py prop): each of its
        /// part's meshes (renderers and colliders), its bounds in the part's frame, and its transforms by name.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.DemoMods, StartsAt = StartState.Flight, Craft = "Test Submarine Prop", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 600)]
        public static IEnumerator Measure_PropGeometry(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            Part prop = FlightGlobals.ActiveVessel.parts.First(q => q.partInfo.name == "FSPROpellerElectric");
            Transform pt = prop.partTransform;
            int n = 0;
            foreach (MeshFilter mf in prop.GetComponentsInChildren<MeshFilter>(true))
            {
                if (mf.sharedMesh == null)
                    continue;
                var bounds = new Bounds(pt.InverseTransformPoint(mf.transform.TransformPoint(mf.sharedMesh.vertices[0])), Vector3.zero);
                foreach (Vector3 v in mf.sharedMesh.vertices)
                    bounds.Encapsulate(pt.InverseTransformPoint(mf.transform.TransformPoint(v)));
                bool collider = mf.GetComponent<Collider>() != null;
                ctx.Record($"mesh {++n}", $"{mf.name} ({mf.sharedMesh.vertexCount} verts{(collider ? ", collider" : "")}, " +
                                          $"{(mf.GetComponent<Renderer>()?.enabled ?? false ? "shown" : "hidden")}): {bounds.min:F3} to {bounds.max:F3}");
            }
            ctx.Record("transforms", string.Join(", ", prop.GetComponentsInChildren<Transform>(true).Select(x => $"{x.name}@{pt.InverseTransformPoint(x.position):F2}")));
            ctx.Record("part", $"mass {prop.mass:F3} t, attach {prop.attPos0}, its origin in the hull's frame {FlightGlobals.ActiveVessel.rootPart.partTransform.InverseTransformPoint(pt.position):F3}");
        }

        private static IEnumerator SubDemo(TestContext ctx)
        {
            double reserve = ctx.ParamDouble("reserve", 0.005), surfacedShare = ctx.ParamDouble("surfaced", 0.88);
            // awash: its tanks part full (awashReserve of buoyancy over its weight), its middle held awashDepth deep (its
            // deck at the water: its top 0.6 m above its middle)
            double awashReserve = ctx.ParamDouble("awashReserve", 0.04), awashTarget = ctx.ParamDouble("awashDepth", 0.45);
            double depthM = ctx.ParamDouble("depth", 12), speed = ctx.ParamDouble("speed", 8), breachPitch = ctx.ParamDouble("breachPitch", 30);
            double awashSec = ctx.ParamDouble("awashSec", 25), diveSec = ctx.ParamDouble("diveSec", 40), breachSec = ctx.ParamDouble("breachSec", 25);
            double floatSec = ctx.ParamDouble("floatSec", 10), hover = ctx.ParamDouble("hover", 0), hoverNow = 0, hoverSet = 0;
            // blowTrim: see the breach; breachDepth (m, default 0): its middle above that deep, it has broken the surface
            double blowTrim = ctx.ParamDouble("blowTrim", 0), blowMoved = 0, breachDepth = ctx.ParamDouble("breachDepth", 0);
            double blowDelay = ctx.ParamDouble("blowDelay", 0);
            // the pitch autopilot's damping (per deg/s, default 0.08) and the depth's (deg per m/s, default 6): a big boat's slower
            double pitchD = ctx.ParamDouble("pitchD", 0.08), vzDamp = ctx.ParamDouble("vzDamp", 6);
            bool blown = true;
            bool record = ctx.Param("record") != "false";
            bool hydroWas = FARDebugValues.hydrodynamics, crashWas = CheatOptions.NoCrashDamage;
            WaterParams.Apply(ctx);
            GameObject recorderGo = null;
            var fixedWait = new WaitForFixedUpdate();
            try
            {
                FARDebugValues.hydrodynamics = ctx.Param("hydro") != "off";
                ctx.Record("hydrodynamics", FARDebugValues.hydrodynamics.ToString());
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

                // neutral: its buoyancy wholly submerged, held still and level at 15 m (whichever water model)
                Vector3 up = v.upAxis;
                Vector3 h = Vector3.ProjectOnPlane(hull.Forward, up).normalized;
                var ups = new List<double>();
                double t0 = Planetarium.GetUniversalTime();
                while (Planetarium.GetUniversalTime() - t0 < 2.5)
                {
                    Flight.PointAtAoA(0, h, up);
                    yield return fixedWait;
                    v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-15 - Depthless(hull, body)));
                    RigidVessel.Set(v, Vector3.zero, Vector3.zero);
                    if (Planetarium.GetUniversalTime() - t0 > 1.5)
                        ups.Add(Vector3.Dot(WaterForce(v), up));
                }
                double neutralT = ups.Average() / g;
                // (the water model off: stock's buoyancy can be more than its tanks can flood, recorded, not asserted)
                bool toldShort = false;
                Action<double> Short = FARDebugValues.hydrodynamics ? null : (Action<double>)(s =>
                {
                    if (!toldShort)
                        ctx.Record("too buoyant", $"its tanks full, {s:F1} t short of the mass asked for: stock's buoyancy {neutralT:F0} t");
                    toldShort = true;
                });
                ctx.Record("buoyancy", $"{ups.Average():F2} kN wholly submerged: neutral at {neutralT:F3} t (dry {dry:F3} t)");

                // trim tanks (two LiquidFuel tanks besides the hull, fore and aft): at its dived mass, held at 15 m, fuel
                // pumped between them until its CG is over its center of buoyancy, as a real boat's trim pumps do
                var trimTanks = v.parts.Where(q => q != v.rootPart && q.Resources.Get("LiquidFuel") != null)
                                 .OrderByDescending(q => Vector3.Dot(q.transform.position - hull.Position, hull.Forward)).ToList();
                if (trimTanks.Count == 2)
                {
                    SetMass(v, (1 - reserve) * neutralT, Short);
                    PartResource fore = trimTanks[0].Resources.Get("LiquidFuel"), aft = trimTanks[1].Resources.Get("LiquidFuel");
                    for (int pass = 0; pass < 4; pass++)
                    {
                        var aheads = new List<double>();
                        t0 = Planetarium.GetUniversalTime();
                        while (Planetarium.GetUniversalTime() - t0 < 2)
                        {
                            Flight.PointAtAoA(0, h, up);
                            yield return fixedWait;
                            v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-15 - Depthless(hull, body)));
                            RigidVessel.Set(v, Vector3.zero, Vector3.zero);
                            if (Planetarium.GetUniversalTime() - t0 > 1.2)
                            {
                                Vector3 hNow = Vector3.ProjectOnPlane(hull.Forward, up).normalized;
                                aheads.Add(Vector3.Dot(WaterMoment(v, v.CoM), Vector3.Cross(hNow, up)) / Vector3.Dot(WaterForce(v), up));
                            }
                        }
                        double ahead = aheads.Average();
                        double apart = Vector3.Dot(trimTanks[0].transform.position - trimTanks[1].transform.position, hull.Forward);
                        // (its center of buoyancy ahead of its CG: fuel forward)
                        double moved = ahead * RigidVessel.Mass(v) / apart / fore.info.density;
                        moved = Math.Min(moved, Math.Min(aft.amount, fore.maxAmount - fore.amount));
                        moved = Math.Max(moved, -Math.Min(fore.amount, aft.maxAmount - aft.amount));
                        ctx.Record($"trim {pass + 1}", $"its center of buoyancy {ahead:+0.000;-0.000} m ahead of its CG: " +
                                                        $"{moved * fore.info.density:+0.00;-0.00} t pumped forward");
                        if (Math.Abs(ahead) < 0.005)
                            break;
                        fore.amount += moved;
                        aft.amount -= moved;
                        trimTanks[0].UpdateMass();
                        trimTanks[1].UpdateMass();
                        yield return Wait.Frames(2);
                    }
                    ctx.Record("trim tanks", $"fore {fore.amount * fore.info.density:F2} t, aft {aft.amount * aft.info.density:F2} t of {fore.maxAmount * fore.info.density:F2} t");
                }

                // awash: its tanks part full, brought up from the hold level and still to its deck at the water (not
                // put on the water again: a second drop)
                SetMass(v, (1 - awashReserve) * neutralT, Short);
                yield return Wait.Frames(2);
                t0 = Planetarium.GetUniversalTime();
                while (Planetarium.GetUniversalTime() - t0 < 0.5)
                {
                    Flight.PointAtAoA(0, h, up);
                    yield return fixedWait;
                    v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-awashTarget - Depthless(hull, body)));
                    RigidVessel.Set(v, Vector3.zero, Vector3.zero);
                }
                yield return Wait.GameSeconds(8);
                double DepthNow() => -FlightGlobals.getAltitudeAtPos((Vector3d)hull.Position, body);
                double Vz() => Vector3.Dot((Vector3)v.srf_velocity, v.upAxis);
                double Pitch() => Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.up, v.upAxis), -1, 1)) * 180 / Math.PI;
                double Roll() => Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.right, v.upAxis), -1, 1)) * 180 / Math.PI;
                double awashDepth = DepthNow();
                ctx.Record("awash", $"{v.situation}, its middle {awashDepth:F2} m deep, pitch {Pitch():F1} deg");

                // the camera abeam, its heading found by trying two (as the floatplane's demo)
                up = v.upAxis;
                Vector3 h0 = Vector3.ProjectOnPlane(v.ReferenceTransform.up, up).normalized;
                FlightCamera cam = FlightCamera.fetch;
                float camHdg = 0, camPitch = (float)(ctx.ParamDouble("camPitch", 12) * Mathf.Deg2Rad);
                PlaningVideoTests.Recorder rec = null;
                if (record)
                {
                    cam.minPitch = -0.5f;
                    cam.SetDistanceImmediate((float)ctx.ParamDouble("camDistance", 7.8));
                    Vector3 side = Vector3.Cross(up, h0).normalized;
                    Vector3 look = Quaternion.AngleAxis((float)ctx.ParamDouble("camAngle", 20), up) * side;
                    float a0 = 0, a1 = 0;
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
                    camHdg = Math.Abs(slope) > 0.2f ? -a0 / slope : 0;
                    GameEvents.onHideUI.Fire();
                    string frames = Path.Combine(ctx.ResultsDir, "frames");
                    Directory.CreateDirectory(frames);
                    recorderGo = new GameObject("SubDemoRecorder");
                    rec = recorderGo.AddComponent<PlaningVideoTests.Recorder>();
                    rec.Dir = frames;
                    Time.captureFramerate = 30;
                }
                if (!record || ctx.Param("hyperWarp") != null)
                    TestWarp.Hyper(ctx, true);

                // its engine: the test motor (a stock engine module), or Firespitter's propeller (its own module), started
                // as a player would (its Activate event)
                ModuleEngines motor = v.parts.Select(q => q.Modules.GetModule<ModuleEngines>()).FirstOrDefault(m => m != null);
                PartModule fsEngine = v.parts.SelectMany(q => q.Modules.Cast<PartModule>()).FirstOrDefault(m => m.moduleName == "FSengine");
                if (motor != null)
                    motor.Activate();
                else if (fsEngine != null)
                    fsEngine.Events["Activate"].Invoke();
                Assert.True(motor != null || fsEngine != null, "an engine: the test motor or Firespitter's propeller");
                ctx.Record("engine", motor != null ? motor.part.partInfo.title : $"{fsEngine.part.partInfo.title} ({fsEngine.part.mass:F3} t)");
                Ctrl.Attach();
                var csv = new StringBuilder("t,phase,target_m,depth_m,vz_ms,V_ms,pitch_deg,pitch_target_deg,roll_deg,ctrl_pitch,throttle\n");
                double Heading() => Vector3.SignedAngle(h0, Vector3.ProjectOnPlane(v.ReferenceTransform.up, v.upAxis), v.upAxis);
                double tStart = Planetarium.GetUniversalTime(), pitchWas = Pitch(), rollWas = Roll(), headingWas = Heading();
                double depthI = 0, pitchI = 0, throttleI = 0, tumbledAt = double.NaN, breachedAt = double.NaN, deepest = 0, highest = double.NaN;
                double surfaceDepth = ctx.ParamDouble("surfaceDepth", 0.3);
                bool wasSurfaced = false;
                var phases = new List<(string phase, double target, float throttle, double sec)>
                {
                    ("awash", awashTarget, 1f, awashSec), ("dive", depthM, 1f, diveSec), ("breach", double.NaN, 1f, breachSec), ("float", double.NaN, 0f, floatSec),
                };
                bool done = false;
                foreach (var (phase, target, throttle, sec) in phases)
                {
                    if (done)
                        break;
                    double ts = Planetarium.GetUniversalTime(), headingTarget = Heading();
                    if (phase == "dive")
                    {
                        // the tanks flooded: its reserve
                        SetMass(v, (1 - reserve) * neutralT, Short);
                        ctx.Record("flood", $"at {DepthNow():F2} m deep: {RigidVessel.Mass(v):F3} t");
                    }
                    if (phase == "breach")
                    {
                        // the ballast blown: its mass to surfaced's share of neutral (blowDelay s, default 0, after the
                        // trim below)
                        blown = false;
                        // (blowTrim, given: its forward tanks blown first, trim water pumped aft, blowTrim of the fore
                        // tank's capacity, its bow up; pumped back once surfaced)
                        if (blowTrim > 0 && trimTanks.Count == 2)
                        {
                            PartResource fore = trimTanks[0].Resources.Get("LiquidFuel"), aft = trimTanks[1].Resources.Get("LiquidFuel");
                            blowMoved = Math.Min(blowTrim * fore.maxAmount, Math.Min(fore.amount, aft.maxAmount - aft.amount));
                            fore.amount -= blowMoved;
                            aft.amount += blowMoved;
                            trimTanks[0].UpdateMass();
                            trimTanks[1].UpdateMass();
                            ctx.Record("blow trim", $"{blowMoved * fore.info.density:F2} t pumped aft");
                        }
                    }
                    Ctrl.SetThrottle(throttle);
                    while (Planetarium.GetUniversalTime() - ts < sec)
                    {
                        yield return fixedWait;
                        double dt = Math.Max(1e-3, TimeWarp.fixedDeltaTime), t = Planetarium.GetUniversalTime() - tStart;
                        double d = DepthNow(), vz = Vz(), pitch = Pitch(), roll = Roll(), V = v.srfSpeed;
                        double q = (pitch - pitchWas) / dt, p = (roll - rollWas) / dt;
                        pitchWas = pitch;
                        rollWas = roll;
                        deepest = Math.Max(deepest, d);
                        if (phase == "breach" && !blown && Planetarium.GetUniversalTime() - ts >= blowDelay)
                        {
                            blown = true;
                            SetMass(v, surfacedShare * neutralT, Short);
                            ctx.Record("blow", $"at {d:F1} m deep, pitch {pitch:F1} deg: {RigidVessel.Mass(v):F3} t, " +
                                               $"{(1 - surfacedShare) * neutralT * g:F1} kN of buoyancy over its weight");
                        }
                        // (hover, given: diving and holding depth on its ballast as well as its planes, the tanks pumped
                        // up to hover's share of neutral heavier or lighter for the depth and its rate, at most that a
                        // 20 s: a big boat's planes too weak at its speed alone)
                        if (hover > 0 && phase == "dive")
                        {
                            double want = Math.Max(-1, Math.Min(1, (0.15 * (target - d) + vz) / 0.6));
                            double step = hover / 20 * dt;
                            hoverNow = Math.Max(hoverNow - step, Math.Min(hoverNow + step, want * hover));
                            if (Math.Abs(hoverNow - hoverSet) * neutralT > 0.05)
                            {
                                hoverSet = hoverNow;
                                SetMass(v, (1 - reserve + hoverSet) * neutralT, Short);
                            }
                        }
                        if (phase == "breach" || phase == "float")
                            highest = double.IsNaN(highest) ? -d : Math.Max(highest, -d);
                        if (phase == "breach" && double.IsNaN(breachedAt) && d < breachDepth)
                            breachedAt = t;
                        // (surfaced: the trim water blown aft pumped back over 20 s)
                        if (blowMoved > 0 && !double.IsNaN(breachedAt))
                        {
                            PartResource fore = trimTanks[0].Resources.Get("LiquidFuel"), aft = trimTanks[1].Resources.Get("LiquidFuel");
                            double back = Math.Min(blowMoved, Math.Max(1e-3, blowTrim * fore.maxAmount) / 20 * dt);
                            fore.amount += back;
                            aft.amount -= back;
                            blowMoved -= back;
                            trimTanks[0].UpdateMass();
                            trimTanks[1].UpdateMass();
                        }
                        float th = throttle;
                        // (breaching, full power until through the surface; then the speed held, as cruising)
                        bool surfaced = phase == "breach" && !double.IsNaN(breachedAt);
                        if (throttle > 0 && (phase != "breach" || surfaced))
                        {
                            double err = speed - V;
                            throttleI = Math.Max(0, Math.Min(1, throttleI + 0.05 * err * dt));
                            th = (float)Math.Max(0, Math.Min(1, throttleI + 0.2 * err));
                        }
                        Ctrl.SetThrottle(th);
                        // the depth autopilot (SubTests.Mission's): the pitch for the depth, the planes for the pitch;
                        // breaching, the pitch held up until through the surface, then the surface held (its middle at
                        // surfaceDepth); floating, the engine off, the planes holding it level as it slows
                        double pitchTarget;
                        if (phase == "breach" && !surfaced)
                            pitchTarget = breachPitch;
                        else if (phase == "float")
                            pitchTarget = 0;
                        else
                        {
                            double goal = surfaced ? surfaceDepth : target;
                            if (surfaced && !wasSurfaced)
                                depthI = 0;
                            depthI = Math.Max(-30, Math.Min(30, depthI + (goal - d) * dt));
                            pitchTarget = -Mathf.Clamp((float)(2 * (goal - d) + vzDamp * vz + 0.15 * depthI), -12, 12);
                        }
                        wasSurfaced = surfaced;
                        pitchI = Math.Max(-0.5, Math.Min(0.5, pitchI + 0.02 * (pitchTarget - pitch) * dt));
                        float cp = Mathf.Clamp((float)(0.08 * (pitchTarget - pitch) - pitchD * q + pitchI), -1, 1);
                        float cr = Mathf.Clamp((float)(0.05 * roll + 0.05 * p), -1, 1);
                        double heading = Heading(), r = Mathf.DeltaAngle((float)headingWas, (float)heading) / dt;
                        headingWas = heading;
                        double rateTarget = Mathf.Clamp(0.3f * Mathf.DeltaAngle((float)heading, (float)headingTarget), -3, 3);
                        float cy = Mathf.Clamp((float)(0.03 * (rateTarget - r)), -0.5f, 0.5f);
                        Ctrl.Set(pitch: cp, yaw: cy, roll: cr);
                        if (double.IsNaN(tumbledAt) && (Math.Abs(pitch) > 75 || Math.Abs(roll) > 75))
                        {
                            tumbledAt = t;
                            ctx.Record("tumbled", $"at {t:F1} s ({phase}): pitch {pitch:F0} deg, roll {roll:F0} deg, {d:F1} m deep");
                        }
                        if (!double.IsNaN(tumbledAt) && t - tumbledAt > 5)
                            done = true;
                        if (rec != null)
                        {
                            cam.camHdg = camHdg;
                            cam.camPitch = camPitch;
                            string caption = phase switch
                            {
                                "awash" => "Awash: tanks part full, deck at the water, cruising",
                                "dive" => d < 2 ? "Flooding the tanks: diving" : d < depthM - 1 ? $"Diving to {depthM:F0} m on the planes" : $"Holding {depthM:F0} m",
                                "breach" => double.IsNaN(breachedAt) ? "Emergency blow: ballast out, full power, planes up"
                                    : t - breachedAt < 4 ? "Breach!" : "Surfaced: planes level, running on the surface",
                                _ => "Stopped: floating",
                            };
                            if (!double.IsNaN(tumbledAt))
                                caption = "Out of control: tumbling";
                            rec.Lines = new[]
                            {
                                $"Test submarine - FAR water model {(FARDebugValues.hydrodynamics ? "on" : "off")}",
                                $"depth {d,5:F1} m   speed {V,4:F1} m/s   pitch {pitch,4:F0} deg",
                                caption,
                            };
                        }
                        csv.AppendLine(string.Join(",", F(t), phase, F(target), F(d), F(vz), F(V), F(pitch), F(pitchTarget), F(roll), F(cp), F(th)));
                    }
                }
                Ctrl.Neutral();
                Ctrl.SetThrottle(0);
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "subdemo.csv"), csv.ToString());
                ctx.Record("run", $"deepest {deepest:F1} m; " + (double.IsNaN(breachedAt) ? "never broke the surface" : $"broke the surface at {breachedAt:F1} s, its middle up to {highest:F1} m above the water") +
                                  (double.IsNaN(tumbledAt) ? "; never tumbled" : $"; tumbled at {tumbledAt:F1} s"));
                if (rec != null)
                    ctx.Record("frames", rec.Frame);
            }
            finally
            {
                Time.captureFramerate = 0;
                if (recorderGo != null)
                    UnityEngine.Object.Destroy(recorderGo);
                GameEvents.onShowUI.Fire();
                Ctrl.SetThrottle(0f);
                Ctrl.Neutral();
                TestWarp.Hyper(ctx, false);
                FARDebugValues.hydrodynamics = hydroWas;
                CheatOptions.NoCrashDamage = crashWas;
                WaterParams.Restore();
            }
        }

        // the hull's fuel (LiquidFuel first, then Oxidizer) for a total mass (t); the dry mass. shortBy given: the hull's
        // tanks too small told to it (the tons short), not asserted
        private static double SetMass(Vessel v, double massT, Action<double> shortBy = null)
        {
            // (the hull's fuel only: a ballast tank's ore stays)
            foreach (string name in new[] { "LiquidFuel", "Oxidizer" })
                if (v.rootPart.Resources.Get(name) is PartResource r0)
                    r0.amount = 0;
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
            if (shortBy != null && massT > 0 && need >= 1e-3)
                shortBy(need);
            else
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
