using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR in flight on a stock plane placed in level flight: the force it applies against the vessel's own acceleration,
    /// its prediction API against the force it applies, and FARAPI's flight numbers against the conditions.
    /// </summary>
    public static class FlightTests
    {

        // over the sea east of KSC, heading east
        private static FarFlightProbe.Condition Cruise(double aoaDeg) => Far.At(0.5, 3000, aoaDeg, $"cruise {aoaDeg}");

        /// <summary>
        /// The craft in the air at the condition, SAS off (it would move the control surfaces) and gear up: placed once
        /// to come off the runway and settle (FAR revoxelizes and rebuilds its prediction state once the vessel is
        /// flying), then placed again at the condition.
        /// </summary>
        internal static IEnumerator Airborne(FarFlightProbe.Condition c)
        {
            Vessel v = FlightGlobals.ActiveVessel;
            v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
            Flight.IgnoreMaxTemperature(true);
            Ctrl.SetThrottle(0f);
            yield return FarFlightProbe.Place(c);
            Ctrl.RetractGear();
            yield return Wait.Seconds(8);
            yield return FarFlightProbe.Place(c);
        }

        /// <summary>
        /// Held at an angle of attack: the drag and lift FAR applies are what accelerate the vessel (its kinematic force:
        /// at zero throttle with the gear up, the aero force is the only one but gravity).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_AppliedForceIsTheAeroForce(TestContext ctx)
        {
            yield return Wait.Seconds(5);   // FAR voxelizes the craft after it loads
            FarFlightProbe.Condition c = Cruise(4);
            yield return Airborne(c);
            var samples = new List<FarFlightProbe.Sample>();
            yield return FarFlightProbe.Hold(c, 25, 50, samples, predict: false);
            Assert.True(samples.Count >= 40, $"samples held ({samples.Count})");
            double drag = samples.Average(s => s.Flight[0]), lift = samples.Average(s => s.Flight[1]);
            double kinDrag = samples.Where(s => !double.IsNaN(s.Kinematic[0])).Average(s => s.Kinematic[0]);
            double kinLift = samples.Where(s => !double.IsNaN(s.Kinematic[1])).Average(s => s.Kinematic[1]);
            ctx.Record("drag", $"applied {drag:F2} kN, kinematic {kinDrag:F2}");
            ctx.Record("lift", $"applied {lift:F2} kN, kinematic {kinLift:F2}");
            Assert.True(drag > 0 && lift > 0, $"drag ({drag:F2} kN) and lift ({lift:F2} kN) at 4 deg");
            Assert.Near(drag, kinDrag, 0.05 * drag + 0.2, "drag applied against the drag the vessel feels");
            Assert.Near(lift, kinLift, 0.05 * lift + 0.2, "lift applied against the lift the vessel feels");
        }

        /// <summary>
        /// FAR's vessel snapshot (FARAPI.CaptureVesselSnapshot, CalculateSnapshotAeroForcesAt: a prediction at a place and
        /// time, from a copy of the vessel's aero model) at the state the vessel flies in is the force FAR applies there,
        /// and full pitch input changes the pitching moment. Before predictions took the game atmosphere, ram drag and
        /// settled wings, the Aeris 4A at Mach 0.5, 3 km, 4 deg was predicted with 8.6 kN of drag against 14.2 kN applied
        /// and 138.0 kN of total force against 128.1 (measured). Off the main thread: Flight_SnapshotIsExactOnManyThreads.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_SnapshotPredictsTheAppliedForce(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Cruise(4);
            yield return Airborne(c);
            var samples = new List<FarFlightProbe.Sample>();
            yield return FarFlightProbe.Hold(c, 25, 30, samples, predict: false);
            double drag = samples.Average(s => s.Flight[0]), lift = samples.Average(s => s.Flight[1]);

            Vessel v = FlightGlobals.ActiveVessel;
            FARVesselSnapshot snap = FARAPI.CaptureVesselSnapshot(v);
            Assert.NotNull(snap, "a snapshot of the flying vessel");
            Vector3 vel = v.srf_velocity;
            var where = new Vector3d(v.latitude, v.longitude, v.altitude);
            double ut = Planetarium.GetUniversalTime();
            Assert.True(FARAPI.CalculateSnapshotAeroForcesAt(snap, out Vector3 f, out Vector3 tq, vel, where, ut), "the snapshot prediction ran");
            double predDrag = -Vector3.Dot(f, vel.normalized);
            double predTotal = f.magnitude, total = Math.Sqrt(drag * drag + lift * lift);
            ctx.Record("drag", $"applied {drag:F2} kN, snapshot {predDrag:F2}");
            ctx.Record("force", $"applied |F| {total:F2} kN, snapshot {predTotal:F2}");
            Assert.Near(drag, predDrag, 0.03 * drag + 0.05, "the snapshot's drag against the drag FAR applies");
            Assert.Near(total, predTotal, 0.03 * total + 0.05, "the snapshot's total force against the one FAR applies");

            // full nose-up and nose-down input: the control surfaces move, and so does the pitching moment
            Vector3 pitchAxis = Flight.PitchUpAxis(v);
            Assert.True(FARAPI.CalculateSnapshotAeroForcesWithControlsAt(snap, out _, out Vector3 tUp, vel, where, ut, 1, 0, 0), "prediction with pitch +1");
            Assert.True(FARAPI.CalculateSnapshotAeroForcesWithControlsAt(snap, out _, out Vector3 tDown, vel, where, ut, -1, 0, 0), "prediction with pitch -1");
            double mUp = Vector3.Dot(tUp, pitchAxis), mDown = Vector3.Dot(tDown, pitchAxis), m0 = Vector3.Dot(tq, pitchAxis);
            ctx.Record("pitchMoment", $"input -1 {mDown:F2}, 0 {m0:F2}, +1 {mUp:F2} kN m");
            Assert.True(mUp > m0 && m0 > mDown, $"pitching moment rises with pitch input ({mDown:F2} < {m0:F2} < {mUp:F2} kN m)");
        }

        /// <summary>
        /// FAR's prediction API (FARAPI.CalculateVesselAeroForcesAt) at the state the vessel flies in gives the force FAR
        /// applies there. The altitude-only API it replaced predicted the Aeris 4A at Mach 0.5, 3 km, 4 deg with 40% too
        /// little drag and 8% too much lift (8.5 against 14.1 kN, 137.9 against 127.3 kN).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_PredictionIsTheAppliedForce(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0.5, 3000, 4, "cruise 4");
            yield return Airborne(c);
            Assert.True(FarApi.HasPredictAt, "FAR has FARAPI.CalculateVesselAeroForcesAt");
            var samples = new List<FarFlightProbe.Sample>();
            yield return FarFlightProbe.Hold(c, 25, 50, samples);
            double drag = samples.Average(s => s.Flight[0]), lift = samples.Average(s => s.Flight[1]);
            double predDrag = samples.Average(s => s.Predicted[0]), predLift = samples.Average(s => s.Predicted[1]);
            ctx.Record("drag", $"applied {drag:F2} kN, predicted {predDrag:F2}");
            ctx.Record("lift", $"applied {lift:F2} kN, predicted {predLift:F2}");
            Assert.Near(drag, predDrag, 0.03 * drag + 0.05, "FAR's prediction of the drag it applies");
            Assert.Near(lift, predLift, 0.03 * lift + 0.05, "FAR's prediction of the lift it applies");
        }

        // Exact equality of two force/torque pairs (Unity's == is equality within about 1e-5).
        private static bool Same(Vector3 f1, Vector3 t1, Vector3 f2, Vector3 t2) =>
            f1.x.Equals(f2.x) && f1.y.Equals(f2.y) && f1.z.Equals(f2.z) && t1.x.Equals(t2.x) && t1.y.Equals(t2.y) && t1.z.Equals(t2.z);

        private static string Exact(Vector3 x) =>
            $"({x.x.ToString("R", CultureInfo.InvariantCulture)}, {x.y.ToString("R", CultureInfo.InvariantCulture)}, {x.z.ToString("R", CultureInfo.InvariantCulture)})";

        /// <summary>
        /// Flying at 5 deg with the control surfaces locked (so the control inputs cannot move them between predictions)
        /// and settled; the number of surfaces locked goes in locked[0].
        /// </summary>
        private static IEnumerator SettledWithSurfacesLocked(FarFlightProbe.Condition c, int[] locked)
        {
            yield return Wait.Seconds(5);
            locked[0] = FarApi.SetMaxDeflect(FlightGlobals.ActiveVessel, 0);
            yield return Airborne(c);
            yield return FarFlightProbe.Hold(c, 25, 1, new List<FarFlightProbe.Sample>(), predict: false);
        }

        // Unrelated conditions to predict between: Mach 0.3-8, AoA -10 to 45 deg (plus 2 deg a repeat), 1-50 km, each at
        // its own place and time.
        private static List<(Vector3 Vel, Vector3d LatLonAlt, double UT)> OtherConditions(Vessel v, double ut, int repeats)
        {
            CelestialBody body = v.mainBody;
            Vector3 nose = v.ReferenceTransform.up, roof = -v.ReferenceTransform.forward;
            var list = new List<(Vector3, Vector3d, double)>();
            for (int rep = 0; rep < repeats; ++rep)
                foreach (double mach in new[] { 0.3, 0.9, 2.0, 5.0, 8.0 })
                    foreach (double aoa in new[] { -10.0, 0.0, 10.0, 25.0, 45.0 })
                        foreach (double altM in new[] { 1000.0, 20000.0, 50000.0 })
                        {
                            int n = list.Count + 1;
                            double p = body.GetPressure(altM);
                            double speed = mach * body.GetSpeedOfSound(p, body.GetDensity(p, body.GetTemperature(altM)));
                            double a = (aoa + 2 * rep) * Math.PI / 180;
                            list.Add(((float)speed * ((float)Math.Cos(a) * nose - (float)Math.Sin(a) * roof),
                                      new Vector3d(60 - 7 * n % 120, 180 - 13 * n % 360, altM), ut + 900 * n));
                        }
            return list;
        }

        /// <summary>
        /// A prediction leaves nothing behind: FAR's prediction API is called by other mods (Trajectories over a whole
        /// grid of conditions, MechJeb's VesselState) while the vessel flies, so a prediction must neither change the
        /// state the flight reads nor depend on the predictions before it. In one frame: every value field of FAR's
        /// per-part state (FarStateSnapshot) taken, the vessel's own condition predicted, 75 unrelated conditions
        /// predicted, the own condition predicted again, the state taken again. No field may change and the two
        /// predictions of the own condition must be exactly equal. Writes isolation.md (the fields that changed).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_PredictionsLeaveFarAsItWas(TestContext ctx)
        {
            FarFlightProbe.Condition c = Cruise(5);
            var locked = new int[1];
            yield return SettledWithSurfacesLocked(c, locked);
            ctx.Record("surfacesLocked", locked[0]);

            ctx.Step("State, predictions, state (one frame)");
            Vessel v = FlightGlobals.ActiveVessel;
            double ut = Planetarium.GetUniversalTime();
            Vector3 velA = v.srf_velocity;
            var whereA = new Vector3d(v.latitude, v.longitude, v.altitude);
            var others = OtherConditions(v, ut, 1);

            Dictionary<string, string> before = FarStateSnapshot.Take(v);
            FarApi.ClearMemo(v);
            FARAPI.CalculateVesselAeroForcesAt(v, out Vector3 fA0, out Vector3 tA0, velA, whereA, ut);
            foreach (var o in others)
            {
                FarApi.ClearMemo(v);
                FARAPI.CalculateVesselAeroForcesAt(v, out _, out _, o.Vel, o.LatLonAlt, o.UT);
            }
            FarApi.ClearMemo(v);
            FARAPI.CalculateVesselAeroForcesAt(v, out Vector3 fA1, out Vector3 tA1, velA, whereA, ut);
            Dictionary<string, string> after = FarStateSnapshot.Take(v);

            List<string> changed = FarStateSnapshot.Diff(before, after);
            bool sameA = Same(fA0, tA0, fA1, tA1);
            int parts = changed.Select(FarStateSnapshot.PartOf).Distinct().Count();
            ctx.Record("fieldsWatched", before.Count);
            ctx.Record("fieldsChanged", changed.Count);
            ctx.Record("partsChanged", parts);
            ctx.Record("sameAnswerForA", sameA);
            if (!sameA)
                ctx.Record("answerForA", $"force {Exact(fA0)} then {Exact(fA1)}; torque {Exact(tA0)} then {Exact(tA1)}");

            var md = new StringBuilder("# FAR prediction isolation\n\n");
            md.Append($"{v.vesselName} flying at {c}, control surfaces locked. In one frame: {before.Count} value fields of FAR's per-part state ");
            md.Append($"taken, the own condition predicted, {others.Count} unrelated conditions predicted, the own condition predicted again, the state taken again.\n\n");
            md.Append($"- fields changed: **{changed.Count}** on {parts} parts\n");
            md.Append($"- same answer for the own condition before and after: **{(sameA ? "yes" : "no")}**\n");
            md.Append("\n## Changed fields, by name\n\n");
            foreach (var g in changed.Select(FarStateSnapshot.FieldOf).GroupBy(f => f).OrderByDescending(g => g.Count()))
                md.Append($"- {g.Key}: {g.Count()} parts\n");
            md.Append("\n## Every change\n\n```\n").Append(string.Join("\n", changed.Take(2000))).Append("\n```\n");
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "isolation.md"), md.ToString());

            Assert.True(changed.Count == 0, $"predictions left FAR's flight state as it was ({changed.Count} fields changed: {FarStateSnapshot.Summarize(changed, 8)})");
            Assert.True(sameA, "the own condition predicted the same before and after the others");
        }

        /// <summary>
        /// FAR's snapshot predictions off the main thread, flying with the surfaces locked: (1) in the capture frame, the
        /// snapshot's answer for the vessel's own condition against the live API's, to 1e-6 relative (the same inputs by
        /// different float arithmetic; everything else here is exact); (2) every input (the own condition and 300
        /// unrelated conditions and places) on one worker thread exactly as on the main thread; (3) 8 workers at once,
        /// each in its own order, the main thread waiting inside one frame: every answer exact and FAR's state
        /// (FarStateSnapshot) unchanged; (4) 8 workers for 100 frames while the vessel flies, the main thread predicting
        /// live and capturing new snapshots meanwhile: every answer still exact; (5) the live-vessel API refuses a worker
        /// thread (zero); (6) after the gear comes down and FAR re-voxelizes, the old snapshot is refused and a new one
        /// works. Writes threads.md.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_SnapshotIsExactOnManyThreads(TestContext ctx)
        {
            const int threads = 8, repeats = 3, flyFrames = 100;
            FarFlightProbe.Condition c = Cruise(5);
            yield return SettledWithSurfacesLocked(c, new int[1]);

            // ---- the capture frame: everything up to the 8-thread batch happens with no physics step in between
            ctx.Step("Capture frame: snapshot vs live, main thread vs one worker, 8 workers");
            Vessel v = FlightGlobals.ActiveVessel;
            double ut = Planetarium.GetUniversalTime();
            var inputs = new List<(Vector3 Vel, Vector3d LatLonAlt, double UT)> { (v.srf_velocity, new Vector3d(v.latitude, v.longitude, v.altitude), ut) };
            inputs.AddRange(OtherConditions(v, ut, 4));
            int count = inputs.Count;

            FARVesselSnapshot snap = FARAPI.CaptureVesselSnapshot(v);
            Assert.NotNull(snap, "a snapshot of the flying vessel");
            FarApi.ClearMemo(v);
            FARAPI.CalculateVesselAeroForcesAt(v, out Vector3 fLive, out Vector3 tLive, inputs[0].Vel, inputs[0].LatLonAlt, inputs[0].UT);

            var main = new Vector3[count, 2];
            var ok = new bool[count];
            for (int i = 0; i < count; ++i)
                ok[i] = FARAPI.CalculateSnapshotAeroForcesAt(snap, out main[i, 0], out main[i, 1], inputs[i].Vel, inputs[i].LatLonAlt, inputs[i].UT);
            int refused = ok.Count(b => !b);
            // The live API reads Unity's transforms and KSP's atmosphere functions, the snapshot its copies of them: the
            // same inputs by different arithmetic, so they agree to float rounding, not bit for bit.
            const double LiveTolerance = 1e-6;
            double relForce = (main[0, 0] - fLive).magnitude / Math.Max(1e-9f, fLive.magnitude);
            // A trimmed vessel's torque is near zero, so its rounding is scaled by what it is made of: the force times
            // the vessel's size (a float's rounding in a lever arm shows as force x that rounding).
            double torqueScale = Math.Max(tLive.magnitude, fLive.magnitude * v.vesselSize.magnitude);
            double relTorque = (main[0, 1] - tLive).magnitude / Math.Max(1e-9, torqueScale);
            bool snapMatchesLive = relForce <= LiveTolerance && relTorque <= LiveTolerance;
            ctx.Record("snapshotVsLiveRel", $"force {relForce:E2}, torque {relTorque:E2} (tolerance {LiveTolerance:E0})");

            var errors = new System.Collections.Concurrent.ConcurrentQueue<string>();
            var mismatches = new int[1];
            // Runs every input in the given order `times` times, counting answers that differ from the main thread's.
            void Work(int[] order, int times)
            {
                try
                {
                    for (int r = 0; r < times; ++r)
                        foreach (int i in order)
                        {
                            bool good = FARAPI.CalculateSnapshotAeroForcesAt(snap, out Vector3 f, out Vector3 t, inputs[i].Vel, inputs[i].LatLonAlt, inputs[i].UT);
                            if (good != ok[i] || !Same(f, t, main[i, 0], main[i, 1]))
                                Interlocked.Increment(ref mismatches[0]);
                        }
                }
                catch (Exception e) { errors.Enqueue(e.ToString()); }
            }

            int[] identity = Enumerable.Range(0, count).ToArray();
            var one = new Thread(() => Work(identity, 1));
            one.Start();
            one.Join();
            int oneWorkerMismatches = mismatches[0];

            Dictionary<string, string> before = FarStateSnapshot.Take(v);
            mismatches[0] = 0;
            var rng = new System.Random(4242);
            var workers = new List<Thread>();
            var sw = System.Diagnostics.Stopwatch.StartNew();
            for (int k = 0; k < threads; ++k)
            {
                int[] order = identity.OrderBy(_ => rng.Next()).ToArray();
                workers.Add(new Thread(() => Work(order, repeats)));
            }
            foreach (var w in workers) w.Start();
            foreach (var w in workers) w.Join();
            double frozenBatchMs = sw.Elapsed.TotalMilliseconds;
            List<string> changed = FarStateSnapshot.Diff(before, FarStateSnapshot.Take(v));
            int frozenMismatches = mismatches[0];
            ctx.Record("frozenBatch", $"{threads} threads x {repeats} x {count} in {frozenBatchMs:F0} ms");

            // ---- flying: workers keep predicting across frames while the main thread flies and predicts live
            ctx.Step($"{threads} workers for {flyFrames} frames while flying");
            mismatches[0] = 0;
            var stop = new bool[1];
            int calls = 0;
            workers.Clear();
            for (int k = 0; k < threads; ++k)
            {
                int[] order = identity.OrderBy(_ => rng.Next()).ToArray();
                workers.Add(new Thread(() =>
                {
                    while (!Volatile.Read(ref stop[0]))
                    {
                        Work(order, 1);
                        Interlocked.Add(ref calls, order.Length);
                    }
                }));
            }
            foreach (var w in workers) w.Start();
            double maxLiveMs = 0, maxCaptureMs = 0;
            int captures = 0, capturesRefused = 0;
            var fixedWait = new WaitForFixedUpdate();
            try
            {
                for (int frame = 0; frame < flyFrames; ++frame)
                {
                    Flight.PointAtAoA(c.AoADeg);
                    foreach (Part part in FlightGlobals.ActiveVessel.parts)
                        if (part.Rigidbody != null) part.Rigidbody.angularVelocity = Vector3.zero;
                    yield return fixedWait;
                    v = FlightGlobals.ActiveVessel;
                    var where = new Vector3d(v.latitude, v.longitude, v.altitude);
                    double now = Planetarium.GetUniversalTime();
                    var t0 = System.Diagnostics.Stopwatch.StartNew();
                    FarApi.ClearMemo(v);
                    FARAPI.CalculateVesselAeroForcesAt(v, out _, out _, v.srf_velocity, where, now);
                    maxLiveMs = Math.Max(maxLiveMs, t0.Elapsed.TotalMilliseconds);
                    if (frame % 10 != 0)
                        continue;
                    t0 = System.Diagnostics.Stopwatch.StartNew();
                    FARVesselSnapshot s2 = FARAPI.CaptureVesselSnapshot(v);
                    maxCaptureMs = Math.Max(maxCaptureMs, t0.Elapsed.TotalMilliseconds);
                    ++captures;
                    if (s2 == null || !FARAPI.CalculateSnapshotAeroForcesAt(s2, out _, out _, v.srf_velocity, where, now))
                        ++capturesRefused;
                }
            }
            finally
            {
                Volatile.Write(ref stop[0], true);
                foreach (var w in workers) w.Join();
            }
            int flyingMismatches = mismatches[0];
            ctx.Record("mainThreadMaxMs", $"live prediction {maxLiveMs:F2}, capture {maxCaptureMs:F2}");

            // ---- the live API from a worker must refuse
            ctx.Step("Live API from a worker, stale snapshot");
            Vector3 fOff = Vector3.one, tOff = Vector3.one;
            Vessel vw = FlightGlobals.ActiveVessel;
            Vector3 velW = vw.srf_velocity;
            var lla = new Vector3d(vw.latitude, vw.longitude, vw.altitude);
            double utW = Planetarium.GetUniversalTime();
            var off = new Thread(() =>
            {
                try { FARAPI.CalculateVesselAeroForcesAt(vw, out fOff, out tOff, velW, lla, utW); }
                catch (Exception e) { errors.Enqueue("live API off-thread: " + e.Message); }
            });
            off.Start();
            off.Join();
            bool liveRefusedOffThread = fOff == Vector3.zero && tOff == Vector3.zero;

            // ---- stale: the gear changes the shape, FAR re-voxelizes, the old snapshot must be refused
            Ctrl.Gear(true);
            yield return Wait.Seconds(6);
            v = FlightGlobals.ActiveVessel;
            bool staleRefused = !FARAPI.CalculateSnapshotAeroForcesAt(snap, out _, out _, inputs[0].Vel, inputs[0].LatLonAlt, inputs[0].UT);
            FARVesselSnapshot fresh = FARAPI.CaptureVesselSnapshot(v);
            bool freshWorks = fresh != null && FARAPI.CalculateSnapshotAeroForcesAt(fresh, out _, out _, v.srf_velocity,
                new Vector3d(v.latitude, v.longitude, v.altitude), Planetarium.GetUniversalTime());

            var md = new StringBuilder("# FAR predictions off the main thread\n\n");
            md.Append($"{v.vesselName} flying at {c}, surfaces locked; {count} inputs (the own condition and unrelated conditions, places and times).\n\n");
            md.Append("| check | result |\n|---|---|\n");
            md.Append($"| snapshot answer = live answer to {LiveTolerance:E0} relative, own condition, capture frame | {snapMatchesLive} (force {relForce:E2}, torque {relTorque:E2}) |\n");
            md.Append($"| fresh snapshot refused | {refused} of {count} |\n");
            md.Append($"| one worker = main thread | {count - oneWorkerMismatches} of {count} exact |\n");
            md.Append($"| {threads} workers x {repeats}, main thread waiting in one frame | {frozenMismatches} mismatches in {threads * repeats * count}, {frozenBatchMs:F0} ms |\n");
            md.Append($"| FAR flight state changed by that batch | {changed.Count} fields |\n");
            md.Append($"| {threads} workers for {flyFrames} frames while flying | {flyingMismatches} mismatches in {calls} calls |\n");
            md.Append($"| main thread meanwhile: live prediction / capture, max ms | {maxLiveMs:F2} / {maxCaptureMs:F2} |\n");
            md.Append($"| snapshots captured while flying (refused) | {captures} ({capturesRefused}) |\n");
            md.Append($"| live-vessel API from a worker refused | {liveRefusedOffThread} |\n");
            md.Append($"| old snapshot refused after re-voxelization / new one works | {staleRefused} / {freshWorks} |\n");
            md.Append($"| exceptions on workers | {errors.Count} |\n");
            if (errors.Count > 0)
                md.Append("\n## Exceptions\n\n```\n").Append(string.Join("\n---\n", errors.Take(5))).Append("\n```\n");
            if (changed.Count > 0)
                md.Append("\n## Changed fields\n\n```\n").Append(string.Join("\n", changed.Take(500))).Append("\n```\n");
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "threads.md"), md.ToString());

            Assert.True(errors.Count == 0, $"no exceptions on worker threads ({errors.Count}; first: {errors.FirstOrDefault()})");
            Assert.True(snapMatchesLive, $"the snapshot's answer for the own condition is the live API's to {LiveTolerance:E0} (force {relForce:E2}, torque {relTorque:E2})");
            Assert.True(refused == 0, $"a fresh snapshot is not refused ({refused} of {count})");
            Assert.True(oneWorkerMismatches == 0, $"one worker thread gives the main thread's answers ({oneWorkerMismatches} differ)");
            Assert.True(frozenMismatches == 0, $"{threads} concurrent workers give the main thread's answers ({frozenMismatches} differ)");
            Assert.True(changed.Count == 0, $"concurrent predictions leave FAR's flight state as it was ({changed.Count} fields: {FarStateSnapshot.Summarize(changed, 8)})");
            Assert.True(flyingMismatches == 0, $"workers predicting while the vessel flies give the main thread's answers ({flyingMismatches} of {calls} differ)");
            Assert.True(capturesRefused == 0, $"snapshots captured while flying work ({capturesRefused} of {captures} refused)");
            Assert.True(liveRefusedOffThread, "the live-vessel API refuses a worker thread");
            Assert.True(staleRefused && freshWorks, $"a snapshot taken before re-voxelization is refused ({staleRefused}) and a new one works ({freshWorks})");
        }

        /// <summary>FARAPI's flight numbers: dynamic pressure from the air and speed, the angle of attack held, Cl and Cd signs.</summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_ApiReportsTheConditions(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Cruise(5);
            yield return Airborne(c);
            var samples = new List<FarFlightProbe.Sample>();
            yield return FarFlightProbe.Hold(c, 25, 10, samples, predict: false);

            Vessel v = FlightGlobals.ActiveVessel;
            double q = 0.5 * v.atmDensity * v.srfSpeed * v.srfSpeed / 1000;   // kPa, as FAR reports it
            ctx.Record("dynPres", $"FAR {FARAPI.VesselDynPres(v):F3} kPa, 1/2 rho v^2 {q:F3}");
            ctx.Record("aoa", FARAPI.VesselAoA(v));
            ctx.Record("coefficients", $"Cl {FARAPI.VesselLiftCoeff(v):F4} Cd {FARAPI.VesselDragCoeff(v):F4} refArea {FARAPI.VesselRefArea(v):F2}");
            Assert.Near(q, FARAPI.VesselDynPres(v), 0.02 * q, "dynamic pressure is 1/2 rho v^2");
            Assert.Near(c.AoADeg, FARAPI.VesselAoA(v), 1.0, "the angle of attack held");
            Assert.True(FARAPI.VesselLiftCoeff(v) > 0, "positive Cl at a positive angle of attack");
            Assert.True(FARAPI.VesselDragCoeff(v) > 0, "positive Cd");
            Assert.True(FARAPI.VesselRefArea(v) > 0, "a reference area");
            Assert.True(FARAPI.VesselVoxelizationCompletedEver(v), "the vessel was voxelized");

            // the altitude-only prediction: drag against the velocity and lift up, of the flight's size (how close it
            // comes is for the prediction work's place-and-time API to test: master's is 40% short on drag)
            ctx.Step("CalculateVesselAeroForces at the vessel's speed and altitude");
            double drag = samples.Average(s => s.Flight[0]), lift = samples.Average(s => s.Flight[1]);
            Vector3 vel = v.srf_velocity;
            FARAPI.CalculateVesselAeroForces(v, out Vector3 f, out Vector3 tq, vel, v.altitude);
            Vector3 up = Vector3.ProjectOnPlane(v.upAxis, vel).normalized;
            double predDrag = -Vector3.Dot(f, vel.normalized), predLift = Vector3.Dot(f, up);
            ctx.Record("altitudeApi", $"drag {predDrag:F2} kN (flying {drag:F2}), lift {predLift:F2} kN (flying {lift:F2}), torque {tq.magnitude:F2} kN m");
            Assert.True(predDrag > 0.3 * drag && predDrag < 3 * drag, $"the predicted drag is the flight's to within a factor of 3 ({predDrag:F2} against {drag:F2} kN)");
            Assert.True(predLift > 0.5 * lift && predLift < 2 * lift, $"the predicted lift is the flight's to within a factor of 2 ({predLift:F2} against {lift:F2} kN)");

            ctx.Step("ToggleAirspeedDisplay, for every vessel and for this one");
            AirspeedSettingsGUI air = Far.FlightGui(v).airSpeedGUI;
            bool allAtStart = AirspeedSettingsGUI.allEnabled, vesselAtStart = air.enabled;
            try
            {
                Assert.True(FARAPI.ToggleAirspeedDisplay(false), "turning the airspeed display off everywhere");
                Assert.False(AirspeedSettingsGUI.allEnabled, "the airspeed display off everywhere");
                Assert.True(FARAPI.ToggleAirspeedDisplay(), "toggling it everywhere");
                Assert.True(AirspeedSettingsGUI.allEnabled, "toggled back on everywhere");
                Assert.True(FARAPI.ToggleAirspeedDisplay(false, v), "turning it off for the vessel");
                Assert.False(air.enabled, "off for the vessel");
                Assert.True(AirspeedSettingsGUI.allEnabled, "still on everywhere else");
                Assert.True(FARAPI.ToggleAirspeedDisplay(null, v), "toggling it for the vessel");
                Assert.True(air.enabled, "back on for the vessel");
            }
            finally
            {
                AirspeedSettingsGUI.allEnabled = allAtStart;
                air.enabled = vesselAtStart;
            }

            ctx.Step("the rest of the flight numbers, for the vessel and the active vessel");
            // valid except while a revoxelization (FAR redoes the voxels in flight now and then) is under way; the
            // numbers below are all read in the frame it is
            yield return Wait.Until(() => FARAPI.VesselVoxelizationCompletedAndValid(v), 60, "a valid voxelization", shot: false);
            double rho = v.atmDensity, eas = FARAPI.VesselEAS(v), ias = FARAPI.VesselIAS(v);
            ctx.Record("airspeeds", $"TAS {v.srfSpeed:F1} EAS {eas:F1} IAS {ias:F1} m/s");
            Assert.Near(v.srfSpeed * Math.Sqrt(rho / 1.225), eas, 0.02 * eas, "EAS is TAS * sqrt(rho / rho0)");
            // the pitot reading at Mach 0.5, 3 km: compressibility puts it a little above the EAS
            Assert.True(ias > 0.99 * eas && ias < 1.06 * eas, $"IAS just above the EAS ({ias:F1} against {eas:F1} m/s)");
            Assert.Near(0, FARAPI.VesselSideslip(v), 1.0, "no sideslip heading along the velocity");
            Assert.True(FARAPI.VesselStallFrac(v) >= 0 && FARAPI.VesselStallFrac(v) < 0.05, $"unstalled at 5 deg ({FARAPI.VesselStallFrac(v):F3})");
            double bc = FARAPI.VesselBallisticCoeff(v), tv = FARAPI.VesselTermVelEst(v);
            ctx.Record("ballistic", $"coefficient {bc:F1} kg/m^2, terminal velocity {tv:F1} m/s");
            Assert.Near(v.GetTotalMass() * 1000 / (FARAPI.VesselDragCoeff(v) * FARAPI.VesselRefArea(v)), bc, 0.02 * bc,
                        "ballistic coefficient m / (Cd S)");
            double g = FlightGlobals.getGeeForceAtPosition(v.CoM).magnitude;
            Assert.Near(Math.Sqrt(2 * bc * g / rho), tv, 0.02 * tv, "terminal velocity sqrt(2 BC g / rho)");
            Vector3 aero = FARAPI.VesselAerodynamicForce(v);
            double appliedDrag = -Vector3.Dot(aero, v.srf_velocity.normalized);
            // the vessel flies free since the hold: against FAR's own coefficient now
            double cdqs = FARAPI.VesselDragCoeff(v) * FARAPI.VesselDynPres(v) * FARAPI.VesselRefArea(v);
            ctx.Record("drag", $"aerodynamic force {appliedDrag:F2} kN, Cd q S {cdqs:F2} kN (held {drag:F2})");
            Assert.Near(cdqs, appliedDrag, 0.02 * cdqs + 0.05, "the aerodynamic force's drag is Cd q S");
            Assert.False(FARAPI.VesselVoxelizationCompletedAndValid(null), "no voxelization for no vessel");
            Assert.Equal(FARAPI.VesselAoA(v), FARAPI.ActiveVesselAoA(), "the active vessel's angle of attack");
            Assert.Equal(FARAPI.VesselDynPres(v), FARAPI.ActiveVesselDynPres(), "the active vessel's dynamic pressure");
            Assert.Equal(FARAPI.VesselLiftCoeff(v), FARAPI.ActiveVesselLiftCoeff(), "the active vessel's Cl");
            Assert.Equal(FARAPI.VesselDragCoeff(v), FARAPI.ActiveVesselDragCoeff(), "the active vessel's Cd");
            Assert.Equal(FARAPI.VesselRefArea(v), FARAPI.ActiveVesselRefArea(), "the active vessel's reference area");
            Assert.Equal(bc, FARAPI.ActiveVesselBallisticCoeff(), "the active vessel's ballistic coefficient");
            Assert.Equal(tv, FARAPI.ActiveVesselTermVelEst(), "the active vessel's terminal velocity");
            Assert.Equal(FARAPI.VesselSideslip(v), FARAPI.ActiveVesselSideslip(), "the active vessel's sideslip");
            Assert.Equal(FARAPI.VesselStallFrac(v), FARAPI.ActiveVesselStallFrac(), "the active vessel's stall fraction");
            Assert.Equal(eas, FARAPI.ActiveVesselEAS(), "the active vessel's EAS");
            Assert.Equal(ias, FARAPI.ActiveVesselIAS(), "the active vessel's IAS");
            Assert.Equal(aero, FARAPI.ActiveVesselAerodynamicForce(), "the active vessel's aerodynamic force");
            Assert.Equal(FARAPI.VesselAerodynamicTorque(v), FARAPI.ActiveVesselAerodynamicTorque(), "the active vessel's aerodynamic torque");
            Assert.Equal(0.0, FARAPI.VesselTSFC(v), "no fuel burnt at zero throttle");

            ctx.Step("TSFC with the jets running");
            ModuleEngines[] jets = v.parts.SelectMany(p => p.FindModulesImplementing<ModuleEngines>())
                                    .Where(e => e.propellants.Any(pr => pr.name == "IntakeAir")).ToArray();
            Assert.True(jets.Length > 0, "the craft has jets");
            foreach (ModuleEngines e in jets)
                e.Activate();
            Ctrl.SetThrottle(1f);
            yield return Wait.Until(() => jets.Any(e => e.finalThrust > 1), 30, "the jets spooled up");
            yield return Wait.Seconds(1);
            ModuleEngines[] running = jets.Where(e => e.EngineIgnited && !e.engineShutdown).ToArray();
            double thrust = running.Sum(e => e.finalThrust), thrustIsp = running.Sum(e => e.finalThrust * e.realIsp);
            double tsfc = FARAPI.VesselTSFC(v);
            ctx.Record("tsfc", $"{tsfc:F4} /h, from {running.Length} jet(s): {thrust:F1} kN at Isp {thrustIsp / thrust:F0} s");
            // FAR's: per hour, 1 / the thrust-weighted Isp
            Assert.Near(3600 * thrust / thrustIsp, tsfc, 0.03 * tsfc, "TSFC is 3600 / Isp per hour");
            Assert.Equal(tsfc, FARAPI.ActiveVesselTSFC(), "the active vessel's TSFC");

            ctx.Step("RasterPropMonitor's FAR variables (FARPM)");
            // RPM finds the module on a part and asks it for its variables by name, in the frame it draws
            FARPM rpm = v.rootPart.gameObject.AddComponent<FARPM>();
            try
            {
                var expected = new Dictionary<string, object>
                {
                    ["FARPM_DYNAMIC_PRESSURE_Q"] = FARAPI.ActiveVesselDynPres(),
                    ["FARPM_LIFT_COEFFICIENT_CL"] = FARAPI.ActiveVesselLiftCoeff(),
                    ["FARPM_DRAG_COEFFICIENT_CD"] = FARAPI.ActiveVesselDragCoeff(),
                    ["FARPM_REFAREA"] = FARAPI.ActiveVesselRefArea(),
                    ["FARPM_MACHNUMBER"] = v.mach,
                    ["FARPM_TERMINALVELOCITY"] = FARAPI.ActiveVesselTermVelEst(),
                    ["FARPM_BALLISTIC_COEFFICIENT"] = FARAPI.ActiveVesselBallisticCoeff(),
                    ["FARPM_ANGLE_OF_ATTACK"] = FARAPI.ActiveVesselAoA(),
                    ["FARPM_SIDESLIP"] = FARAPI.ActiveVesselSideslip(),
                    ["FARPM_THRUST_SPECIFIC_FUEL_CONSUMPTION"] = FARAPI.ActiveVesselTSFC(),
                    ["FARPM_STALL_FRACTION"] = FARAPI.ActiveVesselStallFrac(),
                };
                foreach (KeyValuePair<string, object> pair in expected)
                    Assert.Equal(pair.Value, rpm.ProcessVariable(pair.Key), pair.Key);
                Assert.True(rpm.ProcessVariable("FARPM_NO_SUCH_VARIABLE") == null, "an unknown variable is null");
                Assert.True(rpm.ProcessVariable("FARPM_MACHNUMBER") is double mach && mach > 0.3, "the Mach number in flight");
            }
            finally
            {
                UnityEngine.Object.Destroy(rpm);
            }

            Ctrl.SetThrottle(0f);
        }

        private const string Rocket = "ComSat Lx Unclamped";   // stock, its launch clamp taken off (crafts/)

        /// <summary>The rocket on the pad, voxelized.</summary>
        private static IEnumerator RocketOnPad()
        {
            yield return Wait.Seconds(6);   // FAR voxelizes the vessel once it is on the pad
            Vessel v = FlightGlobals.ActiveVessel;
            yield return Wait.Until(() => FARAPI.VesselVoxelizationCompletedEver(v), 60, "the vessel voxelized", shot: false);
        }

        /// <summary>
        /// FARAPI's contract for other mods off the main thread and with nothing to work on. From a worker thread the
        /// altitude prediction is zero and a snapshot is refused (null); their warnings are not logged on the worker
        /// (Unity's log is not safe there) but on the main thread a frame later, the prediction's once only; FARVesselAero's
        /// own guards, for a mod calling it directly, do the same. A prediction with every slot taken (one made inside
        /// another on the main thread) is zero, with a warning once, and has its force again once they are free. A null vessel
        /// or snapshot gives zero, null or false without throwing. On a rocket (fins, but no flaps or spoilers) the flap setting is
        /// -1 and the spoilers off; a vessel FAR has no flight GUI for cannot have its airspeed display toggled; the gas
        /// at another altitude above the vessel is the body's there.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_ApiOffTheMainThreadAndWithoutAVessel(TestContext ctx)
        {
            yield return RocketOnPad();
            Vessel v = FlightGlobals.ActiveVessel;
            Vector3 vel = (Vector3)v.upAxis * 150;
            using var exceptions = new ExceptionLog();

            var logged = new List<(LogType Type, string Text, bool MainThread)>();
            void OnLog(string condition, string stackTrace, LogType type)
            {
                if (condition.StartsWith("[FAR") && (condition.Contains("off the main thread") || condition.Contains("API Error") ||
                                                     condition.Contains("no prediction slot free")))
                    lock (logged)
                        logged.Add((type, condition, Threading.MainThread.IsMainThread));
            }

            int Logged(string text)
            {
                lock (logged)
                    return logged.Count(l => l.Text.Contains(text));
            }

            Application.logMessageReceivedThreaded += OnLog;
            try
            {
                ctx.Step("from a worker thread");
                FARAPI.CalculateVesselAeroForces(v, out Vector3 onMain, out _, vel, 1000);
                ctx.Record("mainThreadForce", $"{onMain.magnitude:F2} kN");
                Assert.True(onMain.magnitude > 0, "the prediction on the main thread has a force");
                bool warnedBefore = (bool)Refl.GetStatic(typeof(FARAPI), "loggedOffMainThread");
                // FARVesselAero's own guards, for a mod calling it directly (FARAPI checks the thread before them)
                FARVesselAero aero = v.GetComponent<FARVesselAero>();
                bool directWarnedBefore = (bool)Refl.GetStatic(typeof(FARVesselAero), "loggedOffMainThread");
                bool directSnapshotWarnedBefore = (bool)Refl.GetStatic(typeof(FARVesselAero), "loggedSnapshotOffMainThread");
                var worker = System.Threading.Tasks.Task.Run(() =>
                {
                    FARAPI.CalculateVesselAeroForces(v, out Vector3 f1, out Vector3 t1, vel, 1000);
                    FARAPI.CalculateVesselAeroForces(v, out Vector3 f2, out Vector3 t2, vel, 1000);
                    FARVesselSnapshot s = FARAPI.CaptureVesselSnapshot(v);
                    aero.SimulateAeroProperties(out Vector3 f3, out Vector3 t3, vel, 1000);
                    FARVesselSnapshot s3 = aero.CaptureSnapshot();
                    return (f1, t1, f2, t2, s, f3, t3, s3);
                });
                var r = worker.Result;
                Assert.True(r.f1 == Vector3.zero && r.t1 == Vector3.zero && r.f2 == Vector3.zero && r.t2 == Vector3.zero,
                            $"the prediction off the main thread is zero ({r.f1}, {r.f2})");
                Assert.True(r.s == null, "no snapshot off the main thread");
                Assert.True(r.f3 == Vector3.zero && r.t3 == Vector3.zero, $"FARVesselAero's prediction off the main thread is zero ({r.f3})");
                Assert.True(r.s3 == null, "and FARVesselAero's snapshot null");
                Assert.Equal(0, Logged("off the main thread"), "nothing logged on the worker thread");

                yield return Wait.Frames(3);
                ctx.Record("offThreadWarnings", string.Join(" | ", logged.Select(l => $"{l.Type} main={l.MainThread}: {l.Text}")));
                Assert.Equal(warnedBefore ? 0 : 1, Logged("CalculateVesselAeroForces(At) called off the main thread"),
                             "the prediction's warning, once (if not already given this session)");
                Assert.Equal(directWarnedBefore ? 0 : 1, Logged("FARVesselAero.SimulateAeroProperties called off the main thread"),
                             "FARVesselAero's prediction warning, once a session");
                Assert.Equal(directSnapshotWarnedBefore ? 1 : 2, Logged("CaptureVesselSnapshot called off the main thread"),
                             "the snapshot's warning: FARAPI's every time, FARVesselAero's once a session");
                lock (logged)
                    Assert.True(logged.All(l => l.MainThread && l.Type == LogType.Warning), "the warnings logged on the main thread");

                ctx.Step("no vessel, no snapshot");
                FARAPI.CalculateVesselAeroForces(null, out Vector3 fn, out Vector3 tn, vel, 1000);
                Assert.True(fn == Vector3.zero && tn == Vector3.zero, "no force for no vessel");
                Assert.Equal(1, Logged("attempted to simulate aerodynamics of null vessel"), "the null vessel is reported");
                Assert.True(FARAPI.CaptureVesselSnapshot(null) == null, "no snapshot of no vessel");
                var where = new Vector3d(v.latitude, v.longitude, 1000);
                double ut = Planetarium.GetUniversalTime();
                Assert.False(FARAPI.CalculateSnapshotAeroForcesAt(null, out Vector3 fs, out _, vel, where, ut), "no prediction from no snapshot");
                Assert.False(FARAPI.CalculateSnapshotAeroForcesWithControlsAt(null, out Vector3 fc, out _, vel, where, ut, 1, 0, 0),
                             "no prediction with controls from no snapshot");
                Assert.True(fs == Vector3.zero && fc == Vector3.zero, "and zero force");
                Assert.False(FARAPI.VesselVoxelizationCompletedEver(null), "no voxelization of no vessel");

                ctx.Step("every prediction slot taken");
                // a prediction made from inside another on the main thread, with all the others' slots busy, gets none:
                // the main thread does not wait. Holding them all here is that, without another thread
                bool noSlotWarnedBefore = (bool)Refl.GetStatic(typeof(FARVesselAero), "loggedNoSlot");
                FARVesselSnapshot slotSnap = FARAPI.CaptureVesselSnapshot(v);
                Assert.NotNull(slotSnap, "a snapshot of the vessel");
                var slotWhere = new Vector3d(v.latitude, v.longitude, v.altitude);
                var held = new List<ferram4.SimThreadContext.PredictionCall>();
                try
                {
                    ferram4.SimThreadContext.PredictionCall c;
                    while ((c = ferram4.SimThreadContext.BeginPrediction(vel)) != null)
                        held.Add(c);
                    ctx.Record("slotsHeld", held.Count.ToString());
                    Assert.True(held.Count >= 2, $"the main thread's slot and the pool's ({held.Count})");
                    // a velocity not asked for before: a remembered result would be given without a slot
                    aero.SimulateAeroProperties(out Vector3 fBusy, out Vector3 tBusy, vel * 1.37f, 1000);
                    Assert.True(fBusy == Vector3.zero && tBusy == Vector3.zero, $"no slot, no prediction ({fBusy})");
                    bool snapBusy = FARAPI.CalculateSnapshotAeroForcesAt(slotSnap, out Vector3 fSnapBusy, out _, vel, slotWhere, Planetarium.GetUniversalTime());
                    Assert.True(!snapBusy && fSnapBusy == Vector3.zero, $"no slot, no snapshot prediction ({fSnapBusy})");
                }
                finally
                {
                    for (int i = held.Count - 1; i >= 0; i--)
                        ferram4.SimThreadContext.EndPrediction(held[i]);
                }

                aero.SimulateAeroProperties(out Vector3 fFree, out _, vel * 1.37f, 1000);
                Assert.True(fFree.magnitude > 0, "with the slots given back, the same prediction has a force");
                Assert.True(FARAPI.CalculateSnapshotAeroForcesAt(slotSnap, out Vector3 fSnapFree, out _, vel, slotWhere, Planetarium.GetUniversalTime()) &&
                            fSnapFree.magnitude > 0, $"and the snapshot's ({fSnapFree})");
                yield return Wait.Frames(2);
                Assert.Equal(noSlotWarnedBefore ? 0 : 1, Logged("no prediction slot free"), "the warning, once a session");
            }
            finally
            {
                Application.logMessageReceivedThreaded -= OnLog;
            }

            ctx.Step("a rocket's fins: no flaps, no spoilers");
            FARControllableSurface[] surfaces = v.parts.Select(p => p.FindModuleImplementing<FARControllableSurface>()).Where(s => s != null).ToArray();
            ctx.Record("controlSurfaces", surfaces.Length);
            Assert.False(surfaces.Any(s => s.isFlap || s.isSpoiler), "none of the rocket's control surfaces is a flap or a spoiler");
            Assert.Equal(-1, FARAPI.VesselFlapSetting(v), "no flap setting without flaps");
            Assert.False(FARAPI.VesselSpoilerSetting(v), "no spoilers out without spoilers");

            ctx.Step("the airspeed display of a vessel without FAR's flight GUI");
            FlightGUI gui = Far.FlightGui(v);
            bool shown = gui.airSpeedGUI.enabled;
            FlightGUI.vesselFlightGUI.Remove(v);
            try
            {
                Assert.False(FARAPI.ToggleAirspeedDisplay(true, v), "no airspeed display to switch");
            }
            finally
            {
                FlightGUI.vesselFlightGUI[v] = gui;
            }
            Assert.Equal(shown, gui.airSpeedGUI.enabled, "the vessel's display untouched");

            ctx.Step("the gas at another altitude");
            double ut2 = Planetarium.GetUniversalTime();
            GasProperties high = FARAtmosphere.GetGasProperties(v, 5000, ut2), here = FARAtmosphere.GetGasProperties(v, ut2);
            GasProperties body = FARAtmosphere.GetGasProperties(v.mainBody, new Vector3d(v.latitude, v.longitude, 5000), ut2);
            ctx.Record("gas5km", $"{high.Pressure:F0} Pa {high.Temperature:F1} K (here {here.Pressure:F0} Pa)");
            Assert.Equal(body.Pressure, high.Pressure, "the pressure at 5 km above the vessel's place");
            Assert.Equal(body.Temperature, high.Temperature, "the temperature at 5 km above the vessel's place");
            Assert.Near(v.mainBody.GetPressure(5000) * 1000, high.Pressure, 1e-3 * high.Pressure, "the body's pressure at 5 km");
            Assert.True(high.Pressure < 0.7 * here.Pressure, "lower than on the pad");
            exceptions.AssertNone("calling FARAPI off the main thread and with nulls");
        }

        /// <summary>
        /// A snapshot's atmosphere at its edges. Above the atmosphere there is no force (a trajectory mod predicting
        /// from orbit). Under a custom pressure function not set as thread-safe a worker thread's prediction is
        /// refused; under a thread-safe one that throws the prediction fails (false) and nothing escapes. The atmosphere a
        /// snapshot reproduces (AtmosphereSnapshot) is the body's with each way a body may describe it: a pressure and a
        /// temperature curve in metres (Kerbin's), normalized curves, and no curves (the lapse-rate formulas).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_SnapshotAtmosphereAtItsEdges(TestContext ctx)
        {
            yield return RocketOnPad();
            Vessel v = FlightGlobals.ActiveVessel;
            CelestialBody kerbin = v.mainBody;
            Vector3 vel = (Vector3)v.upAxis * 300;
            double ut = Planetarium.GetUniversalTime();
            Vector3d At(double alt) => new Vector3d(v.latitude, v.longitude, alt);
            // the lapse-rate formula runs to NaN above the altitude where its temperature would be negative
            bool Same(double game, double snapshot) =>
                double.IsNaN(game) ? double.IsNaN(snapshot) : Math.Abs(game - snapshot) <= 1e-5 * Math.Abs(game) + 1e-9;

            ctx.Step("above the atmosphere");
            FARVesselSnapshot snap = FARAPI.CaptureVesselSnapshot(v);
            Assert.NotNull(snap, "a snapshot of the rocket");
            Assert.True(snap.OffMainThread, "the default atmosphere: predictions on any thread");
            Assert.True(FARAPI.CalculateSnapshotAeroForcesAt(snap, out Vector3 low, out _, vel, At(2000), ut), "a prediction at 2 km");
            Assert.True(low.magnitude > 0, "a force at 2 km");
            bool ok = FARAPI.CalculateSnapshotAeroForcesAt(snap, out Vector3 space, out Vector3 spaceTorque, vel, At(kerbin.atmosphereDepth + 10000), ut);
            ctx.Record("aboveAtmosphere", $"returned {ok}, force {space}, torque {spaceTorque}");
            Assert.True(space == Vector3.zero && spaceTorque == Vector3.zero, "no force or torque above the atmosphere");

            try
            {
                ctx.Step("a custom pressure function, not thread-safe");
                FARAtmosphere.SetPressureFunction((b, lla, t) => b.GetPressure(lla.z) * 1000);
                FARVesselSnapshot custom = FARAPI.CaptureVesselSnapshot(v);
                Assert.False(custom.OffMainThread, "predictions on the main thread only");
                Assert.True(FARAPI.CalculateSnapshotAeroForcesAt(custom, out Vector3 main, out _, vel, At(2000), ut), "it predicts on the main thread");
                Assert.Near(0, (main - low).magnitude, 1e-3 * low.magnitude, "the same force as the default atmosphere's");
                var worker = System.Threading.Tasks.Task.Run(() =>
                {
                    bool r = FARAPI.CalculateSnapshotAeroForcesAt(custom, out Vector3 f, out _, vel, At(2000), ut);
                    return (r, f);
                }).Result;
                Assert.False(worker.r, "refused on a worker thread");
                Assert.True(worker.f == Vector3.zero, "with zero force");

                ctx.Step("a thread-safe custom pressure function that throws where the prediction asks");
                // it answers where the snapshot checks it at capture (the pad, half way up), not between 1 and 3 km
                FARAtmosphere.SetPressureFunctionThreadSafe((b, lla, t) => lla.z > 1000 && lla.z < 3000 ? throw new InvalidOperationException("test") : b.GetPressure(lla.z) * 1000);
                FARVesselSnapshot throwing = FARAPI.CaptureVesselSnapshot(v);
                Assert.True(throwing.OffMainThread, "a thread-safe function: predictions on any thread");
                Assert.False(FARAPI.CalculateSnapshotAeroForcesAt(throwing, out Vector3 tf, out _, vel, At(2000), ut), "the prediction fails");
                Assert.True(tf == Vector3.zero, "with zero force");
                var w2 = System.Threading.Tasks.Task.Run(() => FARAPI.CalculateSnapshotAeroForcesAt(throwing, out _, out _, vel, At(2000), ut));
                Assert.False(w2.Result, "and fails on a worker thread, without an exception");
            }
            finally
            {
                FARAtmosphere.SetPressureFunction(null);
            }

            ctx.Step("the reproduced atmosphere against the body's, each way a body describes it");
            bool usePressure = kerbin.atmosphereUsePressureCurve, pressureNormalized = kerbin.atmospherePressureCurveIsNormalized;
            bool useTemperature = kerbin.atmosphereUseTemperatureCurve, temperatureNormalized = kerbin.atmosphereTemperatureCurveIsNormalized;
            var cases = new[] { (true, false, "curves in metres"), (true, true, "normalized curves"), (false, false, "no curves") };
            try
            {
                foreach ((bool useCurve, bool normalized, string what) in cases)
                {
                    kerbin.atmosphereUsePressureCurve = kerbin.atmosphereUseTemperatureCurve = useCurve;
                    kerbin.atmospherePressureCurveIsNormalized = kerbin.atmosphereTemperatureCurveIsNormalized = normalized;
                    var atm = new AtmosphereSnapshot(kerbin);
                    var rows = new List<string>();
                    foreach (double alt in new[] { 0.0, 1500, 12000, 35000, 69000, kerbin.atmosphereDepth, kerbin.atmosphereDepth + 5000 })
                    {
                        double p = kerbin.GetPressure(alt), t = kerbin.GetTemperature(alt);
                        double ps = atm.GetPressureKPa(alt), ts = atm.GetBaseTemperature(alt);
                        rows.Add($"{alt / 1000:0.#} km {ps:G5}/{p:G5} kPa {ts:F2}/{t:F2} K");
                        Assert.True(Same(p, ps), $"{what}: the pressure at {alt} m ({ps} against {p} kPa)");
                        Assert.True(Same(t, ts), $"{what}: the temperature at {alt} m ({ts} against {t} K)");
                    }
                    ctx.Record(what, string.Join("; ", rows));
                }
            }
            finally
            {
                kerbin.atmosphereUsePressureCurve = usePressure;
                kerbin.atmospherePressureCurveIsNormalized = pressureNormalized;
                kerbin.atmosphereUseTemperatureCurve = useTemperature;
                kerbin.atmosphereTemperatureCurveIsNormalized = temperatureNormalized;
            }
        }

        /// <summary>
        /// The debug voxels with the difficulty setting "32 bit indices" (DebugVoxelMesh.Use32BitIndices, a player option
        /// for big craft): the vessel's voxel mesh made after it is switched on has 32-bit indices and all its voxels in
        /// one submesh; with 16-bit indices the same voxels are split into submeshes of at most 65535 vertices. Both
        /// show the same vertices and triangles.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Rocket, CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Pad_DebugVoxelsWith32BitIndices(TestContext ctx)
        {
            yield return RocketOnPad();
            Vessel v = FlightGlobals.ActiveVessel;
            FARVesselAero aero = v.GetComponent<FARVesselAero>();
            bool use32 = Geometry.DebugVoxelMesh.Use32BitIndices, debugInFlight = FARConfig.Voxelization.DebugInFlight;
            using var exceptions = new ExceptionLog();
            var meshes = new Dictionary<bool, (int Vertices, int Indices, int Submeshes, UnityEngine.Rendering.IndexFormat Format)>();
            try
            {
                // in flight FAR drops the voxel once made unless debugInFlight (what the debug voxel button needs)
                FARConfig.Voxelization.DebugInFlight = true;
                foreach (bool wide in new[] { true, false })
                {
                    ctx.Step(wide ? "32-bit indices" : "16-bit indices");
                    Geometry.DebugVoxelMesh.Use32BitIndices = wide;
                    object old = Refl.TryGet<object>(aero.VehicleAero, "_voxel");
                    aero.VesselUpdateEvent(v);
                    yield return Wait.Until(() => Refl.TryGet<object>(aero.VehicleAero, "_voxel") is { } nv && !ReferenceEquals(nv, old) &&
                                                  !aero.VehicleAero.Voxelizing, 60, "a new voxel", shot: false);
                    aero.VehicleAero.DebugVisualizeVoxels(v.transform.localToWorldMatrix);
                    var mesh = (Component)Refl.Get(Refl.Get(aero.VehicleAero, "_voxel"), "voxelMesh");
                    yield return Wait.Until(() => mesh.gameObject.activeSelf, 30, "the debug voxels shown", shot: false);
                    Mesh m = ((MeshFilter)mesh.GetComponent(typeof(MeshFilter))).sharedMesh;
                    int indices = Enumerable.Range(0, m.subMeshCount).Sum(i => m.GetIndices(i).Length);
                    int maxIndex = Enumerable.Range(0, m.subMeshCount).Max(i => m.GetIndices(i).DefaultIfEmpty(0).Max());
                    meshes[wide] = (m.vertexCount, indices, m.subMeshCount, m.indexFormat);
                    ctx.Record(wide ? "mesh32" : "mesh16", $"{m.vertexCount} vertices, {indices} indices in {m.subMeshCount} submeshes, {m.indexFormat}");
                    Assert.True(m.vertexCount > 0 && indices > 0, "voxels to show");
                    Assert.True(maxIndex < m.vertexCount, "every index names a vertex");
                    aero.VehicleAero.DebugVisualizeVoxels(v.transform.localToWorldMatrix);   // hidden again
                    yield return Wait.Frames(3);
                }
            }
            finally
            {
                Geometry.DebugVoxelMesh.Use32BitIndices = use32;
                FARConfig.Voxelization.DebugInFlight = debugInFlight;
            }

            var m32 = meshes[true];
            var m16 = meshes[false];
            Assert.Equal(UnityEngine.Rendering.IndexFormat.UInt32, m32.Format, "32-bit indices when switched on");
            Assert.Equal(1, m32.Submeshes, "one submesh with 32-bit indices");
            Assert.Equal(UnityEngine.Rendering.IndexFormat.UInt16, m16.Format, "16-bit indices when off");
            Assert.True(m16.Submeshes >= (m16.Vertices + 65534) / 65535, $"16-bit submeshes of at most 65535 vertices ({m16.Submeshes} for {m16.Vertices})");
            Assert.Equal(m32.Vertices, m16.Vertices, "the same vertices either way");
            Assert.Equal(m32.Indices, m16.Indices, "the same primitives either way");
            exceptions.AssertNone("showing the debug voxels");
        }

        /// <summary>
        /// FARAtmosphere's functions and a part's force modifier (FARAPI.SetPartAeroForceModifier) as another mod sets
        /// them: each reaches FAR's predictions and the force it applies, a throwing one falls back to the default, a
        /// non-thread-safe one keeps snapshots on the main thread, and clearing it puts the default back.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_AtmosphereAndForceHooksReachFar(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Cruise(4);
            yield return Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            CelestialBody body = v.mainBody;
            Vector3 vel = v.srf_velocity;
            double alt = v.altitude;
            Vector3 upNormal = Vector3.ProjectOnPlane(v.upAxis, vel).normalized;

            (double Drag, double Lift, Vector3 Force) Predict()
            {
                FARAPI.CalculateVesselAeroForces(v, out Vector3 f, out _, vel, alt);
                return (-Vector3.Dot(f, vel.normalized), Vector3.Dot(f, upNormal), f);
            }

            ctx.Step("the default functions");
            Assert.False(FARAtmosphere.IsCustom, "the default atmosphere at the start");
            GasProperties gas = FARAtmosphere.GetGasProperties(v);
            Assert.Near(v.staticPressurekPa * 1000, gas.Pressure, 0.01 * gas.Pressure, "FAR's pressure is the vessel's");
            Assert.Near(PhysicsGlobals.IdealGasConstant / body.atmosphereMolarMass, FARAtmosphere.GetGasConstant(v), 1e-9, "the body's gas constant");
            Assert.Equal(body.atmosphereAdiabaticIndex, FARAtmosphere.GetAdiabaticIndex(v), "the body's adiabatic index");
            Assert.Near(Math.Sqrt(gas.AdiabaticIndex * gas.GasConstant * gas.Temperature), gas.SpeedOfSound, 1e-9, "the speed of sound");
            var p0 = Predict();
            ctx.Record("default", $"drag {p0.Drag:F2} lift {p0.Lift:F2} kN");
            Assert.True(p0.Drag > 0 && p0.Lift > 0, "drag and lift at 4 deg");

            try
            {
                ctx.Step("twice the pressure: twice the density, the Mach number the same");
                Func<CelestialBody, Vector3d, double, double> doubled = (b, lla, ut) => 2 * b.GetPressure(lla.z) * 1000;
                FARAtmosphere.SetPressureFunction(doubled);
                Assert.True(FARAtmosphere.IsCustom, "a custom atmosphere");
                Assert.Near(2 * gas.Pressure, FARAtmosphere.GetPressure(v), 0.01 * gas.Pressure, "the custom pressure");
                var p2 = Predict();
                ctx.Record("doubledPressure", $"drag {p2.Drag:F2} lift {p2.Lift:F2} kN");
                Assert.Near(2, p2.Lift / p0.Lift, 0.1, "lift doubles with the density");
                // the skin friction coefficient falls as the Reynolds number rises with the density
                Assert.True(p2.Drag / p0.Drag > 1.6 && p2.Drag / p0.Drag < 2.05, $"drag nearly doubles ({p2.Drag / p0.Drag:F3}x)");
                FARVesselSnapshot snap = FARAPI.CaptureVesselSnapshot(v);
                Assert.False(snap.OffMainThread, "a snapshot under a non-thread-safe function predicts on the main thread only");
                Assert.True(FARAPI.CalculateSnapshotAeroForcesAt(snap, out Vector3 fs, out _, vel, new Vector3d(v.latitude, v.longitude, alt),
                                                                Planetarium.GetUniversalTime()), "the snapshot predicts on the main thread");
                Assert.Near(0, (fs - p2.Force).magnitude, 0.01 * p2.Force.magnitude, "the snapshot's prediction under the custom pressure");
                FARAtmosphere.SetPressureFunctionThreadSafe(doubled);
                Assert.True(FARAPI.CaptureVesselSnapshot(v).OffMainThread, "one under a thread-safe function predicts on any thread");

                FARAtmosphere.SetPressureFunction(null);
                Assert.False(FARAtmosphere.IsCustom, "back to the default atmosphere");
                var back = Predict();
                Assert.Near(0, (back.Force - p0.Force).magnitude, 1e-3 * p0.Force.magnitude, "the default prediction again");

                ctx.Step("a hotter atmosphere, and a lighter gas, of the same density and speed of sound");
                // hotter than the game's temperature at the point, which a prediction takes by default
                FARAtmosphere.SetTemperatureFunction((b, lla, ut) => 1.21 * FARAtmosphere.GameTemperature(b, lla, ut));
                var hot = Predict();
                FARAtmosphere.SetTemperatureFunctionThreadSafe(null);
                FARAtmosphere.SetGasConstantFunction((b, lla, ut) => 1.21 * PhysicsGlobals.IdealGasConstant / b.atmosphereMolarMass);
                Assert.Near(1.21 * gas.GasConstant, FARAtmosphere.GetGasConstant(v), 1e-6 * gas.GasConstant, "the custom gas constant");
                var light = Predict();
                FARAtmosphere.SetGasConstantFunctionThreadSafe(null);
                ctx.Record("hotAndLight", $"hot lift {hot.Lift:F2} kN, light gas lift {light.Lift:F2} kN");
                // density / 1.21 and Mach / 1.1: lift down by about a fifth
                Assert.True(hot.Lift / p0.Lift > 0.72 && hot.Lift / p0.Lift < 0.88, $"lift in the hot air ({hot.Lift / p0.Lift:F3}x)");
                // the two differ in the viscosity only (it follows the temperature)
                Assert.Near(hot.Lift, light.Lift, 0.03 * hot.Lift, "the lighter gas lifts like the hotter air");

                ctx.Step("a stiffer gas: the same density, a faster speed of sound");
                FARAtmosphere.SetAdiabaticIndexFunction((b, lla, ut) => 1.21 * b.atmosphereAdiabaticIndex);
                Assert.Near(1.1 * gas.SpeedOfSound, FARAtmosphere.GetGasProperties(v).SpeedOfSound, 0.002 * gas.SpeedOfSound,
                            "the speed of sound with the custom adiabatic index");
                var stiff = Predict();
                FARAtmosphere.SetAdiabaticIndexFunctionThreadSafe(null);
                ctx.Record("stiff", $"lift {stiff.Lift:F2} kN");
                Assert.True(stiff.Lift < p0.Lift && stiff.Lift > 0.9 * p0.Lift, $"a little less lift at the lower Mach number ({stiff.Lift / p0.Lift:F3}x)");

                ctx.Step("a function that throws: FAR falls back to the default");
                FARAtmosphere.SetTemperatureFunction((b, lla, ut) => throw new InvalidOperationException("test"));
                var thrown = Predict();
                FARAtmosphere.SetTemperatureFunction(null);
                Assert.Near(0, (thrown.Force - p0.Force).magnitude, 1e-3 * p0.Force.magnitude, "the default prediction under a throwing function");

                ctx.Step("a part force modifier: the body forces scale by its factor");
                Part[] parts = v.parts.Where(p => p.FindModuleImplementing<FARAeroPartModule>() != null).ToArray();

                Vector3 Scaled(float k)
                {
                    foreach (Part p in parts)
                        FARAPI.SetPartAeroForceModifier(p, (part, local) => local * k);
                    return Predict().Force;
                }

                Vector3 f0 = Scaled(0), f1 = Scaled(1), f2 = Scaled(2);
                Assert.True(FARAPI.GetPartAeroForceModifier(parts[0]) != null, "the modifier is the part's");
                Assert.False(FARAPI.CaptureVesselSnapshot(v).OffMainThread, "a snapshot with a modifier predicts on the main thread only");
                foreach (Part p in parts)
                    FARAPI.SetPartAeroForceModifier(p, null);
                Vector3 none = Predict().Force;
                ctx.Record("modifier", $"x0 {f0.magnitude:F2}, x1 {f1.magnitude:F2}, x2 {f2.magnitude:F2}, none {none.magnitude:F2} kN; " +
                                       $"x1 - none {(f1 - none).magnitude:F4} kN");
                Assert.True(!float.IsNaN(f0.x + f1.x + f2.x), "finite forces under the modifier");
                // the wings carry most of the lift: the body parts a kN or so
                Assert.True((f1 - f0).magnitude > 0.3, $"the body parts carry some of the force ({(f1 - f0).magnitude:F2} kN)");
                Assert.Near(0, ((f2 - f1) - (f1 - f0)).magnitude, 0.02 * (f1 - f0).magnitude + 1e-3, "the body force scales with the factor");
                // to float rounding: a section with a zero component (no torque, say) keeps its force
                Assert.Near(0, (f1 - none).magnitude, 1e-5 * none.magnitude, "a factor of 1 is no modifier");
                Assert.Near(0, (none - p0.Force).magnitude, 1e-5 * none.magnitude, "the prediction with the modifiers gone is the first");

                ctx.Step("a 20 m/s headwind on the flying vessel");
                var calm = new List<FarFlightProbe.Sample>();
                yield return FarFlightProbe.Hold(c, 25, 10, calm, predict: false);
                int calls = 0;
                Vector3 windDir = -v.srf_velocity.normalized;
                FARAtmosphere.SetWindFunction((b, part, pos) =>
                {
                    calls++;
                    return 20 * windDir;
                });
                var windy = new List<FarFlightProbe.Sample>();
                yield return FarFlightProbe.Hold(c, 25, 10, windy, predict: false);
                FARAtmosphere.SetWindFunction(null);
                double calmDrag = calm.Average(s => s.Flight[0]), windyDrag = windy.Average(s => s.Flight[0]);
                double speed = calm.Average(s => s.Airspeed), expected = Math.Pow((speed + 20) / speed, 2);
                ctx.Record("headwind", $"drag {calmDrag:F2} -> {windyDrag:F2} kN ({windyDrag / calmDrag:F3}x, dynamic pressure {expected:F3}x), {calls} calls");
                Assert.True(calls > 0, "FAR asked the wind function");
                Assert.True(windyDrag / calmDrag > 1 + 0.5 * (expected - 1) && windyDrag / calmDrag < 1 + 1.5 * (expected - 1),
                            $"the drag rises with the airspeed into the wind ({windyDrag / calmDrag:F3}x against {expected:F3}x)");
            }
            finally
            {
                FARAtmosphere.SetWindFunction(null);
                FARAtmosphere.SetPressureFunction(null);
                FARAtmosphere.SetTemperatureFunction(null);
                FARAtmosphere.SetAdiabaticIndexFunction(null);
                FARAtmosphere.SetGasConstantFunction(null);
                foreach (Part p in v.parts)
                    if (p.FindModuleImplementing<FARAeroPartModule>() != null)
                        FARAPI.SetPartAeroForceModifier(p, null);
            }
        }
    }
}

