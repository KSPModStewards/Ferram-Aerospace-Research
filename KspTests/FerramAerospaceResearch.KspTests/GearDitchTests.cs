using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.Settings;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The stock airliner (Stearwing A300: under-wing Goliaths, the small and medium gear) ditching: set down on the sea
    /// gliding (speed m/s, sink m/s), gear up or down (gear=up|down), at a touchdown attitude (entryPitch deg nose up, bank
    /// deg right wing down), the pilot holding that pitch and the wings level through the run (pilot=on), joints breakable
    /// (joints=breakable), hard (every joint unbreakable but the cheat off, so FAR's water tear-off still runs) or
    /// unbreakable, crash damage off. Each step: the deceleration, the water's force on the gear parts, the engines and
    /// the rest (drag along the flow, and each of FAR's terms), the gear's joint loads, the wet wings' force (FAR's wing
    /// model in the water) and the run over the water (ditch.csv). videoSec > 0 films it from abeam (ditch.mp4).
    /// </summary>
    public static class GearDitchTests
    {
        /// <summary>
        /// Ditching as a pilot does it: nose up 10 deg, flaps full, gear down, at 65 m/s, the stick holding the nose up and the
        /// wings level, its dry mass x0.6 (the stock Stearwing, 45 t, cannot hold itself up below ~80 m/s), the
        /// tail in first; joints hard (a wing caught in the water is Ditch_WingTornOff's). The fuselage alone should slow
        /// it, the wings kept off the water (NASA TM X-2445's heavy transport model: trimmed to near level, a smooth run of
        /// 3-6 fuselage lengths, about 3 g).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Stearwing A300", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Ditch_GearAndEnginesDrag(TestContext ctx) =>
            Ditch(ctx, new Dictionary<string, string>
            {
                ["gear"] = "down", ["joints"] = "hard", ["entryPitch"] = "10", ["pilot"] = "on", ["flaps"] = "3", ["massScale"] = "0.6",
                ["assertDitch"] = "true",
            });

        /// <summary>
        /// A wing caught in the water: the airliner set down level, banked 25 deg (its right wing's tip in first), gear up,
        /// no pilot, joints breakable. The wing planes on the water (PartHydrodynamics.PlaningFoilShare) and its load, past
        /// what its joint holds, tears it off (FARWingAerodynamicModel's water tear-off) instead of the airliner riding on it.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Stearwing A300", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Ditch_WingTornOff(TestContext ctx) =>
            Ditch(ctx, new Dictionary<string, string>
            {
                ["gear"] = "up", ["joints"] = "breakable", ["entryPitch"] = "0", ["bank"] = "25", ["pilot"] = "off", ["seconds"] = "6",
                ["assertWingTorn"] = "true",
            });

        private static IEnumerator Ditch(TestContext ctx, Dictionary<string, string> defaults)
        {
            string Par(string key) => ctx.Param(key) ?? (defaults.TryGetValue(key, out string d) ? d : null);
            double ParD(string key, double d) => Par(key) is string x ? double.Parse(x, CultureInfo.InvariantCulture) : d;
            bool gearDown = (Par("gear") ?? "down") == "down";
            bool unbreakable = Par("joints") == "unbreakable";
            // (joints=hard: every KSP joint unbreakable but the cheat off, so FAR's water tear-off still runs)
            bool hard = unbreakable || Par("joints") == "hard";
            double speed = double.Parse(Par("speed") ?? "65", CultureInfo.InvariantCulture);
            double sink = double.Parse(Par("sink") ?? "1.5", CultureInfo.InvariantCulture);
            double aoa = double.Parse(Par("aoa") ?? "6", CultureInfo.InvariantCulture);
            double seconds = double.Parse(Par("seconds") ?? "15", CultureInfo.InvariantCulture);

            yield return Wait.Seconds(5);
            // fuel: the share of its fuel left (default 0.1: a ditching near dry; full, 9 t of its 54, it flies only past
            // ~70 m/s at 10 deg, and out of the water at 50 m/s had no lift to hold its nose up with)
            double fuel = ParD("fuel", 0.1);
            foreach (Part p in FlightGlobals.ActiveVessel.parts)
                foreach (PartResource r in p.Resources)
                    if (r.resourceName == "LiquidFuel" || r.resourceName == "Oxidizer")
                        r.amount = r.maxAmount * fuel;
            // massScale: every part's dry mass times this, its balance kept (the stock Stearwing, 45 t near dry, needs
            // ~85 m/s to hold itself up at 7-8 deg even with its flaps down; an airliner ditches at its approach speed)
            double massScale = ParD("massScale", 1);
            if (massScale != 1)
                foreach (Part p in FlightGlobals.ActiveVessel.parts)
                {
                    // (stock sets a part's mass from its prefab's again: the change is a module's negative mass)
                    var mod = (DitchMassScale)p.AddModule(nameof(DitchMassScale), true);
                    mod.Scale = (float)massScale;
                    p.UpdateMass();
                }
            ctx.Record("fuel", $"{fuel:P0} of its fuel, dry mass x{massScale:G3}: {FlightGlobals.ActiveVessel.GetTotalMass():F1} t");
            ctx.Record("heaviest", string.Join(", ", FlightGlobals.ActiveVessel.parts.OrderByDescending(q => q.mass + q.GetResourceMass()).Take(10).Select(q => $"{q.partInfo.name} {q.mass + q.GetResourceMass():F2} t")));
            FarFlightProbe.Condition c = Far.At(0, 600, aoa, "glide", speedMs: speed);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;
            if (gearDown)
            {
                ExtendGear(v);
                yield return Wait.Seconds(8);
                yield return FarFlightProbe.Place(c);
            }
            ctx.Record("gear", $"{(gearDown ? "down" : "up")}: {Ctrl.GearStates()}");
            // flaps=N (0-3): the main wing's inboard horizontal control surfaces as flaps at setting N (an airliner ditches
            // flaps down: NASA TM X-2445's model 40 deg; without them the Stearwing needs 13-14 deg to hold itself up at
            // 65 m/s), the outboard ones and the tail's left to the stick
            int flapSetting = (int)ParD("flaps", 0);
            double flapTimeWas = FARAeroData.ControlSurfaceTimeConstantFlap;
            var flaps = new List<ferram4.FARControllableSurface>();
            if (flapSetting > 0)
            {
                Vector3 nose = v.ReferenceTransform.up, right = v.ReferenceTransform.right;
                double span = v.parts.Max(q => Math.Abs(Vector3.Dot(q.transform.position - v.CoM, right)));
                foreach (Part q in v.parts)
                    foreach (ferram4.FARControllableSurface cs in q.Modules.OfType<ferram4.FARControllableSurface>())
                    {
                        Vector3 off = q.transform.position - v.CoM;
                        bool flat = Math.Abs(Vector3.Dot(cs.part.partTransform.forward, v.ReferenceTransform.forward)) > 0.8;
                        if (flat && Math.Abs(Vector3.Dot(off, nose)) < 8 && Math.Abs(Vector3.Dot(off, right)) < 0.55 * span)
                        {
                            cs.isSpoiler = false;
                            cs.isFlap = true;
                            flaps.Add(cs);
                        }
                    }
                for (int i = 0; i < flapSetting; ++i)
                    FARAPI.VesselIncreaseFlapDeflection(v);
                FARAeroData.ControlSurfaceTimeConstantFlap = 0.5;
                yield return FarFlightProbe.Place(c);
                yield return FarFlightProbe.SettleSurfaces(c);
            }
            ctx.Record("flaps", $"setting {FARAPI.VesselFlapSetting(v)} on {flaps.Count}: " +
                                string.Join(", ", flaps.Select(f => $"{f.part.partInfo.name} {f.ControlDeflection:F0} deg")));

            bool crashWas = CheatOptions.NoCrashDamage, jointsWas = CheatOptions.UnbreakableJoints;
            double entryWas = PartHydrodynamics.EntryAddedMass, impactWas = PartHydrodynamics.ImpactPressureCoefficient;
            double breakWas = FARAeroPartModule.WaterBreakFactor, sepWas = PartHydrodynamics.SeparationDrop;
            bool entryDebugWas = VesselHydrodynamics.EntryDebug, failuresWas = FARDebugValues.allowStructuralFailures;
            int logStart = FlightLogger.eventLog.Count;
            using var exceptions = new ExceptionLog();
            PlaningVideoTests.Recorder rec = null;
            try
            {
                CheatOptions.NoCrashDamage = true;
                // (the model knobs: hd.<name> and the rest, WaterParams)
                WaterParams.Apply(ctx);
                CheatOptions.UnbreakableJoints = unbreakable;
                // (knock-outs: entry=0 no entry momentum, impact=0 no impact pressure; entryDebug=true logs each hull's entry)
                if (Par("breakFactor") != null) FARAeroPartModule.WaterBreakFactor = double.Parse(Par("breakFactor"), CultureInfo.InvariantCulture);
                if (Par("entry") != null) PartHydrodynamics.EntryAddedMass = double.Parse(Par("entry"), CultureInfo.InvariantCulture);
                if (Par("impact") != null) PartHydrodynamics.ImpactPressureCoefficient = double.Parse(Par("impact"), CultureInfo.InvariantCulture);
                // (sepDrop: PartHydrodynamics.SeparationDrop; 1 keeps the flow on a rising afterbody, its suction)
                if (Par("sepDrop") != null) PartHydrodynamics.SeparationDrop = double.Parse(Par("sepDrop"), CultureInfo.InvariantCulture);
                VesselHydrodynamics.EntryDebug = Par("entryDebug") == "true";
                // (aeroFailures=false: FAR's aero-structural failures off; they decouple parts whatever the joint cheat says)
                if (Par("aeroFailures") != null) FARDebugValues.allowStructuralFailures = Par("aeroFailures") == "true";
                if (!FARDebugValues.allowStructuralFailures)
                {
                    // (the limits are taken at load: lifted on each part, the wings' fields and the body's private ones)
                    var maxY = HarmonyLib.AccessTools.FieldRefAccess<FARAeroPartModule, double>("partForceMaxY");
                    var maxXZ = HarmonyLib.AccessTools.FieldRefAccess<FARAeroPartModule, double>("partForceMaxXZ");
                    foreach (Part p in v.parts)
                    {
                        foreach (ferram4.FARWingAerodynamicModel w in p.Modules.OfType<ferram4.FARWingAerodynamicModel>())
                            w.YmaxForce = w.XZmaxForce = double.MaxValue;
                        foreach (FARAeroPartModule apm in p.Modules.OfType<FARAeroPartModule>())
                            maxY(apm) = maxXZ(apm) = double.MaxValue;
                    }
                }
                ctx.Record("knobs", $"EntryAddedMass {PartHydrodynamics.EntryAddedMass}, ImpactPressureCoefficient {PartHydrodynamics.ImpactPressureCoefficient}, joints {(unbreakable ? "unbreakable" : hard ? "hard (cheat off)" : "breakable")}, WaterBreakFactor {FARAeroPartModule.WaterBreakFactor}, SeparationDrop {PartHydrodynamics.SeparationDrop}");
                bool IsGear(Part p) => p.Modules.OfType<ModuleWheelBase>().Any();
                bool IsEngine(Part p) => p.Modules.OfType<ModuleEngines>().Any();
                List<Part> gear = v.parts.Where(IsGear).ToList(), engines = v.parts.Where(IsEngine).ToList();
                ctx.Record("parts", $"{v.parts.Count} parts, {v.GetTotalMass():F1} t; gear " +
                                    string.Join(", ", gear.Select(p => $"{p.partInfo.name} {p.mass:F2} t breaks {p.breakingForce:G4} kN")) +
                                    "; engines " + string.Join(", ", engines.Select(p => $"{p.partInfo.name} {p.mass:F2} t breaks {p.breakingForce:G4} kN")));

                // videoSec > 0: filmed from abeam (camDistance m, camPitch deg) from the set-down, ditch.mp4 (WaterTests.StartVideo)
                double videoSec = ParD("videoSec", 0);
                if (videoSec > 0)
                {
                    yield return WaterTests.SideCamera(v, (float)ParD("camDistance", 90), (float)ParD("camPitch", 8));
                    rec = WaterTests.StartVideo(ctx, "GearDitchRecorder");
                }

                ctx.Step("ditch");
                // the touchdown attitude: nose up entryPitch deg (a pilot ditching holds it up, the tail in first), banked
                // bank deg (+: right wing down), not turning
                double entryPitch = ParD("entryPitch", double.NaN), bank = ParD("bank", 0);
                if (!double.IsNaN(entryPitch))
                    SetAttitude(v, entryPitch, bank);
                Physics.SyncTransforms();
                // (its lowest collider point clear=0.3 m above the sea, not its CoM 1 m: an airliner's belly would start
                // under the water, its whole run wetted at once in the first step)
                double clear = double.Parse(Par("clear") ?? "0.3", CultureInfo.InvariantCulture);
                double lowest = Lowest(v);
                v.SetPosition(v.vesselTransform.position - (Vector3d)v.upAxis * (lowest - clear));
                v.ChangeWorldVelocity((Vector3d)v.upAxis * (-v.verticalSpeed - sink));
                if (hard)
                    foreach (Part p in v.parts)
                        if (p.attachJoint != null && p.attachJoint.Joint != null)
                        {
                            p.attachJoint.Joint.breakForce = float.PositiveInfinity;
                            p.attachJoint.Joint.breakTorque = float.PositiveInfinity;
                        }
                // jointScale=k: every attach joint k times as strong (break force and torque); FAR's water tear-off of a
                // wet wing reads its joint's break force, so it scales too (wingJointScale=k: the wings' joints only)
                if (!hard && (Par("jointScale") != null || Par("wingJointScale") != null))
                    foreach (Part p in v.parts)
                        if (p.attachJoint != null && p.attachJoint.Joint != null)
                        {
                            bool wing = p.Modules.OfType<ferram4.FARWingAerodynamicModel>().Any();
                            float k = float.Parse((wing ? Par("wingJointScale") : null) ?? Par("jointScale") ?? "1", CultureInfo.InvariantCulture);
                            p.attachJoint.Joint.breakForce *= k;
                            p.attachJoint.Joint.breakTorque *= k;
                        }
                // crashTolerance=X: each part's (m/s); stock's buoyancy destroys a part striking the water faster,
                // crash cheat or not ("splashed down hard")
                if (Par("crashTolerance") is string ct)
                    foreach (Part p in v.parts)
                        p.crashTolerance = float.Parse(ct, CultureInfo.InvariantCulture);
                var fixedWait = new WaitForFixedUpdate();
                var all = new List<Part>(v.parts);
                // pilot=on: the stick holds the touchdown pitch and the wings level through the run (pitchSign, rollSign:
                // the stick's senses)
                bool pilot = Par("pilot") == "on" && !double.IsNaN(entryPitch);
                double pitchSign = ParD("pitchSign", 1), rollSign = ParD("rollSign", 1);
                if (pilot)
                    Ctrl.Attach();
                double pitchPrev = Pitch(v), rollPrev = Roll(v);
                // the wings the water tears off: each wing part's water stress (its load over its joint's break force,
                // smoothed: FAR's water tear-off) last seen on the airliner, and whether it was ever wet there
                var wingStress = new Dictionary<ferram4.FARWingAerodynamicModel, double>();
                var wingWet = new HashSet<ferram4.FARWingAerodynamicModel>();
                var wingPeakLoad = new Dictionary<ferram4.FARWingAerodynamicModel, (double kN, double stress)>();
                foreach (Part p in v.parts)
                    foreach (ferram4.FARWingAerodynamicModel w in p.Modules.OfType<ferram4.FARWingAerodynamicModel>())
                        wingStress[w] = 0;
                yield return fixedWait;
                ctx.Record("placed", $"{v.parts.Count} parts a step after placing, lowest point {Lowest(v):F2} m (was {lowest:F2}), CoM {v.altitude:F2} m, {-v.verticalSpeed:F2} m/s down");
                int waitSteps = 0;
                while (!v.parts.Any(p => p.submergedPortion > 0) && waitSteps < 1000)
                {
                    yield return fixedWait;
                    waitSteps++;
                }
                ctx.Record("contact", $"{v.parts.Count} parts at the first wet step ({waitSteps} steps after placing), lost: " +
                                      string.Join(", ", all.Where(p => p == null || p.vessel != v).Take(12).Select(p => p == null ? "(destroyed)" : p.partInfo.name)));
                double t0 = Planetarium.GetUniversalTime(), fuselage = 0;
                ctx.Record("splash", $"{v.horizontalSrfSpeed:F1} m/s horizontal, {-v.verticalSpeed:F1} m/s down, pitch {Pitch(v):F1} deg, {v.GetTotalMass():F1} t");
                // (its length along the nose: the run lengths of model ditchings are in fuselage lengths)
                {
                    Vector3 nose = v.ReferenceTransform.up;
                    float lo = float.MaxValue, hi = float.MinValue;
                    foreach (Part p in v.parts)
                        foreach (Collider col in p.GetComponentsInChildren<Collider>())
                            if (col.enabled && !col.isTrigger)
                            {
                                Bounds b = col.bounds;
                                for (int i = 0; i < 8; i++)
                                {
                                    float d = Vector3.Dot(b.center + Vector3.Scale(b.extents, new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1)), nose);
                                    lo = Math.Min(lo, d);
                                    hi = Math.Max(hi, d);
                                }
                            }
                    ctx.Record("length", $"{hi - lo:F1} m nose to tail (collider bounds)");
                    fuselage = hi - lo;
                }
                Vector3d start = v.CoMD;

                var csv = new StringBuilder("t,hs,vs,pitch,decel_g,parts,gearDrag,engDrag,restDrag,gearImpact,gearMomentum,gearCross,gearFriction," +
                                            "engImpact,engMomentum,engCross,engFriction,gearJointMax,gearJointBreak," +
                                            "upImpact,upMomentum,upCross,upFriction,upHydrostatic,upRadiation,upTotal,alt," +
                                            "wingDrag,wingUp,wingSide,wingMax,yawRate,rollRate,pitchRate,run,roll,stickPitch,stickRoll,bank,heading,wingRollM,wingYawM,wingPitchM,engRollM,engYawM,hullRollM,hullYawM\n");
                double wingPeak = 0;
                // the longest it was out of the water once in it (s), while still running (a skip: a second or more)
                double dry = 0, dryMost = 0;
                string wingPeakAt = "none";
                Vector3d vPrev = (Vector3d)v.rootPart.Rigidbody.velocity + Krakensbane.GetFrameVelocity();
                var tearPeak = new Dictionary<string, double>();   // each gear/engine part's smoothed water drag over its tear-off limit, at most
                double peakDecel = 0, peakGear = 0, peakEng = 0, peakRest = 0, peakJointShare = 0, sumDecel = 0, sumGear = 0, sumEng = 0, sumRest = 0;
                int n3 = 0, partsMin = v.parts.Count;
                string firstLost = null;
                var lostSeen = new HashSet<Part>();
                var lostLog = new List<string>();
                var engineRise = new Dictionary<Part, (int line, double hMax)>();
                // the plane's top (its reference transform's forward or back, whichever is up at the start: it starts
                // within 90 deg of level); the most bank, heading swing and water moments (kN m) seen
                float topSign = Vector3.Dot(v.ReferenceTransform.forward, (Vector3)v.upAxis) >= 0 ? 1 : -1;
                double heading = 0, bankMost = 0, headingMost = 0, wingRollMost = 0, wingYawMost = 0, engYawMost = 0, hullYawMost = 0;
                double crushMin = 1, crushRawMost = 0, crushedSec = 0, floodMost = 0, crushOver = 0, crushDv = 0;
                Vector3 headingPrev = Vector3.zero;
                int partsStart = v.parts.Count;
                while (Planetarium.GetUniversalTime() - t0 < Math.Max(seconds, videoSec) && (v.horizontalSrfSpeed > 1 || rec != null))
                {
                    yield return fixedWait;
                    double t = Planetarium.GetUniversalTime() - t0, dt = TimeWarp.fixedDeltaTime;
                    double pitchNow = Pitch(v), rollNow = Roll(v);
                    if (pilot)
                    {
                        Ctrl.Pitch = (float)(pitchSign * (PilotPitchGain * (entryPitch - pitchNow) - PilotPitchDamp * (pitchNow - pitchPrev) / dt));
                        Ctrl.Roll = (float)(rollSign * (PilotRollGain * rollNow + PilotRollDamp * (rollNow - rollPrev) / dt));
                    }
                    pitchPrev = pitchNow;
                    rollPrev = rollNow;
                    foreach (ferram4.FARWingAerodynamicModel w in wingStress.Keys.ToList())
                        if (w != null && w.part != null && w.part.vessel == v)
                        {
                            wingStress[w] = w.WaterStress;
                            if (w.part.submergedPortion > 0)
                            {
                                wingWet.Add(w);
                                var was = wingPeakLoad.TryGetValue(w, out var pk) ? pk : (0.0, 0.0);
                                wingPeakLoad[w] = (Math.Max(was.Item1, w.worldSpaceForce.magnitude), Math.Max(was.Item2, w.WaterStress));
                            }
                        }
                    // (the root's rigidbody: vessel.srf_velocity drops to 0 for a slowed floater, a false 25 g step)
                    Vector3d vel = (Vector3d)v.rootPart.Rigidbody.velocity + Krakensbane.GetFrameVelocity();
                    Vector3d flat = Vector3d.Exclude(v.upAxis, vPrev);
                    Vector3 along = flat.sqrMagnitude > 1e-6 ? (Vector3)flat.normalized : Vector3.zero;
                    double decel = -Vector3d.Dot(vel - vPrev, along) / dt / 9.81;
                    vPrev = vel;
                    // the wet wings' force (FAR's wing model, water density on their wetted share), on the plane: drag
                    // along the flow, up, to the right; the most on one part
                    double wgD = 0, wgU = 0, wgS = 0, wgMax = 0;
                    // (and the water's moments about the CoM: the wet wings', the engines', the rest's)
                    Vector3 comW = (Vector3)v.CoMD, mWing = Vector3.zero, mEng = Vector3.zero, mHull = Vector3.zero;
                    {
                        Vector3 upW = (Vector3)v.upAxis, sideW = Vector3.Cross(upW, along);
                        foreach (Part p in v.parts)
                            if (p.submergedPortion > 0)
                                foreach (ferram4.FARWingAerodynamicModel w in p.Modules.OfType<ferram4.FARWingAerodynamicModel>())
                                {
                                    Vector3 f = w.worldSpaceForce;
                                    mWing += Vector3.Cross((Vector3)w.AerodynamicCenter - comW, f);
                                    wgD -= Vector3.Dot(f, along);
                                    wgU += Vector3.Dot(f, upW);
                                    wgS += Vector3.Dot(f, sideW);
                                    if (f.magnitude > wgMax)
                                        wgMax = f.magnitude;
                                    if (f.magnitude > wingPeak)
                                    {
                                        wingPeak = f.magnitude;
                                        wingPeakAt = $"{p.partInfo.name} {f.magnitude:F0} kN at {t:F2} s, {p.submergedPortion:F2} under";
                                    }
                                }
                    }
                    Vector3 wl = v.ReferenceTransform.InverseTransformDirection(v.angularVelocity);
                    if (rec != null)
                        rec.Lines = new[]
                        {
                            $"Stearwing A300 ditching at {speed:F0} m/s, gear {(gearDown ? "down" : "up")}  -  FAR {(FARDebugValues.hydrodynamics ? "PartHydrodynamics" : "legacy water forces")}",
                            $"t {t,5:F2} s   speed {v.srfSpeed,6:F1} m/s   height {v.altitude,6:F1} m   decel {decel,5:F2} g",
                            $"pitch {Pitch(v),5:F0} deg   parts {v.parts.Count}/{partsStart}   wet {v.parts.Count(p => p.submergedPortion > 0)}"
                        };

                    double gD = 0, eD = 0, rD = 0;
                    Vector4 gT = Vector4.zero, eT = Vector4.zero;   // impact, momentum, crossflow, friction: drag along the flow (kN)
                    double jointMax = 0, jointBreak = 0;
                    Vector3 upAx = (Vector3)v.upAxis;
                    double uI = 0, uM = 0, uC = 0, uF = 0, uH = 0, uR = 0, uT = 0;
                    foreach (Part p in v.parts)
                    {
                        PartHydrodynamics hd = p.FindModuleImplementing<FARAeroPartModule>()?.Hydrodynamics;
                        if (hd == null || hd.Hull == null)
                            continue;
                        double d = -Vector3.Dot(hd.LastForce, along);
                        var terms = new Vector4(-Vector3.Dot(hd.LastImpactForce, along), -Vector3.Dot(hd.LastMomentumForce, along),
                                                -Vector3.Dot(hd.LastCrossflowForce, along), -Vector3.Dot(hd.LastFrictionForce, along));
                        uI += Vector3.Dot(hd.LastImpactForce, upAx); uM += Vector3.Dot(hd.LastMomentumForce, upAx);
                        uC += Vector3.Dot(hd.LastCrossflowForce, upAx); uF += Vector3.Dot(hd.LastFrictionForce, upAx);
                        uH += Vector3.Dot(hd.LastHydrostaticForce, upAx); uR += Vector3.Dot(hd.LastRadiationForce, upAx);
                        uT += Vector3.Dot(hd.LastForce, upAx);
                        if (IsGear(p)) { gD += d; gT += terms; }
                        else if (IsEngine(p)) { eD += d; eT += terms; }
                        else rD += d;
                        if (hd.LastForce != Vector3.zero)
                        {
                            Vector3 at = p.rb != null ? p.rb.worldCenterOfMass : p.transform.position;
                            Vector3 mo = Vector3.Cross(at - comW, hd.LastForce) + hd.LastTorque;
                            if (IsEngine(p)) mEng += mo; else mHull += mo;
                        }
                    }
                    // the moments about the CoM in the plane's axes (kN m): roll about the nose, yaw about its top, pitch
                    // about its right wing; and the full bank (+-180, right wing down +) and heading swing from the start
                    Vector3 axNose = v.ReferenceTransform.up, axRight = v.ReferenceTransform.right, axTop = v.ReferenceTransform.forward * topSign;
                    Vector3 Axes(Vector3 m) => new Vector3(Vector3.Dot(m, axNose), Vector3.Dot(m, axTop), Vector3.Dot(m, axRight));
                    Vector3 aW = Axes(mWing), aE = Axes(mEng), aH = Axes(mHull);
                    double bankNow = Math.Atan2(-Vector3.Dot(axRight, upAx), Vector3.Dot(axTop, upAx)) * Mathf.Rad2Deg;
                    Vector3 noseFlat = Vector3.ProjectOnPlane(axNose, upAx);
                    if (noseFlat.sqrMagnitude > 1e-4 && headingPrev.sqrMagnitude > 1e-4)
                        heading += Vector3.SignedAngle(headingPrev, noseFlat, upAx);
                    if (noseFlat.sqrMagnitude > 1e-4) headingPrev = noseFlat;
                    bankMost = Math.Max(bankMost, Math.Abs(bankNow));
                    headingMost = Math.Max(headingMost, Math.Abs(heading));
                    wingRollMost = Math.Max(wingRollMost, Math.Abs(aW.x)); wingYawMost = Math.Max(wingYawMost, Math.Abs(aW.y));
                    engYawMost = Math.Max(engYawMost, Math.Abs(aE.y)); hullYawMost = Math.Max(hullYawMost, Math.Abs(aH.y));
                    // (the crush cap, hd.CrushG: the least scale it put on the water's forces, the most raw load, how long it held)
                    if (v.GetComponent<FARVesselAero>()?.Hydrodynamics is VesselHydrodynamics vhd)
                    {
                        crushMin = Math.Min(crushMin, vhd.CrushScale);
                        crushRawMost = Math.Max(crushRawMost, vhd.LastCrushTotal / (v.totalMass * 9.80665));
                        crushOver = Math.Max(crushOver, vhd.LastCrushTotal / Math.Max(1e-6, vhd.LastCrushLimit)); crushDv = vhd.CrushDv;
                        if (vhd.CrushScale < 1) crushedSec += TimeWarp.fixedDeltaTime;
                        floodMost = Math.Max(floodMost, vhd.Flood);
                    }
                    foreach (Part p in v.parts.Where(p => IsGear(p) || IsEngine(p)))
                    {
                        FARAeroPartModule apm = p.FindModuleImplementing<FARAeroPartModule>();
                        if (apm == null) continue;
                        string key = $"{p.partInfo.name}#{p.flightID % 1000}";
                        double r = apm.WaterDragSmoothed / Math.Max(1e-6, p.breakingForce * FARAeroPartModule.WaterBreakFactor);
                        tearPeak[key] = Math.Max(tearPeak.TryGetValue(key, out double was) ? was : 0, r);
                    }
                    foreach (Part p in v.parts.Where(IsGear))
                    {
                        Joint j = p.attachJoint != null ? p.attachJoint.Joint : null;
                        if (j == null) continue;
                        double f = j.currentForce.magnitude * 0.001;   // kN
                        if (f > jointMax) { jointMax = f; jointBreak = p.breakingForce; }
                    }
                    // each part as it leaves: when, which side (the wing on the camera's side is "near"), its height;
                    // a lost engine's highest point after it
                    Vector3 camAt = FlightCamera.fetch != null ? FlightCamera.fetch.transform.position : Vector3.zero;
                    foreach (Part p in all)
                    {
                        if (p == null || p.vessel == v || lostSeen.Contains(p)) continue;
                        lostSeen.Add(p);
                        Vector3 off = p.transform.position - (Vector3)v.CoMD;
                        double right = Vector3.Dot(off, v.ReferenceTransform.right);
                        string side = Math.Abs(right) < 1.5 ? "centre" : (right > 0 ? "right" : "left") +
                                      (rec != null ? (Vector3.Dot(off, camAt - (Vector3)v.CoMD) > 0 ? " near" : " far") : "");
                        double h = FlightGlobals.getAltitudeAtPos(p.transform.position);
                        lostLog.Add($"{t:F2} s {p.partInfo.name} {side} {h:F1} m");
                        if (p.FindModuleImplementing<ModuleEngines>() != null) engineRise[p] = (lostLog.Count - 1, h);
                    }
                    foreach (Part p in engineRise.Keys.ToList())
                        if (p != null)
                            engineRise[p] = (engineRise[p].Item1, Math.Max(engineRise[p].Item2, FlightGlobals.getAltitudeAtPos(p.transform.position)));
                    if (v.parts.Count < partsMin)
                    {
                        partsMin = v.parts.Count;
                        firstLost ??= $"{t:F2} s: " + string.Join(", ", all.Where(p => p == null || p.vessel != v).Take(10)
                                                                         .Select(p => p == null ? "(destroyed)" : p.partInfo.name)) + "...";
                    }
                    peakDecel = Math.Max(peakDecel, decel);
                    if (v.horizontalSrfSpeed > 5)
                    {
                        dry = v.parts.Any(q => q.submergedPortion > 0) ? 0 : dry + dt;
                        dryMost = Math.Max(dryMost, dry);
                    }
                    peakGear = Math.Max(peakGear, gD);
                    peakEng = Math.Max(peakEng, eD);
                    peakRest = Math.Max(peakRest, rD);
                    if (jointBreak > 0) peakJointShare = Math.Max(peakJointShare, jointMax / jointBreak);
                    if (t < 3)
                    {
                        sumDecel += decel; sumGear += gD; sumEng += eD; sumRest += rD; n3++;
                    }
                    csv.AppendLine(string.Join(",", new[] { t, v.horizontalSrfSpeed, v.verticalSpeed, Pitch(v), decel, v.parts.Count, gD, eD, rD,
                                                            gT.x, gT.y, gT.z, gT.w, eT.x, eT.y, eT.z, eT.w, jointMax, jointBreak,
                                                            uI, uM, uC, uF, uH, uR, uT, v.altitude,
                                                            wgD, wgU, wgS, wgMax, wl.z * Mathf.Rad2Deg, wl.y * Mathf.Rad2Deg, wl.x * Mathf.Rad2Deg,
                                                            Vector3d.Exclude(v.upAxis, v.CoMD - start).magnitude, rollNow,
                                                            Ctrl.Pitch ?? 0, Ctrl.Roll ?? 0, bankNow, heading, aW.x, aW.y, aW.z, aE.x, aE.y, aH.x, aH.y }.Select(F)));
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "ditch.csv"), csv.ToString());
                ctx.Record("run", $"{Vector3d.Exclude(v.upAxis, v.CoMD - start).magnitude:F0} m over the water; the most on a wet wing {wingPeakAt}");
                int m = Math.Max(1, n3);
                ctx.Record("decel", $"peak {peakDecel:F2} g, first 3 s mean {sumDecel / m:F2} g; {v.horizontalSrfSpeed:F1} m/s after {Planetarium.GetUniversalTime() - t0:F1} s");
                ctx.Record("waterDrag", $"first 3 s mean gear {sumGear / m:F0} kN, engines {sumEng / m:F0} kN, rest {sumRest / m:F0} kN; " +
                                        $"peaks gear {peakGear:F0}, engines {peakEng:F0}, rest {peakRest:F0} kN");
                ctx.Record("tearOff", "smoothed water drag over the tear-off limit, at most: " + string.Join(", ", tearPeak.Select(kv => $"{kv.Key} {kv.Value:F2}")));
                ctx.Record("gearJoints", $"highest joint load {peakJointShare:P0} of its breaking force");
                ctx.Record("partsLost", $"{partsStart - partsMin} (first at {firstLost ?? "none"}); pitch at the end {Pitch(v):F1} deg");
                foreach (var kv in engineRise)
                    lostLog[kv.Value.line] += $" (highest after {kv.Value.hMax:F1} m)";
                ctx.Record("breakup", string.Join("; ", lostLog));
                ctx.Record("attitude", $"most bank {bankMost:F0} deg, heading swing {headingMost:F0} deg" +
                                       (bankMost > 120 || headingMost > 90 ? " (over / cartwheel)" : ""));
                ctx.Record("crush", $"cap {PartHydrodynamics.CrushG:G3} g: least scale {crushMin:F2}, most joint load {crushRawMost:F1} g ({crushOver:F2} of what they hold), capped {crushedSec:F2} s, taken off {crushDv:F1} m/s, flooded {floodMost:P0} (most {PartHydrodynamics.FloodMax:P0})");
                ctx.Record("moments", $"most about the CoM: wet wings roll {wingRollMost:F0}, yaw {wingYawMost:F0} kN m; " +
                                      $"engines yaw {engYawMost:F0}; rest of the hull yaw {hullYawMost:F0} kN m");
                List<string> events = FlightLogger.eventLog.Skip(logStart).ToList();
                ctx.Record("flightLog", $"{events.Count} events (aero failures on: {FARDebugValues.allowStructuralFailures}): " + string.Join(" | ", events.Take(15)));
                // the wings the water tore off: a wing part off the airliner that was wet on it, by FAR's water tear-off
                // (its water stress near its break force) or its joint breaking (a hull plate: the water's force on its own
                // part)
                List<string> torn = wingStress.Where(kv => wingWet.Contains(kv.Key))
                                              .Where(kv => kv.Key == null || kv.Key.part == null || kv.Key.part.vessel != v)
                                              .Select(kv => $"{(kv.Key != null && kv.Key.part != null ? kv.Key.part.partInfo.name : "(destroyed)")} " +
                                                            (kv.Value > 0.8 ? $"torn off ({kv.Value:F2})" : "joint")).ToList();
                ctx.Record("wingsTorn", $"{torn.Count}: " + string.Join(", ", torn));
                ctx.Record("wetWings", string.Join(", ", wingPeakLoad.OrderByDescending(kv => kv.Value.kN).Take(8)
                                                      .Select(kv => $"{(kv.Key != null && kv.Key.part != null ? kv.Key.part.partInfo.name : "(destroyed)")} " +
                                                                    $"{kv.Value.kN:F0} kN, stress {kv.Value.stress:F2}")));
                ctx.Record("lengths", $"run {Vector3d.Exclude(v.upAxis, v.CoMD - start).magnitude / Math.Max(1, fuselage):F1} fuselage lengths");
                double runLengths = Vector3d.Exclude(v.upAxis, v.CoMD - start).magnitude / Math.Max(1, fuselage);
                ctx.Record("ride", $"longest out of the water {dryMost:F2} s; peak {peakDecel:F2} g; most on a wet wing {wingPeak:F0} kN " +
                                   $"({wingPeak / (v.GetTotalMass() * 9.81):F2} of its weight)");
                if (Par("assertDitch") == "true")
                {
                    // NASA TM X-2445's heavy transport model, calm water, 7-12 deg: trimmed to near level, a smooth run of
                    // 3-6 fuselage lengths, about 3 g (4-7 g in its rougher runs)
                    Assert.True(runLengths > 3 && runLengths < 7, $"a run of 3-7 fuselage lengths ({runLengths:F1})");
                    Assert.True(dryMost < 0.5, $"it runs on the water, no skips ({dryMost:F2} s out of it at most)");
                    Assert.True(peakDecel < 6, $"the deceleration within an airliner ditching's ({peakDecel:F2} g)");
                    Assert.True(wingPeak < 0.5 * v.GetTotalMass() * 9.81, $"its wings kept off the water ({wingPeak:F0} kN at most on one)");
                }
                if (Par("assertWingTorn") == "true")
                    Assert.True(torn.Count > 0, "the water tears off the wing caught in it");
                exceptions.AssertNone("ditching");
            }
            finally
            {
                WaterTests.StopVideo(rec);
                if (Ctrl.Attached != null)
                    Ctrl.Detach();
                CheatOptions.NoCrashDamage = crashWas;
                WaterParams.Restore();
                CheatOptions.UnbreakableJoints = jointsWas;
                PartHydrodynamics.EntryAddedMass = entryWas;
                FARAeroPartModule.WaterBreakFactor = breakWas;
                PartHydrodynamics.SeparationDrop = sepWas;
                PartHydrodynamics.ImpactPressureCoefficient = impactWas;
                VesselHydrodynamics.EntryDebug = entryDebugWas;
                FARDebugValues.allowStructuralFailures = failuresWas;
                FARAeroData.ControlSurfaceTimeConstantFlap = flapTimeWas;
                foreach (ferram4.FARControllableSurface cs in flaps)
                    if (cs != null)
                        cs.isFlap = false;
            }
        }

        private static void ExtendGear(Vessel v)
        {
            var param = new KSPActionParam(KSPActionGroup.Gear, KSPActionType.Activate);
            foreach (Part p in v.parts)
                foreach (PartModule pm in p.Modules)
                    foreach (BaseAction a in pm.Actions)
                        if ((a.actionGroup & KSPActionGroup.Gear) != 0 && a.active)
                            a.Invoke(param);
            v.ActionGroups.SetGroup(KSPActionGroup.Gear, true);
        }

        // the vessel's lowest point above sea level (m): each collider's surface point nearest a point far below it (its
        // lowest, near enough: a box's, a ball's, a convex mesh's); a concave mesh, which Unity cannot do that for, by its
        // world bounds' lowest corner (lower than its surface: a tilted part's box reaches well below it)
        internal static double Lowest(Vessel v)
        {
            double low = double.MaxValue;
            Vector3 up = (Vector3)v.upAxis;
            foreach (Part p in v.parts)
                foreach (Collider col in p.GetComponentsInChildren<Collider>())
                {
                    if (!col.enabled || col.isTrigger) continue;
                    Bounds b = col.bounds;
                    if (col is BoxCollider || col is SphereCollider || col is CapsuleCollider || col is MeshCollider mc && mc.convex)
                    {
                        Vector3 q = col.ClosestPoint(b.center - up * 1000f);
                        low = Math.Min(low, FlightGlobals.getAltitudeAtPos((Vector3d)q, v.mainBody));
                        continue;
                    }
                    for (int i = 0; i < 8; i++)
                    {
                        Vector3 corner = b.center + Vector3.Scale(b.extents, new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
                        low = Math.Min(low, FlightGlobals.getAltitudeAtPos((Vector3d)corner, v.mainBody));
                    }
                }
            return low;
        }

        private static double Pitch(Vessel v)
        {
            Vector3 nose = v.ReferenceTransform.up;
            return 90 - Vector3.Angle(nose, (Vector3)v.upAxis);
        }

        private static double Roll(Vessel v) => Math.Asin(Mathf.Clamp(Vector3.Dot(v.ReferenceTransform.right, v.upAxis), -1, 1)) * Mathf.Rad2Deg;

        // the pilot's stick: per degree off the pitch held and per deg/s of pitch rate; per degree of bank and deg/s of roll
        private const double PilotPitchGain = 0.15, PilotPitchDamp = 0.05, PilotRollGain = 0.1, PilotRollDamp = 0.03;

        // the vessel turned to pitchDeg nose up and bankDeg right wing down, its heading kept, its spin stopped
        internal static void SetAttitude(Vessel v, double pitchDeg, double bankDeg)
        {
            Vector3 up = v.upAxis, nose = v.ReferenceTransform.up, right = v.ReferenceTransform.right;
            Vector3 h = Vector3.ProjectOnPlane(nose, up).normalized;
            Vector3 rh = Vector3.ProjectOnPlane(Vector3.ProjectOnPlane(right, up), h).normalized;
            float p = (float)(pitchDeg * Mathf.Deg2Rad), b = (float)(bankDeg * Mathf.Deg2Rad);
            Vector3 nose1 = Mathf.Cos(p) * h + Mathf.Sin(p) * up;
            Vector3 d = Vector3.Cross(nose1, rh).normalized;
            if (Vector3.Dot(d, up) < 0)
                d = -d;
            Vector3 right1 = Mathf.Cos(b) * rh - Mathf.Sin(b) * d;
            Quaternion turn = Quaternion.LookRotation(nose1, Vector3.Cross(nose1, right1)) *
                              Quaternion.Inverse(Quaternion.LookRotation(nose, Vector3.Cross(nose, right)));
            v.SetRotation(turn * v.vesselTransform.rotation);
            foreach (Part part in v.parts)
                if (part.rb != null)
                    part.rb.angularVelocity = Vector3.zero;
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);
    }

    /// <summary>A part's dry mass times Scale (GearDitchTests' massScale): a negative module mass, stock's own way to change it.</summary>
    public class DitchMassScale : PartModule, IPartMassModifier
    {
        public float Scale = 1;
        public float GetModuleMass(float defaultMass, ModifierStagingSituation sit) => -defaultMass * (1 - Scale);
        public ModifierChangeWhen GetModuleMassChangeWhen() => ModifierChangeWhen.FIXED;
    }
}
