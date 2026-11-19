using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.FARPartGeometry.GeometryModification;
using FerramAerospaceResearch.RealChuteLite;
using KSP.Localization;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR in the editor as a craft is changed: a wing panel taken off and put back, a wing's mass slider, the flap,
    /// spoiler and pitch-with-angle-of-attack settings of the control surfaces in the static analysis, the intakes of
    /// engine nacelles, the chute info window in the hangar, and TweakScale's rescale event.
    /// </summary>
    public static class EditorPartChangeTests
    {
        private const string Mallard = "Mallard";               // stock SPH: two nacelles with circular intakes on them
        private const string Vtol = "Rocket-power VTOL";        // stock SPH: two radial chutes in mirror symmetry

        private static readonly MethodInfo DetachPart = typeof(EditorLogic).GetMethod("detachPart", BindingFlags.Instance | BindingFlags.NonPublic);
        private static readonly MethodInfo AttachPart = typeof(EditorLogic).GetMethod("attachPart", BindingFlags.Instance | BindingFlags.NonPublic);

        /// <summary>The instant sim at one condition with the flap setting and spoilers given (as Far.Instant otherwise).</summary>
        private static InstantConditionSimOutput Instant(double alphaDeg, double mach, int flaps = 0, bool spoilers = false)
        {
            var input = new InstantConditionSimInput(alphaDeg, 0, 0, 0, 0, 0, mach, 0, flaps, spoilers);
            Far.EditorSim.GetClCdCmSteady(input, out InstantConditionSimOutput output, true, true);
            return output;
        }

        /// <summary>The wing area the editor's sim takes its coefficients over (InstantConditionSim.Counts' wings).</summary>
        private static double RefArea() =>
            EditorLogic.SortedShipList.Select(p => p.FindModuleImplementing<FARWingAerodynamicModel>())
                       .Where(w => w != null && InstantConditionSim.Counts(w)).Sum(w => w.S);

        /// <summary>A part (no children) taken off the craft as the editor does when it is picked up, and the editor told.</summary>
        private static void Detach(Part p)
        {
            Assert.NotNull(DetachPart, "EditorLogic.detachPart");
            DetachPart.Invoke(EditorLogic.fetch, new object[] { p });
            // the part's own detach callback, as the editor's pick-up fires it (detachPart alone does not)
            p.OnEditorDetach?.Invoke();
            GameEvents.onEditorPartEvent.Fire(ConstructionEventType.PartDetached, p);
            GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
        }

        /// <summary>The part put back on its parent by its surface attachment node where it was, as the editor attaches a dropped part.</summary>
        private static void AttachSurface(Part p, Part parent)
        {
            Assert.NotNull(AttachPart, "EditorLogic.attachPart");
            Type attachmentType = AttachPart.GetParameters()[1].ParameterType;
            object attach = Activator.CreateInstance(attachmentType, true);
            void Put(string name, object value) => attachmentType.GetField(name).SetValue(attach, value);
            Put("possible", true);
            Put("mode", AttachModes.SRF_ATTACH);
            Put("potentialParent", parent);
            Put("caller", p);
            Put("callerPartNode", p.srfAttachNode);
            Put("otherPartNode", null);
            Put("position", p.transform.position);
            Put("rotation", p.transform.rotation);
            AttachPart.Invoke(EditorLogic.fetch, new[] { p, attach });
            p.OnEditorAttach?.Invoke();
            GameEvents.onEditorPartEvent.Fire(ConstructionEventType.PartAttached, p);
            GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
        }

        /// <summary>
        /// The delta wing an aileron of the Aeris 4A sits on, its mass slider at 1.5: its wing mass half as much again.
        /// The aileron (a FAR control surface, a wing to FAR) taken off the delta wing: out of the voxel model (less
        /// volume), out of the reference area and out of the lift (Cl times the area, at 4 deg), and the delta wing, which
        /// carried a third of the aileron's area in its mass, lighter on the next frame. Put back where it was: in the
        /// part tree and the reference area again, and the delta wing's mass back. (The craft is loaded again after,
        /// for the tests that follow in the editor.)
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Editor_DetachingAWingTakesItOutOfTheModel(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            yield return Far.Settled("the Aeris 4A");
            ShipConstruct ship = EditorLogic.fetch.ship;
            Part aileron = ship.parts.First(p => p.partInfo.name == "StandardCtrlSrf");
            Part parent = aileron.parent;
            var wing = parent.FindModuleImplementing<FARWingAerodynamicModel>();
            Assert.NotNull(wing, $"the aileron's parent ({parent.partInfo.name}) is a FAR wing");
            Assert.True(aileron.children.Count == 0 && aileron.srfAttachNode?.attachedPart == parent, "the aileron surface-attached to its wing, with nothing on it");
            using var log = new ExceptionLog();
            double vol = Far.Volume(Far.EditorVehicle), area = RefArea(), lift = Far.Instant(4, 0.5).Cl * area;
            float mass = wing.curWingMass;
            int parts = ship.parts.Count;
            double aileronArea = aileron.FindModuleImplementing<FARWingAerodynamicModel>().S;
            ctx.Record("whole", $"{parts} parts; volume {vol:F3} m3, wing area {area:F3} m2, Cl x area at 4 deg {lift:F4} m2, " +
                                $"{parent.partInfo.name} wing mass {mass:F4} t");
            ctx.Step("the delta wing's mass slider");
            float multiplier = wing.massMultiplier;
            try
            {
                wing.massMultiplier = multiplier * 1.5f;
                yield return Wait.Until(() => Math.Abs(wing.curWingMass - 1.5f * mass) < 1e-3 * mass, 10, "the wing mass to follow its slider", shot: false,
                                        detail: () => $"{wing.curWingMass:F4} t, from {mass:F4}");
            }
            finally
            {
                wing.massMultiplier = multiplier;
            }

            yield return Wait.Until(() => Math.Abs(wing.curWingMass - mass) < 1e-4, 10, "the wing mass back with its slider", shot: false);

            try
            {
                ctx.Step("the aileron taken off");
                Detach(aileron);
                Assert.Equal(parts - 1, ship.parts.Count, "the craft's parts without the aileron");
                yield return Wait.Until(() => wing.curWingMass < mass - 1e-4, 10, "the delta wing's mass to drop", shot: false,
                                        detail: () => $"{wing.curWingMass:F4} t");
                float massOff = wing.curWingMass;
                yield return Far.Revoxelize("the Aeris without an aileron");
                yield return Far.Settled("the Aeris without an aileron");
                double volOff = Far.Volume(Far.EditorVehicle), areaOff = RefArea(), liftOff = Far.Instant(4, 0.5).Cl * areaOff;
                ctx.Record("detached", $"volume {volOff:F3} m3, wing area {areaOff:F3} m2, Cl x area at 4 deg {liftOff:F4} m2, wing mass {massOff:F4} t");
                Assert.True(volOff < vol, $"less volume without the aileron ({volOff:F3} against {vol:F3} m3)");
                Assert.Near(area - aileronArea, areaOff, 1e-3 * area, "the wing area without the aileron's (m2)");
                Assert.True(liftOff < lift - 1e-3, $"less lift without the aileron ({liftOff:F4} against {lift:F4} m2)");

                ctx.Step("the aileron put back");
                AttachSurface(aileron, parent);
                Assert.Equal(parts, ship.parts.Count, "the craft's parts with the aileron back");
                yield return Wait.Until(() => Math.Abs(wing.curWingMass - mass) < 1e-4, 10, "the delta wing's mass back", shot: false,
                                        detail: () => $"{wing.curWingMass:F4} t");
                Assert.True(EditorLogic.SortedShipList.Contains(aileron), "the aileron in the craft's part tree again");
                Assert.Near(area, RefArea(), 1e-3 * area, "the wing area with the aileron back (m2)");
                // (the voxel model is not checked here: put back by attachPart alone, not picked up and dropped as the
                // editor does, the aileron is not in FAR's next voxelization)
                var geo = aileron.FindModuleImplementing<GeometryPartModule>();
                ctx.Record("reattached", $"geometry ready {geo?.Ready}, {geo?.meshDataList?.Count} meshes, layer {aileron.gameObject.layer}");
                log.AssertNone("taking an aileron off and putting it back");
            }
            finally
            {
                // the craft as it was for the tests after (the editor keeps it), whatever happened to it here: loaded again
                EditorLogic.LoadShipFromFile(ctx.CraftPath(Far.Plane, "SPH"));
            }

        }

        /// <summary>
        /// TweakScale's rescale event (OnPartScaleChanged, see TweakScaleEvent) sent to the Aeris 4A's delta wing, in an
        /// instance without TweakScale: a scale missing, 0, negative, infinite, not a number or not a number type leaves
        /// FAR's wing model as it was, with a warning from FAR for each (the wing model's; the geometry, which does not
        /// use the scale, rebuilds) and no exception. A
        /// scale of 2 (as a float, and as a double) doubles the wing model's span and mean chord and rebuilds its geometry;
        /// 1 puts them back. A rescale that throws is logged by FAR and goes no further: part.SendEvent has no try/catch
        /// of its own, so it would end TweakScale's rescale of the part and every other module's handling of the event.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Editor_TweakScaleEventTakesOnlyAUsableScale(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            yield return Far.Settled("the Aeris 4A");
            Part part = EditorLogic.fetch.ship.parts.First(p => p.partInfo.name == "deltaWing");
            var wing = part.FindModuleImplementing<FARWingAerodynamicModel>();
            var geometry = part.FindModuleImplementing<GeometryPartModule>();
            Assert.NotNull(wing, "the delta wing is a FAR wing");
            Assert.NotNull(geometry, "the delta wing has FAR's geometry");
            Assert.False(part.Modules.Contains("TweakScale"), "no TweakScale on the delta wing in this instance");
            double b2 = wing.b_2, mac = wing.MAC;
            Assert.Near(b2, wing.b_2_actual, 1e-6, "the unscaled wing's span is the prefab's");

            var warnings = new List<string>();
            var failures = new List<string>();
            void OnLog(string condition, string stackTrace, LogType type)
            {
                if (type == LogType.Warning && condition.Contains("TweakScale's rescale event"))
                    warnings.Add(condition);
                if (type == LogType.Exception && condition.Contains("rescale to"))
                    failures.Add(condition);
            }

            void Send(Action<BaseEventDetails> set)
            {
                var data = new BaseEventDetails(BaseEventDetails.Sender.USER);
                set(data);
                // to the part alone, as TweakScale sends it (without a depth it goes to every part below too)
                part.SendEvent("OnPartScaleChanged", data, 0);
            }

            Application.logMessageReceived += OnLog;
            try
            {
                using (var log = new ExceptionLog())
                {
                    ctx.Step("Events with no usable scale");
                    var bad = new (string what, Action<BaseEventDetails> set)[]
                    {
                        ("no scale", d => d.Set("factorType", 1)),
                        ("0", d => d.Set(TweakScaleEvent.FactorKey, 0f)),
                        ("-2", d => d.Set(TweakScaleEvent.FactorKey, -2f)),
                        ("infinity", d => d.Set(TweakScaleEvent.FactorKey, float.PositiveInfinity)),
                        ("NaN", d => d.Set(TweakScaleEvent.FactorKey, float.NaN)),
                        ("a string", d => d.Set(TweakScaleEvent.FactorKey, "2")),
                    };
                    foreach ((string what, Action<BaseEventDetails> set) in bad)
                    {
                        int before = warnings.Count;
                        Send(set);
                        Assert.Equal(before + 1, warnings.Count, $"FAR's warnings after an event with {what}");
                        Assert.Near(b2, wing.b_2_actual, 1e-6, $"the span after an event with {what}");
                        Assert.Near(mac, wing.MAC_actual, 1e-6, $"the mean chord after an event with {what}");
                    }

                    yield return Wait.Until(() => geometry.Ready, 30, "FAR's geometry of the wing", shot: false);
                    ctx.Record("warning", warnings.LastOrDefault());

                    ctx.Step("Scaled to 2 by the event, as a float and as a double, and back to 1");
                    foreach ((string what, object factor) in new (string, object)[] { ("a float", 2f), ("a double", 2.0) })
                    {
                        List<GeometryMesh> meshes = geometry.meshDataList;
                        Send(d => d.Set(TweakScaleEvent.FactorKey, factor));
                        Assert.Near(2 * b2, wing.b_2_actual, 1e-4 * b2, $"the span scaled by {what} (m)");
                        Assert.Near(2 * mac, wing.MAC_actual, 1e-4 * mac, $"the mean chord scaled by {what} (m)");
                        yield return Wait.Until(() => geometry.Ready && !ReferenceEquals(meshes, geometry.meshDataList), 30,
                                                $"FAR's geometry of the wing rebuilt for {what}", shot: false);
                        Send(d => d.Set(TweakScaleEvent.FactorKey, 1f));
                        Assert.Near(b2, wing.b_2_actual, 1e-4 * b2, "the span back (m)");
                        Assert.Near(mac, wing.MAC_actual, 1e-4 * mac, "the mean chord back (m)");
                        yield return Wait.Until(() => geometry.Ready, 30, "FAR's geometry of the wing", shot: false);
                    }

                    log.AssertNone("TweakScale's rescale events");
                }

                ctx.Step("A rescale that throws");
                var scaled = new BaseEventDetails(BaseEventDetails.Sender.USER);
                scaled.Set(TweakScaleEvent.FactorKey, 2f);
                TweakScaleEvent.Handle(wing, scaled, _ => throw new InvalidOperationException("a rescale that throws"));
                Assert.Equal(1, failures.Count, $"FAR's logged rescale failures ({string.Join(" | ", failures)})");
                ctx.Record("failure", failures[0]);
            }
            finally
            {
                Application.logMessageReceived -= OnLog;
                // the craft as it was for the tests after (the editor keeps it), whatever happened to it here
                EditorLogic.LoadShipFromFile(ctx.CraftPath(Far.Plane, "SPH"));
            }
        }

        /// <summary>
        /// The Aeris 4A's trailing-edge surfaces as flaps and spoilers in the editor's analysis: the elevons as flaps,
        /// their setting 2 gives more lift at 2 deg than flaps up; the ailerons as spoilers, deployed, less lift and more
        /// drag. The elevons set to deflect with angle of attack (pitch due to AoA 100%): with no pitch input they
        /// deflect by about the angle of attack the sim puts them at, and not at all with the setting at 0.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Editor_FlapsSpoilersAndAoATweakableInTheAnalysis(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            yield return Far.Settled("the Aeris 4A");
            List<FARControllableSurface> surfaces = EditorLogic.fetch.ship.parts.SelectMany(p => p.Modules.OfType<FARControllableSurface>()).ToList();
            List<FARControllableSurface> elevons = surfaces.Where(s => s.part.partInfo.name == "elevon2").ToList();
            List<FARControllableSurface> ailerons = surfaces.Where(s => s.part.partInfo.name == "StandardCtrlSrf").ToList();
            Assert.Equal(2, elevons.Count, "the Aeris's elevons");
            Assert.Equal(2, ailerons.Count, "the Aeris's ailerons");
            var saved = surfaces.Select(s => new Tweakables(s)).ToList();
            try
            {
                foreach (FARControllableSurface s in surfaces)
                {
                    s.isFlap = s.isSpoiler = false;
                    s.pitchaxisDueToAoA = 0;
                    s.brakeRudder = 0;
                }

                InstantConditionSimOutput clean = Instant(2, 0.3);
                ctx.Step("the elevons as flaps");
                foreach (FARControllableSurface s in elevons)
                    s.isFlap = true;
                InstantConditionSimOutput flapsUp = Instant(2, 0.3), flaps2 = Instant(2, 0.3, 2);
                double flapDeflection = elevons[0].ControlDeflection;
                ctx.Record("flaps", $"clean Cl {clean.Cl:F4}; flaps 0 Cl {flapsUp.Cl:F4}, flaps 2 Cl {flaps2.Cl:F4} Cd {flaps2.Cd:F5}, " +
                                    $"elevon deflection {string.Join(", ", elevons.Select(s => s.ControlDeflection.ToString("F2")))}");
                Assert.Near(clean.Cl, flapsUp.Cl, 1e-4 + 0.01 * Math.Abs(clean.Cl), "flaps up: Cl as with no flaps");
                Assert.True(Math.Abs(flapDeflection) > 1, $"the flaps deflected at setting 2 ({flapDeflection:F2} deg)");
                Assert.True(flaps2.Cl > flapsUp.Cl + 0.005, $"more lift with the flaps down ({flaps2.Cl:F4} against {flapsUp.Cl:F4})");
                foreach (FARControllableSurface s in elevons)
                    s.isFlap = false;

                ctx.Step("the ailerons as spoilers");
                foreach (FARControllableSurface s in ailerons)
                    s.isSpoiler = true;
                InstantConditionSimOutput stowed = Instant(2, 0.3), spoiled = Instant(2, 0.3, 0, true);
                ctx.Record("spoilers", $"stowed Cl {stowed.Cl:F4} Cd {stowed.Cd:F5}; deployed Cl {spoiled.Cl:F4} Cd {spoiled.Cd:F5}, " +
                                       $"aileron deflection {string.Join(", ", ailerons.Select(s => s.ControlDeflection.ToString("F2")))}");
                Assert.Near(clean.Cl, stowed.Cl, 1e-4 + 0.01 * Math.Abs(clean.Cl), "spoilers stowed: Cl as with none");
                Assert.True(ailerons.All(s => Math.Abs(s.ControlDeflection) > 1), "the spoilers deflected");
                Assert.True(spoiled.Cl < stowed.Cl - 0.002, $"less lift with the spoilers out ({spoiled.Cl:F4} against {stowed.Cl:F4})");
                Assert.True(spoiled.Cd > stowed.Cd, $"more drag with the spoilers out ({spoiled.Cd:F5} against {stowed.Cd:F5})");
                foreach (FARControllableSurface s in ailerons)
                    s.isSpoiler = false;

                ctx.Step("the elevons deflecting with angle of attack");
                InstantConditionSimOutput fixed6 = Instant(6, 0.3);
                double fixedDeflection = elevons.Max(s => Math.Abs(s.ControlDeflection));
                foreach (FARControllableSurface s in elevons)
                    s.pitchaxisDueToAoA = 100;
                InstantConditionSimOutput with6 = Instant(6, 0.3);
                double[] deflections = elevons.Select(s => s.ControlDeflection).ToArray();
                ctx.Record("pitchDueToAoA", $"at 6 deg: setting 0 Cm {fixed6.Cm:F4}, deflection {fixedDeflection:F2}; setting 100 Cm {with6.Cm:F4}, " +
                                            $"deflections {string.Join(", ", deflections.Select(d => d.ToString("F2")))}");
                Assert.True(fixedDeflection < 1e-3, $"no deflection with the setting at 0 ({fixedDeflection:G3} deg)");
                foreach (double d in deflections)
                    Assert.InRange(3, 10, Math.Abs(d), "an elevon's deflection with angle of attack at 6 deg (deg)");
                // (mirrored parts: the same deflection is opposite numbers on the two)
                Assert.Near(Math.Abs(deflections[0]), Math.Abs(deflections[1]), 0.05 * Math.Abs(deflections[0]), "both elevons alike");
                Assert.True(Math.Abs(with6.Cm - fixed6.Cm) > 1e-3, $"the deflection moves the pitching moment ({fixed6.Cm:F4} to {with6.Cm:F4})");
            }
            finally
            {
                foreach (Tweakables t in saved)
                    t.Restore();
            }
        }

        /// <summary>
        /// A brake rudder (the Aeris 4A's ailerons with brake rudder 100% and yaw 100%) in the editor's analysis, which
        /// gives the surfaces no yaw input: they stay put, and the craft's yaw and roll moments are those with no brake
        /// rudder. Given yaw input, the surface on the side of the yaw deflects. Regression: before the fix, SetControlStateEditor uses the
        /// yaw tweakable (yawaxis) where the yaw input belongs (BrakeRudderSide * yawaxis; flight and the snapshot use the
        /// input), so in the editor a brake rudder on one side always deflects. Fixed on another branch.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Editor_BrakeRudderNeedsYawInput(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            yield return Far.Settled("the Aeris 4A");
            List<FARControllableSurface> ailerons = EditorLogic.fetch.ship.parts.SelectMany(p => p.Modules.OfType<FARControllableSurface>())
                                                               .Where(s => s.part.partInfo.name == "StandardCtrlSrf").ToList();
            Assert.Equal(2, ailerons.Count, "the Aeris's ailerons");
            var saved = ailerons.Select(s => new Tweakables(s)).ToList();
            try
            {
                foreach (FARControllableSurface s in ailerons)
                {
                    s.yawaxis = 100;
                    s.brakeRudder = 0;
                }

                InstantConditionSimOutput plain = Instant(2, 0.4);
                foreach (FARControllableSurface s in ailerons)
                    s.brakeRudder = 100;
                InstantConditionSimOutput brake = Instant(2, 0.4);
                double[] deflections = ailerons.Select(s => s.ControlDeflection).ToArray();
                ctx.Record("noYaw", $"deflections {string.Join(", ", deflections.Select(d => d.ToString("F2")))}; Cn {plain.Cn:F5} / {brake.Cn:F5}, " +
                                    $"Croll {plain.C_roll:F5} / {brake.C_roll:F5}");

                // given yaw input directly, as flight gives it: one side's brake rudder opens
                Vector3 com = EditorLogic.RootPart.transform.position;
                var yawed = new double[ailerons.Count];
                for (int i = 0; i < ailerons.Count; i++)
                {
                    ailerons[i].SetControlStateEditor(com, EditorLogic.RootPart.transform.up, 0, 1, 0, 0, false);
                    yawed[i] = ailerons[i].ControlDeflection;
                }

                ctx.Record("yaw", $"yaw input 1: {string.Join(", ", yawed.Select(d => d.ToString("F2")))}");
                Assert.True(yawed.Count(d => Math.Abs(d) > 5) == 1, $"one brake rudder opens with yaw input ({string.Join(", ", yawed.Select(d => d.ToString("F2")))})");
                foreach (double d in deflections)
                    Assert.True(Math.Abs(d) < 1e-3, $"no brake rudder deflection without yaw input ({d:F2} deg)");
                Assert.Near(plain.Cn, brake.Cn, 1e-5 + 0.01 * Math.Abs(plain.Cn), "the yaw moment without yaw input, as with no brake rudder");
                Assert.Near(plain.C_roll, brake.C_roll, 1e-5 + 0.01 * Math.Abs(plain.C_roll), "the roll moment without yaw input, as with no brake rudder");
            }
            finally
            {
                foreach (Tweakables t in saved)
                    t.Restore();
            }
        }

        /// <summary>
        /// The stock Mallard's two engine nacelles (nacelleBody) have their intake at the front with a stack node there, a
        /// circular intake on it. FAR gives each nacelle an intake adjuster that ducts 100 times the intake's area along
        /// the intake's axis only, and takes the area of the node in front of it (0.625 m per unit of node size) as the
        /// threshold beside a fuselage. Without the nacelles' adjusters the sections along the nacelles change; with
        /// them back the model is what it was.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Mallard, TimeoutSec = 900)]
        public static IEnumerator Editor_NacelleIntakesDuctBehindTheirNode(TestContext ctx)
        {
            yield return LoadedCraft("the Mallard", "nacelleBody");
            Part[] nacelles = EditorLogic.fetch.ship.parts.Where(p => p.partInfo.name == "nacelleBody").ToArray();
            Assert.Equal(2, nacelles.Length, "the Mallard's nacelles");
            Vector3[] axes = { Vector3.right, Vector3.left, Vector3.up, Vector3.down, Vector3.forward, Vector3.back };
            foreach (Part p in nacelles)
            {
                GeometryPartModule g = p.FindModuleImplementing<GeometryPartModule>();
                var adjusters = Refl.Get<List<ICrossSectionAdjuster>>(g, "crossSectionAdjusters");
                Assert.Equal(1, adjusters.Count, "one cross-section adjuster on a nacelle");
                var adjuster = adjusters[0] as IntakeCrossSectionAdjuster;
                Assert.NotNull(adjuster, $"an intake adjuster ({adjusters[0].GetType().Name})");
                double intake = 100 * p.FindModuleImplementing<ModuleResourceIntake>().area;
                AttachNode top = p.FindAttachNode("top");
                Assert.NotNull(top, "the nacelle's top node");
                double nodeArea = Math.PI * Math.Pow(0.625 * (top.size == 0 ? 0.5 : top.size), 2);
                ctx.Record(p.persistentId.ToString(), $"intake {intake:F4} m2, threshold {adjuster.AreaThreshold():F4} m2, node size {top.size}, " +
                                                      $"on it {top.attachedPart?.partInfo.name}");
                Assert.Near(-nodeArea, adjuster.AreaThreshold(), 1e-6, "the threshold: minus the front node's area (m2)");
                Assert.Near(intake, adjuster.AreaRemovedFromCrossSection(), 1e-6, "the area ducted (m2)");
                double[] along = axes.Select(a => adjuster.AreaRemovedFromCrossSection(a)).ToArray();
                Assert.Equal(1, along.Count(a => a > 0), $"ducted along one axis of the six ({string.Join(", ", along.Select(a => a.ToString("F3")))})");
                Assert.Near(intake, along.Max(), 1e-6, "the area ducted along the intake's axis (m2)");
            }

            // the reference voxelized the way the comparison is
            yield return Far.Revoxelize("the Mallard as loaded");
            VehicleAerodynamics vehicle = Far.EditorVehicle;
            double vol = Far.Volume(vehicle), cd = Far.Instant(2, 0.6).Cd;
            double[] sections = vehicle.GetCrossSectionAreas();
            var saved = new Dictionary<GeometryPartModule, List<ICrossSectionAdjuster>>();
            try
            {
                ctx.Step("Without the nacelles' adjusters");
                foreach (Part p in nacelles)
                {
                    GeometryPartModule g = p.FindModuleImplementing<GeometryPartModule>();
                    saved[g] = Refl.Get<List<ICrossSectionAdjuster>>(g, "crossSectionAdjusters");
                    Refl.Set(g, "crossSectionAdjusters", new List<ICrossSectionAdjuster>());
                }

                yield return Far.Revoxelize("the Mallard without ducting");
                double volBare = Far.Volume(vehicle), cdBare = Far.Instant(2, 0.6).Cd;
                double[] bare = vehicle.GetCrossSectionAreas();
                double most = sections.Length == bare.Length ? sections.Zip(bare, (a, b) => Math.Abs(a - b)).Max() : double.NaN;
                ctx.Record("ducting", $"volume {vol:F2} m3 ducted, {volBare:F2} without; Cd at Mach 0.6 {cd:F5} ducted, {cdBare:F5} without; " +
                                      $"sections differ by up to {most:F3} m2");
                Assert.Equal(sections.Length, bare.Length, "the sections without the ducting");
                Assert.True(most > 0.05, $"the nacelles' ducting changes the sections along them (by up to {most:F3} m2)");
            }
            finally
            {
                foreach (KeyValuePair<GeometryPartModule, List<ICrossSectionAdjuster>> kv in saved)
                    Refl.Set(kv.Key, "crossSectionAdjusters", kv.Value);
            }

            yield return Far.Revoxelize("the Mallard ducted again");
            Assert.Near(vol, Far.Volume(vehicle), 1e-4 * vol, "the ducted volume again (m3)");
            // 0.5%: under coverage the same voxels (the volume to 1e-4) give a Cd of 0.02801 or 0.02804, either way round
            // between two voxelizations; the ducting itself moves it 0.04%, so the volume and sections are its check
            Assert.Near(cd, Far.Instant(2, 0.6).Cd, 5e-3 * cd, "the ducted Cd again");
        }

        /// <summary>
        /// The Rocket-power VTOL's two chutes in the hangar: one's info window open, opening the other's closes it and
        /// takes its place; the window has no sliders and no copy button in the editor (they are for flight), and its
        /// close button closes it.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Vtol, TimeoutSec = 600)]
        public static IEnumerator Editor_ChuteInfoWindowInTheHangar(TestContext ctx)
        {
            yield return Wait.Until(() => EditorLogic.fetch?.ship?.parts.Count(p => p.FindModuleImplementing<RealChuteFAR>() != null) == 2, 60,
                                    "the VTOL in the hangar", shot: false);
            List<RealChuteFAR> chutes = EditorLogic.fetch.ship.parts.Select(p => p.FindModuleImplementing<RealChuteFAR>()).Where(c => c != null).ToList();
            RealChuteFAR a = chutes[0], b = chutes[1];
            string window = Localizer.Format("RCLGUITitle");
            bool Visible(RealChuteFAR c) => (bool)Refl.Get(c, "visible");
            try
            {
                a.GUIToggleWindow();
                yield return Imgui.WaitWindow(window);
                Refl.Set(a, "window", new Rect(300, 120, 350, 400));
                b.GUIToggleWindow();
                Assert.True(Visible(b) && !Visible(a), $"the second chute's window open, the first's closed ({Visible(a)}, {Visible(b)})");
                var rb = (Rect)Refl.Get(b, "window");
                Assert.True(Math.Abs(rb.x - 300) < 1 && Math.Abs(rb.y - 120) < 1, $"where the first was ({rb.x}, {rb.y})");
                yield return Imgui.WaitWindow(window);
                yield return Imgui.WaitVisible(Localizer.Format("RCLGUI14", b.deployAltitude), window);
                Assert.False(Imgui.IsVisible(Localizer.Format("RCLGUICopy"), window), "no copy button in the editor");
                yield return Imgui.Click(Localizer.Format("RCLGUIClose"), window);
                yield return Imgui.WaitWindowClosed(window);
                Assert.False(Visible(a) || Visible(b), "both windows closed");
            }
            finally
            {
                Refl.Set(a, "visible", false);
                Refl.Set(b, "visible", false);
            }
        }

        /// <summary>A control surface's part-menu settings, to put back.</summary>
        private sealed class Tweakables
        {
            private readonly FARControllableSurface s;
            private readonly float yaw, pitchDueToAoA, brakeRudder;
            private readonly bool isFlap, isSpoiler;

            public Tweakables(FARControllableSurface s)
            {
                this.s = s;
                yaw = s.yawaxis;
                pitchDueToAoA = s.pitchaxisDueToAoA;
                brakeRudder = s.brakeRudder;
                isFlap = s.isFlap;
                isSpoiler = s.isSpoiler;
            }

            public void Restore()
            {
                s.yawaxis = yaw;
                s.pitchaxisDueToAoA = pitchDueToAoA;
                s.brakeRudder = brakeRudder;
                s.isFlap = isFlap;
                s.isSpoiler = isSpoiler;
            }
        }

        private static readonly MethodInfo SetupAdjusters =
            typeof(GeometryPartModule).GetMethod("SetupICrossSectionAdjusters", BindingFlags.Instance | BindingFlags.NonPublic);

        /// <summary>A part's cross-section adjusters made again, as GeometryPartModule.Start makes them, and its one intake adjuster.</summary>
        private static IntakeCrossSectionAdjuster RebuiltIntakeAdjuster(Part p)
        {
            Assert.NotNull(SetupAdjusters, "GeometryPartModule.SetupICrossSectionAdjusters");
            GeometryPartModule g = p.FindModuleImplementing<GeometryPartModule>();
            SetupAdjusters.Invoke(g, null);
            var list = Refl.Get<List<ICrossSectionAdjuster>>(g, "crossSectionAdjusters");
            Assert.Equal(1, list.Count, "one cross-section adjuster on the nacelle");
            var adjuster = list[0] as IntakeCrossSectionAdjuster;
            Assert.NotNull(adjuster, $"an intake adjuster ({list[0].GetType().Name})");
            return adjuster;
        }

        /// <summary>
        /// Intake configs no stock intake has, as a mod's would set them, on the Mallard's nacelles (their adjusters made
        /// again for each): an occludeNode (top, where the circular intake sits) makes the intake covered, so it ducts
        /// nothing and its own node is no threshold, and with both covered the model is the one without the nacelles'
        /// adjusters, not the one as loaded; a front node of size 0 counts as size 0.5; an intake with no intake
        /// transform takes the part's. All put back, the adjusters again what they were and the model with them.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Mallard, TimeoutSec = 900)]
        public static IEnumerator Editor_ModdedIntakeConfigs(TestContext ctx)
        {
            yield return LoadedCraft("the Mallard", "nacelleBody");
            Part[] nacelles = EditorLogic.fetch.ship.parts.Where(p => p.partInfo.name == "nacelleBody").ToArray();
            Assert.Equal(2, nacelles.Length, "the Mallard's nacelles");
            Part a = nacelles[0], b = nacelles[1];
            ModuleResourceIntake ia = a.FindModuleImplementing<ModuleResourceIntake>(), ib = b.FindModuleImplementing<ModuleResourceIntake>();
            AttachNode topA = a.FindAttachNode("top"), topB = b.FindAttachNode("top");
            Assert.NotNull(topA?.attachedPart, "a part on the first nacelle's top node");
            Assert.True(string.IsNullOrEmpty(ia.occludeNode), "a stock nacelle has no occludeNode");
            yield return Far.Revoxelize("the Mallard as loaded");
            double vol = Far.Volume(Far.EditorVehicle);
            double[] sections = Far.EditorVehicle.GetCrossSectionAreas();
            double Most(double[] x, double[] y) => x.Length == y.Length ? x.Zip(y, (u, w) => Math.Abs(u - w)).Max() : double.NaN;
            ModuleResourceIntake[] intakes = { ia, ib };
            AttachNode[] tops = { topA, topB };
            string[] occludeWas = intakes.Select(i => i.occludeNode).ToArray();
            AttachNode[] nodeWas = intakes.Select(i => i.node).ToArray();
            int sizeWas = topB.size;
            Transform transformWas = ib.intakeTransform;
            try
            {
                // one nacelle's ducting alone does not show on the Mallard (the other's is enough): both covered
                ctx.Step("An occludeNode on both nacelles' intakes, their top, a circular intake on each");
                for (int k = 0; k < 2; k++)
                {
                    Assert.NotNull(tops[k]?.attachedPart, "a part on the nacelle's top node");
                    intakes[k].occludeNode = "top";
                    intakes[k].node = tops[k];   // as ModuleResourceIntake.OnStart finds it
                    IntakeCrossSectionAdjuster covered = RebuiltIntakeAdjuster(nacelles[k]);
                    ctx.Record($"occluded{k}", $"node {Refl.Get<AttachNode>(covered, "node")?.id}, on it {tops[k].attachedPart.partInfo.name}; " +
                                               $"area {covered.AreaRemovedFromCrossSection():F4} m2 (open {100 * intakes[k].area:F4}), threshold {covered.AreaThreshold():F4}");
                    Assert.True(ReferenceEquals(tops[k], Refl.Get<AttachNode>(covered, "node")), "the adjuster watches the occlude node");
                    Assert.Equal(0.0, covered.AreaRemovedFromCrossSection(), "covered by the part on it, the intake ducts nothing");
                    Assert.Equal(0.0, covered.AreaThreshold(), "its own node in front is no threshold");
                }

                yield return Far.Revoxelize("the Mallard with its nacelles' intakes covered");
                double volCovered = Far.Volume(Far.EditorVehicle);
                double[] coveredSections = Far.EditorVehicle.GetCrossSectionAreas();
                var kept = new Dictionary<GeometryPartModule, List<ICrossSectionAdjuster>>();
                foreach (Part p in nacelles)
                {
                    GeometryPartModule g = p.FindModuleImplementing<GeometryPartModule>();
                    kept[g] = Refl.Get<List<ICrossSectionAdjuster>>(g, "crossSectionAdjusters");
                    Refl.Set(g, "crossSectionAdjusters", new List<ICrossSectionAdjuster>());
                }

                yield return Far.Revoxelize("the Mallard with no adjusters on its nacelles");
                double[] noneSections = Far.EditorVehicle.GetCrossSectionAreas();
                foreach (KeyValuePair<GeometryPartModule, List<ICrossSectionAdjuster>> kv in kept)
                    Refl.Set(kv.Key, "crossSectionAdjusters", kv.Value);
                double fromLoaded = Most(sections, coveredSections), fromNone = Most(coveredSections, noneSections);
                ctx.Record("covered", $"{vol:F3} m3 as loaded, {volCovered:F3} covered; covered sections differ from as loaded by up to {fromLoaded:F4} m2, " +
                                      $"from no adjusters by {fromNone:F4} m2 (no adjusters from as loaded: {Most(sections, noneSections):F4} m2)");
                Assert.True(fromLoaded > 0.05, $"covering the intakes changes the sections along the nacelles (by up to {fromLoaded:F4} m2)");
                Assert.Near(0, fromNone, 1e-6, "covered intakes duct as none: the model without their adjusters (m2)");
                for (int k = 0; k < 2; k++)
                {
                    intakes[k].occludeNode = occludeWas[k];
                    intakes[k].node = nodeWas[k];
                    RebuiltIntakeAdjuster(nacelles[k]);
                }

                ctx.Step("A front node of size 0 on the second nacelle");
                topB.size = 0;
                IntakeCrossSectionAdjuster small = RebuiltIntakeAdjuster(b);
                double half = Math.PI * Math.Pow(0.625 * 0.5, 2);
                ctx.Record("size0", $"threshold {small.AreaThreshold():F4} m2 (size 0.5's {-half:F4})");
                Assert.Near(-half, small.AreaThreshold(), 1e-6, "a size-0 node taken as size 0.5 (m2)");
                topB.size = sizeWas;

                ctx.Step("No intake transform on the second nacelle");
                ib.intakeTransform = null;
                IntakeCrossSectionAdjuster bare = RebuiltIntakeAdjuster(b);
                Assert.True(ReferenceEquals(b.partTransform, Refl.Get<Transform>(bare, "intakeTrans")), "the part's transform in its place");
            }
            finally
            {
                for (int k = 0; k < 2; k++)
                {
                    intakes[k].occludeNode = occludeWas[k];
                    intakes[k].node = nodeWas[k];
                }
                topB.size = sizeWas;
                ib.intakeTransform = transformWas;
            }

            ctx.Step("The stock configs back");
            foreach (Part p in nacelles)
            {
                IntakeCrossSectionAdjuster again = RebuiltIntakeAdjuster(p);
                Assert.Near(100 * p.FindModuleImplementing<ModuleResourceIntake>().area, again.AreaRemovedFromCrossSection(), 1e-6, "ducting again (m2)");
            }

            yield return Far.Revoxelize("the Mallard with its stock intakes");
            Assert.Near(vol, Far.Volume(Far.EditorVehicle), 1e-4 * vol, "the volume again (m3)");
        }

        /// <summary>A craft loaded in place of another: its parts there, FAR's voxel model as long as the craft, and settled.</summary>
        private static IEnumerator LoadedCraft(string what, params string[] parts)
        {
            yield return Wait.Until(() => Far.EditorReady && parts.All(n => EditorLogic.fetch.ship.parts.Any(p => p.partInfo.name == n)),
                                    120, "FAR's editor on " + what, shot: false);
            Transform root = EditorLogic.RootPart.transform;
            double length = CraftLength(root.up);
            yield return Wait.Until(() => Far.EditorReady && Far.EditorVehicle.Length > 0.85 * length && Far.EditorVehicle.Length < 1.1 * length,
                                    120, $"FAR's voxel model of {what}, {length:F1} m long", shot: false,
                                    detail: () => Far.EditorReady ? $"FAR's model is {Far.EditorVehicle.Length:F2} m long" : "FAR's editor not ready");
            yield return Far.Settled(what);
        }

        /// <summary>The craft's length along an axis: the spread of its meshes' vertices (in world space) along it.</summary>
        private static double CraftLength(Vector3 axis)
        {
            double min = double.PositiveInfinity, max = double.NegativeInfinity;
            foreach (Part p in EditorLogic.fetch.ship.parts)
                foreach (MeshFilter mf in p.GetComponentsInChildren<MeshFilter>())
                {
                    if (mf.sharedMesh == null || mf.GetComponent<Renderer>()?.enabled != true) continue;
                    foreach (Vector3 w in Far.MeshPoints(mf))
                    {
                        double d = Vector3.Dot(w, axis);
                        min = Math.Min(min, d);
                        max = Math.Max(max, d);
                    }
                }

            return max - min;
        }
    }
}
