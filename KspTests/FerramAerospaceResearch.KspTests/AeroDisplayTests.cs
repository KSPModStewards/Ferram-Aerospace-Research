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
    /// The game's aero overlays fed by FAR in flight: stock's aerodynamic force arrows (F12) and the aero data lines in
    /// the part menus (the debug option "Display Aero Data in Action Menus"), shown on a stock plane in cruise.
    /// </summary>
    public static class AeroDisplayTests
    {
        private static ArrowPointer Arrow(object module, string name) => Refl.TryGet<ArrowPointer>(module, name);

        // Unity's null: a destroyed arrow compares equal to null
        private static bool Gone(ArrowPointer a) => a == null;

        /// <summary>
        /// With the force arrows, the torque arrows and the part menu's aero data on, at Mach 0.5 and 4 deg: every part
        /// in the airstream has a lift, a drag and a torque arrow, and its part menu shows the drag and lift FAR
        /// applied to it, split along and across the flow; every wing has its own lift and drag arrows, the lift across
        /// the flow and the drag against it, with the wing's lift and drag in its part menu, and the wings' lift adds
        /// up to most of the plane's. A wing panel destroyed while its arrows show takes them with it, and FAR goes on
        /// without an exception. With the overlays off again, the arrows go and the part menus' lines are hidden.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Flight_AeroForceArrowsAndPartMenuData(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FarFlightProbe.Condition c = Far.At(0.5, 3000, 4);
            yield return FlightTests.Airborne(c);
            Vessel v = FlightGlobals.ActiveVessel;

            bool forceWas = PhysicsGlobals.AeroForceDisplay, dataWas = PhysicsGlobals.AeroDataDisplay;
            bool momentWas = FARDebugValues.showMomentArrows;
            using var exceptions = new ExceptionLog();
            try
            {
                ctx.Step("overlays on");
                PhysicsGlobals.AeroForceDisplay = true;
                PhysicsGlobals.AeroDataDisplay = true;
                FARDebugValues.showMomentArrows = true;
                var samples = new List<FarFlightProbe.Sample>();
                yield return FarFlightProbe.Hold(c, 20, 10, samples, predict: false);
                double vesselLift = samples.Average(s => s.Flight[1]);

                List<FARAeroPartModule> exposed = v.parts.Where(p => !p.ShieldedFromAirstream)
                                                   .Select(p => p.Modules.GetModule<FARAeroPartModule>()).Where(m => m != null).ToList();
                Assert.True(exposed.Count > 5, $"parts of the plane in the airstream ({exposed.Count})");
                int checkedParts = 0;
                foreach (FARAeroPartModule m in exposed)
                {
                    string name = m.part.partInfo.name;
                    Assert.True(!Gone(Arrow(m, "liftArrow")) && !Gone(Arrow(m, "dragArrow")) && !Gone(Arrow(m, "momentArrow")),
                                $"{name} has its lift, drag and torque arrows");
                    Assert.True(m.Fields["dragForce"].guiActive && m.Fields["liftForce"].guiActive, $"{name}'s part menu shows its drag and lift");
                    Vector3 velNorm = m.part.partTransform.localToWorldMatrix.MultiplyVector((Vector3)Refl.Get(m, "partLocalVelNorm"));
                    double drag = Math.Abs(Vector3.Dot(m.worldSpaceAeroForce, velNorm));
                    double lift = (m.worldSpaceAeroForce - Vector3.Dot(m.worldSpaceAeroForce, velNorm) * velNorm).magnitude;
                    Assert.Near(drag, m.dragForce, 0.01 * drag + 0.005, $"{name}'s drag in its part menu against the force FAR applied");
                    Assert.Near(lift, m.liftForce, 0.01 * lift + 0.005, $"{name}'s lift in its part menu against the force FAR applied");
                    checkedParts++;
                }

                List<FARWingAerodynamicModel> wings = v.parts.SelectMany(p => p.Modules.OfType<FARWingAerodynamicModel>())
                                                       .Where(w => !w.isShielded).ToList();
                Assert.True(wings.Count >= 4, $"the plane's wings in the airstream ({wings.Count})");
                Vector3 flow = (Vector3)(v.srf_velocity.normalized);
                double wingLift = 0;
                foreach (FARWingAerodynamicModel w in wings)
                {
                    string name = w.part.partInfo.name;
                    ArrowPointer liftArrow = Arrow(w, "liftArrow"), dragArrow = Arrow(w, "dragArrow");
                    Assert.True(!Gone(liftArrow) && !Gone(dragArrow), $"{name} has its wing lift and drag arrows");
                    Assert.True(w.Fields["liftForceWing"].guiActive && w.Fields["dragForceWing"].guiActive, $"{name}'s part menu shows the wing's lift and drag");
                    Assert.True(w.dragForceWing > 0, $"{name}'s wing drag ({w.dragForceWing:F3} kN)");
                    Assert.True(Vector3.Dot(dragArrow.Direction.normalized, flow) < -0.98, $"{name}'s drag arrow points against the flow");
                    if (w.liftForceWing > 0.05)
                        Assert.Near(0, Vector3.Dot(liftArrow.Direction.normalized, flow), 0.02, $"{name}'s lift arrow across the flow");
                    Assert.Near(w.liftForceWing, liftArrow.Direction.magnitude, 0.01 * w.liftForceWing + 1e-3, $"{name}'s lift arrow is its lift");
                    wingLift += Vector3.Dot(liftArrow.Direction, FlightGlobals.ActiveVessel.upAxis);
                }

                ctx.Record("lift", $"wings {wingLift:F2} kN, plane {vesselLift:F2} kN; {checkedParts} parts, {wings.Count} wings");
                Assert.InRange(0.6 * vesselLift, 1.3 * vesselLift, wingLift, "the wings' lift arrows add up to most of the plane's lift");

                ctx.Step("a wing panel destroyed with its arrows out");
                FARWingAerodynamicModel lost = wings.Where(w => w.part.children.Count == 0 && w.part.parent != null)
                                                    .OrderByDescending(w => w.S).FirstOrDefault();
                Assert.NotNull(lost, "an outboard wing panel");
                FARAeroPartModule lostAero = lost.part.Modules.GetModule<FARAeroPartModule>();
                ArrowPointer[] lostArrows =
                {
                    Arrow(lost, "liftArrow"), Arrow(lost, "dragArrow"), Arrow(lostAero, "liftArrow"), Arrow(lostAero, "dragArrow"),
                    Arrow(lostAero, "momentArrow")
                };
                int partsBefore = v.parts.Count;
                lost.part.Die();
                yield return Wait.Until(() => v.parts.Count < partsBefore && lostArrows.All(Gone), 10, "the panel and its arrows gone", shot: false);
                samples.Clear();
                yield return FarFlightProbe.Hold(c, 5, 10, samples, predict: false);
                Assert.True(samples.All(s => s.Flight.All(x => !double.IsNaN(x) && !double.IsInfinity(x))), "a real force on the plane after the loss");
                exceptions.AssertNone("with a wing destroyed under the arrows");

                ctx.Step("overlays off");
                List<FARAeroPartModule> nowExposed = exposed.Where(m => m != null && m.part != null).ToList();
                List<FARWingAerodynamicModel> nowWings = wings.Where(w => w != null && w.part != null).ToList();
                var shown = nowExposed.SelectMany(m => new[] { Arrow(m, "liftArrow"), Arrow(m, "dragArrow"), Arrow(m, "momentArrow") })
                                      .Concat(nowWings.SelectMany(w => new[] { Arrow(w, "liftArrow"), Arrow(w, "dragArrow") })).ToList();
                PhysicsGlobals.AeroForceDisplay = false;
                PhysicsGlobals.AeroDataDisplay = false;
                yield return Wait.Until(() => shown.All(Gone), 10, "the arrows gone with the overlay off", shot: false);
                foreach (FARAeroPartModule m in nowExposed)
                    Assert.False(m.Fields["dragForce"].guiActive || m.Fields["liftForce"].guiActive,
                                 $"{m.part.partInfo.name}'s aero data hidden in its part menu");
                foreach (FARWingAerodynamicModel w in nowWings)
                    Assert.False(w.Fields["liftForceWing"].guiActive || w.Fields["dragForceWing"].guiActive,
                                 $"{w.part.partInfo.name}'s wing data hidden in its part menu");
                exceptions.AssertNone("with the overlays shown and hidden");
            }
            finally
            {
                PhysicsGlobals.AeroForceDisplay = forceWas;
                PhysicsGlobals.AeroDataDisplay = dataWas;
                FARDebugValues.showMomentArrows = momentWas;
            }
        }
    }
}
