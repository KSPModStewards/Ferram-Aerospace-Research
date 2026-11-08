using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>FAR's flight GUI bookkeeping across vessels that are never loaded.</summary>
    public static class FarFlightGuiTests
    {
        private const BindingFlags Static = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static;

        /// <summary>
        /// FlightGUI.activeFlightGUICount against a vessel that is created and destroyed without ever loading. With
        /// Blizzy's toolbar on, the Aeris 4A is put on the runway, an asteroid is spawned far out (unloaded), then
        /// destroyed. The count must come back to its value before the spawn and agree with
        /// FlightGUI.vesselFlightGUI, and the toolbar button must still be there. (KSPModStewards FAR #151 gives
        /// FlightGUI an activation of loaded vessels only, so OnStart, which counts up, never runs on the asteroid,
        /// while OnDestroy, which counts down and removes the button at zero, still does.)
        /// </summary>
        [UITest("Far_FlightGuiUnloadedVessel", Explicit = true, Kind = "Check", Mods = "Toolbar", StartsAt = StartState.SpaceCenter, TimeoutSec = 900)]
        public static IEnumerator Far_FlightGuiUnloadedVessel(TestContext ctx)
        {
            ctx.Record("farBuild", FarSim.FarVersion());
            Type gui = Reflect.TryFindType("FerramAerospaceResearch.FARGUI.FARFlightGUI.FlightGUI")
                       ?? throw new UITestFailure("FAR's FlightGUI type not found");
            Type debugValues = Reflect.TryFindType("FerramAerospaceResearch.FARDebugValues")
                               ?? throw new UITestFailure("FARDebugValues not found");
            FieldInfo countField = gui.GetField("activeFlightGUICount", Static);
            FieldInfo buttonField = gui.GetField("blizzyFlightGUIButton", Static);
            FieldInfo guisField = gui.GetField("vesselFlightGUI", Static);
            FieldInfo blizzyField = debugValues.GetField("useBlizzyToolbar", Static);
            if (countField == null || buttonField == null || guisField == null || blizzyField == null)
                throw new UITestFailure("FlightGUI/FARDebugValues fields not found");
            int Count() => (int)countField.GetValue(null);
            bool HasButton() => buttonField.GetValue(null) != null;
            int Guis() => (guisField.GetValue(null) as IDictionary)?.Count ?? -1;
            string State() => $"activeFlightGUICount {Count()}, vesselFlightGUI {Guis()}, button {(HasButton() ? "present" : "gone")}";

            bool blizzyWas = (bool)blizzyField.GetValue(null);
            blizzyField.SetValue(null, true);
            try
            {
                ctx.Step("Aeris 4A on the runway, Blizzy's toolbar on");
                yield return Nav.EnsureState(ctx, StartState.Flight, ctx.CraftPath("Aeris 4A", "SPH"), "Runway");
                yield return Wait.FlightReady();
                yield return Wait.Seconds(3);
                List<Vessel> vessels = FlightGlobals.Vessels;
                ctx.Record("vessels", vessels.Count);
                ctx.Record("vesselsLoaded", vessels.Count(v => v.loaded));
                // FAR re-reads useBlizzyToolbar from the save when the scene loads, so make the button here
                if (!HasButton())
                {
                    try
                    {
                        gui.GetMethod("GenerateBlizzyToolbarButton", Static)?.Invoke(null, null);
                    }
                    catch (Exception e)
                    {
                        ctx.Record("buttonError", (e.InnerException ?? e).Message);
                    }
                }
                int count0 = Count();
                bool button0 = HasButton();
                ctx.Record("before", State());

                ctx.Step("Spawn an asteroid far out");
                CelestialBody body = FlightGlobals.ActiveVessel.mainBody;
                double sma = Math.Min(body.Radius * 20, body.sphereOfInfluence * 0.5);
                var orbit = new Orbit(0, 0, sma, 0, 0, 0, Planetarium.GetUniversalTime(), body);
                ProtoVessel pv = DiscoverableObjectsUtil.SpawnAsteroid("FAR test rock", orbit, 151, UntrackedObjectClass.A, 1e7, 1e7);
                Vessel rock = pv?.vesselRef ?? throw new UITestFailure("asteroid spawn returned no vessel");
                yield return Wait.Frames(30);
                Component rockGui = rock.GetComponent(gui);
                ctx.Record("rockLoaded", rock.loaded);
                ctx.Record("rockGuiEnabled", rockGui is Behaviour b && b.enabled);
                ctx.Record("afterSpawn", State());

                ctx.Step("Destroy the asteroid");
                rock.Die();
                yield return Wait.Frames(30);
                yield return Wait.Seconds(1);
                int count2 = Count();
                bool button2 = HasButton();
                ctx.Record("afterDestroy", State());
                ctx.Record("activeVesselGuiShown", FlightGlobals.ActiveVessel != null && Guis() > 0);

                Assert.True(count2 == count0, $"activeFlightGUICount back to {count0} after the unloaded vessel went ({State()})");
                Assert.True(count2 == Guis(), $"activeFlightGUICount equals the FlightGUIs registered ({State()})");
                if (button0)
                    Assert.True(button2, $"FAR's Blizzy toolbar button still there ({State()})");
            }
            finally
            {
                blizzyField.SetValue(null, blizzyWas);
            }
        }
    }
}
