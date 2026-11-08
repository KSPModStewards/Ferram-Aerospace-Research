using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using KSP.Localization;
using KspTest;
using UnityEngine;
using static FerramAerospaceResearch.KspTests.Refl;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// More of FAR's flight window on the stock Aeris 4A, flying: the stability augmentation systems besides the wing
    /// leveler (yaw and pitch dampers, the AoA limiter, the dynamic-pressure control scaling), their settings and
    /// their saving and loading.
    /// </summary>
    public static class FlightGuiCoverageTests
    {
        private static string Loc(string tag) => Localizer.Format(tag);

        private const double Deg2Rad = Math.PI / 180;

        // StabilityAugmentation.systemInstances[i]: 0 roll (wing leveler), 1 yaw, 2 pitch, 3 AoA limiter, 4 DPCR
        private const int Roll = 0, Yaw = 1, Pitch = 2, AoALimiter = 3, QScaling = 4;

        private sealed class Saved
        {
            public object[][] Systems;
            public double Low, High, Q;
            public object Templates;
            public ConfigNode Node;
        }

        private static readonly string[] SystemFields = { "active", "zeroPoint", "kP", "kD", "kI", "lastError", "errorIntegral" };

        /// <summary>
        /// The stability augmentation's control law, each system on its own, fed the flight's state (FAR's flight info
        /// for the vessel, set for each case) and a control state, as the vessel's autopilot callback is each physics
        /// frame. The yaw damper's yaw and the pitch damper's pitch go against the sideslip and the angle of attack, in
        /// proportion (the proportional gain), clamped to full deflection, and they leave the controls alone while
        /// the pilot holds them. The AoA limiter pushes the nose down above its upper limit and up below its lower
        /// one (clamped), and does nothing in between. The dynamic-pressure scaling scales the pilot's inputs by its
        /// reference pressure over the flight's, never up. The wing leveler forgets its integral while the pilot
        /// rolls. In flight, with the limiter's upper limit typed below the plane's angle of attack in the settings
        /// window, the plane's pitch input goes nose down. The settings window shows each system's fields: the gains
        /// and offset, the limiter's limits, the scaling's reference pressure. Saved and loaded again, the gains and
        /// limits come back.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator FlightGui_DampersLimiterAndQScaling(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FlightGUI gui = Far.FlightGui();
            object stab = Get(gui, "_stabilityAugmentation");
            Type type = stab.GetType();
            var systems = (Array)Get(stab, "systemInstances");
            bool shown = Far.FlightGuiShown, settingsOpen = Far.SubWindowOpen(gui, "showSettingsWindow");
            int group = Far.SettingsGroup(gui);
            object dropdown = Get(stab, "systemDropdown");
            int dropdownAt = Get<int>(dropdown, "selectedOption");
            Saved saved = Save(stab, type, systems);
            MethodInfo update = type.GetMethod("OnAutoPilotUpdate", Members) ?? throw new UITestFailure("no OnAutoPilotUpdate");
            try
            {
                for (int i = 0; i < systems.Length; i++)
                    Set(systems.GetValue(i), "active", false);
                FarFlightProbe.Condition cruise = Far.At(0.5, 3000, 3, "cruise M0.5 3");
                yield return FlightTests.Airborne(cruise);
                Vessel v = FlightGlobals.ActiveVessel;
                Assert.True(v.srfSpeed > 5, "flying (the systems act above 5 m/s)");

                FlightCtrlState Run(int system, double kP, Action<object> setInfo, float pitch = 0, float yaw = 0, float roll = 0)
                {
                    for (int i = 0; i < systems.Length; i++)
                        Set(systems.GetValue(i), "active", i == system);
                    object sys = systems.GetValue(system);
                    Set(sys, "kP", kP);
                    Set(sys, "kD", 0.0);
                    Set(sys, "kI", 0.0);
                    Set(sys, "zeroPoint", 0.0);
                    Set(sys, "lastError", 0.0);
                    Set(sys, "errorIntegral", 0.0);
                    object info = Get(stab, "info");   // boxed: set the box, then put it back
                    setInfo(info);
                    Set(stab, "info", info);
                    var state = new FlightCtrlState { pitch = pitch, yaw = yaw, roll = roll };
                    update.Invoke(stab, new object[] { state });
                    return state;
                }

                void Info(object info, string field, double value) => typeof(VesselFlightInfo).GetField(field).SetValue(info, value);

                ctx.Step("Yaw damper");
                FlightCtrlState s = Run(Yaw, 1, i => Info(i, "sideslipAngle", 4));
                Assert.Near(4 * Deg2Rad, s.yaw, 1e-5, "4 deg of sideslip: the yaw is the gain times the sideslip (rad)");
                s = Run(Yaw, 1, i => Info(i, "sideslipAngle", -4));
                Assert.Near(-4 * Deg2Rad, s.yaw, 1e-5, "the other way, the other yaw");
                s = Run(Yaw, 100, i => Info(i, "sideslipAngle", 4));
                Assert.Equal(1f, s.yaw, "a large gain: full deflection, no more");
                s = Run(Yaw, 1, i => Info(i, "sideslipAngle", 4), yaw: 0.5f);
                Assert.Equal(0.5f, s.yaw, "the pilot's yaw left alone");
                Assert.Equal(0f, s.pitch, "the yaw damper leaves the pitch alone");

                ctx.Step("Pitch damper");
                s = Run(Pitch, 1, i => Info(i, "aoA", 3));
                Assert.Near(-3 * Deg2Rad, s.pitch, 1e-5, "3 deg of incidence: nose down by the gain times it (rad)");
                s = Run(Pitch, 100, i => Info(i, "aoA", -3));
                Assert.Equal(1f, s.pitch, "nose up, clamped to full deflection");
                s = Run(Pitch, 1, i => Info(i, "aoA", 3), pitch: -0.3f);
                Assert.Equal(-0.3f, s.pitch, "the pilot's pitch left alone");

                ctx.Step("AoA limiter");
                SetStatic(type, "aoALowLim", -10.0);
                SetStatic(type, "aoAHighLim", 20.0);
                s = Run(AoALimiter, 0.25, i => Info(i, "aoA", 22));
                Assert.Near(-0.5, s.pitch, 1e-5, "2 deg over the upper limit: nose down by the gain times it");
                s = Run(AoALimiter, 0.25, i => Info(i, "aoA", 40));
                Assert.Equal(-1f, s.pitch, "far over: full nose down, no more");
                s = Run(AoALimiter, 0.25, i => Info(i, "aoA", -14));
                Assert.Near(1, s.pitch, 1e-5, "4 deg under the lower limit: nose up");
                s = Run(AoALimiter, 0.25, i => Info(i, "aoA", 5), pitch: 0.2f);
                Assert.Equal(0.2f, s.pitch, "between the limits the pitch is left alone");

                ctx.Step("Dynamic-pressure scaling");
                SetStatic(type, "scalingDynPres", 20.0);
                s = Run(QScaling, 0, i => Info(i, "dynPres", 40), pitch: 0.6f, yaw: -0.4f, roll: 0.2f);
                Assert.Near(0.3, s.pitch, 1e-5, "twice the reference pressure: half the pitch input");
                Assert.Near(-0.2, s.yaw, 1e-5, "half the yaw input");
                Assert.Near(0.1, s.roll, 1e-5, "half the roll input");
                s = Run(QScaling, 0, i => Info(i, "dynPres", 5), pitch: 0.6f);
                Assert.Near(0.6, s.pitch, 1e-6, "below the reference pressure the input is not scaled up");

                ctx.Step("The wing leveler forgets its integral while the pilot rolls");
                for (int i = 0; i < systems.Length; i++)
                    Set(systems.GetValue(i), "active", i == Roll);
                object leveler = systems.GetValue(Roll);
                Set(leveler, "errorIntegral", 0.7);
                update.Invoke(stab, new object[] { new FlightCtrlState { roll = 0.5f } });
                Assert.Equal(0.0, Get<double>(leveler, "errorIntegral"), "the leveler's integral after a roll input");

                ctx.Step("The settings window: each system's fields");
                Restore(stab, type, systems, saved, keepNode: true);
                for (int i = 0; i < systems.Length; i++)
                    Set(systems.GetValue(i), "active", false);
                Far.FlightGuiShown = true;
                yield return Imgui.WaitWindow(Far.MainWindow, 30);
                yield return Imgui.Toggle(Loc("FARFlightGUIFltSettings"), true, Far.MainWindow, exact: true);
                yield return Imgui.WaitWindow(Far.SettingsWindow);
                Far.SetSettingsGroup(gui, Far.GroupStabilityAugmentation);
                foreach ((int sys, string[] labels, string[] absent) in new[]
                {
                    (Yaw, new[] { "FARFlightStabPropGain", "FARFlightStabDerivGain", "FARFlightStabIntGain", "FARFlightStabOffset" }, new[] { "FARFlightStabAoALow" }),
                    (AoALimiter, new[] { "FARFlightStabPropGain", "FARFlightStabAoALow", "FARFlightStabAoAHigh" }, new[] { "FARFlightStabOffset" }),
                    (QScaling, new[] { "FARFlightStabQScaling" }, new[] { "FARFlightStabPropGain" }),
                })
                {
                    Set(dropdown, "selectedOption", sys);
                    yield return Wait.Frames(3);
                    foreach (string l in labels)
                        yield return Imgui.WaitVisible(Loc(l), Far.SettingsWindow);
                    foreach (string l in absent)
                        Assert.False(Imgui.IsVisible(Loc(l), Far.SettingsWindow), $"no '{Loc(l)}' for system {sys}");
                }

                ctx.Step("In flight: the limiter's upper limit typed under the plane's incidence");
                Set(dropdown, "selectedOption", AoALimiter);
                yield return Wait.Frames(2);
                yield return Imgui.SetTextAfterLabel(Loc("FARFlightStabAoAHigh"), "-5", Far.SettingsWindow);
                Assert.Near(-5, (double)GetStatic(type, "aoAHighLim"), 1e-9, "the upper limit typed in");
                yield return FarFlightProbe.Place(cruise);
                object limiter = systems.GetValue(AoALimiter);
                Set(limiter, "kP", 0.25);
                Set(limiter, "kD", 0.0);
                Set(limiter, "kI", 0.0);
                Far.SetStabAug(gui, AoALimiter, true);
                var fixedWait = new WaitForFixedUpdate();
                var pitches = new List<float>();
                for (int n = 0; n < 5; n++)
                {
                    yield return fixedWait;
                    pitches.Add(FlightGlobals.ActiveVessel.ctrlState.pitch);
                }

                double aoa = gui.InfoParameters.aoA;
                ctx.Record("limiterInFlight", $"AoA {aoa:F2} deg, pitch input {string.Join(" ", pitches.Select(p => p.ToString("F3")))}");
                Assert.True(aoa > -5, $"the plane flies above the limit ({aoa:F2} deg)");
                Assert.True(pitches.Skip(1).All(p => p < -0.5f), "the limiter pushes the nose down, hard (0.25 per degree over)");
                Far.SetStabAug(gui, AoALimiter, false);

                ctx.Step("Saved, then loaded again");
                Set(systems.GetValue(Pitch), "kP", 0.125);
                Set(systems.GetValue(Pitch), "kD", 0.75);
                Set(systems.GetValue(Pitch), "kI", 0.0625);
                Set(systems.GetValue(Yaw), "zeroPoint", 1.5);
                Set(systems.GetValue(Yaw), "active", true);
                SetStatic(type, "aoALowLim", -7.0);
                SetStatic(type, "scalingDynPres", 33.0);
                type.GetMethod("SaveSettings", Members).Invoke(stab, null);
                ConfigNode node = FARSettingsScenarioModule.FlightGUISettings.First(t => t.name == "StabilityAugmentationSettings");
                ctx.Record("saved", node.ToString().Replace("\n", " "));
                SetStatic(type, "systemTemplates", null);
                SetStatic(type, "aoALowLim", 0.0);
                SetStatic(type, "aoAHighLim", 0.0);
                SetStatic(type, "scalingDynPres", 0.0);
                type.GetMethod("LoadSettings", Members).Invoke(null, null);
                var templates = (Array)GetStatic(type, "systemTemplates");
                Assert.NotNull(templates, "the systems loaded");
                Assert.Near(0.125, Get<double>(templates.GetValue(Pitch), "kP"), 1e-12, "the pitch damper's proportional gain");
                Assert.Near(0.75, Get<double>(templates.GetValue(Pitch), "kD"), 1e-12, "its derivative gain");
                Assert.Near(0.0625, Get<double>(templates.GetValue(Pitch), "kI"), 1e-12, "its integral gain");
                Assert.Near(1.5, Get<double>(templates.GetValue(Yaw), "zeroPoint"), 1e-12, "the yaw damper's offset");
                Assert.True(Get<bool>(templates.GetValue(Yaw), "active"), "the yaw damper on");
                Assert.False(Get<bool>(templates.GetValue(Pitch), "active"), "the pitch damper off");
                Assert.Near(-7, (double)GetStatic(type, "aoALowLim"), 1e-12, "the limiter's lower limit");
                Assert.Near(-5, (double)GetStatic(type, "aoAHighLim"), 1e-12, "the limiter's upper limit");
                Assert.Near(33, (double)GetStatic(type, "scalingDynPres"), 1e-12, "the scaling's reference pressure");
            }
            finally
            {
                try
                {
                    Restore(stab, type, systems, saved, keepNode: false);
                    Set(dropdown, "selectedOption", dropdownAt);
                    Far.SetSubWindow(gui, "showSettingsWindow", settingsOpen);
                    Far.SetSettingsGroup(gui, group);
                }
                catch (Exception e)
                {
                    KspTest.Log.Warn("FlightGuiCoverageTests: restoring the stability augmentation: " + e.Message);
                }

                Far.FlightGuiShown = shown;
            }
        }

        // ------------------------------------------------------------------------------------------ data window

        private static readonly System.Text.RegularExpressions.Regex Number =
            new System.Text.RegularExpressions.Regex(@"^\s*(-?[0-9]+(\.[0-9]+)?|NaN|-?Infinity)");

        /// <summary>The numbers at the start of each line of the data window's value column.</summary>
        private static List<double> Values(string text) =>
            text.Split('\n').Select(l => Number.Match(l)).Where(m => m.Success)
                .Select(m => double.Parse(m.Groups[1].Value, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture))
                .ToList();

        /// <summary>
        /// The flight data window's sections and FAR's other flight display options, flying with the jets lit at full
        /// throttle. With every section on, the engine section's intake air is a finite percentage (the air the jets
        /// get over what they need), what FAR's flight info says. A section switched off in the settings window takes
        /// its lines out of the window, which shrinks to fit; switched on again they come back and it grows. The
        /// airspeed in mph is the surface speed times 2.2369, with its unit. F2 (the game hiding its UI) hides FAR's
        /// windows, and they come back with the UI.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator FlightGui_DataWindowSectionsF2AndMph(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FlightGUI gui = Far.FlightGui();
            bool shown = Far.FlightGuiShown, dataOpen = Far.SubWindowOpen(gui, "showFlightDataWindow"), settingsOpen = Far.SubWindowOpen(gui, "showSettingsWindow");
            int group = Far.SettingsGroup(gui);
            bool[] sections = Far.FlightDataSections(gui);
            var savedSections = (bool[])sections.Clone();
            AirspeedSettingsGUI air = gui.airSpeedGUI;
            object[] savedAir = Far.AirspeedSetting(air);
            var jets = new List<ModuleEngines>();
            bool hidden = false;
            try
            {
                Far.FlightGuiShown = true;
                yield return Imgui.WaitWindow(Far.MainWindow, 30);
                yield return Imgui.Toggle(Loc("FARFlightGUIFltDataBtn"), true, Far.MainWindow, exact: true);
                yield return Imgui.WaitWindow(Far.DataWindow);
                yield return Imgui.Toggle(Loc("FARFlightGUIFltSettings"), true, Far.MainWindow, exact: true);
                yield return Imgui.WaitWindow(Far.SettingsWindow);
                Far.SetSettingsGroup(gui, Far.GroupFlightData);
                yield return null;
                for (int i = 0; i < sections.Length; ++i)
                    yield return Imgui.Toggle(Loc("FARFlightDataOption" + i), true, Far.SettingsWindow, exact: true);

                ctx.Step("Flying at Mach 0.5, 3 km, the jets lit at full throttle");
                FarFlightProbe.Condition c = Far.At(0.5, 3000, 3, "cruise M0.5 3");
                yield return FlightTests.Airborne(c);
                Vessel v = FlightGlobals.ActiveVessel;
                jets = v.FindPartModulesImplementing<ModuleEngines>().Where(e => e.propellants.Any(p => p.name == "IntakeAir")).ToList();
                Assert.True(jets.Count > 0, "the plane has air-breathing engines");
                foreach (ModuleEngines e in jets)
                    e.Activate();
                Ctrl.SetThrottle(1f);
                yield return Wait.Until(() => jets.All(e => e.EngineIgnited && e.finalThrust > 0), 30, "the jets making thrust", shot: false);
                // the window's text is rebuilt now and then, and the intake air moves as the jets spool up: read once the
                // text has caught up with FAR's flight info
                bool CaughtUp()
                {
                    List<double> v = Values(Far.FlightDataText(gui));
                    double frac = gui.InfoParameters.intakeAirFrac * 100;
                    return v.Count == 25 && !double.IsInfinity(frac) && Math.Abs(v[17] - frac) < 0.05 * frac + 0.1;
                }

                yield return Wait.Until(CaughtUp, 30, "the data window's intake air to catch up with FAR's", shot: false);
                List<double> d = Values(Far.FlightDataText(gui));
                double intake = gui.InfoParameters.intakeAirFrac;
                ctx.Record("engineSection", $"fuel fraction {d[15]}, TSFC {d[16]}, intake air {d[17]} %, Ps {d[18]}; FAR's intake air {intake:G4}");
                Assert.Equal(25, d.Count, "values the data window shows with every section on");
                Assert.True(!double.IsInfinity(intake) && intake > 0, $"with the jets lit FAR's intake air is finite ({intake:G4})");
                Assert.True(!double.IsInfinity(d[17]) && d[17] > 0, $"the window shows the intake air as a finite percentage ({d[17]})");
                Assert.Near(intake * 100, d[17], 0.05 * intake * 100 + 0.1, "the intake air shown is FAR's, in percent");
                Assert.True(d[16] > 0, "the jets burn fuel (TSFC)");

                ctx.Step("The engine section off, then on");
                yield return Wait.Frames(3);
                float tall = ((Rect)GetStatic(typeof(FlightGUI), "dataGuiRect")).height;
                yield return Imgui.Toggle(Loc("FARFlightDataOption6"), false, Far.SettingsWindow, exact: true);
                Assert.False(sections[6], "the engine section off");
                yield return Wait.Frames(5);
                float shorter = ((Rect)GetStatic(typeof(FlightGUI), "dataGuiRect")).height;
                int without = Values(Far.FlightDataText(gui)).Count;
                yield return Imgui.Toggle(Loc("FARFlightDataOption6"), true, Far.SettingsWindow, exact: true);
                yield return Wait.Frames(5);
                float back = ((Rect)GetStatic(typeof(FlightGUI), "dataGuiRect")).height;
                int with = Values(Far.FlightDataText(gui)).Count;
                ctx.Record("dataWindow", $"height {tall:F0} -> {shorter:F0} -> {back:F0}; values {without} without the section, {with} with it");
                Assert.Equal(21, without, "the engine section's four lines gone");
                Assert.Equal(25, with, "and back");
                Assert.True(shorter > 0 && shorter < tall - 10, $"the window shrank to fit ({shorter:F0} against {tall:F0} px)");
                Assert.Near(tall, back, 2, "and grew back (px)");

                ctx.Step("Airspeed in mph");
                Ctrl.SetThrottle(0f);
                Far.SetSettingsGroup(gui, Far.GroupAirspeed);
                if (FlightGlobals.speedDisplayMode != FlightGlobals.SpeedDisplayModes.Surface)
                    FlightGlobals.SetSpeedMode(FlightGlobals.SpeedDisplayModes.Surface);
                yield return Imgui.Select(Loc("FARFlightAirspeedGroundspeed"), Far.SettingsWindow);
                yield return Imgui.Select(Loc("FARFlightAirspeedMPH"), Far.SettingsWindow);
                yield return Wait.Frames(2);
                Assert.True(air.GetVelocityDisplayString(out string text, out AirspeedSettingsGUI.SurfaceVelMode mode), "FAR built the speed display's text");
                double tas = FlightGlobals.ActiveVessel.srfSpeed;
                System.Text.RegularExpressions.Match m = System.Text.RegularExpressions.Regex.Match(text ?? "", @"^\s*-?[0-9]+([.,][0-9]+)?");
                double value = double.Parse(m.Value.Trim(), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.CurrentCulture);
                ctx.Record("mph", $"'{text}' at {tas:F2} m/s");
                Assert.Equal(AirspeedSettingsGUI.SurfaceVelMode.TAS, mode, "surface mode");
                Assert.Near(tas * 2.236936, value, 0.01 * value + 0.3, "mph are m/s x 2.2369");
                Assert.True(text.EndsWith(Loc("FARFlightAirspeedMPH")), $"the unit shown ('{text}')");

                ctx.Step("F2 hides FAR's windows, and they come back");
                Assert.True(Imgui.WindowOpen(Far.MainWindow), "FAR's flight window shown");
                GameEvents.onHideUI.Fire();
                hidden = true;
                yield return Imgui.WaitWindowClosed(Far.MainWindow);
                Assert.False(FlightGUI.showAllGUI, "FAR's windows hidden with the UI");
                // the data window too, though not necessarily the same frame (on a busy CI runner it was a frame behind)
                yield return Imgui.WaitWindowClosed(Far.DataWindow, 5);
                GameEvents.onShowUI.Fire();
                hidden = false;
                yield return Imgui.WaitWindow(Far.MainWindow);
                yield return Imgui.WaitWindow(Far.DataWindow);
                Assert.True(FlightGUI.showAllGUI, "FAR's windows back with the UI");
            }
            finally
            {
                if (hidden)
                    GameEvents.onShowUI.Fire();
                FlightGUI.showAllGUI = true;
                Ctrl.SetThrottle(0f);
                foreach (ModuleEngines e in jets)
                    if (e != null)
                        e.Shutdown();
                for (int i = 0; i < sections.Length; ++i)
                    sections[i] = savedSections[i];
                Far.SetAirspeedSetting(air, savedAir);
                Far.SetSubWindow(gui, "showFlightDataWindow", dataOpen);
                Far.SetSubWindow(gui, "showSettingsWindow", settingsOpen);
                Far.SetSettingsGroup(gui, group);
                Far.FlightGuiShown = shown;
            }
        }

        // ------------------------------------------------------------------------------------------ status

        /// <summary>
        /// The flight window's status box. Fast and low (Mach 0.9 at 500 m, over 40 kPa) it warns of the dynamic
        /// pressure. The message a failing part sends its vessel (AerodynamicFailureStatus) puts up the failure
        /// warning, blinking, over the flight's own status for 5 s, after which the flight's status is back. Well past
        /// the stall (40 deg at Mach 0.25) it warns of a heavy stall, blinking.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator FlightGui_StatusShowsHighQStallAndFailure(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FlightGUI gui = Far.FlightGui();
            bool shown = Far.FlightGuiShown;
            object status = Get(gui, "_flightStatusGUI");
            try
            {
                Far.FlightGuiShown = true;
                yield return Imgui.WaitWindow(Far.MainWindow, 30);

                ctx.Step("Mach 0.9 at 500 m");
                FarFlightProbe.Condition fast = Far.At(0.9, 500, 1, "fast and low");
                yield return FlightTests.Airborne(fast);
                yield return FarFlightProbe.Place(fast);
                string highQ = Loc("FARFlightStatus4");
                yield return Wait.Until(() => Far.FlightStatus(gui) == highQ, 10, "the high dynamic pressure status", shot: false);
                ctx.Record("highQ", $"q {gui.InfoParameters.dynPres:F1} kPa, stall {gui.InfoParameters.stallFraction:F3}");
                Assert.True(gui.InfoParameters.dynPres > 40, "over 40 kPa");
                Assert.False(Get<bool>(status, "statusBlinker"), "a steady warning");
                yield return Imgui.WaitWindow(Far.MainWindow, 30);

                ctx.Step("A part's failure message");
                string failed = Loc("FARFlightStatus5");
                FlightGlobals.ActiveVessel.SendMessage("AerodynamicFailureStatus");
                Assert.Equal(failed, Far.FlightStatus(gui), "the failure warning");
                Assert.True(Get<bool>(status, "statusBlinker"), "blinking");
                bool seen = false, blank = false;
                double t0 = Planetarium.GetUniversalTime();
                while (Planetarium.GetUniversalTime() - t0 < 3 && !(seen && blank))
                {
                    yield return null;
                    Assert.Equal(failed, Far.FlightStatus(gui), "the failure warning held over the flight's status");
                    if (Get<double>(status, "statusBlinkerTimer") < 0.5)
                        seen = true;
                    else
                        blank = true;
                }

                Assert.True(seen && blank, $"the warning blinks (shown {seen}, blank {blank})");
                yield return Wait.Until(() => Planetarium.GetUniversalTime() - t0 > 6, 60, "5 s of game time", shot: false);
                yield return FarFlightProbe.Place(fast);
                yield return Wait.Until(() => Far.FlightStatus(gui) != failed, 10, "the flight's own status back", shot: false);
                ctx.Record("after", Far.FlightStatus(gui));

                ctx.Step("40 deg at Mach 0.25");
                FarFlightProbe.Condition stalled = Far.At(0.25, 3000, 40, "stalled");
                yield return FarFlightProbe.Place(stalled);
                string heavy = Loc("FARFlightStatus1");
                yield return Wait.Until(() => Far.FlightStatus(gui) == heavy, 10, "the heavy stall status", shot: false);
                ctx.Record("stall", $"stall fraction {gui.InfoParameters.stallFraction:F3}");
                Assert.True(gui.InfoParameters.stallFraction > 0.5, "most of the wing stalled");
                Assert.True(Get<bool>(status, "statusBlinker"), "a blinking warning");
            }
            finally
            {
                Far.FlightGuiShown = shown;
            }
        }

        // ------------------------------------------------------------------------------------------ IVA

        /// <summary>
        /// In IVA the cockpit's speed display is FAR's (its InternalSpeedFAR in place of stock's): it reads the
        /// airspeed setting's mode as a short caption and FAR's speed text, for EAS and for IAS alike.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator FlightGui_IvaSpeedIsFars(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FlightGUI gui = Far.FlightGui();
            AirspeedSettingsGUI air = gui.airSpeedGUI;
            object[] savedAir = Far.AirspeedSetting(air);
            try
            {
                if (FlightGlobals.speedDisplayMode != FlightGlobals.SpeedDisplayModes.Surface)
                    FlightGlobals.SetSpeedMode(FlightGlobals.SpeedDisplayModes.Surface);
                FarFlightProbe.Condition c = Far.At(0.5, 3000, 3, "cruise M0.5 3");
                yield return FlightTests.Airborne(c);
                Vessel v = FlightGlobals.ActiveVessel;
                Assert.True(v.GetCrewCount() > 0, "a pilot aboard (for IVA)");

                ctx.Step("IVA");
                CameraManager.Instance.SetCameraIVA();
                yield return Wait.Until(() => CameraManager.Instance.currentCameraMode == CameraManager.CameraMode.IVA, 20, "the IVA camera", shot: false);
                yield return Wait.Until(() => UnityEngine.Object.FindObjectsOfType<InternalSpeedFAR>().Length > 0, 20, "the cockpit's FAR speed display", shot: false);
                InternalSpeedFAR display = UnityEngine.Object.FindObjectsOfType<InternalSpeedFAR>()[0];
                foreach ((AirspeedSettingsGUI.SurfaceVelMode mode, string caption) in new[]
                {
                    (AirspeedSettingsGUI.SurfaceVelMode.EAS, Loc("FARFlightInternalAirspeedEquivalent")),
                    (AirspeedSettingsGUI.SurfaceVelMode.IAS, Loc("FARFlightInternalAirspeedIndicated")),
                })
                {
                    Set(air, "velMode", mode);
                    Set(air, "unitMode", AirspeedSettingsGUI.SurfaceVelUnit.M_S);
                    // SetText fills TextMeshPro's buffer, not its text property: the text as laid out
                    string Shown()
                    {
                        display.textObject.text.ForceMeshUpdate();
                        return display.textObject.text.GetParsedText() ?? "";
                    }

                    yield return Wait.Until(() => Shown().StartsWith(caption), 15, $"the cockpit display in {mode}", shot: false);
                    string shown = Shown();
                    Assert.True(air.GetVelocityDisplayString(out string value, out AirspeedSettingsGUI.SurfaceVelMode m), "FAR built the speed text");
                    ctx.Record(mode.ToString(), $"'{shown}', FAR's '{value}'");
                    Assert.Equal(mode, m, "the airspeed setting's mode");
                    Assert.True(shown.StartsWith(caption), $"the cockpit display's caption is {mode}'s ('{shown}')");
                    double a = Leading(shown.Substring(caption.Length)), b = Leading(value);
                    Assert.Near(b, a, 0.02 * Math.Abs(b) + 0.5, "the cockpit shows FAR's speed");
                }
            }
            finally
            {
                Far.SetAirspeedSetting(air, savedAir);
                if (CameraManager.Instance != null && CameraManager.Instance.currentCameraMode != CameraManager.CameraMode.Flight)
                    CameraManager.Instance.SetCameraFlight();
            }
        }

        private static double Leading(string s)
        {
            System.Text.RegularExpressions.Match m = System.Text.RegularExpressions.Regex.Match(s ?? "", @"^\s*-?[0-9]+([.,][0-9]+)?");
            return m.Success ? double.Parse(m.Value.Trim(), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.CurrentCulture) : double.NaN;
        }

        private static Saved Save(object stab, Type type, Array systems)
        {
            ConfigNode node = FARSettingsScenarioModule.FlightGUISettings?.FirstOrDefault(t => t.name == "StabilityAugmentationSettings");
            return new Saved
            {
                Systems = Enumerable.Range(0, systems.Length).Select(i => SystemFields.Select(f => Get(systems.GetValue(i), f)).ToArray()).ToArray(),
                Low = (double)GetStatic(type, "aoALowLim"),
                High = (double)GetStatic(type, "aoAHighLim"),
                Q = (double)GetStatic(type, "scalingDynPres"),
                Templates = GetStatic(type, "systemTemplates"),
                Node = node?.CreateCopy()
            };
        }

        private static void Restore(object stab, Type type, Array systems, Saved saved, bool keepNode)
        {
            for (int i = 0; i < systems.Length; i++)
                for (int f = 0; f < SystemFields.Length; f++)
                    Set(systems.GetValue(i), SystemFields[f], saved.Systems[i][f]);
            SetStatic(type, "aoALowLim", saved.Low);
            SetStatic(type, "aoAHighLim", saved.High);
            SetStatic(type, "scalingDynPres", saved.Q);
            SetStatic(type, "systemTemplates", saved.Templates);
            if (keepNode)
                return;
            List<ConfigNode> list = FARSettingsScenarioModule.FlightGUISettings;
            ConfigNode node = list?.FirstOrDefault(t => t.name == "StabilityAugmentationSettings");
            if (node == null)
                return;
            if (saved.Node == null)
                list.Remove(node);
            else
            {
                node.ClearData();
                saved.Node.CopyTo(node);
            }
        }
    }
}
