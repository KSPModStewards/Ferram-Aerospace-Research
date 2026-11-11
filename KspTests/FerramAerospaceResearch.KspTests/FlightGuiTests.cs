using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using ferram4;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using KSP.Localization;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's flight windows on a stock plane held at a condition, driven through their own controls: what the flight
    /// data window shows against the flight, the airspeed setting's conversions, the wing leveler against a banked
    /// plane, the aero visualization's tints and the flight log's file.
    /// </summary>
    public static class FlightGuiTests
    {

        private static FarFlightProbe.Condition Cruise(double aoaDeg, double mach = 0.5, double altM = 3000) =>
            Far.At(mach, altM, aoaDeg, $"cruise M{mach} {aoaDeg}");

        private static string Loc(string tag) => Localizer.Format(tag);

        /// <summary>Opens FAR's flight window, as its toolbar button would.</summary>
        private static IEnumerator OpenFlightWindow()
        {
            Far.FlightGuiShown = true;
            yield return Imgui.WaitWindow(Far.MainWindow, 30);
        }

        /// <summary>Opens the settings window from the flight window and picks one of its groups.</summary>
        private static IEnumerator OpenSettings(FlightGUI gui, int group)
        {
            yield return Imgui.Toggle(Loc("FARFlightGUIFltSettings"), true, Far.MainWindow, exact: true);
            yield return Imgui.WaitWindow(Far.SettingsWindow);
            Far.SetSettingsGroup(gui, group);
            yield return null;
        }

        /// <summary>Which of FAR's flight windows were open, to put back after a test (tests share the game).</summary>
        private sealed class Windows
        {
            private readonly FlightGUI gui;
            private readonly bool shown, data, settings;
            private readonly int group;

            public Windows(FlightGUI gui)
            {
                this.gui = gui;
                shown = Far.FlightGuiShown;
                data = Far.SubWindowOpen(gui, "showFlightDataWindow");
                settings = Far.SubWindowOpen(gui, "showSettingsWindow");
                group = Far.SettingsGroup(gui);
            }

            public void Restore()
            {
                try
                {
                    Far.SetSubWindow(gui, "showFlightDataWindow", data);
                    Far.SetSubWindow(gui, "showSettingsWindow", settings);
                    Far.SetSettingsGroup(gui, group);
                }
                catch (Exception e)
                {
                    KspTest.Log.Warn("FAR's flight windows not restored: " + e.Message);
                }

                Far.FlightGuiShown = shown;
            }
        }

        private static readonly Regex Number = new Regex(@"^\s*(-?[0-9]+(\.[0-9]+)?|NaN|-?Infinity)");

        /// <summary>The numbers at the start of each line of FAR's value column (its own formatting: '.' decimals).</summary>
        private static List<double> Values(string text)
        {
            var values = new List<double>();
            foreach (string line in text.Split('\n'))
            {
                Match m = Number.Match(line);
                if (m.Success)
                    values.Add(double.Parse(m.Groups[1].Value, NumberStyles.Float, CultureInfo.InvariantCulture));
            }

            return values;
        }

        /// <summary>The number a speed display string starts with (ToString("F1") in the current culture).</summary>
        private static double Leading(string s)
        {
            Match m = Regex.Match(s ?? "", @"^\s*-?[0-9]+([.,][0-9]+)?");
            return m.Success ? double.Parse(m.Value.Trim(), NumberStyles.Float, CultureInfo.CurrentCulture) : double.NaN;
        }

        /// <summary>Bank angle (deg) from the vessel's own axes: the right wing's climb above the local horizontal.</summary>
        private static double Bank(Vessel v)
        {
            Vector3d up = ((Vector3d)v.CoM - v.mainBody.position).normalized;
            return Math.Asin(Math.Max(-1, Math.Min(1, Vector3d.Dot((Vector3d)v.ReferenceTransform.right, up)))) * 180 / Math.PI;
        }

        /// <summary>
        /// The flight data window with every section switched on (through the settings window's toggles), read while
        /// the plane is held at 5 deg: the attitude, air data, forces and coefficients it shows against the flight
        /// itself, and against each other (Cl q S is the lift, the ballistic coefficient is m / (Cd S), ...).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator FlightGui_DataWindowShowsTheFlight(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FlightGUI gui = Far.FlightGui();
            var windows = new Windows(gui);
            bool[] sections = Far.FlightDataSections(gui);
            var savedSections = (bool[])sections.Clone();
            try
            {
                ctx.Step("open the flight window and its data window");
                yield return OpenFlightWindow();
                yield return Imgui.Toggle(Loc("FARFlightGUIFltDataBtn"), true, Far.MainWindow, exact: true);
                yield return Imgui.WaitWindow(Far.DataWindow);

                ctx.Step("switch every data section on");
                yield return OpenSettings(gui, Far.GroupFlightData);
                for (int i = 0; i < sections.Length; ++i)
                    yield return Imgui.Toggle(Loc("FARFlightDataOption" + i), true, Far.SettingsWindow, exact: true);
                Assert.True(sections.All(s => s), "every flight data section is on after its toggle");

                ctx.Step("held at Mach 0.5, 3 km, 5 deg");
                FarFlightProbe.Condition c = Cruise(5);
                yield return FlightTests.Airborne(c);
                yield return Snap.Shot(ctx, "flight-data");
                var samples = new List<FarFlightProbe.Sample>();
                yield return FarFlightProbe.Hold(c, 25, 10, samples, predict: false);

                Vessel v = FlightGlobals.ActiveVessel;
                VesselFlightInfo info = gui.InfoParameters;
                string text = Far.FlightDataText(gui);
                List<double> d = Values(text);
                ctx.Record("dataText", text.Replace("\n", " | "));
                // PYR 3, AoA + sideslip 2, q 1, forces 3, coefficients + area 4, L/D 2, engine 4, range 4, BC + Vt 2
                Assert.Equal(25, d.Count, "values the data window shows with every section on");
                double pitch = d[0], heading = d[1], roll = d[2], aoa = d[3], beta = d[4], q = d[5];
                double lift = d[6], drag = d[7], side = d[8], cl = d[9], cd = d[10], refArea = d[12];
                double ld = d[13], vld = d[14], tsfc = d[16], fuelBurn = d[21], bc = d[23], vTerm = d[24];

                // the air and the attitude, against the vessel's own state
                double qTrue = 0.5 * v.atmDensity * v.srfSpeed * v.srfSpeed / 1000;
                double gamma = Math.Asin(v.verticalSpeed / v.srfSpeed) * 180 / Math.PI;
                ctx.Record("attitude", $"pitch {pitch} heading {heading} roll {roll} aoa {aoa} beta {beta}; flight path {gamma:F2}");
                Assert.Near(qTrue, q, 0.03 * qTrue, "dynamic pressure shown against 1/2 rho v^2 (kPa)");
                Assert.Near(c.AoADeg, aoa, 1.0, "angle of attack shown against the one held");
                Assert.Near(0, beta, 1.0, "no sideslip shown, wings level and the nose on the heading");
                Assert.Near(aoa + gamma, pitch, 1.5, "pitch shown: the angle of attack plus the flight path angle");
                Assert.Near(c.HeadingDeg, heading, 2.0, "heading shown: east, as placed");
                Assert.Near(0, roll, 1.5, "roll shown: wings level, as held");

                // the forces and the coefficients the window derives from them
                double wingArea = v.parts.SelectMany(p => p.Modules.OfType<FARWingAerodynamicModel>()).Sum(w => w.S);
                ctx.Record("forces", $"lift {lift} drag {drag} side {side} kN; Cl {cl} Cd {cd} S {refArea} (wings {wingArea:F3})");
                Assert.True(lift > 0 && drag > 0 && cl > 0 && cd > 0, "positive lift and drag, and their coefficients, at 5 deg");
                Assert.True(Math.Abs(side) < 0.05 * lift, $"little side force ({side} kN) on a symmetric plane without sideslip");
                Assert.Near(wingArea, refArea, 0.01 * wingArea + 0.002, "the reference area is the wings' area");
                Assert.Near(lift, cl * q * refArea, 0.01 * lift + 0.01, "Cl q S is the lift shown");
                Assert.Near(drag, cd * q * refArea, 0.01 * drag + 0.01, "Cd q S is the drag shown");
                Assert.Near(lift / drag, ld, 0.01 * ld + 0.002, "L/D is lift over drag");
                Assert.Near(v.srfSpeed * ld, vld, 0.02 * vld + 0.01, "V L/D is the airspeed times L/D");

                // the engines are off: no fuel burnt, no range
                Assert.True(tsfc == 0 && fuelBurn == 0, $"no TSFC ({tsfc}) or fuel burn ({fuelBurn}) with the engines off");

                // ballistic coefficient and terminal velocity from the mass, the drag and the air
                double g = FlightGlobals.getGeeForceAtPosition(v.CoM).magnitude;
                double bcTrue = info.fullMass * 1000 / (cd * refArea);
                ctx.Record("ballistic", $"BC {bc} kg/m2 (m {info.fullMass:F3} t, vessel {v.GetTotalMass():F3} t), Vt {vTerm} m/s");
                Assert.Near(v.GetTotalMass(), info.fullMass, 0.03 * v.GetTotalMass(), "the mass FAR uses is the vessel's");
                Assert.Near(bcTrue, bc, 0.015 * bcTrue, "BC is m / (Cd S)");
                Assert.Near(Math.Sqrt(2 * bc * g / v.atmDensity), vTerm, 0.015 * vTerm, "terminal velocity is sqrt(2 BC g / rho)");

                // what the window shows is what FARAPI gives other mods
                Assert.Near(FARAPI.VesselAoA(v), aoa, 0.2, "the AoA shown is FARAPI's");
                Assert.Near(FARAPI.VesselLiftCoeff(v), cl, 0.02 * cl, "the Cl shown is FARAPI's");
                Assert.Near(FARAPI.VesselBallisticCoeff(v), bc, 0.02 * bc, "the BC shown is FARAPI's");
                Assert.Near(FARAPI.VesselTermVelEst(v), vTerm, 0.02 * vTerm, "the terminal velocity shown is FARAPI's");
            }
            finally
            {
                for (int i = 0; i < sections.Length; ++i)
                    sections[i] = savedSections[i];
                windows.Restore();
            }
        }

        /// <summary>A speed display reading: what FAR shows and the flight at that moment.</summary>
        private sealed class SpeedShown
        {
            public string Text;
            public double Value, Tas, Mach, Rho, PressurePa, Ias, Eas;
            public AirspeedSettingsGUI.SurfaceVelMode Mode;
        }

        /// <summary>Places the plane at c, picks the mode and unit on the airspeed grid, and reads the speed shown.</summary>
        private static IEnumerator ShowSpeed(AirspeedSettingsGUI air, FarFlightProbe.Condition c, string mode, string unit, SpeedShown into)
        {
            yield return FarFlightProbe.Place(c);
            yield return Imgui.Select(mode, Far.SettingsWindow);
            yield return Imgui.Select(unit, Far.SettingsWindow);
            yield return FarFlightProbe.Hold(c, 5, 1, new List<FarFlightProbe.Sample>(), predict: false);
            Vessel v = FlightGlobals.ActiveVessel;
            Assert.True(air.GetVelocityDisplayString(out into.Text, out into.Mode),
                        "FAR built the speed display's text (it needs the stock speed display, in surface mode)");
            into.Value = Leading(into.Text);
            into.Tas = v.srfSpeed;
            into.Mach = v.mach;
            into.Rho = v.atmDensity;
            into.PressurePa = v.staticPressurekPa * 1000;
            into.Ias = air.CalculateIAS();
            into.Eas = air.CalculateEAS();
        }

        /// <summary>
        /// The airspeed setting's grid switches the navball speed between surface (true) airspeed, IAS, EAS and Mach, in
        /// m/s, knots or km/h: each reading against the flight - EAS is TAS sqrt(rho / rho0), IAS is the pitot
        /// (compressible) speed from the impact pressure, below TAS and a little above EAS at altitude.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator FlightGui_AirspeedSettingChangesTheSpeedShown(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FlightGUI gui = Far.FlightGui();
            var windows = new Windows(gui);
            AirspeedSettingsGUI air = null;
            object[] savedAir = null;
            try
            {
                ctx.Step("open the airspeed settings");
                yield return OpenFlightWindow();
                yield return OpenSettings(gui, Far.GroupAirspeed);
                yield return Imgui.WaitVisible(Loc("FARFlightAirspeedIndicated"), Far.SettingsWindow);
                air = gui.airSpeedGUI;
                Assert.NotNull(air, "the vessel's airspeed settings");
                savedAir = Far.AirspeedSetting(air);
                if (FlightGlobals.speedDisplayMode != FlightGlobals.SpeedDisplayModes.Surface)
                    FlightGlobals.SetSpeedMode(FlightGlobals.SpeedDisplayModes.Surface);

                FarFlightProbe.Condition c = Cruise(3);
                yield return FlightTests.Airborne(c);

                string ms = Loc("FARFlightAirspeedMeterPerSec");
                var tas = new SpeedShown();
                var ias = new SpeedShown();
                var eas = new SpeedShown();
                var mach = new SpeedShown();
                var knots = new SpeedShown();
                var kmh = new SpeedShown();
                ctx.Step("surface speed");
                yield return ShowSpeed(air, c, Loc("FARFlightAirspeedGroundspeed"), ms, tas);
                ctx.Step("IAS");
                yield return ShowSpeed(air, c, Loc("FARFlightAirspeedIndicated"), ms, ias);
                ctx.Step("EAS");
                yield return ShowSpeed(air, c, Loc("FARFlightAirspeedEquivalent"), ms, eas);
                ctx.Step("Mach");
                yield return ShowSpeed(air, c, Loc("FARAbbrevMach"), ms, mach);
                ctx.Step("knots and km/h");
                yield return ShowSpeed(air, c, Loc("FARFlightAirspeedGroundspeed"), Loc("FARFlightAirspeedKnots"), knots);
                yield return ShowSpeed(air, c, Loc("FARFlightAirspeedGroundspeed"), Loc("FARFlightAirspeedKMH"), kmh);
                foreach ((string name, SpeedShown s) in new[] { ("tas", tas), ("ias", ias), ("eas", eas), ("mach", mach), ("knots", knots), ("kmh", kmh) })
                    ctx.Record(name, $"'{s.Text}' ({s.Mode}) at TAS {s.Tas:F2} m/s, Mach {s.Mach:F4}, rho {s.Rho:F4}");

                Assert.Equal(AirspeedSettingsGUI.SurfaceVelMode.TAS, tas.Mode, "surface mode");
                Assert.Equal(AirspeedSettingsGUI.SurfaceVelMode.IAS, ias.Mode, "IAS mode");
                Assert.Equal(AirspeedSettingsGUI.SurfaceVelMode.EAS, eas.Mode, "EAS mode");
                Assert.Equal(AirspeedSettingsGUI.SurfaceVelMode.MACH, mach.Mode, "Mach mode");

                Assert.Near(tas.Tas, tas.Value, 0.003 * tas.Tas + 0.15, "surface mode shows the airspeed");
                Assert.Near(eas.Tas * Math.Sqrt(eas.Rho / 1.225), eas.Value, 0.005 * eas.Value + 0.15, "EAS is TAS sqrt(rho / 1.225)");
                Assert.Near(mach.Mach, mach.Value, 0.003, "Mach mode shows the Mach number");

                // IAS from the impact pressure q_c = p ((1 + (k-1)/2 M^2)^(k/(k-1)) - 1), as a pitot-static system reads it
                double k = FlightGlobals.ActiveVessel.mainBody.atmosphereAdiabaticIndex;
                double qc = ias.PressurePa * (Math.Pow(1 + (k - 1) / 2 * ias.Mach * ias.Mach, k / (k - 1)) - 1);
                double iasTrue = Math.Sqrt(2 * qc / 1.225);
                ctx.Record("iasTrue", iasTrue);
                Assert.Near(iasTrue, ias.Value, 0.01 * iasTrue + 0.15, "IAS from the impact pressure");
                Assert.Near(ias.Ias, ias.Value, 0.003 * ias.Ias + 0.15, "IAS shown is FAR's (FARAPI.VesselIAS)");
                // at 3 km the air is thinner than at sea level: EAS < IAS (compressibility) < TAS
                Assert.True(eas.Eas < ias.Ias && ias.Ias < ias.Tas,
                            $"EAS ({eas.Eas:F1}) < IAS ({ias.Ias:F1}) < TAS ({ias.Tas:F1}) at altitude");

                Assert.Near(knots.Tas * 1.943844, knots.Value, 0.003 * knots.Value + 0.15, "knots are m/s x 1.9438");
                Assert.Near(kmh.Tas * 3.6, kmh.Value, 0.003 * kmh.Value + 0.15, "km/h are m/s x 3.6");
                Assert.True(knots.Text.EndsWith(Loc("FARFlightAirspeedKnots")) && kmh.Text.EndsWith(Loc("FARFlightAirspeedKMH")),
                            $"the unit is shown ('{knots.Text}', '{kmh.Text}')");
            }
            finally
            {
                if (air != null && savedAir != null)
                    Far.SetAirspeedSetting(air, savedAir);
                windows.Restore();
            }
        }

        /// <summary>
        /// Banks the plane by bankDeg about its velocity just after placing it, then lets it fly hands off, recording
        /// [time, bank from the vessel's axes, FAR's roll angle, roll input] every physics frame.
        /// </summary>
        private static IEnumerator BankedFlight(FarFlightProbe.Condition c, double bankDeg, double seconds, List<double[]> trace)
        {
            yield return FarFlightProbe.Place(c);
            Vessel v = FlightGlobals.ActiveVessel;
            Vector3 vel = ((Vector3)v.srf_velocity).normalized;
            v.SetRotation(Quaternion.AngleAxis((float)bankDeg, vel) * v.transform.rotation);
            foreach (Part p in v.parts)
                if (p.Rigidbody != null)
                    p.Rigidbody.angularVelocity = Vector3.zero;
            var fixedWait = new WaitForFixedUpdate();
            double t0 = Planetarium.GetUniversalTime();
            while (Planetarium.GetUniversalTime() - t0 < seconds)
            {
                yield return fixedWait;
                v = FlightGlobals.ActiveVessel;
                trace.Add(new[] { Planetarium.GetUniversalTime() - t0, Bank(v), Far.FlightGui(v).InfoParameters.rollAngle, v.ctrlState.roll });
            }
        }

        /// <summary>
        /// The flight window's Roll toggle (the wing leveler): the plane banked 30 deg and let go keeps much of its
        /// bank on its own; with the leveler on, FAR rolls it back toward wings level. FAR's roll angle reads the bank.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator FlightGui_WingLevelerRightsTheBank(TestContext ctx)
        {
            const double bank = 30, seconds = 10;
            yield return Wait.Seconds(5);
            FlightGUI gui = Far.FlightGui();
            var windows = new Windows(gui);
            // every system off (a saved setting could have one on); the roll system is switched on below
            bool[] savedStab = Enumerable.Range(0, 5).Select(i => Far.StabAugActive(gui, i)).ToArray();
            try
            {
                yield return OpenFlightWindow();
                for (int i = 0; i < savedStab.Length; ++i)
                    Far.SetStabAug(gui, i, false);
                FarFlightProbe.Condition c = Cruise(3);
                yield return FlightTests.Airborne(c);

                ctx.Step("banked, hands off, leveler off");
                var off = new List<double[]>();
                yield return BankedFlight(c, bank, seconds, off);

                ctx.Step("banked, hands off, leveler on");
                yield return Imgui.Toggle(Loc("FARFlightStabAugLabel0"), true, Far.MainWindow, exact: true);
                Assert.True(Far.StabAugActive(gui, 0), "the Roll toggle switched the wing leveler on");
                var on = new List<double[]>();
                yield return BankedFlight(c, bank, seconds, on);

                ctx.Step("the leveler's gains in the settings window");
                yield return OpenSettings(gui, Far.GroupStabilityAugmentation);
                yield return Imgui.WaitVisible(Loc("FARFlightStabPropGain"), Far.SettingsWindow);

                double start = Math.Abs(on[3][1]), farStart = Math.Abs(on[3][2]);
                double endOff = Math.Abs(off[off.Count - 1][1]), endOn = Math.Abs(on[on.Count - 1][1]);
                double maxInputOff = off.Max(r => Math.Abs(r[3]));
                double inputOn = on.Where(r => r[0] < 2).Average(r => r[3] * Math.Sign(r[1]));
                ctx.Record("bankStart", $"{start:F1} deg (FAR roll {farStart:F1})");
                ctx.Record("bankEnd", $"off {endOff:F1} deg, on {endOn:F1} deg after {seconds} s");
                ctx.Record("rollInput", $"off max {maxInputOff:F3}; on, first 2 s, mean along the bank {inputOn:F3}");

                Assert.Near(bank, start, 3, "the bank given");
                Assert.Near(start, farStart, 2, "FAR's roll angle reads the bank");
                Assert.True(maxInputOff < 0.01, $"no roll input without the leveler ({maxInputOff:F3})");
                // (its ailerons' lift linear in their deflection, FARDebugValues.linearFlapLift, it needs less of them)
                Assert.True(Math.Abs(inputOn) > 0.02, $"the leveler puts in roll ({inputOn:F3} along the bank)");
                Assert.True(endOn < 10 && endOn < 0.5 * endOff,
                            $"the leveler brings the bank back toward level: {endOn:F1} deg with it, {endOff:F1} deg without");
            }
            finally
            {
                for (int i = 0; i < savedStab.Length; ++i)
                    Far.SetStabAug(gui, i, savedStab[i]);
                windows.Restore();
            }
        }

        /// <summary>
        /// The aero visualization's tints, switched from the settings window while the plane is held at 6 deg: with
        /// Tint Cl a wing turns blue by its lift (alpha 0.5), with Tint Cd red by its drag, and with neither the tint
        /// clears.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator FlightGui_AeroTintColoursTheParts(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FlightGUI gui = Far.FlightGui();
            var windows = new Windows(gui);
            AeroVisualizationGUI viz = gui.AeroVizGUI;
            Assert.NotNull(viz, "the flight GUI's aero visualization");
            bool[] savedTints = { viz.TintForCl, viz.TintForCd, viz.TintForStall };
            try
            {
                yield return OpenFlightWindow();
                yield return OpenSettings(gui, Far.GroupAeroViz);
                FarFlightProbe.Condition c = Cruise(6);
                yield return FlightTests.Airborne(c);
                Vessel v = FlightGlobals.ActiveVessel;
                Part wing = v.parts.FirstOrDefault(p => p.Modules.OfType<FARWingAerodynamicModel>().Any(w => !(w is FARControllableSurface)));
                Assert.NotNull(wing, "a fixed wing on the plane");
                Part body = v.rootPart;
                string tintCl = Loc("FARFlightAeroVizTintCl"), tintCd = Loc("FARFlightAeroVizTintCd");

                ctx.Step("Tint Cl");
                yield return Imgui.Toggle(tintCl, true, Far.SettingsWindow, exact: true);
                yield return Imgui.Toggle(tintCd, false, Far.SettingsWindow, exact: true);
                yield return FarFlightProbe.Place(c);
                yield return FarFlightProbe.Hold(c, 0, 15, new List<FarFlightProbe.Sample>(), predict: false);
                Color wingCl = Far.PartTint(wing), bodyCl = Far.PartTint(body);
                ctx.Record("tintCl", $"wing {wingCl}, body {bodyCl}");
                Assert.True(viz.TintForCl && !viz.TintForCd, "the toggles set Tint Cl on, Tint Cd off");
                Assert.Near(0.5, wingCl.a, 0.01, "a tinted wing's alpha");
                Assert.True(wingCl.b > 0.1, $"a lifting wing is tinted blue ({wingCl.b:F3})");
                Assert.True(wingCl.r < 0.01, $"no red without Tint Cd ({wingCl.r:F3})");

                yield return Snap.Shot(ctx, "tint-cl");
                ctx.Step("Tint Cd");
                yield return Imgui.Toggle(tintCl, false, Far.SettingsWindow, exact: true);
                yield return Imgui.Toggle(tintCd, true, Far.SettingsWindow, exact: true);
                yield return FarFlightProbe.Place(c);
                yield return FarFlightProbe.Hold(c, 0, 15, new List<FarFlightProbe.Sample>(), predict: false);
                Color wingCd = Far.PartTint(wing), bodyCd = Far.PartTint(body);
                ctx.Record("tintCd", $"wing {wingCd}, body {bodyCd}");
                Assert.True(wingCd.r > 0.01 && bodyCd.r > 0.01, $"wing ({wingCd.r:F3}) and body ({bodyCd.r:F3}) tinted red by their drag");
                Assert.True(wingCd.b < 0.01, $"no blue without Tint Cl ({wingCd.b:F3})");

                ctx.Step("no tint");
                yield return Imgui.Toggle(tintCd, false, Far.SettingsWindow, exact: true);
                yield return FarFlightProbe.Hold(c, 0, 5, new List<FarFlightProbe.Sample>(), predict: false);
                Color wingNone = Far.PartTint(wing);
                Assert.True(wingNone.a < 0.01, $"the tint clears with every tint off (alpha {wingNone.a:F3})");
            }
            finally
            {
                Far.SetTints(viz, savedTints[0], savedTints[1], savedTints[2]);
                windows.Restore();
            }
        }

        /// <summary>
        /// The flight window's Flt Log toggle, with the log period typed in its field: FAR writes a CSV of the flight
        /// (time, altitude, speed, forces, coefficients, angles) at that period, which matches the flight held.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = Far.Plane, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator FlightGui_FlightLogRecordsTheFlight(TestContext ctx)
        {
            yield return Wait.Seconds(5);
            FlightGUI gui = Far.FlightGui();
            var windows = new Windows(gui);
            FlightDataLogger logger = Far.Logger(gui);
            int savedPeriod = logger.Period;
            string file = null;
            bool existed = true;
            try
            {
                yield return OpenFlightWindow();
                ctx.Step("log period 100 ms");
                yield return Imgui.SetTextAfterLabel(Loc("FARFlightGUIFltLogPeriod"), "100", Far.MainWindow);
                yield return null;
                Assert.Equal(100, logger.Period, "the log period typed in the flight window");

                FarFlightProbe.Condition c = Cruise(4);
                yield return FlightTests.Airborne(c);

                ctx.Step("log 3 s held at 4 deg");
                file = logger.FileName;
                existed = File.Exists(file);
                int linesBefore = existed ? File.ReadAllLines(file).Length : 0;
                yield return Imgui.Toggle(Loc("FARFlightGUIFltLogging"), true, Far.MainWindow, exact: true);
                Assert.True(logger.IsActive, "the Flt Log toggle started the logger");
                var samples = new List<FarFlightProbe.Sample>();
                yield return FarFlightProbe.Hold(c, 0, 150, samples, predict: false);
                yield return Imgui.Toggle(Loc("FARFlightGUIFltLogging"), false, Far.MainWindow, exact: true);
                Assert.False(logger.IsActive, "the Flt Log toggle stopped the logger");

                ctx.Step("read the log");
                string[] lines = File.ReadAllLines(file).Skip(linesBefore).ToArray();
                ctx.Record("file", file);
                ctx.Record("lines", lines.Length);
                // a new file: "sep=, ", the header, then a row per period; each value is followed by ", "
                var rows = new List<double[]>();
                string[] header = null;
                foreach (string line in lines)
                {
                    if (line.StartsWith("sep=") || line.Trim().Length == 0) continue;
                    string[] cells = line.Split(new[] { ", " }, StringSplitOptions.None).Select(s => s.Trim()).Where(s => s.Length > 0).ToArray();
                    if (header == null && cells.Length > 0 && cells[0] == "time")
                    {
                        header = cells;
                        continue;
                    }

                    rows.Add(cells.Select(s => double.TryParse(s, NumberStyles.Float, CultureInfo.CurrentCulture, out double x) ? x : double.NaN).ToArray());
                }

                if (!existed)
                    Assert.True(header != null && header.Length == 18, $"the log's header ({(header == null ? "none" : string.Join("|", header))})");
                Assert.True(rows.Count >= 20, $"a row every 100 ms over 3 s ({rows.Count})");
                Assert.True(rows.All(r => r.Length == 18 && r.All(x => !double.IsNaN(x) && !double.IsInfinity(x))),
                            "every row has 18 numbers");
                var dts = new List<double>();
                for (int i = 1; i < rows.Count; ++i)
                    dts.Add(rows[i][0] - rows[i - 1][0]);
                ctx.Record("period", $"mean {dts.Average():F4} s, min {dts.Min():F4}, max {dts.Max():F4}");
                Assert.True(dts.All(dt => dt > 0), "time rises row by row");
                Assert.InRange(0.095, 0.13, dts.Average(), "rows 100 ms apart (to the physics frame)");

                double speed = samples.Average(s => s.Airspeed), alt = samples.Average(s => s.AltM), q = samples.Average(s => s.QkPa);
                Assert.Near(alt, rows.Average(r => r[1]), 100, "the altitude logged");
                Assert.Near(speed, rows.Average(r => r[2]), 0.05 * speed, "the speed logged");
                Assert.True(rows.Average(r => r[3]) > 0 && rows.Average(r => r[4]) > 0, "lift and drag logged positive at 4 deg");
                Assert.Near(q, rows.Average(r => r[6]), 0.05 * q, "the dynamic pressure logged");
                Assert.Near(c.AoADeg, rows.Average(r => r[12]), 1.0, "the angle of attack logged");
                Assert.Near(c.HeadingDeg, rows.Average(r => r[15]), 2.0, "the heading logged");
            }
            finally
            {
                if (logger != null)
                {
                    if (logger.IsActive) logger.StopLogging();
                    logger.Period = savedPeriod;
                }

                if (file != null && !existed)
                    try { File.Delete(file); }
                    catch (Exception e) { KspTest.Log.Warn("flight log not deleted: " + e.Message); }
                windows.Restore();
            }
        }
    }
}
