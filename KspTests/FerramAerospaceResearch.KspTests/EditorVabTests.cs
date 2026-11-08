using System;
using System.Collections;
using System.Globalization;
using System.Linq;
using System.Text.RegularExpressions;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using KspTest;
using UnityEngine;
using static FerramAerospaceResearch.KspTests.Refl;
using static FerramAerospaceResearch.KspTests.EditorWindow;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's editor window in the VAB, on the stock Kerbal 1 (a rocket with four small fins, its nose up the VAB's y axis),
    /// with AJE loaded (so the Engine Deck tab is offered).
    /// </summary>
    public static class EditorVabTests
    {
        private const string Rocket = "Kerbal 1";

        /// <summary>
        /// The rocket in the VAB, its forward axis up. The Components tab adds up to the static analysis at 100 m/s,
        /// sea level, 2 deg (the parts' Cd and Cm are the craft's), with the VAB's axes. The static tab's AoA arrows
        /// for a sweep from 0 to 10 deg start straight up and lean 10 deg toward the VAB's z. The stability
        /// derivatives are taken on the fins: their area, their area-weighted chord. The Engine Deck finds no engine it
        /// can sample (the solid boosters and liquid engines are no SolverEngines engines) and says so.
        /// </summary>
        [UITest(Kind = "Check", Requires = "AJE", Mods = "AdvancedJetEngine", StartsAt = StartState.Vab, Craft = Rocket, TimeoutSec = 900)]
        public static IEnumerator EditorGui_RocketInTheVab(TestContext ctx)
        {
            try
            {
                Assert.Equal(EditorFacility.VAB, EditorDriver.editorFacility, "the VAB");

                ctx.Step("Components at 100 m/s, sea level, 2 deg");
                yield return OpenWindow("FAREditorModeComponents");
                ResetConditions();
                object comp = Get(Far.Editor, "_components");
                Remember(comp, "speed", "altitude", "aoa", "spoilers");
                yield return Imgui.SetTextAfterLabel("Speed (m/s)", "100", Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStabDerivAlt"), "0", Window);
                yield return Imgui.SetTextAfterLabel("AoA (deg)", "2", Window);
                yield return Far.WaitForEditorVoxels();
                yield return Wait.Frames(2);
                yield return Wait.Until(() => !Get<bool>(comp, "dirty"), 15, "the Components tab to recompute", shot: false);
                string summary = Get<string>(comp, "summary");
                ctx.Record("summary", summary);
                Match m = Regex.Match(summary, @"Mach (?<mach>[-\d.]+), q (?<q>[-\d.]+) kPa; craft Cl (?<cl>[-\d.]+), Cd (?<cd>[-\d.]+), Cm (?<cm>[-\d.]+)");
                Assert.True(m.Success, "the Components summary reads as one: " + summary);
                double mach = Num(m, "mach");
                InstantConditionSimOutput craft = Far.Instant(2, mach);
                ctx.Record("staticAnalysis", $"Cl {craft.Cl:F4} Cd {craft.Cd:F4} Cm {craft.Cm:F4}");
                Assert.True(craft.Cd > 0, "the rocket has drag");
                Assert.Near(craft.Cd, Num(m, "cd"), 1e-4 + 0.01 * craft.Cd, "the parts' Cd against the craft's, nose up the VAB");
                Assert.Near(craft.Cl, Num(m, "cl"), 1e-4 + 0.01 * Math.Abs(craft.Cl), "the parts' Cl against the craft's");
                Assert.Near(craft.Cm, Num(m, "cm"), 1e-4 + 0.01 * Math.Abs(craft.Cm), "the parts' Cm against the craft's");

                ctx.Step("Static analysis: the AoA arrows for a sweep from 0 to 10 deg");
                SelectTab("FAREditorModeStatic");
                yield return Wait.Frames(2);
                object graphGui = Get(Far.Editor, "_editorGraph");
                object inputs = Get(graphGui, "aoASweepInputs");
                Remember(graphGui, "isMachMode");
                Remember(inputs, "lowerBound", "upperBound", "numPts", "pitchSetting", "otherInput", "spoilers");
                if (Get<bool>(graphGui, "isMachMode"))
                    yield return Imgui.Click(L("FAREditorStaticSwitchAoA"), Window);
                yield return Inviscid();
                yield return Imgui.SetTextAfterLabel(L("FAREditorStaticPitchSetting"), "0", Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStaticGraphLowLim"), "0", Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStaticGraphUpLim"), "10", Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStaticGraphPtCount"), "5", Window);
                yield return Imgui.SetTextAfterLabel(L("FARAbbrevMach"), "0.3", Window);
                yield return Imgui.Click(L("FAREditorStaticSweepAoA"), Window);
                Vector3 lower = Get<Vector3>(graphGui, "lowerAoAVec"), upper = Get<Vector3>(graphGui, "upperAoAVec");
                ctx.Record("arrows", $"lower {lower}, upper {upper}");
                double r = 10 * Math.PI / 180;
                Assert.True((lower - Vector3.up).magnitude < 1e-5, $"at 0 deg the arrow points up the VAB ({lower})");
                Assert.True((upper - new Vector3(0, (float)Math.Cos(r), (float)Math.Sin(r))).magnitude < 1e-5, $"at 10 deg it leans 10 deg toward z ({upper})");

                ctx.Step("Stability derivatives at 0.5 km, Mach 0.35");
                SelectTab("FAREditorModeDataStab");
                yield return Wait.Frames(2);
                object stab = Get(Far.Editor, "_stabDeriv");
                Remember(stab, "altitude", "machNumber", "spoilersDeployed");
                yield return Inviscid();
                yield return Imgui.SetTextAfterLabel(L("FAREditorStabDerivAlt"), "0.5", Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStabDerivMach"), "0.35", Window);
                var sim = (EditorSimManager)Get(Far.Editor, "_simManager");
                StabilityDerivOutput before = sim.vehicleData;
                yield return Imgui.Click(L("FAREditorStabDerivCalcButton"), Window);
                StabilityDerivOutput o = sim.vehicleData;
                Assert.True(o != null && !ReferenceEquals(o, before), "the Calculate button gave derivatives");
                var fins = EditorLogic.fetch.ship.parts.Select(p => p.Modules.GetModule<ferram4.FARWingAerodynamicModel>()).Where(w => w != null && !w.GetShielded()).ToList();
                double s = fins.Sum(w => w.S), mac = fins.Sum(w => w.GetMAC() * w.S) / s;
                ctx.Record("reference", $"S {o.area:F4} m2, MAC {o.MAC:F3} m, b {o.b:F3}; {fins.Count} fins, {s:F4} m2, chord {mac:F3} m");
                Assert.True(fins.Count >= 2, "the rocket's fins");
                Assert.Near(s, o.area, 1e-9 + 1e-9 * s, "the reference area is the fins' (m2)");
                Assert.Near(mac, o.MAC, 1e-6 * mac, "the reference chord is the fins' area-weighted chord (m)");
                Assert.True(o.stabDerivs.Skip(3).Take(21).All(d => !double.IsNaN(d) && !double.IsInfinity(d)), "finite derivatives");

                ctx.Step("Engine Deck: no engine to sample");
                SelectTab("FAREditorModeEngineDeck");
                yield return Wait.Frames(2);
                object deckGui = Get(Far.Editor, "_engineDeck");
                Remember(deckGui, "_samples", "_statusMessage");
                yield return Imgui.Click(L("FAREngineDeckRun"), Window, exact: true);
                Assert.Equal(L("FAREngineDeckNoEngines"), Get<string>(deckGui, "_statusMessage").Trim(), "the Engine Deck's status");
                yield return Wait.Frames(2);
                Assert.True(Imgui.IsVisible(L("FAREngineDeckNoEngines"), Window), "the tab says there is no engine to sample");
            }
            finally
            {
                CloseWindow();
            }
        }

        /// <summary>
        /// The stability derivatives of a craft with no wing to take the reference from are taken on its body: the
        /// reference area its largest cross-section, the reference chord its length, a unit span. Here the Aeris 4A's
        /// derivatives with its wings left out of the reference.
        /// Regression: before the fix, StabilityDerivCalculator.CalculateStabilityDerivs sets MAC = _bodyLength and b = 1 for a
        /// wingless craft, then divides both by the area as for the wings' area-weighted sums, so the chord comes out
        /// as length / area (m^-1) and the span as 1 / area.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 600)]
        public static IEnumerator Editor_WinglessReferenceIsTheBody(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            var sim = (EditorSimManager)Get(Far.Editor, "_simManager");
            StabilityDerivCalculator calc = sim.StabDerivCalculator;
            StabilityDerivCalculator.VehicleProperties props = calc.ComputeVehicleProperties(0, false);
            var wingless = new StabilityDerivCalculator.VehicleProperties(props.CoM, props.Mass, props.Ix, props.Iy, props.Iz, props.Ixy, props.Iyz,
                                                                          props.Ixz, new ferram4.FARWingAerodynamicModel[0]);
            CelestialBody body = FlightGlobals.GetHomeBody();
            GasProperties gas = FARAtmosphere.GetGasProperties(body, new Vector3d(0, 0, 500), Planetarium.GetUniversalTime());
            FARAeroUtil.UpdateCurrentActiveBody(body);
            StabilityDerivOutput o = calc.CalculateStabilityDerivs(wingless, body, 500, 0.35, gas.Density, gas.SpeedOfSound, 0, false, 0, 0, 0);
            InstantConditionSim instant = Far.EditorSim;
            ctx.Record("reference", $"S {o.area:F4} m2, MAC {o.MAC:F4} m, b {o.b:F4}; body {instant._maxCrossSectionFromBody:F4} m2, {instant._bodyLength:F3} m long");
            Assert.Near(instant._maxCrossSectionFromBody, o.area, 1e-9, "the reference area is the body's largest cross-section (m2)");
            Assert.Near(instant._bodyLength, o.MAC, 1e-9, "the reference chord is the body's length (m)");
            Assert.Near(1, o.b, 1e-12, "a unit span (m)");
        }

        private static double Num(Match m, string group) => double.Parse(m.Groups[group].Value, CultureInfo.InvariantCulture);
    }
}
