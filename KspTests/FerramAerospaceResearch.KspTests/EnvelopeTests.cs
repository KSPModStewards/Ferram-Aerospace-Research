using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using KSP.Localization;
using KspTest;
using UnityEngine;
using Cell = FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation.PerformanceEnvelopeCalculator.CellResult;
using static FerramAerospaceResearch.KspTests.Refl;
using static FerramAerospaceResearch.KspTests.EditorWindow;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's level-flight performance envelope (the Heatmaps tab), its Engine Deck tab and the flight scene's
    /// heatmap window, on the stock Aeris 4A, driven through the editor window's own fields and buttons.
    ///
    /// The envelope needs thrust, and FAR gets thrust only from SolverEngines engines (EditorEngineDeck asks AJE's
    /// own solver; a stock ModuleEnginesFX is not sampled at all). So the envelope tests run with AJE
    /// (CKAN AdvancedJetEngine), which turns the Aeris 4A's two Whiplashes into AJE J58 turbojets and its shock
    /// cones into AJE inlets: one engine type, so one envelope map. Without AJE the Heatmaps tab falls back to a
    /// stability-only map, which the last test checks.
    ///
    /// The checks are what any jet does: it flies at moderate Mach low down, runs out of lift slow and high, has a
    /// finite ceiling, needs more angle of attack the thinner the air, flies higher when lighter, and its jet thrust
    /// falls with altitude; and the tabs agree with each other (the envelope's thrust is the Engine Deck's, the
    /// multithreaded sweep is the serial one, the flight window shows the editor's map at the vessel's mass).
    /// Sweeps are kept to a few cells. Each sweep's on-disk history entry is deleted again, and the window's fields
    /// and toggles are put back.
    /// </summary>
    public static class EnvelopeTests
    {

        // CKAN identifier of AJE (pulls in SolverEngines), and the assembly the test needs loaded
        private const string AjeMods = "AdvancedJetEngine";
        private const string AjeRequires = "AJE";

        private const double G0 = 9.80665;

        private static string Inv(double v, string fmt) => v.ToString(fmt, CultureInfo.InvariantCulture);

        // ------------------------------------------------------------------------------------------ envelope

        /// <summary>
        /// Engine Deck then Heatmaps on the same grid (Mach 0.3-2.7 x 0-40 km, 5 x 5, with stability). The envelope is
        /// flyable at Mach 0.9 at sea level, closes below 40 km with a ceiling of at least 10 km (shown in the tab's
        /// summary), needs more angle of attack the higher it flies at a Mach, its slowest flyable Mach does not fall
        /// with altitude, and its lightest mass sample flies at least as high with less incidence. Every trimmed cell's
        /// lift carries the weight, its full-throttle thrust is the Engine Deck's at that condition, and its cruise
        /// Isp is a jet's. The 3 mass samples are dry + 1%, dry + 50% and full of the fuel the jets burn (liquid fuel,
        /// not the aerospike's oxidizer); the fuel slider at full shows the full-tank map; the stability map is dimmed
        /// exactly where the envelope is not flyable; the sweep counted every cell; and the static analysis is the
        /// same after the sweep as before.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_AerisFliesWhereAJetShould(TestContext ctx)
        {
            string cachePath = null;
            try
            {
                ctx.Step("Open FAR Analysis; static analysis at Mach 0.5, 2 deg");
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                double cdBefore = SteadyCd();
                CelestialBody body = FlightGlobals.GetHomeBody();

                ctx.Step("Engine Deck: full throttle, Mach 0.3-2.7 x 0-40 km, 5 x 5");
                var deckOut = new DeckOut();
                yield return SampleDeck(new[] { "1", "0.3", "2.7", "0", "40", "5", "5" }, deckOut);
                EditorEngineDeck.Sample[,] deck = deckOut.Samples;

                ctx.Step("Heatmaps: Mach 0.3-2.7 x 0-40 km, 5 x 5, with stability, multithreaded");
                var sw = new SweepOut();
                yield return Sweep(new[] { "0.3", "2.7", "0", "40", "5", "5" }, true, true, sw);
                yield return Snap.Shot(ctx, "envelope");
                cachePath = sw.CachePath;

                Assert.False(sw.StabilityOnly, "with AJE the sweep is an envelope sweep, not stability-only");
                Assert.Equal(1, sw.Results.Count, "one envelope map: both engines are the same kind (AJE turbojet)");
                HeatmapResult r = sw.Results[0];
                Assert.Equal("", r.EngineTypeLabel, "a single-type craft's map has no engine-type label");
                Assert.Equal(sw.CellsTotal, sw.CellsDone, "the sweep counted every cell it promised");
                Assert.Equal(Localizer.Format("FAREnvelopeDone", sw.CellsDone.ToString(CultureInfo.InvariantCulture)), sw.Status,
                             "the status line after the sweep");

                double[] machs = { 0.3, 0.9, 1.5, 2.1, 2.7 }, alts = { 0, 10000, 20000, 30000, 40000 };
                AssertAxes(r, machs, alts);
                for (int i = 0; i < machs.Length; i++)
                    Assert.Near(machs[i], deckOut.Mach[i], 1e-9, $"the Engine Deck's Mach column {i} is the heatmap's");

                int nM = machs.Length, nA = alts.Length;
                Assert.NotNull(r.EnvelopeSamples, "the envelope's mass samples");
                Assert.Equal(3, r.EnvelopeSamples.Length, "mass samples swept (dry, mid, full)");
                Cell[,] dry = r.EnvelopeSamples[0], wet = r.EnvelopeSamples[2], shown = r.Envelope;
                RecordGrid(ctx, "wet", wet, nM, nA);
                RecordGrid(ctx, "dry", dry, nM, nA);
                RecordGrid(ctx, "shown", shown, nM, nA);
                ctx.Record("summary", $"ceiling {r.CeilingM:F0} m, best endurance {r.BestEnduranceHours:F2} h, best range {r.BestRangeKm:F0} km");

                ctx.Step("Mass samples: dry + 1%, dry + 50%, full of the fuel the jets burn");
                ShipConstruct ship = EditorLogic.fetch.ship;
                double shipKg = ship.GetTotalMass() * 1000;
                double lfKg = 0;
                foreach (Part p in ship.parts)
                    foreach (PartResource res in p.Resources)
                        if (res.resourceName == "LiquidFuel")
                            lfKg += res.amount * res.info.density * 1000;
                ctx.Record("masses", $"samples {string.Join(", ", r.SampleMassesKg.Select(m => Inv(m, "F0")))} kg; dry {r.DryMassKg:F0}, ship {shipKg:F0}, liquid fuel {lfKg:F0}");
                Assert.Near(r.SampleMassesKg[2], r.MassKg, 1e-6, "the map shown is swept at the full-tank mass");
                Assert.InRange(0.9 * shipKg, shipKg + 1, r.MassKg, "the full-tank mass against the craft's (kg)");
                Assert.Near(lfKg, r.MassKg - r.DryMassKg, 0.01 * lfKg + 1,
                            "the usable fuel is the liquid fuel the jets burn (not the aerospike's oxidizer) (kg)");
                Assert.Near(r.DryMassKg + 0.01 * lfKg, r.SampleMassesKg[0], 0.002 * lfKg + 1, "the lightest sample keeps 1% of the fuel (kg)");
                Assert.Near(r.DryMassKg + 0.5 * lfKg, r.SampleMassesKg[1], 0.002 * lfKg + 1, "the middle sample is half full (kg)");
                Assert.True(r.RefAreaM2 > 0, $"a reference area ({r.RefAreaM2:F2} m2)");

                ctx.Step("Flyable at moderate Mach, low down");
                Cell moderate = shown[1, 0];
                Assert.True(moderate.Feasible, $"level flight is sustainable at Mach 0.9 at sea level ({Describe(moderate)})");
                Assert.InRange(-3, 15, moderate.Alpha, "the trim angle of attack at Mach 0.9, sea level (deg)");

                ctx.Step("Every trimmed cell: lift = weight, thrust = the Engine Deck's, a jet's Isp");
                int cruiseCells = 0;
                for (int i = 0; i < nM; i++)
                    for (int j = 0; j < nA; j++)
                    {
                        Cell c = wet[i, j];
                        if (!c.Trimmed || c.Extrapolated)
                            continue;
                        string at = $"at Mach {machs[i]}, {alts[j] / 1000} km";
                        double effG = InstantConditionSim.CalculateAccelerationDueToGravity(body, alts[j]) -
                                      c.SpeedMPerS * c.SpeedMPerS / (body.Radius + alts[j]);
                        double weight = r.SampleMassesKg[2] * effG, lift = c.LiftToDrag * c.DragN;
                        Assert.Near(weight, lift, 0.03 * weight + 50, $"the cruise lift (L/D x drag) carries the weight {at} (N)");

                        double t = deck[i, j].ThrustMagnitude;
                        double lo = t * Math.Cos((Math.Abs(c.Alpha) + 5) * Math.PI / 180) * 0.97 - 100, hi = t * 1.03 + 100;
                        Assert.InRange(lo, hi, c.ThrustAvailableN,
                                       $"the envelope's full-throttle thrust along the wind {at} against the Engine Deck's {t / 1000:F2} kN (N)");

                        if (c.Feasible && c.Cruise == PerformanceEnvelopeCalculator.CruiseState.Ok)
                        {
                            cruiseCells++;
                            double isp = c.DragN / (c.FuelFlowKgPerS * G0);
                            Assert.InRange(1e-3, 1.0001, c.ThrottleRequired, $"the cruise throttle {at}");
                            Assert.InRange(300, 12000, isp, $"the cruise Isp (drag over fuel flow) {at} is a jet's (s)");
                            Assert.True(c.EnduranceHours > 0 && !double.IsInfinity(c.EnduranceHours), $"a cruise endurance {at} ({c.EnduranceHours:G4} h)");
                            double breguet = isp * c.LiftToDrag * Math.Log(r.SampleMassesKg[2] / r.DryMassKg) / 3600;
                            Assert.Near(breguet, c.EnduranceHours, 1e-6 * breguet + 1e-9, $"the endurance {at} is Breguet's on the cell's own numbers (h)");
                        }
                    }

                Assert.True(cruiseCells > 0, "the envelope has cruise points (feasible, with a cruise throttle)");

                ctx.Step("A finite ceiling, shown in the summary");
                Assert.False(double.IsNaN(r.CeilingM), "the envelope has a ceiling");
                Assert.InRange(10000, 30000, r.CeilingM, "the ceiling (m)");
                Assert.Near(HighestFeasible(shown, alts), r.CeilingM, 1e-6, "the ceiling is the highest row with a flyable cell");
                for (int k = 0; k < 3; k++)
                    for (int i = 0; i < nM; i++)
                        Assert.False(r.EnvelopeSamples[k][i, nA - 1].Feasible,
                                     $"nothing flies level at 40 km (Mach {machs[i]}, mass sample {k}): the envelope closes in the swept range");
                string ceilingText = "Ceiling: " + Inv(r.CeilingM / 1000, "F1") + " km";
                yield return Wait.Frames(3);
                Assert.True(Imgui.IsVisible(ceilingText, Window), $"the tab shows '{ceilingText}'");
                Assert.False(Imgui.IsVisible("Ceiling clipped", Window), "the tab does not call the ceiling clipped");

                ctx.Step("The envelope narrows with altitude");
                int prevSlowest = -1;
                for (int j = 0; j < nA; j++)
                {
                    int slowest = -1;
                    for (int i = 0; i < nM; i++)
                        if (shown[i, j].Feasible)
                        {
                            slowest = i;
                            break;
                        }

                    if (slowest < 0)
                        continue;
                    Assert.True(slowest >= prevSlowest,
                                $"the slowest flyable Mach does not fall with altitude: Mach {machs[slowest]} at {alts[j] / 1000} km, Mach {(prevSlowest < 0 ? 0 : machs[prevSlowest])} lower down");
                    prevSlowest = slowest;
                }

                for (int i = 0; i < nM; i++)
                    for (int j = 0; j + 1 < nA; j++)
                        if (shown[i, j].Feasible && shown[i, j + 1].Feasible)
                            Assert.True(shown[i, j + 1].Alpha > shown[i, j].Alpha - 0.05,
                                        $"thinner air needs more incidence at Mach {machs[i]}: {shown[i, j + 1].Alpha:F2} deg at {alts[j + 1] / 1000} km, {shown[i, j].Alpha:F2} at {alts[j] / 1000}");

                ctx.Step("Lighter flies higher, at less incidence");
                Assert.True(HighestFeasible(dry, alts) >= HighestFeasible(wet, alts),
                            $"the lightest sample's ceiling ({HighestFeasible(dry, alts)} m) is not under the full one's ({HighestFeasible(wet, alts)} m)");
                Assert.True(CountFeasible(dry, nM, nA) >= CountFeasible(wet, nM, nA),
                            $"the lightest sample flies in at least as many cells ({CountFeasible(dry, nM, nA)} against {CountFeasible(wet, nM, nA)})");
                for (int i = 0; i < nM; i++)
                    for (int j = 0; j < nA; j++)
                        if (dry[i, j].Feasible && wet[i, j].Feasible)
                            Assert.True(dry[i, j].Alpha < wet[i, j].Alpha + 0.05,
                                        $"lighter trims at less incidence at Mach {machs[i]}, {alts[j] / 1000} km ({dry[i, j].Alpha:F2} against {wet[i, j].Alpha:F2} deg)");

                ctx.Step("The stability map, dimmed outside the envelope");
                Assert.True(r.HasStability && r.Stability != null, "the sweep has its stability map");
                for (int i = 0; i < nM; i++)
                    for (int j = 0; j < nA; j++)
                    {
                        HeatmapResult.StabilityCell s = r.Stability[i, j];
                        Assert.NotNull(s, $"the stability cell at Mach {machs[i]}, {alts[j] / 1000} km");
                        Assert.True(s.Score >= 0, $"the stability cell at Mach {machs[i]}, {alts[j] / 1000} km evaluated");
                        Cell c = shown[i, j];
                        int mask = !c.Trimmed || c.Suppressed ? 1 : !c.Feasible ? 2 : 0;
                        Assert.Equal(mask, s.PerfMask, $"the stability cell at Mach {machs[i]}, {alts[j] / 1000} km is dimmed as its envelope cell ({Describe(c)})");
                    }

                ctx.Step("The fuel slider: full shows the full-tank map; between samples the drag lies between theirs");
                HeatmapsGUI hm = Heatmaps;
                var shownCopy = (Cell[,])shown.Clone();
                try
                {
                    r.SetDisplayMass(r.SampleMassesKg[2]);
                    AssertSameMap(shownCopy, r.Envelope, nM, nA, "the full-tank map interpolated to the full-tank mass");

                    double midMass = 0.5 * (r.SampleMassesKg[1] + r.SampleMassesKg[2]);
                    r.SetDisplayMass(midMass);
                    Cell[,] mid = r.EnvelopeSamples[1];
                    for (int i = 0; i < nM; i++)
                        for (int j = 0; j < nA; j++)
                        {
                            Cell a = mid[i, j], b = wet[i, j], m = r.Envelope[i, j];
                            if (!(a.Feasible && b.Feasible) || m.Suppressed)
                                continue;
                            Assert.True(m.Feasible, $"flyable half way between two flyable samples at Mach {machs[i]}, {alts[j] / 1000} km");
                            Assert.InRange(Math.Min(a.DragN, b.DragN) - 1e-6, Math.Max(a.DragN, b.DragN) + 1e-6, m.DragN,
                                           $"the drag half way between the samples at Mach {machs[i]}, {alts[j] / 1000} km (N)");
                        }

                    ctx.Step("The fuel slider moved: between set points it says interpolated, near one it snaps to it");
                    int last = r.SampleMassesKg.Length - 1;
                    string Tonnes(double kg) => (kg / 1000.0).ToString("0.00", CultureInfo.InvariantCulture) + " t";
                    yield return Imgui.SetSliderAfterLabel("Fuel:", last - 0.5f, Window);
                    yield return Wait.Frames(3);
                    Assert.Near(last - 0.5f, Get<float>(hm, "_fuelPos"), 1e-6, "the tab's fuel position after the slider");
                    double between = 0.5 * (r.SampleMassesKg[last - 1] + r.SampleMassesKg[last]);
                    Assert.True(Imgui.IsVisible("interp — " + Tonnes(between), Window), $"the slider's label: interpolated, {Tonnes(between)}");
                    Assert.Near(between, r.MassKg, 1e-3 * between, "the maps shown at the mass half way between (kg)");
                    yield return Imgui.SetSliderAfterLabel("Fuel:", last - 0.01f, Window);
                    yield return Wait.Frames(3);
                    Assert.True(Imgui.IsVisible("full — " + Tonnes(r.SampleMassesKg[last]), Window), "a hair short of full it says full");
                    yield return Imgui.SetSliderAfterLabel("Fuel:", last, Window);
                    yield return Wait.Frames(3);
                }
                finally
                {
                    // the tab's own display rebuild, at its fuel position (full)
                    typeof(HeatmapsGUI).GetMethod("ApplyFuelSample", Members)?.Invoke(hm, null);
                }

                ctx.Step("The static analysis after the sweep is what it was");
                double cdAfter = SteadyCd();
                ctx.Record("Cd", $"before {cdBefore:G6}, after {cdAfter:G6}");
                Assert.Near(cdBefore, cdAfter, 1e-6 * cdBefore + 1e-8, "the static analysis' Cd at Mach 0.5, 2 deg after an envelope sweep");
            }
            finally
            {
                DeleteCache(cachePath);
                CloseWindow();
            }
        }

        /// <summary>
        /// The Heatmaps tab's auto-fit and its m/s axis. Auto-fit (Max altitude and Max Mach on) over a user's range that
        /// is too small (Mach 0.3-1 x 0-5 km, 4 x 4) runs a coarse 10 x 10 probe out to Mach 15 and half the atmosphere,
        /// then sweeps the user's grid out to 15% past the probe's highest flyable cell: wider than the user asked, the
        /// probe left out of the history, the user's grid and minima kept. The speed unit button turns the range into
        /// m/s at the sea-level speed of sound; a sweep in m/s keeps the typed speeds as its axis, each cell at the Mach
        /// that speed is at its altitude.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 1200)]
        public static IEnumerator Envelope_AutoFitAndSpeedAxis(TestContext ctx)
        {
            var caches = new List<string>();
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                CelestialBody body = FlightGlobals.GetHomeBody();
                HeatmapsGUI hm = Heatmaps;
                PrepareHeatmaps(hm);
                string craft = EditorLogic.fetch.ship.shipName;
                int historyBefore = HeatmapPersistence.ListForVessel(craft).Count;

                ctx.Step("Auto-fit both maxima from Mach 0.3-1 x 0-5 km, 4 x 4");
                var fit = new SweepOut();
                // serial: under a coverage recorder the workers contend for its counters, and the probe takes many minutes
                yield return Sweep(new[] { "0.3", "1", "0", "5", "4", "4" }, false, false, fit, autoFit: true);
                caches.Add(fit.CachePath);
                HeatmapResult r = fit.Results[0];
                double machMax = double.Parse(Get<string>(hm, "_machMax"), CultureInfo.InvariantCulture);
                double altMaxKm = double.Parse(Get<string>(hm, "_altMaxKm"), CultureInfo.InvariantCulture);
                ctx.Record("fitted", $"Mach 0.3-{machMax}, 0-{altMaxKm} km; ceiling {r.CeilingM:F0} m");
                RecordGrid(ctx, "fit", r.Envelope, r.MachAxis.Length, r.AltAxis.Length);
                Assert.Equal("4", Get<string>(hm, "_nMach"), "the user's Mach cells after the probe");
                Assert.Equal("4", Get<string>(hm, "_nAlt"), "the user's altitude cells after the probe");
                Assert.Equal("0.3", Get<string>(hm, "_machMin"), "the user's Mach minimum after the probe");
                Assert.Equal("0", Get<string>(hm, "_altMinKm"), "the user's altitude minimum after the probe");
                Assert.Equal(4, r.MachAxis.Length, "the sweep's Mach columns are the user's");
                Assert.Equal(4, r.AltAxis.Length, "the sweep's altitude rows are the user's");
                Assert.Near(machMax, r.MachAxis[3], 1e-9, "the sweep runs to the fitted Mach");
                Assert.Near(altMaxKm * 1000, r.AltAxis[3], 1e-6, "the sweep runs to the fitted altitude");
                Assert.True(machMax > 1 && altMaxKm > 5, "the fit widened the range past the user's (the Aeris flies above Mach 1 and 5 km)");

                // the probe: 10 cells from the user's minimum to Mach 15 and to half the atmosphere; the fit is 15%
                // past one of its cells (capped there)
                double probeAltStep = body.atmosphereDepth / 2 / 9, probeMachStep = (15 - 0.3) / 9;
                int ja = (int)Math.Round(altMaxKm * 1000 / 1.15 / probeAltStep), im = (int)Math.Round((machMax / 1.15 - 0.3) / probeMachStep);
                Assert.Near(Math.Min(ja * probeAltStep * 1.15, body.atmosphereDepth / 2) / 1000, altMaxKm, 0.05,
                            $"the fitted altitude is 15% past probe row {ja} (km)");
                Assert.Near(Math.Min((0.3 + im * probeMachStep) * 1.15, 15), machMax, 0.005, $"the fitted Mach is 15% past probe column {im}");
                Assert.True(r.CeilingM >= r.AltAxis[1], $"the envelope reaches past the frame's lowest third ({r.CeilingM:F0} m)");
                int historyAfter = HeatmapPersistence.ListForVessel(craft).Count;
                Assert.Equal(historyBefore + 1, historyAfter, "one sweep joins the history, not the probe");

                ctx.Step("The speed unit button: the range in m/s at the sea-level speed of sound");
                yield return Imgui.SetTextNth(0, "0.3", Window);
                yield return Imgui.SetTextNth(1, "2", Window);
                double sos0 = EditorAtmosphere.GetGasProperties(body, 0).SpeedOfSound;
                yield return SetHeatmapToggles(false, false, speedInMS: true);
                Assert.Equal(Inv(0.3 * sos0, "F0"), Get<string>(hm, "_machMin"), "the minimum in m/s");
                Assert.Equal(Inv(2 * sos0, "F0"), Get<string>(hm, "_machMax"), "the maximum in m/s");

                ctx.Step("A sweep at 100-700 m/s x 0-10 km, 4 x 3");
                var ms = new SweepOut();
                yield return Sweep(new[] { "100", "700", "0", "10", "4", "3" }, false, false, ms, speedInMS: true);
                caches.Add(ms.CachePath);
                HeatmapResult s = ms.Results[0];
                RecordGrid(ctx, "speeds", s.Envelope, 4, 3);
                Assert.True(s.SpeedAxisMode, "the map's x axis is speed");
                double[] speeds = { 100, 300, 500, 700 }, alts = { 0, 5000, 10000 };
                Assert.Equal(4, s.MachAxis.Length, "speed columns");
                for (int i = 0; i < speeds.Length; i++)
                    Assert.Near(speeds[i], s.MachAxis[i], 1e-9, $"speed column {i} (m/s)");
                int valid = 0;
                for (int j = 0; j < alts.Length; j++)
                {
                    double sos = EditorAtmosphere.GetGasProperties(body, alts[j]).SpeedOfSound;
                    for (int i = 0; i < speeds.Length; i++)
                    {
                        Cell c = s.Envelope[i, j];
                        if (c.SpeedMPerS <= 0)
                            continue;
                        valid++;
                        string at = $"at {speeds[i]} m/s, {alts[j] / 1000} km";
                        Assert.Near(speeds[i] / sos, c.Mach, 1e-3 * c.Mach, $"the Mach {at}");
                        Assert.Near(speeds[i], c.SpeedMPerS, 0.005 * speeds[i], $"the cell's airspeed {at}");
                    }
                }
                Assert.True(valid >= 10, $"cells with an airspeed ({valid} of 12)");
                Assert.True(s.Envelope[1, 0].Feasible, "flyable at 300 m/s at sea level");
            }
            finally
            {
                foreach (string c in caches)
                    DeleteCache(c);
                CloseWindow();
            }
        }

        // ------------------------------------------------------------------------------------------ engine deck

        /// <summary>
        /// Engine Deck at full throttle, Mach 0-2 x 0-15 km: it samples the craft's two jets and two inlets (and says
        /// so), its static sea-level thrust is the engines' tooltip static thrust, jet thrust falls with altitude at
        /// every Mach, ram makes the engines stronger supersonic than subsonic up high, the inlet air is stagnated
        /// (T0 (1 + 0.2 M^2)), half throttle makes less thrust and burns less fuel, the fuel-flow toggle switches the
        /// table, and the engines are left at zero throttle after sampling.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_EngineDeckThrustLapsesWithAltitude(TestContext ctx)
        {
            try
            {
                ctx.Step("Open FAR Analysis on the Engine Deck");
                yield return OpenWindow("FAREditorModeStatic");
                ResetConditions();
                List<Part> jets = EditorLogic.fetch.ship.parts.Where(p => p.Modules.Cast<PartModule>().Any(SolverEnginesInterop.IsSolverEngine)).ToList();
                int nJets = EditorLogic.fetch.ship.parts.Count(p => p.partInfo.name == "turboFanEngine");
                int nInlets = EditorLogic.fetch.ship.parts.Count(p => p.partInfo.name == "shockConeIntake");
                ctx.Record("craft", $"{jets.Count} solver engines ({nJets} Whiplash), {nInlets} shock cones");
                Assert.Equal(nJets, jets.Count, "AJE made both Whiplashes solver engines (and nothing else)");

                ctx.Step("Full throttle, Mach 0-2 x 0-15 km, 5 x 4");
                var full = new DeckOut();
                yield return SampleDeck(new[] { "1", "0", "2", "0", "15", "5", "4" }, full);
                yield return Snap.Shot(ctx, "engine-deck");
                Assert.Equal(nJets, full.Engines, "the deck samples the craft's jets");
                Assert.Equal(nInlets, full.Inlets, "the deck counts the craft's AJE inlets");
                Assert.Equal(Localizer.Format("FAREngineDeckStatus", nJets.ToString(CultureInfo.InvariantCulture), nInlets.ToString(CultureInfo.InvariantCulture)),
                             full.Status, "the deck's status line");
                yield return Wait.Frames(3);
                Assert.True(Imgui.IsVisible(full.Status, Window), "the tab shows its status line");
                Assert.True(Imgui.IsVisible(L("FAREngineDeckThrustHeader"), Window), "the tab shows the thrust table");

                double[] machs = { 0, 0.5, 1, 1.5, 2 }, alts = { 0, 5000, 10000, 15000 };
                EditorEngineDeck.Sample[,] s = full.Samples;
                for (int j = 0; j < alts.Length; j++)
                    ctx.Record($"thrust {alts[j] / 1000} km",
                               string.Join(" ", Enumerable.Range(0, machs.Length).Select(i => $"M{machs[i]}:{s[i, j].ThrustMagnitude / 1000:F1}kN/{s[i, j].FuelFlow:F3}kg/s{(s[i, j].ThermalLimited ? "(hot)" : "")}")));

                EditorEngineDeck.Sample stat = s[0, 0];
                Assert.True(stat.ThrustMagnitude > 0 && stat.FuelFlow > 0, $"the jets make thrust and burn fuel standing still at sea level ({stat.ThrustMagnitude / 1000:F1} kN)");
                Assert.Equal(nJets, stat.RunningCount, "every jet runs at sea level, static");
                Assert.InRange(1e-6, 1.000001, stat.AreaRatio, "the inlet-to-engine area ratio");
                Assert.True(stat.InletArea > 0 && stat.EngineArea > 0, $"inlet ({stat.InletArea:F3} m2) and engine ({stat.EngineArea:F3} m2) areas");

                double tooltipKN = 0;
                bool haveTooltip = true;
                foreach (Part p in jets)
                {
                    double kn = TooltipStaticThrustKN(p);
                    if (double.IsNaN(kn))
                        haveTooltip = false;
                    tooltipKN += kn;
                }

                ctx.Record("staticThrust", $"deck {stat.ThrustMagnitude / 1000:F1} kN, tooltips {(haveTooltip ? Inv(tooltipKN, "F1") + " kN" : "n/a")}");
                if (haveTooltip)
                    Assert.Near(tooltipKN, stat.ThrustMagnitude / 1000, 0.2 * tooltipKN,
                                "the Mach 0 / sea level cell against the engines' tooltip static thrust (kN)");

                ctx.Step("Thrust lapses with altitude; ram helps up high; the inlet air is stagnated");
                for (int i = 0; i < machs.Length; i++)
                {
                    for (int j = 0; j + 1 < alts.Length; j++)
                    {
                        EditorEngineDeck.Sample lo = s[i, j], up = s[i, j + 1];
                        if (lo.ThermalLimited || up.ThermalLimited)
                            continue;
                        Assert.True(up.ThrustMagnitude < lo.ThrustMagnitude,
                                    $"jet thrust falls with altitude at Mach {machs[i]}: {up.ThrustMagnitude / 1000:F1} kN at {alts[j + 1] / 1000} km, {lo.ThrustMagnitude / 1000:F1} at {alts[j] / 1000}");
                        Assert.True(up.AmbientP < lo.AmbientP, $"the ambient pressure falls with altitude at Mach {machs[i]}");
                    }

                    for (int j = 0; j < alts.Length; j++)
                    {
                        EditorEngineDeck.Sample c = s[i, j];
                        Assert.True(!double.IsNaN(c.ThrustMagnitude) && c.ThrustMagnitude >= 0, $"a thrust at Mach {machs[i]}, {alts[j] / 1000} km");
                        double stagnation = c.AmbientT * (1 + 0.2 * machs[i] * machs[i]);
                        Assert.Near(stagnation, c.InletT, 0.02 * stagnation,
                                    $"the inlet temperature is the stagnation temperature at Mach {machs[i]}, {alts[j] / 1000} km (K)");
                    }
                }

                Assert.True(s[3, 2].ThrustMagnitude > s[1, 2].ThrustMagnitude,
                            $"ram: a Mach 3 jet makes more thrust at Mach 1.5 than at 0.5 at 10 km ({s[3, 2].ThrustMagnitude / 1000:F1} against {s[1, 2].ThrustMagnitude / 1000:F1} kN)");

                ctx.Step("The engines are left at zero throttle");
                foreach (Part p in jets)
                    foreach (PartModule m in p.Modules)
                        if (SolverEnginesInterop.IsSolverEngine(m) && m is ModuleEngines e)
                            Assert.Equal(0f, e.currentThrottle, $"{p.partInfo.title}'s throttle after sampling");

                ctx.Step("Half throttle: less thrust, less fuel");
                var half = new DeckOut();
                yield return SampleDeck(new[] { "0.5", "0", "2", "0", "15", "5", "4" }, half);
                EditorEngineDeck.Sample[,] h = half.Samples;
                for (int i = 0; i < machs.Length; i++)
                    for (int j = 0; j < alts.Length; j++)
                    {
                        if (s[i, j].ThrustMagnitude <= 0)
                            continue;
                        Assert.True(h[i, j].ThrustMagnitude <= s[i, j].ThrustMagnitude * 1.001 + 1,
                                    $"half throttle makes no more thrust than full at Mach {machs[i]}, {alts[j] / 1000} km ({h[i, j].ThrustMagnitude / 1000:F1} against {s[i, j].ThrustMagnitude / 1000:F1} kN)");
                        Assert.True(h[i, j].FuelFlow <= s[i, j].FuelFlow * 1.001 + 1e-6,
                                    $"half throttle burns no more fuel than full at Mach {machs[i]}, {alts[j] / 1000} km");
                    }

                Assert.True(h[0, 0].ThrustMagnitude < 0.9 * stat.ThrustMagnitude && h[0, 0].FuelFlow < 0.9 * stat.FuelFlow,
                            $"half throttle, static at sea level: {h[0, 0].ThrustMagnitude / 1000:F1} kN and {h[0, 0].FuelFlow:F3} kg/s against {stat.ThrustMagnitude / 1000:F1} and {stat.FuelFlow:F3}");

                ctx.Step("Show fuel flow");
                yield return Imgui.Toggle(L("FAREngineDeckShowFlow"), true, Window);
                yield return Wait.Frames(3);
                Assert.True(Imgui.IsVisible(L("FAREngineDeckFlowHeader"), Window), "the fuel-flow toggle switches the table to fuel flow");
                Assert.True(Imgui.IsVisible(Inv(h[0, 0].FuelFlow, "F3"), Window), "the table shows the static sea-level fuel flow");
            }
            finally
            {
                CloseWindow();
            }
        }

        // ------------------------------------------------------------------------------------------ the mouse

        private const int TooltipWindowId = 0x7A9B3C0D;   // HeatmapAxes' tooltip overlay window

        /// <summary>
        /// The maps under a (synthetic) mouse. Over an envelope cell the tooltip, drawn beside the pointer, is that cell's:
        /// its Mach, speed and altitude, and its row's envelope (top and slowest level speed at that altitude); over a
        /// stability cell, that cell's Mach, altitude and score; off the maps there is none. A box dragged across the
        /// envelope map zooms every map to the cells it spans, the hover then finding the zoomed view's cells; Reset
        /// zooms out. A press and release in one place is a click, not a box: no zoom. A middle click on a cell logs
        /// FAR's diagnostic of that cell.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_MapAnswersTheMouse(TestContext ctx)
        {
            var caches = new List<string>();
            var diagnostics = new List<string>();
            void Listen(string message, string trace, LogType type)
            {
                if (message != null && message.Contains("FAR heatmap cell diagnostic")) diagnostics.Add(message);
            }

            Application.logMessageReceived += Listen;
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);

                ctx.Step("A 4 x 4 sweep with stability, Mach 0.3-1.2 x 0-9 km");
                var sw = new SweepOut();
                // serial: under a coverage recorder the workers contend for its counters
                yield return Sweep(new[] { "0.3", "1.2", "0", "9", "4", "4" }, true, false, sw);
                caches.Add(sw.CachePath);
                Assert.False(sw.StabilityOnly, "an envelope swept (SolverEngines loaded)");
                HeatmapsGUI hm = Heatmaps;
                HeatmapResult r = sw.Results[0];
                HeatmapRenderer env = Get<List<HeatmapRenderer>>(hm, "_envRenderers")[0];
                HeatmapRenderer stab = Get<HeatmapRenderer>(hm, "_stabRenderer");
                var envTex = Get<Texture2D>(env, "_envTex");
                var stabTex = Get<Texture2D>(stab, "_stabTex");
                yield return Wait.Until(() => Imgui.ImageRect(envTex, Window) != null && Imgui.ImageRect(stabTex, Window) != null, 15,
                                        "both maps drawn", shot: false, detail: Imgui.Dump);
                Rect envMap = Imgui.ImageRect(envTex, Window).Value, stabMap = Imgui.ImageRect(stabTex, Window).Value;
                ctx.Record("maps", $"envelope {envMap}, stability {stabMap}");

                ctx.Step("Hovering an envelope cell");
                (int i, int j) cell = (2, 1);
                yield return HoverTip(CellCentre(envMap, cell, 0, 3, 0, 3));
                Cell c = r.Envelope[cell.i, cell.j];
                string tip = Tip();
                ctx.Record("envelopeTip", tip);
                Assert.True(tip.StartsWith("M " + Inv(c.Mach, "F2")) && tip.Contains(Inv(c.SpeedMPerS, "F0") + " m/s") &&
                            tip.Contains(Inv(r.AltAxis[cell.j] / 1000, "F1") + " km"), $"the tooltip is the hovered cell's ({tip.Split('\n')[0]})");
                Assert.True(tip.Contains("─ At " + Inv(r.AltAxis[cell.j] / 1000, "F1") + " km ─"), "the tooltip has the row's envelope");
                Assert.True(tip.Contains("Top speed") || tip.Contains("Level flight not possible"), "the row's top speed, or why there is none");
                AssertTipBeside(Imgui.Pointer.Value);

                ctx.Step("Hovering a stability cell");
                (int i, int j) scell = (1, 2);
                yield return HoverTip(CellCentre(stabMap, scell, 0, 3, 0, 3));
                tip = Tip();
                ctx.Record("stabilityTip", tip);
                Assert.True(tip.StartsWith("M " + Inv(r.MachAxis[scell.i], "F2") + "  ·  " + Inv(r.AltAxis[scell.j] / 1000, "F1") + " km"),
                            $"the stability tooltip is the hovered cell's ({tip.Split('\n')[0]})");
                Assert.True(tip.Contains("Score:") || tip.Contains("Not evaluated") || tip.Contains("Errored"), "the cell's score");
                AssertTipBeside(Imgui.Pointer.Value);

                ctx.Step("The pointer off the maps");
                yield return Imgui.Hover(new Vector2(envMap.xMax + 40, envMap.y - 40), 5);
                Assert.True(TipRect().x < -1000, $"no tooltip off the maps (its window at {TipRect()})");

                ctx.Step("A click without a drag");
                yield return Imgui.MouseClick(CellCentre(envMap, (1, 1), 0, 3, 0, 3));
                Assert.False((bool)GetStatic(typeof(HeatmapRenderer), "_zoomActive"), "a click is not a box: no zoom");

                ctx.Step("A box dragged from cell (1, 1) to (2, 2)");
                yield return Imgui.MouseDragFromTo(CellCentre(envMap, (1, 1), 0, 3, 0, 3), CellCentre(envMap, (2, 2), 0, 3, 0, 3));
                yield return Wait.Frames(3);
                object zoom = GetStatic(typeof(HeatmapRenderer), "_zoom");
                string window = $"Mach cells {Get(zoom, "IMin")}-{Get(zoom, "IMax")}, altitude cells {Get(zoom, "JMin")}-{Get(zoom, "JMax")}";
                ctx.Record("zoom", window);
                Assert.True((bool)GetStatic(typeof(HeatmapRenderer), "_zoomActive"), "the drag zoomed in");
                Assert.Equal("Mach cells 1-2, altitude cells 1-2", window, "the zoom is the cells the box spans");
                yield return Snap.Shot(ctx, "zoomed");

                // the zoomed view's top left cell: the slowest and highest of the box
                yield return HoverTip(CellCentre(envMap, (1, 2), 1, 2, 1, 2));
                tip = Tip();
                Cell zc = r.Envelope[1, 2];
                Assert.True(tip.StartsWith("M " + Inv(zc.Mach, "F2")) && tip.Contains(Inv(r.AltAxis[2] / 1000, "F1") + " km"),
                            $"in the zoomed view the hover finds the zoomed cells ({tip.Split('\n')[0]})");
                yield return HoverTip(CellCentre(stabMap, (2, 1), 1, 2, 1, 2));
                tip = Tip();
                Assert.True(tip.StartsWith("M " + Inv(r.MachAxis[2], "F2") + "  ·  " + Inv(r.AltAxis[1] / 1000, "F1") + " km"),
                            $"the stability map shares the zoom ({tip.Split('\n')[0]})");

                ctx.Step("Reset");
                yield return Imgui.Click("Reset", Window, exact: true);
                yield return Wait.Frames(2);
                Assert.False((bool)GetStatic(typeof(HeatmapRenderer), "_zoomActive"), "Reset zooms out");

                ctx.Step("A middle click on an envelope cell");
                diagnostics.Clear();
                yield return Imgui.MouseDown(CellCentre(envMap, cell, 0, 3, 0, 3), button: 2);
                yield return Wait.Until(() => diagnostics.Count > 0, 15, "FAR's cell diagnostic in the log", shot: false);
                string diag = diagnostics[0];
                ctx.Record("diagnostic", diag.Length > 1500 ? diag.Substring(0, 1500) : diag);
                Assert.True(diag.Contains($"cell [{cell.i},{cell.j}]") && diag.Contains("M " + Inv(c.Mach, "F3")),
                            "the diagnostic is the clicked cell's");
                Assert.True(diag.Contains("Engine thrust caps"), "the diagnostic lists the engines' thrust caps");
            }
            finally
            {
                Application.logMessageReceived -= Listen;
                Imgui.ReleasePointer();
                HeatmapRenderer.ResetZoom();
                foreach (string path in caches)
                    DeleteCache(path);
                CloseWindow();
            }
        }

        /// <summary>The screen centre of cell (i, j) of a map showing Mach cells i0-i1 and altitude cells j0-j1 (altitude up).</summary>
        private static Vector2 CellCentre(Rect map, (int i, int j) cell, int i0, int i1, int j0, int j1)
        {
            int nI = i1 - i0 + 1, nJ = j1 - j0 + 1;
            return new Vector2(map.x + (cell.i - i0 + 0.5f) / nI * map.width,
                               map.y + (nJ - 1 - (cell.j - j0) + 0.5f) / nJ * map.height);
        }

        private static string Tip() => (string)GetStatic(typeof(HeatmapAxes), "_pendingText") ?? "";

        private static Rect TipRect() =>
            Imgui.VisibleWindows().FirstOrDefault(w => w.Id == TooltipWindowId)?.Rect ?? new Rect(-10000, -10000, 0, 0);

        /// <summary>The pointer over a screen point until the tooltip window is drawn beside it.</summary>
        private static IEnumerator HoverTip(Vector2 screen)
        {
            yield return Imgui.Hover(screen);
            yield return Wait.Until(() => TipBeside(screen), 10, $"the heatmap tooltip beside the pointer at {screen}", shot: false, detail: Imgui.Dump);
        }

        // HeatmapAxes.DrawQueuedTooltip: 14 px right of and below the pointer, or left of / above it near a screen edge
        private static bool TipBeside(Vector2 p)
        {
            Rect t = TipRect();
            return t.x > -1000 && (Math.Abs(t.x - (p.x + 14)) < 1.5f || Math.Abs(t.xMax - (p.x - 14)) < 1.5f || t.x <= 4.5f || t.xMax >= Screen.width - 4.5f) &&
                   (Math.Abs(t.y - (p.y + 14)) < 1.5f || Math.Abs(t.yMax - (p.y - 14)) < 1.5f || t.y <= 4.5f || t.yMax >= Screen.height - 4.5f);
        }

        private static void AssertTipBeside(Vector2 p) => Assert.True(TipBeside(p), $"the tooltip window beside the pointer ({TipRect()} by {p})");

        // ------------------------------------------------------------------------------------------ history

        /// <summary>
        /// The Heatmaps tab's history of the craft's sweeps: each sweep is filed on disk, the newest shown. The back
        /// button shows the older sweep read back from its file: the same maps as swept (every mass sample's cells,
        /// the axes, the ceiling, the stability flags). Forward shows the newer again; Remove deletes the one shown
        /// and its file and shows the one left; Clear History deletes the rest.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_SweepHistoryIsKeptAndBrowsed(TestContext ctx)
        {
            const string Back = "◄", Forward = "►";
            SweepOut first = null, second = null;
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                HeatmapsGUI hm = Heatmaps;
                int before = HeatmapPersistence.ListForVessel(Far.Plane).Count;

                ctx.Step("Two sweeps: with stability, then without");
                first = new SweepOut();
                yield return Sweep(new[] { "0.4", "1.2", "0", "6", "3", "3" }, true, true, first);
                second = new SweepOut();
                yield return Sweep(new[] { "0.6", "1.6", "0", "9", "3", "3" }, false, true, second);
                var history = Get<List<HeatmapPersistence.CacheEntry>>(hm, "_cachedSweeps");
                ctx.Record("history", string.Join(", ", history.Select(h => System.IO.Path.GetFileName(h.Path))));
                Assert.Equal(before + 2, history.Count, "both sweeps filed in the craft's history");
                Assert.True(first.CachePath != null && System.IO.File.Exists(first.CachePath) && System.IO.File.Exists(second.CachePath), "each sweep's file on disk");
                Assert.Equal(history.Count - 1, Get<int>(hm, "_viewIndex"), "the newest sweep shown");

                ctx.Step("Back: the first sweep, read back from its file");
                yield return Imgui.Click(Back, Window, exact: true);
                Assert.Equal(history.Count - 2, Get<int>(hm, "_viewIndex"), "the older sweep shown");
                var shown = Get<List<HeatmapResult>>(hm, "_results");
                Assert.False(ReferenceEquals(shown, first.Results), "the sweep read back, not the one in memory");
                AssertSameSweep(first.Results, shown, "the first sweep read back");
                yield return Snap.Shot(ctx, "older-sweep");

                ctx.Step("Forward: the second sweep");
                yield return Imgui.Click(Forward, Window, exact: true);
                Assert.Equal(history.Count - 1, Get<int>(hm, "_viewIndex"), "the newest sweep shown again");
                AssertSameSweep(second.Results, Get<List<HeatmapResult>>(hm, "_results"), "the second sweep read back");

                ctx.Step("Remove: the second sweep and its file gone");
                yield return Imgui.Click("Remove", Window, exact: true);
                history = Get<List<HeatmapPersistence.CacheEntry>>(hm, "_cachedSweeps");
                Assert.Equal(before + 1, history.Count, "one sweep left in the history");
                Assert.False(System.IO.File.Exists(second.CachePath), "the removed sweep's file deleted");
                AssertSameSweep(first.Results, Get<List<HeatmapResult>>(hm, "_results"), "the sweep left, shown");

                ctx.Step("Clear History: none left");
                yield return Imgui.Click("Clear History", Window, exact: true);
                Assert.Equal(0, Get<List<HeatmapPersistence.CacheEntry>>(hm, "_cachedSweeps").Count, "the history empty");
                Assert.Equal(0, HeatmapPersistence.ListForVessel(Far.Plane).Count, "no sweep of the craft on disk");
                Assert.False(System.IO.File.Exists(first.CachePath), "the first sweep's file deleted");
            }
            finally
            {
                DeleteCache(first?.CachePath);
                DeleteCache(second?.CachePath);
                CloseWindow();
            }
        }

        /// <summary>Two sweeps' maps the same: axes, ceiling, every mass sample's cells, and the stability flags.</summary>
        private static void AssertSameSweep(List<HeatmapResult> expected, List<HeatmapResult> actual, string what)
        {
            Assert.True(actual != null && actual.Count == expected.Count, $"{what}: {expected.Count} map(s)");
            for (int m = 0; m < expected.Count; m++)
            {
                HeatmapResult e = expected[m], a = actual[m];
                AssertAxes(a, e.MachAxis, e.AltAxis);
                Assert.True(e.CeilingM.Equals(a.CeilingM), $"{what}: the ceiling ({e.CeilingM} against {a.CeilingM})");
                Assert.Equal(e.EnvelopeSamples.Length, a.EnvelopeSamples.Length, $"{what}: mass samples");
                for (int k = 0; k < e.EnvelopeSamples.Length; k++)
                    AssertSameMap(e.EnvelopeSamples[k], a.EnvelopeSamples[k], e.MachAxis.Length, e.AltAxis.Length, $"{what}, mass sample {k}");
                Assert.Equal(e.HasStability, a.HasStability, $"{what}: a stability map or none");
                if (!e.HasStability)
                    continue;
                for (int i = 0; i < e.MachAxis.Length; i++)
                    for (int j = 0; j < e.AltAxis.Length; j++)
                    {
                        string fe = e.Stability[i, j] == null ? "-" : string.Join(" ", e.Stability[i, j].OutOfRange.Select(o => o.Item1).OrderBy(n => n));
                        string fa = a.Stability[i, j] == null ? "-" : string.Join(" ", a.Stability[i, j].OutOfRange.Select(o => o.Item1).OrderBy(n => n));
                        Assert.Equal(fe, fa, $"{what}: the stability flags at [{i},{j}]");
                    }
            }
        }

        // ------------------------------------------------------------------------------------------ threads

        /// <summary>
        /// The same 4 x 4 sweep (Mach 0.5-2 x 0-15 km, with stability) multithreaded and on the main thread: the
        /// worker threads (engine solver clones, frozen geometry, the stability map riding along) give the serial
        /// path's map (its stand-alone stability pass is the stability-only fallback's loop): the same cells trim
        /// and fly, at the same angle of attack, drag and thrust, with the same stability flags.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_MultithreadedSweepIsTheSerialOne(TestContext ctx)
        {
            string pathA = null, pathB = null;
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                string[] fields = { "0.5", "2", "0", "15", "4", "4" };

                ctx.Step("Multithreaded sweep");
                var mt = new SweepOut();
                yield return Sweep(fields, true, true, mt);
                pathA = mt.CachePath;

                ctx.Step("Serial sweep");
                var serial = new SweepOut();
                yield return Sweep(fields, true, false, serial);
                pathB = serial.CachePath;

                Assert.Equal(mt.Results.Count, serial.Results.Count, "maps in each sweep");
                HeatmapResult a = mt.Results[0], b = serial.Results[0];
                int nM = a.MachAxis.Length, nA = a.AltAxis.Length;
                Assert.Equal(serial.CellsTotal, serial.CellsDone, "the serial sweep counted every cell it promised");
                int compared = 0, trimMismatch = 0;
                double dAlpha = 0, dDrag = 0, dThrust = 0;
                for (int k = 0; k < a.EnvelopeSamples.Length; k++)
                {
                    RecordGrid(ctx, $"mt{k}", a.EnvelopeSamples[k], nM, nA);
                    RecordGrid(ctx, $"serial{k}", b.EnvelopeSamples[k], nM, nA);
                    for (int i = 0; i < nM; i++)
                        for (int j = 0; j < nA; j++)
                        {
                            Cell x = a.EnvelopeSamples[k][i, j], y = b.EnvelopeSamples[k][i, j];
                            string at = $"Mach {a.MachAxis[i]:F2}, {a.AltAxis[j] / 1000:F0} km, mass sample {k}";
                            if (x.Trimmed != y.Trimmed)
                            {
                                trimMismatch++;
                                ctx.Record("trimMismatch " + at, $"multithreaded {Describe(x)}, serial {Describe(y)}");
                                continue;
                            }

                            if (!x.Trimmed || x.Extrapolated || y.Extrapolated || x.Alpha > 10 || y.Alpha > 10)
                                continue;
                            compared++;
                            dAlpha = Math.Max(dAlpha, Math.Abs(x.Alpha - y.Alpha));
                            dDrag = Math.Max(dDrag, Math.Abs(x.DragN - y.DragN) / Math.Max(1, Math.Abs(y.DragN)));
                            dThrust = Math.Max(dThrust, Math.Abs(x.ThrustAvailableN - y.ThrustAvailableN) / Math.Max(1, Math.Abs(y.ThrustAvailableN)));
                            // The two paths are not bit-identical (some input differs in its last bits), and the trim's
                            // secant solve (1e-4 in Cl) can stop anywhere in its tolerance band: measured on the Aeris 4A,
                            // 0.0016 deg, 0.033% drag and 4e-6 of the thrust at most. The limits are a few times that,
                            // so a threading bug that moves a cell's answer at all shows.
                            Assert.Equal(y.Feasible, x.Feasible, $"flyable in both sweeps or neither at {at}");
                            Assert.Equal(y.Overheats, x.Overheats, $"heat-limited in both sweeps or neither at {at}");
                            Assert.Near(y.Alpha, x.Alpha, 0.005, $"the trim angle of attack at {at} (deg)");
                            Assert.Near(y.DragN, x.DragN, 0.001 * Math.Abs(y.DragN) + 1, $"the cruise drag at {at} (N)");
                            Assert.Near(y.ThrustAvailableN, x.ThrustAvailableN, 1e-4 * Math.Abs(y.ThrustAvailableN) + 1,
                                        $"the full-throttle thrust at {at} (N)");
                        }
                }

                ctx.Record("compared", $"{compared} cells, {trimMismatch} trimmed in one sweep only");
                ctx.Record("largestDifference", $"AoA {dAlpha:G3} deg, drag {dDrag:G3}, thrust {dThrust:G3} (relative)");
                Assert.True(compared >= 10, $"enough cells trimmed well below the stall in both sweeps to compare ({compared})");
                Assert.Equal(0, trimMismatch, "cells that trim in one sweep and not the other");

                Assert.True(a.HasStability && b.HasStability, "both sweeps have their stability map");
                for (int i = 0; i < nM; i++)
                    for (int j = 0; j < nA; j++)
                    {
                        Cell c = a.EnvelopeSamples[2][i, j];
                        if (!c.Feasible || c.Alpha > 10)
                            continue;
                        if (a.Stability[i, j] == null || b.Stability[i, j] == null)
                            continue;
                        string sa = string.Join(" ", a.Stability[i, j].OutOfRange.Select(o => o.Item1).OrderBy(n => n));
                        string sb = string.Join(" ", b.Stability[i, j].OutOfRange.Select(o => o.Item1).OrderBy(n => n));
                        Assert.Equal(sb, sa, $"the stability flags at Mach {a.MachAxis[i]:F2}, {a.AltAxis[j] / 1000:F0} km, threaded against serial");
                    }
            }
            finally
            {
                DeleteCache(pathA);
                DeleteCache(pathB);
                CloseWindow();
            }
        }

        // ------------------------------------------------------------------------------------------ flight

        /// <summary>
        /// A 5 x 5 sweep (Mach 0.4-2 x 0-12 km) in the SPH, then the craft launched to the runway and placed at
        /// Mach 0.8, 3 km. FAR's flight heatmap window shows that sweep as this vessel's (no mismatch warning), at the
        /// vessel's mass, which at full tanks is the editor's full-tank map cell for cell; its ceiling summary is the
        /// editor's; it reads back the vessel's Mach and altitude, and names the nearest cell and that cell's verdict,
        /// which for a jet at Mach 0.8, 3 km is sustainable level flight.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 1200)]
        public static IEnumerator Envelope_FlightWindowShowsTheVesselsEnvelope(TestContext ctx)
        {
            List<HeatmapResult> swept;
            string cachePath = null;
            try
            {
                ctx.Step("Heatmaps: Mach 0.4-2 x 0-12 km, 5 x 5");
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                var sw = new SweepOut();
                yield return Sweep(new[] { "0.4", "2", "0", "12", "5", "5" }, false, true, sw);
                cachePath = sw.CachePath;
                swept = sw.Results;
                Assert.False(sw.StabilityOnly, "an envelope sweep (AJE present)");
                Assert.True(ReferenceEquals(HeatmapResult.Cached, swept), "the finished sweep is the one handed to the flight scene");
                RecordGrid(ctx, "shown", swept[0].Envelope, swept[0].MachAxis.Length, swept[0].AltAxis.Length);
            }
            finally
            {
                // the map travels in memory (HeatmapResult.Cached); its history file is not needed
                DeleteCache(cachePath);
                CloseWindow();
            }

            ctx.Step("Launch to the runway; fly at Mach 0.8, 3 km");
            EditorLogic.fetch.launchSiteName = "Runway";
            yield return Nav.Launch(ctx);
            yield return Wait.Seconds(5);   // FAR voxelizes the craft after it loads
            FarFlightProbe.Condition cruise = Far.At(0.8, 3000, 2, "cruise M0.8 3km");
            yield return FlightTests.Airborne(cruise);

            Vessel v = FlightGlobals.ActiveVessel;
            FlightGUI gui = Far.FlightGui(v);
            bool shown = Far.FlightGuiShown, heatmapOpen = Get<bool>(gui, "showHeatmapWindow");
            try
            {
                ctx.Step("Open the flight heatmap window");
                Far.FlightGuiShown = true;
                yield return Imgui.WaitWindow(Far.MainWindow, 30);
                yield return Imgui.Toggle(L("FARFlightGUIHeatmapBtn"), true, Far.MainWindow, exact: true);
                string title = L("FARFlightHeatmapTitle");
                yield return Imgui.WaitWindow(title);
                yield return Wait.Frames(3);

                Assert.False(Imgui.IsVisible(L("FARFlightHeatmapNone").Substring(0, 20), title), "the window has a map to show");
                HeatmapResult editor = swept[0];
                Assert.True(editor.Matches(v), $"the sweep is recognised as this vessel's: '{editor.CraftName}' with {editor.PartCount} physical parts, vessel '{v.vesselName}' with {HeatmapResult.CountPhysicalParts(v)}");
                Assert.False(Imgui.IsVisible(Localizer.Format("FARFlightHeatmapMismatch", editor.CraftName), title), "no mismatch warning");

                object heatmapGui = Get(gui, "_heatmapGUI");
                Assert.True(ReferenceEquals(Get(heatmapGui, "_boundResults"), swept), "the window shows the editor's sweep");
                var display = Get<List<HeatmapResult>>(heatmapGui, "_displayResults");
                Assert.True(display != null && display.Count == swept.Count, "a display map for each swept map");
                HeatmapResult flight = display[display.Count - 1];
                int nM = flight.MachAxis.Length, nA = flight.AltAxis.Length;

                ctx.Step("The map at the vessel's mass");
                double vesselKg = v.totalMass * 1000;
                ctx.Record("mass", $"vessel {vesselKg:F0} kg, map {flight.MassKg:F0}, swept full {editor.SampleMassesKg[2]:F0}, dry {editor.DryMassKg:F0}");
                Assert.Near(vesselKg, flight.MassKg, Math.Max(20, 0.001 * vesselKg), "the flight map is interpolated to the vessel's mass (kg)");
                string massText = "Map at current mass " + Inv(flight.MassKg / 1000, "F2") + " t";
                Assert.True(Imgui.IsVisible(massText, title), $"the window shows '{massText}'");
                Assert.True(vesselKg >= editor.SampleMassesKg[2] * 0.995,
                            $"the vessel at launch is not lighter than the editor's full-tank mass ({vesselKg:F0} against {editor.SampleMassesKg[2]:F0} kg)");
                RecordGrid(ctx, "flight", flight.Envelope, nM, nA);
                AssertSameMap(editor.Envelope, flight.Envelope, nM, nA, "the flight map at full tanks against the editor's full-tank map");
                string ceilingText = double.IsNaN(editor.CeilingM) ? "Ceiling: " : "Ceiling: " + Inv(editor.CeilingM / 1000, "F1") + " km";
                Assert.True(Imgui.IsVisible(ceilingText, title), $"the window shows the editor's ceiling ('{ceilingText}')");

                ctx.Step("The vessel's own point on the map");
                // placed again: at zero throttle it has coasted down since (to Mach 0.54 by here, measured), and the
                // window reads the vessel as it is, nearer the Mach 0.4 column than the 0.8 one
                yield return FarFlightProbe.Place(cruise);
                ctx.Record("placed", $"mach {v.mach:F3}, srfSpeed {v.srfSpeed:F1} m/s, speed of sound {v.speedOfSound:F1} m/s");
                // the vessel's Mach (what the window reads) comes a few frames after its speed off rails
                yield return Wait.Until(() => v.speedOfSound > 0 && Math.Abs(v.mach - v.srfSpeed / v.speedOfSound) < 0.01, 10,
                                        "the vessel's Mach caught up with its speed", shot: false);
                yield return Wait.Frames(3);
                ctx.Record("reading", $"mach {v.mach:F3}, srfSpeed {v.srfSpeed:F1} m/s");
                Imgui.WindowInfo w = Imgui.FindWindow(title);
                string nowPrefix = Localizer.Format("FARFlightHeatmapNow", "\u0001", "\u0002").Split('\u0001')[0];
                string now = w?.Labels.FirstOrDefault(l => l.StartsWith(nowPrefix, StringComparison.Ordinal));
                ctx.Record("now", now);
                Assert.NotNull(now, $"the window shows where the vessel is ('{nowPrefix}...')");
                MatchCollection nums = Regex.Matches(now.Substring(nowPrefix.Length), @"-?[0-9]+(\.[0-9]+)?");
                Assert.True(nums.Count >= 2, "the vessel's Mach and altitude in: " + now);
                Assert.Near(v.mach, double.Parse(nums[0].Value, CultureInfo.InvariantCulture), 0.05, "the Mach the window reads back");
                Assert.Near(v.altitude / 1000, double.Parse(nums[1].Value, CultureInfo.InvariantCulture), 0.3, "the altitude the window reads back (km)");

                int ci = Far.IndexNear(flight.MachAxis, v.mach), cj = Far.IndexNear(flight.AltAxis, v.altitude);
                Cell here = flight.Envelope[ci, cj];
                string verdict = !here.Trimmed ? L("FARFlightHeatmapPredLift") : here.Feasible ? L("FARFlightHeatmapPredOk") : L("FARFlightHeatmapPredThrust");
                string predicted = Localizer.Format("FARFlightHeatmapPred", Inv(here.Mach, "F2"), Inv(flight.AltAxis[cj] / 1000, "F1"), verdict);
                ctx.Record("here", $"cell [{ci},{cj}] {Describe(here)}");
                yield return Imgui.WaitVisible(predicted, title, 10);
                yield return Snap.Shot(ctx, "flight-heatmap");
                Assert.False(Imgui.IsVisible(L("FARFlightHeatmapOutside"), title), "the vessel is inside the swept range");
                Assert.True(here.Feasible, $"a jet at Mach 0.8, 3 km is in its envelope: the nearest cell (Mach {here.Mach:F2}, {flight.AltAxis[cj] / 1000:F1} km) is {Describe(here)}");

                // a click on a cell reaches HandleCellClick through the map renderer's mouse handling, which a test has
                // no mouse for: the call it makes, with the vessel's own cell
                ctx.Step("A cell clicked: its speed and altitude become AtmosphereAutopilot's cruise targets");
                Assert.True(AtmosphereAutopilotInterop.Available, "FAR found AtmosphereAutopilot");
                AaCreateControllers(v);
                typeof(FlightHeatmapGUI).GetMethod("HandleCellClick", Members).Invoke(heatmapGui, new object[] { v, flight, (ci, cj) });
                string sent = Get<string>(heatmapGui, "_logMessage");
                ctx.Record("cruiseTarget", sent);
                Assert.True(sent.StartsWith("Set "), "the window says the targets were set: " + sent);
                AaCruiseTargets(v, out float aaAlt, out float aaSpeed);
                ctx.Record("aaTargets", $"altitude {aaAlt:F0} m, speed {aaSpeed:F1} m/s; cell {flight.AltAxis[cj]:F0} m, {here.SpeedMPerS:F1} m/s");
                Assert.Near(flight.AltAxis[cj], aaAlt, 1, "AA's cruise altitude is the cell's (m)");
                Assert.Near(here.SpeedMPerS, aaSpeed, 0.1, "AA's cruise speed is the cell's (m/s)");

                // the window's prediction check: its button is compiled out (_showPredictionLogButton), so the call it makes
                ctx.Step("The prediction check against the flight, written to the log");
                var logged = new List<string>();
                Application.LogCallback onLog = (m, s, t) => logged.Add(m);
                Application.logMessageReceived += onLog;
                string result;
                try
                {
                    result = (string)typeof(FlightHeatmapGUI).GetMethod("LogPredictionCheck", Members).Invoke(heatmapGui, new object[] { v, flight });
                }
                finally
                {
                    Application.logMessageReceived -= onLog;
                }

                ctx.Record("predictionCheck", result);
                string check = logged.FirstOrDefault(m => m.Contains("=== FAR heatmap prediction check ==="));
                Assert.True(result.StartsWith("Logged to KSP.log"), "the check says it logged: " + result);
                Assert.NotNull(check, "the check in the log");
                Assert.True(check.Contains("-- MEASURED --") && check.Contains(v.vesselName), "the check has the vessel's measured flight");
            }
            finally
            {
                try
                {
                    Set(gui, "showHeatmapWindow", heatmapOpen);
                }
                catch (Exception e)
                {
                    KspTest.Log.Warn("EnvelopeTests: closing the flight heatmap window: " + e.Message);
                }

                Far.FlightGuiShown = shown;
            }
        }

        // ------------------------------------------------------------------------------------------ no AJE

        /// <summary>
        /// Without SolverEngines there is no thrust to sample: the editor offers no Engine Deck tab; the Heatmaps tab
        /// with stability off refuses to run and says SolverEngines is missing; with stability on it sweeps a
        /// stability-only map (3 x 3, Mach 0.4-1.2 x 0-6 km) with no envelope, says so, and neither hands it to the
        /// flight scene nor files it in the craft's history. With SolverEngines the Engine Deck tab is there.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_WithoutSolverEnginesTheMapIsStabilityOnly(TestContext ctx)
        {
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                var tabs = (string[])Get(Get(Far.Editor, "modeDropdown"), "stringOptions");
                bool deckTab = tabs.Any(t => t.Trim() == L("FAREditorModeEngineDeck"));
                ctx.Record("solverEngines", SolverEnginesInterop.Available);
                Assert.Equal(SolverEnginesInterop.Available, deckTab, "the Engine Deck tab is offered exactly when SolverEngines is there");
                if (SolverEnginesInterop.Available)
                {
                    ctx.Record("skipped", "SolverEngines is loaded: the stability-only fallback cannot be reached");
                    yield break;
                }

                HeatmapsGUI hm = Heatmaps;
                PrepareHeatmaps(hm);
                yield return SetHeatmapToggles(false, true);
                string[] fields = { "0.4", "1.2", "0", "6", "3", "3" };
                for (int i = 0; i < fields.Length; i++)
                    yield return Imgui.SetTextNth(i, fields[i], Window);

                ctx.Step("Stability off: nothing to sweep");
                object before = Get(hm, "_results");
                List<HeatmapResult> cachedBefore = HeatmapResult.Cached;
                int historyBefore = HeatmapPersistence.ListForVessel(Far.Plane).Count;
                yield return Imgui.Click(L("FARStabHeatmapRun"), Window);
                Assert.False(hm.IsSweepRunning, "no sweep runs without engines or stability");
                Assert.True(ReferenceEquals(before, Get(hm, "_results")), "the maps shown are left alone");
                Assert.Equal(L("FAREngineDeckNoSolver"), Get<string>(hm, "_statusMessage").Trim(), "the status line");
                yield return Wait.Frames(3);
                Assert.True(Imgui.IsVisible(L("FAREngineDeckNoSolver"), Window), "the tab says SolverEngines is missing");

                ctx.Step("Stability on: a stability-only map");
                yield return Imgui.Toggle(L("FARHeatmapsIncludeStab"), true, Window);
                yield return Imgui.Click(L("FARStabHeatmapRun"), Window);
                yield return Wait.Until(() => !hm.IsSweepRunning, 600, "the stability sweep", shot: false);
                yield return Snap.Shot(ctx, "stability-only-map");
                var results = Get<List<HeatmapResult>>(hm, "_results");
                Assert.True(results != null && !ReferenceEquals(results, before), "the Run button swept a new map");
                Assert.True(Get<bool>(hm, "_stabilityOnly"), "the sweep is stability-only");
                Assert.Equal(1, results.Count, "one (stability) result");
                HeatmapResult r = results[0];
                Assert.True(r.Envelope == null && r.EnvelopeSamples == null, "no envelope without thrust");
                Assert.True(r.HasStability && r.Stability != null, "the stability map");
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        Assert.True(r.Stability[i, j] != null && r.Stability[i, j].Score >= 0, $"the stability cell [{i},{j}] evaluated");
                Assert.Equal(0, Get<IList>(hm, "_envRenderers").Count, "no envelope maps drawn");
                string status = Get<string>(hm, "_statusMessage");
                ctx.Record("status", status);
                Assert.True(status.Contains("stability map only"), "the status line says the map is stability-only: " + status);
                yield return Wait.Frames(3);
                Assert.True(Imgui.IsVisible("stability map only", Window), "the tab says so");
                Assert.True(Imgui.IsVisible("Stability derivatives", Window), "the tab draws the stability map");
                Assert.True(ReferenceEquals(cachedBefore, HeatmapResult.Cached), "a stability-only map is not handed to the flight scene");
                Assert.Equal(historyBefore, HeatmapPersistence.ListForVessel(Far.Plane).Count, "nor filed in the craft's sweep history");
            }
            finally
            {
                CloseWindow();
            }
        }

        // ------------------------------------------------------------------------------------------ helpers: tabs

        private sealed class DeckOut
        {
            public EditorEngineDeck.Sample[,] Samples;
            public double[] Mach, Alt;
            public string Status;
            public int Engines, Inlets;
        }

        /// <summary>The Engine Deck tab's fields typed in (throttle, Mach min/max, altitude min/max km, cells) and sampled.</summary>
        private static IEnumerator SampleDeck(string[] fields, DeckOut o)
        {
            SelectTab("FAREditorModeEngineDeck");
            yield return Wait.Frames(2);
            object deckGui = Get(Far.Editor, "_engineDeck");
            Remember(deckGui, "_throttle", "_machMin", "_machMax", "_altMinKm", "_altMaxKm", "_nMach", "_nAlt", "_showFuelFlow");
            for (int i = 0; i < fields.Length; i++)
                yield return Imgui.SetTextNth(i, fields[i], Window);
            object before = Get(deckGui, "_samples");
            yield return Imgui.Click(L("FAREngineDeckRun"), Window, exact: true);
            o.Samples = (EditorEngineDeck.Sample[,])Get(deckGui, "_samples");
            o.Status = Get<string>(deckGui, "_statusMessage");
            Assert.True(o.Samples != null && !ReferenceEquals(o.Samples, before), "the Sample button sampled the engines: " + o.Status);
            o.Mach = (double[])Get(deckGui, "_machAxis");
            o.Alt = (double[])Get(deckGui, "_altAxis");
            var deck = (EditorEngineDeck)Get(deckGui, "_deck");
            o.Engines = deck.EngineCount;
            o.Inlets = deck.InletCount;
        }

        // ------------------------------------------------------------------------------------------ helpers: AtmosphereAutopilot

        private static Type Aa(string name)
        {
            AssemblyLoader.LoadedAssembly aa = AssemblyLoader.loadedAssemblies.FirstOrDefault(a => a.assembly.GetName().Name == "AtmosphereAutopilot");
            Assert.NotNull(aa, "AtmosphereAutopilot loaded");
            return aa.assembly.GetType("AtmosphereAutopilot." + name) ?? throw new UITestFailure("AA has no type " + name);
        }

        /// <summary>AA's modules for the vessel (AtmosphereAutopilot.getVesselModules), by type.</summary>
        private static IDictionary AaModules(Vessel v)
        {
            Type autopilot = Aa("AtmosphereAutopilot");
            object instance = autopilot.GetProperty("Instance", Members)?.GetValue(null, null);
            Assert.NotNull(instance, "AA's instance");
            var modules = (IDictionary)autopilot.GetMethod("getVesselModules", new[] { typeof(Vessel) }).Invoke(instance, new object[] { v });
            Assert.True(modules != null && modules.Contains(Aa("TopModuleManager")), "AA's module manager for the vessel");
            return modules;
        }

        /// <summary>
        /// AA's controllers made for the vessel, as its master switch first makes them (TopModuleManager.create_context),
        /// without switching anything on: until then AA has only its module manager, and FAR has no cruise to set.
        /// </summary>
        private static void AaCreateControllers(Vessel v)
        {
            IDictionary modules = AaModules(v);
            if (!modules.Contains(Aa("CruiseController")))
                Aa("TopModuleManager").GetMethod("create_context", Members).Invoke(modules[Aa("TopModuleManager")], null);
            Assert.True(modules.Contains(Aa("CruiseController")) && modules.Contains(Aa("ProgradeThrustController")),
                        "AA's cruise and thrust controllers on the vessel");
        }

        /// <summary>AtmosphereAutopilot's cruise altitude (CruiseController) and speed (ProgradeThrustController) for the vessel, read from AA.</summary>
        private static void AaCruiseTargets(Vessel v, out float altitudeM, out float speed)
        {
            IDictionary modules = AaModules(v);
            PropertyInfo value = Aa("DelayedFieldFloat").GetProperty("Value");
            altitudeM = (float)value.GetValue(Get(modules[Aa("CruiseController")], "desired_altitude"), null);
            speed = (float)value.GetValue(Get(modules[Aa("ProgradeThrustController")], "setpoint_field"), null);
        }

        /// <summary>The static analysis' Cd at Mach 0.5, 2 deg, after a few calls (the wings' stall state settles).</summary>
        private static double SteadyCd()
        {
            double cd = 0;
            for (int k = 0; k < 4; k++)
                cd = Far.Instant(2, 0.5).Cd;
            return cd;
        }

        /// <summary>The number AJE's tooltip gives for an engine's static thrust (wet if it has an afterburner), kN; NaN if none.</summary>
        private static double TooltipStaticThrustKN(Part p)
        {
            if (p.partInfo?.moduleInfos == null)
                return double.NaN;
            foreach (string pattern in new[] { @"Static Thrust \(wet\):\s*(?:</b>)?\s*([0-9][0-9,]*(?:\.[0-9]+)?)\s*kN", @"Static Thrust:\s*(?:</b>)?\s*([0-9][0-9,]*(?:\.[0-9]+)?)\s*kN" })
                foreach (AvailablePart.ModuleInfo mi in p.partInfo.moduleInfos)
                {
                    Match m = Regex.Match(mi.info ?? "", pattern);
                    if (m.Success)
                        return double.Parse(m.Groups[1].Value.Replace(",", ""), CultureInfo.InvariantCulture);
                }

            return double.NaN;
        }

        // ------------------------------------------------------------------------------------------ helpers: maps

        private static void AssertAxes(HeatmapResult r, double[] machs, double[] alts)
        {
            Assert.Equal(machs.Length, r.MachAxis.Length, "Mach columns");
            Assert.Equal(alts.Length, r.AltAxis.Length, "altitude rows");
            Assert.False(r.SpeedAxisMode, "the x axis is Mach");
            for (int i = 0; i < machs.Length; i++)
                Assert.Near(machs[i], r.MachAxis[i], 1e-9, $"Mach column {i}");
            for (int j = 0; j < alts.Length; j++)
                Assert.Near(alts[j], r.AltAxis[j], 1e-6, $"altitude row {j} (m)");
        }

        /// <summary>Two displayed maps the same cell for cell: what trims, flies, is hidden and cruises, at the same drag.</summary>
        private static void AssertSameMap(Cell[,] expected, Cell[,] actual, int nM, int nA, string what)
        {
            var diffs = new List<string>();
            for (int i = 0; i < nM; i++)
                for (int j = 0; j < nA; j++)
                {
                    Cell e = expected[i, j], a = actual[i, j];
                    bool same = e.Trimmed == a.Trimmed && e.Feasible == a.Feasible && e.Suppressed == a.Suppressed &&
                                (!e.Feasible || e.Cruise == a.Cruise) &&
                                (!e.Trimmed || Math.Abs(e.DragN - a.DragN) <= 1e-6 * Math.Abs(e.DragN) + 1e-6);
                    if (!same)
                        diffs.Add($"[{i},{j}] {Describe(e)} -> {Describe(a)}");
                }

            Assert.True(diffs.Count == 0, $"{what}: {diffs.Count} cells differ: {string.Join("; ", diffs.Take(6))}");
        }

        private static double HighestFeasible(Cell[,] grid, double[] alts)
        {
            double best = double.NaN;
            for (int i = 0; i < grid.GetLength(0); i++)
                for (int j = 0; j < grid.GetLength(1); j++)
                    if (grid[i, j].Feasible && (double.IsNaN(best) || alts[j] > best))
                        best = alts[j];
            return best;
        }

        private static int CountFeasible(Cell[,] grid, int nM, int nA)
        {
            int n = 0;
            for (int i = 0; i < nM; i++)
                for (int j = 0; j < nA; j++)
                    if (grid[i, j].Feasible)
                        n++;
            return n;
        }

        private static string Describe(Cell c)
        {
            string kind = !c.Trimmed ? "no trim" :
                          c.Suppressed ? "hidden" :
                          c.Feasible ? (c.Cruise == PerformanceEnvelopeCalculator.CruiseState.Ok ? "cruise" : c.Cruise.ToString()) :
                          c.Overheats ? "heat wall" : "thrust-short";
            if (!c.Trimmed)
                return kind;
            return string.Format(CultureInfo.InvariantCulture, "{0}{1}, AoA {2:F2}, drag {3:F2} kN, thrust {4:F2} kN, throttle {5:F2}",
                                 kind, c.Extrapolated ? " (extrapolated)" : "", c.Alpha, c.DragN / 1000, c.ThrustAvailableN / 1000, c.ThrottleRequired);
        }

        /// <summary>The grid, top row first: '.' no trim, 'x' hidden, 'F' cruise, 'i' flyable without a cruise, 'H' heat wall, 'g' thrust-short.</summary>
        private static void RecordGrid(TestContext ctx, string name, Cell[,] grid, int nM, int nA)
        {
            var sb = new StringBuilder();
            for (int j = nA - 1; j >= 0; j--)
            {
                for (int i = 0; i < nM; i++)
                {
                    Cell c = grid[i, j];
                    sb.Append(!c.Trimmed ? '.' :
                              c.Suppressed ? 'x' :
                              c.Feasible ? (c.Cruise == PerformanceEnvelopeCalculator.CruiseState.Ok ? 'F' : 'i') :
                              c.Overheats ? 'H' : 'g');
                }

                if (j > 0)
                    sb.Append('/');
            }

            ctx.Record("grid." + name, sb.ToString());
        }

    }
}
