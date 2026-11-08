using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using KSP.Localization;
using KspTest;
using Cell = FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation.PerformanceEnvelopeCalculator.CellResult;
using static FerramAerospaceResearch.KspTests.Refl;
using static FerramAerospaceResearch.KspTests.EditorWindow;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The Heatmaps tab's sweep at its edges, on the stock Aeris 4A with AJE (its Whiplashes made J58 turbojets):
    /// the three ways it can run (on the main thread, on worker threads, and on worker threads with the engines
    /// sampled on the main thread, as with a SolverEngines too old to clone) giving one map; rows above the
    /// atmosphere and the heat wall past the jets' top speed; the Cancel button; an edit or leaving the editor
    /// while the workers run; ranges it refuses; and the history of sweeps on disk.
    /// </summary>
    public static class EnvelopeSweepTests
    {
        private const string AjeMods = "AdvancedJetEngine";
        private const string AjeRequires = "AJE";

        private static void AssertNull(object o, string what) => Assert.True(o == null, what + $" (is {o})");

        // ------------------------------------------------------------------------------------------ the three paths

        /// <summary>
        /// One grid, Mach 0.5-6 x 0-80 km, 12 x 5, with stability, swept three ways: serially, multithreaded, and
        /// multithreaded with SolverEngines' engine solvers not cloneable (the parallel trim, then the engines on the
        /// main thread). Each counts every cell it promised. The 80 km row is above Kerbin's atmosphere: blank in all
        /// three (no trim, no stability cell). Past the jets' heat wall the serial and multithreaded sweeps stop
        /// solving and fill the rest of the row as heat wall (the same cells in both); the fallback solves every
        /// cell, and those it solves come out trimmed but not flyable. Everything else is the same map in all three:
        /// what trims, flies and overheats, at the same angle of attack, drag and thrust, with the same stability
        /// flags; and the multithreaded sweep's engines hit their temperature limit somewhere (its per-engine cap).
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 1800)]
        public static IEnumerator Envelope_AllSweepPathsAgreeAtTheEdges(TestContext ctx)
        {
            var caches = new List<string>();
            PropertyInfo parallel = typeof(SolverEnginesInterop).GetProperty(nameof(SolverEnginesInterop.ParallelAvailable), Members);
            bool parallelAtStart = SolverEnginesInterop.ParallelAvailable;
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                CelestialBody body = FlightGlobals.GetHomeBody();
                string[] fields = { "0.5", "6", "0", "80", "12", "5" };
                Assert.True(parallelAtStart, "this SolverEngines' engine solvers can be cloned (the multithreaded engine path)");

                ctx.Step("Serial sweep");
                var serial = new SweepOut();
                yield return Sweep(fields, true, false, serial);
                caches.Add(serial.CachePath);

                ctx.Step("Multithreaded sweep");
                var mt = new SweepOut();
                yield return Sweep(fields, true, true, mt);
                caches.Add(mt.CachePath);

                ctx.Step("Multithreaded sweep, engine solvers not cloneable");
                var fallback = new SweepOut();
                parallel.GetSetMethod(true).Invoke(null, new object[] { false });
                try
                {
                    yield return Sweep(fields, true, true, fallback);
                }
                finally
                {
                    parallel.GetSetMethod(true).Invoke(null, new object[] { parallelAtStart });
                }

                caches.Add(fallback.CachePath);

                SweepOut[] runs = { serial, mt, fallback };
                string[] names = { "serial", "multithreaded", "fallback" };
                for (int s = 0; s < runs.Length; s++)
                {
                    Assert.False(runs[s].StabilityOnly, $"the {names[s]} sweep is an envelope sweep");
                    Assert.Equal(1, runs[s].Results.Count, $"one map from the {names[s]} sweep");
                    Assert.Equal(runs[s].CellsTotal, runs[s].CellsDone, $"the {names[s]} sweep counted every cell it promised");
                    Assert.Equal(Localizer.Format("FAREnvelopeDone", runs[s].CellsDone.ToString(CultureInfo.InvariantCulture)),
                                 runs[s].Status, $"the status line after the {names[s]} sweep");
                    HeatmapResult r = runs[s].Results[0];
                    for (int k = 0; k < r.EnvelopeSamples.Length; k++)
                        Maps.RecordGrid(ctx, $"{names[s]}{k}", r.EnvelopeSamples[k], r.MachAxis.Length, r.AltAxis.Length);
                }

                HeatmapResult a = serial.Results[0], b = mt.Results[0], c = fallback.Results[0];
                int nM = a.MachAxis.Length, nA = a.AltAxis.Length;
                Assert.Equal(12, nM, "Mach columns");
                Assert.Equal(5, nA, "altitude rows");

                ctx.Step("The row above the atmosphere: blank, no stability");
                int top = nA - 1;
                Assert.True(a.AltAxis[top] > body.atmosphereDepth, $"the top row ({a.AltAxis[top]:F0} m) is above the atmosphere ({body.atmosphereDepth:F0} m)");
                for (int s = 0; s < runs.Length; s++)
                {
                    HeatmapResult r = runs[s].Results[0];
                    for (int i = 0; i < nM; i++)
                    {
                        for (int k = 0; k < r.EnvelopeSamples.Length; k++)
                        {
                            Cell e = r.EnvelopeSamples[k][i, top];
                            Assert.True(!e.Trimmed && !e.Feasible && !e.Extrapolated && e.SpeedMPerS == 0,
                                        $"the {names[s]} sweep left Mach column {i} above the atmosphere blank (mass sample {k}: {Maps.Describe(e)})");
                        }

                        Assert.True(r.Stability[i, top] == null, $"no stability cell above the atmosphere (the {names[s]} sweep, Mach column {i})");
                    }
                }

                ctx.Step("Serial against multithreaded: the same map, the same heat-wall tail");
                Maps.Compared sm = Maps.Compare(a, b, "multithreaded against serial", fallbackSolvesTail: false);
                ctx.Record("serialVsMt", sm.ToString());
                Assert.True(sm.Extrapolated > 0, "the heat wall's tail was filled in, not solved (cells past three heat-wall cells in a row)");
                Assert.True(sm.Cells >= 10, $"enough cells trimmed well below the stall to compare ({sm.Cells})");

                ctx.Step("Serial against the fallback: the same map, its tail solved and not flyable");
                Maps.Compared sf = Maps.Compare(a, c, "the fallback against serial", fallbackSolvesTail: true);
                ctx.Record("serialVsFallback", sf.ToString());
                Assert.Equal(sm.Extrapolated, sf.TailSolved, "the fallback solved each cell the serial sweep filled in");
                for (int k = 0; k < c.EnvelopeSamples.Length; k++)
                    for (int i = 0; i < nM; i++)
                        for (int j = 0; j < nA; j++)
                            Assert.False(c.EnvelopeSamples[k][i, j].Extrapolated, $"the fallback fills nothing in (Mach column {i}, row {j}, mass sample {k})");

                ctx.Step("The multithreaded engines' temperature cap");
                int hot = 0;
                for (int k = 0; k < b.EnvelopeSamples.Length; k++)
                    for (int i = 0; i < nM; i++)
                        for (int j = 0; j < nA; j++)
                            if (b.EnvelopeSamples[k][i, j].Overheats && !b.EnvelopeSamples[k][i, j].Extrapolated)
                                hot++;
                ctx.Record("overheatingCells", hot);
                Assert.True(hot > 0, "the multithreaded sweep solved cells where the jets reach their temperature limit");

                ctx.Step("The stability flags, all three");
                for (int i = 0; i < nM; i++)
                    for (int j = 0; j < top; j++)
                    {
                        string fa = Maps.Flags(a.Stability[i, j]), fb = Maps.Flags(b.Stability[i, j]), fc = Maps.Flags(c.Stability[i, j]);
                        Assert.True(a.Stability[i, j] != null && a.Stability[i, j].Score >= 0, $"the stability cell [{i},{j}] evaluated");
                        Assert.Equal(fa, fb, $"the stability flags at [{i},{j}], multithreaded against serial");
                        Assert.Equal(fa, fc, $"the stability flags at [{i},{j}], the fallback against serial");
                    }
            }
            finally
            {
                if (SolverEnginesInterop.ParallelAvailable != parallelAtStart)
                    parallel.GetSetMethod(true).Invoke(null, new object[] { parallelAtStart });
                foreach (string p in caches)
                    DeleteCache(p);
                CloseWindow();
            }
        }

        // ------------------------------------------------------------------------------------------ cancel

        /// <summary>
        /// The Cancel button, a few frames into a sweep: a 20 x 20 multithreaded sweep, a serial sweep with stability
        /// (cancelled in its stability pass, which runs first) and one without (in its envelope pass), and a sweep with
        /// auto-fit on (cancelled in its coarse probe). Each stops within seconds, says it was cancelled, files nothing
        /// in the craft's history and hands nothing to the flight scene; the cancelled probe puts the user's range
        /// and cell counts back in the fields.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 1200)]
        public static IEnumerator Envelope_CancelStopsTheSweep(TestContext ctx)
        {
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                HeatmapsGUI hm = Heatmaps;
                string craft = EditorLogic.fetch.ship.shipName;
                string[] big = { "0.3", "3", "0", "30", "20", "20" };

                ctx.Step("Multithreaded, cancelled in the parallel pass");
                yield return CancelledSweep(ctx, hm, craft, big, false, true, false,
                                            () => Get<Task>(hm, "_sweepTask") != null, "the worker threads running");

                ctx.Step("Serial with stability, cancelled in the stability pass");
                yield return CancelledSweep(ctx, hm, craft, big, true, false, false,
                                            () => Get<int>(hm, "_cellsDone") > 0, "the first stability cells done");

                ctx.Step("Serial without stability, cancelled in the envelope pass");
                yield return CancelledSweep(ctx, hm, craft, big, false, false, false,
                                            () => Get<int>(hm, "_cellsDone") > 0, "the first envelope cells done");

                ctx.Step("Auto-fit, cancelled in its probe");
                string[] small = { "0.3", "1", "0", "5", "4", "3" };
                yield return CancelledSweep(ctx, hm, craft, small, false, false, true,
                                            () => Get<bool>(hm, "_autoFitProbe") && Get<int>(hm, "_cellsDone") > 0, "the probe under way");
                Assert.False(Get<bool>(hm, "_autoFitProbe"), "the probe over");
                string[] names = { "_machMin", "_machMax", "_altMinKm", "_altMaxKm", "_nMach", "_nAlt" };
                for (int i = 0; i < names.Length; i++)
                    Assert.Equal(small[i], Get<string>(hm, names[i]), $"the user's {names[i]} back in its field after the cancelled probe");
            }
            finally
            {
                CloseWindow();
            }
        }

        /// <summary>A sweep started, cancelled with the button once it is under way, and checked to have stopped and left nothing.</summary>
        private static IEnumerator CancelledSweep(TestContext ctx, HeatmapsGUI hm, string craft, string[] fields, bool stability, bool multithread,
                                                  bool autoFit, Func<bool> underWay, string what)
        {
            int history = HeatmapPersistence.ListForVessel(craft).Count;
            List<HeatmapResult> cached = HeatmapResult.Cached;
            yield return StartSweep(fields, stability, multithread, autoFit);
            yield return Wait.Until(() => underWay() || !hm.IsSweepRunning, 300, what, shot: false);
            Assert.True(hm.IsSweepRunning, $"the sweep still running once {what} ({Get<string>(hm, "_statusMessage")})");
            yield return Imgui.Click(L("FARStabHeatmapCancel"), Window, exact: true);
            yield return Wait.Until(() => !hm.IsSweepRunning, 120, "the cancelled sweep to stop", shot: false);
            int done = Get<int>(hm, "_cellsDone"), total = Get<int>(hm, "_cellsTotal");
            ctx.Record("cancelled", $"{done} of {total} cells");
            Assert.Equal(L("FAREnvelopeCancelled"), Get<string>(hm, "_statusMessage").Trim(), "the status line");
            Assert.True(done < total, $"cancelled before the end ({done} of {total} cells)");
            AssertNull(Get<Task>(hm, "_sweepTask"), "no worker task left");
            Assert.True(ReferenceEquals(cached, HeatmapResult.Cached), "nothing handed to the flight scene");
            Assert.Equal(history, HeatmapPersistence.ListForVessel(craft).Count, "nothing filed in the craft's history");
            yield return Wait.Frames(3);
            Assert.True(Imgui.IsVisible(L("FAREnvelopeCancelled"), Window), "the tab says the sweep was cancelled");
            Assert.True(Imgui.IsVisible(L("FARStabHeatmapRun"), Window), "the Run button is back");
        }

        /// <summary>The Heatmaps tab's fields typed in and Run pressed, without waiting for the sweep.</summary>
        private static IEnumerator StartSweep(string[] fields, bool stability, bool multithread, bool autoFit = false)
        {
            SelectTab("FAREditorModeHeatmaps");
            yield return Wait.Frames(2);
            HeatmapsGUI hm = Heatmaps;
            PrepareHeatmaps(hm);
            yield return SetHeatmapToggles(stability, multithread, autoFit);
            for (int i = 0; i < fields.Length; i++)
                yield return Imgui.SetTextNth(i, fields[i], Window);
            yield return Imgui.Click(L("FARStabHeatmapRun"), Window);
            Assert.True(hm.IsSweepRunning, "the Run button started a sweep");
        }

        // ------------------------------------------------------------------------------------------ edits mid-sweep

        /// <summary>
        /// The worker threads of a multithreaded sweep read the craft's parts, so an edit stops them before it lands:
        /// a part moved (onEditorPartEvent), the ship modified (onEditorShipModified) and a voxelization requested
        /// each stop the sweep, the first two joining its workers in the same call, and the sweep says it was
        /// cancelled. Leaving the editor mid-sweep stops it too: no FAR exception, the geometry snapshot the workers
        /// read released and the engines' solver clones dropped.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 1200)]
        public static IEnumerator Envelope_EditingOrLeavingMidSweepJoinsTheWorkers(TestContext ctx)
        {
            var log = new ExceptionLog();
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                HeatmapsGUI hm = Heatmaps;
                string[] big = { "0.3", "3", "0", "30", "20", "20" };
                Part wing = EditorLogic.fetch.ship.parts.First(p => p.Modules.Contains("FARWingAerodynamicModel"));

                ctx.Step("A part moved mid-sweep");
                yield return WorkersRunning(hm, big);
                GameEvents.onEditorPartEvent.Fire(ConstructionEventType.PartOffset, wing);
                AssertJoined(hm, "a part event");
                yield return Stopped(hm, "a part event");
                yield return Far.WaitForEditorVoxels();

                ctx.Step("The ship modified mid-sweep");
                yield return WorkersRunning(hm, big);
                GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
                AssertJoined(hm, "a ship-modified event");
                yield return Stopped(hm, "a ship-modified event");
                yield return Far.WaitForEditorVoxels();

                ctx.Step("A voxelization requested mid-sweep");
                yield return WorkersRunning(hm, big);
                EditorGUI.RequestUpdateVoxel();
                yield return Stopped(hm, "a voxelization request");
                yield return Far.WaitForEditorVoxels();

                ctx.Step("The editor left mid-sweep");
                yield return WorkersRunning(hm, big);
                var sim = (EditorSimManager)Get(Far.Editor, "_simManager");
                object instant = Get(sim.EnvelopeCalculator, "_instantCondition");
                object deck = Get(hm, "_deck");
                Assert.True(Get<bool>(instant, "_simActive"), "the sweep holds the geometry snapshot");
                Assert.NotNull(Get(deck, "_clones"), "the sweep holds the engines' solver clones");
                yield return Nav.ToSpaceCenter(ctx);
                yield return Wait.Seconds(2);
                Assert.False(hm.IsSweepRunning, "the sweep stopped with the editor");
                Task t = Get<Task>(hm, "_sweepTask");
                Assert.True(t == null || t.IsCompleted, "its workers joined");
                Assert.False(Get<bool>(instant, "_simActive"), "the geometry snapshot released");
                AssertNull(Get(deck, "_clones"), "the solver clones dropped");
                log.AssertNone("stopping sweeps mid-way");
            }
            finally
            {
                log.Dispose();
                if (EditorGUI.Instance != null)
                    CloseWindow();
            }
        }

        /// <summary>A 20 x 20 multithreaded sweep started and waited on until its worker threads run.</summary>
        private static IEnumerator WorkersRunning(HeatmapsGUI hm, string[] fields)
        {
            yield return StartSweep(fields, false, true);
            yield return Wait.Until(() => Get<Task>(hm, "_sweepTask") != null || !hm.IsSweepRunning, 300, "the sweep's worker threads", shot: false);
            Assert.True(hm.IsSweepRunning && Get<Task>(hm, "_sweepTask") != null, "the sweep's workers running");
        }

        private static void AssertJoined(HeatmapsGUI hm, string what)
        {
            Task t = Get<Task>(hm, "_sweepTask");
            Assert.True(Get<bool>(hm, "_cancelRequested"), $"{what} cancels the sweep");
            Assert.True(t == null || t.IsCompleted, $"{what} joins the sweep's workers before returning");
        }

        private static IEnumerator Stopped(HeatmapsGUI hm, string what)
        {
            yield return Wait.Until(() => !hm.IsSweepRunning, 120, $"the sweep to stop after {what}", shot: false);
            Assert.Equal(L("FAREnvelopeCancelled"), Get<string>(hm, "_statusMessage").Trim(), $"the status line after {what}");
        }

        // ------------------------------------------------------------------------------------------ bad ranges

        /// <summary>
        /// Ranges that cannot be swept are refused with the reason. On the Heatmaps tab the Run button greys out and
        /// the reason shows beside it as it is typed: a range that is not a number, a decreasing or non-positive
        /// Mach range, a negative or decreasing altitude range, a cell count under 2 or over 100. The Engine Deck
        /// says why when Sample is pressed: a non-numeric or decreasing Mach range, a bad altitude range, cell counts
        /// outside 2-12 and a throttle outside 0-1, sampling nothing.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_BadRangesAreRefusedWithTheReason(TestContext ctx)
        {
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                HeatmapsGUI hm = Heatmaps;
                PrepareHeatmaps(hm);
                yield return SetHeatmapToggles(false, true);
                string[] good = { "0.3", "2", "0", "15", "4", "4" };

                ctx.Step("Heatmaps: each bad range greys Run out, with its reason");
                var cases = new[]
                {
                    (0, "abc", "Mach range must be numeric."),
                    (1, "", "Mach range must be numeric."),
                    (1, "0.2", "Mach range must be increasing and above zero."),
                    (0, "0", "Mach range must be increasing and above zero."),
                    (2, "x", "Altitude range must be numeric."),
                    (2, "-1", "Altitude range must be increasing and non-negative."),
                    (3, "0", "Altitude range must be increasing and non-negative."),
                    (4, "1", "Cell counts must be integers between 2 and 100."),
                    (5, "101", "Cell counts must be integers between 2 and 100."),
                    (4, "3.5", "Cell counts must be integers between 2 and 100."),
                };
                foreach ((int field, string text, string reason) in cases)
                {
                    for (int i = 0; i < good.Length; i++)
                        yield return Imgui.SetTextNth(i, i == field ? text : good[i], Window);
                    yield return Wait.Frames(3);
                    string at = $"'{text}' in field {field}";
                    Assert.True(Imgui.IsVisible(reason, Window), $"the tab says '{reason}' for {at}");
                    Assert.False(RunEnabled(L("FARStabHeatmapRun")), $"Run greyed out for {at}");
                }

                ctx.Step("In m/s the reason names the speed range");
                for (int i = 0; i < good.Length; i++)
                    yield return Imgui.SetTextNth(i, good[i], Window);
                yield return SetHeatmapToggles(false, true, speedInMS: true);
                yield return Imgui.SetTextNth(0, "fast", Window);
                yield return Wait.Frames(3);
                Assert.True(Imgui.IsVisible("Speed range must be numeric.", Window), "the reason for a bad speed range");
                yield return SetHeatmapToggles(false, true);

                ctx.Step("Good ranges: Run enabled, no reason shown");
                for (int i = 0; i < good.Length; i++)
                    yield return Imgui.SetTextNth(i, good[i], Window);
                yield return Wait.Frames(3);
                Assert.True(RunEnabled(L("FARStabHeatmapRun")), "Run enabled for a good range");
                Assert.False(Imgui.IsVisible("must be", Window), "no reason shown for a good range");

                ctx.Step("Engine Deck: each bad range refused on Sample");
                SelectTab("FAREditorModeEngineDeck");
                yield return Wait.Frames(2);
                object deckGui = Get(Far.Editor, "_engineDeck");
                Remember(deckGui, "_throttle", "_machMin", "_machMax", "_altMinKm", "_altMaxKm", "_nMach", "_nAlt", "_showFuelFlow", "_samples", "_statusMessage");
                // throttle, Mach min, max, altitude min, max km, cells along each
                string[] deckGood = { "1", "0", "2", "0", "15", "3", "3" };
                var deckCases = new[]
                {
                    (1, "a", "Mach range must be numeric."),
                    (2, "0", "Mach range must be increasing and non-negative."),
                    (1, "-0.5", "Mach range must be increasing and non-negative."),
                    (4, "km", "Altitude range must be numeric."),
                    (3, "-2", "Altitude range must be increasing and non-negative."),
                    (5, "1", "Cell counts must be integers between 2 and 12."),
                    (6, "13", "Cell counts must be integers between 2 and 12."),
                    (0, "1.5", "Throttle must be between 0 and 1."),
                    (0, "-0.1", "Throttle must be between 0 and 1."),
                    (0, "full", "Throttle must be between 0 and 1."),
                };
                foreach ((int field, string text, string reason) in deckCases)
                {
                    for (int i = 0; i < deckGood.Length; i++)
                        yield return Imgui.SetTextNth(i, i == field ? text : deckGood[i], Window);
                    Set(deckGui, "_samples", null);
                    yield return Imgui.Click(L("FAREngineDeckRun"), Window, exact: true);
                    string at = $"'{text}' in field {field}";
                    Assert.Equal(reason, Get<string>(deckGui, "_statusMessage"), $"the Engine Deck's status for {at}");
                    AssertNull(Get(deckGui, "_samples"), $"nothing sampled for {at}");
                    yield return Wait.Frames(2);
                    Assert.True(Imgui.IsVisible(reason, Window), $"the tab shows '{reason}' for {at}");
                }
            }
            finally
            {
                CloseWindow();
            }
        }

        /// <summary>Whether the editor window's button with this label is drawn enabled.</summary>
        private static bool RunEnabled(string label)
        {
            Imgui.WindowInfo w = Imgui.FindWindow(Window);
            Assert.NotNull(w, "FAR's editor window drawn");
            Imgui.Control b = w.Controls.FirstOrDefault(x => x.Label != null && x.Label.Trim() == label);
            Assert.NotNull(b, $"the '{label}' button drawn");
            return b.Enabled;
        }

        // ------------------------------------------------------------------------------------------ history

        /// <summary>
        /// The history of sweeps on disk, as the tab finds it again. After a sweep, with the tab's maps dropped (as
        /// when the craft is loaded again), the tab shows "Loaded cached heatmap" and the newest sweep for the
        /// craft's part count, read back the same as swept. Files it cannot read are not listed: a garbage file and
        /// one with the right header but another format version, named as the craft's; reading either (or a
        /// missing file) gives nothing; Clear History deletes them with the rest. A sweep whose file went from disk
        /// behind the tab's back is dropped when browsed to. Removing the last sweep says none are cached now.
        /// Renaming the craft to one without sweeps clears the maps. More than 20 sweeps of a craft: the oldest
        /// files go, 20 kept.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 1200)]
        public static IEnumerator Envelope_HistoryReloadsAndSurvivesBadFiles(TestContext ctx)
        {
            const string Back = "◄";
            ShipConstruct ship = EditorLogic.fetch.ship;
            string craft = ship.shipName, pruneCraft = "FAR test prune " + Guid.NewGuid().ToString("N").Substring(0, 6);
            var made = new List<string>();
            SweepOut first = null, second = null, third = null;
            string dir = Path.Combine(KSPUtil.ApplicationRootPath, "GameData", "FerramAerospaceResearch", "PluginData", "Heatmaps");
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                HeatmapsGUI hm = Heatmaps;
                // what an earlier test left behind
                HeatmapPersistence.DeleteForVessel(craft);

                ctx.Step("Two sweeps filed");
                first = new SweepOut();
                yield return Sweep(new[] { "0.4", "1.2", "0", "6", "3", "3" }, false, true, first);
                second = new SweepOut();
                yield return Sweep(new[] { "0.6", "1.6", "0", "9", "3", "3" }, false, true, second);
                Assert.Equal(2, HeatmapPersistence.ListForVessel(craft).Count, "both sweeps on disk");

                ctx.Step("The maps dropped: the tab loads the newest sweep from disk");
                Set(hm, "_results", null);
                Set(hm, "_editorLoadTriedSig", null);
                yield return Wait.Until(() => Get<List<HeatmapResult>>(hm, "_results") != null, 10, "the tab to load a cached sweep", shot: false);
                Assert.Equal("Loaded cached heatmap for " + craft + ".", Get<string>(hm, "_statusMessage"), "the status line");
                Assert.Equal(1, Get<int>(hm, "_viewIndex"), "the newest sweep shown");
                Maps.AssertSameSweep(second.Results, Get<List<HeatmapResult>>(hm, "_results"), "the newest sweep loaded back");
                yield return Wait.Frames(3);
                Assert.True(Imgui.IsVisible("Loaded cached heatmap for " + craft, Window), "the tab says it loaded the cached sweep");

                ctx.Step("The map shown kept on reopening, the history in sync");
                Set(hm, "_viewIndex", 0);
                Set(hm, "_editorLoadTriedSig", null);
                object shown = Get(hm, "_results");
                yield return Wait.Until(() => Get<int>(hm, "_viewIndex") == 1, 10, "the history index to sync", shot: false);
                Assert.True(ReferenceEquals(shown, Get(hm, "_results")), "the craft's map already shown is kept");

                ctx.Step("Files the history cannot read");
                int parts = second.Results[0].PartCount;
                string garbage = Path.Combine(dir, $"{Sanitized(craft)}.{parts}.1.farheat");
                string oldFormat = Path.Combine(dir, $"{Sanitized(craft)}.{parts}.2.farheat");
                string missing = Path.Combine(dir, $"{Sanitized(craft)}.{parts}.3.farheat");
                made.Add(garbage);
                made.Add(oldFormat);
                File.WriteAllBytes(garbage, Encoding.ASCII.GetBytes("not a heatmap"));
                using (FileStream fs = File.Create(oldFormat))
                using (var gz = new System.IO.Compression.GZipStream(fs, System.IO.Compression.CompressionMode.Compress))
                using (var w = new BinaryWriter(gz))
                {
                    w.Write(0x46415248u);   // FAR's magic, "FARH"
                    w.Write(0);             // a format version FAR never wrote
                    w.Write(0);
                }

                List<HeatmapPersistence.CacheEntry> listed = HeatmapPersistence.ListForVessel(craft);
                ctx.Record("listed", string.Join(", ", listed.Select(e => Path.GetFileName(e.Path))));
                Assert.Equal(2, listed.Count, "the history lists the two sweeps, not the unreadable files");
                AssertNull(HeatmapPersistence.LoadPath(garbage), "a garbage file reads as nothing");
                AssertNull(HeatmapPersistence.LoadPath(oldFormat), "another format version reads as nothing");
                AssertNull(HeatmapPersistence.LoadPath(missing), "a missing file reads as nothing");
                List<HeatmapResult> byName = HeatmapPersistence.Load(craft, parts);
                Maps.AssertSameSweep(second.Results, byName, "the newest sweep for the craft and its part count");
                AssertNull(HeatmapPersistence.Load(craft, parts + 1), "no sweep for another part count");

                ctx.Step("A sweep's file deleted behind the tab's back, then browsed to");
                File.Delete(first.CachePath);
                yield return Imgui.Click(Back, Window, exact: true);
                var history = Get<List<HeatmapPersistence.CacheEntry>>(hm, "_cachedSweeps");
                Assert.Equal(1, history.Count, "the history resynced from disk: one sweep left");
                Assert.Equal(second.CachePath, history[0].Path, "the sweep left is the second");

                ctx.Step("Clear History deletes the unreadable files too");
                Set(hm, "_viewIndex", 0);
                yield return Imgui.Click("Clear History", Window, exact: true);
                Assert.False(File.Exists(second.CachePath), "Clear History deleted the sweep's file");
                Assert.False(File.Exists(garbage) || File.Exists(oldFormat), "Clear History deleted the files the history could not read");
                Assert.Equal("Cleared 3 cached sweeps.", Get<string>(hm, "_statusMessage"), "the status line");
                Assert.Equal(-1, Get<int>(hm, "_viewIndex"), "nothing in the history to show");

                ctx.Step("Removing the last sweep");
                third = new SweepOut();
                yield return Sweep(new[] { "0.4", "1.2", "0", "6", "3", "3" }, false, true, third);
                Assert.Equal(1, Get<List<HeatmapPersistence.CacheEntry>>(hm, "_cachedSweeps").Count, "one sweep in the history");
                yield return Imgui.Click("Remove", Window, exact: true);
                Assert.Equal("Removed sweep; none cached for this craft now.", Get<string>(hm, "_statusMessage"), "the status line");
                Assert.False(File.Exists(third.CachePath), "the removed sweep's file deleted");
                Assert.Equal(-1, Get<int>(hm, "_viewIndex"), "nothing in the history to show");

                ctx.Step("The craft renamed to one without sweeps: the maps cleared");
                Assert.NotNull(Get(hm, "_results"), "a map still shown");
                string name = ship.shipName;
                try
                {
                    ship.shipName = "FAR test renamed " + Guid.NewGuid().ToString("N").Substring(0, 6);
                    yield return Wait.Until(() => Get<List<HeatmapResult>>(hm, "_results") == null || Get<List<HeatmapResult>>(hm, "_results").Count == 0,
                                            10, "the maps to clear", shot: false);
                }
                finally
                {
                    ship.shipName = name;
                }

                ctx.Step("21 sweeps of a craft: the oldest pruned");
                var results = second.Results.Select(r => r).ToList();
                string savedName = results[0].CraftName;
                var paths = new List<string>();
                try
                {
                    results[0].CraftName = pruneCraft;
                    for (int n = 0; n < 21; n++)
                    {
                        string p = HeatmapPersistence.Save(results);
                        Assert.NotNull(p, $"sweep {n} saved");
                        paths.Add(p);
                        made.Add(p);
                        // a later write time for each (the prune sorts by it)
                        File.SetLastWriteTime(p, DateTime.Now.AddMinutes(n - 30));
                    }
                }
                finally
                {
                    results[0].CraftName = savedName;
                }

                Assert.Equal(20, HeatmapPersistence.ListForVessel(pruneCraft).Count, "20 sweeps of the craft kept");
                Assert.True(paths.Skip(1).All(File.Exists), "the 20 newest kept");
            }
            finally
            {
                DeleteCache(first?.CachePath);
                DeleteCache(second?.CachePath);
                DeleteCache(third?.CachePath);
                foreach (string p in made)
                    DeleteCache(p);
                HeatmapPersistence.DeleteForVessel(pruneCraft);
                CloseWindow();
            }
        }

        // HeatmapPersistence.Sanitize: letters, digits, '-' and '_' kept, the rest '_'
        private static string Sanitized(string craft) => new string(craft.Select(ch => char.IsLetterOrDigit(ch) || ch == '-' || ch == '_' ? ch : '_').ToArray());
    }

    /// <summary>Envelope maps compared, described and recorded (as EnvelopeTests does, for the tests outside it).</summary>
    internal static class Maps
    {
        public sealed class Compared
        {
            public int Cells, Extrapolated, TailSolved, NearStall;
            public double DAlpha, DDrag, DThrust;

            public override string ToString() =>
                $"{Cells} cells compared, {NearStall} near the stall, {Extrapolated} filled in by both, {TailSolved} filled in by one and solved by the other; " +
                $"largest differences AoA {DAlpha:G3} deg, drag {DDrag:G3}, thrust {DThrust:G3} (relative)";
        }

        /// <summary>
        /// Two sweeps of one grid the same map: every cell trims in both or neither, is filled in (past the heat
        /// wall) in both or neither, and below the stall flies, overheats, and has the same angle of attack, drag and
        /// thrust in both. With fallbackSolvesTail, the second sweep solves the cells the first filled in: trimmed,
        /// not flyable.
        /// </summary>
        public static Compared Compare(HeatmapResult a, HeatmapResult b, string what, bool fallbackSolvesTail)
        {
            var o = new Compared();
            int nM = a.MachAxis.Length, nA = a.AltAxis.Length;
            Assert.Equal(nM, b.MachAxis.Length, $"{what}: Mach columns");
            Assert.Equal(nA, b.AltAxis.Length, $"{what}: altitude rows");
            Assert.Equal(a.EnvelopeSamples.Length, b.EnvelopeSamples.Length, $"{what}: mass samples");
            for (int k = 0; k < a.EnvelopeSamples.Length; k++)
                for (int i = 0; i < nM; i++)
                    for (int j = 0; j < nA; j++)
                    {
                        Cell x = a.EnvelopeSamples[k][i, j], y = b.EnvelopeSamples[k][i, j];
                        string at = $"{what} at Mach {a.MachAxis[i]:F2}, {a.AltAxis[j] / 1000:F0} km, mass sample {k} ({Describe(x)} against {Describe(y)})";
                        if (x.Extrapolated && fallbackSolvesTail)
                        {
                            o.TailSolved++;
                            Assert.True(y.Trimmed && !y.Feasible, $"a heat-wall cell, solved, trims and does not fly: {at}");
                            continue;
                        }

                        Assert.Equal(x.Extrapolated, y.Extrapolated, $"filled in by both or neither: {at}");
                        if (x.Extrapolated)
                        {
                            o.Extrapolated++;
                            Assert.True(x.Overheats && y.Overheats && !x.Feasible && !y.Feasible, $"a filled-in cell is heat wall: {at}");
                            continue;
                        }

                        Assert.Equal(x.Trimmed, y.Trimmed, $"trims in both or neither: {at}");
                        if (!x.Trimmed)
                            continue;
                        if (x.Alpha > 10 || y.Alpha > 10)
                        {
                            o.NearStall++;
                            continue;
                        }

                        o.Cells++;
                        o.DAlpha = Math.Max(o.DAlpha, Math.Abs(x.Alpha - y.Alpha));
                        o.DDrag = Math.Max(o.DDrag, Math.Abs(x.DragN - y.DragN) / Math.Max(1, Math.Abs(x.DragN)));
                        o.DThrust = Math.Max(o.DThrust, Math.Abs(x.ThrustAvailableN - y.ThrustAvailableN) / Math.Max(1, Math.Abs(x.ThrustAvailableN)));
                        // the limits of Envelope_MultithreadedSweepIsTheSerialOne: the paths differ in the last bits
                        // and the trim stops anywhere in its tolerance band
                        Assert.Equal(x.Feasible, y.Feasible, $"flies in both or neither: {at}");
                        Assert.Equal(x.Overheats, y.Overheats, $"overheats in both or neither: {at}");
                        Assert.Near(x.Alpha, y.Alpha, 0.005, $"the trim angle of attack: {at} (deg)");
                        Assert.Near(x.DragN, y.DragN, 0.001 * Math.Abs(x.DragN) + 1, $"the cruise drag: {at} (N)");
                        Assert.Near(x.ThrustAvailableN, y.ThrustAvailableN, 1e-4 * Math.Abs(x.ThrustAvailableN) + 1, $"the full-throttle thrust: {at} (N)");
                    }

            return o;
        }

        /// <summary>The names of a stability cell's derivatives out of range, sorted ("-" no cell).</summary>
        public static string Flags(HeatmapResult.StabilityCell s) =>
            s == null ? "-" : string.Join(" ", s.OutOfRange.Select(o => o.Item1).OrderBy(n => n));

        /// <summary>Two sweeps' maps the same: axes, ceiling, every mass sample's cells, and the stability flags.</summary>
        public static void AssertSameSweep(List<HeatmapResult> expected, List<HeatmapResult> actual, string what)
        {
            Assert.True(actual != null && actual.Count == expected.Count, $"{what}: {expected.Count} map(s)");
            for (int m = 0; m < expected.Count; m++)
            {
                HeatmapResult e = expected[m], a = actual[m];
                Assert.Equal(e.MachAxis.Length, a.MachAxis.Length, $"{what}: Mach columns");
                Assert.Equal(e.AltAxis.Length, a.AltAxis.Length, $"{what}: altitude rows");
                for (int i = 0; i < e.MachAxis.Length; i++)
                    Assert.Near(e.MachAxis[i], a.MachAxis[i], 1e-9, $"{what}: Mach column {i}");
                for (int j = 0; j < e.AltAxis.Length; j++)
                    Assert.Near(e.AltAxis[j], a.AltAxis[j], 1e-6, $"{what}: altitude row {j}");
                Assert.True(e.CeilingM.Equals(a.CeilingM), $"{what}: the ceiling ({e.CeilingM} against {a.CeilingM})");
                Assert.Equal(e.EnvelopeSamples.Length, a.EnvelopeSamples.Length, $"{what}: mass samples");
                for (int k = 0; k < e.EnvelopeSamples.Length; k++)
                    AssertSameMap(e.EnvelopeSamples[k], a.EnvelopeSamples[k], e.MachAxis.Length, e.AltAxis.Length, $"{what}, mass sample {k}");
                Assert.Equal(e.HasStability, a.HasStability, $"{what}: a stability map or none");
                if (e.HasStability)
                    for (int i = 0; i < e.MachAxis.Length; i++)
                        for (int j = 0; j < e.AltAxis.Length; j++)
                            Assert.Equal(Flags(e.Stability[i, j]), Flags(a.Stability[i, j]), $"{what}: the stability flags at [{i},{j}]");
            }
        }

        /// <summary>Two displayed maps the same cell for cell: what trims, flies, is hidden and cruises, at the same drag.</summary>
        public static void AssertSameMap(Cell[,] expected, Cell[,] actual, int nM, int nA, string what)
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

        public static string Describe(Cell c)
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

        /// <summary>The grid, top row first: '.' no trim, 'x' hidden, 'F' cruise, 'i' flyable without a cruise, 'H' heat wall ('E' filled in), 'g' thrust-short.</summary>
        public static void RecordGrid(TestContext ctx, string name, Cell[,] grid, int nM, int nA)
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
                              c.Overheats ? (c.Extrapolated ? 'E' : 'H') : 'g');
                }

                if (j > 0)
                    sb.Append('/');
            }

            ctx.Record("grid." + name, sb.ToString());
        }
    }
}
