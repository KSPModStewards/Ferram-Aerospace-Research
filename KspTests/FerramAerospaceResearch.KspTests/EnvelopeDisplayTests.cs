using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using KSP.Localization;
using KspTest;
using UnityEngine;
using static FerramAerospaceResearch.KspTests.Refl;
using static FerramAerospaceResearch.KspTests.EditorWindow;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The Heatmaps tab's display controls after a sweep, and its auto-recalc, on the stock Aeris 4A with AJE.
    /// </summary>
    public static class EnvelopeDisplayTests
    {
        private const string AjeMods = "AdvancedJetEngine";
        private const string AjeRequires = "AJE";

        /// <summary>
        /// A 4 x 4 sweep with stability (Mach 0.3-1.2 x 0-9 km), then its display controls. Cleanup off shows the
        /// raw full-tank sample, on the cleaned one again, with no new sweep. Each Color-by option recolors the
        /// envelope map (its texture changes, the sweep does not). "Recolor to view" switches off and on. A box
        /// dragged across the stability map (under a synthetic mouse) draws its rubber band while dragging and zooms
        /// every map to the cells it spans.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_MapDisplayControls(TestContext ctx)
        {
            string cache = null;
            HeatmapRenderer.Metric metric = HeatmapRenderer.ColorMetric;
            bool rescale = HeatmapRenderer.RescaleColorsToView;
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                var sw = new SweepOut();
                yield return Sweep(new[] { "0.3", "1.2", "0", "9", "4", "4" }, true, true, sw);
                cache = sw.CachePath;
                HeatmapsGUI hm = Heatmaps;
                HeatmapResult r = sw.Results[0];
                int nM = r.MachAxis.Length, nA = r.AltAxis.Length;
                HeatmapRenderer env = Get<List<HeatmapRenderer>>(hm, "_envRenderers")[0];
                HeatmapRenderer stab = Get<HeatmapRenderer>(hm, "_stabRenderer");

                ctx.Step("Cleanup off: the raw sample; on: the cleaned one");
                var cleaned = (PerformanceEnvelopeCalculator.CellResult[,])r.Envelope.Clone();
                yield return Imgui.Toggle("Cleanup", false, Window);
                Assert.False(HeatmapResult.CleanupDisplay, "Cleanup off");
                Assert.True(ReferenceEquals(sw.Results, Get(hm, "_results")), "no new sweep");
                Maps.AssertSameMap(r.EnvelopeSamples[2], r.Envelope, nM, nA, "the raw full-tank sample shown");
                yield return Imgui.Toggle("Cleanup", true, Window);
                Maps.AssertSameMap(cleaned, r.Envelope, nM, nA, "the cleaned map shown again");

                ctx.Step("Color by each option");
                var envTex = Get<Texture2D>(env, "_envTex");
                yield return Imgui.Toggle(L("FARHeatmapsEndurance"), true, Window, exact: true);
                yield return Wait.Frames(3);
                Color32[] before = envTex.GetPixels32();
                foreach ((string label, HeatmapRenderer.Metric m) in new[] { (L("FARHeatmapsRange"), HeatmapRenderer.Metric.Range), ("AoA", HeatmapRenderer.Metric.Aoa),
                                                                             ("Throttle", HeatmapRenderer.Metric.Throttle), ("Climb", HeatmapRenderer.Metric.Climb) })
                {
                    yield return Imgui.Toggle(label, true, Window, exact: true);
                    yield return Wait.Frames(3);
                    Assert.Equal(m, HeatmapRenderer.ColorMetric, $"the maps colored by {label}");
                    Color32[] now = envTex.GetPixels32();
                    Assert.True(now.Where((c, k) => !c.Equals(before[k])).Any(), $"the envelope map recolored by {label}");
                    before = now;
                }

                Assert.True(ReferenceEquals(sw.Results, Get(hm, "_results")), "recoloring sweeps nothing");

                ctx.Step("Recolor to view");
                yield return Imgui.Toggle("Recolor to view", !rescale, Window);
                Assert.Equal(!rescale, HeatmapRenderer.RescaleColorsToView, "the toggle switched");
                yield return Imgui.Toggle("Recolor to view", rescale, Window);
                Assert.Equal(rescale, HeatmapRenderer.RescaleColorsToView, "and back");

                ctx.Step("A box dragged on the stability map");
                var stabTex = Get<Texture2D>(stab, "_stabTex");
                yield return Wait.Until(() => Imgui.ImageRect(stabTex, Window) != null, 15, "the stability map drawn", shot: false, detail: Imgui.Dump);
                Rect map = Imgui.ImageRect(stabTex, Window).Value;
                Vector2 Centre(int i, int j) => new Vector2(map.x + (i + 0.5f) / nM * map.width, map.y + (nA - 1 - j + 0.5f) / nA * map.height);
                yield return Imgui.MouseDown(Centre(1, 1));
                yield return Imgui.MouseDrag(Centre(2, 2));
                yield return Imgui.MouseDrag(Centre(3, 3));
                Assert.True(ReferenceEquals(stab, GetStatic(typeof(HeatmapRenderer), "_dragOwner")), "the stability map owns the drag (its rubber band drawn)");
                yield return Imgui.MouseUp(Centre(3, 3));
                yield return Wait.Frames(3);
                object zoom = GetStatic(typeof(HeatmapRenderer), "_zoom");
                string window = $"Mach cells {Get(zoom, "IMin")}-{Get(zoom, "IMax")}, altitude cells {Get(zoom, "JMin")}-{Get(zoom, "JMax")}";
                ctx.Record("zoom", window);
                Assert.True((bool)GetStatic(typeof(HeatmapRenderer), "_zoomActive"), "the drag zoomed in");
                Assert.Equal("Mach cells 1-3, altitude cells 1-3", window, "the zoom is the cells the box spans");
            }
            finally
            {
                Imgui.ReleasePointer();
                HeatmapRenderer.ResetZoom();
                HeatmapRenderer.ColorMetric = metric;
                HeatmapRenderer.RescaleColorsToView = rescale;
                HeatmapRenderer.InvalidateColors();
                DeleteCache(cache);
                CloseWindow();
            }
        }

        /// <summary>
        /// The craft with its liquid fuel drained (the oxidizer left: nothing the jets burn): a sweep still finds where
        /// it can fly level, the jets making thrust, but every flyable cell says there is nothing to cruise on (no
        /// endurance or range), and no fuel to sample: every mass sample is the dry mass.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_EmptyTanksFlyWithoutEndurance(TestContext ctx)
        {
            var amounts = new List<KeyValuePair<PartResource, double>>();
            string cache = null;
            try
            {
                foreach (Part p in EditorLogic.fetch.ship.parts)
                    foreach (PartResource res in p.Resources)
                        if (res.resourceName == "LiquidFuel")
                        {
                            amounts.Add(new KeyValuePair<PartResource, double>(res, res.amount));
                            res.amount = 0;
                        }

                Assert.True(amounts.Count > 0 && amounts.Any(a => a.Value > 0), "the craft had liquid fuel to drain");
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                var sw = new SweepOut();
                yield return Sweep(new[] { "0.4", "1.2", "0", "6", "3", "3" }, false, true, sw);
                cache = sw.CachePath;
                HeatmapResult r = sw.Results[0];
                Maps.RecordGrid(ctx, "empty", r.Envelope, 3, 3);
                ctx.Record("masses", $"{string.Join(", ", r.SampleMassesKg.Select(m => m.ToString("F0")))} kg, dry {r.DryMassKg:F0} kg");
                foreach (double m in r.SampleMassesKg)
                    Assert.Near(r.DryMassKg, m, 1, "every mass sample is the dry mass (kg)");
                int feasible = 0;
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                    {
                        PerformanceEnvelopeCalculator.CellResult c = r.Envelope[i, j];
                        if (!c.Feasible)
                            continue;
                        feasible++;
                        Assert.Equal(PerformanceEnvelopeCalculator.CruiseState.NoBurnableFuel, c.Cruise, $"nothing to cruise on at [{i},{j}]");
                        Assert.True(c.ThrustAvailableN > c.DragN, $"the jets out-thrust the drag at [{i},{j}]");
                    }

                Assert.True(feasible > 0, "the drained craft can still fly level somewhere");
            }
            finally
            {
                foreach (KeyValuePair<PartResource, double> a in amounts)
                    a.Key.amount = a.Value;
                DeleteCache(cache);
                CloseWindow();
            }
        }

        /// <summary>
        /// A part given an aero force modifier (FARAPI.SetPartAeroForceModifier, here one that leaves the force as it
        /// is) is called with the live part, so the sweep keeps off the worker threads: the editor's sim says it
        /// cannot sweep in parallel and refuses to start a parallel sweep, a sweep with Multithread on calls the
        /// modifier on the main thread only, and its map is the serial sweep's.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_ForceModifierKeepsTheSweepSerial(TestContext ctx)
        {
            var caches = new List<string>();
            Part part = null;
            int calls = 0, offMain = 0;
            int mainThread = System.Threading.Thread.CurrentThread.ManagedThreadId;
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                string[] fields = { "0.4", "1.6", "0", "9", "3", "3" };
                var serial = new SweepOut();
                yield return Sweep(fields, false, false, serial);
                caches.Add(serial.CachePath);
                Assert.True(Far.EditorSim.CanSweepInParallel, "without a modifier the sim can sweep in parallel");

                ctx.Step("A force modifier on the fuselage");
                part = EditorLogic.fetch.ship.parts.First(p => p.Modules.Contains("FARAeroPartModule") && !p.Modules.Contains("FARWingAerodynamicModel"));
                FARAPI.SetPartAeroForceModifier(part, (p, force) =>
                {
                    System.Threading.Interlocked.Increment(ref calls);
                    if (System.Threading.Thread.CurrentThread.ManagedThreadId != mainThread)
                        System.Threading.Interlocked.Increment(ref offMain);
                    return force;
                });
                Assert.False(Far.EditorSim.CanSweepInParallel, "with a modifier the sim cannot sweep in parallel");
                Assert.False(Far.EditorSim.BeginParallelSweep(), "and refuses to start a parallel sweep");

                ctx.Step("A sweep with Multithread on");
                var mt = new SweepOut();
                yield return Sweep(fields, false, true, mt);
                caches.Add(mt.CachePath);
                ctx.Record("modifier", $"{calls} calls, {offMain} off the main thread");
                Assert.True(calls > 0, "the sweep called the modifier");
                Assert.Equal(0, offMain, "only on the main thread");
                Maps.Compared same = Maps.Compare(serial.Results[0], mt.Results[0], "with the modifier against serial", fallbackSolvesTail: false);
                ctx.Record("compared", same.ToString());
                Assert.True(same.Cells > 0, "cells to compare");
            }
            finally
            {
                if (part != null)
                    FARAPI.SetPartAeroForceModifier(part, null);
                foreach (string c in caches)
                    DeleteCache(c);
                CloseWindow();
            }
        }

        /// <summary>
        /// Auto-recalc on: when the craft is voxelized again (as after an edit) the tab sweeps again on its own, once
        /// the voxelization is done and a short pause has passed, with the same range: a new map, the history one
        /// longer. Off, a voxelization sweeps nothing.
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_AutoRecalcResweepsAfterAnEdit(TestContext ctx)
        {
            var caches = new List<string>();
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(2);
                string craft = EditorLogic.fetch.ship.shipName;
                var sw = new SweepOut();
                yield return Sweep(new[] { "0.4", "1.2", "0", "6", "3", "3" }, false, true, sw);
                caches.Add(sw.CachePath);
                HeatmapsGUI hm = Heatmaps;
                int history = HeatmapPersistence.ListForVessel(craft).Count;

                ctx.Step("Auto-recalc off: a voxelization sweeps nothing");
                yield return Far.Revoxelize("auto-recalc off");
                yield return Wait.Seconds(2);
                Assert.True(ReferenceEquals(sw.Results, Get(hm, "_results")) && !hm.IsSweepRunning, "no sweep with auto-recalc off");

                ctx.Step("Auto-recalc on: a voxelization sweeps again");
                yield return Imgui.Toggle(L("FARHeatmapsAutoRecalc"), true, Window);
                Assert.True(Get<bool>(hm, "_autoRecalc"), "auto-recalc on");
                yield return Far.Revoxelize("auto-recalc on");
                yield return Wait.Until(() => !ReferenceEquals(sw.Results, Get(hm, "_results")) && !hm.IsSweepRunning, 300, "the sweep auto-recalc starts",
                                        shot: false);
                var again = Get<List<HeatmapResult>>(hm, "_results");
                List<HeatmapPersistence.CacheEntry> list = HeatmapPersistence.ListForVessel(craft);
                caches.AddRange(list.Select(e => e.Path));
                Assert.Equal(history + 1, list.Count, "the new sweep filed in the history");
                Assert.Equal(3, again[0].MachAxis.Length, "the same Mach cells");
                Assert.Near(1.2, again[0].MachAxis[2], 1e-9, "the same Mach range");
                Assert.Near(6000, again[0].AltAxis[2], 1e-6, "the same altitude range");
                Assert.Equal(Localizer.Format("FAREnvelopeDone", Get<int>(hm, "_cellsDone").ToString(System.Globalization.CultureInfo.InvariantCulture)),
                             Get<string>(hm, "_statusMessage"), "the status line");
                yield return Imgui.Toggle(L("FARHeatmapsAutoRecalc"), false, Window);
            }
            finally
            {
                foreach (string c in caches)
                    DeleteCache(c);
                CloseWindow();
            }
        }
    }
}
