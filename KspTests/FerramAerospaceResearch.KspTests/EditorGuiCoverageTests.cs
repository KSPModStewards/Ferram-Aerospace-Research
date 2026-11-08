using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using ferram4;
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
    /// More of FAR's editor window on the stock Aeris 4A, driven through its own controls: the Components tab's
    /// coloring by each quantity and its buttons, and the shared "More accurate aerodynamics" option on the static and
    /// stability tabs.
    /// </summary>
    public static class EditorGuiCoverageTests
    {
        private static readonly int ColorId = Shader.PropertyToID("_Color");

        // ------------------------------------------------------------------------------------------ Components

        /// <summary>
        /// The Components tab colors the craft by what is chosen, each part's color moved from its own toward the
        /// quantity's by the opacity. Parts: every part a color, rows and the rest (engines, structure) alike, no two
        /// rows the same, "One color per part". Aspect ratio: only the wing panels, the highest aspect ratio red and
        /// lower ones toward blue, the legend's scale up to the largest. Wings: one color per wing (panels of a wing
        /// alike), the legend listing each wing's aspect ratio. Shielded: exposed panels green, shielded red, retinted at
        /// the opacity its slider is moved to. Cd and Cm:
        /// every row tinted white to red (and blue where a part takes from the craft's), the legend a scale of the part's
        /// share. A part's row clicked lights it white, clicked again lets it go. Spoilers extended on a surface set as a
        /// spoiler add drag. Show Voxels shows the voxel model and hides it again. Leaving the tab takes every color
        /// and the light off.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator EditorGui_ComponentsColorByEachOption(TestContext ctx)
        {
            FARControllableSurface spoiler = null;
            bool wasSpoiler = false, wasFlap = false;
            try
            {
                yield return OpenWindow("FAREditorModeComponents");
                ResetConditions();
                object comp = Get(Far.Editor, "_components");
                object tintDropdown = Get(comp, "tintDropdown");
                Remember(comp, "speed", "altitude", "aoa", "spoilers", "showTable", "opacity");
                Remember(tintDropdown, "selectedOption");
                SelectOption(tintDropdown, "None");
                if (Get<bool>(comp, "spoilers"))
                    yield return Imgui.Click("Spoilers extended", Window);
                yield return Imgui.SetTextAfterLabel("Speed (m/s)", "100", Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStabDerivAlt"), "0", Window);
                yield return Imgui.SetTextAfterLabel("AoA (deg)", "2", Window);
                if (!Get<bool>(comp, "showTable"))
                    yield return Imgui.Click("Show Table", Window);
                yield return Far.WaitForEditorVoxels();
                yield return Recomputed(comp);

                var rows = ((IList)Get(comp, "rows")).Cast<object>().ToList();
                var colored = Get<HashSet<Part>>(comp, "colored");
                float opacity = Get<float>(comp, "opacity");
                List<Part> shipParts = EditorLogic.SortedShipList;
                List<object> wingRows = rows.Where(r => Get(r, "Wing") != null && Get(r, "Part") != null).ToList();
                List<object> bodyRows = rows.Where(r => Get(r, "Wing") == null && Get(r, "Part") != null).ToList();
                ctx.Record("rows", $"{rows.Count} rows ({wingRows.Count} wing panels, {bodyRows.Count} bodies), {shipParts.Count} parts, opacity {opacity:F2}");
                Assert.True(wingRows.Count >= 4 && bodyRows.Count >= 1, "the table has wing panels and bodies");
                Assert.Equal(0, colored.Count, "coloring by None leaves the parts their own colors");

                ctx.Step("Parts");
                yield return Tint(tintDropdown, "Parts");
                Assert.Equal("One color per part: ", Get<string>(comp, "legend"), "the legend");
                Assert.True(Imgui.IsVisible("One color per part", Window), "the tab shows the legend");
                foreach (Part p in shipParts)
                    Assert.True(colored.Contains(p), $"{p.partInfo.title} colored (rows and the rest alike)");
                var rowColors = rows.Where(r => Get(r, "Part") != null).Select(r => Key(TintOf(Get<Part>(r, "Part")))).ToList();
                Assert.Equal(rowColors.Count, rowColors.Distinct().Count(), "no two rows the same color");

                ctx.Step("Aspect Ratio");
                yield return Tint(tintDropdown, "Aspect Ratio");
                double maxAr = wingRows.Max(r => Get<double>(r, "EffectiveAR"));
                string legend = Get<string>(comp, "legend");
                ctx.Record("legend.ar", legend);
                Assert.True(legend.StartsWith("Aspect ratio: 0.0, ") && legend.EndsWith(", " + maxAr.ToString("F1", CultureInfo.InvariantCulture)),
                            $"the legend's scale runs from 0 to the largest aspect ratio, {maxAr:F2}: {legend}");
                foreach (object r in bodyRows)
                    Assert.False(colored.Contains(Get<Part>(r, "Part")), $"{Get<string>(r, "Name")}, a body, left its own colors");
                foreach (Part p in shipParts.Where(p => rows.All(r => !ReferenceEquals(Get(r, "Part"), p))))
                    Assert.False(colored.Contains(p), $"{p.partInfo.title}, in no row, lost the Parts color");
                object most = wingRows.First(r => Get<double>(r, "EffectiveAR") == maxAr);
                AssertTint(Get<Part>(most, "Part"), Color.red, opacity, "the highest aspect ratio panel is red");
                foreach (object r in wingRows)
                {
                    Color shown = TintOf(Get<Part>(r, "Part")), own = OwnColor(Get<Part>(r, "Part"));
                    double ar = Get<double>(r, "EffectiveAR");
                    // the ramp: blue (0) to red (the largest), so the lower the aspect ratio the less red over blue
                    if (ar < 0.5 * maxAr)
                        Assert.True(shown.b - own.b * (1 - opacity) > shown.r - own.r * (1 - opacity) - 1e-3,
                                    $"{Get<string>(r, "Name")} (AR {ar:F2} of {maxAr:F2}) is toward blue ({shown})");
                }

                ctx.Step("Wings");
                yield return Tint(tintDropdown, "Wings");
                legend = Get<string>(comp, "legend");
                ctx.Record("legend.wings", legend);
                var groups = wingRows.Where(r => Get<int>(r, "Group") >= 0).GroupBy(r => Get<int>(r, "Group")).ToList();
                Assert.True(groups.Count >= 1, "the craft has a wing of several panels");
                Assert.True(legend.StartsWith("Wings: 1: AR "), "the legend lists the wings: " + legend);
                foreach (var g in groups)
                {
                    string entry = string.Format(CultureInfo.InvariantCulture, "{0}: AR {1:F1}", g.Key + 1, Get<double>(g.First(), "EffectiveAR"));
                    Assert.True(legend.Contains(entry), $"the legend has '{entry}': {legend}");
                    var colors = g.Select(r => Key(TintOf(Get<Part>(r, "Part")))).Distinct().ToList();
                    Assert.Equal(1, colors.Count, $"the panels of wing {g.Key + 1} one color");
                }

                if (groups.Count > 1)
                    Assert.Equal(groups.Count, groups.Select(g => Key(TintOf(Get<Part>(g.First(), "Part")))).Distinct().Count(), "each wing its own color");
                foreach (object r in wingRows.Where(r => Get<int>(r, "Group") < 0))
                    AssertTint(Get<Part>(r, "Part"), new Color(0.4f, 0.4f, 0.4f), opacity, $"{Get<string>(r, "Name")}, keeping its own aspect ratio, grey");
                if (wingRows.Any(r => Get<int>(r, "Group") < 0))
                    Assert.True(legend.EndsWith("own AR"), "the legend has the own-AR grey: " + legend);

                ctx.Step("Shielded");
                yield return Tint(tintDropdown, "Shielded");
                Assert.Equal("Shielding: shielded (no lift), exposed", Get<string>(comp, "legend"), "the legend");
                foreach (object r in wingRows)
                {
                    bool shielded = Get<bool>(r, "Shielded");
                    AssertTint(Get<Part>(r, "Part"), shielded ? new Color(1, 0.1f, 0.1f) : new Color(0.1f, 0.8f, 0.1f), opacity,
                               $"{Get<string>(r, "Name")}, {(shielded ? "shielded, red" : "exposed, green")}");
                }

                ctx.Step("The opacity slider");
                float slid = opacity < 0.5f ? 0.8f : 0.3f;
                yield return Imgui.SetSliderAfterLabel("Opacity", slid, Window);
                yield return Wait.Frames(2);
                Assert.Near(slid, Get<float>(comp, "opacity"), 1e-6, "the tab's opacity after the slider");
                foreach (object r in wingRows)
                {
                    bool shielded = Get<bool>(r, "Shielded");
                    AssertTint(Get<Part>(r, "Part"), shielded ? new Color(1, 0.1f, 0.1f) : new Color(0.1f, 0.8f, 0.1f), slid,
                               $"{Get<string>(r, "Name")} retinted at the new opacity");
                }

                yield return Imgui.SetSliderAfterLabel("Opacity", opacity, Window);
                yield return Wait.Frames(2);
                Assert.Near(opacity, Get<float>(comp, "opacity"), 1e-6, "and back");

                foreach (string q in new[] { "Cd", "Cm" })
                {
                    ctx.Step(q);
                    yield return Tint(tintDropdown, q);
                    legend = Get<string>(comp, "legend");
                    ctx.Record("legend." + q, legend);
                    bool negative = rows.Any(r => Get<double>(r, q) < 0);
                    Assert.True(legend.StartsWith(q + ", each part's share of the craft's: "), "the legend: " + legend);
                    Assert.Equal(negative ? 5 : 3, ((IList)Get(comp, "legendTicks")).Count,
                                 $"the scale's ticks ({(negative ? "a part takes from the craft's" : "every part adds")})");
                    foreach (object r in rows.Where(r => Get(r, "Part") != null))
                        Assert.True(colored.Contains(Get<Part>(r, "Part")), $"{Get<string>(r, "Name")} tinted by its {q}");
                    object top = rows.Where(r => Get(r, "Part") != null).OrderByDescending(r => Math.Abs(Get<double>(r, q))).First();
                    AssertTint(Get<Part>(top, "Part"), Get<double>(top, q) > 0 ? Color.red : Color.blue, opacity,
                               $"{Get<string>(top, "Name")}, the largest share of {q}, full {(Get<double>(top, q) > 0 ? "red" : "blue")}");
                }

                ctx.Step("A part picked in the table");
                object pickRow = wingRows[0];
                Part picked = Get<Part>(pickRow, "Part");
                yield return Imgui.Click(Get<string>(pickRow, "Name"), Window, exact: true);
                yield return Wait.Frames(2);
                Assert.True(ReferenceEquals(picked, Get(comp, "selected")), "the row's part picked");
                Assert.Equal(Part.HighlightType.AlwaysOn, picked.highlightType, "the picked part lit");
                Assert.True(Imgui.IsVisible("> " + Get<string>(pickRow, "Name"), Window), "the row marked");
                yield return Imgui.Click("> " + Get<string>(pickRow, "Name"), Window, exact: true);
                yield return Wait.Frames(2);
                Assert.True(Get(comp, "selected") == null, "clicked again, nothing picked");
                Assert.Equal(Part.HighlightType.OnMouseOver, picked.highlightType, "the part's light back to stock's");

                ctx.Step("Spoilers extended on a surface set as a spoiler");
                spoiler = EditorLogic.SortedShipList.Select(p => p.Modules.GetModule<FARControllableSurface>()).FirstOrDefault(s => s != null);
                Assert.NotNull(spoiler, "the craft has a control surface");
                wasSpoiler = spoiler.isSpoiler;
                wasFlap = spoiler.isFlap;
                spoiler.isFlap = false;
                spoiler.isSpoiler = true;
                SelectOption(tintDropdown, "None");
                yield return Recomputed(comp);
                double cdIn = SummaryCd(comp);
                yield return Imgui.Click("Spoilers retracted", Window);
                Assert.True(Get<bool>(comp, "spoilers"), "the spoilers extended");
                yield return Recomputed(comp);
                double cdOut = SummaryCd(comp);
                ctx.Record("spoilers", $"Cd {cdIn:F5} in, {cdOut:F5} out");
                Assert.True(cdOut > cdIn, $"the spoiler out adds drag (Cd {cdOut:F5} against {cdIn:F5})");
                yield return Imgui.Click("Spoilers extended", Window);
                Assert.False(Get<bool>(comp, "spoilers"), "the spoilers in again");

                ctx.Step("Show Voxels");
                var aero = Far.EditorVehicle;
                Assert.False(Get<bool>(aero, "visualizing"), "no voxels shown to start with");
                yield return Imgui.Click("Show Voxels", Window, exact: true);
                yield return Wait.Frames(2);
                Assert.True(Get<bool>(aero, "visualizing"), "Show Voxels shows the voxel model");
                yield return Imgui.Click("Show Voxels", Window, exact: true);
                yield return Wait.Frames(2);
                Assert.False(Get<bool>(aero, "visualizing"), "pressed again, the voxels hidden");

                ctx.Step("Leaving the tab");
                yield return Tint(tintDropdown, "Parts");
                yield return Imgui.Click(Get<string>(pickRow, "Name"), Window, exact: true);
                yield return Wait.Frames(2);
                Assert.True(colored.Count > 0 && Get(comp, "selected") != null, "parts colored and one picked");
                SelectTab("FAREditorModeStatic");
                yield return Wait.Frames(3);
                Assert.Equal(0, colored.Count, "leaving the tab takes the colors off");
                Assert.True(Get(comp, "selected") == null && Get(comp, "lastPicked") == null, "and lets the picked part go");
                Assert.Equal(Part.HighlightType.OnMouseOver, picked.highlightType, "the picked part's light back to stock's");
            }
            finally
            {
                if (spoiler != null)
                {
                    spoiler.isSpoiler = wasSpoiler;
                    spoiler.isFlap = wasFlap;
                }

                if (EditorGUI.Instance != null && Refl.TryGet<object>(Far.EditorVehicle, "visualizing") is bool v && v)
                    Far.EditorVehicle.DebugVisualizeVoxels(EditorLogic.RootPart.partTransform.localToWorldMatrix);
                CloseWindow();
            }
        }

        /// <summary>The tab's coloring chosen and applied (on its next layout pass).</summary>
        private static IEnumerator Tint(object dropdown, string option)
        {
            SelectOption(dropdown, option);
            yield return Wait.Frames(3);
        }

        private static IEnumerator Recomputed(object comp)
        {
            yield return Wait.Frames(2);
            yield return Wait.Until(() => !Get<bool>(comp, "dirty"), 15, "the Components tab to recompute", shot: false);
            yield return Wait.Frames(2);
        }

        private static double SummaryCd(object comp)
        {
            string s = Get<string>(comp, "summary");
            System.Text.RegularExpressions.Match m = System.Text.RegularExpressions.Regex.Match(s, @"Cd (?<cd>[-\d.]+)");
            Assert.True(m.Success, "the summary has the craft's Cd: " + s);
            return double.Parse(m.Groups["cd"].Value, CultureInfo.InvariantCulture);
        }

        /// <summary>The color the tab put on a part (its property block's _Color).</summary>
        private static Color TintOf(Part p) => p.mpb.GetColor(ColorId);

        /// <summary>The part's own _Color: its first material that has one.</summary>
        private static Color OwnColor(Part p)
        {
            IEnumerable<Renderer> rs = p.HighlightRenderer != null && p.HighlightRenderer.Count > 0 ? p.HighlightRenderer : p.FindModelComponents<Renderer>();
            Material m = rs.Where(r => r != null && (r is MeshRenderer || r is SkinnedMeshRenderer)).Select(r => r.sharedMaterial)
                           .FirstOrDefault(x => x != null && x.HasProperty(ColorId));
            return m != null ? m.GetColor(ColorId) : Color.white;
        }

        /// <summary>The part's color is its own moved toward c by the opacity (its alpha kept).</summary>
        private static void AssertTint(Part p, Color c, float opacity, string what)
        {
            Color own = OwnColor(p), want = Color.Lerp(own, c, opacity), shown = TintOf(p);
            Assert.True(Math.Abs(want.r - shown.r) < 0.01f && Math.Abs(want.g - shown.g) < 0.01f && Math.Abs(want.b - shown.b) < 0.01f,
                        $"{what}: {shown} against {want} (its own {own})");
        }

        private static string Key(Color c) => $"{c.r:F3} {c.g:F3} {c.b:F3}";

        // ------------------------------------------------------------------------------------------ Heatmaps: body

        /// <summary>
        /// The Heatmaps tab frames its altitude range to the planet picked: for Duna, Eve and Laythe the range becomes
        /// 0 up to the kilometre above where the air has thinned to 2% of its sea-level pressure (inside the
        /// atmosphere). An airless body (the Mun) leaves the range as it was, and a range typed in stays while the
        /// planet does.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 600)]
        public static IEnumerator EditorGui_BodyChangeReframesTheAltitudes(TestContext ctx)
        {
            object bodies = null;
            try
            {
                yield return OpenWindow("FAREditorModeHeatmaps");
                ResetConditions();
                yield return Wait.Frames(3);
                HeatmapsGUI hm = Heatmaps;
                PrepareHeatmaps(hm);
                bodies = Get(Get(Far.Editor, "_editorGraph"), "bodySettingDropdown");
                Assert.True(ReferenceEquals(bodies, Get(hm, "_bodySettingDropdown")), "the tab's planet is the window's shared one");
                double ut = Planetarium.GetUniversalTime();

                foreach (string name in new[] { "Duna", "Eve", "Laythe" })
                {
                    ctx.Step(name);
                    CelestialBody body = FlightGlobals.Bodies.First(b => b.bodyName == name);
                    SelectOption(bodies, name);
                    yield return Wait.Frames(3);
                    string min = Get<string>(hm, "_altMinKm"), max = Get<string>(hm, "_altMaxKm");
                    ctx.Record(name, $"{min}-{max} km, atmosphere {body.atmosphereDepth / 1000:F0} km");
                    Assert.Equal("0", min, $"the range starts at {name}'s surface");
                    int km = int.Parse(max, CultureInfo.InvariantCulture);
                    double p0 = FARAtmosphere.GetPressure(body, new Vector3d(0, 0, 0), ut);
                    double pTop = FARAtmosphere.GetPressure(body, new Vector3d(0, 0, km * 1000.0), ut);
                    double pBelow = FARAtmosphere.GetPressure(body, new Vector3d(0, 0, (km - 1) * 1000.0), ut);
                    Assert.True(km * 1000.0 < body.atmosphereDepth, $"{name}'s range ends inside its atmosphere ({km} km)");
                    Assert.True(pTop <= 0.02 * p0 && pBelow > 0.02 * p0,
                                $"{name}'s range ends at the first kilometre past 2% of the sea-level pressure ({pBelow / p0:P2} at {km - 1} km, {pTop / p0:P2} at {km} km)");
                    Assert.True(Imgui.IsVisible(L("FARStabHeatmapAltRange"), Window), "the tab drawn");
                }

                ctx.Step("A range typed in stays while the planet does");
                yield return Imgui.SetTextNth(3, "17", Window);
                yield return Wait.Frames(5);
                Assert.Equal("17", Get<string>(hm, "_altMaxKm"), "the range typed in kept");

                ctx.Step("The Mun, without air");
                SelectOption(bodies, "Mun");
                yield return Wait.Frames(3);
                Assert.Equal("17", Get<string>(hm, "_altMaxKm"), "an airless body leaves the range alone");

                // the home planet again, seen by the tab, before its fields are put back
                SelectOption(bodies, FlightGlobals.GetHomeBody().bodyName);
                yield return Wait.Frames(3);
            }
            finally
            {
                CloseWindow();
            }
        }

        // ------------------------------------------------------------------------------------------ part edits

        /// <summary>
        /// The editor's part events reach FAR's model of the craft. A part picked up changes nothing (no voxelization
        /// queued). The part at the tail moved 1.5 m back and its move reported (a PartOffset event): a voxelization is
        /// queued, and once done the craft is 1.5 m longer. Moved back and reported as an unknown change (the part's
        /// geometry rebuilt), the craft is its own length again.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 600)]
        public static IEnumerator Editor_PartEditsRevoxelize(TestContext ctx)
        {
            Part nose = null;
            Vector3 at = Vector3.zero;
            try
            {
                yield return Far.WaitForEditorVoxels();
                yield return Far.Settled("the plane");
                double length = Far.EditorVehicle.Length;
                // the part reaching furthest back (the root, the cockpit, is the nose); its children move with it
                Part root = EditorLogic.RootPart;
                nose = EditorLogic.fetch.ship.parts
                                  .Where(p => p != root && p.physicalSignificance == Part.PhysicalSignificance.FULL &&
                                              p.GetComponent<FerramAerospaceResearch.FARPartGeometry.GeometryPartModule>() != null)
                                  .OrderBy(p => p.GetComponentsInChildren<Renderer>().Select(r => r.bounds.min.z).DefaultIfEmpty(float.MaxValue).Min())
                                  .First();
                at = nose.transform.position;
                ctx.Record("nose", $"{nose.partInfo.title} at {at} ({nose.children.Count} attached), root {root.partInfo.title}, craft {length:F3} m");

                ctx.Step("A part picked up");
                GameEvents.onEditorPartEvent.Fire(ConstructionEventType.PartPicked, nose);
                Assert.False(Far.Editor.VoxelizationUpdateQueued, "picking a part up queues no voxelization");

                ctx.Step("The tail part moved 1.5 m back");
                nose.transform.position = at - new Vector3(0, 0, 1.5f);
                GameEvents.onEditorPartEvent.Fire(ConstructionEventType.PartOffset, nose);
                Assert.True(Far.Editor.VoxelizationUpdateQueued, "the move queues a voxelization");
                Assert.True(Get<bool>(Far.Editor, "partMovement"), "and marks the wings' interactions for an update");
                yield return Wait.Until(() => Far.EditorReady && Math.Abs(Far.EditorVehicle.Length - length) > 0.5, 120, "the craft voxelized longer", shot: false);
                yield return Far.Settled("the plane, its nose out");
                double longer = Far.EditorVehicle.Length;
                Assert.Near(length + 1.5, longer, 0.3, "the craft 1.5 m longer (m)");
                Assert.Near(longer, Far.EditorSim._bodyLength, 1e-9, "the analysis takes the new length");

                ctx.Step("Moved back, reported as an unknown change");
                nose.transform.position = at;
                GameEvents.onEditorPartEvent.Fire(ConstructionEventType.Unknown, nose);
                Assert.True(Far.Editor.VoxelizationUpdateQueued, "an unknown change queues a voxelization");
                yield return Wait.Until(() => Far.EditorReady && Math.Abs(Far.EditorVehicle.Length - length) < 0.05, 120, "the craft voxelized its own length", shot: false);
                ctx.Record("lengths", $"{length:F3} m, {longer:F3} m moved, {Far.EditorVehicle.Length:F3} m back");
                // left unsettled, the next test's AoA sweep (under coverage) still had the tail out: Cl 1.7% off
                yield return Far.Settled("the plane, its nose back");
            }
            finally
            {
                if (nose != null && (nose.transform.position - at).magnitude > 1e-4)
                {
                    nose.transform.position = at;
                    GameEvents.onEditorPartEvent.Fire(ConstructionEventType.PartOffset, nose);
                }
            }
        }

        // ------------------------------------------------------------------------------------------ viscous

        /// <summary>
        /// The "More accurate aerodynamics" option, which solves at the flight point's own Reynolds number. On the
        /// static tab, switched on: FAR's config keeps it, and an altitude field appears. A Mach sweep from 0 (taken
        /// as Mach 0.001) gives finite coefficients; at sea level its drag is not the inviscid sweep's, at 10 km it
        /// is higher (a lower Reynolds number, more skin friction); at 100 km, above the atmosphere, it falls back to
        /// the inviscid sweep's drag (FAR's historical skin friction). An AoA sweep at Mach 0 is finite too. On the stability tab the derivatives come
        /// out with it on, the trim drag not the inviscid one; above the atmosphere the tab refuses with its error
        /// and keeps the last result. Switched off again, the config keeps that.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator EditorGui_ViscousOptionUsesTheFlightReynolds(TestContext ctx)
        {
            try
            {
                yield return OpenWindow("FAREditorModeStatic");
                ResetConditions();
                object graphGui = Get(Far.Editor, "_editorGraph");
                object machInputs = Get(graphGui, "machSweepInputs"), aoaInputs = Get(graphGui, "aoASweepInputs");
                Remember(graphGui, "isMachMode", "_altitude");
                Remember(machInputs, "lowerBound", "upperBound", "numPts", "pitchSetting", "otherInput", "spoilers");
                Remember(aoaInputs, "lowerBound", "upperBound", "numPts", "pitchSetting", "otherInput", "spoilers");
                if (!Get<bool>(graphGui, "isMachMode"))
                    yield return Imgui.Click(L("FAREditorStaticSwitchMach"), Window);
                yield return Inviscid();
                if (Get<bool>(machInputs, "spoilers"))
                    yield return Imgui.Toggle(L("FAREditorStabDerivSDeploy"), false, Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStaticPitchSetting"), "0", Window);
                Assert.False(Imgui.IsVisible("Altitude (km)", Window), "no altitude field with the option off");

                ctx.Step("Inviscid Mach sweep 0-1.5 at 2 deg");
                yield return Imgui.SetTextAfterLabel(L("FAREditorStaticGraphLowLim"), "0", Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStaticGraphUpLim"), "1.5", Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStaticGraphPtCount"), "15", Window);
                yield return Imgui.SetTextAfterLabel(L("FARAbbrevAoA"), "2", Window);
                yield return Imgui.Click(L("FAREditorStaticSweepMach"), Window);
                double[] inviscid = MachSweepCd(graphGui, out double[] machs);
                ctx.Record("inviscid", Row(machs, inviscid));

                ctx.Step("The option on");
                yield return Imgui.Toggle(ViscousToggle, true, Window);
                Assert.True(EditorSimManager.UseAccurateViscousCalc, "the option on");
                Assert.Equal(1, FARDebugAndSettings.config.GetValue("editor_useViscousCalc", -1), "FAR's config keeps it on");
                yield return Wait.Frames(3);
                Assert.True(Imgui.IsVisible("Altitude (km)", Window), "the altitude field shown with the option on");

                ctx.Step("Viscous Mach sweeps at 0, 10 and 100 km");
                var byAlt = new Dictionary<string, double[]>();
                foreach (string km in new[] { "0", "10", "100" })
                {
                    yield return Imgui.SetTextAfterLabel("Altitude (km)", km, Window);
                    yield return Imgui.Click(L("FAREditorStaticSweepMach"), Window);
                    double[] cd = MachSweepCd(graphGui, out double[] m);
                    byAlt[km] = cd;
                    ctx.Record("viscous" + km, Row(m, cd));
                    Assert.Near(0.001, m[0], 1e-12, "the sweep from Mach 0 starts at 0.001");
                    Assert.True(cd.All(c => !double.IsNaN(c) && !double.IsInfinity(c) && c > 0), $"finite, positive drag at every Mach at {km} km");
                }

                int i05 = Far.IndexNear(machs, 0.5);
                Assert.True(Math.Abs(byAlt["0"][i05] - inviscid[i05]) > 1e-5, $"at sea level the drag is the flight point's, not the inviscid one ({byAlt["0"][i05]:F5} against {inviscid[i05]:F5} at Mach 0.5)");
                Assert.True(byAlt["10"][i05] > byAlt["0"][i05], $"more skin friction at 10 km than at sea level ({byAlt["10"][i05]:F5} against {byAlt["0"][i05]:F5} at Mach 0.5)");
                for (int i = 0; i < machs.Length; i++)
                    Assert.Near(inviscid[i], byAlt["100"][i], 0.02 * inviscid[i], $"above the atmosphere the drag falls back to the inviscid one at Mach {machs[i]:F2}");

                ctx.Step("Viscous AoA sweep at Mach 0, sea level");
                yield return Imgui.SetTextAfterLabel("Altitude (km)", "0", Window);
                yield return Imgui.Click(L("FAREditorStaticSwitchAoA"), Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStaticGraphLowLim"), "-4", Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStaticGraphUpLim"), "12", Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStaticGraphPtCount"), "9", Window);
                yield return Imgui.SetTextAfterLabel(L("FARAbbrevMach"), "0", Window);
                yield return Imgui.Click(L("FAREditorStaticSweepAoA"), Window);
                Dictionary<string, double[][]> aoaLines = Lines(graphGui);
                double[] aoaCl = aoaLines[L("FARAbbrevCl")][1], aoaCd = aoaLines[L("FARAbbrevCd")][1];
                ctx.Record("aoa", Row(aoaLines[L("FARAbbrevCl")][0], aoaCl));
                Assert.Equal(9, aoaCl.Length, "points in the AoA sweep");
                Assert.True(aoaCl.Concat(aoaCd).All(c => !double.IsNaN(c) && !double.IsInfinity(c)), "finite coefficients at Mach 0");
                Assert.True(aoaCl[8] > aoaCl[0], "lift rises with incidence at Mach 0");

                ctx.Step("Stability derivatives, inviscid then viscous, at 0.5 km, Mach 0.35");
                SelectTab("FAREditorModeDataStab");
                yield return Wait.Frames(2);
                object stab = Get(Far.Editor, "_stabDeriv");
                Remember(stab, "altitude", "machNumber", "spoilersDeployed");
                if (Get<bool>(stab, "spoilersDeployed"))
                    yield return Imgui.Toggle(L("FAREditorStabDerivSDeploy"), false, Window);
                var sim = (EditorSimManager)Get(Far.Editor, "_simManager");
                yield return Imgui.SetTextAfterLabel(L("FAREditorStabDerivAlt"), "0.5", Window);
                yield return Imgui.SetTextAfterLabel(L("FAREditorStabDerivMach"), "0.35", Window);
                yield return Imgui.Toggle(ViscousToggle, false, Window);
                yield return Imgui.Click(L("FAREditorStabDerivCalcButton"), Window);
                StabilityDerivOutput inv = sim.vehicleData;
                yield return Imgui.Toggle(ViscousToggle, true, Window);
                Assert.True(EditorSimManager.UseAccurateViscousCalc, "the option on from the stability tab");
                yield return Imgui.Click(L("FAREditorStabDerivCalcButton"), Window);
                StabilityDerivOutput visc = sim.vehicleData;
                Assert.True(inv != null && visc != null && !ReferenceEquals(inv, visc), "two sets of derivatives");
                ctx.Record("trim", $"inviscid AoA {inv.stableAoA:F3} Cd {inv.stableCd:F5}; viscous AoA {visc.stableAoA:F3} Cd {visc.stableCd:F5}");
                Assert.Equal("", visc.stableAoAState, "the viscous level-flight angle of attack found");
                Assert.True(visc.stabDerivs.All(d => !double.IsNaN(d) && !double.IsInfinity(d)), "finite viscous derivatives");
                Assert.True(Math.Abs(visc.stableCd - inv.stableCd) > 1e-6, "the trim drag is the flight point's, not the inviscid one");

                ctx.Step("Stability above the atmosphere");
                yield return Imgui.SetTextAfterLabel(L("FAREditorStabDerivAlt"), "100", Window);
                yield return Imgui.Click(L("FAREditorStabDerivCalcButton"), Window);
                yield return Wait.Frames(2);
                PopupDialog popup = UnityEngine.Object.FindObjectsOfType<PopupDialog>().FirstOrDefault(d => d.dialogToDisplay?.name == "FARStabDerivError");
                Assert.NotNull(popup, "the tab's error popup");
                popup.Dismiss();
                Assert.True(ReferenceEquals(visc, sim.vehicleData), "no new derivatives above the atmosphere");

                ctx.Step("The option off");
                yield return Imgui.Toggle(ViscousToggle, false, Window);
                Assert.Equal(0, FARDebugAndSettings.config.GetValue("editor_useViscousCalc", -1), "FAR's config keeps it off");
            }
            finally
            {
                foreach (PopupDialog d in UnityEngine.Object.FindObjectsOfType<PopupDialog>())
                    if (d.dialogToDisplay?.name == "FARStabDerivError")
                        d.Dismiss();
                CloseWindow();
            }
        }

        /// <summary>The static tab's last Mach sweep: its Cd line and its Mach points.</summary>
        private static double[] MachSweepCd(object graphGui, out double[] machs)
        {
            Dictionary<string, double[][]> lines = Lines(graphGui);
            machs = lines[L("FARAbbrevCd")][0];
            return lines[L("FARAbbrevCd")][1];
        }

        private static string Row(double[] x, double[] y) =>
            string.Join(" ", Enumerable.Range(0, x.Length).Where(i => i % 3 == 0).Select(i => $"{x[i]:F2}:{y[i]:F5}"));

        /// <summary>The lines the tab's graph plots, by name: x and y.</summary>
        private static Dictionary<string, double[][]> Lines(object tab)
        {
            object graph = Get(tab, "_graph");
            var all = (IDictionary)Get(graph, "allLines");
            var lines = new Dictionary<string, double[][]>();
            foreach (object key in all.Keys)
                lines[(string)key] = new[] { (double[])Get(all[key], "rawDataX"), (double[])Get(all[key], "rawDataY") };
            return lines;
        }
    }
}
