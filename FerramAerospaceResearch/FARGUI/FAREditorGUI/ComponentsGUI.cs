/*
Ferram Aerospace Research v0.16.1.2 "Marangoni"
=========================
Aerodynamics model for Kerbal Space Program

Copyright 2022, Michael Ferrara, aka Ferram4

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Ferram Aerospace Research is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Ferram Aerospace Research.  If not, see <http://www.gnu.org/licenses/>.

   Serious thanks:		a.g., for tons of bugfixes and code-refactorings
				stupid_chris, for the RealChuteLite implementation
            			Taverius, for correcting a ton of incorrect values
				Tetryds, for finding lots of bugs and issues and not letting me get away with them, and work on example crafts
            			sarbian, for refactoring code for working with MechJeb, and the Module Manager updates
            			ialdabaoth (who is awesome), who originally created Module Manager
                        	Regex, for adding RPM support
				DaMichel, for some ferramGraph updates and some control surface-related features
            			Duxwing, for copy editing the readme

   CompatibilityChecker by Majiir, BSD 2-clause http://opensource.org/licenses/BSD-2-Clause

   Part.cfg changes powered by sarbian & ialdabaoth's ModuleManager plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/55219

   ModularFLightIntegrator by Sarbian, Starwaster and Ferram4, MIT: http://opensource.org/licenses/MIT
	http://forum.kerbalspaceprogram.com/threads/118088

   Toolbar integration powered by blizzy78's Toolbar plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/60863
 */

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using KSP.Localization;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI
{
    /// <summary>
    ///     The editor's Components tab: the craft part by part, as FAR models it. A table of the parts that make forces (wing
    ///     panels, control surfaces, bodies) with each one's area, aspect ratios, shielding and forces at a flight
    ///     condition; the parts tinted by any of those; the flaps and spoilers set (and shown) for it; and the voxels.
    /// </summary>
    internal class ComponentsGUI
    {
        private enum Tint
        {
            None,
            Parts,
            AspectRatio,
            Shielded,
            Cl,
            Cd,
            Cm
        }

        private static readonly string[] TintNames = {"None", "Parts", "Aspect Ratio", "Shielded", "Cl", "Cd", "Cm"};

        private sealed class Row
        {
            public Part Part;
            public string Name;
            public FARWingAerodynamicModel Wing;
            public double S, OwnAR, EffectiveAR;
            public bool Shielded;
            // forces at the condition (kN) and the pitching moment about the CoM (kN m), and the part's share of the
            // craft's coefficients (on the wing reference area and MAC, as the other editor tabs report the craft's)
            public double Lift, Drag, Moment, Cl, Cd, Cm;
        }

        private readonly EditorSimManager simManager;
        private readonly GUIDropDown<int> flapDropdown;
        private readonly GUIDropDown<CelestialBody> bodyDropdown;
        private readonly Action showVoxels;

        private string speed = "100", altitude = "0", aoa = "2";
        private bool spoilers;
        private Tint tint = Tint.None;
        private readonly GUIDropDown<Tint> tintDropdown =
            new GUIDropDown<Tint>(TintNames, (Tint[])Enum.GetValues(typeof(Tint))) {VisibleItems = 15};
        // the table of parts shown (off by default: it is long, and the window shrinks to leave the craft in view)
        private bool showTable;
        // the table shown or hidden since the window last fit itself to the tab
        private bool resized;
        private Vector2 scroll;
        private readonly List<Row> rows = new List<Row>();
        private string summary = "";
        // the legend as text (for scripts), and as drawn: a title, then a gradient bar with the value under each of its
        // colors (ticks: the position along the bar, 0..1, and the value) or swatches of color with what each means
        private string legend = "";
        private string legendTitle = "";
        private readonly List<KeyValuePair<float, string>> legendTicks = new List<KeyValuePair<float, string>>();
        private readonly List<KeyValuePair<Color, string>> legendSwatches = new List<KeyValuePair<Color, string>>();
        private Texture2D legendGradient;
        private string legendGradientKey;
        private GUIStyle tickStyle;
        private const float LegendWidth = 420;
        private const int SwatchesPerRow = 5;
        private bool dirty = true;
        private string lastInputs;

        // how much of the color covers the part's own (0: none, 1: all)
        private float opacity = 0.8f;

        // the parts colored: the color goes into the material colors that the shaders multiply their textures by (stock's
        // _Color, TexturesUnlimited's recolor masks; the heat-glow channel FAR tints with in flight is lost on TU's
        // shaders), set in the part's own property block (Part.mpb), which KSP puts back on its renderers whenever it
        // highlights the part (a block of our own on the renderers was lost at the first mouse-over)
        private readonly HashSet<Part> colored = new HashSet<Part>();
        private static readonly int[] ColorProperties =
        {
            Shader.PropertyToID("_Color"), Shader.PropertyToID("_MaskColor1"), Shader.PropertyToID("_MaskColor2"),
            Shader.PropertyToID("_MaskColor3")
        };

        // the part picked in the table, lit by KSP's part highlighter
        private Part selected;
        private GUIStyle nameStyle;

        public ComponentsGUI(
            EditorSimManager simManager,
            GUIDropDown<int> flapDropdown,
            GUIDropDown<CelestialBody> bodyDropdown,
            Action showVoxels
        )
        {
            this.simManager = simManager;
            this.flapDropdown = flapDropdown;
            this.bodyDropdown = bodyDropdown;
            this.showVoxels = showVoxels;
        }

        /// <summary>Whether the window should fit itself to the tab again (the table shown or hidden), once</summary>
        public bool TakeResize()
        {
            bool r = resized;
            resized = false;
            return r;
        }

        /// <summary>The craft's aero data changed (a new voxelization): recompute when next shown</summary>
        public void MarkDirty()
        {
            dirty = true;
        }

        /// <summary>The planet, in the window's top row (beside the tab dropdown) while this tab is shown</summary>
        public void PlanetDisplay()
        {
            GUILayout.Label(Localizer.Format("FAREditorStabDerivPlanet"));
            bodyDropdown.GUIDropDownDisplay(GUILayout.Width(130));
        }

        public void Display()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("Flaps");
            flapDropdown.GUIDropDownDisplay(GUILayout.Width(150));
            if (GUILayout.Button(spoilers ? "Spoilers extended" : "Spoilers retracted", GUILayout.Width(140)))
                spoilers = !spoilers;
            GUILayout.Label("Speed (m/s)");
            speed = GUILayout.TextField(speed, GUILayout.Width(60));
            GUILayout.Label(Localizer.Format("FAREditorStabDerivAlt"));
            altitude = GUILayout.TextField(altitude, GUILayout.Width(50));
            GUILayout.Label("AoA (deg)");
            aoa = GUILayout.TextField(aoa, GUILayout.Width(40));
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Color by");
            tintDropdown.GUIDropDownDisplay(GUILayout.Width(150));
            // taken in the layout pass only: the legend drawn below changes with it, and every event after the layout
            // has to draw the same controls
            if (Event.current.type == EventType.Layout && tintDropdown.ActiveSelection != tint)
            {
                tint = tintDropdown.ActiveSelection;
                ApplyTints();
                resized = true;
            }

            GUI.enabled = !EditorGUI.Instance.VoxelizationUpdateQueued;
            if (GUILayout.Button("Show Voxels", GUILayout.Width(100)))
                showVoxels();
            GUI.enabled = true;
            if (GUILayout.Button(showTable ? "Hide Table" : "Show Table", GUILayout.Width(100)))
            {
                showTable = !showTable;
                resized = true;
            }

            GUILayout.FlexibleSpace();
            GUILayout.Label("Opacity " + opacity.ToString("F2", CultureInfo.InvariantCulture), GUILayout.Width(85));
            float o = GUILayout.HorizontalSlider(opacity, 0, 1, GUILayout.Width(90));
            if (Math.Abs(o - opacity) > 0.005f)
            {
                opacity = o;
                ApplyTints();
            }

            GUILayout.EndHorizontal();

            string inputs = string.Join("|", speed, altitude, aoa, spoilers, flapDropdown.ActiveSelection,
                                        bodyDropdown.ActiveSelection?.bodyName);
            if (inputs != lastInputs)
            {
                lastInputs = inputs;
                dirty = true;
            }

            // the rows only in the layout pass: IMGUI needs the same controls in the event that follows it
            if (dirty && Event.current.type == EventType.Layout && !EditorGUI.Instance.VoxelizationUpdateQueued)
            {
                dirty = false;
                Compute();
                ApplyTints();
            }

            GUILayout.Label(summary);
            DrawLegend();
            if (showTable)
                Table();
        }

        private void Table()
        {
            const int wName = 250, wNum = 52;
            if (nameStyle == null)
                nameStyle = new GUIStyle(GUI.skin.label) {wordWrap = false, clipping = TextClipping.Clip};
            string[] heads = {"S m2", "AR own", "AR", "Shld", "L kN", "D kN", "M kNm", "Cl", "Cd", "Cm"};
            // the scroll view as wide as the columns, so that only its vertical bar shows; the header over the columns
            RectOffset margin = GUI.skin.label.margin, pad = GUI.skin.scrollView.padding;
            float width = wName + heads.Length * wNum + (heads.Length + 1) * Math.Max(margin.left, margin.right) +
                          pad.horizontal + GUI.skin.verticalScrollbar.fixedWidth + 8;
            GUILayout.BeginHorizontal();
            GUILayout.Space(pad.left);
            GUILayout.Label("Part, side and station (click: pick)", GUILayout.Width(wName));
            foreach (string h in heads)
                GUILayout.Label(h, GUILayout.Width(wNum));
            GUILayout.EndHorizontal();

            scroll = GUILayout.BeginScrollView(scroll, false, true, GUILayout.Height(300), GUILayout.Width(width));
            foreach (Row r in rows)
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button((r.Part == selected ? "> " : "") + r.Name, nameStyle, GUILayout.Width(wName)))
                {
                    selected = r.Part == selected ? null : r.Part;
                    ApplyTints();
                }
                bool wing = r.Wing != null;
                Cell(wing ? r.S.ToString("F2", CultureInfo.InvariantCulture) : "", wNum);
                Cell(wing ? r.OwnAR.ToString("F2", CultureInfo.InvariantCulture) : "", wNum);
                Cell(wing ? r.EffectiveAR.ToString("F2", CultureInfo.InvariantCulture) : "", wNum);
                Cell(wing ? (r.Shielded ? "yes" : "") : "", wNum);
                Cell(r.Lift.ToString("F2", CultureInfo.InvariantCulture), wNum);
                Cell(r.Drag.ToString("F2", CultureInfo.InvariantCulture), wNum);
                Cell(r.Moment.ToString("F1", CultureInfo.InvariantCulture), wNum);
                Cell(r.Cl.ToString("F4", CultureInfo.InvariantCulture), wNum);
                Cell(r.Cd.ToString("F4", CultureInfo.InvariantCulture), wNum);
                Cell(r.Cm.ToString("F4", CultureInfo.InvariantCulture), wNum);
                GUILayout.EndHorizontal();
            }

            GUILayout.EndScrollView();
        }

        private static void Cell(string text, int width)
        {
            GUILayout.Label(text, GUILayout.Width(width));
        }

        private static double Parse(string s, double fallback)
        {
            return double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out double v) ? v : fallback;
        }

        /// <summary>
        ///     Every part's forces at the condition, as InstantConditionSim adds them up for the craft (the same
        ///     velocity frame, CoM and reference area), kept part by part
        /// </summary>
        private void Compute()
        {
            rows.Clear();
            List<Part> parts = EditorLogic.SortedShipList;
            List<FARWingAerodynamicModel> wings = simManager.Wings;
            List<FARAeroSection> sections = simManager.AeroSections;
            CelestialBody body = bodyDropdown.ActiveSelection;
            if (parts == null || parts.Count == 0 || wings == null || sections == null || body == null)
            {
                summary = "No craft data yet";
                return;
            }

            FARAeroUtil.UpdateCurrentActiveBody(body);
            double alt = Parse(altitude, 0) * 1000;
            double v = Math.Max(Parse(speed, 100), 0.1);
            double alpha = Parse(aoa, 0) * Math.PI / 180;
            GasProperties gas = FARAtmosphere.GetGasProperties(body, new Vector3d(0, 0, alt), Planetarium.GetUniversalTime());
            double mach = gas.SpeedOfSound > 0 ? v / gas.SpeedOfSound : 0;
            // kPa: the forces below are per unit dynamic pressure (m2), as InstantConditionSim computes them
            double q = 0.5 * gas.Density * v * v * 0.001;
            int flaps = flapDropdown.ActiveSelection;

            Vector3d forward = Vector3.forward, up = Vector3.up, right = Vector3.right;
            if (EditorDriver.editorFacility == EditorFacility.VAB)
            {
                forward = Vector3.up;
                up = -Vector3.forward;
            }

            Vector3d com = Vector3d.zero;
            double mass = 0;
            foreach (Part p in parts)
            {
                if (FARAeroUtil.IsNonphysical(p))
                    continue;
                double m = p.mass + (p.Resources.Count > 0 ? p.GetResourceMass() : 0);
                com += m * (Vector3d)p.transform.TransformPoint(p.CoMOffset);
                mass += m;
            }

            if (mass > 0)
                com /= mass;

            Vector3d velocity = (forward * Math.Cos(alpha) - up * Math.Sin(alpha)).normalized;
            Vector3d liftDown = (-forward * Math.Sin(alpha) - up * Math.Cos(alpha)).normalized;
            Vector3d sideways = Vector3d.Cross(velocity, liftDown).normalized;

            var byPart = new Dictionary<Part, Row>();
            Transform root = EditorLogic.RootPart.partTransform;

            Row RowOf(Part p)
            {
                if (byPart.TryGetValue(p, out Row r))
                    return r;
                // which side, and how far out (m): the craft's parts of one kind share a title
                Vector3 at = root.InverseTransformPoint(p.transform.position);
                string side = Math.Abs(at.x) < 0.05 ? "C" : at.x > 0 ? "R" : "L";
                r = new Row
                {
                    Part = p,
                    Name = string.Format(CultureInfo.InvariantCulture, "{0} {1} {2:F1}",
                                         p.partInfo != null ? p.partInfo.title : p.name, side, Math.Abs(at.x))
                };
                byPart[p] = r;
                rows.Add(r);
                return r;
            }

            double area = 0, mac = 0;
            foreach (FARWingAerodynamicModel w in wings)
            {
                if (!(w && w.part))
                    continue;
                Row r = RowOf(w.part);
                r.Wing = w;

                // as InstantConditionSim: the forces once to set the wing's state, the stall cleared, the controls set
                w.ComputeForceEditor(velocity, mach, 2);
                w.EditorClClear(true);
                if (w is FARControllableSurface surface)
                    surface.SetControlStateEditor(com, velocity, 0, 0, 0, flaps, spoilers);
                Vector3d force = Vector3d.zero;
                if (InstantConditionSim.Counts(w))
                    force = w.ComputeForceEditor(velocity, mach, 2) * 1000;

                r.S = w.S;
                r.OwnAR = w.OwnAR;
                r.EffectiveAR = w.EffectiveAR;
                r.Shielded = w.isShielded;

                Vector3d moment = -Vector3d.Cross(w.GetAerodynamicCenter() - com, force);
                r.Lift += -Vector3d.Dot(force, liftDown);
                r.Drag += -Vector3d.Dot(force, velocity);
                r.Moment += Vector3d.Dot(moment, sideways);
                if (InstantConditionSim.Counts(w))
                {
                    area += w.S;
                    mac += w.GetMAC() * w.S;
                }
            }

            var perPart = new Dictionary<Part, FARCenterQuery>();
            var center = new FARCenterQuery();
            foreach (FARAeroSection section in sections)
                section.PredictionCalculateAeroForces(2, (float)mach, 10000, 0, 0.005f, velocity.normalized, center, perPart);
            foreach (KeyValuePair<Part, FARCenterQuery> kv in perPart)
            {
                if (kv.Key == null)
                    continue;
                Row r = RowOf(kv.Key);
                Vector3d force = kv.Value.force * 1000;
                Vector3d moment = -kv.Value.TorqueAt(com) * 1000;
                r.Lift += -Vector3d.Dot(force, liftDown);
                r.Drag += -Vector3d.Dot(force, velocity);
                r.Moment += Vector3d.Dot(moment, sideways);
            }

            if (area <= 0)
            {
                area = 1;
                mac = 1;
            }
            else
            {
                mac /= area;
            }

            double cl = 0, cd = 0, cm = 0;
            foreach (Row r in rows)
            {
                // per unit q so far (m2, m3): the share of the craft's coefficients, then to kN at the condition
                r.Cl = r.Lift / area;
                r.Cd = r.Drag / area;
                r.Cm = r.Moment / (area * mac);
                cl += r.Cl;
                cd += r.Cd;
                cm += r.Cm;
                r.Lift *= q;
                r.Drag *= q;
                r.Moment *= q;
            }

            rows.Sort((a, b) => Math.Abs(b.Lift) + Math.Abs(b.Drag) > Math.Abs(a.Lift) + Math.Abs(a.Drag) ? 1 :
                                Math.Abs(b.Lift) + Math.Abs(b.Drag) < Math.Abs(a.Lift) + Math.Abs(a.Drag) ? -1 : 0);
            summary = string.Format(CultureInfo.InvariantCulture,
                                    "Mach {0:F3}, q {1:F2} kPa; craft Cl {2:F4}, Cd {3:F4}, Cm {4:F4} on S {5:F2} m2, MAC {6:F2} m",
                                    mach, q, cl, cd, cm, area, mac);
        }

        /// <summary>The parts tinted by the chosen quantity (none: cleared); the legend to go with it</summary>
        public void ApplyTints()
        {
            double max = 0;
            foreach (Row r in rows)
                max = Math.Max(max, Math.Abs(Value(r)));
            if (max <= 0)
                max = 1;
            BuildLegend(max);

            var seen = new HashSet<Part>();
            foreach (Row r in rows)
            {
                if (r.Part == null)
                    continue;
                seen.Add(r.Part);
                SetTint(r.Part, ColorOf(r, max));
            }

            // parts in neither list (structure, engines) take the Parts colors too, and lose any old color
            int k = rows.Count;
            foreach (Part p in EditorLogic.SortedShipList)
            {
                if (p == null || seen.Contains(p))
                    continue;
                SetTint(p, tint == Tint.Parts ? Hue(k++) : Clear);
            }

            // parts gone from the craft
            colored.RemoveWhere(p => p == null);

            // the picked part outlined
            if (lastPicked != selected)
            {
                if (lastPicked != null)
                    lastPicked.SetHighlightDefault();
                if (selected != null)
                {
                    selected.SetHighlightType(Part.HighlightType.AlwaysOn);
                    selected.SetHighlightColor(Color.white);
                    selected.SetHighlight(true, false);
                }

                lastPicked = selected;
            }
        }

        private Part lastPicked;

        /// <summary>The legend for the chosen quantity, its largest magnitude among the parts being max</summary>
        private void BuildLegend(double max)
        {
            legendTitle = "";
            legendTicks.Clear();
            legendSwatches.Clear();
            Func<float, Color> colorAt = null;
            string key = null;
            switch (tint)
            {
                case Tint.Parts:
                    legendTitle = "One color per part";
                    break;
                case Tint.AspectRatio:
                    legendTitle = "Aspect ratio";
                    colorAt = ArColor;
                    for (int i = 0; i <= 4; i++)
                        legendTicks.Add(new KeyValuePair<float, string>(
                                            i / 4f, (max * i / 4).ToString("F1", CultureInfo.InvariantCulture)));
                    key = "AR " + max;
                    break;
                case Tint.Shielded:
                    legendTitle = "Shielding";
                    legendSwatches.Add(new KeyValuePair<Color, string>(ShieldedColor, "shielded (no lift)"));
                    legendSwatches.Add(new KeyValuePair<Color, string>(ExposedColor, "exposed"));
                    break;
                case Tint.Cl:
                case Tint.Cd:
                case Tint.Cm:
                    // blue only when a part takes from the craft's
                    bool negative = rows.Any(r => Value(r) < 0);
                    legendTitle = TintNames[(int)tint] + ", each part's share of the craft's";
                    colorAt = f => ShareColor(negative ? 2 * f - 1 : f);
                    int n = negative ? 5 : 3;
                    for (int i = 0; i < n; i++)
                    {
                        float f = i / (n - 1f);
                        double v = (negative ? 2 * f - 1 : f) * max;
                        legendTicks.Add(new KeyValuePair<float, string>(
                                            f, v.ToString("F4", CultureInfo.InvariantCulture)));
                    }

                    key = tint + " " + negative + " " + max;
                    break;
            }

            if (colorAt != null && key != legendGradientKey)
            {
                if (legendGradient == null)
                    legendGradient = new Texture2D(64, 1, TextureFormat.RGBA32, false)
                    {
                        wrapMode = TextureWrapMode.Clamp, filterMode = FilterMode.Bilinear
                    };
                for (int x = 0; x < legendGradient.width; x++)
                    legendGradient.SetPixel(x, 0, colorAt(x / (legendGradient.width - 1f)));
                legendGradient.Apply();
                legendGradientKey = key;
            }

            IEnumerable<string> entries = legendTicks.Count > 0
                                              ? legendTicks.Select(t => t.Value)
                                              : legendSwatches.Select(w => w.Value);
            legend = legendTitle.Length == 0 ? "" : legendTitle + ": " + string.Join(", ", entries.ToArray());
        }

        // the legend: the title beside a gradient bar with each color's value under it, or rows of swatches
        private void DrawLegend()
        {
            if (tickStyle == null)
                tickStyle = new GUIStyle(GUI.skin.label) {wordWrap = false, clipping = TextClipping.Overflow};
            if (legendTicks.Count > 0 && legendGradient != null)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(legendTitle);
                GUILayout.BeginVertical(GUILayout.Width(LegendWidth));
                Rect bar = GUILayoutUtility.GetRect(LegendWidth, 14, GUILayout.Width(LegendWidth));
                Rect values = GUILayoutUtility.GetRect(LegendWidth, 20, GUILayout.Width(LegendWidth));
                GUILayout.EndVertical();
                GUILayout.FlexibleSpace();
                GUILayout.EndHorizontal();
                if (Event.current.type != EventType.Repaint)
                    return;
                GUI.DrawTexture(bar, legendGradient, ScaleMode.StretchToFill);
                for (int i = 0; i < legendTicks.Count; i++)
                {
                    // each value under its color: the ends inside the bar, the others centered on theirs
                    float x = bar.x + legendTicks[i].Key * bar.width;
                    GUI.DrawTexture(new Rect(x - 1, bar.yMax - 4, 2, 4), Texture2D.whiteTexture);
                    tickStyle.alignment = i == 0 ? TextAnchor.UpperLeft :
                                          i == legendTicks.Count - 1 ? TextAnchor.UpperRight : TextAnchor.UpperCenter;
                    var r = new Rect(i == 0 ? bar.x : i == legendTicks.Count - 1 ? bar.xMax - 80 : x - 40, values.y, 80,
                                     values.height);
                    GUI.Label(r, legendTicks[i].Value, tickStyle);
                }

                return;
            }

            for (int start = 0; start == 0 || start < legendSwatches.Count; start += SwatchesPerRow)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(start == 0 ? legendTitle : "");
                for (int i = start; i < legendSwatches.Count && i < start + SwatchesPerRow; i++)
                {
                    // a gap between one entry and the next
                    if (i > start)
                        GUILayout.Space(16);
                    Rect swatch = GUILayoutUtility.GetRect(14, 20, GUILayout.Width(14), GUILayout.Height(20));
                    if (Event.current.type == EventType.Repaint)
                    {
                        Color old = GUI.color;
                        GUI.color = legendSwatches[i].Key;
                        GUI.DrawTexture(new Rect(swatch.x, swatch.y + 4, 14, 14), Texture2D.whiteTexture);
                        GUI.color = old;
                    }

                    GUILayout.Label(legendSwatches[i].Value);
                }

                GUILayout.FlexibleSpace();
                GUILayout.EndHorizontal();
            }
        }

        private double Value(Row r)
        {
            switch (tint)
            {
                case Tint.AspectRatio:
                    return r.Wing != null ? r.EffectiveAR : 0;
                case Tint.Cl:
                    return r.Cl;
                case Tint.Cd:
                    return r.Cd;
                case Tint.Cm:
                    return r.Cm;
                default:
                    return 0;
            }
        }

        private static readonly Color Clear = new Color(0, 0, 0, 0);
        private static readonly Color ShieldedColor = new Color(1, 0.1f, 0.1f), ExposedColor = new Color(0.1f, 0.8f, 0.1f);

        // aspect ratio, as a fraction of the largest: blue 0, cyan, green, yellow, red at the largest
        private static Color ArColor(float f)
        {
            return Color.HSVToRGB(0.66f * (1 - Mathf.Clamp01(f)), 1, 1);
        }

        // a part's share of a coefficient, as a fraction of the largest either way: white 0, red positive, blue negative
        private static Color ShareColor(float s)
        {
            return s >= 0 ? Color.Lerp(Color.white, Color.red, s) : Color.Lerp(Color.white, Color.blue, -s);
        }

        private Color ColorOf(Row r, double max)
        {
            switch (tint)
            {
                case Tint.Parts:
                    return Hue(rows.IndexOf(r));
                case Tint.AspectRatio:
                    return r.Wing == null ? Clear : ArColor((float)(r.EffectiveAR / max));
                case Tint.Shielded:
                    return r.Wing == null ? Clear : r.Shielded ? ShieldedColor : ExposedColor;
                case Tint.Cl:
                case Tint.Cd:
                case Tint.Cm:
                    return ShareColor((float)(Value(r) / max));
                default:
                    return Clear;
            }
        }

        // well-separated colors: golden-ratio hues, alternating brightness
        private static Color Hue(int i)
        {
            return Color.HSVToRGB(i * 0.618034f % 1f, 0.85f, i % 2 == 0 ? 1f : 0.65f);
        }

        /// <summary>A part colored, by the opacity over its own colors (Clear: its own colors back)</summary>
        private void SetTint(Part p, Color c)
        {
            if (c.a <= 0)
            {
                if (colored.Remove(p))
                    Uncolor(p);
                return;
            }

            // one block for the whole part: each color its materials have (the first material that has it), moved
            // toward this one by the opacity (its alpha, a TU mask's specular or metal share, kept)
            MaterialPropertyBlock block = p.mpb;
            List<Renderer> renderers = Renderers(p);
            foreach (int id in ColorProperties)
            {
                Material m = renderers.Select(r => r.sharedMaterial).FirstOrDefault(x => x != null && x.HasProperty(id));
                if (m == null)
                    continue;
                Color own = m.GetColor(id);
                Color mixed = Color.Lerp(own, c, opacity);
                mixed.a = own.a;
                block.SetColor(id, mixed);
            }

            foreach (Renderer r in renderers)
                r.SetPropertyBlock(block);
            colored.Add(p);
        }

        // the renderers KSP highlights (and puts the part's block on), else the model's
        private static List<Renderer> Renderers(Part p)
        {
            List<Renderer> list = p.HighlightRenderer != null && p.HighlightRenderer.Count > 0
                                      ? p.HighlightRenderer
                                      : p.FindModelComponents<Renderer>();
            return list.Where(r => r != null && (r is MeshRenderer || r is SkinnedMeshRenderer)).ToList();
        }

        // the part's block emptied (Part.ResetMPB leaves other properties in it), then the highlight switched on and off so
        // that KSP writes its rim back into it and puts it on the renderers (emptied alone, the material's own rim glowed
        // green on every part)
        private static void Uncolor(Part p)
        {
            p.mpb.Clear();
            p.SetHighlight(true, false);
            p.SetHighlight(false, false);
            p.SetHighlightDefault();
        }

        /// <summary>Every color cleared (the tab closed or left), the picked part too</summary>
        public void ClearTints()
        {
            foreach (Part p in colored)
                if (p != null)
                    Uncolor(p);
            colored.Clear();
            if (lastPicked != null)
                lastPicked.SetHighlightDefault();
            lastPicked = selected = null;
        }
    }
}
