/*
Ferram Aerospace Research
=========================
Shared axis rendering and hit-testing for (Mach × altitude) heatmaps.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System.Globalization;
using KSP.Localization;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI
{
    /// <summary>
    /// Layout, axis labelling and hover hit-testing shared by the editor's (Mach × altitude)
    /// heatmaps. Kept in one place so the two maps cannot disagree about which way altitude runs —
    /// they previously each carried their own copy of the texture-orientation logic, and both had it
    /// upside down relative to their own tooltips.
    /// </summary>
    internal static class HeatmapAxes
    {
        private const float YAxisWidth = 52f;
        private const float XAxisHeight = 20f;
        private const float AxisTitleHeight = 18f;

        /// <summary>
        /// Texture row for an altitude index. Texture y counts up from the bottom and
        /// GUI.DrawTexture puts the texture's top row at the top of the rect, so the altitude index
        /// maps straight through: index 0 (the ground) lands on the bottom row.
        /// </summary>
        public static int TexRow(int altIndex)
        {
            return altIndex;
        }

        /// <summary>
        /// Reserves layout space for a map of the given texture plus its axis gutters, and returns
        /// the rect the texture itself should occupy.
        /// </summary>
        public static Rect ReserveRect(Texture2D tex, float maxWidth = 480f, float maxHeight = 280f)
        {
            float aspect = (float)tex.width / tex.height;
            float w = maxWidth, h = maxWidth / aspect;
            if (h > maxHeight)
            {
                h = maxHeight;
                w = h * aspect;
            }

            Rect outer = GUILayoutUtility.GetRect(w + YAxisWidth,
                                                  h + XAxisHeight + AxisTitleHeight,
                                                  GUILayout.ExpandWidth(false));
            return new Rect(outer.x + YAxisWidth, outer.y, w, h);
        }

        /// <summary>
        /// Draws tick labels down the left (altitude, km) and along the bottom (Mach), plus the Mach
        /// axis title.
        ///
        /// Ticks sit at cell centres rather than edges: each cell is a sample at a point, not a
        /// bucket spanning a range, so its value belongs to the axis value in its middle.
        /// </summary>
        public static void Draw(Rect map, double[] machAxis, double[] altAxis, bool speedAxis = false)
        {
            if (machAxis == null || altAxis == null)
                return;

            var tickStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 10,
                alignment = TextAnchor.MiddleRight,
                padding = new RectOffset(0, 0, 0, 0)
            };
            tickStyle.normal.textColor = Color.white;

            var xTickStyle = new GUIStyle(tickStyle) {alignment = TextAnchor.UpperCenter};
            var titleStyle = new GUIStyle(tickStyle) {alignment = TextAnchor.MiddleCenter, fontSize = 11};

            float cellH = map.height / altAxis.Length;
            float cellW = map.width / machAxis.Length;

            // Thin labels out when cells get small, so they never overlap.
            int altStep = Mathf.Max(1, Mathf.CeilToInt(14f / cellH));
            int machStep = Mathf.Max(1, Mathf.CeilToInt(34f / cellW));

            for (int j = 0; j < altAxis.Length; j += altStep)
            {
                // Altitude increases up the screen, so index 0 sits on the bottom row.
                float y = map.yMax - (j + 0.5f) * cellH;
                GUI.Label(new Rect(map.x - YAxisWidth, y - 7, YAxisWidth - 6, 14),
                          (altAxis[j] / 1000).ToString("F1", CultureInfo.InvariantCulture),
                          tickStyle);
            }

            string tickFormat = speedAxis ? "F0" : "F2";
            for (int i = 0; i < machAxis.Length; i += machStep)
            {
                float x = map.x + (i + 0.5f) * cellW;
                GUI.Label(new Rect(x - 20, map.yMax + 2, 40, XAxisHeight),
                          machAxis[i].ToString(tickFormat, CultureInfo.InvariantCulture),
                          xTickStyle);
            }

            GUI.Label(new Rect(map.x - YAxisWidth, map.y - 2, YAxisWidth - 6, 14),
                      Localizer.Format("FAREnvelopeAxisAlt"),
                      tickStyle);

            GUI.Label(new Rect(map.x, map.yMax + XAxisHeight, map.width, AxisTitleHeight),
                      speedAxis ? "Speed (m/s)" : Localizer.Format("FAREnvelopeAxisMach"),
                      titleStyle);
        }

        /// <summary>
        /// Outlines one grid cell. The same axis convention as HitTest: Mach increases left to
        /// right, altitude bottom to top, so cell j sits (nAlt-1-j) rows down from the top.
        /// </summary>
        public static void DrawCellBorder(Rect map, double[] machAxis, double[] altAxis, (int i, int j) cell, Color color)
        {
            if (machAxis == null || altAxis == null)
                return;

            float cellW = map.width / machAxis.Length;
            float cellH = map.height / altAxis.Length;
            float x = map.x + cell.i * cellW;
            float y = map.y + (altAxis.Length - 1 - cell.j) * cellH;
            var r = new Rect(x, y, cellW, cellH);

            const float t = 2f;
            Color prev = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(new Rect(r.x, r.y, r.width, t), Texture2D.whiteTexture);        // top
            GUI.DrawTexture(new Rect(r.x, r.yMax - t, r.width, t), Texture2D.whiteTexture); // bottom
            GUI.DrawTexture(new Rect(r.x, r.y, t, r.height), Texture2D.whiteTexture);       // left
            GUI.DrawTexture(new Rect(r.xMax - t, r.y, t, r.height), Texture2D.whiteTexture);// right
            GUI.color = prev;
        }

        /// <summary>
        /// Cell under the cursor, or null if it is outside the map. Inverts screen y, which runs
        /// down, against altitude, which runs up.
        /// </summary>
        public static (int i, int j)? HitTest(Rect map, double[] machAxis, double[] altAxis)
        {
            if (machAxis == null || altAxis == null)
                return null;

            Vector2 mouse = Event.current.mousePosition;
            if (!map.Contains(mouse))
                return null;

            int i = Mathf.Clamp((int)((mouse.x - map.x) / map.width * machAxis.Length), 0, machAxis.Length - 1);
            int j = Mathf.Clamp(altAxis.Length - 1 - (int)((mouse.y - map.y) / map.height * altAxis.Length),
                                0,
                                altAxis.Length - 1);
            return (i, j);
        }

        /// <summary>
        /// True if the map was left-clicked this event. Consumes the event so the click does not
        /// also fall through to the window beneath.
        /// </summary>
        public static bool ClickedIn(Rect map)
        {
            Event e = Event.current;
            if (e == null || e.type != EventType.MouseDown || e.button != 0 || !map.Contains(e.mousePosition))
                return false;

            e.Use();
            return true;
        }

        // A GUILayout.Window clips its contents to the window rect, so a tooltip drawn inside the map
        // window is truncated at the edge; and IMGUI always draws windows on top of plain GUI, so a
        // tooltip drawn at the top level lands BEHIND the map window. The way to get both unclipped
        // AND on top is to draw the tooltip as its own borderless window, brought to the front. The
        // hovered tooltip is recorded (in screen space) during the map's pass, then this overlay
        // window is emitted every frame by DrawQueuedTooltip — off-screen when there is nothing to
        // show, so it is declared consistently across the Layout and Repaint passes.
        private const int TooltipWindowId = 0x7A9B3C0D;

        private static string _pendingText;
        private static Vector2 _pendingScreenPos;
        private static Vector2 _pendingSize;
        private static bool _hasPending;
        private static GUIStyle _tooltipStyle;
        private static readonly GUIContent _tooltipContent = new GUIContent();

        /// <summary>
        /// Records a tooltip to be drawn (unclipped, on top) by <see cref="DrawQueuedTooltip" />. The
        /// mouse position is captured in screen space now, while the map window's transform is active.
        /// </summary>
        public static void DrawTooltip(string text)
        {
            _pendingText = text;
            _pendingScreenPos = GUIUtility.GUIToScreenPoint(Event.current.mousePosition);
            _hasPending = true;
        }

        /// <summary>
        /// Emits the tooltip overlay window. Call once at the top level of OnGUI, after the map
        /// window(s). Draws nothing (window parked off-screen) on frames with no hover.
        /// </summary>
        public static void DrawQueuedTooltip()
        {
            if (_tooltipStyle == null)
            {
                _tooltipStyle = new GUIStyle(GUI.skin.box)
                {
                    alignment = TextAnchor.UpperLeft,
                    wordWrap = false,
                    padding = new RectOffset(6, 6, 4, 4)
                };
                _tooltipStyle.normal.textColor = Color.white;
            }

            Rect rect;
            if (_hasPending)
            {
                _tooltipContent.text = _pendingText;
                _pendingSize = _tooltipStyle.CalcSize(_tooltipContent);

                // Prefer down-right of the cursor; flip near a screen edge, then clamp to the screen.
                float x = _pendingScreenPos.x + 14;
                if (x + _pendingSize.x > Screen.width - 4)
                    x = _pendingScreenPos.x - _pendingSize.x - 14;

                float y = _pendingScreenPos.y + 14;
                if (y + _pendingSize.y > Screen.height - 4)
                    y = _pendingScreenPos.y - _pendingSize.y - 14;

                x = Mathf.Clamp(x, 4f, Mathf.Max(4f, Screen.width - _pendingSize.x - 4));
                y = Mathf.Clamp(y, 4f, Mathf.Max(4f, Screen.height - _pendingSize.y - 4));
                rect = new Rect(x, y, _pendingSize.x, _pendingSize.y);
            }
            else
            {
                rect = new Rect(-10000, -10000, 1, 1); // parked; nothing to show this frame
            }

            GUI.Window(TooltipWindowId, rect, DrawTooltipWindow, GUIContent.none, GUIStyle.none);
            GUI.BringWindowToFront(TooltipWindowId);

            // Consume after the repaint that drew it, so a frame with no fresh hover shows nothing.
            if (Event.current.type == EventType.Repaint)
                _hasPending = false;
        }

        private static void DrawTooltipWindow(int id)
        {
            if (_hasPending)
                GUI.Box(new Rect(0, 0, _pendingSize.x, _pendingSize.y), _pendingText, _tooltipStyle);
        }
    }
}
