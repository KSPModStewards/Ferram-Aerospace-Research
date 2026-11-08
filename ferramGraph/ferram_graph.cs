/* Name:    FerramGraph (Graph GUI Plugin)
 * Version: 1.3   (KSP 0.22+)
Copyright 2022, Michael Ferrara, aka Ferram4

    This file is part of FerramGraph.

    FerramGraph is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FerramGraph is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FerramGraph.  If not, see <http://www.gnu.org/licenses/>. *

 * Disclaimer: You use this at your own risk; this is an alpha plugin for an alpha game; if your computer disintegrates, it's not my fault. :P
 *
 *
 */

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using UnityEngine;
using Object = UnityEngine.Object;

namespace ferram4
{
    public class ferramGraph : IDisposable
    {
        protected readonly Texture2D graph;
        public readonly bool autoscale;
        protected Rect displayRect;

        private Dictionary<string, ferramGraphLine> allLines = new Dictionary<string, ferramGraphLine>();

        private Vector4d bounds;

        public Color backgroundColor = Color.black;
        public Color gridColor = new Color(0.2f, 0.2f, 0.2f);
        public Color axisColor = new Color(0.8f, 0.8f, 0.8f);

        private string leftBound;
        private string rightBound;
        private string topBound;
        private string bottomBound;
        public string horizontalLabel = "Axis Label Here";
        public string verticalLabel = "Axis Label Here";
        private Vector2 ScrollView = Vector2.zero;

        public ferramGraph(
            int width,
            int height,
            double minx = 0,
            double maxx = 1,
            double miny = 0,
            double maxy = 1,
            bool autoscale = false
        )
        {
            graph = new Texture2D(width, height, TextureFormat.ARGB32, false);
            this.autoscale = autoscale;
            SetBoundaries(minx, maxx, miny, maxy);
            displayRect = new Rect(1, 1, graph.width, graph.height);
            GridInit();
        }

        public void Dispose()
        {
            foreach (KeyValuePair<string, ferramGraphLine> pair in allLines)
                pair.Value.ClearTextures();
            allLines = null;
            Object.Destroy(graph);
        }

        public void SetBoundaries(double minx, double maxx, double miny, double maxy)
        {
            bounds.x = minx;
            bounds.y = maxx;
            bounds.z = miny;
            bounds.w = maxy;
            SetBoundaries(bounds);
        }

        public void SetBoundaries(Vector4d boundaries)
        {
            bounds = boundaries;
            leftBound = bounds.x.ToString(CultureInfo.InvariantCulture);
            rightBound = bounds.y.ToString(CultureInfo.InvariantCulture);
            topBound = bounds.w.ToString(CultureInfo.InvariantCulture);
            bottomBound = bounds.z.ToString(CultureInfo.InvariantCulture);
            foreach (KeyValuePair<string, ferramGraphLine> pair in allLines)
                pair.Value.SetBoundaries(bounds);
        }


        public void SetGridScaleUsingPixels(int gridWidth, int gridHeight)
        {
            GridInit(gridWidth, gridHeight);
            Update();
        }

        public void SetGridScaleUsingValues(double gridWidth, double gridHeight)
        {
            int pixelWidth = (int)Math.Round(gridWidth * displayRect.width / (bounds.y - bounds.x));
            int pixelHeight = (int)Math.Round(gridHeight * displayRect.height / (bounds.w - bounds.z));

            if (pixelWidth <= 1)
            {
                pixelWidth = 5;
                Debug.Log("[FAR] [ferramGraph] Warning! Grid width scale too fine for scaling; picking safe alternative");
            }

            if (pixelHeight <= 1)
            {
                pixelHeight = 5;
                Debug.Log("[FAR] [ferramGraph] Warning! Grid height scale too fine for scaling; picking safe alternative");
            }

            SetGridScaleUsingPixels(pixelWidth, pixelHeight);
        }

        // ReSharper disable once UnusedMember.Global
        public void SetLineVerticalScaling(string lineName, double scaling)
        {
            if (!allLines.ContainsKey(lineName))
            {
                MonoBehaviour.print("[FAR] [ferramGraph] Error: No line with that name exists");
                return;
            }

            if (!allLines.TryGetValue(lineName, out ferramGraphLine line))
                return;
            line.UpdateVerticalScaling(scaling);
        }


        // ReSharper disable once UnusedMember.Global
        public void SetLineHorizontalScaling(string lineName, double scaling)
        {
            if (!allLines.ContainsKey(lineName))
            {
                MonoBehaviour.print("[FAR] [ferramGraph] Error: No line with that name exists");
                return;
            }

            if (!allLines.TryGetValue(lineName, out ferramGraphLine line))
                return;
            line.UpdateHorizontalScaling(scaling);
        }

        private void GridInit()
        {
            const int squareSize = 25;
            GridInit(squareSize, squareSize);
        }


        private void GridInit(int widthSize, int heightSize)
        {
            int horizontalAxis = (int)Math.Round(-bounds.x * displayRect.width / (bounds.y - bounds.x));
            int verticalAxis = (int)Math.Round(-bounds.z * displayRect.height / (bounds.w - bounds.z));

            for (int i = 0; i < graph.width; i++)
            {
                for (int j = 0; j < graph.height; j++)
                    if (i - horizontalAxis == 0 || j - verticalAxis == 0)
                        graph.SetPixel(i, j, axisColor);
                    else if ((i - horizontalAxis) % widthSize == 0 || (j - verticalAxis) % heightSize == 0)
                        graph.SetPixel(i, j, gridColor);
                    else
                        graph.SetPixel(i, j, backgroundColor);
            }

            graph.Apply();
        }

        // ReSharper disable once UnusedMember.Global
        public void AddLine(string lineName)
        {
            if (allLines.ContainsKey(lineName))
            {
                MonoBehaviour.print("[FAR] [ferramGraph] Error: A Line with that name already exists");
                return;
            }

            var newLine = new ferramGraphLine((int)displayRect.width, (int)displayRect.height);
            newLine.SetBoundaries(bounds);
            allLines.Add(lineName, newLine);
            Update();
        }

        // ReSharper disable once UnusedMember.Global
        public void AddLine(string lineName, double[] xValues, double[] yValues)
        {
            const int lineThickness = 1;
            AddLine(lineName, xValues, yValues, lineThickness);
        }

        // ReSharper disable once UnusedMember.Global
        public void AddLine(string lineName, double[] xValues, double[] yValues, Color lineColor)
        {
            const int lineThickness = 1;
            AddLine(lineName, xValues, yValues, lineColor, lineThickness);
        }

        public void AddLine(string lineName, double[] xValues, double[] yValues, int lineThickness)
        {
            Color lineColor = Color.red;
            AddLine(lineName, xValues, yValues, lineColor, lineThickness);
        }

        public void AddLine(
            string lineName,
            double[] xValues,
            double[] yValues,
            Color lineColor,
            int lineThickness,
            bool display = true
        )
        {
            if (allLines.ContainsKey(lineName))
            {
                MonoBehaviour.print("[FAR] [ferramGraph] Error: A Line with that name already exists");
                return;
            }

            if (xValues.Length != yValues.Length)
            {
                MonoBehaviour.print("[FAR] [ferramGraph] Error: X and Y value arrays are different lengths");
                return;
            }

            var newLine = new ferramGraphLine((int)displayRect.width, (int)displayRect.height);
            newLine.InputData(xValues, yValues);
            newLine.SetBoundaries(bounds);
            newLine.lineColor = lineColor;
            newLine.lineThickness = lineThickness;
            newLine.backgroundColor = backgroundColor;
            newLine.displayInLegend = display;

            allLines.Add(lineName, newLine);
            Update();
        }

        // ReSharper disable once UnusedMember.Global
        public void RemoveLine(string lineName)
        {
            if (!allLines.ContainsKey(lineName))
            {
                MonoBehaviour.print("[FAR] [ferramGraph] Error: No line with that name exists");
                return;
            }

            ferramGraphLine line = allLines[lineName];
            allLines.Remove(lineName);

            line.ClearTextures();
            Update();
        }

        public void Clear()
        {
            foreach (KeyValuePair<string, ferramGraphLine> line in allLines)
                line.Value.ClearTextures();
            allLines.Clear();
            Update();
        }

        // ReSharper disable once UnusedMember.Global
        public void UpdateLineData(string lineName, double[] xValues, double[] yValues)
        {
            if (xValues.Length != yValues.Length)
            {
                MonoBehaviour.print("[FAR] [ferramGraph] Error: X and Y value arrays are different lengths");
                return;
            }

            if (allLines.TryGetValue(lineName, out ferramGraphLine line))
            {
                line.InputData(xValues, yValues);

                allLines.Remove(lineName);
                allLines.Add(lineName, line);
                Update();
            }
            else
            {
                MonoBehaviour.print("[FAR] [ferramGraph] Error: No line with this name exists");
            }
        }


        /// <summary>
        ///     Use this to update the graph display
        /// </summary>
        public void Update()
        {
            if (autoscale)
            {
                Vector4d extremes = Vector4.zero;
                bool init = false;
                foreach (KeyValuePair<string, ferramGraphLine> pair in allLines)
                {
                    Vector4d tmp = pair.Value.GetExtremeData();

                    if (!init)
                    {
                        extremes.x = tmp.x;
                        extremes.y = tmp.y;
                        extremes.z = tmp.z;
                        extremes.w = tmp.w;
                        init = true;
                    }
                    else
                    {
                        extremes.x = Math.Min(extremes.x, tmp.x);
                        extremes.y = Math.Max(extremes.y, tmp.y);
                        extremes.z = Math.Min(extremes.z, tmp.z);
                        extremes.w = Math.Max(extremes.w, tmp.w);
                    }

                    extremes.x = Math.Floor(extremes.x);
                    extremes.y = Math.Ceiling(extremes.y);
                    extremes.z = Math.Floor(extremes.z);
                    extremes.w = Math.Ceiling(extremes.w);
                }

                SetBoundaries(extremes);
            }

            foreach (KeyValuePair<string, ferramGraphLine> pair in allLines)
            {
                pair.Value.backgroundColor = backgroundColor;
                pair.Value.Update();
            }
        }

        // ReSharper disable once UnusedMember.Global
        public void LineColor(string lineName, Color newColor)
        {
            if (!allLines.TryGetValue(lineName, out ferramGraphLine line))
                return;
            line.lineColor = newColor;

            allLines.Remove(lineName);
            allLines.Add(lineName, line);
        }

        // ReSharper disable once UnusedMember.Global
        public void LineThickness(string lineName, int thickness)
        {
            if (!allLines.TryGetValue(lineName, out ferramGraphLine line))
                return;
            line.lineThickness = Mathf.Clamp(thickness, 1, 6);

            allLines.Remove(lineName);
            allLines.Add(lineName, line);
        }


        /// <summary>
        ///     This displays the graph
        /// </summary>
        public void Display(int horizontalBorder, int verticalBorder)
        {
            ScrollView = GUILayout.BeginScrollView(ScrollView, false, false);
            var BackgroundStyle = new GUIStyle(GUI.skin.box);
            BackgroundStyle.hover = BackgroundStyle.active = BackgroundStyle.normal;

            GUILayout.Space(verticalBorder);
            //Vertical axis and labels

            GUILayout.BeginVertical();
            GUILayout.BeginArea(new Rect(20 + horizontalBorder,
                                         15 + verticalBorder,
                                         30,
                                         displayRect.height + 2 * verticalBorder));

            var LabelStyle = new GUIStyle(GUI.skin.label) {alignment = TextAnchor.UpperCenter};

            GUILayout.Label(topBound, LabelStyle, GUILayout.Height(20), GUILayout.ExpandWidth(true));
            int pixelspace = (int)displayRect.height / 2 - 72;
            GUILayout.Space(pixelspace);
            GUILayout.Label(verticalLabel, LabelStyle, GUILayout.Height(100), GUILayout.ExpandWidth(true));
            GUILayout.Space(pixelspace);
            GUILayout.Label(bottomBound, LabelStyle, GUILayout.Height(20), GUILayout.ExpandWidth(true));

            GUILayout.EndArea();
            GUILayout.EndVertical();


            //Graph itself

            GUILayout.BeginVertical();
            var areaRect = new Rect(50 + horizontalBorder,
                                    15 + verticalBorder,
                                    displayRect.width + 2 * horizontalBorder,
                                    displayRect.height + 2 * verticalBorder);
            GUILayout.BeginArea(areaRect);

            GUI.DrawTexture(displayRect, graph);
            foreach (KeyValuePair<string, ferramGraphLine> pair in allLines)
                GUI.DrawTexture(displayRect, pair.Value.Line());
            GUILayout.EndArea();

            //Horizontal Axis and Labels

            GUILayout.BeginArea(new Rect(50 + horizontalBorder,
                                         displayRect.height + verticalBorder + 15,
                                         displayRect.width + 2 * horizontalBorder,
                                         30));
            GUILayout.BeginHorizontal(GUILayout.Width(displayRect.width));


            GUILayout.Label(leftBound, LabelStyle, GUILayout.Width(20), GUILayout.ExpandWidth(true));
            pixelspace = (int)displayRect.width / 2 - 102;
            GUILayout.Space(pixelspace);
            GUILayout.Label(horizontalLabel, LabelStyle, GUILayout.Width(160));
            GUILayout.Space(pixelspace);
            GUILayout.Label(rightBound, LabelStyle, GUILayout.Width(20), GUILayout.ExpandWidth(true));

            GUILayout.EndHorizontal();
            GUILayout.EndArea();
            GUILayout.EndVertical();

            GUILayout.BeginVertical();

            //Legend Area

            int movementdownwards = ((int)displayRect.height - allLines.Count * 20) / 2;
            foreach (KeyValuePair<string, ferramGraphLine> pair in allLines)
            {
                if (!pair.Value.displayInLegend)
                    continue;

                GUILayout.BeginArea(new Rect(60 + displayRect.width + 2 * horizontalBorder,
                                             15 + verticalBorder + movementdownwards,
                                             25,
                                             15));
                GUI.DrawTexture(new Rect(1, 1, 25, 15), pair.Value.LegendImage());
                GUILayout.EndArea();
                GUILayout.BeginArea(new Rect(85 + displayRect.width + 2 * horizontalBorder,
                                             15 + verticalBorder + movementdownwards,
                                             35,
                                             15));
                GUILayout.Label(pair.Key, LabelStyle);
                GUILayout.EndArea();
                movementdownwards += 20;
            }

            GUILayout.EndVertical();

            int bottomofarea = (int)displayRect.height + 2 * verticalBorder + 30;

            GUILayout.Space(bottomofarea);
            GUILayout.EndScrollView();
        }

        private class ferramGraphLine
        {
            private Texture2D lineDisplay;
            private Texture2D lineLegend;
            public bool displayInLegend;
            private double[] rawDataX = new double[1];
            private double[] rawDataY = new double[1];
            private double[] pixelDataX = new double[1];
            private double[] pixelDataY = new double[1];
            private Vector4d bounds;
            public int lineThickness;
            public Color lineColor;
            public Color backgroundColor;
            private double verticalScaling;
            private double horizontalScaling;

            public ferramGraphLine(int width, int height)
            {
                lineDisplay = new Texture2D(width, height, TextureFormat.ARGB32, false);
                SetBoundaries(new Vector4(0, 1, 0, 1));
                lineThickness = 1;
                lineColor = Color.red;
                verticalScaling = 1;
                horizontalScaling = 1;
            }

            public void InputData(double[] xValues, double[] yValues)
            {
                rawDataX = xValues.replaceNaNs(0, "xValues");
                rawDataY = yValues.replaceNaNs(0, "yValues");
                ConvertRawToPixels(false);
            }

            private void ConvertRawToPixels(bool update = true)
            {
                double xScaling = lineDisplay.width / (bounds.y - bounds.x);
                double yScaling = lineDisplay.height / (bounds.w - bounds.z);
                pixelDataX = rawDataX.toPixelsF(bounds.x, horizontalScaling, xScaling);
                pixelDataY = rawDataY.toPixelsF(bounds.z, verticalScaling, yScaling);
                if (update)
                    Update();
            }

            public void SetBoundaries(Vector4 boundaries)
            {
                bounds = boundaries;
                if (rawDataX.Length > 0)
                    ConvertRawToPixels();
            }

            public void Update()
            {
                ClearLine();
                if (lineThickness < 1)
                    lineThickness = 1;
                int prev = 0;

                for (int k = 1; k < pixelDataX.Length; k++)
                {
                    lineDisplay.DrawLineAAF(pixelDataX[prev],
                                            pixelDataY[prev],
                                            pixelDataX[k],
                                            pixelDataY[k],
                                            lineColor,
                                            lineThickness);
                    prev = k;
                }

                lineDisplay.Apply();
                UpdateLineLegend();
            }

            private void UpdateLineLegend()
            {
                lineLegend = new Texture2D(25, 15, TextureFormat.ARGB32, false);
                for (int i = 0; i < lineLegend.width; i++)
                    for (int j = 0; j < lineLegend.height; j++)
                        lineLegend.SetPixel(i,
                                            j,
                                            Mathf.Abs((int)(j - lineLegend.height / 2f)) < lineThickness
                                                ? lineColor
                                                : backgroundColor);
                lineLegend.Apply();
            }

            private void ClearLine()
            {
                lineDisplay.Clear(ferramDrawingExtensions.EmptyColor);
                lineDisplay.Apply();
            }

            /// <summary>
            ///     XMin, XMax, YMin, YMax
            /// </summary>
            public Vector4d GetExtremeData()
            {
                Vector4d extremes = Vector4d.zero;
                extremes.x = rawDataX.Min();
                extremes.y = rawDataX.Max();
                extremes.z = rawDataY.Min();
                extremes.w = rawDataY.Max();

                return extremes;
            }

            public Texture2D Line()
            {
                return lineDisplay;
            }

            public Texture2D LegendImage()
            {
                return lineLegend;
            }

            public void UpdateVerticalScaling(double scaling)
            {
                verticalScaling = scaling;
                ConvertRawToPixels();
            }

            public void UpdateHorizontalScaling(double scaling)
            {
                horizontalScaling = scaling;
                ConvertRawToPixels();
            }

            public void ClearTextures()
            {
                Object.Destroy(lineLegend);
                Object.Destroy(lineDisplay);
                lineDisplay = null;
                lineLegend = null;
            }
        }
    }
}
