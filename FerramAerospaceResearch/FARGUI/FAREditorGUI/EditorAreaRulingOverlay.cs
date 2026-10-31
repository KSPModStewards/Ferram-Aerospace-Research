/*
Ferram Aerospace Research v0.16.2.0 "Maskell"
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
using FerramAerospaceResearch.Resources;
using UnityEngine;
using Object = UnityEngine.Object;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI
{
    internal class EditorAreaRulingOverlay
    {
        public enum OverlayType
        {
            AREA,
            DERIV,
            COEFF
        }

        private LineRenderer _areaRenderer;
        private LineRenderer _derivRenderer;
        private LineRenderer _coeffRenderer;
        private List<LineRenderer> _markingRenderers;

        private Color _axisColor;
        private Color _crossSectionColor;
        private Color _derivColor;
        private double _yScaleMaxDistance;
        private double _yAxisGridScale;
        private int _numGridLines;

        private Material _rendererMaterial;

        private EditorAreaRulingOverlay()
        {
        }

        public static EditorAreaRulingOverlay CreateNewAreaRulingOverlay(
            Color axisColor,
            Color crossSectionColor,
            Color derivColor,
            double yScaleMaxDistance,
            double yAxisGridScale
        )
        {
            var overlay = new EditorAreaRulingOverlay
            {
                _axisColor = axisColor,
                _crossSectionColor = crossSectionColor,
                _derivColor = derivColor,
                _yScaleMaxDistance = yScaleMaxDistance,
                _yAxisGridScale = yAxisGridScale
            };


            overlay.Initialize();

            return overlay;
        }

        private void Initialize()
        {
            //Based on Kronal Vessel Viewer CoM axes rendering
            if (_rendererMaterial == null)
            {
                Material lineMaterial = FARAssets.Instance.Shaders.LineRenderer;

                _rendererMaterial = new Material(lineMaterial)
                {
                    hideFlags = HideFlags.HideAndDontSave,
                    shader = {hideFlags = HideFlags.HideAndDontSave},
                    renderQueue = 4500
                };
            }

            FARLogger.Debug("Creating renderers with material " + _rendererMaterial);
            FARLogger.Debug("Area color: " +
                            _crossSectionColor +
                            ", Deriv: " +
                            _derivColor +
                            ", Coeff: " +
                            Color.cyan +
                            ", Marking: " +
                            _axisColor);
            _areaRenderer = CreateNewRenderer(_crossSectionColor, 0.1f, _rendererMaterial);
            _derivRenderer = CreateNewRenderer(_derivColor, 0.1f, _rendererMaterial);
            _coeffRenderer = CreateNewRenderer(Color.cyan, 0.1f, _rendererMaterial);

            _markingRenderers = new List<LineRenderer> {CreateNewRenderer(_axisColor, 0.1f, _rendererMaterial)};
        }

        public void Cleanup()
        {
            if (_areaRenderer)
                Object.Destroy(_areaRenderer.gameObject);
            if (_derivRenderer)
                Object.Destroy(_derivRenderer.gameObject);
            if (_coeffRenderer)
                Object.Destroy(_coeffRenderer.gameObject);
            if (_markingRenderers != null)
                foreach (LineRenderer renderer in _markingRenderers)
                    if (renderer)
                        Object.Destroy(renderer.gameObject);

            _markingRenderers = null;

            Object.Destroy(_rendererMaterial);
            _rendererMaterial = null;
        }

        public void RestartOverlay()
        {
            Cleanup();
            Initialize();
        }

        private static LineRenderer CreateNewRenderer(Color color, float width, Material material)
        {
            var o = new GameObject();

            LineRenderer renderer = o.gameObject.AddComponent<LineRenderer>();

            // need to copy the material properties so the same material is not
            // reused between renderers
            var rendererMaterial = new Material(material);

            renderer.useWorldSpace = false;
            if (material.HasProperty(ShaderPropertyIds._Color))
                rendererMaterial.SetColor(ShaderPropertyIds._Color, color);
            else
                FARLogger.Warning("Material " + material + " has no _Color property");
            renderer.material = rendererMaterial;
            renderer.enabled = false;
            renderer.startWidth = width;
            renderer.endWidth = width;
            renderer.sortingOrder = 1;

            return renderer;
        }

        public bool IsVisible(OverlayType type)
        {
            return type switch
            {
                OverlayType.AREA => (_areaRenderer != null && _areaRenderer.enabled),
                OverlayType.DERIV => (_derivRenderer != null && _derivRenderer.enabled),
                OverlayType.COEFF => (_coeffRenderer != null && _coeffRenderer.enabled),
                _ => false
            };
        }

        public bool AnyVisible()
        {
            return _areaRenderer && (_areaRenderer.enabled || _derivRenderer.enabled || _coeffRenderer.enabled);
        }

        public void SetVisibility(OverlayType type, bool visible)
        {
            if (!_areaRenderer)
                RestartOverlay();

            switch (type)
            {
                case OverlayType.AREA:
                    _areaRenderer.enabled = visible;
                    break;
                case OverlayType.DERIV:
                    _derivRenderer.enabled = visible;
                    break;
                case OverlayType.COEFF:
                    _coeffRenderer.enabled = visible;
                    break;
            }

            bool anyVisible = AnyVisible();
            for (int i = 0; i < _markingRenderers.Count; i++)
            {
                _markingRenderers[i].enabled = anyVisible;
                if (i > _numGridLines)
                    _markingRenderers[i].enabled = false;
            }
        }

        public void UpdateAeroData(
            Matrix4x4 voxelLocalToWorldMatrix,
            double[] xCoords,
            double[] yCoordsCrossSection,
            double[] yCoordsDeriv,
            double[] yCoordsPressureCoeffs,
            double maxValue
        )
        {
            _numGridLines = (int)Math.Ceiling(maxValue / _yAxisGridScale); //add one to account for the xAxis
            double gridScale = _yScaleMaxDistance / _numGridLines;
            double scalingFactor = _yScaleMaxDistance / (_yAxisGridScale * _numGridLines);

            if (!_areaRenderer)
                RestartOverlay();

            UpdateRenderer(_areaRenderer, voxelLocalToWorldMatrix, xCoords, yCoordsCrossSection, scalingFactor);
            UpdateRenderer(_derivRenderer, voxelLocalToWorldMatrix, xCoords, yCoordsDeriv, scalingFactor);
            UpdateRenderer(_coeffRenderer, voxelLocalToWorldMatrix, xCoords, yCoordsPressureCoeffs, 10);

            while (_markingRenderers.Count <= _numGridLines)
            {
                LineRenderer newMarkingRenderer = CreateNewRenderer(_axisColor, 0.1f, _rendererMaterial);
                newMarkingRenderer.enabled = _areaRenderer.enabled;
                _markingRenderers.Add(newMarkingRenderer);
            }


            double[] shortXCoords = {xCoords[0], xCoords[xCoords.Length - 1]};

            for (int i = 0; i < _markingRenderers.Count; i++)
            {
                double height = i * gridScale;
                UpdateRenderer(_markingRenderers[i], voxelLocalToWorldMatrix, shortXCoords, new[] {height, height});
                _markingRenderers[i].enabled = i <= _numGridLines && _areaRenderer.enabled;
            }
        }

        private static void UpdateRenderer(
            LineRenderer renderer,
            Matrix4x4 transformMatrix,
            double[] xCoords,
            double[] yCoords,
            double yScalingFactor = 1
        )
        {
            // getting transform is internal call, cache
            Transform transform = renderer.transform;
            transform.parent = EditorLogic.RootPart.partTransform;
            transform.localPosition = Vector3.zero;
            transform.localRotation = Quaternion.identity;
            transform.SetAsFirstSibling();

            renderer.positionCount = xCoords.Length;

            for (int i = 0; i < xCoords.Length; i++)
            {
                Vector3 vec = Vector3.up * (float)xCoords[i] -
                              Vector3.forward * (float)(yCoords[i] * yScalingFactor);
                vec = transformMatrix.MultiplyVector(vec);
                renderer.SetPosition(i, vec);
            }
        }
    }
}
