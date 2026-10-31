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

using System.Collections.Generic;
using FerramAerospaceResearch.FARThreading;
using ProceduralFairings;
using UnityEngine;

namespace FerramAerospaceResearch.FARPartGeometry.GeometryModification
{
    internal class StockProcFairingGeoUpdater : IGeometryUpdater
    {
        private static Dictionary<Part, GeometryPartModule> validParts;
        private readonly ModuleProceduralFairing fairing;
        private readonly GeometryPartModule geoModule;

        private readonly List<Bounds> prevPanelBounds;

        public StockProcFairingGeoUpdater(ModuleProceduralFairing fairing, GeometryPartModule geoModule)
        {
            this.fairing = fairing;
            this.geoModule = geoModule;

            if (validParts == null)
            {
                FARLogger.Info("Fairing event setup");
                validParts = new Dictionary<Part, GeometryPartModule>();
                GameEvents.onFairingsDeployed.Add(FairingDeployGeometryUpdate);
            }

            validParts.Add(geoModule.part, geoModule);

            if (HighLogic.LoadedSceneIsEditor)
                prevPanelBounds = new List<Bounds>();
        }

        public void EditorGeometryUpdate()
        {
            List<FairingPanel> panels = fairing.Panels;
            if (panels == null)
                return;

            //if bounds count doesn't equal panels count, the number of panels changed
            bool rebuildMesh = prevPanelBounds.Count != panels.Count;

            if (rebuildMesh)
                prevPanelBounds.Clear();

            //set them back to where they started to prevent voxelization errors
            for (int i = 0; i < panels.Count; i++)
            {
                FairingPanel p = panels[i];
                var panelBounds = new Bounds();
                if (p != null)
                {
                    p.SetExplodedView(0);
                    p.SetOpacity(1);
                    p.SetTgtExplodedView(0);
                    p.SetTgtOpacity(1);
                    if (p.ColliderContainer)
                        p.ColliderContainer.SetActive(true);

                    panelBounds = p.GetBounds();
                }

                if (i >= prevPanelBounds.Count) //set new panel bounds
                {
                    rebuildMesh = true;
                    prevPanelBounds.Add(panelBounds);
                }
                else if (panelBounds != prevPanelBounds[i])
                {
                    rebuildMesh = true;
                    prevPanelBounds[i] = panelBounds;
                }
            }

            if (rebuildMesh)
                geoModule.RebuildAllMeshData();
        }

        public void FlightGeometryUpdate()
        {
        } //use the fairing events instead

        // ReSharper disable once MemberCanBeMadeStatic.Local
        private void FairingDeployGeometryUpdate(Part p)
        {
            ThreadSafeDebugLogger.Info("Fairing Geometry Update");
            validParts[p].GeometryPartModuleRebuildMeshData();
        }
    }
}
