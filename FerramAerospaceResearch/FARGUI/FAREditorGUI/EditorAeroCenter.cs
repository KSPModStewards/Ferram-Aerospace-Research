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
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI
{
    internal class EditorAeroCenter
    {
        private Vector3 vesselRootLocalAeroCenter;

        private List<FARAeroSection> _currentAeroSections;

        public EditorAeroCenter()
        {
            Instance = this;
        }

        public static EditorAeroCenter Instance { get; private set; }

        public static Vector3 VesselRootLocalAeroCenter
        {
            get { return Instance.vesselRootLocalAeroCenter; }
        }

        public void UpdateAeroData(List<FARAeroSection> aeroSections)
        {
            _currentAeroSections = aeroSections;
            UpdateAerodynamicCenter();
        }

        private void UpdateAerodynamicCenter()
        {
            var aeroSection = new FARCenterQuery();
            var dummy = new FARCenterQuery();

            if (EditorLogic.RootPart is null)
                return;

            Vector3 vel_base, vel_fuzz;

            Transform rootPartTrans = EditorLogic.RootPart.partTransform;
            if (EditorDriver.editorFacility == EditorFacility.SPH)
            {
                vel_base = Vector3.forward;
                vel_fuzz = 0.02f * Vector3.up;
            }
            else
            {
                vel_base = Vector3.up;
                vel_fuzz = -0.02f * Vector3.forward;
            }

            Vector3 vel = (vel_base - vel_fuzz).normalized;

            foreach (FARAeroSection section in _currentAeroSections)
                section.PredictionCalculateAeroForces(1, 0.5f, 100000, 0, 0.005f, vel, aeroSection);

            FARBaseAerodynamics.PrecomputeGlobalCenterOfLift(aeroSection, dummy, vel, 1);

            Vector3 pos = Vector3.zero;
            float mass = 0;
            foreach (Part p in EditorLogic.SortedShipList)
            {
                if (FARAeroUtil.IsNonphysical(p))
                    continue;
                float tmpMass = p.mass + p.GetResourceMass();
                mass += tmpMass;
                pos += p.partTransform.position * tmpMass;
            }

            pos /= mass;

            Vector3 force0 = aeroSection.force;
            Vector3 moment0 = aeroSection.TorqueAt(pos);

            aeroSection.ClearAll();

            vel = (vel_base + vel_fuzz).normalized;

            foreach (FARAeroSection section in _currentAeroSections)
                section.PredictionCalculateAeroForces(1, 0.5f, 100000, 0, 0.005f, vel, aeroSection);

            FARBaseAerodynamics.PrecomputeGlobalCenterOfLift(aeroSection, dummy, vel, 1);

            Vector3 force1 = aeroSection.force;
            Vector3 moment1 = aeroSection.TorqueAt(pos);

            aeroSection.ClearAll();

            Vector3 deltaForce = force1 - force0;
            Vector3 deltaMoment = moment1 - moment0;

            Vector3 deltaForcePerp = Vector3.ProjectOnPlane(deltaForce, vel_base);
            float deltaForcePerpMag = deltaForcePerp.magnitude;

            Vector3 deltaForcePerpNorm = deltaForcePerp / deltaForcePerpMag;

            Vector3 deltaMomentPerp = deltaMoment -
                                      Vector3.Dot(deltaMoment, deltaForcePerpNorm) * deltaForcePerpNorm -
                                      Vector3.Project(deltaMoment, vel_base);

            vesselRootLocalAeroCenter = deltaMomentPerp.magnitude /
                                        deltaForcePerpMag *
                                        Math.Sign(Vector3.Dot(Vector3.Cross(deltaForce, deltaMoment), vel_base)) *
                                        vel_base;

            vesselRootLocalAeroCenter += pos;
            vesselRootLocalAeroCenter = rootPartTrans.worldToLocalMatrix.MultiplyPoint3x4(vesselRootLocalAeroCenter);
        }
    }
}
