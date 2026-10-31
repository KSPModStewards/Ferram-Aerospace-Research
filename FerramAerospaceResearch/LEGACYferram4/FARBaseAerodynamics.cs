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
using FerramAerospaceResearch;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using KSPCommunityFixes;
using UnityEngine;

// ReSharper disable once CheckNamespace
namespace ferram4
{
    public class FARBaseAerodynamics : FARPartModule, ILiftProvider
    {
        [KSPField(isPersistant = false, guiActive = false, guiName = "FARAbbrevCl")]
        public double Cl;

        [KSPField(isPersistant = false, guiActive = false, guiName = "FARAbbrevCd")]
        public double Cd;

        // ReSharper disable once NotAccessedField.Global
        [KSPField(isPersistant = false, guiActive = false, guiName = "FARAbbrevCm")]
        public double Cm;

        protected Vector3d velocityEditor = Vector3.zero;

        protected Transform part_transform;

        [KSPField(isPersistant = false, guiActive = false)]
        public double S;

        [KSPField(isPersistant = false, guiActive = false, guiActiveEditor = true)]
        public bool isShielded = true;

        public double rho;

        // TODO 1.2: provide actual implementation of these new methods

        public bool DisableBodyLift
        {
            get { return false; }
        }

        public bool IsLifting
        {
            get { return true; }
        }

        public void OnCenterOfLiftQuery(CenterOfLiftQuery CoLMarker)
        {
            // Compute the actual center ourselves once per frame
            // Feed the precomputed values to the vanilla indicator
            //hacking the old stuff to work with the new
            CoLMarker.pos = EditorAeroCenter.VesselRootLocalAeroCenter;
            CoLMarker.pos = EditorLogic.RootPart.partTransform.localToWorldMatrix.MultiplyPoint3x4(CoLMarker.pos);
            CoLMarker.dir = Vector3.zero;
            CoLMarker.lift = 1;
        }

        public override void OnAwake()
        {
            base.OnAwake();
            part_transform = part.partTransform;

            if (part.partTransform == null && part == part.vessel.rootPart)
                part_transform = vessel.vesselTransform;
            if (HighLogic.LoadedSceneIsEditor)
                part_transform = part.partTransform;
        }

        public Vector3d GetVelocity()
        {
            if (HighLogic.LoadedSceneIsFlight)
                return part.Rigidbody.velocity +
                       Krakensbane.GetFrameVelocityV3f() -
                       FARAtmosphere.GetWind(FlightGlobals.currentMainBody, part, part.Rigidbody.position);
            return velocityEditor;
        }

        // ReSharper disable once UnusedMember.Global
        public Vector3d GetVelocity(Vector3 refPoint)
        {
            Vector3d velocity = Vector3.zero;
            if (!HighLogic.LoadedSceneIsFlight)
                return velocityEditor;
            if (part.Rigidbody)
                velocity += part.Rigidbody.GetPointVelocity(refPoint);

            velocity += Krakensbane.GetFrameVelocity() - Krakensbane.GetLastCorrection() * TimeWarp.fixedDeltaTime;
            velocity -= FARAtmosphere.GetWind(FlightGlobals.currentMainBody, part, part.Rigidbody.position);

            return velocity;
        }

        protected virtual void ResetCenterOfLift()
        {
            // Clear state when preparing CoL computation
        }

        public virtual Vector3d PrecomputeCenterOfLift(
            Vector3d velocity,
            double MachNumber,
            double density,
            FARCenterQuery center
        )
        {
            return Vector3d.zero;
        }

        public static List<FARBaseAerodynamics> GetAllEditorModules()
        {
            var parts = new List<FARBaseAerodynamics>();

            foreach (Part p in EditorLogic.SortedShipList)
            {
                var modules = p.FindModulesImplementingReadOnly<FARBaseAerodynamics>();
                foreach (FARBaseAerodynamics m in modules)
                    parts.Add(m);
            }

            return parts;
        }

        public static void PrecomputeGlobalCenterOfLift(
            FARCenterQuery lift,
            FARCenterQuery dummy,
            Vector3 vel,
            double density
        )
        {
            /* Center of lift is the location where the derivative of
               the total torque provided by aerodynamic forces relative to
               AoA is zero (or at least minimal). This approximates the
               derivative by a simple subtraction, like before. */

            List<FARBaseAerodynamics> parts = GetAllEditorModules();

            foreach (FARBaseAerodynamics ba in parts)
            {
                ba.velocityEditor = vel;
                ba.ResetCenterOfLift();
            }

            // run computations twice to let things like flap interactions settle
            foreach (FARBaseAerodynamics ba in parts)
                ba.PrecomputeCenterOfLift(vel, 0.5, density, dummy);
            foreach (FARBaseAerodynamics ba in parts)
                ba.PrecomputeCenterOfLift(vel, 0.5, density, lift);
        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            if (node.HasValue("S"))
                double.TryParse(node.GetValue("S"), out S);
        }
    }
}
