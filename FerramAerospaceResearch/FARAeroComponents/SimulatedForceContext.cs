/*
Ferram Aerospace Research v0.16.2.0 "Maskell"
=========================
Copyright 2022, Benjamin Chung, aka BenChung

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
   along with Ferram Aerospace Research.  If not, see <http: //www.gnu.org/licenses/>.

   Serious thanks:        a.g., for tons of bugfixes and code-refactorings
                stupid_chris, for the RealChuteLite implementation
                        Taverius, for correcting a ton of incorrect values
                Tetryds, for finding lots of bugs and issues and not letting me get away with them, and work on example crafts
                        sarbian, for refactoring code for working with MechJeb, and the Module Manager updates
                        ialdabaoth (who is awesome), who originally created Module Manager
                            Regex, for adding RPM support
                DaMichel, for some ferramGraph updates and some control surface-related features
                        Duxwing, for copy editing the readme

   CompatibilityChecker by Majiir, BSD 2-clause http: //opensource.org/licenses/BSD-2-Clause

   Part.cfg changes powered by sarbian & ialdabaoth's ModuleManager plugin; used with permission
    http: //forum.kerbalspaceprogram.com/threads/55219

   ModularFLightIntegrator by Sarbian, Starwaster and Ferram4, MIT: http: //opensource.org/licenses/MIT
    http: //forum.kerbalspaceprogram.com/threads/118088

   Toolbar integration powered by blizzy78's Toolbar plugin; used with permission
    http: //forum.kerbalspaceprogram.com/threads/60863
 */

using System.Collections.Generic;
using ferram4;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    internal class SimulatedForceContext : FARAeroSection.IForceContext
    {
        /// <summary>
        ///     The world-space velocity of the part whose force is being simulated
        /// </summary>
        private Vector3 worldVel;

        /// <summary>
        ///     The center with which force should be accumulated
        /// </summary>
        private FARCenterQuery center;

        /// <summary>
        ///     The atmospheric density that the force is being simulated at
        /// </summary>
        private float atmDensity;

        public SimulatedForceContext(Vector3 worldVel, FARCenterQuery center, float atmDensity)
        {
            this.worldVel = worldVel;
            this.center = center;
            this.atmDensity = atmDensity;
        }

        // ReSharper disable ParameterHidesMember -> updating member values
        public void UpdateSimulationContext(Vector4 worldVel, FARCenterQuery center, float atmDensity)
        {
            this.worldVel = worldVel;
            this.center = center;
            this.atmDensity = atmDensity;
            // ReSharper restore ParameterHidesMember
        }

        public Vector3 LocalVelocity(FARAeroSection.PartData pd)
        {
            if (pd.aeroModule.part == null || pd.aeroModule.part.partTransform == null)
                return Vector3.zero;
            return pd.aeroModule.part.partTransform.InverseTransformVector(worldVel);
        }

        public void ApplyForce(FARAeroSection.PartData pd, Vector3 localVel, Vector3 forceVector, Vector3 torqueVector)
        {
            double tmp = 0.0005 * Vector3.SqrMagnitude(localVel);
            double dynamicPressurekPa = tmp * atmDensity;
            double dragFactor = dynamicPressurekPa *
                                Mathf.Max(PhysicsGlobals.DragCurvePseudoReynolds.Evaluate(atmDensity *
                                                                                          Vector3.Magnitude(localVel)),
                                          1.0f);
            double liftFactor = dynamicPressurekPa;

            Vector3 localVelNorm = Vector3.Normalize(localVel);
            Vector3 localForceTemp = Vector3.Dot(localVelNorm, forceVector) * localVelNorm;
            Vector3 partLocalForce =
                localForceTemp * (float)dragFactor + (forceVector - localForceTemp) * (float)liftFactor;
            forceVector = pd.aeroModule.part.transform.TransformDirection(partLocalForce);
            torqueVector = pd.aeroModule.part.transform.TransformDirection(torqueVector * (float)dynamicPressurekPa);
            if (float.IsNaN(forceVector.x) || float.IsNaN(torqueVector.x))
                return;
            Vector3 centroid =
                pd.aeroModule.part.transform.TransformPoint(pd.centroidPartSpace - pd.aeroModule.part.CoMOffset);
            center.AddForce(centroid, forceVector);
            center.AddTorque(torqueVector);
            if (PerPart == null)
                return;
            if (!PerPart.TryGetValue(pd.aeroModule.part, out FARCenterQuery own))
                PerPart[pd.aeroModule.part] = own = new FARCenterQuery();
            own.AddForce(centroid, forceVector);
            own.AddTorque(torqueVector);
        }

        /// <summary>
        ///     When set, each part's share of the forces is also gathered here (the editor's Components tab)
        /// </summary>
        public Dictionary<Part, FARCenterQuery> PerPart;
    }
}
