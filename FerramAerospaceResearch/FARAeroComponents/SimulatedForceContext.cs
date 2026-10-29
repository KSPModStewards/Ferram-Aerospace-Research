/*
Ferram Aerospace Research v0.16.1.2 "Marangoni"
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
            // A snapshot prediction reads the parts captured with the snapshot; a part that was not captured is
            // left out (zero velocity is skipped), since reading it live is not allowed off the main thread.
            if (pd.aeroModule.TryGetSnapshotState(out FARAeroPartModule.SnapshotState captured))
                return captured.Geom.InverseTransformVector(worldVel);
            if (SimThreadContext.SnapshotStamp != 0)
                return Vector3.zero;
            ferram4.FrozenPartTransform? sweepGeom = pd.aeroModule.SweepGeom;
            if (sweepGeom.HasValue)
                return sweepGeom.Value.InverseTransformVector(worldVel);
            // A sweep worker never reads a live part (Unity is main-thread-only, its == null included): a part the
            // sweep did not freeze is left out, as a snapshot leaves out a part it did not capture.
            if (SimThreadContext.InSweep)
                return Vector3.zero;
            if (pd.aeroModule.part == null || pd.aeroModule.part.partTransform == null)
                return Vector3.zero;
            return pd.aeroModule.part.partTransform.InverseTransformVector(worldVel);
        }

        public void ApplyForce(FARAeroSection.PartData pd, Vector3 localVel, Vector3 forceVector, Vector3 torqueVector)
        {
            double tmp = 0.0005 * Vector3.SqrMagnitude(localVel);
            double dynamicPressurekPa = tmp * atmDensity;

            // PhysicsGlobals.DragCurvePseudoReynolds through its exact, thread-safe copy (the Unity curve keeps a
            // segment cache and is not safe to evaluate concurrently).
            float pseudoReynolds = atmDensity * Vector3.Magnitude(localVel);
            float dragPseudo = FARAeroUtil.DragPseudoReynolds.Evaluate(pseudoReynolds);
            double dragFactor = dynamicPressurekPa * Mathf.Max(dragPseudo, 1.0f);
            double liftFactor = dynamicPressurekPa;

            Vector3 localVelNorm = Vector3.Normalize(localVel);
            Vector3 localForceTemp = Vector3.Dot(localVelNorm, forceVector) * localVelNorm;
            Vector3 partLocalForce =
                localForceTemp * (float)dragFactor + (forceVector - localForceTemp) * (float)liftFactor;

            Vector3 centroid;
            bool snapshot = pd.aeroModule.TryGetSnapshotState(out FARAeroPartModule.SnapshotState captured);
            ferram4.FrozenPartTransform? sweepGeom = snapshot ? null : pd.aeroModule.SweepGeom;
            if (snapshot || sweepGeom.HasValue)
            {
                ferram4.FrozenPartTransform g = snapshot ? captured.Geom : sweepGeom.Value;
                Vector3 comOffset = snapshot ? captured.CoMOffset : pd.aeroModule.SimCoMOffset;
                forceVector = g.TransformDirection(partLocalForce);
                torqueVector = g.TransformDirection(torqueVector * (float)dynamicPressurekPa);
                if (float.IsNaN(forceVector.x) || float.IsNaN(torqueVector.x))
                    return;
                centroid = g.TransformPoint(pd.centroidPartSpace - comOffset);
                center.AddForce(centroid, forceVector);
                center.AddTorque(torqueVector);
                return;
            }

            forceVector = pd.aeroModule.part.transform.TransformDirection(partLocalForce);
            torqueVector = pd.aeroModule.part.transform.TransformDirection(torqueVector * (float)dynamicPressurekPa);
            if (float.IsNaN(forceVector.x) || float.IsNaN(torqueVector.x))
                return;
            centroid =
                pd.aeroModule.part.transform.TransformPoint(pd.centroidPartSpace - pd.aeroModule.part.CoMOffset);
            center.AddForce(centroid, forceVector);
            center.AddTorque(torqueVector);
        }
    }
}
