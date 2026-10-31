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


using UnityEngine;

namespace FerramAerospaceResearch.FARPartGeometry.GeometryModification
{
    internal class IntegratedIntakeEngineCrossSectionAdjuster : ICrossSectionAdjuster
    {
        private const double INTAKE_AREA_SCALAR = 100;

        private Vector3 vehicleBasisForwardVector;
        private double intakeArea;
        private int sign = 1;

        private Matrix4x4 thisToVesselMatrix;
        private Matrix4x4 meshLocalToWorld;
        private ModuleResourceIntake intakeModule;
        private Transform intakeTrans;

        private Part part;

        private IntegratedIntakeEngineCrossSectionAdjuster()
        {
        }

        public Part GetPart()
        {
            return part;
        }

        public bool IntegratedCrossSectionIncreaseDecrease()
        {
            return intakeModule.node == null || intakeModule.node.attachedPart == null;
        }

        public double AreaRemovedFromCrossSection(Vector3 vehicleAxis)
        {
            double dot = Vector3.Dot(vehicleAxis, vehicleBasisForwardVector);
            if (dot > 0.9)
                return intakeArea;
            return 0;
        }

        public double AreaRemovedFromCrossSection()
        {
            if (intakeModule.node == null || intakeModule.node.attachedPart == null)
                return intakeArea * sign;
            //if the intake is covered, switch the math so that it functions like an AirbreathingEngineCrossSectionAdjuster instead
            return -intakeArea * sign;
        }

        public double AreaThreshold()
        {
            return 0;
        }

        public void SetForwardBackwardNoFlowDirection(int direction)
        {
            sign = direction;
        }

        public int GetForwardBackwardNoFlowSign()
        {
            return sign;
        }

        public void TransformBasis(Matrix4x4 matrix)
        {
            Matrix4x4 tempMatrix = thisToVesselMatrix.inverse;
            thisToVesselMatrix = matrix * meshLocalToWorld;

            tempMatrix = thisToVesselMatrix * tempMatrix;

            vehicleBasisForwardVector = tempMatrix.MultiplyVector(vehicleBasisForwardVector);
        }

        public void SetThisToVesselMatrixForTransform()
        {
            meshLocalToWorld = intakeTrans.localToWorldMatrix;
        }

        public void UpdateArea()
        {
        }

        public static IntegratedIntakeEngineCrossSectionAdjuster CreateAdjuster(
            PartModule intake,
            Matrix4x4 worldToVesselMatrix
        )
        {
            var adjuster = new IntegratedIntakeEngineCrossSectionAdjuster();
            adjuster.SetupAdjuster(intake, worldToVesselMatrix);

            return adjuster;
        }

        public static IntegratedIntakeEngineCrossSectionAdjuster CreateAdjuster(
            ModuleResourceIntake intake,
            Matrix4x4 worldToVesselMatrix
        )
        {
            var adjuster = new IntegratedIntakeEngineCrossSectionAdjuster();
            adjuster.SetupAdjuster(intake, worldToVesselMatrix);

            return adjuster;
        }

        public void SetupAdjuster(PartModule intake, Matrix4x4 worldToVesselMatrix)
        {
            if (intake is ModuleResourceIntake module)
                SetupAdjuster(module, worldToVesselMatrix);
            else
                FARLogger.Error($"{intake} is not typeof ModuleResourceIntake");
        }

        public void SetupAdjuster(ModuleResourceIntake intake, Matrix4x4 worldToVesselMatrix)
        {
            part = intake.part;
            intakeModule = intake;
            intakeTrans = intakeModule.intakeTransform;

            thisToVesselMatrix = worldToVesselMatrix * intakeTrans.localToWorldMatrix;

            vehicleBasisForwardVector = Vector3.forward;
            vehicleBasisForwardVector = thisToVesselMatrix.MultiplyVector(vehicleBasisForwardVector);

            intakeArea = INTAKE_AREA_SCALAR * intake.area;

            FARLogger.Info("Integrated cross-section adjuster");
        }
    }
}
