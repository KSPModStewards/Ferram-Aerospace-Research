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
using KSP.UI.Screens.Flight;
using KSPCommunityFixes;
using UnityEngine;
using Object = UnityEngine.Object;

namespace FerramAerospaceResearch.FARGUI.FARFlightGUI
{
    internal class PhysicsCalcs
    {
        private readonly Vessel _vessel;
        private readonly FARVesselAero _vesselAero;

        private readonly FARCenterQuery aeroForces = new FARCenterQuery();
        private readonly int intakeAirId;
        private readonly double intakeAirDensity = 1;
        // private NavBall _navball;

        private List<FARAeroPartModule> _currentAeroModules;
        private List<FARWingAerodynamicModel> _LEGACY_currentWingAeroModel = new List<FARWingAerodynamicModel>();
        private bool useWingArea;
        private double wingArea;

        private VesselFlightInfo vesselInfo;

        public PhysicsCalcs(Vessel vessel, FARVesselAero vesselAerodynamics)
        {
            _vessel = vessel;
            _vesselAero = vesselAerodynamics;

            PartResourceLibrary resLibrary = PartResourceLibrary.Instance;
            PartResourceDefinition r = resLibrary.resourceDefinitions["IntakeAir"];
            if (r == null)
                return;
            intakeAirId = r.id;
            intakeAirDensity = r.density;
        }

        public void UpdateAeroModules(
            List<FARAeroPartModule> newAeroModules,
            List<FARWingAerodynamicModel> legacyWingModels
        )
        {
            _currentAeroModules = newAeroModules;
            _LEGACY_currentWingAeroModel = legacyWingModels;
            wingArea = 0;
            useWingArea = false;
            foreach (FARWingAerodynamicModel w in legacyWingModels)
            {
                if (w is null)
                    continue;
                useWingArea = true;
                wingArea += w.S;
            }
        }

        public VesselFlightInfo UpdatePhysicsParameters()
        {
            vesselInfo = new VesselFlightInfo();
            if (_vessel == null)
                return vesselInfo;

            Vector3d velVector = _vessel.srf_velocity -
                                 FARAtmosphere.GetWind(_vessel.mainBody,
                                                 _vessel.rootPart,
                                                 _vessel.ReferenceTransform.position);
            Vector3d velVectorNorm = velVector.normalized;
            double vesselSpeed = velVector.magnitude;


            CalculateTotalAeroForce();
            CalculateForceBreakdown(velVectorNorm);
            CalculateVesselOrientation(velVectorNorm);
            CalculateEngineAndIntakeBasedParameters(vesselSpeed);
            CalculateBallisticCoefficientAndTermVel();
            CalculateStallFraction();

            return vesselInfo;
        }

        private void CalculateTotalAeroForce()
        {
            aeroForces.ClearAll();


            if (_vessel.dynamicPressurekPa <= 0.00001)
                return;

            if (_currentAeroModules == null)
                return;
            foreach (FARAeroPartModule m in _currentAeroModules)
            {
                if (m is null || m == null)
                    continue;
                aeroForces.AddForce(m.transform.position, m.totalWorldSpaceAeroForce);
                aeroForces.AddTorque(m.worldSpaceTorque);
            }
        }

        private void CalculateForceBreakdown(Vector3d velVectorNorm)
        {
            if (useWingArea)
                vesselInfo.refArea = wingArea;
            else if (_vesselAero && _vesselAero.isValid)
                vesselInfo.refArea = _vesselAero.MaxCrossSectionArea;
            else
                vesselInfo.refArea = 1;

            vesselInfo.dynPres = _vessel.dynamicPressurekPa;

            if (_vessel.dynamicPressurekPa <= 0.00001)
            {
                vesselInfo.dragForce = vesselInfo.liftForce = vesselInfo.sideForce = 0;
                vesselInfo.dragCoeff = vesselInfo.liftCoeff = vesselInfo.sideCoeff = 0;
                vesselInfo.liftToDragRatio = 0;
                return;
            }

            Vector3d com_frc = aeroForces.force;
            Vector3d com_trq = aeroForces.TorqueAt(_vessel.CoM);

            vesselInfo.aerodynamicForce = com_frc;
            vesselInfo.aerodynamicTorque = com_trq;
            //reverse along vel normal will be drag
            vesselInfo.dragForce = -Vector3d.Dot(com_frc, velVectorNorm);

            Vector3d remainderVector = com_frc + velVectorNorm * vesselInfo.dragForce;

            //forward points down for the vessel, so reverse along that will be lift
            vesselInfo.liftForce = -Vector3d.Dot(remainderVector, _vessel.ReferenceTransform.forward);
            //and the side force
            vesselInfo.sideForce = Vector3d.Dot(remainderVector, _vessel.ReferenceTransform.right);

            double invAndDynPresArea = vesselInfo.refArea;
            invAndDynPresArea *= vesselInfo.dynPres;
            invAndDynPresArea = 1 / invAndDynPresArea;

            vesselInfo.dragCoeff = vesselInfo.dragForce * invAndDynPresArea;
            vesselInfo.liftCoeff = vesselInfo.liftForce * invAndDynPresArea;
            vesselInfo.sideCoeff = vesselInfo.sideForce * invAndDynPresArea;

            vesselInfo.liftToDragRatio = vesselInfo.liftForce / vesselInfo.dragForce;
        }

        // private void GetNavball()
        // {
        //     if (HighLogic.LoadedSceneIsFlight)
        //         _navball = Object.FindObjectOfType<NavBall>();
        // }

        private void CalculateVesselOrientation(Vector3d velVectorNorm)
        {
            Transform refTransform = _vessel.ReferenceTransform;

            Vector3 up = refTransform.up;
            Vector3 forward = refTransform.forward;
            Vector3 right = refTransform.right;
            //velocity vector projected onto a plane that divides the airplane into left and right halves
            Vector3 tmpVec = up * Vector3.Dot(up, velVectorNorm) + forward * Vector3.Dot(forward, velVectorNorm);
            vesselInfo.aoA = Vector3.Dot(tmpVec.normalized, forward);
            vesselInfo.aoA = FARMathUtil.rad2deg * Math.Asin(vesselInfo.aoA);
            if (double.IsNaN(vesselInfo.aoA))
                vesselInfo.aoA = 0;

            //velocity vector projected onto the vehicle-horizontal plane
            tmpVec = up * Vector3.Dot(up, velVectorNorm) + right * Vector3.Dot(right, velVectorNorm);
            vesselInfo.sideslipAngle = Vector3.Dot(tmpVec.normalized, right);
            vesselInfo.sideslipAngle = FARMathUtil.rad2deg * Math.Asin(vesselInfo.sideslipAngle);
            if (double.IsNaN(vesselInfo.sideslipAngle))
                vesselInfo.sideslipAngle = 0;

            // if (_navball == null)
            //     GetNavball();
            // if (!_navball)
            //     return;
            // Quaternion vesselRot = Quaternion.Inverse(_navball.relativeGymbal);

            // vesselInfo.headingAngle = vesselRot.eulerAngles.y;
            // vesselInfo.pitchAngle =
            //     vesselRot.eulerAngles.x > 180 ? 360 - vesselRot.eulerAngles.x : -vesselRot.eulerAngles.x;
            // vesselInfo.rollAngle =
            //     vesselRot.eulerAngles.z > 180 ? 360 - vesselRot.eulerAngles.z : -vesselRot.eulerAngles.z;

            // Gives the same numbers as the above, but also works in IVA mode which doesn't have a NavBall instance.
            var localUp = (_vessel.transform.position - _vessel.mainBody.transform.position).normalized;
            var east = Vector3.Cross(localUp, _vessel.mainBody.RotationAxis).normalized;
            var north = Vector3.Cross(east, localUp).normalized;
            vesselInfo.headingAngle = (SignedAngle(north, Vector3.ProjectOnPlane(up, localUp).normalized, localUp) + 360f) % 360f;
            vesselInfo.pitchAngle = 90f - SignedAngle(localUp, up, Vector3.Cross(localUp, up).normalized);
            vesselInfo.rollAngle = SignedAngle(Vector3.Cross(localUp, up).normalized, right, -up);
        }

        /// <summary>
        /// A more precise version of Vector3.SignedAngle at low angles.
        /// From https://www.reddit.com/r/Unity3D/comments/aruqz9/if_vector3angle_is_too_imprecise_or_jittery_at/
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <param name="axis"></param>
        /// <returns></returns>
        float SignedAngle(Vector3 from, Vector3 to, Vector3 axis)
        {
            var cross = Vector3.Cross(from, to);
            var dot = Vector3.Dot(from, to);
            var angle = Mathf.Atan2(cross.magnitude, dot) * Mathf.Rad2Deg;
            if (Vector3.Dot(axis, cross) < 0.0f)
                angle = -angle;
            return angle;
        }

        private void CalculateEngineAndIntakeBasedParameters(double vesselSpeed)
        {
            double totalThrust = 0;
            double totalThrust_Isp = 0;

            double fuelConsumptionVol = 0;
            double airDemandVol = 0;
            double airAvailableVol = 0;

            double invDeltaTime = 1 / TimeWarp.fixedDeltaTime;


            List<Part> partsList = _vessel.Parts;
            foreach (Part p in partsList)
            {
                Rigidbody rb = p.rb;
                if (rb != null)
                {
                    vesselInfo.fullMass += rb.mass;
                    vesselInfo.dryMass += p.mass;
                }

                var engineModules = p.FindModulesImplementingReadOnly<ModuleEngines>();
                foreach (ModuleEngines e in engineModules)
                {
                    FuelConsumptionFromEngineModule(e,
                                                    ref totalThrust,
                                                    ref totalThrust_Isp,
                                                    ref fuelConsumptionVol,
                                                    ref airDemandVol,
                                                    invDeltaTime);
                }

                var intakeModules = p.FindModulesImplementingReadOnly<ModuleResourceIntake>();
                foreach (ModuleResourceIntake intake in intakeModules)
                {
                    if (intake.intakeEnabled)
                    {
                        airAvailableVol += intake.airFlow * intakeAirDensity / invDeltaTime;
                        vesselInfo.fullMass -= p.Resources[intake.resourceName].amount * intakeAirDensity;
                    }
                }
            }

            if (totalThrust > 0)
            {
                vesselInfo.tSFC = totalThrust / totalThrust_Isp; //first, calculate inv Isp
                vesselInfo.tSFC *= 3600;                         //then, convert from 1/s to 1/hr
            }
            else
            {
                vesselInfo.tSFC = 0;
            }

            if (!airDemandVol.NearlyEqual(0))
                vesselInfo.intakeAirFrac = airAvailableVol / airDemandVol;
            else
                vesselInfo.intakeAirFrac = double.PositiveInfinity;

            vesselInfo.specExcessPower = totalThrust - vesselInfo.dragForce;
            vesselInfo.specExcessPower *= vesselSpeed / vesselInfo.fullMass;

            vesselInfo.velocityLiftToDragRatio = vesselSpeed * vesselInfo.liftToDragRatio;
            double L_D_TSFC = 0;
            double VL_D_TSFC = 0;
            if (!vesselInfo.tSFC.NearlyEqual(0))
            {
                L_D_TSFC = vesselInfo.liftToDragRatio / vesselInfo.tSFC;
                VL_D_TSFC = vesselInfo.velocityLiftToDragRatio / vesselInfo.tSFC * 3600;
            }

            vesselInfo.range = vesselInfo.fullMass / vesselInfo.dryMass;
            vesselInfo.range = Math.Log(vesselInfo.range);
            vesselInfo.endurance = L_D_TSFC * vesselInfo.range;
            vesselInfo.range *= VL_D_TSFC * 0.001;
        }

        private void FuelConsumptionFromEngineModule(
            ModuleEngines e,
            ref double totalThrust,
            ref double totalThrust_Isp,
            ref double fuelConsumptionVol,
            ref double airDemandVol,
            double invDeltaTime
        )
        {
            if (!e.EngineIgnited || e.engineShutdown)
                return;
            totalThrust += e.finalThrust;
            totalThrust_Isp += e.finalThrust * e.realIsp;
            foreach (Propellant v in e.propellants)
            {
                if (v.id == intakeAirId)
                    airDemandVol += v.currentRequirement;

                if (!v.ignoreForIsp)
                    fuelConsumptionVol += v.currentRequirement * invDeltaTime;
            }
        }

        private void CalculateBallisticCoefficientAndTermVel()
        {
            if (vesselInfo.dragCoeff.NearlyEqual(0))
            {
                vesselInfo.ballisticCoeff = 0;
                vesselInfo.termVelEst = 0;
                return;
            }

            double geeForce = FlightGlobals.getGeeForceAtPosition(_vessel.CoM).magnitude;

            vesselInfo.ballisticCoeff = vesselInfo.fullMass / (vesselInfo.dragCoeff * vesselInfo.refArea) * 1000;

            vesselInfo.termVelEst = 2 * vesselInfo.ballisticCoeff * geeForce;
            vesselInfo.termVelEst /= _vessel.atmDensity;
            vesselInfo.termVelEst = Math.Sqrt(vesselInfo.termVelEst);
        }

        private void CalculateStallFraction()
        {
            foreach (FARWingAerodynamicModel w in _LEGACY_currentWingAeroModel)
                vesselInfo.stallFraction += w.GetStall() * w.S;

            vesselInfo.stallFraction /= wingArea;
        }
    }
}
