/*
Ferram Aerospace Research v0.16.0.5 "Mader"
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
using FerramAerospaceResearch.Settings;
using ModularFI;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    [KSPAddon(KSPAddon.Startup.SpaceCentre, true)]
    public class ModularFlightIntegratorRegisterer : MonoBehaviour
    {
        public static float MIN_AREA_FRACTION = 0.01f;
        private void Start()
        {
            FARLogger.Info("Modular Flight Integrator function registration started");
            ModularFlightIntegrator.RegisterUpdateAerodynamicsOverride(UpdateAerodynamics);
            ModularFlightIntegrator.RegisterUpdateThermodynamicsPre(UpdateThermodynamicsPre);
            ModularFlightIntegrator.RegisterCalculateAreaExposedOverride(CalculateAreaExposed);
            ModularFlightIntegrator.RegisterCalculateAreaRadiativeOverride(CalculateAreaRadiative);
            ModularFlightIntegrator.RegisterGetSunAreaOverride(CalculateSunArea);
            ModularFlightIntegrator.RegisterGetBodyAreaOverride(CalculateBodyArea);
            ModularFlightIntegrator.RegisterUpdateOcclusionOverride(UpdateOcclusion);
            ModularFlightIntegrator.RegisterSetSkinProperties(SetSkinProperties);
            ModularFlightIntegrator.RegisterUpdateConductionOverride(UpdateConduction);
            FARLogger.Info("Modular Flight Integrator function registration complete");
            Destroy(this);
        }

        private void UpdateConduction(ModularFlightIntegrator fi)
        {
            // Improve the skinSkinTransfer calc: move the exposed fraction into the sqrt.
            foreach (PartThermalData ptd in fi.partThermalDataList)
            {
                double frac = Math.Min(ptd.part.skinExposedAreaFrac, 1.0 - ptd.part.skinExposedAreaFrac);
                double exposedArea = frac * ptd.part.radiativeArea;
                ptd.skinSkinTransfer = ptd.part.skinSkinConductionMult * PhysicsGlobals.SkinSkinConductionFactor * 2.0 * Math.Sqrt(exposedArea);
            }
            fi.BaseFIUpdateConduction();
        }

        // This is the primary consumer of DragCube.ExposedArea, so handling this ourselves may
        // remove the need for any calculation gymnastics we were doing to not interfere with stock handling.
        private static void SetSkinProperties(ModularFlightIntegrator fi, PartThermalData ptd)
        {
            Part part = ptd.part;

            //if (occlusion == null || occlusion.state == VehicleOcclusion.State.Invalid || aeroModule == null)
            if (!(fi.Vessel.GetComponent<VehicleOcclusion>() is VehicleOcclusion occlusion &&
                  part.Modules.GetModule<FARAeroPartModule>() is FARAeroPartModule aeroModule &&
                  occlusion.state != VehicleOcclusion.State.Invalid))
            {
                fi.BaseFIetSkinPropertie(ptd);
                return;
            }
            if (part.skinUnexposedTemperature < PhysicsGlobals.SpaceTemperature)
                part.skinUnexposedTemperature = part.skinTemperature;

            double a = Math.Max(part.radiativeArea, 0.001);
            ptd.radAreaRecip = 1.0 / Math.Max(part.radiativeArea, 0.001);

//            Vector3 localForward = (part.vessel.transform.worldToLocalMatrix * part.vessel.transform.forward);
//            Vector3 localVel = part.vessel.transform.worldToLocalMatrix * fi.Vel;
            double areaFraction = 0;
            if (!part.ShieldedFromAirstream && part.atmDensity > 0)
            {
                // Stock calculation:
                //ptd.convectionArea = UtilMath.Lerp(a, part.exposedArea, -PhysicsGlobals.FullConvectionAreaMin + (fi.mach - PhysicsGlobals.FullToCrossSectionLerpStart) / (PhysicsGlobals.FullToCrossSectionLerpEnd - PhysicsGlobals.FullToCrossSectionLerpStart)) * ptd.convectionAreaMultiplier;
                // Note that part.exposedArea has some scaling based on the mach factor:
                //float dot = Vector3.Dot(direction, faceDirection);
                //float num2 = PhysicsGlobals.DragCurveValue(PhysicsGlobals.SurfaceCurves, ((dot + 1.0)/2), machNumber);
                //retData.exposedArea += this.areaOccluded[index] * num2 / PhysicsGlobals.DragCurveMultiplier.Evaluate(machNumber)
                // PhysicsGlobals.DragCurveValue scales its transsonic calcs by *= PhysicsGlobals.DragCubeMultiplier.Evaluate(machNumber)
                // So thermal convection undoes -that- part of the scaling, but leaves:
                /*
                DRAG_TIP
                {
                    key = 0 1 0 0
                    key = 0.85 1.19 0.6960422 0.6960422
                    key = 1.1 2.83 0.730473 0.730473
                    key = 5 4 0 0
                }
                Mach 1.1 - 5 has a multiplier from 2.83 to 4.  Mach 5+ has a multiplier of 4.
                Does this make sense for convection heating?
                */

                // VehicleOcclusion's handler might need to be more careful about how it averages values.
                // Otherwise parts behind same-size shields can tend towards very small convectionArea when it should probably be 0.
                ptd.convectionArea = occlusion.ConvectionArea(part, fi.Vel);
                ptd.convectionCoeffMultiplier = PhysicsGlobals.SurfaceCurves.dragCurveTip.Evaluate(Convert.ToSingle(fi.mach));

                double d = ptd.convectionArea * ptd.radAreaRecip;
                areaFraction = (!double.IsNaN(d) && d > 0.001) ? d : 0;
                areaFraction = Math.Min(areaFraction, 1);
                if (areaFraction < MIN_AREA_FRACTION)
                    areaFraction = 0;
            }
            if (areaFraction > 0)
            {
                StockSkinTemperatureHandling(ptd, areaFraction);
                ptd.exposed = true;
                part.skinExposedAreaFrac = areaFraction;
                part.skinExposedArea = areaFraction * a;    // == ptd.convectionArea
            }
            else
            {
                if (ptd.exposed)
                    fi.UnifySkinTemp(ptd);
                ptd.exposed = false;
                ptd.convectionArea = part.skinExposedArea = part.skinUnexposedMassMult = 0.0;
                part.skinExposedAreaFrac = part.skinExposedMassMult = 1.0;
            }
            ptd.convectionTempMultiplier = ptd.exposed ? 1 : 0;
        }

        private static void StockSkinTemperatureHandling(PartThermalData ptd, double areaFraction)
        {
            if (areaFraction <= 0)
                return;
            Part part = ptd.part;
            if (!ptd.exposed || areaFraction == 1.0)
                part.skinUnexposedTemperature = part.skinTemperature;
            ptd.exposed = true;
            part.skinExposedMassMult = 1.0 / areaFraction;
            if (areaFraction < 1.0)
                part.skinUnexposedMassMult = 1.0 / (1.0 - areaFraction);
            else
                part.skinUnexposedMassMult = 0.0;

            // If the area fraction has changed since last calculation
            if (part.skinExposedAreaFrac != areaFraction)
            {
                if (part.skinUnexposedTemperature != part.skinTemperature &&
                    part.skinExposedAreaFrac > 0.0 &&
                    part.skinExposedAreaFrac < 1.0 &&
                    areaFraction < 1.0)
                {
                    double unexposedAreaFrac = 1.0 - part.skinExposedAreaFrac;
                    double thermalEnergyInExposedSkin = part.skinTemperature * part.skinExposedAreaFrac * part.skinThermalMass;
                    double thermalEnergyInUnexposedSkin = part.skinUnexposedTemperature * unexposedAreaFrac * part.skinThermalMass;
                    double dAreaFrac = areaFraction - part.skinExposedAreaFrac;
                    double dThermalEnergy;
                    if (dAreaFrac > 0.0)
                        dThermalEnergy = dAreaFrac / unexposedAreaFrac * thermalEnergyInUnexposedSkin;
                    else
                        dThermalEnergy = dAreaFrac / part.skinExposedAreaFrac * thermalEnergyInExposedSkin;
                    part.skinTemperature = (thermalEnergyInExposedSkin + dThermalEnergy) * part.skinExposedMassMult * part.skinThermalMassRecip;
                    part.skinUnexposedTemperature = (thermalEnergyInUnexposedSkin - dThermalEnergy) * part.skinUnexposedMassMult * part.skinThermalMassRecip;
                }
            }
        }

        private static void UpdateOcclusion(ModularFlightIntegrator fi, bool all)
        {
            if (fi.Vessel.GetComponent<VehicleOcclusion>() is VehicleOcclusion occlusion
                && occlusion.state != VehicleOcclusion.State.Invalid)
            {
                foreach (Part p in fi.Vessel.Parts)
                {
                    p.ptd.bodyAreaMultiplier = 1;
                    p.ptd.sunAreaMultiplier = 1;
                    p.ptd.convectionAreaMultiplier = 1;
                    p.ptd.convectionTempMultiplier = 1;
                }
            }
            else
                fi.BaseFIUpdateOcclusion(all);
        }

        private static void UpdateThermodynamicsPre(ModularFlightIntegrator fi)
        {
            bool voxelizationCompleted =
                (fi.Vessel.vesselModules.Find(module => module is FARVesselAero) as FARVesselAero)?
                .HasEverValidVoxelization() ?? false;

            for (int i = 0; i < fi.PartThermalDataCount; i++)
            {
                PartThermalData ptd = fi.partThermalDataList[i];
                Part part = ptd.part;
                if (!(part.Modules.GetModule<FARAeroPartModule>() is FARAeroPartModule aeroModule))
                    continue;

                // make sure drag cube areas are correct based on voxelization
                if (voxelizationCompleted)
                {
                    if (!part.DragCubes.None && aeroModule)
                        for (int j = 0; j < 6; j++)
                            part.DragCubes.AreaOccluded[FARAeroPartModule.ProjectedArea.FaceMap[j]] =
                                (float)aeroModule.ProjectedAreas[j];

                    part.radiativeArea = CalculateAreaRadiative(fi, part, aeroModule);
                    part.exposedArea = part.machNumber > 0
                                           ? CalculateAreaExposed(fi, part, aeroModule)
                                           : part.radiativeArea;
                }
                else
                {
                    part.radiativeArea = fi.BaseFICalculateAreaRadiative(part);
                    part.exposedArea = fi.BaseFICalculateAreaExposed(part);
                }

                if (FARSettings.ExposedAreaLimited && part.exposedArea > part.radiativeArea)
                    part.exposedArea = part.radiativeArea; //sanity check just in case
            }
        }

        private static void UpdateAerodynamics(ModularFlightIntegrator fi, Part part)
        {
            //FIXME Proper model for airbrakes
            if (part.Modules.Contains<ModuleAeroSurface>() ||
                part.Modules.Contains("MissileLauncher") && part.vessel.rootPart == part)
            {
                fi.BaseFIUpdateAerodynamics(part);
            }
            else
            {
                Rigidbody rb = part.rb;
                if (!rb)
                    return;
                part.dragVector = rb.velocity +
                                  Krakensbane.GetFrameVelocity() -
                                  FARAtmosphere.GetWind(FlightGlobals.currentMainBody, part, rb.position);
                part.dragVectorSqrMag = part.dragVector.sqrMagnitude;
                if (part.dragVectorSqrMag.NearlyEqual(0) || part.ShieldedFromAirstream)
                {
                    part.dragVectorMag = 0f;
                    part.dragVectorDir = Vector3.zero;
                    part.dragVectorDirLocal = Vector3.zero;
                    part.dragScalar = 0f;
                }
                else
                {
                    part.dragVectorMag = (float)Math.Sqrt(part.dragVectorSqrMag);
                    part.dragVectorDir = part.dragVector / part.dragVectorMag;
                    part.dragVectorDirLocal = -part.partTransform.InverseTransformDirection(part.dragVectorDir);
                    CalculateLocalDynPresAndAngularDrag(fi, part);
                }

                if (part.DragCubes.None)
                    return;

                part.DragCubes.SetDrag(part.dragVectorDirLocal, (float)fi.mach);
            }
        }

        private static void CalculateLocalDynPresAndAngularDrag(ModularFlightIntegrator fi, Part p)
        {
            p.dynamicPressurekPa = p.atmDensity;
            if (fi.CurrentMainBody.ocean && p.submergedPortion > 0)
            {
                p.submergedDynamicPressurekPa = fi.CurrentMainBody.oceanDensity * 1000;
            }
            else
            {
                p.submergedDynamicPressurekPa = 0;
            }

            double tmp = 0.0005 * p.dragVectorSqrMag;

            p.submergedDynamicPressurekPa *= tmp;
            p.dynamicPressurekPa *= tmp;

            tmp = p.dynamicPressurekPa * (1.0 - p.submergedPortion);
            tmp += p.submergedDynamicPressurekPa *
                   PhysicsGlobals.BuoyancyWaterAngularDragScalar *
                   p.waterAngularDragMultiplier *
                   p.submergedPortion;

            p.rb.angularDrag = (float)(p.angularDrag * tmp * PhysicsGlobals.AngularDragMultiplier);

            tmp = Math.Max(fi.pseudoReDragMult, 1);
            //dyn pres adjusted for submersion
            p.dragScalar = (float)((p.dynamicPressurekPa * (1.0 - p.submergedPortion) +
                                    p.submergedDynamicPressurekPa *
                                    p.submergedPortion *
                                    FARSettings.SubmergedDragMultiplier) *
                                   tmp);
            p.bodyLiftScalar = (float)(p.dynamicPressurekPa * (1.0 - p.submergedPortion) +
                                       p.submergedDynamicPressurekPa *
                                       p.submergedPortion *
                                       FARSettings.SubmergedLiftMultiplier) *
                               p.bodyLiftMultiplier;
        }

        private static double CalculateAreaRadiative(ModularFlightIntegrator fi, Part part)
        {
            FARAeroPartModule module = part.Modules.GetModule<FARAeroPartModule>();
            return CalculateAreaRadiative(fi, part, module);
        }

        private static double CalculateAreaRadiative(
            ModularFlightIntegrator fi,
            Part part,
            FARAeroPartModule aeroModule
        )
        {
            if (aeroModule is null || !FARAPI.VesselVoxelizationCompletedEver(part.vessel))
                return fi.BaseFICalculateAreaRadiative(part);
            double radArea = aeroModule.ProjectedAreas.totalArea;

            return radArea > 0 ? radArea : fi.BaseFICalculateAreaRadiative(part);
        }

        private static double CalculateAreaExposed(ModularFlightIntegrator fi, Part part)
        {
            FARAeroPartModule module = part.Modules.GetModule<FARAeroPartModule>();
            return CalculateAreaExposed(fi, part, module);
        }

        private static double CalculateAreaExposed(ModularFlightIntegrator fi, Part part, FARAeroPartModule aeroModule)
        {
            if (aeroModule is null || !FARAPI.VesselVoxelizationCompletedEver(part.vessel))
                return fi.BaseFICalculateAreaExposed(part);

            // Apparently stock exposed area is actually weighted by some function of mach number...
            // otherwise heating is much lower
            double exposedArea = FARSettings.ExposedAreaUsesKSPHack
                                     ? part.DragCubes.ExposedArea
                                     : aeroModule.ProjectedAreaLocal(-part.dragVectorDirLocal);

            return exposedArea > 0 ? exposedArea : fi.BaseFICalculateAreaExposed(part);
        }

        private static double CalculateSunArea(ModularFlightIntegrator fi, PartThermalData ptd)
        {
            double sunArea = 0;
            if (fi.Vessel.GetComponent<VehicleOcclusion>() is VehicleOcclusion occlusion)
            {
                sunArea = occlusion.SunArea(ptd.part, fi.sunVector);
            }
            else if (ptd.part.Modules.GetModule<FARAeroPartModule>() is FARAeroPartModule module && FARAPI.VesselVoxelizationCompletedEver(ptd.part.vessel))
            {
                sunArea = module.ProjectedAreaWorld(fi.sunVector) * ptd.sunAreaMultiplier;
            }
            return sunArea > 0 ? sunArea : fi.BaseFIGetSunArea(ptd);
        }

        private static double CalculateBodyArea(ModularFlightIntegrator fi, PartThermalData ptd)
        {
            double bodyArea = 0;
            Vector3 bodyVec = fi.Vessel.transform.worldToLocalMatrix * (Vector3)(fi.Vessel.mainBody.position - fi.Vessel.transform.position);
            if (fi.Vessel.GetComponent<VehicleOcclusion>() is VehicleOcclusion occlusion && occlusion.state != VehicleOcclusion.State.Invalid)
            {
                bodyArea = occlusion.BodyArea(ptd.part, bodyVec.normalized);
            }
            else if (ptd.part.Modules.GetModule<FARAeroPartModule>() is FARAeroPartModule module && FARAPI.VesselVoxelizationCompletedEver(ptd.part.vessel))
            {
                bodyArea = module.ProjectedAreaWorld(bodyVec.normalized) * ptd.bodyAreaMultiplier;
            }
            return bodyArea > 0 ? bodyArea : fi.BaseFIBodyArea(ptd);
        }
    }
}
