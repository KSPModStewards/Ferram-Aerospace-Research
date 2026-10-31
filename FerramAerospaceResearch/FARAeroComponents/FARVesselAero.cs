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
using ferram4;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.FARThreading;
using KSPCommunityFixes;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    internal readonly struct CachedSimResults
    {
        public readonly Vector3 VelocityVector;
        public readonly Vector3d Position;
        public readonly Vector3 Force;
        public readonly Vector3 Torque;

        public CachedSimResults(Vector3 velocityVector, Vector3d position, Vector3 force, Vector3 torque)
        {
            VelocityVector = velocityVector;
            Position = position;
            Force = force;
            Torque = torque;
        }
    }

    public class FARVesselAero : VesselModule
    {
        private FlightGUI _flightGUI;
        private int _voxelCount;

        private List<GeometryPartModule> _currentGeoModules;
        private int geoModulesReady;

        private List<FARAeroPartModule> _currentAeroModules;
        private List<FARAeroPartModule> _unusedAeroModules;
        private List<FARAeroSection> _currentAeroSections;
        private List<FARWingAerodynamicModel> _legacyWingModels;

        private int _updateRateLimiter = 20;
        private bool _updateQueued = true;
        private bool _recalcGeoModules;
        private bool setup;

        private VehicleAerodynamics _vehicleAero;
        private VesselIntakeRamDrag _vesselIntakeRamDrag;
        private CachedSimResults lastSimResults;
        public VehicleExposure Exposure { get; private set; }

        internal VehicleAerodynamics VehicleAero
        {
            get { return _vehicleAero; }
        }

        public double Length
        {
            get { return _vehicleAero.Length; }
        }

        public bool isValid
        {
            get { return enabled && _vehicleAero != null; }
        }

        public double MaxCrossSectionArea
        {
            get { return _vehicleAero.MaxCrossSectionArea; }
        }

        public double MachNumber { get; private set; }

        public double ReynoldsNumber { get; private set; }

        protected override void OnStart()
        {
            FARLogger.Info("FARVesselAero on " + vessel.name + " reporting startup");

            _currentGeoModules = new List<GeometryPartModule>();

            Exposure = gameObject.AddComponent<VehicleExposure>();
            Exposure.transform.SetParent(transform, false);
            Exposure.Vessel = vessel;

            foreach (Part p in vessel.parts)
            {
                p.maximum_drag = 0;
                p.minimum_drag = 0;
                p.angularDrag = 0;

                GeometryPartModule g = p.GetComponent<GeometryPartModule>();
                if (!(g is null))
                {
                    _currentGeoModules.Add(g);
                    if (g.Ready)
                        geoModulesReady++;
                }

                if (!p.HasModuleImplementingFast<KerbalEVA>() && !p.HasModuleImplementingFast<FlagSite>())
                    continue;
                FARLogger.Info("Handling Stuff for KerbalEVA / Flag");
                g = (GeometryPartModule)p.AddModule("GeometryPartModule");
                g.OnStart(StartState());
                p.AddModule("FARAeroPartModule").OnStart(StartState());
                _currentGeoModules.Add(g);
            }

            RequestUpdateVoxel(false);

            enabled = true;
        }

        public override Activation GetActivation() => Activation.FlightScene | Activation.LoadedVessels;

        private PartModule.StartState StartState()
        {
            PartModule.StartState startState = PartModule.StartState.None;
            switch (vessel.situation)
            {
                case Vessel.Situations.PRELAUNCH:
                    startState |= PartModule.StartState.PreLaunch;
                    startState |= PartModule.StartState.Landed;
                    break;
                case Vessel.Situations.DOCKED:
                    startState |= PartModule.StartState.Docked;
                    break;
                case Vessel.Situations.ORBITING:
                case Vessel.Situations.ESCAPING:
                    startState |= PartModule.StartState.Orbital;
                    break;
                case Vessel.Situations.SUB_ORBITAL:
                    startState |= PartModule.StartState.SubOrbital;
                    break;
                case Vessel.Situations.SPLASHED:
                    startState |= PartModule.StartState.Splashed;
                    break;
                case Vessel.Situations.FLYING:
                    startState |= PartModule.StartState.Flying;
                    break;
                case Vessel.Situations.LANDED:
                    startState |= PartModule.StartState.Landed;
                    break;
            }

            return startState;
        }

        private void FixedUpdate()
        {
            if (_vehicleAero == null)
                return;
            if (_vehicleAero.CalculationCompleted)
            {
                _vehicleAero.GetNewAeroData(out _currentAeroModules,
                                            out _unusedAeroModules,
                                            out _currentAeroSections,
                                            out _legacyWingModels);

                Exposure.VesselBounds = _vehicleAero.VoxelBounds;

                if (_flightGUI is null)
                    _flightGUI = vessel.GetComponent<FlightGUI>();

                _flightGUI.UpdateAeroModules(_currentAeroModules, _legacyWingModels);

                foreach (FARAeroPartModule a in _unusedAeroModules)
                {
                    a.SetShielded(true);
                    a.ForceLegacyAeroUpdates();
                }

                foreach (FARAeroPartModule a in _currentAeroModules)
                {
                    a.SetShielded(false);
                    a.ForceLegacyAeroUpdates();
                }

                _vesselIntakeRamDrag.UpdateAeroData(_currentAeroModules);
            }

            if (FlightGlobals.ready && _currentAeroSections != null && vessel)
                CalculateAndApplyVesselAeroProperties();
            if (_currentGeoModules.Count > geoModulesReady)
                CheckGeoModulesReady();
            if (_updateRateLimiter < FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate)
                _updateRateLimiter++;
            else if (_updateQueued)
                VesselUpdate(_recalcGeoModules);
        }

        private void CalculateAndApplyVesselAeroProperties()
        {
            float atmDensity = (float)vessel.atmDensity;

            if (atmDensity <= 0)
            {
                MachNumber = 0;
                ReynoldsNumber = 0;
                return;
            }

            MachNumber = vessel.mach;
            ReynoldsNumber = FARAeroUtil.CalculateReynoldsNumber(vessel.atmDensity,
                                                                 Length,
                                                                 vessel.srfSpeed,
                                                                 MachNumber,
                                                                 FARAtmosphere.GetTemperature(vessel),
                                                                 FARAtmosphere.GetAdiabaticIndex(vessel));
            float skinFrictionDragCoefficient = (float)FARAeroUtil.SkinFrictionDrag(ReynoldsNumber, MachNumber);

            float pseudoKnudsenNumber = (float)(MachNumber / (ReynoldsNumber + MachNumber));

            Vector3 frameVel = Krakensbane.GetFrameVelocityV3f();

            //start from the top and come down to improve performance if it needs to remove anything
            for (int i = _currentAeroModules.Count - 1; i >= 0; i--)
            {
                FARAeroPartModule m = _currentAeroModules[i];
                if (m != null && m.part != null && m.part.partTransform != null)
                    m.UpdateVelocityAndAngVelocity(frameVel);
                else
                    _currentAeroModules.RemoveAt(i);
            }

            foreach (FARAeroSection aeroSection in _currentAeroSections)
                aeroSection.FlightCalculateAeroForces((float)MachNumber,
                                                      (float)(ReynoldsNumber / Length),
                                                      pseudoKnudsenNumber,
                                                      skinFrictionDragCoefficient);

            _vesselIntakeRamDrag.ApplyIntakeRamDrag((float)MachNumber, vessel.srf_velocity.normalized);

            foreach (FARAeroPartModule m in _currentAeroModules)
                m.ApplyForces();
        }

        public void SimulateAeroProperties(
            out Vector3 aeroForce,
            out Vector3 aeroTorque,
            Vector3 velocityWorldVector,
            double altitude
        )
        {
            Vector3d position = vessel.CurrentPosition(altitude);

            if (velocityWorldVector.NearlyEqual(lastSimResults.VelocityVector) &&
                (FARAtmosphere.IsCustom
                     // Custom atmospheres are not guaranteed to be independent of latitude and longitude
                     ? position.NearlyEqual(lastSimResults.Position)
                     : altitude.NearlyEqual(lastSimResults.Position.z)))
            {
                aeroForce = lastSimResults.Force;
                aeroTorque = lastSimResults.Torque;
                return;
            }

            var center = new FARCenterQuery();
            var dummy = new FARCenterQuery();

            //Calculate main gas properties
            GasProperties properties =
                FARAtmosphere.GetGasProperties(vessel.mainBody, position, Planetarium.GetUniversalTime());

            if (properties.Pressure <= 0 || properties.Temperature <= 0)
            {
                aeroForce = Vector3.zero;
                aeroTorque = Vector3.zero;
                return;
            }

            float velocityMag = velocityWorldVector.magnitude;
            float machNumber = (float)(velocityMag / properties.SpeedOfSound);
            float reynoldsNumber = (float)FARAeroUtil.CalculateReynoldsNumber(properties.Density,
                                                                              Length,
                                                                              velocityMag,
                                                                              machNumber,
                                                                              properties.Temperature,
                                                                              properties.AdiabaticIndex);

            float reynoldsPerLength = reynoldsNumber / (float)Length;
            float skinFriction = (float)FARAeroUtil.SkinFrictionDrag(reynoldsNumber, machNumber);

            float pseudoKnudsenNumber = machNumber / (reynoldsNumber + machNumber);

            if (_currentAeroSections != null)
            {
                foreach (FARAeroSection curSection in _currentAeroSections)
                    curSection?.PredictionCalculateAeroForces((float)properties.Density,
                                                              machNumber,
                                                              reynoldsPerLength,
                                                              pseudoKnudsenNumber,
                                                              skinFriction,
                                                              velocityWorldVector,
                                                              center);

                foreach (FARWingAerodynamicModel curWing in _legacyWingModels)
                    if (curWing != null)
                        center.AddForce(curWing.transform.position,
                                        curWing.PrecomputeCenterOfLift(velocityWorldVector,
                                                                       machNumber,
                                                                       properties.Density,
                                                                       dummy));
            }

            aeroForce = center.force;
            aeroTorque = center.TorqueAt(vessel.CoM);

            lastSimResults = new CachedSimResults(velocityWorldVector, position, aeroForce, aeroTorque);
        }


        private void TriggerIGeometryUpdaters()
        {
            foreach (GeometryPartModule geoModule in _currentGeoModules)
                geoModule.RunIGeometryUpdaters();
        }

        private void CheckGeoModulesReady()
        {
            geoModulesReady = 0;
            for (int i = 0; i < _currentGeoModules.Count; i++)
            {
                GeometryPartModule g = _currentGeoModules[i];
                if (g == null)
                {
                    _currentGeoModules.RemoveAt(i);
                    i--;
                }
                else
                {
                    geoModulesReady++;
                }
            }
        }

        public void AnimationVoxelUpdate()
        {
            if (_updateRateLimiter == FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate)
                _updateRateLimiter = FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate - 2;
            RequestUpdateVoxel(false);
        }

        public void VesselUpdateEvent(Vessel v)
        {
            if (v != vessel)
                return;
            if (_vehicleAero == null)
            {
                _vehicleAero = new VehicleAerodynamics();
                _vesselIntakeRamDrag = new VesselIntakeRamDrag();
            }

            RequestUpdateVoxel(true);
        }

        public void RequestUpdateVoxel(bool recalcGeoModules)
        {
            if (_updateRateLimiter > FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate)
                _updateRateLimiter = FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate - 2;
            _updateQueued = true;
            _recalcGeoModules |= recalcGeoModules;
        }

        public void VesselUpdate(bool recalcGeoModules)
        {
            if (vessel == null)
            {
                vessel = gameObject.GetComponent<Vessel>();
                if (vessel == null || vessel.vesselTransform == null)
                    return;
            }

            if (_vehicleAero == null)
            {
                _vehicleAero = new VehicleAerodynamics();
                _vesselIntakeRamDrag = new VesselIntakeRamDrag();
            }

            //this has been updated recently in the past; queue an update and return
            if (_updateRateLimiter < FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate)
            {
                _updateQueued = true;
                return;
            }

            _updateRateLimiter = 0;
            _updateQueued = false;
            if (vessel.rootPart.HasModuleImplementingFast<LaunchClamp>())
            {
                DisableModule();
                return;
            }

            if (recalcGeoModules)
            {
                _currentGeoModules.Clear();
                geoModulesReady = 0;
                foreach (Part p in vessel.Parts)
                {
                    GeometryPartModule g = p.FindModuleImplementingFast<GeometryPartModule>();
                    if (g is null)
                        continue;
                    _currentGeoModules.Add(g);
                    if (g.Ready)
                        geoModulesReady++;
                }
            }

            if (_currentGeoModules.Count > geoModulesReady)
            {
                _updateRateLimiter = FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate - 2;
                _updateQueued = true;
                return;
            }

            if (_currentGeoModules.Count == 0)
            {
                DisableModule();
                FARLogger.Info("Disabling FARVesselAero on " + vessel.name + " due to no FARGeometryModules on board");
            }

            TriggerIGeometryUpdaters();
            if (VoxelizationThreadpool.RunInMainThread)
                for (int i = _currentGeoModules.Count - 1; i >= 0; --i)
                {
                    if (_currentGeoModules[i].Ready)
                        continue;
                    _updateRateLimiter = FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate - 2;
                    _updateQueued = true;
                    return;
                }

            _voxelCount = VoxelCountFromType();
            if (!_vehicleAero.TryVoxelUpdate(vessel.vesselTransform.worldToLocalMatrix,
                                             vessel.vesselTransform.localToWorldMatrix,
                                             _voxelCount,
                                             vessel.Parts,
                                             _currentGeoModules,
                                             !setup,
                                             vessel))
            {
                _updateRateLimiter = FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate - 2;
                _updateQueued = true;
            }

            if (!_updateQueued)
                setup = true;

            FARLogger.Info("Updating vessel voxel for " + vessel.vesselName);
        }

        //TODO: have this grab from a config file
        private int VoxelCountFromType()
        {
            if (!vessel.isCommandable)
                return vessel.parts.Count >= 2 ? FARSettingsScenarioModule.VoxelSettings.numVoxelsDebrisVessel : 200;

            return FARSettingsScenarioModule.VoxelSettings.numVoxelsControllableVessel;
        }

        public override void OnLoadVessel()
        {
            if (vessel.loaded)
                if (ModUtils.IsBDArmoryInstalled && vessel.rootPart.Modules.Contains("MissileLauncher") &&
                    vessel.parts.Count == 1)
                {
                    vessel.rootPart.dragModel = Part.DragModel.CUBE;
                    enabled = false;
                    return;
                }

            VesselUpdateEvent(vessel);
            GameEvents.onVesselStandardModification.Add(VesselUpdateEvent);

            base.OnLoadVessel();
        }

        private void Unsubscribe()
        {
            GameEvents.onVesselStandardModification.Remove(VesselUpdateEvent);
        }

        public override void OnUnloadVessel()
        {
            Unsubscribe();

            base.OnUnloadVessel();
        }

        private void OnDestroy()
        {
            Unsubscribe();
            DisableModule();
        }

        private void DisableModule()
        {
            enabled = false;
        }

        public bool HasEverValidVoxelization()
        {
            return FlightGlobals.ready && _currentAeroSections != null && vessel;
        }

        public bool HasValidVoxelizationCurrently()
        {
            return FlightGlobals.ready && _currentAeroSections != null && vessel && !_updateQueued;
        }
    }
}
