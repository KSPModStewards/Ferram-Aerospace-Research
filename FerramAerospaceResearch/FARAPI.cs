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
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.Threading;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using KSPCommunityFixes;
using UnityEngine;

// ReSharper disable UnusedMember.Global

namespace FerramAerospaceResearch
{
    public static class FARAPI
    {
        public static Version Version { get; } = new Version();

        /// <summary>
        ///     Toggle or enable/disable FAR speed display.
        /// </summary>
        /// <param name="enabled">Enable/disable the speed display, null value toggles the speed display</param>
        /// <param name="v">
        ///     Vessel to toggle or enable/disable speed display for, null to apply <paramref name="enabled" />
        ///     globally
        /// </param>
        /// <returns>Success/failure of toggling or enabling/disabling the speed display</returns>
        public static bool ToggleAirspeedDisplay(bool? enabled = null, Vessel v = null)
        {
            if (v == null)
            {
                if (enabled == null)
                    AirspeedSettingsGUI.allEnabled = !AirspeedSettingsGUI.allEnabled;
                else
                    AirspeedSettingsGUI.allEnabled = (bool)enabled;
                return true;
            }

            FlightGUI gui = VesselFlightInfo(v);
            if (gui == null)
                return false;
            AirspeedSettingsGUI airspeedSettingsGUI = gui.airSpeedGUI;
            if (airspeedSettingsGUI == null)
                return false;
            if (enabled == null)
                airspeedSettingsGUI.enabled = !airspeedSettingsGUI.enabled;
            else
                airspeedSettingsGUI.enabled = (bool)enabled;
            return true;
        }

        public static FlightGUI VesselFlightInfo(Vessel v)
        {
            FlightGUI.vesselFlightGUI.TryGetValue(v, out FlightGUI gui);
            return gui;
        }

        public static double ActiveVesselIAS()
        {
            return VesselIAS(FlightGlobals.ActiveVessel);
        }

        public static double VesselIAS(Vessel vessel)
        {
            return VesselFlightInfo(vessel)?.airSpeedGUI?.CalculateIAS() ?? 0;
        }

        public static double ActiveVesselEAS()
        {
            return VesselEAS(FlightGlobals.ActiveVessel);
        }

        public static double VesselEAS(Vessel vessel)
        {
            return VesselFlightInfo(vessel)?.airSpeedGUI?.CalculateEAS() ?? 0;
        }

        public static double ActiveVesselDynPres()
        {
            return VesselDynPres(FlightGlobals.ActiveVessel);
        }

        public static double VesselDynPres(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.dynPres ?? 0;
        }

        public static double ActiveVesselLiftCoeff()
        {
            return VesselLiftCoeff(FlightGlobals.ActiveVessel);
        }

        public static double VesselLiftCoeff(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.liftCoeff ?? 0;
        }

        public static double ActiveVesselDragCoeff()
        {
            return VesselDragCoeff(FlightGlobals.ActiveVessel);
        }

        public static double VesselDragCoeff(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.dragCoeff ?? 0;
        }

        public static double ActiveVesselRefArea()
        {
            return VesselRefArea(FlightGlobals.ActiveVessel);
        }

        public static double VesselRefArea(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.refArea ?? 0;
        }

        public static double ActiveVesselTermVelEst()
        {
            return VesselTermVelEst(FlightGlobals.ActiveVessel);
        }

        public static double VesselTermVelEst(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.termVelEst ?? 0;
        }

        public static double ActiveVesselBallisticCoeff()
        {
            return VesselBallisticCoeff(FlightGlobals.ActiveVessel);
        }

        public static double VesselBallisticCoeff(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.ballisticCoeff ?? 0;
        }

        public static double ActiveVesselAoA()
        {
            return VesselAoA(FlightGlobals.ActiveVessel);
        }

        public static double VesselAoA(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.aoA ?? 0;
        }

        public static double ActiveVesselSideslip()
        {
            return VesselSideslip(FlightGlobals.ActiveVessel);
        }

        public static double VesselSideslip(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.sideslipAngle ?? 0;
        }

        public static double ActiveVesselTSFC()
        {
            return VesselTSFC(FlightGlobals.ActiveVessel);
        }

        public static double VesselTSFC(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.tSFC ?? 0;
        }

        public static double ActiveVesselStallFrac()
        {
            return VesselStallFrac(FlightGlobals.ActiveVessel);
        }

        public static double VesselStallFrac(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.stallFraction ?? 0;
        }

        /// <summary>
        ///     Increases flap deflection level for all control surfaces on this vessel, up to max setting of 3
        /// </summary>
        public static void VesselIncreaseFlapDeflection(Vessel v)
        {
            foreach (Part p in v.parts)
            {
                if (!(p.FindModuleImplementingFast<FARControllableSurface>() is FARControllableSurface surface))
                    continue;
                surface.SetDeflection(surface.flapDeflectionLevel + 1);
            }
        }

        /// <summary>
        ///     Decreases flap deflection level for all control surfaces on this vessel, down to min setting of 0
        /// </summary>
        public static void VesselDecreaseFlapDeflection(Vessel v)
        {
            foreach (Part p in v.parts)
            {
                if (!(p.FindModuleImplementingFast<FARControllableSurface>() is FARControllableSurface surface))
                    continue;
                surface.SetDeflection(surface.flapDeflectionLevel - 1);
            }
        }

        /// <summary>
        ///     Returns flap setting for this vessel
        /// </summary>
        /// <param name="v"></param>
        /// <returns>Flap setting; 0 - 3 indicates no to full flap deflections; -1 indicates lack of any control surface parts</returns>
        public static int VesselFlapSetting(Vessel v)
        {
            foreach (Part p in v.parts)
            {
                if (!(p.FindModuleImplementingFast<FARControllableSurface>() is FARControllableSurface surface))
                    continue;
                if (surface.isFlap)
                    return surface.flapDeflectionLevel;
            }

            return -1;
        }

        /// <summary>
        ///     Sets spoilers to a certain value on this vessel
        /// </summary>
        public static void VesselSetSpoilers(Vessel v, bool spoilerActive)
        {
            foreach (Part p in v.parts)
            {
                if (!(p.FindModuleImplementingFast<FARControllableSurface>() is FARControllableSurface surface))
                    continue;
                surface.brake = spoilerActive;
            }
        }

        /// <summary>
        ///     Returns spoiler setting for this vessel
        /// </summary>
        /// <param name="v"></param>
        /// <returns>Spoiler setting; true indicates active spoilers, false indicates inactive or no spoilers in existence</returns>
        public static bool VesselSpoilerSetting(Vessel v)
        {
            foreach (Part p in v.parts)
            {
                if (!(p.FindModuleImplementingFast<FARControllableSurface>() is FARControllableSurface surface))
                    continue;
                if (surface.isSpoiler)
                    return surface.brake;
            }

            return false;
        }

        /// <summary>
        ///     Returns the current aerodynamic force being experienced by the vehicle in world space
        /// </summary>
        /// <param name="v">The vessel that force is being queried</param>
        /// <returns>The force on the vessel in world space</returns>
        public static Vector3 VesselAerodynamicForce(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.aerodynamicForce ?? Vector3.zero;
        }

        /// <summary>
        ///     Returns the current aerodynamic torque being experienced by the vehicle in world space
        /// </summary>
        /// <param name="v">The vessel that force is being queried</param>
        /// <returns>The torque on the vessel in world space</returns>
        public static Vector3 VesselAerodynamicTorque(Vessel v)
        {
            return VesselFlightInfo(v)?.InfoParameters.aerodynamicTorque ?? Vector3.zero;
        }

        /// <summary>
        ///     Returns the current aerodynamic force being experienced by the active vehicle in world space
        /// </summary>
        /// <returns>The force on the vessel in world space</returns>
        public static Vector3 ActiveVesselAerodynamicForce()
        {
            return VesselAerodynamicForce(FlightGlobals.ActiveVessel);
        }

        /// <summary>
        ///     Returns the current aerodynamic torque being experienced by the active vehicle in world space
        /// </summary>
        /// <returns>The torque on the vessel in world space</returns>
        public static Vector3 ActiveVesselAerodynamicTorque()
        {
            return VesselAerodynamicTorque(FlightGlobals.ActiveVessel);
        }

        /// <summary>
        ///     Calculates the forces and torque on a vessel at a given condition at the CoM
        /// </summary>
        /// <param name="vessel">Vessel in question</param>
        /// <param name="aeroForce">Total aerodynamic force at CoM, in kN</param>
        /// <param name="aeroTorque">Total aerodynamic torque at CoM, in kN * m</param>
        /// <param name="velocityWorldVector">
        ///     Velocity vector in world space relative to the atmosphere for CURRENT vessel
        ///     orientation, m/s
        /// </param>
        /// <param name="altitude">Vessel altitude, in m</param>
        public static void CalculateVesselAeroForces(
            Vessel vessel,
            out Vector3 aeroForce,
            out Vector3 aeroTorque,
            Vector3 velocityWorldVector,
            double altitude
        )
        {
            if (!OnMainThread())
            {
                aeroForce = aeroTorque = Vector3.zero;
                return;
            }

            if (vessel == null)
            {
                aeroForce = aeroTorque = Vector3.zero;
                FARLogger.Error("API Error: attempted to simulate aerodynamics of null vessel");
                return;
            }

            CalculateVesselAeroForcesAt(vessel,
                                      out aeroForce,
                                      out aeroTorque,
                                      velocityWorldVector,
                                      vessel.CurrentPosition(altitude),
                                      Planetarium.GetUniversalTime());
        }

        /// <summary>
        ///     Calculates the forces and torque on a vessel at a given condition at the CoM, with the atmosphere taken
        ///     at a given place and time. The default atmosphere matches the density the game gives a vessel there
        ///     (latitude and sun heating included), so a trajectory predictor passing the predicted point gets the
        ///     forces the vessel will feel there; the altitude-only overload uses the vessel's current latitude,
        ///     longitude and time.
        /// </summary>
        /// <param name="vessel">Vessel in question</param>
        /// <param name="aeroForce">Total aerodynamic force at CoM, in kN</param>
        /// <param name="aeroTorque">Total aerodynamic torque at CoM, in kN * m</param>
        /// <param name="velocityWorldVector">
        ///     Velocity vector in world space relative to the atmosphere for CURRENT vessel
        ///     orientation, m/s
        /// </param>
        /// <param name="latLonAltitude">Latitude and longitude (deg, body-fixed) and altitude (m)</param>
        /// <param name="ut">Universal time the vessel is at that point</param>
        /// <remarks>
        ///     Main thread only: it reads the live vessel. Called from another thread it logs once and returns zero;
        ///     predict from a <see cref="FARVesselSnapshot" /> there.
        /// </remarks>
        public static void CalculateVesselAeroForcesAt(
            Vessel vessel,
            out Vector3 aeroForce,
            out Vector3 aeroTorque,
            Vector3 velocityWorldVector,
            Vector3d latLonAltitude,
            double ut
        )
        {
            aeroForce = aeroTorque = Vector3.zero;
            if (!OnMainThread())
                return;
            if (vessel == null)
            {
                FARLogger.Error("API Error: attempted to simulate aerodynamics of null vessel");
                return;
            }

            FARVesselAero vesselAero = vessel.GetComponent<FARVesselAero>();

            if (vesselAero == null)
            {
                FARLogger.Error($"API Error: vessel {vessel} does not have FARVesselAero aerocomponent for simulation");
                return;
            }

            vesselAero.SimulateAeroPropertiesAt(out aeroForce, out aeroTorque, velocityWorldVector, latLonAltitude, ut);
        }

        private static bool loggedOffMainThread;

        private static bool OnMainThread()
        {
            if (MainThread.IsMainThread)
                return true;
            if (!loggedOffMainThread)
            {
                loggedOffMainThread = true;
                // Off the main thread by definition: logged from it.
                FARLogAnyThread.Warning("FARAPI.CalculateVesselAeroForces(At) called off the main thread; returning zero. " +
                                        "Use CaptureVesselSnapshot on the main thread and CalculateSnapshotAeroForcesAt " +
                                        "on any thread.");
            }

            return false;
        }

        /// <summary>
        ///     Takes the vessel's aerodynamic model as it stands now, for predictions on any thread with
        ///     <see cref="CalculateSnapshotAeroForcesAt" />. Main thread only (null otherwise), and null when the vessel has
        ///     no FAR aerodynamics or has not been voxelized yet. The snapshot holds the voxel sections, the wings and
        ///     control surfaces with their parts' geometry, shielding and deflection, the body parts' geometry, the
        ///     intakes and engine throttle, the center of mass and the body's atmosphere, so a prediction from it reads
        ///     no live game state and gives the same answer on any thread.
        ///
        ///     The atmosphere: FAR's default functions are reproduced for the body from its curves and constants and
        ///     the sun's and the body's positions at capture (the time a point is carried from by the body's rotation
        ///     is the capture time), and checked against the live functions at capture (to a relative 1e-5; a body
        ///     whose atmosphere is not reproduced keeps its snapshots' predictions on the main thread, with a warning). A custom function registered
        ///     through FARAtmosphere is called as it is: on any thread when it was set with threadSafe true, otherwise
        ///     only on the main thread. <see cref="FARVesselSnapshot.OffMainThread" /> says which applies; a part's
        ///     AeroForceModifier likewise keeps predictions on the main thread.
        ///
        ///     A snapshot is stale once the vessel's aerodynamic model changes (a re-voxelization, a part added,
        ///     removed or destroyed, a wing rebuilt, the vessel unloaded): <see cref="FARVesselSnapshot.IsCurrent" />
        ///     turns false and predictions from it return false. Take a new one then.
        /// </summary>
        public static FARVesselSnapshot CaptureVesselSnapshot(Vessel vessel)
        {
            if (vessel == null)
                return null;
            if (!MainThread.IsMainThread)
            {
                FARLogAnyThread.Warning("FARAPI.CaptureVesselSnapshot called off the main thread; returning null.");
                return null;
            }

            FARVesselAero vesselAero = vessel.GetComponent<FARVesselAero>();
            return vesselAero == null ? null : vesselAero.CaptureSnapshot();
        }

        /// <summary>
        ///     Calculates the forces and torque on a snapshot of a vessel at a given condition, at the center of mass
        ///     as captured, with the atmosphere at a given place and time. Callable from any thread (see
        ///     <see cref="FARVesselSnapshot.OffMainThread" />); several threads may predict from one snapshot at once.
        ///     Predictions from the live vessel and from a snapshot of it agree to float rounding: the snapshot
        ///     transforms directions with the captured rotation where the live path asks Unity.
        /// </summary>
        /// <param name="snapshot">From <see cref="CaptureVesselSnapshot" /></param>
        /// <param name="aeroForce">Total aerodynamic force, in kN, world frame of the capture</param>
        /// <param name="aeroTorque">Total aerodynamic torque about the captured center of mass, in kN * m</param>
        /// <param name="velocityWorldVector">
        ///     Velocity relative to the atmosphere in the world frame of the capture, for the vessel's orientation at
        ///     capture, m/s
        /// </param>
        /// <param name="latLonAltitude">Latitude and longitude (deg, body-fixed) and altitude (m)</param>
        /// <param name="ut">Universal time the vessel is at that point</param>
        /// <returns>
        ///     False, with zero force and torque, when the snapshot is stale, when this thread may not predict from it,
        ///     or when the prediction failed.
        /// </returns>
        public static bool CalculateSnapshotAeroForcesAt(
            FARVesselSnapshot snapshot,
            out Vector3 aeroForce,
            out Vector3 aeroTorque,
            Vector3 velocityWorldVector,
            Vector3d latLonAltitude,
            double ut
        )
        {
            aeroForce = aeroTorque = Vector3.zero;
            if (snapshot == null || ReferenceEquals(snapshot.Aero, null))
                return false;
            return snapshot.Aero.SimulateSnapshot(snapshot, velocityWorldVector, latLonAltitude, ut, out aeroForce, out aeroTorque);
        }

        /// <summary>
        ///     <see cref="CalculateSnapshotAeroForcesAt" /> with every control surface deflected for the given control
        ///     inputs instead of as it was at capture: the deflection FAR's control surfaces take for those inputs once
        ///     settled (no blending toward it), with the dynamic deflection factor of the simulated speed and density, the
        ///     surface's settings as captured, and flaps and spoilers as captured. Lets a caller evaluate the vessel at
        ///     another stick position from one snapshot - a trim solve, say - on any thread.
        /// </summary>
        /// <param name="pitch">Pitch input, -1..1</param>
        /// <param name="yaw">Yaw input, -1..1</param>
        /// <param name="roll">Roll input, -1..1</param>
        public static bool CalculateSnapshotAeroForcesWithControlsAt(
            FARVesselSnapshot snapshot,
            out Vector3 aeroForce,
            out Vector3 aeroTorque,
            Vector3 velocityWorldVector,
            Vector3d latLonAltitude,
            double ut,
            float pitch,
            float yaw,
            float roll
        )
        {
            aeroForce = aeroTorque = Vector3.zero;
            if (snapshot == null || ReferenceEquals(snapshot.Aero, null))
                return false;
            return snapshot.Aero.SimulateSnapshot(snapshot, velocityWorldVector, latLonAltitude, ut, out aeroForce, out aeroTorque,
                                                  pitch, yaw, roll);
        }

        /// <summary>
        ///     Method to determine if the given vessel has been successfully voxelized at any time after being loaded
        /// </summary>
        /// <param name="vessel"></param>
        /// <returns>
        ///     True if vessel has ever been successfully voxelized, returns false if not or if the vessel is null and/or
        ///     destroyed
        /// </returns>
        public static bool VesselVoxelizationCompletedEver(Vessel vessel)
        {
            if (vessel == null)
                return false;

            FARVesselAero vesselAeroModule = null;
            foreach (VesselModule vM in vessel.vesselModules)
            {
                if (!(vM is FARVesselAero vesselAero))
                    continue;
                vesselAeroModule = vesselAero;
                break;
            }

            return !(vesselAeroModule is null) && vesselAeroModule.HasEverValidVoxelization();
        }


        /// <summary>
        ///     Method to determine if the given vessel has been successfully voxelized nd currently has valid voxelization
        /// </summary>
        /// <param name="vessel"></param>
        /// <returns>
        ///     True if vessel has valid voxelization currently, returns false if not or if the vessel is null and/or
        ///     destroyed
        /// </returns>
        public static bool VesselVoxelizationCompletedAndValid(Vessel vessel)
        {
            if (vessel == null)
                return false;

            FARVesselAero vesselAeroModule = null;
            foreach (VesselModule vM in vessel.vesselModules)
            {
                if (!(vM is FARVesselAero vesselAero))
                    continue;
                vesselAeroModule = vesselAero;
                break;
            }

            return !(vesselAeroModule is null) && vesselAeroModule.HasValidVoxelizationCurrently();
        }

        /// <summary>
        /// Set a modifier function for part local forces in case FAR does not correctly handle them.
        /// It will be called before applying the forces to the part/simulation.
        /// </summary>
        /// <param name="part">Part to apply modifier to</param>
        /// <param name="modifier">Modifier function of part local forces.
        /// Signature is (part, (drag, lift, torque)) -> (new_drag, new_lift, new_torque)</param>
        public static void SetPartAeroForceModifier(Part part, Func<Part, Vector3, Vector3> modifier)
        {
            part.FindModuleImplementingFast<FARAeroPartModule>().AeroForceModifier = modifier;
        }

        /// <summary>
        /// Get the current part local forces modifier function
        /// </summary>
        /// <param name="part"></param>
        /// <returns>Modifier function of part local forces if set and the part is valid, otherwise null.
        /// Signature is (part, (drag, lift, torque)) -> (new_drag, new_lift, new_torque)</returns>
        public static Func<Part, Vector3, Vector3> GetPartAeroForceModifier(Part part)
        {
            return part.FindModuleImplementingFast<FARAeroPartModule>()?.AeroForceModifier;
        }
    }
}
