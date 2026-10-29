/*
Ferram Aerospace Research
=========================
A vessel's aerodynamic model as it stood at one moment, for predictions on any thread.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Threading;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using UnityEngine;

namespace FerramAerospaceResearch
{
    /// <summary>
    ///     A vessel's aerodynamic model as it stood when <see cref="FARAPI.CaptureVesselSnapshot" /> took it on the main
    ///     thread: the voxel sections, every wing and control surface with its part's transform, mass, shielding and
    ///     deflection, the body parts' transforms and angular velocities, the vessel's center of mass, and the body's
    ///     atmosphere. <see cref="FARAPI.CalculateSnapshotAeroForcesAt" />
    ///     predicts from it on any thread, reading no live game state; world-frame vectors are those of the capture
    ///     frame. Once the vessel changes (a re-voxelization, a part added, removed or destroyed, a wing rebuilt) the
    ///     snapshot is stale and predictions from it return false; take a new one.
    /// </summary>
    public sealed class FARVesselSnapshot
    {
        internal FARVesselSnapshot()
        {
        }

        /// <summary>The vessel the snapshot was taken of.</summary>
        public Vessel Vessel { get; internal set; }

        /// <summary>Universal time at capture.</summary>
        public double CaptureUT { get; internal set; }

        /// <summary>Whether the vessel's aerodynamic model is still the one captured (reads no game state; any thread).</summary>
        public bool IsCurrent
        {
            get { return !ReferenceEquals(Aero, null) && Aero.SnapshotVersion == AeroVersion && WingGeometryVersion() == GeometryVersion; }
        }

        // The captured wings' rebuild counters summed: they only increase, so an equal sum means none changed.
        internal long WingGeometryVersion()
        {
            long sum = 0;
            foreach (FARWingAerodynamicModel w in Wings)
                sum += w.GeometryVersion;
            return sum;
        }

        /// <summary>
        ///     Whether predictions from this snapshot may run off the main thread: false when a custom atmosphere
        ///     function that was not set as thread-safe is in use, when the default atmosphere could not be reproduced
        ///     exactly for this body, or when a part carries an aerodynamic force modifier (a delegate another mod
        ///     sets, which may read game state). Main-thread predictions from the snapshot work either way.
        /// </summary>
        public bool OffMainThread { get; internal set; }

        internal FARVesselAero Aero;
        internal long AeroVersion;
        internal long GeometryVersion;

        internal FARAeroSection[] Sections;
        internal FARAeroPartModule[] Modules;
        internal FARAeroPartModule.SnapshotState[] ModuleStates;

        // Every wing on the vessel, captured; the ones whose forces are summed (the vessel's legacy wing models) are
        // SimulatedWings. The others are there as neighbours a wing reads.
        internal FARWingAerodynamicModel[] Wings;
        internal FARWingAerodynamicModel.SnapshotState[] WingStates;
        internal FARWingAerodynamicModel[] SimulatedWings;

        internal Vector3 CenterOfMass;
        internal double Length;
        internal double AdiabaticIndex;
        internal double VesselAdiabaticIndex;
        internal double ExternalTemperature;
        internal Settings.BodySettings BodyData;

        internal CelestialBody Body;

        // The default atmosphere reproduced for the body; null when it could not be reproduced exactly.
        internal AtmosphereSnapshot Atmosphere;

        // Which property functions are custom (evaluated through FARAtmosphere, thread-safe or on the main thread),
        // and whether every custom one was set as thread-safe.
        internal bool CustomThreadSafe;
        internal bool CustomPressure;
        internal bool CustomTemperature;
        internal bool CustomAdiabaticIndex;
        internal bool CustomGasConstant;

        // The functions a prediction called at capture (FARAtmosphere.Prediction*Function): a custom one as set, else
        // the default. Called for a custom property, and for a default one on the main thread when the atmosphere
        // could not be reproduced.
        internal Func<CelestialBody, Vector3d, double, double> PressureFunction;
        internal Func<CelestialBody, Vector3d, double, double> TemperatureFunction;
        internal Func<CelestialBody, Vector3d, double, double> AdiabaticIndexFunction;
        internal Func<CelestialBody, Vector3d, double, double> GasConstantFunction;

        // A captured function's value, false when it throws (no fallback to a default, which reads live game state).
        private bool TryCall(Func<CelestialBody, Vector3d, double, double> f, Vector3d latLonAltitude, double ut, out double value)
        {
            try
            {
                value = f(Body, latLonAltitude, ut);
                return true;
            }
            catch (Exception)
            {
                value = 0;
                return false;
            }
        }

        /// <summary>
        ///     The gas properties at a place and time as the snapshot's atmosphere gives them: the reproduced default
        ///     where a property is not custom, the custom function (without its live fallback) where it is. False when
        ///     a property cannot be evaluated on this thread or a custom function threw.
        /// </summary>
        internal bool TryGetGasProperties(Vector3d latLonAltitude, double ut, bool mainThread, out GasProperties properties)
        {
            properties = default;
            bool anyCustom = CustomPressure || CustomTemperature || CustomAdiabaticIndex || CustomGasConstant;
            if (!mainThread && anyCustom && !CustomThreadSafe)
                return false;

            double pressure, temperature, adiabaticIndex, gasConstant;
            if (CustomPressure || Atmosphere == null)
            {
                if (!(CustomPressure || mainThread) || !TryCall(PressureFunction, latLonAltitude, ut, out pressure))
                    return false;
            }
            else
            {
                pressure = Atmosphere.GetPressureKPa(latLonAltitude.z) * 1000;
            }

            if (CustomTemperature || Atmosphere == null)
            {
                if (!(CustomTemperature || mainThread) || !TryCall(TemperatureFunction, latLonAltitude, ut, out temperature))
                    return false;
            }
            else
            {
                temperature = Atmosphere.GetTemperature(latLonAltitude, ut);
            }

            if (CustomAdiabaticIndex || Atmosphere == null)
            {
                if (!(CustomAdiabaticIndex || mainThread) || !TryCall(AdiabaticIndexFunction, latLonAltitude, ut, out adiabaticIndex))
                    return false;
            }
            else
            {
                adiabaticIndex = Atmosphere.AdiabaticIndex;
            }

            if (CustomGasConstant || Atmosphere == null)
            {
                if (!(CustomGasConstant || mainThread) || !TryCall(GasConstantFunction, latLonAltitude, ut, out gasConstant))
                    return false;
            }
            else
            {
                gasConstant = Atmosphere.GasConstant;
            }

            properties = new GasProperties(pressure, temperature, adiabaticIndex, gasConstant);
            return true;
        }

        /// <summary>
        ///     Whether the reproduced atmosphere gives what a live prediction takes now at a point
        ///     (FARAtmosphere.GetGasProperties), to a relative 1e-5 in pressure and temperature: float rounding
        ///     in a curve evaluation passes, a different atmosphere (a body type or a patch that changes the game's
        ///     functions) does not. Main thread, at capture.
        /// </summary>
        internal bool AtmosphereMatchesLive(Vector3d latLonAltitude, double ut, out string difference)
        {
            difference = null;
            GasProperties live = FARAtmosphere.GetGasProperties(Body, latLonAltitude, ut);
            if (!TryGetGasProperties(latLonAltitude, ut, true, out GasProperties reproduced))
            {
                difference = "not evaluable";
                return false;
            }

            if (Close(live.Pressure, reproduced.Pressure) && Close(live.Temperature, reproduced.Temperature))
                return true;
            difference = FormattableString.Invariant($"at {latLonAltitude}: pressure {live.Pressure} vs {reproduced.Pressure}, temperature {live.Temperature} vs {reproduced.Temperature}");
            return false;
        }

        private static bool Close(double a, double b)
        {
            return a == b || Math.Abs(a - b) <= 1e-5 * Math.Max(Math.Abs(a), Math.Abs(b));
        }
    }
}
