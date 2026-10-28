/*
Ferram Aerospace Research
=========================
The default atmosphere of one body as the game stood at a moment, evaluable from any thread.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using ferram4;
using UnityEngine;

namespace FerramAerospaceResearch
{
    /// <summary>
    ///     The atmosphere a prediction takes for one body where no custom function is set, reproduced from values taken on the main thread
    ///     (CelestialBody's curves and constants) so they can be evaluated from any thread: pressure
    ///     (CelestialBody.GetPressure), temperature (CelestialBody.GetTemperature), the adiabatic index and the gas
    ///     constant. Each is the game's code with the live reads replaced by the captured values and the curves
    ///     evaluated through <see cref="ExactCurve" />.
    /// </summary>
    internal sealed class AtmosphereSnapshot
    {
        private readonly bool atmosphere;
        private readonly double atmosphereDepth;
        private readonly bool usePressureCurve;
        private readonly bool pressureCurveIsNormalized;
        private readonly double pressureSeaLevel;
        private readonly double temperatureLapseRate;
        private readonly double temperatureSeaLevel;
        private readonly double gasMassLapseRate;
        private readonly ExactCurve pressureCurve;
        private readonly bool useTemperatureCurve;
        private readonly bool temperatureCurveIsNormalized;
        private readonly ExactCurve temperatureCurve;
        private readonly double spaceTemperature;
        private readonly double adiabaticIndex;
        private readonly double gasConstant;

        /// <summary>False when a curve the reproduction needs uses a feature ExactCurve does not reproduce exactly.</summary>
        public bool IsExact { get; }

        /// <summary>Takes the body's state now. Main thread.</summary>
        public AtmosphereSnapshot(CelestialBody body)
        {
            atmosphere = body.atmosphere;
            atmosphereDepth = body.atmosphereDepth;
            usePressureCurve = body.atmosphereUsePressureCurve;
            pressureCurveIsNormalized = body.atmospherePressureCurveIsNormalized;
            pressureSeaLevel = body.atmospherePressureSeaLevel;
            temperatureLapseRate = body.atmosphereTemperatureLapseRate;
            temperatureSeaLevel = body.atmosphereTemperatureSeaLevel;
            gasMassLapseRate = body.atmosphereGasMassLapseRate;
            useTemperatureCurve = body.atmosphereUseTemperatureCurve;
            temperatureCurveIsNormalized = body.atmosphereTemperatureCurveIsNormalized;
            spaceTemperature = PhysicsGlobals.SpaceTemperature;
            adiabaticIndex = body.atmosphereAdiabaticIndex;
            gasConstant = PhysicsGlobals.IdealGasConstant / body.atmosphereMolarMass;

            bool exact = true;
            pressureCurve = Copy(body.atmospherePressureCurve, ref exact);
            temperatureCurve = Copy(body.atmosphereTemperatureCurve, ref exact);
            IsExact = exact;
        }

        private static ExactCurve Copy(FloatCurve curve, ref bool exact)
        {
            if (curve == null || curve.Curve == null)
                return null;
            var c = new ExactCurve(curve);
            exact &= c.IsExact;
            return c;
        }

        private static float Evaluate(ExactCurve curve, float t)
        {
            return curve != null ? curve.Evaluate(t) : 0;
        }

        public double AdiabaticIndex => adiabaticIndex;

        public double GasConstant => gasConstant;

        /// <summary>CelestialBody.GetPressure, in kPa.</summary>
        public double GetPressureKPa(double altitude)
        {
            if (!atmosphere || altitude >= atmosphereDepth)
                return 0.0;
            if (usePressureCurve)
            {
                if (pressureCurveIsNormalized)
                    return Mathf.Lerp(0f, (float)pressureSeaLevel, Evaluate(pressureCurve, (float)(altitude / atmosphereDepth)));
                return Evaluate(pressureCurve, (float)altitude);
            }

            return pressureSeaLevel * Math.Pow(1.0 - temperatureLapseRate * altitude / temperatureSeaLevel, gasMassLapseRate);
        }

        /// <summary>CelestialBody.GetTemperature.</summary>
        public double GetBaseTemperature(double altitude)
        {
            if (altitude >= atmosphereDepth)
                return 0.0;
            if (useTemperatureCurve)
            {
                if (temperatureCurveIsNormalized)
                    return UtilMath.Lerp(spaceTemperature,
                                         temperatureSeaLevel,
                                         Evaluate(temperatureCurve, (float)(altitude / atmosphereDepth)));
                return Evaluate(temperatureCurve, (float)altitude);
            }

            return temperatureSeaLevel - temperatureLapseRate * altitude;
        }

        /// <summary>The temperature a prediction takes (the default function: the altitude curve) at a place and time.</summary>
        public double GetTemperature(Vector3d latLonAltitude, double ut)
        {
            return GetBaseTemperature(latLonAltitude.z);
        }
    }
}
