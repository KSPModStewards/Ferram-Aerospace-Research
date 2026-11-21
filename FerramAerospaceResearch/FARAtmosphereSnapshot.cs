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
    ///     (CelestialBody's curves and constants, the body's frame and position, the sun's position, the body's orbit
    ///     around the sun) so they can be evaluated from any thread: pressure (CelestialBody.GetPressure), temperature
    ///     (FARAtmosphere's GameTemperature: CelestialBody.GetTemperature, GetAtmoThermalStats and GetFullTemperature),
    ///     the adiabatic index and the gas constant. Each is the game's code with the live reads replaced by the
    ///     captured values and the curves evaluated through <see cref="ExactCurve" />. As GameTemperature does, a
    ///     point is carried to the requested time by the body's rotation and the sun's motion is not; the time it is
    ///     carried from is the capture time.
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
        private readonly ExactCurve temperatureSunMultCurve;
        private readonly ExactCurve latitudeBiasCurve;
        private readonly ExactCurve latitudeSunMultCurve;
        private readonly ExactCurve axialSunBiasCurve;
        private readonly ExactCurve axialSunMultCurve;
        private readonly ExactCurve eccentricityBiasCurve;
        private readonly double adiabaticIndex;
        private readonly double gasConstant;

        private readonly bool inFlight;
        private readonly double radius;
        private readonly bool rotates;
        private readonly double rotationPeriod;
        private readonly Planetarium.CelestialFrame bodyFrame;
        private readonly Vector3d bodyPosition;
        private readonly Vector3 bodyUp;
        private readonly bool hasSun;
        private readonly Vector3d sunPosition;
        private readonly double trueAnomaly;
        private readonly double eccentricity;
        private readonly double orbitRadius;
        private readonly double orbitPeR;
        private readonly double orbitApR;
        private readonly double captureUT;

        /// <summary>False when a curve the reproduction needs uses a feature ExactCurve does not reproduce exactly.</summary>
        public bool IsExact { get; }

        /// <summary>Takes the body's state now. Main thread.</summary>
        public AtmosphereSnapshot(CelestialBody body)
        {
            captureUT = Planetarium.GetUniversalTime();
            inFlight = HighLogic.LoadedSceneIsFlight;
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
            temperatureSunMultCurve = Copy(body.atmosphereTemperatureSunMultCurve, ref exact);
            latitudeBiasCurve = Copy(body.latitudeTemperatureBiasCurve, ref exact);
            latitudeSunMultCurve = Copy(body.latitudeTemperatureSunMultCurve, ref exact);
            axialSunBiasCurve = Copy(body.axialTemperatureSunBiasCurve, ref exact);
            axialSunMultCurve = Copy(body.axialTemperatureSunMultCurve, ref exact);
            eccentricityBiasCurve = Copy(body.eccentricityTemperatureBiasCurve, ref exact);
            IsExact = exact;

            radius = body.Radius;
            rotates = body.rotates;
            rotationPeriod = body.rotationPeriod;
            bodyFrame = body.BodyFrame;
            bodyPosition = body.position;
            bodyUp = body.bodyTransform != null ? body.bodyTransform.up : Vector3.up;

            CelestialBody sun = Planetarium.fetch != null ? Planetarium.fetch.Sun : null;
            hasSun = sun != null && sun != body;
            if (!hasSun)
                return;
            sunPosition = sun.position;
            CelestialBody referencing = CelestialBody.GetBodyReferencing(body, sun);
            Orbit orbit = referencing != null ? referencing.orbit : null;
            if (orbit == null)
                return;
            trueAnomaly = orbit.trueAnomaly;
            eccentricity = orbit.eccentricity;
            orbitRadius = orbit.radius;
            orbitPeR = orbit.PeR;
            orbitApR = orbit.ApR;
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

        /// <summary>The temperature a prediction takes (FARAtmosphere.GameTemperature) at a latitude, longitude, altitude and time.</summary>
        public double GetTemperature(Vector3d latLonAltitude, double ut)
        {
            double altitude = latLonAltitude.z;
            double baseTemperature = GetBaseTemperature(altitude);
            if (!inFlight || !atmosphere || altitude > atmosphereDepth)
                return baseTemperature;
            if (!hasSun)
                return baseTemperature;

            double longitude = latLonAltitude.y;
            if (rotates && rotationPeriod != 0)
                longitude += 360.0 * (ut - captureUT) / rotationPeriod;
            Vector3d upAxis = SurfaceNVector(latLonAltitude.x, longitude);
            Vector3d toSun = sunPosition - WorldSurfacePosition(latLonAltitude.x, longitude, altitude);
            double distance = toSun.magnitude;
            Vector3d sunVector = distance > 0 ? toSun / distance : upAxis;
            double temperatureOffset = AtmosphereTemperatureOffset(sunVector, upAxis);
            return GetBaseTemperature(altitude) + Evaluate(temperatureSunMultCurve, (float)altitude) * temperatureOffset;
        }

        // CelestialBody.GetSurfaceNVector
        private Vector3d SurfaceNVector(double lat, double lon)
        {
            lat *= Math.PI / 180.0;
            lon *= Math.PI / 180.0;
            Vector3d r = Planetarium.SphericalVector(lat, lon);
            return bodyFrame.LocalToWorld(r).xzy;
        }

        // CelestialBody.GetWorldSurfacePosition
        private Vector3d WorldSurfacePosition(double lat, double lon, double alt)
        {
            double latRad = lat * (Math.PI / 180.0);
            double lonRad = lon * (Math.PI / 180.0);
            Vector3d relSurfacePosition = Planetarium.SphericalVector(latRad, lonRad).xzy * (radius + alt);
            return bodyFrame.LocalToWorld(relSurfacePosition.xzy).xzy + bodyPosition;
        }

        // The atmosphere temperature offset CelestialBody.GetAtmoThermalStats computes (getBodyFlux false).
        private double AtmosphereTemperatureOffset(Vector3d sunVector, Vector3d upAxis)
        {
            Vector3 up = bodyUp;
            double num = Vector3.Dot(sunVector, up);
            double num2 = Vector3.Dot(up, upAxis);
            double num3 = Math.Acos(num2);
            if (double.IsNaN(num3))
                num3 = num2 < 0.0 ? Math.PI : 0.0;
            double num5 = Math.Acos(num);
            if (double.IsNaN(num5))
                num5 = num < 0.0 ? Math.PI : 0.0;
            double num7 = (1.0 + Math.Cos(num5 - num3)) * 0.5;
            double num8 = (1.0 + Math.Cos(num5 + num3)) * 0.5;
            double num9 = num3;
            if (num3 > Math.PI / 2.0)
                num9 = Math.PI - num9;
            double num12 = (1.0 + (double)Vector3.Dot(sunVector,
                                                      Quaternion.AngleAxis(45f * Mathf.Sign((float)rotationPeriod), up) *
                                                      upAxis)) * 0.5;
            double num13 = num7 - num8;
            double num14;
            if (num13 > 0.001)
            {
                num14 = (num12 - num8) / num13;
                if (double.IsNaN(num14))
                    num14 = num12 > 0.5 ? 1.0 : 0.0;
            }
            else
            {
                num14 = num8 + num13 * 0.5;
            }

            if (!atmosphere)
                return 0.0;

            float num15 = (float)(Math.PI / 2.0 - num9);
            num15 *= 57.29578f;
            float time = ((float)trueAnomaly * 57.29578f + 360f) % 360f;
            double num16 = (double)Evaluate(latitudeBiasCurve, num15) +
                           (double)Evaluate(latitudeSunMultCurve, num15) * num14 +
                           (double)(Evaluate(axialSunBiasCurve, time) * Evaluate(axialSunMultCurve, num15));
            double num17 = eccentricity != 0.0
                               ? Evaluate(eccentricityBiasCurve, (float)((orbitRadius - orbitPeR) / (orbitApR - orbitPeR)))
                               : 0.0;
            return num16 + num17;
        }
    }
}
