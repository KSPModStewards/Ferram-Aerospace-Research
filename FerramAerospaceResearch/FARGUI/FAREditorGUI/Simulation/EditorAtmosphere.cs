/*
Ferram Aerospace Research
=========================
Editor-side atmosphere that reproduces the flight scene's solar/latitude temperature term.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    /// <summary>
    /// Atmospheric conditions for an editor sweep, matched to what the vessel will actually meet in
    /// flight.
    ///
    /// The naive editor answer — body.GetTemperature(altitude), which is what FARAtmosphere,
    /// FlightGlobals.getExternalTemperature and AJE's EngineThermodynamics.AmbientAtAltitude all
    /// return — is the bare altitude curve. Flight is not that: FlightIntegrator sets
    /// vessel.atmosphericTemperature from CelestialBody.GetFullTemperature, which adds a latitude
    /// and solar heating term on top:
    ///
    ///     GetFullTemperature(alt, offset) = GetTemperature(alt)
    ///                                     + atmosphereTemperatureSunMultCurve.Evaluate(alt) * offset
    ///
    /// Measured against flight at the KSC, the bare curve runs ~25 K (8%) cold near the ground. That
    /// matters more than it sounds: colder air is denser, so it inflates dynamic pressure and drops
    /// the speed of sound, and at Mach 3 the stagnation rise turns 25 K into ~100 K at the
    /// compressor face — landing squarely on the turbine temperature margin that sets a jet's thrust.
    /// It was worth 3x on predicted thrust at sea level.
    ///
    /// The sun multiplier curve fades the term out with altitude, which is why the error vanishes by
    /// 20 km and why the discrepancy tracked density rather than Mach.
    ///
    /// Only the sweep uses this; FAR's other tabs still call FARAtmosphere directly and are
    /// unaffected.
    /// </summary>
    internal static class EditorAtmosphere
    {
        /// <summary>
        /// The solar/latitude temperature term for this body and altitude, K.
        ///
        /// Evaluated at the active space centre's latitude and the current universal time, so it
        /// reflects where the craft will launch from and whether it is presently day or night there
        /// — the two things that determine the term and that a craft sitting in the SPH otherwise
        /// has no way to know.
        /// </summary>
        public static double SolarTemperatureOffset(CelestialBody body, double alt)
        {
            if (body == null || !body.atmosphere)
                return 0;

            CelestialBody sun = Planetarium.fetch != null ? Planetarium.fetch.Sun : null;
            if (sun == null)
                return 0;

            SpaceCenter center = SpaceCenter.Instance;
            double lat = center != null ? center.Latitude : 0;
            double lon = center != null ? center.Longitude : 0;

            try
            {
                // Replicate FlightIntegrator exactly. Do NOT use GetFullTemperature(Vector3d): that
                // overload builds sunDot from an un-normalized scaled-space vector (~1e9 magnitude),
                // which in the editor yields a wildly wrong offset — it drove the speed of sound to
                // ~5800 m/s. FI instead uses a normalized world-space sun direction and a unit up.
                Vector3d up = body.GetSurfaceNVector(lat, lon);
                Vector3d sunDir = (sun.position - body.position).normalized;
                double sunDot = Vector3d.Dot(sunDir, up);

                body.GetAtmoThermalStats(false,
                                         sun,
                                         sunDir,
                                         sunDot,
                                         up,
                                         alt,
                                         out double offset,
                                         out double _,
                                         out double _);

                double term = body.atmosphereTemperatureSunMultCurve.Evaluate((float)alt) * offset;

                // The solar term is physically a few tens of K. Anything outside a wide sane band is
                // a computation gone wrong, not real weather — fall back to the bare curve rather
                // than poison density and the speed of sound.
                if (double.IsNaN(term) || double.IsInfinity(term) || Math.Abs(term) > 300)
                    return 0;

                return term;
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "while computing editor solar temperature offset");
                return 0;
            }
        }

        /// <summary>
        /// Gas properties at an altitude, with the flight scene's solar term applied.
        ///
        /// Pressure, adiabatic index and gas constant still come from FARAtmosphere so that
        /// custom-atmosphere mods keep working; only the temperature is corrected, by adding KSP's
        /// term to whatever base FARAtmosphere reports rather than replacing it outright.
        /// </summary>
        public static GasProperties GetGasProperties(CelestialBody body, double alt)
        {
            var latLonAlt = new Vector3d(0, 0, alt);
            double ut = Planetarium.GetUniversalTime();

            double pressure = FARAtmosphere.GetPressure(body, latLonAlt, ut);
            double temperature = FARAtmosphere.GetTemperature(body, latLonAlt, ut) +
                                 SolarTemperatureOffset(body, alt);
            double gamma = FARAtmosphere.GetAdiabaticIndex(body, latLonAlt, ut);
            double gasConstant = FARAtmosphere.GetGasConstant(body, latLonAlt, ut);

            return new GasProperties(pressure, temperature, gamma, gasConstant);
        }
    }
}
