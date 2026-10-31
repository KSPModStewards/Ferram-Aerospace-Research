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
using System.Reflection;
using UnityEngine;

namespace FerramAerospaceResearch
{
    public static partial class FARAtmosphere
    {
        public static Vector3d CurrentPosition(this Vessel vessel)
        {
            return new Vector3d(vessel.latitude, vessel.longitude, vessel.altitude);
        }

        public static Vector3d CurrentPosition(this Vessel vessel, double altitude)
        {
            return new Vector3d(vessel.latitude, vessel.longitude, altitude);
        }

        internal class DelegateDispatcher<Arg1, Arg2, Arg3, Ret>
        {
            private Func<Arg1, Arg2, Arg3, Ret> function;

            public DelegateDispatcher(Func<Arg1, Arg2, Arg3, Ret> function, string name)
            {
                this.function = function;
                Default = function;
                Name = name;
            }

            public Func<Arg1, Arg2, Arg3, Ret> Function
            {
                get { return function; }
                set
                {
                    // if passed null, reset back to default so that function is never null
                    if (value is null)
                    {
                        FARLogger.InfoFormat("FARAtmosphere.{0}: attempted to set null function, using default {1}",
                                             Name,
                                             Default.Method.Name);
                        function = Default;
                    }
                    else
                    {
                        MethodInfo method = value.Method;
                        FARLogger.InfoFormat("FARAtmosphere.{0}: setting function to {1}.{2} from {3}",
                                             Name,
                                             method.DeclaringType?.Name ?? "(global)",
                                             method.Name,
                                             method.Module.Name);
                        function = value;
                    }
                }
            }

            public readonly Func<Arg1, Arg2, Arg3, Ret> Default;
            public readonly string Name;

            public bool IsCustom
            {
                get { return !ReferenceEquals(function, Default); }
            }

            public Ret Invoke(Arg1 arg1, Arg2 arg2, Arg3 arg3)
            {
                // swallow all exceptions so that simulation can still progress
                try
                {
                    return function(arg1, arg2, arg3);
                }
                catch (Exception e)
                {
                    FARLogger.InfoFormat("FARAtmosphere.{0}: exception {1}\n{2}", Name, e.Message, e.StackTrace);
                    return Default(arg1, arg2, arg3);
                }
            }
        }
    }

    public readonly struct GasProperties
    {
        public readonly double Pressure;
        public readonly double Temperature;
        public readonly double Density;
        public readonly double AdiabaticIndex;
        public readonly double GasConstant;

        public double SpeedOfSound
        {
            get { return Math.Sqrt(AdiabaticIndex * GasConstant * Temperature); }
        }

        public GasProperties(double pressure, double temperature, double adiabaticIndex, double gasConstant)
        {
            Pressure = pressure;
            Temperature = temperature;
            Density = Pressure / (gasConstant * Temperature);
            AdiabaticIndex = adiabaticIndex;
            GasConstant = gasConstant;
        }
    }
}

// break namespace so that type aliases can be declared
namespace FerramAerospaceResearch
{
    // delegate aliases to avoid writing them all out, use System.Func<> instead of specific delegates for easier usage
    // with reflection
    using WindDelegate = Func<CelestialBody, Part, Vector3, Vector3>;
    using WindDispatcher = FARAtmosphere.DelegateDispatcher<CelestialBody, Part, Vector3, Vector3>;
    using PropertyDelegate = Func<CelestialBody, Vector3d, double, double>;
    using PropertyDispatcher = FARAtmosphere.DelegateDispatcher<CelestialBody, Vector3d, double, double>;

    /// <summary>
    ///     Entry point where another assembly can specify a function to calculate the wind and atmospheric properties.
    ///     The rest of the simulation uses this class to get the wind and includes it in the
    ///     total airspeed for the simulation.
    /// </summary>
    public static partial class FARAtmosphere
    {
        /// <summary>
        ///     Wind function
        /// </summary>
        private static readonly WindDispatcher windFunction = new((body, part, pos) => Vector3.zero, "Wind");

        /// <summary>
        ///     Simulation pressure function
        /// </summary>
        private static readonly PropertyDispatcher pressureFunction =
            // KSP returns pressure in kPa
            new(((body, latLonAltitude, ut) => body.GetPressure(latLonAltitude.z) * 1000), "Pressure");

        /// <summary>
        ///     Simulation temperature function
        /// </summary>
        private static readonly PropertyDispatcher temperatureFunction =
            new(((body, latLonAltitude, ut) => body.GetTemperature(latLonAltitude.z)), "Temperature");

        /// <summary>
        ///     Simulation adiabatic index function
        /// </summary>
        private static readonly PropertyDispatcher adiabaticIndexFunction =
            new(((body, latLonAltitude, ut) => body.atmosphereAdiabaticIndex), "AdiabaticIndex");

        /// <summary>
        ///     Simulation gas constant function
        /// </summary>
        private static readonly PropertyDispatcher gasConstantFunction =
            new(((body, latLonAltitude, ut) => PhysicsGlobals.IdealGasConstant / body.atmosphereMolarMass),
                "GasConstant");

        public static bool IsCustom { get; private set; }

        private static void UpdateIsCustom()
        {
            IsCustom = windFunction.IsCustom ||
                       pressureFunction.IsCustom ||
                       temperatureFunction.IsCustom ||
                       adiabaticIndexFunction.IsCustom ||
                       gasConstantFunction.IsCustom;
        }

        /// <summary>
        ///     Calculates the wind's intensity using the specified wind function.
        ///     If any exception occurs, it is suppressed and Vector3.zero is returned.
        ///     This function will never throw, (although it will spam the log).
        /// </summary>
        /// <param name="body">Current celestial body</param>
        /// <param name="part">Current part</param>
        /// <param name="position">Position of the part</param>
        /// <returns>Wind as a Vector3 (unit is m/s)</returns>
        public static Vector3 GetWind(CelestialBody body, Part part, Vector3 position)
        {
            return windFunction.Invoke(body, part, position);
        }

        // ReSharper disable once UnusedMember.Global
        /// <summary>
        ///     "Set" method for the wind function.
        /// </summary>
        /// <param name="newFunction">See <see cref="GetWind"/> for parameter meanings</param>
        public static void SetWindFunction(WindDelegate newFunction)
        {
            windFunction.Function = newFunction;
            UpdateIsCustom();
        }

        /// <summary>
        ///     Calculates pressure at the specified universal time.
        /// </summary>
        /// <param name="body">Body to evaluate properties at</param>
        /// <param name="latLonAltitude">Latitude, longitude and altitude</param>
        /// <param name="ut">Universal time</param>
        /// <returns>Pressure in Pa</returns>
        public static double GetPressure(CelestialBody body, Vector3d latLonAltitude, double ut)
        {
            return pressureFunction.Invoke(body, latLonAltitude, ut);
        }

        public static double GetPressure(Vessel vessel, double ut)
        {
            return GetPressure(vessel.mainBody, vessel.CurrentPosition(), ut);
        }

        public static double GetPressure(Vessel vessel)
        {
            return GetPressure(vessel, Planetarium.GetUniversalTime());
        }

        // ReSharper disable once UnusedMember.Global
        /// <summary>
        ///     "Set" method for the simulated pressure function.
        /// </summary>
        /// <param name="newFunction">See <see cref="GetPressure(CelestialBody, Vector3d, double)"/> for parameter meanings</param>
        public static void SetPressureFunction(PropertyDelegate newFunction)
        {
            pressureFunction.Function = newFunction;
            UpdateIsCustom();
        }

        /// <summary>
        ///     Calculates temperature at the specified universal time.
        /// </summary>
        /// <param name="body">Body to evaluate properties at</param>
        /// <param name="latLonAltitude">Latitude, longitude and altitude</param>
        /// <param name="ut">Universal time</param>
        /// <returns>Temperature in K</returns>
        public static double GetTemperature(CelestialBody body, Vector3d latLonAltitude, double ut)
        {
            return temperatureFunction.Invoke(body, latLonAltitude, ut);
        }

        public static double GetTemperature(Vessel vessel, double ut)
        {
            return GetTemperature(vessel.mainBody, vessel.CurrentPosition(), ut);
        }

        public static double GetTemperature(Vessel vessel)
        {
            return GetTemperature(vessel, Planetarium.GetUniversalTime());
        }

        // ReSharper disable once UnusedMember.Global
        /// <summary>
        ///     "Set" method for the simulated temperature function.
        /// </summary>
        /// <param name="newFunction">See <see cref="GetTemperature(CelestialBody, Vector3d, double)"/> for parameter meanings</param>
        public static void SetTemperatureFunction(PropertyDelegate newFunction)
        {
            temperatureFunction.Function = newFunction;
            UpdateIsCustom();
        }

        /// <summary>
        ///     Calculates adiabatic index at the specified universal time.
        /// </summary>
        /// <param name="body">Body to evaluate properties at</param>
        /// <param name="latLonAltitude">Latitude, longitude and altitude</param>
        /// <param name="ut">Universal time</param>
        /// <returns>Adiabatic index</returns>
        public static double GetAdiabaticIndex(CelestialBody body, Vector3d latLonAltitude, double ut)
        {
            return adiabaticIndexFunction.Invoke(body, latLonAltitude, ut);
        }

        public static double GetAdiabaticIndex(Vessel vessel, double ut)
        {
            return GetAdiabaticIndex(vessel.mainBody, vessel.CurrentPosition(), ut);
        }

        public static double GetAdiabaticIndex(Vessel vessel)
        {
            return GetAdiabaticIndex(vessel, Planetarium.GetUniversalTime());
        }

        // ReSharper disable once UnusedMember.Global
        /// <summary>
        ///     "Set" method for the simulated adiabatic index function.
        /// </summary>
        /// <param name="newFunction">See <see cref="GetAdiabaticIndex(CelestialBody, Vector3d, double)"/> for parameter meanings</param>
        public static void SetAdiabaticIndexFunction(PropertyDelegate newFunction)
        {
            adiabaticIndexFunction.Function = newFunction;
            UpdateIsCustom();
        }

        /// <summary>
        ///     Calculates gas constant at the specified universal time.
        /// </summary>
        /// <param name="body">Body to evaluate properties at</param>
        /// <param name="latLonAltitude">Latitude, longitude and altitude</param>
        /// <param name="ut">Universal time</param>
        /// <returns>Gas constant in J/kg/K</returns>
        public static double GetGasConstant(CelestialBody body, Vector3d latLonAltitude, double ut)
        {
            return gasConstantFunction.Invoke(body, latLonAltitude, ut);
        }

        public static double GetGasConstant(Vessel vessel, double ut)
        {
            return GetGasConstant(vessel.mainBody, vessel.CurrentPosition(), ut);
        }

        public static double GetGasConstant(Vessel vessel)
        {
            return GetGasConstant(vessel, Planetarium.GetUniversalTime());
        }

        // ReSharper disable once UnusedMember.Global
        /// <summary>
        ///     "Set" method for the simulated gas constant function.
        /// </summary>
        /// <param name="newFunction">See <see cref="GetGasConstant(CelestialBody, Vector3d, double)"/> for parameter meanings</param>
        public static void SetGasConstantFunction(PropertyDelegate newFunction)
        {
            gasConstantFunction.Function = newFunction;
            UpdateIsCustom();
        }

        /// <summary>
        ///     Calculates static gas properties at the specified universal time.
        /// </summary>
        /// <param name="body">Body to evaluate properties at</param>
        /// <param name="latLonAltitude">Latitude, longitude and altitude</param>
        /// <param name="ut">Universal time</param>
        /// <returns>Static gas properties</returns>
        public static GasProperties GetGasProperties(CelestialBody body, Vector3d latLonAltitude, double ut)
        {
            return new(pressure: GetPressure(body, latLonAltitude, ut),
                       temperature: GetTemperature(body, latLonAltitude, ut),
                       adiabaticIndex: GetAdiabaticIndex(body, latLonAltitude, ut),
                       gasConstant: GetGasConstant(body, latLonAltitude, ut));
        }

        public static GasProperties GetGasProperties(Vessel vessel, double altitude, double ut)
        {
            return GetGasProperties(vessel.mainBody, vessel.CurrentPosition(altitude), ut);
        }

        public static GasProperties GetGasProperties(Vessel vessel, double ut)
        {
            return GetGasProperties(vessel.mainBody, vessel.CurrentPosition(), ut);
        }

        public static GasProperties GetGasProperties(Vessel vessel)
        {
            return GetGasProperties(vessel, Planetarium.GetUniversalTime());
        }
    }
}
