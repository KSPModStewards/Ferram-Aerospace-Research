/*
Ferram Aerospace Research
=========================
Reflection bridge to AtmosphereAutopilot, for pushing cruise targets from the heatmap.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Collections;
using System.Reflection;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    /// <summary>
    /// Late-bound access to AtmosphereAutopilot so a heatmap cell can be turned into a cruise
    /// target. AA is an optional dependency and FAR does not reference it, following the same
    /// pattern as <see cref="SolverEnginesInterop" />.
    ///
    /// This only ever writes setpoints. It never engages an autopilot: clicking a map should not
    /// take control of a flying aircraft, and arming cruise stays the pilot's decision.
    /// </summary>
    internal static class AtmosphereAutopilotInterop
    {
        private static bool initialized;

        /// <summary>True if AA is present and every member below resolved.</summary>
        public static bool Available { get; private set; }

        private static PropertyInfo instanceProperty;
        private static MethodInfo getVesselModules;

        private static Type cruiseControllerType;
        private static Type progradeThrustType;
        private static Type speedSetpointType;
        private static Type speedTypeEnum;

        private static FieldInfo desiredAltitudeField;
        private static PropertyInfo delayedFieldValue;
        private static FieldInfo setpointField;
        private static FieldInfo setpointBackingField;
        private static FieldInfo speedTypeField;
        private static MethodInfo convertFromMps;
        private static ConstructorInfo speedSetpointCtor;
        private static object metersPerSecond;

        public static void Initialize()
        {
            if (initialized)
                return;
            initialized = true;

            try
            {
                Assembly aa = null;
                foreach (AssemblyLoader.LoadedAssembly a in AssemblyLoader.loadedAssemblies)
                    if (a.assembly.GetName().Name == "AtmosphereAutopilot")
                    {
                        aa = a.assembly;
                        break;
                    }

                if (aa == null)
                {
                    FARLogger.Info("AtmosphereAutopilot not loaded; heatmap cruise targeting unavailable");
                    return;
                }

                Type autopilotType = aa.GetType("AtmosphereAutopilot.AtmosphereAutopilot");
                cruiseControllerType = aa.GetType("AtmosphereAutopilot.CruiseController");
                progradeThrustType = aa.GetType("AtmosphereAutopilot.ProgradeThrustController");
                speedSetpointType = aa.GetType("AtmosphereAutopilot.SpeedSetpoint");
                speedTypeEnum = aa.GetType("AtmosphereAutopilot.SpeedType");
                Type delayedFieldType = aa.GetType("AtmosphereAutopilot.DelayedFieldFloat");

                if (autopilotType == null ||
                    cruiseControllerType == null ||
                    progradeThrustType == null ||
                    speedSetpointType == null ||
                    speedTypeEnum == null ||
                    delayedFieldType == null)
                {
                    FARLogger.Warning("AtmosphereAutopilot loaded but expected types missing; cruise targeting unavailable");
                    return;
                }

                instanceProperty = autopilotType.GetProperty("Instance",
                                                             BindingFlags.Public | BindingFlags.Static);
                getVesselModules = autopilotType.GetMethod("getVesselModules", new[] {typeof(Vessel)});

                desiredAltitudeField = cruiseControllerType.GetField("desired_altitude");
                delayedFieldValue = delayedFieldType.GetProperty("Value");
                setpointField = progradeThrustType.GetField("setpoint");

                // Writing `setpoint` alone does not stick: AA drives the controller from
                // setpoint_field (the GUI-backed DelayedFieldFloat) and copies it back over
                // setpoint, so a lone write is overwritten within a frame. Every one of AA's own
                // call sites sets the pair together. It is internal, hence NonPublic.
                setpointBackingField = progradeThrustType.GetField("setpoint_field",
                                                                   BindingFlags.NonPublic | BindingFlags.Instance);

                // The unit the pilot picked in AA's GUI. Feeding m/s into a field the user has set
                // to Mach or knots would silently command something wildly different.
                speedTypeField = progradeThrustType.GetField("type",
                                                             BindingFlags.NonPublic | BindingFlags.Instance);
                convertFromMps = speedSetpointType.GetMethod("convert_from_mps",
                                                             BindingFlags.Public | BindingFlags.Static);

                speedSetpointCtor = speedSetpointType.GetConstructor(new[]
                                                                     {
                                                                         speedTypeEnum, typeof(float), typeof(Vessel)
                                                                     });

                metersPerSecond = Enum.Parse(speedTypeEnum, "MetersPerSecond");

                Available = instanceProperty != null &&
                            getVesselModules != null &&
                            desiredAltitudeField != null &&
                            delayedFieldValue != null &&
                            setpointField != null &&
                            setpointBackingField != null &&
                            speedSetpointCtor != null &&
                            metersPerSecond != null;

                if (Available)
                    FARLogger.Info("AtmosphereAutopilot interop ready; heatmap cruise targeting enabled");
                else
                    FARLogger.Warning("AtmosphereAutopilot interop incomplete; cruise targeting unavailable");
            }
            catch (Exception e)
            {
                Available = false;
                FARLogger.Exception(e, "while binding AtmosphereAutopilot interop");
            }
        }

        private static object GetModule(Vessel vessel, Type moduleType)
        {
            object instance = instanceProperty.GetValue(null, null);
            if (instance == null)
                return null;

            if (!(getVesselModules.Invoke(instance, new object[] {vessel}) is IDictionary modules))
                return null;

            return modules.Contains(moduleType) ? modules[moduleType] : null;
        }

        /// <summary>
        /// Writes cruise altitude and speed targets for <paramref name="vessel" />.
        ///
        /// Does not arm anything — AA's cruise master stays where the pilot left it. The speed goes
        /// in as m/s, which AA compares against surface speed; the heatmap's value is true airspeed,
        /// so the two differ slightly by the body's rotation.
        /// </summary>
        /// <returns>A human-readable result, for display.</returns>
        public static string SetCruiseTarget(Vessel vessel, double speedMPerS, double altitudeM)
        {
            Initialize();
            if (!Available)
                return "AtmosphereAutopilot not available.";

            if (vessel == null)
                return "No vessel.";

            try
            {
                object cruise = GetModule(vessel, cruiseControllerType);
                object thrust = GetModule(vessel, progradeThrustType);

                if (cruise == null && thrust == null)
                    return "No AA modules on this vessel — open the AA window once to create them.";

                var set = new System.Text.StringBuilder("Set ");

                if (cruise != null)
                {
                    object field = desiredAltitudeField.GetValue(cruise);
                    if (field != null)
                    {
                        delayedFieldValue.SetValue(field, (float)altitudeM, null);
                        set.AppendFormat("alt {0:F0} m", altitudeM);
                    }
                }

                if (thrust != null)
                {
                    // Respect whatever unit the pilot has AA's speed field set to, rather than
                    // assuming m/s and writing a number that means something else on screen.
                    object type = metersPerSecond;
                    if (speedTypeField != null)
                        type = speedTypeField.GetValue(thrust) ?? metersPerSecond;

                    var value = (float)speedMPerS;
                    if (convertFromMps != null && !Equals(type, metersPerSecond))
                        value = (float)convertFromMps.Invoke(null, new[] {type, (object)value, vessel});

                    // Both, in this order — the struct is what the controller reads this frame, the
                    // field is what overwrites it on the next one and what the GUI shows.
                    object setpoint = speedSetpointCtor.Invoke(new[] {type, (object)value, vessel});
                    setpointField.SetValue(thrust, setpoint);

                    object backing = setpointBackingField.GetValue(thrust);
                    if (backing != null)
                        delayedFieldValue.SetValue(backing, value, null);

                    if (cruise != null)
                        set.Append(", ");
                    set.AppendFormat("speed {0:F0} ({1})", value, type);
                }

                set.Append(". Arm AA cruise yourself.");
                return set.ToString();
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "while setting AtmosphereAutopilot cruise target");
                return "Failed to set cruise target — see KSP.log.";
            }
        }
    }
}
