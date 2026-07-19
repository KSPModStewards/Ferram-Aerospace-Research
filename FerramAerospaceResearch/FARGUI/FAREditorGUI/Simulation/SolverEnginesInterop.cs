/*
Ferram Aerospace Research
=========================
Reflection bridge to SolverEngines / AJE for editor-side engine sampling.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Reflection;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    /// <summary>
    /// Late-bound access to SolverEngines / AJE, which FAR deliberately does not reference
    /// at compile time (both are optional dependencies; see FARAeroUtil.AJELoaded and the
    /// string-based module lookups in GeometryPartModule / VesselIntakeRamDrag).
    ///
    /// Everything needed is public except ModuleEnginesSolver.engineSolver, which is
    /// protected. The thrust path itself (UpdateInletEffects + UpdateSolver) is public and
    /// touches no vessel state — SolverEngines.EngineModule carries the comment
    /// "ferram4: separate out so function can be called separately for editor sims" above
    /// UpdateInletEffects, and AJE's own part tooltip (AJEJet.GetStaticThrustInfo) already
    /// drives UpdateSolver off-design with no vessel present.
    ///
    /// Members are resolved once on first use. If anything is missing — SolverEngines absent,
    /// or a signature changed — Available stays false and callers fall back.
    /// </summary>
    internal static class SolverEnginesInterop
    {
        private static bool initialized;

        /// <summary>True once every member below resolved successfully.</summary>
        public static bool Available { get; private set; }

        private static Type moduleEnginesSolverType;
        private static Type ajeInletType;
        private static Type thermoType;

        private static ConstructorInfo thermoCtor;
        private static PropertyInfo thermoPressure;
        private static MethodInfo thermoChangeReferenceFrame;

        private static FieldInfo engineSolverField;
        private static FieldInfo engineNeedArea;
        private static FieldInfo engineUseZaxis;
        private static FieldInfo engineMaxTemp;
        private static FieldInfo engineAutoignitionTemp;
        private static MethodInfo solverGetEngineTemp;
        private static MethodInfo engineCreateIfNecessary;
        private static MethodInfo engineUpdateInletEffects;
        private static MethodInfo engineUpdateSolver;

        private static MethodInfo solverGetThrust;
        private static MethodInfo solverGetFuelFlow;
        private static MethodInfo solverGetSfc;
        private static MethodInfo solverGetIsp;
        private static MethodInfo solverGetRunning;

        private static FieldInfo inletArea;
        private static FieldInfo inletOverallTpr;
        private static MethodInfo inletUpdateOverallTpr;

        private static Type flightSysType;
        private static PropertyInfo flightSysInletArea;
        private static PropertyInfo flightSysEngineArea;
        private static PropertyInfo flightSysAreaRatio;
        private static PropertyInfo flightSysOverallTpr;
        private static FieldInfo flightSysAmbientTherm;
        private static FieldInfo flightSysInletTherm;
        private static PropertyInfo thermoTemperature;

        // Reused to avoid allocating an args array per sample; sampling is single-threaded.
        private static readonly object[] args1 = new object[1];
        private static readonly object[] args2 = new object[2];
        private static readonly object[] args3 = new object[3];
        private static readonly object[] args7 = new object[7];

        public static void Initialize()
        {
            if (initialized)
                return;
            initialized = true;

            try
            {
                Assembly solverEngines = null;
                foreach (AssemblyLoader.LoadedAssembly a in AssemblyLoader.loadedAssemblies)
                    if (a.assembly.GetName().Name == "SolverEngines")
                    {
                        solverEngines = a.assembly;
                        break;
                    }

                if (solverEngines == null)
                {
                    FARLogger.Info("SolverEngines not loaded; editor engine sampling unavailable");
                    return;
                }

                moduleEnginesSolverType = solverEngines.GetType("SolverEngines.ModuleEnginesSolver");
                ajeInletType = solverEngines.GetType("SolverEngines.AJEInlet");
                thermoType = solverEngines.GetType("SolverEngines.EngineThermodynamics");

                if (moduleEnginesSolverType == null || ajeInletType == null || thermoType == null)
                {
                    FARLogger.Warning("SolverEngines loaded but expected types missing; engine sampling unavailable");
                    return;
                }

                // EngineThermodynamics(double P, double T, double far, double massRatio).
                // Built directly rather than via AmbientAtAltitude so the engine side shares
                // FARAtmosphere's pressure/temperature with the aero side — AmbientAtAltitude
                // reads FlightGlobals.getStaticPressure, which diverges under custom atmospheres.
                thermoCtor = thermoType.GetConstructor(new[]
                                                       {
                                                           typeof(double), typeof(double), typeof(double),
                                                           typeof(double)
                                                       });
                thermoPressure = thermoType.GetProperty("P");
                thermoChangeReferenceFrame = thermoType.GetMethod("ChangeReferenceFrame", new[] {typeof(double)});

                engineSolverField = moduleEnginesSolverType.GetField("engineSolver",
                                                                     BindingFlags.NonPublic | BindingFlags.Instance);
                engineNeedArea = moduleEnginesSolverType.GetField("Need_Area");
                engineUseZaxis = moduleEnginesSolverType.GetField("useZaxis");
                engineMaxTemp = moduleEnginesSolverType.GetField("maxEngineTemp");
                engineAutoignitionTemp = moduleEnginesSolverType.GetField("autoignitionTemp");
                engineCreateIfNecessary = moduleEnginesSolverType.GetMethod("CreateEngineIfNecessary",
                                                                            Type.EmptyTypes);
                engineUpdateInletEffects = moduleEnginesSolverType.GetMethod("UpdateInletEffects",
                                                                             new[]
                                                                             {
                                                                                 thermoType, typeof(double),
                                                                                 typeof(double)
                                                                             });
                engineUpdateSolver = moduleEnginesSolverType.GetMethod("UpdateSolver",
                                                                       new[]
                                                                       {
                                                                           thermoType, typeof(double),
                                                                           typeof(Vector3d), typeof(double),
                                                                           typeof(bool), typeof(bool), typeof(bool)
                                                                       });

                Type engineSolverType = engineSolverField?.FieldType;
                if (engineSolverType != null)
                {
                    solverGetThrust = engineSolverType.GetMethod("GetThrust", Type.EmptyTypes);
                    solverGetFuelFlow = engineSolverType.GetMethod("GetFuelFlow", Type.EmptyTypes);
                    solverGetSfc = engineSolverType.GetMethod("GetSFC", Type.EmptyTypes);
                    solverGetIsp = engineSolverType.GetMethod("GetIsp", Type.EmptyTypes);
                    solverGetRunning = engineSolverType.GetMethod("GetRunning", Type.EmptyTypes);
                    solverGetEngineTemp = engineSolverType.GetMethod("GetEngineTemp", Type.EmptyTypes);
                }

                inletArea = ajeInletType.GetField("Area");
                inletOverallTpr = ajeInletType.GetField("overallTPR");
                inletUpdateOverallTpr = ajeInletType.GetMethod("UpdateOverallTPR",
                                                               new[] {typeof(Vector3), typeof(double)});

                // SolverFlightSys is the vessel-level inlet accounting the editor deck had to
                // reconstruct. Reading its live values lets the two be compared directly instead of
                // guessing which term diverges. Diagnostic only — absence must not disable sampling,
                // so these are bound separately from the Available check below.
                flightSysType = solverEngines.GetType("SolverEngines.SolverFlightSys");
                if (flightSysType != null)
                {
                    flightSysInletArea = flightSysType.GetProperty("InletArea");
                    flightSysEngineArea = flightSysType.GetProperty("EngineArea");
                    flightSysAreaRatio = flightSysType.GetProperty("AreaRatio");
                    flightSysOverallTpr = flightSysType.GetProperty("OverallTPR");
                    flightSysAmbientTherm = flightSysType.GetField("AmbientTherm");
                    flightSysInletTherm = flightSysType.GetField("InletTherm");
                }

                thermoTemperature = thermoType.GetProperty("T");

                Available = thermoCtor != null &&
                            thermoPressure != null &&
                            thermoChangeReferenceFrame != null &&
                            engineSolverField != null &&
                            engineNeedArea != null &&
                            engineUseZaxis != null &&
                            engineCreateIfNecessary != null &&
                            engineUpdateInletEffects != null &&
                            engineUpdateSolver != null &&
                            solverGetThrust != null &&
                            solverGetFuelFlow != null &&
                            solverGetSfc != null &&
                            solverGetIsp != null &&
                            solverGetRunning != null &&
                            inletArea != null &&
                            inletOverallTpr != null &&
                            inletUpdateOverallTpr != null;

                if (Available)
                    FARLogger.Info("SolverEngines interop ready; editor engine sampling enabled");
                else
                    FARLogger.Warning("SolverEngines interop incomplete; engine sampling unavailable");
            }
            catch (Exception e)
            {
                Available = false;
                FARLogger.Exception(e, "while binding SolverEngines interop");
            }
        }

        public static bool IsSolverEngine(PartModule m)
        {
            return Available && m != null && moduleEnginesSolverType.IsInstanceOfType(m);
        }

        public static bool IsAJEInlet(PartModule m)
        {
            return Available && m != null && ajeInletType.IsInstanceOfType(m);
        }

        /// <summary>Boxed EngineThermodynamics from absolute pressure (Pa) and temperature (K).</summary>
        public static object MakeThermo(double pressurePa, double temperatureK)
        {
            return thermoCtor.Invoke(new object[] {pressurePa, temperatureK, 0d, 1d});
        }

        /// <summary>
        /// Total conditions in a frame moving at <paramref name="speed" /> m/s. Returns a new box;
        /// the input is not modified.
        /// </summary>
        public static object ChangeReferenceFrame(object thermo, double speed)
        {
            args1[0] = speed;
            return thermoChangeReferenceFrame.Invoke(thermo, args1);
        }

        public static double GetPressure(object thermo)
        {
            return (double)thermoPressure.GetValue(thermo, null);
        }

        /// <summary>Sets total pressure on a boxed EngineThermodynamics, mutating the box in place.</summary>
        public static void SetPressure(object thermo, double pressurePa)
        {
            thermoPressure.SetValue(thermo, pressurePa, null);
        }

        public static void CreateEngineIfNecessary(PartModule engine)
        {
            engineCreateIfNecessary.Invoke(engine, null);
        }

        /// <summary>Intake area this engine wants, m^2. Populated in the editor by EngineModule.Start.</summary>
        public static float GetNeedArea(PartModule engine)
        {
            return (float)engineNeedArea.GetValue(engine);
        }

        /// <summary>
        /// True if thrust runs along the thrust transform's -Z, false for -Y. Mirrors the axis
        /// choice in EngineModule.FixedUpdate.
        /// </summary>
        public static bool GetUseZaxis(PartModule engine)
        {
            return (bool)engineUseZaxis.GetValue(engine);
        }

        /// <summary>
        /// Ratio of the solver's engine temperature to the part's maximum, after the last
        /// UpdateSolver. In flight, EngineModule.UpdateTemp explodes the part once this exceeds 1 —
        /// which is what stops a turbojet at high Mach, where ram heating drives the compressor-face
        /// temperature past the limit. Returns 0 (no limit known) if either member is unbound or the
        /// max is non-positive, so an old SolverEngines simply skips the check rather than breaking.
        ///
        /// Bound separately from Available: sampling must still work without it.
        /// </summary>
        /// <summary>
        /// The fuel's autoignition temperature, K, or a non-positive value if unknown/disabled.
        /// A compressor-less engine (ramjet) only combusts once ram heating brings the inlet above
        /// this — the check flight applies via CanAutoRestart and that direct solver sampling
        /// otherwise skips.
        /// </summary>
        public static double GetAutoignitionTemp(PartModule engine)
        {
            if (engineAutoignitionTemp == null)
                return 0;

            var t = (float)engineAutoignitionTemp.GetValue(engine);
            return float.IsInfinity(t) ? 0 : t;
        }

        public static double GetThermalRatio(PartModule engine)
        {
            if (engineMaxTemp == null || solverGetEngineTemp == null)
                return 0;

            var maxTemp = (double)engineMaxTemp.GetValue(engine);
            if (maxTemp <= 0)
                return 0;

            object solver = engineSolverField.GetValue(engine);
            if (solver == null)
                return 0;

            var temp = (double)solverGetEngineTemp.Invoke(solver, null);
            return temp / maxTemp;
        }

        public static void UpdateInletEffects(PartModule engine, object inletTherm, double areaRatio, double tpr)
        {
            args3[0] = inletTherm;
            args3[1] = areaRatio;
            args3[2] = tpr;
            engineUpdateInletEffects.Invoke(engine, args3);
        }

        /// <summary>
        /// Drives the engine's solver to a flight condition. Returns false if the solver threw.
        ///
        /// AJE does not merely return NaN at conditions it cannot resolve — it throws out of
        /// Math.Sign, which reflection then rewraps as TargetInvocationException. Off-design
        /// sampling reaches conditions the flight path never would, so this is expected rather than
        /// exceptional, and it must not escape into a GUI render.
        /// </summary>
        public static bool UpdateSolver(
            PartModule engine,
            object ambientTherm,
            double altitude,
            Vector3d velocity,
            double mach,
            bool ignited,
            bool oxygen,
            bool underwater
        )
        {
            args7[0] = ambientTherm;
            args7[1] = altitude;
            args7[2] = velocity;
            args7[3] = mach;
            args7[4] = ignited;
            args7[5] = oxygen;
            args7[6] = underwater;

            try
            {
                engineUpdateSolver.Invoke(engine, args7);
                return true;
            }
            catch (TargetInvocationException e)
            {
                FARLogger.DebugFormat("Engine solver failed at M {0}, alt {1}: {2}",
                                      mach,
                                      altitude,
                                      e.InnerException?.Message ?? e.Message);
                return false;
            }
        }

        /// <summary>
        /// Reads the solver outputs left by the last UpdateSolver call. Thrust is newtons,
        /// fuel flow kg/s, SFC in AJE's kg/kgf-hr, Isp seconds.
        /// </summary>
        public static bool TryGetSolverOutputs(
            PartModule engine,
            out double thrustN,
            out double fuelFlow,
            out double sfc,
            out double isp,
            out bool running
        )
        {
            thrustN = 0;
            fuelFlow = 0;
            sfc = 0;
            isp = 0;
            running = false;

            object solver = engineSolverField.GetValue(engine);
            if (solver == null)
                return false;

            thrustN = (double)solverGetThrust.Invoke(solver, null);
            fuelFlow = (double)solverGetFuelFlow.Invoke(solver, null);
            sfc = (double)solverGetSfc.Invoke(solver, null);
            isp = (double)solverGetIsp.Invoke(solver, null);
            running = (bool)solverGetRunning.Invoke(solver, null);
            return true;
        }

        public static float GetInletArea(PartModule inlet)
        {
            return (float)inletArea.GetValue(inlet);
        }

        public static float GetInletOverallTPR(PartModule inlet)
        {
            return (float)inletOverallTpr.GetValue(inlet);
        }

        /// <summary>
        /// Reads the live vessel-level inlet accounting from SolverEngines' SolverFlightSys, so the
        /// editor deck's reconstruction of the same arithmetic can be checked against it. Flight
        /// only; returns false if the VesselModule is absent or the properties did not bind.
        /// </summary>
        public static bool TryGetFlightInletState(
            Vessel vessel,
            out double inletArea,
            out double engineArea,
            out double areaRatio,
            out double overallTpr
        )
        {
            inletArea = engineArea = areaRatio = overallTpr = 0;

            if (vessel == null || flightSysType == null || flightSysInletArea == null)
                return false;

            Component sys = vessel.GetComponent(flightSysType);
            if (sys == null)
                return false;

            inletArea = (double)flightSysInletArea.GetValue(sys, null);
            engineArea = (double)flightSysEngineArea.GetValue(sys, null);
            areaRatio = (double)flightSysAreaRatio.GetValue(sys, null);
            overallTpr = (double)flightSysOverallTpr.GetValue(sys, null);
            return true;
        }

        public static double GetTemperature(object thermo)
        {
            return (double)thermoTemperature.GetValue(thermo, null);
        }

        /// <summary>
        /// Reads the live ambient and inlet thermodynamic state SolverFlightSys is feeding the
        /// engines. The editor deck builds the same two from FARAtmosphere; comparing them shows
        /// whether a thrust discrepancy comes from the conditions being handed to the solver rather
        /// than from the solver itself. Flight only.
        /// </summary>
        public static bool TryGetFlightThermo(
            Vessel vessel,
            out double ambientP,
            out double ambientT,
            out double inletP,
            out double inletT
        )
        {
            ambientP = ambientT = inletP = inletT = 0;

            if (vessel == null || flightSysType == null || flightSysAmbientTherm == null)
                return false;

            Component sys = vessel.GetComponent(flightSysType);
            if (sys == null)
                return false;

            object ambient = flightSysAmbientTherm.GetValue(sys);
            object inlet = flightSysInletTherm.GetValue(sys);
            if (ambient == null || inlet == null)
                return false;

            ambientP = GetPressure(ambient);
            ambientT = GetTemperature(ambient);
            inletP = GetPressure(inlet);
            inletT = GetTemperature(inlet);
            return true;
        }

        /// <summary>
        /// Recomputes the inlet's overallTPR for a flight condition. Safe in the editor —
        /// it only reads intakeTransform.forward, not vessel state. Note the sibling
        /// AJEInlet.UsableArea() is NOT editor-safe: it calls IntakeActive() -> InAtmosphere(),
        /// which dereferences part.vessel.altitude.
        /// </summary>
        public static void UpdateInletOverallTPR(PartModule inlet, Vector3 velocity, double mach)
        {
            args2[0] = velocity;
            args2[1] = mach;
            inletUpdateOverallTpr.Invoke(inlet, args2);
        }
    }
}
