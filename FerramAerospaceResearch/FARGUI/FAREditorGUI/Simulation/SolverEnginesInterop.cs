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
using System.Linq;
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
        private static FieldInfo engineThrustUpperLimit; // may be null on older SolverEngines: cap is then a no-op
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

        // --- parallel engine sampling: drive per-thread solver clones directly ---
        // The serial path above mutates the one live ModuleEnginesSolver per engine. The parallel
        // sweep instead clones each engine's EngineSolver per worker slot (MemberwiseClone — every
        // performance field is a value type or the value-type EngineThermodynamics struct, so a
        // shallow copy is independent, apart from the curves CloneSolver copies and RealFuels' random
        // draws, see rfSolverType) and drives the clone's own SetEngineState/SetFreestreamAndInlet/
        // CalculatePerformance, reading the read-only module scalars off a snapshot. Resolved
        // separately from Available so an old SolverEngines still samples serially.
        public static bool ParallelAvailable { get; private set; }

        private static MethodInfo objectMemberwiseClone;
        private static MethodInfo solverSetEngineState;
        private static MethodInfo solverSetFreestreamAndInlet;
        private static MethodInfo solverCalculatePerformance;
        private static FieldInfo engineFlowMult;
        private static FieldInfo engineIspMult;
        private static FieldInfo engineMultFlow;
        private static FieldInfo engineMultIsp;
        private static FieldInfo engineLastPropFrac;

        // RealFuels' SolverRF draws a run-to-run variance from one static System.Random shared by every
        // RealFuels engine whenever it evaluates a burning engine (its wasCombusting is never set, so the
        // "!wasCombusting && combusting" ignition test is true on every call). System.Random is not
        // thread-safe: sweep workers drawing from it together corrupt it, after which it returns 0 and
        // RealFuels' polar-method normal sampler loops forever - in the sweep, and on the next ignition in
        // flight. A clone is made nominal instead: no variance, and wasCombusting held true so the draw
        // never happens. Without those fields, SolverRF clones are driven one at a time.
        private static Type rfSolverType;
        private static FieldInfo rfWasCombusting;
        private static FieldInfo[] rfVarianceFields;
        private static bool rfNominal;
        private static readonly object rfSerialLock = new object();

        // Per-thread arg arrays for the drive path (the shared args* above are not thread-safe).
        [ThreadStatic] private static object[] driveState2;
        [ThreadStatic] private static object[] driveStream7;
        [ThreadStatic] private static object[] drivePerf4;

        /// <summary>
        /// The read-only per-engine values UpdateSolver folds in, snapshotted once on the main thread
        /// so the parallel drive path needs no reflection reads against the live module.
        /// </summary>
        public struct EngineDriveScalars
        {
            public double FlowMultiplier;          // flowMult * multFlow
            public double IspMultiplier;           // ispMult * multIsp
            public double LastPropellantFraction;
            public double MaxEngineTemp;
            public float ThrustPercentage;
            public float NeedArea;
            public double ThrustUpperLimitKN;      // flight's soft thrust cap (kN); double.MaxValue = uncapped
            public bool SquaresThrottleLimiter;    // AJE jet/ramjet/rotor apply (0.01*thrustPercentage) twice
        }

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
                // Flight clips solver thrust past this per-engine cap before it becomes finalThrust; the
                // deck reads raw solver thrust, so it must apply the same clip. Absent on older
                // SolverEngines (null) -> treated as uncapped.
                engineThrustUpperLimit = moduleEnginesSolverType.GetField("thrustUpperLimit");
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

                    // Drive path for per-thread solver clones.
                    objectMemberwiseClone = typeof(object).GetMethod("MemberwiseClone",
                                                                     BindingFlags.NonPublic | BindingFlags.Instance);
                    solverSetEngineState = engineSolverType.GetMethod("SetEngineState",
                                                                      new[] {typeof(bool), typeof(double)});
                    solverSetFreestreamAndInlet =
                        engineSolverType.GetMethod("SetFreestreamAndInlet",
                                                   new[]
                                                   {
                                                       thermoType, thermoType, typeof(double), typeof(double),
                                                       typeof(Vector3), typeof(bool), typeof(bool)
                                                   });
                    solverCalculatePerformance =
                        engineSolverType.GetMethod("CalculatePerformance",
                                                   new[]
                                                   {
                                                       typeof(double), typeof(double), typeof(double), typeof(double)
                                                   });
                }

                // flowMult/ispMult are public on ModuleEnginesSolver; multFlow/multIsp are inherited
                // public from stock ModuleEngines; lastPropellantFraction is protected on the solver.
                engineFlowMult = moduleEnginesSolverType.GetField("flowMult");
                engineIspMult = moduleEnginesSolverType.GetField("ispMult");
                engineMultFlow = moduleEnginesSolverType.GetField("multFlow");
                engineMultIsp = moduleEnginesSolverType.GetField("multIsp");
                engineLastPropFrac = moduleEnginesSolverType.GetField("lastPropellantFraction",
                                                                      BindingFlags.NonPublic | BindingFlags.Instance);

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

                ParallelAvailable = Available &&
                                    objectMemberwiseClone != null &&
                                    solverSetEngineState != null &&
                                    solverSetFreestreamAndInlet != null &&
                                    solverCalculatePerformance != null &&
                                    engineFlowMult != null &&
                                    engineIspMult != null &&
                                    engineMultFlow != null &&
                                    engineMultIsp != null &&
                                    engineLastPropFrac != null &&
                                    engineMaxTemp != null;

                ResolveRealFuels();

                if (Available)
                    FARLogger.Info("SolverEngines interop ready; editor engine sampling enabled" +
                                   (ParallelAvailable ? " (parallel-capable)" : " (serial only)"));
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

        /// <summary>The engine's temperature limit (maxEngineTemp), K; 0 if unavailable.</summary>
        public static double GetMaxEngineTemp(PartModule engine)
        {
            if (engineMaxTemp == null)
                return 0;
            return (double)engineMaxTemp.GetValue(engine);
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
        /// Flight's soft thrust cap (SolverEngines EngineModule.CalculateEngineParams): above the
        /// per-engine thrustUpperLimit (kN), thrust grows at only a 10% slope. Raw solver thrust runs far
        /// past the cap as Mach rises, so the deck applies the same clip or it over-predicts thrust
        /// (worsening with Mach). Fuel flow is deliberately not capped -- flight does not cap it either.
        /// </summary>
        private static double SoftCapThrustN(double thrustN, double thrustUpperLimitKN)
        {
            double kN = thrustN * 0.001;
            if (kN > thrustUpperLimitKN)
                kN = thrustUpperLimitKN + (kN - thrustUpperLimitKN) * 0.1;
            return kN * 1000.0;
        }

        /// <summary>
        /// True for the AJE engine modules whose UpdateThrottle override applies the thrust-limiter factor
        /// (0.01*thrustPercentage) a SECOND time (AJEJet.cs:229, AJERamjet.cs:82, AJERotor.cs:178), so the
        /// solver throttle is main-throttle * limiter^2. AJEPropeller and generic ModuleEnginesSolver apply
        /// it once. The deck must match whichever the module does, or a set thrust limiter mispredicts.
        /// </summary>
        public static bool SquaresThrottleLimiter(PartModule engine)
        {
            if (engine == null)
                return false;
            string n = engine.GetType().Name;
            return n == "ModuleEnginesAJEJet" || n == "ModuleEnginesAJERamjet" || n == "ModuleEnginesAJERotor";
        }

        /// <summary>
        /// The engine's soft thrust cap in kN; double.MaxValue (uncapped) when nothing sets it. RealFuels
        /// engines keep the cap in their active ModuleEngineConfigs CONFIG node and do NOT push it onto the
        /// ModuleEnginesSolver.thrustUpperLimit field in the editor (it lands there only in flight), so when
        /// the module field reads unset, fall back to that CONFIG node — which is present from part load.
        /// </summary>
        private static double ThrustUpperLimitKN(PartModule engine)
        {
            if (engineThrustUpperLimit != null)
            {
                double tul = Convert.ToDouble(engineThrustUpperLimit.GetValue(engine));
                if (tul > 0 && tul < double.MaxValue)
                    return tul;
            }

            return RealFuelsConfigThrustUpperLimitKN(engine);
        }

        /// <summary>
        /// Reads thrustUpperLimit (kN) from the active RealFuels ModuleEngineConfigs CONFIG node on the
        /// engine's part, matched by module name so FAR keeps no RealFuels reference. double.MaxValue when
        /// RealFuels is absent, the part has no configs module, or the active config sets no cap.
        /// </summary>
        private static double RealFuelsConfigThrustUpperLimitKN(PartModule engine)
        {
            Part part = engine != null ? engine.part : null;
            if (part == null || part.Modules == null)
                return double.MaxValue;

            foreach (PartModule pm in part.Modules)
            {
                if (pm == null || !pm.GetType().Name.StartsWith("ModuleEngineConfigs", StringComparison.Ordinal))
                    continue;

                Type t = pm.GetType();
                FieldInfo cfgNameField = t.GetField("configuration"); // both on ModuleEngineConfigsBase
                FieldInfo configsField = t.GetField("configs");
                if (cfgNameField == null || configsField == null)
                    continue;

                string active = cfgNameField.GetValue(pm) as string;
                var configs = configsField.GetValue(pm) as System.Collections.IEnumerable;
                if (string.IsNullOrEmpty(active) || configs == null)
                    continue;

                foreach (object o in configs)
                {
                    if (!(o is ConfigNode node) || node.GetValue("name") != active)
                        continue;

                    if (node.HasValue("thrustUpperLimit") &&
                        double.TryParse(node.GetValue("thrustUpperLimit"),
                                        System.Globalization.NumberStyles.Any,
                                        System.Globalization.CultureInfo.InvariantCulture,
                                        out double tul) &&
                        tul > 0)
                        return tul;

                    return double.MaxValue; // matched the active config; it just sets no cap
                }
            }

            return double.MaxValue;
        }

        /// <summary>
        /// Diagnostic: logs each editor engine's resolved soft thrust cap — the raw ModuleEnginesSolver
        /// field, the RealFuels CONFIG value, and which one <see cref="ThrustUpperLimitKN" /> uses — so a
        /// stuck-uncapped cell can be traced without a flight comparison.
        /// </summary>
        public static void AppendEditorEngineCaps(System.Text.StringBuilder sb)
        {
            if (!Available)
            {
                sb.AppendLine("  (SolverEngines/AJE not available)");
                return;
            }

            ShipConstruct ship = EditorLogic.fetch != null ? EditorLogic.fetch.ship : null;
            if (ship == null || ship.Parts == null)
            {
                sb.AppendLine("  (no editor ship)");
                return;
            }

            foreach (Part p in ship.Parts)
                foreach (PartModule pm in p.Modules)
                {
                    if (!moduleEnginesSolverType.IsInstanceOfType(pm))
                        continue;

                    double field = engineThrustUpperLimit != null
                                       ? Convert.ToDouble(engineThrustUpperLimit.GetValue(pm))
                                       : double.NaN;
                    sb.AppendLine(string.Format(System.Globalization.CultureInfo.InvariantCulture,
                                                "  {0} [{1}]: field={2}  rfConfig={3}  -> used={4} kN",
                                                p.partInfo != null ? p.partInfo.title : p.name,
                                                pm.GetType().Name,
                                                FormatCap(field),
                                                FormatCap(RealFuelsConfigThrustUpperLimitKN(pm)),
                                                FormatCap(ThrustUpperLimitKN(pm))));
                }
        }

        private static string FormatCap(double kN)
        {
            if (double.IsNaN(kN))
                return "n/a";
            return kN >= double.MaxValue
                       ? "MaxValue"
                       : kN.ToString("F1", System.Globalization.CultureInfo.InvariantCulture);
        }

        /// <summary>
        /// Diagnostic: the engine's raw (uncapped) solver thrust, its soft-capped thrust (both N), and the
        /// cap used (kN) — so a dump can show whether the over-prediction is in the raw thrust or the cap.
        /// </summary>
        public static void GetThrustBreakdown(PartModule engine, out double rawN, out double cappedN, out double capKN)
        {
            rawN = 0;
            cappedN = 0;
            capKN = double.MaxValue;

            object solver = engineSolverField.GetValue(engine);
            if (solver == null)
                return;

            rawN = (double)solverGetThrust.Invoke(solver, null);
            capKN = ThrustUpperLimitKN(engine);
            cappedN = SoftCapThrustN(rawN, capKN);
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

            thrustN = SoftCapThrustN((double)solverGetThrust.Invoke(solver, null), ThrustUpperLimitKN(engine));
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
        /// The thrust-weighted solver throttle (0-1) the running air-breathers are actually at in flight —
        /// each engine's currentThrottle (AFTER the thrust limiter, squared for AJE) weighted by the thrust
        /// it produces. Weighting by thrust (not the max) keeps the aggregate meaningful when a craft runs
        /// DIFFERENT per-engine thrust limiters, so it compares apples-to-apples with the sim's solved
        /// ThrottleRequired. Flight only. Returns false if no air-breather is producing positive thrust.
        /// </summary>
        public static bool TryGetFlightSolverThrottle(Vessel vessel, out double throttle)
        {
            throttle = 0;
            if (vessel == null || vessel.parts == null || moduleEnginesSolverType == null)
                return false;

            double weightSum = 0, throttleWeightSum = 0;
            foreach (Part p in vessel.parts)
                foreach (PartModule pm in p.Modules)
                {
                    if (!moduleEnginesSolverType.IsInstanceOfType(pm) || !(pm is ModuleEngines e))
                        continue;
                    if (!e.EngineIgnited || e.flameout)
                        continue;

                    double w = Math.Max(0, e.finalThrust); // only thrust-producing engines weight the mean
                    weightSum += w;
                    throttleWeightSum += w * e.currentThrottle;
                }

            if (weightSum <= 0)
                return false;
            throttle = throttleWeightSum / weightSum;
            return true;
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

        /// <summary>Snapshots the read-only scalars the drive path needs. Main thread; call once per engine.</summary>
        public static EngineDriveScalars GetDriveScalars(PartModule engine)
        {
            // These stock/SolverEngines fields differ in float-vs-double between members and KSP
            // versions (maxEngineTemp is a double, multFlow/multIsp are floats); Convert.ToDouble
            // unboxes any of them without an InvalidCastException.
            var s = new EngineDriveScalars();
            double flowMult = Convert.ToDouble(engineFlowMult.GetValue(engine));
            double ispMult = Convert.ToDouble(engineIspMult.GetValue(engine));
            double multFlow = Convert.ToDouble(engineMultFlow.GetValue(engine));
            double multIsp = Convert.ToDouble(engineMultIsp.GetValue(engine));
            s.FlowMultiplier = flowMult * multFlow;
            s.IspMultiplier = ispMult * multIsp;
            s.LastPropellantFraction = Convert.ToDouble(engineLastPropFrac.GetValue(engine));
            s.MaxEngineTemp = Convert.ToDouble(engineMaxTemp.GetValue(engine));
            s.ThrustUpperLimitKN = ThrustUpperLimitKN(engine);
            return s;
        }

        // RealFuels' SolverRF and the fields that make a clone nominal (see rfSolverType).
        private static void ResolveRealFuels()
        {
            foreach (AssemblyLoader.LoadedAssembly a in AssemblyLoader.loadedAssemblies)
                if (a.assembly.GetName().Name == "RealFuels")
                {
                    rfSolverType = a.assembly.GetType("RealFuels.SolverRF");
                    break;
                }
            if (rfSolverType == null)
                return;

            const BindingFlags inst = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
            rfWasCombusting = rfSolverType.GetField("wasCombusting", inst);
            string[] names = { "varyFlow", "varyIsp", "varyMR", "baseVaryFlow", "baseVaryIsp", "baseVaryMR", "runVaryFlow", "runVaryIsp", "runVaryMR" };
            rfVarianceFields = names.Select(n => rfSolverType.GetField(n, inst)).ToArray();
            rfNominal = rfWasCombusting != null && rfWasCombusting.FieldType == typeof(bool) &&
                        rfVarianceFields.All(f => f != null && f.FieldType == typeof(double));
            if (!rfNominal)
                FARLogger.Warning("RealFuels SolverRF fields not as expected; its engine clones are sampled one at a time");
        }

        private static bool IsRealFuels(object solver) => rfSolverType != null && rfSolverType.IsInstanceOfType(solver);

        /// <summary>
        /// An independent copy of the engine's EngineSolver for one worker slot. MemberwiseClone
        /// copies every value field (including the value-type EngineThermodynamics gas states) by
        /// value; only reference-typed FloatCurve fields (the centrifugal-flow maps on a jet) would
        /// stay shared, and AnimationCurve.Evaluate mutates an internal cache, so each is deep-copied.
        /// A RealFuels solver is made nominal (see rfSolverType). Main thread only — builds Unity
        /// AnimationCurves.
        /// </summary>
        public static object CloneSolver(PartModule engine)
        {
            object solver = engineSolverField.GetValue(engine);
            if (solver == null)
                return null;

            object clone = objectMemberwiseClone.Invoke(solver, null);

            for (Type t = clone.GetType(); t != null && t != typeof(object); t = t.BaseType)
                foreach (FieldInfo f in t.GetFields(BindingFlags.Instance |
                                                    BindingFlags.Public |
                                                    BindingFlags.NonPublic |
                                                    BindingFlags.DeclaredOnly))
                {
                    if (!typeof(FloatCurve).IsAssignableFrom(f.FieldType))
                        continue;
                    if (f.GetValue(clone) is FloatCurve src)
                        f.SetValue(clone, CloneFloatCurve(src));
                }

            if (rfNominal && IsRealFuels(clone))
            {
                foreach (FieldInfo f in rfVarianceFields)
                    f.SetValue(clone, 0d);
                rfWasCombusting.SetValue(clone, true);
            }

            return clone;
        }

        private static FloatCurve CloneFloatCurve(FloatCurve src)
        {
            var dst = new FloatCurve();
            AnimationCurve c = src.Curve;
            if (c != null)
                foreach (Keyframe k in c.keys)
                    dst.Add(k.time, k.value, k.inTangent, k.outTangent);
            return dst;
        }

        /// <summary>
        /// Drives one cloned solver to a flight condition and reads its outputs. Thread-safe: touches
        /// only the passed clone and per-thread arg arrays. Returns false (skip the engine) if the
        /// solver threw — AJE throws out of Math.Sign on NaN rather than returning it, exactly as the
        /// serial <see cref="UpdateSolver" /> guards against.
        /// </summary>
        public static bool DriveClonedSolver(
            object cloneSolver,
            EngineDriveScalars scalars,
            object ambientTherm,
            object inletTherm,
            double areaRatio,
            double altitude,
            Vector3 velocity,
            double mach,
            double currentThrottle,
            out double thrustN,
            out double fuelFlow,
            out double engineTemp,
            out bool running
        )
        {
            thrustN = 0;
            fuelFlow = 0;
            engineTemp = 0;
            running = false;

            object[] a2 = driveState2 ?? (driveState2 = new object[2]);
            a2[0] = true; // ignited
            a2[1] = scalars.LastPropellantFraction;

            object[] a7 = driveStream7 ?? (driveStream7 = new object[7]);
            a7[0] = ambientTherm;
            a7[1] = inletTherm;
            a7[2] = altitude;
            a7[3] = mach;
            a7[4] = velocity;
            a7[5] = true;  // oxygen
            a7[6] = false; // underwater

            object[] a4 = drivePerf4 ?? (drivePerf4 = new object[4]);
            a4[0] = areaRatio;
            a4[1] = currentThrottle;
            a4[2] = scalars.FlowMultiplier;
            a4[3] = scalars.IspMultiplier;

            bool realFuels = IsRealFuels(cloneSolver);
            try
            {
                if (realFuels && rfNominal)
                    rfWasCombusting.SetValue(cloneSolver, true); // in case a RealFuels build does set it
                if (realFuels && !rfNominal)
                {
                    lock (rfSerialLock)
                    {
                        solverSetEngineState.Invoke(cloneSolver, a2);
                        solverSetFreestreamAndInlet.Invoke(cloneSolver, a7);
                        solverCalculatePerformance.Invoke(cloneSolver, a4);
                    }
                }
                else
                {
                    solverSetEngineState.Invoke(cloneSolver, a2);
                    solverSetFreestreamAndInlet.Invoke(cloneSolver, a7);
                    solverCalculatePerformance.Invoke(cloneSolver, a4);
                }
            }
            catch (TargetInvocationException)
            {
                return false;
            }

            thrustN = SoftCapThrustN((double)solverGetThrust.Invoke(cloneSolver, null), scalars.ThrustUpperLimitKN);
            fuelFlow = (double)solverGetFuelFlow.Invoke(cloneSolver, null);
            running = (bool)solverGetRunning.Invoke(cloneSolver, null);
            engineTemp = solverGetEngineTemp != null ? (double)solverGetEngineTemp.Invoke(cloneSolver, null) : 0;
            return true;
        }
    }
}
