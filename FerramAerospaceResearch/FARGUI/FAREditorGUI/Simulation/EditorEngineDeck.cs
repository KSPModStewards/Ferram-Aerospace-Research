/*
Ferram Aerospace Research
=========================
Editor-side engine sampling: thrust and fuel flow at an arbitrary (Mach, altitude, throttle).

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    /// <summary>
    /// Samples the ship's SolverEngines/AJE engines at off-design flight conditions in the editor.
    ///
    /// Nothing here models thrust — AJE's own solver is asked for it. This is the same call that
    /// produces the part tooltip's static thrust (AJEJet.GetStaticThrustInfo drives UpdateSolver at
    /// Mach 0 / sea level and prints the answer); the only difference is that Mach and altitude are
    /// non-zero. What this class DOES own is the vessel-level inlet bookkeeping that
    /// SolverEngines.SolverFlightSys normally does — that's a VesselModule and so does not exist in
    /// the editor. That logic (area ratio, area-weighted TPR, reference-frame shift) mirrors
    /// SolverFlightSys.FixedUpdate.
    ///
    /// Sampling clobbers each engine module's solver state and currentThrottle. This is inert in the
    /// editor because SolverEngines.EngineModule.FixedUpdate early-returns before running the solver
    /// there; AJE relies on the same property for its tooltips and simply resets currentThrottle
    /// afterwards, which Restore() does here.
    /// </summary>
    internal class EditorEngineDeck
    {
        /// <summary>Aggregate engine state at one flight condition.</summary>
        public struct Sample
        {
            /// <summary>
            /// Thrust vector, editor world space, newtons — summed over engines that survive the
            /// heat at this condition. An engine over its temperature limit is treated as shut down
            /// (its thrust and fuel excluded), which is what a pilot does and what lets a
            /// turbojet+ramjet aircraft hand off from one to the other with rising Mach.
            /// </summary>
            public Vector3d Thrust;

            /// <summary>Magnitude of <see cref="Thrust" />, newtons.</summary>
            public double ThrustMagnitude;

            /// <summary>Total fuel mass flow of the sampled engines, kg/s.</summary>
            public double FuelFlow;

            /// <summary>Number of engines the solver reported as running.</summary>
            public int RunningCount;

            /// <summary>
            /// Highest engine-temperature-to-limit ratio across the engines, at the sampled throttle.
            /// At or above the limit the engine melts in flight (EngineModule.UpdateTemp). Because a
            /// ramjet's temperature depends on throttle, the caller reads this at the cruise throttle
            /// to decide the thermal wall, not at full throttle.
            /// </summary>
            public double MaxThermalRatio;

            // Inlet bookkeeping, recorded so it can be diffed against SolverFlightSys's public
            // InletArea/EngineArea/AreaRatio/OverallTPR in flight. This is the part of the deck
            // reconstructed without a reference implementation to copy, so it is the part most
            // worth being able to check.
            public double InletArea;
            public double EngineArea;
            public double AreaRatio;
            public double OverallTpr;

            /// <summary>Ambient and inlet conditions handed to the solver, Pa and K.</summary>
            public double AmbientP;

            public double AmbientT;
            public double InletP;
            public double InletT;
        }

        /// <summary>
        /// Thermal ratio at or above which an engine is treated as shut down for this sweep. Just
        /// under 1 (the melt point) because flying on the ragged edge of destruction is not a usable
        /// operating point. Matches the threshold PerformanceEnvelopeCalculator uses.
        /// </summary>
        public const double ThermalLimit = 0.98;

        /// <summary>A set of engines of one kind (e.g. all turbojets, or all ramjets).</summary>
        public class EngineGroup
        {
            public string Label;

            /// <summary>True for ramjets/scramjets — engines with no compressor heat wall.</summary>
            public bool Compressorless;

            public readonly List<PartModule> Engines = new List<PartModule>();
        }

        private readonly List<PartModule> _engines = new List<PartModule>();
        private readonly List<PartModule> _compressorless = new List<PartModule>();
        private readonly List<ModuleResourceIntake> _inlets = new List<ModuleResourceIntake>();
        private readonly List<EngineGroup> _groups = new List<EngineGroup>();

        // Engines the current Evaluate should use. Defaults to every engine; a per-type sweep points
        // it at one group so thrust, fuel and engine-inlet-area all reflect only that type running —
        // which is also what SolverFlightSys reports in flight when the pilot flies on one type.
        private List<PartModule> _active;

        // --- parallel sampling state ---
        // Built on the main thread by BeginParallel and consumed read-only by worker threads. Each
        // engine gets one solver clone per worker slot so threads never share solver state; the
        // read-only module scalars and thrust direction are snapshotted alongside. IntakeSnap freezes
        // the intake geometry the ram-drag magnitude reads, so that too runs off the main thread.
        private bool _parallel;
        private object[][] _clones; // [engineIndex][slot]
        private SolverEnginesInterop.EngineDriveScalars[] _scalars;
        private Vector3d[] _thrustDir;
        private Dictionary<PartModule, int> _engineIndexOf;
        private IntakeSnap[] _intakeSnap;

        private struct IntakeSnap
        {
            public Vector3d Forward;
            public double Area;
        }

        /// <summary>
        /// The main-thread-only part of a cell's engine conditions — gas state and the inlet geometry
        /// bookkeeping that reads Unity intake transforms. Group-independent; the small group-dependent
        /// scaling (engine area → area ratio → inlet pressure) is applied per group in EvaluateEnv.
        /// </summary>
        public struct CellEnv
        {
            public bool Valid;
            public double Mach;
            public double Alt;
            public double Speed;
            public Vector3d Forward;
            public double AmbientP;
            public double AmbientT;
            public double InletArea;
            public double AreaWeightedTpr;  // sum over intakes of area * overallTPR
            public double InletPreP;        // stagnation inlet pressure before the *overallTpr scaling
            public double InletPreT;        // stagnation inlet temperature
        }

        /// <summary>The engines of one map, resolved to clone-array indices, with their summed intake demand.</summary>
        public sealed class ParallelGroupContext
        {
            public int[] EngineIndices;
            public double EngineArea;
        }

        /// <summary>Engine kinds present on the craft, one entry per distinct module type.</summary>
        public IReadOnlyList<EngineGroup> Groups
        {
            get { return _groups; }
        }

        /// <summary>Restricts sampling to one group, or pass null to use every engine.</summary>
        public void SetActiveGroup(EngineGroup group)
        {
            _active = group != null ? group.Engines : _engines;
        }

        /// <summary>
        /// Restricts sampling to the compressor-less engines (ramjets, scramjets) — the ones with no
        /// throttle-independent compressor heat wall. Used by the combined map to fall back to a
        /// ramjet-only handoff where running the turbojets too would melt them.
        /// </summary>
        public void SetActiveCompressorless()
        {
            _active = _compressorless;
        }

        /// <summary>
        /// True when there is a compressor-less engine to hand off to AND something else to shed —
        /// i.e. the combined map has a distinct reduced-engine state worth falling back to.
        /// </summary>
        public bool CanShedToCompressorless
        {
            get { return _compressorless.Count > 0 && _compressorless.Count < _engines.Count; }
        }

        private List<PartModule> Active
        {
            get { return _active ?? _engines; }
        }

        /// <summary>Engines whose solver could be sampled.</summary>
        public int EngineCount
        {
            get { return _engines.Count; }
        }

        public int InletCount
        {
            get { return _inlets.Count; }
        }

        public bool Ready
        {
            get { return SolverEnginesInterop.Available && _engines.Count > 0; }
        }

        /// <summary>
        /// Rebuilds the engine/inlet lists from the current editor ship. Cheap; call after any
        /// change to the craft.
        /// </summary>
        public void UpdateShip()
        {
            _engines.Clear();
            _compressorless.Clear();
            _inlets.Clear();
            _groups.Clear();
            _active = null;

            SolverEnginesInterop.Initialize();
            if (!SolverEnginesInterop.Available)
                return;

            List<Part> partsList = EditorLogic.SortedShipList;
            if (partsList == null)
                return;

            foreach (Part p in partsList)
            {
                if (p == null)
                    continue;

                foreach (PartModule m in p.Modules)
                {
                    if (SolverEnginesInterop.IsSolverEngine(m))
                    {
                        // Start() normally creates the solver in the editor, but a part added this
                        // frame may not have run it yet.
                        SolverEnginesInterop.CreateEngineIfNecessary(m);
                        _engines.Add(m);
                        AddToGroup(m);
                        if (IsCompressorless(m))
                            _compressorless.Add(m);
                    }
                    else if (SolverEnginesInterop.IsAJEInlet(m) && m is ModuleResourceIntake intake)
                    {
                        _inlets.Add(intake);
                    }
                }
            }
        }

        /// <summary>
        /// True for engines with no mechanical compressor — ramjets, scramjets — which rely on ram
        /// heating to autoignite and so need the inlet-temperature gate. Identified by module type
        /// name, following FAR's existing string-based AJE detection.
        /// </summary>
        private static bool IsCompressorless(PartModule m)
        {
            string n = m.GetType().Name;
            return n.IndexOf("Ramjet", StringComparison.Ordinal) >= 0 ||
                   n.IndexOf("Scramjet", StringComparison.Ordinal) >= 0;
        }

        /// <summary>Files an engine under the group for its module type, creating the group if new.</summary>
        private void AddToGroup(PartModule m)
        {
            string label = FriendlyEngineType(m.GetType().Name);

            foreach (EngineGroup g in _groups)
                if (g.Label == label)
                {
                    g.Engines.Add(m);
                    return;
                }

            var group = new EngineGroup {Label = label, Compressorless = IsCompressorless(m)};
            group.Engines.Add(m);
            _groups.Add(group);
        }

        /// <summary>
        /// A readable name for an engine module type. AJE's classes map to the everyday words;
        /// anything else is the type name with the boilerplate prefix stripped, so unknown engine
        /// mods still get sensibly grouped rather than lumped together.
        /// </summary>
        private static string FriendlyEngineType(string typeName)
        {
            switch (typeName)
            {
                case "ModuleEnginesAJEJet":
                    return "Turbojet/fan";
                case "ModuleEnginesAJERamjet":
                    return "Ramjet";
                case "ModuleEnginesAJEPropeller":
                    return "Propeller";
                case "ModuleEnginesAJERotor":
                    return "Rotor";
                default:
                    if (typeName.StartsWith("ModuleEngines"))
                        return typeName.Substring("ModuleEngines".Length);
                    return typeName;
            }
        }

        /// <summary>
        /// Resource ids the collected engines actually consume for thrust. Propellants flagged
        /// ignoreForIsp are excluded, as are AJE air-breathers' intake air — AJE strips the
        /// IntakeAir PROPELLANT from its jets entirely (zzWildcards.cfg) and models the inlet
        /// internally, so in practice this comes back as just the fuel.
        /// </summary>
        public HashSet<int> GetBurnablePropellantIds()
        {
            var ids = new HashSet<int>();

            foreach (PartModule m in _engines)
            {
                if (!(m is ModuleEngines e) || e.propellants == null)
                    continue;

                foreach (Propellant prop in e.propellants)
                {
                    if (prop.ignoreForIsp)
                        continue;

                    PartResourceDefinition def = PartResourceLibrary.Instance?.GetDefinition(prop.id);
                    if (def == null || def.density <= 0)
                        continue;

                    ids.Add(prop.id);
                }
            }

            return ids;
        }

        /// <summary>
        /// Unit vector along the vehicle's nose in editor world space. Matches the convention in
        /// InstantConditionSim.GetClCdCmSteady, which builds its velocity vector from these same
        /// world axes rather than anything part-local.
        /// </summary>
        public static Vector3d VehicleForward()
        {
            return EditorDriver.editorFacility == EditorFacility.VAB ? Vector3d.up : Vector3d.forward;
        }

        /// <summary>
        /// Editor-safe stand-in for AJEInlet.IntakeActive(). The real one calls InAtmosphere(),
        /// which dereferences part.vessel and throws in the editor. InAtmosphere is true by
        /// construction here (callers only sample altitudes with pressure > 0) and the underwater
        /// check never applies in the editor.
        /// </summary>
        private static bool IntakeActiveInEditor(ModuleResourceIntake intake)
        {
            return intake.intakeEnabled &&
                   !intake.part.ShieldedFromAirstream &&
                   intake.intakeTransform != null;
        }

        /// <summary>
        /// Evaluates every engine at one flight condition and returns the aggregate.
        /// </summary>
        /// <param name="body">Body whose atmosphere to sample.</param>
        /// <param name="alt">Altitude ASL, m. Caller must ensure this is inside the atmosphere.</param>
        /// <param name="mach">Free-stream Mach number.</param>
        /// <param name="throttle">Commanded throttle, 0-1. For AJE jets with an afterburner, the
        /// top of the range engages it, so 1.0 is wet and ~2/3 is dry (see AJEJet.GetStaticThrustInfo).</param>
        public Sample Evaluate(CelestialBody body, double alt, double mach, double throttle)
        {
            return Evaluate(body, alt, mach, throttle, null);
        }

        /// <summary>
        /// As <see cref="Evaluate(CelestialBody,double,double,double)" />, but when <paramref name="diag" />
        /// is non-null appends a per-engine raw-vs-capped thrust / Isp / fuel breakdown and the vessel-level
        /// inlet terms — the middle-click cell diagnostic, to localize a high-Mach thrust over-prediction.
        /// </summary>
        public Sample Evaluate(CelestialBody body, double alt, double mach, double throttle, StringBuilder diag)
        {
            var sample = new Sample();
            if (!Ready)
                return sample;

            var latLonAlt = new Vector3d(0, 0, alt);
            double ut = Planetarium.GetUniversalTime();

            // EditorAtmosphere rather than FARAtmosphere or EngineThermodynamics.AmbientAtAltitude:
            // all three of those return the bare altitude temperature curve, which runs ~25 K cold
            // against flight near the ground. A jet is acutely sensitive to inlet temperature, so
            // that was worth 3x on thrust at sea level. Pressure is Pa, as the ctor wants.
            GasProperties gas = EditorAtmosphere.GetGasProperties(body, alt);
            if (gas.Pressure <= 0)
                return sample;

            double speed = mach * gas.SpeedOfSound;
            Vector3d forward = VehicleForward();
            var velocity = (Vector3)(forward * speed);

            object ambientTherm = SolverEnginesInterop.MakeThermo(gas.Pressure, gas.Temperature);

            // --- vessel-level inlet accounting; mirrors SolverFlightSys.FixedUpdate ---
            // Engine area is summed over the active group only: in flight, SolverFlightSys counts
            // just the ignited engines, so a turbojet-only sweep must count only turbojet area to
            // match. Inlets are shared across all engines, so inlet area stays whole-ship.
            double engineArea = 0;
            foreach (PartModule e in Active)
                engineArea += SolverEnginesInterop.GetNeedArea(e);

            double inletArea = 0;
            double overallTpr = 0;
            foreach (ModuleResourceIntake intake in _inlets)
            {
                if (!IntakeActiveInEditor(intake))
                    continue;

                // Inlet recovery depends on the angle between the intake and the relative wind.
                // Sampled at zero AoA; the AoA dependence is second-order and the craft's trim
                // attitude is not known until the aero solve runs.
                SolverEnginesInterop.UpdateInletOverallTPR(intake, velocity, mach);

                double area = SolverEnginesInterop.GetInletArea(intake);
                inletArea += area;
                overallTpr += area * SolverEnginesInterop.GetInletOverallTPR(intake);
            }

            double areaRatio;
            if (engineArea <= 0)
            {
                // Nothing here breathes (a rocket solver, e.g. ModuleEnginesRF), so inlet recovery
                // is meaningless — leave the inlet conditions untouched. SolverFlightSys instead
                // leaves OverallTPR as an un-normalized area-weighted sum on this branch, which
                // would scale pressure by an arbitrary factor; not worth reproducing.
                areaRatio = 1;
                overallTpr = 1;
            }
            else if (inletArea <= 0)
            {
                // An air-breather with no usable inlet produces nothing. Returning early rather
                // than following SolverFlightSys into OverallTPR = 0 — that drives inlet pressure
                // to exactly zero, and the jet solver divides through pressure ratios, so it comes
                // back NaN rather than "no thrust". SolverFlightSys never meets this case because a
                // vessel in flight with a running jet always has an intake.
                return sample;
            }
            else
            {
                areaRatio = Math.Min(1, inletArea / engineArea);
                overallTpr /= inletArea;
                overallTpr *= areaRatio;
            }

            // Distinct box from ambientTherm — SetPressure mutates in place, and aliasing the two
            // would corrupt the ambient conditions handed to UpdateSolver below.
            object inletTherm = speed < 0.01
                                    ? SolverEnginesInterop.MakeThermo(gas.Pressure, gas.Temperature)
                                    : SolverEnginesInterop.ChangeReferenceFrame(ambientTherm, speed);
            SolverEnginesInterop.SetPressure(inletTherm, SolverEnginesInterop.GetPressure(inletTherm) * overallTpr);

            // --- drive each engine ---
            var throttleF = (float)Math.Max(0, Math.Min(1, throttle));
            Vector3d thrustSum = Vector3d.zero;

            foreach (PartModule m in Active)
            {
                if (!(m is ModuleEngines e))
                    continue;

                // Ramjet light-up is left to AJE's solver, not gated here. SolverRamjet stops
                // combustion when ram recovery drops too low and returns zero thrust; above that it
                // runs (from ~Mach 0.5 up). autoignitionTemp is not a light-up threshold in AJE (it
                // only feeds CanAutoRestart), so gating on it wrongly zeroed ramjet thrust that
                // flight produces.
                SolverEnginesInterop.UpdateInletEffects(m, inletTherm, areaRatio, overallTpr);

                // Assigned directly rather than via UpdateThrottle(), which integrates spool-up
                // against TimeWarp.fixedDeltaTime. GetStaticThrustInfo does the same. The AJE jet/ramjet
                // UpdateThrottle applies the thrust limiter a second time (squares it), so match that here
                // or a set thrust limiter mispredicts; inert at the usual thrustPercentage = 100.
                float limiter = 0.01f * e.thrustPercentage;
                e.currentThrottle = SolverEnginesInterop.SquaresThrottleLimiter(m)
                                        ? throttleF * limiter * limiter
                                        : throttleF * limiter;

                // A solver that threw has left its outputs meaningless — skip the engine rather than
                // read whatever happens to be in its fields.
                if (!SolverEnginesInterop.UpdateSolver(m,
                                                       ambientTherm,
                                                       alt,
                                                       forward * speed,
                                                       mach,
                                                       true,
                                                       true,
                                                       false))
                    continue;

                if (!SolverEnginesInterop.TryGetSolverOutputs(m,
                                                              out double thrustN,
                                                              out double fuelFlow,
                                                              out double sfc,
                                                              out double isp,
                                                              out bool running))
                    continue;

                if (diag != null)
                {
                    SolverEnginesInterop.GetThrustBreakdown(m, out double rawN, out double cappedN, out double capKN);
                    diag.AppendFormat(CultureInfo.InvariantCulture,
                                      "  {0}: raw {1:F1} -> capped {2:F1} kN (cap {3})  fuel {4:F3} kg/s  Isp {5:F0} s  SFC {6:F3}  thermal {7:F0}%  running {8}\n",
                                      e.part.partInfo.title, rawN / 1000, cappedN / 1000,
                                      capKN >= double.MaxValue ? "none" : capKN.ToString("F1", CultureInfo.InvariantCulture),
                                      fuelFlow, isp, sfc, SolverEnginesInterop.GetThermalRatio(m) * 100, running);
                }

                // The solver can hand back non-finite values at conditions it cannot resolve.
                // Dropping them here keeps a single bad cell from poisoning the whole sum, and
                // later the envelope solve, with a NaN that is hard to trace back.
                if (!FARMathUtil.IsFinite(thrustN) || !FARMathUtil.IsFinite(fuelFlow))
                {
                    FARLogger.DebugFormat("Engine {0} returned non-finite performance at M {1}, alt {2}",
                                          e.part.partInfo.title,
                                          mach,
                                          alt);
                    continue;
                }

                Vector3d engineThrust = ThrustDirection(m, e) * thrustN;

                // Thermal ratio at THIS throttle. It is not folded into a shed/keep decision here:
                // a ramjet's temperature (nozzle exit) falls with throttle, so whether it survives
                // depends on the throttle it will actually cruise at — a full-throttle sample would
                // wrongly wall off cells the aircraft flies through at part throttle. The caller
                // decides feasibility at the cruise throttle instead. (A jet's temp is compressor
                // exit, throttle-independent, so the same check still walls it correctly.)
                double thermalRatio = SolverEnginesInterop.GetThermalRatio(m);
                if (thermalRatio > sample.MaxThermalRatio)
                    sample.MaxThermalRatio = thermalRatio;

                thrustSum += engineThrust;
                sample.FuelFlow += fuelFlow;
                if (running)
                    sample.RunningCount++;
            }

            sample.Thrust = thrustSum;
            sample.ThrustMagnitude = thrustSum.magnitude;
            sample.InletArea = inletArea;
            sample.EngineArea = engineArea;
            sample.AreaRatio = areaRatio;
            sample.OverallTpr = overallTpr;
            sample.AmbientP = gas.Pressure;
            sample.AmbientT = gas.Temperature;
            sample.InletP = SolverEnginesInterop.GetPressure(inletTherm);
            sample.InletT = SolverEnginesInterop.GetTemperature(inletTherm);

            if (diag != null)
            {
                double qPa = 0.5 * gas.Density * speed * speed;
                double ramMag = IntakeRamDragMagnitudeN(mach, qPa, forward);
                double ramDragEff = throttleF >= 0.5f ? 0 : ramMag * (1 - 2 * throttleF);
                diag.AppendFormat(CultureInfo.InvariantCulture,
                                  "  vessel: inletArea {0:F4} engineArea {1:F4} m2  areaRatio {2:F3}  overallTPR {3:F3}\n" +
                                  "          ambientP {4:F3} kPa  inletP(postTPR) {5:F3} kPa  ambientT {6:F1} K  inletT {7:F1} K  rho {8:F5}\n" +
                                  "          thrustSum(capped) {9:F1} kN  ramDrag {10:F2} kN (magnitude {11:F2})\n",
                                  inletArea, engineArea, areaRatio, overallTpr,
                                  gas.Pressure / 1000, sample.InletP / 1000, gas.Temperature, sample.InletT, gas.Density,
                                  sample.ThrustMagnitude / 1000, ramDragEff / 1000, ramMag / 1000);
            }

            return sample;
        }

        /// <summary>
        /// Unit thrust direction in editor world space, averaged over the engine's thrust transforms
        /// and weighted the way EngineModule.FixedUpdate weights them. Thrust opposes the exhaust,
        /// hence the negation.
        /// </summary>
        private static Vector3d ThrustDirection(PartModule m, ModuleEngines e)
        {
            if (e.thrustTransforms == null || e.thrustTransforms.Count == 0)
                return VehicleForward();

            bool useZ = SolverEnginesInterop.GetUseZaxis(m);
            Vector3d dir = Vector3d.zero;

            for (int i = 0; i < e.thrustTransforms.Count; i++)
            {
                Transform t = e.thrustTransforms[i];
                if (t == null)
                    continue;

                float mult = e.thrustTransformMultipliers != null && i < e.thrustTransformMultipliers.Count
                                 ? e.thrustTransformMultipliers[i]
                                 : 1f;

                dir += (Vector3d)(useZ ? -t.forward : -t.up) * mult;
            }

            return dir.magnitude > 1e-6 ? dir.normalized : VehicleForward();
        }

        // VesselIntakeRamDrag's nozzle factor: 0.25 * (1 - 0.25).
        private const double AvgNozzleVelFactor = 0.1875;

        /// <summary>
        /// Intake spillage/ram drag at part throttle, newtons, as the throttle-independent magnitude
        /// M: ram drag = t >= 0.5 ? 0 : M * (1 - 2t). Mirrors VesselIntakeRamDrag, which flight adds
        /// on top of the engine's net thrust but the sweep otherwise omits. velDir is the cruise wind
        /// direction in editor world space; qPa the dynamic pressure in Pascals.
        /// </summary>
        public double IntakeRamDragMagnitudeN(double mach, double qPa, Vector3d velDir)
        {
            if (qPa <= 0)
                return 0;

            Vector3d vel = velDir.normalized;
            double areaCos = 0;

            // Off the main thread the live intake transforms cannot be read; use the frozen snapshot.
            if (_parallel)
            {
                if (_intakeSnap == null)
                    return 0;
                foreach (IntakeSnap snap in _intakeSnap)
                {
                    double cosAoA = Vector3d.Dot(snap.Forward, vel);
                    if (cosAoA > 0)
                        areaCos += cosAoA * snap.Area;
                }
            }
            else
            {
                if (_inlets.Count == 0)
                    return 0;
                foreach (ModuleResourceIntake intake in _inlets)
                {
                    if (!IntakeActiveInEditor(intake))
                        continue;

                    double cosAoA = Vector3d.Dot((Vector3d)intake.intakeTransform.forward, vel);
                    if (cosAoA > 0)
                        areaCos += cosAoA * intake.area;
                }
            }

            if (areaCos <= 0)
                return 0;

            // Flight applies force(kN) = areaCos * RamDragPerArea(M) * 100 * q(kPa) at full spillage;
            // with q in Pa this is the same force in N.
            double ramPerArea = 2.0 / (1.0 + mach * mach) * AvgNozzleVelFactor + 0.1;
            return areaCos * ramPerArea * 100.0 * qPa;
        }

        /// <summary>
        /// Mints the per-worker solver clones and snapshots the read-only engine/intake state the
        /// parallel drive path reads, so <see cref="EvaluateEnv" /> can run on worker threads. Main
        /// thread only. Returns false (caller falls back to the serial pass) if the interop cannot
        /// drive clones. Pair with <see cref="EndParallel" />.
        /// </summary>
        public bool BeginParallel(int slotCount)
        {
            if (!Ready || !SolverEnginesInterop.ParallelAvailable || slotCount < 1)
                return false;

            // Any failure minting the clone pool (an unexpected engine type, a reflection surprise)
            // falls back to the serial engine pass rather than aborting the whole sweep.
            try
            {
                int n = _engines.Count;
                _clones = new object[n][];
                _scalars = new SolverEnginesInterop.EngineDriveScalars[n];
                _thrustDir = new Vector3d[n];
                _engineIndexOf = new Dictionary<PartModule, int>(n);

                for (int ei = 0; ei < n; ei++)
                {
                    PartModule m = _engines[ei];
                    _engineIndexOf[m] = ei;

                    SolverEnginesInterop.EngineDriveScalars s = SolverEnginesInterop.GetDriveScalars(m);
                    s.NeedArea = SolverEnginesInterop.GetNeedArea(m);
                    s.ThrustPercentage = m is ModuleEngines e0 ? e0.thrustPercentage : 100f;
                    s.SquaresThrottleLimiter = SolverEnginesInterop.SquaresThrottleLimiter(m);
                    _scalars[ei] = s;

                    _thrustDir[ei] = m is ModuleEngines e1 ? ThrustDirection(m, e1) : VehicleForward();

                    var arr = new object[slotCount];
                    for (int slot = 0; slot < slotCount; slot++)
                        arr[slot] = SolverEnginesInterop.CloneSolver(m);
                    _clones[ei] = arr;
                }

                var snaps = new List<IntakeSnap>(_inlets.Count);
                foreach (ModuleResourceIntake intake in _inlets)
                {
                    if (!IntakeActiveInEditor(intake))
                        continue;
                    snaps.Add(new IntakeSnap
                    {
                        Forward = (Vector3d)intake.intakeTransform.forward,
                        Area = intake.area
                    });
                }

                _intakeSnap = snaps.ToArray();
                _parallel = true;
                return true;
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "while building the parallel engine clone pool; falling back to serial engine pass");
                EndParallel();
                return false;
            }
        }

        /// <summary>Releases the parallel clone pool. Main thread; call in the sweep's finally.</summary>
        public void EndParallel()
        {
            _parallel = false;
            _clones = null;
            _scalars = null;
            _thrustDir = null;
            _engineIndexOf = null;
            _intakeSnap = null;
        }

        /// <summary>
        /// Resolves an engine group to the clone-array indices and summed intake demand a worker needs.
        /// Main thread, after <see cref="BeginParallel" />. Pass null for the combined all-engines map.
        /// </summary>
        public ParallelGroupContext BuildGroupContext(EngineGroup group)
        {
            return BuildGroupContextFromList(group != null ? group.Engines : _engines);
        }

        /// <summary>The compressor-less engines as a group context, for the combined map's ram-only shed.</summary>
        public ParallelGroupContext CompressorlessContext()
        {
            return BuildGroupContextFromList(_compressorless);
        }

        private ParallelGroupContext BuildGroupContextFromList(List<PartModule> list)
        {
            var idx = new int[list.Count];
            double area = 0;
            for (int nn = 0; nn < list.Count; nn++)
            {
                int ei = _engineIndexOf[list[nn]];
                idx[nn] = ei;
                area += _scalars[ei].NeedArea;
            }

            return new ParallelGroupContext {EngineIndices = idx, EngineArea = area};
        }

        /// <summary>
        /// The main-thread half of <see cref="Evaluate" />: gas state plus the inlet bookkeeping that
        /// reads Unity intake transforms (area ratio, area-weighted TPR, stagnation inlet conditions).
        /// Group-independent, so it is computed once per (Mach, altitude) cell and shared across the
        /// per-type and combined maps.
        /// </summary>
        public CellEnv BuildCellEnv(GasProperties gas, double alt, double mach)
        {
            var env = new CellEnv {Mach = mach, Alt = alt};
            if (gas.Pressure <= 0)
                return env;

            double speed = mach * gas.SpeedOfSound;
            Vector3d forward = VehicleForward();
            var velocity = (Vector3)(forward * speed);

            env.Forward = forward;
            env.Speed = speed;
            env.AmbientP = gas.Pressure;
            env.AmbientT = gas.Temperature;

            double inletArea = 0;
            double areaWeightedTpr = 0;
            foreach (ModuleResourceIntake intake in _inlets)
            {
                if (!IntakeActiveInEditor(intake))
                    continue;

                SolverEnginesInterop.UpdateInletOverallTPR(intake, velocity, mach);
                double area = SolverEnginesInterop.GetInletArea(intake);
                inletArea += area;
                areaWeightedTpr += area * SolverEnginesInterop.GetInletOverallTPR(intake);
            }

            env.InletArea = inletArea;
            env.AreaWeightedTpr = areaWeightedTpr;

            // Stagnation (total) inlet conditions before the *overallTpr pressure scaling, which is
            // group-dependent and so applied in EvaluateEnv. Below ~still air the ambient stands in.
            if (speed < 0.01)
            {
                env.InletPreP = gas.Pressure;
                env.InletPreT = gas.Temperature;
            }
            else
            {
                object ambient = SolverEnginesInterop.MakeThermo(gas.Pressure, gas.Temperature);
                object inlet = SolverEnginesInterop.ChangeReferenceFrame(ambient, speed);
                env.InletPreP = SolverEnginesInterop.GetPressure(inlet);
                env.InletPreT = SolverEnginesInterop.GetTemperature(inlet);
            }

            env.Valid = true;
            return env;
        }

        /// <summary>
        /// The worker-thread counterpart of <see cref="Evaluate" />: drives the slot's solver clones
        /// for one engine group at a throttle, using the pre-built <see cref="CellEnv" />. Mirrors the
        /// area-ratio / TPR / inlet-pressure arithmetic and the aggregation of the serial path, but
        /// touches only the passed slot's clones and per-thread arg arrays.
        /// </summary>
        public Sample EvaluateEnv(in CellEnv env, ParallelGroupContext group, int slot, double throttle)
        {
            var sample = new Sample();
            if (!env.Valid || group == null || _clones == null)
                return sample;

            double engineArea = group.EngineArea;
            double inletArea = env.InletArea;
            double areaRatio;
            double overallTpr;

            if (engineArea <= 0)
            {
                areaRatio = 1;
                overallTpr = 1;
            }
            else if (inletArea <= 0)
            {
                return sample;
            }
            else
            {
                areaRatio = Math.Min(1, inletArea / engineArea);
                overallTpr = env.AreaWeightedTpr / inletArea * areaRatio;
            }

            double inletP = env.InletPreP * overallTpr;
            object ambientTherm = SolverEnginesInterop.MakeThermo(env.AmbientP, env.AmbientT);
            object inletTherm = SolverEnginesInterop.MakeThermo(inletP, env.InletPreT);

            var vel = (Vector3)(env.Forward * env.Speed);
            var throttleF = (float)Math.Max(0, Math.Min(1, throttle));
            Vector3d thrustSum = Vector3d.zero;

            foreach (int ei in group.EngineIndices)
            {
                object clone = _clones[ei][slot];
                if (clone == null)
                    continue;

                SolverEnginesInterop.EngineDriveScalars s = _scalars[ei];
                double limiter = 0.01f * s.ThrustPercentage;
                double currentThrottle = s.SquaresThrottleLimiter ? throttleF * limiter * limiter : throttleF * limiter;

                if (!SolverEnginesInterop.DriveClonedSolver(clone,
                                                            s,
                                                            ambientTherm,
                                                            inletTherm,
                                                            areaRatio,
                                                            env.Alt,
                                                            vel,
                                                            env.Mach,
                                                            currentThrottle,
                                                            out double thrustN,
                                                            out double fuelFlow,
                                                            out double engineTemp,
                                                            out bool running))
                    continue;

                if (!FARMathUtil.IsFinite(thrustN) || !FARMathUtil.IsFinite(fuelFlow))
                    continue;

                thrustSum += _thrustDir[ei] * thrustN;
                sample.FuelFlow += fuelFlow;

                double thermalRatio = s.MaxEngineTemp > 0 ? engineTemp / s.MaxEngineTemp : 0;
                if (thermalRatio > sample.MaxThermalRatio)
                    sample.MaxThermalRatio = thermalRatio;

                if (running)
                    sample.RunningCount++;
            }

            sample.Thrust = thrustSum;
            sample.ThrustMagnitude = thrustSum.magnitude;
            sample.InletArea = inletArea;
            sample.EngineArea = engineArea;
            sample.AreaRatio = areaRatio;
            sample.OverallTpr = overallTpr;
            sample.AmbientP = env.AmbientP;
            sample.AmbientT = env.AmbientT;
            sample.InletP = inletP;
            sample.InletT = env.InletPreT;
            return sample;
        }

        /// <summary>
        /// Returns engines to an idle state after sampling. Mirrors the currentThrottle reset at the
        /// end of AJEJet.GetInfo.
        /// </summary>
        public void Restore()
        {
            foreach (PartModule m in _engines)
                if (m is ModuleEngines e)
                    e.currentThrottle = 0f;
        }
    }
}
