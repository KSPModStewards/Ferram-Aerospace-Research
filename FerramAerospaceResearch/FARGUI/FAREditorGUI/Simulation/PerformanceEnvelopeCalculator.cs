/*
Ferram Aerospace Research
=========================
Level-flight performance envelope: top speed, ceiling and endurance over (Mach, altitude).

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Collections.Generic;
using ferram4;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    /// <summary>
    /// Answers, for one (Mach, altitude) cell: can the vehicle hold level flight here, at what
    /// throttle, and for how long?
    ///
    /// Top speed at an altitude, the level-flight ceiling and an endurance map are three views of
    /// the same grid of these cells — the top speed is the fastest feasible cell in a row, the
    /// ceiling is the highest row with any feasible cell, and endurance is the cell value. They are
    /// not separate sweeps.
    ///
    /// Level flight requires L = W and T = D. Both aero coefficients and thrust are needed at the
    /// same condition: drag comes from InstantConditionSim (trimmed for lift), thrust from
    /// EditorEngineDeck (AJE's own solver). Thrust must be re-sampled per cell — it lapses with
    /// altitude, and that lapse is the entire reason a ceiling exists. Minimum level-flight drag is
    /// W/(L/D)max, which has no density term, so with altitude-independent thrust the envelope would
    /// never close.
    /// </summary>
    internal class PerformanceEnvelopeCalculator
    {
        private const double G0 = 9.80665;

        /// <summary>Fixed-point passes reconciling trim AoA against the lift component of thrust.</summary>
        private const int ThrustLiftIterations = 2;

        private readonly InstantConditionSim _instantCondition;

        // Cm change across full pitch deflection below which the craft is treated as having no pitch
        // authority (trims at neutral) rather than being control-limited at this attitude.
        private const double PitchAuthorityEpsilon = 1e-3;

        // Fuel fraction the leanest mass sample keeps, so it still solves a cruise and reports range
        // (a 0% sample has none). See ComputeMassSamples.
        private const double MinFuelFraction = 0.01;

        public PerformanceEnvelopeCalculator(InstantConditionSim instantConditionSim)
        {
            _instantCondition = instantConditionSim;
        }

        /// <summary>
        /// Freezes the main-thread-only geometry the aero solve reads so the cruise trim can run on
        /// worker threads. Main thread only; pair with <see cref="EndParallelSweep" />.
        /// </summary>
        public void BeginParallelSweep()
        {
            _instantCondition.BeginParallelSweep();
        }

        public void EndParallelSweep()
        {
            _instantCondition.EndParallelSweep();
        }

        /// <summary>
        /// Vehicle-level quantities that do not vary with flight condition, so they are computed
        /// once per sweep rather than per cell.
        /// </summary>
        public struct VehicleProperties
        {
            public bool Valid;

            /// <summary>Wet mass, kg.</summary>
            public double MassKg;

            public Vector3d CoM;

            /// <summary>Reference wing area, m^2.</summary>
            public double Area;

            /// <summary>Mass of resources the engines can actually burn, kg.</summary>
            public double UsableFuelKg;
        }

        /// <summary>Why a sustainable cell has no endurance figure.</summary>
        public enum CruiseState
        {
            Ok,

            /// <summary>Even a closed throttle out-thrusts drag — no steady cruise at this speed.</summary>
            IdleExceedsDrag,

            /// <summary>Nothing aboard that the engines can burn.</summary>
            NoBurnableFuel,

            /// <summary>The throttle root find did not settle.</summary>
            Unsolved
        }

        public struct CellResult
        {
            /// <summary>An AoA holding L = W was found.</summary>
            public bool Trimmed;

            /// <summary>
            /// Not individually computed — assumed a heat wall. The sweep scans each row low-to-high Mach
            /// and, once several cells hit the heat wall, fills the rest of the high-Mach tail as heat wall
            /// rather than solving them, since stagnation heat only rises further out. Rendered like the
            /// class it was assigned; the tooltip flags it as assumed so its zeroed numbers are not read as
            /// measured. (The low-Mach stall lead-in is cheap, so it is always solved, never extrapolated.)
            /// </summary>
            public bool Extrapolated;

            public CruiseState Cruise;

            /// <summary>Level flight is sustainable — thrust matches drag and the engine survives.</summary>
            public bool Feasible;

            /// <summary>
            /// Thrust covers drag, but the engine would exceed its temperature limit and melt here.
            /// A distinct failure from thrust-limited: the aircraft has the power, the engine just
            /// cannot take the heat. This is what caps a turbojet's top speed.
            /// </summary>
            public bool Overheats;

            /// <summary>
            /// Hidden by a post-sweep cleanup pass (isolated grey dot, or a trimmed cell disconnected from
            /// the main envelope) — the renderer paints it black and it is excluded from the flyable
            /// envelope, but it KEEPS its measured drag/thrust so tooltips, mass interpolation and the
            /// flight prediction-check can still read them. <see cref="Trimmed" /> stays true for that.
            /// </summary>
            public bool Suppressed;

            /// <summary>
            /// Engine-temperature-to-limit ratio at the cruise throttle this cell would need;
            /// >= 1 means the engine melts trying to hold this speed.
            /// </summary>
            public double ThermalRatio;

            /// <summary>
            /// This is a fallback result on the combined map: running every engine would overheat
            /// here, so the turbojets are shut off and only the compressor-less engines (ramjets)
            /// carry the aircraft. Feasible, but on a reduced engine set — the renderer darkens it to
            /// set it apart from cells flown on full combined power.
            /// </summary>
            public bool ReducedEngines;

            /// <summary>Trim angle of attack, degrees.</summary>
            public double Alpha;

            /// <summary>
            /// Free-stream Mach at this cell. Stored per cell rather than read off the x-axis because
            /// a speed-parameterised sweep (fixed m/s columns) has a Mach that varies down each column
            /// with the local speed of sound.
            /// </summary>
            public double Mach;

            /// <summary>True airspeed, m/s. Mach means different speeds at different altitudes.</summary>
            public double SpeedMPerS;

            public double DragN;

            /// <summary>Cruise drag split by source (WingDragN + BodyDragN + RamDragN == DragN), for
            /// the flight prediction-check diagnostic. Body is the voxel aero sections; Ram is the
            /// intake spillage drag flight adds at part throttle.</summary>
            public double WingDragN;
            public double BodyDragN;
            public double RamDragN;

            /// <summary>Wing drag subdivided (WingInducedDragN + WingProfileDragN == WingDragN in
            /// attached flow). Profile is zero-lift + skin friction.</summary>
            public double WingInducedDragN;
            public double WingProfileDragN;

            /// <summary>Cruise lift split by source (WingLiftN + BodyLiftN == weight), for the flight
            /// prediction-check diagnostic — to check whether the body over-generates lift.</summary>
            public double WingLiftN;
            public double BodyLiftN;

            /// <summary>
            /// Pitch-moment coefficient at the cruise trim. The cruise solve balances lift only, not
            /// pitch, so a large value flags that the cell is flying an untrimmed attitude — i.e. the
            /// drag is missing the control-deflection trim drag that flight carries.
            /// </summary>
            public double CmCruise;

            /// <summary>Trim control input at cruise, -1..1. Near +-1 means the trim saturated the
            /// deflection limit — pair with a non-zero CmCruise to spot an un-trimmable high-AoA cell.</summary>
            public double CruisePitch;

            /// <summary>Sustainable thrust along the relative wind, N — every engine held at or below its
            /// temperature limit (thermally capped), so this is the max thrust the aircraft can hold, not a
            /// melting full-throttle figure.</summary>
            public double ThrustAvailableN;

            /// <summary>Positive when the vehicle can accelerate or climb here, N (against sustainable thrust).</summary>
            public double ExcessThrustN;

            /// <summary>
            /// Best steady rate of climb at CONSTANT true airspeed, m/s — specific excess power, V*(T-D)/W,
            /// at full (thermally capped) thrust. This is V/S with no speed traded: the excess thrust climbs
            /// the weight at angle gamma where sin(gamma) = (T-D)/W. Capped at V (sin(gamma) <= 1): where the
            /// excess would need a steeper-than-vertical climb, the surplus accelerates instead of climbing.
            /// Negative on a walled cell — the best sustained vertical speed is downward.
            /// </summary>
            public double ClimbRateMPerS;

            /// <summary>Throttle holding T = D, or NaN if drag cannot be matched.</summary>
            public double ThrottleRequired;

            public double FuelFlowKgPerS;

            public double LiftToDrag;

            /// <summary>Breguet endurance at the cruise throttle, hours.</summary>
            public double EnduranceHours;

            // Inlet bookkeeping at full throttle, for diffing against flight's SolverFlightSys.
            public double InletArea;
            public double EngineArea;
            public double AreaRatio;
            public double OverallTpr;
            public double AmbientP;
            public double AmbientT;
            public double InletP;
            public double InletT;
        }

        /// <summary>
        /// Walks the editor ship for mass, CoM, reference area and burnable fuel. Mirrors the
        /// mass/area loop in StabilityDerivCalculator, with the addition of usable-fuel accounting,
        /// which FAR has no existing notion of.
        /// </summary>
        public VehicleProperties ComputeVehicleProperties(EditorEngineDeck deck)
        {
            var props = new VehicleProperties();

            List<Part> partsList = EditorLogic.SortedShipList;
            if (partsList == null || partsList.Count == 0)
                return props;

            HashSet<int> burnableIds = deck.GetBurnablePropellantIds();

            double mass = 0;
            double area = 0;
            double fuelMass = 0;
            Vector3d CoM = Vector3d.zero;

            foreach (Part p in partsList)
            {
                if (FARAeroUtil.IsNonphysical(p))
                    continue;

                double partMass = p.mass;
                if (p.Resources.Count > 0)
                    partMass += p.GetResourceMass();

                // p.mass already includes module mass in the editor; starting from p.partInfo.mass
                // would be required to use GetModuleMass instead.
                CoM += partMass * (Vector3d)p.transform.TransformPoint(p.CoMOffset);
                mass += partMass;

                foreach (PartResource res in p.Resources)
                {
                    if (res.info == null || !burnableIds.Contains(res.info.id))
                        continue;
                    fuelMass += res.amount * res.info.density;
                }

                FARWingAerodynamicModel w = p.GetComponent<FARWingAerodynamicModel>();
                if (w == null || w.isShielded)
                    continue;

                area += w.S;
            }

            if (mass <= 0)
                return props;

            CoM /= mass;

            // Wingless craft fall back to the body reference area, as the other editor sims do.
            if (area.NearlyEqual(0))
                area = _instantCondition._maxCrossSectionFromBody;

            if (area <= 0)
                return props;

            props.Valid = true;
            props.CoM = CoM;
            props.Area = area;
            props.MassKg = mass * 1000;
            props.UsableFuelKg = fuelMass * 1000;
            return props;
        }

        /// <summary>
        /// Vehicle properties at <paramref name="count" /> masses from dry (empty tanks) up to wet
        /// (full), ascending. Only mass and usable fuel vary — CoM and reference area are held at the
        /// wet values, an approximation that ignores the CoM shift as tanks drain. Used to sweep the
        /// envelope at several masses so the flight scene can interpolate it as fuel burns.
        /// </summary>
        public VehicleProperties[] ComputeMassSamples(EditorEngineDeck deck, int count)
        {
            if (count < 1)
                count = 1;

            VehicleProperties wet = ComputeVehicleProperties(deck);
            var samples = new VehicleProperties[count];

            if (!wet.Valid)
            {
                for (int k = 0; k < count; k++)
                    samples[k] = wet; // all invalid; caller checks Valid
                return samples;
            }

            double dryMass = wet.MassKg - wet.UsableFuelKg;
            double fuel = wet.UsableFuelKg;

            for (int k = 0; k < count; k++)
            {
                // Never sample a truly dry (0% fuel) load: with no burnable fuel a cell has no cruise
                // and no range/endurance, so interpolating a near-empty aircraft toward it would blank
                // the range. Keep a sliver of fuel in the leanest sample; the Breguet m1 stays the true
                // dry mass (set by the caller), so the range there is small but real, not NaN.
                double frac = count <= 1 ? 1.0 : (double)k / (count - 1);
                frac = Math.Max(frac, MinFuelFraction);
                VehicleProperties s = wet; // struct copy keeps CoM and Area
                s.MassKg = dryMass + frac * fuel;
                s.UsableFuelKg = frac * fuel;
                samples[k] = s;
            }

            return samples;
        }

        /// <summary>
        /// A sensible top of the altitude sweep for an air-breather on this body: the altitude where
        /// static pressure has fallen to a small fraction of its sea-level value.
        ///
        /// Derived rather than hardcoded because scale height varies enormously between bodies —
        /// Kerbin's is roughly 5.6 km against Earth's 8.5 km, so an altitude range that frames a
        /// jet's envelope on one badly misframes it on the other. Anchoring on a pressure ratio
        /// tracks the physics that actually ends the envelope.
        /// </summary>
        public static double SuggestedMaxAltitude(CelestialBody body, double pressureFraction = 0.02)
        {
            if (body == null || !body.atmosphere)
                return 0;

            double ut = Planetarium.GetUniversalTime();
            double seaLevel = FARAtmosphere.GetPressure(body, new Vector3d(0, 0, 0), ut);
            if (seaLevel <= 0)
                return 0;

            double target = seaLevel * pressureFraction;
            double low = 0;
            double high = body.atmosphereDepth;

            // Pressure decreases monotonically with altitude, so bisection is safe and needs no
            // assumption about the curve's shape (bodies may use an arbitrary pressureCurve).
            if (FARAtmosphere.GetPressure(body, new Vector3d(0, 0, high), ut) >= target)
                return high;

            for (int i = 0; i < 40; i++)
            {
                double mid = 0.5 * (low + high);
                if (FARAtmosphere.GetPressure(body, new Vector3d(0, 0, mid), ut) > target)
                    low = mid;
                else
                    high = mid;
            }

            return 0.5 * (low + high);
        }

        /// <summary>
        /// Unit vector along the relative wind, editor world space, at zero sideslip and roll.
        /// Matches the basis InstantConditionSim.GetClCdCmSteady builds its velocity from.
        /// </summary>
        private static void Directions(double alphaDeg, out Vector3d velocity, out Vector3d liftDown)
        {
            Vector3d forward = Vector3d.forward;
            Vector3d up = Vector3d.up;

            if (EditorDriver.editorFacility == EditorFacility.VAB)
            {
                forward = Vector3d.up;
                up = -Vector3d.forward;
            }

            double sinAlpha = Math.Sin(alphaDeg * Math.PI / 180);
            double cosAlpha = Math.Cos(alphaDeg * Math.PI / 180);

            velocity = forward * cosAlpha - up * sinAlpha;
            velocity.Normalize();

            liftDown = -forward * sinAlpha - up * cosAlpha;
            liftDown.Normalize();
        }

        /// <summary>
        /// Finds the AoA where lift balances <paramref name="netWeightN" />. Returns false when no
        /// such AoA exists — typically past the stall, or beyond the vehicle's lift capability at
        /// this dynamic pressure, which is exactly how the envelope's low-speed edge shows up.
        /// </summary>
        private bool TryTrim(
            double mach,
            double netWeightN,
            double q,
            double area,
            Vector3d CoM,
            int flapSetting,
            bool spoilers,
            double alphaGuess,
            out double alpha,
            out InstantConditionSimOutput output,
            double pitch = 0
        )
        {
            double neededCl = netWeightN / (q * area);

            _instantCondition.SetState(mach, neededCl, CoM, pitch, flapSetting, spoilers);
            FARMathUtil.OptimizationResult result = FARMathUtil.Secant(_instantCondition.FunctionIterateForAlpha,
                                                                       alphaGuess,
                                                                       alphaGuess + 2,
                                                                       1e-4,
                                                                       1e-4,
                                                                       minLimit: -90,
                                                                       maxLimit: 90);

            output = _instantCondition.iterationOutput;
            alpha = result.Result;

            if (!result.Converged || !FARMathUtil.IsFinite(alpha))
            {
                alpha = alphaGuess;
                return false;
            }

            return true;
        }

        /// <summary>
        /// Trims the aircraft for level cruise in BOTH pitch and lift: finds the pitch-control setting
        /// that zeroes the pitching moment while, at each setting, the angle of attack is solved to
        /// hold Cl = weight. Real cruise is trimmed — on a tailless delta the elevons deflect to
        /// balance, which adds control drag and forces a higher AoA (more induced drag) — so solving
        /// lift alone (the previous behaviour) under-predicts cruise drag by exactly that trim drag.
        ///
        /// Falls back to the neutral-control (lift-only) trim when the craft has no pitch authority or
        /// the moment simply cannot be zeroed, so an uncontrolled airframe behaves as before.
        /// </summary>
        private bool TrimCruise(
            double mach,
            double netWeightN,
            double q,
            double area,
            Vector3d CoM,
            int flapSetting,
            bool spoilers,
            double alphaGuess,
            out double alpha,
            out double pitch,
            out InstantConditionSimOutput output
        )
        {
            double a = alphaGuess;
            InstantConditionSimOutput o = default;

            // Cm after trimming AoA for Cl = weight at this pitch-control setting. NaN when no AoA
            // holds lift, which excludes that deflection from the bracket below.
            double CmAtPitch(double p)
            {
                bool trimmedAoA = TryTrim(mach, netWeightN, q, area, CoM, flapSetting, spoilers, a,
                                          out double solvedAlpha, out o, p);
                if (trimmedAoA)
                    a = solvedAlpha;
                return trimmedAoA ? o.Cm : double.NaN;
            }

            // Zero Cm across the pitch control with Brent's method, bracketed at full deflection each
            // way. Secant used to quit on its own step size while the moment residual was still
            // non-zero, leaving high-AoA cells under-deflected and their cruise drag under-predicted;
            // a bracketing method converges on the moment itself (same reason the throttle solve uses
            // Brent). Prefer a bracket that includes neutral, so the tighter side is used when the
            // opposite full deflection cannot hold lift.
            double cmMid = CmAtPitch(0);
            double cmNeg = CmAtPitch(-1);
            double cmPos = CmAtPitch(1);

            double lo = double.NaN, hi = double.NaN;
            if (FARMathUtil.IsFinite(cmMid) && FARMathUtil.IsFinite(cmPos) && cmMid * cmPos <= 0)
            {
                lo = 0;
                hi = 1;
            }
            else if (FARMathUtil.IsFinite(cmMid) && FARMathUtil.IsFinite(cmNeg) && cmMid * cmNeg <= 0)
            {
                lo = -1;
                hi = 0;
            }
            else if (FARMathUtil.IsFinite(cmNeg) && FARMathUtil.IsFinite(cmPos) && cmNeg * cmPos <= 0)
            {
                lo = -1;
                hi = 1;
            }

            if (!double.IsNaN(lo))
            {
                FARMathUtil.OptimizationResult res = FARMathUtil.BrentsMethod(CmAtPitch, lo, hi, 1e-4, 100);
                if (res.Converged && FARMathUtil.IsFinite(res.Result))
                {
                    // Re-trim AoA at the solved pitch so alpha and coefficients are consistent.
                    bool okTrim = TryTrim(mach, netWeightN, q, area, CoM, flapSetting, spoilers, a,
                                          out alpha, out output, res.Result);
                    if (okTrim)
                    {
                        pitch = res.Result;
                        return true;
                    }
                }
            }
            else if (FARMathUtil.IsFinite(cmNeg) && FARMathUtil.IsFinite(cmPos) &&
                     Math.Abs(cmPos - cmNeg) > PitchAuthorityEpsilon)
            {
                // The controls move the moment but no deflection in range zeroes it: the attitude is
                // not holdable, so report un-trimmable rather than a bogus under-trimmed cruise that
                // would show an unreachable cell as flyable.
                pitch = 0;
                alpha = alphaGuess;
                output = default;
                return false;
            }

            // No pitch authority (or a full deflection that cannot hold lift): trim at neutral, the
            // way an uncontrolled airframe always did.
            pitch = 0;
            return TryTrim(mach, netWeightN, q, area, CoM, flapSetting, spoilers, alphaGuess,
                           out alpha, out output, 0);
        }

        /// <summary>
        /// The per-cell aerodynamics — dynamic pressure, weight and the cruise trim (drag, L/D) —
        /// that every engine configuration at this flight point shares. Produced by
        /// <see cref="BeginCell" /> and consumed by <see cref="EvaluateWithAero" />, so evaluating a
        /// second or third engine set for the same cell (the per-type and combined maps) does not
        /// repeat the expensive InstantConditionSim solve.
        /// </summary>
        public struct AeroContext
        {
            public bool Valid;   // gas/q usable
            public bool Trimmed; // a cruise AoA holding L = W was found
            public double Mach;
            public double SpeedMPerS;
            public double Q;
            public double WeightN;
            public double Alpha;
            public Vector3d CruiseVelDir;
            public double DragN;
            public double WingDragN;
            public double BodyDragN;
            public double WingInducedDragN;
            public double WingProfileDragN;
            public double WingLiftN;
            public double BodyLiftN;
            public double CmCruise;
            public double CruisePitch;
            public double LiftToDrag;
            public InstantConditionSimOutput CruiseOutput;
            public int FlapSetting;
            public bool Spoilers;
        }

        /// <summary>
        /// Solves the shared aerodynamics for one cell and sets the real viscous AeroCondition on the
        /// InstantConditionSim. The caller MUST pair this with <see cref="EndCell" /> to clear that
        /// condition — it is instance state on the shared sim, and the stability sweep runs against
        /// the same object, so leaving it set would silently change results meant to match FAR's own
        /// tab. Between the two, evaluate as many engine sets as needed with <see cref="EvaluateWithAero" />.
        /// </summary>
        /// <summary>
        /// The atmosphere reads a cell's trim needs — the viscous condition and the gas properties.
        /// Both call into KSP's CelestialBody/Planetarium, which are main-thread-only, so the parallel
        /// sweep computes them here on the main thread and hands them to <see cref="BeginCellPrecomputed" />.
        /// </summary>
        public void GetCellAtmosphere(
            CelestialBody body,
            double alt,
            double mach,
            out GasProperties gas,
            out SimAeroCondition aeroCond
        )
        {
            gas = EditorAtmosphere.GetGasProperties(body, alt);
            aeroCond = SimAeroCondition.ForFlightPoint(body, alt, mach, _instantCondition._bodyLength);
        }

        /// <summary>Sets the sim's (thread-local) viscous condition for the calling thread's cell.</summary>
        public void SetAeroCondition(SimAeroCondition? aeroCond)
        {
            _instantCondition.AeroCondition = aeroCond;
        }

        public AeroContext BeginCell(
            CelestialBody body,
            double alt,
            double mach,
            int flapSetting,
            bool spoilers,
            VehicleProperties props,
            double alphaSeed = 0
        )
        {
            GetCellAtmosphere(body, alt, mach, out GasProperties gas, out SimAeroCondition aeroCond);
            return BeginCellPrecomputed(body, alt, mach, flapSetting, spoilers, props, alphaSeed, gas, aeroCond);
        }

        /// <summary>
        /// The thread-safe core of <see cref="BeginCell" />: given the pre-sampled atmosphere (see
        /// <see cref="GetCellAtmosphere" />), solves the cruise trim. Reads only plain data and the
        /// frozen geometry snapshot, so it runs on the parallel sweep's worker threads.
        /// </summary>
        public AeroContext BeginCellPrecomputed(
            CelestialBody body,
            double alt,
            double mach,
            int flapSetting,
            bool spoilers,
            VehicleProperties props,
            double alphaSeed,
            GasProperties gas,
            SimAeroCondition aeroCond
        )
        {
            _instantCondition.AeroCondition = aeroCond;

            var ctx = new AeroContext {Mach = mach, FlapSetting = flapSetting, Spoilers = spoilers};
            if (!props.Valid)
                return ctx;

            if (gas.Pressure <= 0)
                return ctx;

            double u0 = gas.SpeedOfSound * mach;
            double q = 0.5 * gas.Density * u0 * u0;
            if (q <= 0)
                return ctx;

            ctx.Valid = true;
            ctx.SpeedMPerS = u0;
            ctx.Q = q;

            // Gravity, less the centrifugal relief of orbital speed — significant at high Mach.
            double effectiveG = InstantConditionSim.CalculateAccelerationDueToGravity(body, alt);
            effectiveG -= u0 * u0 / (alt + body.Radius);
            ctx.WeightN = props.MassKg * effectiveG;

            // Cruise trim: thrust only balances drag here, so its lift component is D*tan(alpha) —
            // a fraction of a percent of weight at any condition where endurance is worth reading.
            // Taken as zero. Solves pitch AND lift together so the drag includes trim drag (thrust's
            // small lift component is still ignored; the elevon deflection to zero Cm is not).
            ctx.Trimmed = TrimCruise(mach,
                                     ctx.WeightN,
                                     q,
                                     props.Area,
                                     props.CoM,
                                     flapSetting,
                                     spoilers,
                                     alphaSeed,
                                     out double alphaCruise,
                                     out double cruisePitch,
                                     out InstantConditionSimOutput cruiseOutput);

            ctx.Alpha = alphaCruise;
            ctx.CruisePitch = cruisePitch;
            ctx.CruiseOutput = cruiseOutput;

            if (ctx.Trimmed)
            {
                Directions(alphaCruise, out Vector3d cruiseVelDir, out Vector3d _cruiseLiftDown);
                ctx.CruiseVelDir = cruiseVelDir;
                ctx.DragN = cruiseOutput.Cd * q * props.Area;
                ctx.WingDragN = cruiseOutput.WingCd * q * props.Area;
                ctx.BodyDragN = cruiseOutput.BodyCd * q * props.Area;
                ctx.WingInducedDragN = cruiseOutput.WingCdInduced * q * props.Area;
                ctx.WingProfileDragN = cruiseOutput.WingCdProfile * q * props.Area;
                ctx.WingLiftN = cruiseOutput.WingCl * q * props.Area;
                ctx.BodyLiftN = cruiseOutput.BodyCl * q * props.Area;
                ctx.CmCruise = cruiseOutput.Cm;
                ctx.LiftToDrag = cruiseOutput.Cd > 0 ? cruiseOutput.Cl / cruiseOutput.Cd : 0;
            }

            return ctx;
        }

        /// <summary>Clears the AeroCondition set by <see cref="BeginCell" />. Always call this after a cell.</summary>
        public void EndCell()
        {
            _instantCondition.AeroCondition = null;
        }

        /// <summary>
        /// Completes one cell for a single engine configuration (the deck's active group), reusing the
        /// aerodynamics <see cref="BeginCell" /> already solved. Only the engine-dependent work is done
        /// here — full-throttle thrust, the thrust-lift feasibility trim, and the cruise-throttle solve.
        /// </summary>
        /// <summary>
        /// Serial wrapper: samples the deck's live engines directly. Used by the single-threaded sweep
        /// and the Engine Deck tab.
        /// </summary>
        public CellResult EvaluateWithAero(
            AeroContext ctx,
            CelestialBody body,
            double alt,
            double mach,
            EditorEngineDeck deck,
            VehicleProperties props
        )
        {
            return EvaluateWithAeroCore(ctx,
                                        mach,
                                        props,
                                        deck.Ready,
                                        t => deck.Evaluate(body, alt, mach, t),
                                        deck.IntakeRamDragMagnitudeN(mach, ctx.Q, ctx.CruiseVelDir));
        }

        /// <summary>
        /// Parallel wrapper: samples per-worker solver clones through <see cref="EditorEngineDeck.EvaluateEnv" />
        /// against the pre-built cell environment, so the whole engine pass runs on worker threads.
        /// </summary>
        public CellResult EvaluateWithAeroParallel(
            AeroContext ctx,
            EditorEngineDeck deck,
            in EditorEngineDeck.CellEnv env,
            EditorEngineDeck.ParallelGroupContext group,
            int slot,
            VehicleProperties props
        )
        {
            // env is passed by 'in' and cannot be captured by the lambda; copy the pieces it needs.
            EditorEngineDeck.CellEnv cellEnv = env;
            return EvaluateWithAeroCore(ctx,
                                        ctx.Mach,
                                        props,
                                        deck.Ready,
                                        t => deck.EvaluateEnv(cellEnv, group, slot, t),
                                        deck.IntakeRamDragMagnitudeN(ctx.Mach, ctx.Q, ctx.CruiseVelDir));
        }

        private CellResult EvaluateWithAeroCore(
            AeroContext ctx,
            double mach,
            VehicleProperties props,
            bool deckReady,
            Func<double, EditorEngineDeck.Sample> sampleEngines,
            double ramDragMagN
        )
        {
            var cell = new CellResult
            {
                ThrottleRequired = double.NaN,
                EnduranceHours = double.NaN,
                Mach = ctx.Mach,
                SpeedMPerS = ctx.SpeedMPerS
            };

            if (!ctx.Valid || !props.Valid || !deckReady)
                return cell;

            double q = ctx.Q;
            double weightN = ctx.WeightN;

            // Thrust does not depend on AoA, so one full-throttle sample serves the whole cell.
            EditorEngineDeck.Sample maxSample = sampleEngines(1.0);

            cell.InletArea = maxSample.InletArea;
            cell.EngineArea = maxSample.EngineArea;
            cell.AreaRatio = maxSample.AreaRatio;
            cell.OverallTpr = maxSample.OverallTpr;
            cell.AmbientP = maxSample.AmbientP;
            cell.AmbientT = maxSample.AmbientT;
            cell.InletP = maxSample.InletP;
            cell.InletT = maxSample.InletT;

            cell.Trimmed = ctx.Trimmed;
            cell.Alpha = ctx.Alpha;

            if (!ctx.Trimmed)
                return cell;

            cell.DragN = ctx.DragN;
            cell.WingDragN = ctx.WingDragN;
            cell.BodyDragN = ctx.BodyDragN;
            cell.WingInducedDragN = ctx.WingInducedDragN;
            cell.WingProfileDragN = ctx.WingProfileDragN;
            cell.WingLiftN = ctx.WingLiftN;
            cell.BodyLiftN = ctx.BodyLiftN;
            cell.CmCruise = ctx.CmCruise;
            cell.CruisePitch = ctx.CruisePitch;
            cell.LiftToDrag = ctx.LiftToDrag;

            // Feasibility trim is a separate question and needs its own trim. At full throttle the
            // thrust vector carries a real share of the weight, lowering the AoA needed and with it
            // the drag — which is precisely what decides whether the ceiling is here or higher up.
            // Thrust-dependent, so it is redone for each engine set (the one cheap-ish aero cost that
            // does not reuse across sets); the cruise trim above is shared.
            double alphaMax = ctx.Alpha;
            double thrustLift = 0;
            bool trimmedMax = true;
            InstantConditionSimOutput maxOutput = ctx.CruiseOutput;

            for (int i = 0; i < ThrustLiftIterations; i++)
            {
                Directions(alphaMax, out Vector3d _, out Vector3d liftDown);
                thrustLift = -Vector3d.Dot(maxSample.Thrust, liftDown);

                // Thrust alone can exceed what is needed to hold the aircraft up; clamp so the trim
                // target stays a lifting condition rather than flipping sign and oscillating.
                thrustLift = FARMathUtil.Clamp(thrustLift, 0, weightN);

                // Reuse the cruise pitch trim rather than solving pitch again here — a good enough
                // approximation of the elevon setting at full throttle, and it keeps the feasibility
                // trim's drag consistent with cruise without a second nested solve per engine set.
                trimmedMax = TryTrim(mach,
                                     weightN - thrustLift,
                                     q,
                                     props.Area,
                                     props.CoM,
                                     ctx.FlapSetting,
                                     ctx.Spoilers,
                                     alphaMax,
                                     out alphaMax,
                                     out maxOutput,
                                     ctx.CruisePitch);

                if (!trimmedMax)
                    break;
            }

            // Fall back to the cruise trim if the full-throttle trim would not settle.
            if (!trimmedMax)
            {
                alphaMax = ctx.Alpha;
                maxOutput = ctx.CruiseOutput;
            }

            Directions(alphaMax, out Vector3d maxVelDir, out Vector3d _2);

            double dragAtMax = maxOutput.Cd * q * props.Area;
            cell.ThrustAvailableN = Vector3d.Dot(maxSample.Thrust, maxVelDir);
            cell.ExcessThrustN = cell.ThrustAvailableN - dragAtMax;

            // Steady climb rate at constant speed: V * sin(gamma), sin(gamma) = (T-D)/W clamped to [-1, 1].
            // Uses the full-throttle feasibility trim (thrust already carries part of the weight, so dragAtMax
            // is the reduced-lift drag of a climb), so this is the best sustained V/S with no speed traded.
            cell.ClimbRateMPerS = ctx.SpeedMPerS * FARMathUtil.Clamp(cell.ExcessThrustN / weightN, -1, 1);

            // maxSample is already thermally capped: each engine is held at its own temperature limit
            // (a heat-managing pilot's per-engine thrust limiter), so ThrustAvailableN is the SUSTAINABLE
            // max, not a melting full-throttle figure. A cell whose thermally-limited thrust cannot reach
            // drag is walled; Overheats marks that the wall is a heat wall (an engine is at its limiter),
            // which is also what tells the combined map to shed its turbojets to ram power.
            if (cell.ExcessThrustN < 0)
            {
                cell.ThermalRatio = maxSample.MaxThermalRatio; // report the (uncapped) hot ratio even when walled
                cell.Overheats = maxSample.ThermalLimited;
                return cell;
            }

            if (props.UsableFuelKg <= 0)
            {
                cell.Feasible = true;
                cell.Cruise = CruiseState.NoBurnableFuel;
                cell.ThermalRatio = maxSample.MaxThermalRatio;
                cell.Overheats = maxSample.ThermalLimited;
                return cell;
            }

            // Cruise throttle: engine-only root find, so it costs no aero evaluations. Drag is held
            // at the trimmed value — re-trimming per throttle iteration would multiply the cell cost
            // by the aero solve for a correction that is small wherever endurance is worth reading.
            // The thermal ratio is captured AT the cruise throttle: a ramjet runs cooler at part
            // throttle, so a cell it flies through at 30% must not be walled by its full-throttle
            // temperature. A jet's temp is throttle-independent, so this still walls it correctly.
            cell.Cruise = TrySolveCruiseThrottle(sampleEngines,
                                                 ctx.CruiseVelDir,
                                                 ctx.DragN,
                                                 ramDragMagN,
                                                 out double throttle,
                                                 out double fuelFlow,
                                                 out double thermalAtCruise,
                                                 out double cruiseDragN);
            cell.ThermalRatio = thermalAtCruise;

            switch (cell.Cruise)
            {
                case CruiseState.Ok:
                    cell.Feasible = true;
                    cell.ThrottleRequired = throttle;
                    cell.FuelFlowKgPerS = fuelFlow;
                    // Overheats means "flown on a reduced thrust limiter here". Keyed on the FULL-throttle
                    // sample, not the cruise throttle: a heat-managing pilot sets the limiter for full-throttle
                    // safety (so an instant firewall does not melt the engine) and leaves it there, so the cell
                    // runs limited even when the cruise throttle alone would be cool. On the combined map this
                    // also flags the turbojet shed. maxSample already thermally caps ThrustAvailableN.
                    cell.Overheats = maxSample.ThermalLimited;
                    // Add intake spillage drag to the reported cruise drag. Lift (L/D * drag) is held
                    // fixed so endurance tracks the higher fuel burn, not a changed lift.
                    cell.RamDragN = cruiseDragN - ctx.DragN;
                    cell.LiftToDrag = ctx.LiftToDrag * ctx.DragN / cruiseDragN;
                    cell.DragN = cruiseDragN;
                    cell.EnduranceHours = Endurance(cell.DragN, fuelFlow, cell.LiftToDrag, props);
                    return cell;

                case CruiseState.IdleExceedsDrag:
                    // Flyable — the aircraft simply cannot hold this speed steadily (it would
                    // accelerate). Feasible, just not a cruise point.
                    cell.Feasible = true;
                    return cell;

                default:
                    // Unsolved: enough thrust existed but no throttle trimmed a cruise. Leave it
                    // infeasible.
                    return cell;
            }
        }

        /// <summary>Evaluates one cell for a single engine set — BeginCell/EvaluateWithAero/EndCell in one call.</summary>
        public CellResult EvaluateCell(
            CelestialBody body,
            double alt,
            double mach,
            int flapSetting,
            bool spoilers,
            EditorEngineDeck deck,
            VehicleProperties props
        )
        {
            AeroContext ctx = BeginCell(body, alt, mach, flapSetting, spoilers, props);
            try
            {
                return EvaluateWithAero(ctx, body, alt, mach, deck, props);
            }
            finally
            {
                EndCell();
            }
        }

        /// <summary>
        /// Finds the throttle where thrust along the wind equals <paramref name="dragN" />.
        /// </summary>
        private static CruiseState TrySolveCruiseThrottle(
            Func<double, EditorEngineDeck.Sample> sampleEngines,
            Vector3d velDir,
            double dragN,
            double ramDragMagN,
            out double throttle,
            out double fuelFlow,
            out double thermalAtThrottle,
            out double totalDragN
        )
        {
            throttle = double.NaN;
            fuelFlow = 0;
            thermalAtThrottle = 0;
            totalDragN = dragN;

            double lastFlow = 0;
            double lastThermal = 0;

            // Intake spillage drag falls with throttle and is zero above half throttle, so it belongs
            // inside the root-find: the cruise throttle and the drag it balances stay consistent.
            double RamDrag(double t)
            {
                return t >= 0.5 ? 0 : ramDragMagN * (1 - 2 * t);
            }

            double Excess(double t)
            {
                EditorEngineDeck.Sample s = sampleEngines(t);
                lastFlow = s.FuelFlow;
                lastThermal = s.MaxThermalRatio;
                return Vector3d.Dot(s.Thrust, velDir) - (dragN + RamDrag(t));
            }

            // A closed throttle that still out-thrusts drag means no steady cruise exists at this
            // speed — the aircraft would accelerate out of it. Worth detecting up front: there is no
            // root to find, and the solver would otherwise just wander to its limit and report a
            // generic failure.
            if (Excess(0) > 0)
            {
                thermalAtThrottle = lastThermal; // idle temp, the coolest the engine runs
                totalDragN = dragN + RamDrag(0);
                return CruiseState.IdleExceedsDrag;
            }

            // Brent rather than Secant: an afterburning engine's thrust is not smooth in throttle.
            // AJE saturates the dry spool and lights the afterburner at the same point —
            //   mainThrottle = min(1.5t, 1);  abThrottle = max(3t - 2, 0)
            // — putting two slope discontinuities at t = 2/3. A derivative-based solver thrashes
            // across that kink, which is exactly where cells near the envelope boundary land. The
            // bracket [0, 1] is known good here, and bracketing methods do not care about kinks.
            FARMathUtil.OptimizationResult result = FARMathUtil.BrentsMethod(Excess, 0, 1, 1e-3, 100);

            if (!result.Converged || !FARMathUtil.IsFinite(result.Result))
                return CruiseState.Unsolved;

            throttle = FARMathUtil.Clamp(result.Result, 0, 1);

            // Brent's last probe is not necessarily the converged point, so resample to make the
            // reported flow and temperature correspond to the reported throttle.
            Excess(throttle);
            fuelFlow = lastFlow;
            thermalAtThrottle = lastThermal;
            totalDragN = dragN + RamDrag(throttle);
            // Brent already converged on a real T = D point (a genuine solver failure returned above), so a
            // zero fuel flow here just means the cruise throttle rounded to near-idle: thrust hugely exceeds
            // drag, so the engine holds this speed on almost nothing. That is flyable — just with no
            // meaningful endurance figure — not a failure. Walling it hides a strong engine's whole map
            // (a turbojet down low, where it out-thrusts drag by 100+ kN).
            return fuelFlow > 0 ? CruiseState.Ok : CruiseState.IdleExceedsDrag;
        }

        /// <summary>
        /// Breguet endurance for a jet, in hours: E = Isp * (L/D) * ln(m0/m1).
        ///
        /// Follows the convention already used for the flight-scene readout in PhysicsCalcs, which
        /// expresses it as (L/D)/tSFC * ln(m0/m1) with tSFC = 3600/Isp. AJE reports SFC with the
        /// same 3600/Isp definition, so the two agree without conversion. Unlike the flight version,
        /// m1 here counts only fuel the engines can actually burn rather than every resource aboard.
        /// </summary>
        private static double Endurance(double dragN, double fuelFlowKgPerS, double liftToDrag, VehicleProperties props)
        {
            double emptyMass = props.MassKg - props.UsableFuelKg;
            return EnduranceHours(dragN, fuelFlowKgPerS, liftToDrag, props.MassKg, emptyMass);
        }

        /// <summary>
        /// Breguet endurance in hours from the current mass down to <paramref name="dryMassKg" />:
        /// E = Isp * (L/D) * ln(m0/m1). Public so the flight scene can recompute it from interpolated
        /// aerodynamics against the aircraft's live mass, rather than interpolating a cached figure.
        /// </summary>
        public static double EnduranceHours(
            double dragN,
            double fuelFlowKgPerS,
            double liftToDrag,
            double currentMassKg,
            double dryMassKg
        )
        {
            if (fuelFlowKgPerS <= 0 || liftToDrag <= 0 || dryMassKg <= 0 || currentMassKg <= dryMassKg)
                return double.NaN;

            // Effective Isp of the whole installation at the cruise point. Thrust equals drag here.
            double isp = dragN / (fuelFlowKgPerS * G0);
            if (!FARMathUtil.IsFinite(isp) || isp <= 0)
                return double.NaN;

            return isp * liftToDrag * Math.Log(currentMassKg / dryMassKg) / 3600;
        }

        /// <summary>Cruise range for a cell, km: endurance (hr) * true airspeed (m/s) * 3.6.</summary>
        public static double RangeKm(CellResult c)
        {
            return FARMathUtil.IsFinite(c.EnduranceHours) ? c.EnduranceHours * c.SpeedMPerS * 3.6 : double.NaN;
        }
    }
}
