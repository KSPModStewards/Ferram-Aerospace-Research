/*
Ferram Aerospace Research v0.16.1.2 "Marangoni"
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
using System.Collections.Generic;
using System.Threading;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    internal class InstantConditionSim
    {
        // Per-call solver state is thread-local so the parallel heatmap sweep can trim
        // independent (Mach, altitude) cells concurrently against the one sim instance. The
        // single-threaded tabs use the calling (main) thread's state, unchanged.
        // Per-thread-slot solver state (see SimThreadContext); array indexing replaces ThreadLocal,
        // which was too slow under Mono to leave on the hot path.
        private readonly InstantConditionSimInput[] _iterationInputSlots =
            new InstantConditionSimInput[SimThreadContext.MaxSlots];

        private InstantConditionSimInput iterationInput
        {
            get
            {
                int s = SimThreadContext.Slot;
                InstantConditionSimInput v = _iterationInputSlots[s];
                if (v == null)
                {
                    v = new InstantConditionSimInput();
                    _iterationInputSlots[s] = v;
                }

                return v;
            }
        }

        private List<FARAeroSection> _currentAeroSections;
        private List<FARAeroPartModule> _currentAeroModules;
        private List<FARWingAerodynamicModel> _wingAerodynamicModel;

        internal List<FARWingAerodynamicModel> WingModels => _wingAerodynamicModel;

        public double _maxCrossSectionFromBody;
        public double _bodyLength;

        private readonly double[] _neededClSlots = new double[SimThreadContext.MaxSlots];

        private double neededCl
        {
            get => _neededClSlots[SimThreadContext.Slot];
            set => _neededClSlots[SimThreadContext.Slot] = value;
        }

        private readonly InstantConditionSimOutput[] _iterationOutputSlots =
            new InstantConditionSimOutput[SimThreadContext.MaxSlots];

        public InstantConditionSimOutput iterationOutput
        {
            get => _iterationOutputSlots[SimThreadContext.Slot];
            set => _iterationOutputSlots[SimThreadContext.Slot] = value;
        }

        // Sweep snapshot: geometry the aero solve reads from the main-thread-only Unity API,
        // captured once so worker threads can read plain data. Active only during a parallel sweep.
        private bool _simActive;
        private Vector3d _simCoM;
        private EditorFacility _simEditorFacility;

        /// <summary>
        /// Real viscous conditions for the point being simulated, or null to keep the historical
        /// fixed constants.
        ///
        /// This sim evaluates forces with a unit velocity vector and density 2 so that q works out
        /// to exactly 1 and forces read as coefficients. That destroys the Reynolds number, so both
        /// the wing model and the aero sections fall back to a hardcoded skin friction of 0.005.
        /// Measured against flight, that is roughly double the true value at sea-level Reynolds and
        /// below it at low Reynolds — drag comes out ~2x high near the ground and low at altitude.
        ///
        /// Setting this makes a caller's sweep use the real numbers. It is deliberately opt-in and
        /// defaults to null so Static Analysis and the stability derivative tabs keep their existing
        /// behaviour. Callers must clear it when done: it is instance state, and leaving it set
        /// would silently change results for whatever runs next.
        /// </summary>
        private readonly SimAeroCondition?[] _aeroConditionSlots =
            new SimAeroCondition?[SimThreadContext.MaxSlots];

        public SimAeroCondition? AeroCondition
        {
            get => _aeroConditionSlots[SimThreadContext.Slot];
            set => _aeroConditionSlots[SimThreadContext.Slot] = value;
        }

        public bool Ready
        {
            get { return _currentAeroSections != null && _currentAeroModules != null && _wingAerodynamicModel != null; }
        }

        public void UpdateAeroData(
            List<FARAeroPartModule> aeroModules,
            List<FARAeroSection> aeroSections,
            VehicleAerodynamics vehicleAero,
            List<FARWingAerodynamicModel> wingAerodynamicModel
        )
        {
            _currentAeroModules = aeroModules;
            _currentAeroSections = aeroSections;
            _wingAerodynamicModel = wingAerodynamicModel;
            _maxCrossSectionFromBody = vehicleAero.MaxCrossSectionArea;
            _bodyLength = vehicleAero.Length;
        }

        public static double CalculateAccelerationDueToGravity(CelestialBody body, double alt)
        {
            double radius = body.Radius + alt;
            double mu = body.gravParameter;

            double accel = radius * radius;
            accel = mu / accel;
            return accel;
        }

        public void GetClCdCmSteady(
            InstantConditionSimInput input,
            out InstantConditionSimOutput output,
            bool clear,
            bool reset_stall = false
        )
        {
            output = new InstantConditionSimOutput();

            double area = 0;
            double MAC = 0;
            double b_2 = 0;

            Vector3d forward = Vector3.forward;
            Vector3d up = Vector3.up;
            Vector3d right = Vector3.right;

            Vector3d CoM;

            // During a parallel sweep the ship's CoM and editor facility are fixed, so they are
            // snapshotted once on the main thread and a sweep worker reads the snapshot (the loop
            // below reads Unity part transforms, which are main-thread-only). Everywhere else,
            // the main thread's own evaluations during a sweep included, recompute them live
            // exactly as before.
            bool sweep = _simActive && SimThreadContext.InSweep;
            EditorFacility facility = sweep ? _simEditorFacility : EditorDriver.editorFacility;
            if (facility == EditorFacility.VAB)
            {
                forward = Vector3.up;
                up = -Vector3.forward;
            }

            if (sweep)
            {
                CoM = _simCoM;
            }
            else
            {
                CoM = Vector3d.zero;
                double mass = 0;
                List<Part> partsList = EditorLogic.SortedShipList;
                foreach (Part p in partsList)
                {
                    if (FARAeroUtil.IsNonphysical(p))
                        continue;

                    double partMass = p.mass;
                    if (p.Resources.Count > 0)
                        partMass += p.GetResourceMass();

                    // If you want to use GetModuleMass, you need to start from p.partInfo.mass, not p.mass
                    CoM += partMass * (Vector3d)p.transform.TransformPoint(p.CoMOffset);
                    mass += partMass;
                }

                CoM /= mass;
            }

            // Rodhern: The original reference directions (velocity, liftVector, sideways) did not form an orthonormal
            //  basis. That in turn produced some counterintuitive calculation results, such as coupled yaw and pitch
            //  derivatives. A more thorough discussion of the topic can be found on the KSP forums:
            //  https://forum.kerbalspaceprogram.com/index.php?/topic/19321-131-ferram-aerospace-research-v01591-liepmann-4218/&do=findComment&comment=2781270
            //  The reference directions have been replaced by new ones that are orthonormal by construction.
            //  In dkavolis branch Vector3.Cross() and Vector3d.Normalize() are used explicitly. There is no apparent
            //  benefit to this other than possibly improved readability.

            double sinAlpha = Math.Sin(input.alpha * Math.PI / 180);
            double cosAlpha = Math.Sqrt(Math.Max(1 - sinAlpha * sinAlpha, 0));

            double sinBeta = Math.Sin(input.beta * Math.PI / 180);
            double cosBeta = Math.Sqrt(Math.Max(1 - sinBeta * sinBeta, 0));

            double sinPhi = Math.Sin(input.phi * Math.PI / 180);
            double cosPhi = Math.Sqrt(Math.Max(1 - sinPhi * sinPhi, 0));

            double alphaDot = input.alphaDot * Math.PI / 180;
            double betaDot = input.betaDot * Math.PI / 180;
            double phiDot = input.phiDot * Math.PI / 180;

            Vector3d velocity = forward * cosAlpha * cosBeta;
            velocity += right * (sinPhi * sinAlpha * cosBeta + cosPhi * sinBeta);
            velocity += -up * (cosPhi * sinAlpha * cosBeta - sinPhi * sinBeta);
            velocity.Normalize();

            Vector3d liftDown = -forward * sinAlpha;
            liftDown += right * sinPhi * cosAlpha;
            liftDown += -up * cosPhi * cosAlpha;
            liftDown.Normalize();

            Vector3d sideways = Vector3.Cross(velocity, liftDown);
            sideways.Normalize();

            Vector3d angVel = forward * (phiDot - sinAlpha * betaDot);
            angVel += right * (cosPhi * alphaDot + cosAlpha * sinPhi * betaDot);
            angVel += up * (sinPhi * alphaDot - cosAlpha * cosPhi * betaDot);


            double? skinFriction = AeroCondition?.SkinFriction;
            double? condReynoldsPerLength = AeroCondition?.ReynoldsPerLength;

            foreach (FARWingAerodynamicModel w in _wingAerodynamicModel)
            {
                if (!(w && w.part))
                    continue;

                // Skin friction from THIS wing's own chord (MAC), the way flight computes it. A single
                // vehicle-length value understates wing skin friction badly, since a wing chord is far
                // shorter than the fuselage (higher Reynolds -> lower Cf), which under-predicts wing
                // profile drag. Reynolds/length is length-independent, so scale it by the wing's MAC.
                double wingMAC = w.GetMAC();
                double? wingSkinFriction = condReynoldsPerLength > 0 && wingMAC > 0
                                               ? FARAeroUtil.SkinFrictionDrag(condReynoldsPerLength.Value * wingMAC,
                                                                              input.machNumber)
                                               : skinFriction;

                w.ComputeForceEditor(velocity, input.machNumber, 2, wingSkinFriction);

                if (clear)
                    w.EditorClClear(reset_stall);

                Vector3d relPos = w.GetAerodynamicCenter() - CoM;

                Vector3d vel = velocity + Vector3d.Cross(angVel, relPos);

                if (w is FARControllableSurface controllableSurface)
                    controllableSurface.SetControlStateEditor(CoM,
                                                              vel,
                                                              (float)input.pitchValue,
                                                              0,
                                                              0,
                                                              input.flaps,
                                                              input.spoilers);
                else if (w.GetShielded())
                    continue;

                Vector3d force = w.ComputeForceEditor(vel.normalized, input.machNumber, 2, wingSkinFriction) * 1000;

                output.Cl += -Vector3d.Dot(force, liftDown);
                output.Cy += Vector3d.Dot(force, sideways);
                output.Cd += -Vector3d.Dot(force, velocity);

                output.WingCl += -Vector3d.Dot(force, liftDown);
                output.WingCd += -Vector3d.Dot(force, velocity);

                // Cd split accumulates area-weighted coefficients directly (the force above already
                // resolves to Cd*S, so these are on the same scale and sum to the wing Cd contribution).
                output.WingCdInduced += w.GetCdInduced() * w.S;
                output.WingCdProfile += w.GetCdProfile() * w.S;

                Vector3d moment = -Vector3d.Cross(relPos, force);

                output.Cm += Vector3d.Dot(moment, sideways);
                output.Cn += Vector3d.Dot(moment, liftDown);
                output.C_roll += Vector3d.Dot(moment, velocity);

                area += w.S;
                MAC += w.GetMAC() * w.S;
                b_2 += w.Getb_2() * w.S;
            }

            // Density stays 2 with a unit velocity vector: that is what makes q exactly 1, so these
            // forces come out as coefficients. Only the viscous terms are substituted.
            float reynoldsPerLength = AeroCondition.HasValue ? (float)AeroCondition.Value.ReynoldsPerLength : 10000;
            float pseudoKnudsen = AeroCondition.HasValue ? (float)AeroCondition.Value.PseudoKnudsen : 0;
            var sectionSkinFriction = (float)(skinFriction ?? 0.005);

            var center = new FARCenterQuery();
            foreach (FARAeroSection aeroSection in _currentAeroSections)
                aeroSection.PredictionCalculateAeroForces(2,
                                                          (float)input.machNumber,
                                                          reynoldsPerLength,
                                                          pseudoKnudsen,
                                                          sectionSkinFriction,
                                                          velocity.normalized,
                                                          center);

            Vector3d centerForce = center.force * 1000;

            output.Cl += -Vector3d.Dot(centerForce, liftDown);
            output.Cy += Vector3d.Dot(centerForce, sideways);
            output.Cd += -Vector3d.Dot(centerForce, velocity);

            output.BodyCl += -Vector3d.Dot(centerForce, liftDown);
            output.BodyCd += -Vector3d.Dot(centerForce, velocity);

            Vector3d centerMoment = -center.TorqueAt(CoM) * 1000;

            output.Cm += Vector3d.Dot(centerMoment, sideways);
            output.Cn += Vector3d.Dot(centerMoment, liftDown);
            output.C_roll += Vector3d.Dot(centerMoment, velocity);

            if (area.NearlyEqual(0))
            {
                area = _maxCrossSectionFromBody;
                b_2 = 1;
                MAC = _bodyLength;
            }

            double recipArea = 1 / area;

            MAC *= recipArea;
            b_2 *= recipArea;
            output.Cl *= recipArea;
            output.Cd *= recipArea;
            output.Cm *= recipArea / MAC;
            output.Cy *= recipArea;
            output.Cn *= recipArea / b_2;
            output.C_roll *= recipArea / b_2;

            output.WingCl *= recipArea;
            output.WingCd *= recipArea;
            output.BodyCl *= recipArea;
            output.BodyCd *= recipArea;
            output.WingCdInduced *= recipArea;
            output.WingCdProfile *= recipArea;
        }

        public void SetState(double machNumber, double Cl, Vector3d CoM, double pitch, int flapSetting, bool spoilers)
        {
            iterationInput.machNumber = machNumber;
            neededCl = Cl;
            iterationInput.pitchValue = pitch;
            iterationInput.flaps = flapSetting;
            iterationInput.spoilers = spoilers;

            iterationInput.alphaDot = 0;
            iterationInput.beta = 0;
            iterationInput.betaDot = 0;
            iterationInput.phi = 0;
            iterationInput.phiDot = 0;

            foreach (FARWingAerodynamicModel w in _wingAerodynamicModel)
            {
                if (w.GetShielded())
                    continue;

                if (w is FARControllableSurface controllableSurface)
                    controllableSurface.SetControlStateEditor(CoM,
                                                              Vector3.up,
                                                              (float)pitch,
                                                              0,
                                                              0,
                                                              flapSetting,
                                                              spoilers);
            }
        }

        public double FunctionIterateForAlpha(double alpha)
        {
            // iterationInput/iterationOutput are thread-local (properties); take a local for the
            // `out` argument, then publish it back to the thread-local slot.
            InstantConditionSimInput input = iterationInput;
            input.alpha = alpha;
            GetClCdCmSteady(input, out InstantConditionSimOutput output, true, true);
            iterationOutput = output;
            return output.Cl - neededCl;
        }

        /// <summary>
        /// Begins a parallel sweep: snapshots the main-thread-only geometry the aero solve reads
        /// (ship CoM, editor facility, every wing and body-section part transform) into plain data
        /// so the trim can run on worker threads, which read it while they hold a sweep slot
        /// (SimThreadContext.AcquireSlot). Must be called on the main thread, paired with
        /// <see cref="EndParallelSweep" /> in a finally.
        /// Returns false, having set up nothing, when a part carries an AeroForceModifier (a delegate
        /// another mod sets, called with the live part, which may read game state): run the sweep
        /// serially then.
        /// </summary>
        public bool BeginParallelSweep()
        {
            if (_currentAeroModules != null)
                foreach (FARAeroPartModule m in _currentAeroModules)
                    if (m != null && m.AeroForceModifier != null)
                        return false;

            _simEditorFacility = EditorDriver.editorFacility;
            _simCoM = ComputeEditorCoM();

            // The aero solve reads shared static state that is unsafe to touch concurrently: the
            // adiabatic index (which calls Planetarium/atmosphere) is snapshotted here on the main thread
            // for the workers (FARAeroUtil.SimAdiabaticIndexOverride), and every curve the solve evaluates
            // is built here (the solve evaluates their exact, thread-safe copies, see ExactCurve).
            FARAeroUtil.SimAdiabaticIndexOverride = null;
            double gamma = FARAeroUtil.CurrentAdiabaticIndex;
            var warmBody = FARAeroUtil.CurrentBody;

            FARAeroUtil.EnsureCurves();

            FARAeroUtil.SimAdiabaticIndexOverride = gamma;

            if (_wingAerodynamicModel != null)
                foreach (FARWingAerodynamicModel w in _wingAerodynamicModel)
                    if (w && w.part)
                        w.SetSimGeometry(true);

            if (_currentAeroModules != null)
                foreach (FARAeroPartModule m in _currentAeroModules)
                    if (m != null)
                        m.SetSimGeometry(true);

            // New slot epoch: the main thread takes slot 0 on its next scratch access (the warm-up cell)
            // and each worker takes the next slot on its first access.
            SimThreadContext.BeginSweep();

            _simActive = true;
            return true;
        }

        /// <summary>Releases the sweep snapshot, restoring the live-transform path. Main thread only.</summary>
        public void EndParallelSweep()
        {
            _simActive = false;
            FARAeroUtil.SimAdiabaticIndexOverride = null;

            // Clear every slot's viscous AeroCondition. The sweep's main-thread warm-up cell sets it
            // with no paired EndCell, so without this the accuracy skin-friction/Reynolds terms would
            // linger on the main thread's slot and leak into FAR's own static/stability tabs, which
            // must run on the null-condition (original) path. Workers have already joined here.
            for (int i = 0; i < _aeroConditionSlots.Length; i++)
                _aeroConditionSlots[i] = null;

            if (_wingAerodynamicModel != null)
                foreach (FARWingAerodynamicModel w in _wingAerodynamicModel)
                    if (w && w.part)
                        w.SetSimGeometry(false);

            if (_currentAeroModules != null)
                foreach (FARAeroPartModule m in _currentAeroModules)
                    if (m != null)
                        m.SetSimGeometry(false);
        }

        private static Vector3d ComputeEditorCoM()
        {
            Vector3d CoM = Vector3d.zero;
            double mass = 0;
            List<Part> partsList = EditorLogic.SortedShipList;
            if (partsList == null)
                return CoM;

            foreach (Part p in partsList)
            {
                if (FARAeroUtil.IsNonphysical(p))
                    continue;

                double partMass = p.mass;
                if (p.Resources.Count > 0)
                    partMass += p.GetResourceMass();

                CoM += partMass * (Vector3d)p.transform.TransformPoint(p.CoMOffset);
                mass += partMass;
            }

            if (mass > 0)
                CoM /= mass;
            return CoM;
        }
    }
}
