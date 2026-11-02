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
using System.Collections.Generic;
using System.Threading;
using FerramAerospaceResearch;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.Settings;
using KSP.Localization;
using KSPCommunityFixes;
using TweakScale;
using UnityEngine;

// ReSharper disable once CheckNamespace
namespace ferram4
{
    /// <summary>
    ///     This calculates the lift and drag on a wing in the atmosphere
    ///     It uses Prandtl lifting line theory to calculate the basic lift and drag coefficients and includes compressibility
    ///     corrections for subsonic and supersonic flows; transonic regime has placeholder
    /// </summary>
    public class FARWingAerodynamicModel : FARBaseAerodynamics, IRescalable<FARWingAerodynamicModel>, IPartMassModifier
    {
        protected const double criticalCl = 1.6;
        // rawAoAmax is read by FARWingInteraction from upstream wings (per-thread
        // safety: synced from _scratch at end of DoCalculateForces). AoAmax/liftslope
        // /rawLiftSlope/cosSweepAngle/piARe/minStall now live in WingScratch.
        public double rawAoAmax = 15;

        [KSPField(isPersistant = false, guiActive = false, guiActiveEditor = false)]
        public float wingBaseMassMultiplier = 1f;

        [KSPField(isPersistant = false, guiActive = false, guiActiveEditor = true)]
        public float curWingMass = 1;

        private float desiredMass;
        private float baseMass;

        [KSPField(guiName = "FARWingMassStrength", isPersistant = true, guiActiveEditor = true, guiActive = false),
         UI_FloatRange(maxValue = 4.0f, minValue = 0.05f, scene = UI_Scene.Editor, stepIncrement = 0.05f)]
        public float massMultiplier = 1.0f;

        public float oldMassMultiplier = -1f;

        [KSPField(isPersistant = false)] public double MAC;

        public double MAC_actual;

        [KSPField(isPersistant = false)] public double e;

        [KSPField(isPersistant = false)] public int nonSideAttach; //This is for ailerons and the small ctrl surf

        [KSPField(isPersistant = false)] public double TaperRatio;

        [KSPField(isPersistant = false, guiActive = true, guiName = "FARWingStalled")]
        protected double stall;

        // Per-call mutable state (minStall, piARe, cosSweepAngle, effective_b_2,
        // effective_MAC, effective_AR, transformed_AR) lives in _scratch - see
        // WingScratch.cs for the full set.

        [KSPField(isPersistant = false)] public double b_2; //span

        public double b_2_actual; //span

        [KSPField(isPersistant = false)] public double MidChordSweep;

        private double MidChordSweepSideways;

        private ArrowPointer liftArrow;
        private ArrowPointer dragArrow;

        private bool fieldsVisible;

        // ReSharper disable once NotAccessedField.Global -> unity
        [KSPField(isPersistant = false,
                  guiActive = false,
                  guiActiveEditor = false,
                  guiFormat = "F3",
                  guiUnits = "FARUnitKN")]
        public float dragForceWing;

        // ReSharper disable once NotAccessedField.Global -> unity
        [KSPField(isPersistant = false,
                  guiActive = false,
                  guiActiveEditor = false,
                  guiFormat = "F3",
                  guiUnits = "FARUnitKN")]
        public float liftForceWing;

        // rawLiftSlope/liftslope live on _scratch (per-call). zeroLiftCdIncrement is
        // setup-time geometry and stays here.
        protected double zeroLiftCdIncrement;

        private double refAreaChildren;

        public Vector3d AerodynamicCenter = Vector3d.zero;
        private Vector3d CurWingCentroid = Vector3d.zero;
        private Vector3d ParallelInPlane = Vector3d.zero;
        private Vector3d perp = Vector3d.zero;
        private Vector3d liftDirection = Vector3d.zero;

        [KSPField(isPersistant = false)] public Vector3 rootMidChordOffsetFromOrig;

        // in local coordinates
        private Vector3d localWingCentroid = Vector3d.zero;
        private Vector3d sweepPerpLocal, sweepPerp2Local;

        private Vector3d ParallelInPlaneLocal = Vector3d.zero;

        private FARWingInteraction wingInteraction;
        internal FARWingInteraction WingInteraction => wingInteraction;
        // the aspect ratio the lift slope uses, and the panel's own (semispan over chord), as last computed
        internal double EffectiveAR => _scratch.effective_AR;
        internal double OwnAR => _scratch.transformed_AR;
        private FARAeroPartModule aeroModule;

        public short srfAttachNegative = 1;

        private FARWingAerodynamicModel parentWing;
        private bool updateMassNextFrame;
        [KSPField(isPersistant = true)] public float massOverride = float.MinValue;

        public float? MassOverride
        {
            get { return (massOverride > float.MinValue) ? massOverride : null; }
            set
            {
                Fields[nameof(curWingMass)].guiActiveEditor = value is not null;
                Fields[nameof(massMultiplier)].guiActiveEditor = value is not null;
                massOverride = value ?? float.MinValue;
            }
        }

        protected double ClIncrementFromRear;

        public double YmaxForce = double.MaxValue;
        public double XZmaxForce = double.MaxValue;

        public Vector3 worldSpaceForce;

        protected double NUFAR_areaExposedFactor;
        protected double NUFAR_totalExposedAreaFactor;

        // Per-call mutable state, thread-local so the parallel heatmap sweep can evaluate
        // independent (Mach, altitude) cells against the same wing module concurrently.
        // The single-threaded flight/editor paths transparently use the calling thread's
        // instance (the main thread), so behaviour there is unchanged.
        // One scratch per thread slot (see SimThreadContext). Indexing an array by a [ThreadStatic]
        // slot is far cheaper than ThreadLocal.Value under Mono, which dominated the parallel sweep.
        // Each slot is used by a single thread at a time, so the lazy create needs no synchronization.
        // A slot's scratch, once created, is never replaced or written by anything but the thread using the
        // slot, so every access during a solve returns the same object and nothing else changes it mid-solve.
        private readonly WingScratch[] _scratchSlots = new WingScratch[SimThreadContext.MaxSlots];

        internal WingScratch _scratch
        {
            get
            {
                int s = SimThreadContext.Slot;
                WingScratch sc = _scratchSlots[s];
                if (sc == null)
                {
                    sc = new WingScratch {rawAoAmax = rawAoAmax, transformed_AR = b_2_actual / MAC_actual};
                    _scratchSlots[s] = sc;
                }

                return sc;
            }
        }

        // Geometry changed: refresh the geometry-derived value in slot 0's scratch (flight and the serial
        // editor tabs), as the instance field was refreshed before. The rest (effective AR/MAC/span, stall)
        // persists across a geometry update as the instance fields did; dropping the scratch reset it and
        // made the first evaluation after a rebuild differ from the original code. Other slots are left alone:
        // another thread may be solving on one, and a solve sets transformed_AR (CalculateSubsonicLiftSlope)
        // before anything reads it, so the seed only matters to slot 0's first evaluation. Slots not yet
        // created seed from the geometry.
        private void ReseedScratch(double transformedAR)
        {
            WingScratch sc = _scratchSlots[0];
            if (sc != null)
                sc.transformed_AR = transformedAR;
        }

        // Snapshot of this part's transform, non-null only while a sweep is running. A sweep
        // worker's solve reads geometry from it (SweepGeom) instead of the Unity Transform,
        // which is main-thread-only; flight, the single-threaded tabs and the main thread's
        // own solves during a sweep read the live transform exactly as before. Also captures
        // the two other main-thread reads on the solve path (rigidbody mass and the physics
        // timestep).
        public FrozenPartTransform? SimGeom;
        private double simRbMass;
        private double simFixedDeltaTime;

        /// <summary>The sweep's frozen geometry on a sweep worker (SimThreadContext.InSweep), else none.</summary>
        internal FrozenPartTransform? SweepGeom
        {
            get { return SimThreadContext.InSweep ? SimGeom : null; }
        }

        private bool massScaleReady;
        public double FinalLiftSlope { get; private set; }

        public float GetModuleMass(float defaultMass, ModifierStagingSituation sit)
        {
            if (MassOverride is not null)
                return (float)MassOverride;
            if (massScaleReady)
                return desiredMass - baseMass;
            return 0;
        }

        public ModifierChangeWhen GetModuleMassChangeWhen()
        {
            return ModifierChangeWhen.FIXED;
        }

        public void OnRescale(ScalingFactor factor)
        {
            b_2_actual = factor.absolute.linear * b_2;
            MAC_actual = factor.absolute.linear * MAC;
            if (part.Modules.Contains("TweakScale"))
            {
                PartModule m = part.Modules["TweakScale"];
                float massScale = (float)m.Fields.GetValue("MassScale");
                baseMass = part.partInfo.partPrefab.mass + part.partInfo.partPrefab.mass * (massScale - 1);
                FARLogger.Info("TweakScale massScale for FAR usage: " + massScale);
            }

            massScaleReady = false;

            StartInitialization();
        }

        public void NUFAR_ClearExposedAreaFactor()
        {
            NUFAR_areaExposedFactor = 0;
            NUFAR_totalExposedAreaFactor = 0;
        }

        public void NUFAR_CalculateExposedAreaFactor()
        {
            FARAeroPartModule a = part.FindModuleImplementingFast<FARAeroPartModule>();

            NUFAR_areaExposedFactor = Math.Min(a.ProjectedAreas.kN, a.ProjectedAreas.kP);
            NUFAR_totalExposedAreaFactor = Math.Max(a.ProjectedAreas.kN, a.ProjectedAreas.kP);
        }

        public void NUFAR_SetExposedAreaFactor()
        {
            List<Part> counterparts = part.symmetryCounterparts;
            double counterpartsCount = 1;
            double sum = NUFAR_areaExposedFactor;
            double totalExposedSum = NUFAR_totalExposedAreaFactor;

            foreach (Part p in counterparts)
            {
                if (p == null)
                    continue;
                FARWingAerodynamicModel model = this is FARControllableSurface
                                                    ? p.FindModuleImplementingFast<FARControllableSurface>()
                                                    : p.FindModuleImplementingFast<FARWingAerodynamicModel>();

                ++counterpartsCount;
                sum += model.NUFAR_areaExposedFactor;
                totalExposedSum += model.NUFAR_totalExposedAreaFactor;
            }

            double tmp = 1 / counterpartsCount;
            sum *= tmp;
            totalExposedSum *= tmp;

            NUFAR_areaExposedFactor = sum;
            NUFAR_totalExposedAreaFactor = totalExposedSum;

            foreach (Part p in counterparts)
            {
                if (p == null)
                    continue;
                FARWingAerodynamicModel model = this is FARControllableSurface
                                                    ? p.FindModuleImplementingFast<FARControllableSurface>()
                                                    : p.FindModuleImplementingFast<FARWingAerodynamicModel>();

                model.NUFAR_areaExposedFactor = sum;
                model.NUFAR_totalExposedAreaFactor = totalExposedSum;
            }
        }

        public void NUFAR_UpdateShieldingStateFromAreaFactor()
        {
            isShielded = NUFAR_areaExposedFactor < 0.1 * S;
            FARWingInteraction.InvalidateWingGroups(part);
        }

        /// <summary>
        ///     Whether the voxels find this panel covered by other parts: shielded, for a control surface at rest (a
        ///     deflected one comes out of the cover, see FARControllableSurface.CheckShielded). Changes only with the voxels
        /// </summary>
        internal bool ShieldedAtRest => NUFAR_areaExposedFactor < 0.1 * S;

        public double GetStall()
        {
            return _scratch.stall;
        }

        /// <summary>
        /// Starts this wing's scratch on the calling thread's slot (a prediction's) from the flight's, slot 0's, as
        /// the prediction found it when it ran on the flight's state; nothing a previous prediction left is carried
        /// into this one. A snapshot prediction installs the flight's state as it was captured instead
        /// (<see cref="InstallSnapshotState" />): the flight's slot is not read then.
        /// </summary>
        internal virtual void BeginPredictionState()
        {
            int s = SimThreadContext.Slot;
            WingScratch sc = _scratchSlots[s];
            if (sc == null)
                _scratchSlots[s] = sc = new WingScratch();
            if (SimThreadContext.SnapshotStamp != 0)
                return;
            sc.CopyFrom(_scratchSlots[0] ?? NewScratch());
            wingInteraction?.BeginPredictionState();
        }

        /// <summary>A scratch as a slot's first is made, for a wing the flight has not evaluated yet.</summary>
        private WingScratch NewScratch()
        {
            return new WingScratch {rawAoAmax = rawAoAmax, transformed_AR = b_2_actual / MAC_actual};
        }

        /// <summary>
        /// The live state the solve reads from the part and the game, captured on the main thread with a vessel
        /// snapshot so a snapshot prediction can run on any thread.
        /// </summary>
        internal struct SnapshotState
        {
            public FrozenPartTransform Geom;
            public double RbMass;
            public double FixedDeltaTime;
            public bool Shielded;
            public double SubmergedPortion;
            public double SubmergedLiftScalar;
            public double SubmergedDragScalar;
            public Vector3d Velocity;
            public WingScratch Seed;
            public WingInteractionScratch InteractionSeed;
            public ControlSurfaceScratch Control;
        }

        /// <summary>
        /// Captures what the solve reads live from this wing's part, and the flight's state a prediction starts from.
        /// Main thread.
        /// </summary>
        internal virtual SnapshotState CaptureSnapshotState()
        {
            // the wing group a prediction off the main thread takes its aspect ratio from
            wingInteraction?.ResolveWingGroup();
            var seed = new WingScratch();
            seed.CopyFrom(_scratchSlots[0] ?? NewScratch());
            return new SnapshotState
            {
                Geom = new FrozenPartTransform(part_transform),
                RbMass = part.rb != null ? part.rb.mass : part.mass,
                FixedDeltaTime = TimeWarp.fixedDeltaTime,
                Shielded = isShielded,
                SubmergedPortion = part.submergedPortion,
                SubmergedLiftScalar = part.submergedLiftScalar,
                SubmergedDragScalar = part.submergedDragScalar,
                // A packed vessel's parts have no rigidbody (GetVelocity reads it); they move with the vessel.
                Velocity = part.Rigidbody != null ? GetVelocity() : vessel.srf_velocity,
                Seed = seed,
                InteractionSeed = wingInteraction?.CaptureSeed()
            };
        }

        /// <summary>
        /// Puts captured state on the calling thread's slot for the snapshot prediction with this stamp, after
        /// <see cref="BeginPredictionState" />.
        /// </summary>
        internal virtual void InstallSnapshotState(in SnapshotState state, long stamp)
        {
            WingScratch sc = _scratch;
            sc.CopyFrom(state.Seed);
            wingInteraction?.InstallSeed(state.InteractionSeed);
            sc.CapturedGeom = state.Geom;
            sc.CapturedRbMass = state.RbMass;
            sc.CapturedFixedDeltaTime = state.FixedDeltaTime;
            sc.CapturedShielded = state.Shielded;
            sc.CapturedSubmergedPortion = state.SubmergedPortion;
            sc.CapturedSubmergedLiftScalar = state.SubmergedLiftScalar;
            sc.CapturedSubmergedDragScalar = state.SubmergedDragScalar;
            sc.CapturedVelocity = state.Velocity;
            sc.CaptureStamp = stamp;
        }

        /// <summary>This slot's scratch, and whether it holds the state captured for the current snapshot prediction.</summary>
        private bool SnapshotInstalled(out WingScratch sc)
        {
            long stamp = SimThreadContext.SnapshotStamp;
            if (stamp == 0)
            {
                sc = null;
                return false;
            }

            sc = _scratch;
            return sc.CaptureStamp == stamp;
        }

        /// <summary>
        /// The geometry the solve reads: the snapshot's inside a snapshot prediction, the sweep's frozen copy during a
        /// parallel sweep, else none (the live transform).
        /// </summary>
        internal FrozenPartTransform? SolveGeom
        {
            get
            {
                if (SimThreadContext.LiveOnly)
                    return null;
                return SnapshotInstalled(out WingScratch sc) ? sc.CapturedGeom : SweepGeom;
            }
        }

        /// <summary>The wing's forward (normal) direction as the solve sees it.</summary>
        internal Vector3 SolveForward
        {
            get
            {
                if (SimThreadContext.LiveOnly)
                    return transform.forward;
                FrozenPartTransform? g = SolveGeom;
                return g.HasValue ? g.Value.Forward : transform.forward;
            }
        }

        /// <summary>
        /// Inside a snapshot prediction, whether this wing was captured with the snapshot: a wing that was not (it
        /// was destroyed, or added since) is left out, as the flight leaves out a destroyed neighbour.
        /// </summary>
        internal bool CapturedForSolve
        {
            get { return SnapshotInstalled(out WingScratch _); }
        }

        /// <summary>
        /// Shielded state as the solve should see it. Plain wings use the (sweep-invariant) instance
        /// field; control surfaces override this to read the per-thread value, since they rewrite
        /// shielded state from their deflection every trim step and the instance field would race.
        /// </summary>
        public virtual bool GetShielded()
        {
            return SnapshotInstalled(out WingScratch sc) ? sc.CapturedShielded : isShielded;
        }

        public double GetRawAoAmax()
        {
            return _scratch.rawAoAmax;
        }

        public double GetCdInduced()
        {
            return _scratch.CdInduced;
        }

        public double GetCdProfile()
        {
            return _scratch.CdProfile;
        }

        // Editor velocity comes from the thread-local scratch so that a neighbour wing read
        // during the parallel sweep (FARWingInteraction) sees the value from the calling
        // thread's cell, not whichever thread last touched the shared instance field.
        public override Vector3d GetVelocity()
        {
            if (!HighLogic.LoadedSceneIsFlight)
                return _scratch.velocityEditor;
            // A snapshot prediction reads the part's velocity as captured (a neighbour reads it for its angle of
            // attack): the live one is the rigidbody's, which only the main thread may read.
            if (SnapshotInstalled(out WingScratch sc))
                return sc.CapturedVelocity;
            return base.GetVelocity();
        }

        // The CoL marker path sets the velocity on every module before evaluating any of them, so the
        // upstream-wing reads in FARWingInteraction see it; that has to land in the scratch GetVelocity reads.
        public override void SetEditorVelocity(Vector3d vel)
        {
            if (SimThreadContext.Slot == 0)
                base.SetEditorVelocity(vel);
            _scratch.velocityEditor = vel;
        }

        /// <summary>
        /// Snapshots the Unity reads on the editor solve path (the part transform, rigidbody mass and
        /// physics timestep) into plain data so the sweep can run on worker threads. Must be called on
        /// the main thread. Pass false to release the snapshot when the sweep ends, restoring the live
        /// transform path used by flight and the single-threaded tabs.
        /// </summary>
        public virtual void SetSimGeometry(bool capture)
        {
            if (!capture)
            {
                SimGeom = null;
                return;
            }

            SimGeom = new FrozenPartTransform(part_transform);
            simRbMass = part.rb != null ? part.rb.mass : part.mass;
            simFixedDeltaTime = TimeWarp.fixedDeltaTime;
            // the wing group the sweep's workers take their aspect ratio from
            wingInteraction?.ResolveWingGroup();
        }

        // ReSharper disable once UnusedMember.Global
        public double GetCl()
        {
            double ClUpwards = 1;
            if (HighLogic.LoadedSceneIsFlight)
                ClUpwards = Vector3.Dot(liftDirection, -vessel.vesselTransform.forward);
            ClUpwards *= Cl;

            return ClUpwards;
        }

        // ReSharper disable once UnusedMember.Global
        public double GetCd()
        {
            return Cd;
        }

        public Vector3d GetAerodynamicCenter()
        {
            return _scratch.AerodynamicCenter;
        }

        public double GetMAC()
        {
            return _scratch.effective_MAC;
        }

        public double Getb_2()
        {
            return _scratch.effective_b_2;
        }

        public Vector3d GetLiftDirection()
        {
            return liftDirection;
        }

        public double GetRawLiftSlope()
        {
            return _scratch.rawLiftSlope;
        }

        public double GetCosSweepAngle()
        {
            return _scratch.cosSweepAngle;
        }

        public double GetCd0()
        {
            return _scratch.zeroLiftCdIncrement;
        }

        /// <summary>
        /// Editor-side force evaluation.
        /// </summary>
        /// <param name="skinFrictionOverride">
        /// Skin friction coefficient to use instead of the editor's default constant. Outside
        /// flight, DoCalculateForces cannot derive skin friction itself: the editor sim passes a
        /// unit velocity vector (so that q works out to exactly 1 and forces come out as
        /// coefficients), which makes the Reynolds number meaningless. It therefore falls back to a
        /// fixed 0.005, which is roughly double the true value at sea-level Reynolds numbers and
        /// below it at low ones. Callers that know the real flight condition being simulated can
        /// supply the correct value here; passing null preserves the historical behaviour.
        /// </param>
        public Vector3d ComputeForceEditor(
            Vector3d velocityVector,
            double M,
            double density,
            double? skinFrictionOverride = null
        )
        {
            _scratch.velocityEditor = velocityVector;
            if (SimThreadContext.Slot == 0)
            {
                velocityEditor = velocityVector;
                rho = density;
            }

            double AoA = CalculateAoA(velocityVector);
            return CalculateForces(velocityVector, M, AoA, density, true, skinFrictionOverride);
        }

        public void ComputeClCdEditor(Vector3d velocityVector, double M, double density)
        {
            _scratch.velocityEditor = velocityVector;
            if (SimThreadContext.Slot == 0)
            {
                velocityEditor = velocityVector;
                rho = density;
            }

            double AoA = CalculateAoA(velocityVector);
            CalculateForces(velocityVector, M, AoA, density);
        }

        protected override void ResetCenterOfLift()
        {
            _scratch.stall = 0;
            if (SimThreadContext.Slot != 0)
                return;
            rho = 1;
            stall = 0;
        }

        public override Vector3d PrecomputeCenterOfLift(
            Vector3d velocity,
            double MachNumber,
            double density,
            FARCenterQuery center
        )
        {
            try
            {
                double AoA = CalculateAoA(velocity);

                Vector3d force = CalculateForces(velocity, MachNumber, AoA, density, double.PositiveInfinity, false);
                center.AddForce(AerodynamicCenter, force);

                return force;
            }
            catch
            {
                //FIX ME!!!
                //Yell at KSP devs so that I don't have to engage in bad code practice
                return Vector3.zero;
            }
        }


        public void EditorClClear(bool reset_stall)
        {
            _scratch.Cl = 0;
            _scratch.Cd = 0;
            if (reset_stall)
                _scratch.stall = 0;
            if (SimThreadContext.Slot != 0)
                return;
            Cl = Cd = 0;
            if (reset_stall)
                stall = 0;
        }

        private void PrecomputeCentroid()
        {
            Vector3d WC = rootMidChordOffsetFromOrig;
            if (nonSideAttach <= 0)
                WC += -b_2_actual /
                      3 *
                      (1 + TaperRatio * 2) /
                      (1 + TaperRatio) *
                      (Vector3d.right * srfAttachNegative +
                       Vector3d.up * Math.Tan(MidChordSweep * FARMathUtil.deg2rad));
            else
                WC += -MAC_actual * 0.7 * Vector3d.up;

            localWingCentroid = WC;
        }

        public Vector3 WingCentroid()
        {
            if (SimThreadContext.LiveOnly)
                return part_transform.TransformDirection(localWingCentroid) + part.partTransform.position;
            FrozenPartTransform? g = SolveGeom;
            if (g.HasValue)
                return g.Value.TransformDirection(localWingCentroid) + g.Value.Position;
            return part_transform.TransformDirection(localWingCentroid) + part.partTransform.position;
        }

        /// <summary>
        ///     The corners of this panel's planform as FAR models it (world), in order around it: root leading edge, root
        ///     trailing edge, tip trailing edge, tip leading edge
        /// </summary>
        internal void PlanformCorners(Vector3[] corners)
        {
            Vector3d rootMid = rootMidChordOffsetFromOrig;
            Vector3d tipMid;
            double rootChord, tipChord;
            if (nonSideAttach <= 0)
            {
                // MAC_actual is the mean chord (S = b_2 MAC): rootChord (1 + taper) / 2
                double taper = TaperRatio;
                rootChord = 2 * MAC_actual / (1 + taper);
                tipChord = rootChord * taper;
                tipMid = rootMid -
                         b_2_actual *
                         (Vector3d.right * srfAttachNegative + Vector3d.up * Math.Tan(MidChordSweep * FARMathUtil.deg2rad));
            }
            else
            {
                // a control surface hung from its leading edge, centered on its centroid
                Vector3d centroid = rootMid - MAC_actual * 0.7 * Vector3d.up;
                rootMid = centroid + 0.5 * b_2_actual * Vector3d.right;
                tipMid = centroid - 0.5 * b_2_actual * Vector3d.right;
                rootChord = tipChord = MAC_actual;
            }

            Vector3 origin = part.partTransform.position;
            corners[0] = origin + part_transform.TransformDirection(rootMid + 0.5 * rootChord * Vector3d.up);
            corners[1] = origin + part_transform.TransformDirection(rootMid - 0.5 * rootChord * Vector3d.up);
            corners[2] = origin + part_transform.TransformDirection(tipMid - 0.5 * tipChord * Vector3d.up);
            corners[3] = origin + part_transform.TransformDirection(tipMid + 0.5 * tipChord * Vector3d.up);
        }

        private Vector3d CalculateAerodynamicCenter(double MachNumber, double AoA, Vector3d WC)
        {
            Vector3d AC_offset = Vector3d.zero;
            if (nonSideAttach <= 0)
            {
                double tmp = Math.Cos(AoA);
                tmp *= tmp;
                if (MachNumber < 0.85)
                {
                    AC_offset = _scratch.effective_MAC * 0.25 * _scratch.ParallelInPlane;
                }
                else if (MachNumber > 1.4)
                {
                    AC_offset = _scratch.effective_MAC * 0.10 * _scratch.ParallelInPlane;
                }
                else if (MachNumber >= 1)
                {
                    AC_offset = _scratch.effective_MAC * (-0.375 * MachNumber + 0.625) * _scratch.ParallelInPlane;
                }
                //This is for the transonic instability, which is lessened for highly swept wings
                else
                {
                    double sweepFactor = _scratch.cosSweepAngle * _scratch.cosSweepAngle * tmp;
                    if (MachNumber < 0.9)
                        AC_offset = _scratch.effective_MAC * ((MachNumber - 0.85) * 2 * sweepFactor + 0.25) * _scratch.ParallelInPlane;
                    else
                        AC_offset = _scratch.effective_MAC * ((1 - MachNumber) * sweepFactor + 0.25) * _scratch.ParallelInPlane;
                }

                AC_offset *= tmp;
            }

            WC += AC_offset;

            return WC; //WC updated to AC
        }

        public override void Initialization()
        {
            base.Initialization();
            if (b_2_actual.NearlyEqual(0))
            {
                b_2_actual = b_2;
                MAC_actual = MAC;
            }

            if (baseMass <= 0)
                //part.prefabMass is apparently not set until the end of part.Start(), which runs after Start() for PartModules
                baseMass = part.partInfo.partPrefab.mass;

            StartInitialization();
            if (HighLogic.LoadedSceneIsEditor)
            {
                part.OnEditorAttach += OnWingAttach;
                part.OnEditorDetach += OnWingDetach;
            }


            OnVesselPartsChange += UpdateThisWingInteractions;
            if (MassOverride is not null)
            {
                Fields[nameof(curWingMass)].guiActiveEditor = false;
                Fields[nameof(massMultiplier)].guiActiveEditor = false;
            }
        }

        private long geometryVersion;

        /// <summary>
        /// Changes whenever something a prediction reads from this wing is rebuilt on the main thread (its geometry,
        /// its neighbour lists and exposures): bumped before and after each rebuild, so a snapshot prediction that
        /// overlaps one sees it changed. Any thread.
        /// </summary>
        internal long GeometryVersion => Interlocked.Read(ref geometryVersion);

        /// <summary>Marks a rebuild of wing state predictions read (see <see cref="GeometryVersion"/>). Main thread.</summary>
        internal void BumpGeometryVersion()
        {
            Interlocked.Increment(ref geometryVersion);
        }

        public void StartInitialization()
        {
            // One bump pair around the whole rebuild: between the geometry's and the neighbours' own pairs, a new
            // interaction with empty neighbour lists replaces the old one, and a snapshot prediction running in that
            // gap alone would see no version change.
            BumpGeometryVersion();
            try
            {
                MathAndFunctionInitialization();
                aeroModule = part.GetComponent<FARAeroPartModule>();

                if (aeroModule == null)
                    FARLogger.Error("Could not find FARAeroPartModule on same part as FARWingAerodynamicModel!");

                OnWingAttach();
                massScaleReady = true;
                wingInteraction = new FARWingInteraction(this, part, rootMidChordOffsetFromOrig, srfAttachNegative);
                UpdateThisWingInteractions();
            }
            finally
            {
                BumpGeometryVersion();
            }
        }

        public void MathAndFunctionInitialization()
        {
            BumpGeometryVersion();
            try
            {
                MathAndFunctionInitializationCore();
            }
            finally
            {
                BumpGeometryVersion();
            }
        }

        private void MathAndFunctionInitializationCore()
        {
            S = b_2_actual * MAC_actual;

            if (part.srfAttachNode.originalOrientation.x < 0)
                srfAttachNegative = -1;

            double transformedAR = b_2_actual / MAC_actual;
            ReseedScratch(transformedAR);

            MidChordSweepSideways = (1 - TaperRatio) / (1 + TaperRatio);

            MidChordSweepSideways =
                (Math.PI * 0.5 -
                 Math.Atan(Math.Tan(MidChordSweep * FARMathUtil.deg2rad) +
                           MidChordSweepSideways * 4 / transformedAR)) *
                MidChordSweepSideways *
                0.5;

            double sweepHalfChord = MidChordSweep * FARMathUtil.deg2rad;

            //Vector perpendicular to midChord line
            sweepPerpLocal = Vector3d.up * Math.Cos(sweepHalfChord) +
                             Vector3d.right * Math.Sin(sweepHalfChord) * srfAttachNegative;
            //Vector perpendicular to midChord line2
            sweepPerp2Local = Vector3d.up * Math.Sin(MidChordSweepSideways) -
                              Vector3d.right * Math.Cos(MidChordSweepSideways) * srfAttachNegative;

            PrecomputeCentroid();

            if (!FARDebugValues.allowStructuralFailures)
                return;
            foreach (FARPartStressTemplate temp in FARAeroStress.StressTemplates)
                if (temp.Name == "wingStress")
                {
                    FARPartStressTemplate template = temp;

                    YmaxForce = template.YMaxStress; //in MPa
                    YmaxForce *= S;

                    XZmaxForce = template.XZMaxStress;
                    XZmaxForce *= S;
                    break;
                }

            double maxForceMult = Math.Pow(massMultiplier, FARAeroUtil.massStressPower);
            YmaxForce *= maxForceMult;
            XZmaxForce *= maxForceMult;
        }

        public void EditorUpdateWingInteractions()
        {
            UpdateThisWingInteractions();
        }

        public void UpdateThisWingInteractions()
        {
            BumpGeometryVersion();
            try
            {
                if (VesselPartList == null)
                    VesselPartList = GetShipPartList();
                if (wingInteraction == null)
                    wingInteraction = new FARWingInteraction(this, part, rootMidChordOffsetFromOrig, srfAttachNegative);

                wingInteraction.UpdateWingInteraction(VesselPartList, nonSideAttach == 1);
            }
            finally
            {
                BumpGeometryVersion();
            }
        }

        public virtual void FixedUpdate()
        {
            // With unity objects, "foo" or "foo != null" calls a method to check if
            // it's destroyed. !(foo is null) just checks if it is actually null.
            if (HighLogic.LoadedSceneIsFlight && FlightGlobals.ready && !isShielded)
            {
                Rigidbody rb = part.Rigidbody;
                Vessel partVessel = part.vessel;

                if (!rb || !partVessel || partVessel.packed)
                    return;

                // Check that rb is not destroyed, but vessel is just not null
                if (partVessel.atmDensity > 0)
                {
                    _scratch.CurWingCentroid = CurWingCentroid = WingCentroid();

                    Vector3d velocity = rb.GetPointVelocity(CurWingCentroid) +
                                        Krakensbane.GetFrameVelocity() -
                                        FARAtmosphere.GetWind(FlightGlobals.currentMainBody, part, rb.position);

                    double v_scalar = velocity.magnitude;

                    // water's contribution to rho
                    double waterRho = 0;
                    if (partVessel.mainBody.ocean)
                    {
                        waterRho = partVessel.mainBody.oceanDensity * 1000 * part.submergedPortion;
                        rho = waterRho + part.atmDensity * (1 - part.submergedPortion);
                    }
                    else
                    {
                        rho = part.atmDensity;
                    }

                    double machNumber = partVessel.mach;
                    if (rho > 0 && v_scalar > 0.1)
                    {
                        double AoA = CalculateAoA(velocity);
                        double failureForceScaling = FARAeroUtil.GetFailureForceScaling(partVessel);
                        Vector3d force = DoCalculateForces(velocity, machNumber, AoA, rho, failureForceScaling);

                        worldSpaceForce = force;

                        if (part.submergedPortion > 0)
                        {
                            Vector3 velNorm = velocity / v_scalar;
                            Vector3 worldSpaceDragForce = Vector3.Dot(velNorm, force) * velNorm;
                            Vector3 worldSpaceLiftForce = worldSpaceForce - worldSpaceDragForce;

                            Vector3 waterDragForce, waterLiftForce;
                            if (part.submergedPortion < 1)
                            {
                                float waterFraction = (float)(waterRho / rho);

                                waterDragForce = worldSpaceDragForce * waterFraction; //calculate areaDrag vector
                                waterLiftForce = worldSpaceLiftForce * waterFraction;

                                worldSpaceDragForce -= waterDragForce;
                                worldSpaceLiftForce -= waterLiftForce;

                                waterDragForce *= Math.Min((float)part.submergedDragScalar, 1);
                                waterLiftForce *= (float)part.submergedLiftScalar;
                            }
                            else
                            {
                                waterDragForce = worldSpaceDragForce * Math.Min((float)part.submergedDragScalar, 1);
                                waterLiftForce = worldSpaceLiftForce * (float)part.submergedLiftScalar;

                                worldSpaceDragForce = worldSpaceLiftForce = Vector3.zero;
                            }

                            //extra water drag factor for wings
                            aeroModule.hackWaterDragVal +=
                                Math.Abs(waterDragForce.magnitude / (rb.mass * rb.velocity.magnitude)) * 5;

                            waterLiftForce *= (float)PhysicsGlobals.BuoyancyWaterLiftScalarEnd;
                            if (part.partBuoyancy.splashedCounter < PhysicsGlobals.BuoyancyWaterDragTimer)
                                waterLiftForce *=
                                    (float)(part.partBuoyancy.splashedCounter / PhysicsGlobals.BuoyancyWaterDragTimer);

                            double waterLiftScalar;
                            //reduce lift drastically when wing is in water
                            if (part.submergedPortion < 0.05)
                            {
                                waterLiftScalar = 396.0 * part.submergedPortion;
                                waterLiftScalar -= 39.6;
                                waterLiftScalar *= part.submergedPortion;
                                waterLiftScalar++;
                            }
                            else if (part.submergedPortion > 0.95)
                            {
                                waterLiftScalar = 396.0 * part.submergedPortion;
                                waterLiftScalar -= 752.4;
                                waterLiftScalar *= part.submergedPortion;
                                waterLiftScalar += 357.4;
                            }
                            else
                            {
                                waterLiftScalar = 0.01;
                            }

                            waterLiftForce *= (float)waterLiftScalar;
                            worldSpaceLiftForce *= (float)waterLiftScalar;

                            force = worldSpaceDragForce + worldSpaceLiftForce + waterLiftForce;
                            worldSpaceForce = force + waterDragForce;
                        }

                        Vector3d scaledForce = worldSpaceForce;
                        //This accounts for the effect of flap effects only being handled by the rearward surface
                        scaledForce *= S / (S + wingInteraction.EffectiveUpstreamArea);

                        Vector3 forward = part_transform.forward;
                        double forwardScaledForceMag = Vector3d.Dot(scaledForce, forward);
                        Vector3d forwardScaledForce = forwardScaledForceMag * (Vector3d)forward;

                        if (Math.Abs(forwardScaledForceMag) >
                            YmaxForce * failureForceScaling * (1 + part.submergedPortion * 1000) ||
                            (scaledForce - forwardScaledForce).magnitude >
                            XZmaxForce * failureForceScaling * (1 + part.submergedPortion * 1000))
                            if (part.parent && !partVessel.packed)
                            {
                                partVessel.SendMessage("AerodynamicFailureStatus");
                                string msg = Localizer.Format("FARFlightLogAeroFailure",
                                                              KSPUtil.PrintTimeStamp(FlightLogger.met),
                                                              part.partInfo.title);
                                FlightLogger.eventLog.Add(msg);
                                part.decouple(25);
                                if (FARDebugValues.aeroFailureExplosions)
                                    FXMonger.Explode(part, AerodynamicCenter, 1);
                            }

                        part.AddForceAtPosition(force, AerodynamicCenter);
                    }
                    else
                    {
                        stall = _scratch.stall = 0;
                        wingInteraction.ResetWingInteractions();
                    }
                }
                else
                {
                    stall = _scratch.stall = 0;
                    wingInteraction.ResetWingInteractions();
                }
            }
            else
            {
                if (isShielded)
                    Cl = Cd = Cm = stall = _scratch.stall = 0;
                if (!(liftArrow is null))
                {
                    Destroy(liftArrow);
                    liftArrow = null;
                }

                // ReSharper disable once InvertIf
                if (!(dragArrow is null))
                {
                    Destroy(dragArrow);
                    dragArrow = null;
                }
            }
        }

        //This version also updates the wing centroid
        public Vector3d CalculateForces(
            Vector3d velocity,
            double MachNumber,
            double AoA,
            double density,
            bool updateAeroArrows = true,
            double? skinFrictionOverride = null
        )
        {
            _scratch.CurWingCentroid = WingCentroid();
            if (SimThreadContext.Slot == 0)
                CurWingCentroid = _scratch.CurWingCentroid;

            return DoCalculateForces(velocity,
                                     MachNumber,
                                     AoA,
                                     density,
                                     1,
                                     updateAeroArrows,
                                     skinFrictionOverride);
        }

        public Vector3d CalculateForces(
            Vector3d velocity,
            double MachNumber,
            double AoA,
            double density,
            double failureForceScaling,
            bool updateAeroArrows = true,
            double? skinFrictionOverride = null
        )
        {
            _scratch.CurWingCentroid = WingCentroid();
            if (SimThreadContext.Slot == 0)
                CurWingCentroid = _scratch.CurWingCentroid;

            return DoCalculateForces(velocity,
                                     MachNumber,
                                     AoA,
                                     density,
                                     failureForceScaling,
                                     updateAeroArrows,
                                     skinFrictionOverride);
        }

        private Vector3d DoCalculateForces(
            Vector3d velocity,
            double MachNumber,
            double AoA,
            double density,
            double failureForceScaling,
            bool updateAeroArrows = true,
            double? skinFrictionOverride = null
        )
        {
            double v_scalar = velocity.magnitude;

            bool snapshot = SnapshotInstalled(out WingScratch captured);
            FrozenPartTransform? geom = snapshot ? captured.CapturedGeom : SweepGeom;
            Vector3 forward = geom.HasValue ? geom.Value.Forward : part_transform.forward;
            Vector3d velocity_normalized = velocity / v_scalar;

            double q = density * v_scalar * v_scalar * 0.0005; //dynamic pressure, q

            //Projection of velocity vector onto the plane of the wing
            _scratch.ParallelInPlane = Vector3d.Exclude(forward, velocity).normalized;
            //This just gives the vector to cross with the velocity vector
            _scratch.perp = Vector3d.Cross(forward, _scratch.ParallelInPlane).normalized;
            _scratch.liftDirection = Vector3d.Cross(_scratch.perp, velocity).normalized;

            _scratch.ParallelInPlaneLocal = geom.HasValue
                                                ? geom.Value.InverseTransformDirection(_scratch.ParallelInPlane)
                                                : part_transform.InverseTransformDirection(_scratch.ParallelInPlane);

            // Calculate the adjusted AC position (uses ParallelInPlane)
            _scratch.AerodynamicCenter = CalculateAerodynamicCenter(MachNumber, AoA, _scratch.CurWingCentroid);

            //Throw AoA into lifting line theory and adjust for part exposure and compressibility effects

            // Outside flight the Reynolds number cannot be derived here: the editor sim passes a
            // unit velocity vector so that q comes out to exactly 1 and forces read as
            // coefficients, which makes v_scalar useless for this. The 0.005 fallback is roughly
            // double the true coefficient at sea-level Reynolds numbers and below it at low ones,
            // so callers that know the real condition being simulated can pass it in instead.
            double skinFrictionDrag = skinFrictionOverride ??
                                      (HighLogic.LoadedSceneIsFlight
                                           ? FARAeroUtil.SkinFrictionDrag(density,
                                                                          _scratch.effective_MAC,
                                                                          v_scalar,
                                                                          MachNumber,
                                                                          SimThreadContext.Prediction?.ExternalTemperature ??
                                                                          vessel.externalTemperature,
                                                                          SimThreadContext.Prediction?.VesselAdiabaticIndex ??
                                                                          FARAtmosphere.GetAdiabaticIndex(vessel))
                                           : 0.005);


            skinFrictionDrag *= 1.1; //account for thickness

            CalculateCoefficients(MachNumber, AoA, skinFrictionDrag);


            //lift and drag vectors
            Vector3d L, D;
            double submergedPortion = snapshot ? captured.CapturedSubmergedPortion : part.submergedPortion;
            if (failureForceScaling >= 1 && submergedPortion > 0)
            {
                double submergedLiftScalar = snapshot ? captured.CapturedSubmergedLiftScalar : part.submergedLiftScalar;
                double submergedDragScalar = snapshot ? captured.CapturedSubmergedDragScalar : part.submergedDragScalar;
                //lift; submergedDynPreskPa handles lift
                L = _scratch.liftDirection *
                    (_scratch.Cl * S) *
                    q *
                    (submergedPortion * submergedLiftScalar + 1 - submergedPortion);
                //drag is parallel to velocity vector
                D = -velocity_normalized *
                    (_scratch.Cd * S) *
                    q *
                    (submergedPortion * submergedDragScalar + 1 - submergedPortion);
            }
            else
            {
                //lift; submergedDynPreskPa handles lift
                L = _scratch.liftDirection * (_scratch.Cl * S) * q;
                //drag is parallel to velocity vector
                D = -velocity_normalized * (_scratch.Cd * S) * q;
            }

            // Arrow display touches Unity objects (ArrowPointer create/destroy); skip it on the
            // frozen sweep path, which may run off the main thread.
            if (updateAeroArrows && !geom.HasValue)
                UpdateAeroDisplay(L, D);

            Vector3d force = L + D;
            if (double.IsNaN(force.sqrMagnitude) || double.IsNaN(_scratch.AerodynamicCenter.sqrMagnitude))
            {
                FARLogAnyThread.Warning("Error: Aerodynamic force = " +
                                        force.magnitude +
                                        " AC Loc = " +
                                        _scratch.AerodynamicCenter.magnitude +
                                        " AoA = " +
                                        AoA +
                                        "\n\rMAC = " +
                                        _scratch.effective_MAC +
                                        " B_2 = " +
                                        _scratch.effective_b_2 +
                                        " sweepAngle = " +
                                        _scratch.cosSweepAngle +
                                        "\n\rMidChordSweep = " +
                                        MidChordSweep +
                                        " MidChordSweepSideways = " +
                                        MidChordSweepSideways +
                                        "\n\r at " +
                                        (part.partInfo != null ? part.partInfo.name : "?"));
                force = _scratch.AerodynamicCenter = Vector3d.zero;
            }

            bool sweep = !snapshot && SweepGeom.HasValue;
            double rbMass = snapshot ? captured.CapturedRbMass : sweep ? simRbMass : part.rb.mass;
            double fixedDt = snapshot ? captured.CapturedFixedDeltaTime : sweep ? simFixedDeltaTime : TimeWarp.fixedDeltaTime;
            double numericalControlFactor = rbMass * v_scalar * 0.67 / (force.magnitude * fixedDt);
            force *= Math.Min(numericalControlFactor, 1);

            // Sync scratch back to instance fields for KSPField/observable consumers and the UI.
            // NOTE: the solve path itself never reads these instance fields - downstream wings and
            // the sim read the per-slot scratch via the Get*() accessors (GetStall/GetRawAoAmax/
            // GetCd0/GetCdInduced/GetAerodynamicCenter). Do not reintroduce a direct instance read on
            // the solve path. Only slot 0 (flight and the serial editor tabs, on the main thread)
            // writes them: a worker or another slot writing here would race with the main thread and
            // change what the flight and the UI see.
            if (SimThreadContext.Slot != 0)
                return force;
            Cl = _scratch.Cl;
            Cd = _scratch.Cd;
            CdInduced = _scratch.CdInduced;
            CdProfile = _scratch.CdProfile;
            stall = _scratch.stall;
            e = _scratch.e;
            rawAoAmax = _scratch.rawAoAmax;
            AerodynamicCenter = _scratch.AerodynamicCenter;
            liftDirection = _scratch.liftDirection;

            return force;
        }

        private void Update()
        {
            if (updateMassNextFrame)
            {
                GetRefAreaChildren();
                UpdateMassToAccountForArea();
                updateMassNextFrame = false;
            }
            else if (HighLogic.LoadedSceneIsEditor && !massMultiplier.NearlyEqual(oldMassMultiplier))
            {
                GetRefAreaChildren();
                UpdateMassToAccountForArea();
            }
        }

        [KSPEvent]
        public void OnWingAttach()
        {
            if (part.parent)
                parentWing = part.parent.GetComponent<FARWingAerodynamicModel>();

            GetRefAreaChildren();

            UpdateMassToAccountForArea();
        }

        public void OnWingDetach()
        {
            if (!(parentWing is null))
                parentWing.updateMassNextFrame = true;
        }

        private void UpdateMassToAccountForArea()
        {
            float supportedArea = (float)(refAreaChildren + S);
            if (!(parentWing is null))
                //if any supported area has been transferred to another part, we must remove it from here
                supportedArea *= 0.66666667f;
            curWingMass = supportedArea * (float)FARAeroUtil.massPerWingAreaSupported * massMultiplier;

            desiredMass = curWingMass * wingBaseMassMultiplier;

            oldMassMultiplier = massMultiplier;
        }

        private void GetRefAreaChildren()
        {
            refAreaChildren = 0;

            foreach (Part p in part.children)
            {
                FARWingAerodynamicModel childWing = p.GetComponent<FARWingAerodynamicModel>();
                if (childWing is null)
                    continue;

                //Take 1/3 of the area of the child wings
                refAreaChildren += (childWing.refAreaChildren + childWing.S) * 0.33333333333333333333;
            }

            if (parentWing is null)
                return;
            parentWing.GetRefAreaChildren();
            parentWing.UpdateMassToAccountForArea();
        }

        public virtual double CalculateAoA(Vector3d velocity)
        {
            Vector3 forward;
            if (SimThreadContext.LiveOnly)
                forward = part_transform.forward;
            else
            {
                FrozenPartTransform? g = SolveGeom;
                forward = g.HasValue ? g.Value.Forward : part_transform.forward;
            }
            double PerpVelocity = Vector3d.Dot(forward, velocity.normalized);
            return Math.Asin(PerpVelocity.Clamp(-1, 1));
        }

        //Calculates camber and flap effects due to wing interactions
        private void CalculateWingCamberInteractions(
            double MachNumber,
            double AoA,
            out double ACshift,
            out double ACweight
        )
        {
            ACshift = 0;
            ACweight = 0;
            _scratch.ClIncrementFromRear = 0;

            _scratch.rawAoAmax = CalculateAoAmax(MachNumber);

            _scratch.liftslope = _scratch.rawLiftSlope;
            wingInteraction.UpdateOrientationForInteraction(_scratch.ParallelInPlaneLocal);
            wingInteraction.CalculateEffectsOfUpstreamWing(AoA,
                                                           MachNumber,
                                                           _scratch.ParallelInPlaneLocal,
                                                           ref ACweight,
                                                           ref ACshift,
                                                           ref _scratch.ClIncrementFromRear);
            double effectiveUpstreamInfluence = wingInteraction.EffectiveUpstreamInfluence;

            if (effectiveUpstreamInfluence > 0)
            {
                effectiveUpstreamInfluence = wingInteraction.EffectiveUpstreamInfluence;

                _scratch.AoAmax = wingInteraction.EffectiveUpstreamAoAMax;
                _scratch.liftslope *= 1 - effectiveUpstreamInfluence;
                _scratch.liftslope += wingInteraction.EffectiveUpstreamLiftSlope;

                _scratch.cosSweepAngle *= 1 - effectiveUpstreamInfluence;
                _scratch.cosSweepAngle += wingInteraction.EffectiveUpstreamCosSweepAngle;
                _scratch.cosSweepAngle = _scratch.cosSweepAngle.Clamp(0d, 1d);
            }
            else
            {
                _scratch.liftslope = _scratch.rawLiftSlope;
                _scratch.AoAmax = 0;
            }

            _scratch.AoAmax += _scratch.rawAoAmax;
        }

        //Calculates current stall fraction based on previous stall fraction and current data.
        private void DetermineStall(double AoA)
        {
            double lastStall = _scratch.stall;
            double effectiveUpstreamStall = wingInteraction.EffectiveUpstreamStall;

            _scratch.stall = 0;
            double absAoA = Math.Abs(AoA);

            if (absAoA > _scratch.AoAmax)
            {
                _scratch.stall = ((absAoA - _scratch.AoAmax) * 10).Clamp(0, 1);
                _scratch.stall = Math.Max(_scratch.stall, lastStall);
                _scratch.stall += effectiveUpstreamStall;
            }
            else if (absAoA < _scratch.AoAmax)
            {
                _scratch.stall = 1 - ((_scratch.AoAmax - absAoA) * 25).Clamp(0, 1);
                _scratch.stall = Math.Min(_scratch.stall, lastStall);
                _scratch.stall += effectiveUpstreamStall;
            }
            else
            {
                _scratch.stall = lastStall;
            }

            _scratch.stall = _scratch.stall.Clamp(0, 1);
            if (_scratch.stall < 1e-5)
                _scratch.stall = 0;
        }


        /// <summary>
        ///     This calculates the lift and drag coefficients
        /// </summary>
        private void CalculateCoefficients(double MachNumber, double AoA, double skinFrictionCoefficient)
        {
            _scratch.minStall = 0;

            _scratch.rawLiftSlope = CalculateSubsonicLiftSlope(MachNumber); // / AoA;     //Prandtl lifting Line


            CalculateWingCamberInteractions(MachNumber, AoA, out double ACshift, out double ACweight);
            DetermineStall(AoA);

            double beta = Math.Sqrt(MachNumber * MachNumber - 1);
            if (double.IsNaN(beta) || beta < 0.66332495807107996982298654733414)
                beta = 0.66332495807107996982298654733414;

            double TanSweep = Math.Sqrt((1 - _scratch.cosSweepAngle * _scratch.cosSweepAngle).Clamp(0, 1)) / _scratch.cosSweepAngle;
            double beta_TanSweep = beta / TanSweep;


            double Cd0 = CdCompressibilityZeroLiftIncrement(MachNumber, _scratch.cosSweepAngle, TanSweep, beta_TanSweep, beta) +
                         2 * skinFrictionCoefficient;
            _scratch.CdProfile = Cd0; // zero-lift + skin friction; the induced part is captured per branch
            double CdMax = CdMaxFlatPlate(MachNumber, beta);
            _scratch.e = FARAeroUtil.CalculateOswaldsEfficiencyNitaScholz(_scratch.effective_AR, _scratch.cosSweepAngle, Cd0, TaperRatio);
            _scratch.piARe = _scratch.effective_AR * _scratch.e * Math.PI;

            double CosAoA = Math.Cos(AoA);

            if (MachNumber <= 0.8)
            {
                double Cn = _scratch.liftslope;
                _scratch.FinalLiftSlope = _scratch.liftslope;
                double sinAoA = Math.Sqrt((1 - CosAoA * CosAoA).Clamp(0, 1));
                _scratch.Cl = Cn * CosAoA * Math.Sign(AoA);

                _scratch.Cl += _scratch.ClIncrementFromRear;
                _scratch.Cl *= sinAoA;

                if (Math.Abs(_scratch.Cl) > Math.Abs(ACweight))
                    ACshift *= Math.Abs(ACweight / _scratch.Cl).Clamp(0, 1);
                _scratch.Cd = _scratch.Cl * _scratch.Cl / _scratch.piARe; //Drag due to 3D effects on wing and base constant
                _scratch.CdInduced = _scratch.Cd;
                _scratch.Cd += Cd0;
            }
            /*
             * Supersonic nonlinear lift / drag code
             *
             */
            else if (MachNumber > 1.4)
            {
                double coefMult = 2 / (FARAeroUtil.CurrentAdiabaticIndex * MachNumber * MachNumber);

                double supersonicLENormalForceFactor = CalculateSupersonicLEFactor(beta, TanSweep, beta_TanSweep);

                double normalForce = GetSupersonicPressureDifference(MachNumber, AoA);
                _scratch.FinalLiftSlope = coefMult * normalForce * supersonicLENormalForceFactor;

                _scratch.Cl = _scratch.FinalLiftSlope * CosAoA * Math.Sign(AoA);
                _scratch.Cd = beta * _scratch.Cl * _scratch.Cl / _scratch.piARe;
                _scratch.CdInduced = _scratch.Cd;

                _scratch.Cd += Cd0;
            }
            /*
             * Transonic nonlinear lift / drag code
             * This uses a blend of subsonic and supersonic aerodynamics to try and smooth the gap between the two regimes
             */
            else
            {
                //This determines the weight of supersonic flow; subsonic uses 1-this
                double supScale = 2 * MachNumber;
                supScale -= 6.6;
                supScale *= MachNumber;
                supScale += 6.72;
                supScale *= MachNumber;
                supScale += -2.176;
                supScale *= -4.6296296296296296296296296296296;

                double Cn = _scratch.liftslope;
                double sinAoA = Math.Sqrt((1 - CosAoA * CosAoA).Clamp(0, 1));
                _scratch.Cl = Cn * CosAoA * sinAoA * Math.Sign(AoA);

                if (MachNumber <= 1)
                {
                    _scratch.Cl += _scratch.ClIncrementFromRear * sinAoA;
                    if (Math.Abs(_scratch.Cl) > Math.Abs(ACweight))
                        ACshift *= Math.Abs(ACweight / _scratch.Cl).Clamp(0, 1);
                }

                _scratch.FinalLiftSlope = Cn * (1 - supScale);
                _scratch.Cl *= 1 - supScale;

                double M = MachNumber.Clamp(1.2, double.PositiveInfinity);

                double coefMult = 2 / (FARAeroUtil.CurrentAdiabaticIndex * M * M);

                double supersonicLENormalForceFactor = CalculateSupersonicLEFactor(beta, TanSweep, beta_TanSweep);

                double normalForce = GetSupersonicPressureDifference(M, AoA);

                double supersonicLiftSlope = coefMult * normalForce * supersonicLENormalForceFactor * supScale;
                _scratch.FinalLiftSlope += supersonicLiftSlope;


                _scratch.Cl += CosAoA * Math.Sign(AoA) * supersonicLiftSlope;

                double effectiveBeta = beta * supScale + (1 - supScale);

                _scratch.Cd = effectiveBeta * _scratch.Cl * _scratch.Cl / _scratch.piARe;
                _scratch.CdInduced = _scratch.Cd;

                _scratch.Cd += Cd0;
            }

            //AC shift due to flaps
            Vector3d ACShiftVec;
            if (!double.IsNaN(ACshift) && MachNumber <= 1)
                ACShiftVec = ACshift * _scratch.ParallelInPlane;
            else
                ACShiftVec = Vector3d.zero;

            //Stalling effects
            _scratch.stall = _scratch.stall.Clamp(_scratch.minStall, 1);

            //AC shift due to stall
            if (_scratch.stall > 0)
                ACShiftVec -= 0.75 / criticalCl * MAC_actual * Math.Abs(_scratch.Cl) * _scratch.stall * _scratch.ParallelInPlane * CosAoA;

            _scratch.Cl -= _scratch.Cl * _scratch.stall * 0.769;
            _scratch.Cd += _scratch.Cd * _scratch.stall * 3;
            _scratch.Cd = Math.Max(_scratch.Cd, CdMax * (1 - CosAoA * CosAoA));

            _scratch.AerodynamicCenter += ACShiftVec;

            _scratch.Cl *= wingInteraction.ClInterferenceFactor;

            _scratch.FinalLiftSlope *= wingInteraction.ClInterferenceFactor;
            // Read by GetPotentialTorque (the control authority SAS and autopilots see), so only slot 0 sets it.
            if (SimThreadContext.Slot == 0)
                FinalLiftSlope = _scratch.FinalLiftSlope;

            _scratch.ClIncrementFromRear = 0;
        }

        //Calculates effect of the Mach cone being in front of, along, or behind the leading edge of the wing
        private double CalculateSupersonicLEFactor(double beta, double TanSweep, double beta_TanSweep)
        {
            double SupersonicLEFactor;
            double ARTanSweep = _scratch.effective_AR * TanSweep;

            if (beta_TanSweep < 1) //"subsonic" leading edge, scales with Tan Sweep
            {
                if (beta_TanSweep < 0.5)
                {
                    SupersonicLEFactor = 1.57 * _scratch.effective_AR;
                    SupersonicLEFactor /= ARTanSweep + 0.5;
                }
                else
                {
                    SupersonicLEFactor = (1.57 - 0.28 * (beta_TanSweep - 0.5)) * _scratch.effective_AR;
                    SupersonicLEFactor /= ARTanSweep + 0.5 - (beta_TanSweep - 0.5) * 0.25;
                }

                SupersonicLEFactor *= beta;
            }
            else //"supersonic" leading edge, scales with beta
            {
                beta_TanSweep = 1 / beta_TanSweep;

                SupersonicLEFactor = 1.43 * ARTanSweep;
                SupersonicLEFactor /= ARTanSweep + 0.375;
                SupersonicLEFactor--;
                SupersonicLEFactor *= beta_TanSweep;
                SupersonicLEFactor++;
            }

            return SupersonicLEFactor;
        }

        //This models the wing using a symmetric diamond airfoil

        private static double GetSupersonicPressureDifference(double M, double AoA)
        {
            double maxSinBeta = FARAeroUtil.CalculateSinMaxShockAngle(M, FARAeroUtil.CurrentAdiabaticIndex);
            double minSinBeta = 1 / M;

            //In radians, Corresponds to ~2.8 degrees or approximately what you would get from a ~4.8% thick diamond airfoil
            const double halfAngle = 0.05;

            double AbsAoA = Math.Abs(AoA);

            //Region 1 is the upper surface ahead of the max thickness
            double angle1 = halfAngle - AbsAoA;
            double M1;
            //pressure ratio wrt to freestream pressure
            double p1 = angle1 >= 0
                            ? ShockWaveCalculation(angle1, M, out M1, maxSinBeta, minSinBeta)
                            : PMExpansionCalculation(Math.Abs(angle1), M, out M1);

            //Region 2 is the upper surface behind the max thickness
            double p2 = PMExpansionCalculation(2 * halfAngle, M1) * p1;

            //Region 3 is the lower surface ahead of the max thickness
            double angle3 = halfAngle + AbsAoA;
            //pressure ratio wrt to freestream pressure
            double p3 = ShockWaveCalculation(angle3, M, out double M3, maxSinBeta, minSinBeta);

            //Region 4 is the lower surface behind the max thickness
            double p4 = PMExpansionCalculation(2 * halfAngle, M3) * p3;

            double pRatio = (p3 + p4 - (p1 + p2)) * 0.5;

            return pRatio;
        }

        //Calculates pressure ratio of turning a supersonic flow through a particular angle using a shockwave
        private static double ShockWaveCalculation(
            double angle,
            double inM,
            out double outM,
            double maxSinBeta,
            double minSinBeta
        )
        {
            double sinBeta =
                FARAeroUtil.CalculateSinWeakObliqueShockAngle(inM, FARAeroUtil.CurrentAdiabaticIndex, angle);
            if (double.IsNaN(sinBeta))
                sinBeta = maxSinBeta;

            sinBeta.Clamp(minSinBeta, maxSinBeta);

            double normalInM = sinBeta * inM;
            normalInM = normalInM.Clamp(1, double.PositiveInfinity);

            double tanM = inM * Math.Sqrt((1 - sinBeta * sinBeta).Clamp(0, 1));

            double normalOutM = FARAeroUtil.MachBehindShockCalc(normalInM);

            outM = Math.Sqrt(normalOutM * normalOutM + tanM * tanM);

            double pRatio = FARAeroUtil.PressureBehindShockCalc(normalInM);

            return pRatio;
        }

        //Calculates pressure ratio due to turning a supersonic flow through a Prandtl-Meyer Expansion
        private static double PMExpansionCalculation(double angle, double inM, out double outM)
        {
            inM = inM.Clamp(1, double.PositiveInfinity);
            double nu1 = FARAeroUtil.PrandtlMeyerMachEval((float)inM);
            double theta = angle * FARMathUtil.rad2deg;
            double nu2 = nu1 + theta;
            if (nu2 >= FARAeroUtil.maxPrandtlMeyerTurnAngle)
                nu2 = FARAeroUtil.maxPrandtlMeyerTurnAngle;
            outM = FARAeroUtil.PrandtlMeyerAngleEval((float)nu2);

            return FARAeroUtil.StagnationPressureCalc(inM) / FARAeroUtil.StagnationPressureCalc(outM);
        }

        //Calculates pressure ratio due to turning a supersonic flow through a Prandtl-Meyer Expansion
        private static double PMExpansionCalculation(double angle, double inM)
        {
            inM = inM.Clamp(1, double.PositiveInfinity);
            double nu1 = FARAeroUtil.PrandtlMeyerMachEval((float)inM);
            double theta = angle * FARMathUtil.rad2deg;
            double nu2 = nu1 + theta;
            if (nu2 >= FARAeroUtil.maxPrandtlMeyerTurnAngle)
                nu2 = FARAeroUtil.maxPrandtlMeyerTurnAngle;
            float outM = FARAeroUtil.PrandtlMeyerAngleEval((float)nu2);

            return FARAeroUtil.StagnationPressureCalc(inM) / FARAeroUtil.StagnationPressureCalc(outM);
        }

        //Short calculation for peak AoA for stalling
        protected double CalculateAoAmax(double MachNumber)
        {
            double StallAngle;
            if (MachNumber < 0.8)
            {
                StallAngle = criticalCl / _scratch.liftslope;
            }
            else if (MachNumber > 1.4)
            {
                StallAngle = 1.0471975511965977461542144610932; //60 degrees in radians
            }
            else
            {
                double tmp = criticalCl / _scratch.liftslope;
                StallAngle = (MachNumber - 0.8) *
                             (1.0471975511965977461542144610932 - tmp) *
                             1.6666666666666666666666666666667 +
                             tmp;
            }

            return StallAngle;
        }

        //Calculates subsonic liftslope
        private double CalculateSubsonicLiftSlope(double MachNumber)
        {
            double CosPartAngle = Vector3.Dot(sweepPerpLocal, _scratch.ParallelInPlaneLocal).Clamp(-1, 1);
            double tmp = Vector3.Dot(sweepPerp2Local, _scratch.ParallelInPlaneLocal).Clamp(-1, 1);

            //Based on perpendicular vector find which line is the right one
            double sweepHalfChord = Math.Abs(CosPartAngle) > Math.Abs(tmp) ? CosPartAngle : tmp;

            CosPartAngle = _scratch.ParallelInPlaneLocal.y.Clamp(-1, 1);

            CosPartAngle *= CosPartAngle;
            //Get the squared values for the angles
            double SinPartAngle2 = (1d - CosPartAngle).Clamp(0, 1);

            _scratch.effective_b_2 = Math.Max(b_2_actual * CosPartAngle, MAC_actual * SinPartAngle2);
            _scratch.effective_MAC = MAC_actual * CosPartAngle + b_2_actual * SinPartAngle2;
            _scratch.transformed_AR = _scratch.effective_b_2 / _scratch.effective_MAC;

            //convert to tangent
            sweepHalfChord = Math.Sqrt(Math.Max(1 - sweepHalfChord * sweepHalfChord, 0)) / sweepHalfChord;

            SetSweepAngle(sweepHalfChord);

            _scratch.effective_AR = wingInteraction.SpanChainedAR(_scratch.transformed_AR,
                                                                  out double bodyLiftFactor,
                                                                  out double groupTanSweep);

            //Even this range of effective ARs is large, but it keeps the Oswald's Efficiency numbers in check
            _scratch.effective_AR = _scratch.effective_AR.Clamp(0.25, 30d);

            if (MachNumber < 0.9)
                tmp = 1d - MachNumber * MachNumber;
            else
                tmp = 0.19;

            // a wing on a bare body lifts as the whole wing through the body: that wing's half-chord sweep
            double sweepTmp = double.IsNaN(groupTanSweep) ? sweepHalfChord : groupTanSweep;
            sweepTmp *= sweepTmp;

            tmp += sweepTmp;
            tmp = tmp * _scratch.effective_AR * _scratch.effective_AR;
            tmp += 4;
            tmp = Math.Sqrt(tmp);
            tmp += 2;
            tmp = 1 / tmp;
            tmp *= 2 * Math.PI;

            // and the panels carry that wing's lift on the body between its roots
            return tmp * _scratch.effective_AR * bodyLiftFactor;
        }

        //Transforms cos sweep of the midchord to cosine(sweep of the leading edge)
        private void SetSweepAngle(double tanSweepHalfChord)
        {
            double tmp = (1d - TaperRatio) / (1d + TaperRatio);
            tmp *= 2d / _scratch.transformed_AR;
            tanSweepHalfChord += tmp;
            _scratch.cosSweepAngle = 1d / Math.Sqrt(1d + tanSweepHalfChord * tanSweepHalfChord);
            if (_scratch.cosSweepAngle > 1d)
                _scratch.cosSweepAngle = 1d;
        }


        /// <summary>
        ///     Calculates Cd at 90 degrees AoA so that the numbers are done correctly
        /// </summary>
        private static double CdMaxFlatPlate(double M, double beta)
        {
            if (M < 0.5)
                return 2;
            if (M > 1.2)
                return 0.4 / (beta * beta) + 1.75;
            if (M >= 1)
                return 3.39 - 0.609091 * M;
            double result = M - 0.5;
            result *= result;
            return result * 2 + 2;
        }

        /// <summary>
        ///     This modifies the Cd to account for compressibility effects due to increasing Mach number
        /// </summary>
        private double CdCompressibilityZeroLiftIncrement(
            double M,
            double SweepAngle,
            double TanSweep,
            double beta_TanSweep,
            double beta
        )
        {
            double thisInteractionFactor = 1;
            if (wingInteraction.HasWingsUpstream)
            {
                if (wingInteraction.EffectiveUpstreamInfluence > 0.99)
                {
                    _scratch.zeroLiftCdIncrement = wingInteraction.EffectiveUpstreamCd0;
                    return _scratch.zeroLiftCdIncrement;
                }

                thisInteractionFactor = 1 - wingInteraction.EffectiveUpstreamInfluence;
            }

            //Based on the method of DATCOM Section 4.1.5.1-C
            if (M > 1.4)
            {
                //Subsonic leading edge
                if (beta_TanSweep < 1)
                    //This constant is due to airfoil shape and thickness
                    _scratch.zeroLiftCdIncrement = 0.009216 / TanSweep;
                //Supersonic leading edge
                else
                    _scratch.zeroLiftCdIncrement = 0.009216 / beta;
                _scratch.zeroLiftCdIncrement *= thisInteractionFactor;
                _scratch.zeroLiftCdIncrement +=
                    wingInteraction.EffectiveUpstreamCd0 * wingInteraction.EffectiveUpstreamInfluence;
                return _scratch.zeroLiftCdIncrement;
            }


            //Based on the method of DATCOM Section 4.1.5.1-B
            double tmp = 1 / Math.Sqrt(SweepAngle);

            double dd_MachNumber = 0.8 * tmp; //Find Drag Divergence Mach Number

            if (M < dd_MachNumber) //If below this number,
            {
                _scratch.zeroLiftCdIncrement = 0;
                return 0;
            }

            double peak_MachNumber = 1.1 * tmp;

            double peak_Increment = 0.025 * FARMathUtil.PowApprox(SweepAngle, 2.5);

            if (M > peak_MachNumber)
            {
                _scratch.zeroLiftCdIncrement = peak_Increment;
            }
            else
            {
                tmp = dd_MachNumber - peak_MachNumber;
                tmp = tmp * tmp * tmp;
                tmp = 1 / tmp;

                double CdIncrement = 2 * M;
                CdIncrement -= 3 * (dd_MachNumber + peak_MachNumber);
                CdIncrement *= M;
                CdIncrement += 6 * dd_MachNumber * peak_MachNumber;
                CdIncrement *= M;
                CdIncrement += dd_MachNumber * dd_MachNumber * (dd_MachNumber - 3 * peak_MachNumber);
                CdIncrement *= tmp;
                CdIncrement *= peak_Increment;

                _scratch.zeroLiftCdIncrement = CdIncrement;
            }

            double scalingMachNumber = Math.Min(peak_MachNumber, 1.2);

            if (M < scalingMachNumber)
            {
                _scratch.zeroLiftCdIncrement *= thisInteractionFactor;
                _scratch.zeroLiftCdIncrement +=
                    wingInteraction.EffectiveUpstreamCd0 * wingInteraction.EffectiveUpstreamInfluence;
                return _scratch.zeroLiftCdIncrement;
            }

            double scale = (M - 1.4) / (scalingMachNumber - 1.4);
            _scratch.zeroLiftCdIncrement *= scale;
            scale = 1 - scale;

            //Subsonic leading edge
            if (beta_TanSweep < 1)
                //This constant is due to airfoil shape and thickness
                _scratch.zeroLiftCdIncrement += 0.009216 / TanSweep * scale;
            //Supersonic leading edge
            else
                _scratch.zeroLiftCdIncrement += 0.009216 / beta * scale;
            _scratch.zeroLiftCdIncrement *= thisInteractionFactor;
            _scratch.zeroLiftCdIncrement += wingInteraction.EffectiveUpstreamCd0 * wingInteraction.EffectiveUpstreamInfluence;

            return _scratch.zeroLiftCdIncrement;
        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            if (node.HasValue("b_2"))
                double.TryParse(node.GetValue("b_2"), out b_2);
            if (node.HasValue("MAC"))
                double.TryParse(node.GetValue("MAC"), out MAC);
            if (node.HasValue("TaperRatio"))
                double.TryParse(node.GetValue("TaperRatio"), out TaperRatio);
            if (node.HasValue("nonSideAttach"))
                int.TryParse(node.GetValue("nonSideAttach"), out nonSideAttach);
            if (node.HasValue("MidChordSweep"))
                double.TryParse(node.GetValue("MidChordSweep"), out MidChordSweep);
            if (node.HasValue("massOverride") && float.TryParse(node.GetValue("massOverride"), out float mass))
                MassOverride = mass;
        }

        private void UpdateAeroDisplay(Vector3 lift, Vector3 drag)
        {
            if (PhysicsGlobals.AeroForceDisplay)
            {
                if (liftArrow == null)
                {
                    liftArrow = ArrowPointer.Create(part_transform,
                                                    localWingCentroid,
                                                    lift,
                                                    lift.magnitude * FARKSPAddonFlightScene.FARAeroForceDisplayScale,
                                                    FARConfig.GUIColors.ClColor,
                                                    true);
                }
                else
                {
                    liftArrow.Direction = lift;
                    liftArrow.Length = lift.magnitude * FARKSPAddonFlightScene.FARAeroForceDisplayScale;
                }

                if (dragArrow == null)
                {
                    dragArrow = ArrowPointer.Create(part_transform,
                                                    localWingCentroid,
                                                    drag,
                                                    drag.magnitude * FARKSPAddonFlightScene.FARAeroForceDisplayScale,
                                                    FARConfig.GUIColors.CdColor,
                                                    true);
                }
                else
                {
                    dragArrow.Direction = drag;
                    dragArrow.Length = drag.magnitude * FARKSPAddonFlightScene.FARAeroForceDisplayScale;
                }
            }
            else
            {
                if (!(liftArrow is null))
                {
                    Destroy(liftArrow);
                    liftArrow = null;
                }

                if (!(dragArrow is null))
                {
                    Destroy(dragArrow);
                    dragArrow = null;
                }
            }

            if (PhysicsGlobals.AeroDataDisplay)
            {
                if (!fieldsVisible)
                {
                    Fields["dragForceWing"].guiActive = true;
                    Fields["liftForceWing"].guiActive = true;
                    fieldsVisible = true;
                }

                dragForceWing = drag.magnitude;
                liftForceWing = lift.magnitude;
            }
            else if (fieldsVisible)
            {
                Fields["dragForceWing"].guiActive = false;
                Fields["liftForceWing"].guiActive = false;
                fieldsVisible = false;
            }
        }

        protected override void OnDestroy()
        {
            base.OnDestroy();

            if (liftArrow != null)
            {
                Destroy(liftArrow);
                liftArrow = null;
            }

            if (dragArrow != null)
            {
                Destroy(dragArrow);
                dragArrow = null;
            }

            // ReSharper disable once InvertIf
            if (wingInteraction != null)
            {
                wingInteraction.Destroy();
                wingInteraction = null;
            }
        }
    }
}
