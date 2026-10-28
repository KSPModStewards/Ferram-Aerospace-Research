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
using System.Threading;
using FerramAerospaceResearch;
using FerramAerospaceResearch.Settings;
using UnityEngine;

// ReSharper disable once CheckNamespace
namespace ferram4
{
    public class FARControllableSurface : FARWingAerodynamicModel, ITorqueProvider
    {
        protected Transform movableSection;

        private bool flipAxis;


        // ReSharper disable once NotAccessedField.Global
        [KSPField(isPersistant = false)] public Vector3 controlSurfacePivot = new Vector3(1f, 0f, 0f);

        [KSPField(isPersistant = false)] public float ctrlSurfFrac = 1;

        [KSPField(isPersistant = false)] public string transformName = "obj_ctrlSrf";

        // These TWO fields MUST be set up so that they are copied by Object.Instantiate.
        // Otherwise detaching and re-attaching wings with deflected flaps etc breaks until save/load.
        [SerializeField] protected Quaternion MovableOrig = Quaternion.identity;

        [SerializeField] private bool MovableOrigReady;

        //        protected int MovableSectionFlip = 1;

        private bool prevStdCtrl = true;

        [KSPField(guiName = "FARCtrlSurfPitch", isPersistant = true, guiActiveEditor = false, guiActive = false),
         UI_FloatRange(affectSymCounterparts = UI_Scene.All,
                       maxValue = 100.0f,
                       minValue = -100f,
                       scene = UI_Scene.All,
                       stepIncrement = 5f)]
        public float pitchaxis = 100.0f;

        [KSPField(guiName = "FARCtrlSurfYaw", isPersistant = true, guiActiveEditor = false, guiActive = false),
         UI_FloatRange(affectSymCounterparts = UI_Scene.All,
                       maxValue = 100.0f,
                       minValue = -100f,
                       scene = UI_Scene.All,
                       stepIncrement = 5f)]
        public float yawaxis = 100.0f;

        [KSPField(guiName = "FARCtrlSurfRoll", isPersistant = true, guiActiveEditor = false, guiActive = false),
         UI_FloatRange(affectSymCounterparts = UI_Scene.All,
                       maxValue = 100.0f,
                       minValue = -100f,
                       scene = UI_Scene.All,
                       stepIncrement = 5f)]
        public float rollaxis = 100.0f;

        [KSPField(guiName = "FARCtrlSurfAoA", isPersistant = true, guiActiveEditor = false, guiActive = false),
         UI_FloatRange(affectSymCounterparts = UI_Scene.All,
                       maxValue = 200.0f,
                       minValue = -200f,
                       scene = UI_Scene.All,
                       stepIncrement = 5f)]
        public float pitchaxisDueToAoA;

        [KSPField(guiName = "FARCtrlSurfBrakeRudder", isPersistant = true, guiActiveEditor = false, guiActive = false),
         UI_FloatRange(affectSymCounterparts = UI_Scene.All,
                       maxValue = 100.0f,
                       minValue = -100f,
                       scene = UI_Scene.All,
                       stepIncrement = 5f)]
        public float brakeRudder;

        [KSPField(guiName = "FARCtrlSurfCtrlDeflect", guiActiveEditor = false, isPersistant = true),
         UI_FloatRange(affectSymCounterparts = UI_Scene.All,
                       maxValue = 40,
                       minValue = -40,
                       scene = UI_Scene.All,
                       stepIncrement = 0.5f)]
        public float maxdeflect = 15;

        // ReSharper disable once ConvertToConstant.Local
        // ReSharper disable once FieldCanBeMadeReadOnly.Local
        [KSPField(guiName = "FARCtrlSurfFlapSpoiler", guiActiveEditor = true, guiActive = true),
         UI_Toggle(affectSymCounterparts = UI_Scene.All,
                   scene = UI_Scene.All,
                   disabledText = "FARCtrlSurfStdText",
                   enabledText = "FARCtrlSurfStdText")]
        private bool showFlpCtrl = false;

        // ReSharper disable once FieldCanBeMadeReadOnly.Local
        // ReSharper disable once ConvertToConstant.Local
        [KSPField(guiName = "FARCtrlSurfStdTitle", guiActiveEditor = true, guiActive = true),
         UI_Toggle(affectSymCounterparts = UI_Scene.All,
                   scene = UI_Scene.All,
                   disabledText = "FARCtrlSurfStdText",
                   enabledText = "FARCtrlSurfStdText")]
        private bool showStdCtrl = false;

        private bool prevFlpCtrl = true;

        [KSPField(guiName = "FARCtrlSurfFlap", isPersistant = true, guiActiveEditor = false, guiActive = false),
         UI_Toggle(affectSymCounterparts = UI_Scene.All,
                   enabledText = "FARCtrlSurfFlapActive",
                   scene = UI_Scene.All,
                   disabledText = "FARCtrlSurfFlapInActive")]
        public bool isFlap;

        private bool prevIsFlap;

        [KSPField(guiName = "FARCtrlSurfSpoiler", isPersistant = true, guiActiveEditor = false, guiActive = false),
         UI_Toggle(affectSymCounterparts = UI_Scene.All,
                   enabledText = "FARCtrlSurfFlapActive",
                   scene = UI_Scene.All,
                   disabledText = "FARCtrlSurfFlapInActive")]
        public bool isSpoiler;

        private bool prevIsSpoiler;

        [KSPField(isPersistant = true, guiName = "FARCtrlSurfFlapSetting")]
        public int flapDeflectionLevel = 2;

        [KSPField(guiName = "FARCtrlSurfFlapDeflect", guiActiveEditor = false, isPersistant = true),
         UI_FloatRange(affectSymCounterparts = UI_Scene.All,
                       maxValue = 85,
                       minValue = -85,
                       scene = UI_Scene.All,
                       stepIncrement = 0.5f)]
        public float maxdeflectFlap = 15;

        [KSPField(guiName = "FARCtrlShowDynamicDeflection", guiActiveEditor = true, guiActive = true),
         UI_Toggle(affectSymCounterparts = UI_Scene.All,
                   scene = UI_Scene.All,
                   disabledText = "FARCtrlSurfStdText",
                   enabledText = "FARCtrlSurfStdText")]
        private bool showDynamicDeflection = false;

        [KSPField(guiName = "FARCtrlDynamicDeflection",
                  isPersistant = true,
                  guiActiveEditor = false,
                  guiActive = false),
         UI_Toggle(affectSymCounterparts = UI_Scene.All,
                   enabledText = "FARCtrlSurfFlapActive",
                   scene = UI_Scene.All,
                   disabledText = "FARCtrlSurfFlapInActive")]
        public bool isDynamicDeflection = false;

        public bool isPrevDynamicDeflection = true;

        [KSPField(guiName = "FARCtrlDynamicStartSpeed",
                  isPersistant = true,
                  guiActiveEditor = false,
                  guiActive = false),
         UI_FloatRange(affectSymCounterparts = UI_Scene.All,
                       maxValue = 1000.0f,
                       minValue = 0f,
                       scene = UI_Scene.All,
                       stepIncrement = 10f)]
        public float dynamicControlStartSpeed = 200.0f;

        [KSPField(guiName = "FARCtrlDynamicExponent", isPersistant = true, guiActiveEditor = false, guiActive = false),
         UI_FloatRange(affectSymCounterparts = UI_Scene.All,
                       maxValue = 4.0f,
                       minValue = 0.0f,
                       scene = UI_Scene.All,
                       stepIncrement = 0.1f)]
        public float exponent = 2f;

        [KSPField(guiName = "FARCtrlDynamicMinControl",
                  isPersistant = true,
                  guiActiveEditor = false,
                  guiActive = false),
         UI_FloatRange(affectSymCounterparts = UI_Scene.All,
                       maxValue = 1f,
                       minValue = 0f,
                       scene = UI_Scene.All,
                       stepIncrement = 0.05f)]
        public float minControl = 0.1f;

        // Per-call control-surface state, thread-local so the parallel heatmap sweep can
        // evaluate independent cells against the same surface concurrently. The wing math
        // uses the inherited (also thread-local) _scratch; this covers the control-specific
        // fields. Backed by properties so every existing call site is unchanged; on the
        // single-threaded flight/editor paths this is the main thread's instance, so
        // behaviour there is identical.
        // Per-thread-slot control scratch (see SimThreadContext); array index beats ThreadLocal.Value
        // under Mono. Each slot is single-threaded, so the lazy create needs no synchronisation.
        private readonly ControlSurfaceScratch[] _ctrlScratchSlots =
            new ControlSurfaceScratch[SimThreadContext.MaxSlots];

        private ControlSurfaceScratch _ctrlScratch
        {
            get
            {
                int s = SimThreadContext.Slot;
                ControlSurfaceScratch sc = _ctrlScratchSlots[s];
                if (sc == null)
                {
                    sc = new ControlSurfaceScratch {IsShielded = isShielded};
                    _ctrlScratchSlots[s] = sc;
                }

                return sc;
            }
        }

        /// <summary>
        /// A prediction flies the surface as it is deflected now: the control state (deflection, deflected normal,
        /// control mapping) is the flight's, slot 0's, and its shielding the flight's.
        /// </summary>
        internal override SnapshotState CaptureSnapshotState()
        {
            SnapshotState state = base.CaptureSnapshotState();
            // The flight's control state (slot 0) and its shielding, which flight writes to the instance.
            var control = new ControlSurfaceScratch();
            ControlSurfaceScratch flight = _ctrlScratchSlots[0];
            if (flight != null)
                control.CopyControlStateFrom(flight);
            control.IsShielded = isShielded;
            control.Settings = new ControlSurfaceSettings
            {
                PitchAxis = pitchaxis,
                YawAxis = yawaxis,
                RollAxis = rollaxis,
                BrakeRudder = brakeRudder,
                MaxDeflect = maxdeflect,
                PitchAxisDueToAoA = pitchaxisDueToAoA,
                CtrlSurfFrac = ctrlSurfFrac,
                IsDynamicDeflection = isDynamicDeflection,
                Deflects = animated && movableSection is not null && part.isControllable,
                DynamicControlStartSpeed = dynamicControlStartSpeed,
                Exponent = exponent,
                MinControl = minControl,
                AtmDensityASL = vessel != null && vessel.lastBody != null ? vessel.lastBody.atmDensityASL : 1.0,
                AreaExposedFactor = NUFAR_areaExposedFactor,
                TotalExposedAreaFactor = NUFAR_totalExposedAreaFactor,
                S = S
            };
            state.Control = control;
            return state;
        }

        /// <summary>
        /// Inside a snapshot prediction, after the captured state is installed: deflects the surface as
        /// AoAOffsetFromControl would for these control inputs (-1..1), held rather than blended toward, at the simulated
        /// airflow (velocity, world frame of the capture) and density, from the settings captured with the snapshot. The
        /// flap and spoiler deflection stays as captured; the deflected normal and the shielding follow as
        /// DeflectionAnimation and CheckShielded set them. Reads no live state; any thread.
        /// </summary>
        internal void ApplySnapshotControls(double pitch, double yaw, double roll, Vector3d velocity, double density)
        {
            ControlSurfaceScratch sc = _ctrlScratch;
            ControlSurfaceSettings st = sc.Settings;
            // a surface flight does not move stays as captured
            if (st == null || !st.Deflects)
                return;
            double desired = 0;
            if (density > 0)
            {
                if (!st.PitchAxis.NearlyEqual(0))
                    desired += sc.PitchLocation * pitch * st.PitchAxis * 0.01;
                if (!st.YawAxis.NearlyEqual(0))
                    desired += sc.YawLocation * yaw * st.YawAxis * 0.01;
                if (!st.RollAxis.NearlyEqual(0))
                    desired += sc.RollLocation * roll * st.RollAxis * 0.01;
                if (!st.BrakeRudder.NearlyEqual(0))
                    desired += sc.BrakeRudderLocation * Math.Max(0.0, sc.BrakeRudderSide * yaw) * st.BrakeRudder * 0.01;
                desired *= st.MaxDeflect;
                if (!st.PitchAxisDueToAoA.NearlyEqual(0) && velocity.magnitude > 5)
                {
                    // the angle the surface would be at undeflected, as AoAOffsetFromControl takes it
                    double aoa = FARMathUtil.rad2deg * base.CalculateAoA(velocity);
                    if (double.IsNaN(aoa))
                        aoa = 0;
                    desired += aoa * st.PitchAxisDueToAoA * 0.01;
                }

                desired *= sc.AoAsign;
                double factor = 1;
                double speed = velocity.magnitude;
                if (density > 0.01 && st.IsDynamicDeflection && speed > 0)
                    factor = Math.Pow(st.DynamicControlStartSpeed / speed, st.Exponent) / (density / st.AtmDensityASL);
                desired *= factor.Clamp(st.MinControl, 1);
                desired = desired.Clamp(-Math.Abs(st.MaxDeflect), Math.Abs(st.MaxDeflect));
            }

            sc.AoAdesiredControl = desired;
            sc.AoAcurrentControl = desired;
            sc.AoAoffset = sc.AoAcurrentFlap + desired;
            sc.lastAoAoffset = sc.AoAoffset;
            double radAoAoffset = sc.AoAoffset * FARMathUtil.deg2rad * st.CtrlSurfFrac;
            Vector3d dn = sc.deflectedNormal;
            dn.y = Math.Sin(radAoAoffset);
            double tmp = 1 - dn.y * dn.y;
            if (tmp < 0)
                tmp = 0;
            dn.z = Math.Sqrt(tmp);
            sc.deflectedNormal = dn;
            if (st.AreaExposedFactor < 0.1 * st.S && !st.TotalExposedAreaFactor.NearlyEqual(0))
                sc.IsShielded = Math.Abs(sc.AoAoffset) <= 5;
        }

        internal override void InstallSnapshotState(in SnapshotState state, long stamp)
        {
            base.InstallSnapshotState(in state, stamp);
            if (state.Control != null)
                _ctrlScratch.CopyControlStateFrom(state.Control);
        }

        internal override void BeginPredictionState()
        {
            base.BeginPredictionState();
            // A snapshot prediction installs the captured control state next; the flight's slot is not read then.
            if (SimThreadContext.SnapshotStamp != 0)
                return;
            ControlSurfaceScratch flight = _ctrlScratchSlots[0];
            ControlSurfaceScratch sc = _ctrlScratch;
            if (flight != null)
                sc.CopyControlStateFrom(flight);
            else
                sc.CopyControlStateFrom(new ControlSurfaceScratch());
            // Flight writes the shield state to the instance (CheckShielded), not to slot 0's scratch.
            sc.IsShielded = isShielded;
        }

        // A control surface also reads the root part's transform and its parent's position;
        // snapshot those alongside the base part transform for the off-thread sweep.
        private FrozenPartTransform? simRootGeom;
        private Vector3 simParentPos;

        public override void SetSimGeometry(bool capture)
        {
            base.SetSimGeometry(capture);
            if (!capture)
            {
                simRootGeom = null;
                return;
            }

            // Each sweep slot starts from the shielding as it is now. A slot's scratch is kept from one sweep to the
            // next, and between sweeps the shielding can change on the instance (a re-voxelization, a deflection on
            // the main thread); a slot seeded once would keep a stale value, and a surface it has as shielded is
            // skipped by the trim and never deflected there again. No worker runs yet (main thread, sweep start).
            for (int s = 1; s <= SimThreadContext.SweepPoolSize; s++)
                if (_ctrlScratchSlots[s] != null)
                    _ctrlScratchSlots[s].IsShielded = isShielded;

            if (EditorLogic.RootPart != null)
                simRootGeom = new FrozenPartTransform(EditorLogic.RootPart.partTransform);
            simParentPos = part.parent != null ? part.parent.partTransform.position : Vector3.zero;
        }

        protected double PitchLocation
        {
            get => _ctrlScratch.PitchLocation;
            set => _ctrlScratch.PitchLocation = value;
        }

        protected double YawLocation
        {
            get => _ctrlScratch.YawLocation;
            set => _ctrlScratch.YawLocation = value;
        }

        protected double RollLocation
        {
            get => _ctrlScratch.RollLocation;
            set => _ctrlScratch.RollLocation = value;
        }

        protected double BrakeRudderLocation
        {
            get => _ctrlScratch.BrakeRudderLocation;
            set => _ctrlScratch.BrakeRudderLocation = value;
        }

        protected double BrakeRudderSide
        {
            get => _ctrlScratch.BrakeRudderSide;
            set => _ctrlScratch.BrakeRudderSide = value;
        }

        protected int flapLocation
        {
            get => _ctrlScratch.flapLocation;
            set => _ctrlScratch.flapLocation = value;
        }

        protected int spoilerLocation
        {
            get => _ctrlScratch.spoilerLocation;
            set => _ctrlScratch.spoilerLocation = value;
        }

        private double AoAsign
        {
            get => _ctrlScratch.AoAsign;
            set => _ctrlScratch.AoAsign = value;
        }

        //DaMichel: treat desired AoA's from flap and stick inputs separately for different animation rates
        private double AoAdesiredControl
        {
            get => _ctrlScratch.AoAdesiredControl;
            set => _ctrlScratch.AoAdesiredControl = value;
        }

        private double AoAdesiredFlap
        {
            get => _ctrlScratch.AoAdesiredFlap;
            set => _ctrlScratch.AoAdesiredFlap = value;
        }

        private double AoAcurrentControl // current deflection due to control inputs
        {
            get => _ctrlScratch.AoAcurrentControl;
            set => _ctrlScratch.AoAcurrentControl = value;
        }

        private double AoAcurrentFlap // current deflection due to flap/spoiler deployment
        {
            get => _ctrlScratch.AoAcurrentFlap;
            set => _ctrlScratch.AoAcurrentFlap = value;
        }

        private double AoAoffset // total current deflection
        {
            get => _ctrlScratch.AoAoffset;
            set => _ctrlScratch.AoAoffset = value;
        }

        private double lastAoAoffset
        {
            get => _ctrlScratch.lastAoAoffset;
            set => _ctrlScratch.lastAoAoffset = value;
        }

        private Vector3d deflectedNormal
        {
            get => _ctrlScratch.deflectedNormal;
            set => _ctrlScratch.deflectedNormal = value;
        }
        public bool brake;
        private bool justStarted;

        private Transform lastReferenceTransform;
        private bool animated;

        public static double timeConstant
        {
            get { return FARAeroData.ControlSurfaceTimeConstant; }
            set { FARAeroData.ControlSurfaceTimeConstant = value; }
        }

        public static double timeConstantFlap
        {
            get { return FARAeroData.ControlSurfaceTimeConstantFlap; }
            set { FARAeroData.ControlSurfaceTimeConstantFlap = value; }
        }

        public static double timeConstantSpoiler
        {
            get { return FARAeroData.ControlSurfaceTimeConstantSpoiler; }
            set { FARAeroData.ControlSurfaceTimeConstantSpoiler = value; }
        }

        // TODO 1.2: ITorqueProvider now reports two Vector3s, positive torque(that produced by control actuation 1,1,1) and negative torque(that produced by -1,-1,-1).
        public void GetPotentialTorque(out Vector3 pos, out Vector3 neg)
        {
            //get max lift coeff
            Vector3 maxLiftVec = FinalLiftSlope * GetLiftDirection() * maxdeflect * Math.PI / 180;
            //get an actual lift vector out of it
            maxLiftVec *= (float)(vessel.dynamicPressurekPa * S);

            Vector3 relPosVector = AerodynamicCenter - vessel.CoM;

            var maxMomentVector = Vector3.Cross(relPosVector, maxLiftVec);

            Vector3 vesselRelMaxMoment = vessel.ReferenceTransform.worldToLocalMatrix.MultiplyVector(maxMomentVector);

            Vector3 resultVector = Vector3.zero;
            resultVector.x = (float)Math.Abs(vesselRelMaxMoment.x * pitchaxis * PitchLocation * 0.01);
            resultVector.z = (float)Math.Abs(vesselRelMaxMoment.z * yawaxis * YawLocation * 0.01);
            resultVector.y = (float)Math.Abs(vesselRelMaxMoment.y * rollaxis * RollLocation * 0.01);

            pos = resultVector;
            neg = -resultVector;
        }


        [FARAction("FARCtrlActionSpoiler", FARActionGroupConfiguration.ID_SPOILER)]
        public void ActivateSpoiler(KSPActionParam param)
        {
            brake = !(param.type > 0);
        }

        [FARAction("FARCtrlActionIncFlap", FARActionGroupConfiguration.ID_INCREASE_FLAP_DEFLECTION)]
        public void IncreaseDeflect(KSPActionParam param)
        {
            param.Cooldown = 0.25f;
            SetDeflection(flapDeflectionLevel + 1);
        }

        [KSPEvent(name = "DeflectMore", active = false, guiActive = true, guiName = "FARCtrlEventIncFlap")]
        public void DeflectMore()
        {
            SetDeflection(flapDeflectionLevel + 1);
            UpdateFlapDeflect();
        }

        [FARAction("FARCtrlActionDecFlap", FARActionGroupConfiguration.ID_DECREASE_FLAP_DEFLECTION)]
        public void DecreaseDeflect(KSPActionParam param)
        {
            param.Cooldown = 0.25f;
            SetDeflection(flapDeflectionLevel - 1);
        }

        [KSPEvent(name = "DeflectLess", active = false, guiActive = true, guiName = "FARCtrlEventDecFlap")]
        public void DeflectLess()
        {
            SetDeflection(flapDeflectionLevel - 1);
            UpdateFlapDeflect();
        }

        private void UpdateFlapDeflect()
        {
            foreach (Part p in part.symmetryCounterparts)
            {
                foreach (PartModule m in p.Modules)
                    if (m is FARControllableSurface controllableSurface)
                        controllableSurface.SetDeflection(flapDeflectionLevel);
            }
        }

        private void CheckFieldVisibility()
        {
            if (showStdCtrl != prevStdCtrl)
            {
                Fields["pitchaxis"].guiActiveEditor = showStdCtrl;
                Fields["yawaxis"].guiActiveEditor = showStdCtrl;
                Fields["rollaxis"].guiActiveEditor = showStdCtrl;
                Fields["pitchaxisDueToAoA"].guiActiveEditor = showStdCtrl;
                Fields["brakeRudder"].guiActiveEditor = showStdCtrl;
                Fields["maxdeflect"].guiActiveEditor = showStdCtrl;

                Fields["pitchaxis"].guiActive = showStdCtrl;
                Fields["yawaxis"].guiActive = showStdCtrl;
                Fields["rollaxis"].guiActive = showStdCtrl;
                Fields["pitchaxisDueToAoA"].guiActive = showStdCtrl;
                Fields["brakeRudder"].guiActive = showStdCtrl;
                Fields["maxdeflect"].guiActive = showStdCtrl;
                prevStdCtrl = showStdCtrl;
            }

            if (showDynamicDeflection != isPrevDynamicDeflection)
            {
                Fields["dynamicControlStartSpeed"].guiActiveEditor = showDynamicDeflection;
                Fields["exponent"].guiActiveEditor = showDynamicDeflection;
                Fields["minControl"].guiActiveEditor = showDynamicDeflection;
                Fields["isDynamicDeflection"].guiActiveEditor = showDynamicDeflection;

                Fields["dynamicControlStartSpeed"].guiActive = showDynamicDeflection;
                Fields["exponent"].guiActive = showDynamicDeflection;
                Fields["minControl"].guiActive = showDynamicDeflection;
                Fields["isDynamicDeflection"].guiActive = showDynamicDeflection;
                isPrevDynamicDeflection = showDynamicDeflection;
            }

            if (showFlpCtrl != prevFlpCtrl)
            {
                Fields["isFlap"].guiActiveEditor = showFlpCtrl;
                Fields["isSpoiler"].guiActiveEditor = showFlpCtrl;
                Fields["maxdeflectFlap"].guiActiveEditor = showFlpCtrl;

                Fields["isFlap"].guiActive = showFlpCtrl;
                Fields["isSpoiler"].guiActive = showFlpCtrl;
                Fields["maxdeflectFlap"].guiActive = showFlpCtrl;
                prevFlpCtrl = showFlpCtrl;
            }

            if (isFlap != prevIsFlap)
            {
                prevIsFlap = isFlap;
                isSpoiler = false;
                prevIsSpoiler = false;
                UpdateEvents();
            }

            // ReSharper disable once InvertIf
            if (isSpoiler != prevIsSpoiler)
            {
                prevIsSpoiler = isSpoiler;
                isFlap = false;
                prevIsFlap = false;
                UpdateEvents();
            }
        }

        public void SetDeflection(int newstate)
        {
            flapDeflectionLevel = Math.Max(0, Math.Min(3, newstate));
            UpdateEvents();
        }

        public void UpdateEvents()
        {
            Fields["flapDeflectionLevel"].guiActive = isFlap;
            Events["DeflectMore"].active = isFlap && flapDeflectionLevel < 3;
            Events["DeflectLess"].active = isFlap && flapDeflectionLevel > 0;
            if (!isFlap)
                flapDeflectionLevel = 0;
        }

        public override void Initialization()
        {
            base.Initialization();
            if (part.Modules.GetModule<ModuleControlSurface>())
                part.RemoveModule(part.Modules.GetModule<ModuleControlSurface>());

            OnVesselPartsChange += CalculateSurfaceFunctions;
            UpdateEvents();
            prevIsFlap = isFlap;
            prevIsSpoiler = isSpoiler;
            if (!isFlap)
                flapDeflectionLevel = 0;


            justStarted = true;
            if (vessel)
                lastReferenceTransform = vessel.ReferenceTransform;

            if (!FARDebugValues.allowStructuralFailures)
                return;
            foreach (FARPartStressTemplate temp in FARAeroStress.StressTemplates)
                if (temp.Name == "ctrlSurfStress")
                {
                    FARPartStressTemplate template = temp;
                    double maxForceMult = Math.Pow(massMultiplier, FARAeroUtil.massStressPower);

                    YmaxForce *= 1 - ctrlSurfFrac;
                    XZmaxForce *= 1 - ctrlSurfFrac;

                    double tmp = template.YMaxStress; //in MPa
                    tmp *= S * ctrlSurfFrac * maxForceMult;
                    YmaxForce += tmp;

                    tmp = template.XZMaxStress; //in MPa
                    tmp *= S * ctrlSurfFrac * maxForceMult;
                    XZmaxForce += tmp;
                    break;
                }
        }

        public override void OnStartFinished(StartState state)
        {
            base.OnStartFinished(state);

            UpdateMovableSection();
        }

        private void UpdateMovableSection()
        {
            movableSection = part.FindModelTransform(transformName); //And the transform
            if (movableSection is null)
            {
                FARLogger.WarningFormat("No valid control surface model with transform name {0} found on {1}, animations disabled",
                                        transformName,
                                        part?.partName);
                animated = false;
                return;
            }

            if (!MovableOrigReady)
            {
                // In parts copied by symmetry, these fields should already be set,
                // while the transform may not be in the original orientation anymore.
                MovableOrig = movableSection.localRotation; //Its original orientation
                MovableOrigReady = true;
            }

            flipAxis = Vector3.Dot(movableSection.right, part.partTransform.right) <= 0;

            animated = HighLogic.LoadedSceneIsFlight && part is not null && vessel is not null;
        }

        public override void FixedUpdate()
        {
            if (justStarted)
            {
                UpdateShipPartsList();
                CalculateSurfaceFunctions();
            }

            if (animated)
            {
                // make sure the transform is still alive
                if (movableSection == null)
                {
                    MovableOrigReady = false;
                    UpdateMovableSection();
                }

                bool process = part.isControllable || justStarted && isFlap;

                if (movableSection is not null && process && part.Rigidbody)
                {
                    // Set member vars for desired AoA
                    if (isSpoiler)
                        AoAOffsetFromSpoilerDeflection();
                    else
                        AoAOffsetFromFlapDeflection();
                    AoAOffsetFromControl();
                    //DaMichel: put deflection change here so that AoAOffsetFromControlInput does only the thing which the name suggests
                    ChangeDeflection();
                    DeflectionAnimation();
                }
            }

            CheckFieldVisibility();

            base.FixedUpdate();
            justStarted = false;

            if (!vessel || vessel.ReferenceTransform == lastReferenceTransform)
                return;
            justStarted = true;
            lastReferenceTransform = vessel.ReferenceTransform;
        }

        private void CheckShielded()
        {
            if (!(NUFAR_areaExposedFactor < 0.1 * S) || NUFAR_totalExposedAreaFactor.NearlyEqual(0))
                return;

            // A sweep worker or a prediction writes its slot's value (the one GetShielded reads there); the
            // instance field is the flight's and the main thread's, shared with every other thread.
            if (SlotShielding)
                _ctrlScratch.IsShielded = Math.Abs(AoAoffset) <= 5;
            else
                isShielded = Math.Abs(AoAoffset) <= 5;
        }

        // Whether the shield state lives in the slot's scratch: on a sweep worker and inside a prediction.
        private bool SlotShielding
        {
            get { return SweepGeom.HasValue || SimThreadContext.Prediction != null; }
        }

        public override bool GetShielded()
        {
            return SlotShielding ? _ctrlScratch.IsShielded : isShielded;
        }

        public void CalculateSurfaceFunctions()
        {
            if (HighLogic.LoadedSceneIsEditor && (!FlightGlobals.ready || vessel is null || part.partTransform is null))
                return;

            // use caching to improve performance since all vector properties call into native code
            Vector3 partForward = part.partTransform.forward;
            Vector3 partPosition = part.partTransform.position;
            Vector3 forward, right, up;
            if (HighLogic.LoadedSceneIsFlight)
            {
                forward = vessel.ReferenceTransform.forward;
                right = vessel.ReferenceTransform.right;
                up = vessel.ReferenceTransform.up;
            }
            else
            {
                forward = EditorLogic.RootPart.partTransform.forward;
                right = EditorLogic.RootPart.partTransform.right;
                up = EditorLogic.RootPart.partTransform.up;
            }

            if (part.symMethod == SymmetryMethod.Mirror || part.symmetryCounterparts.Count < 1)
            {
                flapLocation = Math.Sign(Vector3.Dot(forward, partForward));

                spoilerLocation = -flapLocation;
            }
            else if (part.parent != null)
            {
                flapLocation = Math.Sign(Vector3.Dot(partPosition - part.parent.partTransform.position, partForward));
                spoilerLocation = flapLocation;
            }
            else
            {
                flapLocation = 1;
                spoilerLocation = flapLocation;
            }

            Vector3 CoM = Vector3.zero;
            float mass = 0;
            foreach (Part p in VesselPartList)
            {
                CoM += p.transform.position * p.mass;
                mass += p.mass;
            }

            CoM /= mass;

            if (HighLogic.LoadedSceneIsEditor && (isFlap || isSpoiler))
                SetControlStateEditor(CoM, part.partTransform.up, 0, 0, 0, 0, false);

            Vector3 CoMoffset = (partPosition - CoM).normalized;
            Vector3 partUp = part.partTransform.up;
            PitchLocation = Vector3.Dot(partForward, forward) * Math.Sign(Vector3.Dot(CoMoffset, up));
            YawLocation = -Vector3.Dot(partForward, right) * Math.Sign(Vector3.Dot(CoMoffset, up));
            RollLocation = Vector3.Dot(partForward, forward) * Math.Sign(Vector3.Dot(CoMoffset, -right));
            float roll2 = Vector3.Dot(partForward, right) * Math.Sign(Vector3.Dot(CoMoffset, forward));
            BrakeRudderLocation = Vector3.Dot(partForward, forward);
            BrakeRudderSide = Math.Sign(Vector3.Dot(CoMoffset, right));
            AoAsign = Math.Sign(Vector3.Dot(partUp, up));
            RollLocation += roll2;

            //DaMichel: this is important to force a reset of the flap/spoiler model orientation to the desired value.
            // What apparently happens on loading a new flight scene is that first the model (obj_ctrlSrf)
            // orientation is set correctly by DeflectionAnimation(). But then the orientations is mysteriously
            // zeroed-out. And this definitely doesn't happen in this module. However OnVesselPartsChange
            // subscribers are called afterwards, so we have a chance to fix the broken orientation state.
            lastAoAoffset = double.MaxValue;
        }

        private void AoAOffsetFromSpoilerDeflection()
        {
            if (brake)
                AoAdesiredFlap = maxdeflectFlap * spoilerLocation;
            else
                AoAdesiredFlap = 0;
            AoAdesiredFlap = AoAdesiredFlap.Clamp(-Math.Abs(maxdeflectFlap), Math.Abs(maxdeflectFlap));
        }


        private void AoAOffsetFromFlapDeflection()
        {
            AoAdesiredFlap = maxdeflectFlap * flapLocation * flapDeflectionLevel * 0.33333333333;
            AoAdesiredFlap = AoAdesiredFlap.Clamp(-Math.Abs(maxdeflectFlap), Math.Abs(maxdeflectFlap));
        }

        private void AoAOffsetFromControl()
        {
            AoAdesiredControl = 0;
            if (vessel is null || vessel.atmDensity <= 0)
                return;
            if (!pitchaxis.NearlyEqual(0))
                AoAdesiredControl += PitchLocation * vessel.ctrlState.pitch * pitchaxis * 0.01;
            if (!yawaxis.NearlyEqual(0))
                AoAdesiredControl += YawLocation * vessel.ctrlState.yaw * yawaxis * 0.01;
            if (!rollaxis.NearlyEqual(0))
                AoAdesiredControl += RollLocation * vessel.ctrlState.roll * rollaxis * 0.01;
            if (!brakeRudder.NearlyEqual(0))
                AoAdesiredControl += BrakeRudderLocation *
                                     Math.Max(0.0, BrakeRudderSide * vessel.ctrlState.yaw) *
                                     brakeRudder *
                                     0.01;
            AoAdesiredControl *= maxdeflect;
            if (!pitchaxisDueToAoA.NearlyEqual(0))
            {
                Vector3d vel = GetVelocity();
                double velMag = vel.magnitude;
                if (velMag > 5)
                {
                    //using base.CalculateAoA gets the deflection using WingAeroModel's code, which does not account for deflection; this gives us the AoA that the surface _would_ be at if it hadn't deflected at all.
                    double AoA = base.CalculateAoA(vel);
                    AoA = FARMathUtil.rad2deg * AoA;
                    if (double.IsNaN(AoA))
                        AoA = 0;
                    AoAdesiredControl += AoA * pitchaxisDueToAoA * 0.01;
                }
            }

            AoAdesiredControl *= AoAsign;
            AoAdesiredControl *= CalculateDynamicControlFactor();
            AoAdesiredControl = AoAdesiredControl.Clamp(-Math.Abs(maxdeflect), Math.Abs(maxdeflect));
        }

        private double CalculateDynamicControlFactor()
        {
            double factor = 1;
            if (vessel.atmDensity > 0.01 && isDynamicDeflection && vessel.srfSpeed > 0)
            {
                factor = Math.Pow(dynamicControlStartSpeed / vessel.srfSpeed, exponent) /
                         (vessel.atmDensity / vessel.lastBody.atmDensityASL);
            }

            return factor.Clamp(minControl, 1);
        }

        public override double CalculateAoA(Vector3d velocity)
        {
            // Use the vector computed by DeflectionAnimation
            Vector3d perp;
            if (SimThreadContext.LiveOnly)
                perp = part_transform.TransformDirection(deflectedNormal);
            else
            {
                FrozenPartTransform? g = SolveGeom;
                perp = g.HasValue
                           ? g.Value.TransformDirection(deflectedNormal)
                           : part_transform.TransformDirection(deflectedNormal);
            }
            double PerpVelocity = Vector3d.Dot(perp, velocity.normalized);
            return Math.Asin(PerpVelocity.Clamp(-1, 1));
        }

        // Had to add this one since the parent class don't use AoAoffset and adding it would break GetWingInFrontOf
        // ReSharper disable once UnusedMember.Global
        public double CalculateAoA(Vector3d velocity, double offset)
        {
            double radAoAoffset = offset * FARMathUtil.deg2rad * ctrlSurfFrac;
            Vector3 perp =
                part_transform.TransformDirection(new Vector3d(0, Math.Sin(radAoAoffset), Math.Cos(radAoAoffset)));
            double PerpVelocity = Vector3d.Dot(perp, velocity.normalized);
            return Math.Asin(PerpVelocity.Clamp(-1, 1));
        }

        //DaMichel: Factored the time evolution for deflection AoA into this function. This one results into an exponential asymptotic
        //"decay" towards the desired value. Good for stick inputs, i suppose, and the original method.
        private static double BlendDeflectionExp(
            double current,
            double desired,
            double blendTimeConstant,
            bool forceSetToDesired
        )
        {
            double error = desired - current;
            // DaMichel: i changed the threshold since i noticed a "bump" at max deflection
            if (!forceSetToDesired && Math.Abs(error) >= 0.01)
            {
                double tmp1 = error / blendTimeConstant;
                current += (TimeWarp.fixedDeltaTime * tmp1).Clamp(-Math.Abs(0.6 * error), Math.Abs(0.6 * error));
            }
            else
            {
                current = desired;
            }

            return current;
        }

        //DaMichel: Similarly, this is used for constant rate movement towards the desired value. I presume it is more realistic for
        //for slow moving flaps and spoilers. It looks better anyways.
        //ferram4: The time constant specifies the time it would take for a first-order system to reach its steady-state value,
        //assuming that it was proportional to only the initial error, not the error as a function of time
        private static double BlendDeflectionLinear(
            double current,
            double desired,
            double maximumDeflection,
            double blendTimeConstant,
            bool forceSetToDesired
        )
        {
            double error = desired - current;
            if (!forceSetToDesired && Math.Abs(error) >= 0.1)
            {
                double degreesPerSecond = Math.Max(Math.Abs(maximumDeflection), Math.Abs(current)) / blendTimeConstant;
                double tmp = current + TimeWarp.fixedDeltaTime * degreesPerSecond * Math.Sign(desired - current);
                current = error > 0 ? tmp.Clamp(current, desired) : tmp.Clamp(desired, current);
            }
            else
            {
                return desired;
            }

            return current;
        }

        // Determines current deflection contributions from stick and flap/spoiler settings and update current total deflection (AoAoffset).
        private void ChangeDeflection()
        {
            if (!AoAcurrentControl.NearlyEqual(AoAdesiredControl))
                AoAcurrentControl = BlendDeflectionExp(AoAcurrentControl, AoAdesiredControl, timeConstant, justStarted);

            if (!AoAcurrentFlap.NearlyEqual(AoAdesiredFlap))
                AoAcurrentFlap = BlendDeflectionLinear(AoAcurrentFlap,
                                                       AoAdesiredFlap,
                                                       maxdeflectFlap,
                                                       isSpoiler ? timeConstantSpoiler : timeConstantFlap,
                                                       justStarted);
            AoAoffset = AoAcurrentFlap + AoAcurrentControl;
        }

        /// <summary>
        ///     This animates a deflection based on AoAoffset
        /// </summary>
        protected void DeflectionAnimation()
        {
            // Don't recalculate on insignificant variations
            if (Math.Abs(lastAoAoffset - AoAoffset) < 0.01)
                return;

            lastAoAoffset = AoAoffset;

            // Compute a vector for CalculateAoA. deflectedNormal is a property (thread-local
            // scratch), so its components are updated through a local rather than in place.
            double radAoAoffset = AoAoffset * FARMathUtil.deg2rad * ctrlSurfFrac;
            Vector3d dn = deflectedNormal;
            dn.y = Math.Sin(radAoAoffset);
            double tmp = 1 - dn.y * dn.y;
            if (tmp < 0)
                tmp = 0;
            dn.z = Math.Sqrt(tmp);
            deflectedNormal = dn;

            // Visually animate the surface. Skipped on a sweep worker, off the main thread; the
            // animation mutates a Unity Transform and is cosmetic.
            if (!SweepGeom.HasValue)
            {
                movableSection.localRotation = MovableOrig;
                if (!AoAoffset.NearlyEqual(0))
                {
                    Quaternion localRot = flipAxis
                                              ? Quaternion.FromToRotation(deflectedNormal, new Vector3(0, 0, 1))
                                              : Quaternion.FromToRotation(new Vector3(0, 0, 1), deflectedNormal);

                    movableSection.localRotation *= localRot;
                }
            }

            CheckShielded();
        }

        public void SetControlStateEditor(
            Vector3 CoM,
            Vector3 velocityVec,
            float pitch,
            float yaw,
            float roll,
            int flap,
            bool braking
        )
        {
            if (!HighLogic.LoadedSceneIsEditor)
                return;

            // Transform reads come from the frozen snapshot on a sweep worker (off the main
            // thread) and from the live transforms otherwise.
            Vector3 partPosition, partForward, partUp, forward, up, right;
            FrozenPartTransform? sweepGeom = SweepGeom;
            if (sweepGeom.HasValue)
            {
                partPosition = sweepGeom.Value.Position;
                partForward = sweepGeom.Value.Forward;
                partUp = sweepGeom.Value.Up;
                forward = simRootGeom.Value.Forward;
                up = simRootGeom.Value.Up;
                right = simRootGeom.Value.Right;
            }
            else
            {
                Transform partTransform = part.partTransform;
                Transform rootTransform = EditorLogic.RootPart.partTransform;
                partPosition = partTransform.position;
                partForward = partTransform.forward;
                partUp = partTransform.up;
                forward = rootTransform.forward;
                up = rootTransform.up;
                right = rootTransform.right;
            }

            Vector3 CoMoffset = partPosition - CoM;

            PitchLocation = Vector3.Dot(partForward, forward) * Math.Sign(Vector3.Dot(CoMoffset, up));
            YawLocation = -Vector3.Dot(partForward, right) * Math.Sign(Vector3.Dot(CoMoffset, up));
            RollLocation = Vector3.Dot(partForward, forward) * Math.Sign(Vector3.Dot(CoMoffset, -right));
            BrakeRudderLocation = Vector3.Dot(partForward, forward);
            BrakeRudderSide = Mathf.Sign(Vector3.Dot(CoMoffset, right));
            AoAsign = Math.Sign(Vector3.Dot(partUp, up));
            AoAdesiredControl = 0;
            if (!pitchaxis.NearlyEqual(0))
                AoAdesiredControl += PitchLocation * pitch * pitchaxis * 0.01;
            if (!yawaxis.NearlyEqual(0))
                AoAdesiredControl += YawLocation * yaw * yawaxis * 0.01;
            if (!rollaxis.NearlyEqual(0))
                AoAdesiredControl += RollLocation * roll * rollaxis * 0.01;
            if (!brakeRudder.NearlyEqual(0))
                AoAdesiredControl +=
                    BrakeRudderLocation * Math.Max(0.0, BrakeRudderSide * yawaxis) * brakeRudder * 0.01;
            AoAdesiredControl *= maxdeflect;
            if (!pitchaxisDueToAoA.NearlyEqual(0))
            {
                //velocity vector projected onto a plane that divides the airplane into left and right halves
                Vector3 tmpVec = up * Vector3.Dot(up, velocityVec) + forward * Vector3.Dot(forward, velocityVec);
                //using base.CalculateAoA gets the deflection using WingAeroModel's code, which does not account for deflection; this gives us the AoA that the surface _would_ be at if it hadn't deflected at all.
                double AoA = base.CalculateAoA(tmpVec.normalized);
                AoA = FARMathUtil.rad2deg * AoA;
                if (double.IsNaN(AoA))
                    AoA = 0;
                AoAdesiredControl += AoA * pitchaxisDueToAoA * 0.01;
            }

            AoAdesiredControl *= AoAsign;
            AoAdesiredControl = AoAdesiredControl.Clamp(-Math.Abs(maxdeflect), Math.Abs(maxdeflect));
            AoAcurrentControl = AoAdesiredControl;
            AoAcurrentFlap = 0;

            if (part.symMethod == SymmetryMethod.Mirror || part.symmetryCounterparts.Count < 1)
            {
                flapLocation =
                    Math.Sign(Vector3.Dot(HighLogic.LoadedSceneIsFlight
                                              ? vessel.ReferenceTransform.forward
                                              : forward,
                                          partForward));

                spoilerLocation = -flapLocation;
            }
            else if (part.parent != null)
            {
                Vector3 parentPos = sweepGeom.HasValue ? simParentPos : part.parent.partTransform.position;
                flapLocation = Math.Sign(Vector3.Dot(partPosition - parentPos, partForward));
                spoilerLocation = flapLocation;
            }
            else
            {
                flapLocation = 1;
                spoilerLocation = flapLocation;
            }

            if (isFlap)
                AoAcurrentFlap += maxdeflectFlap * flapLocation * flap * 0.3333333333333;
            else if (isSpoiler)
                AoAcurrentFlap += braking ? maxdeflectFlap * spoilerLocation : 0;

            AoAdesiredFlap = AoAcurrentFlap;
            AoAoffset = AoAcurrentFlap + AoAcurrentControl;
            DeflectionAnimation();
        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            bool tmpBool;
            if (node.HasValue("pitchaxis"))
                if (bool.TryParse(node.GetValue("pitchaxis"), out tmpBool))
                    pitchaxis = tmpBool ? 100 : 0;
            if (node.HasValue("yawaxis"))
                if (bool.TryParse(node.GetValue("yawaxis"), out tmpBool))
                    yawaxis = tmpBool ? 100 : 0;
            // ReSharper disable once InvertIf
            if (node.HasValue("rollaxis"))
                if (bool.TryParse(node.GetValue("rollaxis"), out tmpBool))
                    rollaxis = tmpBool ? 100 : 0;
        }

        //For some reason, all the UIRange values are saved in the config files, and there is no way to prevent that
        //This makes the options limited for people loading old crafts with new FAR
        //This resets the values to what they should be
        // ReSharper disable once UnusedMember.Local
        private void FixAllUIRanges()
        {
            FixWrongUIRange("pitchaxis", 100, -100);
            FixWrongUIRange("yawaxis", 100, -100);
            FixWrongUIRange("rollaxis", 100, -100);
            FixWrongUIRange("brakeRudder", 100, -100);
            FixWrongUIRange("maxdeflect", 40, -40);
            FixWrongUIRange("maxdeflectFlap", 85, -85);
        }

        private void FixWrongUIRange(string field, float upperRange, float lowerRange)
        {
            var tmpUI = (UI_FloatRange)Fields[field].uiControlEditor;
            tmpUI.maxValue = upperRange;
            tmpUI.minValue = lowerRange;
        }
    }
}
