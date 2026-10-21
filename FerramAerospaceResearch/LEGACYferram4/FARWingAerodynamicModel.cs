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
using FerramAerospaceResearch;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.Settings;
using KSP.Localization;
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
        public double rawAoAmax = 15;
        private double AoAmax = 15;

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

        private double minStall;

        private double piARe = 1; //induced drag factor

        [KSPField(isPersistant = false)] public double b_2; //span

        public double b_2_actual; //span

        [KSPField(isPersistant = false)] public double MidChordSweep;

        private double MidChordSweepSideways;

        private double cosSweepAngle;

        private double effective_b_2 = 1;
        private double effective_MAC = 1;

        protected double effective_AR = 4;
        protected double transformed_AR = 4;

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

        private double rawLiftSlope;
        private double liftslope;
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
        internal double EffectiveAR => effective_AR;
        internal double OwnAR => transformed_AR;
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
            FARAeroPartModule a = part.Modules.GetModule<FARAeroPartModule>();

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
                                                    ? p.Modules.GetModule<FARControllableSurface>()
                                                    : p.Modules.GetModule<FARWingAerodynamicModel>();

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
                                                    ? p.Modules.GetModule<FARControllableSurface>()
                                                    : p.Modules.GetModule<FARWingAerodynamicModel>();

                model.NUFAR_areaExposedFactor = sum;
                model.NUFAR_totalExposedAreaFactor = totalExposedSum;
            }
        }

        public void NUFAR_UpdateShieldingStateFromAreaFactor()
        {
            isShielded = NUFAR_areaExposedFactor < 0.1 * S;
            FARWingInteraction.InvalidateWingGroups();
        }

        public double GetStall()
        {
            return stall;
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
            return AerodynamicCenter;
        }

        public double GetMAC()
        {
            return effective_MAC;
        }

        public double Getb_2()
        {
            return effective_b_2;
        }

        public Vector3d GetLiftDirection()
        {
            return liftDirection;
        }

        public double GetRawLiftSlope()
        {
            return rawLiftSlope;
        }

        public double GetCosSweepAngle()
        {
            return cosSweepAngle;
        }

        public double GetCd0()
        {
            return zeroLiftCdIncrement;
        }

        public Vector3d ComputeForceEditor(Vector3d velocityVector, double M, double density)
        {
            velocityEditor = velocityVector;

            rho = density;

            double AoA = CalculateAoA(velocityVector);
            return CalculateForces(velocityVector, M, AoA, density);
        }

        public void ComputeClCdEditor(Vector3d velocityVector, double M, double density)
        {
            velocityEditor = velocityVector;

            rho = density;

            double AoA = CalculateAoA(velocityVector);
            CalculateForces(velocityVector, M, AoA, density);
        }

        protected override void ResetCenterOfLift()
        {
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
            Cl = 0;
            Cd = 0;
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
                    AC_offset = effective_MAC * 0.25 * ParallelInPlane;
                }
                else if (MachNumber > 1.4)
                {
                    AC_offset = effective_MAC * 0.10 * ParallelInPlane;
                }
                else if (MachNumber >= 1)
                {
                    AC_offset = effective_MAC * (-0.375 * MachNumber + 0.625) * ParallelInPlane;
                }
                //This is for the transonic instability, which is lessened for highly swept wings
                else
                {
                    double sweepFactor = cosSweepAngle * cosSweepAngle * tmp;
                    if (MachNumber < 0.9)
                        AC_offset = effective_MAC * ((MachNumber - 0.85) * 2 * sweepFactor + 0.25) * ParallelInPlane;
                    else
                        AC_offset = effective_MAC * ((1 - MachNumber) * sweepFactor + 0.25) * ParallelInPlane;
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

        public void StartInitialization()
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

        public void MathAndFunctionInitialization()
        {
            S = b_2_actual * MAC_actual;

            if (part.srfAttachNode.originalOrientation.x < 0)
                srfAttachNegative = -1;

            transformed_AR = b_2_actual / MAC_actual;

            MidChordSweepSideways = (1 - TaperRatio) / (1 + TaperRatio);

            MidChordSweepSideways =
                (Math.PI * 0.5 -
                 Math.Atan(Math.Tan(MidChordSweep * FARMathUtil.deg2rad) +
                           MidChordSweepSideways * 4 / transformed_AR)) *
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
            if (VesselPartList == null)
                VesselPartList = GetShipPartList();
            if (wingInteraction == null)
                wingInteraction = new FARWingInteraction(this, part, rootMidChordOffsetFromOrig, srfAttachNegative);

            wingInteraction.UpdateWingInteraction(VesselPartList, nonSideAttach == 1);
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
                    CurWingCentroid = WingCentroid();

                    Vector3d velocity = rb.GetPointVelocity(CurWingCentroid) +
                                        Krakensbane.GetFrameVelocity() -
                                        FARAtmosphere.GetWind(FlightGlobals.currentMainBody, part, rb.position);

                    double v_scalar = velocity.magnitude;

                    if (partVessel.mainBody.ocean)
                        rho = partVessel.mainBody.oceanDensity * 1000 * part.submergedPortion +
                              part.atmDensity * (1 - part.submergedPortion);
                    else
                        rho = part.atmDensity;

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
                                float waterFraction = (float)(part.submergedDynamicPressurekPa * part.submergedPortion);
                                waterFraction /= (float)rho;

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
                                string msg = string.Format(Localizer.Format("FARFlightLogAeroFailure"),
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
                        stall = 0;
                        wingInteraction.ResetWingInteractions();
                    }
                }
                else
                {
                    stall = 0;
                    wingInteraction.ResetWingInteractions();
                }
            }
            else
            {
                if (isShielded)
                    Cl = Cd = Cm = stall = 0;
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
            bool updateAeroArrows = true
        )
        {
            CurWingCentroid = WingCentroid();

            return DoCalculateForces(velocity, MachNumber, AoA, density, 1, updateAeroArrows);
        }

        public Vector3d CalculateForces(
            Vector3d velocity,
            double MachNumber,
            double AoA,
            double density,
            double failureForceScaling,
            bool updateAeroArrows = true
        )
        {
            CurWingCentroid = WingCentroid();

            return DoCalculateForces(velocity, MachNumber, AoA, density, failureForceScaling, updateAeroArrows);
        }

        private Vector3d DoCalculateForces(
            Vector3d velocity,
            double MachNumber,
            double AoA,
            double density,
            double failureForceScaling,
            bool updateAeroArrows = true
        )
        {
            double v_scalar = velocity.magnitude;

            Vector3 forward = part_transform.forward;
            Vector3d velocity_normalized = velocity / v_scalar;

            double q = density * v_scalar * v_scalar * 0.0005; //dynamic pressure, q

            //Projection of velocity vector onto the plane of the wing
            ParallelInPlane = Vector3d.Exclude(forward, velocity).normalized;
            //This just gives the vector to cross with the velocity vector
            perp = Vector3d.Cross(forward, ParallelInPlane).normalized;
            liftDirection = Vector3d.Cross(perp, velocity).normalized;

            ParallelInPlaneLocal = part_transform.InverseTransformDirection(ParallelInPlane);

            // Calculate the adjusted AC position (uses ParallelInPlane)
            AerodynamicCenter = CalculateAerodynamicCenter(MachNumber, AoA, CurWingCentroid);

            //Throw AoA into lifting line theory and adjust for part exposure and compressibility effects

            double skinFrictionDrag = HighLogic.LoadedSceneIsFlight
                                          ? FARAeroUtil.SkinFrictionDrag(density,
                                                                         effective_MAC,
                                                                         v_scalar,
                                                                         MachNumber,
                                                                         vessel.externalTemperature,
                                                                         FARAtmosphere.GetAdiabaticIndex(vessel))
                                          : 0.005;


            skinFrictionDrag *= 1.1; //account for thickness

            CalculateCoefficients(MachNumber, AoA, skinFrictionDrag);


            //lift and drag vectors
            Vector3d L, D;
            if (failureForceScaling >= 1 && part.submergedPortion > 0)
            {
                //lift; submergedDynPreskPa handles lift
                L = liftDirection *
                    (Cl * S) *
                    q *
                    (part.submergedPortion * part.submergedLiftScalar + 1 - part.submergedPortion);
                //drag is parallel to velocity vector
                D = -velocity_normalized *
                    (Cd * S) *
                    q *
                    (part.submergedPortion * part.submergedDragScalar + 1 - part.submergedPortion);
            }
            else
            {
                //lift; submergedDynPreskPa handles lift
                L = liftDirection * (Cl * S) * q;
                //drag is parallel to velocity vector
                D = -velocity_normalized * (Cd * S) * q;
            }

            if (updateAeroArrows)
                UpdateAeroDisplay(L, D);

            Vector3d force = L + D;
            if (double.IsNaN(force.sqrMagnitude) || double.IsNaN(AerodynamicCenter.sqrMagnitude))
            {
                FARLogger.Warning("Error: Aerodynamic force = " +
                                  force.magnitude +
                                  " AC Loc = " +
                                  AerodynamicCenter.magnitude +
                                  " AoA = " +
                                  AoA +
                                  "\n\rMAC = " +
                                  effective_MAC +
                                  " B_2 = " +
                                  effective_b_2 +
                                  " sweepAngle = " +
                                  cosSweepAngle +
                                  "\n\rMidChordSweep = " +
                                  MidChordSweep +
                                  " MidChordSweepSideways = " +
                                  MidChordSweepSideways +
                                  "\n\r at " +
                                  part.name);
                force = AerodynamicCenter = Vector3d.zero;
            }

            double numericalControlFactor =
                part.rb.mass * v_scalar * 0.67 / (force.magnitude * TimeWarp.fixedDeltaTime);
            force *= Math.Min(numericalControlFactor, 1);


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
            double PerpVelocity = Vector3d.Dot(part_transform.forward, velocity.normalized);
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
            ClIncrementFromRear = 0;

            rawAoAmax = CalculateAoAmax(MachNumber);

            liftslope = rawLiftSlope;
            wingInteraction.UpdateOrientationForInteraction(ParallelInPlaneLocal);
            wingInteraction.CalculateEffectsOfUpstreamWing(AoA,
                                                           MachNumber,
                                                           ParallelInPlaneLocal,
                                                           ref ACweight,
                                                           ref ACshift,
                                                           ref ClIncrementFromRear);
            double effectiveUpstreamInfluence = wingInteraction.EffectiveUpstreamInfluence;

            if (effectiveUpstreamInfluence > 0)
            {
                effectiveUpstreamInfluence = wingInteraction.EffectiveUpstreamInfluence;

                AoAmax = wingInteraction.EffectiveUpstreamAoAMax;
                liftslope *= 1 - effectiveUpstreamInfluence;
                liftslope += wingInteraction.EffectiveUpstreamLiftSlope;

                cosSweepAngle *= 1 - effectiveUpstreamInfluence;
                cosSweepAngle += wingInteraction.EffectiveUpstreamCosSweepAngle;
                cosSweepAngle = cosSweepAngle.Clamp(0d, 1d);
            }
            else
            {
                liftslope = rawLiftSlope;
                AoAmax = 0;
            }

            AoAmax += rawAoAmax;
        }

        //Calculates current stall fraction based on previous stall fraction and current data.
        private void DetermineStall(double AoA)
        {
            double lastStall = stall;
            double effectiveUpstreamStall = wingInteraction.EffectiveUpstreamStall;

            stall = 0;
            double absAoA = Math.Abs(AoA);

            if (absAoA > AoAmax)
            {
                stall = ((absAoA - AoAmax) * 10).Clamp(0, 1);
                stall = Math.Max(stall, lastStall);
                stall += effectiveUpstreamStall;
            }
            else if (absAoA < AoAmax)
            {
                stall = 1 - ((AoAmax - absAoA) * 25).Clamp(0, 1);
                stall = Math.Min(stall, lastStall);
                stall += effectiveUpstreamStall;
            }
            else
            {
                stall = lastStall;
            }

            stall = stall.Clamp(0, 1);
            if (stall < 1e-5)
                stall = 0;
        }


        /// <summary>
        ///     This calculates the lift and drag coefficients
        /// </summary>
        private void CalculateCoefficients(double MachNumber, double AoA, double skinFrictionCoefficient)
        {
            minStall = 0;

            rawLiftSlope = CalculateSubsonicLiftSlope(MachNumber); // / AoA;     //Prandtl lifting Line


            CalculateWingCamberInteractions(MachNumber, AoA, out double ACshift, out double ACweight);
            DetermineStall(AoA);

            double beta = Math.Sqrt(MachNumber * MachNumber - 1);
            if (double.IsNaN(beta) || beta < 0.66332495807107996982298654733414)
                beta = 0.66332495807107996982298654733414;

            double TanSweep = Math.Sqrt((1 - cosSweepAngle * cosSweepAngle).Clamp(0, 1)) / cosSweepAngle;
            double beta_TanSweep = beta / TanSweep;


            double Cd0 = CdCompressibilityZeroLiftIncrement(MachNumber, cosSweepAngle, TanSweep, beta_TanSweep, beta) +
                         2 * skinFrictionCoefficient;
            double CdMax = CdMaxFlatPlate(MachNumber, beta);
            e = FARAeroUtil.CalculateOswaldsEfficiencyNitaScholz(effective_AR, cosSweepAngle, Cd0, TaperRatio);
            piARe = effective_AR * e * Math.PI;

            double CosAoA = Math.Cos(AoA);

            if (MachNumber <= 0.8)
            {
                double Cn = liftslope;
                FinalLiftSlope = liftslope;
                double sinAoA = Math.Sqrt((1 - CosAoA * CosAoA).Clamp(0, 1));
                Cl = Cn * CosAoA * Math.Sign(AoA);

                Cl += ClIncrementFromRear;
                Cl *= sinAoA;

                if (Math.Abs(Cl) > Math.Abs(ACweight))
                    ACshift *= Math.Abs(ACweight / Cl).Clamp(0, 1);
                Cd = Cl * Cl / piARe; //Drag due to 3D effects on wing and base constant
                Cd += Cd0;
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
                FinalLiftSlope = coefMult * normalForce * supersonicLENormalForceFactor;

                Cl = FinalLiftSlope * CosAoA * Math.Sign(AoA);
                Cd = beta * Cl * Cl / piARe;

                Cd += Cd0;
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

                double Cn = liftslope;
                double sinAoA = Math.Sqrt((1 - CosAoA * CosAoA).Clamp(0, 1));
                Cl = Cn * CosAoA * sinAoA * Math.Sign(AoA);

                if (MachNumber <= 1)
                {
                    Cl += ClIncrementFromRear * sinAoA;
                    if (Math.Abs(Cl) > Math.Abs(ACweight))
                        ACshift *= Math.Abs(ACweight / Cl).Clamp(0, 1);
                }

                FinalLiftSlope = Cn * (1 - supScale);
                Cl *= 1 - supScale;

                double M = MachNumber.Clamp(1.2, double.PositiveInfinity);

                double coefMult = 2 / (FARAeroUtil.CurrentAdiabaticIndex * M * M);

                double supersonicLENormalForceFactor = CalculateSupersonicLEFactor(beta, TanSweep, beta_TanSweep);

                double normalForce = GetSupersonicPressureDifference(M, AoA);

                double supersonicLiftSlope = coefMult * normalForce * supersonicLENormalForceFactor * supScale;
                FinalLiftSlope += supersonicLiftSlope;


                Cl += CosAoA * Math.Sign(AoA) * supersonicLiftSlope;

                double effectiveBeta = beta * supScale + (1 - supScale);

                Cd = effectiveBeta * Cl * Cl / piARe;

                Cd += Cd0;
            }

            //AC shift due to flaps
            Vector3d ACShiftVec;
            if (!double.IsNaN(ACshift) && MachNumber <= 1)
                ACShiftVec = ACshift * ParallelInPlane;
            else
                ACShiftVec = Vector3d.zero;

            //Stalling effects
            stall = stall.Clamp(minStall, 1);

            //AC shift due to stall
            if (stall > 0)
                ACShiftVec -= 0.75 / criticalCl * MAC_actual * Math.Abs(Cl) * stall * ParallelInPlane * CosAoA;

            Cl -= Cl * stall * 0.769;
            Cd += Cd * stall * 3;
            Cd = Math.Max(Cd, CdMax * (1 - CosAoA * CosAoA));

            AerodynamicCenter += ACShiftVec;

            Cl *= wingInteraction.ClInterferenceFactor;

            FinalLiftSlope *= wingInteraction.ClInterferenceFactor;

            ClIncrementFromRear = 0;
        }

        //Calculates effect of the Mach cone being in front of, along, or behind the leading edge of the wing
        private double CalculateSupersonicLEFactor(double beta, double TanSweep, double beta_TanSweep)
        {
            double SupersonicLEFactor;
            double ARTanSweep = effective_AR * TanSweep;

            if (beta_TanSweep < 1) //"subsonic" leading edge, scales with Tan Sweep
            {
                if (beta_TanSweep < 0.5)
                {
                    SupersonicLEFactor = 1.57 * effective_AR;
                    SupersonicLEFactor /= ARTanSweep + 0.5;
                }
                else
                {
                    SupersonicLEFactor = (1.57 - 0.28 * (beta_TanSweep - 0.5)) * effective_AR;
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
            double nu1 = FARAeroUtil.PrandtlMeyerMach.Evaluate((float)inM);
            double theta = angle * FARMathUtil.rad2deg;
            double nu2 = nu1 + theta;
            if (nu2 >= FARAeroUtil.maxPrandtlMeyerTurnAngle)
                nu2 = FARAeroUtil.maxPrandtlMeyerTurnAngle;
            outM = FARAeroUtil.PrandtlMeyerAngle.Evaluate((float)nu2);

            return FARAeroUtil.StagnationPressureCalc(inM) / FARAeroUtil.StagnationPressureCalc(outM);
        }

        //Calculates pressure ratio due to turning a supersonic flow through a Prandtl-Meyer Expansion
        private static double PMExpansionCalculation(double angle, double inM)
        {
            inM = inM.Clamp(1, double.PositiveInfinity);
            double nu1 = FARAeroUtil.PrandtlMeyerMach.Evaluate((float)inM);
            double theta = angle * FARMathUtil.rad2deg;
            double nu2 = nu1 + theta;
            if (nu2 >= FARAeroUtil.maxPrandtlMeyerTurnAngle)
                nu2 = FARAeroUtil.maxPrandtlMeyerTurnAngle;
            float outM = FARAeroUtil.PrandtlMeyerAngle.Evaluate((float)nu2);

            return FARAeroUtil.StagnationPressureCalc(inM) / FARAeroUtil.StagnationPressureCalc(outM);
        }

        //Short calculation for peak AoA for stalling
        protected double CalculateAoAmax(double MachNumber)
        {
            double StallAngle;
            if (MachNumber < 0.8)
            {
                StallAngle = criticalCl / liftslope;
            }
            else if (MachNumber > 1.4)
            {
                StallAngle = 1.0471975511965977461542144610932; //60 degrees in radians
            }
            else
            {
                double tmp = criticalCl / liftslope;
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
            double CosPartAngle = Vector3.Dot(sweepPerpLocal, ParallelInPlaneLocal).Clamp(-1, 1);
            double tmp = Vector3.Dot(sweepPerp2Local, ParallelInPlaneLocal).Clamp(-1, 1);

            //Based on perpendicular vector find which line is the right one
            double sweepHalfChord = Math.Abs(CosPartAngle) > Math.Abs(tmp) ? CosPartAngle : tmp;

            CosPartAngle = ParallelInPlaneLocal.y.Clamp(-1, 1);

            CosPartAngle *= CosPartAngle;
            //Get the squared values for the angles
            double SinPartAngle2 = (1d - CosPartAngle).Clamp(0, 1);

            effective_b_2 = Math.Max(b_2_actual * CosPartAngle, MAC_actual * SinPartAngle2);
            effective_MAC = MAC_actual * CosPartAngle + b_2_actual * SinPartAngle2;
            transformed_AR = effective_b_2 / effective_MAC;

            //convert to tangent
            sweepHalfChord = Math.Sqrt(Math.Max(1 - sweepHalfChord * sweepHalfChord, 0)) / sweepHalfChord;

            SetSweepAngle(sweepHalfChord);

            effective_AR = wingInteraction.SpanChainedAR(transformed_AR);

            //Even this range of effective ARs is large, but it keeps the Oswald's Efficiency numbers in check
            effective_AR = effective_AR.Clamp(0.25, 30d);

            if (MachNumber < 0.9)
                tmp = 1d - MachNumber * MachNumber;
            else
                tmp = 0.19;

            // a wing on a bare body lifts as the whole wing through the body: that wing's half-chord sweep
            double sweepTmp = double.IsNaN(wingInteraction.GroupTanSweep) ? sweepHalfChord : wingInteraction.GroupTanSweep;
            sweepTmp *= sweepTmp;

            tmp += sweepTmp;
            tmp = tmp * effective_AR * effective_AR;
            tmp += 4;
            tmp = Math.Sqrt(tmp);
            tmp += 2;
            tmp = 1 / tmp;
            tmp *= 2 * Math.PI;

            // and the panels carry that wing's lift on the body between its roots
            return tmp * effective_AR * wingInteraction.BodyLiftFactor;
        }

        //Transforms cos sweep of the midchord to cosine(sweep of the leading edge)
        private void SetSweepAngle(double tanSweepHalfChord)
        {
            double tmp = (1d - TaperRatio) / (1d + TaperRatio);
            tmp *= 2d / transformed_AR;
            tanSweepHalfChord += tmp;
            cosSweepAngle = 1d / Math.Sqrt(1d + tanSweepHalfChord * tanSweepHalfChord);
            if (cosSweepAngle > 1d)
                cosSweepAngle = 1d;
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
                    zeroLiftCdIncrement = wingInteraction.EffectiveUpstreamCd0;
                    return zeroLiftCdIncrement;
                }

                thisInteractionFactor = 1 - wingInteraction.EffectiveUpstreamInfluence;
            }

            //Based on the method of DATCOM Section 4.1.5.1-C
            if (M > 1.4)
            {
                //Subsonic leading edge
                if (beta_TanSweep < 1)
                    //This constant is due to airfoil shape and thickness
                    zeroLiftCdIncrement = 0.009216 / TanSweep;
                //Supersonic leading edge
                else
                    zeroLiftCdIncrement = 0.009216 / beta;
                zeroLiftCdIncrement *= thisInteractionFactor;
                zeroLiftCdIncrement +=
                    wingInteraction.EffectiveUpstreamCd0 * wingInteraction.EffectiveUpstreamInfluence;
                return zeroLiftCdIncrement;
            }


            //Based on the method of DATCOM Section 4.1.5.1-B
            double tmp = 1 / Math.Sqrt(SweepAngle);

            double dd_MachNumber = 0.8 * tmp; //Find Drag Divergence Mach Number

            if (M < dd_MachNumber) //If below this number,
            {
                zeroLiftCdIncrement = 0;
                return 0;
            }

            double peak_MachNumber = 1.1 * tmp;

            double peak_Increment = 0.025 * FARMathUtil.PowApprox(SweepAngle, 2.5);

            if (M > peak_MachNumber)
            {
                zeroLiftCdIncrement = peak_Increment;
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

                zeroLiftCdIncrement = CdIncrement;
            }

            double scalingMachNumber = Math.Min(peak_MachNumber, 1.2);

            if (M < scalingMachNumber)
            {
                zeroLiftCdIncrement *= thisInteractionFactor;
                zeroLiftCdIncrement +=
                    wingInteraction.EffectiveUpstreamCd0 * wingInteraction.EffectiveUpstreamInfluence;
                return zeroLiftCdIncrement;
            }

            double scale = (M - 1.4) / (scalingMachNumber - 1.4);
            zeroLiftCdIncrement *= scale;
            scale = 1 - scale;

            //Subsonic leading edge
            if (beta_TanSweep < 1)
                //This constant is due to airfoil shape and thickness
                zeroLiftCdIncrement += 0.009216 / TanSweep * scale;
            //Supersonic leading edge
            else
                zeroLiftCdIncrement += 0.009216 / beta * scale;
            zeroLiftCdIncrement *= thisInteractionFactor;
            zeroLiftCdIncrement += wingInteraction.EffectiveUpstreamCd0 * wingInteraction.EffectiveUpstreamInfluence;

            return zeroLiftCdIncrement;
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
