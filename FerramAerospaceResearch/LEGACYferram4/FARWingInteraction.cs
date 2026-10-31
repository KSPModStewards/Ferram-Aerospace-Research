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
using System.Linq;
using FerramAerospaceResearch;
using UnityEngine;

// ReSharper disable once CheckNamespace
namespace ferram4
{
    public class FARWingInteraction
    {
        private static readonly RaycastHitComparer _comparer = new RaycastHitComparer();


        private static FloatCurve wingCamberFactor;
        private static FloatCurve wingCamberMoment;
        private readonly Vector3 rootChordMidLocal;
        private readonly short srfAttachFlipped;

        private FARWingAerodynamicModel parentWingModule;
        private Part parentWingPart;
        private Vector3 rootChordMidPt;

        private List<FARWingAerodynamicModel> nearbyWingModulesForwardList = new List<FARWingAerodynamicModel>();
        private List<FARWingAerodynamicModel> nearbyWingModulesBackwardList = new List<FARWingAerodynamicModel>();
        private List<FARWingAerodynamicModel> nearbyWingModulesLeftwardList = new List<FARWingAerodynamicModel>();
        private List<FARWingAerodynamicModel> nearbyWingModulesRightwardList = new List<FARWingAerodynamicModel>();

        private List<double> nearbyWingModulesForwardInfluence = new List<double>();
        private List<double> nearbyWingModulesBackwardInfluence = new List<double>();
        private List<double> nearbyWingModulesLeftwardInfluence = new List<double>();
        private List<double> nearbyWingModulesRightwardInfluence = new List<double>();

        private double forwardExposure;
        private double backwardExposure;
        private double leftwardExposure;
        private double rightwardExposure;

        public FARWingInteraction(
            FARWingAerodynamicModel parentModule,
            Part parentPart,
            Vector3 rootChordMid,
            short srfAttachNegative
        )
        {
            parentWingModule = parentModule;
            parentWingPart = parentPart;
            rootChordMidLocal = rootChordMid;
            srfAttachFlipped = srfAttachNegative;

            if (wingCamberFactor == null)
            {
                wingCamberFactor = new FloatCurve();
                wingCamberFactor.Add(0, 0);
                for (double i = 0.1; i <= 0.9; i += 0.1)
                {
                    double tmp = i * 2;
                    tmp--;
                    tmp = Math.Acos(tmp);

                    tmp -= Math.Sin(tmp);
                    tmp /= Math.PI;
                    tmp = 1 - tmp;

                    wingCamberFactor.Add((float)i, (float)tmp);
                }

                wingCamberFactor.Add(1, 1);
            }

            if (wingCamberMoment != null)
                return;
            wingCamberMoment = new FloatCurve();
            for (double i = 0; i <= 1; i += 0.1)
            {
                double tmp = i * 2;
                tmp--;
                tmp = Math.Acos(tmp);

                tmp = (Math.Sin(2 * tmp) - 2 * Math.Sin(tmp)) / (8 * (Math.PI - tmp + Math.Sin(tmp)));

                wingCamberMoment.Add((float)i, (float)tmp);
            }
        }

        public double EffectiveUpstreamMAC { get; private set; }
        public double EffectiveUpstreamb_2 { get; private set; }
        public double EffectiveUpstreamLiftSlope { get; private set; }
        public double EffectiveUpstreamArea { get; private set; }
        public double EffectiveUpstreamStall { get; private set; }
        public double EffectiveUpstreamCosSweepAngle { get; private set; }
        public double EffectiveUpstreamAoAMax { get; private set; }
        public double EffectiveUpstreamAoA { get; private set; }
        public double EffectiveUpstreamCd0 { get; private set; }
        public double EffectiveUpstreamInfluence { get; private set; }
        public bool HasWingsUpstream { get; private set; }

        public double ARFactor { get; private set; } = 1;

        public double ClInterferenceFactor { get; private set; } = 1;

        public void Destroy()
        {
            nearbyWingModulesForwardList = nearbyWingModulesBackwardList =
                                               nearbyWingModulesLeftwardList = nearbyWingModulesRightwardList = null;
            nearbyWingModulesForwardInfluence = nearbyWingModulesBackwardInfluence =
                                                    nearbyWingModulesLeftwardInfluence =
                                                        nearbyWingModulesRightwardInfluence = null;
            parentWingModule = null;
            parentWingPart = null;
        }

        /// <summary>
        ///     Called when plane is stopped to get rid of old wing interaction data
        /// </summary>
        public void ResetWingInteractions()
        {
            EffectiveUpstreamLiftSlope = 0;
            EffectiveUpstreamStall = 0;
            EffectiveUpstreamCosSweepAngle = 0;
            EffectiveUpstreamAoAMax = 0;
            EffectiveUpstreamAoA = 0;
            EffectiveUpstreamCd0 = 0;
            EffectiveUpstreamInfluence = 0;
        }

        /// <summary>
        ///     Recalculates all nearby wings; call when vessel has changed shape
        /// </summary>
        /// <param name="VesselPartList">A list of all parts on this vessel</param>
        /// <param name="isSmallSrf">
        ///     If a part should be considered a small attachable surface, like an aileron, elevator, etc;
        ///     used to calculate nearby wings properly
        /// </param>
        public void UpdateWingInteraction(List<Part> VesselPartList, bool isSmallSrf)
        {
            float flt_MAC = (float)parentWingModule.MAC_actual;
            float flt_b_2 = (float)parentWingModule.b_2_actual;
            float flt_TaperRatio = (float)parentWingModule.TaperRatio;
            float flt_MidChordSweep = (float)parentWingModule.MidChordSweep;

            FARWingAerodynamicModel[] nearbyWingModulesForward;
            FARWingAerodynamicModel[] nearbyWingModulesBackward;
            FARWingAerodynamicModel[] nearbyWingModulesLeftward;
            FARWingAerodynamicModel[] nearbyWingModulesRightward;

            if (parentWingPart == null)
                return;

            rootChordMidPt = parentWingPart.partTransform.position +
                             parentWingPart.partTransform.TransformDirection(rootChordMidLocal);

            Vector3 up = parentWingPart.partTransform.up;
            Vector3 right = parentWingPart.partTransform.right;
            if (isSmallSrf)
            {
                forwardExposure = ExposureSmallSrf(out nearbyWingModulesForward, up, VesselPartList, flt_MAC, flt_MAC);

                backwardExposure =
                    ExposureSmallSrf(out nearbyWingModulesBackward, -up, VesselPartList, flt_MAC, flt_MAC);

                leftwardExposure =
                    ExposureSmallSrf(out nearbyWingModulesLeftward, -right, VesselPartList, flt_b_2, flt_MAC);

                rightwardExposure =
                    ExposureSmallSrf(out nearbyWingModulesRightward, right, VesselPartList, flt_b_2, flt_MAC);
            }
            else
            {
                forwardExposure = ExposureInChordDirection(out nearbyWingModulesForward,
                                                           up,
                                                           VesselPartList,
                                                           flt_b_2,
                                                           flt_MAC,
                                                           flt_MidChordSweep);

                backwardExposure = ExposureInChordDirection(out nearbyWingModulesBackward,
                                                            -up,
                                                            VesselPartList,
                                                            flt_b_2,
                                                            flt_MAC,
                                                            flt_MidChordSweep);

                leftwardExposure = ExposureInSpanDirection(out nearbyWingModulesLeftward,
                                                           -right,
                                                           VesselPartList,
                                                           flt_b_2,
                                                           flt_MAC,
                                                           flt_TaperRatio,
                                                           flt_MidChordSweep);

                rightwardExposure = ExposureInSpanDirection(out nearbyWingModulesRightward,
                                                            right,
                                                            VesselPartList,
                                                            flt_b_2,
                                                            flt_MAC,
                                                            flt_TaperRatio,
                                                            flt_MidChordSweep);
            }

            CompressArrayToList(nearbyWingModulesForward,
                                ref nearbyWingModulesForwardList,
                                ref nearbyWingModulesForwardInfluence);
            CompressArrayToList(nearbyWingModulesBackward,
                                ref nearbyWingModulesBackwardList,
                                ref nearbyWingModulesBackwardInfluence);
            CompressArrayToList(nearbyWingModulesLeftward,
                                ref nearbyWingModulesLeftwardList,
                                ref nearbyWingModulesLeftwardInfluence);
            CompressArrayToList(nearbyWingModulesRightward,
                                ref nearbyWingModulesRightwardList,
                                ref nearbyWingModulesRightwardInfluence);

            //This part handles effects of biplanes, triplanes, etc.
            double ClCdInterference = 0;
            Vector3 forward = parentWingPart.partTransform.forward;
            ClCdInterference += 0.5f * WingInterference(forward, VesselPartList, flt_b_2);
            ClCdInterference += 0.5f * WingInterference(-forward, VesselPartList, flt_b_2);

            ClInterferenceFactor = ClCdInterference;
        }

        //This updates the interactions of all wings near this one; call this one when something changes rather than all of them at once
        // ReSharper disable once UnusedMember.Global
        public HashSet<FARWingAerodynamicModel> UpdateNearbyWingInteractions(
            HashSet<FARWingAerodynamicModel> wingsHandled
        )
        {
            //Hashset to avoid repeating the same one affected

            foreach (FARWingAerodynamicModel w in nearbyWingModulesForwardList)
            {
                if (wingsHandled.Contains(w))
                    continue;
                w.UpdateThisWingInteractions();
                wingsHandled.Add(w);
            }

            foreach (FARWingAerodynamicModel w in nearbyWingModulesBackwardList)
            {
                if (wingsHandled.Contains(w))
                    continue;
                w.UpdateThisWingInteractions();
                wingsHandled.Add(w);
            }

            foreach (FARWingAerodynamicModel w in nearbyWingModulesRightwardList)
            {
                if (wingsHandled.Contains(w))
                    continue;
                w.UpdateThisWingInteractions();
                wingsHandled.Add(w);
            }

            foreach (FARWingAerodynamicModel w in nearbyWingModulesLeftwardList)
            {
                if (wingsHandled.Contains(w))
                    continue;
                w.UpdateThisWingInteractions();
                wingsHandled.Add(w);
            }

            return wingsHandled;
        }

        private void CompressArrayToList(
            FARWingAerodynamicModel[] arrayIn,
            ref List<FARWingAerodynamicModel> moduleList,
            ref List<double> associatedInfluences
        )
        {
            moduleList.Clear();
            associatedInfluences.Clear();
            double influencePerIndex = 1 / (double)arrayIn.Length;

            Vector3 forward = parentWingPart.partTransform.forward;
            foreach (FARWingAerodynamicModel w in arrayIn)
            {
                if (w is null)
                    continue;
                bool foundModule = false;
                double influence = influencePerIndex * Math.Abs(Vector3.Dot(forward, w.part.partTransform.forward));
                for (int j = 0; j < moduleList.Count; j++)
                {
                    if (moduleList[j] != w)
                        continue;
                    associatedInfluences[j] += influence;
                    foundModule = true;
                    break;
                }

                if (foundModule)
                    continue;

                moduleList.Add(w);
                associatedInfluences.Add(influence);
            }
        }

        private double WingInterference(Vector3 rayDirection, List<Part> PartList, float dist)
        {
            double interferenceValue = 1;

            var ray = new Ray
            {
                origin = parentWingModule.WingCentroid(),
                direction = rayDirection
            };


            var hit = new RaycastHit();

            bool gotSomething = false;

            hit.distance = 0;
            RaycastHit[] hits = Physics.RaycastAll(ray, dist, FARAeroUtil.RaycastMask);
            foreach (RaycastHit h in hits)
            {
                if (h.collider == null)
                    continue;
                foreach (Part p in PartList)
                {
                    if (p == null)
                        continue;

                    if (p == parentWingPart)
                        continue;

                    var w = p.GetComponent<FARWingAerodynamicModel>();

                    if (w != null)
                    {
                        Collider[] colliders = w.PartColliders;

                        // ReSharper disable once LoopCanBeConvertedToQuery -> closure
                        foreach (Collider collider in colliders)
                            if (h.collider == collider && h.distance > 0)
                            {
                                double tmp = h.distance / dist;
                                tmp = tmp.Clamp(0, 1);
                                double tmp2 =
                                    Math.Abs(Vector3.Dot(parentWingPart.partTransform.forward,
                                                         w.part.partTransform.forward));
                                tmp = 1 - (1 - tmp) * tmp2;
                                interferenceValue = Math.Min(tmp, interferenceValue);
                                gotSomething = true;

                                break;
                            }
                    }

                    if (gotSomething)
                        break;
                }
            }

            return interferenceValue;
        }

        private double ExposureInChordDirection(
            out FARWingAerodynamicModel[] nearbyWings,
            Vector3 rayDirection,
            List<Part> vesselPartList,
            float b_2,
            float MAC,
            float MidChordSweep
        )
        {
            var ray = new Ray {direction = rayDirection};

            nearbyWings = new FARWingAerodynamicModel[5];

            double exposure = 1;
            Vector3 partTransformRight = parentWingPart.partTransform.right;
            Vector3 partTransformUp = parentWingPart.partTransform.up;
            for (int i = 0; i < 5; i++)
            {
                //shift the raycast origin along the midchord line
                ray.origin = rootChordMidPt +
                             (float)(i * 0.2 + 0.1) *
                             -b_2 *
                             (partTransformRight * srfAttachFlipped +
                              partTransformUp * (float)Math.Tan(MidChordSweep * FARMathUtil.deg2rad));

                RaycastHit[] hits = Physics.RaycastAll(ray, MAC, FARAeroUtil.RaycastMask);

                nearbyWings[i] = ExposureHitDetectionAndWingDetection(hits, vesselPartList, ref exposure, 0.2);
            }

            return exposure;
        }

        private double ExposureInSpanDirection(
            out FARWingAerodynamicModel[] nearbyWings,
            Vector3 rayDirection,
            List<Part> vesselPartList,
            float b_2,
            float MAC,
            float TaperRatio,
            float MidChordSweep
        )
        {
            var ray = new Ray {direction = rayDirection};

            nearbyWings = new FARWingAerodynamicModel[5];

            double exposure = 1;

            Vector3 partTransformRight = parentWingPart.partTransform.right;
            Vector3 partTransformUp = parentWingPart.partTransform.up;
            for (int i = 0; i < 5; i++)
            {
                //shift the origin along the midchord line
                ray.origin = rootChordMidPt +
                             0.5f *
                             -b_2 *
                             (partTransformRight * srfAttachFlipped +
                              partTransformUp * (float)Math.Tan(MidChordSweep * FARMathUtil.deg2rad));

                float chord_length = 2 * MAC / (1 + TaperRatio); //first, calculate the root chord

                //determine the chord length based on how far down the span it is
                chord_length = chord_length * (1 - 0.5f) + TaperRatio * chord_length * 0.5f;

                ray.origin += chord_length * (-0.4f + 0.2f * i) * partTransformUp;

                RaycastHit[] hits = Physics.RaycastAll(ray, b_2, FARAeroUtil.RaycastMask);

                nearbyWings[i] = ExposureHitDetectionAndWingDetection(hits, vesselPartList, ref exposure, 0.2);
            }

            return exposure;
        }

        private double ExposureSmallSrf(
            out FARWingAerodynamicModel[] nearbyWings,
            Vector3 rayDirection,
            List<Part> vesselPartList,
            float rayCastDist,
            float MAC
        )
        {
            var ray = new Ray {direction = rayDirection};

            nearbyWings = new FARWingAerodynamicModel[1];

            double exposure = 1;
            ray.origin = rootChordMidPt - MAC * 0.7f * parentWingPart.partTransform.up;

            RaycastHit[] hits = Physics.RaycastAll(ray.origin, ray.direction, rayCastDist, FARAeroUtil.RaycastMask);

            nearbyWings[0] = ExposureHitDetectionAndWingDetection(hits, vesselPartList, ref exposure, 1);

            return exposure;
        }

        private FARWingAerodynamicModel ExposureHitDetectionAndWingDetection(
            RaycastHit[] hits,
            List<Part> vesselPartList,
            ref double exposure,
            double exposureDecreasePerHit
        )
        {
            bool firstHit = true;
            double wingInteractionFactor = 0;

            FARWingAerodynamicModel wingHit = null;

            RaycastHit[] sortedHits = SortHitsByDistance(hits);
            foreach (RaycastHit h in sortedHits)
            {
                bool gotSomething = false;
                if (h.collider == null)
                    continue;
                foreach (Part p in vesselPartList)
                {
                    if (p == null || p == parentWingPart)
                        continue;

                    var farModule = p.GetComponent<FARPartModule>();

                    Collider[] colliders;

                    if (!(farModule is null))
                    {
                        colliders = farModule.PartColliders;
                        if (colliders == null)
                        {
                            farModule.TriggerPartColliderUpdate();
                            colliders = farModule.PartColliders;
                        }
                    }
                    else
                    {
                        colliders = new[] {p.collider};
                    }

                    // ReSharper disable once LoopCanBeConvertedToQuery -> closure
                    foreach (Collider collider in colliders)
                        if (h.collider == collider && h.distance > 0)
                        {
                            if (firstHit)
                            {
                                exposure -= exposureDecreasePerHit;
                                firstHit = false;
                            }

                            var hitModule = p.GetComponent<FARWingAerodynamicModel>();
                            if (hitModule != null)
                            {
                                double tmp =
                                    Math.Abs(Vector3.Dot(p.transform.forward, parentWingPart.partTransform.forward));
                                if (tmp > wingInteractionFactor + 0.01)
                                {
                                    wingInteractionFactor = tmp;
                                    wingHit = hitModule;
                                }
                            }

                            gotSomething = true;
                            break;
                        }

                    if (gotSomething)
                        break;
                }
            }

            return wingHit;
        }

        private static RaycastHit[] SortHitsByDistance(RaycastHit[] unsortedList)
        {
            List<RaycastHit> sortedHits = unsortedList.ToList();

            sortedHits.Sort(_comparer);

            return sortedHits.ToArray();
        }

        private bool DetermineWingsUpstream(double wingForwardDir, double wingRightwardDir)
        {
            if (wingForwardDir > 0)
            {
                if (nearbyWingModulesForwardList.Count > 0)
                    return true;
            }
            else
            {
                if (nearbyWingModulesBackwardList.Count > 0)
                    return true;
            }

            if (wingRightwardDir > 0)
            {
                if (nearbyWingModulesRightwardList.Count > 0)
                    return true;
            }
            else
            {
                if (nearbyWingModulesLeftwardList.Count > 0)
                    return true;
            }

            return false;
        }


        /// <summary>
        ///     Accounts for increments in lift due to camber changes from upstream wings, and returns changes for this wing part;
        ///     returns true if there are wings in front of it
        /// </summary>
        /// <param name="thisWingAoA">AoA of this wing in rad</param>
        /// <param name="thisWingMachNumber">Mach Number of this wing in rad</param>
        /// <param name="parallelInPlaneLocal">Local coordinate system (right, forward, ..)</param>
        /// <param name="ACweight">Weighting value for applying ACshift</param>
        /// <param name="ACshift">Value used to shift the wing AC due to interactive effects</param>
        /// <param name="ClIncrementFromRear">Increase in Cl due to this</param>
        /// <returns></returns>
        public void CalculateEffectsOfUpstreamWing(
            double thisWingAoA,
            double thisWingMachNumber,
            Vector3d parallelInPlaneLocal,
            ref double ACweight,
            ref double ACshift,
            ref double ClIncrementFromRear
        )
        {
            double thisWingMAC = parentWingModule.GetMAC();
            double thisWingb_2 = parentWingModule.Getb_2();

            EffectiveUpstreamMAC = 0;
            EffectiveUpstreamb_2 = 0;
            EffectiveUpstreamArea = 0;

            EffectiveUpstreamLiftSlope = 0;
            EffectiveUpstreamStall = 0;
            EffectiveUpstreamCosSweepAngle = 0;
            EffectiveUpstreamAoAMax = 0;
            EffectiveUpstreamAoA = 0;
            EffectiveUpstreamCd0 = 0;
            EffectiveUpstreamInfluence = 0;

            double wingForwardDir = parallelInPlaneLocal.y;
            double wingRightwardDir = parallelInPlaneLocal.x * srfAttachFlipped;

            if (wingForwardDir > 0)
            {
                wingForwardDir *= wingForwardDir;
                UpdateUpstreamValuesFromWingModules(nearbyWingModulesForwardList,
                                                    nearbyWingModulesForwardInfluence,
                                                    wingForwardDir,
                                                    thisWingAoA);
            }
            else
            {
                wingForwardDir *= wingForwardDir;
                UpdateUpstreamValuesFromWingModules(nearbyWingModulesBackwardList,
                                                    nearbyWingModulesBackwardInfluence,
                                                    wingForwardDir,
                                                    thisWingAoA);
            }

            if (wingRightwardDir > 0)
            {
                wingRightwardDir *= wingRightwardDir;
                UpdateUpstreamValuesFromWingModules(nearbyWingModulesRightwardList,
                                                    nearbyWingModulesRightwardInfluence,
                                                    wingRightwardDir,
                                                    thisWingAoA);
            }
            else
            {
                wingRightwardDir *= wingRightwardDir;
                UpdateUpstreamValuesFromWingModules(nearbyWingModulesLeftwardList,
                                                    nearbyWingModulesLeftwardInfluence,
                                                    wingRightwardDir,
                                                    thisWingAoA);
            }

            double MachCoeff = (1 - thisWingMachNumber * thisWingMachNumber).Clamp(0, 1);

            if (MachCoeff.NearlyEqual(0))
                return;
            double flapRatio = (thisWingMAC / (thisWingMAC + EffectiveUpstreamMAC)).Clamp(0, 1);
            float flt_flapRatio = (float)flapRatio;
            //Flap Effectiveness Factor
            double flapFactor = wingCamberFactor.Evaluate(flt_flapRatio);
            //Change in moment due to change in lift from flap
            double dCm_dCl = wingCamberMoment.Evaluate(flt_flapRatio);

            //This accounts for the wing possibly having a longer span than the flap
            double WingFraction = (thisWingb_2 / EffectiveUpstreamb_2).Clamp(0, 1);
            //This accounts for the flap possibly having a longer span than the wing it's attached to
            double FlapFraction = (EffectiveUpstreamb_2 / thisWingb_2).Clamp(0, 1);

            //Lift created by the flap interaction
            double ClIncrement = flapFactor * EffectiveUpstreamLiftSlope * EffectiveUpstreamAoA;
            //Increase the Cl so that even though we're working with the flap's area, it accounts for the added lift across the entire object
            ClIncrement *= (parentWingModule.S * FlapFraction + EffectiveUpstreamArea * WingFraction) /
                           parentWingModule.S;

            // Total flap Cl for the purpose of applying ACshift, including the bit subtracted below
            ACweight = ClIncrement * MachCoeff;

            //Removing additional angle so that lift of the flap is calculated as lift at wing angle + lift due to flap interaction rather than being greater
            ClIncrement -= FlapFraction * EffectiveUpstreamLiftSlope * EffectiveUpstreamAoA;

            //Change in Cm with change in Cl
            ACshift = (dCm_dCl + 0.75 * (1 - flapRatio)) * (thisWingMAC + EffectiveUpstreamMAC);

            ClIncrementFromRear = ClIncrement * MachCoeff;
        }

        /// <summary>
        ///     Updates all FARWingInteraction orientation-based variables using the in-wing-plane velocity vector
        /// </summary>
        /// <param name="parallelInPlaneLocal">Normalized local velocity vector projected onto wing surface</param>
        public void UpdateOrientationForInteraction(Vector3d parallelInPlaneLocal)
        {
            double wingForwardDir = parallelInPlaneLocal.y;
            double wingRightwardDir = parallelInPlaneLocal.x * srfAttachFlipped;

            ARFactor = CalculateARFactor(wingForwardDir, wingRightwardDir);
            HasWingsUpstream = DetermineWingsUpstream(wingForwardDir, wingRightwardDir);
        }

        private void UpdateUpstreamValuesFromWingModules(
            List<FARWingAerodynamicModel> wingModules,
            List<double> associatedInfluences,
            double directionalInfluence,
            double thisWingAoA
        )
        {
            directionalInfluence = Math.Abs(directionalInfluence);
            for (int i = 0; i < wingModules.Count; i++)
            {
                FARWingAerodynamicModel wingModule = wingModules[i];
                double wingInfluenceFactor = associatedInfluences[i] * directionalInfluence;

                if (wingModule == null)
                {
                    HandleNullPart(wingModules, associatedInfluences, i);
                    i--;
                    continue;
                }

                if (wingModule.isShielded)
                    continue;

                double tmp = Vector3.Dot(wingModule.transform.forward, parentWingModule.transform.forward);

                EffectiveUpstreamMAC += wingModule.GetMAC() * wingInfluenceFactor;
                EffectiveUpstreamb_2 += wingModule.Getb_2() * wingInfluenceFactor;
                EffectiveUpstreamArea += wingModule.S * wingInfluenceFactor;

                EffectiveUpstreamLiftSlope += wingModule.GetRawLiftSlope() * wingInfluenceFactor;
                EffectiveUpstreamStall += wingModule.GetStall() * wingInfluenceFactor;
                EffectiveUpstreamCosSweepAngle += wingModule.GetCosSweepAngle() * wingInfluenceFactor;
                EffectiveUpstreamAoAMax += wingModule.rawAoAmax * wingInfluenceFactor;
                EffectiveUpstreamCd0 += wingModule.GetCd0() * wingInfluenceFactor;
                EffectiveUpstreamInfluence += wingInfluenceFactor;

                double wAoA = wingModule.CalculateAoA(wingModule.GetVelocity()) * Math.Sign(tmp);
                //First, make sure that the AoA are wrt the same direction; then account for any strange angling of the part that shouldn't be there
                tmp = (thisWingAoA - wAoA) * wingInfluenceFactor;

                EffectiveUpstreamAoA += tmp;
            }
        }

        private static void HandleNullPart(
            List<FARWingAerodynamicModel> wingModules,
            List<double> associatedInfluences,
            int index
        )
        {
            wingModules.RemoveAt(index);
            associatedInfluences.RemoveAt(index);

            double influenceSum = associatedInfluences.Sum();
            influenceSum = 1 / influenceSum;

            for (int j = 0; j < associatedInfluences.Count; j++)
                associatedInfluences[j] *= influenceSum;
        }

        /// <summary>
        ///     Calculates effect of nearby wings on the effective aspect ratio multiplier of this wing
        /// </summary>
        /// <param name="wingForwardDir">Local velocity vector forward component</param>
        /// <param name="wingRightwardDir">Local velocity vector leftward component</param>
        /// <returns>Factor to multiply aspect ratio by to calculate effective lift slope and drag</returns>
        private double CalculateARFactor(double wingForwardDir, double wingRightwardDir)
        {
            double wingtipExposure = 0;
            double wingrootExposure = 0;

            if (wingForwardDir > 0)
            {
                wingForwardDir *= wingForwardDir;
                wingtipExposure += leftwardExposure * wingForwardDir;
                wingrootExposure += rightwardExposure * wingForwardDir;
            }
            else
            {
                wingForwardDir *= wingForwardDir;
                wingtipExposure += rightwardExposure * wingForwardDir;
                wingrootExposure += leftwardExposure * wingForwardDir;
            }

            if (wingRightwardDir > 0)
            {
                wingRightwardDir *= wingRightwardDir;
                wingtipExposure += backwardExposure * wingRightwardDir;
                wingrootExposure += forwardExposure * wingRightwardDir;
            }
            else
            {
                wingRightwardDir *= wingRightwardDir;
                wingtipExposure += forwardExposure * wingRightwardDir;
                wingrootExposure += backwardExposure * wingRightwardDir;
            }

            wingtipExposure = 1 - wingtipExposure;
            wingrootExposure = 1 - wingrootExposure;


            double effective_AR_modifier = wingrootExposure + wingtipExposure;

            if (effective_AR_modifier < 1)
                return effective_AR_modifier + 1;
            return 2 * (2 - effective_AR_modifier) + 8 * (effective_AR_modifier - 1);
        }

        private class RaycastHitComparer : IComparer<RaycastHit>
        {
            public int Compare(RaycastHit h1, RaycastHit h2)
            {
                return h1.distance.CompareTo(h2.distance);
            }
        }
    }
}
