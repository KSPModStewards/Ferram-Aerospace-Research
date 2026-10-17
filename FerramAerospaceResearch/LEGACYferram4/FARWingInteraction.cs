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
            vesselParts = VesselPartList;
            wingGroupGeneration++;
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

        /// <summary>
        ///     Finds every wing's group again: call when which panels are shielded may have changed
        /// </summary>
        internal static void InvalidateWingGroups()
        {
            wingGroupGeneration++;
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

        // a control surface: the wing it is on (the wing group its part touches), found again with the groups, and the
        // control surface as a panel
        private WingGroup hostWing;
        private WingGroup.Panel hostPanel;
        private int hostGeneration = -1;

        /// <summary>The wing group this control surface touches, if it is one (the largest, if it touches several)</summary>
        private bool FindHostWing()
        {
            if (hostGeneration == wingGroupGeneration)
                return hostWing != null && hostWing.Frame != null;
            hostGeneration = wingGroupGeneration;
            hostWing = null;
            hostPanel = null;
            if (!(parentWingModule is FARControllableSurface) || vesselParts == null || parentWingPart == null)
                return false;
            Transform frame = WingGroup.FrameOf(parentWingPart);
            WingGroup.Panel panel = WingGroup.MakePanel(parentWingModule, frame);
            if (panel == null)
                return false;

            var tested = new HashSet<WingGroup>();
            double best = 0;
            foreach (Part p in vesselParts)
            {
                if (p == null || p == parentWingPart)
                    continue;
                FARWingAerodynamicModel w = p.FindModuleImplementing<FARWingAerodynamicModel>();
                if (w == null || w is FARControllableSurface || w.isShielded || w.WingInteraction == null)
                    continue;
                FARWingInteraction wi = w.WingInteraction;
                if (wi.wingGroup == null || wi.wingGroup.Generation != wingGroupGeneration)
                    WingGroup.Build(w, vesselParts);
                WingGroup g = wi.wingGroup;
                if (g == null || g.Frame != frame || !tested.Add(g) || !g.Touches(panel))
                    continue;
                double area = g.Members.Sum(m => m.S);
                if (area > best)
                {
                    best = area;
                    hostWing = g;
                }
            }

            hostPanel = panel;
            return hostWing != null;
        }

        // the wing this panel is part of, found again whenever any wing's neighbors have been
        private static int wingGroupGeneration;
        private WingGroup wingGroup;
        private List<Part> vesselParts;

        /// <summary>
        ///     The effective aspect ratio of this panel. Panels that touch, in about the same plane, are one wing however it
        ///     was cut: along the span, along the chord, or overlapping. Each panel takes the aspect ratio of the whole wing
        ///     (WingGroup.AspectRatio), and a control surface that of the wing it is on. For a wing on a bare body it also
        ///     sets the whole wing's half-chord sweep and the factor for the lift of the body between the roots
        ///     (GroupTanSweep, BodyLiftFactor). A panel that touches no other keeps its own aspect ratio times ARFactor, as
        ///     before.
        /// </summary>
        /// <param name="transformedAR">This panel's own aspect ratio (semispan over chord, along the flow)</param>
        /// <param name="parallelInPlaneLocal">Normalized local velocity vector projected onto wing surface</param>
        internal double SpanChainedAR(double transformedAR, Vector3d parallelInPlaneLocal)
        {
            double own = transformedAR * ARFactor;
            BodyLiftFactor = 1;
            GroupTanSweep = double.NaN;
            if (parentWingPart == null || vesselParts == null)
                return own;
            Vector3 flow = parentWingPart.partTransform.TransformDirection((Vector3)parallelInPlaneLocal);
            // a control surface is part of the wing it is on, and takes that wing's aspect ratio
            WingGroup g;
            if (parentWingModule is FARControllableSurface && FindHostWing() && hostWing.Members.Count > 1 &&
                hostWing.Measure(flow))
            {
                g = hostWing;
            }
            else
            {
                if (wingGroup == null || wingGroup.Generation != wingGroupGeneration)
                    WingGroup.Build(parentWingModule, vesselParts);
                g = wingGroup;
                if (g == null || g.Members.Count <= 1 || !g.Measure(flow))
                    return own;
            }

            if (g.RootOnBody)
            {
                // the whole wing through the body: its sweep, and the lift of the strip of body between the roots
                BodyLiftFactor = g.BodyLiftFactor;
                GroupTanSweep = g.HalfChordTan;
            }

            return g.AspectRatio();
        }

        // for a wing on a bare body (see WingGroup.BodyLiftFactor): its lift slope's factor for the body between the
        // roots, and the tangent of the whole wing's half-chord sweep, which replaces the panel's own (NaN: the panel's)
        internal double BodyLiftFactor = 1, GroupTanSweep = double.NaN;

        /// <summary>
        ///     Panels of one kind (wing panels, or control surfaces) that touch one another in about the same plane: one
        ///     wing. Shielded panels make no lift and are left out. Positions and directions are in Frame, the vessel's root
        ///     part, so that they hold as the vessel turns and moves.
        /// </summary>
        private sealed class WingGroup
        {
            // panels join while their normals are within 30 deg
            private const float MinCos = 0.866f;

            public sealed class Panel
            {
                public FARWingAerodynamicModel Wing;
                // FAR's planform, which the wing's span and area are measured on
                public readonly Vector3[] Corners = new Vector3[4];
                // the part's model seen from above its plane (convex hull, world), which touching is tested on
                public Vector3[] Outline;
                public Vector3 Center, Normal, Right;
                public float Chord;
            }

            public readonly List<FARWingAerodynamicModel> Members = new List<FARWingAerodynamicModel>();
            private readonly List<Panel> panels = new List<Panel>();
            public int Generation;
            // the vessel's parts when the group was built: the panels between a root and its mirror are looked for there
            private List<Part> vesselParts;
            public Transform Frame;
            public int PanelCount => panels.Count;
            public FARWingAerodynamicModel PanelWing(int i) => panels[i].Wing;

            public static Transform FrameOf(Part part) =>
                part.localRoot != null ? part.localRoot.partTransform : part.partTransform;
            public double B2, Area, ExposureMin, ExposureMax;
            // whether a body holds the wing's root, and then its span from tip to the mirror tip across that body (m)
            public bool RootOnBody;
            public double TipToTip;
            // whether the wing takes in a panel's mirror image: it spans the centerline
            public bool Straddles;
            // the area of the control surfaces on this wing (m^2), and, for a wing on a bare body, of the strip of the
            // body between its root and the body's center at the root chord (m^2): both part of the wing's planform
            public double ControlArea, BodyStrip;
            private bool controlAreaFound;
            // the tangent of the whole wing's half-chord sweep, from its mid-chord line fitted across the span
            public double HalfChordTan;

            /// <summary>
            ///     For a wing on a bare body, how much more lift the wing makes than its own panels would alone: the wing and
            ///     the body together lift as the whole wing through the body (slender-body theory; the vortex lattice with the
            ///     body bridged and the NACA wing-body tests agree), so the panels carry the strip of body between the roots
            ///     too. 1 otherwise. Call after Measure.
            /// </summary>
            public double BodyLiftFactor => RootOnBody ? (Area + ControlArea + BodyStrip) / (Area + ControlArea) : 1;
            private Vector3 measuredFlow;
            private bool measured;

            public static void Build(FARWingAerodynamicModel start, List<Part> parts)
            {
                var g = new WingGroup { Generation = wingGroupGeneration, Frame = FrameOf(start.part), vesselParts = parts };
                Panel first = MakePanel(start, g.Frame);
                if (first == null || start.isShielded)
                {
                    g.Members.Add(start);
                    start.WingInteraction.wingGroup = g;
                    return;
                }

                bool control = start is FARControllableSurface;
                var candidates = new List<Panel>();
                foreach (Part p in parts)
                {
                    if (p == null || p == start.part)
                        continue;
                    FARWingAerodynamicModel w = p.FindModuleImplementing<FARWingAerodynamicModel>();
                    if (w == null || w.WingInteraction == null || w.isShielded || w is FARControllableSurface != control)
                        continue;
                    if (FrameOf(p) != g.Frame)
                        continue;
                    Panel c = MakePanel(w, g.Frame);
                    if (c != null)
                        candidates.Add(c);
                }

                // every panel reachable through touching panels
                g.panels.Add(first);
                for (int i = 0; i < g.panels.Count; i++)
                    for (int j = candidates.Count - 1; j >= 0; j--)
                        if (Touch(g.panels[i], candidates[j]))
                        {
                            g.panels.Add(candidates[j]);
                            candidates.RemoveAt(j);
                        }

                var memberParts = new HashSet<Part>();
                foreach (Panel p in g.panels)
                {
                    g.Members.Add(p.Wing);
                    memberParts.Add(p.Wing.part);
                    p.Wing.WingInteraction.wingGroup = g;
                }

                foreach (Panel p in g.panels)
                    foreach (Part mirror in p.Wing.part.symmetryCounterparts)
                        if (memberParts.Contains(mirror))
                            g.Straddles = true;
            }

            public static Panel MakePanel(FARWingAerodynamicModel w, Transform frame)
            {
                if (w == null || w.WingInteraction == null || w.MAC_actual <= 0 || w.b_2_actual <= 0)
                    return null;
                var p = new Panel { Wing = w };
                w.PlanformCorners(p.Corners);
                for (int k = 0; k < 4; k++)
                    p.Corners[k] = frame.InverseTransformPoint(p.Corners[k]);
                Vector3 normal = w.part.partTransform.forward;
                p.Normal = frame.InverseTransformDirection(normal);
                p.Right = frame.InverseTransformDirection(w.part.partTransform.right);
                p.Chord = (float)w.MAC_actual;
                Vector3[] outline = ModelOutline(w.part, normal);
                if (outline != null)
                    for (int k = 0; k < outline.Length; k++)
                        outline[k] = frame.InverseTransformPoint(outline[k]);
                p.Outline = outline ?? p.Corners;
                p.Center = Vector3.zero;
                foreach (Vector3 c in p.Outline)
                    p.Center += c;
                p.Center /= p.Outline.Length;
                return p;
            }

            /// <summary>
            ///     The convex hull of a part's model meshes projected on its plane (world): what the panel covers as the
            ///     player sees it. FAR's planform for a stock part can fall short of the mesh, and a stock wing's collider is
            ///     a simpler shape than its mesh; either can leave a gap between panels that overlap. Null without a readable
            ///     mesh.
            /// </summary>
            private static Vector3[] ModelOutline(Part part, Vector3 normal)
            {
                Transform t = part.partTransform;
                Vector3 origin = t.position, e1 = t.right, e2 = Vector3.Cross(normal, e1).normalized;
                var points = new List<Vector2>();
                foreach (MeshFilter mf in part.FindModelComponents<MeshFilter>())
                {
                    if (mf == null || mf.sharedMesh == null || !mf.sharedMesh.isReadable)
                        continue;
                    foreach (Vector3 v in mf.sharedMesh.vertices)
                    {
                        Vector3 d = mf.transform.TransformPoint(v) - origin;
                        points.Add(new Vector2(Vector3.Dot(d, e1), Vector3.Dot(d, e2)));
                    }
                }

                if (points.Count < 3)
                    return null;
                List<Vector2> hull = ConvexHull(points);
                if (hull.Count < 3)
                    return null;
                var outline = new Vector3[hull.Count];
                for (int i = 0; i < hull.Count; i++)
                    outline[i] = origin + hull[i].x * e1 + hull[i].y * e2;
                return outline;
            }

            /// <summary>Andrew's monotone chain: the convex hull, counterclockwise</summary>
            private static List<Vector2> ConvexHull(List<Vector2> points)
            {
                points.Sort((a, b) => a.x != b.x ? a.x.CompareTo(b.x) : a.y.CompareTo(b.y));
                var hull = new List<Vector2>(2 * points.Count);

                float Cross(Vector2 o, Vector2 a, Vector2 b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);

                foreach (Vector2 p in points)
                {
                    while (hull.Count >= 2 && Cross(hull[hull.Count - 2], hull[hull.Count - 1], p) <= 0)
                        hull.RemoveAt(hull.Count - 1);
                    hull.Add(p);
                }

                int lower = hull.Count + 1;
                for (int i = points.Count - 2; i >= 0; i--)
                {
                    while (hull.Count >= lower && Cross(hull[hull.Count - 2], hull[hull.Count - 1], points[i]) <= 0)
                        hull.RemoveAt(hull.Count - 1);
                    hull.Add(points[i]);
                }

                hull.RemoveAt(hull.Count - 1);
                return hull;
            }

            /// <summary>Whether a panel (a control surface) touches any panel of this wing</summary>
            public bool Touches(Panel surface)
            {
                foreach (Panel p in panels)
                    if (Touch(p, surface))
                        return true;
                return false;
            }

            /// <summary>
            ///     Whether two panels touch: about parallel, and their parts' outlines meeting, within a small gap across
            ///     and a small step in height. Panels that overlap at about one height touch; so do panels that meet at a
            ///     bend (dihedral), measured where they meet; a layer stacked above or below another does not.
            /// </summary>
            private static bool Touch(Panel a, Panel b)
            {
                if (Math.Abs(Vector3.Dot(a.Normal, b.Normal)) < MinCos)
                    return false;
                float chord = Math.Min(a.Chord, b.Chord);
                float height = 0.2f + 0.25f * chord;
                const float gap = 0.05f;

                // overlapping in a's plane, centers at about one height
                Vector3 d = b.Center - a.Center;
                if (Math.Abs(Vector3.Dot(d, a.Normal)) <= height && Math.Abs(Vector3.Dot(d, b.Normal)) <= height &&
                    !Separated(a.Outline, a, b, gap) && !Separated(b.Outline, a, b, gap))
                    return true;

                // meeting at an edge or a corner, at about one height there
                return Step(a, b, gap) <= height || Step(b, a, gap) <= height;
            }

            /// <summary>
            ///     The smallest height above or below b's plane of a point on a's outline (its corners and points along its
            ///     edges) that lies over b's outline, within gap of it
            /// </summary>
            private static float Step(Panel a, Panel b, float gap)
            {
                float best = float.MaxValue;
                Vector3[] o = a.Outline;
                for (int i = 0; i < o.Length; i++)
                {
                    Vector3 p0 = o[i], p1 = o[(i + 1) % o.Length];
                    for (int k = 0; k < 4; k++)
                    {
                        Vector3 q = Vector3.Lerp(p0, p1, 0.25f * k);
                        float h = Vector3.Dot(q - b.Center, b.Normal);
                        if (Math.Abs(h) < best && DistanceInPlane(q - h * b.Normal, b) <= gap)
                            best = Math.Abs(h);
                    }
                }

                return best;
            }

            /// <summary>How far a point in b's plane lies outside b's outline (0 inside)</summary>
            private static float DistanceInPlane(Vector3 q, Panel b)
            {
                Vector3[] o = b.Outline;
                bool left = false, right = false;
                float nearest = float.MaxValue;
                for (int i = 0; i < o.Length; i++)
                {
                    Vector3 p0 = o[i], edge = o[(i + 1) % o.Length] - p0;
                    // inside a convex outline the point is on the same side of every edge, whichever way it runs
                    float side = Vector3.Dot(Vector3.Cross(edge, q - p0), b.Normal);
                    left |= side > 0;
                    right |= side < 0;
                    float t = Mathf.Clamp01(Vector3.Dot(q - p0, edge) / Mathf.Max(edge.sqrMagnitude, 1e-8f));
                    nearest = Math.Min(nearest, (p0 + t * edge - q).magnitude);
                }

                return left && right ? nearest : 0;
            }

            private static bool Separated(Vector3[] edges, Panel a, Panel b, float gap)
            {
                for (int i = 0; i < edges.Length; i++)
                {
                    Vector3 edge = edges[(i + 1) % edges.Length] - edges[i];
                    Vector3 axis = Vector3.Cross(a.Normal, edge);
                    if (axis.sqrMagnitude < 1e-8f)
                        continue;
                    axis.Normalize();
                    Range(a.Outline, axis, out float aMin, out float aMax);
                    Range(b.Outline, axis, out float bMin, out float bMax);
                    if (bMin - aMax > gap || aMin - bMax > gap)
                        return true;
                }

                return false;
            }

            private static void Range(Vector3[] corners, Vector3 axis, out float min, out float max)
            {
                min = max = Vector3.Dot(corners[0], axis);
                for (int i = 1; i < corners.Length; i++)
                {
                    float x = Vector3.Dot(corners[i], axis);
                    if (x < min)
                        min = x;
                    else if (x > max)
                        max = x;
                }
            }

            /// <summary>
            ///     The wing's semispan across the flow, its area (overlaps counted once), and the exposures of its two
            ///     ends, for a flow direction (world); false if the flow is about normal to the wing, or the frame is gone
            /// </summary>
            public bool Measure(Vector3 flow)
            {
                if (Frame == null)
                    return false;
                flow = Frame.InverseTransformDirection(flow);
                Vector3 n = panels[0].Normal;
                Vector3 f = Vector3.ProjectOnPlane(flow, n);
                if (f.sqrMagnitude < 1e-6f)
                    return false;
                f.Normalize();
                if (measured && Vector3.Dot(f, measuredFlow) > 0.9998f)
                    return true;
                Vector3 s = Vector3.Cross(n, f).normalized;

                // the planforms in (along the flow, across it)
                int count = panels.Count;
                var u = new float[count, 4];
                var v = new float[count, 4];
                var stations = new List<float>(4 * count);
                double trapezoidArea = 0, farArea = 0;
                for (int i = 0; i < count; i++)
                {
                    for (int k = 0; k < 4; k++)
                    {
                        u[i, k] = Vector3.Dot(panels[i].Corners[k], f);
                        v[i, k] = Vector3.Dot(panels[i].Corners[k], s);
                        stations.Add(v[i, k]);
                    }

                    double a = 0;
                    for (int k = 0; k < 4; k++)
                        a += u[i, k] * v[i, (k + 1) % 4] - u[i, (k + 1) % 4] * v[i, k];
                    trapezoidArea += Math.Abs(0.5 * a);
                    farArea += panels[i].Wing.S;
                }

                stations.Sort();
                float vMin = stations[0], vMax = stations[stations.Count - 1];

                // the chord the panels cover together at stations across the flow
                double span = 0, area = 0, chordAtMin = 0, chordAtMax = 0;
                double sw = 0, swv = 0, swu = 0, swvv = 0, swvu = 0;
                var lo = new List<float>(count);
                var hi = new List<float>(count);
                for (int j = 1; j < stations.Count; j++)
                {
                    float w = stations[j] - stations[j - 1];
                    if (w < 1e-4f)
                        continue;
                    const int steps = 4;
                    for (int q = 0; q < steps; q++)
                    {
                        float at = stations[j - 1] + (q + 0.5f) * w / steps;
                        lo.Clear();
                        hi.Clear();
                        for (int i = 0; i < count; i++)
                        {
                            float min = float.MaxValue, max = float.MinValue;
                            for (int k = 0; k < 4; k++)
                            {
                                int k2 = (k + 1) % 4;
                                float v1 = v[i, k], v2 = v[i, k2];
                                if ((v1 - at) * (v2 - at) > 0 || v1 == v2)
                                    continue;
                                float x = u[i, k] + (at - v1) / (v2 - v1) * (u[i, k2] - u[i, k]);
                                min = Math.Min(min, x);
                                max = Math.Max(max, x);
                            }

                            if (max > min)
                            {
                                lo.Add(min);
                                hi.Add(max);
                            }
                        }

                        double chord = MergedLength(lo, hi);
                        if (chord <= 0)
                            continue;
                        if (span <= 0)
                            chordAtMin = chord;
                        chordAtMax = chord;
                        span += w / steps;
                        area += chord * w / steps;
                        // the mid-chord line, fitted across the span weighted by chord
                        double mid = 0.5 * (lo.Min() + hi.Max()), wt = chord * w / steps;
                        sw += wt;
                        swv += wt * at;
                        swu += wt * mid;
                        swvv += wt * at * at;
                        swvu += wt * at * mid;
                    }
                }

                if (span <= 0 || area <= 0)
                    return false;
                double det = sw * swvv - swv * swv;
                HalfChordTan = det > 1e-9 ? Math.Abs((sw * swvu - swv * swu) / det) : 0;
                // FAR's panel area is semispan times MAC, a little more than a tapered panel's planform
                area *= farArea / Math.Max(trapezoidArea, 1e-6);
                B2 = span;
                Area = area;
                ExposureMin = EndExposure(v, vMin, -s, -vMin, span, out Part bodyMin);
                ExposureMax = EndExposure(v, vMax, s, vMax, span, out Part bodyMax);
                // the root is the end a body holds (the one farther from it, when a pod also holds the tip); the mirror
                // wing's tip lies as far beyond the body's center
                RootOnBody = false;
                TipToTip = span;
                BodyStrip = 0;
                if (!controlAreaFound)
                {
                    controlAreaFound = true;
                    ControlArea = FindControlArea();
                }

                if (!Straddles && (bodyMin != null || bodyMax != null))
                {
                    float mid = 0.5f * (vMin + vMax);
                    float cMin = bodyMin == null ? mid : Vector3.Dot(Frame.InverseTransformPoint(bodyMin.partTransform.position), s);
                    float cMax = bodyMax == null ? mid : Vector3.Dot(Frame.InverseTransformPoint(bodyMax.partTransform.position), s);
                    bool rootAtMin = Math.Abs(cMin - mid) >= Math.Abs(cMax - mid);
                    float c = rootAtMin ? cMin : cMax;
                    double tipToTip = 2 * Math.Abs((rootAtMin ? vMax : vMin) - c);
                    if (tipToTip > span && !LiftingBetween(rootAtMin ? vMin : vMax, c, n, f, s, u, v))
                    {
                        RootOnBody = true;
                        TipToTip = tipToTip;
                        BodyStrip = (rootAtMin ? chordAtMin : chordAtMax) * (0.5 * tipToTip - span);
                    }
                }
                measuredFlow = f;
                measured = true;
                return true;
            }

            /// <summary>The area of the control surfaces that touch this wing, in its frame (m^2)</summary>
            private double FindControlArea()
            {
                double sum = 0;
                if (vesselParts == null || panels.Count == 0 || panels[0].Wing is FARControllableSurface)
                    return 0;
                foreach (Part p in vesselParts)
                {
                    if (p == null || FrameOf(p) != Frame)
                        continue;
                    FARWingAerodynamicModel w = p.FindModuleImplementing<FARWingAerodynamicModel>();
                    if (!(w is FARControllableSurface) || w.isShielded)
                        continue;
                    Panel c = MakePanel(w, Frame);
                    if (c != null && Touches(c))
                        sum += w.S;
                }

                return sum;
            }

            /// <summary>
            ///     The wing's aspect ratio, its span squared over its planform with the control surfaces on it included. A
            ///     wing spanning the centerline is a whole wing. A wing on a bare body is the whole wing through the body:
            ///     its span from tip to tip, the strip of body between the roots in its area. Otherwise it is a half wing
            ///     whose covered root (the mirror half, another wing) doubles it. A covered tip (a pod, a winglet) is an
            ///     endplate, worth a little more. Call after Measure.
            /// </summary>
            public double AspectRatio()
            {
                double area = Area + ControlArea;
                double coveredMin = (1 - ExposureMin).Clamp(0, 1), coveredMax = (1 - ExposureMax).Clamp(0, 1);
                if (Straddles)
                    return B2 * B2 / area * (1 + 0.25 * (coveredMin + coveredMax));
                double rootCovered = Math.Max(coveredMin, coveredMax), tipCovered = Math.Min(coveredMin, coveredMax);
                if (RootOnBody)
                    return TipToTip * TipToTip / (2 * (area + BodyStrip)) * (1 + 0.25 * tipCovered);
                return B2 * B2 / area * (1 + rootCovered + 0.5 * tipCovered);
            }

            /// <summary>
            ///     Whether another lifting panel in about this wing's plane lies between its root (at root, across the
            ///     flow) and the mirror root beyond the body's center c, over the root's chord: a strake or a lifting
            ///     center section, which carries the wing across itself; only a bare body between the roots makes the wing
            ///     and its mirror one wing through the body
            /// </summary>
            private bool LiftingBetween(float root, float c, Vector3 n, Vector3 f, Vector3 s, float[,] u, float[,] v)
            {
                // the root's chord along the flow: the panels at the root end
                float near = 0.02f * (float)B2 + 0.05f, uLo = float.MaxValue, uHi = float.MinValue;
                for (int i = 0; i < panels.Count; i++)
                    for (int k = 0; k < 4; k++)
                        if (Math.Abs(v[i, k] - root) < near)
                            for (int m = 0; m < 4; m++)
                            {
                                uLo = Math.Min(uLo, u[i, m]);
                                uHi = Math.Max(uHi, u[i, m]);
                            }
                if (uHi <= uLo || vesselParts == null)
                    return false;
                float half = Math.Abs(root - c) - 0.05f;
                var members = new HashSet<FARWingAerodynamicModel>(Members);
                var corners = new Vector3[4];
                foreach (Part p in vesselParts)
                {
                    FARWingAerodynamicModel w = p == null ? null : p.FindModuleImplementing<FARWingAerodynamicModel>();
                    if (w == null || w.isShielded || members.Contains(w) || w.MAC_actual <= 0)
                        continue;
                    if (Math.Abs(Vector3.Dot(Frame.InverseTransformDirection(p.partTransform.forward), n)) < 0.7f)
                        continue;
                    w.PlanformCorners(corners);
                    float pvLo = float.MaxValue, pvHi = float.MinValue, puLo = float.MaxValue, puHi = float.MinValue;
                    foreach (Vector3 corner in corners)
                    {
                        Vector3 q = Frame.InverseTransformPoint(corner);
                        float pv = Vector3.Dot(q, s) - c, pu = Vector3.Dot(q, f);
                        pvLo = Math.Min(pvLo, pv);
                        pvHi = Math.Max(pvHi, pv);
                        puLo = Math.Min(puLo, pu);
                        puHi = Math.Max(puHi, pu);
                    }
                    if (pvHi > -half && pvLo < half && puHi > uLo && puLo < uHi)
                        return true;
                }
                return false;
            }

            private static double MergedLength(List<float> lo, List<float> hi)
            {
                int count = lo.Count;
                if (count == 0)
                    return 0;
                // insertion sort by start: a handful of panels at most
                for (int i = 1; i < count; i++)
                    for (int j = i; j > 0 && lo[j] < lo[j - 1]; j--)
                    {
                        float t = lo[j];
                        lo[j] = lo[j - 1];
                        lo[j - 1] = t;
                        t = hi[j];
                        hi[j] = hi[j - 1];
                        hi[j - 1] = t;
                    }

                double length = 0;
                float start = lo[0], end = hi[0];
                for (int i = 1; i < count; i++)
                {
                    if (lo[i] > end)
                    {
                        length += end - start;
                        start = lo[i];
                        end = hi[i];
                    }
                    else if (hi[i] > end)
                        end = hi[i];
                }

                return length + end - start;
            }

            /// <summary>
            ///     How exposed one end of the wing is (1: a free tip, 0: against a body): a part that is not a wing, attached
            ///     to any panel of the wing and lying out at that end, covers it (the fuselage a wing root is attached to,
            ///     whichever panel holds it; a pod at a tip); otherwise the side rays of the panels at that end that point
            ///     along the span
            /// </summary>
            /// <param name="end">Where the end is across the flow</param>
            /// <param name="outward">Out of the wing at that end</param>
            /// <param name="endOutward">Where the end is along outward</param>
            private double EndExposure(float[,] v, float end, Vector3 outward, float endOutward, double span, out Part body)
            {
                // the body's center lies beyond a root, or within this much of the end: a strake's planform can reach
                // further inboard than the panel that holds the wing to the fuselage
                float bodyReach = 0.15f * (float)span;
                foreach (Panel p in panels)
                    if ((body = BodyBeyond(p.Wing.part, outward, endOutward - bodyReach)) != null)
                        return 0;
                body = null;

                float near = 0.02f * (float)span + 0.05f;
                double sum = 0;
                int rays = 0;
                for (int i = 0; i < panels.Count; i++)
                {
                    float reach = Math.Abs(v[i, 0] - end);
                    for (int k = 1; k < 4; k++)
                        reach = Math.Min(reach, Math.Abs(v[i, k] - end));
                    if (reach > near)
                        continue;
                    Panel p = panels[i];
                    float along = Vector3.Dot(p.Right, outward);
                    if (Math.Abs(along) < 0.7f)
                        continue;
                    FARWingInteraction wi = p.Wing.WingInteraction;
                    sum += along > 0 ? wi.rightwardExposure : wi.leftwardExposure;
                    rays++;
                }

                return rays > 0 ? sum / rays : 1;
            }

            /// <summary>
            ///     The part that is not a wing, attached to a panel, that reaches past a point along outward: the fuselage at
            ///     a wing root, a pod at a tip. Null if none.
            /// </summary>
            private Part BodyBeyond(Part panel, Vector3 outward, float from)
            {
                bool Beyond(Part p) =>
                    p != null && p.FindModuleImplementing<FARWingAerodynamicModel>() == null &&
                    Vector3.Dot(Frame.InverseTransformPoint(p.partTransform.position), outward) > from;

                if (Beyond(panel.parent))
                    return panel.parent;
                foreach (Part child in panel.children)
                    if (Beyond(child))
                        return child;
                return null;
            }
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
