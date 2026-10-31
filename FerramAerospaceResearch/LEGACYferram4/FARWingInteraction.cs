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
using System.Threading;
using FerramAerospaceResearch;
using FerramAerospaceResearch.Threading;
using UnityEngine;

// ReSharper disable once CheckNamespace
namespace ferram4
{
    public class FARWingInteraction
    {
        private static readonly RaycastHitComparer _comparer = new RaycastHitComparer();


        private static FloatCurve wingCamberFactor;
        private static FloatCurve wingCamberMoment;

        // The camber curves as every slot evaluates them: exact copies any thread can evaluate (see ExactCurve).
        private static volatile ExactCurve wingCamberFactorExact;
        private static volatile ExactCurve wingCamberMomentExact;
        private static readonly object camberLock = new object();

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

            GenerateCamberCurves();
        }

        /// <summary>Builds the camber curves and their exact copies once. Harmless to call again.</summary>
        public static void GenerateCamberCurves()
        {
            if (wingCamberMomentExact != null)
                return;
            lock (camberLock)
            {
                if (wingCamberMomentExact == null)
                    BuildCamberCurves();
            }
        }

        private static void BuildCamberCurves()
        {
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

            if (wingCamberMoment == null)
            {
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

            wingCamberFactorExact = new ExactCurve(wingCamberFactor);
            wingCamberMomentExact = new ExactCurve(wingCamberMoment);
        }

        // Per-call mutable state lives in _scratch so multiple threads can evaluate
        // independent (Mach, altitude) cells against the same FARWingInteraction
        // instance using their own scratch. Thread-local so each worker in the parallel
        // sweep gets its own; the single-threaded paths use the calling (main) thread's.
        // Public properties delegate to scratch.
        // Per-thread-slot scratch (see SimThreadContext) - an array index is much cheaper than
        // ThreadLocal.Value under Mono. Each slot is single-threaded, so the lazy create is safe.
        private readonly WingInteractionScratch[] _scratchSlots =
            new WingInteractionScratch[SimThreadContext.MaxSlots];

        /// <summary>
        /// Starts the calling thread's slot (a prediction's) from the flight's, slot 0's. The wing's lift slope reads
        /// ARFactor before the orientation update refreshes it, so a stale slot would carry the last call into this one.
        /// </summary>
        internal void BeginPredictionState()
        {
            int s = SimThreadContext.Slot;
            WingInteractionScratch sc = _scratchSlots[s];
            if (sc == null)
                _scratchSlots[s] = sc = new WingInteractionScratch();
            sc.CopyFrom(_scratchSlots[0] ?? new WingInteractionScratch());
        }

        /// <summary>A copy of the flight's state (slot 0's), for a vessel snapshot. Main thread.</summary>
        internal WingInteractionScratch CaptureSeed()
        {
            var seed = new WingInteractionScratch();
            seed.CopyFrom(_scratchSlots[0] ?? new WingInteractionScratch());
            return seed;
        }

        /// <summary>Starts the calling thread's slot (a snapshot prediction's) from the captured flight state.</summary>
        internal void InstallSeed(WingInteractionScratch seed)
        {
            int s = SimThreadContext.Slot;
            WingInteractionScratch sc = _scratchSlots[s];
            if (sc == null)
                _scratchSlots[s] = sc = new WingInteractionScratch();
            sc.CopyFrom(seed ?? new WingInteractionScratch());
        }

        internal WingInteractionScratch _scratch
        {
            get
            {
                int s = SimThreadContext.Slot;
                WingInteractionScratch sc = _scratchSlots[s];
                if (sc == null)
                {
                    sc = new WingInteractionScratch();
                    _scratchSlots[s] = sc;
                }

                return sc;
            }
        }

        public double EffectiveUpstreamMAC => _scratch.EffectiveUpstreamMAC;
        public double EffectiveUpstreamb_2 => _scratch.EffectiveUpstreamb_2;
        public double EffectiveUpstreamLiftSlope => _scratch.EffectiveUpstreamLiftSlope;
        public double EffectiveUpstreamArea => _scratch.EffectiveUpstreamArea;
        public double EffectiveUpstreamStall => _scratch.EffectiveUpstreamStall;
        public double EffectiveUpstreamCosSweepAngle => _scratch.EffectiveUpstreamCosSweepAngle;
        public double EffectiveUpstreamAoAMax => _scratch.EffectiveUpstreamAoAMax;
        public double EffectiveUpstreamAoA => _scratch.EffectiveUpstreamAoA;
        public double EffectiveUpstreamCd0 => _scratch.EffectiveUpstreamCd0;
        public double EffectiveUpstreamInfluence => _scratch.EffectiveUpstreamInfluence;
        public bool HasWingsUpstream => _scratch.HasWingsUpstream;

        public double ARFactor => _scratch.ARFactor;

        // Set from the neighboring parts when the vessel changes (UpdateWingInteraction), not per evaluation, so
        // it is shared by every slot; in the scratch, slots other than 0 read the default of 1.
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
            _scratch.EffectiveUpstreamLiftSlope = 0;
            _scratch.EffectiveUpstreamStall = 0;
            _scratch.EffectiveUpstreamCosSweepAngle = 0;
            _scratch.EffectiveUpstreamAoAMax = 0;
            _scratch.EffectiveUpstreamAoA = 0;
            _scratch.EffectiveUpstreamCd0 = 0;
            _scratch.EffectiveUpstreamInfluence = 0;
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

            _scratch.EffectiveUpstreamMAC = 0;
            _scratch.EffectiveUpstreamb_2 = 0;
            _scratch.EffectiveUpstreamArea = 0;

            _scratch.EffectiveUpstreamLiftSlope = 0;
            _scratch.EffectiveUpstreamStall = 0;
            _scratch.EffectiveUpstreamCosSweepAngle = 0;
            _scratch.EffectiveUpstreamAoAMax = 0;
            _scratch.EffectiveUpstreamAoA = 0;
            _scratch.EffectiveUpstreamCd0 = 0;
            _scratch.EffectiveUpstreamInfluence = 0;

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
            double flapFactor = wingCamberFactorExact.Evaluate(flt_flapRatio);
            //Change in moment due to change in lift from flap
            double dCm_dCl = wingCamberMomentExact.Evaluate(flt_flapRatio);

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

            _scratch.ARFactor = CalculateARFactor(wingForwardDir, wingRightwardDir);
            _scratch.HasWingsUpstream = DetermineWingsUpstream(wingForwardDir, wingRightwardDir);
        }

        private void UpdateUpstreamValuesFromWingModules(
            List<FARWingAerodynamicModel> wingModules,
            List<double> associatedInfluences,
            double directionalInfluence,
            double thisWingAoA
        )
        {
            directionalInfluence = Math.Abs(directionalInfluence);

            // Only the flight and the serial editor tabs (slot 0, main thread) prune a destroyed neighbour from the
            // lists. Any other slot - a sweep worker, a prediction, possibly on another thread - reads the lists
            // other threads may be reading too, so it leaves them alone: it skips the destroyed neighbours and
            // scales the others' influence by 1 / their sum, which is what pruning does.
            bool prune = SimThreadContext.Slot == 0 && MainThread.IsMainThread;
            double influenceScale = prune ? 1 : LiveInfluenceScale(wingModules, associatedInfluences);

            for (int i = 0; i < wingModules.Count; i++)
            {
                FARWingAerodynamicModel wingModule = wingModules[i];

                if (prune)
                {
                    if (wingModule == null)
                    {
                        // A snapshot prediction on another thread may be reading the lists.
                        parentWingModule.BumpGeometryVersion();
                        HandleNullPart(wingModules, associatedInfluences, i);
                        parentWingModule.BumpGeometryVersion();
                        i--;
                        continue;
                    }
                }
                else if (IsGone(wingModule))
                {
                    continue;
                }

                double wingInfluenceFactor = prune
                                                 ? associatedInfluences[i] * directionalInfluence
                                                 : associatedInfluences[i] * influenceScale * directionalInfluence;

                if (wingModule.GetShielded())
                    continue;

                Vector3 wingForward = wingModule.SolveForward;
                Vector3 parentForward = parentWingModule.SolveForward;
                double tmp = Vector3.Dot(wingForward, parentForward);

                // the neighbour's values on this slot (its GetMAC, Getb_2, ... read the same scratch)
                WingScratch neighbour = wingModule._scratch;
                _scratch.EffectiveUpstreamMAC += neighbour.effective_MAC * wingInfluenceFactor;
                _scratch.EffectiveUpstreamb_2 += neighbour.effective_b_2 * wingInfluenceFactor;
                _scratch.EffectiveUpstreamArea += wingModule.S * wingInfluenceFactor;

                _scratch.EffectiveUpstreamLiftSlope += neighbour.rawLiftSlope * wingInfluenceFactor;
                _scratch.EffectiveUpstreamStall += neighbour.stall * wingInfluenceFactor;
                _scratch.EffectiveUpstreamCosSweepAngle += neighbour.cosSweepAngle * wingInfluenceFactor;
                _scratch.EffectiveUpstreamAoAMax += neighbour.rawAoAmax * wingInfluenceFactor;
                _scratch.EffectiveUpstreamCd0 += neighbour.zeroLiftCdIncrement * wingInfluenceFactor;
                _scratch.EffectiveUpstreamInfluence += wingInfluenceFactor;

                double wAoA = wingModule.CalculateAoA(wingModule.GetVelocity()) * Math.Sign(tmp);
                //First, make sure that the AoA are wrt the same direction; then account for any strange angling of the part that shouldn't be there
                tmp = (thisWingAoA - wAoA) * wingInfluenceFactor;

                _scratch.EffectiveUpstreamAoA += tmp;
            }
        }

        /// <summary>
        ///     A neighbour destroyed since the lists were built. Unity's destroyed-object check is a main-thread call;
        ///     off the main thread only a missing reference counts.
        /// </summary>
        private static bool IsGone(FARWingAerodynamicModel wingModule)
        {
            if (ReferenceEquals(wingModule, null))
                return true;
            // A snapshot prediction takes the wings captured with the snapshot, on whichever thread it runs.
            if (SimThreadContext.SnapshotStamp != 0)
                return !wingModule.CapturedForSolve;
            return MainThread.IsMainThread && wingModule == null;
        }

        /// <summary>
        ///     The factor pruning the destroyed neighbours would renormalize the rest by (1 / the sum of their
        ///     influences), or exactly 1 when none is destroyed.
        /// </summary>
        private static double LiveInfluenceScale(List<FARWingAerodynamicModel> wingModules, List<double> associatedInfluences)
        {
            bool anyGone = false;
            double sum = 0;
            for (int i = 0; i < wingModules.Count; i++)
                if (IsGone(wingModules[i]))
                    anyGone = true;
                else
                    sum += associatedInfluences[i];
            return anyGone ? 1 / sum : 1;
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
