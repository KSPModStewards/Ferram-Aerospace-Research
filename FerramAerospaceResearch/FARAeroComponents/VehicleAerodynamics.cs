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
using ferram4;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.FARPartGeometry.GeometryModification;
using FerramAerospaceResearch.FARThreading;
using KSPCommunityFixes;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    internal class VehicleAerodynamics
    {
        private static double[] indexSqrt = new double[1];
        private static readonly object _commonLocker = new object();
        private static Stack<FARAeroSection> currentlyUnusedSections;

        private readonly Dictionary<Part, PartTransformInfo> _partWorldToLocalMatrixDict =
            new Dictionary<Part, PartTransformInfo>(ObjectReferenceEqualityComparer<Part>.Default);

        private readonly Dictionary<FARAeroPartModule, FARAeroPartModule.ProjectedArea> _moduleAndAreasDict =
            new Dictionary<FARAeroPartModule, FARAeroPartModule.ProjectedArea>(ObjectReferenceEqualityComparer<
                                                                                   FARAeroPartModule>.Default);

        private readonly List<FARAeroPartModule> includedModules = new List<FARAeroPartModule>();
        private readonly List<float> weighting = new List<float>();

        private VehicleVoxel _voxel;
        private VoxelCrossSection[] _vehicleCrossSection = Array.Empty<VoxelCrossSection>();
        private double[] _ductedAreaAdjustment = new double[1];

        private int _voxelCount;

        private double _maxCrossSectionArea;

        private Matrix4x4 _worldToLocalMatrix, _localToWorldMatrix;

        private Vector3d _voxelLowerRightCorner;
        private double _voxelElementSize;
        private double _sectionThickness;

        private Vector3 _vehicleMainAxis;
        private List<Part> _vehiclePartList;

        private List<GeometryPartModule> _currentGeoModules;

        private List<FARAeroPartModule> _currentAeroModules = new List<FARAeroPartModule>();
        private List<FARAeroPartModule> _newAeroModules = new List<FARAeroPartModule>();

        private List<FARAeroPartModule> _currentUnusedAeroModules = new List<FARAeroPartModule>();
        private List<FARAeroPartModule> _newUnusedAeroModules = new List<FARAeroPartModule>();

        private List<FARAeroSection> _currentAeroSections = new List<FARAeroSection>();
        private List<FARAeroSection> _newAeroSections = new List<FARAeroSection>();

        private List<FARWingAerodynamicModel> _legacyWingModels = new List<FARWingAerodynamicModel>();

        private List<ICrossSectionAdjuster> activeAdjusters = new List<ICrossSectionAdjuster>();

        private int validSectionCount;
        private int firstSection;

        private bool visualizing;
        public bool Voxelizing { get; private set; }

        public Bounds VoxelBounds
        {
            get { return _voxel?.Bounds ?? new Bounds(); }
        }

        public VehicleAerodynamics()
        {
            if (currentlyUnusedSections == null)
                currentlyUnusedSections = new Stack<FARAeroSection>();
        }

        public double Length { get; private set; }

        public double MaxCrossSectionArea
        {
            get { return _maxCrossSectionArea; }
        }

        public bool CalculationCompleted { get; private set; }

        public double SonicDragArea { get; private set; }

        public double CriticalMach { get; private set; }

        public double SectionThickness
        {
            get { return _sectionThickness; }
        }

        public void ForceCleanup()
        {
            if (_voxel != null)
            {
                _voxel.CleanupVoxel();
                _voxel = null;
            }

            _vehicleCrossSection = null;
            _ductedAreaAdjustment = null;

            _currentAeroModules = null;
            _newAeroModules = null;

            _currentUnusedAeroModules = null;
            _newUnusedAeroModules = null;

            _currentAeroSections = null;
            _newAeroSections = null;

            _legacyWingModels = null;

            _vehiclePartList = null;

            activeAdjusters = null;
        }

        private static void GenerateIndexSqrtLookup(int numStations)
        {
            indexSqrt = new double[numStations];
            for (int i = 0; i < numStations; i++)
                indexSqrt[i] = Math.Sqrt(i);
        }

        //Used by other classes to update their aeroModule and aeroSection lists
        //When these functions fire, all the data that was once restricted to the voxelization thread is passed over to the main unity thread

        public void GetNewAeroData(
            out List<FARAeroPartModule> aeroModules,
            out List<FARAeroPartModule> unusedAeroModules,
            out List<FARAeroSection> aeroSections,
            out List<FARWingAerodynamicModel> legacyWingModel
        )
        {
            CalculationCompleted = false;
            List<FARAeroPartModule> tmpAeroModules = _currentAeroModules;
            aeroModules = _currentAeroModules = _newAeroModules;
            _newAeroModules = tmpAeroModules;

            List<FARAeroSection> tmpAeroSections = _currentAeroSections;
            aeroSections = _currentAeroSections = _newAeroSections;
            _newAeroSections = tmpAeroSections;

            tmpAeroModules = _currentUnusedAeroModules;
            unusedAeroModules = _currentUnusedAeroModules = _newUnusedAeroModules;
            _newUnusedAeroModules = tmpAeroModules;


            legacyWingModel = LEGACY_UpdateWingAerodynamicModels();
        }

        public void GetNewAeroData(out List<FARAeroPartModule> aeroModules, out List<FARAeroSection> aeroSections)
        {
            CalculationCompleted = false;
            List<FARAeroPartModule> tmpAeroModules = _currentAeroModules;
            aeroModules = _currentAeroModules = _newAeroModules;
            _newAeroModules = tmpAeroModules;

            List<FARAeroSection> tmpAeroSections = _currentAeroSections;
            aeroSections = _currentAeroSections = _newAeroSections;
            _newAeroSections = tmpAeroSections;

            tmpAeroModules = _currentUnusedAeroModules;
            _currentUnusedAeroModules = _newUnusedAeroModules;
            _newUnusedAeroModules = tmpAeroModules;

            LEGACY_UpdateWingAerodynamicModels();
        }

        private List<FARWingAerodynamicModel> LEGACY_UpdateWingAerodynamicModels()
        {
            _legacyWingModels.Clear();
            foreach (FARAeroPartModule aeroModule in _currentAeroModules)
            {
                Part p = aeroModule.part;
                if (!p)
                    continue;
                if (p.FindModuleImplementingFast<FARWingAerodynamicModel>() is FARWingAerodynamicModel w)
                {
                    w.isShielded = false;
                    w.NUFAR_ClearExposedAreaFactor();
                    _legacyWingModels.Add(w);
                }
                else if (p.FindModuleImplementingFast<FARControllableSurface>() is FARControllableSurface c)
                {
                    c.isShielded = false;
                    c.NUFAR_ClearExposedAreaFactor();
                    _legacyWingModels.Add(c);
                }
            }

            foreach (FARWingAerodynamicModel w in _legacyWingModels)
                w.NUFAR_CalculateExposedAreaFactor();

            foreach (FARWingAerodynamicModel w in _legacyWingModels)
                w.NUFAR_SetExposedAreaFactor();
            foreach (FARWingAerodynamicModel w in _legacyWingModels)
                w.NUFAR_UpdateShieldingStateFromAreaFactor();
            return _legacyWingModels;
        }

        //returns various data for use in displaying outside this class

        public Matrix4x4 VoxelAxisToLocalCoordMatrix()
        {
            return Matrix4x4.TRS(Vector3.zero, Quaternion.FromToRotation(_vehicleMainAxis, Vector3.up), Vector3.one);
        }

        public double FirstSectionXOffset()
        {
            double offset = Vector3d.Dot(_vehicleMainAxis, _voxelLowerRightCorner);
            offset += firstSection * _sectionThickness;

            return offset;
        }

        public double[] GetPressureCoeffs()
        {
            var pressureCoeffs = new double[validSectionCount];
            return GetPressureCoeffs(pressureCoeffs);
        }

        public double[] GetPressureCoeffs(double[] pressureCoeffs)
        {
            for (int i = firstSection; i < validSectionCount + firstSection; i++)
                pressureCoeffs[i - firstSection] = _vehicleCrossSection[i].cpSonicForward;

            return pressureCoeffs;
        }


        public double[] GetCrossSectionAreas()
        {
            var areas = new double[validSectionCount];
            return GetCrossSectionAreas(areas);
        }

        public double[] GetCrossSectionAreas(double[] areas)
        {
            for (int i = firstSection; i < validSectionCount + firstSection; i++)
                areas[i - firstSection] = _vehicleCrossSection[i].area;

            return areas;
        }

        public double[] GetCrossSection2ndAreaDerivs()
        {
            var areaDerivs = new double[validSectionCount];
            return GetCrossSection2ndAreaDerivs(areaDerivs);
        }

        public double[] GetCrossSection2ndAreaDerivs(double[] areaDerivs)
        {
            for (int i = firstSection; i < validSectionCount + firstSection; i++)
                areaDerivs[i - firstSection] = _vehicleCrossSection[i].secondAreaDeriv;

            return areaDerivs;
        }

        //Handling for display of debug voxels

        private void ClearDebugVoxel()
        {
            _voxel.ClearVisualVoxels();
            visualizing = false;
        }

        private void DisplayDebugVoxels(Matrix4x4 localToWorldMatrix)
        {
            _voxel.VisualizeVoxel(localToWorldMatrix);
            visualizing = true;
        }

        public void DebugVisualizeVoxels(Matrix4x4 localToWorldMatrix)
        {
            if (visualizing)
                ClearDebugVoxel();
            else
                DisplayDebugVoxels(localToWorldMatrix);
        }

        //This function will attempt to voxelize the vessel, as long as it isn't being voxelized currently all data that is on the Unity thread should be processed here before being passed to the other threads
        public bool TryVoxelUpdate(
            Matrix4x4 worldToLocalMatrix,
            Matrix4x4 localToWorldMatrix,
            int voxelCount,
            List<Part> vehiclePartList,
            List<GeometryPartModule> currentGeoModules,
            bool updateGeometryPartModules = true,
            Vessel vessel = null
        )
        {
            //set to true when this function ends; only continue to voxelizing if the voxelization thread has not been queued
            //this should catch conditions where this function is called again before the voxelization thread starts
            if (Voxelizing)
                return false;
            //only continue if the voxelizing thread has not locked this object
            if (!Monitor.TryEnter(this, 0))
                return false;
            try
            {
                //ensure that the main thread isn't going to try to read the updated section data while it is being worked with
                CalculationCompleted = false;
                //Bunch of voxel setup data
                _voxelCount = voxelCount;

                _worldToLocalMatrix = worldToLocalMatrix;
                _localToWorldMatrix = localToWorldMatrix;
                _vehiclePartList = vehiclePartList;
                _currentGeoModules = currentGeoModules;

                _partWorldToLocalMatrixDict.Clear();

                foreach (GeometryPartModule g in _currentGeoModules)
                {
                    _partWorldToLocalMatrixDict.Add(g.part, new PartTransformInfo(g.part.partTransform));
                    if (updateGeometryPartModules)
                        g.UpdateTransformMatrixList(_worldToLocalMatrix);
                }

                _vehicleMainAxis = CalculateVehicleMainAxis();
                //If the voxel still exists, cleanup everything so we can continue;
                visualizing = false;

                _voxel?.CleanupVoxel();

                //set flag so that this function can't run again before voxelizing completes and queue voxelizing thread
                Voxelizing = true;
                VoxelizationThreadpool.Instance.QueueVoxelization(() => CreateVoxel(vessel));
                return true;
            }
            finally
            {
                Monitor.Exit(this);
            }
        }

        //And this actually creates the voxel and then begins the aero properties determination
        private void CreateVoxel(Vessel vessel = null)
        {
            lock (this) //lock this object to prevent race with main thread
            {
                try
                {
                    //Actually voxelize it
                    _voxel = VehicleVoxel.CreateNewVoxel(_currentGeoModules, _voxelCount, vessel: vessel);
                    if (_vehicleCrossSection.Length < _voxel.MaxArrayLength)
                        _vehicleCrossSection = _voxel.EmptyCrossSectionArray;

                    _voxelLowerRightCorner = _voxel.LocalLowerRightCorner;
                    _voxelElementSize = _voxel.ElementSize;

                    CalculateVesselAeroProperties();
                    CalculationCompleted = true;
                }
                catch (Exception e)
                {
                    ThreadSafeDebugLogger.Exception(e);
                }
                finally
                {
                    //Always, when we finish up, if we're in flight, cleanup the voxel
                    if (HighLogic.LoadedSceneIsFlight && !FARConfig.Voxelization.DebugInFlight && _voxel != null)
                    {
                        _voxel.CleanupVoxel();
                        _voxel = null;
                    }

                    //And unset the flag so that the main thread can queue it again
                    Voxelizing = false;
                }
            }
        }


        private Vector3 CalculateVehicleMainAxis()
        {
            Vector3 axis = Vector3.zero;
            var hitParts = new HashSet<Part>();

            bool hasPartsForAxis = false;

            foreach (Part p in _vehiclePartList)
            {
                if (p == null || hitParts.Contains(p))
                    continue;

                // Could be left null if a launch clamp
                var geoModule = p.FindModuleImplementingFast<GeometryPartModule>();

                hitParts.Add(p);

                Vector3 tmpCandVector = Vector3.zero;
                Vector3 candVector = Vector3.zero;

                //intakes are probably pointing in the direction we're gonna be going in
                if (p.FindModuleImplementingFast<ModuleResourceIntake>() is ModuleResourceIntake intake)
                {
                    Transform intakeTrans = p.FindModelTransform(intake.intakeTransformName);
                    if (!(intakeTrans is null))
                        candVector = intakeTrans.TransformDirection(Vector3.forward);
                }
                //aggregate wings for later calc...
                else if (geoModule == null ||
                         geoModule.IgnoreForMainAxis ||
                         p.HasModuleImplementingFast<FARWingAerodynamicModel>() ||
                         p.HasModuleImplementingFast<FARControllableSurface>() ||
                         p.HasModuleImplementingFast<ModuleWheelBase>() ||
                         p.Modules.Contains("KSPWheelBase"))
                {
                    continue;
                }
                else
                {
                    if (p.srfAttachNode != null && p.srfAttachNode.attachedPart != null)
                    {
                        tmpCandVector = p.srfAttachNode.orientation;
                        tmpCandVector = new Vector3(0,
                                                    Math.Abs(tmpCandVector.x) + Math.Abs(tmpCandVector.z),
                                                    Math.Abs(tmpCandVector.y));

                        if (p.srfAttachNode.position.sqrMagnitude.NearlyEqual(0) && tmpCandVector == Vector3.forward)
                            tmpCandVector = Vector3.up;

                        if (tmpCandVector.z > tmpCandVector.x && tmpCandVector.z > tmpCandVector.y)
                            tmpCandVector = Vector3.forward;
                        else if (tmpCandVector.y > tmpCandVector.x && tmpCandVector.y > tmpCandVector.z)
                            tmpCandVector = Vector3.up;
                        else
                            tmpCandVector = Vector3.right;
                    }
                    else
                    {
                        tmpCandVector = Vector3.up;
                    }

                    candVector = p.partTransform.TransformDirection(tmpCandVector);
                }

                foreach (Part q in p.symmetryCounterparts)
                {
                    if (q == null || hitParts.Contains(q))
                        continue;

                    hitParts.Add(q);

                    //intakes are probably pointing in the direction we're gonna be going in
                    if (q.FindModuleImplementingFast<ModuleResourceIntake>() is ModuleResourceIntake intake2)
                    {
                        Transform intakeTrans = q.FindModelTransform(intake2.intakeTransformName);
                        if (!(intakeTrans is null))
                            candVector += intakeTrans.TransformDirection(Vector3.forward);
                    }
                    else
                    {
                        candVector += q.partTransform.TransformDirection(tmpCandVector);
                    }
                }

                //set that we will get a valid axis out of this operation
                hasPartsForAxis = true;

                candVector = _worldToLocalMatrix.MultiplyVector(candVector);
                candVector.x = Math.Abs(candVector.x);
                candVector.y = Math.Abs(candVector.y);
                candVector.z = Math.Abs(candVector.z);

                Vector3 size = geoModule.overallMeshBounds.size;

                axis += size.x * size.y * size.z * candVector; //scale part influence by approximate size
            }

            if (axis == Vector3.zero)
                axis = Vector3.up; //something in case things fall through somehow

            if (!hasPartsForAxis)
                return Vector3.up; //welp, no parts that we can rely on for determining the axis; fall back to up
            float dotProdX = Math.Abs(Vector3.Dot(axis, Vector3.right));
            float dotProdY = Math.Abs(Vector3.Dot(axis, Vector3.up));
            float dotProdZ = Math.Abs(Vector3.Dot(axis, Vector3.forward));

            if (dotProdY > 2 * dotProdX && dotProdY > 2 * dotProdZ)
                return Vector3.up;

            if (dotProdX > 2 * dotProdY && dotProdX > 2 * dotProdZ)
                return Vector3.right;

            if (dotProdZ > 2 * dotProdX && dotProdZ > 2 * dotProdY)
                return Vector3.forward;

            //Otherwise, now we need to use axis, since it's obviously not close to anything else
            return axis.normalized;
        }

        //Smooths out area and area 2nd deriv distributions to deal with noise in the representation
        private static unsafe void GaussianSmoothCrossSections(
            VoxelCrossSection[] vehicleCrossSection,
            double stdDevCutoff,
            double lengthPercentFactor,
            double sectionThickness,
            double length,
            int frontIndex,
            int backIndex,
            int areaSmoothingIterations,
            int derivSmoothingIterations
        )
        {
            double stdDev = length * lengthPercentFactor;
            int numVals = (int)Math.Ceiling(stdDevCutoff * stdDev / sectionThickness);

            if (numVals <= 1)
                return;

            double* gaussianFactors = stackalloc double[numVals];
            double* prevUncorrectedVals = stackalloc double[numVals];
            double* futureUncorrectedVals = stackalloc double[numVals - 1];

            double invVariance = 1 / (stdDev * stdDev);

            //calculate Gaussian factors for each of the points that will be hit
            for (int i = 0; i < numVals; i++)
            {
                double factor = i * sectionThickness;
                factor *= factor;
                gaussianFactors[i] = Math.Exp(-0.5 * factor * invVariance);
            }

            //then sum them up...
            double sum = 0;
            for (int i = 0; i < numVals; i++)
                if (i == 0)
                    sum += gaussianFactors[i];
                else
                    sum += 2 * gaussianFactors[i];

            double invSum = 1 / sum; //and then use that to normalize the factors

            for (int i = 0; i < numVals; i++)
                gaussianFactors[i] *= invSum;

            //first smooth the area itself.  This has a greater effect on the 2nd deriv due to the effect of noise on derivatives
            for (int j = 0; j < areaSmoothingIterations; j++)
            {
                for (int i = 0; i < numVals; i++)
                    //set all the vals to 0 to prevent screwups between iterations
                    prevUncorrectedVals[i] = vehicleCrossSection[frontIndex].area;

                for (int i = frontIndex; i <= backIndex; i++) //area smoothing pass
                {
                    for (int k = numVals - 1; k > 0; k--)
                        prevUncorrectedVals[k] = prevUncorrectedVals[k - 1]; //shift prev vals down
                    double curValue = vehicleCrossSection[i].area;
                    prevUncorrectedVals[0] = curValue; //and set the central value


                    for (int k = 0; k < numVals - 1; k++) //update future vals
                        if (i + k < backIndex)
                            futureUncorrectedVals[k] = vehicleCrossSection[i + k + 1].area;
                        else
                            futureUncorrectedVals[k] = vehicleCrossSection[backIndex].area;
                    curValue = 0; //zero for coming calculations...

                    double borderScaling = 1; //factor to correct for the 0s lurking at the borders of the curve...

                    for (int k = 0; k < numVals; k++)
                    {
                        double val = prevUncorrectedVals[k];
                        double gaussianFactor = gaussianFactors[k];

                        curValue += gaussianFactor * val; //central and previous values;
                        if (val.NearlyEqual(0))
                            borderScaling -= gaussianFactor;
                    }

                    for (int k = 0; k < numVals - 1; k++)
                    {
                        double val = futureUncorrectedVals[k];
                        double gaussianFactor = gaussianFactors[k + 1];

                        curValue += gaussianFactor * val; //future values
                        if (val.NearlyEqual(0))
                            borderScaling -= gaussianFactor;
                    }

                    if (borderScaling > 0)
                        curValue /= borderScaling; //and now all of the 0s beyond the edge have been removed

                    vehicleCrossSection[i].area = curValue;
                }
            }

            CalculateCrossSectionSecondDerivs(vehicleCrossSection, numVals, frontIndex, backIndex, sectionThickness);

            //and now smooth the derivs
            for (int j = 0; j < derivSmoothingIterations; j++)
            {
                for (int i = 0; i < numVals; i++)
                    //set all the vals to 0 to prevent screwups between iterations
                    prevUncorrectedVals[i] = vehicleCrossSection[frontIndex].secondAreaDeriv;

                for (int i = frontIndex; i <= backIndex; i++) //deriv smoothing pass
                {
                    for (int k = numVals - 1; k > 0; k--)
                        prevUncorrectedVals[k] = prevUncorrectedVals[k - 1]; //shift prev vals down
                    double curValue = vehicleCrossSection[i].secondAreaDeriv;
                    prevUncorrectedVals[0] = curValue; //and set the central value


                    for (int k = 0; k < numVals - 1; k++) //update future vals
                        if (i + k < backIndex)
                            futureUncorrectedVals[k] = vehicleCrossSection[i + k + 1].secondAreaDeriv;
                        else
                            futureUncorrectedVals[k] = vehicleCrossSection[backIndex].secondAreaDeriv;
                    curValue = 0; //zero for coming calculations...

                    double borderScaling = 1; //factor to correct for the 0s lurking at the borders of the curve...

                    for (int k = 0; k < numVals; k++)
                    {
                        double val = prevUncorrectedVals[k];
                        double gaussianFactor = gaussianFactors[k];

                        curValue += gaussianFactor * val; //central and previous values;
                        if (val.NearlyEqual(0))
                            borderScaling -= gaussianFactor;
                    }

                    for (int k = 0; k < numVals - 1; k++)
                    {
                        double val = futureUncorrectedVals[k];
                        double gaussianFactor = gaussianFactors[k + 1];

                        curValue += gaussianFactor * val; //future values
                        if (val.NearlyEqual(0))
                            borderScaling -= gaussianFactor;
                    }

                    if (borderScaling > 0)
                        curValue /= borderScaling; //and now all of the 0s beyond the edge have been removed

                    vehicleCrossSection[i].secondAreaDeriv = curValue;
                }
            }
        }

        //Based on http://www.holoborodko.com/pavel/downloads/NoiseRobustSecondDerivative.pdf
        private static unsafe void CalculateCrossSectionSecondDerivs(
            VoxelCrossSection[] vehicleCrossSection,
            int oneSidedFilterLength,
            int frontIndex,
            int backIndex,
            double sectionThickness
        )
        {
            if (oneSidedFilterLength < 2)
            {
                oneSidedFilterLength = 2;
                ThreadSafeDebugLogger.Info("Needed to adjust filter length up");
            }
            else if (oneSidedFilterLength > 40)
            {
                oneSidedFilterLength = 40;
                ThreadSafeDebugLogger.Info("Reducing filter length to prevent overflow");
            }

            int M = oneSidedFilterLength;
            int N = M * 2 + 1;
            long* sK = stackalloc long[M + 1];

            for (int i = 0; i <= M; i++)
                sK[i] = CalculateSk(i, M, N);
            double denom = Math.Pow(2, N - 3);
            denom *= sectionThickness * sectionThickness;
            denom = 1 / denom;

            int lowIndex = Math.Max(frontIndex - 1, 0);
            int highIndex = Math.Min(backIndex + 1, vehicleCrossSection.Length - 1);

            for (int i = lowIndex + M; i <= highIndex - M; i++)
            {
                double secondDeriv = 0;
                if (i >= frontIndex && i <= backIndex)
                    secondDeriv = sK[0] * vehicleCrossSection[i].area;

                for (int k = 1; k <= M; k++)
                {
                    double forwardArea, backwardArea;

                    if (i + k <= backIndex)
                        backwardArea = vehicleCrossSection[i + k].area;
                    else
                        backwardArea = 0;

                    if (i - k >= frontIndex)
                        forwardArea = vehicleCrossSection[i - k].area;
                    else
                        forwardArea = 0;

                    secondDeriv += sK[k] * (forwardArea + backwardArea);
                }

                vehicleCrossSection[i].secondAreaDeriv = secondDeriv * denom;
            }

            //forward difference
            for (int i = frontIndex; i < lowIndex + M; i++)
            {
                double secondDeriv = 0;

                secondDeriv += vehicleCrossSection[i].area;
                if (i + 2 <= backIndex)
                    secondDeriv += vehicleCrossSection[i + 2].area;
                if (i + 1 <= backIndex)
                    secondDeriv -= 2 * vehicleCrossSection[i + 1].area;

                secondDeriv /= sectionThickness * sectionThickness;

                vehicleCrossSection[i].secondAreaDeriv = secondDeriv;
            }

            //backward difference
            for (int i = highIndex - M + 1; i <= backIndex; i++)
            {
                double secondDeriv = 0;

                secondDeriv += vehicleCrossSection[i].area;
                if (i - 2 >= frontIndex)
                    secondDeriv += vehicleCrossSection[i - 2].area;
                if (i - 1 >= frontIndex)
                    secondDeriv -= 2 * vehicleCrossSection[i - 1].area;

                secondDeriv /= sectionThickness * sectionThickness;

                vehicleCrossSection[i].secondAreaDeriv = secondDeriv;
            }
        }

        private static long CalculateSk(long k, int M, int N)
        {
            if (k > M)
                return 0;
            if (k == M)
                return 1;
            long val = (2 * N - 10) * CalculateSk(k + 1, M, N);
            val -= (N + 2 * k + 3) * CalculateSk(k + 2, M, N);
            val /= N - 2 * k - 1;

            return val;
        }

        private void AdjustCrossSectionForAirDucting(
            VoxelCrossSection[] vehicleCrossSection,
            List<GeometryPartModule> geometryModules,
            int front,
            int back,
            ref double maxCrossSectionArea
        )
        {
            foreach (GeometryPartModule g in geometryModules)
                g.GetICrossSectionAdjusters(activeAdjusters, _worldToLocalMatrix, _vehicleMainAxis);

            double intakeArea = 0;
            double engineExitArea = 0;

            foreach (ICrossSectionAdjuster adjuster in activeAdjusters)
                switch (adjuster)
                {
                    case AirbreathingEngineCrossSectionAdjuster _:
                        engineExitArea += Math.Abs(adjuster.AreaRemovedFromCrossSection());
                        break;
                    case IntakeCrossSectionAdjuster _:
                        intakeArea += Math.Abs(adjuster.AreaRemovedFromCrossSection());
                        break;
                    case IntegratedIntakeEngineCrossSectionAdjuster _:
                        engineExitArea += Math.Abs(adjuster.AreaRemovedFromCrossSection());
                        intakeArea += Math.Abs(adjuster.AreaRemovedFromCrossSection());
                        break;
                }

            //if they exist, go through the calculations
            if (!intakeArea.NearlyEqual(0) && !engineExitArea.NearlyEqual(0))
            {
                if (_ductedAreaAdjustment.Length != vehicleCrossSection.Length)
                    _ductedAreaAdjustment = new double[vehicleCrossSection.Length];


                int frontMostIndex = -1, backMostIndex = -1;

                //sweep through entire vehicle
                for (int i = 0; i < _ductedAreaAdjustment.Length; i++)
                {
                    double ductedArea = 0; //area based on the voxel size
                    double voxelCountScale = _voxelElementSize * _voxelElementSize;
                    //and all the intakes / engines
                    if (i >= front && i <= back)
                    {
                        foreach (ICrossSectionAdjuster adjuster in activeAdjusters)
                        {
                            if (adjuster.IntegratedCrossSectionIncreaseDecrease())
                                continue;

                            if (adjuster.AreaRemovedFromCrossSection().NearlyEqual(0))
                                continue;

                            Part p = adjuster.GetPart();

                            //see if you can find that in this section
                            if (!vehicleCrossSection[i]
                                 .partSideAreaValues.TryGetValue(p, out VoxelCrossSection.SideAreaValues val))
                                continue;
                            if (adjuster.AreaRemovedFromCrossSection() > 0)
                                ductedArea += Math.Max(0,
                                                       val.crossSectionalAreaCount * voxelCountScale +
                                                       adjuster.AreaThreshold());
                            else
                                ductedArea -= Math.Max(0,
                                                       val.crossSectionalAreaCount * voxelCountScale +
                                                       adjuster.AreaThreshold());
                        }

                        ductedArea *= 0.75;

                        if (!ductedArea.NearlyEqual(0))
                            if (frontMostIndex < 0)
                                frontMostIndex = i;
                            else
                                backMostIndex = i;
                    }

                    _ductedAreaAdjustment[i] = ductedArea;
                }

                double tmpArea = _ductedAreaAdjustment[0];

                for (int i = 1; i < _ductedAreaAdjustment.Length; i++)
                {
                    double areaAdjustment = _ductedAreaAdjustment[i];
                    double prevAreaAdjustment = tmpArea;

                    tmpArea = areaAdjustment; //store for next iteration

                    if (areaAdjustment <= 0 || prevAreaAdjustment <= 0)
                        continue;
                    double areaChange = areaAdjustment - prevAreaAdjustment;
                    if (areaChange > 0)
                        //this transforms this into a change in area, but only for increases (intakes)
                    {
                        _ductedAreaAdjustment[i] = areaChange;
                    }
                    else
                    {
                        tmpArea = prevAreaAdjustment;
                        _ductedAreaAdjustment[i] = 0;
                    }
                }

                tmpArea = _ductedAreaAdjustment[_ductedAreaAdjustment.Length - 1];

                for (int i = _ductedAreaAdjustment.Length - 1; i >= 0; i--)
                {
                    double areaAdjustment = _ductedAreaAdjustment[i];
                    double prevAreaAdjustment = tmpArea;

                    tmpArea = areaAdjustment; //store for next iteration

                    if (areaAdjustment >= 0 || prevAreaAdjustment >= 0)
                        continue;
                    double areaChange = areaAdjustment - prevAreaAdjustment;
                    if (areaChange < 0)
                        //this transforms this into a change in area, but only for decreases (engines)
                    {
                        _ductedAreaAdjustment[i] = areaChange;
                    }
                    else
                    {
                        tmpArea = prevAreaAdjustment;
                        _ductedAreaAdjustment[i] = 0;
                    }
                }

                for (int i = _ductedAreaAdjustment.Length - 1; i >= 0; i--)
                {
                    double areaAdjustment = 0;
                    for (int j = 0; j <= i; j++)
                        areaAdjustment += _ductedAreaAdjustment[j];

                    _ductedAreaAdjustment[i] = areaAdjustment;
                }

                for (int i = 0; i < vehicleCrossSection.Length; i++)
                {
                    double ductedArea = 0; //area based on the voxel size
                    double actualArea = 0; //area based on intake and engine data

                    //and all the intakes / engines
                    foreach (ICrossSectionAdjuster adjuster in activeAdjusters)
                    {
                        if (!adjuster.IntegratedCrossSectionIncreaseDecrease())
                            continue;

                        Part p = adjuster.GetPart();

                        //see if you can find that in this section
                        if (!vehicleCrossSection[i]
                             .partSideAreaValues.TryGetValue(p, out VoxelCrossSection.SideAreaValues val))
                            continue;
                        ductedArea += val.crossSectionalAreaCount;
                        actualArea += adjuster.AreaRemovedFromCrossSection();
                    }

                    ductedArea *= _voxelElementSize * _voxelElementSize * 0.75;

                    if (Math.Abs(actualArea) < Math.Abs(ductedArea))
                        ductedArea = actualArea;

                    if (!ductedArea.NearlyEqual(0))
                        if (i < frontMostIndex)
                            frontMostIndex = i;
                        else if (i > backMostIndex)
                            backMostIndex = i;

                    _ductedAreaAdjustment[i] += ductedArea;
                }

                int index = _ductedAreaAdjustment.Length - 1;
                double endVoxelArea = _ductedAreaAdjustment[index];

                double currentArea = endVoxelArea;

                while (currentArea > 0)
                {
                    currentArea -= endVoxelArea;
                    _ductedAreaAdjustment[index] = currentArea;

                    --index;

                    if (index < 0)
                        break;

                    currentArea = _ductedAreaAdjustment[index];
                }

                maxCrossSectionArea = 0;
                //put upper limit on area lost
                for (int i = 0; i < vehicleCrossSection.Length; i++)
                {
                    double areaUnchanged = vehicleCrossSection[i].area;
                    double areaChanged = -_ductedAreaAdjustment[i];
                    if (areaChanged > 0)
                        areaChanged = 0;
                    areaChanged += areaUnchanged;

                    double tmpTotalArea = Math.Max(0.15 * areaUnchanged, areaChanged);
                    if (tmpTotalArea > maxCrossSectionArea)
                        maxCrossSectionArea = tmpTotalArea;

                    vehicleCrossSection[i].area = tmpTotalArea;
                }
            }

            activeAdjusters.Clear();
        }

        private void CalculateVesselAeroProperties()
        {
            _voxel.CrossSectionData(_vehicleCrossSection,
                                    _vehicleMainAxis,
                                    out int front,
                                    out int back,
                                    out _sectionThickness,
                                    out _maxCrossSectionArea);

            int numSections = back - front;
            Length = _sectionThickness * numSections;

            double voxelVolume = _voxel.Volume;

            double filledVolume = 0;
            for (int i = front; i <= back; i++)
            {
                if (double.IsNaN(_vehicleCrossSection[i].area))
                    ThreadSafeDebugLogger.Error("FAR VOXEL ERROR: Voxel CrossSection Area is NaN at section " + i);

                filledVolume += _vehicleCrossSection[i].area;
            }

            filledVolume *= _sectionThickness; //total volume taken up by the filled voxel

            //determines how fine the grid is compared to the vehicle.  Accounts for loss in precision and added smoothing because of unused sections of voxel volume
            double gridFillednessFactor = filledVolume / voxelVolume;

            gridFillednessFactor *= 25; //used to handle relatively empty, but still alright, planes
            double stdDevCutoff = 3;
            stdDevCutoff *= gridFillednessFactor;
            if (stdDevCutoff < 0.5)
                stdDevCutoff = 0.5;
            if (stdDevCutoff > 3)
                stdDevCutoff = 3;

            double invMaxRadFactor = 1f / Math.Sqrt(_maxCrossSectionArea / Math.PI);

            //vehicle length / max diameter, as calculated from sect thickness * num sections / (2 * max radius)
            double finenessRatio = _sectionThickness * numSections * 0.5 * invMaxRadFactor;

            int extraLowFinenessRatioDerivSmoothingPasses = (int)Math.Round((5f - finenessRatio) * 0.5f) *
                                                            FARSettingsScenarioModule.Settings.numDerivSmoothingPasses;
            if (extraLowFinenessRatioDerivSmoothingPasses < 0)
                extraLowFinenessRatioDerivSmoothingPasses = 0;

            int extraAreaSmoothingPasses = (int)Math.Round((gridFillednessFactor / 25.0 - 0.5) * 4.0);
            if (extraAreaSmoothingPasses < 0)
                extraAreaSmoothingPasses = 0;


            ThreadSafeDebugLogger.Info("Std dev for smoothing: " +
                                       stdDevCutoff +
                                       " voxel total vol: " +
                                       voxelVolume +
                                       " filled vol: " +
                                       filledVolume);

            AdjustCrossSectionForAirDucting(_vehicleCrossSection,
                                            _currentGeoModules,
                                            front,
                                            back,
                                            ref _maxCrossSectionArea);

            GaussianSmoothCrossSections(_vehicleCrossSection,
                                        stdDevCutoff,
                                        FARSettingsScenarioModule.Settings.gaussianVehicleLengthFractionForSmoothing,
                                        _sectionThickness,
                                        Length,
                                        front,
                                        back,
                                        FARSettingsScenarioModule.Settings.numAreaSmoothingPasses +
                                        extraAreaSmoothingPasses,
                                        FARSettingsScenarioModule.Settings.numDerivSmoothingPasses +
                                        extraLowFinenessRatioDerivSmoothingPasses);

            CalculateSonicPressure(_vehicleCrossSection, front, back, _sectionThickness);

            validSectionCount = numSections;
            firstSection = front;

            //recalc these with adjusted cross-sections
            invMaxRadFactor = 1f / Math.Sqrt(_maxCrossSectionArea / Math.PI);

            //vehicle length / max diameter, as calculated from sect thickness * num sections / (2 * max radius)
            finenessRatio = _sectionThickness * numSections * 0.5 * invMaxRadFactor;

            //skin friction and pressure drag for a body, taken from 1978 USAF Stability And Control DATCOM, Section 4.2.3.1, Paragraph A
            //pressure drag for a subsonic / transonic body due to skin friction
            double viscousDragFactor = 60 / (finenessRatio * finenessRatio * finenessRatio) + 0.0025 * finenessRatio;
            viscousDragFactor++;

            viscousDragFactor /= numSections; //fraction of viscous drag applied to each section

            double criticalMachNumber = CalculateCriticalMachNumber(finenessRatio);

            CriticalMach = criticalMachNumber *
                           CriticalMachFactorForUnsmoothCrossSection(_vehicleCrossSection,
                                                                     finenessRatio,
                                                                     _sectionThickness);

            float lowFinenessRatioFactor = 1f;
            lowFinenessRatioFactor += 1f / (1 + 0.5f * (float)finenessRatio);
            float lowFinenessRatioBlendFactor = lowFinenessRatioFactor--;

            _moduleAndAreasDict.Clear();

            var tmpAeroModules =
                new HashSet<FARAeroPartModule>(ObjectReferenceEqualityComparer<FARAeroPartModule>.Default);
            SonicDragArea = 0;

            if (_newAeroSections.Capacity < numSections + 1)
                _newAeroSections.Capacity = numSections + 1;

            int aeroSectionIndex = 0;
            FARAeroSection prevSection = null;

            for (int i = 0; i <= numSections; i++) //index in the cross sections
            {
                int index = i + front; //index along the actual body

                double prevArea, nextArea;

                double curArea = _vehicleCrossSection[index].area;
                if (i == 0)
                    prevArea = 0;
                else
                    prevArea = _vehicleCrossSection[index - 1].area;
                if (i == numSections)
                    nextArea = 0;
                else
                    nextArea = _vehicleCrossSection[index + 1].area;

                FARAeroSection currentSection = null;

                if (aeroSectionIndex < _newAeroSections.Count)
                    currentSection = _newAeroSections[aeroSectionIndex];
                else
                    lock (_commonLocker)
                    {
                        if (currentlyUnusedSections.Count > 0)
                            currentSection = currentlyUnusedSections.Pop();
                    }

                if (currentSection == null)
                    currentSection = FARAeroSection.CreateNewAeroSection();

                FARFloatCurve xForcePressureAoA0 = currentSection.xForcePressureAoA0;
                FARFloatCurve xForcePressureAoA180 = currentSection.xForcePressureAoA180;
                FARFloatCurve xForceSkinFriction = currentSection.xForceSkinFriction;

                //Potential and Viscous lift calculations
                float potentialFlowNormalForce;
                if (i == 0)
                    potentialFlowNormalForce = (float)(nextArea - curArea);
                else if (i == numSections)
                    potentialFlowNormalForce = (float)(curArea - prevArea);
                else
                    potentialFlowNormalForce = (float)(nextArea - prevArea) * 0.5f; //calculated from area change

                float areaChangeMax = (float)Math.Min(Math.Min(nextArea, prevArea) * 0.1, Length * 0.01);

                float sonicBaseDrag = 0.21f;

                sonicBaseDrag *= potentialFlowNormalForce; //area base drag acts over

                if (potentialFlowNormalForce > areaChangeMax)
                    potentialFlowNormalForce = areaChangeMax;
                else if (potentialFlowNormalForce < -areaChangeMax)
                    potentialFlowNormalForce = -areaChangeMax;
                else if (!areaChangeMax.NearlyEqual(0))
                    //some scaling for small changes in cross-section
                    sonicBaseDrag *= Math.Abs(potentialFlowNormalForce / areaChangeMax);

                double flatnessRatio = _vehicleCrossSection[index].flatnessRatio;
                if (flatnessRatio >= 1)
                    sonicBaseDrag /= (float)(flatnessRatio * flatnessRatio);
                else
                    sonicBaseDrag *= (float)(flatnessRatio * flatnessRatio);

                float hypersonicDragForward =
                    (float)CalculateHypersonicDrag(prevArea, curArea, _sectionThickness); //negative forces
                float hypersonicDragBackward = (float)CalculateHypersonicDrag(nextArea, curArea, _sectionThickness);

                float hypersonicDragForwardFrac = 0, hypersonicDragBackwardFrac = 0;

                if (!curArea.NearlyEqual(prevArea))
                    hypersonicDragForwardFrac = Math.Abs(hypersonicDragForward * 0.5f / (float)(curArea - prevArea));
                if (!curArea.NearlyEqual(nextArea))
                    hypersonicDragBackwardFrac = Math.Abs(hypersonicDragBackward * 0.5f / (float)(curArea - nextArea));

                hypersonicDragForwardFrac *= hypersonicDragForwardFrac; //^2
                hypersonicDragForwardFrac *= hypersonicDragForwardFrac; //^4

                hypersonicDragBackwardFrac *= hypersonicDragBackwardFrac; //^2
                hypersonicDragBackwardFrac *= hypersonicDragBackwardFrac; //^4

                if (flatnessRatio >= 1)
                {
                    hypersonicDragForwardFrac /= (float)(flatnessRatio * flatnessRatio);
                    hypersonicDragBackwardFrac /= (float)(flatnessRatio * flatnessRatio);
                }
                else
                {
                    hypersonicDragForwardFrac *= (float)(flatnessRatio * flatnessRatio);
                    hypersonicDragBackwardFrac *= (float)(flatnessRatio * flatnessRatio);
                }

                float hypersonicMomentForward = (float)CalculateHypersonicMoment(prevArea, curArea, _sectionThickness);
                float hypersonicMomentBackward = (float)CalculateHypersonicMoment(nextArea, curArea, _sectionThickness);


                xForcePressureAoA0.SetPoint(5, new Vector3d(35, hypersonicDragForward, 0));
                xForcePressureAoA180.SetPoint(5, new Vector3d(35, -hypersonicDragBackward, 0));

                float sonicAoA0Drag, sonicAoA180Drag;

                double cPSonicForward = _vehicleCrossSection[index].cpSonicForward;
                double cPSonicBackward = _vehicleCrossSection[index].cpSonicBackward;

                double areaForForces = (curArea + prevArea - (nextArea + curArea)) * 0.5;

                if (sonicBaseDrag > 0) //occurs with increase in area; force applied at 180 AoA
                {
                    //hypersonic drag used as a proxy for effects due to flow separation
                    xForcePressureAoA0.SetPoint(0,
                                                new Vector3d(CriticalMach,
                                                             0.325f *
                                                             hypersonicDragForward *
                                                             hypersonicDragForwardFrac *
                                                             lowFinenessRatioFactor,
                                                             0));
                    xForcePressureAoA180.SetPoint(0,
                                                  new Vector3d(CriticalMach,
                                                               (sonicBaseDrag * 0.2f -
                                                                0.325f *
                                                                hypersonicDragBackward *
                                                                hypersonicDragBackwardFrac) *
                                                               lowFinenessRatioFactor,
                                                               0));


                    hypersonicDragBackwardFrac += 1f; //avg fracs with 1 to get intermediate frac
                    hypersonicDragBackwardFrac *= 0.5f;

                    hypersonicDragForwardFrac += 1f;
                    hypersonicDragForwardFrac *= 0.5f;

                    sonicAoA0Drag = -(float)(cPSonicForward * areaForForces) +
                                    0.3f * hypersonicDragForward * hypersonicDragForwardFrac;
                    //at high finenessRatios, use the entire above section for sonic drag
                    sonicAoA0Drag *= 1 - lowFinenessRatioBlendFactor;
                    //at very low finenessRatios, use a boosted version of the hypersonic drag
                    sonicAoA0Drag += hypersonicDragForward *
                                     hypersonicDragForwardFrac *
                                     lowFinenessRatioBlendFactor *
                                     1.4f;

                    sonicAoA180Drag = (float)(cPSonicBackward * -areaForForces) +
                                      sonicBaseDrag -
                                      0.3f * hypersonicDragBackward * hypersonicDragBackwardFrac;
                    //at high finenessRatios, use the entire above section for sonic drag
                    sonicAoA180Drag *= 1 - lowFinenessRatioBlendFactor;
                    //at very low finenessRatios, use a boosted version of the hypersonic drag
                    sonicAoA180Drag += (-hypersonicDragBackward * hypersonicDragBackwardFrac * 1.4f + sonicBaseDrag) *
                                       lowFinenessRatioBlendFactor;
                }
                else if (sonicBaseDrag < 0)
                {
                    xForcePressureAoA0.SetPoint(0,
                                                new Vector3d(CriticalMach,
                                                             (sonicBaseDrag * 0.2f +
                                                              0.325f *
                                                              hypersonicDragForward *
                                                              hypersonicDragForwardFrac) *
                                                             lowFinenessRatioFactor,
                                                             0));
                    xForcePressureAoA180.SetPoint(0,
                                                  new Vector3d(CriticalMach,
                                                               -(0.325f *
                                                                 hypersonicDragBackward *
                                                                 hypersonicDragBackwardFrac) *
                                                               lowFinenessRatioFactor,
                                                               0));

                    hypersonicDragBackwardFrac += 1f; //avg fracs with 1 to get intermediate frac
                    hypersonicDragBackwardFrac *= 0.5f;

                    hypersonicDragForwardFrac += 1f;
                    hypersonicDragForwardFrac *= 0.5f;

                    sonicAoA0Drag = -(float)(cPSonicForward * areaForForces) +
                                    sonicBaseDrag +
                                    0.3f * hypersonicDragForward * hypersonicDragForwardFrac;
                    //at high finenessRatios, use the entire above section for sonic drag
                    sonicAoA0Drag *= 1 - lowFinenessRatioBlendFactor;
                    //at very low finenessRatios, use a boosted version of the hypersonic drag
                    sonicAoA0Drag += (hypersonicDragForward * hypersonicDragForwardFrac * 1.4f + sonicBaseDrag) *
                                     lowFinenessRatioBlendFactor;

                    sonicAoA180Drag = (float)(cPSonicBackward * -areaForForces) -
                                      0.3f * hypersonicDragBackward * hypersonicDragBackwardFrac;
                    //at high finenessRatios, use the entire above section for sonic drag
                    sonicAoA180Drag *= 1 - lowFinenessRatioBlendFactor;
                    //at very low finenessRatios, use a boosted version of the hypersonic drag
                    sonicAoA180Drag += -hypersonicDragBackward *
                                       hypersonicDragBackwardFrac *
                                       1.4f *
                                       lowFinenessRatioBlendFactor;
                }
                else
                {
                    xForcePressureAoA0.SetPoint(0,
                                                new Vector3d(CriticalMach,
                                                             0.325f *
                                                             hypersonicDragForward *
                                                             hypersonicDragForwardFrac *
                                                             lowFinenessRatioFactor,
                                                             0));
                    xForcePressureAoA180.SetPoint(0,
                                                  new Vector3d(CriticalMach,
                                                               -(0.325f *
                                                                 hypersonicDragBackward *
                                                                 hypersonicDragBackwardFrac) *
                                                               lowFinenessRatioFactor,
                                                               0));

                    hypersonicDragBackwardFrac += 1f; //avg fracs with 1 to get intermediate frac
                    hypersonicDragBackwardFrac *= 0.5f;

                    hypersonicDragForwardFrac += 1f;
                    hypersonicDragForwardFrac *= 0.5f;

                    sonicAoA0Drag = -(float)(cPSonicForward * areaForForces) +
                                    0.3f * hypersonicDragForward * hypersonicDragForwardFrac;
                    //at high finenessRatios, use the entire above section for sonic drag
                    sonicAoA0Drag *= 1 - lowFinenessRatioBlendFactor;
                    //at very low finenessRatios, use a boosted version of the hypersonic drag
                    sonicAoA0Drag += hypersonicDragForward *
                                     hypersonicDragForwardFrac *
                                     lowFinenessRatioBlendFactor *
                                     1.4f;

                    sonicAoA180Drag = (float)(cPSonicBackward * -areaForForces) -
                                      0.3f * hypersonicDragBackward * hypersonicDragBackwardFrac;
                    //at high finenessRatios, use the entire above section for sonic drag
                    sonicAoA180Drag *= 1 - lowFinenessRatioBlendFactor;
                    //at very low finenessRatios, use a boosted version of the hypersonic drag
                    sonicAoA180Drag += -hypersonicDragBackward *
                                       hypersonicDragBackwardFrac *
                                       1.4f *
                                       lowFinenessRatioBlendFactor;
                }

                float diffSonicHyperAoA0 = Math.Abs(sonicAoA0Drag) - Math.Abs(hypersonicDragForward);
                float diffSonicHyperAoA180 = Math.Abs(sonicAoA180Drag) - Math.Abs(hypersonicDragBackward);


                xForcePressureAoA0.SetPoint(1, new Vector3d(1f, sonicAoA0Drag, 0));
                xForcePressureAoA180.SetPoint(1, new Vector3d(1f, sonicAoA180Drag, 0));

                //need to recalc slope here
                xForcePressureAoA0.SetPoint(2,
                                            new Vector3d(2f,
                                                         sonicAoA0Drag * 0.5773503f +
                                                         (1 - 0.5773503f) * hypersonicDragForward,
                                                         -0.2735292 * diffSonicHyperAoA0));
                xForcePressureAoA180.SetPoint(2,
                                              new Vector3d(2f,
                                                           sonicAoA180Drag * 0.5773503f -
                                                           (1 - 0.5773503f) * hypersonicDragBackward,
                                                           -0.2735292 * diffSonicHyperAoA180));

                xForcePressureAoA0.SetPoint(3,
                                            new Vector3d(5f,
                                                         sonicAoA0Drag * 0.2041242f +
                                                         (1 - 0.2041242f) * hypersonicDragForward,
                                                         -0.04252587f * diffSonicHyperAoA0));
                xForcePressureAoA180.SetPoint(3,
                                              new Vector3d(5f,
                                                           sonicAoA180Drag * 0.2041242f -
                                                           (1 - 0.2041242f) * hypersonicDragBackward,
                                                           -0.04252587f * diffSonicHyperAoA180));

                xForcePressureAoA0.SetPoint(4,
                                            new Vector3d(10f,
                                                         sonicAoA0Drag * 0.1005038f +
                                                         (1 - 0.1005038f) * hypersonicDragForward,
                                                         -0.0101519f * diffSonicHyperAoA0));
                xForcePressureAoA180.SetPoint(4,
                                              new Vector3d(10f,
                                                           sonicAoA180Drag * 0.1005038f -
                                                           (1 - 0.1005038f) * hypersonicDragBackward,
                                                           -0.0101519f * diffSonicHyperAoA180));

                Vector3 xRefVector;
                if (index == front || index == back)
                {
                    xRefVector = _vehicleMainAxis;
                }
                else
                {
                    xRefVector = _vehicleCrossSection[index - 1].centroid - _vehicleCrossSection[index + 1].centroid;
                    Vector3 offMainAxisVec = Vector3.ProjectOnPlane(xRefVector, _vehicleMainAxis);
                    float tanAoA = offMainAxisVec.magnitude / (2f * (float)_sectionThickness);
                    if (tanAoA > 0.17632698070846497347109038686862f)
                    {
                        offMainAxisVec.Normalize();
                        offMainAxisVec *= 0.17632698070846497347109038686862f; //max acceptable is 10 degrees
                        xRefVector = _vehicleMainAxis + offMainAxisVec;
                    }

                    xRefVector.Normalize();
                }

                Vector3 nRefVector = Matrix4x4
                                     .TRS(Vector3.zero,
                                          Quaternion.FromToRotation(_vehicleMainAxis, xRefVector),
                                          Vector3.one)
                                     .MultiplyVector(_vehicleCrossSection[index].flatNormalVector);

                Vector3 centroid = _localToWorldMatrix.MultiplyPoint3x4(_vehicleCrossSection[index].centroid);
                xRefVector = _localToWorldMatrix.MultiplyVector(xRefVector);
                nRefVector = _localToWorldMatrix.MultiplyVector(nRefVector);

                Dictionary<Part, VoxelCrossSection.SideAreaValues> includedPartsAndAreas =
                    _vehicleCrossSection[index].partSideAreaValues;

                float weightingFactor = 0;

                double surfaceArea = 0;
                foreach (KeyValuePair<Part, VoxelCrossSection.SideAreaValues> pair in includedPartsAndAreas)
                {
                    VoxelCrossSection.SideAreaValues areas = pair.Value;
                    surfaceArea += areas.iN + areas.iP + areas.jN + areas.jP + areas.kN + areas.kP;

                    Part key = pair.Key;
                    if (key is null)
                        continue;

                    if (!(key.FindModuleImplementingFast<FARAeroPartModule>() is FARAeroPartModule m))
                        continue;

                    includedModules.Add(m);

                    if (_moduleAndAreasDict.ContainsKey(m))
                        _moduleAndAreasDict[m] += areas;
                    else
                        _moduleAndAreasDict[m] = areas;

                    weightingFactor += (float)pair.Value.exposedAreaCount;
                    weighting.Add((float)pair.Value.exposedAreaCount);
                }

                weightingFactor = 1 / weightingFactor;
                for (int j = 0; j < weighting.Count; j++)
                    weighting[j] *= weightingFactor;

                float viscCrossflowDrag = (float)(Math.Sqrt(curArea / Math.PI) * _sectionThickness * 2d);

                //subsonic incompressible viscous drag
                xForceSkinFriction.SetPoint(0, new Vector3d(0, surfaceArea * viscousDragFactor, 0));
                //transonic viscous drag
                xForceSkinFriction.SetPoint(1, new Vector3d(1, surfaceArea * viscousDragFactor, 0));
                //above Mach 1.4, viscous is purely surface drag, no pressure-related components simulated
                xForceSkinFriction.SetPoint(2, new Vector3d(2, (float)surfaceArea, 0));

                currentSection.UpdateAeroSection(potentialFlowNormalForce,
                                                 viscCrossflowDrag,
                                                 viscCrossflowDrag / (float)_sectionThickness,
                                                 (float)flatnessRatio,
                                                 hypersonicMomentForward,
                                                 hypersonicMomentBackward,
                                                 centroid,
                                                 xRefVector,
                                                 nRefVector,
                                                 _localToWorldMatrix,
                                                 _vehicleMainAxis,
                                                 includedModules,
                                                 weighting,
                                                 _partWorldToLocalMatrixDict);


                if (prevSection != null && prevSection.CanMerge(currentSection))
                {
                    prevSection.MergeAeroSection(currentSection);
                    currentSection.ClearAeroSection();
                }
                else
                {
                    if (aeroSectionIndex < _newAeroSections.Count)
                        _newAeroSections[aeroSectionIndex] = currentSection;
                    else
                        _newAeroSections.Add(currentSection);

                    prevSection = currentSection;
                    ++aeroSectionIndex;
                }

                foreach (FARAeroPartModule a in includedModules)
                    tmpAeroModules.Add(a);

                includedModules.Clear();
                weighting.Clear();
            }

            if (_newAeroSections.Count > aeroSectionIndex + 1) //deal with sections that are unneeded now
                lock (_commonLocker)
                {
                    for (int i = _newAeroSections.Count - 1; i > aeroSectionIndex; --i)
                    {
                        FARAeroSection unusedSection = _newAeroSections[i];
                        _newAeroSections.RemoveAt(i);

                        unusedSection.ClearAeroSection();
                        if (currentlyUnusedSections.Count < 64)
                            //if there aren't that many extra ones stored, add them to the stack to be reused
                            currentlyUnusedSections.Push(unusedSection);
                    }
                }

            if (_moduleAndAreasDict.Count > 0)
                VoxelizationThreadpool.Instance.RunOnMainThread(() =>
                {
                    foreach (KeyValuePair<FARAeroPartModule, FARAeroPartModule.ProjectedArea> pair in
                             _moduleAndAreasDict)
                        pair.Key.SetProjectedArea(pair.Value, _localToWorldMatrix);
                });

            int aeroIndex = 0;
            if (_newAeroModules.Capacity < tmpAeroModules.Count)
                _newAeroModules.Capacity = tmpAeroModules.Count;


            foreach (FARAeroPartModule module in tmpAeroModules)
            {
                if (aeroIndex < _newAeroModules.Count)
                    _newAeroModules[aeroIndex] = module;
                else
                    _newAeroModules.Add(module);
                ++aeroIndex;
            }

            //at this point, aeroIndex is what the count of _newAeroModules _should_ be, but due to the possibility of the previous state having more modules, this is not guaranteed
            for (int i = _newAeroModules.Count - 1; i >= aeroIndex; --i)
                _newAeroModules.RemoveAt(i); //steadily remove the modules from the end that shouldn't be there

            _newUnusedAeroModules.Clear();

            VoxelizationThreadpool.Instance.RunOnMainThread(() =>
            {
                foreach (GeometryPartModule geoModule in _currentGeoModules)
                {
                    if (!geoModule)
                        continue;

                    var aeroModule = geoModule.GetComponent<FARAeroPartModule>();
                    if (aeroModule != null && !tmpAeroModules.Contains(aeroModule))
                        _newUnusedAeroModules.Add(aeroModule);
                }
            });
        }

        public void UpdateSonicDragArea()
        {
            var center = new FARCenterQuery();

            Vector3 worldMainAxis = _localToWorldMatrix.MultiplyVector(_vehicleMainAxis);
            worldMainAxis.Normalize();

            foreach (FARAeroSection a in _newAeroSections)
                a.PredictionCalculateAeroForces(2f, 1f, 50000f, 0, 0.005f, worldMainAxis, center);

            SonicDragArea = Vector3.Dot(center.force, worldMainAxis) * -1000;
        }

        private static double CalculateHypersonicMoment(double lowArea, double highArea, double sectionThickness)
        {
            if (lowArea >= highArea)
                return 0;

            double r1 = Math.Sqrt(lowArea / Math.PI);
            double r2 = Math.Sqrt(highArea / Math.PI);

            double moment = r2 * r2 + r1 * r1 + sectionThickness * sectionThickness * 0.5;
            moment *= 2 * Math.PI;

            double radDiffSq = r2 - r1;
            radDiffSq *= radDiffSq;

            moment *= radDiffSq;
            moment /= sectionThickness * sectionThickness + radDiffSq;

            return -moment * sectionThickness;
        }

        private static void CalculateSonicPressure(
            VoxelCrossSection[] vehicleCrossSection,
            int front,
            int back,
            double sectionThickness
        )
        {
            lock (_commonLocker)
            {
                if (vehicleCrossSection.Length > indexSqrt.Length + 1)
                    GenerateIndexSqrtLookup(vehicleCrossSection.Length + 2);
            }

            const double machTest = 1.2;
            double beta = Math.Sqrt(machTest * machTest - 1);

            double cP90 = CalcMaxCp(machTest);

            for (int i = front; i <= back; i++)
            {
                double cP = CalculateCpLinearForward(vehicleCrossSection,
                                                     i,
                                                     front,
                                                     beta,
                                                     sectionThickness,
                                                     double.PositiveInfinity);

                cP *= -0.5;
                cP = AdjustVelForFinitePressure(cP);
                cP *= -2;
                if (cP < 0)
                {
                    double firstDerivArea;
                    if (i == front)
                    {
                        firstDerivArea = vehicleCrossSection[i].area - vehicleCrossSection[i + 1].area;
                        firstDerivArea /= sectionThickness;
                    }
                    else if (i == back)
                    {
                        firstDerivArea = vehicleCrossSection[i - 1].area - vehicleCrossSection[i].area;
                        firstDerivArea /= sectionThickness;
                    }
                    else
                    {
                        firstDerivArea = vehicleCrossSection[i - 1].area - vehicleCrossSection[i + 1].area;
                        firstDerivArea /= sectionThickness;
                        firstDerivArea *= 0.5;
                    }

                    cP = AdjustCpForNonlinearEffects(cP, vehicleCrossSection[i].area, firstDerivArea, beta, machTest);
                }

                if (cP > cP90)
                    cP = cP90;

                vehicleCrossSection[i].cpSonicForward = cP;
            }

            for (int i = back; i >= front; i--)
            {
                double cP = CalculateCpLinearBackward(vehicleCrossSection,
                                                      i,
                                                      back,
                                                      beta,
                                                      sectionThickness,
                                                      double.PositiveInfinity);

                cP *= -0.5;
                cP = AdjustVelForFinitePressure(cP);
                cP *= -2;
                if (cP < 0)
                {
                    double firstDerivArea;
                    if (i == front)
                    {
                        firstDerivArea = vehicleCrossSection[i].area - vehicleCrossSection[i + 1].area;
                        firstDerivArea /= -sectionThickness;
                    }
                    else if (i == back)
                    {
                        firstDerivArea = vehicleCrossSection[i - 1].area - vehicleCrossSection[i].area;
                        firstDerivArea /= -sectionThickness;
                    }
                    else
                    {
                        firstDerivArea = vehicleCrossSection[i - 1].area - vehicleCrossSection[i + 1].area;
                        firstDerivArea /= -sectionThickness;
                        firstDerivArea *= 0.5;
                    }

                    cP = AdjustCpForNonlinearEffects(cP, vehicleCrossSection[i].area, firstDerivArea, beta, machTest);
                }

                if (cP > cP90)
                    cP = cP90;

                vehicleCrossSection[i].cpSonicBackward = cP;
            }
        }

        //Taken from Appendix A of NASA TR R-213
        private static double CalculateCpLinearForward(
            VoxelCrossSection[] vehicleCrossSection,
            int index,
            int front,
            double beta,
            double sectionThickness,
            double maxCrossSection
        )
        {
            double cP = 0;

            double cutoff = maxCrossSection * 2;

            double tmp1 = indexSqrt[index - (front - 2)];
            for (int i = front - 1; i <= index; i++)
            {
                double tmp2 = indexSqrt[index - i];
                double tmp = tmp1 - tmp2;
                tmp1 = tmp2;

                if (i >= 0)
                    tmp *= MathClampAbs(vehicleCrossSection[i].secondAreaDeriv, cutoff);
                else
                    tmp *= 0;

                cP += tmp;
            }

            double avgArea = vehicleCrossSection[index].area;

            cP *= Math.Sqrt(0.5 * sectionThickness / (beta * Math.Sqrt(Math.PI * avgArea)));

            if (cP > 2 || double.IsNaN(cP) || double.IsInfinity(cP))
                cP = 2; //2 is highest cP possible under any circumstances, so we clamp it here if there is an issue

            return cP;
        }

        private static double CalculateCpLinearBackward(
            VoxelCrossSection[] vehicleCrossSection,
            int index,
            int back,
            double beta,
            double sectionThickness,
            double maxCrossSection
        )
        {
            double cP = 0;

            double cutoff = maxCrossSection * 2;

            double tmp1 = indexSqrt[back + 2 - index];
            for (int i = back + 1; i >= index; i--)
            {
                double tmp2 = indexSqrt[i - index];
                double tmp = tmp1 - tmp2;
                tmp1 = tmp2;

                if (i < vehicleCrossSection.Length)
                    tmp *= MathClampAbs(vehicleCrossSection[i].secondAreaDeriv, cutoff);
                else
                    tmp *= 0;

                cP += tmp;
            }

            double avgArea = vehicleCrossSection[index].area;

            cP *= Math.Sqrt(0.5 * sectionThickness / (beta * Math.Sqrt(Math.PI * avgArea)));

            if (cP > 2 || double.IsNaN(cP) || double.IsInfinity(cP))
                cP = 2; //2 is highest cP possible under any circumstances, so we clamp it here if there is an issue

            return cP;
        }

        // ReSharper disable once UnusedMember.Local
        private double CalculateCpNoseDiscont(int index, double noseAreaSlope, double sectionThickness)
        {
            double cP_noseDiscont = index * sectionThickness * Math.PI;
            cP_noseDiscont = noseAreaSlope / cP_noseDiscont;

            return cP_noseDiscont;
        }

        private static double AdjustVelForFinitePressure(double vel)
        {
            if (vel > 0)
                return vel;

            double newVel = 1.0 - vel;
            newVel *= newVel;
            newVel = 2.0 * newVel / (1.0 + newVel);

            newVel = 1.0 - newVel;

            return newVel;
        }

        private static double CalcMaxCp(double mach)
        {
            double machSqr = mach * mach;
            double cP90 = 7.0 * machSqr - 1.0;
            cP90 = Math.Pow(6.0 / cP90, 2.5);
            cP90 *= Math.Pow(1.2 * machSqr, 3.5);
            cP90--;
            cP90 /= 0.7 * machSqr;

            return cP90;
        }

        private static double AdjustCpForNonlinearEffects(
            double cP,
            double area,
            double firstDerivArea,
            double beta,
            double freestreamMach
        )
        {
            double nuFreestream = PrandtlMeyerExpansionAngle(freestreamMach, freestreamMach);
            double deflectionAngle = CalculateEquivalentDeflectionAngle(cP, area, firstDerivArea, freestreamMach, beta);

            return cPPrandtlMeyerExpansion(freestreamMach, nuFreestream, deflectionAngle);
        }

        private static double CalculateEquivalentDeflectionAngle(
            double linCp,
            double area,
            double firstDerivArea,
            double machNumber,
            double beta
        )
        {
            double turnAngle = area * Math.PI;
            turnAngle = 2.0 * Math.Sqrt(turnAngle);
            turnAngle = firstDerivArea / turnAngle;
            turnAngle = Math.Atan(turnAngle);

            double uI = linCp * -0.5;
            double uO = AdjustVelForFinitePressure(-turnAngle / beta);

            double machI = machNumber * (1.0 + uI);
            double machO = machNumber * (1.0 + uO);

            double nuI = PrandtlMeyerExpansionAngle(machI, machNumber);
            double nuO = PrandtlMeyerExpansionAngle(machO, machNumber);

            double totalTurnAngle = turnAngle + nuO - nuI;
            return totalTurnAngle;
        }

        private static double cPPrandtlMeyerExpansion(double machNumber, double nuFreestream, double deflectionAngle)
        {
            double nu = nuFreestream - deflectionAngle;

            if (nu > 180 / (130.45 * Math.PI))
                nu = 180 / (130.45 * Math.PI);

            double effectiveFactor = nu / 130.45 * 180 / Math.PI;
            if (effectiveFactor < 0)
                effectiveFactor = 0;
            double exp = -0.42 * Math.Sqrt(effectiveFactor);
            exp += 0.313 * effectiveFactor;
            exp += 0.56;

            double effectiveMach = 1 - Math.Pow(effectiveFactor, exp);
            effectiveMach = 1 / effectiveMach;

            double cP = StagPresRatio(effectiveMach, nu) / StagPresRatio(machNumber, nuFreestream);
            cP--;
            cP /= 0.7 * machNumber * machNumber;
            return cP;
        }

        private static double StagPresRatio(double machNumber, double nu)
        {
            double ratio = nu + Math.Acos(1 / machNumber);
            ratio *= 2.0 / Math.Sqrt(6.0);
            ratio = 1 + Math.Cos(ratio);
            ratio /= 2.4;
            ratio = Math.Pow(ratio, 3.5);

            return ratio;
        }

        private static double PrandtlMeyerExpansionAngle(double localMach, double freestreamMach)
        {
            double nu;
            if (localMach < 1.0)
            {
                double freestreamNu = PrandtlMeyerExpansionAngle(freestreamMach, freestreamMach);
                nu = 1.0 - localMach;
                nu *= nu;
                nu *= freestreamNu - Math.PI * 0.5;
            }
            else
            {
                nu = localMach * localMach - 1.0;
                nu /= 6.0;
                nu = Math.Sqrt(nu);
                nu = Math.Atan(nu) * Math.Sqrt(6.0);
                nu -= Math.Acos(1 / localMach);
            }

            return nu;
        }

        private static double CalculateHypersonicDrag(double lowArea, double highArea, double sectionThickness)
        {
            if (lowArea >= highArea)
                return 0;

            double r1 = Math.Sqrt(lowArea / Math.PI);
            double r2 = Math.Sqrt(highArea / Math.PI);

            double radDiff = r2 - r1;
            double radDiffSq = radDiff * radDiff;

            double drag = sectionThickness * sectionThickness + radDiffSq;
            drag = 2d * Math.PI / drag;
            drag *= radDiff * radDiffSq * (r1 + r2);

            return -drag; //force is negative
        }

        // ReSharper disable once UnusedMember.Local
        private double CalcAllTransonicWaveDrag(
            VoxelCrossSection[] sections,
            int front,
            int numSections,
            double sectionThickness
        )
        {
            double drag = 0;

            for (int j = 0; j < numSections; j++)
            {
                double accumulator = 0;

                double Lj = (j + 0.5) * Math.Log(j + 0.5);
                if (j > 0)
                    Lj -= (j - 0.5) * Math.Log(j - 0.5);

                for (int i = j; i < numSections; i++)
                    accumulator += sections[front + i].secondAreaDeriv * sections[front + i - j].secondAreaDeriv;

                drag += accumulator * Lj;
            }

            drag *= -sectionThickness * sectionThickness / Math.PI;
            return drag;
        }

        // ReSharper disable once UnusedMember.Local
        private double CalculateTransonicWaveDrag(
            int i,
            int index,
            int numSections,
            int front,
            double sectionThickness,
            double cutoff
        )
        {
            double currentSectAreaCrossSection = MathClampAbs(_vehicleCrossSection[index].secondAreaDeriv, cutoff);

            if (currentSectAreaCrossSection.NearlyEqual(0)) //quick escape for 0 cross-section section drag
                return 0;

            int limDoubleDrag = Math.Min(i, numSections - i);
            double sectionThicknessSq = sectionThickness * sectionThickness;

            double lj3rdTerm = Math.Log(sectionThickness) - 1;

            double lj2ndTerm = 0.5 * Math.Log(0.5);
            double drag = currentSectAreaCrossSection * (lj2ndTerm + lj3rdTerm);


            for (int j = 1; j <= limDoubleDrag; j++) //section of influence from ahead and behind
            {
                double thisLj = (j + 0.5) * Math.Log(j + 0.5);
                double tmp = thisLj;
                thisLj -= lj2ndTerm;
                lj2ndTerm = tmp;

                tmp = MathClampAbs(_vehicleCrossSection[index + j].secondAreaDeriv, cutoff);
                tmp += MathClampAbs(_vehicleCrossSection[index - j].secondAreaDeriv, cutoff);

                thisLj += lj3rdTerm;

                drag += tmp * thisLj;
            }

            if (i < numSections - i)
                for (int j = 2 * i + 1; j < numSections; j++)
                {
                    double thisLj = (j - i + 0.5) * Math.Log(j - i + 0.5);
                    double tmp = thisLj;
                    thisLj -= lj2ndTerm;
                    lj2ndTerm = tmp;

                    tmp = MathClampAbs(_vehicleCrossSection[j + front].secondAreaDeriv, cutoff);

                    thisLj += lj3rdTerm;

                    drag += tmp * thisLj;
                }
            else if (i > numSections - i)
                for (int j = numSections - 2 * i - 2; j >= 0; j--)
                {
                    double thisLj = (i - j + 0.5) * Math.Log(i - j + 0.5);
                    double tmp = thisLj;
                    thisLj -= lj2ndTerm;
                    lj2ndTerm = tmp;

                    tmp = MathClampAbs(_vehicleCrossSection[j + front].secondAreaDeriv, cutoff);

                    thisLj += lj3rdTerm;

                    drag += tmp * thisLj;
                }

            drag *= sectionThicknessSq;
            drag /= 2 * Math.PI;
            drag *= currentSectAreaCrossSection;
            return drag;
        }

        private static double CalculateCriticalMachNumber(double finenessRatio)
        {
            if (finenessRatio > 10)
                return 0.925;
            if (finenessRatio > 6)
                return 0.00625 * finenessRatio + 0.8625;
            if (finenessRatio > 4)
                return 0.025 * finenessRatio + 0.75;
            if (finenessRatio > 3)
                return 0.07 * finenessRatio + 0.57;
            if (finenessRatio > 1.5)
                return 0.33 * finenessRatio - 0.21;
            return 0.285;
        }

        private double CriticalMachFactorForUnsmoothCrossSection(
            VoxelCrossSection[] crossSections,
            double finenessRatio,
            double sectionThickness
        )
        {
            double maxAbsRateOfChange = 0;
            double maxSecondDeriv = 0;
            double prevArea = 0;
            double invSectionThickness = 1 / sectionThickness;

            for (int i = 0; i < crossSections.Length; i++)
            {
                double currentArea = crossSections[i].area;
                double absRateOfChange = Math.Abs(currentArea - prevArea) * invSectionThickness;
                if (absRateOfChange > maxAbsRateOfChange)
                    maxAbsRateOfChange = absRateOfChange;
                prevArea = currentArea;
                maxSecondDeriv = Math.Max(maxSecondDeriv, Math.Abs(crossSections[i].secondAreaDeriv));
            }

            double maxCritMachAdjustmentFactor =
                2 * _maxCrossSectionArea + 5 * (0.5 * maxAbsRateOfChange + 0.3 * maxSecondDeriv);
            //will vary based on x = maxAbsRateOfChange / _maxCrossSectionArea from 1 @ x = 0 to 0.5 as x -> infinity
            maxCritMachAdjustmentFactor = 0.5 +
                                          (_maxCrossSectionArea -
                                           0.5 * (0.5 * maxAbsRateOfChange + 0.3 * maxSecondDeriv)) /
                                          maxCritMachAdjustmentFactor;

            double critAdjustmentFactor = 4 + finenessRatio;
            critAdjustmentFactor = 3.5 * (1 - maxCritMachAdjustmentFactor) / critAdjustmentFactor;
            critAdjustmentFactor += maxCritMachAdjustmentFactor;

            if (critAdjustmentFactor > 1)
                critAdjustmentFactor = 1;

            return critAdjustmentFactor;
        }

        private static double MathClampAbs(double value, double abs)
        {
            if (value < -abs)
                return -abs;
            return value > abs ? abs : value;
        }
    }
}
