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
using System.Text;
using System.Threading;
using ferram4;
using FerramAerospaceResearch.FARThreading;
using FerramAerospaceResearch.Geometry;
using KSPCommunityFixes;
using UnityEngine;

namespace FerramAerospaceResearch.FARPartGeometry
{
    public class VehicleVoxel
    {
        private const int MAX_SWEEP_PLANES_IN_QUEUE = 8;

        private const double RC = 0.5;
        private static int MAX_CHUNKS_IN_QUEUE = 4500;
        private static readonly Stack<VoxelChunk> clearedChunks = new Stack<VoxelChunk>();
        private static Stack<SweepPlanePoint[,]> clearedPlanes;

        private static int MAX_CHUNKS_ALLOWED;
        private static int chunksInUse;

        private static double maxLocation = 255;
        private static byte maxLocationByte = 255;
        private static bool useHigherResVoxels;
        private readonly object _locker = new object();
        private DebugVoxelMesh voxelMesh;

        private double invElementSize;
        private VoxelChunk[,,] voxelChunks;
        private Dictionary<Part, int> partPriorities;
        private int xLength, yLength, zLength;
        private int xCellLength, yCellLength, zCellLength;
        private int threadsQueued;

        private VehicleVoxel(Vessel vessel)
        {
            VoxelizationThreadpool.Instance.RunOnMainThread(() =>
            {
                voxelMesh = DebugVoxelMesh.Create(vessel == null ? null : vessel.vesselTransform);
                voxelMesh.gameObject.SetActive(false);
            });
        }

        public double ElementSize { get; private set; }

        public Vector3d LocalLowerRightCorner { get; private set; }
        public Bounds Bounds { get; private set; }

        public VoxelCrossSection[] EmptyCrossSectionArray
        {
            get
            {
                var array = new VoxelCrossSection[MaxArrayLength];
                for (int i = 0; i < array.Length; i++)
                    array[i].partSideAreaValues =
                        new Dictionary<Part, VoxelCrossSection.SideAreaValues>(ObjectReferenceEqualityComparer<Part>
                                                                                   .Default);
                return array;
            }
        }

        public int MaxArrayLength
        {
            get { return yCellLength + xCellLength + zCellLength; }
        }

        public double Volume { get; private set; }

        public static void VoxelSetup()
        {
            lock (clearedChunks)
            {
                useHigherResVoxels = FARSettingsScenarioModule.VoxelSettings.useHigherResVoxelPoints;
                maxLocation = useHigherResVoxels ? 255 : 15;

                maxLocationByte = (byte)maxLocation;

                if (clearedPlanes == null)
                {
                    clearedPlanes = new Stack<SweepPlanePoint[,]>();
                    for (int i = 0; i < MAX_SWEEP_PLANES_IN_QUEUE; i++)
                        clearedPlanes.Push(new SweepPlanePoint[1, 1]);

                    clearedPlanes.TrimExcess();
                }

                //2.2 / 64
                int chunksForQueue =
                    (int)Math.Ceiling(FARSettingsScenarioModule.VoxelSettings.numVoxelsControllableVessel * 0.034375);

                if (MAX_CHUNKS_IN_QUEUE == chunksForQueue)
                    return;
                clearedChunks.Clear();
                MAX_CHUNKS_IN_QUEUE = chunksForQueue;
                MAX_CHUNKS_ALLOWED = (int)Math.Ceiling(1.5 * MAX_CHUNKS_IN_QUEUE);

                ThreadSafeDebugLogger.Info("" + MAX_CHUNKS_IN_QUEUE + " " + MAX_CHUNKS_ALLOWED);
                for (int i = 0; i < MAX_CHUNKS_IN_QUEUE; i++)
                    clearedChunks.Push(new VoxelChunk(0, Vector3.zero, 0, 0, 0, null, useHigherResVoxels));

                clearedChunks.TrimExcess();
            }
        }

        public static VehicleVoxel CreateNewVoxel(
            List<GeometryPartModule> geoModules,
            int elementCount,
            bool multiThreaded = true,
            bool solidify = true,
            Vessel vessel = null
        )
        {
            var newVoxel = new VehicleVoxel(vessel);

            newVoxel.CreateVoxel(geoModules, elementCount, multiThreaded, solidify);

            return newVoxel;
        }

        private void CreateVoxel(
            List<GeometryPartModule> geoModules,
            int elementCount,
            bool multiThreaded,
            bool solidify
        )
        {
            var min = new Vector3d(double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity);
            var max = new Vector3d(double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity);

            partPriorities = new Dictionary<Part, int>(ObjectReferenceEqualityComparer<Part>.Default);
            //Determine bounds and "overriding parts" from geoModules
            foreach (GeometryPartModule m in geoModules)
            {
                if (m is null)
                    continue;
                bool cont = true;
                while (!m.Ready)
                {
                    Thread.SpinWait(5);

                    bool test;
                    if (VoxelizationThreadpool.RunInMainThread)
                        test = m.destroyed;
                    else
                        test = m == null;

                    if (!test)
                        continue;
                    cont = false;
                    break;
                }

                if (!cont || !m.Valid)
                    continue;

                Vector3d minBounds = m.overallMeshBounds.min;
                Vector3d maxBounds = m.overallMeshBounds.max;

                min = Vector3d.Min(min, minBounds);
                max = Vector3d.Max(max, maxBounds);

                if (!(m.part is null))
                    partPriorities.Add(m.part, PartPriorityLevel(m));
            }

            Vector3d size = max - min;

            Volume = size.x * size.y * size.z; //from bounds, get voxel volume

            if (double.IsInfinity(Volume)) //...if something broke, get out of here
            {
                ThreadSafeDebugLogger.Error("Voxel Volume was infinity; ending voxelization");
                return;
            }

            double elementVol = Volume / elementCount;
            ElementSize = Math.Pow(elementVol, 1d / 3d);
            invElementSize = 1 / ElementSize;

            double tmp = 0.125 * invElementSize;

            xLength = (int)Math.Ceiling(size.x * tmp) + 2;
            yLength = (int)Math.Ceiling(size.y * tmp) + 2;
            zLength = (int)Math.Ceiling(size.z * tmp) + 2;

            lock (clearedChunks) //make sure that we can actually voxelize without breaking the memory limits
            {
                while (chunksInUse >= MAX_CHUNKS_ALLOWED)
                {
                    ThreadSafeDebugLogger.Info("Voxel waiting for chunks to be released");
                    Monitor.Wait(clearedChunks);
                }

                chunksInUse += xLength * yLength * zLength;
            }

            xCellLength = xLength * 8;
            yCellLength = yLength * 8;
            zCellLength = zLength * 8;

            //this will be the distance from the center to the edges of the voxel object
            var extents = new Vector3d
            {
                x = xLength * 4 * ElementSize,
                y = yLength * 4 * ElementSize,
                z = zLength * 4 * ElementSize
            };

            Vector3d center = (max + min) * 0.5f; //Center of the vessel

            //This places the center of the voxel at the center of the vehicle to achieve maximum symmetry
            LocalLowerRightCorner = center - extents;
            Bounds = new Bounds(center, 2 * extents);

            voxelChunks = new VoxelChunk[xLength, yLength, zLength];

            try
            {
                BuildVoxel(geoModules, multiThreaded, solidify);
            }
            catch (Exception e)
            {
                ThreadSafeDebugLogger.Exception(e);
            }
        }

        private static int PartPriorityLevel(GeometryPartModule g)
        {
            if (g.part is null)
                return int.MinValue;

            PartModuleList modules = g.part.Modules;

            if (g.HasCrossSectionAdjusters && g.MaxCrossSectionAdjusterArea > 0)
                return int.MaxValue;
            if (modules.Contains("ProceduralFairingSide") || g.part.HasModuleImplementingFast<ModuleProceduralFairing>())
                return 3;
            if (modules.Contains("ProceduralFairingBase"))
                return 2;
            if (g.part.HasModuleImplementingFast<FARControllableSurface>() ||
                g.part.HasModuleImplementingFast<ModuleRCS>() ||
                g.part.HasModuleImplementingFast<ModuleEngines>())
                return 1;

            return -1;
        }

        private void BuildVoxel(List<GeometryPartModule> geoModules, bool multiThreaded, bool solidify)
        {
            threadsQueued = Environment.ProcessorCount - 1;

            if (!multiThreaded)
            //Go through it backwards; this ensures that children (and so interior to cargo bay parts) are handled first
            {
                foreach (GeometryPartModule m in geoModules)
                {
                    if (!m.Valid)
                        continue;
                    foreach (GeometryMesh mesh in m.meshDataList)
                        if (mesh.valid && mesh.gameObjectActiveInHierarchy)
                            UpdateFromMesh(mesh, m.part);
                }
            }
            else
            {
                int count = threadsQueued;
                for (int i = 0; i < count; i++)
                {
                    var meshParams = new VoxelShellMeshParams(geoModules.Count * i / count,
                                                              (i + 1) * geoModules.Count / count,
                                                              geoModules);

                    ThreadPool.QueueUserWorkItem(UpdateFromMesh, meshParams);
                }

                lock (_locker)
                {
                    while (threadsQueued > 0)
                        Monitor.Wait(_locker);
                }
            }

            if (!solidify)
                return;
            threadsQueued = 2;

            if (multiThreaded)
            {
                var data = new VoxelSolidParams(0, yLength / 2 * 8, true);
                ThreadPool.QueueUserWorkItem(SolidifyVoxel, data);
                data = new VoxelSolidParams(yLength / 2 * 8, yCellLength, false);
                ThreadPool.QueueUserWorkItem(SolidifyVoxel, data);
            }
            else
            {
                SolidifyVoxel(0, yCellLength, false);
            }

            if (!multiThreaded)
                return;
            lock (_locker)
            {
                while (threadsQueued > 0)
                    Monitor.Wait(_locker);
            }
        }

        public void CleanupVoxel()
        {
            RecycleVoxelChunks();
            ClearVisualVoxels();
        }


        public void RecycleVoxelChunks()
        {
            lock (clearedChunks)
            {
                RecycleChunksFromArray();
                chunksInUse -= xLength * yLength * zLength;
                Monitor.Pulse(clearedChunks);
            }
        }

        private void RecycleChunksFromArray()
        {
            for (int i = 0; i < xLength; i++)
                for (int j = 0; j < yLength; j++)
                    for (int k = 0; k < zLength; k++)
                    {
                        VoxelChunk chunk = voxelChunks[i, j, k];
                        if (chunk == null)
                            continue;

                        chunk.ClearChunk();

                        if (clearedChunks.Count < MAX_CHUNKS_IN_QUEUE)
                            clearedChunks.Push(chunk);
                        else
                            return;
                    }
        }

        public void CrossSectionData(
            VoxelCrossSection[] crossSections,
            Vector3 orientationVector,
            out int frontIndex,
            out int backIndex,
            out double sectionThickness,
            out double maxCrossSectionArea
        )
        {
            //TODO: Look into setting better limits for iterating over sweep plane to improve off-axis performance

            Vector4d plane = CalculateEquationOfSweepPlane(orientationVector, out double wInc);

            double x = Math.Abs(plane.x);
            double y = Math.Abs(plane.y);
            double z = Math.Abs(plane.z);

            double elementArea = ElementSize * ElementSize;

            bool frontIndexFound = false;
            frontIndex = 0;
            backIndex = crossSections.Length - 1;

            sectionThickness = ElementSize;

            var sectionNormalToVesselCoords = Matrix4x4.TRS(Vector3.zero,
                                                            Quaternion.FromToRotation(new Vector3(0, 0, 1),
                                                                orientationVector),
                                                            Vector3.one);
            Matrix4x4 vesselToSectionNormal = sectionNormalToVesselCoords.inverse;

            //Code has multiple optimizations to take advantage of the limited range of values that are included.  They are listed below
            //(int)Math.Ceiling(x) -> (int)(x + 1)      for x > 0
            //(int)Math.Round(x) -> (int)(x + 0.5f)     for x > 0
            //(int)Math.Floor(x) -> (int)(x)            for x > 0

            //Check y first, since it is most likely to be the flow direction
            if (y >= z && y >= x)
            {
                int sectionCount = yCellLength + (int)(xCellLength * x / y + 1) + (int)(zCellLength * z / y + 1);
                sectionCount = Math.Min(sectionCount, crossSections.Length);
                double angleSizeIncreaseFactor = Math.Sqrt((x + y + z) / y);
                //account for different angles effects on voxel cube's projected area
                elementArea *= angleSizeIncreaseFactor;

                ThreadSafeDebugLogger.Info("Voxel Element CrossSection Area: " + elementArea);

                double invMag = 1 / Math.Sqrt(x * x + y * y + z * z);

                sectionThickness *= y * invMag;

                double invYPlane = 1 / plane.y;

                plane.x *= invYPlane;
                plane.z *= invYPlane;
                plane.w *= invYPlane;

                wInc *= invYPlane;

                //shift the plane to auto-account for midpoint rounding, allowing it to be simple casts to int
                plane.w -= 0.5;

                for (int m = 0; m < sectionCount; m++)
                {
                    double areaCount = 0;
                    double centx = 0;
                    double centy = 0;
                    double centz = 0;

                    double i_xx = 0, i_xy = 0, i_yy = 0;
                    Dictionary<Part, VoxelCrossSection.SideAreaValues> partSideAreas =
                        crossSections[m].partSideAreaValues;
                    partSideAreas.Clear();

                    //Overall ones iterate over the actual voxel indices (to make use of the equation of the plane) but are used to get chunk indices
                    for (int iOverall = 0; iOverall < xCellLength; iOverall += 8)
                        for (int kOverall = 0; kOverall < zCellLength; kOverall += 8)
                        {
                            int jSect1, jSect3;

                            //Determine high and low points on this quad of the plane
                            if (plane.x * plane.z > 0.0)
                            {
                                //End points of the plane
                                jSect1 = (int)-(plane.x * iOverall + plane.z * kOverall + plane.w);
                                jSect3 = (int)-(plane.x * (iOverall + 7) + plane.z * (kOverall + 7) + plane.w);
                            }
                            else
                            {
                                jSect1 = (int)-(plane.x * (iOverall + 7) + plane.z * kOverall + plane.w);
                                jSect3 = (int)-(plane.x * iOverall + plane.z * (kOverall + 7) + plane.w);
                            }

                            int jSect2 = (int)((jSect1 + jSect3) * 0.5); //Central point

                            int jMin = Math.Min(jSect1, jSect3);
                            int jMax = Math.Max(jSect1, jSect3) + 1;

                            jSect1 = jMin >> 3;
                            jSect2 >>= 3;
                            jSect3 = jMax - 1 >> 3;

                            if (jSect1 >= yLength) //if the smallest sect is above the limit, they all are
                                continue;

                            if (jSect3 < 0) //if the largest sect is below the limit, they all are
                                continue;

                            //If chunk indices are identical, only get that one and it's very simple; only need to check 1 and 3, because if any are different, it must be those
                            if (jSect1 == jSect3)
                            {
                                if (jSect1 < 0)
                                    continue;

                                VoxelChunk sect = voxelChunks[iOverall >> 3, jSect1, kOverall >> 3];

                                if (sect == null)
                                    continue;

                                //Finally, iterate over the chunk
                                for (int i = iOverall; i < iOverall + 8; i++)
                                {
                                    double tmp = plane.x * i + plane.w;
                                    for (int k = kOverall; k < kOverall + 8; k++)
                                    {
                                        int j = (int)-(plane.z * k + tmp);

                                        if (j < jMin || j > jMax)
                                            continue;

                                        int index = i + 8 * j + 64 * k;

                                        PartSizePair pair = sect.GetVoxelPartSizePairGlobalIndex(index);

                                        if (pair.part is null)
                                            continue;
                                        DetermineIfPartGetsForcesAndAreas(partSideAreas, pair, i, j, k);

                                        double size = pair.GetSize();
                                        if (size > 1.0)
                                            size = 1.0;

                                        areaCount += size;
                                        centx += i * size;
                                        centy += j * size;
                                        centz += k * size;

                                        Vector3 location = vesselToSectionNormal.MultiplyVector(new Vector3(i, j, k));
                                        i_xx += location.x * location.x * size;
                                        i_xy += location.x * location.y * size;
                                        i_yy += location.y * location.y * size;
                                    }
                                }
                            }
                            else //Two or three different indices requires separate handling
                            {
                                VoxelChunk sect1 = null, sect2 = null, sect3 = null;

                                int iSect = iOverall >> 3;
                                int kSect = kOverall >> 3;
                                bool validSects = false;
                                if (!(jSect1 < 0)) //this block ensures that there are sections here to check
                                {
                                    sect1 = voxelChunks[iSect, jSect1, kSect];
                                    if (sect1 != null)
                                        validSects = true;
                                }

                                if (!(jSect2 < 0 || jSect2 >= yLength))
                                {
                                    sect2 = voxelChunks[iSect, jSect2, kSect];
                                    if (sect2 != null && sect2 != sect1)
                                        validSects = true;
                                }

                                if (!(jSect3 >= yLength))
                                {
                                    sect3 = voxelChunks[iSect, jSect3, kSect];
                                    if (sect3 != null && sect3 != sect2 && sect3 != sect1)
                                        validSects = true;
                                }

                                if (!validSects)
                                    continue;

                                for (int i = iOverall; i < iOverall + 8; i++)
                                {
                                    double tmp = plane.x * i + plane.w;
                                    for (int k = kOverall; k < kOverall + 8; k++)
                                    {
                                        int j = (int)-(plane.z * k + tmp);

                                        if (j < jMin || j > jMax)
                                            continue;

                                        int index = i + 8 * j + 64 * k;

                                        PartSizePair pair;

                                        int jSect = j >> 3;
                                        if (jSect == jSect1 && sect1 != null)
                                            pair = sect1.GetVoxelPartSizePairGlobalIndex(index);
                                        else if (jSect == jSect2 && sect2 != null)
                                            pair = sect2.GetVoxelPartSizePairGlobalIndex(index);
                                        else if (jSect == jSect3 && sect3 != null)
                                            pair = sect3.GetVoxelPartSizePairGlobalIndex(index);
                                        else
                                            continue;

                                        if (pair.part is null)
                                            continue;
                                        DetermineIfPartGetsForcesAndAreas(partSideAreas, pair, i, j, k);

                                        double size = pair.GetSize();
                                        if (size > 1.0)
                                            size = 1.0;

                                        areaCount += size;
                                        centx += i * size;
                                        centy += j * size;
                                        centz += k * size;

                                        Vector3 location = vesselToSectionNormal.MultiplyVector(new Vector3(i, j, k));
                                        i_xx += location.x * location.x * size;
                                        i_xy += location.x * location.y * size;
                                        i_yy += location.y * location.y * size;
                                    }
                                }
                            }
                        }

                    var centroid = new Vector3d(centx, centy, centz);
                    if (areaCount > 0)
                    {
                        if (frontIndexFound)
                        {
                            backIndex = m;
                        }
                        else
                        {
                            frontIndexFound = true;
                            frontIndex = m;
                        }

                        centroid /= areaCount;
                    }

                    Vector3 localCentroid = vesselToSectionNormal.MultiplyVector(centroid);
                    i_xx -= areaCount * localCentroid.x * localCentroid.x;
                    i_xy -= areaCount * localCentroid.x * localCentroid.y;
                    i_yy -= areaCount * localCentroid.y * localCentroid.y;

                    double tanPrinAngle = TanPrincipalAxisAngle(i_xx, i_yy, i_xy);
                    Vector3 axis1 = new Vector3(1, 0, 0), axis2 = new Vector3(0, 0, 1);
                    double flatnessRatio = 1;

                    if (!tanPrinAngle.NearlyEqual(0))
                    {
                        axis1 = new Vector3(1, 0, (float)tanPrinAngle);
                        axis1.Normalize();
                        axis2 = new Vector3(axis1.z, 0, -axis1.x);

                        flatnessRatio = i_xy * axis2.z / axis2.x + i_xx;
                        flatnessRatio = (i_xy * tanPrinAngle + i_xx) / flatnessRatio;
                        flatnessRatio = Math.Sqrt(Math.Sqrt(flatnessRatio));
                    }

                    if (double.IsNaN(flatnessRatio))
                        flatnessRatio = 1;

                    Vector3 principalAxis;
                    if (flatnessRatio > 1)
                    {
                        principalAxis = axis1;
                    }
                    else
                    {
                        flatnessRatio = 1 / flatnessRatio;
                        principalAxis = axis2;
                    }

                    if (flatnessRatio > 10)
                        flatnessRatio = 10;

                    principalAxis = sectionNormalToVesselCoords.MultiplyVector(principalAxis);

                    crossSections[m].centroid = centroid * ElementSize + LocalLowerRightCorner;

                    if (double.IsNaN(areaCount))
                        ThreadSafeDebugLogger.Error("FAR VOXEL ERROR: areacount is NaN at section " + m);

                    crossSections[m].area = areaCount * elementArea;
                    crossSections[m].flatnessRatio = flatnessRatio;
                    crossSections[m].flatNormalVector = principalAxis;

                    plane.w += wInc;
                }
            }
            else if (x > y && x > z)
            {
                int sectionCount = xCellLength + (int)(yCellLength * y / x + 1) + (int)(zCellLength * z / x + 1);
                sectionCount = Math.Min(sectionCount, crossSections.Length);
                double angleSizeIncreaseFactor = Math.Sqrt((x + y + z) / x);
                //account for different angles effects on voxel cube's projected area
                elementArea *= angleSizeIncreaseFactor;

                ThreadSafeDebugLogger.Info("Voxel Element CrossSection Area: " + elementArea);

                double invMag = 1 / Math.Sqrt(x * x + y * y + z * z);

                sectionThickness *= x * invMag;

                double i_xx = 0, i_xy = 0, i_yy = 0;

                double invXPlane = 1 / plane.x;

                plane.y *= invXPlane;
                plane.z *= invXPlane;
                plane.w *= invXPlane;

                wInc *= invXPlane;

                for (int m = 0; m < sectionCount; m++)
                {
                    double areaCount = 0;
                    double centx = 0;
                    double centy = 0;
                    double centz = 0;

                    Dictionary<Part, VoxelCrossSection.SideAreaValues> partSideAreas =
                        crossSections[m].partSideAreaValues;
                    partSideAreas.Clear();

                    for (int jOverall = 0; jOverall < yCellLength; jOverall += 8)
                        for (int kOverall = 0; kOverall < zCellLength; kOverall += 8)
                        {
                            int iSect1, iSect3;

                            if (plane.y * plane.z > 0)
                            {
                                iSect1 = (int)(-(plane.y * jOverall + plane.z * kOverall + plane.w) + 0.5);
                                iSect3 = (int)(-(plane.y * (jOverall + 7) + plane.z * (kOverall + 7) + plane.w) + 0.5);
                            }
                            else
                            {
                                iSect1 = (int)(-(plane.y * (jOverall + 7) + plane.z * kOverall + plane.w) + 0.5);
                                iSect3 = (int)(-(plane.y * jOverall + plane.z * (kOverall + 7) + plane.w) + 0.5);
                            }

                            int iSect2 = (int)((iSect1 + iSect3) * 0.5);

                            int iMin = Math.Min(iSect1, iSect3);
                            int iMax = Math.Max(iSect1, iSect3) + 1;

                            iSect1 = iMin >> 3;
                            iSect2 >>= 3;
                            iSect3 = iMax - 1 >> 3;

                            if (iSect1 >= xLength) //if the smallest sect is above the limit, they all are
                                continue;

                            if (iSect3 < 0) //if the largest sect is below the limit, they all are
                                continue;

                            if (iSect1 == iSect3)
                            {
                                if (iSect1 < 0)
                                    continue;

                                VoxelChunk sect = voxelChunks[iSect1, jOverall >> 3, kOverall >> 3];

                                if (sect == null)
                                    continue;

                                for (int j = jOverall; j < jOverall + 8; j++)
                                {
                                    double tmp = plane.y * j + plane.w;
                                    for (int k = kOverall; k < kOverall + 8; k++)
                                    {
                                        int i = (int)(-(plane.z * k + tmp) + 0.5);

                                        if (i < iMin || i > iMax)
                                            continue;

                                        int index = i + 8 * j + 64 * k;

                                        PartSizePair pair = sect.GetVoxelPartSizePairGlobalIndex(index);

                                        if (pair.part is null)
                                            continue;
                                        DetermineIfPartGetsForcesAndAreas(partSideAreas, pair, i, j, k);

                                        double size = pair.GetSize();
                                        if (size > 1.0)
                                            size = 1.0;

                                        areaCount += size;
                                        centx += i * size;
                                        centy += j * size;
                                        centz += k * size;

                                        Vector3 location = vesselToSectionNormal.MultiplyVector(new Vector3(i, j, k));
                                        i_xx += location.x * location.x * size;
                                        i_xy += location.x * location.y * size;
                                        i_yy += location.y * location.y * size;
                                    }
                                }
                            }
                            else
                            {
                                VoxelChunk sect1 = null, sect2 = null, sect3 = null;

                                int jSect = jOverall >> 3;
                                int kSect = kOverall >> 3;

                                bool validSects = false;
                                if (!(iSect1 < 0))
                                {
                                    sect1 = voxelChunks[iSect1, jSect, kSect];
                                    if (sect1 != null)
                                        validSects = true;
                                }

                                if (!(iSect2 < 0 || iSect2 >= xLength))
                                {
                                    sect2 = voxelChunks[iSect2, jSect, kSect];
                                    if (sect2 != null && sect2 != sect1)
                                        validSects = true;
                                }

                                if (!(iSect3 >= xLength))
                                {
                                    sect3 = voxelChunks[iSect3, jSect, kSect];
                                    if (sect3 != null && sect3 != sect2 && sect3 != sect1)
                                        validSects = true;
                                }

                                if (!validSects)
                                    continue;

                                for (int j = jOverall; j < jOverall + 8; j++)
                                {
                                    double tmp = plane.y * j + plane.w;
                                    for (int k = kOverall; k < kOverall + 8; k++)
                                    {
                                        int i = (int)(-(plane.z * k + tmp) + 0.5);

                                        if (i < iMin || i > iMax)
                                            continue;

                                        int index = i + 8 * j + 64 * k;

                                        PartSizePair pair;

                                        int iSect = i >> 3;
                                        if (iSect == iSect1 && sect1 != null)
                                            pair = sect1.GetVoxelPartSizePairGlobalIndex(index);
                                        else if (iSect == iSect2 && sect2 != null)
                                            pair = sect2.GetVoxelPartSizePairGlobalIndex(index);
                                        else if (iSect == iSect3 && sect3 != null)
                                            pair = sect3.GetVoxelPartSizePairGlobalIndex(index);
                                        else
                                            continue;

                                        if (pair.part is null)
                                            continue;
                                        DetermineIfPartGetsForcesAndAreas(partSideAreas, pair, i, j, k);

                                        double size = pair.GetSize();
                                        if (size > 1.0)
                                            size = 1.0;

                                        areaCount += size;
                                        centx += i * size;
                                        centy += j * size;
                                        centz += k * size;

                                        Vector3 location = vesselToSectionNormal.MultiplyVector(new Vector3(i, j, k));
                                        i_xx += location.x * location.x * size;
                                        i_xy += location.x * location.y * size;
                                        i_yy += location.y * location.y * size;
                                    }
                                }
                            }
                        }

                    var centroid = new Vector3d(centx, centy, centz);
                    if (areaCount > 0)
                    {
                        if (frontIndexFound)
                        {
                            backIndex = m;
                        }
                        else
                        {
                            frontIndexFound = true;
                            frontIndex = m;
                        }

                        centroid /= areaCount;
                    }

                    Vector3 localCentroid = vesselToSectionNormal.MultiplyVector(centroid);
                    i_xx -= areaCount * localCentroid.x * localCentroid.x;
                    i_xy -= areaCount * localCentroid.x * localCentroid.y;
                    i_yy -= areaCount * localCentroid.y * localCentroid.y;

                    double tanPrinAngle = TanPrincipalAxisAngle(i_xx, i_yy, i_xy);
                    Vector3 axis1 = new Vector3(1, 0, 0), axis2 = new Vector3(0, 0, 1);
                    double flatnessRatio = 1;

                    if (!tanPrinAngle.NearlyEqual(0))
                    {
                        axis1 = new Vector3(1, 0, (float)tanPrinAngle);
                        axis1.Normalize();
                        axis2 = new Vector3(axis1.z, 0, -axis1.x);

                        flatnessRatio = i_xy * axis2.z / axis2.x + i_xx;
                        flatnessRatio = (i_xy * tanPrinAngle + i_xx) / flatnessRatio;
                        flatnessRatio = Math.Sqrt(Math.Sqrt(flatnessRatio));
                    }

                    if (double.IsNaN(flatnessRatio))
                        flatnessRatio = 1;

                    Vector3 principalAxis;
                    if (flatnessRatio > 1)
                    {
                        principalAxis = axis1;
                    }
                    else
                    {
                        flatnessRatio = 1 / flatnessRatio;
                        principalAxis = axis2;
                    }

                    if (flatnessRatio > 10)
                        flatnessRatio = 10;

                    principalAxis = sectionNormalToVesselCoords.MultiplyVector(principalAxis);

                    crossSections[m].centroid = centroid * ElementSize + LocalLowerRightCorner;

                    if (double.IsNaN(areaCount))
                        ThreadSafeDebugLogger.Error("FAR VOXEL ERROR: areacount is NaN at section " + m);

                    crossSections[m].area = areaCount * elementArea;
                    crossSections[m].flatnessRatio = flatnessRatio;
                    crossSections[m].flatNormalVector = principalAxis;

                    plane.w += wInc;
                }
            }
            else
            {
                int sectionCount = zCellLength + (int)(xCellLength * x / z + 1) + (int)(yCellLength * y / z + 1);
                sectionCount = Math.Min(sectionCount, crossSections.Length);
                //account for different angles effects on voxel cube's projected area
                double angleSizeIncreaseFactor = Math.Sqrt((x + y + z) / z);
                //account for different angles effects on voxel cube's projected area
                elementArea *= angleSizeIncreaseFactor;

                ThreadSafeDebugLogger.Info("Voxel Element CrossSection Area: " + elementArea);

                double invMag = 1 / Math.Sqrt(x * x + y * y + z * z);

                sectionThickness *= z * invMag;

                double i_xx = 0, i_xy = 0, i_yy = 0;

                double invZPlane = 1 / plane.z;

                plane.y *= invZPlane;
                plane.x *= invZPlane;
                plane.w *= invZPlane;

                wInc *= invZPlane;

                for (int m = 0; m < sectionCount; m++)
                {
                    double areaCount = 0;
                    double centx = 0;
                    double centy = 0;
                    double centz = 0;

                    Dictionary<Part, VoxelCrossSection.SideAreaValues> partSideAreas =
                        crossSections[m].partSideAreaValues;
                    partSideAreas.Clear();

                    //Overall ones iterate over the actual voxel indices (to make use of the equation of the plane) but are used to get chunk indices
                    for (int iOverall = 0; iOverall < xCellLength; iOverall += 8)
                        for (int jOverall = 0; jOverall < yCellLength; jOverall += 8)
                        {
                            int kSect1, kSect3;

                            //Determine high and low points on this quad of the plane
                            if (plane.x * plane.y > 0)
                            {
                                kSect1 = (int)(-(plane.x * iOverall + plane.y * jOverall + plane.w) + 0.5);
                                kSect3 = (int)(-(plane.x * (iOverall + 7) + plane.y * (jOverall + 7) + plane.w) + 0.5);
                            }
                            else
                            {
                                kSect1 = (int)(-(plane.x * (iOverall + 7) + plane.y * jOverall + plane.w) + 0.5);
                                kSect3 = (int)(-(plane.x * iOverall + plane.y * (jOverall + 7) + plane.w) + 0.5);
                            }

                            int kSect2 = (int)((kSect1 + kSect3) * 0.5);

                            int kMin = Math.Min(kSect1, kSect3);
                            int kMax = Math.Max(kSect1, kSect3) + 1;

                            kSect1 = kMin >> 3;
                            kSect2 >>= 3;
                            kSect3 = kMax - 1 >> 3;

                            if (kSect1 >= zLength) //if the smallest sect is above the limit, they all are
                                continue;

                            if (kSect3 < 0) //if the largest sect is below the limit, they all are
                                continue;

                            //If chunk indices are identical, only get that one and it's very simple
                            if (kSect1 == kSect3)
                            {
                                if (kSect1 < 0)
                                    continue;

                                VoxelChunk sect = voxelChunks[iOverall >> 3, jOverall >> 3, kSect1];

                                if (sect == null)
                                    continue;

                                //Finally, iterate over the chunk
                                for (int i = iOverall; i < iOverall + 8; i++)
                                {
                                    double tmp = plane.x * i + plane.w;
                                    for (int j = jOverall; j < jOverall + 8; j++)
                                    {
                                        int k = (int)(-(plane.y * j + tmp) + 0.5);


                                        if (k < kMin || k > kMax)
                                            continue;

                                        int index = i + 8 * j + 64 * k;

                                        PartSizePair pair = sect.GetVoxelPartSizePairGlobalIndex(index);

                                        if (pair.part is null)
                                            continue;
                                        DetermineIfPartGetsForcesAndAreas(partSideAreas, pair, i, j, k);

                                        double size = pair.GetSize();
                                        if (size > 1.0)
                                            size = 1.0;

                                        areaCount += size;
                                        centx += i * size;
                                        centy += j * size;
                                        centz += k * size;

                                        Vector3 location = vesselToSectionNormal.MultiplyVector(new Vector3(i, j, k));
                                        i_xx += location.x * location.x * size;
                                        i_xy += location.x * location.y * size;
                                        i_yy += location.y * location.y * size;
                                    }
                                }
                            }
                            else
                            {
                                VoxelChunk sect1 = null, sect2 = null, sect3 = null;

                                int iSect = iOverall >> 3;
                                int jSect = jOverall >> 3;

                                bool validSects = false;
                                //If indices are different, this section of the plane crosses two chunks
                                if (!(kSect1 < 0))
                                {
                                    sect1 = voxelChunks[iSect, jSect, kSect1];
                                    if (sect1 != null)
                                        validSects = true;
                                }

                                if (!(kSect2 < 0 || kSect2 >= zLength))
                                {
                                    sect2 = voxelChunks[iSect, jSect, kSect2];
                                    if (sect2 != null && sect2 != sect1)
                                        validSects = true;
                                }

                                if (!(kSect3 >= zLength))
                                {
                                    sect3 = voxelChunks[iSect, jSect, kSect3];
                                    if (sect3 != null && sect3 != sect2 && sect3 != sect1)
                                        validSects = true;
                                }

                                if (!validSects)
                                    continue;

                                for (int i = iOverall; i < iOverall + 8; i++)
                                {
                                    double tmp = plane.x * i + plane.w;
                                    for (int j = jOverall; j < jOverall + 8; j++)
                                    {
                                        int k = (int)(-(plane.y * j + tmp) + 0.5);

                                        if (k < kMin || k > kMax)
                                            continue;

                                        int index = i + 8 * j + 64 * k;

                                        PartSizePair pair;

                                        int kSect = k >> 3;
                                        if (kSect == kSect1 && sect1 != null)
                                            pair = sect1.GetVoxelPartSizePairGlobalIndex(index);
                                        else if (kSect == kSect2 && sect2 != null)
                                            pair = sect2.GetVoxelPartSizePairGlobalIndex(index);
                                        else if (kSect == kSect3 && sect3 != null)
                                            pair = sect3.GetVoxelPartSizePairGlobalIndex(index);
                                        else
                                            continue;

                                        if (pair.part is null)
                                            continue;
                                        DetermineIfPartGetsForcesAndAreas(partSideAreas, pair, i, j, k);

                                        double size = pair.GetSize();
                                        if (size > 1.0)
                                            size = 1.0;
                                        areaCount += size;
                                        centx += i * size;
                                        centy += j * size;
                                        centz += k * size;

                                        Vector3 location = vesselToSectionNormal.MultiplyVector(new Vector3(i, j, k));
                                        i_xx += location.x * location.x * size;
                                        i_xy += location.x * location.y * size;
                                        i_yy += location.y * location.y * size;
                                    }
                                }
                            }
                        }

                    var centroid = new Vector3d(centx, centy, centz);
                    if (areaCount > 0)
                    {
                        if (frontIndexFound)
                        {
                            backIndex = m;
                        }
                        else
                        {
                            frontIndexFound = true;
                            frontIndex = m;
                        }

                        centroid /= areaCount;
                    }

                    Vector3 localCentroid = vesselToSectionNormal.MultiplyVector(centroid);
                    i_xx -= areaCount * localCentroid.x * localCentroid.x;
                    i_xy -= areaCount * localCentroid.x * localCentroid.y;
                    i_yy -= areaCount * localCentroid.y * localCentroid.y;

                    double tanPrinAngle = TanPrincipalAxisAngle(i_xx, i_yy, i_xy);
                    Vector3 axis1 = new Vector3(1, 0, 0), axis2 = new Vector3(0, 0, 1);
                    double flatnessRatio = 1;

                    if (!tanPrinAngle.NearlyEqual(0))
                    {
                        axis1 = new Vector3(1, 0, (float)tanPrinAngle);
                        axis1.Normalize();
                        axis2 = new Vector3(axis1.z, 0, -axis1.x);

                        flatnessRatio = i_xy * axis2.z / axis2.x + i_xx;
                        flatnessRatio = (i_xy * tanPrinAngle + i_xx) / flatnessRatio;
                        flatnessRatio = Math.Sqrt(Math.Sqrt(flatnessRatio));
                    }

                    if (double.IsNaN(flatnessRatio))
                        flatnessRatio = 1;

                    Vector3 principalAxis;
                    if (flatnessRatio > 1)
                    {
                        principalAxis = axis1;
                    }
                    else
                    {
                        flatnessRatio = 1 / flatnessRatio;
                        principalAxis = axis2;
                    }

                    if (flatnessRatio > 10)
                        flatnessRatio = 10;

                    principalAxis = sectionNormalToVesselCoords.MultiplyVector(principalAxis);

                    crossSections[m].centroid = centroid * ElementSize + LocalLowerRightCorner;

                    if (double.IsNaN(areaCount))
                        ThreadSafeDebugLogger.Error("FAR VOXEL ERROR: areacount is NaN at section " + m);

                    crossSections[m].area = areaCount * elementArea;
                    crossSections[m].flatnessRatio = flatnessRatio;
                    crossSections[m].flatNormalVector = principalAxis;

                    plane.w += wInc;
                }
            }

            double denom = 1 / (sectionThickness * sectionThickness);
            maxCrossSectionArea = 0;


            for (int i = frontIndex; i <= backIndex; i++) //calculate 2nd derivs, raw
            {
                double areaM1, area0, areaP1;

                if (i == frontIndex) //forward difference for frontIndex
                {
                    areaM1 = crossSections[i].area;
                    area0 = crossSections[i + 1].area;
                    areaP1 = crossSections[i + 2].area;
                }
                else if (i == backIndex) //backward difference for backIndex
                {
                    areaM1 = crossSections[i - 2].area;
                    area0 = crossSections[i - 1].area;
                    areaP1 = crossSections[i].area;
                }
                else //central difference for all others
                {
                    areaM1 = crossSections[i - 1].area;
                    area0 = crossSections[i].area;
                    areaP1 = crossSections[i + 1].area;
                }

                double areaSecondDeriv = areaM1 + areaP1 - 2 * area0;
                areaSecondDeriv *= denom;

                crossSections[i].secondAreaDeriv = areaSecondDeriv;

                if (crossSections[i].area > maxCrossSectionArea)
                    maxCrossSectionArea = crossSections[i].area;
            }
        }

        private void DetermineIfPartGetsForcesAndAreas(
            Dictionary<Part, VoxelCrossSection.SideAreaValues> partSideAreas,
            PartSizePair voxel,
            int i,
            int j,
            int k
        )
        {
            // ReSharper disable BitwiseOperatorOnEnumWithoutFlags
            VoxelOrientationPlane filledPlanes = VoxelOrientationPlane.NONE;
            bool partGetsForces = true;

            Part p = voxel.part;

            if (!partSideAreas.TryGetValue(p, out VoxelCrossSection.SideAreaValues areas))
            {
                areas = new VoxelCrossSection.SideAreaValues();
                partGetsForces = false;
            }

            if (i + 1 >= xCellLength || !VoxelPointExistsAtPos(i + 1, j, k))
            {
                areas.iP += ElementSize * ElementSize;
                areas.exposedAreaCount++;
                partGetsForces = true;
            }
            else
            {
                filledPlanes |= VoxelOrientationPlane.X_UP;
            }

            if (i - 1 < 0 || !VoxelPointExistsAtPos(i - 1, j, k))
            {
                areas.iN += ElementSize * ElementSize;
                areas.exposedAreaCount++;
                partGetsForces = true;
            }
            else
            {
                filledPlanes |= VoxelOrientationPlane.X_DOWN;
            }

            if (j + 1 >= yCellLength || !VoxelPointExistsAtPos(i, j + 1, k))
            {
                areas.jP += ElementSize * ElementSize;
                areas.exposedAreaCount++;
                partGetsForces = true;
            }
            else
            {
                filledPlanes |= VoxelOrientationPlane.Y_UP;
            }

            if (j - 1 < 0 || !VoxelPointExistsAtPos(i, j - 1, k))
            {
                areas.jN += ElementSize * ElementSize;
                areas.exposedAreaCount++;
                partGetsForces = true;
            }
            else
            {
                filledPlanes |= VoxelOrientationPlane.Y_DOWN;
            }

            if (k + 1 >= zCellLength || !VoxelPointExistsAtPos(i, j, k + 1))
            {
                areas.kP += ElementSize * ElementSize;
                areas.exposedAreaCount++;
                partGetsForces = true;
            }
            else
            {
                filledPlanes |= VoxelOrientationPlane.Z_UP;
            }

            if (k - 1 < 0 || !VoxelPointExistsAtPos(i, j, k - 1))
            {
                areas.kN += ElementSize * ElementSize;
                areas.exposedAreaCount++;
                partGetsForces = true;
            }
            else
            {
                filledPlanes |= VoxelOrientationPlane.Z_DOWN;
            }

            if (partGetsForces)
            {
                areas.crossSectionalAreaCount++;
                partSideAreas[p] = areas;
            }

            voxel.SetFilledSides(filledPlanes);
            // ReSharper restore BitwiseOperatorOnEnumWithoutFlags
        }

        private static double TanPrincipalAxisAngle(double Ixx, double Iyy, double Ixy)
        {
            if (Ixx.NearlyEqual(Iyy))
                return 0;

            double tan2Angle = 2d * Ixy / (Ixx - Iyy);
            double tanAngle = 1 + tan2Angle * tan2Angle;
            tanAngle = Math.Sqrt(tanAngle);
            tanAngle++;
            tanAngle = tan2Angle / tanAngle;

            return tanAngle;
        }

        public void ClearVisualVoxels()
        {
            if (voxelMesh == null)
                return;
            VoxelizationThreadpool.Instance.RunOnMainThread(() =>
            {
                if (voxelMesh == null) { FARLogger.Debug($"Visual voxel has disappeared!"); return; }
                FARLogger.Debug("Clearing visual voxels");
                voxelMesh.gameObject.SetActive(false);
                voxelMesh.Clear();
            });
        }

        public void VisualizeVoxel(Matrix4x4 vesselLocalToWorldMatrix)
        {
            FARLogger.Debug("Creating visual voxels");
            // Unity's <from>to<to> matrix naming is confusing, would be better to use <to>from<from> to actually
            // correspond with multiplication order...
            Matrix4x4 vesselLocalToVoxelMeshMatrix = voxelMesh.transform.worldToLocalMatrix * vesselLocalToWorldMatrix;
            var builder = new DebugVoxel.Builder();
            var tintMap = new PartTint();
            voxelMesh.Clear(builder, xLength * yLength * zLength * 128, false);
            for (int i = 0; i < xLength; i++)
            {
                for (int j = 0; j < yLength; j++)
                {
                    for (int k = 0; k < zLength; k++)
                        voxelChunks[i, j, k]
                            ?.VisualizeVoxels(vesselLocalToVoxelMeshMatrix, tintMap, voxelMesh, builder);
                }
            }

            // TODO: should be a list view in GUI
            if (FARLogger.IsEnabledFor(LogLevel.Debug))
            {
                StringBuilder sb = StringBuilderCache.Acquire();
                sb.AppendLine("Tints applied:");
                foreach (KeyValuePair<Part, Color> pair in tintMap)
                {
                    sb.Append(pair.Key.name)
                      .Append(" (")
                      .Append(pair.Key.persistentId)
                      .Append(") = ")
                      .AppendLine(pair.Value.ToString());
                }

                FARLogger.Debug(sb);
                sb.Release();
            }

            VoxelizationThreadpool.Instance.RunOnMainThread(() =>
            {
                voxelMesh.Apply(builder);
                voxelMesh.gameObject.SetActive(true);
            });
        }

        //Only use to change size, not part
        private void SetVoxelPointNoLock(int i, int j, int k)
        {
            //Find the voxel section that this point points to

            int iSec = i >> 3;
            int jSec = j >> 3;
            int kSec = k >> 3;

            VoxelChunk section = voxelChunks[iSec, jSec, kSec];
            if (section == null)
            {
                lock (clearedChunks)
                {
                    if (clearedChunks.Count > 0)
                        section = clearedChunks.Pop();
                }

                if (section == null)
                    section = new VoxelChunk(ElementSize,
                                             LocalLowerRightCorner + new Vector3d(iSec, jSec, kSec) * ElementSize * 8,
                                             iSec * 8,
                                             jSec * 8,
                                             kSec * 8,
                                             partPriorities,
                                             useHigherResVoxels);
                else
                    section.SetChunk(ElementSize,
                                     LocalLowerRightCorner + new Vector3d(iSec, jSec, kSec) * ElementSize * 8,
                                     iSec * 8,
                                     jSec * 8,
                                     kSec * 8,
                                     partPriorities);

                voxelChunks[iSec, jSec, kSec] = section;
            }

            section.SetVoxelPointGlobalIndexNoLock(i + j * 8 + k * 64, maxLocationByte);
        }

        //Use when guaranteed that you will not attempt to write to the same section simultaneously
        // ReSharper disable once UnusedMember.Local
        private void SetVoxelPointPartOnlyNoLock(int i, int j, int k, Part part)
        {
            //Find the voxel section that this point points to

            int iSec = i >> 3;
            int jSec = j >> 3;
            int kSec = k >> 3;

            VoxelChunk section = voxelChunks[iSec, jSec, kSec];
            if (section == null)
            {
                lock (clearedChunks)
                {
                    if (clearedChunks.Count > 0)
                        section = clearedChunks.Pop();
                }

                if (section == null)
                    section = new VoxelChunk(ElementSize,
                                             LocalLowerRightCorner + new Vector3d(iSec, jSec, kSec) * ElementSize * 8,
                                             iSec * 8,
                                             jSec * 8,
                                             kSec * 8,
                                             partPriorities,
                                             useHigherResVoxels);
                else
                    section.SetChunk(ElementSize,
                                     LocalLowerRightCorner + new Vector3d(iSec, jSec, kSec) * ElementSize * 8,
                                     iSec * 8,
                                     jSec * 8,
                                     kSec * 8,
                                     partPriorities);

                voxelChunks[iSec, jSec, kSec] = section;
            }

            section.SetVoxelPointPartOnlyGlobalIndexNoLock(i + j * 8 + k * 64, part);
        }

        //Use when guaranteed that you will not attempt to write to the same section simultaneously
        private void SetVoxelPointNoLock(int i, int j, int k, Part part)
        {
            //Find the voxel section that this point points to

            int iSec = i >> 3;
            int jSec = j >> 3;
            int kSec = k >> 3;

            VoxelChunk section = voxelChunks[iSec, jSec, kSec];
            if (section == null)
            {
                lock (clearedChunks)
                {
                    if (clearedChunks.Count > 0)
                        section = clearedChunks.Pop();
                }

                if (section == null)
                    section = new VoxelChunk(ElementSize,
                                             LocalLowerRightCorner + new Vector3d(iSec, jSec, kSec) * ElementSize * 8,
                                             iSec * 8,
                                             jSec * 8,
                                             kSec * 8,
                                             partPriorities,
                                             useHigherResVoxels);
                else
                    section.SetChunk(ElementSize,
                                     LocalLowerRightCorner + new Vector3d(iSec, jSec, kSec) * ElementSize * 8,
                                     iSec * 8,
                                     jSec * 8,
                                     kSec * 8,
                                     partPriorities);

                voxelChunks[iSec, jSec, kSec] = section;
            }

            section.SetVoxelPointGlobalIndexNoLock(i + j * 8 + k * 64, part, maxLocationByte);
        }

        private void SetVoxelPoint(int i, int j, int k, Part part, VoxelOrientationPlane plane, byte location)
        {
            //Find the voxel section that this point points to

            int iSec = i >> 3;
            int jSec = j >> 3;
            int kSec = k >> 3;

            VoxelChunk section;

            lock (voxelChunks)
            {
                section = voxelChunks[iSec, jSec, kSec];
                if (section == null)
                {
                    lock (clearedChunks)
                    {
                        if (clearedChunks.Count > 0)
                            section = clearedChunks.Pop();
                    }

                    if (section == null)
                        section = new VoxelChunk(ElementSize,
                                                 LocalLowerRightCorner +
                                                 new Vector3d(iSec, jSec, kSec) * ElementSize * 8,
                                                 iSec * 8,
                                                 jSec * 8,
                                                 kSec * 8,
                                                 partPriorities,
                                                 useHigherResVoxels);
                    else
                        section.SetChunk(ElementSize,
                                         LocalLowerRightCorner + new Vector3d(iSec, jSec, kSec) * ElementSize * 8,
                                         iSec * 8,
                                         jSec * 8,
                                         kSec * 8,
                                         partPriorities);

                    voxelChunks[iSec, jSec, kSec] = section;
                }
            }

            section.SetVoxelPointGlobalIndex(i + j * 8 + k * 64, part, location, plane);
        }

        // ReSharper disable once UnusedMember.Local
        private VoxelChunk GetVoxelChunk(int i, int j, int k)
        {
            //Find the voxel section that this point points to
            int iSec = i >> 3;
            int jSec = j >> 3;
            int kSec = k >> 3;

            return voxelChunks[iSec, jSec, kSec];
        }

        private bool VoxelPointExistsAtPos(int i, int j, int k)
        {
            //Find the voxel section that this point points to

            int iSec = i >> 3;
            int jSec = j >> 3;
            int kSec = k >> 3;

            //No locks are needed because reading and writing are not done in different threads simultaneously
            VoxelChunk section = voxelChunks[iSec, jSec, kSec];
            return section != null && section.VoxelPointExistsGlobalIndex(i + j * 8 + k * 64);
        }

        private Part GetPartAtVoxelPos(int i, int j, int k)
        {
            //Find the voxel section that this point points to

            int iSec = i >> 3;
            int jSec = j >> 3;
            int kSec = k >> 3;

            //No locks are needed because reading and writing are not done in different threads simultaneously
            return voxelChunks[iSec, jSec, kSec]?.GetVoxelPartGlobalIndex(i + j * 8 + k * 64);
        }

        // ReSharper disable once UnusedMember.Local
        private Part GetPartAtVoxelPos(int i, int j, int k, ref VoxelChunk section)
        {
            return section.GetVoxelPartGlobalIndex(i + j * 8 + k * 64);
        }

        private void UpdateFromMesh(object meshParamsObject)
        {
            try
            {
                if (VoxelizationThreadpool.RunInMainThread)
                {
                    var meshes = new List<GeometryMesh>();
                    VoxelizationThreadpool.Instance.RunOnMainThread(() =>
                    {
                        var meshParams = (VoxelShellMeshParams)meshParamsObject;
                        for (int i = meshParams.lowerIndex; i < meshParams.upperIndex; i++)
                        {
                            GeometryPartModule module = meshParams.modules[i];
                            if (module == null || !module.Valid || module.meshDataList == null)
                                continue;

                            foreach (GeometryMesh mesh in module.meshDataList)
                                lock (mesh)
                                {
                                    if (mesh.meshTransform != null && mesh.gameObjectActiveInHierarchy && mesh.valid)
                                        meshes.Add(mesh);
                                }
                        }
                    });
                    foreach (GeometryMesh mesh in meshes)
                        UpdateFromMesh(mesh, mesh.part);
                }
                else
                {
                    var meshParams = (VoxelShellMeshParams)meshParamsObject;
                    for (int i = meshParams.lowerIndex; i < meshParams.upperIndex; i++)
                    {
                        GeometryPartModule module = meshParams.modules[i];
                        if (module == null || !module.Valid || module.meshDataList == null)
                            continue;

                        foreach (GeometryMesh mesh in module.meshDataList)
                        {
                            bool updateFromMesh = false;
                            lock (mesh)
                            {
                                if (mesh.meshTransform != null && mesh.gameObjectActiveInHierarchy && mesh.valid)
                                    updateFromMesh = true;
                            }

                            if (updateFromMesh)
                                UpdateFromMesh(mesh, mesh.part);
                        }
                    }
                }
            }
            catch (Exception e)
            {
                ThreadSafeDebugLogger.Exception(e);
            }
            finally
            {
                lock (_locker)
                {
                    threadsQueued--;
                    Monitor.Pulse(_locker);
                }
            }
        }

        private void UpdateFromMesh(GeometryMesh mesh, Part part)
        {
            for (int a = 0; a < mesh.triangles.Length; a += 3)
            {
                Vector3 vert1 = mesh.vertices[mesh.triangles[a]];
                Vector3 vert2 = mesh.vertices[mesh.triangles[a + 1]];
                Vector3 vert3 = mesh.vertices[mesh.triangles[a + 2]];

                CalculateVoxelShellForTriangle(vert1, vert2, vert3, part, mesh.invertXYZ);
            }
        }

        // ReSharper disable once UnusedMember.Local
        private void CalculateVoxelShellFromTinyMesh(Vector3 minMesh, Vector3 maxMesh, Part part)
        {
            Vector3 min = (minMesh - LocalLowerRightCorner) * invElementSize;
            Vector3 max = (maxMesh - LocalLowerRightCorner) * invElementSize;

            int lowerI = (int)min.x;
            int lowerJ = (int)min.y;
            int lowerK = (int)min.z;

            int upperI = (int)(max.x + 1);
            int upperJ = (int)(max.y + 1);
            int upperK = (int)(max.z + 1);

            lowerI = Math.Max(lowerI, 0);
            lowerJ = Math.Max(lowerJ, 0);
            lowerK = Math.Max(lowerK, 0);

            upperI = Math.Min(upperI, xCellLength - 1);
            upperJ = Math.Min(upperJ, yCellLength - 1);
            upperK = Math.Min(upperK, zCellLength - 1);

            for (int i = lowerI; i <= upperI; i++)
                for (int j = lowerJ; j <= upperJ; j++)
                    for (int k = lowerK; k <= upperK; k++)
                        SetVoxelPoint(i, j, k, part, VoxelOrientationPlane.FILL_VOXEL, 1);
        }

        private void CalculateVoxelShellForTriangle(
            Vector3 vert1,
            Vector3 vert2,
            Vector3 vert3,
            Part part,
            int invertXYZ
        )
        {
            Vector4 indexPlane = CalculateEquationOfPlaneInIndices(vert1, vert2, vert3);

            double x = Math.Abs(indexPlane.x);
            double y = Math.Abs(indexPlane.y);
            double z = Math.Abs(indexPlane.z);

            double testSum = x + y + z;
            if (testSum.NearlyEqual(0) || double.IsNaN(testSum) || double.IsInfinity(testSum))
                //after much user confusion, we're just gonna quietly swallow this error; need to change so there's a debug switch
                return;

            if (z >= y && z >= x * 0.999)
                VoxelShellTrianglePerpZ(indexPlane, vert1, vert2, vert3, part, invertXYZ);
            else if (x > z && x >= y)
                VoxelShellTrianglePerpX(indexPlane, vert1, vert2, vert3, part, invertXYZ);
            else
                VoxelShellTrianglePerpY(indexPlane, vert1, vert2, vert3, part, invertXYZ);
        }

        private void VoxelShellTrianglePerpX(
            Vector4 indexPlane,
            Vector3 vert1,
            Vector3 vert2,
            Vector3 vert3,
            Part part,
            int invertTri
        )
        {
            Vector3 vert1Proj = (vert1 - LocalLowerRightCorner) * invElementSize;
            Vector3 vert2Proj = (vert2 - LocalLowerRightCorner) * invElementSize;
            Vector3 vert3Proj = (vert3 - LocalLowerRightCorner) * invElementSize;

            Vector3 p1p2 = vert2Proj - vert1Proj;
            Vector3 p1p3 = vert3Proj - vert1Proj;

            int signW = -Math.Sign(Vector3.Cross(p1p2, p1p3).x) * invertTri;

            double dot12_12 = p1p2.z * p1p2.z + p1p2.y * p1p2.y;
            double dot12_13 = p1p2.z * p1p3.z + p1p2.y * p1p3.y;
            double dot13_13 = p1p3.z * p1p3.z + p1p3.y * p1p3.y;

            double invDenom = 1 / (dot12_12 * dot13_13 - dot12_13 * dot12_13);

            int lowJ = (int)(Math.Min(vert1Proj.y, Math.Min(vert2Proj.y, vert3Proj.y)) - 1);
            int highJ = (int)Math.Ceiling(Math.Max(vert1Proj.y, Math.Max(vert2Proj.y, vert3Proj.y)) + 1);
            int lowK = (int)(Math.Min(vert1Proj.z, Math.Min(vert2Proj.z, vert3Proj.z)) - 1);
            int highK = (int)Math.Ceiling(Math.Max(vert1Proj.z, Math.Max(vert2Proj.z, vert3Proj.z)) + 1);

            if (lowJ < 0)
                lowJ = 0;
            if (lowK < 0)
                lowK = 0;
            if (highJ >= yCellLength)
                highJ = yCellLength - 1;
            if (highK >= zCellLength)
                highK = zCellLength - 1;

            double invIndexPlaneX = 1 / indexPlane.x;

            for (int j = lowJ; j <= highJ; ++j)
                for (int k = lowK; k <= highK; ++k)
                {
                    var pt = new Vector3(0, j, k);
                    Vector3 p1TestPt = pt - vert1Proj;
                    double dot12_test = p1p2.z * p1TestPt.z + p1p2.y * p1TestPt.y;
                    double dot13_test = p1p3.z * p1TestPt.z + p1p3.y * p1TestPt.y;

                    double u = (dot13_13 * dot12_test - dot12_13 * dot13_test) * invDenom;
                    double v = (dot12_12 * dot13_test - dot12_13 * dot12_test) * invDenom;

                    double iFloat = -(indexPlane.y * j + indexPlane.z * k + indexPlane.w) * invIndexPlaneX;
                    int i = (int)Math.Round(iFloat);
                    if (i < 0 || i >= xCellLength)
                        continue;

                    pt.x = i;
                    p1TestPt.x = pt.x - vert1Proj.x;

                    VoxelOrientationPlane plane;
                    byte location;
                    double floatLoc;
                    if (u >= 0 && v >= 0 && u + v <= 1)
                    {
                        floatLoc = (i - iFloat) * signW + 0.5;
                        floatLoc *= maxLocation;

                        if (floatLoc > maxLocation)
                            floatLoc = maxLocation;
                        if (floatLoc < 0)
                            floatLoc = 0;

                        location = (byte)Math.Ceiling(floatLoc);
                        plane = signW <= 0 ? VoxelOrientationPlane.X_UP : VoxelOrientationPlane.X_DOWN;

                        SetVoxelPoint(i, j, k, part, plane, location);
                        continue;
                    }

                    Vector3 p2TestPt = pt - vert2Proj;
                    Vector3 p3TestPt = pt - vert3Proj;
                    if (u + v < 0.5 && p1TestPt.magnitude <= RC ||
                        (u < 0.5 || u + v > 0.5) && p2TestPt.magnitude <= RC ||
                        (v < 0.5 || u + v > 0.5) && p3TestPt.magnitude <= RC)
                    {
                        floatLoc = (i - iFloat) * signW + 0.5;
                        floatLoc *= maxLocation * 0.25d;

                        if (floatLoc > maxLocation)
                            floatLoc = maxLocation;
                        if (floatLoc < 0)
                            floatLoc = 0;

                        location = (byte)Math.Ceiling(floatLoc);
                        plane = signW <= 0 ? VoxelOrientationPlane.X_UP : VoxelOrientationPlane.X_DOWN;

                        SetVoxelPoint(i, j, k, part, plane, location);
                        continue;
                    }

                    bool validDistFromSide = false;
                    double distFromSide = DistanceFromSide(p1p2, p1TestPt);
                    if (distFromSide <= RC)
                    {
                        validDistFromSide = true;
                    }
                    else
                    {
                        distFromSide = DistanceFromSide(p1p3, p1TestPt);
                        if (distFromSide <= RC)
                        {
                            validDistFromSide = true;
                        }
                        else
                        {
                            distFromSide = DistanceFromSide(vert3Proj - vert2Proj, p2TestPt);
                            if (distFromSide <= RC)
                                validDistFromSide = true;
                        }
                    }

                    if (!validDistFromSide)
                        continue;
                    floatLoc = (i - iFloat) * signW + 0.5;
                    floatLoc *= maxLocation * (RC - distFromSide);

                    if (floatLoc > maxLocation)
                        floatLoc = maxLocation;
                    if (floatLoc < 0)
                        floatLoc = 0;

                    location = (byte)Math.Ceiling(floatLoc);
                    plane = signW <= 0 ? VoxelOrientationPlane.X_UP : VoxelOrientationPlane.X_DOWN;

                    SetVoxelPoint(i, j, k, part, plane, location);
                }
        }

        private void VoxelShellTrianglePerpY(
            Vector4 indexPlane,
            Vector3 vert1,
            Vector3 vert2,
            Vector3 vert3,
            Part part,
            int invertTri
        )
        {
            Vector3 vert1Proj = (vert1 - LocalLowerRightCorner) * invElementSize;
            Vector3 vert2Proj = (vert2 - LocalLowerRightCorner) * invElementSize;
            Vector3 vert3Proj = (vert3 - LocalLowerRightCorner) * invElementSize;

            Vector3 p1p2 = vert2Proj - vert1Proj;
            Vector3 p1p3 = vert3Proj - vert1Proj;

            int signW = -Math.Sign(Vector3.Cross(p1p2, p1p3).y) * invertTri;

            double dot12_12 = p1p2.x * p1p2.x + p1p2.z * p1p2.z;
            double dot12_13 = p1p2.x * p1p3.x + p1p2.z * p1p3.z;
            double dot13_13 = p1p3.x * p1p3.x + p1p3.z * p1p3.z;

            double invDenom = 1 / (dot12_12 * dot13_13 - dot12_13 * dot12_13);

            int lowI = (int)(Math.Min(vert1Proj.x, Math.Min(vert2Proj.x, vert3Proj.x)) - 1);
            int highI = (int)Math.Ceiling(Math.Max(vert1Proj.x, Math.Max(vert2Proj.x, vert3Proj.x)) + 1);
            int lowK = (int)(Math.Min(vert1Proj.z, Math.Min(vert2Proj.z, vert3Proj.z)) - 1);
            int highK = (int)Math.Ceiling(Math.Max(vert1Proj.z, Math.Max(vert2Proj.z, vert3Proj.z)) + 1);


            if (lowI < 0)
                lowI = 0;
            if (lowK < 0)
                lowK = 0;
            if (highI >= xCellLength)
                highI = xCellLength - 1;
            if (highK >= zCellLength)
                highK = zCellLength - 1;

            double invIndexPlaneY = 1 / indexPlane.y;

            for (int i = lowI; i <= highI; ++i)
                for (int k = lowK; k <= highK; ++k)
                {
                    var pt = new Vector3(i, 0, k);
                    Vector3 p1TestPt = pt - vert1Proj;
                    double dot12_test = p1p2.x * p1TestPt.x + p1p2.z * p1TestPt.z;
                    double dot13_test = p1p3.x * p1TestPt.x + p1p3.z * p1TestPt.z;

                    double u = (dot13_13 * dot12_test - dot12_13 * dot13_test) * invDenom;
                    double v = (dot12_12 * dot13_test - dot12_13 * dot12_test) * invDenom;

                    double jFloat = -(indexPlane.x * i + indexPlane.z * k + indexPlane.w) * invIndexPlaneY;
                    int j = (int)Math.Round(jFloat);

                    if (j < 0 || j >= yCellLength)
                        continue;

                    pt.y = j;
                    p1TestPt.y = pt.y - vert1Proj.y;

                    double floatLoc;
                    byte location;
                    VoxelOrientationPlane plane;
                    if (u >= 0 && v >= 0 && u + v <= 1)
                    {
                        floatLoc = (j - jFloat) * signW + 0.5;
                        floatLoc *= maxLocation;

                        if (floatLoc > maxLocation)
                            floatLoc = maxLocation;
                        if (floatLoc < 0)
                            floatLoc = 0;

                        location = (byte)Math.Ceiling(floatLoc);
                        plane = signW <= 0 ? VoxelOrientationPlane.Y_UP : VoxelOrientationPlane.Y_DOWN;

                        SetVoxelPoint(i, j, k, part, plane, location);
                        continue;
                    }

                    Vector3 p2TestPt = pt - vert2Proj;
                    Vector3 p3TestPt = pt - vert3Proj;

                    if (u + v < 0.5 && p1TestPt.magnitude <= RC ||
                        (u < 0.5 || u + v > 0.5) && p2TestPt.magnitude <= RC ||
                        (v < 0.5 || u + v > 0.5) && p3TestPt.magnitude <= RC)
                    {
                        floatLoc = (j - jFloat) * signW + 0.5;
                        floatLoc *= maxLocation * 0.25d;

                        if (floatLoc > maxLocation)
                            floatLoc = maxLocation;
                        if (floatLoc < 0)
                            floatLoc = 0;

                        location = (byte)Math.Ceiling(floatLoc);
                        plane = signW <= 0 ? VoxelOrientationPlane.Y_UP : VoxelOrientationPlane.Y_DOWN;

                        SetVoxelPoint(i, j, k, part, plane, location);
                        continue;
                    }

                    bool validDistFromSide = false;
                    double distFromSide = DistanceFromSide(p1p2, p1TestPt);
                    if (distFromSide <= RC)
                    {
                        validDistFromSide = true;
                    }
                    else
                    {
                        distFromSide = DistanceFromSide(p1p3, p1TestPt);
                        if (distFromSide <= RC)
                        {
                            validDistFromSide = true;
                        }
                        else
                        {
                            distFromSide = DistanceFromSide(vert3Proj - vert2Proj, p2TestPt);
                            if (distFromSide <= RC)
                                validDistFromSide = true;
                        }
                    }

                    if (!validDistFromSide)
                        continue;
                    floatLoc = (j - jFloat) * signW + 0.5;
                    floatLoc *= maxLocation * (RC - distFromSide);

                    if (floatLoc > maxLocation)
                        floatLoc = maxLocation;
                    if (floatLoc < 0)
                        floatLoc = 0;

                    location = (byte)Math.Ceiling(floatLoc);
                    plane = signW <= 0 ? VoxelOrientationPlane.Y_UP : VoxelOrientationPlane.Y_DOWN;

                    SetVoxelPoint(i, j, k, part, plane, location);
                }
        }

        private void VoxelShellTrianglePerpZ(
            Vector4 indexPlane,
            Vector3 vert1,
            Vector3 vert2,
            Vector3 vert3,
            Part part,
            int invertTri
        )
        {
            Vector3 vert1Proj = (vert1 - LocalLowerRightCorner) * invElementSize;
            Vector3 vert2Proj = (vert2 - LocalLowerRightCorner) * invElementSize;
            Vector3 vert3Proj = (vert3 - LocalLowerRightCorner) * invElementSize;

            Vector3 p1p2 = vert2Proj - vert1Proj;
            Vector3 p1p3 = vert3Proj - vert1Proj;

            int signW = -Math.Sign(Vector3.Cross(p1p2, p1p3).z) * invertTri;

            double dot12_12 = p1p2.x * p1p2.x + p1p2.y * p1p2.y;
            double dot12_13 = p1p2.x * p1p3.x + p1p2.y * p1p3.y;
            double dot13_13 = p1p3.x * p1p3.x + p1p3.y * p1p3.y;

            double invDenom = 1 / (dot12_12 * dot13_13 - dot12_13 * dot12_13);

            int lowI = (int)(Math.Min(vert1Proj.x, Math.Min(vert2Proj.x, vert3Proj.x)) - 1);
            int highI = (int)Math.Ceiling(Math.Max(vert1Proj.x, Math.Max(vert2Proj.x, vert3Proj.x)) + 1);
            int lowJ = (int)(Math.Min(vert1Proj.y, Math.Min(vert2Proj.y, vert3Proj.y)) - 1);
            int highJ = (int)Math.Ceiling(Math.Max(vert1Proj.y, Math.Max(vert2Proj.y, vert3Proj.y)) + 1);


            if (lowJ < 0)
                lowJ = 0;
            if (lowI < 0)
                lowI = 0;
            if (highJ >= yCellLength)
                highJ = yCellLength - 1;
            if (highI >= xCellLength)
                highI = xCellLength - 1;

            double invIndexPlaneZ = 1 / indexPlane.z;

            for (int i = lowI; i <= highI; ++i)
                for (int j = lowJ; j <= highJ; ++j)
                {
                    var pt = new Vector3(i, j, 0);
                    Vector3 p1TestPt = pt - vert1Proj;
                    double dot12_test = p1p2.x * p1TestPt.x + p1p2.y * p1TestPt.y;
                    double dot13_test = p1p3.x * p1TestPt.x + p1p3.y * p1TestPt.y;

                    double u = (dot13_13 * dot12_test - dot12_13 * dot13_test) * invDenom;
                    double v = (dot12_12 * dot13_test - dot12_13 * dot12_test) * invDenom;

                    double kFloat = -(indexPlane.x * i + indexPlane.y * j + indexPlane.w) * invIndexPlaneZ;
                    int k = (int)Math.Round(kFloat);
                    if (k < 0 || k >= zCellLength)
                        continue;

                    pt.z = k;
                    p1TestPt.z = pt.z - vert1Proj.z;

                    byte location;
                    VoxelOrientationPlane plane;
                    double floatLoc;
                    if (u >= 0 && v >= 0 && u + v <= 1)
                    {
                        floatLoc = (k - kFloat) * signW + 0.5;
                        floatLoc *= maxLocation;

                        if (floatLoc > maxLocation)
                            floatLoc = maxLocation;
                        if (floatLoc < 0)
                            floatLoc = 0;

                        location = (byte)Math.Ceiling(floatLoc);
                        plane = signW <= 0 ? VoxelOrientationPlane.Z_UP : VoxelOrientationPlane.Z_DOWN;

                        SetVoxelPoint(i, j, k, part, plane, location);
                        continue;
                    }

                    Vector3 p2TestPt = pt - vert2Proj;
                    Vector3 p3TestPt = pt - vert3Proj;
                    if (u + v < 0.5 && p1TestPt.magnitude <= RC ||
                        (u < 0.5 || u + v > 0.5) && p2TestPt.magnitude <= RC ||
                        (v < 0.5 || u + v > 0.5) && p3TestPt.magnitude <= RC)
                    {
                        floatLoc = (k - kFloat) * signW + 0.5;
                        floatLoc *= maxLocation * 0.25d;

                        if (floatLoc > maxLocation)
                            floatLoc = maxLocation;
                        if (floatLoc < 0)
                            floatLoc = 0;

                        location = (byte)Math.Ceiling(floatLoc);
                        plane = signW <= 0 ? VoxelOrientationPlane.Z_UP : VoxelOrientationPlane.Z_DOWN;

                        SetVoxelPoint(i, j, k, part, plane, location);
                        continue;
                    }


                    bool validDistFromSide = false;
                    double distFromSide = DistanceFromSide(p1p2, p1TestPt);
                    if (distFromSide <= RC)
                    {
                        validDistFromSide = true;
                    }
                    else
                    {
                        distFromSide = DistanceFromSide(p1p3, p1TestPt);
                        if (distFromSide <= RC)
                        {
                            validDistFromSide = true;
                        }
                        else
                        {
                            distFromSide = DistanceFromSide(vert3Proj - vert2Proj, p2TestPt);
                            if (distFromSide <= RC)
                                validDistFromSide = true;
                        }
                    }

                    if (!validDistFromSide)
                        continue;
                    floatLoc = (k - kFloat) * signW + 0.5;
                    floatLoc *= maxLocation * (RC - distFromSide);

                    if (floatLoc > maxLocation)
                        floatLoc = maxLocation;
                    if (floatLoc < 0)
                        floatLoc = 0;

                    location = (byte)Math.Ceiling(floatLoc);
                    plane = signW <= 0 ? VoxelOrientationPlane.Z_UP : VoxelOrientationPlane.Z_DOWN;

                    SetVoxelPoint(i, j, k, part, plane, location);
                }
        }

        private static double DistanceFromSide(Vector3 sideVector, Vector3 testVec)
        {
            float sideDot = Vector3.Dot(sideVector, testVec);
            if (sideDot < 0)
                return 1;

            float sideSqMag = sideVector.sqrMagnitude;

            if (sideDot > sideSqMag)
                return 1;

            Vector3 perpVector = sideDot / sideSqMag * sideVector;
            perpVector = testVec - perpVector;

            return perpVector.magnitude;
        }

        private Vector4d CalculateEquationOfSweepPlane(Vector3 normalVector, out double wInc)
        {
            var result = new Vector4(normalVector.x, normalVector.y, normalVector.z);

            if (result.x > 0)
                result.w -= result.x * xCellLength;
            if (result.y > 0)
                result.w -= result.y * yCellLength;
            if (result.z > 0)
                result.w -= result.z * zCellLength;

            float x = Math.Abs(result.x);
            float y = Math.Abs(result.y);
            float z = Math.Abs(result.z);

            if (y >= x && y >= z)
                wInc = y;
            else if (x > y && x > z)
                wInc = x;
            else
                wInc = z;

            return result;
        }

        private Vector4d CalculateEquationOfPlaneInIndices(Vector3d pt1, Vector3d pt2, Vector3d pt3)
        {
            Vector3d p1p2 = pt2 - pt1;
            Vector3d p1p3 = pt3 - pt1;

            var tmp = Vector3d.Cross(p1p2, p1p3);

            var result = new Vector4d(tmp.x, tmp.y, tmp.z);

            result.w = result.x * (LocalLowerRightCorner.x - pt1.x) +
                       result.y * (LocalLowerRightCorner.y - pt1.y) +
                       result.z * (LocalLowerRightCorner.z - pt1.z);
            result.w *= invElementSize;

            return result;
        }


        // ReSharper disable once UnusedMember.Local
        private Vector4d CalculateEquationOfPlane(Vector3d pt1, Vector3d pt2, Vector3d pt3)
        {
            Vector3d p1p2 = pt2 - pt1;
            Vector3d p1p3 = pt3 - pt1;

            var tmp = Vector3d.Cross(p1p2, p1p3);

            var result = new Vector4d(tmp.x, tmp.y, tmp.z);

            result.w = -(pt1.x * result.x + pt1.y * result.y + pt1.z * result.z);

            return result;
        }

        // ReSharper disable once UnusedMember.Local
        private Vector4d TransformPlaneToIndices(Vector4d plane)
        {
            var newPlane = new Vector4d
            {
                x = plane.x * ElementSize,
                y = plane.y * ElementSize,
                z = plane.z * ElementSize,
                w = plane.w +
                    plane.x * LocalLowerRightCorner.x +
                    plane.y * LocalLowerRightCorner.y +
                    plane.z * LocalLowerRightCorner.z
            };

            return newPlane;
        }

        private void SolidifyVoxel(object uncastData)
        {
            var parameters = (VoxelSolidParams)uncastData;
            try
            {
                SolidifyVoxel(parameters.lowJ, parameters.highJ, parameters.increasingJ);
            }
            catch (Exception e)
            {
                ThreadSafeDebugLogger.Exception(e);
            }
            finally
            {
                lock (_locker)
                {
                    threadsQueued--;
                    Monitor.Pulse(_locker);
                }
            }
        }

        private void SolidifyVoxel(int lowJ, int highJ, bool increasingJ)
        {
            SweepPlanePoint[,] plane;
            lock (clearedPlanes)
            {
                while (clearedPlanes.Count == 0)
                    Monitor.Wait(clearedPlanes);

                plane = clearedPlanes.Pop();
            }

            try
            {
                int xLen = plane.GetLength(0);
                int zLen = plane.GetLength(1);
                if (xLen < xCellLength || zLen < zCellLength)
                    plane = new SweepPlanePoint[Math.Max(xCellLength, xLen), Math.Max(zCellLength, zLen)];

                var activePts = new List<SweepPlanePoint>();
                var inactiveInteriorPts = new HashSet<SweepPlanePoint>();
                var neighboringSweepPlanePts = new SweepPlanePoint[4];

                if (increasingJ)
                    for (int j = lowJ; j < highJ; j++) //Iterate from back of vehicle to front
                        SolidifyLoop(j, j - 1, plane, activePts, inactiveInteriorPts, neighboringSweepPlanePts);
                else
                    for (int j = highJ - 1; j >= lowJ; j--) //Iterate from front of vehicle to back
                        SolidifyLoop(j, j + 1, plane, activePts, inactiveInteriorPts, neighboringSweepPlanePts);
            }
            catch (Exception e)
            {
                ThreadSafeDebugLogger.Exception(e);
            }
            finally
            {
                CleanSweepPlane(plane);
                lock (clearedPlanes)
                {
                    clearedPlanes.Push(plane);
                    Monitor.Pulse(clearedPlanes);
                }
            }
        }

        private static void CleanSweepPlane(SweepPlanePoint[,] sweepPlane)
        {
            int lengthX = sweepPlane.GetLength(0);
            int lengthZ = sweepPlane.GetLength(1);

            for (int i = 0; i < lengthX; i++)
                for (int k = 0; k < lengthZ; k++)
                {
                    SweepPlanePoint pt = sweepPlane[i, k];
                    pt?.Clear();
                }
        }

        private void SolidifyLoop(
            int j,
            int lastJ,
            SweepPlanePoint[,] sweepPlane,
            List<SweepPlanePoint> activePts,
            HashSet<SweepPlanePoint> inactiveInteriorPts,
            SweepPlanePoint[] neighboringSweepPlanePts
        )
        {
            //Iterate across the cross-section plane to add voxel shell and mark active interior points
            for (int i = 0; i < xCellLength; i++)
                for (int k = 0; k < zCellLength; k++)
                {
                    SweepPlanePoint pt = sweepPlane[i, k];
                    Part p = GetPartAtVoxelPos(i, j, k);

                    //If there is a section of voxel there, but no pt, add a new voxel shell pt to the sweep plane
                    if (pt == null)
                    {
                        if (p is null)
                            continue;
                        pt = new SweepPlanePoint(p, i, k) { jLastInactive = j };
                        sweepPlane[i, k] = pt;
                    }
                    else
                    {
                        //If there is a pt there, but no part listed, this is an interior pt or the cross-section is shrinking
                        if (p is null)
                        {
                            switch (pt.mark)
                            {
                                //label it as active so that it can be determined if it is interior or not once all the points have been updated
                                case SweepPlanePoint.MarkingType.VoxelShell:
                                    activePts.Add(pt); //And add it to the list of active interior pts
                                    pt.mark = SweepPlanePoint.MarkingType.Active;
                                    break;
                                //if this shell was previously interior, we need to know so that we can set it to the correct active
                                case SweepPlanePoint.MarkingType.VoxelShellPreviouslyInterior:
                                    activePts.Add(pt); //And add it to the list of active interior pts
                                    pt.mark = SweepPlanePoint.MarkingType.ActivePassedThroughInternalShell;
                                    break;
                            }

                            //Only other situation is that it is an inactive point, in which case we do nothing here, because it is already taken care of
                        }
                        //only run this if it's not already labeled as part of a voxel shell
                        else if (pt.mark != SweepPlanePoint.MarkingType.VoxelShell &&
                                 pt.mark != SweepPlanePoint.MarkingType.VoxelShellPreviouslyInterior)
                        {
                            //Make sure the point is labeled as a voxel shell if there is already a part there
                            inactiveInteriorPts.Remove(pt);

                            pt.mark = pt.mark == SweepPlanePoint.MarkingType.Clear
                                          ? SweepPlanePoint.MarkingType.VoxelShell
                                          : SweepPlanePoint.MarkingType.VoxelShellPreviouslyInterior;


                            pt.part = p;
                            pt.jLastInactive = j;
                        }
                    }
                }

            //Then, iterate through all active points for this section
            for (int i = 0; i < activePts.Count; i++)
            {
                SweepPlanePoint activeInteriorPt = activePts[i]; //Get active interior pt
                if (activeInteriorPt.i + 1 < xCellLength)        //And all of its 4-neighbors
                    neighboringSweepPlanePts[0] = sweepPlane[activeInteriorPt.i + 1, activeInteriorPt.k];
                else
                    neighboringSweepPlanePts[0] = null;
                if (activeInteriorPt.i - 1 > 0)
                    neighboringSweepPlanePts[1] = sweepPlane[activeInteriorPt.i - 1, activeInteriorPt.k];
                else
                    neighboringSweepPlanePts[1] = null;
                if (activeInteriorPt.k + 1 < zCellLength)
                    neighboringSweepPlanePts[2] = sweepPlane[activeInteriorPt.i, activeInteriorPt.k + 1];
                else
                    neighboringSweepPlanePts[2] = null;
                if (activeInteriorPt.k - 1 > 0)
                    neighboringSweepPlanePts[3] = sweepPlane[activeInteriorPt.i, activeInteriorPt.k - 1];
                else
                    neighboringSweepPlanePts[3] = null;

                //Check if the active point is surrounded by all 4 neighbors
                bool remove =
                    neighboringSweepPlanePts.Any(neighbor => neighbor == null ||
                                                             neighbor.mark == SweepPlanePoint.MarkingType.Clear);
                if (remove) //If it is set to be removed...
                {
                    //Go through all the neighboring points
                    foreach (SweepPlanePoint neighbor in neighboringSweepPlanePts)
                    {
                        //For the ones that exist, and are inactive interior...
                        if (neighbor == null || neighbor.mark != SweepPlanePoint.MarkingType.InactiveInterior)
                            continue;
                        inactiveInteriorPts.Remove(neighbor);               //remove them from inactiveInterior
                        neighbor.mark = SweepPlanePoint.MarkingType.Active; //...mark them active
                        activePts.Add(neighbor);                            //And add them to the end of activePts
                    }

                    SweepPlanePoint pt = sweepPlane[activeInteriorPt.i, activeInteriorPt.k];
                    //Then, set this point to be marked clear in the sweepPlane
                    pt.mark = SweepPlanePoint.MarkingType.Clear;
                    pt.ductingParts = false;
                    pt.part = null;
                }
                else
                {
                    //If it's surrounded by other points, it's inactive; add it to that list
                    if (activeInteriorPt.mark == SweepPlanePoint.MarkingType.ActivePassedThroughInternalShell)
                    {
                        if (activeInteriorPt.ductingParts)
                        {
                            if (activeInteriorPt.jLastInactive < j)
                                for (int mJ = activeInteriorPt.jLastInactive; mJ < j; mJ++)
                                    //used to make sure that internal part boundaries for cargo bays don't result in dips in cross-section
                                    SetVoxelPointNoLock(activeInteriorPt.i,
                                                        mJ,
                                                        activeInteriorPt.k,
                                                        activeInteriorPt.part);
                            else
                                for (int mJ = lastJ; mJ <= activeInteriorPt.jLastInactive; mJ++)
                                    //used to make sure that internal part boundaries for cargo bays don't result in dips in cross-section
                                    SetVoxelPointNoLock(activeInteriorPt.i,
                                                        mJ,
                                                        activeInteriorPt.k,
                                                        activeInteriorPt.part);
                        }
                        else
                        {
                            if (activeInteriorPt.jLastInactive < j)
                                for (int mJ = activeInteriorPt.jLastInactive; mJ < j; mJ++)
                                    //used to make sure that internal part boundaries for cargo bays don't result in dips in cross-section
                                    SetVoxelPointNoLock(activeInteriorPt.i, mJ, activeInteriorPt.k);
                            else
                                for (int mJ = lastJ; mJ <= activeInteriorPt.jLastInactive; mJ++)
                                    //used to make sure that internal part boundaries for cargo bays don't result in dips in cross-section
                                    SetVoxelPointNoLock(activeInteriorPt.i, mJ, activeInteriorPt.k);
                        }
                    }

                    activeInteriorPt.mark = SweepPlanePoint.MarkingType.InactiveInterior;
                    activeInteriorPt.ductingParts = false;
                    inactiveInteriorPts.Add(activeInteriorPt);
                }
            }

            activePts.Clear(); //Clear activePts every iteration

            //Any remaining inactive interior pts are guaranteed to be on the inside of the vehicle
            foreach (SweepPlanePoint inactivePt in inactiveInteriorPts)
                //Get each and update the voxel accordingly
                SetVoxelPointNoLock(inactivePt.i, j, inactivePt.k, inactivePt.part);
        }

        private class SweepPlanePoint
        {
            public enum MarkingType
            {
                VoxelShell,
                VoxelShellPreviouslyInterior,
                Active,
                ActivePassedThroughInternalShell,
                InactiveInterior,
                Clear
            }

            public readonly int i;
            public readonly int k;
            public Part part;
            public int jLastInactive;
            public bool ductingParts;

            public MarkingType mark = MarkingType.VoxelShell;

            public SweepPlanePoint(Part part, int i, int k)
            {
                this.i = i;
                this.k = k;
                this.part = part;
            }

            public void Clear()
            {
                jLastInactive = 0;
                mark = MarkingType.Clear;
                ductingParts = false;
                part = null;
            }
        }

        private struct VoxelSolidParams
        {
            public readonly int lowJ;
            public readonly int highJ;
            public readonly bool increasingJ;

            public VoxelSolidParams(int lowJ, int highJ, bool increasingJ)
            {
                this.lowJ = lowJ;
                this.highJ = highJ;
                this.increasingJ = increasingJ;
            }
        }

        private struct VoxelShellMeshParams
        {
            public readonly List<GeometryPartModule> modules;
            public readonly int lowerIndex;
            public readonly int upperIndex;

            public VoxelShellMeshParams(int lowerIndex, int upperIndex, List<GeometryPartModule> modules)
            {
                this.lowerIndex = lowerIndex;
                this.upperIndex = upperIndex;
                this.modules = modules;
            }
        }
    }
}
