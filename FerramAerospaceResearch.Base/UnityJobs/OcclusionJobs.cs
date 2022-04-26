using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace FerramAerospaceResearch.UnityJobs
{
    [BurstCompile]
    public struct SphereDistanceInfo : IComparable<SphereDistanceInfo>, IEquatable<SphereDistanceInfo>
    {
        public quaternion q;
        public float distance;

        public int CompareTo(SphereDistanceInfo obj)
        {
            return distance.CompareTo(obj.distance);
        }

        public bool Equals(SphereDistanceInfo other)
        {
            return q.Equals(other.q) && distance.Equals(other.distance);
        }
    }

    public struct EMPTY_STRUCT { }

    //http://extremelearning.com.au/how-to-evenly-distribute-points-on-a-sphere-more-effectively-than-the-canonical-fibonacci-lattice/
    [BurstCompile]
    public struct SpherePointsJob : IJobParallelFor
    {
        [ReadOnly] public int points;
        [ReadOnly] public float epsilon;
        [WriteOnly] public NativeArray<quaternion> results;
        private const float GoldenRatio = 1.61803398875f;   //(1 + math.sqrt(5)) / 2;
        private const float ThetaConstantTerm = 2 * math.PI / GoldenRatio;
        private readonly float denomRecip;

        public SpherePointsJob(int points, float epsilon) : this()
        {
            this.points = points;
            this.epsilon = epsilon;
            denomRecip = 1f / (points - 1 + (2 * epsilon));
        }

        public void Execute(int index)
        {
            if (Unity.Burst.CompilerServices.Hint.Unlikely(index == 0))
            {
                results[index] = quaternion.identity;
            }
            else if (Unity.Burst.CompilerServices.Hint.Unlikely(index == points - 1))
            {
                results[index] = quaternion.Euler(180, 0, 0);
            }
            else
            {
                float theta = index * ThetaConstantTerm;
                float num = 2 * (index + epsilon);
                float cosPhi = 1 - num * denomRecip;
                float sinPhi = math.sqrt(1 - cosPhi * cosPhi);
                math.sincos(theta, out float sinTheta, out float cosTheta);
                var res = new float3(cosTheta * sinPhi,
                                        sinTheta * sinPhi,
                                        cosPhi);
                results[index] = Quaternion.FromToRotation(new float3(0,0,1), res);
            }
        }
    }

    [BurstCompile]
    public struct SetBoundsJob : IJobParallelFor
    {
        [ReadOnly] public float3 boundsCenter;
        [ReadOnly] public float3 extents;
        [ReadOnly] public NativeArray<quaternion> quaternions;
        [WriteOnly] public NativeArray<float2> bounds;

        // We now have center, extents, and the raycaster plane (normal = rotation * vessel.forward)
        // Project each of the 8 corners of the bounding box onto the plane.
        // https://stackoverflow.com/questions/9605556/how-to-project-a-point-onto-a-plane-in-3d/41897378#41897378
        public void Execute(int index)
        {
            float3 normal = math.mul(quaternions[index], new float3(0, 0, 1));
            float3 origin = float3.zero + math.length(extents) * normal;
            float2 min = new(float.PositiveInfinity, float.PositiveInfinity);
            float2 max = new(float.NegativeInfinity, float.NegativeInfinity);
            float3 ext = extents / 2;
            float3 center = boundsCenter;
            unsafe
            {
                float3* arr = stackalloc[]
                {
                    center - ext,
                    new float3(center.x - ext.x, center.y - ext.y, center.z + ext.z),
                    new float3(center.x - ext.x, center.y + ext.y, center.z - ext.z),
                    new float3(center.x - ext.x, center.y + ext.y, center.z + ext.z),
                    new float3(center.x + ext.x, center.y - ext.y, center.z - ext.z),
                    new float3(center.x + ext.x, center.y - ext.y, center.z + ext.z),
                    new float3(center.x + ext.x, center.y + ext.y, center.z - ext.z),
                    center + ext,
                };
                var quaternionInverse = math.inverse(quaternions[index]);
                for (int i = 0; i < 8; i++)
                {
                    // Project the bounding box onto the plane
                    float dist = math.dot(arr[i] - origin, normal);
                    float3 projection = arr[i] - dist * normal;
                    // Rotate the projection back towards the vessel alignment
                    projection = math.mul(quaternionInverse, projection);

                    // The x,y coords of projection are again aligned to the vessel axes.
                    min = math.min(min, projection.xy);
                    max = math.max(max, projection.xy);
                }
            }

            bounds[index] = math.abs(max - min);
        }
    }

    [BurstCompile]
    public struct SetIntervalAndDimensionsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> bounds;
        [ReadOnly] public int maxDim;
        [ReadOnly] public float resolution;
        [WriteOnly] public NativeArray<float2> interval;
        [WriteOnly] public NativeArray<int2> dims;

        public void Execute(int index)
        {
            var minSize = new int2(math.ceil(bounds[index] / resolution));
            dims[index] = math.min(minSize, maxDim);
            interval[index] = math.max(bounds[index] / maxDim, resolution);
        }
    }

    [BurstCompile]
    public struct SetVectorsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<quaternion> quaternions;
        [ReadOnly] public float4x4 localToWorldMatrix;
        [WriteOnly] public NativeArray<float3> forward;
        [WriteOnly] public NativeArray<float3> up;
        [WriteOnly] public NativeArray<float3> right;

        public void Execute(int index)
        {
            float3 dir = math.mul(quaternions[index], new float3(0, 0, 1));
            float4 fwd = math.mul(localToWorldMatrix, new float4(dir.x, dir.y, dir.z, 0));
            forward[index] = fwd.xyz;

            dir = math.mul(quaternions[index], new float3(0, 1, 0));
            var upLoc = math.mul(localToWorldMatrix, new float4(dir.x, dir.y, dir.z, 0));
            up[index] = upLoc.xyz;

            right[index] = math.cross(fwd.xyz, upLoc.xyz);
        }
    }

    [BurstCompile]
    public struct AngleBetween : IJobParallelFor
    {
        [ReadOnly] public float3 dir;
        [ReadOnly] public NativeArray<quaternion> rotations;
        [WriteOnly] public NativeArray<float> angles;

        public void Execute(int index)
        {
            float3 rot_dir = math.mul(rotations[index], new float3(0, 0, 1));
            angles[index] = math.degrees(math.acos(math.clamp(math.dot(dir, rot_dir), -1, 1)));
        }
    }

    [BurstCompile]
    public struct GeneralDistanceInfo : IJob
    {
        [ReadOnly] public NativeArray<float> distances;
        [WriteOnly] public NativeArray<float3> output;

        public void Execute()
        {
            float min = float.PositiveInfinity;
            float max = 0;
            float total = 0;
            for (int i = 0; i < distances.Length; i++)
            {
                total += distances[i];
                max = math.max(max, distances[i]);
                min = math.min(min, distances[i]);
            }
            output[0] = new float3(min, max, total / distances.Length);
        }
    }

    [BurstCompile]
    public struct DotProductJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> A;
        [ReadOnly] public float3 B;
        [WriteOnly] public NativeArray<float> result;

        public void Execute(int index)
        {
            result[index] = math.dot(A[index], B);
        }
    }

    [BurstCompile]
    public struct SortedFilterAndMapByDistanceJob : IJob
    {
        [ReadOnly] public NativeArray<quaternion> quaternions;
        [ReadOnly] public NativeArray<float> distances;
        [ReadOnly] public NativeArray<float> cutoffDistance;
        public NativeList<SphereDistanceInfo> sdiList;
        [WriteOnly] public NativeList<quaternion> orderedQuaternions;

        public void Execute()
        {
            for (int i = 0; i < quaternions.Length; i++)
            {
                if (distances[i] < cutoffDistance[0])
                {
                    SphereDistanceInfo sdi = new()
                    {
                        q = quaternions[i],
                        distance = distances[i]
                    };
                    sdiList.Add(sdi);
                }
            }
            NativeSortExtension.Sort(sdiList);
            for (int i = 0; i < sdiList.Length; i++)
                orderedQuaternions.Add(sdiList[i].q);
        }
    }

    [BurstCompile]
    public struct MakeQuaternionIndexMapJob : IJob
    {
        [ReadOnly] public NativeArray<quaternion> arr;
        [WriteOnly] public NativeHashMap<quaternion, int> map;

        public void Execute()
        {
            for (int index=0; index<arr.Length; index++)
                map.TryAdd(arr[index], index);
        }
    }

    // Given an array of items and a map, update the map s.t. map[item] = min(current value, index in array)
    // Can't parallelize the read/write cycle of the NativeHashMap, so can't do .ParallelWriter
    [BurstCompile]
    public struct ListToIndexedMap : IJob
    {
        [ReadOnly] public NativeArray<quaternion> sorted;
        public NativeHashMap<quaternion, int> map;
        public void Execute()
        {
            for (int index=0; index < sorted.Length; index++)
            {
                quaternion q = sorted[index];
                if (map.TryGetValue(q, out int i) && index < i)
                {
                    map[q] = index;
                }
                else
                    map.TryAdd(q, index);
            }
        }
    }

    [BurstCompile]
    public struct BucketSortByPriority : IJobParallelFor
    {
        [ReadOnly] public NativeArray<quaternion> quaternions;
        [ReadOnly] public NativeHashMap<quaternion, int> priorityMap;
        [ReadOnly] public NativeHashMap<quaternion, EMPTY_STRUCT> completed;
        [ReadOnly] public int maxIndexForPriority0;
        [WriteOnly] public NativeMultiHashMap<int, int>.ParallelWriter map;

        public void Execute(int index)
        {
            if (!completed.ContainsKey(quaternions[index]))
            {
                int pri = 2;
                if (priorityMap.TryGetValue(quaternions[index], out int i))
                    pri = math.min(pri, (i <= maxIndexForPriority0) ? 0 : 1);
                map.Add(pri, index);
            }
        }
    }

    [BurstCompile]
    public struct ClearNativeHashMap<T1,T2> : IJob where T1:struct, IEquatable<T1> where T2:struct
    {
        public NativeHashMap<T1, T2> map;
        public void Execute() => map.Clear();
    }


    [BurstCompile]
    public struct ClearNativeMultiHashMap<T1, T2> : IJob where T1 : struct, IEquatable<T1> where T2 : struct
    {
        public NativeMultiHashMap<T1, T2> map;
        public void Execute() => map.Clear();
    }

    // hitsMap is the mapping from collider to all of the indicies in the hitsIn array.
    [BurstCompile]
    public struct RaycastPrepareJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<RaycastHit> hitsIn;
        [WriteOnly] public NativeMultiHashMap<int, int>.ParallelWriter hitsMap;

        public void Execute(int index)
        {
            unsafe
            {
                RaycastHit* array_ptr = (RaycastHit*)hitsIn.GetUnsafeReadOnlyPtr();
                int collider_id = *(int*)((byte*)&array_ptr[index] + 40);
                if (collider_id != 0)
                    hitsMap.Add(collider_id, index);
            }
        }
    }

    // hits is the array of all Raycasts
    // hitMap is the mapping from collider to all of the indicies in the hits array.
    [BurstCompile]
    public struct RaycastProcessorJob : IJobNativeMultiHashMapMergedSharedKeyIndices
    {
        [ReadOnly] public NativeArray<RaycastHit> hits;
        [ReadOnly] public NativeMultiHashMap<int, int> hitMap;
        [ReadOnly] public float area;
        [WriteOnly] public NativeHashMap<int, float>.ParallelWriter hitsOut;
        public NativeArray<float> areaSum;

        // index is the *value* in the MultiHashMap hitMap
        // The key generating this is a RaycastHitSummary = Tuple<quaternion,collider_id>
        public void ExecuteFirst(int index)
        {
            unsafe
            {
                areaSum[index] = area;
                hitsOut.TryAdd(index, area);
            }
        }

        public void ExecuteNext(int firstIndex, int index)
        {
            float t = areaSum[firstIndex];
            areaSum[firstIndex] = t + area;
        }
    }

    [BurstCompile]
    public struct OcclusionRaycastBuilder : IJobParallelFor
    {
        [ReadOnly] public float3 forwardDir;
        [ReadOnly] public float3 rightDir;
        [ReadOnly] public float3 upDir;
        [ReadOnly] public float3 startPosition;
        [ReadOnly] public float offset;
        [ReadOnly] public int2 dimensions;
        [ReadOnly] public float2 interval;
        [WriteOnly] public NativeArray<RaycastCommand> commands;

        public void Execute(int index)
        {
            // Everything is already rotated by the quaternion rotation and transformed to worldspace
            // Iterate raycasts in X, Y
            float3 origin = startPosition + offset * forwardDir;
            int row = index / dimensions.x;
            int col = index % dimensions.x;
            float row_eff = row - (dimensions.y / 2);
            float col_eff = col - (dimensions.x / 2);

            // All raycasts are in the same direction.
            float3 pos = origin + (row_eff * interval.y * upDir) + (col_eff * interval.x * rightDir);
            commands[index] = new RaycastCommand(pos, -forwardDir, distance: 1e5f, layerMask: 19, maxHits: 1);
        }
    }
}
