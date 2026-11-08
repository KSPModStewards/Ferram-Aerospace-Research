using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;

namespace FerramAerospaceResearch.Geometry
{
    [RequireComponent(typeof(MeshRenderer)), RequireComponent(typeof(MeshFilter))]
    public class DebugVoxelMesh : MonoBehaviour
    {
        // limit of vertices in each mesh imposed by Unity if using 16 bit indices
        public const int MaxVerticesPerSubmesh = 65535;

        public static bool Use32BitIndices = false;

        private int currentOffset;
        private int indicesPerSubmesh;
        private int verticesPerSubmesh;
        private int nextVertexCount;
        private int verticesPerVoxel;
        public Mesh Mesh { get; private set; }
        public MeshRenderer Renderer { get; private set; }
        public MeshFilter Filter { get; private set; }
        public MeshBuildData Data { get; } = new MeshBuildData();

        private bool uInt32BitIndices;

        private bool UInt32BitIndices
        {
            // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
            // get { return uInt32BitIndices; }
            set
            {
                uInt32BitIndices = value;
                Mesh.indexFormat = value ? IndexFormat.UInt32 : IndexFormat.UInt16;
            }
        }

        public static DebugVoxelMesh Create(Transform parent = null)
        {
            var go = new GameObject("DebugVoxelMesh", typeof(DebugVoxelMesh));
            DebugVoxelMesh mesh = go.GetComponent<DebugVoxelMesh>();

            if (parent != null)
                go.transform.SetParent(parent);

            go.SetActive(true);
            return mesh;
        }

        private void Awake()
        {
            FARLogger.Debug("Setting up debug voxel mesh");
            Mesh = new Mesh();
            UInt32BitIndices = Use32BitIndices;
            Filter = GetComponent<MeshFilter>();
            Renderer = GetComponent<MeshRenderer>();

            Filter.mesh = Mesh;

            // voxel mesh should not be affected by lights
            Renderer.lightProbeUsage = LightProbeUsage.Off;
            Renderer.reflectionProbeUsage = ReflectionProbeUsage.Off;
            Renderer.shadowCastingMode = ShadowCastingMode.Off;
            Renderer.receiveShadows = false;
            Renderer.motionVectorGenerationMode = MotionVectorGenerationMode.ForceNoMotion;
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public void Rebuild<T, Builder, Voxels>(Builder builder, Voxels voxels, int count)
        //     where Builder : IDebugVoxelMeshBuilder<T> where Voxels : IEnumerator<T>
        // {
        //     Clear(builder, count);
        //     while (voxels.MoveNext())
        //         Add(builder, voxels.Current);
        //     Apply(builder);
        // }

        private static void SetupNestedList<T>(List<List<T>> list, int meshes, int countPerSubmesh)
        {
            if (list.Capacity < meshes)
                list.Capacity = meshes;

            for (int i = list.Count; i < meshes; i++)
                list.Add(new List<T>());

            for (int i = 0; i < meshes; i++)
            {
                if (list[i].Capacity < countPerSubmesh)
                    list[i].Capacity = countPerSubmesh;
            }
        }

        public void Clear(bool clearMesh = true)
        {
            if (clearMesh)
                Mesh.Clear();
            UInt32BitIndices = Use32BitIndices;
            Data.Uvs.Clear();
            Data.Vertices.Clear();
            Data.Colors.Clear();
            foreach (List<int> sublist in Data.SubmeshIndices)
                sublist.Clear();

            Data.CurrentSubmesh = 0;
            nextVertexCount = 0;
            currentOffset = 0;
        }

        public void Clear<Builder>(Builder builder, int count, bool clearMesh = true)
            where Builder : IDebugVoxelMeshBuilder
        {
            Clear(clearMesh);
            Reserve(builder, count);
        }

        public void Reserve<Builder>(Builder builder, int count) where Builder : IDebugVoxelMeshBuilder
        {
            int meshes;
            int voxelsPerSubmesh;
            if (uInt32BitIndices)
            {
                voxelsPerSubmesh = count;
                meshes = 1;
            }
            else
            {
                voxelsPerSubmesh = MaxVerticesPerSubmesh / builder.VerticesPerVoxel;
                meshes = count / voxelsPerSubmesh + 1;
            }

            indicesPerSubmesh = voxelsPerSubmesh * builder.PrimitivesPerVoxel * IndicesPerPrimitive(builder.Topology);
            verticesPerSubmesh = voxelsPerSubmesh * builder.VerticesPerVoxel;
            nextVertexCount = verticesPerSubmesh;
            verticesPerVoxel = builder.VerticesPerVoxel;

            int nVertices = count * builder.VerticesPerVoxel;
            if (Data.Vertices.Capacity < nVertices)
                Data.Vertices.Capacity = nVertices;

            if (builder.HasColor && Data.Colors.Capacity < nVertices)
                Data.Colors.Capacity = nVertices;

            SetupNestedList(Data.SubmeshIndices, meshes, indicesPerSubmesh);
            SetupNestedList(Data.Uvs, builder.UVChannels, nVertices);

            FARLogger.InfoFormat("Reserved {0} voxels in {1} submeshes", count.ToString(), meshes.ToString());
        }

        public void Add<T, Builder>(Builder builder, T voxel) where Builder : IDebugVoxelMeshBuilder<T>
        {
            builder.Build(voxel, Data, currentOffset);

            // check if submesh is filled, 32 bit indices can store 4B vertices so there should be no need to check
            if (uInt32BitIndices || Data.Vertices.Count < nextVertexCount)
                return;

            currentOffset = Data.Vertices.Count;
            Data.CurrentSubmesh++;
            nextVertexCount = Data.Vertices.Count + verticesPerSubmesh;

            // make sure the next list for indices is valid and reserve memory to for performance
            if (Data.Indices.Count > Data.CurrentSubmesh)
                return;
            Data.SubmeshIndices.Add(new List<int>());
            Data.Indices.Capacity = indicesPerSubmesh;
        }

        public void Apply<Builder>(Builder builder) where Builder : IDebugVoxelMeshBuilder
        {
            Mesh.Clear();
            FARLogger.InfoFormat("Built voxel mesh with {0} voxels in {1} submeshes",
                                 (Data.Vertices.Count / verticesPerVoxel).ToString(),
                                 Data.SubmeshIndices.Count.ToString());

            Mesh.SetVertices(Data.Vertices);
            Mesh.SetColors(Data.Colors);
            for (int i = 0; i < Data.Uvs.Count; i++)
            {
                if (Data.Uvs[i].Count == 0)
                    continue;
                Mesh.SetUVs(i, Data.Uvs[i]);
            }

            //TODO: replace with Mesh.SetIndices(List<int>, ...) when using Unity 2019.3+

            if (uInt32BitIndices)
            {
                // only 1 submesh
                Renderer.material = builder.MeshMaterial;
                Mesh.SetIndices(Data.SubmeshIndices[0].ToArray(), builder.Topology, 0);
            }
            else
            {
                // ignore empty index lists
                int count = Data.SubmeshIndices.Sum(list => list.Count == 0 ? 0 : 1);
                Mesh.subMeshCount = count;
                var materials = new Material[count];

                int offset = 0;
                int j = 0;
                foreach (List<int> t in Data.SubmeshIndices)
                {
                    if (t.Count == 0)
                        continue;
                    Mesh.SetIndices(t.ToArray(), builder.Topology, j, false, offset);
                    offset += verticesPerSubmesh;
                    materials[j] = builder.MeshMaterial;
                    j++;
                }

                Renderer.materials = materials;
            }
        }

        private void OnDestroy()
        {
            Mesh.Clear();
            Mesh = null;
            Filter = null;
            Renderer = null;
        }

        public static int IndicesPerPrimitive(MeshTopology topology)
        {
            return topology switch
            {
                MeshTopology.Triangles => 3,
                MeshTopology.Quads => 4,
                MeshTopology.Lines => 2,
                MeshTopology.Points => 1,
                MeshTopology.LineStrip => throw new NotImplementedException("LineStrip is not supported"),
                _ => throw new ArgumentOutOfRangeException(nameof(topology), topology, null)
            };
        }
    }
}
