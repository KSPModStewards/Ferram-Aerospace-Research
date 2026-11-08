using System;
using FerramAerospaceResearch.Geometry;
using UnityEngine;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>The debug voxel mesh's index count per primitive, for each mesh topology it can be built with.</summary>
    public class VoxelMeshTests
    {
        [SkippableTheory]
        [InlineData(MeshTopology.Triangles, 3)]
        [InlineData(MeshTopology.Quads, 4)]
        [InlineData(MeshTopology.Lines, 2)]
        [InlineData(MeshTopology.Points, 1)]
        public void IndicesPerPrimitive(MeshTopology topology, int indices)
        {
            UnityRuntime.Require();
            Assert.Equal(indices, DebugVoxelMesh.IndicesPerPrimitive(topology));
        }

        [SkippableFact]
        public void AStripTopologyIsRefused()
        {
            UnityRuntime.Require();
            Assert.Throws<NotImplementedException>(() => DebugVoxelMesh.IndicesPerPrimitive(MeshTopology.LineStrip));
            Assert.Throws<ArgumentOutOfRangeException>(() => DebugVoxelMesh.IndicesPerPrimitive((MeshTopology)99));
        }
    }
}
