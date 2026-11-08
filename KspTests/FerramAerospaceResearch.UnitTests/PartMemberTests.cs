using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.Serialization;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.FARPartGeometry.GeometryModification;
using FerramAerospaceResearch.Geometry;
using FerramAerospaceResearch.Geometry.Exposure;
using FerramAerospaceResearch.RealChuteLite;
using FerramAerospaceResearch.Resources;
using UnityEngine;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// Members of FAR's part, geometry and exposure types that FAR does not call yet, on objects made without the game:
    /// parts and part modules are Unity components, made here uninitialized (no native object), which is enough for
    /// code that only keeps or compares them.
    /// </summary>
    public class PartMemberTests
    {
        private static T Uninitialized<T>() => (T)FormatterServices.GetUninitializedObject(typeof(T));

        /// <summary>
        /// ProjectedArea's index setter writes the field at that index, in declaration order (iP iN jP jN kP kN, then
        /// the total), as the getter reads it.
        /// </summary>
        [SkippableFact]
        public void ProjectedAreaIndexesItsFieldsInOrder()
        {
            UnityRuntime.Require();   // the struct's static face tables (Vector3d)
            var area = new FARAeroPartModule.ProjectedArea();
            for (int i = 0; i < 7; i++)
                area[i] = i + 1;

            Assert.Equal(new double[] { 1, 2, 3, 4, 5, 6, 7 },
                         new[] { area.iP, area.iN, area.jP, area.jN, area.kP, area.kN, area.totalArea });
            for (int i = 0; i < 7; i++)
                Assert.Equal(i + 1, area[i]);
        }

        /// <summary>Adding two projected areas adds each side; the total is the left one's (callers set it after).</summary>
        [SkippableFact]
        public void ProjectedAreasAddSideBySide()
        {
            UnityRuntime.Require();
            var a = new FARAeroPartModule.ProjectedArea { iP = 1, iN = 2, jP = 3, jN = 4, kP = 5, kN = 6, totalArea = 21 };
            var b = new FARAeroPartModule.ProjectedArea { iP = 10, iN = 20, jP = 30, jN = 40, kP = 50, kN = 60, totalArea = 210 };
            FARAeroPartModule.ProjectedArea sum = a + b;
            Assert.Equal(new double[] { 11, 22, 33, 44, 55, 66 }, new[] { sum.iP, sum.iN, sum.jP, sum.jN, sum.kP, sum.kN });
            Assert.Equal(21, sum.totalArea);
            Assert.Equal(1, a.iP);   // a value type: the operands are unchanged
        }

        /// <summary>MeshBuildData.Indices sets the current submesh's index list, leaving the others.</summary>
        [Fact]
        public void MeshBuildDataIndicesReplacesTheCurrentSubmesh()
        {
            var data = new MeshBuildData();
            List<int> first = new List<int> { 0, 1, 2 }, replaced = new List<int> { 3, 4, 5 };
            data.SubmeshIndices.Add(first);
            data.SubmeshIndices.Add(replaced);
            data.CurrentSubmesh = 1;

            var indices = new List<int> { 6, 7, 8 };
            data.Indices = indices;
            Assert.Same(indices, data.SubmeshIndices[1]);
            Assert.Same(indices, data.Indices);
            Assert.Same(first, data.SubmeshIndices[0]);
        }

        public static IEnumerable<object[]> IntakeAdjusters()
        {
            yield return new object[] { typeof(IntakeCrossSectionAdjuster) };
            yield return new object[] { typeof(IntegratedIntakeEngineCrossSectionAdjuster) };
        }

        /// <summary>
        /// The intake adjusters' flow sign starts forward (1) and reads back what SetForwardBackwardNoFlowDirection
        /// set, through the cross-section adjuster interface.
        /// </summary>
        [Theory]
        [MemberData(nameof(IntakeAdjusters))]
        public void IntakeFlowSignReadsBackTheDirectionSet(Type type)
        {
            var adjuster = (ICrossSectionAdjuster)Activator.CreateInstance(type, true);
            Assert.Equal(1, adjuster.GetForwardBackwardNoFlowSign());
            foreach (int direction in new[] { -1, 0, 1 })
            {
                adjuster.SetForwardBackwardNoFlowDirection(direction);
                Assert.Equal(direction, adjuster.GetForwardBackwardNoFlowSign());
            }
        }

        /// <summary>
        /// PartTint's indexer gives back the tint a part was given and refuses a part it never saw; both interface
        /// enumerators and the struct one list each tinted part once.
        /// </summary>
        [SkippableFact]
        public void PartTintIndexesAndEnumeratesItsParts()
        {
            UnityRuntime.RequireLog();   // FARConfig's voxel colors
            Part a = Uninitialized<Part>(), b = Uninitialized<Part>(), other = Uninitialized<Part>();
            var tint = new PartTint("no such map");   // the built-in default map
            Color ta = tint.Add(a), tb = tint.GetOrAdd(b);

            Assert.Equal(ta, tint[a]);
            Assert.Equal(tb, tint[b]);
            Assert.Throws<KeyNotFoundException>(() => tint[other]);

            var parts = new List<Part>();
            foreach (KeyValuePair<Part, Color> pair in tint)
                parts.Add(pair.Key);
            Assert.Equal(2, parts.Count);
            Assert.Contains(parts, p => ReferenceEquals(p, a));
            Assert.Contains(parts, p => ReferenceEquals(p, b));

            Assert.Equal(parts, ((IEnumerable<KeyValuePair<Part, Color>>)tint).Select(p => p.Key));
            Assert.Equal(parts, ((IEnumerable)tint).Cast<KeyValuePair<Part, Color>>().Select(p => p.Key));
            Assert.Equal(new[] { ta, tb }, ((IEnumerable<KeyValuePair<Part, Color>>)tint).Select(p => p.Value));
        }

        /// <summary>
        /// An exposure renderer reports its render size's aspect ratio, starts on the GPU (Device), and gives each object
        /// the color it renders it with (ColorOf), one per object in the order they were added.
        /// </summary>
        [SkippableFact]
        public void ExposureRendererReportsItsSettingsAndObjectColors()
        {
            UnityRuntime.Require();   // Color
            using var renderer = new Renderer<Part>();
            Assert.Equal(PhysicalDevice.GPU, renderer.Device);
            Assert.Equal(1f, renderer.AspectRatio);   // 512 x 512
            // an int2 (Unity.Mathematics, which these tests do not reference)
            PropertyInfo renderSize = typeof(Renderer<Part>).GetProperty("RenderSize");
            renderSize.SetValue(renderer, Activator.CreateInstance(renderSize.PropertyType, 800, 400));
            Assert.Equal(2f, renderer.AspectRatio);

            Part a = Uninitialized<Part>(), b = Uninitialized<Part>();
            MethodInfo add = typeof(Renderer<Part>).GetMethod("GetObjColor", BindingFlags.NonPublic | BindingFlags.Instance);
            var ca = (Color)add.Invoke(renderer, new object[] { a });
            var cb = (Color)add.Invoke(renderer, new object[] { b });
            Assert.Equal(ca, renderer.ColorOf(a));
            Assert.Equal(cb, renderer.ColorOf(b));
            Assert.Equal((Color)ColorUintConverter.AsColor(0), ca);
            Assert.Equal((Color)ColorUintConverter.AsColor(1), cb);
            Assert.Throws<KeyNotFoundException>(() => renderer.ColorOf(Uninitialized<Part>()));
        }

        /// <summary>
        /// The exposure executor counts the jobs it is running (JobsCount), and a job's handle says whether it is done.
        /// The executor is a component, made here without Unity; the job is taken back out by hand, as its Update would.
        /// </summary>
        [Fact]
        public void ExecutorCountsItsJobsAndHandlesReportDone()
        {
            var executor = Uninitialized<Executor>();
            var jobs = (IList)typeof(Executor).GetField("activeJobs", BindingFlags.NonPublic | BindingFlags.Static).GetValue(null);
            int before = Executor.JobsCount;

            Executor.Handle handle = executor.Execute(default);
            object job = typeof(Executor.Handle).GetField("job", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(handle);
            try
            {
                Assert.Equal(before + 1, Executor.JobsCount);
                Assert.True(handle.IsValid);
                Assert.False(handle.IsDone);
                ((Executor.JobData)job).done = true;
                Assert.True(handle.IsDone);
            }
            finally
            {
                jobs.Remove(job);
            }

            Assert.Equal(before, Executor.JobsCount);
        }

        [Fact]
        public void DummyShieldIsItsPart()
        {
            var part = Uninitialized<Part>();
            var shield = new DummyAirstreamShield { part = part };
            Assert.Same(part, shield.GetPart());
            Assert.True(shield.ClosedAndLocked());
        }

        /// <summary>A RealChute's cost modifier is FIXED: KSP need not ask for it again as the part changes.</summary>
        [Fact]
        public void RealChuteCostChangesAreFixed()
        {
            Assert.Equal(ModifierChangeWhen.FIXED, Uninitialized<RealChuteFAR>().GetModuleCostChangeWhen());
        }

        /// <summary>
        /// A PhysicsWatch made with a time reports it in milliseconds and as a TimeSpan, and stopping one that is not
        /// running changes nothing.
        /// </summary>
        [Fact]
        public void PhysicsWatchReportsItsTime()
        {
            var watch = new PhysicsWatch(2.0006);
            Assert.Equal(2001, watch.ElapsedMilliseconds);   // rounded
            Assert.Equal("00:00:02.0006000", watch.ToString());
            Assert.Equal(0, new PhysicsWatch().ElapsedMilliseconds);
            Assert.Equal(TimeSpan.Zero.ToString(), new PhysicsWatch().ToString());
        }
    }
}
