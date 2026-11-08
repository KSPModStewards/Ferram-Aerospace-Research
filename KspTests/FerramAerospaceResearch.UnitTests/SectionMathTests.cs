using System;
using System.Linq;
using System.Reflection;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARPartGeometry;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// The private helpers of the body model that take numbers only: VehicleAerodynamics' critical Mach number
    /// table, its clamp and its Gaussian smoothing of the cross-sections.
    /// </summary>
    public class SectionMathTests
    {
        private const BindingFlags Private = BindingFlags.NonPublic | BindingFlags.Static;

        private static MethodInfo Method(Type t, string name) =>
            t.GetMethod(name, Private) ?? throw new MissingMethodException(t.Name, name);

        private static double CriticalMach(double fineness) =>
            (double)Method(typeof(VehicleAerodynamics), "CalculateCriticalMachNumber").Invoke(null, new object[] { fineness });

        private static double ClampAbs(double value, double abs) =>
            (double)Method(typeof(VehicleAerodynamics), "MathClampAbs").Invoke(null, new object[] { value, abs });

        private static void Smooth(VoxelCrossSection[] sections, double lengthPercentFactor, double sectionThickness, int areaIterations,
                                   int derivIterations) =>
            Method(typeof(VehicleAerodynamics), "GaussianSmoothCrossSections")
                .Invoke(null, new object[]
                {
                    sections, 2.0, lengthPercentFactor, sectionThickness, sections.Length * sectionThickness, 0, sections.Length - 1,
                    areaIterations, derivIterations
                });

        /// <summary>
        /// The transonic drag onset against the body's fineness ratio: FAR's piecewise table (0.285 for a blunt body up
        /// to 0.925 for a slender one), continuous where its pieces meet, and never falling as the body gets slenderer.
        /// </summary>
        [Theory]
        [InlineData(1.0, 0.285)]
        [InlineData(2.0, 0.45)]
        [InlineData(3.5, 0.815)]
        [InlineData(5.0, 0.875)]
        [InlineData(8.0, 0.9125)]
        [InlineData(12.0, 0.925)]
        public void CriticalMachFollowsTheFinenessTable(double fineness, double mach) => Assert.Equal(mach, CriticalMach(fineness), 9);

        [Theory]
        [InlineData(1.5)]
        [InlineData(3.0)]
        [InlineData(4.0)]
        [InlineData(6.0)]
        [InlineData(10.0)]
        public void CriticalMachIsContinuousAtTheTableJoins(double fineness) =>
            Assert.Equal(CriticalMach(fineness - 1e-9), CriticalMach(fineness + 1e-9), 6);

        [Fact]
        public void CriticalMachNeverFallsWithFineness()
        {
            double last = 0;
            for (double f = 0.5; f < 15; f += 0.05)
            {
                double m = CriticalMach(f);
                Assert.True(m >= last - 1e-12, $"critical Mach {m} at fineness {f} below {last} just before");
                last = m;
            }
        }

        [Theory]
        [InlineData(5.0, 2.0, 2.0)]
        [InlineData(-5.0, 2.0, -2.0)]
        [InlineData(1.5, 2.0, 1.5)]
        [InlineData(-1.5, 2.0, -1.5)]
        public void ClampAbsKeepsTheValueWithinTheMagnitude(double value, double abs, double clamped) =>
            Assert.Equal(clamped, ClampAbs(value, abs));

        private static VoxelCrossSection[] Profile(params double[] areas) =>
            areas.Select(a => new VoxelCrossSection { area = a }).ToArray();

        /// <summary>A body of constant section smoothed: the area stays what it was, and its second derivative is nil.</summary>
        [SkippableFact]
        public void SmoothingKeepsAConstantSection()
        {
            UnityRuntime.Require();   // Vector3d in the sections
            VoxelCrossSection[] s = Profile(Enumerable.Repeat(2.0, 40).ToArray());
            Smooth(s, 0.1, 0.1, 2, 2);
            foreach (VoxelCrossSection x in s)
            {
                Assert.Equal(2.0, x.area, 9);
                Assert.Equal(0.0, x.secondAreaDeriv, 9);
            }
        }

        /// <summary>
        /// Empty sections at the ends (the space before the nose and after the tail) are left out of the average: the
        /// body's sections next to them keep their area instead of being pulled toward zero, and the empty ones take
        /// the body's.
        /// </summary>
        [SkippableFact]
        public void SmoothingLeavesOutTheEmptyEnds()
        {
            UnityRuntime.Require();
            double[] areas = Enumerable.Repeat(0.0, 5).Concat(Enumerable.Repeat(3.0, 30)).Concat(Enumerable.Repeat(0.0, 5)).ToArray();
            VoxelCrossSection[] s = Profile(areas);
            Smooth(s, 0.05, 0.1, 1, 1);
            for (int i = 5; i < 35; i++)
                Assert.Equal(3.0, s[i].area, 9);
            Assert.True(s[4].area > 0 && s[35].area > 0, $"the empty sections next to the body take some of its area ({s[4].area}, {s[35].area})");
            Assert.All(s, x => Assert.False(double.IsNaN(x.secondAreaDeriv) || double.IsInfinity(x.secondAreaDeriv)));
        }

        /// <summary>A filter narrower than two sections does nothing: a noisy profile comes back as it went in.</summary>
        [SkippableFact]
        public void SmoothingNarrowerThanASectionChangesNothing()
        {
            UnityRuntime.Require();
            double[] areas = { 1, 5, 2, 7, 3, 9, 4 };
            VoxelCrossSection[] s = Profile(areas);
            for (int i = 0; i < s.Length; i++)
                s[i].secondAreaDeriv = 10 * i;
            // 2 standard deviations of 0.7 * 0.001 m against 0.1 m sections: one value
            Smooth(s, 0.001, 0.1, 3, 3);
            Assert.Equal(areas, s.Select(x => x.area).ToArray());
            Assert.Equal(Enumerable.Range(0, s.Length).Select(i => 10.0 * i).ToArray(), s.Select(x => x.secondAreaDeriv).ToArray());
        }
    }
}
