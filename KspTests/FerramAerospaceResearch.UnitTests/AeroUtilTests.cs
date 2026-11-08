using System;
using System.Runtime.Serialization;
using FerramAerospaceResearch.Settings;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// FARAeroUtil's gas dynamics and skin friction, against the standard relations for a perfect gas (gamma 1.4 unless
    /// said otherwise; values as tabulated in NACA Report 1135).
    /// </summary>
    [Collection(Gas.Collection)]
    public class AeroUtilTests
    {
        private const double Gamma = 1.4;

        [Theory]
        [InlineData(1.0, 1.0)]
        [InlineData(2.0, 4.5)]
        [InlineData(3.0, 10.333333)]
        public void NormalShockPressureRatio(double mach, double p2OverP1)
        {
            using (Gas.Gamma(Gamma))
                Assert.Equal(p2OverP1, FARAeroUtil.PressureBehindShockCalc(mach), 5);
        }

        [Theory]
        [InlineData(1.0, 1.0)]
        [InlineData(2.0, 0.577350)]
        [InlineData(3.0, 0.475191)]
        public void NormalShockDownstreamMach(double mach, double mach2)
        {
            using (Gas.Gamma(Gamma))
                Assert.Equal(mach2, FARAeroUtil.MachBehindShockCalc(mach), 5);
        }

        [Theory]
        [InlineData(0.0, 1.0)]
        [InlineData(1.0, 1.892929)]
        [InlineData(2.0, 7.824449)]
        public void IsentropicStagnationPressureRatio(double mach, double p0OverP)
        {
            using (Gas.Gamma(Gamma))
                Assert.Equal(p0OverP, FARAeroUtil.StagnationPressureCalc(mach), 5);
        }

        [Theory]
        [InlineData(2.0, 5.6405)]
        [InlineData(3.0, 12.0611)]
        public void RayleighPitotPressureRatio(double mach, double p02OverP1)
        {
            using (Gas.Gamma(Gamma))
                Assert.Equal(p02OverP1, FARAeroUtil.RayleighPitotTubeStagPressure(mach), 3);
        }

        [Theory]
        [InlineData(0.5)]
        [InlineData(1.0)]
        public void PitotPressureIsIsentropicUpToMachOne(double mach)
        {
            using (Gas.Gamma(Gamma))
                Assert.Equal(FARAeroUtil.StagnationPressureCalc(mach), FARAeroUtil.RayleighPitotTubeStagPressure(mach), 12);
        }

        [Fact]
        public void PitotPressureIsContinuousThroughMachOne()
        {
            using (Gas.Gamma(Gamma))
                Assert.Equal(FARAeroUtil.RayleighPitotTubeStagPressure(1 - 1e-9), FARAeroUtil.RayleighPitotTubeStagPressure(1 + 1e-9), 6);
        }

        [Fact]
        public void ShockRelationsFollowTheAdiabaticIndex()
        {
            double air, co2;
            using (Gas.Gamma(1.4))
                air = FARAeroUtil.PressureBehindShockCalc(2);
            using (Gas.Gamma(1.3))
                co2 = FARAeroUtil.PressureBehindShockCalc(2);
            Assert.Equal((2 * 1.3 * 4 - 0.3) / 2.3, co2, 10);
            Assert.NotEqual(air, co2);
        }

        // oblique shocks: theta-beta-Mach, weak solution
        [Theory]
        [InlineData(2.0, 10.0, 39.3139)]
        [InlineData(3.0, 20.0, 37.7636)]
        public void WeakObliqueShockAngle(double mach, double deflectionDeg, double betaDeg)
        {
            double sinBeta = FARAeroUtil.CalculateSinWeakObliqueShockAngle(mach, Gamma, deflectionDeg * Math.PI / 180);
            Assert.Equal(betaDeg, Math.Asin(sinBeta) * 180 / Math.PI, 1);
        }

        [Theory]
        [InlineData(1.5, 5.0)]
        [InlineData(2.0, 15.0)]
        [InlineData(4.0, 30.0)]
        [InlineData(8.0, 10.0)]
        [InlineData(20.0, 40.0)]
        public void WeakObliqueShockAngleTurnsTheFlowThroughTheDeflection(double mach, double deflectionDeg)
        {
            // the theta-beta-Mach relation back from the shock angle gives the deflection it was solved for
            double beta = Math.Asin(FARAeroUtil.CalculateSinWeakObliqueShockAngle(mach, Gamma, deflectionDeg * Math.PI / 180));
            double m2s2 = mach * mach * Math.Sin(beta) * Math.Sin(beta);
            double tanTheta = 2 / Math.Tan(beta) * (m2s2 - 1) / (mach * mach * (Gamma + Math.Cos(2 * beta)) + 2);
            Assert.Equal(deflectionDeg, Math.Atan(tanTheta) * 180 / Math.PI, 1);
            // the weak solution: above the Mach angle, below the angle of maximum deflection
            Assert.InRange(beta, Math.Asin(1 / mach), Math.Asin(FARAeroUtil.CalculateSinMaxShockAngle(mach, Gamma)));
        }

        [Theory]
        [InlineData(1.5, 12.11)]
        [InlineData(2.0, 22.97)]
        [InlineData(3.0, 34.07)]
        [InlineData(10.0, 44.43)]
        public void ObliqueShockBelowTheMaximumDeflectionIsAttached(double mach, double maxDeflectionDeg)
        {
            Assert.False(double.IsNaN(FARAeroUtil.CalculateSinWeakObliqueShockAngle(mach, Gamma, (maxDeflectionDeg - 1) * Math.PI / 180)));
        }

        // FAR's cubic solution counts a shock as attached while its discriminant is below 0.001, not 0: up to 8-10 deg
        // past the maximum deflection it gives a shock angle (above the angle at the maximum, which the wing model
        // means to clamp to but does not: FARWingAerodynamicModel.ShockWaveCalculation drops Clamp's result).
        [Theory(Skip = "FAR bug: CalculateSinWeakObliqueShockAngle stays attached up to ~8-10 deg past the maximum deflection (D > 0.001)")]
        [InlineData(1.5, 12.11)]
        [InlineData(2.0, 22.97)]
        [InlineData(3.0, 34.07)]
        [InlineData(10.0, 44.43)]
        public void ObliqueShockAboveTheMaximumDeflectionIsDetached(double mach, double maxDeflectionDeg)
        {
            Assert.True(double.IsNaN(FARAeroUtil.CalculateSinWeakObliqueShockAngle(mach, Gamma, (maxDeflectionDeg + 1) * Math.PI / 180)));
        }

        // skin friction: flat plate, FAR's 8% roughness allowance on top
        private const double Roughness = 1.08;

        [Theory]
        [InlineData(1e4)]
        [InlineData(1e5)]
        [InlineData(4e5)]
        public void LaminarSkinFrictionIsBlasius(double reynolds)
        {
            Assert.Equal(1.328 / Math.Sqrt(reynolds) * Roughness, FARAeroUtil.SkinFrictionDrag(reynolds, 0.1), 9);
        }

        [Theory]
        [InlineData(1e6)]
        [InlineData(1e7)]
        [InlineData(1e8)]
        public void TurbulentSkinFrictionIsPrandtlSchlichtingWithTransition(double reynolds)
        {
            // Cf = 0.074 / Re^0.2 - A / Re, A ~ 1742 for transition at Re 5e5
            double expected = (0.074 / Math.Pow(reynolds, 0.2) - 1742 / reynolds) * Roughness;
            Assert.InRange(FARAeroUtil.SkinFrictionDrag(reynolds, 0.5) / expected, 0.995, 1.005);
        }

        [Fact]
        public void SkinFrictionIsContinuousAtTransition()
        {
            double below = FARAeroUtil.SkinFrictionDrag(5e5 * (1 - 1e-9), 0.3);
            double above = FARAeroUtil.SkinFrictionDrag(5e5 * (1 + 1e-9), 0.3);
            Assert.InRange(above / below, 0.999, 1.001);
        }

        [Theory]
        [InlineData(1e3, 5e5, -1)]    // laminar
        [InlineData(5e5, 2e6, 1)]     // transition: the turbulent share of the plate grows faster than Cf falls
        [InlineData(3e6, 1e9, -1)]    // turbulent
        public void SkinFrictionAgainstReynoldsNumber(double from, double to, int sign)
        {
            double previous = FARAeroUtil.SkinFrictionDrag(from, 0.2);
            for (double re = from * 1.1; re < to; re *= 1.1)
            {
                double cf = FARAeroUtil.SkinFrictionDrag(re, 0.2);
                Assert.True(Math.Sign(cf - previous) == sign, $"Cf {(cf > previous ? "rose" : "fell")} to {cf} at Re {re:E2}");
                previous = cf;
            }
        }

        [Fact]
        public void RarefiedSkinFrictionTendsToFreeMolecularLimit()
        {
            // past M/Re = 0.01 FAR blends from Blasius toward 0.075
            double laminar = 1.328 / Math.Sqrt(100) * Roughness;
            double blended = FARAeroUtil.SkinFrictionDrag(100, 5);
            Assert.InRange(blended, Math.Min(laminar, 0.075 * Roughness), Math.Max(laminar, 0.075 * Roughness));
            Assert.Equal(0.075 * Roughness, FARAeroUtil.SkinFrictionDrag(100, 1e6), 4);
        }

        // Sutherland's law, the body's reference viscosity at its reference temperature, S = 110 K
        private static readonly BodySettings Sea = new BodySettings { ReferenceViscosity = 1.789e-5, ReferenceTemperature = 288.15 };

        [Theory]
        [InlineData(288.15, 1.789e-5)]
        [InlineData(400.0, 2.2843e-5)]
        [InlineData(200.0, 1.3287e-5)]
        public void ViscosityFollowsSutherland(double tempK, double viscosity)
        {
            using (Gas.Air(Gamma, Sea))
                Assert.InRange(FARAeroUtil.CalculateCurrentViscosity(tempK) / viscosity, 0.999, 1.001);
        }

        [Theory]
        [InlineData(0.0, 1.0)]
        [InlineData(2.0, 1.519152)]   // Eckert: 1 + 0.032 M^2 + 0.58 r (gamma - 1) / 2 M^2, r = 0.843
        public void ReferenceTemperatureRatioIsEckert(double mach, double ratio)
        {
            Assert.Equal(ratio, FARAeroUtil.ReferenceTemperatureRatio(mach, 0.843, Gamma), 6);
        }

        [Fact]
        public void ReynoldsNumberIsDensityLengthSpeedOverViscosityAtTheReferenceTemperature()
        {
            using (Gas.Air(Gamma, Sea))
            {
                double refTemp = 288.15 * FARAeroUtil.ReferenceTemperatureRatio(0.1, 0.843, Gamma);
                double expected = 1.225 * 2 * 34 / FARAeroUtil.CalculateCurrentViscosity(refTemp);
                Assert.Equal(expected, FARAeroUtil.CalculateReynoldsNumber(1.225, 2, 34, 0.1, 288.15, Gamma), 3);
                Assert.Equal(0, FARAeroUtil.CalculateReynoldsNumber(1.225, 0, 34, 0.1, 288.15, Gamma));
                Assert.Equal(0, FARAeroUtil.SkinFrictionDrag(1.225, 0, 34, 0.1, 288.15, Gamma));
            }
        }

        /// <summary>
        /// The failure force scaling for a body and altitude: 1 in the air (above sea level) or over a body with no
        /// ocean, at any altitude; at and below sea level on an ocean body, 5% of FAR's water-to-air density factor
        /// (814.5) per meter of depth, a meter at least.
        /// </summary>
        [Theory]
        [InlineData(true, 100.0, 1.0)]
        [InlineData(true, 0.001, 1.0)]
        [InlineData(false, -50.0, 1.0)]
        [InlineData(true, 0.0, 40.725510204)]
        [InlineData(true, -0.5, 40.725510204)]
        [InlineData(true, -1.0, 40.725510204)]
        [InlineData(true, -10.0, 407.25510204)]
        public void FailureForceScalingByDepth(bool ocean, double altitude, double scaling)
        {
            var body = (CelestialBody)FormatterServices.GetUninitializedObject(typeof(CelestialBody));
            body.ocean = ocean;
            Assert.Equal(scaling, FARAeroUtil.GetFailureForceScaling(body, altitude), 6);
        }
    }
}
