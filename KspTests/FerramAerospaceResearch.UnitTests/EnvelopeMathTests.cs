using System;
using System.Reflection;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// The envelope sweep's number-only helpers: the Breguet endurance (PerformanceEnvelopeCalculator.EnduranceHours),
    /// the engine deck's type labels and thermal throttle cap (EditorEngineDeck, private statics), and the RK4
    /// integrator the stability tab's time responses use (RungeKutta4) on a system that blows up.
    /// </summary>
    public class EnvelopeMathTests
    {
        private const BindingFlags Private = BindingFlags.NonPublic | BindingFlags.Static;
        private const double G0 = 9.80665;

        private static MethodInfo Method(Type t, string name) =>
            t.GetMethod(name, Private) ?? throw new MissingMethodException(t.Name, name);

        private static string FriendlyEngineType(string typeName) =>
            (string)Method(typeof(EditorEngineDeck), "FriendlyEngineType").Invoke(null, new object[] { typeName });

        private static double ThermalCapThrottle(double throttle, double thermalRatio, double thermal0) =>
            (double)Method(typeof(EditorEngineDeck), "ThermalCapThrottle")
                .Invoke(null, new object[] { throttle, thermalRatio, thermal0 });

        /// <summary>E = Isp * (L/D) * ln(m0/m1), Isp = D / (fuel flow * g0), in hours.</summary>
        [Fact]
        public void EnduranceIsBreguets()
        {
            double isp = 10000 / (1.0 * G0);
            Assert.Equal(isp * 12 * Math.Log(2) / 3600, PerformanceEnvelopeCalculator.EnduranceHours(10000, 1, 12, 2000, 1000), 12);
            // twice the fuel flow for the same drag halves the Isp and with it the endurance
            Assert.Equal(PerformanceEnvelopeCalculator.EnduranceHours(10000, 1, 12, 2000, 1000) / 2,
                         PerformanceEnvelopeCalculator.EnduranceHours(10000, 2, 12, 2000, 1000), 12);
        }

        [Theory]
        [InlineData(10000, 0, 12, 2000, 1000)]                // no fuel flow
        [InlineData(10000, -1, 12, 2000, 1000)]
        [InlineData(10000, 1, 0, 2000, 1000)]                 // no lift over drag
        [InlineData(10000, 1, 12, 2000, 0)]                   // no dry mass
        [InlineData(10000, 1, 12, 1000, 1000)]                // at the dry mass: nothing to burn
        [InlineData(10000, 1, 12, 900, 1000)]                 // below it
        [InlineData(0, 1, 12, 2000, 1000)]                    // zero drag: Isp 0
        [InlineData(-5000, 1, 12, 2000, 1000)]                // negative Isp
        [InlineData(double.NaN, 1, 12, 2000, 1000)]           // drag unresolved
        [InlineData(double.PositiveInfinity, 1, 12, 2000, 1000)]
        public void EnduranceWithoutAnAnswerIsNaN(double drag, double fuelFlow, double liftToDrag, double mass, double dry) =>
            Assert.True(double.IsNaN(PerformanceEnvelopeCalculator.EnduranceHours(drag, fuelFlow, liftToDrag, mass, dry)));

        [Fact]
        public void RangeIsEnduranceTimesSpeed()
        {
            var c = new PerformanceEnvelopeCalculator.CellResult { EnduranceHours = 2, SpeedMPerS = 250 };
            Assert.Equal(2 * 250 * 3.6, PerformanceEnvelopeCalculator.RangeKm(c), 12);
            c.EnduranceHours = double.NaN;
            Assert.True(double.IsNaN(PerformanceEnvelopeCalculator.RangeKm(c)));
        }

        [Theory]
        [InlineData("ModuleEnginesAJEJet", "Turbojet/fan")]
        [InlineData("ModuleEnginesAJERamjet", "Ramjet")]
        [InlineData("ModuleEnginesAJEPropeller", "Propeller")]
        [InlineData("ModuleEnginesAJERotor", "Rotor")]
        [InlineData("ModuleEnginesFX", "FX")]                 // the boilerplate prefix stripped
        [InlineData("ModuleEnginesRF", "RF")]
        [InlineData("ModuleEngines", "")]
        [InlineData("MyEngineModule", "MyEngineModule")]      // unknown: kept as it is
        public void EngineTypesGetReadableLabels(string typeName, string label) => Assert.Equal(label, FriendlyEngineType(typeName));

        /// <summary>Temperature linear in throttle from thermal0 (throttle 0) to thermalRatio (the demand): the cap is
        /// the throttle where it reaches EditorEngineDeck.ThermalLimit.</summary>
        [Fact]
        public void ThermalCapIsWhereTheTemperatureMeetsTheLimit()
        {
            double cap = ThermalCapThrottle(1.0, 1.2, 0.5);
            Assert.Equal((EditorEngineDeck.ThermalLimit - 0.5) / (1.2 - 0.5), cap, 12);
            Assert.Equal(EditorEngineDeck.ThermalLimit, 0.5 + (1.2 - 0.5) * cap, 12);
            // a half-throttle demand scales the cap with it
            Assert.Equal(cap * 0.5, ThermalCapThrottle(0.5, 1.2, 0.5), 12);
        }

        [Theory]
        [InlineData(1.0, 1.2, 1.2)]           // flat: temperature set by the inlet alone, no throttle helps
        [InlineData(1.0, 1.0, 1.0 + 1e-7)]    // within the 1e-6 slope tolerance
        [InlineData(1.0, 1.0, 1.3)]           // falling with throttle
        public void ThermalCapShutsTheEngineWithoutASlope(double throttle, double thermalRatio, double thermal0) =>
            Assert.Equal(0.0, ThermalCapThrottle(throttle, thermalRatio, thermal0));

        [Fact]
        public void ThermalCapIsClampedToZeroAndTheDemand()
        {
            Assert.Equal(0.0, ThermalCapThrottle(1.0, 1.5, 0.99));    // over the limit even at idle
            Assert.Equal(0.8, ThermalCapThrottle(0.8, 0.9, 0.5), 12); // under the limit at the demand: the line meets it past 0.8
        }

        private static SimMatrix Scalar(double a)
        {
            var m = new SimMatrix(1, 1);
            m.Add(a, 0, 0);
            return m;
        }

        /// <summary>x' = -x from 1: RK4 follows exp(-t) (its weights are floats, so to ~1e-6).</summary>
        [SkippableFact]
        public void SolvesADecay()
        {
            UnityRuntime.Require();   // Vector4
            var rk = new RungeKutta4(1, 0.01, Scalar(-1), new[] { 1.0 });
            rk.Solve();
            Assert.Equal(100, rk.time.Length);
            for (int j = 0; j < rk.time.Length; j++)
            {
                Assert.Equal(j * 0.01, rk.time[j], 9);
                Assert.Equal(Math.Exp(-rk.time[j]), rk.soln[0, j], 5);
            }
        }

        /// <summary>A state's time history; a state the system does not have reads as zeros (and is logged).</summary>
        [SkippableFact]
        public void GetSolutionOfAMissingStateIsZeros()
        {
            UnityRuntime.RequireLog();   // FARLogger
            var rk = new RungeKutta4(0.625, 0.125, Scalar(-1), new[] { 2.0 });
            rk.Solve();
            double[] x = rk.GetSolution(0);
            Assert.Equal(5, x.Length);
            for (int j = 0; j < x.Length; j++)
                Assert.Equal(rk.soln[0, j], x[j]);
            Assert.Equal(2.0, x[0]);
            Assert.Equal(new double[5], rk.GetSolution(1));
        }

        /// <summary>
        /// x' = 1e30 x overflows after a few steps: the solution stops at the last finite state, the rest of the arrays
        /// left zero (never NaN or infinite, which the time-response graph could not draw).
        /// </summary>
        [SkippableFact]
        public void DivergingSystemStopsAtTheFirstNonFiniteState()
        {
            UnityRuntime.Require();   // Vector4
            var rk = new RungeKutta4(10, 1, Scalar(1e30), new[] { 1.0 });
            rk.Solve();
            Assert.Equal(10, rk.time.Length);

            int written = 1;
            while (written < rk.time.Length && rk.soln[0, written] != 0)
                written++;
            Assert.InRange(written, 2, rk.time.Length - 1);   // a few finite steps, then it stopped
            for (int j = 0; j < rk.time.Length; j++)
            {
                Assert.True(FARMathUtil.IsFinite(rk.soln[0, j]), $"soln[{j}] = {rk.soln[0, j]}");
                if (j < written)
                {
                    Assert.Equal(j, rk.time[j]);
                    if (j > 0)
                        Assert.True(rk.soln[0, j] > rk.soln[0, j - 1] * 1e10);   // growing fast
                }
                else
                {
                    Assert.Equal(0.0, rk.soln[0, j]);
                    Assert.Equal(0.0, rk.time[j]);
                }
            }
        }
    }
}
