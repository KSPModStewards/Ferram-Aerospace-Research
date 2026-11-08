using System;
using ferram4;
using FerramAerospaceResearch.Settings;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// The adiabatic index FARAeroUtil's formulas read, without the game: they take it from the calling thread's
    /// prediction when there is one (SimThreadContext.PredictionCall.AdiabaticIndex), else from the current body's
    /// atmosphere. <c>using (Gas.Gamma(1.4)) { ... }</c>.
    /// </summary>
    public static class Gas
    {
        /// <summary>
        /// The test collection of the tests that take prediction slots: the slots are shared by every thread, and a test
        /// that takes all of them would find one taken by a test running beside it.
        /// </summary>
        public const string Collection = "FAR's prediction slots";

        public static IDisposable Gamma(double gamma) => Air(gamma, null);

        /// <summary>The adiabatic index, and the body's air (Sutherland's reference viscosity and temperature) too.</summary>
        public static IDisposable Air(double gamma, BodySettings? body)
        {
            SimThreadContext.PredictionCall call = SimThreadContext.BeginPrediction(default);
            call.AdiabaticIndex = gamma;
            call.BodyData = body;
            return new End(call);
        }

        private sealed class End : IDisposable
        {
            private SimThreadContext.PredictionCall call;
            public End(SimThreadContext.PredictionCall call) => this.call = call;

            public void Dispose()
            {
                if (call == null) return;
                SimThreadContext.EndPrediction(call);
                call = null;
            }
        }
    }
}
