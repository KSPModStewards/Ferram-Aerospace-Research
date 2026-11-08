using System;
using System.Collections;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>FAR's predictions (FARAPI.CalculateVesselAeroForcesAt) against the game's own state.</summary>
    public static class FarPredictionTests
    {
        /// <summary>
        /// FAR's prediction atmosphere against the game's (builds with FARAPI.CalculateVesselAeroForcesAt). On the runway,
        /// the predicted lift at the vessel's latitude, at 0/90/180/270 deg of longitude away and at several altitudes:
        /// its ratio to the lift at the vessel's own longitude should follow the game's density ratio between the
        /// points (Trajectories' StockAeroUtil, the FlightIntegrator formula), at a speed low enough (liftSpeed, 40 m/s)
        /// that the few percent of temperature between points moves the lift coefficient by far less than that; and a
        /// quarter rotation later at the vessel's longitude, which should equal a quarter turn east now. Records FAR's
        /// public gas density (FARAtmosphere.GetGasProperties) against vessel.atmDensity and the game's: on a FAR whose
        /// default atmosphere is the altitude curve again (predictions alone take the game's) it is not asserted.
        /// Params: speed (150 m/s, for the quarter-turn check), liftSpeed (40 m/s), aoa (5 deg), alts ("100,10000,30000").
        /// </summary>
        [UITest("Far_PredictionAtmosphere_Flight", Kind = "Check", Mods = "Trajectories", StartsAt = StartState.Flight, Craft = "Aeris 4A", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 600)]
        public static IEnumerator Far_PredictionAtmosphere_Flight(TestContext ctx)
        {
            yield return Wait.Seconds(10);   // FAR voxelizes the craft after load
            Assert.True(FarApi.HasPredictAt, "FAR has FARAPI.CalculateVesselAeroForcesAt");
            var inv = System.Globalization.CultureInfo.InvariantCulture;
            var v = FlightGlobals.ActiveVessel;
            CelestialBody body = v.mainBody;
            double now = Planetarium.GetUniversalTime();

            double farRho = FarApi.FarPredictionDensity(v);
            ctx.Record("farPublicRhoOverAtmDensity", Math.Round(farRho / v.atmDensity, 5));

            Vector3 vel = FarSim.BodyVelocity(v, ctx.ParamDouble("speed", 150), ctx.ParamDouble("aoa", 5), 0);
            Vector3 slow = FarSim.BodyVelocity(v, ctx.ParamDouble("liftSpeed", 40), ctx.ParamDouble("aoa", 5), 0);
            double lat = v.latitude, lon = v.longitude;
            double worstRho = 0, worstOld = 0, worstRepeat = 0, worstTurn = 0, worstLift = 0;
            foreach (string s in (ctx.Param("alts") ?? "100,10000,30000").Split(','))
            {
                double alt = double.Parse(s.Trim(), inv);
                double oldRho = body.GetDensity(body.GetPressure(alt), body.GetTemperature(alt));
                double lift0 = double.NaN, rho0 = double.NaN, slow0 = double.NaN;
                for (int dLon = 0; dLon <= 360; dLon += 90)
                {
                    Assert.True(FarApi.PredictAt(v, vel, lat, lon + dLon, alt, now, out double lift, out _), "CalculateVesselAeroForcesAt");
                    FarApi.PredictAt(v, slow, lat, lon + dLon, alt, now, out double slowLift, out _);
                    double game = TrajApi.StockDensityAt(body, body.GetWorldSurfacePosition(lat, lon + dLon, alt));
                    double far = FarApi.FarDensityAt(body, lat, lon + dLon, alt, now);
                    if (dLon == 0) { lift0 = lift; rho0 = game; slow0 = slowLift; }
                    worstRho = Math.Max(worstRho, Math.Abs(far / game - 1));
                    worstLift = Math.Max(worstLift, Math.Abs(slowLift / slow0 / (game / rho0) - 1));
                    worstOld = Math.Max(worstOld, Math.Abs(oldRho / game - 1));
                    if (dLon == 360) worstRepeat = Math.Max(worstRepeat, Math.Abs(lift / lift0 - 1));
                    ctx.Record($"alt{alt:F0}.dLon{dLon}", $"FAR public/game density {far / game:F5} (altitude curve {oldRho / game:F4}); lift x{lift / lift0:F4}, at {ctx.ParamDouble("liftSpeed", 40):F0} m/s x{slowLift / slow0:F5}, vs density x{game / rho0:F5}");
                }
                if (body.rotates && body.rotationPeriod != 0)
                {
                    FarApi.PredictAt(v, vel, lat, lon, alt, now + body.rotationPeriod / 4, out double later, out _);
                    FarApi.PredictAt(v, vel, lat, lon + 90, alt, now, out double east, out _);
                    worstTurn = Math.Max(worstTurn, Math.Abs(later / east - 1));
                }
            }
            ctx.Record("worstFarPublicVsGameDensityPct", Math.Round(worstRho * 100, 4));
            ctx.Record("worstSlowLiftVsDensityPct", Math.Round(worstLift * 100, 4));
            ctx.Record("worstOldAtmosphereErrPct", Math.Round(worstOld * 100, 3));
            // Not asserted: the first call at a new altitude differs from a repeat because downstream wings read their
            // upstream wing's lift slope, stall, sweep and Cd0 from its last calculation (FARWingInteraction), whatever
            // state that was at. A separate FAR issue from the atmosphere.
            ctx.Record("worstRepeatPct", Math.Round(worstRepeat * 100, 4));
            ctx.Record("worstQuarterTurnVsEastPct", Math.Round(worstTurn * 100, 4));
            Assert.True(worstLift < 0.002, $"FAR's predicted lift follows the game's density between points (worst {worstLift * 100:F3}%)");
            Assert.True(worstTurn < 1e-3, $"a quarter turn later matches 90 deg east now (worst {worstTurn * 100:F3}%)");
        }
    }
}
