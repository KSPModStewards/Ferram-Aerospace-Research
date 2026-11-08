using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using KspTest;
using KspTest.MechJeb;

namespace FerramAerospaceResearch.KspTests.MechJeb
{
    /// <summary>FAR flight tests flown with MechJeb (FAR's; MechJeb's own tests are in the MechJeb2 repo, MechJebKspTest).</summary>
    public static class FarMechJebFlightTests
    {
        /// <summary>
        /// Part heating through an atmospheric entry, for comparing FAR builds: the plane is put on an entry orbit
        /// with the stock set-orbit cheat, holds an angle of attack on MechJeb, and every part's skin and interior
        /// temperature, convective flux, stock exposed area and FAR voxel area are logged through the entry (parts
        /// over their limit are kept by the ignore-max-temperature cheat). Params: craft (default Aeris 4A), peKm
        /// (default 30), apKm (default 90), aoa (deg, default 10), gear (up|down, default up), startAltKm (default
        /// 69, sampling starts below it), stopAltKm (default 20), stopSpeed (m/s, default 500), physWarp (default 2),
        /// sampleSec (default 0.5), rcs (on|off, default on), holdBandDeg (default 3) and holdMaxOutside (fraction of
        /// samples below 60 km allowed outside the band, default 0.05): a run whose attitude hold leaves the band more
        /// than that is reported as failed, since its heating is not comparable.
        /// </summary>
        [UITest("Far_Heating_Flight", Explicit = true, Kind = "Check", Mods = "MechJeb2", Requires = "MechJeb", StartsAt = StartState.Flight, Craft = "Aeris 4A", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 1800)]
        public static IEnumerator Far_Heating_Flight(TestContext ctx)
        {
            Assert.True(Mj.Present, "vessel has a MechJebCore (MechJebForAll or an MJ part)");
            var inv = System.Globalization.CultureInfo.InvariantCulture;
            double peKm = ctx.ParamDouble("peKm", 30), apKm = ctx.ParamDouble("apKm", 90), aoa = ctx.ParamDouble("aoa", 10);
            bool gearDown = (ctx.Param("gear") ?? "up").Equals("down", StringComparison.OrdinalIgnoreCase);
            double startAlt = ctx.ParamDouble("startAltKm", 69) * 1000, stopAlt = ctx.ParamDouble("stopAltKm", 20) * 1000, stopSpeed = ctx.ParamDouble("stopSpeed", 500);
            double sampleSec = ctx.ParamDouble("sampleSec", 0.5);
            var v = FlightGlobals.ActiveVessel;
            Flight.IgnoreMaxTemperature(true);

            ctx.Step("Set the entry orbit");
            Flight.SetOrbit(peKm * 1000, apKm * 1000);
            yield return Wait.Until(() => FlightGlobals.ActiveVessel.altitude > apKm * 1000 * 0.9, 30, "vessel at the apoapsis");
            yield return Wait.Seconds(2);
            v = FlightGlobals.ActiveVessel;
            ctx.Record("apoapsisKm", Math.Round(v.orbit.ApA / 1000, 1)); ctx.Record("periapsisKm", Math.Round(v.orbit.PeA / 1000, 1));
            Ctrl.Gear(gearDown);
            v.ActionGroups.SetGroup(KSPActionGroup.RCS, !(ctx.Param("rcs") ?? "on").Equals("off", StringComparison.OrdinalIgnoreCase));
            Mj.SmartAssSurfaceProgradeOffset(aoa);
            yield return Wait.Seconds(5);
            yield return Wait.Until(() => !Ctrl.GearStates().Contains("ing"), 30, "gear finished moving", shot: false);
            ctx.Record("gearStates", Ctrl.GearStates());

            ctx.Step("Warp to the atmosphere");
            Flight.Warp(3);
            yield return Wait.Until(() => FlightGlobals.ActiveVessel.altitude < startAlt + 8000, 900, "approach the atmosphere", shot: false);
            Flight.WarpOff();
            yield return Wait.Until(() => TimeWarp.CurrentRate <= 1.01, 30, "off rails", shot: false);
            Mj.SmartAssSurfaceProgradeOffset(aoa);
            Flight.PhysicsWarpRate(ctx.ParamDouble("physWarp", 2));

            ctx.Step("Entry");
            var rows = new System.Text.StringBuilder("t,altitude,srfSpeed,mach,dynPreskPa,aoa,part,persistentId,skinTemp,temp,convFluxKW,skinExposedArea,exposedArea,radiativeArea,farArea\n");
            var peakSkin = new Dictionary<string, double>(); var peakInt = new Dictionary<string, double>(); var partArea = new Dictionary<string, double>();
            double t0 = Planetarium.GetUniversalTime(), nextSample = 0, vesselPeakSkin = 0; int samples = 0;
            double holdBand = ctx.ParamDouble("holdBandDeg", 3), holdMaxOutside = ctx.ParamDouble("holdMaxOutside", 0.05);
            int holdSamples = 0, holdOutside = 0; double aoaSum = 0, aoaSq = 0, aoaMin = double.PositiveInfinity, aoaMax = double.NegativeInfinity;
            yield return Wait.Until(() =>
            {
                var av = FlightGlobals.ActiveVessel;
                if (av == null || av.state == Vessel.State.DEAD) return true;
                double t = Planetarium.GetUniversalTime() - t0;
                if (av.altitude < startAlt && t >= nextSample)
                {
                    nextSample = t + sampleSec; ++samples;
                    double a = Flight.AoA;
                    if (av.altitude < 60000 && !double.IsNaN(a))
                    {
                        ++holdSamples; aoaSum += a; aoaSq += a * a; aoaMin = Math.Min(aoaMin, a); aoaMax = Math.Max(aoaMax, a);
                        if (Math.Abs(a - aoa) > holdBand) ++holdOutside;
                    }
                    string head = string.Join(",", new[] { t.ToString("F1", inv), av.altitude.ToString("F0", inv), av.srfSpeed.ToString("F1", inv), av.mach.ToString("F3", inv), av.dynamicPressurekPa.ToString("F3", inv), Flight.AoA.ToString("F2", inv) });
                    foreach (Part p in av.parts)
                    {
                        string key = (p.partInfo != null ? p.partInfo.name : p.name) + "#" + p.persistentId;
                        double fa = FarApi.PartArea(p);
                        rows.Append(head).Append(',').Append(p.partInfo != null ? p.partInfo.name : p.name).Append(',').Append(p.persistentId).Append(',')
                            .Append(p.skinTemperature.ToString("F1", inv)).Append(',').Append(p.temperature.ToString("F1", inv)).Append(',')
                            .Append((p.thermalConvectionFlux).ToString("F3", inv)).Append(',').Append(p.skinExposedArea.ToString("F3", inv)).Append(',')
                            .Append(p.exposedArea.ToString("F3", inv)).Append(',').Append(p.radiativeArea.ToString("F3", inv)).Append(',').Append(fa.ToString("F3", inv)).Append('\n');
                        if (!peakSkin.ContainsKey(key) || p.skinTemperature > peakSkin[key]) peakSkin[key] = p.skinTemperature;
                        if (!peakInt.ContainsKey(key) || p.temperature > peakInt[key]) peakInt[key] = p.temperature;
                        if (!partArea.ContainsKey(key)) partArea[key] = fa;
                        if (p.skinTemperature > vesselPeakSkin) vesselPeakSkin = p.skinTemperature;
                    }
                }
                return av.altitude < stopAlt || (av.altitude < startAlt && av.srfSpeed < stopSpeed);
            }, 1500, "entry to the stop altitude or speed", shot: false);
            Flight.WarpOff();
            File.WriteAllText(Path.Combine(ctx.ResultsDir, "heating-" + (gearDown ? "gearDown" : "gearUp") + ".csv"), rows.ToString());
            ctx.Record("samples", samples);
            ctx.Record("vesselPeakSkinTemp", Math.Round(vesselPeakSkin, 1));
            ctx.Record("alive", !Flight.Dead);
            bool holdOk = holdSamples > 0 && holdOutside <= holdMaxOutside * holdSamples;
            if (holdSamples > 0)
            {
                double mean = aoaSum / holdSamples, sd = Math.Sqrt(Math.Max(0, aoaSq / holdSamples - mean * mean));
                ctx.Record("aoaMean", Math.Round(mean, 2)); ctx.Record("aoaSd", Math.Round(sd, 2)); ctx.Record("aoaMin", Math.Round(aoaMin, 1)); ctx.Record("aoaMax", Math.Round(aoaMax, 1));
                ctx.Record("aoaOutsideBandFraction", Math.Round((double)holdOutside / holdSamples, 3));
            }
            ctx.Record("holdOk", holdOk);
            foreach (var kv in peakSkin) { ctx.Record(kv.Key + ".peakSkin", Math.Round(kv.Value, 1)); ctx.Record(kv.Key + ".peakInt", Math.Round(peakInt[kv.Key], 1)); ctx.Record(kv.Key + ".farArea", Math.Round(partArea[kv.Key], 3)); }
            try { Mj.SmartAssOff(); } catch (UITestFailure) { }
            Assert.True(holdOk, $"attitude hold stayed within {holdBand} deg of {aoa} deg AoA below 60 km ({holdOutside} of {holdSamples} samples outside, min {aoaMin:F1} max {aoaMax:F1})");
        }
    }
}
