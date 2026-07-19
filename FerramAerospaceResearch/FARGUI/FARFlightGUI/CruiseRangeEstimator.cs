/*
Ferram Aerospace Research
=========================
Live cruise range from the measured fuel burn rate over a rolling window.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Collections.Generic;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FARFlightGUI
{
    /// <summary>
    /// Projects still-air range from the fuel burn rate actually measured over the last few seconds,
    /// rather than the instantaneous estimate FAR shows as "Range". Ported from RP-1's SustainedCruise
    /// contract parameter: every mass-bearing resource the vessel is burning is sampled on a rolling
    /// window, its burn rate averaged over that window, and the Breguet range of whichever fuel runs
    /// out first is reported. Sampling runs continuously, so the estimate converges within one window
    /// once the burn rate settles — including while climbing or accelerating into cruise. Also exposes
    /// the aggregate burn rate and remaining mass across the burning resources.
    /// </summary>
    internal class CruiseRangeEstimator
    {
        private const double RateWindowSeconds = 5.0;
        private const float UpdateInterval = 0.5f;
        private const double MinRate = 1e-4; // below this the craft isn't really burning this resource

        private float _lastUpdate = -1f;
        private double _lastCheckTime = double.NegativeInfinity;
        private int _lastPartCount = -1;
        private uint _lastVesselId;

        // sampleTimes[k] is the time of tick k; sampleAmounts[k] maps resource id -> vessel-wide total then.
        private readonly List<double> _sampleTimes = new List<double>();
        private readonly List<Dictionary<int, double>> _sampleAmounts = new List<Dictionary<int, double>>();
        private readonly HashSet<int> _massBearingIds = new HashSet<int>();

        /// <summary>Latest projected range, metres. Zero until the window fills and a fuel is burning.</summary>
        public double RangeMeters { get; private set; }

        /// <summary>Aggregate mass burn rate across the burning resources, kg/s.</summary>
        public double TotalBurnKgPerSec { get; private set; }

        /// <summary>Remaining mass of the burning resources, kg.</summary>
        public double TotalFuelKg { get; private set; }

        public void Update(Vessel v)
        {
            if (v == null)
            {
                Reset();
                return;
            }

            if (Time.fixedTime - _lastUpdate < UpdateInterval)
                return;
            _lastUpdate = Time.fixedTime;

            double now = Time.fixedTime;
            double dt = now - _lastCheckTime;
            bool gap = dt > UpdateInterval * 4.0 || dt < 0; // pause / warp / first tick / vessel swap
            if (v.persistentId != _lastVesselId || v.parts.Count != _lastPartCount || gap)
            {
                _lastVesselId = v.persistentId;
                _lastPartCount = v.parts.Count;
                Reset();
            }

            _lastCheckTime = now;

            _sampleTimes.Add(now);
            _sampleAmounts.Add(SampleResources(v));
            while (_sampleTimes.Count > 2 && now - _sampleTimes[1] >= RateWindowSeconds)
            {
                _sampleTimes.RemoveAt(0);
                _sampleAmounts.RemoveAt(0);
            }

            ComputeMetrics(v, v.srfSpeed);
        }

        private void Reset()
        {
            _sampleTimes.Clear();
            _sampleAmounts.Clear();
            RangeMeters = 0.0;
            TotalBurnKgPerSec = 0.0;
            TotalFuelKg = 0.0;
        }

        // Over every resource present for the whole window and actually being consumed: the smallest
        // projected range (the endurance-limiting fuel), plus the aggregate mass burn rate and
        // remaining mass. Leaves all three at zero when nothing qualifies.
        private void ComputeMetrics(Vessel v, double speed)
        {
            RangeMeters = 0.0;
            TotalBurnKgPerSec = 0.0;
            TotalFuelKg = 0.0;

            int n = _sampleTimes.Count;
            if (n < 2)
                return;

            double span = _sampleTimes[n - 1] - _sampleTimes[0];
            if (span < RateWindowSeconds * 0.9)
                return;

            Dictionary<int, double> oldest = _sampleAmounts[0];
            Dictionary<int, double> newest = _sampleAmounts[n - 1];
            bool any = false;
            double best = double.MaxValue;
            double totalRateMassT = 0.0; // t/s
            double totalRemMassT = 0.0;  // t

            foreach (KeyValuePair<int, double> kv in newest)
            {
                // Only rank resources present for the whole window, else a tank coming online mid-window
                // reads as an impossibly high burn rate.
                if (!oldest.TryGetValue(kv.Key, out double old))
                    continue;

                double rate = (old - kv.Value) / span; // units/s
                if (rate < MinRate)
                    continue;

                PartResourceDefinition def = PartResourceLibrary.Instance.GetDefinition(kv.Key);
                double dens = def != null ? def.density : 0.0;

                double range = ProjectedRange(v, speed, kv.Value, rate, dens);
                if (range < best)
                    best = range;

                totalRateMassT += rate * dens;
                totalRemMassT += kv.Value * dens;
                any = true;
            }

            if (!any)
                return;

            RangeMeters = best;
            TotalBurnKgPerSec = totalRateMassT * 1000.0;
            TotalFuelKg = totalRemMassT * 1000.0;
        }

        // Breguet range from now on a single resource. Accounts for the craft lightening as fuel burns
        // (burn rate falls with weight in level cruise, so real range exceeds a naive fuel/rate*speed
        // extrapolation): range = speed * (M / mdot) * ln(M / Mempty). Falls back to the linear form
        // when mass/density data is unavailable.
        private static double ProjectedRange(Vessel v, double speed, double currentUnits, double rateUnits, double dens)
        {
            double M = v.totalMass;              // current total mass (t)
            double mFuel = currentUnits * dens;  // remaining mass of this resource (t)
            double mEmpty = M - mFuel;           // mass when it runs out (t)
            double massRate = rateUnits * dens;  // burn rate (t/s)
            if (dens > 0.0 && massRate > 0.0 && mFuel > 1e-6 && mEmpty > 1e-3)
                return speed * (M / massRate) * Math.Log(M / mEmpty);
            return speed * (currentUnits / rateUnits); // linear fallback
        }

        // Vessel-wide total of every mass-bearing resource, keyed by id. Massless resources
        // (ElectricCharge, IntakeAir) are skipped so they can't masquerade as a fuel.
        private Dictionary<int, double> SampleResources(Vessel v)
        {
            _massBearingIds.Clear();
            for (int i = 0; i < v.parts.Count; i++)
            {
                PartResourceList prl = v.parts[i].Resources;
                for (int j = 0; j < prl.Count; j++)
                {
                    PartResource pr = prl[j];
                    if (pr.info.density > 0.0)
                        _massBearingIds.Add(pr.info.id);
                }
            }

            var snapshot = new Dictionary<int, double>(_massBearingIds.Count);
            foreach (int id in _massBearingIds)
            {
                v.GetConnectedResourceTotals(id, out double amount, out double _);
                snapshot[id] = amount;
            }

            return snapshot;
        }
    }
}
