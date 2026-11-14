using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The NACA Model 11-A flying-boat hull towed as NACA towed it (TN 470, Parkinson 1933): held at a fixed trim with
    /// a fixed load on the water, free to rise and sink, at constant speeds. TN 470's Table II gives the resistance,
    /// the draft at the step and the trimming moment for each load, trim and speed.
    /// </summary>
    public static class NacaHullTests
    {
        private const string Naca = "NACA 11-A", NacaModel = "NACA 11-A Model";

        /// <summary>
        /// NACA 11-A.craft (crafts/naca_11a.py: one CustomProceduralParts Spine lofted from Table I, beam 2 m) with its
        /// CG put at TN 470's centre of moments. For each trim: the hull is held at that trim (its base line's angle to
        /// the water) and an upward force through the CG leaves the load asked for on the water (NACA's counterweights);
        /// it heaves freely. First at rest (8 s settling, 4 s averaged: the step's draft and the mesh volume against the
        /// hand calculation), then towed at each speed (6 s settling, 4 s averaged): resistance (the tow line's force),
        /// the air's share (FAR's API on the same attitude in air), draft at the step, and the trimming moment about the
        /// centre of moments (bow up positive, from the pitch rate the hold takes out each step).
        /// Writes naca-{vessel}.csv. Parameters: load (C_Delta, default 0.221 = TN 470's 40 lb), trims (deg, + separated),
        /// cvs (C_V, + separated), restDrafts (hand step drafts at rest for those trims at the 2 m beam, m, + separated).
        /// Explicit: CustomProceduralParts comes from its checkout beside this repository's.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.WaterMods,
                StartsAt = StartState.Flight, Craft = Naca, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Tow_Naca11A(TestContext ctx) => TowNaca(ctx, 2.0);

        /// <summary>
        /// As Tow_Naca11A, the hull at the tank model's own size (NACA 11-A Model.craft, 17 in beam): the same Froude
        /// numbers at the tank's Reynolds number, so its skin friction is the tank's.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.WaterMods,
                StartsAt = StartState.Flight, Craft = NacaModel, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Tow_Naca11AModel(TestContext ctx) => TowNaca(ctx, 17 * 0.0254);

        /// <summary>
        /// As Tow_Naca11A, the hull split into three parts (crafts/split_hull.py: cut at station 4 and at the step's
        /// forward station, so the step's face is the afterbody's): FAR's water should not tell it from the one-part hull.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.WaterMods,
                StartsAt = StartState.Flight, Craft = Naca + " Split", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Tow_Naca11ASplit(TestContext ctx) => TowNaca(ctx, 2.0);

        /// <summary>As Tow_Naca11AModel, the model split as Tow_Naca11ASplit's hull is.</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = TestWarp.WaterMods,
                StartsAt = StartState.Flight, Craft = NacaModel + " Split", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Tow_Naca11AModelSplit(TestContext ctx) => TowNaca(ctx, 17 * 0.0254);

        private static IEnumerator TowNaca(TestContext ctx, double beam)
        {
            // crafts/naca_11a.py: the model's inch at this beam; the step at the spine's middle, the base line its axis
            double Beam = beam, Inch = beam / 17.0, size = beam / 2.0;
            double KeelAtStep = 14.0 * Inch;                                        // below the base line
            double MomentCentreFwd = 6.18 * Inch, MomentCentreUp = 6.31 * Inch;     // from the step, the base line
            double handVolume = 20.91 * size * size * size;
            double load = ctx.ParamDouble("load", 0.221);
            double[] trims = ctx.ParamList("trims", "3+5+7+9").Select(Parse).ToArray();
            double[] cvs = ctx.ParamList("cvs", "1.0+1.25+1.5+1.75+2.0+2.25+2.5+2.75+3.0+3.5+4.0+4.5+5.0+5.5+6.0+6.5").Select(Parse).ToArray();
            double[] restDrafts = ctx.ParamList("restDrafts", "0.457+0.465+0.460").Select(s => Parse(s) * size).ToArray();
            bool rigid = ctx.Param("rigid") != "false";      // a hull of several parts put back in its built shape every step
            double bendDeg = 0, bendM = 0;
            bool hydroWas = FARDebugValues.hydrodynamics;
            string hydro = ctx.Param("hydro");
            if (!string.IsNullOrEmpty(hydro))
                FARDebugValues.hydrodynamics = bool.Parse(hydro);
            double pbWas = FARAeroComponents.PartHydrodynamics.PlaningBuoyancy;
            WaterParams.Apply(ctx);
            // (as WaterParams left it: hd.PlaningBuoyancy given, that)
            FARAeroComponents.PartHydrodynamics.PlaningBuoyancy = ctx.ParamDouble("planingBuoyancy", FARAeroComponents.PartHydrodynamics.PlaningBuoyancy);
            ctx.Record("model", FARDebugValues.hydrodynamics
                                    ? $"PartHydrodynamics (planing buoyancy x{FARAeroComponents.PartHydrodynamics.PlaningBuoyancy})"
                                    : "legacy water share of the aero force");

            yield return Wait.Seconds(5);
            Vessel v = FlightGlobals.ActiveVessel;
            HullParts hull = HullParts.Whole(v);
            ctx.Record("parts", $"{hull.Parts.Length}: " + string.Join(", ", hull.Parts.Select(q => $"{q.partInfo.name} at {hull.InverseTransformPoint(q.transform.position).y:F3}")) +
                                $", {hull.Length:F3} m long");
            foreach (PartResource r in hull.Parts.SelectMany(q => q.Resources.Cast<PartResource>()))
                r.amount = 0;
            yield return Wait.Frames(2);

            bool crashWas = CheatOptions.NoCrashDamage;
            try
            {
                CheatOptions.NoCrashDamage = true;
                v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                yield return FarFlightProbe.Place(Far.At(0, 600, 0, "naca", speedMs: 5));
                v = FlightGlobals.ActiveVessel;
                CelestialBody body = v.mainBody;
                hull = HullParts.Whole(v);
                Rigidbody rb = hull.Ref.rb;
                Vector3 up = v.upAxis;
                Vector3 h = Vector3.ProjectOnPlane(hull.Forward, up).normalized;
                Vector3 bowUp = Vector3.Cross(h, up).normalized;     // rotation about it raises the bow
                Vector3 keel = PlaningVideoTests.KeelAxis(hull.Ref);
                Assert.True(Mathf.Abs(keel.z) > 0.5f, $"the keel is along the part's z (sections' -y): {keel}");
                DumpObj(hull, Path.Combine(ctx.ResultsDir, $"collider-{v.vesselName}.obj"), true);
                DumpObj(hull, Path.Combine(ctx.ResultsDir, $"visual-{v.vesselName}.obj"), false);

                double rho = body.oceanDensity * 1000;
                double sqrtGb = Math.Sqrt(9.80665 * body.GeeASL * Beam);
                double delta = load * rho * 9.80665 * body.GeeASL * Beam * Beam * Beam / 1000;   // kN
                double mass = v.GetTotalMass();
                ctx.Record("hull", $"mass {mass:F3} t, load on the water {delta:F2} kN (C_Delta {load}), keel along local {keel}");

                // CG at the centre of moments: forward of the step (spine middle) and above the base line (spine axis)
                hull.SetCoM(v, new Vector3(0, (float)MomentCentreFwd, 0) - keel * (float)MomentCentreUp);
                Vector3 stepKeel = keel * (float)KeelAtStep;

                // each trim's runs (C_V, C_R, step draft in model inches, trimming moment in model lb ft), for TN 470
                var runs = new Dictionary<double, List<(double cv, double cr, double draft, double moment)>>();
                var rows = new StringBuilder("trim_deg,C_V,V_ms,R_kN,R_sd,air_kN,C_R,R_over_Delta,draft_step_m,draft_step_in,moment_kNm,C_M,moment_lbft_model," +
                                             "hydroUp_kN,buoyUp_kN,wetLength_m,limiter\n");
                var fixedWait = new WaitForFixedUpdate();
                // hyper-warped (parameter hyperWarp; the hull held every step: the same run, sooner)
                TestWarp.Hyper(ctx, true);
                var stripCsv = new StringBuilder("trim_deg,C_V,x_m,width_m,ma_kg_m,v_ms,jump_N,load_N,separated,y_part_m,z_part_m,momUp_kN,crossUp_kN,hydrostaticUp_kN,friction_kN,impact_kN\n");
                FARAeroComponents.PartHydrodynamics.RecordStrips = true;
                // the hull's attitude at a trim, against the local vertical where it is now (it runs kilometres)
                Quaternion Hold(double tau)
                {
                    up = (Vector3)FlightGlobals.getUpAxis(body, RigidVessel.CoM(v));
                    h = Vector3.ProjectOnPlane(h, up).normalized;
                    bowUp = Vector3.Cross(h, up).normalized;
                    Vector3 fwd = Quaternion.AngleAxis((float)tau, bowUp) * h;
                    Vector3 down = Vector3.Cross(fwd, bowUp).normalized;          // perpendicular to fwd, below it
                    return Quaternion.LookRotation(keel.z > 0 ? down : -down, fwd);
                }

                for (int ti = 0; ti < trims.Length; ti++)
                {
                    double tau = trims[ti];
                    v.SetRotation(Hold(tau));
                    yield return new WaitForFixedUpdate();
                    double stepAlt = FlightGlobals.getAltitudeAtPos(hull.TransformPoint(stepKeel), body);
                    v.SetPosition(v.vesselTransform.position + (Vector3d)up * (-0.45 * size - stepAlt));
                    RigidVessel.Set(v, Vector3.zero, Vector3.zero);

                    double setAlong = 0;
                    var phases = new List<double> { 0 };
                    phases.AddRange(cvs);
                    foreach (double cv in phases)
                    {
                        double V = cv * sqrtGb;
                        ctx.Step(cv == 0 ? $"trim {tau}: at rest" : $"trim {tau}: C_V {cv}");
                        double settle = cv == 0 ? 8 : 6, t0 = Planetarium.GetUniversalTime();
                        var tow = new List<double>(); var air = new List<double>(); var draft = new List<double>(); var mom = new List<double>();
                        var hUp = new List<double>(); var bUp = new List<double>(); var wetL = new List<double>(); var lim = new List<double>();
                        int n = 0;
                        while (Planetarium.GetUniversalTime() - t0 < settle + 4)
                        {
                            // after the physics step: the tow line's force, the hold's moment; then speed, trim and the
                            // counterweight's share of the weight for the next step
                            double dt = TimeWarp.fixedDeltaTime;
                            Vector3 vel = RigidVessel.Velocity(v);
                            Vector3 bowUpWas = bowUp, hWas = h;
                            Quaternion hold = Hold(tau);
                            double towKN = mass * (setAlong - Vector3.Dot(vel, hWas)) / dt;
                            Vector3 torque = RigidVessel.AngularMomentum(v) / (float)dt;   // kN m, what the water put in
                            double momentKNm = Vector3.Dot(torque, bowUpWas);

                            Vector3d com = RigidVessel.CoM(v);
                            double g = -Vector3d.Dot(FlightGlobals.getGeeForceAtPosition(com) + FlightGlobals.getCentrifugalAcc(com, body), (Vector3d)up);
                            double lift = mass * g - delta;                  // the counterweight, kN
                            double vUp = Vector3.Dot(vel, up) + lift / mass * dt;
                            (double d, double m) bend = RigidVessel.Bend(v, hull.Ref);
                            bendDeg = Math.Max(bendDeg, bend.d);
                            bendM = Math.Max(bendM, bend.m);
                            if (rigid)
                                RigidVessel.Snap(v, hull.Ref);
                            RigidVessel.Rotate(v, hull.Ref, hold);
                            RigidVessel.Set(v, up * (float)vUp + h * (float)V, Vector3.zero);
                            setAlong = V;

                            double t = Planetarium.GetUniversalTime() - t0;
                            if (t >= settle)
                            {
                                double d = -FlightGlobals.getAltitudeAtPos(hull.TransformPoint(stepKeel), body);
                                tow.Add(towKN); draft.Add(d); mom.Add(momentKNm);
                                hUp.Add(Vector3.Dot(hull.Sum(x => x.LastForce), up));
                                bUp.Add(Vector3.Dot(hull.StockBuoyancy + hull.Sum(x => x.LastHydrostaticForce), up));
                                wetL.Add(hull.WettedLength); lim.Add(hull.Limiter);
                                if (n % 10 == 0 && V > 0)
                                {
                                    FARAPI.CalculateVesselAeroForces(v, out Vector3 f, out _, h * (float)V, Math.Max(0, v.altitude));
                                    air.Add(-Vector3.Dot(f, h));
                                }
                            }
                            n++;
                            yield return fixedWait;
                        }

                        foreach (var st in hull.StripData)
                        {
                            Vector3 lp = hull.InverseTransformPoint(st.point);
                            stripCsv.AppendLine(string.Join(",", F(tau), F(cv), F(st.x), F(st.width), F(st.ma), F(st.v), F(st.jump), F(st.load), st.separated ? "1" : "0",
                                                            F(lp.y), F(lp.z), F(Vector3.Dot(hull.Sum(x => x.LastMomentumForce), up)), F(Vector3.Dot(hull.Sum(x => x.LastCrossflowForce), up)),
                                                            F(Vector3.Dot(hull.Sum(x => x.LastHydrostaticForce), up)), F(-Vector3.Dot(hull.Sum(x => x.LastFrictionForce), h)),
                                                            F(-Vector3.Dot(hull.Sum(x => x.LastImpactForce), h))));
                        }
                        double R = tow.Average(), dS = draft.Average(), M = mom.Average();
                        double cM = M / (rho * 9.80665 * body.GeeASL / 1000 * Math.Pow(Beam, 4));
                        double mModel = cM * 63.6 * Math.Pow(17.0 / 12, 4);
                        rows.AppendLine(string.Join(",", F(tau), F(cv), F(V), F(R), F(Sd(tow)), F(air.Count > 0 ? air.Average() : 0),
                                                    F(R / (delta / load)), F(R / delta), F(dS), F(dS / Inch), F(M), F(cM), F(mModel),
                                                    F(hUp.Average()), F(bUp.Average()), F(wetL.Average()), F(lim.Min())));
                        if (cv == 0)
                        {
                            bool owns = FARDebugValues.hydrodynamics && hull.OwnsBuoyancy;
                            double meshVolume = hull.MeshVolume;
                            double hand = ti < restDrafts.Length ? restDrafts[ti] : double.NaN;
                            ctx.Record($"rest {tau}", $"draft at the step {dS:F3} m (hand {hand:F3}), buoyancy {bUp.Average():F2} kN of {delta:F2}, " +
                                                      $"moment {mModel:F1} lb ft model, " + (owns ? $"mesh volume {meshVolume:F2} m3 (hand {handVolume:F2})" : "stock buoyancy"));
                            if (owns)
                            {
                                Assert.True(Math.Abs(meshVolume / handVolume - 1) < 0.02, $"mesh volume {meshVolume:F2} m3 within 2% of {handVolume:F2}");
                                if (!double.IsNaN(hand) && Math.Abs(load - 0.221) < 1e-6)
                                    Assert.True(Math.Abs(dS - hand) < 0.02 * size, $"rest draft at the step {dS:F4} m within {0.02 * size:F4} of {hand:F4}");
                            }
                        }
                        else
                        {
                            ctx.Record($"trim {tau} C_V {cv}", $"R/Delta {R / delta:F3} (C_R {R / (delta / load):F4}), draft at the step {dS / Inch:F2} in model, " +
                                                              $"moment {mModel:F1} lb ft model, hydro up {hUp.Average():F1} kN, buoyancy {bUp.Average():F1}");
                            if (!runs.TryGetValue(tau, out var list))
                                runs[tau] = list = new List<(double, double, double, double)>();
                            list.Add((cv, R / (delta / load), dS / Inch, mModel));
                        }
                    }
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, $"naca-{v.vesselName}.csv"), rows.ToString());
                File.WriteAllText(Path.Combine(ctx.ResultsDir, $"naca-strips-{v.vesselName}.csv"), stripCsv.ToString());
                // against TN 470's Table II at the 40 lb load (its printed values), each point's own trim, FAR's run
                // interpolated to its C_V: every point's resistance within WideResistance (a share) and step draft
                // within WideDraft (model in), and at least TightShare of them within ResistanceTolerance and
                // DraftTolerance, asserted at the tank model's size (its Reynolds number, its skin friction: a bigger
                // hull's C_R is lower), recorded at any; the trimming moment recorded. (FAR's misses: 5 - 7 deg early
                // in planing, C_V 3.2 - 4.4, ~1.25 - 1.3x and riding ~0.3 - 0.45 in deep; 3 deg above C_V 5 ~0.7x)
                if (Math.Abs(load - 0.221) < 1e-6)
                {
                    bool asserted = Math.Abs(beam - 17 * 0.0254) < 1e-6;
                    var cmp = new StringBuilder();
                    var off = new List<string>();
                    var wideOff = new List<string>();
                    double logSum = 0;
                    int nCmp = 0;
                    foreach (var tn in Tn470Load40)
                    {
                        if (!runs.TryGetValue(tn.trim, out var list) || list.Count < 2)
                            continue;
                        var s = list.OrderBy(x => x.cv).ToList();
                        int k = s.FindIndex(x => x.cv >= tn.cv);
                        if (k <= 0)
                            continue;   // (outside FAR's runs: none extrapolated)
                        var lo = s[k - 1]; var hi = s[k];
                        double f = (tn.cv - lo.cv) / (hi.cv - lo.cv);
                        double cr = lo.cr + f * (hi.cr - lo.cr), dr = lo.draft + f * (hi.draft - lo.draft), mo = lo.moment + f * (hi.moment - lo.moment);
                        double ratio = cr / tn.cr;
                        logSum += Math.Log(ratio);
                        nCmp++;
                        cmp.Append($"{tn.trim:F0} deg C_V {tn.cv:F2}: C_R {cr:F4}/{tn.cr:F4} ({ratio:F2}x), draft {dr:F2}/{tn.draft:F2} in, moment {mo:F1}/{tn.moment:F1} lb ft; ");
                        string what = $"{tn.trim:F0} deg C_V {tn.cv:F2} C_R {ratio:F2}x draft {dr - tn.draft:+0.00;-0.00} in";
                        if (Math.Abs(ratio - 1) > ResistanceTolerance || Math.Abs(dr - tn.draft) > DraftTolerance)
                            off.Add(what);
                        if (Math.Abs(ratio - 1) > WideResistance || Math.Abs(dr - tn.draft) > WideDraft)
                            wideOff.Add(what);
                    }
                    ctx.Record("tn470", cmp.ToString());
                    ctx.Record("tn470 fit", $"{nCmp} points: resistance {Math.Exp(logSum / Math.Max(1, nCmp)):F2}x TN 470's (geometric mean); " +
                                            $"{off.Count} outside {ResistanceTolerance:P0} / {DraftTolerance:F2} in: {string.Join("; ", off)}");
                    if (asserted)
                    {
                        Assert.True(wideOff.Count == 0, $"resistance within {WideResistance:P0} and step draft within {WideDraft:F2} in of TN 470's " +
                                                        $"Table II at every point ({wideOff.Count} of {nCmp} off: {string.Join("; ", wideOff.Take(8))})");
                        double tight = 1 - (double)off.Count / Math.Max(1, nCmp);
                        Assert.True(tight >= TightShare, $"at least {TightShare:P0} of TN 470's points within {ResistanceTolerance:P0} and " +
                                                         $"{DraftTolerance:F2} in ({tight:P0}: {off.Count} of {nCmp} off)");
                    }
                }
                ctx.Record("bend", $"the joints bent the hull up to {bendDeg:F3} deg, {bendM * 1000:F1} mm " +
                                   (rigid ? "in a step (put back every step)" : "(left free)"));
            }
            finally
            {
                TestWarp.Hyper(ctx, false);
                FARAeroComponents.PartHydrodynamics.RecordStrips = false;
                CheatOptions.NoCrashDamage = crashWas;
                FARDebugValues.hydrodynamics = hydroWas;
                FARAeroComponents.PartHydrodynamics.PlaningBuoyancy = pbWas;
                WaterParams.Restore();
            }
        }

        public const double ResistanceTolerance = 0.25, DraftTolerance = 0.35, WideResistance = 0.35, WideDraft = 0.7, TightShare = 0.75;

        /// <summary>
        /// TN 470's Table II at the 40 lb load (C_Delta 0.221), its printed values (PDF p13-15): trim (deg), C_V, C_R
        /// (R / w b^3), the draft at the step (model in) and the trimming moment about the centre of moments (model lb ft,
        /// bow up +). w b^3 180.83 lb, sqrt(g b) 6.754 fps.
        /// </summary>
        private static readonly (double trim, double cv, double cr, double draft, double moment)[] Tn470Load40 =
        {
            (3, 1.022, 0.0171, 4.35, 9.5),
            (3, 1.273, 0.0243, 4.20, 16.4),
            (3, 1.525, 0.0265, 4.10, 18.3),
            (3, 1.658, 0.0315, 4.10, 22.6),
            (3, 1.984, 0.0437, 3.90, 34.8),
            (3, 2.147, 0.0448, 4.00, 41.8),
            (3, 2.339, 0.0481, 3.80, 43.5),
            (3, 2.650, 0.0470, 3.50, 41.8),
            (3, 2.813, 0.0487, 3.40, 41.0),
            (3, 2.902, 0.0448, 3.50, 36.6),
            (3, 3.109, 0.0448, 3.00, 32.3),
            (3, 3.213, 0.0470, 2.80, 31.3),
            (3, 3.479, 0.0481, 2.60, 24.4),
            (3, 3.835, 0.0498, 2.50, 17.5),
            (3, 4.619, 0.0597, 2.10, 11.3),
            (3, 5.449, 0.0658, 1.70, 6.1),
            (3, 6.189, 0.0713, 1.60, 2.5),
            (4, 5.686, 0.0536, 1.50, -1.9),
            (4, 6.367, 0.0614, 1.40, -3.7),
            (5, 1.096, 0.0205, 4.35, -4.4),
            (5, 1.273, 0.0210, 4.20, -1.9),
            (5, 1.495, 0.0232, 4.00, 2.5),
            (5, 1.792, 0.0288, 4.10, 15.7),
            (5, 1.954, 0.0310, 4.00, 22.7),
            (5, 2.176, 0.0337, 3.80, 28.8),
            (5, 2.443, 0.0348, 3.30, 26.1),
            (5, 2.724, 0.0348, 2.80, 19.9),
            (5, 2.932, 0.0359, 2.75, 13.1),
            (5, 3.198, 0.0343, 2.50, 8.7),
            (5, 3.553, 0.0371, 2.20, 6.1),
            (5, 3.953, 0.0393, 2.00, 3.4),
            (5, 4.264, 0.0426, 1.90, 0.8),
            (5, 5.034, 0.0476, 1.50, -2.8),
            (5, 5.212, 0.0476, 1.50, -4.4),
            (5, 5.804, 0.0542, 1.30, -6.3),
            (5, 6.544, 0.0625, 1.30, -8.0),
            (6, 5.848, 0.0575, 1.35, -9.8),
            (6, 6.589, 0.0697, 1.25, -11.5),
            (7, 1.259, 0.0249, 4.05, -14.8),
            (7, 1.466, 0.0271, 3.80, -11.4),
            (7, 1.481, 0.0277, 3.80, -9.8),
            (7, 1.643, 0.0282, 3.65, -2.8),
            (7, 1.940, 0.0348, 3.55, 7.7),
            (7, 2.221, 0.0359, 3.10, 10.4),
            (7, 2.487, 0.0359, 3.15, 8.7),
            (7, 2.724, 0.0365, 2.70, 3.4),
            (7, 2.976, 0.0371, 2.30, 0.9),
            (7, 3.376, 0.0382, 2.05, -0.9),
            (7, 3.776, 0.0404, 1.70, -2.6),
            (7, 4.427, 0.0453, 1.35, -6.9),
            (7, 5.286, 0.0553, 1.40, -11.4),
            (9, 1.673, 0.0343, 3.15, -14.9),
            (9, 1.821, 0.0376, 3.20, -7.9),
            (9, 2.043, 0.0387, 2.90, -3.6),
            (9, 2.339, 0.0409, 2.30, -3.5),
            (9, 2.650, 0.0426, 2.20, -3.6),
            (9, 2.813, 0.0426, 2.30, -3.6),
            (9, 3.124, 0.0448, 1.85, -4.4),
        };

        // the hull's collider mesh in the hull's frame (what FAR's water forces see), for checking against the lines
        private static void DumpObj(HullParts hull, string path, bool collider)
        {
            var sb = new StringBuilder();
            int baseIndex = 1;
            IEnumerable<(Component c, Mesh m)> meshes = collider
                ? hull.Parts.SelectMany(p => p.GetComponentsInChildren<MeshCollider>()).Where(c => c.sharedMesh != null).Select(c => ((Component)c, c.sharedMesh))
                : hull.Parts.SelectMany(p => p.GetComponentsInChildren<MeshFilter>()).Where(f => f.sharedMesh != null && f.sharedMesh.isReadable).Select(f => ((Component)f, f.sharedMesh));
            foreach ((Component mc, Mesh m) in meshes)
            {
                sb.AppendLine($"o {mc.name}");
                foreach (Vector3 w in m.vertices)
                {
                    Vector3 q = hull.InverseTransformPoint(mc.transform.TransformPoint(w));
                    sb.AppendLine($"v {F(q.x)} {F(q.y)} {F(q.z)}");
                }
                int[] t = m.triangles;
                for (int i = 0; i < t.Length; i += 3)
                    sb.AppendLine($"f {t[i] + baseIndex} {t[i + 1] + baseIndex} {t[i + 2] + baseIndex}");
                baseIndex += m.vertexCount;
            }
            File.WriteAllText(path, sb.ToString());
        }

        private static double Parse(string s) => double.Parse(s, CultureInfo.InvariantCulture);

        private static double Sd(List<double> x)
        {
            if (x.Count < 2) return double.NaN;
            double m = x.Average();
            return Math.Sqrt(x.Sum(y => (y - m) * (y - m)) / (x.Count - 1));
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);
    }
}
