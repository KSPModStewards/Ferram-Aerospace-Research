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
        [UITest(Kind = "Measure", Explicit = true, Mods = @"..\..\..\CustomProceduralParts\GameData\CustomProceduralParts",
                StartsAt = StartState.Flight, Craft = Naca, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Tow_Naca11A(TestContext ctx) => TowNaca(ctx, 2.0);

        /// <summary>
        /// As Tow_Naca11A, the hull at the tank model's own size (NACA 11-A Model.craft, 17 in beam): the same Froude
        /// numbers at the tank's Reynolds number, so its skin friction is the tank's.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = @"..\..\..\CustomProceduralParts\GameData\CustomProceduralParts",
                StartsAt = StartState.Flight, Craft = NacaModel, CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Tow_Naca11AModel(TestContext ctx) => TowNaca(ctx, 17 * 0.0254);

        /// <summary>
        /// As Tow_Naca11A, the hull split into three parts (crafts/split_hull.py: cut at station 4 and at the step's
        /// forward station, so the step's face is the afterbody's): FAR's water should not tell it from the one-part hull.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = @"..\..\..\CustomProceduralParts\GameData\CustomProceduralParts",
                StartsAt = StartState.Flight, Craft = Naca + " Split", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Tow_Naca11ASplit(TestContext ctx) => TowNaca(ctx, 2.0);

        /// <summary>As Tow_Naca11AModel, the model split as Tow_Naca11ASplit's hull is.</summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = @"..\..\..\CustomProceduralParts\GameData\CustomProceduralParts",
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
            double[] trims = ctx.ParamList("trims", "3+5+7").Select(Parse).ToArray();
            double[] cvs = ctx.ParamList("cvs", "1.0+1.25+1.5+1.75+2.0+2.25+2.5+2.75+3.0+3.5+4.0+4.5+5.0+5.5+6.0+6.5").Select(Parse).ToArray();
            double[] restDrafts = ctx.ParamList("restDrafts", "0.457+0.465+0.460").Select(s => Parse(s) * size).ToArray();
            bool rigid = ctx.Param("rigid") != "false";      // a hull of several parts put back in its built shape every step
            double bendDeg = 0, bendM = 0;
            bool hydroWas = Settings.FARSettings.Hydrodynamics;
            string hydro = ctx.Param("hydro");
            if (!string.IsNullOrEmpty(hydro))
                Settings.FARSettings.Hydrodynamics = bool.Parse(hydro);
            double pbWas = FARAeroComponents.PartHydrodynamics.PlaningBuoyancy;
            WaterParams.Apply(ctx);
            FARAeroComponents.PartHydrodynamics.PlaningBuoyancy = ctx.ParamDouble("planingBuoyancy", pbWas);
            ctx.Record("model", Settings.FARSettings.Hydrodynamics
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

                var rows = new StringBuilder("trim_deg,C_V,V_ms,R_kN,R_sd,air_kN,C_R,R_over_Delta,draft_step_m,draft_step_in,moment_kNm,C_M,moment_lbft_model," +
                                             "hydroUp_kN,buoyUp_kN,wetLength_m,limiter\n");
                var fixedWait = new WaitForFixedUpdate();
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
                            bool owns = Settings.FARSettings.Hydrodynamics && hull.OwnsBuoyancy;
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
                            ctx.Record($"trim {tau} C_V {cv}", $"R/Delta {R / delta:F3} (C_R {R / (delta / load):F4}), draft at the step {dS / Inch:F2} in model, " +
                                                              $"moment {mModel:F1} lb ft model, hydro up {hUp.Average():F1} kN, buoyancy {bUp.Average():F1}");
                    }
                }
                ctx.Record("bend", $"the joints bent the hull up to {bendDeg:F3} deg, {bendM * 1000:F1} mm " +
                                   (rigid ? "in a step (put back every step)" : "(left free)"));
                File.WriteAllText(Path.Combine(ctx.ResultsDir, $"naca-{v.vesselName}.csv"), rows.ToString());
                File.WriteAllText(Path.Combine(ctx.ResultsDir, $"naca-strips-{v.vesselName}.csv"), stripCsv.ToString());
            }
            finally
            {
                FARAeroComponents.PartHydrodynamics.RecordStrips = false;
                CheatOptions.NoCrashDamage = crashWas;
                Settings.FARSettings.Hydrodynamics = hydroWas;
                FARAeroComponents.PartHydrodynamics.PlaningBuoyancy = pbWas;
                WaterParams.Restore();
            }
        }

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
