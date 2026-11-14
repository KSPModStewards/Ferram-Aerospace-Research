using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using FerramAerospaceResearch.FARAeroComponents;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// What FAR's water model (PartHydrodynamics) costs on a large craft: a ditching, run once per configuration from the
    /// same start (FAR's water model off, then on with each mesh source: FAR's own, the parts' colliders, their visible
    /// meshes).
    /// </summary>
    public static class WaterBenchmarkTests
    {
        /// <summary>
        /// FAR SkyEye AS (155 parts) ditched at 60 m/s, 2 m/s down (crash damage off, joints unbreakable), per
        /// configuration (parameter configs, + separated: off, far, colliders, visual):
        /// - the splash: the first impactSec (default 10) game seconds in the water at real time: the water model's own time
        ///   per physics step (clipping, contact faces, strips, mesh builds), the frame time, the wet parts and their
        ///   triangles;
        /// - afloat: TimeControl hyper-warp at hyper (default 50) for wallSec (default 15) wall seconds once it has
        ///   settled 10 s: game seconds per wall second, the whole game's throughput (stock buoyancy and PhysX included),
        ///   and the water model's time per step there.
        /// Writes water-bench.csv (a row a configuration). Parameters: configs, impactSec, hyper, wallSec.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = "TimeControl", StartsAt = StartState.Flight, Craft = "FAR SkyEye AS", CraftDir = "SPH",
                LaunchSite = "Runway", TimeoutSec = 2400)]
        public static IEnumerator Bench_WaterSplashdown(TestContext ctx)
        {
            string[] configs = ctx.ParamList("configs", "off+far+colliders+visual").ToArray();
            double impactSec = ctx.ParamDouble("impactSec", 10), hyper = ctx.ParamDouble("hyper", 50), wallSec = ctx.ParamDouble("wallSec", 15);
            yield return Wait.Seconds(5);
            bool hydroWas = FARDebugValues.hydrodynamics;
            PartHydrodynamics.MeshSourceKind sourceWas = PartHydrodynamics.MeshSource;
            bool crashWas = CheatOptions.NoCrashDamage, jointsWas = CheatOptions.UnbreakableJoints;
            var csv = new StringBuilder("config,parts,wetParts,wetTriangles,impact_steps,impact_hydro_ms_step,impact_gather_ms_step,impact_solve_ms_step," +
                                        "contact_ms_total,contact_pairs,mesh_ms_total,impact_frame_ms,impact_polys_step,float_game_s_per_wall_s,float_hydro_ms_step," +
                                        "float_frame_ms,float_steps\n");
            double tick = 1000.0 / Stopwatch.Frequency;
            try
            {
                CheatOptions.NoCrashDamage = true;
                CheatOptions.UnbreakableJoints = true;
                foreach (string config in configs)
                {
                    ctx.Step(config);
                    FARDebugValues.hydrodynamics = config != "off";
                    if (config == "colliders") PartHydrodynamics.MeshSource = PartHydrodynamics.MeshSourceKind.Colliders;
                    else if (config == "visual") PartHydrodynamics.MeshSource = PartHydrodynamics.MeshSourceKind.Visual;
                    else PartHydrodynamics.MeshSource = PartHydrodynamics.MeshSourceKind.Far;

                    Flight.HyperWarpOff();
                    yield return FlightTests.Airborne(Far.At(0, 600, 2, "glide", speedMs: 60));
                    Vessel v = FlightGlobals.ActiveVessel;
                    Assert.True(v.mainBody.ocean, "over the sea");
                    v.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                    v.SetPosition(v.vesselTransform.position - (Vector3d)v.upAxis * (v.altitude - 1));
                    v.ChangeWorldVelocity((Vector3d)v.upAxis * (-v.verticalSpeed - 2));
                    yield return Wait.Until(() => v.parts.Any(p => p.submergedPortion > 0), 15, "in the water", shot: false);

                    // the splash, at real time
                    PartHydrodynamics.ResetProfile();
                    var fixedWait = new WaitForFixedUpdate();
                    double t0 = Planetarium.GetUniversalTime();
                    int frames0 = Time.frameCount;
                    float wall0 = Time.realtimeSinceStartup;
                    int maxWet = 0, steps = 0;
                    while (Planetarium.GetUniversalTime() - t0 < impactSec)
                    {
                        yield return fixedWait;
                        steps++;
                        maxWet = Math.Max(maxWet, v.parts.Count(p => p.submergedPortion > 0));
                    }
                    double impactFrameMs = (Time.realtimeSinceStartup - wall0) * 1000 / Math.Max(1, Time.frameCount - frames0);
                    long hSteps = Math.Max(1, PartHydrodynamics.ProfileSteps);
                    double gatherMs = PartHydrodynamics.ProfileGatherTicks * tick, solveMs = PartHydrodynamics.ProfileSolveTicks * tick;
                    double contactMs = PartHydrodynamics.ProfileContactTicks * tick, meshMs = PartHydrodynamics.ProfileMeshTicks * tick;
                    long pairs = PartHydrodynamics.ProfileContactPairs, polys = PartHydrodynamics.ProfilePolygons;
                    // the triangles the water model reads on the wet parts
                    int wetTris = config == "off" ? 0 : v.parts.Where(p => p.submergedPortion > 0)
                                   .Sum(p => p.Modules.GetModule<FARAeroPartModule>()?.Hydrodynamics?.MeshTriangles ?? 0);

                    // afloat: settle, then the throughput at hyper-warp
                    yield return Wait.GameSeconds(10);
                    PartHydrodynamics.ResetProfile();
                    bool warped = Flight.HyperWarp(hyper, 1.0);
                    yield return Wait.Seconds(1);
                    double g0 = Planetarium.GetUniversalTime();
                    wall0 = Time.realtimeSinceStartup;
                    frames0 = Time.frameCount;
                    long s0 = PartHydrodynamics.ProfileSteps, h0 = PartHydrodynamics.ProfileGatherTicks + PartHydrodynamics.ProfileSolveTicks;
                    int fSteps = 0;
                    while (Time.realtimeSinceStartup - wall0 < wallSec)
                    {
                        yield return fixedWait;
                        fSteps++;
                    }
                    double wall = Time.realtimeSinceStartup - wall0, game = Planetarium.GetUniversalTime() - g0;
                    double floatFrameMs = wall * 1000 / Math.Max(1, Time.frameCount - frames0);
                    long fh = PartHydrodynamics.ProfileSteps - s0;
                    double floatHydroMs = fh > 0 ? (PartHydrodynamics.ProfileGatherTicks + PartHydrodynamics.ProfileSolveTicks - h0) * tick / fh : 0;
                    Flight.HyperWarpOff();

                    csv.AppendLine(string.Join(",", config, v.parts.Count.ToString(), maxWet.ToString(), wetTris.ToString(), steps.ToString(),
                                               F((gatherMs + solveMs) / hSteps), F(gatherMs / hSteps), F(solveMs / hSteps), F(contactMs), pairs.ToString(), F(meshMs),
                                               F(impactFrameMs), F((double)polys / hSteps), F(game / wall), F(floatHydroMs), F(floatFrameMs), fSteps.ToString()));
                    ctx.Record(config, $"{v.parts.Count} parts, up to {maxWet} wet ({wetTris} triangles in the water model's meshes now wet); splash {impactSec:F0} s: " +
                                       $"water model {(gatherMs + solveMs) / hSteps:F3} ms a step (clip {gatherMs / hSteps:F3}, strips {solveMs / hSteps:F3}; " +
                                       $"contacts {contactMs:F1} ms over {pairs} pairs, mesh builds {meshMs:F1} ms, once), frame {impactFrameMs:F1} ms; " +
                                       $"afloat at hyper {hyper:F0} ({(warped ? "on" : "refused")}): {game / wall:F2} game s per wall s, water model {floatHydroMs:F3} ms a step, " +
                                       $"frame {floatFrameMs:F1} ms");
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, "water-bench.csv"), csv.ToString());
            }
            finally
            {
                Flight.HyperWarpOff();
                FARDebugValues.hydrodynamics = hydroWas;
                PartHydrodynamics.MeshSource = sourceWas;
                CheatOptions.NoCrashDamage = crashWas;
                CheatOptions.UnbreakableJoints = jointsWas;
            }
        }

        private static string F(double x) => x.ToString("G5", CultureInfo.InvariantCulture);
    }
}
