using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARPartGeometry;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// A stock asteroid in FAR's voxel model. Its part (PotatoRoid) has no model of its own: ModuleAsteroid builds the
    /// rock's mesh procedurally once the part starts, and FAR finds that mesh through the module's private PAsteroid
    /// (PartGeometryExtensions.ProceduralAsteroidTransforms) and rebuilds the part's geometry once in flight
    /// (StockProcAsteroidGeoUpdater). And a vessel of one part spawned beside the capsule the same way: the inflatable
    /// heat shield, whose model FAR rebuilds as it inflates.
    /// </summary>
    public static class AsteroidTests
    {
        /// <summary>
        /// A small (class A) asteroid spawned 150 m above the Kerbal 1 in a 100 km orbit, on the capsule's own velocity:
        /// it loads, and FAR voxelizes it from its procedural mesh. The asteroid's part has FAR's geometry module with
        /// the asteroid's updater and the rock's meshes, and its voxel model holds about the volume the rock's mesh
        /// encloses: the voxels are at most a few times the volume of a voxel layer over the surface off it.
        /// Regression: before the fix, an asteroid's part has no prefab (part.partInfo.partPrefab is null), and GeometryPartModule.OnStart
        /// reads part.partInfo.partPrefab.FindModuleImplementingFast unguarded: it throws a NullReferenceException, the
        /// module never builds its meshes (never Ready), and FARVesselAero waits for it for ever: FAR never voxelizes an
        /// asteroid, and ProceduralAsteroidTransforms and the asteroid updater never run.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 900)]
        public static IEnumerator Flight_AsteroidIsVoxelized(TestContext ctx)
        {
            yield return Wait.Seconds(3);
            Flight.IgnoreMaxTemperature(true);
            Ctrl.SetThrottle(0f);
            ctx.Step("the capsule in orbit");
            Flight.SetOrbit(100000, 100000);
            yield return Wait.Until(() => FlightGlobals.ActiveVessel != null && !FlightGlobals.ActiveVessel.packed && FlightGlobals.ActiveVessel.altitude > 90000,
                                    60, "the capsule off rails in orbit", shot: false);
            yield return Wait.Seconds(1);
            Vessel v = FlightGlobals.ActiveVessel;

            ctx.Step("an asteroid beside it");
            using var log = new ExceptionLog();
            double ut = Planetarium.GetUniversalTime();
            Vector3d r = v.orbit.getRelativePositionAtUT(ut), vel = v.orbit.getOrbitalVelocityAtUT(ut);
            var o = new Orbit();
            o.UpdateFromStateVectors(r + r.normalized * 150, vel, v.mainBody, ut);
            ProtoVessel pv = DiscoverableObjectsUtil.SpawnAsteroid("FAR test asteroid", o, 20261105u, UntrackedObjectClass.A, 1e7, 1e7);
            Assert.NotNull(pv, "the asteroid spawned");
            Vessel rock = null;
            try
            {
                yield return Wait.Until(() => (rock = pv.vesselRef) != null && rock.loaded && rock.parts.Count > 0, 60, "the asteroid loaded", shot: false,
                                        detail: () => $"vesselRef {pv.vesselRef != null}, loaded {pv.vesselRef?.loaded}, " +
                                                      $"{(pv.vesselRef != null ? (pv.vesselRef.GetWorldPos3D() - v.GetWorldPos3D()).magnitude : -1):F0} m away");
                FARVesselAero aero = rock.GetComponent<FARVesselAero>();
                Assert.NotNull(aero, "FAR's vessel module on the asteroid");
                Part p0 = rock.rootPart;
                var geo0 = p0.FindModuleImplementing<GeometryPartModule>();
                Assert.NotNull(geo0, "FAR's geometry module on the asteroid");
                ctx.Record("started", $"partInfo {p0.partInfo != null}, prefab {p0.partInfo?.partPrefab != null}, " +
                                      $"prefab geometry module {p0.partInfo?.partPrefab?.FindModuleImplementing<GeometryPartModule>() != null}, " +
                                      $"collider {p0.collider != null}, scene set up {Refl.Get(geo0, "_sceneSetup")}, " +
                                      $"modules {string.Join(" ", p0.Modules.Cast<PartModule>().Select(m => m.moduleName))}");
                yield return Wait.Until(() => geo0.Ready, 30, "FAR's geometry module on the asteroid ready", shot: false,
                                        detail: () => $"started {Refl.Get(geo0, "_started")}, scene set up {Refl.Get(geo0, "_sceneSetup")}, " +
                                                      $"meshes {geo0.meshDataList?.Count}");
                yield return Wait.Until(() => FARAPI.VesselVoxelizationCompletedEver(rock) && aero.isValid && aero.VehicleAero?.Length > 0, 120,
                                        "FAR's model of the asteroid", shot: false);
                yield return Wait.Seconds(1);

                Part part = rock.rootPart;
                var asteroid = part.FindModuleImplementing<ModuleAsteroid>();
                Assert.NotNull(asteroid, "the asteroid's ModuleAsteroid");
                var geo = part.FindModuleImplementing<GeometryPartModule>();
                Assert.NotNull(geo, "FAR's geometry module on the asteroid");
                var updaters = (IList)Refl.Get(geo, "geometryUpdaters");
                Assert.True(updaters != null && updaters.Cast<object>().Any(u => u.GetType().Name == "StockProcAsteroidGeoUpdater"),
                            "the asteroid's geometry updater");
                Assert.True(geo.meshDataList != null && geo.meshDataList.Count > 0, "FAR has meshes for the asteroid");

                // the rock's mesh: the procedural asteroid's (ModuleAsteroid.paGenerated)
                var pa = (PAsteroid)Refl.Get(asteroid, "paGenerated");
                Assert.NotNull(pa, "the asteroid's procedural rock");
                double meshVolume = 0;
                int meshes = 0;
                foreach (MeshFilter mf in pa.GetComponentsInChildren<MeshFilter>())
                {
                    Mesh m = mf.sharedMesh;
                    if (m == null || !m.isReadable || mf.GetComponent<Renderer>()?.enabled != true)
                        continue;
                    meshes++;
                    meshVolume += Math.Abs(EnclosedVolume(m, mf.transform.localToWorldMatrix));
                }

                double voxelVolume = Far.Volume(aero.VehicleAero);
                ctx.Record("asteroid", $"{rock.vesselName}: {meshes} meshes, {meshVolume:F2} m3 enclosed; voxels {voxelVolume:F2} m3, " +
                                       $"length {aero.Length:F2} m, max cross-section {aero.MaxCrossSectionArea:F2} m2, {geo.meshDataList.Count} FAR meshes, " +
                                       $"mass {rock.GetTotalMass():F1} t");
                Assert.True(meshes > 0 && meshVolume > 1, $"the rock has a mesh ({meshes}, {meshVolume:F2} m3)");
                Assert.True(voxelVolume > 0.75 * meshVolume && voxelVolume < 1.6 * meshVolume,
                            $"the asteroid's voxel volume near its mesh's ({voxelVolume:F2} m3 against {meshVolume:F2} m3)");
                log.AssertNone("voxelizing the asteroid");
            }
            finally
            {
                if (rock != null && rock != FlightGlobals.ActiveVessel)
                    rock.Die();
                else if (rock == null)
                    HighLogic.CurrentGame?.flightState?.protoVessels?.Remove(pv);
            }
        }

        /// <summary>
        /// The stock inflatable heat shield (HIAD), spawned on its own 150 m above the Kerbal 1 in a 100 km orbit: folded,
        /// a 1.25 m stack part; inflated by its animation (ModuleAnimateGeneric), FAR rebuilds its meshes from the
        /// animated model (FAR's geometry override for it: rebuildOnAnimation) and the voxel model's largest
        /// cross-section grows to about the 10 m disc's area.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 900)]
        public static IEnumerator Flight_InflatedHeatShieldGrowsTheModel(TestContext ctx)
        {
            yield return Wait.Seconds(3);
            Flight.IgnoreMaxTemperature(true);
            Ctrl.SetThrottle(0f);
            ctx.Step("the capsule in orbit");
            Flight.SetOrbit(100000, 100000);
            yield return Wait.Until(() => FlightGlobals.ActiveVessel != null && !FlightGlobals.ActiveVessel.packed && FlightGlobals.ActiveVessel.altitude > 90000,
                                    60, "the capsule off rails in orbit", shot: false);
            yield return Wait.Seconds(1);
            Vessel v = FlightGlobals.ActiveVessel;

            ctx.Step("a heat shield beside it");
            using var log = new ExceptionLog();
            double ut = Planetarium.GetUniversalTime();
            Vector3d r = v.orbit.getRelativePositionAtUT(ut), vel = v.orbit.getOrbitalVelocityAtUT(ut);
            var o = new Orbit();
            o.UpdateFromStateVectors(r + r.normalized * 150, vel, v.mainBody, ut);
            ConfigNode partNode = ProtoVessel.CreatePartNode("InflatableHeatShield", ShipConstruction.GetUniqueFlightID(HighLogic.CurrentGame.flightState));
            ConfigNode vesselNode = ProtoVessel.CreateVesselNode("FAR test HIAD", VesselType.Debris, o, 0, new[] { partNode }, new ConfigNode("ACTIONGROUPS"),
                                                                 ProtoVessel.CreateDiscoveryNode(DiscoveryLevels.Owned, UntrackedObjectClass.A, 1e7, 1e7));
            ProtoVessel pv = HighLogic.CurrentGame.AddVessel(vesselNode);
            Assert.NotNull(pv, "the heat shield spawned");
            Vessel hiad = null;
            try
            {
                yield return Wait.Until(() => (hiad = pv.vesselRef) != null && hiad.loaded && !hiad.packed && hiad.parts.Count > 0, 60, "the heat shield loaded",
                                        shot: false);
                FARVesselAero aero = hiad.GetComponent<FARVesselAero>();
                Assert.NotNull(aero, "FAR's vessel module on the heat shield");
                yield return Wait.Until(() => FARAPI.VesselVoxelizationCompletedEver(hiad) && aero.isValid && aero.VehicleAero?.Length > 0, 120,
                                        "FAR's model of the folded heat shield", shot: false);
                yield return Wait.Seconds(1);
                Part part = hiad.rootPart;
                var geo = part.FindModuleImplementing<GeometryPartModule>();
                Assert.NotNull(geo, "FAR's geometry module on the heat shield");
                double folded = aero.MaxCrossSectionArea;
                int skinned = part.GetComponentsInChildren<SkinnedMeshRenderer>().Length;
                ctx.Record("folded", $"max cross-section {folded:F2} m2, length {aero.Length:F2} m, {geo.meshDataList?.Count} FAR meshes, {skinned} skinned renderers");

                ctx.Step("inflated");
                var anim = part.FindModuleImplementing<ModuleAnimateGeneric>();
                Assert.NotNull(anim, "the heat shield's inflation animation");
                anim.Toggle();
                yield return Wait.Until(() => anim.animTime > 0.99f, 60, "the heat shield inflated", shot: false, detail: () => $"animTime {anim.animTime:F2}");
                const double disc = Math.PI * 5 * 5;
                yield return Wait.Until(() => aero.MaxCrossSectionArea > 0.5 * disc, 120, "FAR's model of the inflated heat shield", shot: false,
                                        detail: () => $"max cross-section {aero.MaxCrossSectionArea:F2} m2");
                yield return Wait.Seconds(2);
                ctx.Record("inflated", $"max cross-section {aero.MaxCrossSectionArea:F2} m2, length {aero.Length:F2} m, {geo.meshDataList?.Count} FAR meshes");
                Assert.InRange(0.5 * disc, 1.25 * disc, aero.MaxCrossSectionArea, "the inflated shield's largest cross-section against a 10 m disc's (m2)");
                Assert.True(aero.MaxCrossSectionArea > 5 * folded, $"far larger than folded ({aero.MaxCrossSectionArea:F2} against {folded:F2} m2)");
                log.AssertNone("inflating the heat shield");
            }
            finally
            {
                if (hiad != null && hiad != FlightGlobals.ActiveVessel)
                    hiad.Die();
                else if (hiad == null)
                    HighLogic.CurrentGame?.flightState?.protoVessels?.Remove(pv);
            }
        }

        /// <summary>The volume a closed mesh encloses (signed: its triangles' tetrahedra from the origin), in world units.</summary>
        private static double EnclosedVolume(Mesh m, Matrix4x4 toWorld)
        {
            Vector3[] verts = m.vertices;
            int[] tris = m.triangles;
            double sum = 0;
            for (int i = 0; i + 2 < tris.Length; i += 3)
            {
                Vector3d a = toWorld.MultiplyPoint3x4(verts[tris[i]]), b = toWorld.MultiplyPoint3x4(verts[tris[i + 1]]), c = toWorld.MultiplyPoint3x4(verts[tris[i + 2]]);
                Vector3d o = toWorld.MultiplyPoint3x4(Vector3.zero);
                sum += Vector3d.Dot(a - o, Vector3d.Cross(b - o, c - o)) / 6;
            }

            return sum;
        }
    }
}
