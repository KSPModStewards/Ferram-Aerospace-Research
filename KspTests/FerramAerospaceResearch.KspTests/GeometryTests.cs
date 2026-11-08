using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARPartGeometry;
using KspTest;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's voxel model following the vessel's shape in flight: what a fairing encloses is out of the airflow until the
    /// fairing goes; a kerbal out on EVA is a box.
    /// </summary>
    public static class GeometryTests
    {
        /// <summary>
        /// A kerbal out on EVA from the stock Kerbal 1 on the pad: FAR gives the kerbal, whose part has no meshes FAR can
        /// use, its geometry and aero modules as it comes out, and models it as a 0.5 x 0.8 x 0.5 m box: 0.8 m long along
        /// its height, 0.25 m2 across, 0.2 m3. The same kerbal goes out as each EVA part there is: male and female, in the
        /// default and the slim suit (the vintage and future suits need the Making History and Breaking Ground expansions,
        /// and are the slim ones without them). The female kerbal in the default suit is the odd one: her part's prefab
        /// (part.partInfo.partPrefab, kerbalEVA_female_Exp) is not PartLoader's and never woke, so it has no
        /// partTransform, which Part.FindModelTransform needs.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Flight_KerbalOnEvaIsABox(TestContext ctx)
        {
            yield return Wait.Seconds(3);
            Vessel v = FlightGlobals.ActiveVessel;
            Part pod = v.parts.FirstOrDefault(p => p.protoModuleCrew.Count > 0);
            if (pod == null)
            {
                // launched with nobody aboard (every kerbal still out on an earlier test's flight): one seated in the pod
                pod = v.parts.FirstOrDefault(p => p.CrewCapacity > 0);
                Assert.NotNull(pod, "a part on the Kerbal 1 that seats a kerbal");
                KerbalRoster roster = HighLogic.CurrentGame.CrewRoster;
                ProtoCrewMember hired = roster.Crew.FirstOrDefault(c => c.rosterStatus == ProtoCrewMember.RosterStatus.Available) ??
                                        roster.GetNextOrNewKerbal();
                pod.AddCrewmember(hired);
                v.CrewListSetDirty();
                v.SpawnCrew();
                ctx.Record("crew", $"launched with nobody aboard: {hired.name} seated in the {pod.partInfo.title}");
                yield return Wait.Seconds(2);
            }

            ProtoCrewMember crew = pod.protoModuleCrew[0];
            ProtoCrewMember.Gender gender0 = crew.gender;
            ProtoCrewMember.KerbalSuit suit0 = crew.suit;
            try
            {
                foreach (ProtoCrewMember.Gender gender in new[] { ProtoCrewMember.Gender.Male, ProtoCrewMember.Gender.Female })
                foreach (ProtoCrewMember.KerbalSuit suit in new[] { ProtoCrewMember.KerbalSuit.Default, ProtoCrewMember.KerbalSuit.Slim })
                    yield return KerbalOutOnEva(ctx, pod, crew, gender, suit);
            }
            finally
            {
                crew.gender = gender0;
                crew.suit = suit0;
            }
        }

        private static IEnumerator KerbalOutOnEva(TestContext ctx, Part pod, ProtoCrewMember crew, ProtoCrewMember.Gender gender, ProtoCrewMember.KerbalSuit suit)
        {
            crew.gender = gender;
            crew.suit = suit;
            string partName = crew.GetKerbalEVAPartName();
            using var log = new ExceptionLog();

            ctx.Step($"{crew.name} out on EVA as {partName}");
            Assert.True(FlightEVA.fetch.spawnEVA(crew, pod, pod.airlock, true), "the kerbal went out on EVA");
            yield return Wait.Until(() => FlightGlobals.ActiveVessel != null && FlightGlobals.ActiveVessel.isEVA, 30, "the kerbal on EVA", shot: false);
            Vessel eva = FlightGlobals.ActiveVessel;
            Part kerbal = eva.rootPart;
            Assert.Equal(partName, kerbal.partInfo.name, "the kerbal's EVA part");
            var aero = eva.GetComponent<FARVesselAero>();
            Assert.NotNull(aero, "FAR's vessel module on the kerbal");
            yield return Wait.Until(() => aero.isValid && aero.VehicleAero.Length > 0 && FARAPI.VesselVoxelizationCompletedEver(eva), 60,
                                    $"FAR's model of the kerbal ({partName})", shot: false);
            yield return Wait.Seconds(1);
            Assert.NotNull(kerbal.FindModuleImplementing<GeometryPartModule>(), "FAR's geometry module on the kerbal");
            Assert.NotNull(kerbal.FindModuleImplementing<FARAeroPartModule>(), "FAR's aero module on the kerbal");
            double volume = Far.Volume(aero.VehicleAero), length = aero.Length, area = aero.MaxCrossSectionArea;
            ctx.Record(partName, $"length {length:F3} m, max cross-section {area:F4} m2, volume {volume:F4} m3");
            Assert.Near(0.8, length, 0.12, "the box's height along its axis");
            Assert.Near(0.25, area, 0.25 * 0.25, "the box's 0.5 x 0.5 m cross-section");
            Assert.Near(0.2, volume, 0.25 * 0.2, "the box's volume");
            log.AssertNone($"the kerbal out on EVA as {partName}");

            ctx.Step("Back aboard");
            kerbal.FindModuleImplementing<KerbalEVA>()?.BoardPart(pod);
            yield return Wait.Until(() => FlightGlobals.ActiveVessel != null && !FlightGlobals.ActiveVessel.isEVA && pod.protoModuleCrew.Contains(crew),
                                    30, "the kerbal back aboard", shot: false);
            yield return Wait.Seconds(1);
        }

        /// <summary>
        /// The stock ComSat Lx (its launch clamp taken off: crafts/) carries its satellite in a procedural fairing: FAR shields the satellite's parts (no exposed
        /// area, no force) while the fairing is closed, and exposes them once it is jettisoned and the vessel revoxelized.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "ComSat Lx Unclamped", CraftDir = "VAB", TimeoutSec = 900)]
        public static IEnumerator Flight_FairingShieldsItsPayloadUntilJettisoned(TestContext ctx)
        {
            yield return Wait.Seconds(3);
            yield return Wait.Until(() => FARAPI.VesselVoxelizationCompletedEver(FlightGlobals.ActiveVessel), 90, "the vessel voxelized", shot: false);
            yield return Wait.Seconds(2);   // FARVesselAero applies the new sections (shielding) in its next FixedUpdates
            Vessel v = FlightGlobals.ActiveVessel;
            Part fairingBase = v.parts.FirstOrDefault(p => p.Modules.Contains<ModuleProceduralFairing>());
            Assert.NotNull(fairingBase, "the ComSat Lx's fairing");
            List<Part> payload = Far.PayloadOf(v, fairingBase);
            Part core = v.rootPart;
            Assert.True(payload.Contains(core), "the probe core is in the payload");
            int shielded = payload.Count(p => p.ShieldedFromAirstream);
            ctx.Record("payload", $"{payload.Count} parts, {shielded} shielded: {string.Join(", ", payload.Where(p => p.ShieldedFromAirstream).Select(p => p.partInfo.name))}");
            Assert.True(core.ShieldedFromAirstream, "the probe core inside the closed fairing is shielded");
            Assert.True(shielded * 2 >= payload.Count, $"most of the payload is shielded ({shielded} of {payload.Count})");
            Assert.True(core.Modules.GetModule<FARAeroPartModule>().ProjectedAreas.totalArea <= 0, "no exposed area on the shielded core");

            ctx.Step("Jettison the fairing");
            fairingBase.Modules.GetModule<ModuleProceduralFairing>().DeployFairing();
            yield return Wait.Until(() => !core.ShieldedFromAirstream, 30, "the probe core exposed after the fairing goes", shot: false);
            v = FlightGlobals.ActiveVessel;
            int stillShielded = payload.Where(p => p != null && p.vessel == v).Count(p => p.ShieldedFromAirstream);
            ctx.Record("afterJettison", $"{stillShielded} payload parts still shielded");
            Assert.True(core.Modules.GetModule<FARAeroPartModule>().ProjectedAreas.totalArea > 0, "the exposed core has area in the airflow");
            Assert.True(stillShielded < shielded, $"fewer payload parts shielded after the jettison ({stillShielded}, from {shielded})");
        }

        /// <summary>
        /// Regression: a part's mesh list rebuilt while a voxelization thread reads it. RebuildAllMeshData put a new,
        /// empty list in meshDataList and added the meshes to it there, so a worker that took the list then enumerated one
        /// being added to: "Collection was modified" in VehicleVoxel.UpdateFromMesh (seen inflating the HIAD, whose
        /// animation rebuilds its meshes as voxelizations run). A thread enumerates the Kerbal 1's part with the most
        /// meshes while the main thread rebuilds them, 300 times: the list it takes is never added to.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Flight_MeshListRebuiltWhileAThreadReadsIt(TestContext ctx)
        {
            yield return Wait.Seconds(3);
            Vessel v = FlightGlobals.ActiveVessel;
            GeometryPartModule g = v.parts.Select(p => p.FindModuleImplementing<GeometryPartModule>())
                                    .Where(m => m != null && m.meshDataList != null).OrderByDescending(m => m.meshDataList.Count).FirstOrDefault();
            Assert.NotNull(g, "a part with FAR meshes");
            ctx.Record("part", $"{g.part.partInfo.name}, {g.meshDataList.Count} meshes");
            int errors = 0, reads = 0;
            string first = null;
            bool stop = false;
            var reader = new System.Threading.Thread(() =>
            {
                while (!System.Threading.Volatile.Read(ref stop))
                {
                    try
                    {
                        List<GeometryMesh> list = g.meshDataList;
                        if (list == null) continue;
                        int n = 0;
                        foreach (GeometryMesh m in list)
                            n += m == null ? 0 : 1;
                        System.Threading.Interlocked.Increment(ref reads);
                    }
                    catch (System.InvalidOperationException x)
                    {
                        if (System.Threading.Interlocked.Increment(ref errors) == 1) first = x.Message;
                    }
                }
            }) { IsBackground = true };
            reader.Start();
            try
            {
                for (int k = 0; k < 300; k++)
                {
                    g.RebuildAllMeshData();
                    if (k % 25 == 24)
                        yield return null;
                }
            }
            finally
            {
                System.Threading.Volatile.Write(ref stop, true);
                reader.Join(5000);
            }

            ctx.Record("reads", $"{reads} enumerations, {errors} of them broken ({first})");
            Assert.True(reads > 0, "the thread read the list");
            Assert.Equal(0, errors, "enumerations broken by the rebuild");
        }

        /// <summary>
        /// Regression: a mesh gone from a part (its transform destroyed) taken out of the list a voxelization thread may be
        /// reading. UpdateTransformMatrixList removed it from the list in place; an enumeration of the list as it was
        /// broke on its next step. Now the list is replaced by one without it, and the old one stays whole.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Flight, Craft = "Kerbal 1", CraftDir = "VAB", TimeoutSec = 600)]
        public static IEnumerator Flight_GoneMeshLeavesTheListBeingRead(TestContext ctx)
        {
            yield return Wait.Seconds(3);
            Vessel v = FlightGlobals.ActiveVessel;
            GeometryMesh leaf = null;
            GeometryPartModule g = null;
            foreach (Part p in v.parts)
            {
                GeometryPartModule m = p.FindModuleImplementing<GeometryPartModule>();
                if (m?.meshDataList == null || m.meshDataList.Count < 2) continue;
                leaf = m.meshDataList.FirstOrDefault(x => x.meshTransform != null && x.meshTransform.childCount == 0 && x.meshTransform != p.transform);
                if (leaf == null) continue;
                g = m;
                break;
            }

            Assert.NotNull(g, "a part with two meshes or more, one on a transform of its own");
            List<GeometryMesh> before = g.meshDataList;
            int count = before.Count;
            ctx.Record("part", $"{g.part.partInfo.name}: {count} meshes, {leaf.meshTransform.name} destroyed");
            List<GeometryMesh>.Enumerator reading = before.GetEnumerator();
            Assert.True(reading.MoveNext(), "the list read from its first mesh");
            Object.Destroy(leaf.meshTransform.gameObject);
            yield return null;
            Assert.True(leaf.meshTransform == null, "the mesh's transform gone");
            g.UpdateTransformMatrixList(v.vesselTransform.worldToLocalMatrix);
            int rest = 0;
            string broke = null;
            try
            {
                while (reading.MoveNext())
                    rest++;
            }
            catch (System.InvalidOperationException x)
            {
                broke = x.Message;
            }

            ctx.Record("after", $"{g.meshDataList.Count} meshes now; the old list read on: {(broke ?? $"{rest} more")}");
            Assert.True(broke == null, $"the list being read is not changed under it ({broke})");
            Assert.Equal(count - 1, g.meshDataList.Count, "the gone mesh out of the part's list");
            Assert.False(g.meshDataList.Contains(leaf), "and not in it");
        }
    }
}
