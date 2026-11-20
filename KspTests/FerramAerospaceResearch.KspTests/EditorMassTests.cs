using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using FerramAerospaceResearch.FARPartGeometry;
using KspTest;
using UnityEngine;
using static FerramAerospaceResearch.KspTests.Refl;
using static FerramAerospaceResearch.KspTests.EditorWindow;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The editor's mass (issue 194): physicsless parts weigh what they do in flight, where KSP adds their mass to
    /// their parent's rigidbody, and the Heatmaps tab says when the map it shows is not of the craft as it is now.
    /// </summary>
    public static class EditorMassTests
    {
        private const string AjeRequires = "AJE";
        private const string AjeMods = "AdvancedJetEngine";
        private static readonly string[] SmallSweep = { "0.3", "0.9", "0", "6", "3", "3" };

        private static readonly MethodInfo DetachPart = typeof(EditorLogic).GetMethod("detachPart", BindingFlags.Instance | BindingFlags.NonPublic);
        private static readonly MethodInfo AttachPart = typeof(EditorLogic).GetMethod("attachPart", BindingFlags.Instance | BindingFlags.NonPublic);

        /// <summary>
        /// The craft's mass (t) as KSP has it, crew and all. GetTotalMass resets every part's mass to its own (UpdateMass)
        /// until the next FixedUpdate puts the crew's back in their seat's (Part.FixedUpdate): read FAR's numbers first.
        /// </summary>
        private static double ShipMass() => EditorLogic.fetch.ship.GetTotalMass();

        /// <summary>
        /// The Aeris 4A has physicsless parts (11 of its 40). The stability tab's mass is the craft's, theirs and all,
        /// and each physicsless part's mass is in the physical part it hangs off: the parts' EditorPartMass add up to
        /// the craft's.
        /// Regression: before the fix, the editor left physicsless parts out of the mass (0.355 t of the Aeris 4A's
        /// 22.430 t), though KSP adds their mass to their parent's rigidbody in flight (FlightIntegrator).
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 600)]
        public static IEnumerator Editor_PhysicslessPartsAreInTheMass(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            yield return Far.Settled("the Aeris 4A");
            List<Part> parts = EditorLogic.fetch.ship.parts;
            List<Part> physicsless = parts.Where(FARAeroUtil.IsNonphysical).ToList();
            double physicslessMass = physicsless.Sum(p => (double)p.mass + p.GetResourceMass());
            ctx.Record("physicsless", $"{physicsless.Count} of {parts.Count} parts, {physicslessMass:F4} t: " +
                                      string.Join(", ", physicsless.Select(p => p.partInfo.name).Distinct()));
            Assert.True(physicsless.Count > 0 && physicslessMass > 0.01, "the Aeris 4A has physicsless parts with mass");

            double summed = parts.Sum(FARAeroUtil.EditorPartMass);

            var sim = (EditorSimManager)Get(Far.Editor, "_simManager");

            double stab = sim.StabDerivCalculator.ComputeVehicleProperties(0, false).Mass / 1000;

            double ship = ShipMass();
            ctx.Record("mass", $"ship {ship:F4} t, parts' EditorPartMass {summed:F4} t, stability {stab:F4} t");
            Assert.Near(ship, summed, 1e-4, "the parts' rigidbody masses add up to the craft's (t)");
            Assert.Near(ship, stab, 1e-4, "the stability tab's mass is the craft's (t)");
            foreach (Part p in physicsless)
                Assert.True(FARAeroUtil.MassCounts(p) && FARAeroUtil.EditorPartMass(p) == 0,
                            $"{p.partInfo.name}'s mass is its parent's, not its own");
        }

        /// <summary>
        /// The Heatmaps tab on the Aeris 4A, swept: the map is the craft's. A fuel tank half emptied (the same parts,
        /// another mass) and the tab flags the map as of another craft; filled again, it does not. An I-beam added (another
        /// part count): flagged again, with the tab showing the cached sweep of the craft without it; taken off again,
        /// it does not.
        /// Regression: before the fix, the tab showed the newest cached sweep of the craft at any part count, and a map
        /// from before an edit, with nothing to say so (issue 194: the map's mass that of the craft without its blades).
        /// </summary>
        [UITest(Kind = "Check", Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Envelope_StaleMapIsFlagged(TestContext ctx)
        {
            string cachePath = null;
            PartResource fuel = null;
            double fuelAmount = 0;
            Part added = null;
            try
            {
                yield return Far.WaitForEditorVoxels();
                yield return Far.Settled("the Aeris 4A");
                yield return OpenWindow("FAREditorModeHeatmaps");
                var sw = new SweepOut();
                yield return Sweep(SmallSweep, false, true, sw);
                cachePath = sw.CachePath;
                HeatmapsGUI hm = Heatmaps;
                HeatmapResult r = sw.Results[0];
                ctx.Record("map", $"{r.PartCount} parts, {r.SampleMassesKg[r.SampleMassesKg.Length - 1] / 1000:F3} t");
                yield return Wait.Until(() => hm.StaleReason() == null, 5, "the fresh map not flagged", shot: false,
                                        detail: () => hm.StaleReason());

                ctx.Step("a tank half emptied");
                fuel = EditorLogic.fetch.ship.parts.SelectMany(p => p.Resources.Cast<PartResource>())
                                  .First(res => res.resourceName == "LiquidFuel" && res.amount > 10);
                fuelAmount = fuel.amount;
                fuel.amount = fuelAmount / 2;
                yield return Wait.Until(() => hm.StaleReason() != null, 5, "the map flagged after the fuel change", shot: false);
                ctx.Record("fuelChanged", hm.StaleReason());
                yield return Snap.Shot(ctx, "stale-mass");
                fuel.amount = fuelAmount;
                fuel = null;
                yield return Wait.Until(() => hm.StaleReason() == null, 5, "the flag gone with the fuel back", shot: false,
                                        detail: () => hm.StaleReason());

                ctx.Step("an I-beam added");
                Part root = EditorLogic.RootPart;
                added = Spawn(PartLoader.getPartInfoByName("structuralIBeam1"));
                added.transform.position = root.transform.position + root.transform.right * 0.7f;
                added.transform.rotation = root.transform.rotation;
                AttachSurface(added, root);
                yield return Far.WaitForEditorVoxels();
                yield return Wait.Until(() => hm.StaleReason() != null, 5, "the map flagged with the I-beam on", shot: false);
                ctx.Record("partAdded", $"{hm.StaleReason()}; status '{Get<string>(hm, "_statusMessage")}'");
                Assert.True(hm.StaleReason().Contains($"{r.PartCount} physical parts") && hm.StaleReason().Contains($"{r.PartCount + 1} physical parts"),
                            "the flag gives the map's part count and the craft's: " + hm.StaleReason());
                yield return Snap.Shot(ctx, "stale-parts");

                ctx.Step("the I-beam taken off again");
                Detach(added);
                UnityEngine.Object.Destroy(added.gameObject);
                added = null;
                yield return Far.WaitForEditorVoxels();
                yield return Wait.Until(() => hm.StaleReason() == null, 5, "the flag gone with the I-beam off", shot: false,
                                        detail: () => hm.StaleReason());
            }
            finally
            {
                if (fuel != null)
                    fuel.amount = fuelAmount;
                DeleteCache(cachePath);
                CloseWindow();
                // the craft as it was for the tests after (the editor keeps it): loaded again
                if (added != null)
                    EditorLogic.LoadShipFromFile(ctx.CraftPath(Far.Plane, "SPH"));
            }
        }

        // ------------------------------------------------------------------------------------------ Breaking Ground

        private const string Blade = "mediumPropeller";
        private const int Blades = 8;

        /// <summary>
        /// Issue 194 with the Breaking Ground blades: 8 medium propeller blades on the Aeris 4A's root, a sweep before
        /// and after. The blades count (FAR leaves them out of its aero only), the tab flags the cached sweep of the
        /// bare craft it shows after the edit, and the new sweep's mass is the craft's.
        /// Needs the blade parts as a local mod (KSPTEST_MODS=path/to/BGBladeTest: the DLC's Parts/Propellers cfgs and
        /// models, MODEL paths pointed there), so Explicit; not for the repository or CI (the parts are the DLC's).
        /// </summary>
        [UITest(Kind = "Check", Explicit = true, Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane,
                TimeoutSec = 1500)]
        public static IEnumerator Issue194_BladeMassInHeatmaps(TestContext ctx) => BladeMass(ctx, false);

        /// <summary>
        /// Issue 194's occlusion case: the same blades, each placed wholly inside the craft's biggest part it fits in
        /// (its mesh box, rotated to the part's, inside the part's). Hidden or not, they weigh the same.
        /// </summary>
        [UITest(Kind = "Check", Explicit = true, Requires = AjeRequires, Mods = AjeMods, StartsAt = StartState.Sph, Craft = Far.Plane,
                TimeoutSec = 1500)]
        public static IEnumerator Issue194_HiddenBladeMassInHeatmaps(TestContext ctx) => BladeMass(ctx, true);

        private static IEnumerator BladeMass(TestContext ctx, bool hidden)
        {
            var caches = new List<string>();
            try
            {
                AvailablePart ap = PartLoader.getPartInfoByName(Blade);
                Assert.NotNull(ap, $"the {Blade} part (KSPTEST_MODS needs the BGBladeTest folder)");
                Part prefab = ap.partPrefab;
                ctx.Record("bladePrefab", $"mass {prefab.mass} t, PhysicsSignificance {prefab.PhysicsSignificance}, " +
                                          $"GeometryPartModule {prefab.Modules.Contains<GeometryPartModule>()}, " +
                                          $"FARAeroPartModule {prefab.Modules.Contains<FARAeroComponents.FARAeroPartModule>()}, " +
                                          $"FAR wing {prefab.Modules.Contains<ferram4.FARWingAerodynamicModel>()}, " +
                                          $"ModuleControlSurface {prefab.Modules.Contains<ModuleControlSurface>()}, " +
                                          $"collider {(prefab.collider != null)}, dragModel {prefab.dragModel}");

                yield return Far.WaitForEditorVoxels();
                yield return Far.Settled("the Aeris 4A");
                var sim = (EditorSimManager)Get(Far.Editor, "_simManager");

                ctx.Step("bare: sweep");
                double bareStab = sim.StabDerivCalculator.ComputeVehicleProperties(0, false).Mass / 1000;
                double bare = ShipMass();
                ctx.Record("bare", $"{EditorLogic.fetch.ship.parts.Count} parts, {bare:F4} t, stability calc {bareStab:F4} t");
                yield return Wait.Seconds(0.1); // the crew's mass back in the cockpit's before the sweep
                var nonAero = new List<KeyValuePair<Part, string>>();
                yield return NonAeroRows(nonAero);
                ctx.Record("bareNonAero", Rows(nonAero));
                yield return OpenWindow("FAREditorModeHeatmaps");
                var sw = new SweepOut();
                yield return Sweep(SmallSweep, false, true, sw);
                caches.Add(sw.CachePath);
                ctx.Record("bareSweep", Shown(Heatmaps));

                Part root = EditorLogic.RootPart;
                if (!hidden)
                {
                    ctx.Step($"{Blades} blades on the root");
                    for (int i = 0; i < Blades; i++)
                    {
                        Part b = Spawn(ap);
                        float side = i % 2 == 0 ? 1 : -1;
                        b.transform.position = root.transform.position + root.transform.right * (0.7f * side) +
                                               root.transform.up * (0.3f * (i / 2) - 0.45f);
                        b.transform.rotation = root.transform.rotation;
                        AttachSurface(b, root);
                    }
                }
                else
                {
                    Bounds blade = LocalBounds(prefab);
                    Vector3 bladeSize = blade.size;
                    Part host = null;
                    Bounds hostBox = default;
                    foreach (Part p in EditorLogic.fetch.ship.parts)
                    {
                        if (FARAeroUtil.IsNonphysical(p))
                            continue;
                        Bounds b = LocalBounds(p);
                        if (Fits(bladeSize, b.size) && (host == null || Volume(b.size) > Volume(hostBox.size)))
                        {
                            host = p;
                            hostBox = b;
                        }
                    }

                    Assert.NotNull(host, $"a part the blade ({Fmt(bladeSize)} m) fits inside");
                    // the blade's longest and middle axes along the host's
                    Quaternion turn = Quaternion.LookRotation(Axis(hostBox.size, 0), Axis(hostBox.size, 1)) *
                                      Quaternion.Inverse(Quaternion.LookRotation(Axis(bladeSize, 0), Axis(bladeSize, 1)));
                    ctx.Step($"{Blades} blades inside the {host.partInfo.title}");
                    ctx.Record("host", $"{host.partInfo.name} box {Fmt(hostBox.size)} m, blade box {Fmt(bladeSize)} m");
                    float slack = hostBox.size[Longest(hostBox.size)] - bladeSize[Longest(bladeSize)];
                    for (int i = 0; i < Blades; i++)
                    {
                        Part b = Spawn(ap);
                        // spread along the host's long axis, within the slack
                        Vector3 at = hostBox.center + Axis(hostBox.size, 0) * (slack * 0.4f * ((i + 0.5f) / Blades - 0.5f));
                        b.transform.rotation = host.transform.rotation * turn;
                        b.transform.position = host.transform.TransformPoint(at) - b.transform.rotation * blade.center;
                        AttachSurface(b, host);
                        Bounds inHost = BoundsIn(b, host.transform);
                        Assert.True(hostBox.Contains(inHost.min) && hostBox.Contains(inHost.max),
                                    $"blade {i} inside the host's box: blade {Fmt(inHost.min)}..{Fmt(inHost.max)}, " +
                                    $"host {Fmt(hostBox.min)}..{Fmt(hostBox.max)}");
                    }
                }

                yield return Wait.Frames(10);
                yield return Far.WaitForEditorVoxels();
                yield return Far.Settled("the Aeris 4A with blades");
                List<Part> blades = EditorLogic.fetch.ship.parts.Where(p => p.partInfo.name == Blade).ToList();
                Assert.Equal(Blades, blades.Count, "blades on the ship");
                double stab = sim.StabDerivCalculator.ComputeVehicleProperties(0, false).Mass / 1000;
                double withBlades = ShipMass();
                yield return Wait.Seconds(0.1); // the crew's mass back in the cockpit's
                ctx.Record("withBlades", $"{EditorLogic.fetch.ship.parts.Count} parts, {withBlades:F4} t, stability calc {stab:F4} t; " +
                                         $"blades nonphysical: {blades.Count(FARAeroUtil.IsNonphysical)}");
                ctx.Record("bladeAero", string.Join(", ", blades.Select(b =>
                {
                    var m = b.Modules.GetModule<FARAeroComponents.FARAeroPartModule>();
                    return m == null ? "no FARAeroPartModule" : $"{m.ProjectedAreas.totalArea:F3} m2";
                })));
                yield return Snap.Shot(ctx, hidden ? "hidden-blades" : "blades");
                Assert.Near(bare + Blades * prefab.mass, withBlades, 1e-4, "the blades' mass on the ship (t)");
                Assert.Near(withBlades, stab, 1e-4, "the stability tab's mass, blades and all (t)");

                ctx.Step("the Components table's non-aero rows name the blades");
                yield return NonAeroRows(nonAero);
                ctx.Record("nonAero", Rows(nonAero));
                // the table scrolled to its end, where the non-aero rows are
                Set(Get(Far.Editor, "_components"), "scroll", new Vector2(0, 1e6f));
                yield return Wait.Frames(3);
                yield return Snap.Shot(ctx, hidden ? "components-hidden-blades" : "components-blades");
                List<KeyValuePair<Part, string>> bladeRows = nonAero.Where(r => r.Key.partInfo.name == Blade).ToList();
                Assert.Equal(Blades, bladeRows.Count, "the blades' non-aero rows");
                Assert.True(bladeRows.All(r => r.Value.StartsWith("no FAR shape")), "each as having no FAR shape: " + Rows(bladeRows));

                ctx.Step("the Heatmaps tab before a new sweep");
                SelectTab("FAREditorModeStatic");
                yield return Wait.Frames(3);
                SelectTab("FAREditorModeHeatmaps");
                HeatmapsGUI hm = Heatmaps;
                yield return Wait.Until(() => hm.StaleReason() != null, 5, "the bare craft's map flagged", shot: false);
                ctx.Record("tabAfterEdit", Shown(hm) + "; flag: " + hm.StaleReason());

                ctx.Step("with blades: sweep");
                var sw2 = new SweepOut();
                yield return Sweep(SmallSweep, false, true, sw2);
                caches.Add(sw2.CachePath);
                ctx.Record("bladeSweep", Shown(hm));
                Assert.Near(withBlades, sw2.Results[0].MassKg / 1000, 1e-3, "the swept wet mass is the craft's, blades and all (t)");
                yield return Wait.Until(() => hm.StaleReason() == null, 5, "the new map not flagged", shot: false,
                                        detail: () => hm.StaleReason());
            }
            finally
            {
                foreach (string c in caches)
                    DeleteCache(c);
                CloseWindow();
                // the craft without the blades for the tests after (the editor keeps it)
                EditorLogic.LoadShipFromFile(ctx.CraftPath(Far.Plane, "SPH"));
            }
        }

        /// <summary>
        /// The Components tab shown and computed for the craft as it is, its table with the non-aero parts (clicked on as a
        /// player would): those parts, each with why
        /// </summary>
        private static IEnumerator NonAeroRows(List<KeyValuePair<Part, string>> into)
        {
            yield return OpenWindow("FAREditorModeComponents");
            object comp = Get(Far.Editor, "_components");
            Remember(comp, "showTable", "showNonAero");
            if (!Get<bool>(comp, "showTable"))
                yield return Imgui.Click("Show Table", Window);
            if (!Get<bool>(comp, "showNonAero"))
                yield return Imgui.Click("Show Non-Aero", Window);
            yield return Wait.Until(() => Get<bool>(comp, "showNonAero"), 5, "the non-aero rows shown", shot: false);
            Set(comp, "dirty", true);
            yield return Far.WaitForEditorVoxels();
            yield return Wait.Until(() => !Get<bool>(comp, "dirty"), 15, "the Components tab to recompute", shot: false);
            yield return Wait.Frames(3);
            into.Clear();
            foreach (object row in (IEnumerable)Get(comp, "nonAeroRows"))
                into.Add(new KeyValuePair<Part, string>((Part)Get(row, "Part"), Get<string>(row, "Reason")));
        }

        private static string Rows(IEnumerable<KeyValuePair<Part, string>> rows) =>
            rows.Any() ? string.Join("; ", rows.Select(r => $"{r.Key.partInfo.name}: {r.Value}")) : "(none)";

        /// <summary>The part's shown meshes' box in its own frame (m)</summary>
        private static Bounds LocalBounds(Part p) => BoundsIn(p, p.transform);

        /// <summary>The part's shown meshes' box in the given frame (m); a prefab's too, inactive as it is</summary>
        private static Bounds BoundsIn(Part p, Transform frame)
        {
            var box = new Bounds();
            bool any = false;
            foreach (Renderer r in p.GetComponentsInChildren<Renderer>(true))
            {
                Mesh mesh = r is SkinnedMeshRenderer s ? s.sharedMesh : r.GetComponent<MeshFilter>()?.sharedMesh;
                if (mesh == null || !r.enabled || !Active(r.transform, p.transform))
                    continue;
                Bounds mb = mesh.bounds;
                for (int c = 0; c < 8; c++)
                {
                    Vector3 corner = mb.center + Vector3.Scale(mb.extents, new Vector3((c & 1) != 0 ? 1 : -1, (c & 2) != 0 ? 1 : -1, (c & 4) != 0 ? 1 : -1));
                    Vector3 local = frame.InverseTransformPoint(r.transform.TransformPoint(corner));
                    if (any)
                        box.Encapsulate(local);
                    else
                        box = new Bounds(local, Vector3.zero);
                    any = true;
                }
            }

            return box;
        }

        /// <summary>
        /// Active up to the part (a prefab's root is inactive), and the part's own: in the editor its children's
        /// transforms are under its
        /// </summary>
        private static bool Active(Transform t, Transform part)
        {
            for (; t != null && t != part; t = t.parent)
                if (!t.gameObject.activeSelf || t.GetComponent<Part>() != null)
                    return false;
            return true;
        }

        /// <summary>
        /// The blade's box fits in the host's, longest axis along longest: room to spare along it, and in the
        /// cross-section a box whose corners clear a round fuselage's skin
        /// </summary>
        private static bool Fits(Vector3 blade, Vector3 host)
        {
            float[] b = Sorted(blade), h = Sorted(host);
            return b[0] <= 0.95f * h[0] && b[1] <= 0.7f * h[1] && b[2] <= 0.7f * h[2];
        }

        private static float[] Sorted(Vector3 v) => new[] { v.x, v.y, v.z }.OrderByDescending(x => x).ToArray();
        private static float Volume(Vector3 v) => v.x * v.y * v.z;

        /// <summary>The local axis of the box's rank-th biggest side (0: the longest)</summary>
        private static Vector3 Axis(Vector3 size, int rank)
        {
            int i = new[] { 0, 1, 2 }.OrderByDescending(k => size[k]).ElementAt(rank);
            Vector3 a = Vector3.zero;
            a[i] = 1;
            return a;
        }

        private static int Longest(Vector3 v) => v.x >= v.y && v.x >= v.z ? 0 : v.y >= v.z ? 1 : 2;
        private static string Fmt(Vector3 v) => $"({v.x:F2}, {v.y:F2}, {v.z:F2})";

        private static string Shown(HeatmapsGUI hm)
        {
            var results = Get<List<HeatmapResult>>(hm, "_results");
            if (results == null || results.Count == 0)
                return "no map";
            HeatmapResult r = results[0];
            return $"map mass {r.MassKg / 1000:F3} t, {r.PartCount} parts; status '{Get<string>(hm, "_statusMessage")}'";
        }

        private static Part Spawn(AvailablePart ap)
        {
            Part part = UnityEngine.Object.Instantiate(ap.partPrefab);
            part.gameObject.SetActive(true);
            part.name = ap.name;
            part.persistentId = FlightGlobals.CheckPartpersistentId(part.persistentId, part, false, true);
            part.InitializeModules();
            return part;
        }

        /// <summary>A part (no children) taken off the craft as the editor does when it is picked up, and the editor told.</summary>
        private static void Detach(Part p)
        {
            Assert.NotNull(DetachPart, "EditorLogic.detachPart");
            DetachPart.Invoke(EditorLogic.fetch, new object[] { p });
            p.OnEditorDetach?.Invoke();
            GameEvents.onEditorPartEvent.Fire(ConstructionEventType.PartDetached, p);
            GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
        }

        /// <summary>The part put on its parent by its surface attachment node where it is, as the editor attaches a dropped part.</summary>
        private static void AttachSurface(Part p, Part parent)
        {
            Assert.NotNull(AttachPart, "EditorLogic.attachPart");
            Type attachmentType = AttachPart.GetParameters()[1].ParameterType;
            object attach = Activator.CreateInstance(attachmentType, true);
            void Put(string name, object value) => attachmentType.GetField(name).SetValue(attach, value);
            Put("possible", true);
            Put("mode", AttachModes.SRF_ATTACH);
            Put("potentialParent", parent);
            Put("caller", p);
            Put("callerPartNode", p.srfAttachNode);
            Put("otherPartNode", null);
            Put("position", p.transform.position);
            Put("rotation", p.transform.rotation);
            AttachPart.Invoke(EditorLogic.fetch, new[] { p, attach });
            p.OnEditorAttach?.Invoke();
            GameEvents.onEditorPartEvent.Fire(ConstructionEventType.PartAttached, p);
            GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
        }
    }
}
