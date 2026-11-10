using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using KspTest;
using UnityEngine;
using static FerramAerospaceResearch.KspTests.Refl;
using static FerramAerospaceResearch.KspTests.EditorWindow;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>Pictures of how FAR groups a craft's wing panels into wings for their aspect ratio (Components tab).</summary>
    public static class WingGroupShots
    {
        private const string Mods = @"AdvancedJetEngine;Firespitter;TimeControl;..\..\..\CustomProceduralParts\GameData\CustomProceduralParts;" +
                                    @"..\FerramAerospaceResearch.KspTests.Run\mods\FARTestFloatplane";

        /// <summary>
        /// Each craft (parameter crafts, + separated, SPH; default the Cessna floatplane as built, with plain wing panels
        /// beside its merged control strips, and with B9's own hinged control surfaces) in the SPH, FAR's Components tab
        /// colouring it by Wings (each wing a colour, grey: a panel keeping its own aspect ratio) and by Aspect Ratio, the
        /// table shown; screenshots from above at three headings, and each panel's wing and aspect ratios recorded.
        /// </summary>
        [UITest(Kind = "Measure", Explicit = true, Mods = Mods, StartsAt = StartState.Sph, Craft = "Cessna 172M Floatplane", TimeoutSec = 900)]
        public static IEnumerator Shot_C172WingGroups(TestContext ctx)
        {
            string[] crafts = (ctx.Param("crafts") ?? "Cessna 172M Floatplane+Cessna 172M Floatplane NoPlain+Cessna 172M Floatplane Hinged").Split('+');
            foreach (string raw in crafts)
            {
                string craft = raw.Trim();
                ctx.Step(craft);
                yield return Nav.LoadCraft(ctx, ctx.CraftPath(craft, "SPH"));
                yield return Wait.Seconds(2);
                yield return OpenWindow("FAREditorModeComponents");
                ResetConditions();
                object comp = Get(Far.Editor, "_components");
                object tint = Get(comp, "tintDropdown");
                yield return Imgui.SetTextAfterLabel("Speed (m/s)", "30", Window);
                yield return Imgui.SetTextAfterLabel("AoA (deg)", "4", Window);
                if (!Get<bool>(comp, "showTable"))
                    yield return Imgui.Click("Show Table", Window);
                yield return Far.WaitForEditorVoxels();
                yield return Recomputed(comp);

                var rows = ((IList)Get(comp, "rows")).Cast<object>().Where(r => Get(r, "Wing") != null && Get(r, "Part") != null).ToList();
                var sb = new StringBuilder();
                foreach (object r in rows.OrderBy(r => Get<int>(r, "Group")).ThenBy(r => Get<string>(r, "Name")))
                    sb.Append(string.Format(CultureInfo.InvariantCulture, "{0} [wing {1}] S {2:F2} own AR {3:F2} effective {4:F2}; ",
                                            Get<string>(r, "Name"), Get<int>(r, "Group") < 0 ? "-" : (Get<int>(r, "Group") + 1).ToString(),
                                            Get<double>(r, "S"), Get<double>(r, "OwnAR"), Get<double>(r, "EffectiveAR")));
                ctx.Record(craft + " panels", sb.ToString());
                // a B9 wing panel's materials: their shaders and which colour properties they have (what a tint can set)
                Part b9 = EditorLogic.fetch.ship.parts.FirstOrDefault(q => q.partInfo.name.StartsWith("B9.Aero"));
                if (b9 != null)
                {
                    string[] props = { "_Color", "_MaskColor1", "_TintColor", "_MainColor", "_RimColor", "_TemperatureColor", "_BurnColor", "_EmissiveColor", "_SpecColor" };
                    ctx.Record(craft + " b9 materials", string.Join("; ", b9.FindModelComponents<Renderer>().Where(r => r.sharedMaterial != null)
                        .Select(r => $"{r.name}: {r.sharedMaterial.shader.name} [{string.Join(" ", props.Where(r.sharedMaterial.HasProperty))}]").Distinct()));
                }
                File.WriteAllText(Path.Combine(ctx.ResultsDir, $"wings-{Short(craft)}.json"), Dump(rows));

                foreach (string option in new[] { "Parts", "Wings", "Aspect Ratio" })
                {
                    SelectOption(tint, option);
                    yield return Wait.Frames(5);
                    ctx.Record(craft + " " + option, Get<string>(comp, "legend"));
                    // straight down from above, the colours as the game draws them (no UI), then the window and its table
                    yield return new WaitForEndOfFrame();
                    TopDown(System.IO.Path.Combine(ctx.ResultsDir, $"top-{Short(craft)}-{option.Replace(" ", "")}.png"));
                    if (option == "Wings")
                    {
                        yield return Snap.Shot(ctx, $"{Short(craft)}_table");
                        // a wing panel and its mirror: each renderer, its shader, the colours its block and material have
                        Part left = EditorLogic.fetch.ship.parts.FirstOrDefault(q => q.partInfo.name.Contains("Wing") && q.symmetryCounterparts.Count > 0);
                        if (left != null)
                            foreach (Part q in new[] { left, left.symmetryCounterparts[0] })
                            {
                                var mb = new MaterialPropertyBlock();
                                ctx.Record($"{craft} renderers {q.persistentId}", string.Join("; ", q.FindModelComponents<Renderer>().Select(r =>
                                {
                                    r.GetPropertyBlock(mb);
                                    Material m = r.sharedMaterial;
                                    string c(string n) => m != null && m.HasProperty(n) ? $"{n} mat {m.GetColor(n)} block {mb.GetColor(n)}" : "";
                                    return $"{r.name} on {r.enabled}/{r.gameObject.activeInHierarchy} {(m != null ? m.shader.name : "none")} x{r.sharedMaterials.Length} " +
                                           $"{c("_Color")} {c("_BurnColor")} {c("_EmissiveColor")} {c("_TemperatureColor")}";
                                })));
                            }
                    }
                }
                SelectOption(tint, "None");
                yield return Wait.Frames(3);
                CloseWindow();
            }
        }

        // the panels as FAR measures them (its planform corners), their wing, kind and aspect ratios, and the other parts'
        // model points, in the root part's frame (x across, y forward, z down): for a plan view drawn outside the game
        private static string Dump(List<object> rows)
        {
            Transform root = EditorLogic.RootPart.partTransform;
            string P(Vector3 w)
            {
                Vector3 q = root.InverseTransformPoint(w);
                return string.Format(CultureInfo.InvariantCulture, "[{0:F3},{1:F3},{2:F3}]", q.x, q.y, q.z);
            }
            var sb = new StringBuilder("{\"panels\":[");
            var wingParts = new HashSet<Part>();
            bool first = true;
            foreach (object r in rows)
            {
                var w = (ferram4.FARWingAerodynamicModel)Get(r, "Wing");
                wingParts.Add(w.part);
                var c = new Vector3[4];
                w.PlanformCorners(c);
                sb.Append(first ? "" : ",");
                first = false;
                sb.Append(string.Format(CultureInfo.InvariantCulture,
                                        "{{\"name\":\"{0}\",\"group\":{1},\"own\":{2:F3},\"eff\":{3:F3},\"control\":{4},\"corners\":[{5}]}}",
                                        Get<string>(r, "Name").Replace("\"", "'"), Get<int>(r, "Group"), Get<double>(r, "OwnAR"),
                                        Get<double>(r, "EffectiveAR"), w is ferram4.FARControllableSurface ? "true" : "false",
                                        string.Join(",", c.Select(P))));
            }
            sb.Append("],\"bodies\":[");
            first = true;
            foreach (Part p in EditorLogic.fetch.ship.parts.Where(p => !wingParts.Contains(p)))
            {
                var pts = p.GetComponentsInChildren<MeshFilter>().Where(f => f.sharedMesh != null && f.sharedMesh.isReadable && f.GetComponentInParent<Part>() == p)
                           .SelectMany(f => f.sharedMesh.vertices.Where((v, i) => i % 7 == 0).Select(v => f.transform.TransformPoint(v))).ToList();
                if (pts.Count == 0)
                    continue;
                sb.Append(first ? "" : ",");
                first = false;
                sb.Append("{\"name\":\"" + p.partInfo.name + "\",\"pts\":[" + string.Join(",", pts.Select(P)) + "]}");
            }
            return sb.Append("]}").ToString();
        }

        // the craft from straight above, nose up, an orthographic camera of our own rendered to a PNG (the SPH's own
        // camera cannot look down further than 60 deg)
        private static void TopDown(string path)
        {
            List<Part> parts = EditorLogic.fetch.ship.parts;
            Bounds bounds = default;
            bool first = true;
            Vector3 rootPos = EditorLogic.RootPart.partTransform.position;
            foreach (Renderer r in parts.SelectMany(p => p.FindModelComponents<Renderer>()).Where(r => r.enabled && (r is MeshRenderer || r is SkinnedMeshRenderer) && (r.bounds.center - rootPos).magnitude < 12f))
            {
                if (first)
                {
                    bounds = r.bounds;
                    first = false;
                }
                else
                    bounds.Encapsulate(r.bounds);
            }
            Vector3 nose = Vector3.ProjectOnPlane(EditorLogic.RootPart.partTransform.up, Vector3.up).normalized;
            const int W = 1600, H = 1600;
            var go = new GameObject("FARTestTopDown");
            try
            {
                // no specular shine for the picture (it washes a mirrored panel's colour out to white): each part's own
                // block, its tint kept, the specular colour black
                int spec = Shader.PropertyToID("_SpecColor");
                foreach (Part p in parts)
                {
                    p.mpb.SetColor(spec, Color.black);
                    p.mpb.SetColor(Shader.PropertyToID("_RimColor"), Color.black);
                    foreach (Renderer r in p.FindModelComponents<Renderer>())
                        r.SetPropertyBlock(p.mpb);
                }
                Camera cam = go.AddComponent<Camera>();
                Camera main = Camera.main;
                if (main != null)
                    cam.CopyFrom(main);
                cam.orthographic = true;
                cam.orthographicSize = Mathf.Max(bounds.extents.x, bounds.extents.z) * 1.08f;
                cam.transform.position = bounds.center + Vector3.up * (bounds.extents.y + 6f);
                cam.transform.rotation = Quaternion.LookRotation(Vector3.down, nose);
                cam.nearClipPlane = 0.1f;
                cam.farClipPlane = bounds.extents.y * 2 + 20f;
                cam.clearFlags = CameraClearFlags.SolidColor;
                cam.backgroundColor = new Color(0.82f, 0.84f, 0.86f);
                var rt = new RenderTexture(W, H, 24, RenderTextureFormat.ARGB32);
                cam.targetTexture = rt;
                cam.Render();
                RenderTexture was = RenderTexture.active;
                RenderTexture.active = rt;
                var tex = new Texture2D(W, H, TextureFormat.RGB24, false);
                tex.ReadPixels(new Rect(0, 0, W, H), 0, 0);
                tex.Apply();
                RenderTexture.active = was;
                cam.targetTexture = null;
                File.WriteAllBytes(path, tex.EncodeToPNG());
                UnityEngine.Object.Destroy(tex);
                rt.Release();
                UnityEngine.Object.Destroy(rt);
            }
            finally
            {
                UnityEngine.Object.Destroy(go);
            }
        }

        // the SPH camera high over the craft, looking down at it from a heading (rad)
        private static void Aim(float hdg)
        {
            SPHCamera cam = UnityEngine.Object.FindObjectOfType<SPHCamera>();
            if (cam == null)
                return;
            cam.camPitch = cam.maxPitch * Mathf.Deg2Rad;
            cam.camHdg = hdg;
            Set(cam, "distance", 11f);
        }

        private static string Short(string craft) => craft.Replace("Cessna 172M Floatplane", "c172").Replace(' ', '_');

        private static IEnumerator Recomputed(object comp)
        {
            yield return Wait.Frames(2);
            yield return Wait.Until(() => !Get<bool>(comp, "dirty"), 15, "the Components tab to recompute", shot: false);
            yield return Wait.Frames(2);
        }
    }
}
