using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Reflection;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// A Spine part's collider at a level of detail: its points per ring (CustomProceduralParts' colliderPoints), which
    /// is what FAR's water model reads its hull from. A model whose results go with how finely the hull is cut is wrong
    /// at one detail or the other: the tow tests run at several (parameter details, collider points per ring,
    /// '+'-separated; 0: as the craft has it) and hold each level's results to the others'.
    /// </summary>
    internal static class ColliderDetail
    {
        /// <summary>
        /// The levels to run at: parameter details, else levelsDefault; the craft's alone (0) when the vessel has no
        /// Spine part (nothing to change) or none are given.
        /// </summary>
        public static int[] Levels(TestContext ctx, string levelsDefault, Vessel v)
        {
            int[] levels = ctx.ParamList("details", levelsDefault).Where(x => x.Length > 0)
                              .Select(x => int.Parse(x, CultureInfo.InvariantCulture)).ToArray();
            bool spine = v.parts.Any(p => p.Modules.Cast<PartModule>().Any(m => m.GetType().Name == "ProceduralShapeSpine" && m.isEnabled));
            return levels.Length > 0 && spine ? levels : new[] { 0 };
        }

        /// <summary>
        /// Every Spine part of the vessel rebuilt with points collider points per ring (its section's points raised to
        /// at least as many, where they are fewer), FAR's meshes of it rebuilt; 0 leaves the craft's. Fails the test when
        /// CustomProceduralParts has no colliderPoints (a build before it). Hyper-warp goes off (its caller turns it on
        /// again after): the frames the rebuild takes, 16 physics steps each at 16x, left a hull massless and its state NaN.
        /// </summary>
        public static IEnumerator Set(TestContext ctx, Vessel v, int points)
        {
            if (points <= 0)
                yield break;
            Flight.HyperWarpOff();
            var changed = new List<string>();
            // each part's CoM offset kept: a test may have set it (HullParts.SetCoM), and a rebuilt shape resets its own
            var coms = v.parts.ToDictionary(p => p, p => p.CoMOffset);
            foreach (Part p in v.parts)
                foreach (PartModule m in p.Modules)
                {
                    Type t = m.GetType();
                    // (every ProceduralParts part carries every shape's module, the unused ones disabled: rebuilt, one
                    // would put its default Spine's collider on a cone)
                    if (t.Name != "ProceduralShapeSpine" || !m.isEnabled)
                        continue;
                    FieldInfo cp = t.GetField("colliderPoints", BindingFlags.Public | BindingFlags.Instance);
                    Assert.True(cp != null, "CustomProceduralParts with colliderPoints (the Spine collider's detail)");
                    FieldInfo sp = t.GetField("sectionPoints", BindingFlags.Public | BindingFlags.Instance);
                    if (sp != null && (int)sp.GetValue(m) < points)
                    {
                        // its next choice at or above points (CustomProceduralParts' PointChoices), at most its largest
                        var choices = t.GetField("PointChoices", BindingFlags.NonPublic | BindingFlags.Static)?.GetValue(null) as int[]
                                      ?? new[] { 24, 32, 48, 64, 96 };
                        sp.SetValue(m, choices.Where(c => c >= points).DefaultIfEmpty(choices.Max()).Min());
                    }
                    cp.SetValue(m, points);
                    t.GetMethod("UpdateShape", new[] { typeof(bool) })?.Invoke(m, new object[] { true });
                    p.SendMessage("GeometryPartModuleRebuildMeshData", SendMessageOptions.DontRequireReceiver);
                    changed.Add($"{p.partInfo.name} ({(sp != null ? sp.GetValue(m) : "?")} section points)");
                }
            ctx.Record($"collider {points}", $"{points} points per ring: " + (changed.Count > 0 ? string.Join(", ", changed) : "no Spine part"));
            yield return Wait.Frames(5);
            foreach (var kv in coms.Where(kv => kv.Key != null))
            {
                kv.Key.CoMOffset = kv.Value;
                if (kv.Key.rb != null)
                    kv.Key.rb.centerOfMass = kv.Value;
            }
        }
    }
}
