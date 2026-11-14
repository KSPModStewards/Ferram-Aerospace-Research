using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using KSP.UI.Screens;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's water model by hand: a toolbar button in flight (a blue circle with a wave) that puts the active vessel on
    /// the water as the water tests place their craft. Where it is, if it is over the sea; else at the tests' site (Kerbin:
    /// 0 N 60 W, east of KSC; RSS's Earth: off Cape Canaveral). Placed on rails at 600 m and back off them at rest, its nose
    /// level along its heading (a vessel built upright, from the VAB, stays upright), lowered until its lowest point
    /// touches the water, held still while the water takes its weight, then let go. The Set Position cheat's ease-in
    /// (its gravity cut to a tenth until the vessel lands, which on the water it never does) is turned off first.
    /// In the test assembly, so it is in any install the tests have been deployed to; shown with FAR's debug tools
    /// (FARDebugValues.DebugTools, the debug options window's "Show Debug and Experimental Tools").
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class FarWaterTools : MonoBehaviour
    {
        private ApplicationLauncherButton button;
        private bool busy;

        private void Start()
        {
            GameEvents.onGUIApplicationLauncherReady.Add(AddButton);
            FARDebugValues.DebugToolsChanged += OnDebugToolsChanged;
            if (ApplicationLauncher.Ready)
                AddButton();
        }

        private void OnDestroy()
        {
            GameEvents.onGUIApplicationLauncherReady.Remove(AddButton);
            FARDebugValues.DebugToolsChanged -= OnDebugToolsChanged;
            RemoveButton();
        }

        private void OnDebugToolsChanged()
        {
            if (FARDebugValues.DebugTools)
                AddButton();
            else
                RemoveButton();
        }

        private void RemoveButton()
        {
            if (button != null && ApplicationLauncher.Instance != null)
                ApplicationLauncher.Instance.RemoveModApplication(button);
            button = null;
        }

        private void AddButton()
        {
            if (button != null || ApplicationLauncher.Instance == null || !FARDebugValues.DebugTools)
                return;
            button = ApplicationLauncher.Instance.AddModApplication(Click, Click, null, null, null, null,
                                                                    ApplicationLauncher.AppScenes.FLIGHT, Icon());
        }

        private void Click()
        {
            button?.SetFalse(false);
            if (!busy)
                StartCoroutine(Run(Flatten(PutOnWater(Say))));
        }

        // runs it as a coroutine, its exceptions reported rather than left in the log
        private IEnumerator Run(IEnumerator e)
        {
            busy = true;
            while (true)
            {
                object current;
                try
                {
                    if (!e.MoveNext())
                        break;
                    current = e.Current;
                }
                catch (Exception x)
                {
                    Say("FAR test: could not put it on the water: " + x.Message);
                    Debug.LogException(x);
                    break;
                }
                yield return current;
            }
            busy = false;
        }

        private static void Say(string s)
        {
            ScreenMessages.PostScreenMessage(s, 6, ScreenMessageStyle.UPPER_CENTER);
            Debug.Log("[FAR test] " + s);
        }

        /// <summary>Nested enumerators (KspTest's waits) run in place, for Unity's coroutine.</summary>
        internal static IEnumerator Flatten(IEnumerator root)
        {
            var stack = new Stack<IEnumerator>();
            stack.Push(root);
            while (stack.Count > 0)
            {
                IEnumerator e = stack.Peek();
                if (!e.MoveNext())
                {
                    stack.Pop();
                    continue;
                }
                if (e.Current is IEnumerator inner)
                {
                    stack.Push(inner);
                    continue;
                }
                yield return e.Current;
            }
        }

        /// <summary>The water tests' site on a body: RSS's Earth off Cape Canaveral, else 0 N 60 W (Kerbin's sea).</summary>
        internal static (double lat, double lon) WaterSite(CelestialBody body) =>
            body.Radius > 1e6 ? (28.45, -80.45) : (0.0, -60.0);

        /// <summary>Puts the active vessel on the water (see the class); say reports what it did.</summary>
        internal static IEnumerator PutOnWater(Action<string> say)
        {
            Vessel v = FlightGlobals.ActiveVessel;
            if (v == null)
                yield break;
            CelestialBody body = v.mainBody;
            if (!body.ocean)
            {
                say($"FAR test: {body.bodyName} has no sea");
                yield break;
            }
            double lat = v.latitude, lon = v.longitude;
            bool here = v.altitude < 20000 && body.TerrainAltitude(lat, lon) < -5;
            if (!here)
                (lat, lon) = WaterSite(body);

            // its nose's heading (a vessel standing up, as the VAB builds them, keeps standing)
            Vector3 up = v.upAxis;
            Vector3 nose = v.ReferenceTransform != null ? v.ReferenceTransform.up : v.transform.up;
            bool upright = Mathf.Abs(Vector3.Dot(nose, up)) > 0.7f;
            Vector3 nh = Vector3.ProjectOnPlane(nose, up);
            double heading = upright || nh.sqrMagnitude < 1e-4f ? 90
                : Math.Atan2(Vector3.Dot(nh, (Vector3)v.east), Vector3.Dot(nh, (Vector3)v.north)) * 180 / Math.PI;

            EaseInOff(v);
            TimeWarp.SetRate(0, true);
            yield return FarFlightProbe.Place(new FarFlightProbe.Condition
            {
                Name = "on the water", AltM = Math.Max(0, body.TerrainAltitude(lat, lon)) + 600, Lat = lat, Lon = lon, HeadingDeg = heading
            });
            v = FlightGlobals.ActiveVessel;
            up = v.upAxis;
            if (upright)
            {
                Transform rt = v.ReferenceTransform != null ? v.ReferenceTransform : v.transform;
                v.SetRotation(Quaternion.FromToRotation(rt.up, up) * v.transform.rotation);
            }
            Still(v, 0);
            yield return new WaitForFixedUpdate();

            // its lowest point (its colliders' bounds) at the water
            double low = double.MaxValue;
            foreach (Part p in v.parts)
                foreach (Collider c in p.GetComponentsInChildren<Collider>())
                {
                    if (!c.enabled || c.isTrigger)
                        continue;
                    Bounds b = c.bounds;
                    for (int i = 0; i < 8; i++)
                    {
                        Vector3 corner = b.center + Vector3.Scale(b.extents, new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
                        low = Math.Min(low, FlightGlobals.getAltitudeAtPos((Vector3d)corner, body));
                    }
                }
            if (low < double.MaxValue)
                v.SetPosition(v.vesselTransform.position - (Vector3d)up * low);
            Still(v, 0);

            // held while the water takes its weight: its motion damped for a few seconds, then let go
            double t0 = Planetarium.GetUniversalTime();
            while (Planetarium.GetUniversalTime() - t0 < 4)
            {
                yield return new WaitForFixedUpdate();
                Still(FlightGlobals.ActiveVessel, 0.9f);
            }
            v = FlightGlobals.ActiveVessel;
            say($"FAR test: {v.vesselName} on the water at {lat:F2}, {lon:F2}{(here ? " (where it was)" : "")}");
        }

        // every part's velocity and spin times keep (0: stopped)
        private static void Still(Vessel v, float keep)
        {
            foreach (Part p in v.parts)
                if (p.Rigidbody != null)
                {
                    p.Rigidbody.velocity *= keep;
                    p.Rigidbody.angularVelocity *= keep;
                }
        }

        /// <summary>The Set Position cheat's ease-in off (FlightGlobals' own, as its "Disable ease-in" does).</summary>
        internal static void EaseInOff(Vessel v)
        {
            if (!v.easingInToSurface)
                return;
            typeof(FlightGlobals).GetMethod("ToggleVesselEaseIn", BindingFlags.NonPublic | BindingFlags.Instance)
                ?.Invoke(FlightGlobals.fetch, new object[] { v, false, 1.0 });
        }

        // a blue disc with a white wave
        private static Texture2D Icon()
        {
            const int n = 38;
            var tex = new Texture2D(n, n, TextureFormat.ARGB32, false);
            var clear = new Color(0, 0, 0, 0);
            var blue = new Color(0.15f, 0.45f, 0.85f, 1);
            for (int y = 0; y < n; y++)
                for (int x = 0; x < n; x++)
                {
                    float dx = x - n / 2f + 0.5f, dy = y - n / 2f + 0.5f;
                    bool disc = dx * dx + dy * dy <= 17 * 17;
                    float wave = n / 2f + 3 * Mathf.Sin(dx / 4f);
                    bool white = disc && Math.Abs(y - wave) < 1.6f && Math.Abs(dx) < 13;
                    tex.SetPixel(x, y, white ? Color.white : disc ? blue : clear);
                }
            tex.Apply();
            return tex;
        }
    }
}
