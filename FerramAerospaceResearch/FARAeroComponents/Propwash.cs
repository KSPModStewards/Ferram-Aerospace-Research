using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    /// <summary>
    /// The slipstream of a vessel's propellers on its wing panels and control surfaces. Each propeller is an actuator
    /// disc (momentum theory): its thrust T over its disc area A speeds the air through it, the far slipstream at
    /// sqrt(V0^2 + 2T/(rho A)) (V0 the airspeed along its axis), building along the axis from half that rise at the disc
    /// (the speed on an actuator disc's axis, vi (1 + d / sqrt(d^2 + R^2)) at d behind it) and narrowing as it speeds up
    /// (the flow through it kept). A panel's share of its planform inside the stream (a grid of points over FAR's
    /// planform) sees the stream's speed added along the axis; the panel's airspeed is the dynamic-pressure-weighted
    /// blend of the two. What it does: a tail in a single-engine propeller's wash keeps its authority and its pitch
    /// damping at taxi, takeoff and hump speeds, where the free stream alone gives it next to none (a floatplane at
    /// 20 mph: about fifteen times the dynamic pressure at full power).
    /// Propellers: AJE's (ModuleEnginesAJEPropeller: its propThrust, kN, and its solver's diameter), at the engine's
    /// thrust transform. No swirl, no mixing of the stream with the air around it (a tail a few diameters behind).
    /// The model options' "Propeller slipstream" (FARDebugValues.propwash) switches it.
    /// </summary>
    internal static class Propwash
    {
        // (the stream is followed this many diameters behind the disc; past that: none)
        public static double MaxDiameters = 8;

        private struct Disc
        {
            public Vector3 Center, Axis;   // Axis: the thrust's direction (forward); the stream runs to -Axis
            public float Radius, Thrust;   // m, N
        }

        private static readonly Dictionary<Guid, (double time, List<Disc> discs)> cache = new Dictionary<Guid, (double, List<Disc>)>();
        private static readonly Dictionary<Type, (FieldInfo thrust, FieldInfo diam, FieldInfo solver, MethodInfo getDiam)> types =
            new Dictionary<Type, (FieldInfo, FieldInfo, FieldInfo, MethodInfo)>();
        private static readonly Vector3[] corners = new Vector3[4];

        private static List<Disc> Discs(Vessel v)
        {
            double now = Time.fixedTime;
            if (cache.TryGetValue(v.id, out var c) && c.time == now)
                return c.discs;
            List<Disc> list = c.discs ?? new List<Disc>();
            list.Clear();
            foreach (Part p in v.parts)
                foreach (PartModule m in p.Modules)
                {
                    if (!(m is ModuleEngines e) || !e.EngineIgnited || e.thrustTransforms == null || e.thrustTransforms.Count == 0)
                        continue;
                    var f = Fields(m.GetType());
                    if (f.thrust == null)
                        continue;
                    double thrust = Convert.ToDouble(f.thrust.GetValue(m)) * 1000;
                    double diam = f.diam != null ? Convert.ToDouble(f.diam.GetValue(m)) : -1;
                    if (!(diam > 0) && f.solver != null && f.getDiam != null)
                    {
                        object solver = f.solver.GetValue(m);
                        if (solver != null)
                            diam = Convert.ToDouble(f.getDiam.Invoke(solver, null));
                    }
                    if (!(thrust > 0) || !(diam > 0))
                        continue;
                    Transform t = e.thrustTransforms[0];
                    list.Add(new Disc { Center = t.position, Axis = -t.forward, Radius = (float)(diam / 2), Thrust = (float)thrust });
                }
            cache[v.id] = (now, list);
            if (cache.Count > 64)
                cache.Clear();
            return list;
        }

        private static (FieldInfo thrust, FieldInfo diam, FieldInfo solver, MethodInfo getDiam) Fields(Type t)
        {
            if (types.TryGetValue(t, out var f))
                return f;
            const BindingFlags all = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
            FieldInfo thrust = null, diam = null, solver = null;
            MethodInfo getDiam = null;
            if (t.Name == "ModuleEnginesAJEPropeller")
            {
                thrust = t.GetField("propThrust", all);
                diam = t.GetField("propDiam", all);
                for (Type b = t; b != null && solver == null; b = b.BaseType)
                    solver = b.GetField("solverProp", all | BindingFlags.DeclaredOnly);
                getDiam = solver?.FieldType.GetMethod("GetDiameter", BindingFlags.Instance | BindingFlags.Public);
            }
            f = (thrust, diam, solver, getDiam);
            types[t] = f;
            return f;
        }

        /// <summary>
        /// The airspeed a wing panel sees (its velocity through the air, as FAR's wing model takes it) with the vessel's
        /// propellers' slipstreams on its planform; the velocity as it is with none reaching it.
        /// </summary>
        public static Vector3d Apply(ferram4.FARWingAerodynamicModel w, Vessel v, Vector3d velocity, double rho)
        {
            if (!FARDebugValues.propwash || v == null || rho <= 0)
                return velocity;
            List<Disc> discs = Discs(v);
            if (discs.Count == 0)
                return velocity;
            w.PlanformCorners(corners);
            // a 4 x 3 grid over the planform (span by chord), each point the middle of its cell
            const int ns = 4, nc = 3;
            double q0 = velocity.sqrMagnitude, qSum = 0;
            Vector3d vSum = Vector3d.zero;
            for (int i = 0; i < ns; i++)
            {
                float s = (i + 0.5f) / ns;
                Vector3 le = Vector3.Lerp(corners[0], corners[3], s), te = Vector3.Lerp(corners[1], corners[2], s);
                for (int j = 0; j < nc; j++)
                {
                    Vector3 pt = Vector3.Lerp(le, te, (j + 0.5f) / nc);
                    Vector3d vp = velocity;
                    foreach (Disc d in discs)
                    {
                        Vector3 r = pt - d.Center;
                        float behind = -Vector3.Dot(r, d.Axis);
                        if (behind <= 0 || behind > MaxDiameters * 2 * d.Radius)
                            continue;
                        double v0 = Math.Max(0, Vector3d.Dot(velocity, d.Axis));
                        double area = Math.PI * d.Radius * d.Radius;
                        double vFar = Math.Sqrt(v0 * v0 + 2 * d.Thrust / (rho * area));
                        double vi = (vFar - v0) / 2;
                        double rise = vi * (1 + behind / Math.Sqrt(behind * behind + d.Radius * d.Radius));
                        double radius = d.Radius * Math.Sqrt((v0 + vi) / Math.Max(1e-3, v0 + rise));
                        if ((r + d.Axis * behind).magnitude > radius)
                            continue;
                        vp += (Vector3d)d.Axis * rise;
                    }
                    vSum += vp;
                    qSum += vp.sqrMagnitude;
                }
            }
            int n = ns * nc;
            if (Math.Abs(qSum / n - q0) < 1e-6 * Math.Max(1, q0))
                return velocity;
            // the points' mean direction, at the speed of their mean dynamic pressure
            Vector3d dir = vSum / n;
            double mag = dir.magnitude;
            return mag > 1e-6 ? dir * (Math.Sqrt(qSum / n) / mag) : velocity;
        }
    }
}
