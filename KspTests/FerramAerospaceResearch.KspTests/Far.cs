using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using KSP.Localization;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// What the tests reach in FAR that it keeps private: the editor window's simulation and vehicle aerodynamics.
    /// Everything else the tests call directly (InternalsVisibleTo).
    /// </summary>
    internal static class Far
    {
        /// <summary>
        /// A mesh's points in world space: its vertices, or for a mesh that cannot be read (its data on the GPU only) the
        /// corners of its bounds, which can; never a point that is not finite. On Linux some drawn mesh of the ComSat Lx
        /// has such points, and one taken into a running Vector3.Min/Max (whose comparisons a NaN fails) drops all the
        /// craft measured before it: its length came out 6.5 m for FAR's (right) 9.05.
        /// </summary>
        public static IEnumerable<Vector3> MeshPoints(MeshFilter mf) => AllMeshPoints(mf).Where(Finite);

        /// <summary>The points of a mesh that are not finite (NaN or infinite, in world space): none in a sound one.</summary>
        public static int NonFinitePoints(MeshFilter mf) => AllMeshPoints(mf).Count(w => !Finite(w));

        private static bool Finite(Vector3 v) => !float.IsNaN(v.x + v.y + v.z) && !float.IsInfinity(v.x + v.y + v.z);

        private static IEnumerable<Vector3> AllMeshPoints(MeshFilter mf)
        {
            Mesh mesh = mf.sharedMesh;
            if (mesh == null)
                yield break;
            if (mesh.isReadable)
            {
                foreach (Vector3 v in mesh.vertices)
                    yield return mf.transform.TransformPoint(v);
                yield break;
            }

            Bounds b = mesh.bounds;
            for (int i = 0; i < 8; i++)
                yield return mf.transform.TransformPoint(new Vector3((i & 1) == 0 ? b.min.x : b.max.x, (i & 2) == 0 ? b.min.y : b.max.y,
                                                                     (i & 4) == 0 ? b.min.z : b.max.z));
        }

        /// <summary>The stock plane most tests fly or analyse.</summary>
        public const string Plane = "Aeris 4A";

        private static T Field<T>(object o, string name) where T : class => Refl.TryGet<T>(o, name);

        // ------------------------------------------------------------------ flight conditions

        /// <summary>
        /// A condition over the sea east of KSC (0 N, 60 W), heading east: the site every flight test is placed at.
        /// A positive speed (m/s) is flown instead of the Mach.
        /// </summary>
        public static FarFlightProbe.Condition At(double mach, double altM, double aoaDeg, string name = null, double speedMs = 0) =>
            new FarFlightProbe.Condition
            {
                Name = name ?? $"M{mach} {altM / 1000:0.#} km {aoaDeg} deg", Mach = mach, SpeedMs = speedMs, AltM = altM,
                AoADeg = aoaDeg, Lat = 0, Lon = -60, HeadingDeg = 90
            };

        // ------------------------------------------------------------------ parts

        /// <summary>The vessel's parts not under the fairing (in its part tree): what it carries, and the rest of the stack above.</summary>
        public static List<Part> PayloadOf(Vessel v, Part fairing)
        {
            var below = new HashSet<Part>();
            var stack = new Stack<Part>();
            stack.Push(fairing);
            while (stack.Count > 0)
            {
                Part p = stack.Pop();
                if (!below.Add(p))
                    continue;
                foreach (Part c in p.children)
                    stack.Push(c);
            }

            return v.parts.Where(p => !below.Contains(p)).ToList();
        }

        /// <summary>The names of the resources a part holds.</summary>
        public static string[] Resources(Part p) => p.Resources.Cast<PartResource>().Select(r => r.resourceName).ToArray();

        /// <summary>Holds liquid fuel and oxidizer, and nothing else: what FAR's stress templates call a rocket fuel tank.</summary>
        public static bool IsRocketTank(Part p)
        {
            string[] res = Resources(p);
            return res.Length == 2 && res.Contains("LiquidFuel") && res.Contains("Oxidizer");
        }

        /// <summary>The index of the value nearest v.</summary>
        public static int IndexNear(double[] x, double v)
        {
            int best = 0;
            for (int i = 1; i < x.Length; i++)
                if (Math.Abs(x[i] - v) < Math.Abs(x[best] - v))
                    best = i;
            return best;
        }

        // ------------------------------------------------------------------ editor

        public static EditorGUI Editor => EditorGUI.Instance ?? throw new UITestFailure("FAR's editor window is not there");

        /// <summary>The editor's instant-condition simulation (the static analysis and the stability derivatives' sim).</summary>
        public static InstantConditionSim EditorSim =>
            Field<InstantConditionSim>(Editor, "_instantSim") ?? throw new UITestFailure("FAR's editor has no _instantSim");

        /// <summary>The editor's vehicle aerodynamics: the voxel model of the craft and its cross-sections.</summary>
        public static VehicleAerodynamics EditorVehicle =>
            Field<VehicleAerodynamics>(Editor, "_vehicleAero") ?? throw new UITestFailure("FAR's editor has no _vehicleAero");

        public static bool EditorReady => EditorGUI.Instance != null && !EditorGUI.Instance.VoxelizationUpdateQueued &&
                                          Field<InstantConditionSim>(EditorGUI.Instance, "_instantSim")?.Ready == true;

        /// <summary>Waits for the craft in the editor to be voxelized and its simulation to have the result.</summary>
        public static IEnumerator WaitForEditorVoxels(double timeoutSec = 90)
        {
            // ready with an empty sim for a moment after a craft is loaded in place of another
            yield return Wait.Until(() => EditorReady && Instant(2, 0.5).Cd > 0, timeoutSec, "FAR's editor voxels and sim", shot: false);
        }

        /// <summary>
        /// FAR's model of the craft done changing: its lift at 2 deg and its voxel volume the same over two seconds, with
        /// no voxelization queued or running. A craft just loaded is voxelized again as its parts finish starting, and
        /// the wings' interactions follow; read before, a craft comes out a few percent off.
        /// </summary>
        public static IEnumerator Settled(string what)
        {
            double cl = double.NaN, vol = double.NaN, lastDcl = 0, lastDvol = 0;
            int notReady = 0, changes = 0;
            float stableSince = float.MaxValue, deadline = UnityEngine.Time.realtimeSinceStartup + 120;
            while (UnityEngine.Time.realtimeSinceStartup < deadline)
            {
                yield return Wait.Seconds(0.5);
                // a voxelization on its way leaves the last model's numbers standing still (a slow one, under coverage
                // beside other KSPs, for tens of seconds): not settled until it is done
                if (!EditorReady || EditorVehicle?.Voxelizing == true)
                {
                    notReady++;
                    stableSince = float.MaxValue;
                    continue;
                }
                double c = Instant(2, 0.5).Cl, v = Volume(EditorVehicle);
                if (double.IsNaN(cl) || Math.Abs(c - cl) > 1e-7 || Math.Abs(v - vol) > 1e-6)
                {
                    changes++;
                    lastDcl = c - cl;
                    lastDvol = v - vol;
                    cl = c;
                    vol = v;
                    stableSince = UnityEngine.Time.realtimeSinceStartup;
                }
                else if (UnityEngine.Time.realtimeSinceStartup - stableSince >= 2)
                    yield break;
            }

            throw new UITestFailure($"FAR's model of {what} never settled (Cl at 2 deg {cl:F5}, volume {vol:F3} m3; " +
                                    $"{changes} changes, the last {lastDcl:G3} and {lastDvol:G3} m3; not ready {notReady} times)");
        }

        /// <summary>The craft in the editor voxelized again (after a setting or part change FAR does not see itself).</summary>
        public static IEnumerator Revoxelize(string what)
        {
            VehicleAerodynamics v = EditorVehicle;
            EditorGUI.RequestUpdateVoxel();
            // the request is taken when the voxelization is queued (Voxelizing set on the way), then the thread runs
            yield return Wait.Until(() => !Editor.VoxelizationUpdateQueued, 60, $"FAR's editor to voxelize ({what})", shot: false);
            yield return Wait.Until(() => !v.Voxelizing, 120, $"FAR's voxelization to finish ({what})", shot: false);
            yield return WaitForEditorVoxels();
            // the wings' interactions follow the voxels: a Cd read straight after was 0.1% off under coverage
            yield return Settled(what);
        }

        /// <summary>The voxel model's volume: its cross-sections times their thickness.</summary>
        public static double Volume(VehicleAerodynamics v) => v.GetCrossSectionAreas().Sum() * v.SectionThickness;

        /// <summary>
        /// The instant sim at one condition, steady (no rates), no flaps or spoilers, the wings' stall from nothing (as FAR's
        /// trim iteration does): kept, a wing near its stall angle creeps further into it on every call.
        /// </summary>
        public static InstantConditionSimOutput Instant(double alphaDeg, double mach, double betaDeg = 0, double pitch = 0)
        {
            var input = new InstantConditionSimInput(alphaDeg, betaDeg, 0, 0, 0, 0, mach, pitch, 0, false);
            EditorSim.GetClCdCmSteady(input, out InstantConditionSimOutput output, true, true);
            return output;
        }

        // ------------------------------------------------------------------ flight GUI

        /// <summary>The flight window's title starts with this ("FAR, " + the version).</summary>
        public const string MainWindow = "FAR, ";

        /// <summary>The flight data window's title (FARFlightDataTitle).</summary>
        public static string DataWindow => Localizer.Format("FARFlightDataTitle");

        /// <summary>
        /// The flight settings window: FlightGUI titles it with the tag FARFlightSettings, which the localization does not
        /// define (it has FARFlightSettingsTitle), so it shows the tag itself; either way the title has "Settings" in it.
        /// </summary>
        public const string SettingsWindow = "Settings";

        /// <summary>The settings window's groups, in FlightGUI's drop-down order.</summary>
        public const int GroupFlightData = 0, GroupStabilityAugmentation = 1, GroupAirspeed = 2, GroupAeroViz = 3;

        private static object Get(object o, string name) => Refl.Get(o, name);

        private static void Set(object o, string name, object value) => Refl.Set(o, name, value);

        /// <summary>
        /// FAR's debug tools (FARDebugValues.DebugTools: the flight window's debug window and flight assistance, the
        /// put-on-water button) on or off for a test, put back as they were when disposed:
        /// <c>using var tools = Far.DebugTools();</c>
        /// </summary>
        public static IDisposable DebugTools(bool on = true)
        {
            bool was = FARDebugValues.DebugTools;
            FARDebugValues.DebugTools = on;
            return new Restore(() => FARDebugValues.DebugTools = was);
        }

        private sealed class Restore : IDisposable
        {
            private Action undo;

            public Restore(Action undo) => this.undo = undo;

            public void Dispose()
            {
                undo?.Invoke();
                undo = null;
            }
        }

        /// <summary>The active vessel's FAR flight GUI (a VesselModule).</summary>
        public static FlightGUI FlightGui(Vessel v = null) =>
            FARAPI.VesselFlightInfo(v ?? FlightGlobals.ActiveVessel) ?? throw new UITestFailure("the vessel has no FAR FlightGUI");

        /// <summary>FAR's flight window shown (FlightGUI.showGUI, what its toolbar button toggles).</summary>
        public static bool FlightGuiShown
        {
            get => (bool)Refl.GetStatic(typeof(FlightGUI), "showGUI");
            set => Refl.SetStatic(typeof(FlightGUI), "showGUI", value);
        }

        /// <summary>Whether one of the flight window's sub-windows is open: showFlightDataWindow, showSettingsWindow, ...</summary>
        public static bool SubWindowOpen(FlightGUI gui, string field) => (bool)Get(gui, field);

        public static void SetSubWindow(FlightGUI gui, string field, bool open) => Set(gui, field, open);

        /// <summary>
        /// Picks the settings window's group. Its drop-down list closes as soon as the mouse is not over it (there is
        /// no mouse in a test), so the selection is set the way the list's button would set it. By the group's value
        /// (GroupFlightData, ...), not its place in the list: flight assistance is only listed with FAR's debug tools.
        /// </summary>
        public static void SetSettingsGroup(FlightGUI gui, int group)
        {
            object dropdown = Get(gui, "settingsWindow");
            int at = Array.IndexOf((int[])Get(dropdown, "typeOptions"), group);
            if (at < 0)
                throw new UITestFailure($"the settings window does not list group {group} (flight assistance needs Far.DebugTools())");
            Set(dropdown, "selectedOption", at);
        }

        /// <summary>The settings window's group (GroupFlightData, ...).</summary>
        public static int SettingsGroup(FlightGUI gui)
        {
            object dropdown = Get(gui, "settingsWindow");
            return ((int[])Get(dropdown, "typeOptions"))[(int)Get(dropdown, "selectedOption")];
        }

        /// <summary>The text of the flight data window's value column, as last drawn.</summary>
        public static string FlightDataText(FlightGUI gui) => ((StringBuilder)Get(Get(gui, "_flightDataGUI"), "dataStringBuilder")).ToString();

        /// <summary>The flight data window's section switches (FlightDataGUI.activeFlightDataSections), live.</summary>
        public static bool[] FlightDataSections(FlightGUI gui) => (bool[])Get(Get(gui, "_flightDataGUI"), "activeFlightDataSections");

        /// <summary>What the flight window's status box says (FlightStatusGUI.statusString).</summary>
        public static string FlightStatus(FlightGUI gui) => (string)Get(Get(gui, "_flightStatusGUI"), "statusString");

        // StabilityAugmentation.systemInstances[i]: 0 roll (wing leveler), 1 yaw, 2 pitch, 3 AoA limiter, 4 DPCR
        private static object StabSystem(FlightGUI gui, int i) => ((Array)Get(Get(gui, "_stabilityAugmentation"), "systemInstances")).GetValue(i);

        public static bool StabAugActive(FlightGUI gui, int system) => (bool)Get(StabSystem(gui, system), "active");

        public static void SetStabAug(FlightGUI gui, int system, bool on) => Set(StabSystem(gui, system), "active", on);

        /// <summary>The airspeed setting's mode and unit (enum values), to put back after a test.</summary>
        public static object[] AirspeedSetting(AirspeedSettingsGUI a) => new[] { Get(a, "velMode"), Get(a, "unitMode") };

        public static void SetAirspeedSetting(AirspeedSettingsGUI a, object[] saved)
        {
            Set(a, "velMode", saved[0]);
            Set(a, "unitMode", saved[1]);
        }

        /// <summary>Switches the aero visualization's tints (their setters are private: the GUI's toggles set them).</summary>
        public static void SetTints(AeroVisualizationGUI viz, bool cl, bool cd, bool stall)
        {
            typeof(AeroVisualizationGUI).GetProperty(nameof(AeroVisualizationGUI.TintForCl))?.SetValue(viz, cl, null);
            typeof(AeroVisualizationGUI).GetProperty(nameof(AeroVisualizationGUI.TintForCd))?.SetValue(viz, cd, null);
            typeof(AeroVisualizationGUI).GetProperty(nameof(AeroVisualizationGUI.TintForStall))?.SetValue(viz, stall, null);
        }

        /// <summary>The vessel's flight data logger (FlightGUI.flightDataLogger).</summary>
        public static FlightDataLogger Logger(FlightGUI gui) =>
            Field<FlightDataLogger>(gui, "flightDataLogger") ?? throw new UITestFailure("FAR's FlightGUI has no flightDataLogger");

        /// <summary>
        /// The tint FAR's aero visualization last gave a part (alpha 0 untinted): what the part's FARAeroPartModule last
        /// set on its MaterialColorUpdater (stock's, which writes it to the renderers' _TemperatureColor), else the most
        /// opaque _TemperatureColor on the part's renderers.
        /// </summary>
        public static Color PartTint(Part p)
        {
            FARAeroPartModule aero = p.Modules.GetModule<FARAeroPartModule>();
            MaterialColorUpdater updater = aero != null ? Field<MaterialColorUpdater>(aero, "materialColorUpdater") : null;
            // the colour it was last given (stock keeps it in a non-public field)
            if (updater != null && Refl.Find(updater.GetType(), "setColor")?.GetValue(updater) is Color set)
                return set;
            Color best = new Color(0, 0, 0, 0);
            var block = new MaterialPropertyBlock();
            foreach (Renderer r in p.GetComponentsInChildren<Renderer>())
            {
                r.GetPropertyBlock(block);
                Color c = block.GetColor(PhysicsGlobals.TemperaturePropertyID);
                if (c.a > best.a) best = c;
            }

            return best;
        }
    }
}
