/*
Ferram Aerospace Research v0.16.2.0 "Maskell"
=========================
Aerodynamics model for Kerbal Space Program

Copyright 2022, Michael Ferrara, aka Ferram4

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Ferram Aerospace Research is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Ferram Aerospace Research.  If not, see <http://www.gnu.org/licenses/>.

   Serious thanks:		a.g., for tons of bugfixes and code-refactorings
				stupid_chris, for the RealChuteLite implementation
            			Taverius, for correcting a ton of incorrect values
				Tetryds, for finding lots of bugs and issues and not letting me get away with them, and work on example crafts
            			sarbian, for refactoring code for working with MechJeb, and the Module Manager updates
            			ialdabaoth (who is awesome), who originally created Module Manager
                        	Regex, for adding RPM support
				DaMichel, for some ferramGraph updates and some control surface-related features
            			Duxwing, for copy editing the readme

   CompatibilityChecker by Majiir, BSD 2-clause http://opensource.org/licenses/BSD-2-Clause

   Part.cfg changes powered by sarbian & ialdabaoth's ModuleManager plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/55219

   ModularFLightIntegrator by Sarbian, Starwaster and Ferram4, MIT: http://opensource.org/licenses/MIT
	http://forum.kerbalspaceprogram.com/threads/118088

   Toolbar integration powered by blizzy78's Toolbar plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/60863
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARPartGeometry.GeometryModification;
using FerramAerospaceResearch.Settings;
using KSP.UI.Screens;
using KSPCommunityFixes;
using TweakScale;
using UnityEngine;

namespace FerramAerospaceResearch.FARPartGeometry
{
    internal struct GeometryPartModuleConfig
    {
        public bool forceUseColliders;

        public bool forceUseMeshes;

        public bool ignoreForMainAxis;

        public List<string> ignoredTransforms, unignoredTransforms;

        public bool ignoreIfNoRenderer;

        public bool rebuildOnAnimation;

        public GeometryPartModuleConfig()
        {
            forceUseColliders = false;
            forceUseMeshes = false;
            ignoreForMainAxis = false;
            ignoredTransforms = new List<string>();
            unignoredTransforms = new List<string>();
            ignoreIfNoRenderer = true;
            rebuildOnAnimation = false;
        }
    }

    public class GeometryPartModule : PartModule, IRescalable<GeometryPartModule>
    {
        private static int ignoreLayer0 = -1;
        public bool destroyed;

        // ReSharper disable once NotAccessedField.Global -> unity
        public Transform partTransform;

        // ReSharper disable once NotAccessedField.Global -> unity
        public Rigidbody partRigidBody;

        public Bounds overallMeshBounds;
        public Bounds localMeshBounds;

        public List<GeometryMesh> meshDataList;
        private List<IGeometryUpdater> geometryUpdaters;
        private List<ICrossSectionAdjuster> crossSectionAdjusters;

        private List<AnimationState> animStates;
        private List<float> animStateTime;

        private bool _started;
        private bool _ready;
        private bool _sceneSetup;

        private int _sendUpdateTick;
        private int _meshesToUpdate = -1;

        private GeometryPartModuleConfig config = new();

        public bool HasCrossSectionAdjusters
        {
            get
            {
                if (crossSectionAdjusters == null)
                    return false;

                return crossSectionAdjusters.Count > 0;
            }
        }

        public double MaxCrossSectionAdjusterArea
        {
            get
            {
                if (crossSectionAdjusters == null)
                    return 0;

                double value = 0;

                foreach (ICrossSectionAdjuster adjuster in crossSectionAdjusters)
                {
                    double tmp = Math.Abs(adjuster.AreaRemovedFromCrossSection());
                    if (tmp > value)
                        value = tmp;
                }

                return value;
            }
        }

        public bool Ready
        {
            get { return _ready && _started && _sceneSetup; }
        }

        public bool Valid { get; private set; } = true;


        public bool IgnoreForMainAxis
        {
            get { return config.ignoreForMainAxis; }
        }

        public void OnRescale(ScalingFactor factor)
        {
            if (meshDataList == null)
                return;

            Rescale(factor.absolute.linear * Vector3.one);
        }

        private void DebugAddMesh(Transform t)
        {
            if (FARLogger.IsEnabledFor(LogLevel.Debug))
                debugInfo.meshes.Add(t.name);
        }

        private void DebugAddCollider(Transform t)
        {
            if (FARLogger.IsEnabledFor(LogLevel.Debug))
                debugInfo.colliders.Add(t.name);
        }

        private void DebugAddNoRenderer(Transform t)
        {
            if (FARLogger.IsEnabledFor(LogLevel.Debug))
                debugInfo.noRenderer.Add(t.name);
        }

        private void DebugClear()
        {
            if (FARLogger.IsEnabledFor(LogLevel.Debug))
                debugInfo.Clear();
        }

        private void DebugPrint()
        {
            if (FARLogger.IsEnabledFor(LogLevel.Debug))
                debugInfo.Print(part);
        }

        private void OnEditorAttach()
        {
            if (meshDataList is null || meshDataList.Count == 0)
            {
                RebuildAllMeshData();
            }
        }

        public override void OnAwake()
        {
            base.OnAwake();

            // Part has connected member that would work but it is not public...
            part.OnEditorAttach += OnEditorAttach;
        }

        private void Start()
        {
            destroyed = false;

            //RebuildAllMeshData();
            SetupIGeometryUpdaters();
            SetupICrossSectionAdjusters();
            GetAnimations();
        }

        private void OnDestroy()
        {
            meshDataList = null;
            geometryUpdaters = null;
            crossSectionAdjusters = null;
            animStates = null;
            animStateTime = null;
            destroyed = true;

            part.OnEditorAttach -= OnEditorAttach;
        }

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            _sceneSetup = true; //this exists only to ensure that OnStart has occurred first
            if (ignoreLayer0 < 0)
                ignoreLayer0 = LayerMask.NameToLayer("TransparentFX");

            GeometryPartModule modulePrefab = part.partInfo.partPrefab.FindModuleImplementingFast<GeometryPartModule>();
            if (modulePrefab is not null)
                config = modulePrefab.config;

            if (part.collider == null &&
                !part.HasModuleImplementingFast<ModuleWheelBase>() &&
                !part.HasModuleImplementingFast<KerbalEVA>() &&
                !part.HasModuleImplementingFast<FlagSite>())
                return;

            if (HighLogic.LoadedSceneIsEditor)
                StartCoroutine(DoRebuildMeshEditor());
            else if (HighLogic.LoadedSceneIsFlight)
                StartCoroutine(DoRebuildMeshFlight());
        }

        private IEnumerator DoRebuildMeshFlight()
        {
            var waiter = new WaitForFixedUpdate();

            while (!FlightGlobals.ready)
                yield return waiter;

            // have to wait for the vessel to be loaded fully so that unused model transforms are disabled before
            // gathering meshes for voxelization
            while (part.vessel.HoldPhysics)
                yield return waiter;

            RebuildAllMeshData();
        }

        private IEnumerator DoRebuildMeshEditor()
        {
            var waiter = new WaitForFixedUpdate();

            // skip a physics frame to allow part setup to complete
            yield return waiter;

            while (!ApplicationLauncher.Ready)
                yield return waiter;

            // don't voxelize until the part is placed
            while (EditorLogic.SelectedPart == part)
                yield return waiter;

            // skip if not in the ship, OnEditorAttach will rebuild the mesh data
            if (!EditorLogic.SortedShipList.Contains(part))
                yield break;

            RebuildAllMeshData();
        }

        private void FixedUpdate()
        {
            if (!_ready && _meshesToUpdate == 0)
            {
                (localMeshBounds, overallMeshBounds) = SetBoundsFromMeshes();

                // @DRVeyl: Force all cubes to have the same bounds.  Do this any time you recalculate a mesh
                // (ie when handling animations since this breaks the cubes for anything other than the *current* cube.)
                foreach (DragCube cube in part.DragCubes.Cubes)
                {
                    cube.Size = localMeshBounds.size;
                    cube.Center = localMeshBounds.center;
                }

                part.DragCubes.ForceUpdate(true, true);

                _ready = true;
            }

            if (animStates != null && animStates.Count > 0)
                CheckAnimations();
        }

        public void ClearMeshData()
        {
            meshDataList = null;
            _ready = false;
        }

        public void GeometryPartModuleRebuildMeshData()
        {
            // skip parts that have been picked up, picking up should not invalidate previous voxelization
            if (part.gameObject.layer == ignoreLayer0)
                return;

            RebuildAllMeshData();
            UpdateVoxelShape();
        }

        internal void RebuildAllMeshData()
        {
            if (!(HighLogic.LoadedSceneIsFlight || HighLogic.LoadedSceneIsEditor))
                return;

            _ready = false;

            //if the previous transform order hasn't been completed yet, wait here to let it
            while (_meshesToUpdate > 0)
                if (this == null)
                    return;

            partTransform = part.partTransform;
            List<Transform> meshTransforms = part.PartModelTransformList();
            meshTransforms.RemoveAll(IgnoredPredicate);
            List<MeshData> geometryMeshes = CreateMeshListFromTransforms(ref meshTransforms);

            meshDataList = new List<GeometryMesh>();

            Matrix4x4 worldToVesselMatrix = HighLogic.LoadedSceneIsFlight
                                                ? vessel.vesselTransform.worldToLocalMatrix
                                                : EditorLogic.RootPart.partTransform.worldToLocalMatrix;
            for (int i = 0; i < meshTransforms.Count; ++i)
            {
                MeshData m = geometryMeshes[i];
                if (m.vertices.Length <= 0)
                {
                    geometryMeshes.RemoveAt(i);
                    meshTransforms.RemoveAt(i);
                    --i;
                    continue;
                }

                var geoMesh = new GeometryMesh(m, meshTransforms[i], worldToVesselMatrix, this);
                meshDataList.Add(geoMesh);
            }

            _meshesToUpdate = 0;
            _started = true;
        }

        private bool IgnoredPredicate(Transform t)
        {
            if (config.unignoredTransforms.Contains(t.name))
                return false;

            if (config.ignoredTransforms.Contains(t.name))
                return true;

            if (t.gameObject.layer != ignoreLayer0)
            {
                Transform prefabTransform = part.partInfo.partPrefab?.FindModelTransform(t.gameObject.name);
                if (prefabTransform is null || prefabTransform.gameObject.layer != ignoreLayer0)
                    return false;
            }

            FARLogger.DebugFormat("Ignoring {2} ({0}::{1}) for voxelization: layer is ignored",
                                  part.name,
                                  t.gameObject.name,
                                  t.name);
            return true;
        }

        private (Bounds, Bounds) SetBoundsFromMeshes()
        {
            if (meshDataList.Count == 0)
            {
                // If the mesh is empty, try rebuilding it. This can happen when a part was added to the editor but not
                // the ship before adding it to the ship
                RebuildAllMeshData();
            }

            Vector3 upper = Vector3.one * float.NegativeInfinity, lower = Vector3.one * float.PositiveInfinity;
            Vector3 upperLocal = Vector3.one * float.NegativeInfinity,
                    lowerLocal = Vector3.one * float.PositiveInfinity;
            foreach (GeometryMesh geoMesh in meshDataList)
            {
                if (!geoMesh.valid)
                    continue;

                upper = Vector3.Max(upper, geoMesh.bounds.max);
                lower = Vector3.Min(lower, geoMesh.bounds.min);

                Bounds meshBounds = GeometryMesh.TransformBounds(geoMesh.meshLocalBounds, geoMesh.meshLocalToPart);
                upperLocal = Vector3.Max(upperLocal, meshBounds.max);
                lowerLocal = Vector3.Min(lowerLocal, meshBounds.min);
            }

            var overallBounds = new Bounds((upper + lower) * 0.5f, upper - lower);
            var localBounds = new Bounds((upperLocal + lowerLocal) * 0.5f, upperLocal - lowerLocal);

            float tmpTestBounds = overallBounds.center.x +
                                  overallBounds.center.y +
                                  overallBounds.center.z +
                                  overallBounds.extents.x +
                                  overallBounds.extents.y +
                                  overallBounds.extents.z;
            if (float.IsNaN(tmpTestBounds) || float.IsInfinity(tmpTestBounds))
            {
                FARLogger.Info("Overall bounds error in " + part.partInfo.title + " " + meshDataList.Count + " meshes");
                Valid = false;
            }
            else
            {
                Valid = true;
            }

            return (localBounds, overallBounds);
        }

        private void GetAnimations()
        {
            Animation[] animations = part.FindModelAnimators();

            if (animations.Length == 0)
                return;

            animStates = new List<AnimationState>();
            animStateTime = new List<float>();

            foreach (PartModule m in part.Modules)
            {
                FindAnimStatesInModule(animations, m, "animationName");
                FindAnimStatesInModule(animations, m, "animationStateName");
                FindAnimStatesInModule(animations, m, "animName");
                FindAnimStatesInModule(animations, m, "deployAnimationName");
            }
        }

        private void FindAnimStatesInModule(Animation[] animations, PartModule m, string fieldName)
        {
            if (FARAnimOverrides.FieldNameForModule(m.moduleName) == fieldName)
                return;
            FieldInfo field = m.GetType().GetField(fieldName);
            if (field == null)
                return;
            //This handles stock and Firespitter deployment animations
            string animationName = (string)field.GetValue(m);
            foreach (Animation anim in animations)
            {
                if (anim == null)
                    continue;
                AnimationState state = anim[animationName];
                if (!state)
                    continue;
                animStates.Add(state);
                animStateTime.Add(state.time);
            }
        }

        private void SetupIGeometryUpdaters()
        {
            geometryUpdaters = new List<IGeometryUpdater>();
            if (part is CompoundPart compoundPart)
            {
                var compoundUpdate = new CompoundPartGeoUpdater(compoundPart, this);
                geometryUpdaters.Add(compoundUpdate);
            }

            if (part.FindModulesImplementingReadOnly<ModuleProceduralFairing>() is List<PartModule> pmList &&
                pmList.Count > 0)
            {
                foreach (ModuleProceduralFairing fairing in pmList)
                {
                    var fairingUpdater = new StockProcFairingGeoUpdater(fairing, this);
                    geometryUpdaters.Add(fairingUpdater);
                }
            }

            if (part.FindModulesImplementingReadOnly<ModuleJettison>() is List<PartModule> pmList2 &&
                pmList2.Count > 0)
            {
                foreach (ModuleJettison engineFairing in pmList2)
                {
                    var fairingUpdater = new StockJettisonTransformGeoUpdater(engineFairing, this);
                    geometryUpdaters.Add(fairingUpdater);
                }
            }

            if (!part.HasModuleImplementingFast<ModuleAsteroid>())
                return;
            var asteroidUpdater = new StockProcAsteroidGeoUpdater(this);
            geometryUpdaters.Add(asteroidUpdater);
        }

        private void SetupICrossSectionAdjusters()
        {
            Matrix4x4 worldToVesselMatrix = HighLogic.LoadedSceneIsFlight
                                                ? vessel.vesselTransform.worldToLocalMatrix
                                                : EditorLogic.RootPart.partTransform.worldToLocalMatrix;
            crossSectionAdjusters = new List<ICrossSectionAdjuster>();

            string intakeType = "", engineType = "";

            //hard-coded support for AJE; TODO: separate out for more configurable compatibility on 3rd-party end
            if (part.Modules.Contains("ModuleEnginesAJEJet"))
                engineType = "ModuleEnginesAJEJet";
            else if (part.Modules.Contains("ModuleEngines"))
                engineType = "ModuleEngines";
            else if (part.Modules.Contains("ModuleEnginesFX"))
                engineType = "ModuleEnginesFX";


            if (part.Modules.Contains("AJEInlet"))
                intakeType = "AJEInlet";
            else if (part.Modules.Contains("ModuleResourceIntake"))
                intakeType = "ModuleResourceIntake";

            if (intakeType != "" && engineType != "")
            {
                PartModule module = part.Modules[intakeType];

                if (module is ModuleResourceIntake intake)
                {
                    var intakeAdjuster =
                        IntegratedIntakeEngineCrossSectionAdjuster.CreateAdjuster(intake, worldToVesselMatrix);
                    crossSectionAdjusters.Add(intakeAdjuster);
                }
                else
                {
                    var intakeAdjuster =
                        IntegratedIntakeEngineCrossSectionAdjuster.CreateAdjuster(module, worldToVesselMatrix);
                    crossSectionAdjusters.Add(intakeAdjuster);
                }

                return;
            }

            if (intakeType != "")
            {
                PartModule module = part.Modules[intakeType];

                if (module is ModuleResourceIntake intake)
                {
                    var intakeAdjuster = IntakeCrossSectionAdjuster.CreateAdjuster(intake, worldToVesselMatrix);
                    crossSectionAdjusters.Add(intakeAdjuster);
                }
                else
                {
                    var intakeAdjuster = IntakeCrossSectionAdjuster.CreateAdjuster(module, worldToVesselMatrix);
                    crossSectionAdjusters.Add(intakeAdjuster);
                }

                return;
            }

            if (engineType == "")
                return;
            var engines = (ModuleEngines)part.Modules[engineType];
            bool airBreather = false;

            if (engineType == "ModuleEnginesAJEJet")
                airBreather = true;
            else if (engines.propellants.Any(p => p.name == "IntakeAir"))
                airBreather = true;

            if (!airBreather)
                return;
            var engineAdjuster = new AirbreathingEngineCrossSectionAdjuster(engines, worldToVesselMatrix);
            crossSectionAdjusters.Add(engineAdjuster);
        }

        public void RunIGeometryUpdaters()
        {
            if (HighLogic.LoadedSceneIsEditor)
                foreach (IGeometryUpdater geoUpdater in geometryUpdaters)
                    geoUpdater.EditorGeometryUpdate();
            else if (HighLogic.LoadedSceneIsFlight)
                foreach (IGeometryUpdater geoUpdater in geometryUpdaters)
                    geoUpdater.FlightGeometryUpdate();
        }

        public void GetICrossSectionAdjusters(
            List<ICrossSectionAdjuster> activeAdjusters,
            Matrix4x4 basis,
            Vector3 vehicleMainAxis
        )
        {
            if (crossSectionAdjusters == null)
                return;

            foreach (ICrossSectionAdjuster adjuster in crossSectionAdjusters)
                if (!adjuster.AreaRemovedFromCrossSection(vehicleMainAxis).NearlyEqual(0))
                {
                    adjuster.SetForwardBackwardNoFlowDirection(1);
                    activeAdjusters.Add(adjuster);
                }
                else if (!adjuster.AreaRemovedFromCrossSection(-vehicleMainAxis).NearlyEqual(0))
                {
                    adjuster.SetForwardBackwardNoFlowDirection(-1);
                    activeAdjusters.Add(adjuster);
                }
                else
                {
                    adjuster.SetForwardBackwardNoFlowDirection(0);
                }
        }

        private void CheckAnimations()
        {
            bool updateShape = false;
            if (_sendUpdateTick > 30)
            {
                _sendUpdateTick = 0;
                for (int i = 0; i < animStates.Count; ++i)
                {
                    AnimationState state = animStates[i];
                    if (state == null)
                    {
                        animStates.RemoveAt(i);
                        animStateTime.RemoveAt(i);
                        --i;
                        continue;
                    }

                    float prevNormTime = animStateTime[i];

                    //if the anim is not playing, but it was, also send the event to be sure that we closed
                    if (Math.Abs(prevNormTime - state.time) <= 10E-5)
                        continue;
                    animStateTime[i] = state.time;
                    updateShape = true;
                }
            }
            else
            {
                ++_sendUpdateTick;
            }

            if (!updateShape)
                return;
            if (config.rebuildOnAnimation)
                RebuildAllMeshData();
            else
                //event to update voxel, with rate limiter for computer's sanity and error reduction
                UpdateShapeWithAnims();
            UpdateVoxelShape();
        }

        private void UpdateShapeWithAnims()
        {
            Matrix4x4 transformMatrix = HighLogic.LoadedSceneIsFlight
                                            ? vessel.vesselTransform.worldToLocalMatrix
                                            : EditorLogic.RootPart.partTransform.worldToLocalMatrix;

            UpdateTransformMatrixList(transformMatrix);
        }

        private void UpdateVoxelShape()
        {
            if (HighLogic.LoadedSceneIsFlight)
                vessel.SendMessage("AnimationVoxelUpdate");
            else if (HighLogic.LoadedSceneIsEditor)
                EditorGUI.RequestUpdateVoxel();
        }

        public void EditorUpdate()
        {
            Matrix4x4 transformMatrix = EditorLogic.RootPart.partTransform.worldToLocalMatrix;
            UpdateTransformMatrixList(transformMatrix);
        }

        public void UpdateTransformMatrixList(Matrix4x4 worldToVesselMatrix)
        {
            if (meshDataList != null)
            {
                _ready = false;
                //if the previous transform order hasn't been completed yet, wait here to let it
                while (_meshesToUpdate > 0)
                    if (this == null)
                        return;
                _ready = false;

                _meshesToUpdate = meshDataList.Count;
                for (int i = 0; i < meshDataList.Count; ++i)
                {
                    GeometryMesh mesh = meshDataList[i];
                    if (mesh.TrySetThisToVesselMatrixForTransform())
                    {
                        mesh.TransformBasis(worldToVesselMatrix);
                    }
                    else
                    {
                        FARLogger.Info("A mesh on " + part.partInfo.title + " did not exist and was removed");
                        meshDataList.RemoveAt(i);
                        --i;
                        lock (this)
                        {
                            --_meshesToUpdate;
                        }
                    }
                }
            }

            if (crossSectionAdjusters == null)
                return;
            foreach (ICrossSectionAdjuster adjuster in crossSectionAdjusters)
            {
                adjuster.SetThisToVesselMatrixForTransform();
                adjuster.TransformBasis(worldToVesselMatrix);
                adjuster.UpdateArea();
            }
        }

        internal void DecrementMeshesToUpdate()
        {
            lock (this)
            {
                --_meshesToUpdate;
                if (_meshesToUpdate < 0)
                    _meshesToUpdate = 0;
            }
        }

        private static MeshData GetColliderMeshData(Transform t)
        {
            MeshCollider mc = t.GetComponent<MeshCollider>();
            if (mc != null)
            {
                //we can't used mc.sharedMesh because it does not contain all the triangles or verts for some reason
                //must instead get the mesh filter and use its shared mesh

                MeshFilter mf = t.GetComponent<MeshFilter>();
                if (mf != null)
                {
                    Mesh m = mf.sharedMesh;

                    if (m != null)
                        return new MeshData(m.vertices, m.triangles, m.bounds);
                }
                else
                {
                    //but if we can't, grab the sharedMesh anyway and try to use that
                    Mesh m = mc.sharedMesh;

                    if (m != null)
                        return new MeshData(m.vertices, m.triangles, m.bounds);
                }
            }
            else
            {
                BoxCollider bc = t.GetComponent<BoxCollider>();
                if (bc != null)
                    return CreateBoxMeshFromBoxCollider(bc.size, bc.center);
            }

            return null;
        }

        private MeshData GetVisibleMeshData(Transform t, bool skipIfNoRenderer, bool onlyMeshes)
        {
            Mesh m;
            MeshFilter mf = t.GetComponent<MeshFilter>();

            //if we've decided to force use of meshes, we don't want colliders
            if (onlyMeshes && t.GetComponent<MeshCollider>() != null)
                return null;

            if (mf != null)
            {
                if (skipIfNoRenderer && !config.unignoredTransforms.Contains(t.name))
                {
                    MeshRenderer mr = t.GetComponent<MeshRenderer>();
                    if (mr == null)
                    {
                        DebugAddNoRenderer(t);
                        return null;
                    }
                }

                m = mf.sharedMesh;

                if (part.HasModuleImplementingFast<ModuleProceduralFairing>() || part.HasModuleImplementingFast<ModuleAsteroid>())
                    return new MeshData(m.vertices, m.triangles, m.bounds);

                return new MeshData(m.vertices, m.triangles, m.bounds);
            }

            SkinnedMeshRenderer smr = t.GetComponent<SkinnedMeshRenderer>();
            if (smr == null)
                return null;
            m = new Mesh();
            smr.BakeMesh(m);
            var md = new MeshData(m.vertices, m.triangles, m.bounds, true);

            Destroy(m); //ensure that no memory is left over
            return md;
        }

        private List<MeshData> CreateMeshListFromTransforms(ref List<Transform> meshTransforms)
        {
            DebugClear();
            var meshList = new List<MeshData>();
            var validTransformList = new List<Transform>();

            if (part.HasModuleImplementingFast<KerbalEVA>() || part.HasModuleImplementingFast<FlagSite>())
            {
                FARLogger.Info("Adding vox box to Kerbal / Flag");
                meshList.Add(CreateBoxMeshForKerbalEVA());
                validTransformList.Add(part.partTransform);
                meshTransforms = validTransformList;
                return meshList;
            }

            Matrix4x4 worldToLocalMatrix = part.partTransform.worldToLocalMatrix;
            Bounds rendererBounds = part.GetPartOverallMeshBoundsInBasis(worldToLocalMatrix);
            Bounds colliderBounds = part.GetPartColliderBoundsInBasis(worldToLocalMatrix);

            bool cantUseColliders = true;
            bool isFairing = part.HasModuleImplementingFast<ModuleProceduralFairing>() ||
                             part.Modules.Contains("ProceduralFairingSide");
            bool isDrill = part.HasModuleImplementingFast<ModuleAsteroidDrill>() ||
                           part.HasModuleImplementingFast<ModuleResourceHarvester>();

            //Voxelize colliders
            if ((config.forceUseColliders ||
                 isFairing ||
                 isDrill ||
                 rendererBounds.size.x * rendererBounds.size.z < colliderBounds.size.x * colliderBounds.size.z * 1.6f &&
                 rendererBounds.size.y < colliderBounds.size.y * 1.2f &&
                 (rendererBounds.center - colliderBounds.center).magnitude < 0.3f) &&
                !config.forceUseMeshes)
                foreach (Transform t in meshTransforms)
                {
                    MeshData md = GetColliderMeshData(t);
                    if (md == null)
                        continue;

                    DebugAddCollider(t);
                    meshList.Add(md);
                    validTransformList.Add(t);
                    cantUseColliders = false;
                }


            if (part.FindModulesImplementingReadOnly<ModuleJettison>() is List<PartModule> jettisons &&
                jettisons.Count > 0)
            {
                bool variants = part.HasModuleImplementingFast<ModulePartVariants>();
                var jettisonTransforms = new HashSet<string>();
                foreach (ModuleJettison j in jettisons)
                {
                    if (j.jettisonTransform == null)
                        continue;

                    if (variants)
                        // with part variants, jettison name is a comma separated list of transform names
                        foreach (string transformName in j.jettisonName.Split(','))
                            jettisonTransforms.Add(transformName);
                    else
                        jettisonTransforms.Add(j.jettisonTransform.name);
                    if (j.isJettisoned)
                        continue;

                    Transform t = j.jettisonTransform;
                    if (t.gameObject.activeInHierarchy == false)
                        continue;

                    MeshData md = GetVisibleMeshData(t, config.ignoreIfNoRenderer, false);
                    if (md == null)
                        continue;

                    DebugAddMesh(t);
                    meshList.Add(md);
                    validTransformList.Add(t);
                }

                //Voxelize Everything
                if ((cantUseColliders || config.forceUseMeshes || isFairing) && !isDrill)
                    foreach (Transform t in meshTransforms)
                    {
                        if (jettisonTransforms.Contains(t.name))
                            continue;
                        MeshData md = GetVisibleMeshData(t, config.ignoreIfNoRenderer, false);
                        if (md == null)
                            continue;

                        DebugAddMesh(t);
                        meshList.Add(md);
                        validTransformList.Add(t);
                    }
            }
            else
            {
                //Voxelize Everything
                if ((cantUseColliders || config.forceUseMeshes || isFairing) && !isDrill)
                    foreach (Transform t in meshTransforms)
                    {
                        MeshData md = GetVisibleMeshData(t, config.ignoreIfNoRenderer, false);
                        if (md == null)
                            continue;

                        DebugAddMesh(t);
                        meshList.Add(md);
                        validTransformList.Add(t);
                    }
            }

            DebugPrint();
            meshTransforms = validTransformList;
            return meshList;
        }

        private static MeshData CreateBoxMeshFromBoxCollider(Vector3 size, Vector3 center)
        {
            var Points = new List<Vector3>();
            var Verts = new List<Vector3>();
            var Tris = new List<int>();

            Vector3 extents = size * 0.5f;

            Points.Add(new Vector3(center.x - extents.x, center.y + extents.y, center.z - extents.z));
            Points.Add(new Vector3(center.x + extents.x, center.y + extents.y, center.z - extents.z));
            Points.Add(new Vector3(center.x + extents.x, center.y - extents.y, center.z - extents.z));
            Points.Add(new Vector3(center.x - extents.x, center.y - extents.y, center.z - extents.z));
            Points.Add(new Vector3(center.x + extents.x, center.y + extents.y, center.z + extents.z));
            Points.Add(new Vector3(center.x - extents.x, center.y + extents.y, center.z + extents.z));
            Points.Add(new Vector3(center.x - extents.x, center.y - extents.y, center.z + extents.z));
            Points.Add(new Vector3(center.x + extents.x, center.y - extents.y, center.z + extents.z));
            // Front plane
            Verts.Add(Points[0]);
            Verts.Add(Points[1]);
            Verts.Add(Points[2]);
            Verts.Add(Points[3]);
            // Back plane
            Verts.Add(Points[4]);
            Verts.Add(Points[5]);
            Verts.Add(Points[6]);
            Verts.Add(Points[7]);
            // Left plane
            Verts.Add(Points[5]);
            Verts.Add(Points[0]);
            Verts.Add(Points[3]);
            Verts.Add(Points[6]);
            // Right plane
            Verts.Add(Points[1]);
            Verts.Add(Points[4]);
            Verts.Add(Points[7]);
            Verts.Add(Points[2]);
            // Top plane
            Verts.Add(Points[5]);
            Verts.Add(Points[4]);
            Verts.Add(Points[1]);
            Verts.Add(Points[0]);
            // Bottom plane
            Verts.Add(Points[3]);
            Verts.Add(Points[2]);
            Verts.Add(Points[7]);
            Verts.Add(Points[6]);
            // Front Plane
            Tris.Add(0);
            Tris.Add(1);
            Tris.Add(2);
            Tris.Add(2);
            Tris.Add(3);
            Tris.Add(0);
            // Back Plane
            Tris.Add(4);
            Tris.Add(5);
            Tris.Add(6);
            Tris.Add(6);
            Tris.Add(7);
            Tris.Add(4);
            // Left Plane
            Tris.Add(8);
            Tris.Add(9);
            Tris.Add(10);
            Tris.Add(10);
            Tris.Add(11);
            Tris.Add(8);
            // Right Plane
            Tris.Add(12);
            Tris.Add(13);
            Tris.Add(14);
            Tris.Add(14);
            Tris.Add(15);
            Tris.Add(12);
            // Top Plane
            Tris.Add(16);
            Tris.Add(17);
            Tris.Add(18);
            Tris.Add(18);
            Tris.Add(19);
            Tris.Add(16);
            // Bottom Plane
            Tris.Add(20);
            Tris.Add(21);
            Tris.Add(22);
            Tris.Add(22);
            Tris.Add(23);
            Tris.Add(20);

            var mesh = new MeshData(Verts.ToArray(), Tris.ToArray(), new Bounds(center, size));

            return mesh;
        }

        private static MeshData CreateBoxMeshForKerbalEVA()
        {
            return CreateBoxMeshFromBoxCollider(new Vector3(0.5f, 0.8f, 0.5f), Vector3.zero);
        }

        public void RC_Rescale(Vector3 relativeRescaleFactor)
        {
            //this is currently just a wrapper, in the future if Rescale changes this can change to maintain compatibility
            Rescale(relativeRescaleFactor);
        }

        public void Rescale(Vector3 relativeRescaleFactor)
        {
            RebuildAllMeshData();
        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);

            // Flight passes node from the craft config which is empty, using the loaded prefab instead
            if (HighLogic.LoadedSceneIsFlight)
                return;

            LoadBool(node, "forceUseColliders", ref config.forceUseColliders);
            LoadBool(node, "forceUseMeshes", ref config.forceUseMeshes);
            LoadBool(node, "ignoreForMainAxis", ref config.ignoreForMainAxis);
            LoadBool(node, "ignoreIfNoRenderer", ref config.ignoreIfNoRenderer);
            LoadBool(node, "rebuildOnAnimation", ref config.rebuildOnAnimation);
            LoadList(node, "ignoreTransform", ref config.ignoredTransforms);
            LoadList(node, "unignoreTransform", ref config.unignoredTransforms);
        }

        private void LoadBool(ConfigNode node, string nodeName, ref bool value)
        {
            if (!node.HasValue(nodeName))
                return;
            bool.TryParse(node.GetValue(nodeName), out value);
            _ready = false;
        }

        private void LoadList(ConfigNode node, string nodeName, ref List<string> list)
        {
            if (!node.HasValue(nodeName))
                return;
            list.AddRange(node.GetValues(nodeName));
            _ready = false;
        }

        private class DebugInfoBuilder
        {
            public readonly List<string> meshes;
            public readonly List<string> colliders;
            public readonly List<string> noRenderer;

            public DebugInfoBuilder()
            {
                meshes = new List<string>();
                colliders = new List<string>();
                noRenderer = new List<string>();
            }

            public void Clear()
            {
                meshes.Clear();
                colliders.Clear();
                noRenderer.Clear();
            }

            public void Print(Part p)
            {
                StringBuilder sb = StringBuilderCache.Acquire();
                sb.Append($"{p.name} - mesh build info:");
                if (meshes.Count > 0)
                {
                    sb.Append("\n     Meshes: ");
                    sb.Append(string.Join(", ", meshes.ToArray()));
                }

                if (colliders.Count > 0)
                {
                    sb.Append("\n     Colliders: ");
                    sb.Append(string.Join(", ", colliders.ToArray()));
                }

                if (noRenderer.Count > 0)
                {
                    sb.Append("\n     No renderer found: ");
                    sb.Append(string.Join(", ", noRenderer.ToArray()));
                }

                FARLogger.Debug(sb.ToStringAndRelease());
            }
        }

        private readonly DebugInfoBuilder debugInfo = new DebugInfoBuilder();
    }
}
