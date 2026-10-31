using System;
using System.Collections.Generic;
using System.Reflection;
using FerramAerospaceResearch.FARGUI;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.Geometry.Exposure;
using FerramAerospaceResearch.Resources;
using Unity.Mathematics;
using UnityEngine;
using Renderer = UnityEngine.Renderer;

namespace FerramAerospaceResearch.FARAeroComponents
{
    public class VehicleExposure : MonoBehaviour
    {
        public enum Direction
        {
            Airstream,
            Sun,
            Body
        };

        public static readonly Device[] DeviceOptions = { Device.None, Device.PreferGPU, Device.CPU, Device.GPU };
        public static readonly Direction[] DirectionOptions = { Direction.Airstream, Direction.Sun, Direction.Body };

        private readonly Renderer<Part> exposureRenderer = new();
        private BoundsRenderer boundsRenderer;

        private readonly Dictionary<Part, List<Renderer>> cachedRenderers =
            new(ObjectReferenceEqualityComparer<Part>.Default);

        private Device computeDevice = FARConfig.Exposure.Device;

        public Device ComputeDevice
        {
            get { return computeDevice; }
            set
            {
                computeDevice = value;
                computeDevice.Select((device, renderer) => renderer.Device = device, exposureRenderer);
            }
        }

        private RenderRequest airstreamRequest;
        private RenderRequest sunRequest;
        private RenderRequest bodyRequest;
        private readonly List<RenderRequest> requests = new();

        private ExposureDebugger AirstreamDebugger
        {
            get { return (ExposureDebugger)airstreamRequest.debugger; }
        }

        private ExposureDebugger SunDebugger
        {
            get { return (ExposureDebugger)sunRequest.debugger; }
        }

        private ExposureDebugger BodyDebugger
        {
            get { return (ExposureDebugger)bodyRequest.debugger; }
        }

        public Bounds VesselBounds
        {
            get { return exposureRenderer.Bounds; }
            set
            {
                exposureRenderer.Bounds = value;
                boundsRenderer.VesselBounds = value;
            }
        }

        private Vessel vessel;

        public Vessel Vessel
        {
            get { return vessel; }
            set
            {
                if (vessel == value)
                    return;
                vessel = value;
                OnVesselEvent(value);
            }
        }

        public int2 RenderSize
        {
            get { return exposureRenderer.RenderSize; }
            set { exposureRenderer.RenderSize = value; }
        }

        private Color debugBackgroundColor = FARConfig.Exposure.DebugBackgroundColor;

        public Color DebugBackgroundColor
        {
            get { return debugBackgroundColor; }
            set
            {
                debugBackgroundColor = value;
                airstreamRequest.debugger.BackgroundColor = value;
                sunRequest.debugger.BackgroundColor = value;
                bodyRequest.debugger.BackgroundColor = value;
            }
        }

        public bool Enabled
        {
            get { return ComputeDevice != Device.None; }
        }

        public bool InAtmosphere
        {
            get { return Vessel.atmDensity > 1e-10; }
        }

        private static readonly MethodInfo partInit =
            typeof(Part).GetMethod("CreateRendererLists", BindingFlags.Instance | BindingFlags.NonPublic);

        private static readonly object[] noArgs = Array.Empty<object>();
        private static readonly GUILayoutOption[] noLayoutOptions = Array.Empty<GUILayoutOption>();

        private bool displayArrow;
        private bool displayLabels = true;
        private bool overrideCameraShader;
        private GUIDropDown<Device> deviceSelect;
        private GUIDropDown<Direction> directionSelect;
        private Direction displayedDirection;

        private void Awake()
        {
            deviceSelect = new GUIDropDown<Device>(new[]
                                                   {
                                                       LocalizerExtensions.Get("FARDeviceNone"),
                                                       LocalizerExtensions.Get("FARDevicePreferGPU"),
                                                       LocalizerExtensions.Get("FARDeviceCPU"),
                                                       LocalizerExtensions.Get("FARDeviceGPU"),
                                                   },
                                                   DeviceOptions,
                                                   DeviceOptions.IndexOf(ComputeDevice));
            directionSelect = new GUIDropDown<Direction>(new[]
                                                         {
                                                             LocalizerExtensions
                                                                 .Get("FARExposureDirectionAirstream"),
                                                             LocalizerExtensions.Get("FARExposureDirectionSun"),
                                                             LocalizerExtensions.Get("FARExposureDirectionBody"),
                                                         },
                                                         DirectionOptions);

            ComputeDevice = computeDevice;
            exposureRenderer.RenderSize = new int2(FARConfig.Exposure.Width, FARConfig.Exposure.Height);
            exposureRenderer.Material = FARAssets.Instance.Shaders.ExposedSurface;
            exposureRenderer.RenderSize = RenderSize;
            exposureRenderer.PixelCountKernel = FARAssets.Instance.ComputeShaders.CountPixels.Kernel;
            exposureRenderer.PixelCountShader = FARAssets.Instance.ComputeShaders.CountPixels;

            airstreamRequest = new RenderRequest
            {
                debugger = new ExposureDebugger
                {
                    Material = FARAssets.Instance.Shaders.ExposedSurfaceDebug,
                    BackgroundColor = debugBackgroundColor,
                    enabled = true,
                    EnableDebugTexture = false,
                    ObjectColors = FARConfig.Voxelization.ColorMapTexture(),
                    ArrowColor = Color.red,
                    DisplayArrow = false
                },
                result = new RenderResult<Part> { renderer = exposureRenderer },
                userData = this
            };

            sunRequest = new RenderRequest
            {
                debugger = new ExposureDebugger
                {
                    Material = airstreamRequest.debugger.Material,
                    BackgroundColor = debugBackgroundColor,
                    enabled = true,
                    EnableDebugTexture = false,
                    ObjectColors = airstreamRequest.debugger.ObjectColors,
                    ArrowColor = Color.yellow,
                    DisplayArrow = false
                },
                result = new RenderResult<Part> { renderer = exposureRenderer },
                userData = this
            };

            bodyRequest = new RenderRequest
            {
                debugger = new ExposureDebugger
                {
                    Material = airstreamRequest.debugger.Material,
                    BackgroundColor = debugBackgroundColor,
                    enabled = true,
                    EnableDebugTexture = false,
                    ObjectColors = airstreamRequest.debugger.ObjectColors,
                    ArrowColor = Color.blue,
                    DisplayArrow = false
                },
                result = new RenderResult<Part> { renderer = exposureRenderer },
                userData = this
            };

            boundsRenderer = BoundsRenderer.Get();
        }

        private void Start()
        {
            GameEvents.onVesselStandardModification.Add(OnVesselEvent);
            GameEvents.onVesselChange.Add(OnActiveVesselEvent);

            enabled = Vessel == FlightGlobals.ActiveVessel;
        }

        public void ResetParts<T>(T parts) where T : IEnumerable<Part>
        {
            exposureRenderer.Reset();
            cachedRenderers.Clear();
            foreach (Part part in parts)
            {
                partInit.Invoke(part, noArgs); // make sure renderers are setup by the part
                List<Renderer> renderers = part.FindModelRenderersCached();
                // for some reason the list of renderers can contain nulls on some loads
                renderers.RemoveAll(renderer => renderer != null && renderer.gameObject.layer != 0);
                exposureRenderer.SetupRenderers(part, renderers, part.mpb);
                cachedRenderers.Add(part, renderers);
            }
        }

        private void SetupPartsForDebugCamera()
        {
            foreach (KeyValuePair<Part, List<Renderer>> pair in cachedRenderers)
            {
                pair.Key.mpb.SetTexture(ShaderPropertyIds._ColorTex, airstreamRequest.debugger.ObjectColors);
                pair.Key.mpb.SetColor(ShaderPropertyIds._BackgroundColor, debugBackgroundColor);

                foreach (Renderer renderer in pair.Value)
                    renderer.SetPropertyBlock(pair.Key.mpb);
            }
        }

        private void UpdateRenderers()
        {
            foreach (KeyValuePair<Part, List<Renderer>> pair in cachedRenderers)
                exposureRenderer.SetupRenderers(pair.Key, pair.Value, pair.Key.mpb);
        }

        private void OnVesselEvent(Vessel v)
        {
            if (v == Vessel)
                ResetParts(Vessel.parts);
        }

        private void OnActiveVesselEvent(Vessel v)
        {
            enabled = v == Vessel;
        }

        private void LateUpdate()
        {
            if (FlightDriver.Pause || exposureRenderer.Count == 0 || !Enabled)
                return;

            switch (displayedDirection)
            {
                case Direction.Airstream:
                    AirstreamDebugger.EnableDebugTexture = false;
                    break;
                case Direction.Sun:
                    SunDebugger.EnableDebugTexture = false;
                    break;
                case Direction.Body:
                    BodyDebugger.EnableDebugTexture = false;
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            // TODO: bounds seem off in KSP
            float4x4 vesselLocalToWorldMatrix = Vessel.transform.localToWorldMatrix;
            requests.Clear();

            Transform vesselTransform = Vessel.transform;

            if (InAtmosphere && FARConfig.Exposure.Airstream)
            {
                Vector3 forward = Vessel.velocityD.magnitude < 0.001
                                      ? Vector3.forward
                                      : vesselTransform.InverseTransformDirection(Vessel.srf_velocity).normalized;
                requests.Add(airstreamRequest with { lookDir = -forward });
            }

            if (FARConfig.Exposure.Sun)
            {
                Vector3 fromSun =
                    (VesselBounds.center - vesselTransform.InverseTransformPoint(FlightGlobals.Bodies[0].position))
                    .normalized;
                requests.Add(sunRequest with { lookDir = fromSun });
            }

            if (FARConfig.Exposure.Body)
            {
                Vector3 fromBody =
                    (VesselBounds.center - vesselTransform.InverseTransformPoint(Vessel.mainBody.position)).normalized;
                requests.Add(bodyRequest with { lookDir = fromBody });
            }

            if (requests.Count != 0)
                exposureRenderer.Render(requests, vesselLocalToWorldMatrix);
        }

        public bool Display()
        {
            deviceSelect.GUIDropDownDisplay(noLayoutOptions);
            ComputeDevice = deviceSelect.ActiveSelection;

            GUILayout.BeginHorizontal();
            int2 size = RenderSize;
            size.x = GUIUtils.TextEntryForInt(LocalizerExtensions.Get("FARExposureWidthLabel"), 50, size.x);
            size.y = GUIUtils.TextEntryForInt(LocalizerExtensions.Get("FARExposureHeightLabel"), 50, size.y);
            RenderSize = size;
            GUILayout.EndHorizontal();

            // TODO: background color picker

            displayArrow = GUILayout.Toggle(displayArrow, LocalizerExtensions.Get("FARExposureArrowLabel"));
            AirstreamDebugger.DisplayArrow = displayArrow && InAtmosphere && FARConfig.Exposure.Airstream;
            SunDebugger.DisplayArrow = displayArrow && FARConfig.Exposure.Sun;
            BodyDebugger.DisplayArrow = displayArrow && FARConfig.Exposure.Body;

            boundsRenderer.enabled =
                GUILayout.Toggle(boundsRenderer.enabled, LocalizerExtensions.Get("FARDrawBoundsLabel"));

            bool overridden = overrideCameraShader;
            overrideCameraShader =
                GUILayout.Toggle(overrideCameraShader, LocalizerExtensions.Get("FARExposureShaderLabel"));
            if (overridden != overrideCameraShader)
            {
                if (overrideCameraShader)
                {
                    SetupPartsForDebugCamera();
                    Camera.main.SetReplacementShader(FARAssets.Instance.Shaders.ExposedSurfaceCamera, null);
                }
                else
                    Camera.main.ResetReplacementShader();
            }

            Direction previousDirection = directionSelect.ActiveSelection;
            directionSelect.GUIDropDownDisplay(noLayoutOptions);
            displayedDirection = directionSelect.ActiveSelection;

            return displayedDirection switch
                   {
                       Direction.Airstream => DisplayRequestDebug(airstreamRequest, FARConfig.Exposure.Airstream),
                       Direction.Sun => DisplayRequestDebug(sunRequest, FARConfig.Exposure.Sun),
                       Direction.Body => DisplayRequestDebug(bodyRequest, FARConfig.Exposure.Body),
                       _ => throw new ArgumentOutOfRangeException()
                   } ||
                   previousDirection != displayedDirection;
        }

        private void OnDestroy()
        {
            GameEvents.onVesselStandardModification.Remove(OnVesselEvent);
            GameEvents.onVesselChange.Remove(OnActiveVesselEvent);

            exposureRenderer.Dispose();
            requests.Clear();

            airstreamRequest.debugger.Dispose();
            airstreamRequest = default;

            sunRequest.debugger.Dispose();
            sunRequest = default;

            bodyRequest.debugger.Dispose();
            bodyRequest = default;
        }

        private bool DisplayRequestDebug(in RenderRequest request, Observable<bool> toggle)
        {
            var debugger = (ExposureDebugger)request.debugger;
            debugger.EnableDebugTexture = true;

            bool enable = toggle;
            toggle.Set(GUILayout.Toggle(toggle, LocalizerExtensions.Get("FARExposureEnableLabel")));

            if (!toggle)
                return enable;

            bool previous = displayLabels;
            displayLabels = GUILayout.Toggle(displayLabels, LocalizerExtensions.Get("FARExposureShowLabelsLabel"));

            if (Enabled)
                debugger.Display(250, displayLabels);

            return previous != displayLabels || toggle != enable;
        }
    }
}
