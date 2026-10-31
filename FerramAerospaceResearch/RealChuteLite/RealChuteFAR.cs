using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using FerramAerospaceResearch.PartExtensions;
using KSP.Localization;
using KSP.UI.Screens;
using KSPCommunityFixes;
using UnityEngine;
using Random = System.Random;

/* RealChuteLite is the work of Christophe Savard (stupid_chris), and is licensed the same way than the rest of FAR is.
 * If you have any questions about this code, or want to report something, don't annoy ferram about it, ask me
 * directly on GitHub, the forums, or IRC. */

namespace FerramAerospaceResearch.RealChuteLite
{
    public class RealChuteFAR : PartModule, IModuleInfo, IMultipleDragCube, IPartMassModifier, IPartCostModifier
    {
        /// <summary>
        ///     Parachute deployment states
        /// </summary>
        public enum DeploymentStates
        {
            NONE,
            STOWED,
            PREDEPLOYED,
            DEPLOYED,
            CUT
        }

        /// <summary>
        ///     Parachute deployment safety state
        /// </summary>
        public enum SafeState
        {
            SAFE,
            RISKY,
            DANGEROUS
        }

        public const float areaDensity = 5.65E-5f, areaCost = 0.075f, staticCd = 1; //t/m², and F/m² for the first two
        public const double startTemp = 300, maxTemp = 493.15;                      //In °K
        public const double specificHeat = 1700, absoluteZero = -273.15;            //Specific heat in J/kg*K

        //More useful constants
        public const int maxSpares = 5;

        //Material constants
        public static readonly string materialName = Localizer.Format("RCLMatNylon");
        public static readonly string stowed = Localizer.Format("RCLStatusStowed");
        public static readonly string predeployed = Localizer.Format("RCLStatusPreDep");
        public static readonly string deployed = Localizer.Format("RCLStatusDep");
        public static readonly string cut = Localizer.Format("RCLStatusCut");
        public static readonly string[] cubeNames = {"STOWED", "RCDEPLOYED", "DEPLOYED", "SEMIDEPLOYED", "PACKED"};

        //Quick enum parsing/tostring dictionaries
        private static readonly Dictionary<DeploymentStates, string> names = new Dictionary<DeploymentStates, string>(5)
        {
            {DeploymentStates.NONE, string.Empty},
            {DeploymentStates.STOWED, stowed},
            {DeploymentStates.PREDEPLOYED, predeployed},
            {DeploymentStates.DEPLOYED, deployed},
            {DeploymentStates.CUT, cut}
        };

        private static readonly Dictionary<string, DeploymentStates> states =
            new Dictionary<string, DeploymentStates>(5)
            {
                {string.Empty, DeploymentStates.NONE},
                {stowed, DeploymentStates.STOWED},
                {predeployed, DeploymentStates.PREDEPLOYED},
                {deployed, DeploymentStates.DEPLOYED},
                {cut, DeploymentStates.CUT}
            };

        //Bold KSP style GUI label
        private static GUIStyle boldLabel;

        //Yellow KSP style GUI label
        private static GUIStyle yellowLabel;

        //Red KSP style GUI label
        private static GUIStyle redLabel;
        private readonly PhysicsWatch failedTimer = new PhysicsWatch(), randomTimer = new PhysicsWatch();
        private readonly int id = Guid.NewGuid().GetHashCode();

        //Stealing values from the stock module
        [KSPField]
        public float autoCutSpeed = 0.5f;

        [KSPField(guiName = "RCLSettingMinPres", isPersistant = true, guiActive = true, guiActiveEditor = true),
         UI_FloatRange(stepIncrement = 0.01f, maxValue = 0.5f, minValue = 0.01f)]
        public float minAirPressureToOpen = 0.01f;

        [KSPField(guiName = "RCLSettingAlt", isPersistant = true, guiActive = true, guiActiveEditor = true),
         UI_FloatRange(stepIncrement = 50f, maxValue = 5000f, minValue = 50f)]
        public float deployAltitude = 700;

        [KSPField]
        public string capName = "cap", canopyName = "canopy";

        [KSPField]
        public string semiDeployedAnimation = "semiDeploy", fullyDeployedAnimation = "fullyDeploy";

        [KSPField]
        public float semiDeploymentSpeed = 0.5f, deploymentSpeed = 0.16667f;

        [KSPField]
        public bool invertCanopy = true;

        // ReSharper disable once NotAccessedField.Global -> unity
        //Persistant fields
        //this cannot be persistent to ensure that bad values aren't saved, and since these chutes aren't customizable there's no reason to save this
        [KSPField(isPersistant = false)]
        public float preDeployedDiameter = 1, deployedDiameter = 25;

        [KSPField(isPersistant = true)]
        public float caseMass, time;

        [KSPField(isPersistant = true)]
        public bool armed, staged, initiated;

        [KSPField(isPersistant = true, guiActive = true, guiName = "RCLStatusSpare")]
        public int chuteCount = 5;

        [KSPField(isPersistant = true)]
        public string depState = Localizer.Format("RCLStatusStowed");

        [KSPField(isPersistant = true)]
        public float currentArea;

        [KSPField(isPersistant = true)]
        public double chuteTemperature = 300;

        // ReSharper disable once NotAccessedField.Global -> unity
        [KSPField(isPersistant = true,
                  guiActive = false,
                  guiName = "RCLStatusChuteTemp",
                  guiFormat = "0.00",
                  guiUnits = "RCLTempUnit")]
        public float currentTemp = 20;

        // ReSharper disable once NotAccessedField.Global -> unity
        [KSPField(guiActive = false, guiName = "RCLStatusMaxTemp", guiFormat = "0.00", guiUnits = "RCLTempUnit")]
        public float chuteDisplayMaxTemp = (float)(maxTemp + absoluteZero);

        //Quick access to the part GUI events
        private BaseEvent deploy, disarm, cutE, repack;

        //Flight
        private Vector3 dragVector, pos = new Vector3d();
        private PhysicsWatch dragTimer = new PhysicsWatch();
        private bool displayed, showDisarm;
        private double asl, trueAlt;
        private double atmPressure, atmDensity;
        private float sqrSpeed;
        private double thermMass, convFlux;

        //Part
        private Transform parachute, cap;
        private Rigidbody rigidbody;
        private float randomX, randomY, randomTime;
        private DeploymentStates state = DeploymentStates.NONE;
        private SafeState safeState = SafeState.SAFE;
        private float massDelta;
        private bool shieldedCanDeploy;

        //GUI
        private bool visible, hid;
        private Rect window, drag;
        private Vector2 scroll;

        // If the vessel is stopped on the ground
        public bool GroundStop
        {
            get { return vessel.LandedOrSplashed && vessel.horizontalSrfSpeed < autoCutSpeed; }
        }

        // If the parachute can be repacked
        public bool CanRepack
        {
            get
            {
                return (GroundStop || atmPressure.NearlyEqual(0)) &&
                       DeploymentState == DeploymentStates.CUT &&
                       chuteCount > 0 &&
                       FlightGlobals.ActiveVessel.isEVA;
            }
        }

        //If the Kerbal can repack the chute in career mode
        public static bool CanRepackCareer
        {
            get
            {
                ProtoCrewMember kerbal = FlightGlobals.ActiveVessel.GetVesselCrew()[0];
                return HighLogic.CurrentGame.Mode != Game.Modes.CAREER ||
                       kerbal.experienceTrait.Title == "Engineer" && kerbal.experienceLevel >= 1;
            }
        }

        //Predeployed area of the chute
        public float PreDeployedArea
        {
            get { return GetArea(preDeployedDiameter); }
        }

        //Deployed area of the chute
        public float DeployedArea
        {
            get { return GetArea(deployedDiameter); }
        }

        //The current useful convection area
        private double ConvectionArea
        {
            get
            {
                if (DeploymentState == DeploymentStates.PREDEPLOYED &&
                    dragTimer.Elapsed.Seconds < 1 / semiDeploymentSpeed)
                    return UtilMath.Lerp(0, DeployedArea, dragTimer.Elapsed.Seconds * semiDeploymentSpeed);
                return DeployedArea;
            }
        }

        //Mass of the chute
        public float ChuteMass
        {
            get { return DeployedArea * areaDensity; }
        }

        //Total dry mass of the chute part
        public float TotalMass
        {
            get
            {
                if (caseMass.NearlyEqual(0))
                    caseMass = part.mass;
                return caseMass + ChuteMass;
            }
        }

        //Position to apply the force to
        public Vector3 ForcePosition
        {
            get { return parachute.position; }
        }

        //If the random deployment timer has been spent
        public bool RandomDeployment
        {
            get
            {
                if (!randomTimer.IsRunning)
                    randomTimer.Start();

                if (randomTimer.Elapsed.TotalSeconds < randomTime)
                    return false;
                randomTimer.Reset();
                return true;
            }
        }

        //If the parachute is in a high enough atmospheric pressure to deploy
        public bool PressureCheck
        {
            get { return atmPressure >= minAirPressureToOpen; }
        }

        //If the parachute can deploy
        public bool CanDeploy
        {
            get
            {
                if (GroundStop || atmPressure.NearlyEqual(0) || !(shieldedCanDeploy || !part.ShieldedFromAirstream))
                    return false;
                if (DeploymentState == DeploymentStates.CUT)
                    return false;
                if (PressureCheck)
                    return true;
                return IsDeployed;
            }
        }

        //If the parachute is deployed
        public bool IsDeployed
        {
            get
            {
                return DeploymentState switch
                {
                    DeploymentStates.PREDEPLOYED => true,
                    DeploymentStates.DEPLOYED => true,
                    _ => false
                };
            }
        }

        //Persistent deployment state
        public DeploymentStates DeploymentState
        {
            get
            {
                if (state == DeploymentStates.NONE)
                    DeploymentState = states[depState];
                return state;
            }
            set
            {
                state = value;
                depState = names[value];
            }
        }

        //The inverse thermal mass of the parachute
        public double InvThermalMass
        {
            get
            {
                if (thermMass.NearlyEqual(0))
                    thermMass = 1 / (specificHeat * ChuteMass);
                return thermMass;
            }
        }

        //The current chute emissivity constant
        public double ChuteEmissivity
        {
            get
            {
                if (chuteTemperature < 293.15)
                    return 0.72;
                return chuteTemperature > 403.15
                           ? 0.9
                           : UtilMath.Lerp(0.72, 0.9, (chuteTemperature - 293.15) / 110 + 293.15);
            }
        }

        public static GUIStyle BoldLabel
        {
            get { return boldLabel ??= new GUIStyle(HighLogic.Skin.label) {fontStyle = FontStyle.Bold}; }
        }

        public static GUIStyle YellowLabel
        {
            get
            {
                return yellowLabel ??= new GUIStyle(HighLogic.Skin.label)
                {
                    normal = {textColor = XKCDColors.BrightYellow},
                    hover = {textColor = XKCDColors.BrightYellow}
                };
            }
        }

        public static GUIStyle RedLabel
        {
            get
            {
                return redLabel ??= new GUIStyle(HighLogic.Skin.label)
                {
                    normal = {textColor = XKCDColors.Red},
                    hover = {textColor = XKCDColors.Red}
                };
            }
        }

        private BaseEvent DeployE
        {
            get { return deploy ??= Events["GUIDeploy"]; }
        }

        private BaseEvent Disarm
        {
            get { return disarm ??= Events["GUIDisarm"]; }
        }

        private BaseEvent CutE
        {
            get { return cutE ??= Events["GUICut"]; }
        }

        private BaseEvent Repack
        {
            get { return repack ??= Events["GUIRepack"]; }
        }

        //Not needed
        public Callback<Rect> GetDrawModulePanelCallback()
        {
            return null;
        }

        //Sets module info title
        public string GetModuleTitle()
        {
            return "RealChute";
        }

        //Sets part info field
        public string GetPrimaryField()
        {
            return string.Empty;
        }

        public override string GetInfo()
        {
            //Info in the editor part window
            float tmpPartMass = TotalMass;
            massDelta = 0;
            if (!(part.partInfo?.partPrefab is null))
                massDelta = tmpPartMass - part.partInfo.partPrefab.mass;

            var b = new StringBuilder();
            b.Append(Localizer.Format("RCLModuleInfo0", caseMass));
            b.Append(Localizer.Format("RCLModuleInfo1", maxSpares));
            b.Append(Localizer.Format("RCLModuleInfo2", autoCutSpeed));
            b.AppendLine(Localizer.Format("RCLModuleInfo3", materialName));
            b.Append(Localizer.Format("RCLModuleInfo4", staticCd));
            b.Append(Localizer.Format("RCLModuleInfo5", maxTemp + absoluteZero));
            b.Append(Localizer.Format("RCLModuleInfo6", preDeployedDiameter));
            b.Append(Localizer.Format("RCLModuleInfo7", deployedDiameter));
            b.Append(Localizer.Format("RCLModuleInfo8", minAirPressureToOpen));
            b.Append(Localizer.Format("RCLModuleInfo9", deployAltitude));
            b.Append(Localizer.Format("RCLModuleInfo10",
                                      Math.Round(1 / semiDeploymentSpeed, 1, MidpointRounding.AwayFromZero)));
            b.Append(Localizer.Format("RCLModuleInfo11",
                                      Math.Round(1 / deploymentSpeed, 1, MidpointRounding.AwayFromZero)));

            return b.ToString();
        }

        //Sets the part in the correct position for DragCube rendering
        public void AssumeDragCubePosition(string cubeName)
        {
            if (string.IsNullOrEmpty(cubeName))
                return;
            InitializeAnimationSystem();
            switch (cubeName)
            {
                //DaMichel: now we handle the stock behaviour, too.
                case "PACKED": //stock
                case "STOWED":
                {
                    parachute.gameObject.SetActive(false);
                    cap?.gameObject.SetActive(true);
                    break;
                }

                case "RCDEPLOYED": //This is not a predeployed state, no touchy
                {
                    parachute.gameObject.SetActive(false);
                    cap?.gameObject.SetActive(false);
                    break;
                }

                case "SEMIDEPLOYED": //  stock
                {
                    parachute.gameObject.SetActive(true);
                    cap?.gameObject.SetActive(false);
                    // to the end of the animation
                    part.SkipToAnimationTime(semiDeployedAnimation, 0, 1);
                    break;
                }

                case "DEPLOYED": //  stock
                {
                    parachute.gameObject.SetActive(true);
                    cap?.gameObject.SetActive(false);
                    // to the end of the animation
                    part.SkipToAnimationTime(fullyDeployedAnimation, 0, 1);
                    break;
                }
            }
        }

        //Gives DragCube names
        public string[] GetDragCubeNames()
        {
            return cubeNames;
        }

        //Unused
        public bool UsesProceduralDragCubes()
        {
            return false;
        }

        // TODO 1.2: provide actual implementation of this new method
        public bool IsMultipleCubesActive
        {
            get { return true; }
        }

        //Gives the cost for this parachute
        public float GetModuleCost(float defaultCost, ModifierStagingSituation sit)
        {
            return (float)Math.Round(DeployedArea * areaCost);
        }

        public ModifierChangeWhen GetModuleCostChangeWhen()
        {
            return ModifierChangeWhen.FIXED;
        }

        //For IPartMassModifier
        public float GetModuleMass(float defaultMass, ModifierStagingSituation sit)
        {
            return massDelta;
        }

        public ModifierChangeWhen GetModuleMassChangeWhen()
        {
            return ModifierChangeWhen.FIXED;
        }

        //Deploys the parachutes if possible
        [KSPEvent(guiActive = true,
                  active = true,
                  externalToEVAOnly = true,
                  guiActiveUnfocused = true,
                  guiName = "RCLEventDeploy",
                  unfocusedRange = 5)]
        public void GUIDeploy()
        {
            ActivateRC();
        }

        //Cuts main chute chute
        [KSPEvent(guiActive = true,
                  active = true,
                  externalToEVAOnly = true,
                  guiActiveUnfocused = true,
                  guiName = "RCLEventCut",
                  unfocusedRange = 5)]
        public void GUICut()
        {
            Cut();
        }

        [KSPEvent(guiActive = true,
                  active = true,
                  externalToEVAOnly = true,
                  guiActiveUnfocused = true,
                  guiName = "RCLEventDisarm",
                  unfocusedRange = 5)]
        public void GUIDisarm()
        {
            armed = false;
            showDisarm = false;
            part.stackIcon.SetIconColor(XKCDColors.White);
            DeployE.active = true;
            DeactivateRC();
        }

        //Repacks chute from EVA if in space or on the ground
        [KSPEvent(guiActive = false,
                  active = true,
                  externalToEVAOnly = true,
                  guiActiveUnfocused = true,
                  guiName = "RCLEventRepack",
                  unfocusedRange = 5)]
        public void GUIRepack()
        {
            if (!CanRepack)
                return;
            if (!CanRepackCareer)
            {
                ScreenMessages.PostScreenMessage(Localizer.Format("RCLRepackErrorMessage"),
                                                 5,
                                                 ScreenMessageStyle.UPPER_CENTER);
                return;
            }

            part.Effect("rcrepack");
            Repack.guiActiveUnfocused = false;
            part.stackIcon.SetIconColor(XKCDColors.White);
            if (chuteCount != -1)
                chuteCount--;
            DeploymentState = DeploymentStates.STOWED;
            randomTimer.Reset();
            time = 0;
            cap?.gameObject.SetActive(true);
            part.DragCubes.SetCubeWeight("PACKED", 1);
            part.DragCubes.SetCubeWeight("RCDEPLOYED", 0);
        }

        //Shows the info window
        [KSPEvent(guiActive = true, active = true, guiActiveEditor = true, guiName = "RCLEventToggleInfo")]
        public void GUIToggleWindow()
        {
            if (!visible)
            {
                var parachutes = new List<RealChuteFAR>();
                if (HighLogic.LoadedSceneIsEditor)
                    parachutes.AddRange(EditorLogic.SortedShipList.Where(p => p.HasModuleImplementingFast<RealChuteFAR>())
                                                   .Select(p => p.FindModuleImplementingFast<RealChuteFAR>()));
                else if (HighLogic.LoadedSceneIsFlight)
                    parachutes.AddRange(vessel.FindPartModulesImplementing<RealChuteFAR>());
                if (parachutes.Count > 1 && parachutes.Exists(p => p.visible))
                {
                    RealChuteFAR module = parachutes.Find(p => p.visible);
                    window.x = module.window.x;
                    window.y = module.window.y;
                    module.visible = false;
                }
            }

            visible = !visible;
        }

        //Deploys the parachutes if possible
        [KSPAction("Deploy chute", guiName = "RCLEventDeploy")]
        public void ActionDeploy(KSPActionParam param)
        {
            ActivateRC();
        }

        //Cuts main chute
        [KSPAction("Cut chute", guiName = "RCLEventCut")]
        public void ActionCut(KSPActionParam param)
        {
            if (IsDeployed)
                Cut();
        }

        [KSPAction("Disarm chute", guiName = "RCLEventDisarm")]
        public void ActionDisarm(KSPActionParam param)
        {
            if (armed)
                GUIDisarm();
        }

        //Returns the canopy area of the given Diameter
        public static float GetArea(float diameter)
        {
            return (float)(diameter * diameter * Math.PI / 4);
        }

        //Activates the parachute
        public void ActivateRC()
        {
            staged = true;
            armed = true;
            print("[RealChute]: " + part.partInfo.name + " was activated in stage " + part.inverseStage);
        }

        //Deactivates the parachute
        public void DeactivateRC()
        {
            staged = false;
            print("[RealChute]: " + part.partInfo.name + " was deactivated");
        }

        //Copies stats from the info window to the symmetry counterparts
        private void CopyToCounterparts()
        {
            foreach (Part p in part.symmetryCounterparts)
            {
                var module = p.FindModuleImplementingFast<RealChuteFAR>();
                module.minAirPressureToOpen = minAirPressureToOpen;
                module.deployAltitude = deployAltitude;
            }
        }

        //Deactivates the part
        public void StagingReset()
        {
            DeactivateRC();
            armed = false;
            if (part.inverseStage != 0)
                part.inverseStage -= 1;
            else
                part.inverseStage = StageManager.CurrentStage;
        }

        //Allows the chute to be repacked if available
        public void SetRepack()
        {
            part.stackIcon.SetIconColor(XKCDColors.Red);
            StagingReset();
        }

        //Drag formula calculations
        public float DragCalculation(float area)
        {
            return (float)atmDensity * sqrSpeed * staticCd * area / 2000;
        }

        public override string GetModuleDisplayName()
        {
            return Localizer.Format("RCLModuleTitle");
        }

        //Event when the UI is hidden (F2)
        private void HideUI()
        {
            hid = true;
        }

        //Event when the UI is shown (F2)
        private void ShowUI()
        {
            hid = false;
        }

        //Adds a random noise to the parachute movement
        private void ParachuteNoise()
        {
            parachute.Rotate(new Vector3(5 * (Mathf.PerlinNoise(Time.time, randomX + Mathf.Sin(Time.time)) - 0.5f),
                                         5 * (Mathf.PerlinNoise(Time.time, randomY + Mathf.Sin(Time.time)) - 0.5f),
                                         0));
        }

        //Makes the canopy follow drag direction
        private void FollowDragDirection()
        {
            if (dragVector.sqrMagnitude > 0)
                parachute.rotation = Quaternion.LookRotation(invertCanopy ? dragVector : -dragVector, parachute.up);
            ParachuteNoise();
        }

        //Parachute predeployment
        public void PreDeploy()
        {
            part.stackIcon.SetIconColor(XKCDColors.BrightYellow);
            part.Effect("rcpredeploy");
            DeploymentState = DeploymentStates.PREDEPLOYED;
            parachute.gameObject.SetActive(true);
            cap?.gameObject.SetActive(false);
            part.PlayAnimation(semiDeployedAnimation, semiDeploymentSpeed);
            dragTimer.Start();
            part.DragCubes.SetCubeWeight("PACKED", 0);
            part.DragCubes.SetCubeWeight("RCDEPLOYED", 1);
            Fields["currentTemp"].guiActive = true;
            Fields["chuteDisplayMaxTemp"].guiActive = true;
        }

        //Parachute deployment
        public void Deploy()
        {
            part.stackIcon.SetIconColor(XKCDColors.RadioactiveGreen);
            part.Effect("rcdeploy");
            DeploymentState = DeploymentStates.DEPLOYED;
            dragTimer.Restart();
            part.PlayAnimation(fullyDeployedAnimation, deploymentSpeed);
        }

        //Parachute cutting
        public void Cut()
        {
            part.Effect("rccut");
            DeploymentState = DeploymentStates.CUT;
            parachute.gameObject.SetActive(false);
            currentArea = 0;
            dragTimer.Reset();
            currentTemp = (float)(startTemp + absoluteZero);
            chuteTemperature = startTemp;
            Fields["currentTemp"].guiActive = false;
            Fields["chuteDisplayMaxTemp"].guiActive = false;
            SetRepack();
        }

        //Calculates parachute deployed area
        private float DragDeployment(float currentTime, float debutDiameter, float endDiameter)
        {
            if (!dragTimer.IsRunning)
                dragTimer.Start();

            double t = dragTimer.Elapsed.TotalSeconds;
            time = (float)t;
            if (t <= currentTime)
            {
                /* While this looks linear, area scales with the square of the diameter, and therefore
                 * Deployment will be quadratic. The previous exponential function was too slow at first and rough at the end */
                float currentDiam = Mathf.Lerp(debutDiameter, endDiameter, (float)(t / currentTime));
                currentArea = GetArea(currentDiam);
                return currentArea;
            }

            currentArea = GetArea(endDiameter);
            return currentArea;
        }

        //Drag force vector
        private Vector3 DragForce(float debutDiameter, float endDiameter, float currentTime)
        {
            return DragCalculation(DragDeployment(currentTime, debutDiameter, endDiameter)) * dragVector;
        }

        //Calculates convective flux
        private void CalculateChuteFlux()
        {
            convFlux = vessel.convectiveCoefficient *
                       UtilMath.Lerp(1,
                                     1 +
                                     Math.Sqrt(vessel.mach * vessel.mach * vessel.mach) *
                                     (vessel.dynamicPressurekPa / 101.325),
                                     (vessel.mach - PhysicsGlobals.FullToCrossSectionLerpStart) /
                                     PhysicsGlobals.FullToCrossSectionLerpEnd) *
                       (vessel.externalTemperature - chuteTemperature);
        }

        //Calculates the temperature of the chute and cuts it if needed
        private bool CalculateChuteTemp()
        {
            if (chuteTemperature < PhysicsGlobals.SpaceTemperature)
                chuteTemperature = startTemp;

            double emissiveFlux = 0;
            if (chuteTemperature > 0)
            {
                double temp2 = chuteTemperature * chuteTemperature;
                emissiveFlux =
                    2 *
                    PhysicsGlobals.StefanBoltzmanConstant *
                    ChuteEmissivity *
                    PhysicsGlobals.RadiationFactor *
                    temp2 *
                    temp2;
            }

            chuteTemperature = Math.Max(PhysicsGlobals.SpaceTemperature,
                                        chuteTemperature +
                                        (convFlux - emissiveFlux) *
                                        0.001 *
                                        ConvectionArea *
                                        InvThermalMass *
                                        TimeWarp.fixedDeltaTime);
            if (chuteTemperature > maxTemp)
            {
                ScreenMessages.PostScreenMessage(Localizer.Format("RCLDestroyMessage", part.partInfo.title),
                                                 6f,
                                                 ScreenMessageStyle.UPPER_LEFT);
                Cut();
                return false;
            }

            currentTemp = (float)(chuteTemperature + absoluteZero);
            return true;
        }

        //estimates whether it is safe to deploy the chute or not
        private void CalculateSafeToDeployEstimate()
        {
            SafeState s;
            if (vessel.externalTemperature <= maxTemp || convFlux < 0)
                s = SafeState.SAFE;
            else
                s = chuteTemperature + 0.001 * convFlux * InvThermalMass * DeployedArea * 0.35 <= maxTemp
                        ? SafeState.RISKY
                        : SafeState.DANGEROUS;

            if (safeState == s)
                return;
            safeState = s;
            switch (safeState)
            {
                case SafeState.SAFE:
                    part.stackIcon.SetBackgroundColor(XKCDColors.White);
                    break;

                case SafeState.RISKY:
                    part.stackIcon.SetBackgroundColor(XKCDColors.BrightYellow);
                    break;

                case SafeState.DANGEROUS:
                    part.stackIcon.SetBackgroundColor(XKCDColors.Red);
                    break;
            }
        }

        //Initializes parachute animations
        private void InitializeAnimationSystem()
        {
            //I know this seems random, but trust me, it's needed, else some parachutes don't animate, because fuck you, that's why.
            // ReSharper disable once UnusedVariable -> Needed to animate all parts (stupid_chris)
            Animation anim = part.FindModelAnimators(capName).FirstOrDefault();

            cap = part.FindModelTransform(capName);
            parachute = part.FindModelTransform(canopyName);
            parachute.gameObject.SetActive(true);
            part.InitiateAnimation(semiDeployedAnimation);
            part.InitiateAnimation(fullyDeployedAnimation);
            parachute.gameObject.SetActive(false);
        }

        //Info window
        private void Window(int windowId)
        {
            //Header
            GUI.DragWindow(drag);
            GUILayout.BeginVertical();

            //Top info labels
            StringBuilder b = new StringBuilder(Localizer.Format("RCLGUI0", part.partInfo.title)).AppendLine();
            b.AppendLine(Localizer.Format("RCLGUI0", part.symmetryCounterparts.Count));
            b.AppendLine(Localizer.Format("RCLGUI2", part.TotalMass().ToString("F3")));
            GUILayout.Label(b.ToString());

            //Beginning scroll
            scroll = GUILayout.BeginScrollView(scroll,
                                               false,
                                               false,
                                               GUI.skin.horizontalScrollbar,
                                               GUI.skin.verticalScrollbar,
                                               GUI.skin.box);
            GUILayout.Space(5);
            GUILayout.Label(Localizer.Format("RCLGUI3"), BoldLabel, GUILayout.Width(120));

            //General labels
            b = new StringBuilder(Localizer.Format("RCLGUI4", autoCutSpeed)).AppendLine();
            b.Append(Localizer.Format("RCLGUI5", chuteCount));
            GUILayout.Label(b.ToString());

            //Specific labels
            GUILayout.Label("___________________________________________", BoldLabel);
            GUILayout.Space(3);
            GUILayout.Label(Localizer.Format("RCLGUI6"), BoldLabel, GUILayout.Width(120));
            //Initial label
            b = new StringBuilder();
            b.AppendLine(Localizer.Format("RCLGUI7", materialName));
            b.AppendLine(Localizer.Format("RCLGUI8", staticCd.ToString("0.0")));
            b.AppendLine(Localizer.Format("RCLGUI9", preDeployedDiameter, PreDeployedArea.ToString("0.###")));
            b.AppendLine(Localizer.Format("RCLGUI10", deployedDiameter, DeployedArea.ToString("0.###")));
            GUILayout.Label(b.ToString());

            //DeploymentSafety
            switch (safeState)
            {
                case SafeState.SAFE:
                    GUILayout.Label(Localizer.Format("RCLGUISafe"));
                    break;

                case SafeState.RISKY:
                    GUILayout.Label(Localizer.Format("RCLGUIRisky"), YellowLabel);
                    break;

                case SafeState.DANGEROUS:
                    GUILayout.Label(Localizer.Format("RCLGUIDang"), RedLabel);
                    break;
            }

            //Temperature info
            b = new StringBuilder();
            b.AppendLine(Localizer.Format("RCLGUI11", (maxTemp + absoluteZero).ToString(CultureInfo.InvariantCulture)));
            b.AppendLine(Localizer.Format("RCLGUI12",
                                          Math.Round(chuteTemperature + absoluteZero,
                                                     1,
                                                     MidpointRounding.AwayFromZero)));
            GUILayout.Label(b.ToString(), chuteTemperature / maxTemp > 0.85 ? RedLabel : GUI.skin.label);

            //Predeployment pressure selection
            GUILayout.Label(Localizer.Format("RCLGUI13", minAirPressureToOpen));
            if (HighLogic.LoadedSceneIsFlight)
                //Predeployment pressure slider
                minAirPressureToOpen = GUILayout.HorizontalSlider(minAirPressureToOpen, 0.005f, 1);

            //Deployment altitude selection
            GUILayout.Label(Localizer.Format("RCLGUI14", deployAltitude));
            if (HighLogic.LoadedSceneIsFlight)
                //Deployment altitude slider
                deployAltitude = GUILayout.HorizontalSlider(deployAltitude, 50, 10000);

            //Other labels
            b = new StringBuilder();
            b.AppendLine(Localizer.Format("RCLGUI15",
                                          Math.Round(1 / semiDeploymentSpeed, 1, MidpointRounding.AwayFromZero)));
            b.AppendLine(Localizer.Format("RCLGUI16",
                                          Math.Round(1 / deploymentSpeed, 1, MidpointRounding.AwayFromZero)));
            GUILayout.Label(b.ToString());

            //End scroll
            GUILayout.EndScrollView();

            //Copy button if in flight
            if (HighLogic.LoadedSceneIsFlight && part.symmetryCounterparts.Count > 0)
                CenteredButton(Localizer.Format("RCLGUICopy"), CopyToCounterparts);

            //Close button
            CenteredButton(Localizer.Format("RCLGUIClose"), () => visible = false);

            //Closer
            GUILayout.EndVertical();
        }

        //Creates a centered GUI button
        public static void CenteredButton(string text, Callback callback)
        {
            GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            if (GUILayout.Button(text, HighLogic.Skin.button, GUILayout.Width(150)))
                callback();
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
        }

        private void Update()
        {
            if (!HighLogic.LoadedSceneIsFlight)
                return;

            //Makes the chute icon blink if failed
            if (failedTimer.IsRunning)
            {
                double totalSeconds = failedTimer.Elapsed.TotalSeconds;
                if (totalSeconds <= 2.5)
                {
                    if (!displayed)
                    {
                        ScreenMessages.PostScreenMessage(Localizer.Format("RCLFailDeploy"),
                                                         2.5f,
                                                         ScreenMessageStyle.UPPER_CENTER);
                        if (part.ShieldedFromAirstream)
                            ScreenMessages.PostScreenMessage(Localizer.Format("RCLFailShielded"),
                                                             2.5f,
                                                             ScreenMessageStyle.UPPER_CENTER);
                        else if (GroundStop)
                            ScreenMessages.PostScreenMessage(Localizer.Format("RCLFailGround"),
                                                             2.5f,
                                                             ScreenMessageStyle.UPPER_CENTER);
                        else if (atmPressure.NearlyEqual(0))
                            ScreenMessages.PostScreenMessage(Localizer.Format("RCLFailPres"),
                                                             2.5f,
                                                             ScreenMessageStyle.UPPER_CENTER);
                        else
                            ScreenMessages.PostScreenMessage(Localizer.Format("RCLFailOther"),
                                                             2.5f,
                                                             ScreenMessageStyle.UPPER_CENTER);
                        displayed = true;
                    }

                    if (totalSeconds < 0.5 || totalSeconds >= 1 && totalSeconds < 1.5 || totalSeconds >= 2)
                        part.stackIcon.SetIconColor(XKCDColors.Red);
                    else
                        part.stackIcon.SetIconColor(XKCDColors.White);
                }
                else
                {
                    displayed = false;
                    part.stackIcon.SetIconColor(XKCDColors.White);
                    failedTimer.Reset();
                }
            }

            Disarm.active = armed || showDisarm;
            DeployE.active = !staged && DeploymentState != DeploymentStates.CUT;
            CutE.active = IsDeployed;
            Repack.guiActiveUnfocused = CanRepack;
        }

        private void FixedUpdate()
        {
            //Flight values
            if (!HighLogic.LoadedSceneIsFlight || FlightGlobals.ActiveVessel == null || part.Rigidbody == null)
                return;
            pos = part.partTransform.position;
            asl = FlightGlobals.getAltitudeAtPos(pos);
            trueAlt = asl;
            if (vessel.mainBody.pqsController != null)
            {
                double terrainAlt = vessel.pqsAltitude;
                if (!vessel.mainBody.ocean || terrainAlt > 0)
                    trueAlt -= terrainAlt;
            }

            atmPressure = FlightGlobals.getStaticPressure(asl, vessel.mainBody) * PhysicsGlobals.KpaToAtmospheres;
            atmDensity = part.atmDensity;
            Vector3 velocity = part.Rigidbody.velocity + Krakensbane.GetFrameVelocityV3f();
            sqrSpeed = velocity.sqrMagnitude;
            dragVector = -velocity.normalized;

            if (atmDensity > 0)
                CalculateChuteFlux();
            else
                convFlux = 0;

            CalculateSafeToDeployEstimate();

            if (!staged)
                return;
            //Checks if the parachute must disarm
            if (armed)
            {
                part.stackIcon.SetIconColor(XKCDColors.LightCyan);
                if (CanDeploy)
                    armed = false;
            }
            //Parachute deployments
            else
            {
                //Parachutes
                if (CanDeploy)
                {
                    if (IsDeployed)
                    {
                        if (!CalculateChuteTemp())
                            return;
                        FollowDragDirection();
                    }

                    part.GetComponentCached(ref rigidbody);
                    switch (DeploymentState)
                    {
                        case DeploymentStates.STOWED:
                        {
                            part.stackIcon.SetIconColor(XKCDColors.LightCyan);
                            if (PressureCheck && RandomDeployment)
                                PreDeploy();
                            break;
                        }

                        case DeploymentStates.PREDEPLOYED:
                        {
                            part.AddForceAtPosition(DragForce(0, preDeployedDiameter, 1f / semiDeploymentSpeed),
                                                    ForcePosition);
                            if (trueAlt <= deployAltitude && dragTimer.Elapsed.TotalSeconds >= 1f / semiDeploymentSpeed)
                                Deploy();
                            break;
                        }

                        case DeploymentStates.DEPLOYED:
                        {
                            part.AddForceAtPosition(DragForce(preDeployedDiameter,
                                                              deployedDiameter,
                                                              1f / deploymentSpeed),
                                                    ForcePosition);
                            break;
                        }
                    }
                }
                //Deactivation
                else
                {
                    if (IsDeployed)
                    {
                        Cut();
                    }
                    else
                    {
                        failedTimer.Start();
                        StagingReset();
                    }
                }
            }
        }

        private void OnGUI()
        {
            if (!HighLogic.LoadedSceneIsFlight && !HighLogic.LoadedSceneIsEditor || !visible || hid)
                return;
            GUI.skin = HighLogic.Skin;
            window = GUILayout.Window(id, window, Window, Localizer.Format("RCLGUITitle"));
        }

        private void OnDestroy()
        {
            if (!HighLogic.LoadedSceneIsFlight && !HighLogic.LoadedSceneIsEditor)
                return;
            //Hide/show UI event removal
            GameEvents.onHideUI.Remove(HideUI);
            GameEvents.onShowUI.Remove(ShowUI);
            GameEvents.onStageActivate.Remove(OnStageActivate);
        }

        private void OnStageActivate(int stage)
        {
            if (!staged && stage == part.inverseStage)
                ActivateRC();
        }

        public override void OnStart(StartState startState)
        {
            if (!HighLogic.LoadedSceneIsEditor && !HighLogic.LoadedSceneIsFlight)
                return;

            //Staging icon
            part.stagingIcon = "PARACHUTES";
            InitializeAnimationSystem();

            //First initiation of the part
            if (!initiated)
            {
                initiated = true;
                armed = false;
                chuteCount = maxSpares;
                cap?.gameObject.SetActive(true);
            }

            float tmpPartMass = TotalMass;
            massDelta = 0;
            if (!(part.partInfo?.partPrefab is null))
                massDelta = tmpPartMass - part.partInfo.partPrefab.mass;

            //Flight loading
            if (HighLogic.LoadedSceneIsFlight)
            {
                var random = new Random();
                randomTime = (float)random.NextDouble();
                randomX = (float)(random.NextDouble() * 100);
                randomY = (float)(random.NextDouble() * 100);

                //Hide/show UI event addition
                GameEvents.onHideUI.Add(HideUI);
                GameEvents.onShowUI.Add(ShowUI);
                GameEvents.onStageActivate.Add(OnStageActivate);

                if (CanRepack)
                    SetRepack();

                if (!time.NearlyEqual(0))
                    dragTimer = new PhysicsWatch(time);
                if (DeploymentState != DeploymentStates.STOWED)
                {
                    part.stackIcon.SetIconColor(XKCDColors.Red);
                    cap?.gameObject.SetActive(false);
                }

                if (staged && IsDeployed)
                {
                    parachute.gameObject.SetActive(true);
                    switch (DeploymentState)
                    {
                        case DeploymentStates.PREDEPLOYED:
                            part.SkipToAnimationTime(semiDeployedAnimation, semiDeploymentSpeed, Mathf.Clamp01(time));
                            break;
                        case DeploymentStates.DEPLOYED:
                            part.SkipToAnimationTime(fullyDeployedAnimation, deploymentSpeed, Mathf.Clamp01(time));
                            break;
                    }
                }

                DragCubeList cubes = part.DragCubes;
                //Set stock cubes to 0
                cubes.SetCubeWeight("PACKED", 0);
                cubes.SetCubeWeight("SEMIDEPLOYED", 0);
                cubes.SetCubeWeight("DEPLOYED", 0);

                //Sets RC cubes
                if (DeploymentState == DeploymentStates.STOWED)
                {
                    cubes.SetCubeWeight("PACKED", 1);
                    cubes.SetCubeWeight("RCDEPLOYED", 0);
                }
                else
                {
                    cubes.SetCubeWeight("PACKED", 0);
                    cubes.SetCubeWeight("RCDEPLOYED", 1);
                }
            }

            //GUI
            window = new Rect(200, 100, 350, 400);
            drag = new Rect(0, 0, 350, 30);
        }

        public override void OnLoad(ConfigNode node)
        {
            if (HighLogic.LoadedScene == GameScenes.LOADING || !PartLoader.Instance.IsReady() || part.partInfo == null)
            {
                if (deployAltitude <= 500)
                    deployAltitude += 200;
                node.TryGetValue("shieldedCanDeploy", ref shieldedCanDeploy);
                return;
            }

            Part prefab = part.partInfo.partPrefab;
            massDelta = prefab == null ? 0 : TotalMass - prefab.mass;
            shieldedCanDeploy = prefab.FindModuleImplementingFast<RealChuteFAR>().shieldedCanDeploy;
        }

        public override void OnActive()
        {
            if (!staged)
                ActivateRC();
        }

        public override bool IsStageable()
        {
            return true;
        }
    }
}
