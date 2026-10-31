using System.Collections.Generic;
using FerramAerospaceResearch.Reflection;
using KSPCommunityFixes;

namespace FerramAerospaceResearch.Settings
{
    [ConfigNode("Resources")]
    public class ResourceSettings
    {
        [ConfigValue("excludeRes")]
        public readonly List<string> Excluded = new List<string>();

        [ConfigValue("res")]
        public readonly List<string> Resources = new List<string>();

        // ReSharper disable once FieldCanBeMadeReadOnly.Global
        // ReSharper disable once ConvertToConstant.Global
        [ConfigValue("flowMode")]
        public ResourceFlowMode FlowMode = ResourceFlowMode.NULL;

        [ConfigValue("numReq")]
        public int NumRequired = 0;

        [ConfigValue("rejectUnlistedResources")]
        public bool RejectUnlisted = false;
    }


    [ConfigNode("FARPartStressTemplate")]
    public class FARPartStressTemplate : Interfaces.IConfigNode
    {
        [ConfigValue]
        public readonly ResourceSettings Resources = new ResourceSettings();

        [ConfigValueIgnore]
        public bool FlowModeNeeded;

        [ConfigValue("isSpecialTemplate")]
        public bool IsSpecial = false;

        [ConfigValue("name")]
        public string Name = "default";

        [ConfigValue("requiresCrew")]
        public bool RequiresCrew = false;

        [ConfigValue("XZmaxStress")]
        public double XZMaxStress = 500;

        [ConfigValue("YmaxStress")]
        public double YMaxStress = 500;

        public void BeforeLoaded()
        {
        }

        public void AfterLoaded()
        {
            FlowModeNeeded = Resources.FlowMode == ResourceFlowMode.ALL_VESSEL ||
                             Resources.FlowMode == ResourceFlowMode.STACK_PRIORITY_SEARCH ||
                             Resources.FlowMode == ResourceFlowMode.STAGE_PRIORITY_FLOW ||
                             Resources.FlowMode == ResourceFlowMode.NO_FLOW;

            Resources.Resources.RemoveAll(s => !PartResourceLibrary.Instance.resourceDefinitions.Contains(s));
            Resources.Excluded.RemoveAll(s => !PartResourceLibrary.Instance.resourceDefinitions.Contains(s));
        }

        public void BeforeSaved()
        {
        }

        public void AfterSaved()
        {
        }
    }

    [ConfigNode("FARAeroStress", true)]
    public static class FARAeroStress
    {
        [ConfigValue]
        public static readonly List<FARPartStressTemplate> StressTemplates = new List<FARPartStressTemplate>();


        public static FARPartStressTemplate DetermineStressTemplate(Part p)
        {
            FARPartStressTemplate template = StressTemplates[0];

            int resCount = p.Resources.Count;
            bool crewed = p.CrewCapacity > 0;
            if (p.HasModuleImplementingFast<ModuleAblator>() || p.Resources.Contains("Ablator"))
                return template;

            foreach (FARPartStressTemplate candidate in StressTemplates)
            {
                if (candidate.IsSpecial)
                    continue;
                if (candidate.RequiresCrew != crewed)
                    continue;

                if (resCount < candidate.Resources.NumRequired)
                    continue;

                if (candidate.Resources.RejectUnlisted)
                {
                    bool cont = true;
                    int numRes = 0;
                    foreach (PartResource res in p.Resources)
                        if (candidate.Resources.Resources.Contains(res.info.name))
                        {
                            numRes++;
                            cont = false;
                        }
                        else
                        {
                            cont = true;
                            break;
                        }

                    if (cont || numRes < candidate.Resources.NumRequired)
                        continue;
                }
                else
                {
                    int numRes = 0;
                    // ReSharper disable once LoopCanBeConvertedToQuery
                    foreach (PartResource res in p.Resources)
                        if (!candidate.Resources.Excluded.Contains(res.info.name))
                            if (!candidate.FlowModeNeeded || res.info.resourceFlowMode == candidate.Resources.FlowMode)
                                numRes++;


                    if (numRes < candidate.Resources.NumRequired)
                        continue;
                }

                template = candidate;
            }


            return template;
        }
    }
}
