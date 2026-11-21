using System.Collections.Generic;
using System.Linq;
using KSP.UI.Screens;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>The flight staging list: what is in it, and what should not be.</summary>
    internal static class Staging
    {
        private static IEnumerable<StageIcon> Icons() =>
            StageManager.Instance.Stages.SelectMany(s => s.Icons).Where(i => i != null);

        /// <summary>The parts with an icon in the staging list.</summary>
        internal static List<Part> Parts() => Icons().Select(i => i.ProtoIcon?.Part).Where(p => p != null).ToList();

        /// <summary>The staging list's parts that are not on the active vessel, by name.</summary>
        internal static List<string> Stray() =>
            Parts().Where(p => p.vessel != FlightGlobals.ActiveVessel).Select(p => p.partInfo.name).ToList();

        /// <summary>The stages left with no icon in them.</summary>
        internal static int EmptyStages() => StageManager.Instance.Stages.Count(s => s.Icons.All(i => i == null));

        /// <summary>A part and everything below it in the part tree.</summary>
        internal static HashSet<Part> Subtree(Part p) => new HashSet<Part>(p.FindChildParts<Part>(true)) { p };
    }
}
