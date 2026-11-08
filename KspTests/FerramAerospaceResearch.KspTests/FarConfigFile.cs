using System.Diagnostics;
using System.IO;
using System.Reflection;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR's Custom_FARConfig.cfg kept as it was before a test changed FAR's settings. FAR writes the file whenever the
    /// game saves (ConfigAdapter.SaveAll, on a task) and its settings differ from what it last wrote, so a test's change
    /// is written as the test leaves flight, and its undoing only at the next game save after the test, if there is one
    /// in the session. Left there, the next session starts with the test's settings. Restore, once the settings are back,
    /// has FAR save them at once (ConfigAdapter.Save: later saves compare against it and find nothing to write) and
    /// removes the file if the test made it.
    /// </summary>
    internal sealed class FarConfigFile
    {
        private static readonly FieldInfo Saving = typeof(ConfigAdapter).GetField("saving", BindingFlags.Instance | BindingFlags.NonPublic);

        private readonly string path = Path.Combine(KSPUtil.ApplicationRootPath, "GameData", "Custom_FARConfig.cfg");
        private readonly bool existed;

        public FarConfigFile() => existed = File.Exists(path);

        /// <summary>The restored settings saved; true if the test had made the file (and it is removed).</summary>
        public bool Restore()
        {
            // a save already on its task could write the test's settings after this one
            var wait = Stopwatch.StartNew();
            while (Saving != null && (bool)Saving.GetValue(ConfigAdapter.Instance) && wait.ElapsedMilliseconds < 5000)
                System.Threading.Thread.Sleep(10);
            ConfigAdapter.Save();
            if (existed || !File.Exists(path))
                return false;
            File.Delete(path);
            return true;
        }
    }
}
