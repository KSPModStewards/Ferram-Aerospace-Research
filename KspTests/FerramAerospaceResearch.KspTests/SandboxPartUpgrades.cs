using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The tests' sandbox games start with part upgrades applied, in the save. ProceduralParts turns them on itself, but
    /// only in the first game of a KSP process (a static flag): a session that used its parts and then reloaded the save
    /// got the reloaded game without them, and every procedural part held to its first tech level's sizes (the
    /// ProceduralParts twin of the Aeris loaded with its 3.75 m tank cut to 0.65 m).
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Instantly, true)]
    public class SandboxPartUpgrades : MonoBehaviour
    {
        private void Awake() => Nav.NewGameParameters.Add(Set);

        private static void Set(Game.Modes mode, GameParameters parameters)
        {
            if (mode == Game.Modes.SANDBOX)
                parameters.CustomParams<GameParameters.AdvancedParams>().PartUpgradesInSandbox = true;
        }
    }
}
