using System;
using System.Runtime.CompilerServices;
using UnityEngine;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// Whether Unity's managed code is there to run: it is with the game's own assemblies, not with stripped reference
    /// copies (KSPBuildTools' KSPLibs), where even Vector3's type initializer throws. Engine-native calls (Debug.Log,
    /// Quaternion.Euler, AnimationCurve, ...) throw either way: outside the game they have no engine to call.
    /// </summary>
    public static class UnityRuntime
    {
        private static readonly Lazy<bool> available = new Lazy<bool>(Probe);

        public static bool Available => available.Value;

        /// <summary>Skips the calling test without Unity's managed code.</summary>
        public static void Require() =>
            Skip.IfNot(Available, "Unity's assemblies are stripped (no method bodies): this test needs the game's own");

        /// <summary>Skips the calling test unless FAR's log can be written (and read back, TestLog).</summary>
        public static void RequireLog()
        {
            Require();
            Skip.IfNot(TestLog.Installed, "Unity's log could not be redirected: FAR cannot log outside the game");
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static bool Probe()
        {
            try
            {
                return Vector3.Dot(Vector3.up, Vector3.up) == 1f;
            }
            catch (TypeInitializationException)
            {
                return false;
            }
        }
    }
}
