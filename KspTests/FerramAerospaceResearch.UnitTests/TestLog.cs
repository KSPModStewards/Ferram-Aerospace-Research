using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;
using Object = UnityEngine.Object;

namespace System.Runtime.CompilerServices
{
    // C# 9's module initializers on .NET Framework, which does not define the attribute
    [AttributeUsage(AttributeTargets.Method, Inherited = false)]
    internal sealed class ModuleInitializerAttribute : Attribute
    {
    }
}

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// Unity's log, kept in memory. Outside the game Debug.Log has no engine to write to and throws; FAR's logger
    /// copies Unity's log handler when it is first used, so this goes in before any test runs (the module initializer),
    /// and FAR's messages land here, where tests can read them. Needs Unity's managed code (UnityRuntime.Available).
    /// </summary>
    public sealed class TestLog : ILogHandler
    {
        /// <summary>
        /// The test collection of the tests that log errors or check the log for them: xunit runs collections side by
        /// side, so a test checking that nothing logged an error would otherwise see another test's error.
        /// </summary>
        public const string Collection = "FAR's log";

        public static readonly TestLog Instance = new TestLog();
        private readonly List<string> lines = new List<string>();

        /// <summary>Whether Unity's log goes here (false with stripped assemblies, where FAR cannot log at all).</summary>
        public static bool Installed { get; private set; }

        [ModuleInitializer]
        internal static void Install()
        {
            try
            {
                Debug.unityLogger.logHandler = Instance;
                Installed = true;
            }
            catch (TypeInitializationException)
            {
                // stripped assemblies: Debug's type initializer has no code to run
            }
        }

        /// <summary>The messages logged since the last Clear, as "Type: text".</summary>
        public IReadOnlyList<string> Lines
        {
            get { lock (lines) return lines.ToArray(); }
        }

        public void Clear()
        {
            lock (lines) lines.Clear();
        }

        public void LogFormat(LogType logType, Object context, string format, params object[] args)
        {
            lock (lines) lines.Add($"{logType}: {string.Format(format, args)}");
        }

        public void LogException(Exception exception, Object context)
        {
            lock (lines) lines.Add($"{LogType.Exception}: {exception}");
        }
    }
}
