using System;
using System.Collections.Generic;
using System.Linq;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// FAR exceptions logged while a test runs: ones FAR does not catch (its exposure's in LateUpdate) and ones it
    /// catches and logs (a voxelization thread's, logged from the main thread).
    /// </summary>
    internal sealed class ExceptionLog : IDisposable
    {
        private readonly List<string> messages = new List<string>();

        public ExceptionLog() => Application.logMessageReceived += OnLog;

        private void OnLog(string condition, string stackTrace, LogType type)
        {
            if (type == LogType.Exception && (stackTrace?.Contains("FerramAerospaceResearch") == true || condition.Contains("Exposure")))
                messages.Add(condition);
        }

        public int Count => messages.Count;

        public void Dispose() => Application.logMessageReceived -= OnLog;

        public void AssertNone(string what) =>
            Assert.True(messages.Count == 0, $"no FAR exceptions {what} ({messages.Count}: {string.Join(" | ", messages.Distinct().Take(3))})");
    }
}
