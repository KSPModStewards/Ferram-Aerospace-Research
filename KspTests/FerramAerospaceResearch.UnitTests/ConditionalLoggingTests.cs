// FARLogger's Trace and Assert methods are [Conditional]: their calls are compiled in only where the calling file
// defines the symbol, so this file does (FAR's Release build does not, so its own calls to them are compiled out).
#define LOG_TRACE
#define ASSERT

using System;
using System.Diagnostics;
using System.Reflection;
using System.Runtime.Serialization;
using System.Threading;
using FerramAerospaceResearch.FARThreading;
using FerramAerospaceResearch.Threading;
using UnityEngine;
using AssertionException = UnityEngine.Assertions.AssertionException;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// The log calls FAR compiles out of Release builds (FARLogger's Trace and Assert, and the handler methods behind
    /// them), FARLogger.LogFormat, and ThreadSafeDebugLogger's Warning and Error, which queue a message for the main
    /// thread's Update to log. Each message is unique and the checks look for it, as in LoggingTests.
    /// </summary>
    [Collection(TestLog.Collection)]
    public class ConditionalLoggingTests
    {
        public ConditionalLoggingTests()
        {
            UnityRuntime.RequireLog();
        }

        private static string Unique(string what) => $"{what} {Guid.NewGuid():N}";

        private static void AssertLogged(LogType type, string text) =>
            Assert.Contains(TestLog.Instance.Lines, l => l.StartsWith(type + ": ") && l.Contains(text));

        private static void AssertNotLogged(string text) =>
            Assert.DoesNotContain(TestLog.Instance.Lines, l => l.Contains(text));

        /// <summary>Trace messages are logged as "trace" lines at the Trace level and dropped above it.</summary>
        [SkippableFact]
        public void TraceLogsOnlyAtTraceLevel()
        {
            var handler = new FARLogHandler(LogLevel.Trace);
            string plain = Unique("plain"), formatted = Unique("formatted");
            handler.LogTrace("[tag]", plain);
            handler.LogTraceFormat("[tag]", "{0} = {1}", formatted, 7);
            AssertLogged(LogType.Log, $"[tag] trace: {plain}");
            AssertLogged(LogType.Log, $"[tag] trace: {formatted} = 7");

            var quiet = new FARLogHandler(LogLevel.Debug);
            string dropped = Unique("dropped");
            quiet.LogTrace("[tag]", dropped);
            quiet.LogTraceFormat("[tag]", "{0}", dropped);
            AssertNotLogged(dropped);
        }

        [SkippableFact]
        public void FarLoggerTraceUsesItsTagAndLevel()
        {
            LogLevel saved = FARLogger.Level;
            try
            {
                string dropped = Unique("dropped");
                FARLogger.Level = LogLevel.Debug;
                FARLogger.Trace(dropped);
                FARLogger.TraceFormat("{0}", dropped);
                AssertNotLogged(dropped);

                FARLogger.Level = LogLevel.Trace;
                string plain = Unique("plain"), formatted = Unique("formatted");
                FARLogger.Trace(plain);
                FARLogger.TraceFormat("{0}!", formatted);
                AssertLogged(LogType.Log, $"{FARLogger.Tag} trace: {plain}");
                AssertLogged(LogType.Log, $"{FARLogger.Tag} trace: {formatted}!");
            }
            finally
            {
                FARLogger.Level = saved;
            }
        }

        /// <summary>
        /// A failed assert logs an Assert line and throws Unity's AssertionException (as asserts do elsewhere), even
        /// when its level is below the handler's; a passing one does neither.
        /// </summary>
        [SkippableFact]
        public void FailedAssertLogsAndThrows()
        {
            var handler = new FARLogHandler(LogLevel.Trace);
            string passed = Unique("passed"), failed = Unique("failed"), silent = Unique("silent");
            handler.LogAssertFormat(true, "[tag]", "{0}", passed);
            AssertNotLogged(passed);

            Assert.Throws<AssertionException>(() => handler.LogAssertFormat(false, "[tag]", "{0}!", failed));
            AssertLogged(LogType.Assert, $"[tag]: {failed}!");

            Assert.Throws<AssertionException>(() => new FARLogHandler(LogLevel.Error).LogAssertFormat(false, "[tag]", "{0}", silent));
            AssertNotLogged(silent);
        }

        /// <summary>FARLogger.Assert writes where it was called from: file(line): member: message.</summary>
        [SkippableFact]
        public void FarLoggerAssertNamesItsSource()
        {
            string passed = Unique("passed");
            FARLogger.Assert(true, passed);
            AssertNotLogged(passed);

            string plain = Unique("plain");
            Assert.Throws<AssertionException>(() => FARLogger.Assert(false, plain, "Member", "File.cs", 12));
            AssertLogged(LogType.Assert, $"{FARLogger.Tag}: File.cs(12): Member: {plain}");

            // the caller info defaults to the calling member
            string here = Unique("here");
            Assert.Throws<AssertionException>(() => FARLogger.Assert(false, here));
            Assert.Contains(TestLog.Instance.Lines, l => l.Contains(nameof(FarLoggerAssertNamesItsSource)) && l.Contains(here));
        }

        // FAR's Debug builds, the only ones defining ASSERT, are built without the JIT optimizer
        private static bool FarBuiltWithAsserts =>
            typeof(FARLogger).Assembly.GetCustomAttribute<DebuggableAttribute>()?.IsJITOptimizerDisabled == true;

        /// <summary>
        /// FARLogger.AssertFormat formats the message and hands it to FARLogger.Assert with the source it was given. That
        /// inner call is itself [Conditional("ASSERT")], so it is in FAR only when FAR is built with ASSERT: in a Release
        /// build AssertFormat does nothing, even for a caller that defines ASSERT.
        /// </summary>
        [SkippableFact]
        public void FarLoggerAssertFormatAssertsOnlyInAnAssertBuild()
        {
            string passed = Unique("passed");
            FARLogger.AssertFormat(true, "{0}", "m", "f", 1, passed);
            FARLogger.AssertFormat(true, "{0}", SourceInfo.Current("m", "f", 1), passed);
            AssertNotLogged(passed);

            string formatted = Unique("formatted"), sourced = Unique("sourced");
            SourceInfo source = SourceInfo.Current("Caller", "Source.cs", 14);
            Exception byArgs = Record.Exception(() => FARLogger.AssertFormat(false, "{0}+{1}", "Member", "File.cs", 13, formatted, 2));
            Exception bySource = Record.Exception(() => FARLogger.AssertFormat(false, "{0}-{1}", source, sourced, 3));

            if (FarBuiltWithAsserts)
            {
                Assert.IsType<AssertionException>(byArgs);
                Assert.IsType<AssertionException>(bySource);
                AssertLogged(LogType.Assert, $"{FARLogger.Tag}: File.cs(13): Member: {formatted}+2");
                AssertLogged(LogType.Assert, $"{FARLogger.Tag}: Source.cs(14): Caller: {sourced}-3");
            }
            else
            {
                Assert.Null(byArgs);
                Assert.Null(bySource);
                AssertNotLogged(formatted);
                AssertNotLogged(sourced);
            }
        }

        [SkippableFact]
        public void FarLoggerLogFormatUsesTheGivenLevel()
        {
            string warn = Unique("warn"), error = Unique("error");
            FARLogger.LogFormat(LogLevel.Warning, "{0} at {1}", warn, 1);
            FARLogger.LogFormat(LogLevel.Error, "{0} at {1}", error, 2);
            AssertLogged(LogType.Warning, $"{FARLogger.Tag}: {warn} at 1");
            AssertLogged(LogType.Error, $"{FARLogger.Tag}: {error} at 2");
        }

        /// <summary>
        /// ThreadSafeDebugLogger's Warning and Error only queue the message; the logger's Update (on the main thread in
        /// the game) logs it at its level and empties the queue. The logger is a component, made here without Unity.
        /// </summary>
        [SkippableFact]
        public void ThreadSafeWarningsAndErrorsWaitForUpdate()
        {
            const BindingFlags instance = BindingFlags.NonPublic | BindingFlags.Instance;
            PropertyInfo instanceProperty = typeof(ThreadSafeDebugLogger).GetProperty(nameof(ThreadSafeDebugLogger.Instance));
            FieldInfo messagesField = typeof(ThreadSafeDebugLogger).GetField("messages", instance);

            var logger = (ThreadSafeDebugLogger)FormatterServices.GetUninitializedObject(typeof(ThreadSafeDebugLogger));
            typeof(ThreadSafeDebugLogger).GetField("mutex", instance).SetValue(logger, new object());
            messagesField.SetValue(logger, Activator.CreateInstance(messagesField.FieldType));
            object saved = instanceProperty.GetValue(null);
            instanceProperty.SetValue(null, logger);
            try
            {
                string warn = Unique("warn"), error = Unique("error");
                ThreadSafeDebugLogger.Warning(warn);
                ThreadSafeDebugLogger.Error(error);
                AssertNotLogged(warn);
                AssertNotLogged(error);

                typeof(ThreadSafeDebugLogger).GetMethod("Update", instance).Invoke(logger, null);
                AssertLogged(LogType.Warning, $"{FARLogger.Tag}: {warn}");
                AssertLogged(LogType.Error, $"{FARLogger.Tag}: {error}");
                Assert.Empty((System.Collections.IList)messagesField.GetValue(logger));
            }
            finally
            {
                instanceProperty.SetValue(null, saved);
            }
        }
    }

    /// <summary>
    /// FARLogAnyThread.Exception logs the exception at once on FAR's main thread and queues it for the main thread from
    /// any other. In the prediction slots' collection: SimThreadContextTests sets FAR's main thread too.
    /// </summary>
    [Collection(Gas.Collection)]
    public class AnyThreadLoggingTests
    {
        private static readonly PropertyInfo mainThread = typeof(MainThread).GetProperty(nameof(MainThread.Thread));

        private static readonly FieldInfo mainThreadTasks =
            typeof(MainThread).GetField("tasks", BindingFlags.NonPublic | BindingFlags.Static);

        [SkippableFact]
        public void ExceptionsAreLoggedOnTheMainThread()
        {
            UnityRuntime.RequireLog();
            Thread saved = MainThread.Thread;
            try
            {
                mainThread.SetValue(null, Thread.CurrentThread);
                string message = Unique("message"), thrown = Unique("thrown");
                FARLogAnyThread.Exception(new InvalidOperationException(thrown), message);
                AssertLogged($"{FARLogger.Tag}: {message}");
                AssertLogged(thrown);

                // without a message it uses FARLogger's default
                string bare = Unique("bare");
                FARLogAnyThread.Exception(new InvalidOperationException(bare));
                Assert.Contains(TestLog.Instance.Lines, l => l.Contains($"{FARLogger.Tag}: {FARLogger.DefaultExceptionMessage}"));
                AssertLogged(bare);

                // on another thread it waits for the main thread to run its task queue
                mainThread.SetValue(null, new Thread(() => { }));
                string queued = Unique("queued");
                FARLogAnyThread.Exception(new InvalidOperationException(queued), queued);
                Assert.DoesNotContain(TestLog.Instance.Lines, l => l.Contains(queued));

                var tasks = (System.Collections.Concurrent.ConcurrentQueue<MainThread.Task>)mainThreadTasks.GetValue(null);
                while (tasks.TryDequeue(out MainThread.Task task))
                    MainThread.ExecuteTask(task);
                AssertLogged($"{FARLogger.Tag}: {queued}");
            }
            finally
            {
                mainThread.SetValue(null, saved);
            }
        }

        private static string Unique(string what) => $"{what} {Guid.NewGuid():N}";

        private static void AssertLogged(string text) =>
            Assert.Contains(TestLog.Instance.Lines, l => l.StartsWith(LogType.Exception + ": ") && l.Contains(text));
    }
}
