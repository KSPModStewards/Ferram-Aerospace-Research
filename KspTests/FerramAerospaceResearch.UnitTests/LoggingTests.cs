using System;
using UnityEngine;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// FAR's log (FARLogHandler and the static FARLogger in front of it): each level goes to Unity's log with its log
    /// type, tag and formatted text, and a message below the handler's level is dropped. Other tests log at the same
    /// time (TestLog is shared), so each message here is unique and the checks look for it, not at the whole log.
    /// </summary>
    [Collection(TestLog.Collection)]
    public class LoggingTests
    {
        public LoggingTests()
        {
            UnityRuntime.RequireLog();
        }

        private static string Unique(string what) => $"{what} {Guid.NewGuid():N}";

        private static void AssertLogged(LogType type, string text) =>
            Assert.Contains(TestLog.Instance.Lines, l => l.StartsWith(type + ": ") && l.Contains(text));

        private static void AssertNotLogged(string text) =>
            Assert.DoesNotContain(TestLog.Instance.Lines, l => l.Contains(text));

        [SkippableTheory]
        [InlineData(LogLevel.Trace, LogType.Log)]
        [InlineData(LogLevel.Debug, LogType.Log)]
        [InlineData(LogLevel.Info, LogType.Log)]
        [InlineData(LogLevel.Warning, LogType.Warning)]
        [InlineData(LogLevel.Assert, LogType.Assert)]
        [InlineData(LogLevel.Error, LogType.Error)]
        [InlineData(LogLevel.Exception, LogType.Exception)]
        public void EachLevelHasItsLogType(LogLevel level, LogType type)
        {
            var handler = new FARLogHandler(LogLevel.Trace);
            string plain = Unique("plain"), formatted = Unique("formatted");
            handler.Log(level, "[tag]", plain);
            handler.LogFormat(level, "[tag]", "{0} = {1}", formatted, 42);
            AssertLogged(type, $"[tag]: {plain}");
            AssertLogged(type, $"[tag]: {formatted} = 42");
        }

        [SkippableFact]
        public void AnUnknownLevelIsRefused()
        {
            var handler = new FARLogHandler((LogLevel)0);
            Assert.Throws<ArgumentException>(() => handler.Log((LogLevel)5, "[tag]", "nothing"));
        }

        [SkippableFact]
        public void EachMethodWritesItsLevel()
        {
            var handler = new FARLogHandler(LogLevel.Debug);
            string debug = Unique("debug"), debugF = Unique("debugF"), warn = Unique("warn"), error = Unique("error");
            string exc = Unique("exc");
            var exception = new InvalidOperationException(Unique("the exception"));

            handler.LogDebug("[tag]", debug);
            handler.LogDebugFormat("[tag]", "{0}!", debugF);
            handler.LogWarningFormat("[tag]", "{0}!", warn);
            handler.LogErrorFormat("[tag]", "{0}!", error);
            handler.LogExceptionFormat(exception, "[tag]", "{0}!", exc);

            AssertLogged(LogType.Log, $"[tag] debug: {debug}");
            AssertLogged(LogType.Log, $"[tag] debug: {debugF}!");
            AssertLogged(LogType.Warning, $"[tag]: {warn}!");
            AssertLogged(LogType.Error, $"[tag]: {error}!");
            AssertLogged(LogType.Exception, $"[tag]: {exc}!");
            AssertLogged(LogType.Exception, exception.Message);   // the exception itself, after the message
        }

        [SkippableFact]
        public void MessagesBelowTheLevelAreDropped()
        {
            var handler = new FARLogHandler(LogLevel.Error);
            string debug = Unique("debug"), warn = Unique("warn"), error = Unique("error");
            handler.LogDebug("[tag]", debug);
            handler.LogDebugFormat("[tag]", "{0}", debug);
            handler.LogWarningFormat("[tag]", "{0}", warn);
            handler.LogFormat(LogLevel.Info, "[tag]", "{0}", warn);
            handler.LogErrorFormat("[tag]", "{0}", error);
            AssertNotLogged(debug);
            AssertNotLogged(warn);
            AssertLogged(LogType.Error, error);

            // above Exception nothing is logged, not even exceptions
            var silent = new FARLogHandler((LogLevel)100);
            string exc = Unique("exc");
            var exception = new InvalidOperationException(exc);
            silent.LogException(exception, "[tag]", exc);
            silent.LogExceptionFormat(exception, "[tag]", "{0}", exc);
            AssertNotLogged(exc);
        }

        [SkippableFact]
        public void FarLoggerFormatsWithItsTag()
        {
            string warn = Unique("warn"), error = Unique("error"), exc = Unique("exc");
            FARLogger.WarningFormat("{0} warned", warn);
            FARLogger.ErrorFormat("{0} failed", error);
            FARLogger.ExceptionFormat(new InvalidOperationException("boom"), "{0} threw", exc);
            AssertLogged(LogType.Warning, $"{FARLogger.Tag}: {warn} warned");
            AssertLogged(LogType.Error, $"{FARLogger.Tag}: {error} failed");
            AssertLogged(LogType.Exception, $"{FARLogger.Tag}: {exc} threw");
        }
    }
}
