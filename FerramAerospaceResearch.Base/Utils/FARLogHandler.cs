using System;
using System.Runtime.CompilerServices;
using UnityEngine;
using UnityEngine.Assertions;
using Debug = UnityEngine.Debug;

namespace FerramAerospaceResearch
{
    // UnityEngine.LogType is weird, Exception is at the wrong end and doesn't have runtime debug level...
    public enum LogLevel
    {
        Trace = 10,
        Debug = 20,
        Info = 30,
        Warning = 40,
        Assert = 50,
        Error = 60,
        Exception = 70
    }

    public class FARLogHandler
    {
#if DEBUG
        private const LogLevel DefaultLevel = LogLevel.Debug;
#else
        private const LogLevel DefaultLevel = LogLevel.Info;
#endif

        public FARLogHandler(LogLevel level = DefaultLevel)
        {
            Level = level;
        }

        public ILogHandler LogHandler { get; } = Debug.unityLogger.logHandler;
        public LogLevel Level { get; set; }

        public void Log(LogLevel logLevel, string tag, object message)
        {
            if (EnabledFor(logLevel))
                LogHandler.LogFormat(Convert(logLevel), null, "{0}: {1}", tag, message);
        }

        public void LogTrace(string tag, object message)
        {
            if (EnabledFor(LogLevel.Trace))
                LogHandler.LogFormat(LogType.Log, null, "{0} trace: {1}", tag, message);
        }

        public void LogInfo(string tag, object message)
        {
            if (EnabledFor(LogLevel.Info))
                LogHandler.LogFormat(LogType.Log, null, "{0}: {1}", tag, message);
        }

        public void LogDebug(string tag, object message)
        {
            if (EnabledFor(LogLevel.Debug))
                LogHandler.LogFormat(LogType.Log, null, "{0} debug: {1}", tag, message);
        }

        public void LogWarning(string tag, object message)
        {
            if (EnabledFor(LogLevel.Warning))
                LogHandler.LogFormat(LogType.Warning, null, "{0}: {1}", tag, message);
        }

        public void LogError(string tag, object message)
        {
            if (EnabledFor(LogLevel.Error))
                LogHandler.LogFormat(LogType.Error, null, "{0}: {1}", tag, message);
        }

        public void LogException(Exception exception, string tag, object message)
        {
            if (!EnabledFor(LogLevel.Exception))
                return;
            LogHandler.LogFormat(LogType.Exception, null, "{0}: {1}", tag, message);
            LogHandler.LogException(exception, null);
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public void LogAssert(bool condition, string tag, object message)
        // {
        //     if (condition)
        //         return;

        //     if (EnabledFor(LogLevel.Assert))
        //         LogHandler.LogFormat(LogType.Assert, null, "{0}: {1}", tag, message);

        //     // throw an assertion error to match behaviour with assert in other languages
        //     throw new AssertionException("Assertion condition failed", "");
        // }

        public void LogFormat(LogLevel logLevel, string tag, string format, params object[] args)
        {
            if (EnabledFor(logLevel))
                LogHandler.LogFormat(Convert(logLevel), null, "{0}: {1}", tag, string.Format(format, args));
        }

        public void LogTraceFormat(string tag, string format, params object[] args)
        {
            if (EnabledFor(LogLevel.Trace))
                LogHandler.LogFormat(LogType.Log, null, "{0} trace: {1}", tag, string.Format(format, args));
        }

        public void LogInfoFormat(string tag, string format, params object[] args)
        {
            if (EnabledFor(LogLevel.Info))
                LogHandler.LogFormat(LogType.Log, null, "{0}: {1}", tag, string.Format(format, args));
        }

        public void LogDebugFormat(string tag, string format, params object[] args)
        {
            if (EnabledFor(LogLevel.Debug))
                LogHandler.LogFormat(LogType.Log, null, "{0} debug: {1}", tag, string.Format(format, args));
        }

        public void LogWarningFormat(string tag, string format, params object[] args)
        {
            if (EnabledFor(LogLevel.Warning))
                LogHandler.LogFormat(LogType.Warning, null, "{0}: {1}", tag, string.Format(format, args));
        }

        public void LogErrorFormat(string tag, string format, params object[] args)
        {
            if (EnabledFor(LogLevel.Error))
                LogHandler.LogFormat(LogType.Error, null, "{0}: {1}", tag, string.Format(format, args));
        }

        public void LogExceptionFormat(Exception exception, string tag, string format, params object[] args)
        {
            if (!EnabledFor(LogLevel.Exception))
                return;
            LogHandler.LogFormat(LogType.Exception, null, "{0}: {1}", tag, string.Format(format, args));
            LogHandler.LogException(exception, null);
        }

        public void LogAssertFormat(bool condition, string tag, string format, params object[] args)
        {
            if (condition)
                return;

            if (EnabledFor(LogLevel.Assert))
                LogHandler.LogFormat(LogType.Assert, null, "{0}: {1}", tag, string.Format(format, args));

            // throw an assertion error to match behaviour with assert in other languages
            throw new AssertionException("Assertion condition failed", "");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool EnabledFor(LogLevel logLevel)
        {
            return Level <= logLevel;
        }

        private static LogType Convert(LogLevel logLevel)
        {
            return logLevel switch
            {
                LogLevel.Assert => LogType.Assert,
                LogLevel.Debug => LogType.Log,
                LogLevel.Error => LogType.Error,
                LogLevel.Exception => LogType.Exception,
                LogLevel.Info => LogType.Log,
                LogLevel.Warning => LogType.Warning,
                LogLevel.Trace => LogType.Log,
                _ => throw new ArgumentException("Should never reach here")
            };
        }
    }
}
