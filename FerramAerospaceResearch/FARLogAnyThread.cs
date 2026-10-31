/*
Ferram Aerospace Research
=========================
Logging from any thread.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using FerramAerospaceResearch.Threading;

namespace FerramAerospaceResearch
{
    /// <summary>
    ///     FARLogger calls from code that may run on another thread (a parallel sweep's workers, a snapshot
    ///     prediction): logged at once on the main thread, else queued to it. Unity's log handler, which the game
    ///     hooks with its debug console, is not safe to call from another thread.
    /// </summary>
    internal static class FARLogAnyThread
    {
        public static void Info(object message)
        {
            OnMainThread(() => FARLogger.Info(message));
        }

        public static void Warning(object message)
        {
            OnMainThread(() => FARLogger.Warning(message));
        }

        public static void Exception(Exception exception, object message = null)
        {
            OnMainThread(() => FARLogger.Exception(exception, message));
        }

        private static void OnMainThread(Action log)
        {
            // Before the main thread is known (early startup) everything runs on it.
            if (MainThread.Thread == null || MainThread.IsMainThread)
                log();
            else
                MainThread.StartTask(log);
        }
    }
}
