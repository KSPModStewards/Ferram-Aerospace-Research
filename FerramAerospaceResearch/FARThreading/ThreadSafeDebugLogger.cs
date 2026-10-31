/*
Ferram Aerospace Research v0.16.2.0 "Maskell"
=========================
Aerodynamics model for Kerbal Space Program

Copyright 2022, Michael Ferrara, aka Ferram4

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Ferram Aerospace Research is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Ferram Aerospace Research.  If not, see <http://www.gnu.org/licenses/>.

   Serious thanks:		a.g., for tons of bugfixes and code-refactorings
				stupid_chris, for the RealChuteLite implementation
            			Taverius, for correcting a ton of incorrect values
				Tetryds, for finding lots of bugs and issues and not letting me get away with them, and work on example crafts
            			sarbian, for refactoring code for working with MechJeb, and the Module Manager updates
            			ialdabaoth (who is awesome), who originally created Module Manager
                        	Regex, for adding RPM support
				DaMichel, for some ferramGraph updates and some control surface-related features
            			Duxwing, for copy editing the readme

   CompatibilityChecker by Majiir, BSD 2-clause http://opensource.org/licenses/BSD-2-Clause

   Part.cfg changes powered by sarbian & ialdabaoth's ModuleManager plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/55219

   ModularFLightIntegrator by Sarbian, Starwaster and Ferram4, MIT: http://opensource.org/licenses/MIT
	http://forum.kerbalspaceprogram.com/threads/118088

   Toolbar integration powered by blizzy78's Toolbar plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/60863
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

namespace FerramAerospaceResearch.FARThreading
{
    [KSPAddon(KSPAddon.Startup.Instantly, true)]
    internal class ThreadSafeDebugLogger : MonoBehaviour
    {
        private struct LogMessage
        {
            public LogLevel level;
            public string message;
            public Exception exception;
        }

        private object mutex = new();
        private readonly List<LogMessage> messages = new();
        public static ThreadSafeDebugLogger Instance { get; private set; }

        private void Awake()
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }

        private void Update()
        {
            lock (mutex)
            {
                foreach (LogMessage message in messages)
                {
                    if (message.exception is not null)
                        FARLogger.Exception(message.exception, message.message);
                    else
                        FARLogger.Log(message.level, message.message);
                }

                messages.Clear();
            }
        }

        public static void Log(LogLevel level, string message, Exception exception = null)
        {
            lock (Instance.mutex)
            {
                Instance.messages.Add(new LogMessage
                {
                    exception = exception,
                    level = level,
                    message = message
                });
            }
        }

        [Conditional("LOG_TRACE")]
        public static void Trace(string message)
        {
            Log(LogLevel.Trace, message);
        }

        public static void Debug(string message)
        {
            Log(LogLevel.Debug, message);
        }

        public static void Info(string message)
        {
            Log(LogLevel.Info, message);
        }

        public static void Warning(string message)
        {
            Log(LogLevel.Warning, message);
        }

        public static void Error(string message)
        {
            Log(LogLevel.Error, message);
        }

        public static void Exception(Exception exception, string message = null)
        {
            Log(LogLevel.Exception, message, exception);
        }
    }
}
