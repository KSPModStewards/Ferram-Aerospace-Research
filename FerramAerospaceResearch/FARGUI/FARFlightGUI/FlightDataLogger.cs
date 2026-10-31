/*
Ferram Aerospace Research v0.16.2.0 "Maskell"
=========================
Aerodynamics model for Kerbal Space Program

Copyright 2022, Daumantas Kavolis, aka dkavolis

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
using System.Collections;
using System.Collections.Generic;
using System.IO;
using KSP.Localization;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FARFlightGUI
{
    // inheriting from MonoBehaviour for coroutines and attaching to vessel
    internal class FlightDataLogger : MonoBehaviour
    {
        private const string VesselNameKey = "VESSEL_NAME";
        private const string DatetimeKey = "DATETIME";

        private static readonly string[] headers =
        {
            "time",
            "altitude",
            "speed",
            " lift",
            " drag",
            " lateral force",
            " dynamic pressure",
            " CL",
            " CD",
            " CQ",
            // " reference area",
            " L/D",
            " VL/D",
            " AoA",
            " sideslip",
            " pitch",
            " heading",
            " roll",
            // " dry mass",
            // " full mass",
            // " tSFC",
            // " intake air fraction",
            // " specific excess power",
            // " range",
            // " endurance",
            // " ballistic coefficient",
            // " terminal velocity",
            " stall fraction"
        };

        private readonly Dictionary<string, object> replacements = new Dictionary<string, object>
        {
            {VesselNameKey, "unknown"},
            {DatetimeKey, "no_date"}
        };

        private CsvWriter writer;

        private Coroutine coroutine;
        private bool headerWritten;

        public int Period { get; set; } = 50;
        public int FlushPeriod { get; set; } = 10;

        public bool IsActive
        {
            get { return coroutine != null; }
        }

        public Vessel Vessel { get; private set; }

        public string FileName
        {
            get { return writer.Filename; }
        }

        public static FlightDataLogger CreateLogger(Vessel vessel)
        {
            if (vessel == null)
                throw new ArgumentNullException(nameof(vessel));

            var logger = vessel.gameObject.AddComponent<FlightDataLogger>();
            logger.Vessel = vessel;
            logger.writer = new CsvWriter(logger.GetFilename());
            logger.Period = FARConfig.FlightLog.LogPeriod;
            logger.FlushPeriod = FARConfig.FlightLog.FlushPeriod;

            return logger;
        }

        public void StartLogging()
        {
            if (IsActive)
                return;
            coroutine = StartCoroutine(DoLogging());
        }

        public void StopLogging()
        {
            if (!IsActive)
                return;

            FARLogger.Debug($"Stopping logging to {FileName}");
            StopCoroutine(coroutine);
            writer.Close();
            coroutine = null;
        }

        public void PauseLogging()
        {
            if (!IsActive)
                return;

            FARLogger.Debug($"Pausing logging to {FileName}");
            StopCoroutine(coroutine);
            coroutine = null;
        }

        private IEnumerator DoLogging()
        {
            if (Vessel == null)
            {
                LogError("Cannot log a null vessel");
                yield break;
            }

            if (!FlightGUI.vesselFlightGUI.TryGetValue(Vessel, out FlightGUI gui))
            {
                LogError("Cannot log a vessel without FlightGUI");
                yield break;
            }

            if (gui == null)
            {
                LogError("Cannot log a vessel with null FlightGUI");
                yield break;
            }

            OpenFile();

            if (!headerWritten && !writer.Appending)
            {
                writer.WriteLine(headers);
                headerWritten = true;
            }

            double logTime = Planetarium.fetch.time;
            int flushCounter = 0;
            while (true)
            {
                Log(gui);
                logTime += Period * 0.001;

                while (Planetarium.fetch.time <= logTime)
                    yield return null;

                if (flushCounter >= FlushPeriod)
                {
                    writer.Flush();
                    flushCounter = 0;
                }
                else
                {
                    flushCounter++;
                }
            }
        }

        private void LogError(string msg)
        {
            FARLogger.Error(msg);
            StopLogging();
        }

        private void Log(FlightGUI gui)
        {
            writer.Write(Planetarium.fetch.time);
            writer.Write(Vessel.altitude);
            writer.Write(Vessel.srfSpeed);
            writer.Write(gui.InfoParameters.liftForce);
            writer.Write(gui.InfoParameters.dragForce);
            writer.Write(gui.InfoParameters.sideForce);
            writer.Write(gui.InfoParameters.dynPres);
            writer.Write(gui.InfoParameters.liftCoeff);
            writer.Write(gui.InfoParameters.dragCoeff);
            writer.Write(gui.InfoParameters.sideCoeff);
            // writer.Write(gui.InfoParameters.refArea);
            writer.Write(gui.InfoParameters.liftToDragRatio);
            writer.Write(gui.InfoParameters.velocityLiftToDragRatio);
            writer.Write(gui.InfoParameters.aoA);
            writer.Write(gui.InfoParameters.sideslipAngle);
            writer.Write(gui.InfoParameters.pitchAngle);
            writer.Write(gui.InfoParameters.headingAngle);
            writer.Write(gui.InfoParameters.rollAngle);
            // writer.Write(gui.InfoParameters.dryMass);
            // writer.Write(gui.InfoParameters.fullMass);
            // writer.Write(gui.InfoParameters.tSFC);
            // writer.Write(gui.InfoParameters.intakeAirFrac);
            // writer.Write(gui.InfoParameters.specExcessPower);
            // writer.Write(gui.InfoParameters.range);
            // writer.Write(gui.InfoParameters.endurance);
            // writer.Write(gui.InfoParameters.ballisticCoeff);
            // writer.Write(gui.InfoParameters.termVelEst);
            writer.WriteLine(gui.InfoParameters.stallFraction);
        }

        private string GetFilename()
        {
            if (Vessel != null)
                replacements[VesselNameKey] = Localizer.Format(Vessel.vesselName);
            replacements[DatetimeKey] = DateTime.Now.ToString(FARConfig.FlightLog.DatetimeFormat);
            string filename = Path.Combine(FARConfig.FlightLog.Directory,
                                           KSPUtil.SanitizeFilename(FARConfig.FlightLog.NameFormat.ToString(replacements)));

            return filename;
        }

        private void OpenFile()
        {
            if (writer.IsOpen)
            {
                FARLogger.Debug($"Continuing logging to {FileName}");
                return;
            }

            string directory = Path.GetDirectoryName(FileName);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
            {
                FARLogger.Debug($"Logs directory {directory} does not exist, creating");
                Directory.CreateDirectory(directory);
            }
            else if (File.Exists(FileName))
            {
                FARLogger.Info($"Appending logs to {FileName}");
            }
            else
            {
                FARLogger.Info($"Starting logging to {FileName}");
            }

            writer.Open();
        }

        private void OnDestroy()
        {
            StopLogging();
            Vessel = null;
        }
    }
}
