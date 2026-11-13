/*
Ferram Aerospace Research v0.17.0.0 "Maxwell"
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
using System.Linq;

namespace FerramAerospaceResearch
{
    /// <summary>
    ///     TweakScale's rescale of a part, as FAR's part modules take it: the part event OnPartScaleChanged, which
    ///     TweakScale (Rescaled, and Lisias' line) sends on every rescale with the part's linear scale as the float
    ///     "factorAbsolute". FAR handles the event rather than implementing TweakScale's IRescalable, so it does not
    ///     reference Scale_Redist: the burst compiler resolves a reference by its file name, and TweakScale installs the
    ///     redist as 999_Scale_Redist.dll.
    /// </summary>
    public static class TweakScaleEvent
    {
        public const string FactorKey = "factorAbsolute";

        /// <summary>
        ///     Calls rescale with the event's scale. A scale missing or not a positive finite number is logged and
        ///     ignored. What rescale throws is kept from the sender, as Run.
        /// </summary>
        public static void Handle(PartModule module, BaseEventDetails data, Action<float> rescale)
        {
            float factor = Factor(data);
            if (!(factor > 0) || float.IsInfinity(factor))
            {
                FARLogger.Warning($"{Describe(module)}: TweakScale's rescale event has no usable {FactorKey} " +
                                  $"({Value(data)}), ignored");
                return;
            }

            Run(module, $"rescale to {factor}", () => rescale(factor));
        }

        /// <summary>
        ///     Runs a module's handling of the event, whatever it throws logged and kept from the sender:
        ///     part.SendEvent has no try/catch of its own, so it would end TweakScale's rescale of the part (its other
        ///     handlers, mass and cost) and every other module's handling of the event.
        /// </summary>
        public static void Run(PartModule module, string what, Action action)
        {
            try
            {
                action();
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, $"{Describe(module)}: {what} failed");
            }
        }

        // the event's scale, NaN when it has none; a double taken too (the event's values are untyped)
        public static float Factor(BaseEventDetails data)
        {
            switch (Value(data))
            {
                case float f:
                    return f;
                case double d:
                    return (float)d;
                default:
                    return float.NaN;
            }
        }

        // Get of a missing key is not null: it goes through Unity first, so the keys are looked at instead
        private static object Value(BaseEventDetails data) =>
            data?.Keys?.Cast<object>().Contains(FactorKey) == true ? data.Get(FactorKey) : null;

        private static string Describe(PartModule module) =>
            module == null ? "(no module)" : $"{module.GetType().Name} on {module.part?.partInfo?.name ?? "a part"}";
    }
}
