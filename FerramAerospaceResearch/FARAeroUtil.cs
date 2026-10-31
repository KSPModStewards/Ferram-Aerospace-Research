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
using System.IO;
using ferram4;
using FerramAerospaceResearch.Settings;
using KSPCommunityFixes;
using UnityEngine;

namespace FerramAerospaceResearch
{
    public static class FARAeroUtil
    {
        //Based on ratio of density of water to density of air at SL
        private const double UNDERWATER_DENSITY_FACTOR_MINUS_ONE = 814.51020408163265306122448979592;

        //Standard Reynolds number for transition from laminar to turbulent flow
        private const double TRANSITION_REYNOLDS_NUMBER = 5e5;

        //Multiplier to skin friction due to surface roughness; approximately an 8% increase in drag
        private const double ROUGHNESS_SKIN_FRICTION_MULTIPLIER = 1.08;
        private static FloatCurve prandtlMeyerMach;
        private static FloatCurve prandtlMeyerAngle;
        public static double maxPrandtlMeyerTurnAngle;

        public static double massPerWingAreaSupported;
        public static double massStressPower;
        public static bool AJELoaded;

        public static int prevBodyIndex = -1;
        public static BodySettings currentBodyData;
        private static CelestialBody currentBody;

        public static bool loaded;

        private static List<FARWingAerodynamicModel> curEditorWingCache;

        // Parts currently added to the vehicle in the editor
        private static List<Part> CurEditorPartsCache;

        // Parts currently added, plus the ghost part(s) about to be attached
        private static List<Part> AllEditorPartsCache;

        private static int RaycastMaskVal, RaycastMaskEdit;

        private static readonly string[] RaycastLayers =
        {
            "Default", "TransparentFX", "Local Scenery", "Disconnected Parts"
        };

        public static CelestialBody CurrentBody
        {
            get
            {
                if (!(currentBody is null))
                    return currentBody;
                if (FlightGlobals.Bodies[1] || !FlightGlobals.ActiveVessel)
                    currentBody = FlightGlobals.Bodies[1];
                else
                    currentBody = FlightGlobals.ActiveVessel.mainBody;

                return currentBody;
            }
        }

        public static double CurrentAdiabaticIndex
        {
            get
            {
                double ut = Planetarium.fetch.time;
                if (HighLogic.LoadedSceneIsEditor)
                {
                    // vessels don't exist in editor
                    return FARAtmosphere.GetAdiabaticIndex(CurrentBody, Vector3d.zero, ut);
                }

                Vessel vessel = FlightGlobals.ActiveVessel;
                return FARAtmosphere.GetAdiabaticIndex(CurrentBody,
                                                       new Vector3d(vessel.latitude, vessel.longitude, vessel.altitude),
                                                       ut);
            }
        }

        public static FloatCurve PrandtlMeyerMach
        {
            get
            {
                if (prandtlMeyerMach != null)
                    return prandtlMeyerMach;
                FARLogger.Info("Prandtl-Meyer Expansion Curves Initialized");
                prandtlMeyerMach = new FloatCurve();
                prandtlMeyerAngle = new FloatCurve();
                double M = 1;
                double gamma = CurrentAdiabaticIndex;

                double gamma_ = Math.Sqrt((gamma + 1) / (gamma - 1));

                while (M < 250)
                {
                    double mach = Math.Sqrt(M * M - 1);

                    double nu = Math.Atan(mach / gamma_);
                    nu *= gamma_;
                    nu -= Math.Atan(mach);
                    nu *= FARMathUtil.rad2deg;

                    double nu_mach = (gamma - 1) / 2;
                    nu_mach *= M * M;
                    nu_mach++;
                    nu_mach *= M;
                    nu_mach = mach / nu_mach;
                    nu_mach *= FARMathUtil.rad2deg;

                    prandtlMeyerMach.Add((float)M, (float)nu, (float)nu_mach, (float)nu_mach);

                    nu_mach = 1 / nu_mach;

                    prandtlMeyerAngle.Add((float)nu, (float)M, (float)nu_mach, (float)nu_mach);

                    M += M switch
                    {
                        < 3 => 0.1f,
                        < 10 => 0.5f,
                        < 25 => 2,
                        _ => 25
                    };
                }

                maxPrandtlMeyerTurnAngle = gamma_ - 1;
                maxPrandtlMeyerTurnAngle *= 90;
                return prandtlMeyerMach;
            }
        }

        public static FloatCurve PrandtlMeyerAngle
        {
            get
            {
                if (prandtlMeyerAngle != null)
                    return prandtlMeyerAngle;
                FARLogger.Info("Prandtl-Meyer Expansion Curves Initialized");
                prandtlMeyerMach = new FloatCurve();
                prandtlMeyerAngle = new FloatCurve();
                double M = 1;
                double gamma = CurrentAdiabaticIndex;
                double gamma_ = Math.Sqrt((gamma + 1) / (gamma - 1));

                while (M < 250)
                {
                    double mach = Math.Sqrt(M * M - 1);

                    double nu = Math.Atan(mach / gamma_);
                    nu *= gamma_;
                    nu -= Math.Atan(mach);
                    nu *= FARMathUtil.rad2deg;

                    double nu_mach = (gamma - 1) / 2;
                    nu_mach *= M * M;
                    nu_mach++;
                    nu_mach *= M;
                    nu_mach = mach / nu_mach;
                    nu_mach *= FARMathUtil.rad2deg;

                    prandtlMeyerMach.Add((float)M, (float)nu, (float)nu_mach, (float)nu_mach);

                    nu_mach = 1 / nu_mach;

                    prandtlMeyerAngle.Add((float)nu, (float)M, (float)nu_mach, (float)nu_mach);

                    M += M switch
                    {
                        < 3 => 0.1,
                        < 10 => 0.5,
                        < 25 => 2,
                        _ => 25
                    };
                }

                maxPrandtlMeyerTurnAngle = gamma_ - 1;
                maxPrandtlMeyerTurnAngle *= 90;
                return prandtlMeyerAngle;
            }
        }

        // ReSharper disable once UnusedMember.Global
        public static List<FARWingAerodynamicModel> CurEditorWings
        {
            get { return curEditorWingCache ??= ListEditorWings(); }
        }

        public static List<Part> CurEditorParts
        {
            get { return CurEditorPartsCache ??= ListEditorParts(false); }
        }

        public static List<Part> AllEditorParts
        {
            get { return AllEditorPartsCache ??= ListEditorParts(true); }
        }

        public static int RaycastMask
        {
            get
            {
                // Just to avoid the opaque integer constant; maybe it's enough to
                // document what layers come into it, but this is more explicit.
                if (RaycastMaskVal != 0)
                    return EditorAboutToAttach(true) ? RaycastMaskEdit : RaycastMaskVal;
                foreach (string name in RaycastLayers)
                    RaycastMaskVal |= 1 << LayerMask.NameToLayer(name);

                // When parts are being dragged in the editor, they are put into this
                // layer; however we have to raycast them, or the visible CoL will be
                // different from the one after the parts are attached.
                RaycastMaskEdit = RaycastMaskVal | 1 << LayerMask.NameToLayer("Ignore Raycast");

                FARLogger.Info("Raycast mask: " + RaycastMaskVal + " " + RaycastMaskEdit);

                return EditorAboutToAttach(true) ? RaycastMaskEdit : RaycastMaskVal;
            }
        }

        public static void LoadAeroDataFromConfig()
        {
            if (loaded)
                return;

            AJELoaded = ModUtils.IsAJEInstalled;
            SetDefaultValuesIfNoValuesLoaded();

            loaded = true;

            //Get Kerbin
            currentBodyData = FARAeroData.AtmosphericConfiguration[1];
        }

        private static void SetDefaultValuesIfNoValuesLoaded()
        {
            if (massPerWingAreaSupported.NearlyEqual(0))
                massPerWingAreaSupported = 0.05;
            if (massStressPower.NearlyEqual(0))
                massStressPower = 1.2;
        }

        // ReSharper disable once UnusedMember.Global
        public static double MaxPressureCoefficientCalc(double M)
        {
            double gamma = CurrentAdiabaticIndex;

            if (M <= 0)
                return 1;
            double value = RayleighPitotTubeStagPressure(M);
            //and now to convert to pressure coefficient
            value--;
            value *= 2 / (gamma * M * M);

            return value;
        }

        public static double StagnationPressureCalc(double M)
        {
            double gamma = CurrentAdiabaticIndex;

            double ratio = M * M;
            ratio *= gamma - 1;
            ratio *= 0.5;
            ratio++;

            ratio = Math.Pow(ratio, gamma / (gamma - 1));
            return ratio;
        }

        public static double RayleighPitotTubeStagPressure(double M)
        {
            if (M <= 1)
                return StagnationPressureCalc(M);

            double gamma = CurrentAdiabaticIndex;
            //Rayleigh Pitot Tube Formula; gives max stagnation pressure behind shock
            double value = (gamma + 1) * M;
            value *= value;
            value /= 4 * gamma * M * M - 2 * (gamma - 1);
            value = Math.Pow(value, gamma / (gamma - 1));

            value *= 1 - gamma + 2 * gamma * M * M;
            value /= gamma + 1;

            return value;
        }

        public static double PressureBehindShockCalc(double M)
        {
            double gamma = CurrentAdiabaticIndex;

            double ratio = M * M;
            ratio *= 2 * gamma;
            ratio -= gamma - 1;
            ratio /= gamma + 1;

            return ratio;
        }

        public static double MachBehindShockCalc(double M)
        {
            double gamma = CurrentAdiabaticIndex;

            double ratio = gamma - 1;
            ratio *= M * M;
            ratio += 2;
            ratio /= 2 * gamma * M * M - (gamma - 1);
            ratio = Math.Sqrt(ratio);

            return ratio;
        }

        public static bool IsNonphysical(Part p)
        {
            return p.physicalSignificance == Part.PhysicalSignificance.NONE ||
                   p.HasModuleImplementingFast<LaunchClamp>() ||
                   HighLogic.LoadedSceneIsEditor &&
                   p != EditorLogic.RootPart &&
                   p.PhysicsSignificance == (int)Part.PhysicalSignificance.NONE;
        }

        public static void ResetEditorParts()
        {
            AllEditorPartsCache = CurEditorPartsCache = null;
            curEditorWingCache = null;
        }

        // Checks if there are any ghost parts almost attached to the craft
        public static bool EditorAboutToAttach(bool move_too = false)
        {
            return HighLogic.LoadedSceneIsEditor &&
                   EditorLogic.SelectedPart != null &&
                   (EditorLogic.SelectedPart.potentialParent != null ||
                    move_too && EditorLogic.SelectedPart == EditorLogic.RootPart);
        }

        public static List<Part> ListEditorParts(bool include_selected)
        {
            var list = new List<Part>();

            if (EditorLogic.RootPart)
                RecursePartList(list, EditorLogic.RootPart);

            if (!include_selected || !EditorAboutToAttach())
                return list;
            RecursePartList(list, EditorLogic.SelectedPart);

            foreach (Part sym in EditorLogic.SelectedPart.symmetryCounterparts)
                RecursePartList(list, sym);

            return list;
        }

        public static List<FARWingAerodynamicModel> ListEditorWings()
        {
            List<Part> list = CurEditorParts;

            var wings = new List<FARWingAerodynamicModel>();
            foreach (Part p in list)
            {
                FARWingAerodynamicModel wing = p.GetComponent<FARWingAerodynamicModel>();
                if (!(wing is null))
                    wings.Add(wing);
            }

            return wings;
        }

        private static void RecursePartList(List<Part> list, Part part)
        {
            list.Add(part);
            foreach (Part p in part.children)
                RecursePartList(list, p);
        }

        //This approximates e^x; it's slightly inaccurate, but good enough.  It's much faster than an actual exponential function
        //It runs on the assumption e^x ~= (1 + x/256)^256
        public static double ExponentialApproximation(double x)
        {
            double exp = 1d + x * 0.00390625;
            exp *= exp;
            exp *= exp;
            exp *= exp;
            exp *= exp;
            exp *= exp;
            exp *= exp;
            exp *= exp;
            exp *= exp;

            return exp;
        }

        // ReSharper disable once UnusedMember.Global
        public static double GetFailureForceScaling(CelestialBody body, double altitude)
        {
            if (!body.ocean || altitude > 0)
                return 1;

            double densityMultFactor = Math.Max(-altitude, 1);
            densityMultFactor *= UNDERWATER_DENSITY_FACTOR_MINUS_ONE * 0.05; //base it on the density factor

            return densityMultFactor;
        }

        public static double GetFailureForceScaling(Vessel vessel)
        {
            if (!vessel.mainBody.ocean || vessel.altitude > 0)
                return 1;

            double densityMultFactor = Math.Max(-vessel.altitude, 1);
            densityMultFactor *= UNDERWATER_DENSITY_FACTOR_MINUS_ONE * 0.05; //base it on the density factor

            return densityMultFactor;
        }

        public static double GetCurrentDensity(Vessel v)
        {
            double density = 0;
            double counter = 0;
            foreach (Part p in v.parts)
            {
                if (p.physicalSignificance == Part.PhysicalSignificance.NONE)
                    continue;

                density += p.dynamicPressurekPa * (1.0 - p.submergedPortion);
                density += p.submergedDynamicPressurekPa * p.submergedPortion;
                counter++;
            }

            if (counter > 0)
                density /= counter;
            density *= 2000; //need answers in Pa, not kPa
            density /= v.srfSpeed * v.srfSpeed;

            return density;
        }

        public static double CalculateCurrentViscosity(double tempInK)
        {
            double visc = currentBodyData.ReferenceViscosity; //get viscosity

            double tempRat = tempInK / currentBodyData.ReferenceTemperature;
            tempRat *= tempRat * tempRat;
            tempRat = Math.Sqrt(tempRat);

            visc *= currentBodyData.ReferenceTemperature + 110;
            visc /= tempInK + 110;
            visc *= tempRat;

            return visc;
        }


        public static double ReferenceTemperatureRatio(double machNumber, double recoveryFactor, double gamma)
        {
            double tempRatio = machNumber * machNumber;
            tempRatio *= gamma - 1;
            tempRatio *= 0.5; //account for stagnation temp

            tempRatio *= recoveryFactor; //this accounts for adiabatic wall temp ratio

            tempRatio *= 0.58;
            tempRatio += 0.032 * machNumber * machNumber;

            tempRatio++;
            return tempRatio;
        }

        public static double CalculateReynoldsNumber(
            double density,
            double lengthScale,
            double vel,
            double machNumber,
            double externalTemp,
            double gamma
        )
        {
            if (lengthScale.NearlyEqual(0))
                return 0;

            double refTemp = externalTemp * ReferenceTemperatureRatio(machNumber, 0.843, gamma);
            double visc = CalculateCurrentViscosity(refTemp);
            double Re = lengthScale * density * vel / visc;
            return Re;
        }

        public static double SkinFrictionDrag(
            double density,
            double lengthScale,
            double vel,
            double machNumber,
            double externalTemp,
            double gamma
        )
        {
            if (lengthScale.NearlyEqual(0))
                return 0;

            double Re = CalculateReynoldsNumber(density, lengthScale, vel, machNumber, externalTemp, gamma);

            return SkinFrictionDrag(Re, machNumber);
        }

        public static double SkinFrictionDrag(double reynoldsNumber, double machNumber)
        {
            if (reynoldsNumber < TRANSITION_REYNOLDS_NUMBER)
            {
                double invSqrtRe = 1 / Math.Sqrt(reynoldsNumber);
                double lamCf = 1.328 * invSqrtRe;

                double rarefiedGasVal = machNumber / reynoldsNumber;
                if (rarefiedGasVal > 0.01)
                    return (lamCf + (0.075 - lamCf) * (rarefiedGasVal - 0.01) / (0.99 + rarefiedGasVal)) *
                           ROUGHNESS_SKIN_FRICTION_MULTIPLIER;
                return lamCf * ROUGHNESS_SKIN_FRICTION_MULTIPLIER;
            }

            double transitionFraction = TRANSITION_REYNOLDS_NUMBER / reynoldsNumber;

            double laminarCf = 1.328 / Math.Sqrt(TRANSITION_REYNOLDS_NUMBER);
            double turbulentCfInLaminar = 0.074 / Math.Pow(TRANSITION_REYNOLDS_NUMBER, 0.2);
            double turbulentCf = 0.074 / Math.Pow(reynoldsNumber, 0.2);

            return (turbulentCf - transitionFraction * (turbulentCfInLaminar - laminarCf)) *
                   ROUGHNESS_SKIN_FRICTION_MULTIPLIER;
        }

        public static void UpdateCurrentActiveBody(CelestialBody body)
        {
            if (!(body is null) && body.flightGlobalsIndex != prevBodyIndex)
                UpdateCurrentActiveBody(body.flightGlobalsIndex, body);
        }

        public static void UpdateCurrentActiveBody(int index, CelestialBody body)
        {
            if (index == prevBodyIndex)
                return;
            prevBodyIndex = index;
            currentBodyData = FARAeroData.AtmosphericConfiguration[prevBodyIndex];
            currentBody = body;

            prandtlMeyerMach = null;
            prandtlMeyerAngle = null;
        }

        //Based on NASA Contractor Report 187173, Exact and Approximate Oblique Shock Equations for Real-Time Applications
        public static double CalculateSinWeakObliqueShockAngle(double MachNumber, double gamma, double deflectionAngle)
        {
            double M2 = MachNumber * MachNumber;
            double recipM2 = 1 / M2;
            double sin2def = Math.Sin(deflectionAngle);
            sin2def *= sin2def;

            double b = M2 + 2;
            b *= recipM2;
            b += gamma * sin2def;
            b = -b;

            double c = gamma + 1;
            c *= c * 0.25f;
            c += (gamma - 1) * recipM2;
            c *= sin2def;
            c += (2 * M2 + 1) * recipM2 * recipM2;

            double d = sin2def - 1;
            d *= recipM2 * recipM2;

            double Q = c * 0.33333333 - b * b * 0.111111111;
            double R = 0.16666667 * b * c - 0.5f * d - 0.037037037 * b * b * b;
            double D = Q * Q * Q + R * R;

            if (D > 0.001)
                return double.NaN;

            double phi = Math.Atan(Math.Sqrt((-D).Clamp(0, double.PositiveInfinity)) / R);
            if (R < 0)
                phi += Math.PI;
            phi *= 0.33333333;

            double chiW = -0.33333333 * b -
                          Math.Sqrt((-Q).Clamp(0, double.PositiveInfinity)) *
                          (Math.Cos(phi) - 1.7320508f * Math.Sin(phi));

            double betaW = Math.Sqrt(chiW.Clamp(0, double.PositiveInfinity));

            return betaW;
        }

        public static double CalculateSinMaxShockAngle(double MachNumber, double gamma)
        {
            double M2 = MachNumber * MachNumber;
            double gamP1_2_M2 = (gamma + 1) * 0.5 * M2;

            double b = gamP1_2_M2;
            b = 2 - b;
            b *= M2;

            double a = gamma * M2 * M2;

            double c = gamP1_2_M2 + 1;
            c = -c;

            double tmp = b * b - 4 * a * c;

            double sin2def = -b + Math.Sqrt(tmp.Clamp(0, double.PositiveInfinity));
            sin2def /= 2 * a;

            return Math.Sqrt(sin2def);
        }

        // ReSharper disable once UnusedMember.Global
        public static double MaxShockAngleCheck(double MachNumber, double gamma, out bool attachedShock)
        {
            double M2 = MachNumber * MachNumber;
            double gamP1_2_M2 = (gamma + 1) * 0.5 * M2;

            double b = gamP1_2_M2;
            b = 2 - b;
            b *= M2;

            double a = gamma * M2 * M2;

            double c = gamP1_2_M2 + 1;
            c = -c;

            double tmp = b * b - 4 * a * c;

            attachedShock = tmp > 0;

            return tmp;
        }

        //Calculates Oswald's Efficiency e using Shevell's Method
        // ReSharper disable once UnusedMember.Global
        public static double CalculateOswaldsEfficiency(double AR, double CosSweepAngle, double Cd0)
        {
            double e = 1 - 0.02 * FARMathUtil.PowApprox(AR, 0.7) * FARMathUtil.PowApprox(Math.Acos(CosSweepAngle), 2.2);
            double tmp = AR * Cd0 * Mathf.PI + 1;
            e /= tmp;

            return e;
        }

        //More modern, accurate Oswald's Efficiency
        //http://www.fzt.haw-hamburg.de/pers/Scholz/OPerA/OPerA_PUB_DLRK_12-09-10.pdf
        public static double CalculateOswaldsEfficiencyNitaScholz(
            double AR,
            double CosSweepAngle,
            double Cd0,
            double taperRatio
        )
        {
            //model coupling between taper and sweep
            double deltaTaper = Math.Acos(CosSweepAngle) * FARMathUtil.rad2deg;
            deltaTaper = ExponentialApproximation(-0.0375 * deltaTaper);
            deltaTaper *= 0.45;
            deltaTaper -= 0.357;

            taperRatio -= deltaTaper;

            //theoretic efficiency assuming an unswept wing with no sweep
            double straightWingE = 0.0524 * taperRatio;
            straightWingE += -0.15;
            straightWingE *= taperRatio;
            straightWingE += 0.1659;
            straightWingE *= taperRatio;
            straightWingE += -0.0706;
            straightWingE *= taperRatio;
            straightWingE += 0.0119;

            //Efficiency assuming only sweep and taper contributions; still need viscous contributions
            double theoreticE = straightWingE * AR + 1;
            theoreticE = 1 / theoreticE;

            // 1 - 2 * (fuse dia / span)^2, using avg val for ratio (0.114) because it isn't easy to get here
            //this results in this being a simple constant
            double eWingInterference = 0.974008 * theoreticE;

            double e = 0.38 * Cd0 * AR * Math.PI; //accounts for changes due to Mach number and compressibility
            e *= eWingInterference;
            e += 1;
            e = eWingInterference / e;

            return e;
        }
    }
}
