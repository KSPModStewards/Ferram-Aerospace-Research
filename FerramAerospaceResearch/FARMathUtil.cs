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
using System.Globalization;
using UnityEngine;

namespace FerramAerospaceResearch
{
    public static class FARMathUtil
    {
        public const double rad2deg = 180d / Math.PI;
        public const double deg2rad = Math.PI / 180d;

        // https://floating-point-gui.de/errors/comparison/

        public static bool NearlyEqual(this double a, double b, double epsilon = 1e-14)
        {
            // ReSharper disable CompareOfFloatsByEqualityOperator
            // shortcut, handles infinities
            if (a == b)
                return true;

            // a or b is zero or both are extremely close to it
            // relative error is less meaningful here
            double absA = Math.Abs(a);
            double absB = Math.Abs(b);
            double diff = Math.Abs(a - b);
            if (a == 0 || b == 0 || absA + absB < double.Epsilon)
                return diff < epsilon * double.Epsilon;

            // use relative error
            return diff / Math.Min(absA + absB, double.MaxValue) < epsilon;
            // ReSharper restore CompareOfFloatsByEqualityOperator
        }

        public static bool NearlyEqual(this Vector3 a, Vector3 b, float epsilon = 1e-6f)
        {
            return a.x.NearlyEqual(b.x, epsilon) && a.y.NearlyEqual(b.y, epsilon) && a.z.NearlyEqual(b.z, epsilon);
        }

        public static bool NearlyEqual(this Vector3d a, Vector3d b, double epsilon = 1e-14)
        {
            return a.x.NearlyEqual(b.x, epsilon) && a.y.NearlyEqual(b.y, epsilon) && a.z.NearlyEqual(b.z, epsilon);
        }

        public static bool NearlyEqual(this float a, float b, float epsilon = 1e-6f)
        {
            // ReSharper disable CompareOfFloatsByEqualityOperator
            // shortcut, handles infinities
            if (a == b)
                return true;

            // a or b is zero or both are extremely close to it
            // relative error is less meaningful here
            float absA = Mathf.Abs(a);
            float absB = Mathf.Abs(b);
            float diff = Mathf.Abs(a - b);
            if (a == 0 || b == 0 || absA + absB < float.Epsilon)
                return diff < epsilon * float.Epsilon;

            // use relative error
            return diff / Mathf.Min(absA + absB, float.MaxValue) < epsilon;
            // ReSharper restore CompareOfFloatsByEqualityOperator
        }

        // ReSharper disable once UnusedMember.Global
        public static double Lerp(double x1, double x2, double y1, double y2, double x)
        {
            double y = (y2 - y1) / (x2 - x1);
            y *= x - x1;
            y += y1;
            return y;
        }

        public static T Clamp<T>(this T val, T min, T max) where T : IComparable<T>
        {
            if (val.CompareTo(min) < 0)
                return min;
            return val.CompareTo(max) > 0 ? max : val;
        }

        public static void Swap<T>(ref T a, ref T b)
        {
            T tmp = a;
            a = b;
            b = tmp;
        }

        public static bool IsClose(double a, double b, double aTol = 1e-8, double rTol = 1e-5, bool equalNan = false)
        {
            if (IsFinite(a) && IsFinite(b))
                return Math.Abs(a - b) <= aTol + rTol * Math.Abs(b);

            if (equalNan && double.IsNaN(a) && double.IsNaN(b))
                return true;

            // ReSharper disable once CompareOfFloatsByEqualityOperator
            return a == b;
        }

        public static bool IsFinite(double a)
        {
            return !double.IsNaN(a) && !double.IsInfinity(a);
        }

        public static bool Approximately(double p, double q, double error = double.Epsilon)
        {
            return Math.Abs(p - q) < error;
        }

        public static double ArithmeticGeometricMean(double a, double b, double error)
        {
            while (!Approximately(a, b, error))
            {
                double tmpA = 0.5 * (a + b);
                b = Math.Sqrt(a * b);
                a = tmpA;
            }

            return (a + b) * 0.5;
        }

        public static double ModifiedArithmeticGeometricMean(double a, double b, double error)
        {
            double c = 0;
            while (!Approximately(a, b, error))
            {
                double tmpA = 0.5 * (a + b);
                double tmpSqrt = Math.Sqrt((a - c) * (b - c));
                b = c + tmpSqrt;
                c -= tmpSqrt;
                a = tmpA;
            }

            return (a + b) * 0.5;
        }

        // ReSharper disable once UnusedMember.Global
        public static double CompleteEllipticIntegralSecondKind(double k, double error)
        {
            double value = 2 * ArithmeticGeometricMean(1, k, error);
            value = Math.PI * ModifiedArithmeticGeometricMean(1, k * k, error) / value;

            return value;
        }

        //Approximation of Math.Pow(), as implemented here: http://martin.ankerl.com/2007/10/04/optimized-pow-approximation-for-java-and-c-c/
        public static double PowApprox(double a, double b)
        {
            int tmp = (int)(BitConverter.DoubleToInt64Bits(a) >> 32);
            int tmp2 = (int)(b * (tmp - 1072632447) + 1072632447);
            return BitConverter.Int64BitsToDouble((long)tmp2 << 32);
        }

        // ReSharper disable once UnusedMember.Global
        public static OptimizationResult BrentsMethod(
            Func<double, double> function,
            double a,
            double b,
            double epsilon = 0.001,
            int maxIter = int.MaxValue
        )
        {
            double delta = epsilon * 100;
            double fa = function(a);
            double fb = function(b);

            if (fa * fb >= 0)
            {
                FARLogger.Debug("Brent's method failed to converge in 2 calls due to invalid brackets");
                return new OptimizationResult(0, 2);
            }

            if (Math.Abs(fa) < Math.Abs(fb))
            {
                double tmp = fa;
                fa = fb;
                fb = tmp;

                tmp = a;
                a = b;
                b = tmp;
            }

            double c = a, d = a, fc = function(c);
            int funcCalls = 3;

            double s = b;

            bool flag = true;
            for (int iter = 0; iter < maxIter; iter++)
            {
                if (fa - fc > double.Epsilon && fb - fc > double.Epsilon) //inverse quadratic interpolation
                {
                    s = a * fc * fb / ((fa - fb) * (fa - fc));
                    s += b * fc * fa / ((fb - fa) * (fb - fc));
                    s += c * fc * fb / ((fc - fa) * (fc - fb));
                }
                else
                {
                    s = (b - a) / (fb - fa); //secant method
                    s *= fb;
                    s = b - s;
                }

                double b_s = Math.Abs(b - s), b_c = Math.Abs(b - c), c_d = Math.Abs(c - d);

                //Conditions for bisection method
                bool condition1;
                double a3pb_over4 = (3 * a + b) * 0.25;

                if (a3pb_over4 > b)
                    if (s < a3pb_over4 && s > b)
                        condition1 = false;
                    else
                        condition1 = true;
                else if (s > a3pb_over4 && s < b)
                    condition1 = false;
                else
                    condition1 = true;

                bool condition2;

                if (flag && b_s >= b_c * 0.5)
                    condition2 = true;
                else
                    condition2 = false;

                bool condition3;

                if (!flag && b_s >= c_d * 0.5)
                    condition3 = true;
                else
                    condition3 = false;

                bool condition4;

                if (flag && b_c <= delta)
                    condition4 = true;
                else
                    condition4 = false;

                bool condition5;

                if (!flag && c_d <= delta)
                    condition5 = true;
                else
                    condition5 = false;

                if (condition1 || condition2 || condition3 || condition4 || condition5)
                {
                    s = a + b;
                    s *= 0.5;
                    flag = true;
                }
                else
                {
                    flag = false;
                }

                double fs = function(s);
                funcCalls++;
                d = c;
                c = b;

                if (fa * fs < 0)
                {
                    b = s;
                    fb = fs;
                }
                else
                {
                    a = s;
                    fa = fs;
                }

                if (Math.Abs(fa) < Math.Abs(fb))
                {
                    double tmp = fa;
                    fa = fb;
                    fb = tmp;

                    tmp = a;
                    a = b;
                    b = tmp;
                }

                if (fs.NearlyEqual(0) || Math.Abs(a - b) <= epsilon)
                    return new OptimizationResult(s, funcCalls, true);
            }

            FARLogger.Debug($"Brent's method failed to converged in {funcCalls.ToString()} function calls");

            return new OptimizationResult(s, funcCalls);
        }

        /// <summary>
        ///     C# implementation of
        ///     https://github.com/scipy/scipy/blob/d5617d81064885ef2ec961492bc703f36bb36ee9/scipy/optimize/zeros.py#L95-L363
        ///     with optional <see cref="minLimit" /> and <see cref="maxLimit" /> constraints for physical problems. The solver
        ///     terminates when it either reaches the maximum number of iterations <see cref="maxIter" />, current and previous
        ///     <see cref="function" /> values are equal, current and previous solutions are close enough, are both NaN  or both
        ///     fall outside limits.
        /// </summary>
        /// <param name="function">Function to find root of</param>
        /// <param name="x0">Initial guess</param>
        /// <param name="x1">Optional second guess</param>
        /// <param name="tol">Absolute convergence tolerance</param>
        /// <param name="rTol">Relative convergence tolerance</param>
        /// <param name="maxIter">Maximum number of iterations</param>
        /// <param name="maxLimit">Maximum value of the solution</param>
        /// <param name="minLimit">Minimum value of the solution</param>
        /// <returns><see cref="OptimizationResult" /> solution</returns>
        /// <exception cref="ArgumentException">When initial and second guesses are equal</exception>
        public static OptimizationResult Secant(
            Func<double, double> function,
            double x0,
            double? x1 = null,
            double tol = 0.001,
            double rTol = 0.001,
            int maxIter = 50,
            double maxLimit = double.PositiveInfinity,
            double minLimit = double.NegativeInfinity
        )
        {
            // ReSharper disable CompareOfFloatsByEqualityOperator
            int funcCalls = 0;
            double p0 = x0;
            double p1;
            if (x1 is null)
            {
                const double eps = 1e-4;
                p1 = x0 * (1 + eps);
                p1 += p1 >= 0 ? eps : -eps;
            }
            else
            {
                if (x1 == x0)
                    throw new ArgumentException($"{nameof(x1)} and {nameof(x0)} must be different");
                p1 = (double)x1;
            }

            double q0 = function(p0);
            double q1 = function(p1);
            funcCalls += 2;
            double p = 0;

            if (Math.Abs(q1) < Math.Abs(q0))
            {
                Swap(ref p0, ref p1);
                Swap(ref q0, ref q1);
            }

            for (int itr = 0; itr < maxIter; itr++)
            {
                if (q1 == q0)
                {
                    if (p1 != p0)
                        FARLogger.Warning($"Tolerance of {(p1 - p0).ToString(CultureInfo.InvariantCulture)} reached");
                    FARLogger.Debug($"Secant method converged in {funcCalls.ToString()} function calls");
                    return new OptimizationResult((p1 + p0) / 2, funcCalls, true);
                }

                if (Math.Abs(q1) > Math.Abs(q0))
                    p = (-q0 / q1 * p1 + p0) / (1 - q0 / q1);
                else
                    p = (-q1 / q0 * p0 + p1) / (1 - q1 / q0);

                if (IsClose(p, p1, tol, rTol))
                {
                    FARLogger.Debug($"Secant method converged in {funcCalls.ToString()} function calls with tolerance of {(p1 - p).ToString(CultureInfo.InvariantCulture)}");
                    return new OptimizationResult(p, funcCalls, true);
                }

                p0 = p1;
                q0 = q1;
                p1 = p;

                if (double.IsNaN(p0) && double.IsNaN(p1))
                {
                    FARLogger.Warning($"Both {nameof(p0)} and {nameof(p1)} are NaN, used {funcCalls.ToString()} function calls");
                    return new OptimizationResult(p, funcCalls);
                }

                if (p1 < minLimit && p0 < minLimit || p1 > maxLimit && p0 > maxLimit)
                {
                    FARLogger.Warning($"{nameof(p1)} and {nameof(p0)} are outside the limits, used {funcCalls.ToString()} function calls");
                    return new OptimizationResult(p, funcCalls);
                }

                q1 = function(p1);
                funcCalls++;
            }

            FARLogger.Warning($"Secant method failed to converge in {funcCalls.ToString()} function calls");
            return new OptimizationResult(p, funcCalls);
            // ReSharper restore CompareOfFloatsByEqualityOperator
        }

        public struct OptimizationResult
        {
            public readonly double Result;
            public readonly bool Converged;
            public readonly int FunctionCalls;

            public OptimizationResult(double result, int functionCalls, bool converged = false)
            {
                Result = result;
                FunctionCalls = functionCalls;
                Converged = converged;
            }
        }
    }
}
