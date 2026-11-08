using System;

namespace FerramAerospaceResearch
{
    //recyclable float curve
    internal class FARFloatCurve
    {
        private readonly Vector3d[] controlPoints;
        private readonly CubicSection[] sections;
        private readonly int centerIndex;

        public FARFloatCurve(int numControlPoints)
        {
            controlPoints = new Vector3d[numControlPoints];

            sections = new CubicSection[numControlPoints - 1];

            centerIndex = (sections.Length - 1) / 2;

            SetNextPrevIndices(sections.Length - 1, 0, centerIndex);
        }

        private void SetNextPrevIndices(int upperIndex, int lowerIndex, int curIndex)
        {
            while (true)
            {
                if (upperIndex <= lowerIndex)
                    return;

                int nextIndex = (upperIndex + curIndex + 1) / 2;
                int prevIndex = (lowerIndex + curIndex - 1) / 2;

                sections[curIndex].nextIndex = nextIndex;
                sections[curIndex].prevIndex = prevIndex;

                SetNextPrevIndices(curIndex - 1, lowerIndex, prevIndex);
                lowerIndex = curIndex + 1;
                curIndex = nextIndex;
            }
        }

        //uses x for x point, y for y point and z for dy/dx
        public void SetPoint(int index, Vector3d controlPoint)
        {
            controlPoints[index] = controlPoint;
        }

        public void BakeCurve()
        {
            for (int i = 0; i < sections.Length; i++)
                sections[i].BuildSection(controlPoints[i], controlPoints[i + 1]);
        }

        public double Evaluate(double x)
        {
            int curIndex = centerIndex;
            int count = 0;
            while (true)
            {
                if (count > sections.Length)
                    throw new Exception();
                ++count;
                int check = sections[curIndex].CheckRange(x);

                //above of this cubic's range
                if (check > 0)
                    if (curIndex >= sections.Length - 1)
                        //at upper end of curve, just return max val of last cubic
                    {
                        return sections[curIndex].EvalUpperLim();
                    }
                    else
                    {
                        //otherwise, find next cubic to check and continue
                        curIndex = sections[curIndex].nextIndex;
                        continue;
                    }

                if (check >= 0)
                    //if we get here, we're in range and should evaluate this cubic
                    return sections[curIndex].Evaluate(x);
                if (curIndex <= 0)
                    //at lower end of curve, return min val of first cubic
                    return sections[curIndex].EvalLowerLim();
                //below this cubic's range
                //otherwise, find next cubic to check and continue
                curIndex = sections[curIndex].prevIndex;
            }
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // // ReSharper disable once UnusedMember.Global
        // public void Scale(double scalar)
        // {
        //     for (int i = 0; i < sections.Length; ++i)
        //     {
        //         CubicSection tmpSection = sections[i];
        //         tmpSection.a *= scalar;
        //         tmpSection.b *= scalar;
        //         tmpSection.c *= scalar;
        //         tmpSection.d *= scalar;

        //         sections[i] = tmpSection;
        //     }
        // }

        /// <summary>
        ///     If num curve sections are identical, adds the curve coefficients together, keeping the limits of this curve
        /// </summary>
        /// <param name="otherCurve"></param>
        public void AddCurve(FARFloatCurve otherCurve)
        {
            if (sections.Length != otherCurve.sections.Length)
                throw new ArgumentException("Section array lengths do not match");

            for (int i = 0; i < sections.Length; ++i)
            {
                CubicSection tmpSection = sections[i];
                CubicSection addSection = otherCurve.sections[i];

                tmpSection.a += addSection.a;
                tmpSection.b += addSection.b;
                tmpSection.c += addSection.c;
                tmpSection.d += addSection.d;

                sections[i] = tmpSection;
            }
        }

        private struct CubicSection
        {
            public double a, b, c, d;
            public double upperLim, lowerLim;
            public int nextIndex, prevIndex;

            public void BuildSection(Vector3d lowerInputs, Vector3d upperInputs)
            {
                //Creates cubic from x,y,dy/dx data

                double recipXDiff = 1 / (lowerInputs.x - upperInputs.x);
                double recipXDiffSq = recipXDiff * recipXDiff;

                a = 2 * (upperInputs.y - lowerInputs.y) * recipXDiff;
                a += upperInputs.z + lowerInputs.z;
                a *= recipXDiffSq;

                b = 3 * (upperInputs.x + lowerInputs.x) * (lowerInputs.y - upperInputs.y) * recipXDiff;
                b -= (lowerInputs.x + 2 * upperInputs.x) * lowerInputs.z;
                b -= (2 * lowerInputs.x + upperInputs.x) * upperInputs.z;
                b *= recipXDiffSq;

                c = 6 * upperInputs.x * lowerInputs.x * (upperInputs.y - lowerInputs.y) * recipXDiff;
                c += (2 * lowerInputs.x * upperInputs.x + upperInputs.x * upperInputs.x) * lowerInputs.z;
                c += (2 * lowerInputs.x * upperInputs.x + lowerInputs.x * lowerInputs.x) * upperInputs.z;
                c *= recipXDiffSq;

                d = (3 * lowerInputs.x - upperInputs.x) * upperInputs.x * upperInputs.x * lowerInputs.y;
                d += (lowerInputs.x - 3 * upperInputs.x) * lowerInputs.x * lowerInputs.x * upperInputs.y;
                d *= recipXDiff;

                d -= lowerInputs.x * upperInputs.x * upperInputs.x * lowerInputs.z;
                d -= lowerInputs.x * lowerInputs.x * upperInputs.x * upperInputs.z;
                d *= recipXDiffSq;

                upperLim = upperInputs.x;
                lowerLim = lowerInputs.x;
            }

            public double Evaluate(double x)
            {
                double y = a * x;
                y += b;
                y *= x;
                y += c;
                y *= x;
                y += d;

                return y;
            }

            public double EvalUpperLim()
            {
                return Evaluate(upperLim);
            }

            public double EvalLowerLim()
            {
                return Evaluate(lowerLim);
            }

            public int CheckRange(double x)
            {
                if (x > upperLim)
                    return 1;
                if (x < lowerLim)
                    return -1;

                return 0;
            }
        }
    }
}
