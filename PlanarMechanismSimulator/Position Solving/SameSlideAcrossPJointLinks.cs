using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;

namespace PMKS.PositionSolving
{
    internal class SameSlideAcrossPJointLinks : NonDyadicObjFunctionTerm
    {
        private readonly int jointListIndexBlock1;
        private readonly int jointListIndexBlock2;
        private readonly int jointListIndexSlide1;
        private readonly int varListIndexBlock1;
        private readonly int varListIndexSlide1;
        private readonly int varListIndexBlock2;

        private readonly double distToSlide;
        private readonly double blockAngle;
        /* the following 8 variables are NOT readonly since these are part of what is being optimized */
        private double xBlock1, yBlock1, xBlock2, yBlock2;
        private double xSlide1, ySlide1;



        public SameSlideAcrossPJointLinks(int varListIndexBlock1, int jointListIndexBlock1, double xBlock1, double yBlock1,
            int varListIndexBlock2, int jointListIndexBlock2, double xBlock2, double yBlock2,
            int varListIndexSlide1, int jointListIndexSlide1, double xSlide1, double ySlide1,
            int varListIndexSlide2, int jointListIndexSlide2, double xSlide2, double ySlide2,
            double distToSlide, double blockAngle)
        {
            this.varListIndexBlock1 = varListIndexBlock1;
            this.jointListIndexBlock1 = jointListIndexBlock1;
            this.xBlock1 = xBlock1;
            this.yBlock1 = yBlock1;
            this.varListIndexBlock2 = varListIndexBlock2;
            this.jointListIndexBlock2 = jointListIndexBlock2;
            this.xBlock2 = xBlock2;
            this.yBlock2 = yBlock2;
            this.varListIndexSlide1 = varListIndexSlide1;
            this.jointListIndexSlide1 = jointListIndexSlide1;
            this.xSlide1 = xSlide1;
            this.ySlide1 = ySlide1;
            this.distToSlide = distToSlide;
            this.blockAngle = blockAngle;
        }


        private double blockBaseAngle
        {
            get { return Math.Atan2(yBlock2 - yBlock1, xBlock2 - xBlock1); }
        }

        public override double calculate(double[] x)
        {
            assignPositions(x);
            return innerFunction * innerFunction;
        }

        public override double deriv_wrt_xi(double[] x, int i)
        {
            if (!(i == 2 * varListIndexBlock1 || i == 2 * varListIndexBlock1 + 1 ||
                i == 2 * varListIndexBlock2 || i == 2 * varListIndexBlock2 + 1 ||
                i == 2 * varListIndexSlide1 || i == 2 * varListIndexSlide1 + 1))
                return 0;
            assignPositions(x);
            if (i == 2 * varListIndexBlock1)
                // w.r.t. xBlock1
                return (-2 * (yBlock2 - yBlock1) * innerFunction) / ((xBlock2 - xBlock1 + yBlock2 - yBlock1) * (xBlock2 - xBlock1));
            if (i == 2 * varListIndexBlock1 + 1)
                // w.r.t. yBlock1
                return -2 * innerFunction / (xBlock2 - xBlock1 + yBlock2 - yBlock1);
            if (i == 2 * varListIndexBlock2)
                // w.r.t. xBlock2
                return (2 * (yBlock2 - yBlock1) * innerFunction) / ((xBlock2 - xBlock1 + yBlock2 - yBlock1) * (xBlock2 - xBlock1));
            if (i == 2 * varListIndexBlock2 + 1)
                // w.r.t. yBlock2
                return 2 * innerFunction / (xBlock2 - xBlock1 + yBlock2 - yBlock1);

            if (i == 2 * varListIndexSlide1)
                // w.r.t. xSlide1
                return (2 * (ySlide2 - ySlide1) * innerFunction) / ((xSlide2 - xSlide1 + ySlide2 - ySlide1) * (xSlide2 - xSlide1));
            if (i == 2 * varListIndexSlide1 + 1)
                // w.r.t. ySlide1
                return 2 * innerFunction / (xSlide2 - xSlide1 + ySlide2 - ySlide1);

            throw new Exception("Gradient:you shouldn't be seeing this! how did you get by the initial if-statement?");

        }

        public override double second_deriv_wrt_ij(double[] x, int i, int j)
        {
            if (!(i == 2 * varListIndexBlock1 || i == 2 * varListIndexBlock1 + 1 ||
                i == 2 * varListIndexBlock2 || i == 2 * varListIndexBlock2 + 1 ||
                i == 2 * varListIndexSlide1 || i == 2 * varListIndexSlide1 + 1))
                return 0;
            if (!(j == 2 * varListIndexBlock1 || j == 2 * varListIndexBlock1 + 1 ||
                j == 2 * varListIndexBlock2 || j == 2 * varListIndexBlock2 + 1 ||
                j == 2 * varListIndexSlide1 || j == 2 * varListIndexSlide1 + 1))
                return 0;
            if (i == j)
            {
                if (i == 2 * varListIndexBlock1)
                    // w.r.t. xBlock1
                    return (-2 * (yBlock2 - yBlock1) * innerFunction) / ((xBlock2 - xBlock1 + yBlock2 - yBlock1) * (xBlock2 - xBlock1));
                if (i == 2 * varListIndexBlock1 + 1)
                    // w.r.t. yBlock1
                    return -2 * innerFunction / (xBlock2 - xBlock1 + yBlock2 - yBlock1);
                if (i == 2 * varListIndexBlock2)
                    // w.r.t. xBlock2
                    return (2 * (yBlock2 - yBlock1) * innerFunction) / ((xBlock2 - xBlock1 + yBlock2 - yBlock1) * (xBlock2 - xBlock1));
                if (i == 2 * varListIndexBlock2 + 1)
                    // w.r.t. yBlock2
                    return 2 * innerFunction / (xBlock2 - xBlock1 + yBlock2 - yBlock1);

                if (i == 2 * varListIndexSlide1)
                    // w.r.t. xSlide1
                    return (2 * (ySlide2 - ySlide1) * innerFunction) / ((xSlide2 - xSlide1 + ySlide2 - ySlide1) * (xSlide2 - xSlide1));
                if (i == 2 * varListIndexSlide1 + 1)
                    // w.r.t. ySlide1
                    return 2 * innerFunction / (xSlide2 - xSlide1 + ySlide2 - ySlide1);
            }
            if (i > j)
            {
                //switch so that i is always less than j. double derivative can be calculated either way (don't you remember that?!)
                var temp = i;
                i = j;
                j = temp;
            }
            return double.NaN;

        }

        private void assignPositions(double[] x)
        {
            if (varListIndexBlock1 >= 0)
            {
                xBlock1 = x[2 * varListIndexBlock1];
                yBlock1 = x[2 * varListIndexBlock1 + 1];
            }
            if (varListIndexBlock2 >= 0)
            {
                xBlock2 = x[2 * varListIndexBlock2];
                yBlock2 = x[2 * varListIndexBlock2 + 1];
            }
            if (varListIndexSlide1 >= 0)
            {
                xSlide1 = x[2 * varListIndexSlide1];
                ySlide1 = x[2 * varListIndexSlide1 + 1];
            }
        }

        internal override void SetInitialJointPosition(int index, double x, double y)
        {
            if (index == jointListIndexBlock1)
            {
                xBlock1 = x;
                yBlock1 = y;
            }
            else if (index == jointListIndexBlock2)
            {
                xBlock2 = x;
                yBlock2 = y;
            }
            else if (index == jointListIndexSlide1)
            {
                xSlide1 = x;
                ySlide1 = y;
            }
        }

    }
}