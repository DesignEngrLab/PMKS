using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;

namespace PMKS.PositionSolving
{
    internal class SameSlideAcrossRPJointLinks : NonDyadicObjFunctionTerm
    {
        private readonly int jointListIndexBlock1;
        private readonly int jointListIndexSlide1;
        private readonly int jointListIndexSlide2;
        private readonly int varListIndexBlock1;
        private readonly int varListIndexSlide1;
        private readonly int varListIndexSlide2;

        private readonly double initSlideAngle;
        private readonly double distToSlide;
        /* the following 8 variables are NOT readonly since these are part of what is being optimized */
        private double xBlock1, yBlock1;
        private double xSlide1, ySlide1, xSlide2, ySlide2;



        public SameSlideAcrossRPJointLinks(int varListIndexBlock1, int jointListIndexBlock1, double xBlock1, double yBlock1,
            int varListIndexSlide1, int jointListIndexSlide1, double xSlide1, double ySlide1,
            int varListIndexSlide2, int jointListIndexSlide2, double xSlide2, double ySlide2,
            double initSlideAngle, double distToSlide)
        {
            this.varListIndexBlock1 = varListIndexBlock1;
            this.jointListIndexBlock1 = jointListIndexBlock1;
            this.xBlock1 = xBlock1;
            this.yBlock1 = yBlock1;
            this.varListIndexSlide1 = varListIndexSlide1;
            this.jointListIndexSlide1 = jointListIndexSlide1;
            this.xSlide1 = xSlide1;
            this.ySlide1 = ySlide1;
            this.varListIndexSlide2 = varListIndexSlide2;
            this.jointListIndexSlide2 = jointListIndexSlide2;
            this.xSlide2 = xSlide2;
            this.ySlide2 = ySlide2;
            this.initSlideAngle = initSlideAngle;
            this.distToSlide = distToSlide;
        }

        private double slideBaseAngle
        {
            get { return Math.Atan2(ySlide2 - ySlide1, xSlide2 - xSlide1); }
        }


        public override double calculate(double[] x)
        {
            assignPositions(x);
            return innerFunction * innerFunction;
        }

        public override double deriv_wrt_xi(double[] x, int i)
        {
            if (!(i == 2 * varListIndexBlock1 || i == 2 * varListIndexBlock1 + 1 ||
                i == 2 * varListIndexSlide1 || i == 2 * varListIndexSlide1 + 1 ||
                i == 2 * varListIndexSlide2 || i == 2 * varListIndexSlide2 + 1))
                return 0;
            assignPositions(x);
            if (i == 2 * varListIndexBlock1)
                // w.r.t. xBlock1
                return (-2 * (yBlock2 - yBlock1) * innerFunction) / ((xBlock2 - xBlock1 + yBlock2 - yBlock1) * (xBlock2 - xBlock1));
            if (i == 2 * varListIndexBlock1 + 1)
                // w.r.t. yBlock1
                return -2 * innerFunction / (xBlock2 - xBlock1 + yBlock2 - yBlock1);

            if (i == 2 * varListIndexSlide1)
                // w.r.t. xSlide1
                return (2 * (ySlide2 - ySlide1) * innerFunction) / ((xSlide2 - xSlide1 + ySlide2 - ySlide1) * (xSlide2 - xSlide1));
            if (i == 2 * varListIndexSlide1 + 1)
                // w.r.t. ySlide1
                return 2 * innerFunction / (xSlide2 - xSlide1 + ySlide2 - ySlide1);
            if (i == 2 * varListIndexSlide2)
                // w.r.t. xSlide2
                return (-2 * (ySlide2 - ySlide1) * innerFunction) / ((xSlide2 - xSlide1 + ySlide2 - ySlide1) * (xSlide2 - xSlide1));
            if (i == 2 * varListIndexSlide2 + 1)
                // w.r.t. ySlide2
                return -2 * innerFunction / (xSlide2 - xSlide1 + ySlide2 - ySlide1);

            throw new Exception("Gradient:you shouldn't be seeing this! how did you get by the initial if-statement?");

        }

        public override double second_deriv_wrt_ij(double[] x, int i, int j)
        {
            if (!(i == 2 * varListIndexBlock1 || i == 2 * varListIndexBlock1 + 1 ||
                i == 2 * varListIndexSlide1 || i == 2 * varListIndexSlide1 + 1 ||
                i == 2 * varListIndexSlide2 || i == 2 * varListIndexSlide2 + 1))
                return 0;
            if (!(j == 2 * varListIndexBlock1 || j == 2 * varListIndexBlock1 + 1 ||
                j == 2 * varListIndexSlide1 || j == 2 * varListIndexSlide1 + 1 ||
                j == 2 * varListIndexSlide2 || j == 2 * varListIndexSlide2 + 1))
                return 0;
            if (i == j)
            {
                if (i == 2 * varListIndexBlock1)
                    // w.r.t. xBlock1
                    return (-2 * (yBlock2 - yBlock1) * innerFunction) / ((xBlock2 - xBlock1 + yBlock2 - yBlock1) * (xBlock2 - xBlock1));
                if (i == 2 * varListIndexBlock1 + 1)
                    // w.r.t. yBlock1
                    return -2 * innerFunction / (xBlock2 - xBlock1 + yBlock2 - yBlock1);

                if (i == 2 * varListIndexSlide1)
                    // w.r.t. xSlide1
                    return (2 * (ySlide2 - ySlide1) * innerFunction) / ((xSlide2 - xSlide1 + ySlide2 - ySlide1) * (xSlide2 - xSlide1));
                if (i == 2 * varListIndexSlide1 + 1)
                    // w.r.t. ySlide1
                    return 2 * innerFunction / (xSlide2 - xSlide1 + ySlide2 - ySlide1);
                if (i == 2 * varListIndexSlide2)
                    // w.r.t. xSlide2
                    return (-2 * (ySlide2 - ySlide1) * innerFunction) / ((xSlide2 - xSlide1 + ySlide2 - ySlide1) * (xSlide2 - xSlide1));
                if (i == 2 * varListIndexSlide2 + 1)
                    // w.r.t. ySlide2
                    return -2 * innerFunction / (xSlide2 - xSlide1 + ySlide2 - ySlide1);
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
            if (varListIndexSlide1 >= 0)
            {
                xSlide1 = x[2 * varListIndexSlide1];
                ySlide1 = x[2 * varListIndexSlide1 + 1];
            }
            if (varListIndexSlide2 >= 0)
            {
                xSlide2 = x[2 * varListIndexSlide2];
                ySlide2 = x[2 * varListIndexSlide2 + 1];
            }
        }

        internal override void SetInitialJointPosition(int index, double x, double y)
        {
            if (index == jointListIndexBlock1)
            {
                xBlock1 = x;
                yBlock1 = y;
            }
            else if (index == jointListIndexSlide1)
            {
                xSlide1 = x;
                ySlide1 = y;
            }
            else if (index == jointListIndexSlide2)
            {
                xSlide2 = x;
                ySlide2 = y;
            }
        }

    }
}