using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;

namespace PMKS.PositionSolving
{
    internal class SameSlideAcrossRPJointLinks : SameSlideAcrossJointAbstract
    {
        public SameSlideAcrossRPJointLinks(int varIndexBlock1, int jointIndexBlock1, double xBlock1, double yBlock1,
            int varIndexSlide1, int jointIndexSlide1, double xSlide1, double ySlide1,
            int varIndexSlide2, int jointIndexSlide2, double xSlide2, double ySlide2,
            double initSlideAngle, double distToSlide)
        {
            this.varIndex_Xb = 2 * varIndexBlock1;
            this.jointIndex_Block = jointIndexBlock1;
            this.xBlock = xBlock1;
            this.varIndex_Yb = 2 * varIndexBlock1 + 1;
            this.yBlock = yBlock1;
            this.varIndex_Xs = 2 * varIndexSlide1;
            this.jointIndex_Slide = jointIndexSlide1;
            this.xSlide = xSlide1;
            this.varIndex_Ys = 2 * varIndexSlide1 + 1;
            this.ySlide = ySlide1;
            this.varIndex_Xr = 2 * varIndexSlide2;
            this.jointIndex_SlopeRef = jointIndexSlide2;
            this.xSlopeRef = xSlide2;
            this.varIndex_Yr = 2 * varIndexSlide2 + 1;
            this.ySlopeRef = ySlide2;
            this.initSlideAngle = initSlideAngle;
            this.distToSlide = distToSlide;
        }

        public override double calculate(double[] x)
        {
            assignPositions(x);
            return innerFunction * innerFunction;
        }

        private void assignPositions(double[] x)
        {
            if (varIndex_Xb >= 0)
            {
                xBlock = x[2 * varIndex_Xb];
                yBlock = x[2 * varIndex_Xb + 1];
            }
            if (varIndex_Xs >= 0)
            {
                xSlide = x[2 * varIndex_Xs];
                ySlide = x[2 * varIndex_Xs + 1];
            }
            if (varIndex_Xr >= 0)
            {
                xSlopeRef = x[2 * varIndex_Xr];
                ySlopeRef = x[2 * varIndex_Xr + 1];
            }
            slideBaseAngle = Math.Atan2(ySlopeRef - ySlide, xSlopeRef - xSlide);
            sineAngle = Math.Sin(initSlideAngle + slideBaseAngle);
            cosineAngle = Math.Cos(initSlideAngle + slideBaseAngle);
            innerFunction = distToSlide + vectorX * sineAngle
                            + vectorY * cosineAngle;
            newPointReCalcDerivConstant = true;
        }

        internal override void SetInitialJointPosition(int index, double x, double y)
        {
            if (index == jointIndex_Block)
            {
                xBlock = x;
                yBlock = y;
            }
            else if (index == jointIndex_Slide)
            {
                xSlide = x;
                ySlide = y;
            }
            else if (index == jointIndex_SlopeRef)
            {
                xSlopeRef = x;
                ySlopeRef = y;
            }
        }

    }
}