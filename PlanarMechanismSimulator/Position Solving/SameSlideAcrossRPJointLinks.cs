using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;

namespace PMKS.PositionSolving
{
    internal class SameSlideAcrossRPJointLinks : NonDyadicObjFunctionTerm
    {
        private readonly int jointIndex_Block1;
        private readonly int jointIndex_Slide1;
        private readonly int jointIndex_Slide2;
        private readonly int varIndex_Xb1;
        private readonly int varIndex_Xs1;
        private readonly int varIndex_Xs2;
        private readonly int varIndex_Yb1;
        private readonly int varIndex_Ys1;
        private readonly int varIndex_Ys2;

        private readonly double initSlideAngle;
        private readonly double distToSlide;
        /* the following 8 variables are NOT readonly since these are part of what is being optimized */
        private double xBlock1, yBlock1;
        private double xSlide1, ySlide1, xSlide2, ySlide2;


        private Boolean newPointReCalcDerivConstant;
        private double slideBaseAngle;
        private double innerFunction;
        private double slopeDeltaX, slopeDeltaY;
        private double vectorX, vectorY;
        private double sumSquaredDeltas, squaredDeltaX, squaredDeltaY;
        private double denomSlopeX, denomSlopeY;
        private double deriv_innerFunction_wrt_xBlock1;
        private double deriv_innerFunction_wrt_yBlock1;
        private double deriv_innerFunction_wrt_xSlide1;
        private double deriv_innerFunction_wrt_xSlide2;
        private double deriv_innerFunction_wrt_ySlide1;
        private double deriv_innerFunction_wrt_ySlide2;
        private double sineAngle, cosineAngle;

        public SameSlideAcrossRPJointLinks(int varIndexBlock1, int jointIndexBlock1, double xBlock1, double yBlock1,
            int varIndexSlide1, int jointIndexSlide1, double xSlide1, double ySlide1,
            int varIndexSlide2, int jointIndexSlide2, double xSlide2, double ySlide2,
            double initSlideAngle, double distToSlide)
        {
            this.varIndex_Xb1 = 2 * varIndexBlock1;
            this.jointIndex_Block1 = jointIndexBlock1;
            this.xBlock1 = xBlock1;
            this.varIndex_Yb1 = 2 * varIndexBlock1 + 1;
            this.yBlock1 = yBlock1;
            this.varIndex_Xs1 = 2 * varIndexSlide1;
            this.jointIndex_Slide1 = jointIndexSlide1;
            this.xSlide1 = xSlide1;
            this.varIndex_Ys1 = 2 * varIndexSlide1 + 1;
            this.ySlide1 = ySlide1;
            this.varIndex_Xs2 = 2 * varIndexSlide2;
            this.jointIndex_Slide2 = jointIndexSlide2;
            this.xSlide2 = xSlide2;
            this.varIndex_Ys2 = 2 * varIndexSlide2 + 1;
            this.ySlide2 = ySlide2;
            this.initSlideAngle = initSlideAngle;
            this.distToSlide = distToSlide;
        }

        public override double calculate(double[] x)
        {
            assignPositions(x);
            return innerFunction * innerFunction;
        }

        public override double deriv_wrt_xi(double[] x, int i)
        {
            if (!(i == 2 * varIndex_Xb1 || i == 2 * varIndex_Xb1 + 1 ||
                i == 2 * varIndex_Xs1 || i == 2 * varIndex_Xs1 + 1 ||
                i == 2 * varIndex_Xs2 || i == 2 * varIndex_Xs2 + 1))
                return 0;

            if (newPointReCalcDerivConstant)
            {
                slopeDeltaX = xSlide2 - xSlide1;
                squaredDeltaX = slopeDeltaX * slopeDeltaX;
                slopeDeltaY = ySlide2 - ySlide1;
                squaredDeltaY = slopeDeltaY * slopeDeltaY;
                sumSquaredDeltas = squaredDeltaX + squaredDeltaY;

                vectorX = xSlide1 - xBlock1;
                vectorY = ySlide1 - yBlock1;
                denomSlopeY = 1 + (squaredDeltaY / squaredDeltaX);
                denomSlopeX = -1 * sumSquaredDeltas / slopeDeltaY;
                deriv_innerFunction_wrt_xBlock1 = sineAngle;
                deriv_innerFunction_wrt_yBlock1 = cosineAngle;
                deriv_innerFunction_wrt_ySlide2 = ((vectorX * cosineAngle) - (vectorY * sineAngle)) / denomSlopeY;
                deriv_innerFunction_wrt_xSlide2 = ((vectorX * cosineAngle) - (vectorY * sineAngle)) / denomSlopeX;
                deriv_innerFunction_wrt_ySlide1 = ((-(vectorX * cosineAngle) + (vectorY * sineAngle)) / denomSlopeY)
                    + cosineAngle;/* note that slide1 has an extra term because it is part of the vector, d. Slide 2 is not. */
                deriv_innerFunction_wrt_xSlide1 = ((-(vectorX * cosineAngle) + (vectorY * sineAngle)) / denomSlopeX)
                    + sineAngle;/* note that slide1 has an extra term because it is part of the vector, d. Slide 2 is not. */

                newPointReCalcDerivConstant = false;
            }
            var innerDeriv = 0.0;
            if (i == varIndex_Xb1) innerDeriv = deriv_innerFunction_wrt_xBlock1;
            if (i == varIndex_Yb1) innerDeriv = deriv_innerFunction_wrt_yBlock1;
            if (i == varIndex_Xs1) innerDeriv = deriv_innerFunction_wrt_xSlide1;
            if (i == varIndex_Ys1) innerDeriv = deriv_innerFunction_wrt_ySlide1;
            if (i == varIndex_Xs2) innerDeriv = deriv_innerFunction_wrt_xSlide2;
            if (i == varIndex_Ys2) innerDeriv = deriv_innerFunction_wrt_ySlide2;
            return 2 * innerFunction * innerDeriv;
        }

        public override double second_deriv_wrt_ij(double[] x, int i, int j)
        {
            if (!(i == varIndex_Xb1 || i == varIndex_Yb1 ||
                i == varIndex_Xs1 || i == varIndex_Ys1 ||
                i == varIndex_Xs2 || i == varIndex_Ys2))
                return 0;
            if (!(j == varIndex_Xb1 || j == varIndex_Yb1 ||
                j == varIndex_Xs1 || j == varIndex_Ys1 ||
                j == varIndex_Xs2 || j == varIndex_Ys2))
                return 0;
            #region derivaties with xBlock1 (6)
            if (j == varIndex_Xb1)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Xb1)
            {
                if (j == varIndex_Xb1) return 2 * sineAngle * deriv_innerFunction_wrt_xBlock1;
                if (j == varIndex_Yb1) return 2 * sineAngle * deriv_innerFunction_wrt_yBlock1;
                if (j == varIndex_Ys2)
                    return (2 * innerFunction * cosineAngle / denomSlopeY)
                           + 2 * deriv_innerFunction_wrt_ySlide2 * sineAngle;
                if (j == varIndex_Ys1)
                    return (-2 * innerFunction * cosineAngle / denomSlopeY)
                           + 2 * deriv_innerFunction_wrt_ySlide1 * sineAngle;
                if (j == varIndex_Xs2)
                    return (2 * innerFunction * cosineAngle / denomSlopeX)
                           + 2 * deriv_innerFunction_wrt_xSlide2 * sineAngle;
                if (j == varIndex_Xs1)
                    return (-2 * innerFunction * cosineAngle / denomSlopeX)
                           + 2 * deriv_innerFunction_wrt_xSlide1 * sineAngle;
            }
            #endregion
            #region derivaties with yBlock1 (5)
            if (j == varIndex_Yb1)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Yb1)
            {
                if (j == varIndex_Yb1) return 2 * cosineAngle * deriv_innerFunction_wrt_yBlock1;
                if (j == varIndex_Ys2)
                    return (-2 * innerFunction * sineAngle / denomSlopeY)
                           + 2 * deriv_innerFunction_wrt_ySlide2 * cosineAngle;
                if (j == varIndex_Ys1)
                    return (2 * innerFunction * sineAngle / denomSlopeY)
                           + 2 * deriv_innerFunction_wrt_ySlide1 * cosineAngle;
                if (j == varIndex_Xs2)
                    return (-2 * innerFunction * sineAngle / denomSlopeX)
                           + 2 * deriv_innerFunction_wrt_xSlide2 * cosineAngle;
                if (j == varIndex_Xs1)
                    return (2 * innerFunction * sineAngle / denomSlopeX)
                           + 2 * deriv_innerFunction_wrt_xSlide1 * cosineAngle;
            }
            #endregion
            #region derivatives with ySlide2   (4)
            if (j == varIndex_Ys2)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Ys2)
            {
                if (j == varIndex_Ys2)
                    return ((-vectorX * sineAngle - vectorY * cosineAngle) / denomSlopeY)
                        * (2 * innerFunction / denomSlopeY) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_ySlide2 * denomSlopeY
                            - 4 * innerFunction * (slopeDeltaY / slopeDeltaX)) / (denomSlopeY * denomSlopeY);
                if (j == varIndex_Ys1)
                    return (((-vectorX * sineAngle - vectorY * cosineAngle) / -denomSlopeY)- sineAngle) 
                          * (2 * innerFunction / denomSlopeY) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_ySlide1 * denomSlopeY
                            + 4 * innerFunction * (slopeDeltaY / slopeDeltaX)) / (denomSlopeY * denomSlopeY);
                if (j == varIndex_Xs2)
                    return ((-vectorX * sineAngle - vectorY * cosineAngle) / denomSlopeX)
                          * (2 * innerFunction / denomSlopeY) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_xSlide2 * denomSlopeY
                            + 4 * innerFunction * squaredDeltaY / Math.Pow(slopeDeltaX, 3)) / (denomSlopeY * denomSlopeY);
                if (j == varIndex_Xs1)
                    return (((-vectorX * sineAngle - vectorY * cosineAngle) / -denomSlopeX)- cosineAngle) 
                         * (2 * innerFunction / denomSlopeY) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_xSlide1 * denomSlopeY
                            - 4 * innerFunction * squaredDeltaY / Math.Pow(slopeDeltaX, 3)) / (denomSlopeY * denomSlopeY);
            }
            #endregion
            #region derivatives with xSlide2 (3)
            if (j == varIndex_Xs2)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Xs2)
            {
                if (j == varIndex_Ys1)
                    return (((-vectorX * sineAngle - vectorY * cosineAngle )/ -denomSlopeY)- sineAngle) 
                         * (2 * innerFunction / denomSlopeX) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           ((2 * innerFunction + 2 * slopeDeltaY * deriv_innerFunction_wrt_ySlide1) * sumSquaredDeltas - 4 * innerFunction * squaredDeltaY) /
                           (sumSquaredDeltas * sumSquaredDeltas);
                if (j == varIndex_Xs2)
                    return ((-vectorX * sineAngle - vectorY * cosineAngle) / denomSlopeX)
                      * (2 * innerFunction / denomSlopeX) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (-2 * slopeDeltaY * deriv_innerFunction_wrt_xSlide2 * sumSquaredDeltas + (4 * innerFunction * slopeDeltaY * slopeDeltaX)) /
                           (sumSquaredDeltas * sumSquaredDeltas);
                if (j == varIndex_Xs1)
                    return (((-vectorX * sineAngle - vectorY * cosineAngle)/ -denomSlopeX) - cosineAngle)
                        * (2 * innerFunction / denomSlopeX) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (-2 * slopeDeltaY * deriv_innerFunction_wrt_xSlide1 * sumSquaredDeltas - (4 * innerFunction * slopeDeltaY * slopeDeltaX)) /
                           (sumSquaredDeltas * sumSquaredDeltas);
            }
            #endregion
            #region derivatives with ySlide1 (2)
            if (j == varIndex_Ys1)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Ys1)
            {
                if (j == varIndex_Ys1)
                    return (((vectorX * sineAngle + vectorY * cosineAngle) / -denomSlopeY) + sineAngle)
                          * (2 * innerFunction /denomSlopeY) +
                           (-vectorX * cosineAngle + vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_ySlide1 * denomSlopeY
                            + 4 * innerFunction * (slopeDeltaY / slopeDeltaX)) / (denomSlopeY * denomSlopeY)
                            +2*deriv_innerFunction_wrt_ySlide1*cosineAngle+2*innerFunction*sineAngle/denomSlopeY;
                if (j == varIndex_Xs1)
                    return (((vectorX * sineAngle - vectorY * cosineAngle)/ -denomSlopeX) - cosineAngle) 
                         * (2 * innerFunction/denomSlopeY ) +
                           (-vectorX * cosineAngle + vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_xSlide1 * denomSlopeY
                            - 4 * innerFunction * squaredDeltaY / Math.Pow(slopeDeltaX, 3)) / (denomSlopeY * denomSlopeY)
                            + 2 * deriv_innerFunction_wrt_xSlide1 * cosineAngle - 2 * innerFunction * sineAngle / denomSlopeX;
            }
            #endregion
            #region derivatives with xSlide1 (just double-deriv of xs1)
            if (i == varIndex_Xs1 && j == varIndex_Xs1)
                return (((vectorX * sineAngle - vectorY * cosineAngle) / -denomSlopeX) - cosineAngle)
                     * (2 * innerFunction/denomSlopeX) +
                       (-vectorX * cosineAngle + vectorY * sineAngle) *
                       (2 * deriv_innerFunction_wrt_xSlide1 * denomSlopeY
                        - 4 * innerFunction * squaredDeltaY / Math.Pow(slopeDeltaX, 3)) / (denomSlopeY * denomSlopeY)
                        + 2 * deriv_innerFunction_wrt_xSlide1 * sineAngle + 2 * innerFunction * cosineAngle / denomSlopeX;
            #endregion
            throw new Exception("2nd Derivative in SameAngleAcrossPJoint: you shouldn't be seeing this! how did you get by the initial if-statement?");
        }

        private void assignPositions(double[] x)
        {
            if (varIndex_Xb1 >= 0)
            {
                xBlock1 = x[2 * varIndex_Xb1];
                yBlock1 = x[2 * varIndex_Xb1 + 1];
            }
            if (varIndex_Xs1 >= 0)
            {
                xSlide1 = x[2 * varIndex_Xs1];
                ySlide1 = x[2 * varIndex_Xs1 + 1];
            }
            if (varIndex_Xs2 >= 0)
            {
                xSlide2 = x[2 * varIndex_Xs2];
                ySlide2 = x[2 * varIndex_Xs2 + 1];
            }
            slideBaseAngle = Math.Atan2(ySlide2 - ySlide1, xSlide2 - xSlide1);
            sineAngle = Math.Sin(initSlideAngle + slideBaseAngle);
            cosineAngle = Math.Cos(initSlideAngle + slideBaseAngle);
            innerFunction = distToSlide + vectorX * sineAngle
                            + vectorY * cosineAngle;
            newPointReCalcDerivConstant = true;
        }

        internal override void SetInitialJointPosition(int index, double x, double y)
        {
            if (index == jointIndex_Block1)
            {
                xBlock1 = x;
                yBlock1 = y;
            }
            else if (index == jointIndex_Slide1)
            {
                xSlide1 = x;
                ySlide1 = y;
            }
            else if (index == jointIndex_Slide2)
            {
                xSlide2 = x;
                ySlide2 = y;
            }
        }

    }
}