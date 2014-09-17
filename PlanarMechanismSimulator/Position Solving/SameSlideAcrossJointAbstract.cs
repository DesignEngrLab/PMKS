using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;

namespace PMKS.PositionSolving
{
    internal abstract class SameSlideAcrossJointAbstract : NonDyadicObjFunctionTerm
    {

        protected int jointIndex_Block;
        protected int jointIndex_Slide;
        protected int jointIndex_SlopeRef;
        protected int varIndex_Xb;    //w  varIndex_Xb
        protected int varIndex_Xs;       //x  varIndex_Xs
        protected int varIndex_Xr;          //u  varIndex_Xr
        protected int varIndex_Yb;             //c  varIndex_Yb
        protected int varIndex_Ys;                //z    varIndex_Ys
        protected int varIndex_Yr;                   //v  varIndex_Yr

        // check on Wolfram Alpha: d/dx (l+(c-b)*cos(h+atan((y-z)/(b-x)))+(z-w)*sin(h+atan((y-z)/(b-x))))^2

        protected double initSlideAngle;
        protected double distToSlide;
        /* the following 8 variables are NOT readonly since these are part of what is being optimized */
        protected double xBlock, yBlock;
        protected double xSlide, ySlide, xSlopeRef, ySlopeRef;


        protected Boolean newPointReCalcDerivConstant;
        protected double slideBaseAngle;
        protected double innerFunction;
        protected double slopeDeltaX, slopeDeltaY;
        protected double vectorX, vectorY;
        protected double sumSquaredDeltas, squaredDeltaX, squaredDeltaY;
        protected double denomSlopeX, denomSlopeY;
        protected double deriv_innerFunction_wrt_xBlock;
        protected double deriv_innerFunction_wrt_yBlock;
        protected double deriv_innerFunction_wrt_xSlide;
        protected double deriv_innerFunction_wrt_xSlopeRef;
        protected double deriv_innerFunction_wrt_ySlide;
        protected double deriv_innerFunction_wrt_ySlopeRef;
        protected double sineAngle, cosineAngle;

        // check on Wolfram Alpha: d/dx (l+(c-b)*cos(h+atan((y-z)/(b-x)))+(z-w)*sin(h+atan((y-z)/(b-x))))^2
        public override double deriv_wrt_xi(double[] x, int i)
        {
            if (!(i == 2 * varIndex_Xb || i == 2 * varIndex_Xb + 1 ||
                i == 2 * varIndex_Xs || i == 2 * varIndex_Xs + 1 ||
                i == 2 * varIndex_Xr || i == 2 * varIndex_Xr + 1))
                return 0;

            if (newPointReCalcDerivConstant)
            {
                slopeDeltaX = xSlopeRef - xSlide;
                squaredDeltaX = slopeDeltaX * slopeDeltaX;
                slopeDeltaY = ySlopeRef - ySlide;
                squaredDeltaY = slopeDeltaY * slopeDeltaY;
                sumSquaredDeltas = squaredDeltaX + squaredDeltaY;

                vectorX = xSlide - xBlock;
                vectorY = ySlide - yBlock;
                denomSlopeY = 1 + (squaredDeltaY / squaredDeltaX);
                denomSlopeX = -1 * sumSquaredDeltas / slopeDeltaY;
                deriv_innerFunction_wrt_xBlock = sineAngle;
                deriv_innerFunction_wrt_yBlock = cosineAngle;
                deriv_innerFunction_wrt_ySlopeRef = ((vectorX * cosineAngle) - (vectorY * sineAngle)) / denomSlopeY;
                deriv_innerFunction_wrt_xSlopeRef = ((vectorX * cosineAngle) - (vectorY * sineAngle)) / denomSlopeX;
                deriv_innerFunction_wrt_ySlide = ((-(vectorX * cosineAngle) + (vectorY * sineAngle)) / denomSlopeY)
                    + cosineAngle;/* note that slide1 has an extra term because it is part of the vector, d. Slide 2 is not. */
                deriv_innerFunction_wrt_xSlide = ((-(vectorX * cosineAngle) + (vectorY * sineAngle)) / denomSlopeX)
                    + sineAngle;/* note that slide1 has an extra term because it is part of the vector, d. Slide 2 is not. */

                newPointReCalcDerivConstant = false;
            }
            var innerDeriv = 0.0;
            if (i == varIndex_Xb) innerDeriv = deriv_innerFunction_wrt_xBlock;
            if (i == varIndex_Yb) innerDeriv = deriv_innerFunction_wrt_yBlock;
            if (i == varIndex_Xs) innerDeriv = deriv_innerFunction_wrt_xSlide;
            if (i == varIndex_Ys) innerDeriv = deriv_innerFunction_wrt_ySlide;
            if (i == varIndex_Xr) innerDeriv = deriv_innerFunction_wrt_xSlopeRef;
            if (i == varIndex_Yr) innerDeriv = deriv_innerFunction_wrt_ySlopeRef;
            return 2 * innerFunction * innerDeriv;
        }

        public override double second_deriv_wrt_ij(double[] x, int i, int j)
        {
            if (!(i == varIndex_Xb || i == varIndex_Yb ||
                i == varIndex_Xs || i == varIndex_Ys ||
                i == varIndex_Xr || i == varIndex_Yr))
                return 0;
            if (!(j == varIndex_Xb || j == varIndex_Yb ||
                j == varIndex_Xs || j == varIndex_Ys ||
                j == varIndex_Xr || j == varIndex_Yr))
                return 0;
            #region derivaties with xBlock1 (6)
            if (j == varIndex_Xb)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Xb)
            {
                if (j == varIndex_Xb) return 2 * sineAngle * deriv_innerFunction_wrt_xBlock;
                if (j == varIndex_Yb) return 2 * sineAngle * deriv_innerFunction_wrt_yBlock;
                if (j == varIndex_Yr)
                    return (2 * innerFunction * cosineAngle / denomSlopeY)
                           + 2 * deriv_innerFunction_wrt_ySlopeRef * sineAngle;
                if (j == varIndex_Ys)
                    return (-2 * innerFunction * cosineAngle / denomSlopeY)
                           + 2 * deriv_innerFunction_wrt_ySlide * sineAngle;
                if (j == varIndex_Xr)
                    return (2 * innerFunction * cosineAngle / denomSlopeX)
                           + 2 * deriv_innerFunction_wrt_xSlopeRef * sineAngle;
                if (j == varIndex_Xs)
                    return (-2 * innerFunction * cosineAngle / denomSlopeX)
                           + 2 * deriv_innerFunction_wrt_xSlide * sineAngle;
            }
            #endregion
            #region derivaties with yBlock1 (5)
            if (j == varIndex_Yb)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Yb)
            {
                if (j == varIndex_Yb) return 2 * cosineAngle * deriv_innerFunction_wrt_yBlock;
                if (j == varIndex_Yr)
                    return (-2 * innerFunction * sineAngle / denomSlopeY)
                           + 2 * deriv_innerFunction_wrt_ySlopeRef * cosineAngle;
                if (j == varIndex_Ys)
                    return (2 * innerFunction * sineAngle / denomSlopeY)
                           + 2 * deriv_innerFunction_wrt_ySlide * cosineAngle;
                if (j == varIndex_Xr)
                    return (-2 * innerFunction * sineAngle / denomSlopeX)
                           + 2 * deriv_innerFunction_wrt_xSlopeRef * cosineAngle;
                if (j == varIndex_Xs)
                    return (2 * innerFunction * sineAngle / denomSlopeX)
                           + 2 * deriv_innerFunction_wrt_xSlide * cosineAngle;
            }
            #endregion
            #region derivatives with ySlide2   (4)
            if (j == varIndex_Yr)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Yr)
            {
                if (j == varIndex_Yr)
                    return ((-vectorX * sineAngle - vectorY * cosineAngle) / denomSlopeY)
                        * (2 * innerFunction / denomSlopeY) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_ySlopeRef * denomSlopeY
                            - 4 * innerFunction * (slopeDeltaY / slopeDeltaX)) / (denomSlopeY * denomSlopeY);
                if (j == varIndex_Ys)
                    return (((-vectorX * sineAngle - vectorY * cosineAngle) / -denomSlopeY) - sineAngle)
                          * (2 * innerFunction / denomSlopeY) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_ySlide * denomSlopeY
                            + 4 * innerFunction * (slopeDeltaY / slopeDeltaX)) / (denomSlopeY * denomSlopeY);
                if (j == varIndex_Xr)
                    return ((-vectorX * sineAngle - vectorY * cosineAngle) / denomSlopeX)
                          * (2 * innerFunction / denomSlopeY) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_xSlopeRef * denomSlopeY
                            + 4 * innerFunction * squaredDeltaY / Math.Pow(slopeDeltaX, 3)) / (denomSlopeY * denomSlopeY);
                if (j == varIndex_Xs)
                    return (((-vectorX * sineAngle - vectorY * cosineAngle) / -denomSlopeX) - cosineAngle)
                         * (2 * innerFunction / denomSlopeY) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_xSlide * denomSlopeY
                            - 4 * innerFunction * squaredDeltaY / Math.Pow(slopeDeltaX, 3)) / (denomSlopeY * denomSlopeY);
            }
            #endregion
            #region derivatives with xSlide2 (3)
            if (j == varIndex_Xr)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Xr)
            {
                if (j == varIndex_Ys)
                    return (((-vectorX * sineAngle - vectorY * cosineAngle) / -denomSlopeY) - sineAngle)
                         * (2 * innerFunction / denomSlopeX) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           ((2 * innerFunction + 2 * slopeDeltaY * deriv_innerFunction_wrt_ySlide) * sumSquaredDeltas - 4 * innerFunction * squaredDeltaY) /
                           (sumSquaredDeltas * sumSquaredDeltas);
                if (j == varIndex_Xr)
                    return ((-vectorX * sineAngle - vectorY * cosineAngle) / denomSlopeX)
                      * (2 * innerFunction / denomSlopeX) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (-2 * slopeDeltaY * deriv_innerFunction_wrt_xSlopeRef * sumSquaredDeltas + (4 * innerFunction * slopeDeltaY * slopeDeltaX)) /
                           (sumSquaredDeltas * sumSquaredDeltas);
                if (j == varIndex_Xs)
                    return (((-vectorX * sineAngle - vectorY * cosineAngle) / -denomSlopeX) - cosineAngle)
                        * (2 * innerFunction / denomSlopeX) +
                           (vectorX * cosineAngle - vectorY * sineAngle) *
                           (-2 * slopeDeltaY * deriv_innerFunction_wrt_xSlide * sumSquaredDeltas - (4 * innerFunction * slopeDeltaY * slopeDeltaX)) /
                           (sumSquaredDeltas * sumSquaredDeltas);
            }
            #endregion
            #region derivatives with ySlide1 (2)
            if (j == varIndex_Ys)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Ys)
            {
                if (j == varIndex_Ys)
                    return (((vectorX * sineAngle + vectorY * cosineAngle) / -denomSlopeY) + sineAngle)
                          * (2 * innerFunction / denomSlopeY) +
                           (-vectorX * cosineAngle + vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_ySlide * denomSlopeY
                            + 4 * innerFunction * (slopeDeltaY / slopeDeltaX)) / (denomSlopeY * denomSlopeY)
                            + 2 * deriv_innerFunction_wrt_ySlide * cosineAngle + 2 * innerFunction * sineAngle / denomSlopeY;
                if (j == varIndex_Xs)
                    return (((vectorX * sineAngle - vectorY * cosineAngle) / -denomSlopeX) - cosineAngle)
                         * (2 * innerFunction / denomSlopeY) +
                           (-vectorX * cosineAngle + vectorY * sineAngle) *
                           (2 * deriv_innerFunction_wrt_xSlide * denomSlopeY
                            - 4 * innerFunction * squaredDeltaY / Math.Pow(slopeDeltaX, 3)) / (denomSlopeY * denomSlopeY)
                            + 2 * deriv_innerFunction_wrt_xSlide * cosineAngle - 2 * innerFunction * sineAngle / denomSlopeX;
            }
            #endregion
            #region derivatives with xSlide1 (just double-deriv of xs1)
            if (i == varIndex_Xs && j == varIndex_Xs)
                return (((vectorX * sineAngle - vectorY * cosineAngle) / -denomSlopeX) - cosineAngle)
                     * (2 * innerFunction / denomSlopeX) +
                       (-vectorX * cosineAngle + vectorY * sineAngle) *
                       (2 * deriv_innerFunction_wrt_xSlide * denomSlopeY
                        - 4 * innerFunction * squaredDeltaY / Math.Pow(slopeDeltaX, 3)) / (denomSlopeY * denomSlopeY)
                        + 2 * deriv_innerFunction_wrt_xSlide * sineAngle + 2 * innerFunction * cosineAngle / denomSlopeX;
            #endregion
            throw new Exception("2nd Derivative in SameAngleAcrossPJoint: you shouldn't be seeing this! how did you get by the initial if-statement?");
        }

    }
}