using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;

namespace PMKS.PositionSolving
{
    internal abstract class SameSlideAcrossJointAbstract : NonDyadicObjFunctionTerm
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
        private double denomSlopeX, denomSlopeY;
        private double deriv_innerFunction_wrt_xBlock1;
        private double deriv_innerFunction_wrt_yBlock1;
        private double deriv_innerFunction_wrt_xSlide1;
        private double deriv_innerFunction_wrt_xSlide2;
        private double deriv_innerFunction_wrt_ySlide1;
        private double deriv_innerFunction_wrt_ySlide2;
        private double sineAngle, cosineAngle, secantBaseSquared;


        public override double deriv_wrt_xi(double[] x, int i)
        {
            if (!(i == 2 * varIndex_Xb1 || i == 2 * varIndex_Xb1 + 1 ||
                i == 2 * varIndex_Xs1 || i == 2 * varIndex_Xs1 + 1 ||
                i == 2 * varIndex_Xs2 || i == 2 * varIndex_Xs2 + 1))
                return 0;

            if (newPointReCalcDerivConstant)
            {
                denomSlopeY = 1 + (((ySlide2 - ySlide1) * (ySlide2 - ySlide1)) / ((xSlide2 - xSlide1) * (xSlide2 - xSlide1)));
                denomSlopeX = -1*(((xSlide2 - xSlide1) * (xSlide2 - xSlide1)) + ((ySlide2 - ySlide1) * (ySlide2 - ySlide1))) / (ySlide2-ySlide1 );
                deriv_innerFunction_wrt_xBlock1 = sineAngle;
                deriv_innerFunction_wrt_yBlock1 = cosineAngle;
                deriv_innerFunction_wrt_ySlide2 = (((xSlide1 - xBlock1) * cosineAngle) - ((ySlide1 - yBlock1) * sineAngle)) / denomSlopeY;
                deriv_innerFunction_wrt_xSlide2 = (((xSlide1 - xBlock1) * cosineAngle) - ((ySlide1 - yBlock1) * sineAngle)) / denomSlopeX;
                deriv_innerFunction_wrt_ySlide1 = ((((ySlide1 - yBlock1) * sineAngle) - ((xSlide1 - xBlock1) * cosineAngle)) / denomSlopeY)
                    + cosineAngle;/* note that slide1 has an extra term because it is part of the vector, d. Slide 2 is not. */
                deriv_innerFunction_wrt_xSlide1 = ((((ySlide1 - yBlock1) * sineAngle) - ((xSlide1 - xBlock1) * cosineAngle)) / denomSlopeX)
                    + sineAngle;/* note that slide1 has an extra term because it is part of the vector, d. Slide 2 is not. */
 
                newPointReCalcDerivConstant = false;
            }
            var innerDeriv = 0.0;
            if (i == varIndex_Xb1) innerDeriv = deriv_innerFunction_wrt_xBlock1;
            if (i == varIndex_Yb1) innerDeriv = deriv_innerFunction_wrt_yBlock1;
            if (i == varIndex_Xs1)    innerDeriv = deriv_innerFunction_wrt_xSlide1;
            if (i == varIndex_Ys1)    innerDeriv = deriv_innerFunction_wrt_ySlide1;
            if (i == varIndex_Xs2)  innerDeriv = deriv_innerFunction_wrt_xSlide2;
            if (i == varIndex_Ys2)  innerDeriv = deriv_innerFunction_wrt_ySlide2;
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
            #region derivaties with xBlock1
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
            #region derivaties with yBlock1
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
            #region derivatives with ySlide2
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
                    return (-2*innerFunction/denomSlopeY)*
                           ((xSlide1 - xBlock1)*sineAngle + (ySlide2 - yBlock1)*cosineAngle)
                           +
                           ((xSlide1 - xBlock1)*cosineAngle - (ySlide2 - yBlock1)*sineAngle)*
                           (2*deriv_innerFunction_wrt_ySlide2*denomSlopeY
                            - 4*innerFunction*((ySlide2-ySlide1)/(xSlide2-xSlide1)))/(denomSlopeY*denomSlopeY);
                if (j == varIndex_Ys1)
                    return 1;
                if (j == varIndex_Xs2)
                    return (- 2 * innerFunction / denomSlopeX)*
                           ((xSlide1 - xBlock1) * sineAngle + (ySlide2 - yBlock1) * cosineAngle)
                           +
                           ((xSlide1 - xBlock1) * cosineAngle - (ySlide2 - yBlock1) * sineAngle) *
                           (2 * deriv_innerFunction_wrt_xSlide2 * denomSlopeY
                            - 4 * innerFunction * (Math.Pow(ySlide2 - ySlide1,2) / Math.Pow(xSlide2 - xSlide1,3))) / (denomSlopeY * denomSlopeY);
                if (j == varIndex_Xs1)
                    return 1;

            }
            #endregion

            throw new Exception("2nd Derivative in SameAngleAcrossPJoint :you shouldn't be seeing this! how did you get by the initial if-statement?");
        }

    }
}