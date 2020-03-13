using System;

namespace PMKS.PositionSolving
{
    internal class SameAngleAcrossPJointLinks : NonDyadicObjFunctionTerm
    {
        private readonly int jointIndex_Block1;
        private readonly int jointIndex_Block2;
        private readonly int jointIndex_Slide1;
        private readonly int jointIndex_Slide2;
        private readonly int varIndex_Xb1;
        private readonly int varIndex_Xs1;
        private readonly int varIndex_Xb2;
        private readonly int varIndex_Xs2;
        private readonly int varIndex_Yb1;
        private readonly int varIndex_Ys1;
        private readonly int varIndex_Yb2;
        private readonly int varIndex_Ys2;
                                                  
        private readonly double AngleToLengthScaler;
        private readonly double initSlideAngle;
        private readonly double blockAngle;
        /* the following 8 variables are NOT readonly since these are part of what is being optimized */
        private double xBlock1, yBlock1, xBlock2, yBlock2;
        private double xSlide1, ySlide1, xSlide2, ySlide2;

        private Boolean newPointReCalcDerivConstant;
        private double slideBaseAngle, blockBaseAngle;
        private double innerFunction;
        private double denomBlock, denomSlide;
        private double deriv_innerFunction_wrt_xBlock1;
        private double deriv_innerFunction_wrt_xBlock2;
        private double deriv_innerFunction_wrt_yBlock1;
        private double deriv_innerFunction_wrt_yBlock2;
        private double deriv_innerFunction_wrt_xSlide1;
        private double deriv_innerFunction_wrt_xSlide2;
        private double deriv_innerFunction_wrt_ySlide1;
        private double deriv_innerFunction_wrt_ySlide2;

        internal SameAngleAcrossPJointLinks(int varIndexBlock1, int jointIndexBlock1, double xBlock1, double yBlock1,
            int varIndexBlock2, int jointIndexBlock2, double xBlock2, double yBlock2,
            int varIndexSlide1, int jointIndexSlide1, double xSlide1, double ySlide1,
            int varIndexSlide2, int jointIndexSlide2, double xSlide2, double ySlide2,
            double initSlideAngle, double blockAngle)
        {
            varIndex_Xb1 = 2 * varIndexBlock1;
            varIndex_Yb1 = 2 * varIndexBlock1 + 1;
            jointIndex_Block1 = jointIndexBlock1;
            this.xBlock1 = xBlock1;
            this.yBlock1 = yBlock1;
            varIndex_Xb2 = 2 * varIndexBlock2;
            varIndex_Yb2 = 2 * varIndexBlock2 + 1;
            jointIndex_Block2 = jointIndexBlock2;
            this.xBlock2 = xBlock2;
            this.yBlock2 = yBlock2;
            varIndex_Xs1 = 2 * varIndexSlide1;
            varIndex_Ys1 = 2 * varIndexSlide1 + 1;
            jointIndex_Slide1 = jointIndexSlide1;
            this.xSlide1 = xSlide1;
            this.ySlide1 = ySlide1;
            varIndex_Xs2 = 2 * varIndexSlide2;
            varIndex_Ys2 = 2 * varIndexSlide2 + 1;
            jointIndex_Slide2 = jointIndexSlide2;
            this.xSlide2 = xSlide2;
            this.ySlide2 = ySlide2;
            this.initSlideAngle = initSlideAngle;
            this.blockAngle = blockAngle;
            /* the following scaler is to convert the resulting angle-error into a length-error
             * the other objective function terms are all in terms of length squared. The factor is
             * derived as the average of the two distances between the two joints of each link. 
             * Then the factor is squared. */
            AngleToLengthScaler = (Math.Sqrt((xBlock2 - xBlock1)*(xBlock2 - xBlock1) +
                                                  (yBlock2 - yBlock1)*(yBlock2 - yBlock1))
                                        +
                                        Math.Sqrt((xSlide2 - xSlide1)*(xSlide2 - xSlide1) +
                                                  (ySlide2 - ySlide1)*(ySlide2 - ySlide1)))/2;
            AngleToLengthScaler *= AngleToLengthScaler;
        }

        internal override double calculate(double[] x)
        {
            assignPositions(x);
            return AngleToLengthScaler * innerFunction * innerFunction;
        }

        internal override double deriv_wrt_xi(double[] x, int i)
        {
            if (!(i == varIndex_Xb1 || i == varIndex_Yb1 ||
                i == varIndex_Xb2 || i == varIndex_Yb2 ||
                i == varIndex_Xs1 || i == varIndex_Ys1 ||
                i == varIndex_Xs2 || i == varIndex_Ys2))
                return 0;
            if (newPointReCalcDerivConstant)
            {
                denomBlock = (xBlock2 - xBlock1) * (xBlock2 - xBlock1) + (yBlock2 - yBlock1) * (yBlock2 - yBlock1);
                denomSlide = (xSlide2 - xSlide1) * (xSlide2 - xSlide1) + (ySlide2 - ySlide1) * (ySlide2 - ySlide1);


                deriv_innerFunction_wrt_xBlock1 = -1 * (yBlock2 - yBlock1) / denomBlock;
                deriv_innerFunction_wrt_xBlock2 = (yBlock2 - yBlock1) / denomBlock;
                deriv_innerFunction_wrt_yBlock1 = -1 * (xBlock2 - xBlock1) / denomBlock;
                deriv_innerFunction_wrt_yBlock2 = (xBlock2 - xBlock1) / denomBlock;

                deriv_innerFunction_wrt_xSlide1 = (ySlide2 - ySlide1) / denomSlide;
                deriv_innerFunction_wrt_xSlide2 = -1 * (ySlide2 - ySlide1) / denomSlide;
                deriv_innerFunction_wrt_ySlide1 = (xSlide2 - xSlide1) / denomSlide;
                deriv_innerFunction_wrt_ySlide2 = -1 * (xSlide2 - xSlide1) / denomSlide;
                newPointReCalcDerivConstant = false;
            }
            // assignPositions(x);
            var answer = 0.0;
            if (i == varIndex_Xb1)   // w.r.t. xBlock1
                answer = deriv_innerFunction_wrt_xBlock1;
            if (i == varIndex_Yb1)   // w.r.t. yBlock1
                answer = deriv_innerFunction_wrt_yBlock1;
            if (i == varIndex_Xb2)   // w.r.t. xBlock2
                answer = deriv_innerFunction_wrt_xBlock2;
            if (i == varIndex_Yb2)   // w.r.t. yBlock2
                answer = deriv_innerFunction_wrt_yBlock2;
            if (i == varIndex_Xs1)   // w.r.t. xSlide1
                answer = deriv_innerFunction_wrt_xSlide1;
            if (i == varIndex_Ys1)   // w.r.t. ySlide1
                answer = deriv_innerFunction_wrt_ySlide1;
            if (i == varIndex_Xs2)  // w.r.t. xSlide2
                answer = deriv_innerFunction_wrt_xSlide2;
            if (i == varIndex_Ys2)   // w.r.t. ySlide2
                answer = deriv_innerFunction_wrt_ySlide2;
            return 2 *AngleToLengthScaler * innerFunction * answer;
        }

        internal override double second_deriv_wrt_ij(double[] x, int i, int j)
        {
            if (!(i == varIndex_Xb1 || i == varIndex_Yb1 ||
                i == varIndex_Xb2 || i == varIndex_Yb2 ||
                i == varIndex_Xs1 || i == varIndex_Ys1 ||
                i == varIndex_Xs2 || i == varIndex_Ys2))
                return 0;
            if (!(j == varIndex_Xb1 || j == varIndex_Yb1 ||
                j == varIndex_Xb2 || j == varIndex_Yb2 ||
                j == varIndex_Xs1 || j == varIndex_Ys1 ||
                j == varIndex_Xs2 || j == varIndex_Ys2))
                return 0;
            #region derivatives in which one is a slide-Y variable
            if (i == varIndex_Ys2 || i == varIndex_Ys1 || j == varIndex_Ys2 || j == varIndex_Ys1)
            {
                if (j == varIndex_Ys2 || j == varIndex_Ys1)
                {
                    //switch so that i is always varIndex_Ys2 || i == varIndex_Ys1. Remember second derivative order has no affect
                    var temp = i;
                    i = j;
                    j = temp;
                }
                var outerSign = (i == varIndex_Ys2) ? 1 : -1;
                if (j == varIndex_Xb1 || j == varIndex_Xb2 || j == varIndex_Yb1 || j == varIndex_Yb2)
                {    /* for i = Ys1 or Ys2 */
                    var coeff = 2 * (xSlide2 - xSlide1) / denomSlide;
                    if (j == varIndex_Xb1) return AngleToLengthScaler * outerSign * coeff * deriv_innerFunction_wrt_xBlock1;
                    if (j == varIndex_Xb2) return AngleToLengthScaler * outerSign * coeff * deriv_innerFunction_wrt_xBlock2;
                    if (j == varIndex_Yb1) return AngleToLengthScaler * outerSign * coeff * deriv_innerFunction_wrt_yBlock1;
                    if (j == varIndex_Yb2) return AngleToLengthScaler * outerSign * coeff * deriv_innerFunction_wrt_yBlock2;
                    /* that's 8 down, 28 to go */
                }
                if (j == varIndex_Xs1)
                    return AngleToLengthScaler * ((2 * outerSign * innerFunction * ((xSlide2 - xSlide1) * deriv_innerFunction_wrt_xSlide1 - 1)) *
                            denomSlide + 4 * innerFunction * (xSlide2 - xSlide1) * (xSlide2 - xSlide1)) /
                           (denomSlide * denomSlide);
                if (j == varIndex_Xs2)
                    return AngleToLengthScaler * ((2 * outerSign * innerFunction * ((xSlide2 - xSlide1) * deriv_innerFunction_wrt_xSlide2 + 1)) *
                            denomSlide - 4 * innerFunction * (xSlide2 - xSlide1) * (xSlide2 - xSlide1)) /
                           (denomSlide * denomSlide);
                if (j == varIndex_Ys1)
                    return AngleToLengthScaler * (2 * outerSign * (xSlide2 - xSlide1) * deriv_innerFunction_wrt_ySlide1 * denomSlide +
                            4 * innerFunction * (xSlide2 - xSlide1) * (ySlide2 - ySlide1)) /
                           (denomSlide * denomSlide);
                if (j == varIndex_Ys2)
                    return AngleToLengthScaler * (2 * outerSign * (xSlide2 - xSlide1) * deriv_innerFunction_wrt_ySlide2 * denomSlide -
                            4 * innerFunction * (xSlide2 - xSlide1) * (ySlide2 - ySlide1)) /
                           (denomSlide * denomSlide);
            }
            #endregion
            #region derivatives in which one is a block-Y variable
            if (i == varIndex_Yb2 || i == varIndex_Yb1 || j == varIndex_Yb2 || j == varIndex_Yb1)
            {
                if (j == varIndex_Yb2 || j == varIndex_Yb1)
                {
                    //switch so that i is always varIndex_Yb2 || i == varIndex_Yb1. Remember second derivative order has no affect
                    var temp = i;
                    i = j;
                    j = temp;
                }
                var outerSign = (i == varIndex_Yb1) ? 1 : -1;
                if (j == varIndex_Xs1 || j == varIndex_Xs2 || j == varIndex_Ys1 || j == varIndex_Ys2)
                {
                    var coeff = 2 * (xBlock2 - xBlock1) / denomBlock;
                    if (j == varIndex_Xs1) return AngleToLengthScaler * outerSign * coeff * deriv_innerFunction_wrt_xSlide1;
                    if (j == varIndex_Xs2) return AngleToLengthScaler * outerSign * coeff * deriv_innerFunction_wrt_xSlide2;
                }
                if (j == varIndex_Xb1)
                    return AngleToLengthScaler * ((2 * outerSign * innerFunction * ((xBlock2 - xBlock1) * deriv_innerFunction_wrt_xBlock1 - 1)) *
                            denomBlock + 4 * innerFunction * (xBlock2 - xBlock1) * (xBlock2 - xBlock1)) /
                           (denomBlock * denomBlock);
                if (j == varIndex_Xb2)
                    return AngleToLengthScaler * ((2 * outerSign * innerFunction * ((xBlock2 - xBlock1) * deriv_innerFunction_wrt_xBlock2 + 1)) *
                            denomBlock - 4 * innerFunction * (xBlock2 - xBlock1) * (xBlock2 - xBlock1)) /
                           (denomBlock * denomBlock);
                if (j == varIndex_Yb1)
                    return AngleToLengthScaler * (2 * outerSign * (xBlock2 - xBlock1) * deriv_innerFunction_wrt_yBlock1 * denomBlock +
                            4 * innerFunction * (xBlock2 - xBlock1) * (yBlock2 - yBlock1)) /
                           (denomBlock * denomBlock);
                if (j == varIndex_Yb2)
                    return AngleToLengthScaler * (2 * outerSign * (xBlock2 - xBlock1) * deriv_innerFunction_wrt_yBlock2 * denomBlock -
                            4 * innerFunction * (xBlock2 - xBlock1) * (yBlock2 - yBlock1)) /
                           (denomBlock * denomBlock);
            }
            #endregion
            #region derivates in which both are X coordinates
            else
            {   /* the last 10 are del-X and del-X   */
                if ((i == varIndex_Xb1 || i == varIndex_Xb2) && (j == varIndex_Xs1 || j == varIndex_Xs2))
                {
                    /* this switches i and j and then goes into the next condition */
                    var temp = i;
                    i = j;
                    j = temp;
                }
                if ((i == varIndex_Xs1 || i == varIndex_Xs2) && (j == varIndex_Xb1 || j == varIndex_Xb2))
                {
                    var outerSign = (i == varIndex_Xs2) ? 1 : -1;
                    var innerDeriv = (j == varIndex_Xb1)
                        ? deriv_innerFunction_wrt_xBlock1
                        : deriv_innerFunction_wrt_xBlock2;
                    return AngleToLengthScaler * outerSign * 2 * innerFunction * (ySlide2 - ySlide1) * innerDeriv / denomSlide;
                }

                /* both are on the slide side */
                if ((i == varIndex_Xs1 || i == varIndex_Xs2) && (j == varIndex_Xs1 || j == varIndex_Xs2))
                {
                    var iSign = (i == varIndex_Xs1) ? 1 : -1;
                    var jSign = (j == varIndex_Xs1) ? 1 : -1;
                    var innerDeriv = (j == varIndex_Xs2)
                        ? deriv_innerFunction_wrt_xSlide2
                        : deriv_innerFunction_wrt_xSlide1;
                    return AngleToLengthScaler * (2.0 * jSign * (ySlide2 - ySlide1) * innerDeriv * denomSlide
                            + jSign * iSign * 4 * innerFunction * (ySlide2 - ySlide1) * (xSlide2 - xSlide1)) /
                           (denomSlide * denomSlide);
                }

                /* both are on the block side */
                if ((i == varIndex_Xb1 || i == varIndex_Xb2) && (j == varIndex_Xb1 || j == varIndex_Xb2))
                {
                    var iSign = (i == varIndex_Xb2) ? 1 : -1;
                    var jSign = (j == varIndex_Xb2) ? 1 : -1;
                    var innerDeriv = (j == varIndex_Xb2)
                        ? deriv_innerFunction_wrt_xBlock2
                        : deriv_innerFunction_wrt_xBlock1;
                    return AngleToLengthScaler * (2.0 * jSign * (yBlock2 - yBlock1) * innerDeriv * denomBlock
                           - jSign * iSign * 4 * innerFunction * (yBlock2 - yBlock1) * (xBlock2 - xBlock1)) /
                           (denomBlock * denomBlock);
                }
            }
            #endregion

            throw new Exception("2nd Derivative in SameAngleAcrossPJoint :you shouldn't be seeing this! how did you get by the initial if-statement?");
        }

        private void assignPositions(double[] x)
        {
            if (varIndex_Xb1 >= 0)
            {
                xBlock1 = x[ varIndex_Xb1];
                yBlock1 = x[ varIndex_Xb1 + 1];
            }
            if (varIndex_Xb2 >= 0)
            {
                xBlock2 = x[ varIndex_Xb2];
                yBlock2 = x[ varIndex_Xb2 + 1];
            }
            if (varIndex_Xs1 >= 0)
            {
                xSlide1 = x[ varIndex_Xs1];
                ySlide1 = x[ varIndex_Xs1 + 1];
            }
            if (varIndex_Xs2 >= 0)
            {
                xSlide2 = x[ varIndex_Xs2];
                ySlide2 = x[ varIndex_Xs2 + 1];
            }
            slideBaseAngle = Math.Atan2(ySlide2 - ySlide1, xSlide2 - xSlide1);
            blockBaseAngle = Math.Atan2(yBlock2 - yBlock1, xBlock2 - xBlock1);
            innerFunction = blockAngle + blockBaseAngle - (initSlideAngle + slideBaseAngle);
            newPointReCalcDerivConstant = true;

        }

        internal override void SetInitialJointPosition(int index, double x, double y)
        {
            if (index == jointIndex_Block1)
            {
                xBlock1 = x;
                yBlock1 = y;
            }
            else if (index == jointIndex_Block2)
            {
                xBlock2 = x;
                yBlock2 = y;
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