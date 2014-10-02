using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;

namespace PMKS.PositionSolving
{

    internal class SameSlideAcrossPJointLinks : SameSlideAcrossJointAbstract
    {
        internal SameSlideAcrossPJointLinks(int varIndexBlock, int jointIndexBlock,
            int varIndexSlide, int jointIndexSlide,
            int varIndexRef, int jointIndexRef,
            double angleOffset, double distToSlide)
            /** Note that the block and slide are switched s.t. the angle works out correctly. **/
            : base(varIndexSlide, jointIndexSlide,
            varIndexBlock, jointIndexBlock,
            varIndexRef, jointIndexRef,
             angleOffset, -distToSlide)
        {
        }

    }
    internal class SameSlideAcrossRPJointLinks : SameSlideAcrossJointAbstract
    {
        internal SameSlideAcrossRPJointLinks(int varIndexBlock, int jointIndexBlock,
            int varIndexSlide, int jointIndexSlide,
            int varIndexRef, int jointIndexRef,
            double angleOffset, double distToSlide)
            : base(varIndexBlock, jointIndexBlock,
             varIndexSlide, jointIndexSlide,
             varIndexRef, jointIndexRef,
             angleOffset, distToSlide)
        {
        }

    }
    internal abstract class SameSlideAcrossJointAbstract : NonDyadicObjFunctionTerm
    {
        #region the following fields are only set in the constructor
        protected readonly int jointIndexBlock;
        protected readonly int jointIndexSlide;
        protected readonly int jointIndexRef;
        protected readonly int varIndex_Xb;
        protected readonly int varIndex_Yb;
        protected readonly int varIndex_Xs;
        protected readonly int varIndex_Ys;
        protected readonly int varIndex_Xr;
        protected readonly int varIndex_Yr;
        protected readonly double angleOffset;
        protected readonly double distToSlide;
        #endregion

        #region the following 6 variables are NOT readonly since these are part of what is being optimized


        // check on Wolfram Alpha: d/db (l+(w-b)*cos(h+atan((v-y)/(u-b)))+(y-c)*sin(h+atan((v-y)/(u-b))))^2
        protected double xB, yB;       //  xBlock, yBlock = b & c
        protected double xS, yS;       //  xSlide, ySlide = w & y
        protected double xR, yR;           //  xRef, yRef = u & v
        #endregion

        #region the following feilds are solved  in assignpositions
        protected Boolean newPointReCalcDerivConstant;
        protected double slideBaseAngle;
        protected double innerFunction;
        protected double slopeDeltaX, slopeDeltaY;
        protected double vectorX, vectorY;
        protected double sumSquaredDeltas, squaredDeltaX, squaredDeltaY;
        protected double derivAngle_xR, derivAngle_yR;
        protected double derivAngle_xS, derivAngle_yS;
        protected double sineAngle, cosineAngle;
        protected Dictionary<int, double> InnerDerivatives;

        #endregion

        protected SameSlideAcrossJointAbstract(int varIndexBlock, int jointIndexBlock,
            int varIndexSlide, int jointIndexSlide,
            int varIndexRef, int jointIndexRef,
            double angleOffset, double distToSlide)
        {
            this.varIndex_Xb = 2 * varIndexBlock;
            this.varIndex_Yb = 2 * varIndexBlock + 1;
            this.jointIndexBlock = jointIndexBlock;

            this.varIndex_Xs = 2 * varIndexSlide;
            this.varIndex_Ys = 2 * varIndexSlide + 1;
            this.jointIndexSlide = jointIndexSlide;

            this.varIndex_Xr = 2 * varIndexRef;
            this.varIndex_Yr = 2 * varIndexRef + 1;
            this.jointIndexRef = jointIndexRef;

            this.angleOffset = angleOffset;
            this.distToSlide = distToSlide;

            this.InnerDerivatives = new Dictionary<int, double>();
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
                xB = x[varIndex_Xb];
                yB = x[varIndex_Yb];
            }
            if (varIndex_Xs >= 0)
            {
                xS = x[varIndex_Xs];
                yS = x[varIndex_Ys];
            }
            if (varIndex_Xr >= 0)
            {
                xR = x[varIndex_Xr];
                yR = x[varIndex_Yr];
            }
            slopeDeltaX = xR - xS;
            slopeDeltaY = yR - yS;
            slideBaseAngle = Math.Atan2(slopeDeltaY, slopeDeltaX);
            sineAngle = Math.Sin(angleOffset + slideBaseAngle);
            cosineAngle = Math.Cos(angleOffset + slideBaseAngle);
            vectorX = xS - xB;
            vectorY = yS - yB;
            innerFunction = distToSlide + vectorX * sineAngle
                            - vectorY * cosineAngle;
            newPointReCalcDerivConstant = true;
        }


        internal override void SetInitialJointPosition(int index, double xNew, double yNew)
        {
            if (index == jointIndexBlock)
            {
                xB = xNew;
                yB = yNew;
            }
            else if (index == jointIndexSlide)
            {
                xS = xNew;
                yS = yNew;
            }
            else if (index == jointIndexRef)
            {
                xR = xNew;
                yR = yNew;
            }
        }

        public override double deriv_wrt_xi(double[] x, int i)
        {
            if (!(i == varIndex_Xb || i == varIndex_Yb ||
               i == varIndex_Xs || i == varIndex_Ys ||
               i == varIndex_Xr || i == varIndex_Yr))
                return 0;

            if (newPointReCalcDerivConstant)
            {
                slopeDeltaX = xR - xS;
                squaredDeltaX = slopeDeltaX * slopeDeltaX;
                slopeDeltaY = yR - yS;
                squaredDeltaY = slopeDeltaY * slopeDeltaY;
                sumSquaredDeltas = squaredDeltaX + squaredDeltaY;

                derivAngle_yR = 1 / ((squaredDeltaY / slopeDeltaX) + slopeDeltaX);
                derivAngle_yS = -derivAngle_yR;
                derivAngle_xS = slopeDeltaY / sumSquaredDeltas;  // really this is a negative of a negative
                derivAngle_xR = -derivAngle_xS;                  // for simplicity, I defined them backwards.
                var derivOfMainTerms = (vectorX * cosineAngle) + (vectorY * sineAngle);
                InnerDerivatives.Clear();
                if (varIndex_Xb >= 0) InnerDerivatives.Add(varIndex_Xb, -sineAngle);
                if (varIndex_Yb >= 0) InnerDerivatives.Add(varIndex_Yb, -cosineAngle);
                if (varIndex_Yr >= 0)
                    InnerDerivatives.Add(varIndex_Yr, derivOfMainTerms * derivAngle_yR);
                if (varIndex_Xr >= 0)
                    InnerDerivatives.Add(varIndex_Xr, derivOfMainTerms * derivAngle_xR);
                if (varIndex_Ys >= 0)
                    InnerDerivatives.Add(varIndex_Ys, derivOfMainTerms * derivAngle_yS - cosineAngle);
                /* note that slide has an extra term because it is part of the vectorY. Reference is not. */
                if (varIndex_Xs >= 0)
                    InnerDerivatives.Add(varIndex_Xs, derivOfMainTerms * derivAngle_xS + sineAngle);

                newPointReCalcDerivConstant = false;
            }
            return 2 * innerFunction * InnerDerivatives[i];
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
            var doubleDerivIJ = 0.0;
            #region derivaties with xBlock1 (6)
            if (j == varIndex_Xb)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Xb)
            {   /* dg/dxb = -sineAngle */
                if (j == varIndex_Xb || j == varIndex_Yb) doubleDerivIJ = 0;
                if (j == varIndex_Yr) doubleDerivIJ = derivAngle_yR;
                if (j == varIndex_Xr) doubleDerivIJ = derivAngle_xR;
                if (j == varIndex_Ys) doubleDerivIJ = derivAngle_yS;
                if (j == varIndex_Xs) doubleDerivIJ = derivAngle_xS;
                doubleDerivIJ *= -cosineAngle;
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
            {   /* dg/dxb = -cosineAngle */
                if (j == varIndex_Yb) doubleDerivIJ = 0;
                if (j == varIndex_Yr) doubleDerivIJ = derivAngle_yR;
                if (j == varIndex_Xr) doubleDerivIJ = derivAngle_xR;
                if (j == varIndex_Ys) doubleDerivIJ = derivAngle_yS;
                if (j == varIndex_Xs) doubleDerivIJ = derivAngle_xS;
                doubleDerivIJ *= sineAngle;
            }
            #endregion
            var derivOfMainTerms = (vectorX * cosineAngle) + (vectorY * sineAngle);
            #region derivatives with yRef   (4)
            if (j == varIndex_Yr)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Yr)
            {   /* dg/dyR =  ((vectorX * cosineAngle) + (vectorY * sineAngle)) * derivAngle_yR      */
                var derivOfderivOfMainTerms = 0.0;
                var derivOfderivAngle_yR = 0.0;
                if (j == varIndex_Yr)
                {
                    derivOfderivOfMainTerms = vectorY * cosineAngle * derivAngle_yR;
                    derivOfderivAngle_yR = -2 * slopeDeltaX * slopeDeltaY / (sumSquaredDeltas * sumSquaredDeltas);
                }
                if (j == varIndex_Xr)
                {
                    derivOfderivOfMainTerms = -vectorX * sineAngle * derivAngle_xR;
                    derivOfderivAngle_yR = -(xR + yR - xS - yS) * (xR - yR - xS + yS)
                        / (sumSquaredDeltas * sumSquaredDeltas);
                }
                if (j == varIndex_Ys)
                {
                    derivOfderivOfMainTerms = vectorY * cosineAngle * derivAngle_yS + sineAngle;
                    derivOfderivAngle_yR = 2 * slopeDeltaX * slopeDeltaY / (sumSquaredDeltas * sumSquaredDeltas);
                }
                if (j == varIndex_Xs)
                {
                    derivOfderivOfMainTerms = -vectorX * sineAngle * derivAngle_xS + cosineAngle;
                    derivOfderivAngle_yR = (xR + yR - xS - yS) * (xR - yR - xS + yS)
                    / (sumSquaredDeltas * sumSquaredDeltas);
                }
                doubleDerivIJ = derivOfMainTerms * derivOfderivAngle_yR + derivOfderivOfMainTerms * derivAngle_yR;
            }
            #endregion
            #region derivatives with xRef   (3)
            if (j == varIndex_Xr)
            {
                //switch so that i is always varIndex_Yb1 and go into next condition
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == varIndex_Xr)
            {   /* dg/dxR =  ((vectorX * cosineAngle) + (vectorY * sineAngle)) * derivAngle_xR      */
                var derivOfderivOfMainTerms = 0.0;
                var derivOfderivAngle_xR = 0.0;
                if (j == varIndex_Xr)
                {
                    derivOfderivOfMainTerms = -vectorX * sineAngle * derivAngle_xR;
                    derivOfderivAngle_xR = 2 * slopeDeltaX * slopeDeltaY / (sumSquaredDeltas * sumSquaredDeltas);
                }
                if (j == varIndex_Ys)
                {
                    derivOfderivOfMainTerms = vectorY * cosineAngle * derivAngle_yS + sineAngle;
                    derivOfderivAngle_xR = (squaredDeltaY - squaredDeltaX) / (sumSquaredDeltas * sumSquaredDeltas);
                }
                if (j == varIndex_Xs)
                {
                    derivOfderivOfMainTerms = -vectorX * sineAngle * derivAngle_xS + cosineAngle;
                    derivOfderivAngle_xR = -2 * slopeDeltaX * slopeDeltaY / (sumSquaredDeltas * sumSquaredDeltas);
                }
                doubleDerivIJ = derivOfMainTerms * derivOfderivAngle_xR + derivOfderivOfMainTerms * derivAngle_xR;
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
            {   /* dg/dyS =  ((vectorX * cosineAngle) + (vectorY * sineAngle)) * derivAngle_yS - cosineAngle*/
                var derivOfderivOfMainTerms = 0.0;
                var derivOfderivAngle_yS = 0.0;
                var derivThirdTerm = 0.0;
                if (j == varIndex_Ys)
                {
                    derivOfderivOfMainTerms = vectorY * cosineAngle * derivAngle_yS + sineAngle;
                    derivOfderivAngle_yS = -2 * slopeDeltaX * slopeDeltaY / (sumSquaredDeltas * sumSquaredDeltas);
                    derivThirdTerm = sineAngle * derivAngle_yS;
                }
                if (j == varIndex_Xs)
                {
                    derivOfderivOfMainTerms = -vectorX * sineAngle * derivAngle_xS + cosineAngle;
                    derivOfderivAngle_yS = -(xR + yR - xS - yS) * (xR - yR - xS + yS)
                    / (sumSquaredDeltas * sumSquaredDeltas);
                    derivThirdTerm = sineAngle * derivAngle_xS;
                }
                doubleDerivIJ = derivOfMainTerms * derivOfderivAngle_yS + derivOfderivOfMainTerms * derivAngle_yS
                    + derivThirdTerm;
            }
            #endregion
            #region derivatives with xSlide1 (just double-deriv of xs1)
            if (i == varIndex_Xs && j == varIndex_Xs)
            {   /* dg/dxS =  ((vectorX * cosineAngle) + (vectorY * sineAngle))  * derivAngle_xS + sineAngle*/
                var derivOfderivOfMainTerms = -vectorX * sineAngle * derivAngle_xS + cosineAngle;
                var derivOfderivAngle_xS = 2 * slopeDeltaX * slopeDeltaY / (sumSquaredDeltas * sumSquaredDeltas);
                var derivThirdTerm = cosineAngle * derivAngle_xR;
                doubleDerivIJ = derivOfMainTerms * derivOfderivAngle_xS + derivOfderivOfMainTerms * derivAngle_yS
                    + derivThirdTerm;
            }
            #endregion

            return 2 * (InnerDerivatives[i] * InnerDerivatives[j] + innerFunction * doubleDerivIJ);
        }

    }
}