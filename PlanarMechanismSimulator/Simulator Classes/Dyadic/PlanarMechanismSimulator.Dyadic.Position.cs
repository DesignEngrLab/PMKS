using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    /// <summary>
    /// 
    /// </summary>
    public partial class Simulator : IDependentAnalysis
    {
        private double positionError;
        internal enum PositionAnalysisResults { NoSolvableDyadFound, Normal, InvalidPosition, BranchingProbable }

        private PositionAnalysisResults posResult;
        private const double BranchRatio = 0.5;
        private Boolean DefineNewPositions(double positionChange, double[,] currentJointParams, double[,] currentLinkParams,
            double[,] oldJointParams, double[,] oldLinkParams)
        {
            InitializeJointsAndLinks(positionChange, currentJointParams, currentLinkParams, oldJointParams, oldLinkParams);
            var numUnknownJoints = joints.Count(j => j.knownState != KnownState.Fully);
            if (numUnknownJoints == 0)
            {
                WriteValuesToMatrices(currentJointParams, currentLinkParams);
                return true;
            }
            do
            {
                posResult = PositionAnalysisResults.NoSolvableDyadFound;
                foreach (var j in joints.Where(j => j.knownState != KnownState.Fully && j.Link2 != null))
                {
                    joint knownJoint1;
                    joint knownJoint2;
                    double angleChange;
                    switch (j.jointType)
                    {
                        case JointTypes.R:
                            #region R-R-R
                            if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaCircleIntersection(j, knownJoint1, knownJoint2);
                                assignJointPosition(j, j.Link1, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(j, j.Link1);
                                setLinkPositionFromRotate(j, j.Link2);
                            }
                            #endregion
                            #region R-R-P
                            else if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaCircleAndLineIntersection(j, knownJoint1, knownJoint2, out angleChange);
                                assignJointPosition(j, j.Link1, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(j, j.Link1, angleChange);
                                setLinkPositionFromTranslation(j, j.Link2, sJPoint.x, sJPoint.y);
                            }
                            #endregion
                            #region P-R-R
                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                                     && FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaCircleAndLineIntersection(j, knownJoint2, knownJoint1, out angleChange);
                                assignJointPosition(j, j.Link1, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(j, j.Link2, angleChange);
                                setLinkPositionFromTranslation(j, j.Link1, sJPoint.x, sJPoint.y);
                            }
                            #endregion
                            #region P-R-P
                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                                     && FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaIntersectingLines(j, knownJoint1, knownJoint2);
                                assignJointPosition(j, j.Link1, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromTranslation(j, j.Link1, sJPoint.x, sJPoint.y);
                                setLinkPositionFromTranslation(j, j.Link2, sJPoint.x, sJPoint.y);
                            }
                            #endregion
                            break;
                        case JointTypes.P:
                            #region R-P-R
                            if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveRPRIntersection(j, knownJoint1, knownJoint2, out angleChange);
                                assignJointPosition(j, j.Link2, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(knownJoint1, j.Link1, angleChange);
                                setLinkPositionFromRotate(j, j.Link2, angleChange);
                            }
                            #endregion
                            #region P-P-R
                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                                     && FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                /* in this case, the block is on the rotating link and the slide is on the sliding link */
                                var sJPoint = solveViaSlopeToCircleIntersectionPPR(j, knownJoint1, knownJoint2, out angleChange);
                                assignJointPosition(j, j.Link2, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromTranslation(knownJoint1, j.Link1, sJPoint.x, sJPoint.y, j.SlideAngle);
                                setLinkPositionFromTranslation(j, j.Link2, sJPoint.x, sJPoint.y);
                            }
                            #endregion
                            #region R-P-P
                            else if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                /* in this case, the slide is on the rotating link and the block is on the sliding link */
                                point sJPoint = solveViaSlopeToCircleIntersectionRPP(j, knownJoint1, knownJoint2, out angleChange);
                                assignJointPosition(j, j.Link2, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(knownJoint1, j.Link1, angleChange);
                                setLinkPositionFromTranslation(j, j.Link2, sJPoint.x, sJPoint.y);
                            }
                            #endregion
                            #region P-P-P
                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaIntersectingLines(j, knownJoint1, knownJoint2);
                                assignJointPosition(j, j.Link1, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromTranslation(knownJoint1, j.Link1, sJPoint.x, sJPoint.y, knownJoint1.SlideAngle);
                                setLinkPositionFromTranslation(j, j.Link2, sJPoint.x, sJPoint.y, j.SlideAngle);
                            }
                            #endregion
                            break;
                        case JointTypes.RP:
                            #region R-RP-R&P
                            //if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                            //    FindKnownPositionAndSlopeOnLink(j.Link2, out knownJoint2))
                            //{
                            //    throw new Exception("I didn't think it was possible to have an unknown joint for R-RP-R&P");
                            //    /* why is it not possible?
                            //     * because j.Link2 would be the fixed pivot within the joint and since it was known fully, 
                            //     * j would have j.knownState = KnownState.Fully and would not be cycled over. */
                            //}
                            #endregion
                            #region P-RP-R&P
                            //else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                            //  &&
                            //  FindKnownPositionAndSlopeOnLink(j.Link2,
                            //                                  out knownJoint2))
                            //{
                            //    throw new Exception("I didn't think it was possible to have an unknown joint for R-RP-R&P");
                            //    /* same as above. */
                            //}
                            #endregion
                            #region R&P-RP-R
                            if (FindKnownPositionAndSlopeOnLink(j.Link1, out knownJoint1) &&
                                FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveRotatePinToSlot(j, knownJoint1, knownJoint2, out angleChange);
                                assignJointPosition(j, j.Link2, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(j, j.Link2, angleChange);
                            }
                            #endregion
                            #region R&P-RP-P
                            else if (FindKnownPositionAndSlopeOnLink(j.Link1, out knownJoint1) &&
                                    FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                /* not sure this is right, but j must be partially known so it has enough
                                 * information to define the first line. */
                                var sJPoint = solveViaIntersectingLines(j, j, knownJoint2);
                                assignJointPosition(j, j.Link1, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromTranslation(j, j.Link2, sJPoint.x, sJPoint.y, j.SlideAngle);
                            }
                            #endregion
                            break;
                        case JointTypes.G:
                            #region R-G-R&P
                            if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                FindKnownPositionAndSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveGearAngleAndPos_R_G_RP(j, knownJoint1, knownJoint2, out angleChange);
                                assignJointPosition(j, j.Link1, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(j, j.Link1, angleChange);
                            }
                            #endregion
                            #region R&P-G-R
                            if (FindKnownPositionAndSlopeOnLink(j.Link1,
                                    out knownJoint2)
    && FindKnownPositionOnLink(j.Link2, out knownJoint1))
                            {

                            }
                            #endregion

                            #region P-G-R&P

                            if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                                &&
                                FindKnownPositionAndSlopeOnLink(j.Link2,
                                                                out knownJoint2))
                            {

                            }

                            #endregion

                            #region R&P-G-P

                            if (FindKnownPositionAndSlopeOnLink(j.Link1,
                                                                out knownJoint1)
                                && FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {

                            }

                            #endregion

                            break;
                    }
                }
                numUnknownJoints = joints.Count(j => j.knownState != KnownState.Fully);
            } while (posResult != PositionAnalysisResults.NoSolvableDyadFound &&
                     numUnknownJoints > 0);
            if (posResult == PositionAnalysisResults.NoSolvableDyadFound && numUnknownJoints > 0)
            {
                if (NDPS == null)
                    NDPS = new NonDyadicPositionSolver(links, joints, firstInputJointIndex,
                                inputJointIndex, inputLinkIndex, epsilon);
                if (!NDPS.Run_PositionsAreClose()) return false;
            }
            WriteValuesToMatrices(currentJointParams, currentLinkParams);
            return true;
        }

        private void WriteValuesToMatrices(double[,] currentJointParams, double[,] currentLinkParams)
        {
            for (int i = 0; i < numJoints; i++)
            {
                var j = joints[i];
                currentJointParams[i, 0] = j.x;
                currentJointParams[i, 1] = j.y;
            }
            for (int i = 0; i < numLinks; i++)
            {
                var l = links[i];
                currentLinkParams[i, 0] = l.Angle;
            }
        }

        private void InitializeJointsAndLinks(double positionChange, double[,] currentJointParams, double[,] currentLinkParams,
            double[,] oldJointParams, double[,] oldLinkParams)
        {
            positionError = 0.0;
            joint fixedGndJoint = null;
            for (int i = 0; i < numJoints; i++)
            {
                var j = joints[i];
                j.xLast = oldJointParams[i, 0];
                j.yLast = oldJointParams[i, 1];
                j.xNumerical = currentJointParams[i, 0];
                j.yNumerical = currentJointParams[i, 1];
                if (i >= inputJointIndex && j.FixedWithRespectTo(groundLink) && fixedGndJoint == null)
                {
                    fixedGndJoint = j;
                    /* there has to be at least one joint connected to ground which is fixed to ground. */
                    // todo: if not, should probably create one in Simulator set-up functions (PlanarMechanismSimulator.Main.cs).
                    assignJointPosition(j, groundLink, j.xLast, j.yLast);
                }
                else joints[i].knownState = KnownState.Unknown;
            }
            for (int i = 0; i < numLinks; i++)
            {
                var l = links[i];
                l.AngleLast = oldLinkParams[i, 0];
                l.AngleNumerical = currentLinkParams[i, 0];
                l.AngleIsKnown = false;
            }
            setLinkPositionFromRotate(fixedGndJoint, groundLink, 0.0);
            if (inputJoint.jointType == JointTypes.R)
                setLinkPositionFromRotate(inputJoint, inputLink, positionChange);
            else if (inputJoint.jointType == JointTypes.P)
                setLinkPositionFromTranslation(inputJoint, inputLink, positionChange, positionChange,
                    inputJoint.SlideAngle);
            else throw new Exception("Input is not of type R or P (as currently required at the beginning of DefineNewPositions");
        }

        #region Dyad Solving Methods
        private point solveViaCircleIntersection(joint j, joint knownJoint1, joint knownJoint2)
        {
            var r1 = j.Link1.lengthBetween(j, knownJoint1);
            var r2 = j.Link2.lengthBetween(j, knownJoint2);
            /* taken from http://2000clicks.com/MathHelp/GeometryConicSectionCircleIntersection.aspx */
            if (Constants.sameCloseZero(r1)) return knownJoint1;
            if (Constants.sameCloseZero(r2)) return knownJoint2;

            var dSquared = Constants.distanceSqared(knownJoint1, knownJoint2);
            var ratio = (r1 * r1 - r2 * r2) / (2 * dSquared);
            var xBase = (knownJoint1.x + knownJoint2.x) / 2 + (knownJoint2.x - knownJoint1.x) * ratio;
            var yBase = (knownJoint1.y + knownJoint2.y) / 2 + (knownJoint2.y - knownJoint1.y) * ratio;
            var fourTimesKsquared = ((r1 + r2) * (r1 + r2) - dSquared) * (dSquared - (r1 - r2) * (r1 - r2));

            if (Constants.sameCloseZero(fourTimesKsquared)) return new point(xBase, yBase);
            if (fourTimesKsquared < 0) return new point(double.NaN, double.NaN);

            var K = Math.Sqrt(fourTimesKsquared) / 4;
            var xOffset = 2 * (knownJoint2.y - knownJoint1.y) * K / dSquared;
            var yOffset = 2 * (knownJoint1.x - knownJoint2.x) * K / dSquared;
            var xPos = xBase + xOffset;
            var yPos = yBase + yOffset;
            var xNeg = xBase - xOffset;
            var yNeg = yBase - yOffset;
            var distPosSquared = Constants.distanceSqared(xPos, yPos, j.xNumerical, j.yNumerical);
            var distNegSquared = Constants.distanceSqared(xNeg, yNeg, j.xNumerical, j.yNumerical);
            if (distNegSquared < distPosSquared)
                return new point(xNeg, yNeg);
            return new point(xPos, yPos);
        }

        private point solveViaCircleAndLineIntersection(joint j, joint circleCenterJoint, joint lineJoint, out double angleChange)
        {
            var circleLink = (j.Link1.joints.Contains(circleCenterJoint)) ? j.Link1 : j.Link2;
            var slideLink = (j.Link1 != circleLink) ? j.Link1 : j.Link2;
            var r1 = circleLink.lengthBetween(j, circleCenterJoint);
            angleChange = double.NaN;
            double slopeB = Math.Tan(lineJoint.SlideAngle);
            var ptB = defineParallelLineThroughJoint(j, lineJoint, slideLink);

            var ptC = Constants.solveViaIntersectingLines(-1 / slopeB, circleCenterJoint, slopeB, ptB);
            var distToChord = Constants.distance(circleCenterJoint, ptC);
            if (distToChord > r1) return new point(double.NaN, double.NaN);
            if (Constants.sameCloseZero(distToChord, r1)) return ptC;
            var alpha = Math.Acos(distToChord / r1);
            var thetaBase = Constants.angle(circleCenterJoint, ptC);
            var thetaNeg = thetaBase - alpha;
            var xNeg = circleCenterJoint.x + r1 * Math.Cos(thetaNeg);
            var yNeg = circleCenterJoint.y + r1 * Math.Sin(thetaNeg);
            var distNegSquared = Constants.distanceSqared(j.xNumerical, j.yNumerical, xNeg, yNeg);

            var thetaPos = thetaBase + alpha;
            var xPos = circleCenterJoint.x + r1 * Math.Cos(thetaPos);
            var yPos = circleCenterJoint.y + r1 * Math.Sin(thetaPos);
            var distPosSquared = Constants.distanceSqared(j.xNumerical, j.yNumerical, xPos, yPos);
            var oldTheta = Constants.angle(circleCenterJoint.xLast, circleCenterJoint.yLast, j.xLast, j.yLast);
            if (distNegSquared < distPosSquared)
            {
                angleChange = thetaNeg - oldTheta;
                return new point(xNeg, yNeg);
            }
            angleChange = thetaPos - oldTheta;
            return new point(xPos, yPos);
        }


        #region Methods for P-?-P
        private point solveViaIntersectingLines(joint j, joint knownJointA, joint knownJointB)
        {
            var ptA = defineParallelLineThroughJoint(j, knownJointA, j.Link1);
            var ptB = defineParallelLineThroughJoint(j, knownJointB, j.Link2);
            return Constants.solveViaIntersectingLines(Math.Tan(knownJointA.SlideAngle), ptA,
                Math.Tan(knownJointB.SlideAngle), ptB);
        }
        private point defineParallelLineThroughJoint(joint positionJoint, joint slopeJoint, link thisLink)
        {
            var thetaA = slopeJoint.SlideAngle;
            Boolean higherOffset;
            var orthoPt = thisLink.findOrthoPoint(positionJoint, slopeJoint, out higherOffset);
            var distance = Constants.distance(orthoPt, positionJoint);
            int plusOrMinus = (higherOffset) ? +1 : -1;
            return new point(slopeJoint.x + distance * Math.Cos(thetaA + plusOrMinus * Math.PI / 2),
                slopeJoint.y + distance * Math.Sin(thetaA + plusOrMinus * Math.PI / 2));
        }
        #endregion


        // the basis of R-P-R dyad determination method is the complex little function
        private point solveRPRIntersection(joint j, joint knownJoint1, joint knownJoint2, out double angleChange)
        {
            var rAC = j.Link2.lengthBetween(j, knownJoint2);
            var rBC = j.Link1.lengthBetween(j, knownJoint1);

            double oldTheta = Constants.angle(knownJoint2.xLast, knownJoint2.yLast, j.xLast, j.yLast);
            var alpha = j.Link2.angleOfBlockToJoint(j, knownJoint2);
            var numPt = new point(j.xNumerical, j.yNumerical);
            angleChange = double.NaN;
            var lAB = Constants.distance(knownJoint2, knownJoint1);
            var phi = Constants.angle(knownJoint2, knownJoint1);
            if ((rBC + rAC * Math.Sin(alpha)) > lAB) return new point(double.NaN, double.NaN);
            /* first, the internal positive case */
            var beta = Math.Asin((rBC + rAC * Math.Sin(alpha)) / lAB);
            var thetaIntPos = Math.PI - alpha - beta + phi;
            var xIntPos = knownJoint2.x + rAC * Math.Cos(thetaIntPos);
            var yIntPos = knownJoint2.y + rAC * Math.Sin(thetaIntPos);
            var distthetaIntPosSquared = Constants.distanceSqared(xIntPos, yIntPos, numPt.x, numPt.y);

            /* second, the internal negative case */
            var thetaIntNeg = beta + phi - alpha;
            var xIntNeg = knownJoint2.x + rAC * Math.Cos(thetaIntNeg);
            var yIntNeg = knownJoint2.y + rAC * Math.Sin(thetaIntNeg);
            var distthetaIntNegSquared = Constants.distanceSqared(xIntNeg, yIntNeg, numPt.x, numPt.y);

            /* first, the External positive case */
            beta = Math.Asin((rBC - rAC * Math.Sin(alpha)) / lAB);
            var thetaExtPos = Math.PI - alpha + beta + phi;
            var xExtPos = knownJoint2.x + rAC * Math.Cos(thetaExtPos);
            var yExtPos = knownJoint2.y + rAC * Math.Sin(thetaExtPos);
            var distthetaExtPosSquared = Constants.distanceSqared(xExtPos, yExtPos, numPt.x, numPt.y);

            /* second, the External negative case */
            var thetaExtNeg = phi - beta - alpha;
            var xExtNeg = knownJoint2.x + rAC * Math.Cos(thetaExtNeg);
            var yExtNeg = knownJoint2.y + rAC * Math.Sin(thetaExtNeg);
            var distthetaExtNegSquared = Constants.distanceSqared(xExtNeg, yExtNeg, numPt.x, numPt.y);

            var distance = new List<double>
                               {
                                   distthetaIntPosSquared, distthetaIntNegSquared,
                                   distthetaExtPosSquared, distthetaExtNegSquared
                               };
            var minDist = distance.Min();
            point jPoint;
            switch (distance.IndexOf(minDist))
            {
                case 0:
                    angleChange = thetaIntPos - oldTheta;
                    jPoint = new point(xIntPos, yIntPos);
                    break;
                case 1:
                    angleChange = thetaIntNeg - oldTheta;
                    jPoint = new point(xIntNeg, yIntNeg);
                    break;
                case 2:
                    angleChange = thetaExtPos - oldTheta;
                    jPoint = new point(xExtPos, yExtPos);
                    break;
                default:
                    angleChange = thetaExtNeg - oldTheta;
                    jPoint = new point(xExtNeg, yExtNeg);
                    break;
            }
            while (angleChange < -Math.PI / 2) angleChange += Math.PI;
            while (angleChange > Math.PI / 2) angleChange -= Math.PI;
            return jPoint;
        }



        private point solveViaSlopeToCircleIntersectionPPR(joint j, joint slideJoint, joint circCenterJoint,
             out double angleChange)
        {
            /* in this case, the block is on the rotating link and the slide is on the sliding link */
            var circleLink = j.Link2;
            var radius = circleLink.lengthBetween(j, circCenterJoint);
            var slideAngleC = j.SlideAngle;
            var alpha = circleLink.angleOfBlockToJoint(j, circCenterJoint);
            var thetaNeg = slideAngleC - alpha;
            var xNeg = circCenterJoint.x + radius * Math.Cos(thetaNeg);
            var yNeg = circCenterJoint.y + radius * Math.Sin(thetaNeg);
            var distNegSquared = Constants.distanceSqared(xNeg, yNeg, j.xNumerical, j.yNumerical);

            var thetaPos = (thetaNeg < 0) ? thetaNeg + Math.PI : thetaNeg - Math.PI;
            var xPos = circCenterJoint.x + radius * Math.Cos(thetaPos);
            var yPos = circCenterJoint.y + radius * Math.Sin(thetaPos);
            var distPosSquared = Constants.distanceSqared(xPos, yPos, j.xNumerical, j.yNumerical);

            var oldTheta = Constants.angle(slideJoint.xLast, slideJoint.yLast, j.xLast, j.yLast);
            if (distNegSquared < distPosSquared)
            {
                angleChange = thetaNeg - oldTheta;
                return new point(xNeg, yNeg);
            }
            angleChange = thetaPos - oldTheta;
            return new point(xPos, yPos);
        }

        private point solveViaSlopeToCircleIntersectionRPP(joint j, joint circCenterIndex, joint slideIndex,
             out double angleChange)
        { /* in this case, the slide is on the rotating link and the block is on the sliding link */
            //var numPtX = currentJointParams[jIndex, 0];
            //var numPtY = currentJointParams[jIndex, 1];
            //var circCenterPt = new point(currentJointParams[circCenterIndex, 0], currentJointParams[circCenterIndex, 0]);
            var rAC = j.Link2.lengthBetween(j, circCenterIndex);
            double slopeB = Math.Tan(slideIndex.SlideAngle);
            var ptB = defineParallelLineThroughJoint(j, slideIndex, j.Link2);
            // need to find proper link1 angle and thus slideAngle for goal, 
            // this will set up the line that goes through the point
            var alpha = j.Link2.angleOfBlockToJoint(j, slideIndex);
            var actualSlideAngle = slideIndex.SlideAngle + alpha;
            var thetaNeg = actualSlideAngle + Math.PI / 2;
            var orthoPt = new point(circCenterIndex.x + rAC * Math.Cos(thetaNeg), circCenterIndex.y + rAC * Math.Sin(thetaNeg));
            var slopeA = Math.Tan(thetaNeg);
            var ptNeg = Constants.solveViaIntersectingLines(slopeA, orthoPt, slopeB, ptB);
            var distNegSquared = Constants.distanceSqared(ptNeg.x, ptNeg.y, j.xNumerical, j.yNumerical);

            var thetaPos = thetaNeg + Math.PI;
            orthoPt = new point(circCenterIndex.x + rAC * Math.Cos(thetaPos), circCenterIndex.y + rAC * Math.Sin(thetaPos));
            slopeA = Math.Tan(thetaPos);
            var ptPos = Constants.solveViaIntersectingLines(slopeA, orthoPt, slopeB, ptB);
            var distPosSquared = Constants.distanceSqared(ptPos.x, ptPos.y, j.xNumerical, j.yNumerical);

            var oldTheta = Constants.angle(circCenterIndex.xLast, circCenterIndex.yLast, j.xLast, j.yLast);
            if (distNegSquared < distPosSquared)
            {
                angleChange = thetaNeg - oldTheta;
                return ptNeg;
            }
            angleChange = thetaPos - oldTheta;
            return ptPos;
        }
        #endregion

        #region Dyadic solving methods beyond -R- and -P-
        private point solveGearAngleAndPos_R_G_RP(joint j, joint rollingCenter, joint fixedCenter, out double angleChange)
        {
            var gData = gearsData[j];
            angleChange = Constants.angle(rollingCenter.x, rollingCenter.y,
                fixedCenter.x, fixedCenter.y);
            angleChange -= Constants.angle(rollingCenter.xLast, rollingCenter.yLast,
                fixedCenter.xLast, fixedCenter.yLast);
            if (rollingCenter.jointType != JointTypes.P)
            {
                var g2AngleChange = j.Link2.AngleNumerical - j.Link2.AngleLast;
                angleChange -= gData.radius2 * g2AngleChange / gData.radius1;
            }
            var ratio = gData.radius1 / (gData.radius1 + gData.radius2);
            var x = rollingCenter.x;
            x += ratio * (x - 1);  //todo: need to check this equation - where did it come from?
            var y = rollingCenter.y;
            y += ratio * (y - 1);
            return new point(x, y);
        }

        private point solveRotatePinToSlot(joint j, joint fixedJoint, joint circleCenterJoint, out double angleChange)
        {
            var circleLink = j.Link2;
            var fixedLink = j.Link1;
            var r1 = circleLink.lengthBetween(j, circleCenterJoint);
            double slopeB = Math.Tan(j.SlideAngle);

            var ptC = Constants.solveViaIntersectingLines(-1 / slopeB, circleCenterJoint, slopeB, j);
            var distToChord = Constants.distance(circleCenterJoint, ptC);
            if (distToChord > r1)
            {
                angleChange = double.NaN;
                return new point(double.NaN, double.NaN);
            }
            if (Constants.sameCloseZero(distToChord, r1))
            {
                angleChange = 0.0;
                return ptC;
            }
            var alpha = Math.Acos(distToChord / r1);
            var thetaBase = Constants.angle(circleCenterJoint, ptC);
            var numPt = new point(j.xNumerical, j.yNumerical);
            var thetaNeg = thetaBase - alpha;
            var ptNeg = new point(circleCenterJoint.x + r1 * Math.Cos(thetaNeg), circleCenterJoint.y + r1 * Math.Sin(thetaNeg));
            var distNegSquared = Constants.distanceSqared(numPt, ptNeg);

            var thetaPos = thetaBase + alpha;
            var ptPos = new point(circleCenterJoint.x + r1 * Math.Cos(thetaPos), circleCenterJoint.y + r1 * Math.Sin(thetaPos));
            var distPosSquared = Constants.distanceSqared(numPt, ptPos);
            var oldTheta = Constants.angle(circleCenterJoint.xLast, circleCenterJoint.yLast, j.xLast, j.yLast);
            if (distNegSquared < distPosSquared)
            {
                angleChange = thetaNeg - oldTheta;
                return ptNeg;
            }
            angleChange = thetaPos - oldTheta;
            return ptPos;
        }

        private double solveRotateSlotToPin(joint fixedJoint, joint slideJoint, link thisLink)
        {
            Boolean higherOffset;
            var thetaA = slideJoint.SlideAngle;
            thisLink.findOrthoPoint(fixedJoint, slideJoint, out higherOffset);
            var distanceBetweenJoints = Constants.distance(fixedJoint, slideJoint);
            var oldDistance = Constants.distance(fixedJoint.xLast, fixedJoint.yLast,
                slideJoint.xLast, slideJoint.yLast);
            var dist2Slide = thisLink.lengthBetween(fixedJoint, slideJoint);
            if (dist2Slide > distanceBetweenJoints)
            {
                posResult = PositionAnalysisResults.InvalidPosition;
                return double.NaN;
            }
            if (thetaA > 0) return Math.Acos(dist2Slide / distanceBetweenJoints) - Math.Acos(dist2Slide / oldDistance);
            return Math.Acos(dist2Slide / oldDistance) - Math.Acos(dist2Slide / distanceBetweenJoints);
        }
        #endregion

        #region set & find link position and angles
        private void assignJointPosition(joint j, link thisLink, point newPoint)
        {
            assignJointPosition(j, thisLink, newPoint.x, newPoint.y);
        }

        private void assignJointPosition(joint j, link thisLink, double xNew, double yNew)
        {
            if (double.IsInfinity(xNew) || double.IsInfinity(yNew) ||
                double.IsNaN(xNew) || double.IsNaN(yNew))
                posResult = PositionAnalysisResults.InvalidPosition;
            else
            {
                j.knownState = j.SlidingWithRespectTo(thisLink) ? KnownState.Partially : KnownState.Fully;
                j.x = xNew; j.y = yNew;
                xNew -= j.xNumerical;
                yNew -= j.yNumerical;
                positionError += xNew * xNew + yNew * yNew;
                posResult = PositionAnalysisResults.Normal;
            }
        }


        private void setLinkPositionFromRotate(joint knownJoint, link thisLink, double angleChange = double.NaN)
        {
            if (knownJoint.knownState != KnownState.Fully) throw new Exception("not a knownstate assigned to knownJoint.");
            if (knownJoint.SlidingWithRespectTo(thisLink))
            {
                knownJoint = thisLink.joints.FirstOrDefault(j => j != knownJoint && j.knownState == KnownState.Fully
                        && j.FixedWithRespectTo(thisLink));
                if (knownJoint == null) return;
            }
            if (thisLink.AngleIsKnown) return;
            if (double.IsNaN(angleChange))
            {
                var otherKnownPosition = thisLink.joints.FirstOrDefault(j => j != knownJoint && j.knownState != KnownState.Unknown
                    && j.FixedWithRespectTo(thisLink));
                if (otherKnownPosition == null)
                    otherKnownPosition = thisLink.joints.FirstOrDefault(j => j != knownJoint && j.knownState != KnownState.Unknown);
                if (otherKnownPosition == null) return;
                var new_j2j_Angle = Constants.angle(knownJoint, otherKnownPosition);
                var old_j2j_Angle = Constants.angle(knownJoint.xLast, knownJoint.yLast, otherKnownPosition.xLast, otherKnownPosition.yLast);
                angleChange = new_j2j_Angle - old_j2j_Angle;
                if (otherKnownPosition.SlidingWithRespectTo(thisLink))
                    angleChange += solveRotateSlotToPin(knownJoint, otherKnownPosition, thisLink);
            }
            thisLink.Angle = thisLink.AngleLast + angleChange;
            thisLink.AngleIsKnown = true;
            foreach (var j in thisLink.joints.Where(j => j.knownState != KnownState.Fully))
            {
                var length = thisLink.lengthBetween(j, knownJoint);
                var angle = Constants.angle(knownJoint.xLast, knownJoint.yLast, j.xLast, j.yLast);
                angle += angleChange;
                assignJointPosition(j, thisLink, knownJoint.x + length * Math.Cos(angle),
                                    knownJoint.y + length * Math.Sin(angle));
                var otherLink = j.OtherLink(thisLink);
                if (otherLink != null && j.knownState == KnownState.Fully)
                    setLinkPositionFromRotate(j, otherLink, (j.jointType == JointTypes.P) ? angleChange : double.NaN);
            }
        }

        private void setLinkPositionFromTranslation(joint knownJoint, link thisLink,
            double deltaX, double deltaY, double angle = double.NaN)
        {
            foreach (var j in thisLink.joints.Where(j => j != knownJoint && j.knownState != KnownState.Fully))
            {
                if (double.IsNaN(angle))
                    assignJointPosition(j, thisLink, j.xLast + deltaX, j.yLast + deltaY);
                else
                    assignJointPosition(j, thisLink, j.xLast + deltaX * Math.Cos(angle),
                        j.yLast + deltaY * Math.Sin(angle));
                if (!j.FixedWithRespectTo(thisLink)) continue;
                if (j.jointType == JointTypes.P)
                    setLinkPositionFromTranslation(j, j.OtherLink(thisLink), deltaX, deltaY, j.SlideAngle);
                else setLinkPositionFromRotate(j, j.OtherLink(thisLink));
            }
        }



        private static bool FindKnownPositionAndSlopeOnLink(link link, out joint knownJoint)
        {
            knownJoint = null;
            if (!link.AngleIsKnown) return false;
            return FindKnownPositionOnLink(link, out knownJoint);
        }
        private static bool FindKnownPositionOnLink(link link, out joint knownJoint)
        {
            knownJoint = null;//knownJoint = link.joints.FirstOrDefault(j => !unknownPositions.Contains(j) && j.jointType == JointTypes.R && j.Link2 != null);
            knownJoint = link.joints.FirstOrDefault(j => j.knownState == KnownState.Fully);
            if (knownJoint != null) return true;
            return false;
        }
        private static bool FindKnownSlopeOnLink(link link, out joint knownJoint)
        {
            knownJoint = null;
            if (!link.AngleIsKnown) return false;
            //knownJoint = link.joints.FirstOrDefault(j => !unknownPositions.Contains(j) && j.jointType == JointTypes.P && j.Link2 != null);
            knownJoint = link.joints.FirstOrDefault(j => j.knownState != KnownState.Unknown);
            if (knownJoint != null) return true;
            return false;
        }
        #endregion
    }
}