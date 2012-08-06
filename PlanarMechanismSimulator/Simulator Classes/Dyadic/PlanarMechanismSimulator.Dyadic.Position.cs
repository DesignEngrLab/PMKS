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
            positionError = 0.0;
            var fixedGndJointIndex = -1;
            for (int i = 0; i < numJoints; i++)
            {
                var j = joints[i];
                if (i >= inputJointIndex && j.FixedWithRespectTo(groundLink))
                {
                    fixedGndJointIndex = i;
                    assignJointPosition(i, groundLink, currentJointParams,
                        new point(oldJointParams[i, 0], oldJointParams[i, 1]));
                }
                else joints[i].knownState = KnownState.Unknown;
            }
            for (int i = 0; i < numLinks; i++) links[i].AngleIsKnown = false;

            setLinkPositionFromRotate(joints[fixedGndJointIndex], fixedGndJointIndex, inputLinkIndex + 1,
                                      currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, 0.0);
            if (inputJoint.jointType == JointTypes.R)
                setLinkPositionFromRotate(inputJoint, inputJointIndex, inputLinkIndex,
                                          currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, positionChange);
            else if (inputJoint.jointType == JointTypes.P)
                setLinkPositionFromTranslation(inputJoint, inputJointIndex, inputLinkIndex,
                                          currentJointParams, oldJointParams, currentLinkParams, oldLinkParams,
                                          positionChange, positionChange, slideAngle(inputJoint, currentLinkParams));
            else throw new Exception("Input is not of type R or P (as currently required at the beginning of DefineNewPositions");
            do
            {
                posResult = PositionAnalysisResults.NoSolvableDyadFound;
                for (int jIndex = 0; jIndex < numJoints; jIndex++)
                {
                    var j = joints[jIndex];
                    if (joints[jIndex].knownState == KnownState.Fully || joints[jIndex].Link2 == null) continue;
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
                                var sJPoint = solveViaCircleIntersection(j, jIndex, knownJoint1, knownJoint2, currentJointParams);
                                assignJointPosition(jIndex, j.Link1, currentJointParams, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(j, jIndex, links.IndexOf(j.Link1), currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams);
                                setLinkPositionFromRotate(j, jIndex, links.IndexOf(j.Link2), currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams);
                            }
                            #endregion
                            #region R-R-P
                            else if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaCircleAndLineIntersection(j, jIndex, knownJoint1, knownJoint2,
                                    currentJointParams, oldJointParams, currentLinkParams, out angleChange);
                                assignJointPosition(jIndex, j.Link1, currentJointParams, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(j, jIndex, links.IndexOf(j.Link1), currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkPositionFromTranslation(j, jIndex, links.IndexOf(j.Link2), currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams, sJPoint.X, sJPoint.Y);
                            }
                            #endregion
                            #region P-R-R
                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                                     && FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaCircleAndLineIntersection(j, jIndex, knownJoint2, knownJoint1,
                                    currentJointParams, oldJointParams, currentLinkParams, out angleChange);
                                assignJointPosition(jIndex, j.Link1, currentJointParams, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(j, jIndex, links.IndexOf(j.Link2), currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkPositionFromTranslation(j, jIndex, links.IndexOf(j.Link1), currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams, sJPoint.X, sJPoint.Y);
                            }
                            #endregion
                            #region P-R-P
                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                                     && FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaIntersectingLines(j, jIndex, knownJoint1, knownJoint2,
                                    currentJointParams, currentLinkParams);
                                assignJointPosition(jIndex, j.Link1, currentJointParams, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromTranslation(j, jIndex, links.IndexOf(j.Link1), currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams, sJPoint.X, sJPoint.Y);
                                setLinkPositionFromTranslation(j, jIndex, links.IndexOf(j.Link2), currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams, sJPoint.X, sJPoint.Y);
                            }
                            #endregion
                            break;
                        case JointTypes.P:
                            #region R-P-R
                            if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveRPRIntersection(j, jIndex, knownJoint1, knownJoint2,
                                                                   currentJointParams,
                                                                   oldJointParams, oldLinkParams, out angleChange);
                                assignJointPosition(jIndex, j.Link1, currentJointParams, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(knownJoint1, joints.IndexOf(knownJoint1), links.IndexOf(j.Link1),
                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
                                setLinkPositionFromRotate(j, jIndex, links.IndexOf(j.Link2),
                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
                            }
                            #endregion
                            #region P-P-R
                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                                     && FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                /* in this case, the block is on the rotating link and the slide is on the sliding link */
                                var kPIndex1 = joints.IndexOf(knownJoint1);
                                var kPIndex2 = joints.IndexOf(knownJoint2);
                                var sJPoint = solveViaSlopeToCircleIntersectionPPR(j, jIndex, kPIndex1, kPIndex2,
                                    currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, out angleChange);
                                assignJointPosition(jIndex, j.Link1, currentJointParams, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromTranslation(knownJoint1, kPIndex1, links.IndexOf(j.Link1),
                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, sJPoint.X, sJPoint.Y, slideAngle(j, currentLinkParams));
                                setLinkPositionFromTranslation(j, jIndex, links.IndexOf(j.Link2),
                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, sJPoint.X, sJPoint.Y);
                            }
                            #endregion
                            #region R-P-P
                            else if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                /* in this case, the slide is on the rotating link and the block is on the sliding link */
                                var kPIndex1 = joints.IndexOf(knownJoint1);
                                var kPIndex2 = joints.IndexOf(knownJoint2);
                                point sJPoint = solveViaSlopeToCircleIntersectionRPP(j, jIndex, kPIndex1, kPIndex2,
                                    currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, out angleChange);
                                assignJointPosition(jIndex, j.Link1, currentJointParams, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(knownJoint1, kPIndex1, links.IndexOf(j.Link1),
                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
                                setLinkPositionFromTranslation(j, jIndex, links.IndexOf(j.Link2),
                                                               currentJointParams, oldJointParams, currentLinkParams, oldLinkParams,
                                                               sJPoint.X, sJPoint.Y);
                            }
                            #endregion
                            #region P-P-P
                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaIntersectingLines(j, jIndex, knownJoint1, knownJoint2,
                                    currentJointParams, currentLinkParams);
                                assignJointPosition(jIndex, j.Link1, currentJointParams, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromTranslation(knownJoint1, joints.IndexOf(knownJoint1), links.IndexOf(j.Link1),
                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, sJPoint.X, sJPoint.Y, slideAngle(j, currentLinkParams));
                                setLinkPositionFromTranslation(j, jIndex, links.IndexOf(j.Link2), currentJointParams, oldJointParams,
                                    currentLinkParams, oldLinkParams, sJPoint.X, sJPoint.Y);
                            }
                            #endregion
                            break;
                        case JointTypes.RP:
                            #region R-RP-R&P
                            if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                FindKnownPositionAndSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                throw new Exception("I didn't think it was possible to have an unknown joint for R-RP-R&P");
                            }
                            #endregion
                            #region R&P-RP-R
                            else if (FindKnownPositionAndSlopeOnLink(j.Link1, out knownJoint1) &&
                                FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveRotatePinToSlot(j, jIndex, joints.IndexOf(knownJoint2), currentJointParams,
                                    oldJointParams, currentLinkParams, out angleChange);
                                assignJointPosition(jIndex, j.Link2, currentJointParams, sJPoint);
                                if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                                setLinkPositionFromRotate(j, jIndex, links.IndexOf(j.Link2), currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                            }

                            #endregion

                            #region P-RP-R&P

                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                              &&
                              FindKnownPositionAndSlopeOnLink(j.Link2,
                                                              out knownJoint2))
                            {
                                //tricky - just need to assign position to j.Link1 but there be nothing to assign position to on that link
                                // if both/all joints are slides.
                            }

                            #endregion

                            #region R&P-RP-P

                            else if (FindKnownPositionAndSlopeOnLink(j.Link1,
                                                               out knownJoint1)
                               && FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                // intersection of two lines
                            }

                            #endregion

                            break;
                        case JointTypes.G:

                            #region R-G-R&P

                            if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                FindKnownPositionAndSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveGearAngleAndPos_R_G_RP(j, currentJointParams, oldJointParams,
                                                                          currentLinkParams, oldLinkParams,
                                                                          out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) ||
                                    double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                //need to plan how to displace rack (use setLinkPositionFromSlideTranslation)
                                var deltaX = sJPoint.X - oldJointParams[jIndex, 0];
                                var deltaY = sJPoint.Y - oldJointParams[jIndex, 1];
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint1, joints.IndexOf(knownJoint1),
                                                          links.IndexOf(j.Link1),
                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
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
            } while (posResult != PositionAnalysisResults.NoSolvableDyadFound &&
                     joints.Count(j => j.knownState != KnownState.Fully) > 0);
            if (posResult == PositionAnalysisResults.NoSolvableDyadFound)
                return NDPS.Run_PositionsAreClose(currentJointParams, currentLinkParams, oldJointParams,
                                                  oldLinkParams);
            return true;
        }

        #region Dyad Solving Methods
        private point solveViaCircleIntersection(joint j, int jIndex, joint knownJoint1, joint knownJoint2, double[,] currentJointParams)
        {
            var kPIndex1 = joints.IndexOf(knownJoint1);
            var kPIndex2 = joints.IndexOf(knownJoint2);
            var r1 = j.Link1.lengthBetween(j, knownJoint1);
            var ptA = new point(currentJointParams[kPIndex1, 0],
                                currentJointParams[kPIndex1, 1]);
            var r2 = j.Link2.lengthBetween(j, knownJoint2);
            var ptB = new point(currentJointParams[kPIndex2, 0],
                                currentJointParams[kPIndex2, 1]);
            var numPt = new point(currentJointParams[jIndex, 0],
                              currentJointParams[jIndex, 1]);
            /* taken from http://2000clicks.com/MathHelp/GeometryConicSectionCircleIntersection.aspx */
            if (Constants.sameCloseZero(r1)) return ptA;
            if (Constants.sameCloseZero(r2)) return ptB;

            var dSquared = (ptA.X - ptB.X) * (ptA.X - ptB.X) + (ptA.Y - ptB.Y) * (ptA.Y - ptB.Y);
            var ratio = (r1 * r1 - r2 * r2) / (2 * dSquared);
            var xBase = (ptA.X + ptB.X) / 2 + (ptB.X - ptA.X) * ratio;
            var yBase = (ptA.Y + ptB.Y) / 2 + (ptB.Y - ptA.Y) * ratio;
            var fourTimesKsquared = ((r1 + r2) * (r1 + r2) - dSquared) * (dSquared - (r1 - r2) * (r1 - r2));

            if (Constants.sameCloseZero(fourTimesKsquared)) return new point(xBase, yBase);
            if (fourTimesKsquared < 0) return new point(double.NaN, double.NaN);

            var K = Math.Sqrt(fourTimesKsquared) / 4;
            var xOffset = 2 * (ptB.Y - ptA.Y) * K / dSquared;
            var yOffset = 2 * (ptA.X - ptB.X) * K / dSquared;
            var xPos = xBase + xOffset;
            var yPos = yBase + yOffset;
            var xNeg = xBase - xOffset;
            var yNeg = yBase - yOffset;
            var distPosSquared = Constants.distanceSqared(xPos, yPos, numPt.X, numPt.Y);
            var distNegSquared = Constants.distanceSqared(xNeg, yNeg, numPt.X, numPt.Y);
            if (distNegSquared < distPosSquared)
                return new point(xNeg, yNeg);
            return new point(xPos, yPos);
        }

        private point solveViaCircleAndLineIntersection(joint j, int jIndex, joint circleCenterJoint, joint lineJoint,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, out double angleChange)
        {
            var circleLink = (j.Link1.joints.Contains(circleCenterJoint)) ? j.Link1 : j.Link2;
            var slideLink = (j.Link1 != circleLink) ? j.Link1 : j.Link2;
            var r1 = circleLink.lengthBetween(j, circleCenterJoint);
            int circCenterIndex = joints.IndexOf(circleCenterJoint);
            int lineJointIndex = joints.IndexOf(lineJoint);
            angleChange = double.NaN;
            var ptA = new point(currentJointParams[circCenterIndex, 0], currentJointParams[circCenterIndex, 1]);
            double slopeB;
            var ptB = defineParallelLineThroughJoint(jIndex, lineJointIndex, slideLink, currentJointParams,
                currentLinkParams, out slopeB);

            var ptC = solveViaIntersectingLines(-1 / slopeB, ptA, slopeB, ptB);
            var distToChord = Constants.distance(ptA, ptC);
            if (distToChord > r1) return new point(double.NaN, double.NaN);
            if (Constants.sameCloseZero(distToChord, r1)) return ptC;
            var alpha = Math.Acos(distToChord / r1);
            var thetaBase = Constants.angle(ptA, ptC);
            var numX = currentJointParams[jIndex, 0];
            var numY = currentJointParams[jIndex, 1];
            var thetaNeg = thetaBase - alpha;
            var xNeg = ptA.X + r1 * Math.Cos(thetaNeg);
            var yNeg = ptA.Y + r1 * Math.Sin(thetaNeg);
            var distNegSquared = Constants.distanceSqared(numX, numY, xNeg, yNeg);

            var thetaPos = thetaBase + alpha;
            var xPos = ptA.X + r1 * Math.Cos(thetaPos);
            var yPos = ptA.Y + r1 * Math.Sin(thetaPos);
            var distPosSquared = Constants.distanceSqared(numX, numY, xPos, yPos);
            var oldTheta = Constants.angle(oldJointParams[circCenterIndex, 0], oldJointParams[circCenterIndex, 1], oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
            if (distNegSquared < distPosSquared)
            {
                angleChange = thetaNeg - oldTheta;
                return new point(xNeg, yNeg);
            }
            angleChange = thetaPos - oldTheta;
            return new point(xPos, yPos);
        }


        #region Methods for P-?-P
        private point solveViaIntersectingLines(joint j, int jIndex, joint knownJointA, joint knownJointB,
            double[,] currentJointParams, double[,] currentLinkParams)
        {
            double slopeA, slopeB;
            var ptA = defineParallelLineThroughJoint(jIndex, joints.IndexOf(knownJointA), j.Link1, currentJointParams,
                currentLinkParams, out slopeA);
            var ptB = defineParallelLineThroughJoint(jIndex, joints.IndexOf(knownJointB), j.Link2, currentJointParams,
                currentLinkParams, out slopeB);
            return solveViaIntersectingLines(slopeA, ptA, slopeB, ptB);
        }
        private point solveViaIntersectingLines(double slopeA, point ptA, double slopeB, point ptB)
        {
            if (Constants.sameCloseZero(ptA.X, ptB.X) && Constants.sameCloseZero(ptA.Y, ptB.Y)) return ptA;
            if (Constants.sameCloseZero(slopeA, slopeB)) return new point(double.NaN, double.NaN);
            var offsetA = ptA.Y - slopeA * ptA.X;
            var offsetB = ptB.Y - slopeB * ptB.X;
            if (double.IsNaN(slopeA) || double.IsInfinity(slopeA))
                return new point(ptA.X, slopeB * ptA.X + offsetB);
            if (double.IsNaN(slopeB) || double.IsInfinity(slopeB))
                return new point(ptB.X, slopeA * ptB.X + offsetA);

            var x = (offsetB - offsetA) / (slopeA - slopeB);
            var y = slopeA * x + offsetA;
            return new point(x, y);
        }
        private point defineParallelLineThroughJoint(int positionJointIndex, int slopeJointIndex,
          link thisLink, double[,] currentJointParams, double[,] currentLinkParams, out double slope)
        {
            var thetaA = slideAngle(joints[slopeJointIndex], currentLinkParams);
            slope = Math.Tan(thetaA);
            Boolean higherOffset;
            Constants.findOrthoPoint(currentJointParams[positionJointIndex, 0], currentJointParams[positionJointIndex, 1],
                currentJointParams[slopeJointIndex, 0], currentJointParams[slopeJointIndex, 1], thetaA, out higherOffset);
            var lineRef = new point(currentJointParams[slopeJointIndex, 0], currentJointParams[slopeJointIndex, 1]);
            var distance = thisLink.lengthBetween(joints[positionJointIndex], joints[slopeJointIndex]);
            int plusOrMinus = (higherOffset) ? +1 : -1;
            lineRef.X = lineRef.X + distance * Math.Cos(thetaA + plusOrMinus * Math.PI / 2);
            lineRef.Y = lineRef.Y + distance * Math.Sin(thetaA + plusOrMinus * Math.PI / 2);
            return lineRef;
        }
        #endregion


        // the basis of R-P-R dyad determination method is the complex little function
        private point solveRPRIntersection(joint j, int jIndex, joint knownJoint1, joint knownJoint2, double[,] currentJointParams,
            double[,] oldJointParams, double[,] oldLinkParams, out double angleChange)
        {
            var kPIndex1 = joints.IndexOf(knownJoint1);
            var kPIndex2 = joints.IndexOf(knownJoint2);
            var ptB = new point(currentJointParams[kPIndex1, 0],
                                        currentJointParams[kPIndex1, 1]);
            var ptA = new point(currentJointParams[kPIndex2, 0],
                                        currentJointParams[kPIndex2, 1]);
            var rAC = j.Link2.lengthBetween(j, knownJoint2);
            var rBC = j.Link1.lengthBetween(j, knownJoint1);

            double oldTheta = Constants.angle(oldJointParams[kPIndex2, 0],
                                              oldJointParams[kPIndex2, 1],
                                              oldJointParams[jIndex, 0],
                                              oldJointParams[jIndex, 1]);
            var alpha = angleOfBlockToJoint(jIndex, kPIndex2,
                                            oldJointParams, oldLinkParams);
            var numPt = new point(currentJointParams[jIndex, 0],
                                  currentJointParams[jIndex, 1]);
            angleChange = double.NaN;
            var lAB = Constants.distance(ptA, ptB);
            var phi = Constants.angle(ptA, ptB);
            if ((rBC + rAC * Math.Sin(alpha)) > lAB) return new point(double.NaN, double.NaN);
            /* first, the internal positive case */
            var beta = Math.Asin((rBC + rAC * Math.Sin(alpha)) / lAB);
            var thetaIntPos = Math.PI - alpha - beta + phi;
            var xIntPos = ptA.X + rAC * Math.Cos(thetaIntPos);
            var yIntPos = ptA.Y + rAC * Math.Sin(thetaIntPos);
            var distthetaIntPosSquared = Constants.distanceSqared(xIntPos, yIntPos, numPt.X, numPt.Y);

            /* second, the internal negative case */
            var thetaIntNeg = beta + phi - alpha;
            var xIntNeg = ptA.X + rAC * Math.Cos(thetaIntNeg);
            var yIntNeg = ptA.Y + rAC * Math.Sin(thetaIntNeg);
            var distthetaIntNegSquared = Constants.distanceSqared(xIntNeg, yIntNeg, numPt.X, numPt.Y);

            /* first, the External positive case */
            beta = Math.Asin((rBC - rAC * Math.Sin(alpha)) / lAB);
            var thetaExtPos = Math.PI - alpha + beta + phi;
            var xExtPos = ptA.X + rAC * Math.Cos(thetaExtPos);
            var yExtPos = ptA.Y + rAC * Math.Sin(thetaExtPos);
            var distthetaExtPosSquared = Constants.distanceSqared(xExtPos, yExtPos, numPt.X, numPt.Y);

            /* second, the External negative case */
            var thetaExtNeg = phi - beta - alpha;
            var xExtNeg = ptA.X + rAC * Math.Cos(thetaExtNeg);
            var yExtNeg = ptA.Y + rAC * Math.Sin(thetaExtNeg);
            var distthetaExtNegSquared = Constants.distanceSqared(xExtNeg, yExtNeg, numPt.X, numPt.Y);

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



        private point solveViaSlopeToCircleIntersectionPPR(joint j, int jIndex, int slideIndex, int circCenterIndex,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams,
             out double angleChange)
        {
            /* in this case, the block is on the rotating link and the slide is on the sliding link */
            var numPtX = currentJointParams[jIndex, 0];
            var numPtY = currentJointParams[jIndex, 1];
            var circCenterPt = new point(currentJointParams[circCenterIndex, 0], currentJointParams[circCenterIndex, 0]);
            var radius = j.Link2.lengthBetween(j, joints[circCenterIndex]);
            var slideAngleC = slideAngle(j, currentLinkParams);
            var alpha = angleOfBlockToJoint(jIndex, circCenterIndex, oldJointParams, oldLinkParams);
            var thetaNeg = slideAngleC - alpha;
            var xNeg = circCenterPt.X + radius * Math.Cos(thetaNeg);
            var yNeg = circCenterPt.Y + radius * Math.Sin(thetaNeg);
            var distNegSquared = Constants.distanceSqared(xNeg, yNeg, numPtX, numPtY);

            var thetaPos = (thetaNeg < 0) ? thetaNeg + Math.PI : thetaNeg - Math.PI;
            var xPos = circCenterPt.X + radius * Math.Cos(thetaPos);
            var yPos = circCenterPt.Y + radius * Math.Sin(thetaPos);
            var distPosSquared = Constants.distanceSqared(xPos, yPos, numPtX, numPtY);

            var oldTheta = Constants.angle(oldJointParams[slideIndex, 0], oldJointParams[slideIndex, 1],
                oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
            if (distNegSquared < distPosSquared)
            {
                angleChange = thetaNeg - oldTheta;
                return new point(xNeg, yNeg);
            }
            angleChange = thetaPos - oldTheta;
            return new point(xPos, yPos);
        }

        private point solveViaSlopeToCircleIntersectionRPP(joint j, int jIndex, int circCenterIndex, int slideIndex,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams,
             out double angleChange)
        { /* in this case, the slide is on the rotating link and the block is on the sliding link */
            var numPtX = currentJointParams[jIndex, 0];
            var numPtY = currentJointParams[jIndex, 1];
            var circCenterPt = new point(currentJointParams[circCenterIndex, 0], currentJointParams[circCenterIndex, 0]);
            var rAC = j.Link2.lengthBetween(j, joints[circCenterIndex]);
            double slopeB;
            var ptB = defineParallelLineThroughJoint(jIndex, slideIndex, j.Link2, currentJointParams,
                currentLinkParams, out slopeB);
            // need to find proper link1 angle and thus slideAngle for goal, 
            // this will set up the line that goes through the point
            var alpha = angleOfBlockToJoint(jIndex, slideIndex, oldJointParams, oldLinkParams);
            var actualSlideAngle = slideAngle(joints[slideIndex], currentLinkParams) + alpha;
            var thetaNeg = actualSlideAngle + Math.PI / 2;
            var orthoPt = new point(circCenterPt.X + rAC * Math.Cos(thetaNeg), circCenterPt.Y + rAC * Math.Sin(thetaNeg));
            var slopeA = Math.Tan(thetaNeg);
            var ptNeg = solveViaIntersectingLines(slopeA, orthoPt, slopeB, ptB);
            var distNegSquared = Constants.distanceSqared(ptNeg.X, ptNeg.Y, numPtX, numPtY);

            var thetaPos = thetaNeg + Math.PI;
            orthoPt = new point(circCenterPt.X + rAC * Math.Cos(thetaPos), circCenterPt.Y + rAC * Math.Sin(thetaPos));
            slopeA = Math.Tan(thetaPos);
            var ptPos = solveViaIntersectingLines(slopeA, orthoPt, slopeB, ptB);
            var distPosSquared = Constants.distanceSqared(ptPos.X, ptPos.Y, numPtX, numPtY);

            var oldTheta = Constants.angle(oldJointParams[circCenterIndex, 0], oldJointParams[circCenterIndex, 1],
                oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
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
        private point solveGearAngleAndPos_R_G_RP(joint j, double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams,
            double[,] oldLinkParams,
            out double angleChange)
        {
            var gData = gearsData[j];
            //if (knownJoint1 == gData.gearCenter1)
            //{
            angleChange = Constants.angle(currentJointParams[gData.gearCenter1Index, 0],
                                          currentJointParams[gData.gearCenter1Index, 1],
                                          currentJointParams[gData.gearCenter2Index, 0],
                                          currentJointParams[gData.gearCenter2Index, 1]);
            angleChange -= Constants.angle(oldJointParams[gData.gearCenter1Index, 0],
                                           oldJointParams[gData.gearCenter1Index, 1],
                                           oldJointParams[gData.gearCenter2Index, 0],
                                           oldJointParams[gData.gearCenter2Index, 1]);
            if (gData.gearCenter1.jointType != JointTypes.P)
            {
                var g2LinkIndex = links.IndexOf(j.Link2);
                var g2AngleChange = currentLinkParams[g2LinkIndex, 0] -
                                    oldLinkParams[g2LinkIndex, 0];
                angleChange -= gData.radius2 * g2AngleChange / gData.radius1;
            }
            var ratio = gData.radius1 / (gData.radius1 + gData.radius2);
            var x = currentJointParams[gData.gearCenter1Index, 0];
            x += ratio * (currentJointParams[gData.gearCenter1Index, 0] - x);
            var y = currentJointParams[gData.gearCenter1Index, 1];
            y += ratio * (currentJointParams[gData.gearCenter1Index, 1] - 1);
            return new point(x, y);
        }

        private point solveRotatePinToSlot(joint j, int jIndex, int circleCenterIndex, double[,] currentJointParams, double[,] oldJointParams,
            double[,] currentLinkParams, out double angleChange)
        {
            var circleLink = j.Link2;
            var slideLink = j.Link1;
            var circleCenterJoint = joints[circleCenterIndex];
            var r1 = circleLink.lengthBetween(j, circleCenterJoint);
            var ptA = new point(currentJointParams[circleCenterIndex, 0], currentJointParams[circleCenterIndex, 1]);
            double slopeB;
            var ptB = defineParallelLineThroughJoint(jIndex, jIndex, slideLink, currentJointParams,
                currentLinkParams, out slopeB);

            var ptC = solveViaIntersectingLines(-1 / slopeB, ptA, slopeB, ptB);
            var distToChord = Constants.distance(ptA, ptC);
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
            var thetaBase = Constants.angle(ptA, ptC);
            var numPt = new point(currentJointParams[jIndex, 0], currentJointParams[jIndex, 1]);
            var thetaNeg = thetaBase - alpha;
            var ptNeg = new point(ptA.X + r1 * Math.Cos(thetaNeg), ptA.Y + r1 * Math.Sin(thetaNeg));
            var distNegSquared = Constants.distanceSqared(numPt, ptNeg);

            var thetaPos = thetaBase + alpha;
            var ptPos = new point(ptA.X + r1 * Math.Cos(thetaPos), ptA.Y + r1 * Math.Sin(thetaPos));
            var distPosSquared = Constants.distanceSqared(numPt, ptPos);
            var oldTheta = Constants.angle(oldJointParams[circleCenterIndex, 0], oldJointParams[circleCenterIndex, 1], oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
            if (distNegSquared < distPosSquared)
            {
                angleChange = thetaNeg - oldTheta;
                return ptNeg;
            }
            angleChange = thetaPos - oldTheta;
            return ptPos;
        }

        private double solveRotateSlotToPin(int fixedJointIndex, int slideJointKnownIndex, int linkIndex, double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams)
        {
            Boolean higherOffset;
            var thetaA = slideAngle(joints[slideJointKnownIndex], currentLinkParams);
            Constants.findOrthoPoint(currentJointParams[fixedJointIndex, 0], currentJointParams[fixedJointIndex, 1],
                currentJointParams[slideJointKnownIndex, 0], currentJointParams[slideJointKnownIndex, 1], thetaA, out higherOffset);
            var distanceBetweenJoints = Constants.distance(currentJointParams[fixedJointIndex, 0], currentJointParams[fixedJointIndex, 1],
                currentJointParams[slideJointKnownIndex, 0], currentJointParams[slideJointKnownIndex, 1]);
            var oldDistance = Constants.distance(oldJointParams[fixedJointIndex, 0], oldJointParams[fixedJointIndex, 1],
                oldJointParams[slideJointKnownIndex, 0], oldJointParams[slideJointKnownIndex, 1]);
            var dist2Slide = links[linkIndex].lengthBetween(joints[fixedJointIndex], joints[slideJointKnownIndex]);
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
        private void assignJointPosition(int jIndex, link thisLink, double[,] currentJointParams, point ptNew)
        {
            if (double.IsInfinity(ptNew.X) || double.IsInfinity(ptNew.Y) ||
                double.IsNaN(ptNew.X) || double.IsNaN(ptNew.Y))
                posResult = PositionAnalysisResults.InvalidPosition;
            else
            {
                var j = joints[jIndex];
                j.knownState = j.SlidingWithRespectTo(thisLink) ? KnownState.Partially : KnownState.Fully;
                var X = ptNew.X;
                var Y = ptNew.Y;
                ptNew.X -= currentJointParams[jIndex, 0];
                ptNew.Y -= currentJointParams[jIndex, 1];
                positionError += ptNew.X * ptNew.X + ptNew.Y * ptNew.Y;
                currentJointParams[jIndex, 0] = X;
                currentJointParams[jIndex, 1] = Y;
                posResult = PositionAnalysisResults.Normal;
            }
        }


        private void setLinkPositionFromRotate(joint knownJoint, int knownIndex, int linkIndex,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams,
            double[,] oldLinkParams, double angleChange = double.NaN)
        {
            var thisLink = links[linkIndex];
            if (thisLink.AngleIsKnown) return;
            var oldAngle = oldLinkParams[linkIndex, 0];
            if (double.IsNaN(angleChange))
            {
                var otherKnownPosition = thisLink.joints.FirstOrDefault(j => j != knownJoint && j.knownState != KnownState.Unknown);
                if (otherKnownPosition == null ||
                    (knownJoint.SlidingWithRespectTo(thisLink) && otherKnownPosition.SlidingWithRespectTo(thisLink)))
                    return;
                var otherKnownIndex = joints.IndexOf(otherKnownPosition);
                var new_j2j_Angle = Constants.angle(currentJointParams[knownIndex, 0], currentJointParams[knownIndex, 1],
                                                    currentJointParams[otherKnownIndex, 0],
                                                    currentJointParams[otherKnownIndex, 1]);
                var old_j2j_Angle = Constants.angle(oldJointParams[knownIndex, 0], oldJointParams[knownIndex, 1],
                                                    oldJointParams[otherKnownIndex, 0], oldJointParams[otherKnownIndex, 1]);
                angleChange = new_j2j_Angle - old_j2j_Angle;

                if (knownJoint.FixedWithRespectTo(thisLink) && otherKnownPosition.SlidingWithRespectTo(thisLink))
                    angleChange += solveRotateSlotToPin(knownIndex, otherKnownIndex, linkIndex, currentJointParams, oldJointParams,
                                        currentLinkParams, oldLinkParams);
                else if (knownJoint.SlidingWithRespectTo(thisLink) && otherKnownPosition.FixedWithRespectTo(thisLink))
                    angleChange += solveRotateSlotToPin(otherKnownIndex, knownIndex, linkIndex, currentJointParams, oldJointParams,
                                        currentLinkParams, oldLinkParams);
            }

            currentLinkParams[linkIndex, 0] = oldAngle + angleChange;
            thisLink.AngleIsKnown = true;
            //if (knownJoint.knownState != KnownState.Fully) return;
            //if (knownJoint.SlidingWithRespectTo(thisLink)) return;
            foreach (var j in thisLink.joints.Where(j => j != knownJoint && j.knownState != KnownState.Fully))
            {
                var jIndex = joints.IndexOf(j);
                var length = thisLink.lengthBetween(j, knownJoint);
                var angle = Constants.angle(oldJointParams[knownIndex, 0], oldJointParams[knownIndex, 1],
                                             oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
                angle += angleChange;
                assignJointPosition(jIndex, thisLink, currentJointParams,
                    new point(currentJointParams[knownIndex, 0] + length * Math.Cos(angle),
                    currentJointParams[knownIndex, 1] + length * Math.Sin(angle)));
                var otherLink = j.OtherLink(thisLink);
                if (j.FixedWithRespectTo(thisLink) && otherLink != null)
                    setLinkPositionFromRotate(j, jIndex, links.IndexOf(otherLink),
                                               currentJointParams, oldJointParams, currentLinkParams, oldLinkParams,
                                               (j.jointType == JointTypes.P) ? angleChange : double.NaN);
            }
        }

        private void setLinkPositionFromTranslation(joint knownJoint, int knownIndex, int linkIndex,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams,
            double[,] oldLinkParams, double deltaX, double deltaY, double angle = double.NaN)
        {
            var thisLink = links[linkIndex];

            foreach (var j in thisLink.joints.Where(j => j != knownJoint && j.knownState != KnownState.Fully))
            {
                var jIndex = joints.IndexOf(j);
                if (double.IsNaN(angle))
                    assignJointPosition(jIndex, thisLink, currentJointParams, new point(oldJointParams[jIndex, 0] + deltaX,
                                                                                        oldJointParams[jIndex, 1] + deltaY));
                else
                    assignJointPosition(jIndex, thisLink, currentJointParams,
                        new point(oldJointParams[jIndex, 0] + deltaX * Math.Cos(angle),
                        oldJointParams[jIndex, 1] + deltaY * Math.Sin(angle)));
                if (!j.FixedWithRespectTo(thisLink)) continue;
                if (j.jointType == JointTypes.P)
                    setLinkPositionFromTranslation(j, jIndex, links.IndexOf(j.OtherLink(thisLink)),
                                                        currentJointParams, oldJointParams, currentLinkParams, oldLinkParams,
                                                        deltaX, deltaY, slideAngle(j, currentLinkParams));
                else setLinkPositionFromRotate(j, jIndex, links.IndexOf(j.OtherLink(thisLink)),
                                                  currentJointParams, oldJointParams, currentLinkParams,
                                                  oldLinkParams);
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
        private double slideAngle(joint j, double[,] linkParams)
        {
            var result = j.SlideAngle + linkParams[links.IndexOf(j.Link1), 0];
            while (result < -Math.PI / 2) result += Math.PI;
            while (result > Math.PI / 2) result -= Math.PI;
            return result;
        }
        private double angleOfBlockToJoint(int blockJoint, int referenceJoint, double[,] jointParams, double[,] linkParams)
        {
            var theta = Constants.angle(jointParams[blockJoint, 0], jointParams[blockJoint, 1], jointParams[referenceJoint, 0], jointParams[referenceJoint, 1]);
            var result = slideAngle(joints[blockJoint], linkParams) - theta;
            while (result < 0) result += Math.PI;
            while (result > Math.PI) result -= Math.PI;
            return result;
        }
    }
}