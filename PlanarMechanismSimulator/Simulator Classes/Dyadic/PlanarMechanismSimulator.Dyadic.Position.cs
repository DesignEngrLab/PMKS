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
        private Boolean DefineNewPositions(double delta, double[,] currentJointParams, double[,] currentLinkParams,
            double[,] oldJointParams, double[,] oldLinkParams)
        {
            for (int i = inputJointIndex + 1; i < numJoints; i++)
            {
                currentJointParams[i, 0] = oldJointParams[i, 0];
                currentJointParams[i, 1] = oldJointParams[i, 1];
            }
            var unknownPositions = new List<joint>(joints.GetRange(0, inputJointIndex + 1));
            unknownPositions.RemoveAll(j => j.Link2 == null);
            if (inputpivot.jointType == JointTypes.R)
                setLinkPositionFromRotate(inputpivot, inputJointIndex, inputLinkIndex, unknownPositions, currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, delta);
            else if (inputpivot.jointType == JointTypes.P)
                setLinkPositionFromSlide(inputpivot, inputJointIndex, inputLinkIndex, unknownPositions, currentJointParams, oldJointParams, currentLinkParams, oldLinkParams);
            else throw new Exception("Input is not of type R or P (as currently required at the beginning of DefineNewPositions");

            var unknownLinkAngles = new List<link>(links);
            setLinkAngles(0.0, groundLink, unknownLinkAngles, currentLinkParams, oldLinkParams, inputpivot);
            setLinkAngles(delta, inputLink, unknownLinkAngles, currentLinkParams, oldLinkParams, inputpivot);
            // add any links to !unknownLinkAngles which are P to input or ground (don't forget to update currentLinkParams
            int initUnkCount;
            do
            {
                var k = initUnkCount = unknownPositions.Count;
                while (k > 0)
                {
                    var j = unknownPositions[--k];
                    var jIndex = joints.IndexOf(j);
                    joint knownJoint1;
                    joint knownJoint2;
                    switch (j.jointType)
                    {
                        case JointTypes.R:
                            #region R-R-R
                            if (FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1) && FindKnownPositionOnLink(j.Link2, unknownPositions, out knownJoint2))
                            {
                                var kPIndex1 = joints.IndexOf(knownJoint1);
                                var kPIndex2 = joints.IndexOf(knownJoint2);
                                var knownPoint1 = new point(currentJointParams[kPIndex1, 0], currentJointParams[kPIndex1, 1]);
                                var knownPoint2 = new point(currentJointParams[kPIndex2, 0], currentJointParams[kPIndex2, 1]);
                                var sJPoint = solveViaCircleIntersection(j.Link1.lengthBetween(j, knownJoint1), knownPoint1,
                                    j.Link2.lengthBetween(j, knownJoint2), knownPoint2, new point(currentJointParams[jIndex, 0], currentJointParams[jIndex, 1]));
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) || double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                var angleChange = solveAngleChange(sJPoint, jIndex, knownPoint1, kPIndex1, oldJointParams);
                                setLinkPositionFromRotate(knownJoint1, kPIndex1, links.IndexOf(j.Link1), unknownPositions,
                                                                          currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                                angleChange = solveAngleChange(sJPoint, jIndex, knownPoint2, kPIndex2, oldJointParams);
                                setLinkPositionFromRotate(knownJoint2, kPIndex2, links.IndexOf(j.Link2), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link2, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                            }
                            #endregion
                            #region R-R-P
                            else if (FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1)
                                && FindKnownSlopeOnLink(j.Link2, unknownPositions, unknownLinkAngles, out knownJoint2))
                            {
                                int kPIndex1 = joints.IndexOf(knownJoint1);
                                double angleChange;
                                var sJPoint = solveViaCircleAndLineIntersection(j, jIndex, j.Link1.lengthBetween(j, knownJoint1), kPIndex1,
                                    knownJoint2, currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, new point(currentJointParams[jIndex, 0], currentJointParams[jIndex, 1]),
                                    out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) || double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint1, kPIndex1, links.IndexOf(j.Link1), unknownPositions,
                                    currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                                setLinkPositionFromSlide(j, jIndex, links.IndexOf(j.Link2), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams);
                                setLinkAngles(0.0, j.Link2, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                            }
                            #endregion
                            #region P-R-R
                            else if (FindKnownSlopeOnLink(j.Link1, unknownPositions, unknownLinkAngles, out knownJoint1)
                                && FindKnownPositionOnLink(j.Link2, unknownPositions, out knownJoint2))
                            {
                                int kPIndex2 = joints.IndexOf(knownJoint2);
                                double angleChange;
                                var sJPoint = solveViaCircleAndLineIntersection(j, jIndex, j.Link2.lengthBetween(j, knownJoint2), kPIndex2,
                                    knownJoint1, currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, new point(currentJointParams[jIndex, 0], currentJointParams[jIndex, 2]),
                                    out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) || double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint2, kPIndex2, links.IndexOf(j.Link2), unknownPositions,
                                    currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link2, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                                setLinkPositionFromSlide(j, jIndex, links.IndexOf(j.Link1), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams);
                                setLinkAngles(0.0, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                            }
                            #endregion
                            #region P-R-P
                            else if (FindKnownSlopeOnLink(j.Link1, unknownPositions, unknownLinkAngles, out knownJoint1)
                                && FindKnownSlopeOnLink(j.Link2, unknownPositions, unknownLinkAngles, out knownJoint2))
                            {
                                var sJPoint = solveViaIntersectingLines(j, jIndex, knownJoint1, knownJoint2, currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams);
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromSlide(j, jIndex, links.IndexOf(j.Link1), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams);
                                setLinkAngles(0.0, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                                setLinkPositionFromSlide(j, jIndex, links.IndexOf(j.Link2), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams);
                                setLinkAngles(0.0, j.Link2, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                            }
                            #endregion
                            break;

                        case JointTypes.P:
                            #region R-P-R
                            if (FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1) && FindKnownPositionOnLink(j.Link2, unknownPositions, out knownJoint2))
                            {
                                var kPIndex1 = joints.IndexOf(knownJoint1);
                                var kPIndex2 = joints.IndexOf(knownJoint2);
                                var knownPoint1 = new point(currentJointParams[kPIndex1, 0], currentJointParams[kPIndex1, 1]);
                                var knownPoint2 = new point(currentJointParams[kPIndex2, 0], currentJointParams[kPIndex2, 1]);
                                double rAC = j.Link2.lengthBetween(j, knownJoint2);
                                double oldTheta = Constants.angle(oldJointParams[kPIndex2, 0], oldJointParams[kPIndex2, 1], oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
                                double angleChange;
                                var sJPoint = solveRPRIntersection(knownPoint2, rAC, knownPoint1, j.Link1.lengthBetween(j, knownJoint1), oldTheta,
                                     angleOfBlockToJoint(jIndex, kPIndex2, oldJointParams, oldLinkParams), new point(currentJointParams[jIndex, 0], currentJointParams[jIndex, 1]),
                                       out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) || double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint2, kPIndex2, links.IndexOf(j.Link2), unknownPositions,
                                                                          currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link2, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                                setLinkPositionFromRotate(knownJoint1, kPIndex1, links.IndexOf(j.Link1), unknownPositions,
                                                                          currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                            }
                            #endregion
                            #region R-P-P
                            else if (FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1)
                                && FindKnownSlopeOnLink(j.Link2, unknownPositions, unknownLinkAngles, out knownJoint2))
                            { /* in this case, the slide is on the rotating link and the block is on the sliding link */
                                var kPIndex1 = joints.IndexOf(knownJoint1);
                                var kPIndex2 = joints.IndexOf(knownJoint2);
                                double angleChange;
                                point sJPoint = solveViaSlopeToCircleIntersectionRPP(j, jIndex, kPIndex1, kPIndex2, currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams, new point(currentJointParams[jIndex, 0], currentJointParams[jIndex, 1]),
                                    out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) || double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint1, kPIndex1, links.IndexOf(j.Link1), unknownPositions,
                                    currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                                setLinkPositionFromSlide(j, jIndex, links.IndexOf(j.Link2), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams);
                                setLinkAngles(0.0, j.Link2, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                            }
                            #endregion
                            #region P-P-R
                            else if (FindKnownPositionOnLink(j.Link2, unknownPositions, out knownJoint1)
                                && FindKnownSlopeOnLink(j.Link1, unknownPositions, unknownLinkAngles, out knownJoint2))
                            { /* in this case, the block is on the rotating link and the slide is on the sliding link */
                                var kPIndex1 = joints.IndexOf(knownJoint1);
                                var kPIndex2 = joints.IndexOf(knownJoint2);
                                double angleChange;
                                point sJPoint = solveViaSlopeToCircleIntersectionPPR(j, jIndex, kPIndex1, currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, new point(currentJointParams[jIndex, 0], currentJointParams[jIndex, 1]),
                                    out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) || double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint1, kPIndex1, links.IndexOf(j.Link2), unknownPositions,
                                    currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                                setLinkPositionFromSlide(knownJoint2, kPIndex2, links.IndexOf(j.Link1), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams);
                                setLinkAngles(0.0, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                            }
                            #endregion
                            #region P-P-P
                            else if (FindKnownSlopeOnLink(j.Link1, unknownPositions, unknownLinkAngles, out knownJoint1)
                                && FindKnownSlopeOnLink(j.Link2, unknownPositions, unknownLinkAngles, out knownJoint2))
                            {
                                var sJPoint = solveViaIntersectingLines(j, jIndex, knownJoint1, knownJoint2, currentJointParams,
                                    oldJointParams, currentLinkParams, oldLinkParams);
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                if (!Constants.sameCloseZero(angleOfBlockToJoint(jIndex, joints.IndexOf(knownJoint1), oldJointParams, oldLinkParams),
                                    angleOfBlockToJoint(jIndex, joints.IndexOf(knownJoint1), currentJointParams, currentLinkParams)))
                                    return false;
                                setLinkPositionFromSlide(j, jIndex, links.IndexOf(j.Link1), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, knownJoint1);
                                setLinkAngles(0.0, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                                setLinkPositionFromSlide(j, jIndex, links.IndexOf(j.Link2), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, knownJoint2);
                                setLinkAngles(0.0, j.Link2, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                            }
                            #endregion
                            break;

                        case JointTypes.RP:
                            #region R-RP-R&P
                            if (FindKnownPositionAndSlopeOnLink(j.Link2, unknownPositions, unknownLinkAngles)
                               && FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1))
                            {

                            }

                            #endregion
                            #region R&P-RP-R
                            if (FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1) && !unknownLinkAngles.Contains(j.Link1)
                                && FindKnownPositionOnLink(j.Link2, unknownPositions, out knownJoint2))
                            {

                            }
                            #endregion
                            #region P-RP-R&P
                            if (!unknownLinkAngles.Contains(j.Link1) && FindKnownPositionOnLink(j.Link2, unknownPositions, out knownJoint2) && !unknownLinkAngles.Contains(j.Link2))
                            {

                            }
                            #endregion
                            #region R&P-RP-P
                            if (FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1) && !unknownLinkAngles.Contains(j.Link1) && !unknownLinkAngles.Contains(j.Link2))
                            {

                            }
                            #endregion
                            break;

                        case JointTypes.G:
                            #region R-RP-R&P
                            if (FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1) && FindKnownPositionOnLink(j.Link2, unknownPositions, out knownJoint2)
                                && !unknownLinkAngles.Contains(j.Link2))
                            {

                            }

                            #endregion
                            #region R&P-RP-R
                            if (FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1) && !unknownLinkAngles.Contains(j.Link1)
                                && FindKnownPositionOnLink(j.Link2, unknownPositions, out knownJoint2))
                            {

                            }
                            #endregion
                            #region P-RP-R&P
                            if (!unknownLinkAngles.Contains(j.Link1) && FindKnownPositionOnLink(j.Link2, unknownPositions, out knownJoint2) && !unknownLinkAngles.Contains(j.Link2))
                            {

                            }
                            #endregion
                            #region R&P-RP-P
                            if (FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1) && !unknownLinkAngles.Contains(j.Link1) && !unknownLinkAngles.Contains(j.Link2))
                            {

                            }
                            #endregion
                            break;
                    }
                }
            } while (initUnkCount > 0 && (unknownPositions.Count > 0 || initUnkCount == unknownPositions.Count));
            if (initUnkCount > 0 && initUnkCount == unknownPositions.Count)
                return NDPS.Run_PositionsAreClose(currentJointParams, currentLinkParams, oldJointParams, oldLinkParams);
            return true;
        }

        #region Methods for P-?-P
        private double slideAngle(joint j, double[,] linkParams)
        {
            var result = j.SlideAngle + linkParams[links.IndexOf(j.Link1), 0];
            while (result < -Math.PI / 2) result += Math.PI;
            while (result > Math.PI / 2) result -= Math.PI;
            return result;
        }
        private point solveViaIntersectingLines(joint j, int jIndex, joint knownJointA, joint knownJointB,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams)
        {
            double slopeA, offsetA, slopeB, offsetB;
            var ptA = defineParallelLineThroughJoint(j, jIndex, knownJointA, currentJointParams, oldJointParams,
                currentLinkParams, oldLinkParams, out slopeA, out offsetA);
            var ptB = defineParallelLineThroughJoint(j, jIndex, knownJointB, currentJointParams, oldJointParams,
                currentLinkParams, oldLinkParams, out slopeB, out offsetB);
            return solveViaIntersectingLines(slopeA, offsetA, ptA, slopeB, offsetB, ptB);
        }
        private point solveViaIntersectingLines(double slopeA, double offsetA, point ptA, double slopeB, double offsetB, point ptB)
        {
            if (double.IsNaN(slopeA) || double.IsInfinity(slopeA))
                return new point(ptA.X, slopeB * ptA.X + (ptB.Y - slopeB * ptB.X));
            if (double.IsNaN(slopeB) || double.IsInfinity(slopeB))
                return new point(ptB.X, slopeA * ptB.X + (ptA.Y - slopeA * ptA.X));
            if (Constants.sameCloseZero(ptA.X, ptB.X) && Constants.sameCloseZero(ptA.Y, ptB.Y)) return ptA;
            if (Constants.sameCloseZero(slopeA, slopeB)) return new point(double.NaN, double.NaN);

            //var y = (offsetB - offsetA) / (slopeA - slopeB);
            //var x = (y - offsetA) / slopeA;
            var x = (offsetB - offsetA) / (slopeA - slopeB);
            var y = slopeA*x+offsetA;
            return new point(x, y);
        }
        private point defineParallelLineThroughJoint(joint j, int jIndex, joint knownJoint,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams, out double slope, out double offset)
        {
            var jPoint = new point(oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
            var kPIndex1 = joints.IndexOf(knownJoint);
            var thisLink = (j.Link1.joints.Contains(knownJoint)) ? j.Link1 : j.Link2;
            /* the next 8 lines of code is just to determine plusOrMinus - which side of the line is the point on? */
            var ptA = new point(oldJointParams[kPIndex1, 0], oldJointParams[kPIndex1, 1]);
            var thetaA = slideAngle(knownJoint, oldLinkParams);
            slope = Math.Tan(thetaA);
            offset = ptA.Y - slope * ptA.X;
            var dist1 = thisLink.lengthBetween(j, knownJoint);
            if (!knownJoint.LinkIsSlide(thisLink))
                dist1 *= Math.Sin(thetaA + Constants.angle(ptA, jPoint));
            var plusOrMinus = (double.IsNaN(slope) || double.IsInfinity(slope)) ? ((jPoint.X < 0) ? +1 : -1) : Math.Sign(jPoint.Y - slope * jPoint.X - offset);
            /* now solve the current slope, offset and known point */
            ptA = new point(currentJointParams[kPIndex1, 0], currentJointParams[kPIndex1, 1]);
            thetaA = slideAngle(knownJoint, currentLinkParams);
            slope = Math.Tan(thetaA);
            offset = ptA.Y - slope * ptA.X;
            ptA.X = ptA.X + dist1 * Math.Cos(thetaA + plusOrMinus * Math.PI / 2);
            ptA.Y = ptA.Y + dist1 * Math.Sin(thetaA + plusOrMinus * Math.PI / 2);
            offset = ptA.Y - slope * ptA.X;
            return ptA;
        }

        #endregion

        #region Methods for R-?-R
        private double solveAngleChange(point point1, int pt1Index, point point2, int pt2Index, double[,] oldJointParams)
        {
            var newAngle = Constants.angle(point2, point1);
            var oldAngle = Constants.angle(oldJointParams[pt2Index, 0], oldJointParams[pt2Index, 1], oldJointParams[pt1Index, 0], oldJointParams[pt1Index, 1]);
            return newAngle - oldAngle;
        }
        private double angleOfBlockToJoint(int blockJoint, int referenceJoint, double[,] jointParams, double[,] linkParams)
        {
            var theta = Constants.angle(jointParams[blockJoint, 0], jointParams[blockJoint, 1], jointParams[referenceJoint, 0], jointParams[referenceJoint, 1]);
            var result = slideAngle(joints[blockJoint], linkParams) - theta;
            while (result < 0) result += Math.PI;
            while (result > Math.PI) result -= Math.PI;
            return result;
        }

        #region the basis of R-P-R dyad determination method is the complex little function
        private point solveRPRIntersection(point ptA, double rAC, point ptB, double rBC,
            double oldTheta, double alpha, point numPt, out double angleChange)
        {
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
        #endregion

        #region the basis of R-R-R dyad determination method is the intersection of two circles
        private point solveViaCircleIntersection(double r1, point ptA, double r2, point ptB, point numPt)
        {
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
        #endregion


        #endregion

        #region Methods for R-?-P (and vice-versa)

        private point solveViaCircleAndLineIntersection(joint j, int jIndex, double r1, int kpIndex, joint knownJointB,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams,
            point numPt, out double angleChange)
        {
            angleChange = double.NaN;
            var ptA = new point(currentJointParams[kpIndex, 0], currentJointParams[kpIndex, 1]);
            double slopeB, offsetB;
            var ptB = defineParallelLineThroughJoint(j, jIndex, knownJointB, currentJointParams, oldJointParams,
                currentLinkParams, oldLinkParams, out slopeB, out offsetB);

            var ptC = solveViaIntersectingLines(-1 / slopeB, (ptA.Y + ptA.X / slopeB), ptA, slopeB, offsetB, ptB);
            var distToChord = Constants.distance(ptA, ptC);
            if (distToChord > r1) return new point(double.NaN, double.NaN);
            if (Constants.sameCloseZero(distToChord, r1)) return ptC;
            var alpha = Math.Acos(distToChord / r1);
            var thetaBase = Constants.angle(ptA, ptC);

            var thetaNeg = thetaBase - alpha;
            var xNeg = ptA.X + r1 * Math.Cos(thetaNeg);
            var yNeg = ptA.Y + r1 * Math.Sin(thetaNeg);
            var distNegSquared = Constants.distanceSqared(numPt.X, numPt.Y, xNeg, yNeg);

            var thetaPos = thetaBase + alpha;
            var xPos = ptA.X + r1 * Math.Cos(thetaPos);
            var yPos = ptA.Y + r1 * Math.Sin(thetaPos);
            var distPosSquared = Constants.distanceSqared(numPt.X, numPt.Y, xPos, yPos);
            var oldTheta = Constants.angle(oldJointParams[kpIndex, 0], oldJointParams[kpIndex, 1], oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
            if (distNegSquared < distPosSquared)
            {
                angleChange = thetaNeg - oldTheta;
                return new point(xNeg, yNeg);
            }
            angleChange = thetaPos - oldTheta;
            return new point(xPos, yPos);
        }


        private point solveViaSlopeToCircleIntersectionPPR(joint j, int jIndex, int kPIndex1,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams,
            point numPt, out double angleChange)
        {  /* in this case, the block is on the rotating link and the slide is on the sliding link */
            var ptA = new point(currentJointParams[kPIndex1, 0], currentJointParams[kPIndex1, 0]);
            var rAC = j.Link2.lengthBetween(j, joints[kPIndex1]);
            var slideAngleC = slideAngle(j, currentLinkParams);
            var alpha = angleOfBlockToJoint(jIndex, kPIndex1, oldJointParams, oldLinkParams);
            var thetaNeg = slideAngleC - alpha;
            var xNeg = ptA.X + rAC * Math.Cos(thetaNeg);
            var yNeg = ptA.Y + rAC * Math.Sin(thetaNeg);
            var distNegSquared = Constants.distanceSqared(xNeg, yNeg, numPt.X, numPt.Y);

            var thetaPos = thetaNeg + Math.PI;
            var xPos = ptA.X + rAC * Math.Cos(thetaPos);
            var yPos = ptA.Y + rAC * Math.Sin(thetaPos);
            var distPosSquared = Constants.distanceSqared(xPos, yPos, numPt.X, numPt.Y);

            var oldTheta = Constants.angle(oldJointParams[kPIndex1, 0], oldJointParams[kPIndex1, 1],
                oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
            if (distNegSquared < distPosSquared)
            {
                angleChange = thetaNeg - oldTheta;
                return new point(xNeg, yNeg);
            }
            angleChange = thetaPos - oldTheta;
            return new point(xPos, yPos);
        }

        private point solveViaSlopeToCircleIntersectionRPP(joint j, int jIndex, int kPIndex1, int kPIndex2,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams,
            point numPt, out double angleChange)
        { /* in this case, the slide is on the rotating link and the block is on the sliding link */
            var ptA = new point(currentJointParams[kPIndex1, 0], currentJointParams[kPIndex1, 0]);
            var rAC = j.Link2.lengthBetween(j, joints[kPIndex1]);
            double slopeB, offsetB;
            var ptB = defineParallelLineThroughJoint(j, jIndex, joints[kPIndex2], currentJointParams, oldJointParams,
                currentLinkParams, oldLinkParams, out slopeB, out offsetB);
            // need to find proper link1 angle and thus slideAngle for goal, 
            // this will set up the line that goes through the point
            var alpha = angleOfBlockToJoint(jIndex, kPIndex2, oldJointParams, oldLinkParams);
            var actualSlideAngle = slideAngle(joints[kPIndex2], currentLinkParams) + alpha;
            var thetaNeg = actualSlideAngle + Math.PI / 2;
            var orthoPt = new point(ptA.X + rAC * Math.Cos(thetaNeg), ptA.Y + rAC * Math.Sin(thetaNeg));
            var slopeA = Math.Tan(thetaNeg);
            var offsetA = orthoPt.Y - slopeA * orthoPt.X;
            var ptNeg = solveViaIntersectingLines(slopeA, offsetA, orthoPt, slopeB, offsetB, ptB);
            var distNegSquared = Constants.distanceSqared(ptNeg, numPt);

            var thetaPos = thetaNeg + Math.PI;
            orthoPt = new point(ptA.X + rAC * Math.Cos(thetaPos), ptA.Y + rAC * Math.Sin(thetaPos));
            slopeA = Math.Tan(thetaPos);
            offsetA = orthoPt.Y - slopeA * orthoPt.X;
            var ptPos = solveViaIntersectingLines(slopeA, offsetA, orthoPt, slopeB, offsetB, ptB);
            var distPosSquared = Constants.distanceSqared(ptPos, numPt);

            var oldTheta = Constants.angle(oldJointParams[kPIndex1, 0], oldJointParams[kPIndex1, 1],
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

        #region set link position and angles
        private void setLinkAngles(double angleChange, link thisLink, List<link> unknownLinkAngles,
            double[,] currentLinkParams, double[,] oldLinkParams, joint from)
        {
            unknownLinkAngles.Remove(thisLink);
            foreach (var j in thisLink.joints)
            {
                if (j.jointType != JointTypes.P || j == from) continue;
                var newLink = (j.Link1 == thisLink) ? j.Link2 : j.Link1;
                var newLinkIndex = links.IndexOf(newLink);
                currentLinkParams[newLinkIndex, 0] = oldLinkParams[newLinkIndex, 0] + angleChange;
                setLinkAngles(angleChange, newLink, unknownLinkAngles, currentLinkParams, oldLinkParams, j);
                unknownLinkAngles.Remove(newLink);
            }
        }
        private void setLinkPositionFromRotate(joint knownJoint, int knownIndex, int linkIndex, List<joint> unknownPositions,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams, double delta = 0.0)
        {
            var thisLink = links[linkIndex];
            var oldAngle = oldLinkParams[linkIndex, 0];
            currentLinkParams[linkIndex, 0] = oldAngle + delta;

            foreach (var j in thisLink.joints)
                if (unknownPositions.Contains(j) || !j.LinkIsSlide(thisLink))
                { /* so, it will re-enter this section if the P/RP is already known but this is updating the fixed-length side */
                    unknownPositions.Remove(j);
                    if (j == knownJoint) continue;
                    var jIndex = joints.IndexOf(j);
                    var length = thisLink.lengthBetween(j, knownJoint);
                    oldAngle = Constants.angle(oldJointParams[knownIndex, 0], oldJointParams[knownIndex, 1], oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
                    var newAngle = oldAngle + delta;
                    currentJointParams[jIndex, 0] = currentJointParams[knownIndex, 0] + length * Math.Cos(newAngle);
                    currentJointParams[jIndex, 1] = currentJointParams[knownIndex, 1] + length * Math.Sin(newAngle);
                }
        }
        private void setLinkPositionFromSlide(joint knownJoint, int knownIndex, int linkIndex, List<joint> unknownPositions,
           double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams, joint solvedJoint = null)
        {
            var thisLink = links[linkIndex];
            var deltaX = currentJointParams[knownIndex, 0] - oldJointParams[knownIndex, 0];
            var deltaY = currentJointParams[knownIndex, 1] - oldJointParams[knownIndex, 1];
            if (knownJoint.LinkIsSlide(links[linkIndex]))
            {
                foreach (var j in thisLink.joints)
                    if (unknownPositions.Contains(j) || !j.LinkIsSlide(thisLink))
                    {
                        unknownPositions.Remove(j);
                        if (j == knownJoint) continue;
                        var jIndex = joints.IndexOf(j);
                        var jPt = solveViaIntersectingLines(j, jIndex, knownJoint, solvedJoint, currentJointParams, oldJointParams, currentLinkParams, oldLinkParams);
                        currentJointParams[jIndex, 0] = jPt.X;
                        currentJointParams[jIndex, 1] = jPt.Y;
                    }
            }
            else
            {
                foreach (var j in thisLink.joints)
                    if (unknownPositions.Contains(j) || !j.LinkIsSlide(thisLink))
                    {
                        unknownPositions.Remove(j);
                        if (j == knownJoint) continue;
                        var jIndex = joints.IndexOf(j);
                        currentJointParams[jIndex, 0] = oldJointParams[jIndex, 0] + deltaX;
                        currentJointParams[jIndex, 1] = oldJointParams[jIndex, 1] + deltaY;
                    }
            }
        }


        #endregion

        private bool FindKnownPositionOnLink(link link, List<joint> unknownPositions, out joint knownJoint)
        {
            knownJoint = null;
            knownJoint = link.joints.FirstOrDefault(j => !unknownPositions.Contains(j) && j.jointType == JointTypes.R);
            if (knownJoint != null) return true;
            return false;
        }
        private bool FindKnownSlopeOnLink(link link, List<joint> unknownPositions, List<link> unknownLinkAngles, out joint knownJoint)
        {
            knownJoint = null;
            if (unknownLinkAngles.Contains(link)) return false;
            knownJoint = link.joints.FirstOrDefault(j => !unknownPositions.Contains(j) && j.jointType == JointTypes.P);
            if (knownJoint != null) return true;
            return false;
        }
        private bool FindKnownPositionAndSlopeOnLink(link link, List<joint> unknownPositions, List<link> unknownLinkAngles)
        {
            if (unknownLinkAngles.Contains(link)) return false;
            return link.joints.Any(j => !unknownPositions.Contains(j) &&
                                        (j.jointType == JointTypes.R));
        }

    }
}