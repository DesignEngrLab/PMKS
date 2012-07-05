using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        private Boolean AnalyticallyCorrectPositionsDyadic(double delta, double[,] currentJointParams, double[,] currentLinkParams,
            double[,] oldJointParams, double[,] oldLinkParams)
        {
            for (int i = inputJointIndex + 1; i < numJoints; i++)
            {
                currentJointParams[i, 0] = oldJointParams[i, 0];
                currentJointParams[i, 1] = oldJointParams[i, 1];
            }
            var unknownPositions = new List<joint>(joints);
            unknownPositions.RemoveAll(j => j.Link2 == null);
            setLinkPosition(inputpivot, inputJointIndex, inputLinkIndex, unknownPositions, currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, delta);
            setLinkPosition(joints[inputJointIndex+1], inputJointIndex+1, inputLinkIndex+1, unknownPositions, 
                currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, 0.0);

            var unknownLinkAngles = new List<link>(links);
            setLinkAngles(0.0, groundLink, unknownLinkAngles, currentLinkParams, oldLinkParams);
            setLinkAngles(delta, inputLink, unknownLinkAngles, currentLinkParams, oldLinkParams);
            // add any links to !unknownLinkAngles which are P to input or ground (don't forget to update currentLinkParams
            int initUnkCount;
            do
            {
                var k = initUnkCount = unknownPositions.Count;
                while (k > 0)
                {
                    var j = unknownPositions[--k];
                    var jIndex = joints.IndexOf(j);
                    joint knownJoint1 = null;
                    joint knownJoint2 = null;
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
                                setLinkPosition(knownJoint1, kPIndex1, links.IndexOf(j.Link1), unknownPositions,
                                                                          currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams);
                                angleChange = solveAngleChange(sJPoint, jIndex, knownPoint2, kPIndex2, oldJointParams);
                                setLinkPosition(knownJoint2, kPIndex2, links.IndexOf(j.Link2), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link2, unknownLinkAngles, currentLinkParams, oldLinkParams);
                                break;
                            }
                            #endregion
                            #region R-R-P
                            else if (FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1) && !unknownLinkAngles.Contains(j.Link2))
                            {
                                
                            }
                            #endregion
                            #region P-R-R
                            else if (!unknownLinkAngles.Contains(j.Link1) && FindKnownPositionOnLink(j.Link2, unknownPositions, out knownJoint2))
                            {
                                
                            }
                            #endregion
                            #region P-R-P
                            else if (FindKnownSlopeOnLink(j.Link1, unknownPositions, unknownLinkAngles, out knownJoint1)
                                && FindKnownSlopeOnLink(j.Link2, unknownPositions, unknownLinkAngles, out knownJoint2))
                            {
                                var kPIndex1 = joints.IndexOf(knownJoint1);
                                var kPIndex2 = joints.IndexOf(knownJoint2);
                                var knownPoint1 = new point(currentJointParams[kPIndex1, 0], currentJointParams[kPIndex1, 1]);
                                var knownPoint2 = new point(currentJointParams[kPIndex2, 0], currentJointParams[kPIndex2, 1]);
                                var sJPoint = solveViaIntersectingLines(currentSlideAngle(knownJoint1, currentLinkParams), knownPoint1,
                                   currentSlideAngle(knownJoint2, currentLinkParams), knownPoint2);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) || double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPosition(knownJoint1, kPIndex1, links.IndexOf(j.Link1), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, 0.0);
                                setLinkAngles(0.0, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams);
                                setLinkPosition(knownJoint2, kPIndex2, links.IndexOf(j.Link2), unknownPositions,
                                                                         currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, 0.0);
                                setLinkAngles(0.0, j.Link2, unknownLinkAngles, currentLinkParams, oldLinkParams);
                                
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
                                double oldTheta = Math.Atan2(oldJointParams[jIndex, 1] - oldJointParams[kPIndex2, 1], oldJointParams[jIndex, 0] - oldJointParams[kPIndex2, 0]);
                                double angleChange;
                                var sJPoint = solveRPRIntersection(knownPoint2, rAC, knownPoint1, j.Link1.lengthBetween(j, knownJoint1), oldTheta,
                                       oldLinkParams[links.IndexOf(j.Link1), 0] + j.SlideAngle, new point(currentJointParams[jIndex, 0], currentJointParams[jIndex, 1]),
                                       out angleChange);
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPosition(knownJoint2, kPIndex2, links.IndexOf(j.Link2), unknownPositions,
                                                                          currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link2, unknownLinkAngles, currentLinkParams, oldLinkParams);
                                setLinkPosition(knownJoint1, kPIndex1, links.IndexOf(j.Link1), unknownPositions,
                                                                          currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                                setLinkAngles(angleChange, j.Link1, unknownLinkAngles, currentLinkParams, oldLinkParams);
                                
                            }
                            #endregion
                            #region R-P-P
                            else if (FindKnownPositionOnLink(j.Link1, unknownPositions, out knownJoint1) && !unknownLinkAngles.Contains(j.Link2))
                            {
                                
                            }
                            #endregion
                            #region P-P-R
                            else if (!unknownLinkAngles.Contains(j.Link1) && FindKnownPositionOnLink(j.Link2, unknownPositions, out knownJoint2))
                            {
                                
                            }
                            #endregion
                            #region P-P-P
                            else if (!unknownLinkAngles.Contains(j.Link1) && !unknownLinkAngles.Contains(j.Link2))
                            {
                                
                            }
                            #endregion
                             break;

                        case JointTypes.RP:
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
            } while (unknownPositions.Count > 0 || initUnkCount == unknownPositions.Count);
            if (initUnkCount == unknownPositions.Count)
                return NDPS.Run_PositionsAreClose(currentJointParams, currentLinkParams, oldJointParams, oldLinkParams);
            else return true;
        }

        #region Methods for P-?-P
        private double currentSlideAngle(joint j, double[,] currentLinkParams)
        {
            return Math.Tan(j.SlideAngle + currentLinkParams[links.IndexOf(j.Link1), 0]);
        }
        private point solveViaIntersectingLines(double slopeA, point ptA, double slopeB, point ptB)
        {
            if (double.IsNaN(slopeA))
                return new point(ptA.X, slopeB * ptA.X + (ptB.Y - slopeB * ptB.X));
            if (double.IsNaN(slopeB))
                return new point(ptB.X, slopeA * ptB.X + (ptA.Y - slopeA * ptA.X));
            if (Constants.sameCloseZero(ptA.X, ptB.X) && Constants.sameCloseZero(ptA.Y, ptB.Y)) return ptA;
            if (Constants.sameCloseZero(slopeA, slopeB)) return new point(double.NaN, double.NaN);
            var offsetA = ptA.Y - slopeA * ptA.X;
            var offsetB = ptB.Y - slopeB * ptB.X;
            var y = (offsetB - offsetA) / (slopeA - slopeB);
            var x = (y - offsetA) / slopeA;
            return new point(x, y);
        }
        #endregion

        #region Methods for R-P-R and R-R-R
        private double solveAngleChange(point point1, int pt1Index, point point2, int pt2Index, double[,] oldJointParams)
        {
            var newAngle = Math.Atan2(point1.Y - point2.Y, point1.X - point2.X);
            var oldAngle = Math.Atan2(oldJointParams[pt1Index, 1] - oldJointParams[pt2Index, 1], oldJointParams[pt1Index, 0] - oldJointParams[pt2Index, 0]);
            return newAngle - oldAngle;
        }

        #region the basis of R-P-R dyad determination method is the complex little function
        private point solveRPRIntersection(point ptA, double rAC, point ptB, double rBC, 
            double oldTheta, double oldSlideAngle, point numPt, out double angleChange)
        {
            var lAB = Math.Sqrt((ptB.X - ptA.X) * (ptB.X - ptA.X) + (ptB.Y - ptA.Y) * (ptB.Y - ptA.Y));
            var phi = Math.Atan2(ptB.Y - ptA.Y, ptB.X - ptA.X);
            var alpha = oldTheta + oldSlideAngle;
            var thetaNeg = alpha - phi - Math.Asin((rBC + rAC * Math.Sin(alpha)) / lAB);
            var xNeg = ptA.X + rAC * Math.Cos(thetaNeg);
            var yNeg = ptA.Y + rAC * Math.Sin(thetaNeg);
            var distNegSquared = (xNeg - numPt.X) * (xNeg - numPt.X) + (yNeg - numPt.Y) * (yNeg - numPt.Y);

            alpha = oldTheta - oldSlideAngle;
            var thetaPos = alpha + phi + Math.PI / 2 - Math.Asin((rBC - rAC * Math.Sin(alpha)) / lAB);
            var xPos = ptA.X + rAC * Math.Cos(thetaPos);
            var yPos = ptA.Y + rAC * Math.Sin(thetaPos);
            var distPosSquared = (xPos - numPt.X) * (xPos - numPt.X) + (yPos - numPt.Y) * (yPos - numPt.Y);
            if (distNegSquared < distPosSquared)
            {
                angleChange = thetaNeg - oldTheta;
                return new point(xNeg, yNeg);
            }
            angleChange = thetaPos - oldTheta;
            return new point(xPos, yPos);
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
            var distPosSquared = (xPos - numPt.X) * (xPos - numPt.X) + (yPos - numPt.Y) * (yPos - numPt.Y);
            var distNegSquared = (xNeg - numPt.X) * (xNeg - numPt.X) + (yNeg - numPt.Y) * (yNeg - numPt.Y);
            if (distNegSquared < distPosSquared)
                return new point(xNeg, yNeg);
            return new point(xPos, yPos);
        }
        #endregion

        #endregion

        private void setLinkPosition(joint knownJoint, int knownIndex, int linkIndex, List<joint> unknownPositions,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams, double delta)
        {
            var thisLink = links[linkIndex];
            if (knownJoint.jointType == JointTypes.R)
            {
                var oldAngle = oldLinkParams[linkIndex, 0];
                currentLinkParams[linkIndex, 0] = oldAngle + delta;

                foreach (var j in thisLink.joints)
                    if (unknownPositions.Contains(j) || !j.LinkIsSlide(thisLink))
                    {
                        unknownPositions.Remove(j);
                        if (j == knownJoint) continue;
                        var jIndex = joints.IndexOf(j);
                        var length = thisLink.lengthBetween(j, knownJoint);
                        oldAngle = Math.Atan2(oldJointParams[jIndex, 1] - oldJointParams[knownIndex, 1], oldJointParams[jIndex, 0] - oldJointParams[knownIndex, 0]);
                        var newAngle = oldAngle + delta;
                        currentJointParams[jIndex, 0] = currentJointParams[knownIndex, 0] + length * Math.Cos(newAngle);
                        currentJointParams[jIndex, 1] = currentJointParams[knownIndex, 1] + length * Math.Sin(newAngle);
                    }

            }
            else if (knownJoint.jointType == JointTypes.P)
            {
                currentLinkParams[inputLinkIndex, 0] = oldLinkParams[inputLinkIndex, 0];
                /* the block input does not rotate therefore the angle, angular velocity, and angular accelerations are all zero. */
                var angle = currentLinkParams[links.IndexOf(inputpivot.Link1), 0] + inputpivot.SlideAngle;
                var xDelta = delta * Math.Cos(angle);
                var yDelta = delta * Math.Sin(angle);
                for (int i = firstInputJointIndex; i <= inputJointIndex; i++)
                {
                    currentJointParams[i, 0] = oldJointParams[i, 0] + xDelta;
                    currentJointParams[i, 1] = oldJointParams[i, 1] + yDelta;
                }
            }
            else throw new Exception("A G-joint or a RP-joint should not have been sent to setLinkPosition.");
        }

        private void setLinkAngles(double angleChange, link thisLink, List<link> unknownLinkAngles,
            double[,] currentLinkParams, double[,] oldLinkParams)
        {
            unknownLinkAngles.Remove(thisLink);
            foreach (var j in thisLink.joints)
            {
                if (j.jointType != JointTypes.P) continue;
                var newLink = (j.Link1 == thisLink) ? j.Link2 : j.Link1;
                var newLinkIndex = links.IndexOf(newLink);
                currentLinkParams[newLinkIndex, 0] = oldLinkParams[newLinkIndex, 0] + angleChange;
                unknownLinkAngles.Remove(newLink);
            }
        }

        private bool FindKnownPositionOnLink(link link, List<joint> unknownPositions, out joint knownJoint)
        {
            knownJoint = null;
            knownJoint = link.joints.Find(j => !unknownPositions.Contains(j) && j.jointType == JointTypes.R);
            if (knownJoint != null) return true;
            return false;
        }
        private bool FindKnownSlopeOnLink(link link, List<joint> unknownPositions, List<link> unknownLinkAngles, out joint knownJoint)
        {
            knownJoint = null;
            if (unknownLinkAngles.Contains(link)) return false;
            knownJoint = link.joints.Find(j => !unknownPositions.Contains(j) && j.jointType == JointTypes.P);
            if (knownJoint != null) return true;
            return false;
        }

    }
}