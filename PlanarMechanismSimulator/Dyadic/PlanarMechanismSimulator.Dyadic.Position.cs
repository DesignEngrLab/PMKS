using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        private Boolean AnalyticallyCorrectPositionsDyadic(double[,] currentJointParams, double[,] currentLinkParams, double[,] oldJointParams, double[,] oldLinkParams)
        {
            joint knownJoint1 = null;
            joint knownJoint2 = null;
            var knownPositions = new List<joint>(joints.GetRange(firstInputJointIndex, numJoints - firstInputJointIndex));
            var unknownPositions = new List<joint>(joints.GetRange(0, firstInputJointIndex).RemoveAll(j => j.Link2 == null));
            var knownLinkAngles = new List<link> { links[inputLinkIndex], links[inputLinkIndex + 1] };
            // add any links to knownLinkAngles which are P to input or ground (don't forget to update currentLinkParams
            while (unknownPositions.Count > 0)
            {
                #region Link1 has a known joint position and both links have known angles
                var solvableJoint = unknownPositions.FirstOrDefault(j => j.Link1.joints.Any(jj => knownPositions.Contains(jj))
                      && knownLinkAngles.Contains(j.Link1) && knownLinkAngles.Contains(j.Link2));
                if (solvableJoint != null)
                { }
                #endregion
                #region Link2 has a known joint position and both links have known angles
                solvableJoint = unknownPositions.FirstOrDefault(j => j.Link2.joints.Any(jj => knownPositions.Contains(jj))
                    && knownLinkAngles.Contains(j.Link1) && knownLinkAngles.Contains(j.Link2));
                if (solvableJoint != null)
                { }
                #endregion
                #region both links have known angles
                solvableJoint = unknownPositions.FirstOrDefault(j => knownLinkAngles.Contains(j.Link1) && knownLinkAngles.Contains(j.Link2));
                if (solvableJoint != null)
                { }
                #endregion
                #region Link1 has a known joint position and Link2 is at a known angle
                solvableJoint = unknownPositions.FirstOrDefault(j => j.Link1.joints.Any(jj => knownPositions.Contains(jj)) && knownLinkAngles.Contains(j.Link2));
                if (solvableJoint != null)
                { }
                #endregion
                #region Link2 has a known joint position and Link1 is at a known angle
                solvableJoint = unknownPositions.FirstOrDefault(j => j.Link2.joints.Any(jj => knownPositions.Contains(jj)) && knownLinkAngles.Contains(j.Link1));
                if (solvableJoint != null)
                { }
                #endregion
                #region Link1 and Link2 have known joint positions
                solvableJoint = unknownPositions.FirstOrDefault(j => FindKnownPositionOnLink(j.Link1, knownPositions, out knownJoint1)
                    && FindKnownPositionOnLink(j.Link2, knownPositions, out knownJoint2));
                if (solvableJoint != null)
                {
                    var sJIndex = joints.IndexOf(solvableJoint);
                    var kPIndex1 = joints.IndexOf(knownJoint1);
                    var kPIndex2 = joints.IndexOf(knownJoint2);
                    var knownPoint1 = new point(currentJointParams[kPIndex1, 0], currentJointParams[kPIndex1, 1]);
                    var knownPoint2 = new point(currentJointParams[kPIndex2, 0], currentJointParams[kPIndex2, 2]);
                    List<joint> newKnownJointsFromLink1 = null;
                    List<joint> newKnownJointsFromLink2 = null;
                    List<link> newKnownLinkAnglesFromLink1 = null;
                    List<link> newKnownLinkAnglesFromLink2 = null;
                    double angleChange;
                    switch (solvableJoint.jointType)
                    {
                        case JointTypes.RP:
                        case JointTypes.G:
                            break;
                        case JointTypes.R:
                            var sJPoint = solveViaCircleIntersection(solvableJoint.Link1.lengthBetween(solvableJoint, knownJoint1), knownPoint1,
                                solvableJoint.Link2.lengthBetween(solvableJoint, knownJoint2), knownPoint2, new point(currentJointParams[sJIndex, 0], currentJointParams[sJIndex, 1]));
                            if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) || double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                return false;
                            currentJointParams[sJIndex, 0] = sJPoint.X;
                            currentJointParams[sJIndex, 1] = sJPoint.Y;
                            angleChange = solveAngleChange(sJPoint, sJIndex, knownPoint1, kPIndex1, oldJointParams);
                            newKnownJointsFromLink1 = setLinkPosition(knownJoint1, kPIndex1, links.IndexOf(solvableJoint.Link1),
                                                                      currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                            newKnownLinkAnglesFromLink1 = setLinkAngles(angleChange, solvableJoint.Link1, newKnownJointsFromLink1, currentLinkParams, oldLinkParams);
                            angleChange = solveAngleChange(sJPoint, sJIndex, knownPoint2, kPIndex2, oldJointParams);
                            newKnownJointsFromLink2 = setLinkPosition(knownJoint2, kPIndex2, links.IndexOf(solvableJoint.Link2),
                                                                      currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                            newKnownLinkAnglesFromLink2 = setLinkAngles(angleChange, solvableJoint.Link2, newKnownJointsFromLink2, currentLinkParams, oldLinkParams);
                            goto default;
                        case JointTypes.P:
                            double rAC = solvableJoint.Link2.lengthBetween(solvableJoint, knownJoint2);
                            angleChange = solveRPRIntersection(knownPoint2, rAC, knownPoint1, solvableJoint.Link1.lengthBetween(solvableJoint, knownJoint1),
                                new point(oldJointParams[sJIndex, 0], oldJointParams[sJIndex, 1]), new point(oldJointParams[kPIndex2, 0], oldJointParams[kPIndex2, 1]),
                                oldLinkParams[links.IndexOf(solvableJoint.Link1), 0] + solvableJoint.SlideAngle);
                            currentJointParams[sJIndex, 0] = rAC * Math.Cos(oldLinkParams[links.IndexOf(solvableJoint.Link2), 0] + angleChange);
                            currentJointParams[sJIndex, 1] = rAC * Math.Sin(oldLinkParams[links.IndexOf(solvableJoint.Link2), 0] + angleChange);
                            newKnownJointsFromLink2 = setLinkPosition(knownJoint2, kPIndex2, links.IndexOf(solvableJoint.Link2),
                                                                      currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                            newKnownLinkAnglesFromLink2 = setLinkAngles(angleChange, solvableJoint.Link2, newKnownJointsFromLink2, currentLinkParams, oldLinkParams);
                            newKnownJointsFromLink1 = setLinkPosition(knownJoint1, kPIndex1, links.IndexOf(solvableJoint.Link1),
                                                                      currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, angleChange);
                            newKnownLinkAnglesFromLink1 = setLinkAngles(angleChange, solvableJoint.Link1, newKnownJointsFromLink1, currentLinkParams, oldLinkParams);
                            goto default;
                        default:
                            knownPositions.Add(solvableJoint);
                            knownPositions.AddRange(newKnownJointsFromLink1);
                            knownPositions.AddRange(newKnownJointsFromLink2);
                            unknownPositions.Remove(solvableJoint);
                            unknownPositions.RemoveAll(newKnownJointsFromLink1.Contains);
                            unknownPositions.RemoveAll(newKnownJointsFromLink2.Contains);
                            knownLinkAngles.Add(solvableJoint.Link1);
                            knownLinkAngles.Add(solvableJoint.Link2);
                            knownLinkAngles.AddRange(newKnownLinkAnglesFromLink1);
                            knownLinkAngles.AddRange(newKnownLinkAnglesFromLink2);
                            break;
                    }
                }
                #endregion
                else return NDPS.Run_PositionsAreClose(currentJointParams, currentLinkParams, oldJointParams, oldLinkParams);

            } while (unknownPositions.Count > 0) ;
            return true;
        }

        private double solveRPRIntersection(point ptA, double rAC, point ptB, double rBC, point oldPtC, point oldPtA, double oldSlideAngle)
        {
            var lAB = Math.Sqrt((ptB.X - ptA.X) * (ptB.X - ptA.X) + (ptB.Y - ptA.Y) * (ptB.Y - ptA.Y));
            var phi = Math.Atan2(ptB.Y - ptA.Y, ptB.X - ptA.X);
            var oldTheta = Math.Atan2(oldPtC.Y - oldPtA.Y, oldPtC.X - oldPtA.X);
            var alpha = Math.PI - oldTheta - oldSlideAngle;
            return Math.PI - alpha - phi - Math.Asin((rAC * Math.Sin(alpha) + rAC) / lAB);
        }

        #region Methods for R-P-R and R-R-R
        private double solveAngleChange(point point1, int pt1Index, point point2, int pt2Index, double[,] oldJointParams)
        {
            var newAngle = Math.Atan2(point1.Y - point2.Y, point1.X - point2.X);
            var oldAngle = Math.Atan2(oldJointParams[pt1Index, 1] - oldJointParams[pt2Index, 1], oldJointParams[pt1Index, 0] - oldJointParams[pt2Index, 0]);
            return newAngle - oldAngle;
        }


        private List<joint> setLinkPosition(joint knownJoint, int knownIndex, int linkIndex,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams, double deltaAngle)
        {
            var oldAngle = oldLinkParams[linkIndex, 0];
            currentLinkParams[linkIndex, 0] = oldAngle + deltaAngle;
            var newKnownJoints = new List<joint>();
            var thisLink = links[linkIndex];
            foreach (var j in thisLink.joints)
            {
                if (j == knownJoint || j.LinkIsSlide(thisLink)) continue;
                var jIndex = joints.IndexOf(j);
                newKnownJoints.Add(j);
                var length = thisLink.lengthBetween(j, knownJoint);
                oldAngle = Math.Atan2(oldJointParams[jIndex, 1] - oldJointParams[knownIndex, 1], oldJointParams[jIndex, 0] - oldJointParams[knownIndex, 0]);
                var newAngle = oldAngle + deltaAngle;
                currentJointParams[jIndex, 0] = currentJointParams[knownIndex, 0] + length * Math.Cos(newAngle);
                currentJointParams[jIndex, 1] = currentJointParams[knownIndex, 1] + length * Math.Sin(newAngle);
            }
            return newKnownJoints;
        }

        private List<link> setLinkAngles(double angleChange, link thisLink, IEnumerable<joint> knownJoints, double[,] currentLinkParams, double[,] oldLinkParams)
        {
            var newKnownLinkAngles = new List<link>();
            foreach (var j in knownJoints)
            {
                if (j.jointType != JointTypes.P) continue;
                var newLink = (j.Link1 == thisLink) ? j.Link2 : j.Link1;
                var newLinkIndex = links.IndexOf(newLink);
                currentLinkParams[newLinkIndex, 0] = oldLinkParams[newLinkIndex, 0] + angleChange;
                newKnownLinkAngles.Add(newLink);
            }
            return newKnownLinkAngles;
        }
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


        private bool FindKnownPositionOnLink(link link, List<joint> knownPositions, out joint knownJoint)
        {
            knownJoint = null;
            knownJoint = link.joints.Find(j => knownPositions.Contains(j) && !j.LinkIsSlide(link)); // and j is not a gear?
            if (knownJoint != null) return true;
            return false;
        }

    }
}