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
            var fixedGndJointIndex = -1;
            for (int i = 0; i < numJoints; i++)
            {
                if (i >= inputJointIndex && joints[i].FixedWithRespectToLink(groundLink))
                {
                    fixedGndJointIndex = i;
                    joints[i].PositionIsUnknown = joints[i].SlideLineIsUnknown = false;
                }
                else joints[i].PositionIsUnknown = joints[i].SlideLineIsUnknown = true;
            }
            for (int i = 0; i < numLinks; i++) links[i].AngleIsUnknown = true;

            setLinkPositionFromRotate(joints[fixedGndJointIndex], fixedGndJointIndex, inputLinkIndex + 1,
                currentJointParams, oldJointParams, currentLinkParams, oldLinkParams);
            setLinkAngles(0.0, groundLink, inputLinkIndex + 1, currentLinkParams, oldLinkParams, inputpivot);
            if (inputpivot.jointType == JointTypes.R)
                setLinkPositionFromRotate(inputpivot, inputJointIndex, inputLinkIndex,
                    currentJointParams, oldJointParams, currentLinkParams, oldLinkParams, delta);
            else if (inputpivot.jointType == JointTypes.P)
                setLinkPositionFromSlideTranslation(inputLinkIndex, currentJointParams, oldJointParams,
                        delta, delta, slideAngle(inputpivot, currentLinkParams));
            else throw new Exception("Input is not of type R or P (as currently required at the beginning of DefineNewPositions");
            int initUnkCount = joints.Count(j => j.SlideLineIsUnknown || j.PositionIsUnknown) +
                               links.Count(c => c.AngleIsUnknown);
            Boolean nonDyadic = false;
            while (initUnkCount > 0)
            {
                for (int jIndex = 0; jIndex < numJoints; jIndex++)
                {
                    if (!joints[jIndex].PositionIsUnknown || joints[jIndex].Link2 == null) continue;
                    var j = joints[jIndex];
                    joint knownJoint1;
                    joint knownJoint2;
                    switch (j.jointType)
                    {
                        case JointTypes.R:
                            #region R-R-R
                            if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                     FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                var kPIndex1 = joints.IndexOf(knownJoint1);
                                var kPIndex2 = joints.IndexOf(knownJoint2);
                                var knownPoint1 = new point(currentJointParams[kPIndex1, 0],
                                                            currentJointParams[kPIndex1, 1]);
                                var knownPoint2 = new point(currentJointParams[kPIndex2, 0],
                                                            currentJointParams[kPIndex2, 1]);
                                var sJPoint = solveViaCircleIntersection(j.Link1.lengthBetween(j, knownJoint1),
                                                                         knownPoint1,
                                                                         j.Link2.lengthBetween(j, knownJoint2),
                                                                         knownPoint2,
                                                                         new point(currentJointParams[jIndex, 0],
                                                                                   currentJointParams[jIndex, 1]));
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) ||
                                    double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                var angleChange = solveAngleChange(sJPoint, jIndex, knownPoint1, kPIndex1,
                                                                   oldJointParams);
                                setLinkPositionFromRotate(knownJoint1, kPIndex1, links.IndexOf(j.Link1),

                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
                                angleChange = solveAngleChange(sJPoint, jIndex, knownPoint2, kPIndex2,
                                                               oldJointParams);
                                setLinkPositionFromRotate(knownJoint2, kPIndex2, links.IndexOf(j.Link2),

                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
                            }

                            #endregion
                            #region R-R-P

                            else if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                int kPIndex1 = joints.IndexOf(knownJoint1);
                                double angleChange;
                                var sJPoint = solveViaCircleAndLineIntersection(j, jIndex,
                                                                                j.Link1.lengthBetween(j, knownJoint1),
                                                                                knownJoint1, knownJoint2,
                                                                                currentJointParams, oldJointParams,
                                                                                currentLinkParams, out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) ||
                                    double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                var deltaX = sJPoint.X - oldJointParams[jIndex, 0];
                                var deltaY = sJPoint.Y - oldJointParams[jIndex, 1];
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint1, kPIndex1, links.IndexOf(j.Link1),

                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
                                setLinkPositionFromFixedTranslation(links.IndexOf(j.Link2),
                                                                    currentJointParams, oldJointParams, deltaX, deltaY);
                            }
                            #endregion
                            #region P-R-R

                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                                     && FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                int kPIndex2 = joints.IndexOf(knownJoint2);
                                double angleChange;
                                var sJPoint = solveViaCircleAndLineIntersection(j, jIndex,
                                                                                j.Link2.lengthBetween(j, knownJoint2),
                                                                                knownJoint2, knownJoint1,
                                                                                currentJointParams, oldJointParams,
                                                                                currentLinkParams, out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) ||
                                    double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                var deltaX = sJPoint.X - oldJointParams[jIndex, 0];
                                var deltaY = sJPoint.Y - oldJointParams[jIndex, 1];
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint2, kPIndex2, links.IndexOf(j.Link2),

                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
                                setLinkPositionFromFixedTranslation(links.IndexOf(j.Link1),
                                                                    currentJointParams, oldJointParams, deltaX, deltaY);
                            }
                            #endregion
                            #region P-R-P

                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                              && FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaIntersectingLines(j, knownJoint1, knownJoint2, currentJointParams,
                                                                        currentLinkParams);
                                var deltaX = sJPoint.X - oldJointParams[jIndex, 0];
                                var deltaY = sJPoint.Y - oldJointParams[jIndex, 1];
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromFixedTranslation(links.IndexOf(j.Link1),
                                                                    currentJointParams, oldJointParams, deltaX, deltaY);
                                setLinkPositionFromFixedTranslation(links.IndexOf(j.Link2),
                                                                    currentJointParams, oldJointParams, deltaX, deltaY);
                            }
                            #endregion
                            break;
                        case JointTypes.P:
                            #region R-P-R
                            if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                    FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                var kPIndex1 = joints.IndexOf(knownJoint1);
                                var kPIndex2 = joints.IndexOf(knownJoint2);
                                var knownPoint1 = new point(currentJointParams[kPIndex1, 0],
                                                            currentJointParams[kPIndex1, 1]);
                                var knownPoint2 = new point(currentJointParams[kPIndex2, 0],
                                                            currentJointParams[kPIndex2, 1]);
                                double rAC = j.Link2.lengthBetween(j, knownJoint2);
                                double oldTheta = Constants.angle(oldJointParams[kPIndex2, 0],
                                                                  oldJointParams[kPIndex2, 1],
                                                                  oldJointParams[jIndex, 0],
                                                                  oldJointParams[jIndex, 1]);
                                double angleChange;
                                var sJPoint = solveRPRIntersection(knownPoint2, rAC, knownPoint1,
                                                                   j.Link1.lengthBetween(j, knownJoint1), oldTheta,
                                                                   angleOfBlockToJoint(jIndex, kPIndex2,
                                                                                       oldJointParams, oldLinkParams),
                                                                   new point(currentJointParams[jIndex, 0],
                                                                             currentJointParams[jIndex, 1]),
                                                                   out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) ||
                                    double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint2, kPIndex2, links.IndexOf(j.Link2),

                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
                                setLinkPositionFromRotate(knownJoint1, kPIndex1, links.IndexOf(j.Link1),

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
                                double angleChange;
                                point sJPoint = solveViaSlopeToCircleIntersectionPPR(j, jIndex, kPIndex1,
                                                                                     currentJointParams, oldJointParams,
                                                                                     currentLinkParams,
                                                                                     oldLinkParams,
                                                                                     new point(
                                                                                         currentJointParams[jIndex, 0],
                                                                                         currentJointParams[jIndex, 1]),
                                                                                     out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) ||
                                    double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                var deltaX = sJPoint.X - oldJointParams[jIndex, 0];
                                var deltaY = sJPoint.Y - oldJointParams[jIndex, 1];
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint2, kPIndex2, links.IndexOf(j.Link2),

                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
                                setLinkPositionFromSlideTranslation(links.IndexOf(j.Link1),
                                                                    currentJointParams, oldJointParams,
                                                                    deltaX, deltaY,
                                                                    slideAngle(knownJoint1, currentLinkParams));
                            }
                            #endregion
                            #region R-P-P

                            else if (FindKnownPositionOnLink(j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                /* in this case, the slide is on the rotating link and the block is on the sliding link */
                                var kPIndex1 = joints.IndexOf(knownJoint1);
                                var kPIndex2 = joints.IndexOf(knownJoint2);
                                double angleChange;
                                point sJPoint = solveViaSlopeToCircleIntersectionRPP(j, jIndex, kPIndex1, kPIndex2,
                                                                                     currentJointParams,
                                                                                     oldJointParams, currentLinkParams,
                                                                                     oldLinkParams,
                                                                                     new point(
                                                                                         currentJointParams[jIndex, 0],
                                                                                         currentJointParams[jIndex, 1]),
                                                                                     out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) ||
                                    double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                var deltaX = sJPoint.X - oldJointParams[jIndex, 0];
                                var deltaY = sJPoint.Y - oldJointParams[jIndex, 1];
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint1, kPIndex1, links.IndexOf(j.Link1),

                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
                                setLinkPositionFromFixedTranslation(links.IndexOf(j.Link2),
                                                                    currentJointParams, oldJointParams, deltaX, deltaY);
                            }
                            #endregion
                            #region P-P-P
                            else if (FindKnownSlopeOnLink(j.Link1, out knownJoint1) && FindKnownSlopeOnLink(j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaIntersectingLines(j, knownJoint1, knownJoint2, currentJointParams,
                                                                        currentLinkParams);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) ||
                                    double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                var deltaX = sJPoint.X - oldJointParams[jIndex, 0];
                                var deltaY = sJPoint.Y - oldJointParams[jIndex, 1];
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                if (
                                    !Constants.sameCloseZero(
                                        angleOfBlockToJoint(jIndex, joints.IndexOf(knownJoint1), oldJointParams,
                                                            oldLinkParams),
                                        angleOfBlockToJoint(jIndex, joints.IndexOf(knownJoint1), currentJointParams,
                                                            currentLinkParams)))
                                    return false;
                                setLinkPositionFromSlideTranslation(links.IndexOf(j.Link1),
                                                                    currentJointParams, oldJointParams,
                                                                    deltaX, deltaY,
                                                                    slideAngle(knownJoint1, currentLinkParams));
                                setLinkPositionFromFixedTranslation(links.IndexOf(j.Link2),
                                                                    currentJointParams, oldJointParams, deltaX, deltaY);
                            }
                            #endregion
                            break;
                        case JointTypes.RP:
                            #region R-RP-R&P

                            if (FindKnownPositionOnLink(j.Link1, out knownJoint1)
                                &&
                                FindKnownPositionAndSlopeOnLink(j.Link2,
                                                                out knownJoint2))
                            {
                                // whoa! this opens up a whole new element. The loop is not just about unknown positions but unknown link angles
                                // (this is likely to come up again for gears). The loop is iterating through unknown positions only, and the 
                                // convergence is also dependent on that. 
                                var kPIndex1 = joints.IndexOf(knownJoint1);
                                joints.IndexOf(knownJoint2);
                                var angleChange = solveRotateSlotToPin(j, jIndex, kPIndex1, currentJointParams,
                                                                       oldJointParams, currentLinkParams, oldLinkParams,
                                                                       new point(currentJointParams[jIndex, 0],
                                                                                 currentJointParams[jIndex, 1]));
                                if (double.IsNaN(angleChange)) return false;
                                setLinkPositionFromRotate(knownJoint1, kPIndex1, links.IndexOf(j.Link1),

                                                          currentJointParams, oldJointParams, currentLinkParams,
                                                          oldLinkParams, angleChange);
                            }

                            #endregion
                            #region R&P-RP-R

                            if (FindKnownPositionAndSlopeOnLink(j.Link1,
                                                                out knownJoint1)
                                && FindKnownPositionOnLink(j.Link2, out knownJoint2))
                            {
                                // circle to line intersection                               
                            }

                            #endregion
                            #region P-RP-R&P

                            if (FindKnownSlopeOnLink(j.Link1, out knownJoint1)
                                &&
                                FindKnownPositionAndSlopeOnLink(j.Link2,
                                                                out knownJoint2))
                            {
                                //tricky - just need to assign position to j.Link1 but there be nothing to assign position to on that link
                                // if both/all joints are slides.
                            }

                            #endregion
                            #region R&P-RP-P

                            if (FindKnownPositionAndSlopeOnLink(j.Link1,
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
                                double angleChange;
                                var sJPoint = solveGearAngleAndPos_R_G_RP(j, currentJointParams, oldJointParams,
                                                                                currentLinkParams, oldLinkParams, out angleChange);
                                if (double.IsInfinity(sJPoint.X) || double.IsInfinity(sJPoint.Y) ||
                                    double.IsNaN(sJPoint.X) || double.IsNaN(sJPoint.Y))
                                    return false;
                                //need to plan how to displace rack (use setLinkPositionFromSlideTranslation)
                                var deltaX = sJPoint.X - oldJointParams[jIndex, 0];
                                var deltaY = sJPoint.Y - oldJointParams[jIndex, 1];
                                currentJointParams[jIndex, 0] = sJPoint.X;
                                currentJointParams[jIndex, 1] = sJPoint.Y;
                                setLinkPositionFromRotate(knownJoint1, joints.IndexOf(knownJoint1), links.IndexOf(j.Link1),
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
                for (int cIndex = 0; cIndex < numLinks; cIndex++)
                {
                    if (links[cIndex].AngleIsUnknown)
                    {
                        var knownJoints =
                            links[cIndex].joints.Where(j => !j.PositionIsUnknown || !j.SlideLineIsUnknown).ToList();
                        if (knownJoints.Count<=1)continue;
                        
                    var j = joints[jIndex];

                }
                var newUnkCount = joints.Count(j => j.SlideLineIsUnknown || j.PositionIsUnknown) +
                                  links.Count(c => c.AngleIsUnknown);
                if (initUnkCount > newUnkCount)
                {
                    initUnkCount = newUnkCount;
                    continue;
                }
                nonDyadic = true;
                break;
            }
            if (nonDyadic)
                return NDPS.Run_PositionsAreClose(currentJointParams, currentLinkParams, oldJointParams, oldLinkParams);
            return true;
        }

        private point solveGearAngleAndPos_R_G_RP(joint j, double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams,
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

        private double solveRotateSlotToPin(joint j, int jIndex, int kpIndex, double[,] currentJointParams, double[,] oldJointParams,
            double[,] currentLinkParams, double[,] oldLinkParams, point jPoint)
        {
            var ptA = new point(currentJointParams[kpIndex, 0], currentJointParams[kpIndex, 1]);
            var dist = Constants.distance(ptA, jPoint);
            var distToSlide = j.Link1.lengthBetween(j, joints[kpIndex]);
            if (dist < distToSlide) return double.NaN;
            if (Constants.sameCloseZero(dist, distToSlide)) return 0.0;

            var alpha = Math.Acos(distToSlide / dist);
            var thetaBase = Constants.angle(ptA, jPoint);
            var oldTheta = Constants.angle(oldJointParams[kpIndex, 0], oldJointParams[kpIndex, 1], oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
            var linkIndex = links.IndexOf(j.Link1);
            var numDeltaTheta = currentLinkParams[linkIndex, 0] - oldLinkParams[linkIndex, 0];
            var thetaNegChange = thetaBase - alpha - oldTheta;
            var thetaPosChange = thetaBase + alpha - oldTheta;

            return (Math.Abs(thetaNegChange - numDeltaTheta) < Math.Abs(thetaPosChange - numDeltaTheta)) ? thetaNegChange : thetaPosChange;
        }

        #region Methods for P-?-P
        private double slideAngle(joint j, double[,] linkParams)
        {
            var result = j.SlideAngle + linkParams[links.IndexOf(j.Link1), 0];
            while (result < -Math.PI / 2) result += Math.PI;
            while (result > Math.PI / 2) result -= Math.PI;
            return result;
        }
        private point solveViaIntersectingLines(joint j, joint knownJointA, joint knownJointB,
            double[,] currentJointParams, double[,] currentLinkParams)
        {
            double slopeA, slopeB;
            var ptA = defineParallelLineThroughJoint(j, knownJointA, joints.IndexOf(knownJointA), currentJointParams,
                currentLinkParams, out slopeA);
            var ptB = defineParallelLineThroughJoint(j, knownJointB, joints.IndexOf(knownJointB), currentJointParams,
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
        private point defineParallelLineThroughJoint(joint positionJoint, joint slopeJoint, int slopeJointIndex,
            double[,] currentJointParams, double[,] currentLinkParams, out double slope)
        {
            var slopePoint = link.findOrthoPoint(slopeJoint, positionJoint);
            var angleDiff = Math.Atan2(positionJoint.initY, positionJoint.initX) - Math.Atan2(slopePoint.Y, slopePoint.X);
            while (angleDiff < -Math.PI / 2) angleDiff += Math.PI;
            while (angleDiff > Math.PI / 2) angleDiff -= Math.PI;
            var plusOrMinus = Math.Sign(angleDiff);
            var distance = Constants.distance(slopePoint, positionJoint);

            var ptA = new point(currentJointParams[slopeJointIndex, 0], currentJointParams[slopeJointIndex, 1]);
            var thetaA = slideAngle(slopeJoint, currentLinkParams);
            slope = Math.Tan(thetaA);
            ptA.X = ptA.X + distance * Math.Cos(thetaA + plusOrMinus * Math.PI / 2);
            ptA.Y = ptA.Y + distance * Math.Sin(thetaA + plusOrMinus * Math.PI / 2);
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

        private point solveViaCircleAndLineIntersection(joint j, int jIndex, double r1, joint circleCenterJoint, joint lineJoint,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, out double angleChange)
        {
            int circCenterIndex = joints.IndexOf(circleCenterJoint);
            int lineJointIndex = joints.IndexOf(lineJoint);
            angleChange = double.NaN;
            var ptA = new point(currentJointParams[circCenterIndex, 0], currentJointParams[circCenterIndex, 1]);
            double slopeB;
            var ptB = defineParallelLineThroughJoint(j, lineJoint, lineJointIndex, currentJointParams,
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
            double slopeB;
            var ptB = defineParallelLineThroughJoint(j, joints[kPIndex2], kPIndex2, currentJointParams,
                currentLinkParams, out slopeB);
            // need to find proper link1 angle and thus slideAngle for goal, 
            // this will set up the line that goes through the point
            var alpha = angleOfBlockToJoint(jIndex, kPIndex2, oldJointParams, oldLinkParams);
            var actualSlideAngle = slideAngle(joints[kPIndex2], currentLinkParams) + alpha;
            var thetaNeg = actualSlideAngle + Math.PI / 2;
            var orthoPt = new point(ptA.X + rAC * Math.Cos(thetaNeg), ptA.Y + rAC * Math.Sin(thetaNeg));
            var slopeA = Math.Tan(thetaNeg);
            var ptNeg = solveViaIntersectingLines(slopeA, orthoPt, slopeB, ptB);
            var distNegSquared = Constants.distanceSqared(ptNeg, numPt);

            var thetaPos = thetaNeg + Math.PI;
            orthoPt = new point(ptA.X + rAC * Math.Cos(thetaPos), ptA.Y + rAC * Math.Sin(thetaPos));
            slopeA = Math.Tan(thetaPos);
            var ptPos = solveViaIntersectingLines(slopeA, orthoPt, slopeB, ptB);
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

        #region set & find link position and angles
        private void setLinkPositionFromRotate(joint knownJoint, int knownIndex, int linkIndex,
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams, double delta = 0.0)
        {
            var thisLink = links[linkIndex];

            foreach (var j in thisLink.joints.Where(j => j != knownJoint && j.PositionIsUnknown))
            {
                if (j.SlidingWithRespectToLink(thisLink))
                    j.SlideLineIsUnknown = false;
                else j.SlideLineIsUnknown = j.PositionIsUnknown = false;
                var jIndex = joints.IndexOf(j);
                var length = thisLink.lengthBetween(j, knownJoint);
                var angle = Constants.angle(oldJointParams[knownIndex, 0], oldJointParams[knownIndex, 1],
                                             oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
                angle += delta;
                currentJointParams[jIndex, 0] = currentJointParams[knownIndex, 0] + length * Math.Cos(angle);
                currentJointParams[jIndex, 1] = currentJointParams[knownIndex, 1] + length * Math.Sin(angle);
            }
            setLinkAngles(delta, thisLink, linkIndex, currentLinkParams, oldLinkParams, knownJoint);
        }
        private void setLinkAngles(double angleChange, link thisLink, int linkIndex,
            double[,] currentLinkParams, double[,] oldLinkParams, joint from = null)
        {
            var oldAngle = oldLinkParams[linkIndex, 0];
            currentLinkParams[linkIndex, 0] = oldAngle + angleChange;
            thisLink.AngleIsUnknown = false;
            foreach (var j in thisLink.joints)
            {
                if (j.jointType != JointTypes.P || j == from) continue;
                var newLink = j.OtherLink(thisLink);
                setLinkAngles(angleChange, newLink, links.IndexOf(newLink), currentLinkParams, oldLinkParams, j);
            }
        }
        private void setLinkPositionFromFixedTranslation(int linkIndex,
           double[,] currentJointParams, double[,] oldJointParams, double deltaX, double deltaY)
        {
            var thisLink = links[linkIndex];

            foreach (var j in thisLink.joints.Where(j => j.PositionIsUnknown))
            {
                if (j.SlidingWithRespectToLink(thisLink))
                    j.SlideLineIsUnknown = false;
                else j.SlideLineIsUnknown = j.PositionIsUnknown = false;
                var jIndex = joints.IndexOf(j);
                currentJointParams[jIndex, 0] = oldJointParams[jIndex, 0] + deltaX;
                currentJointParams[jIndex, 1] = oldJointParams[jIndex, 1] + deltaY;
            }
        }

        private void setLinkPositionFromSlideTranslation(int linkIndex,
           double[,] currentJointParams, double[,] oldJointParams, double deltaX, double deltaY, double angle)
        {
            var thisLink = links[linkIndex];

            foreach (var j in thisLink.joints.Where(j => j.PositionIsUnknown))
            {
                if (j.SlidingWithRespectToLink(thisLink))
                    j.SlideLineIsUnknown = false;
                else j.SlideLineIsUnknown = j.PositionIsUnknown = false;
                var jIndex = joints.IndexOf(j);
                currentJointParams[jIndex, 0] = oldJointParams[jIndex, 0] + deltaX * Math.Cos(angle);
                currentJointParams[jIndex, 1] = oldJointParams[jIndex, 1] + deltaY * Math.Sin(angle);
            }
        }


        private static bool FindKnownPositionAndSlopeOnLink(link link, out joint knownJoint)
        {
            knownJoint = null;
            if (link.AngleIsUnknown) return false;
            return FindKnownPositionOnLink(link, out knownJoint);
        }
        private static bool FindKnownPositionOnLink(link link, out joint knownJoint)
        {
            knownJoint = null;
            //knownJoint = link.joints.FirstOrDefault(j => !unknownPositions.Contains(j) && j.jointType == JointTypes.R && j.Link2 != null);
            knownJoint = link.joints.FirstOrDefault(j => !j.PositionIsUnknown);
            if (knownJoint != null) return true;
            return false;
        }
        private static bool FindKnownSlopeOnLink(link link, out joint knownJoint)
        {
            knownJoint = null;
            if (link.AngleIsUnknown) return false;
            //knownJoint = link.joints.FirstOrDefault(j => !unknownPositions.Contains(j) && j.jointType == JointTypes.P && j.Link2 != null);
            knownJoint = link.joints.FirstOrDefault(j => !j.SlideLineIsUnknown);
            if (knownJoint != null) return true;
            return false;
        }
        #endregion
    }
}