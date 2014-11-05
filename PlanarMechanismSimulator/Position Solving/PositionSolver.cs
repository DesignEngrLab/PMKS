using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using PMKS;
using StarMathLib;

namespace PMKS.PositionSolving
{
    /// <summary>
    /// 
    /// </summary>
    internal class PositionFinder
    {
        public double PositionError
        {
            get { return _posError; }
            private set { if (_posError < value) _posError = value; }
        }

        private double _posError;
        private NonDyadicPositionSolver NDPS;

        public PositionAnalysisResults posResult { get; internal set; }
        private const double BranchRatio = 0.5;
        internal readonly List<joint> joints;
        internal readonly List<link> links;
        private readonly Dictionary<int, GearData> gearsData;
        private readonly int numLinks;
        private readonly link inputLink;
        private readonly link groundLink;
        private readonly joint inputJoint;
        private readonly double maximumDeltaX;
        private readonly double maximumDeltaY;
        private readonly double minimumDeltaX;
        private readonly double minimumDeltaY;

        public PositionFinder(List<joint> joints, List<link> links, Dictionary<int, GearData> gearsData,
            int inputJointIndex)
        {
            this.joints = joints;
            inputJoint = joints[inputJointIndex];
            this.links = links;
            numLinks = links.Count;
            inputLink = links[numLinks - 2];
            groundLink = links[numLinks - 1];
            this.gearsData = gearsData;
            /* this has been commented out because occasionally a mechanism can be defined in which all joints have the
             * same x or same y value. Imainge a quick return with all joints starting on the x-axis. */
            var xBounding = joints.Max(j => j.xInitial) - joints.Min(j => j.xInitial);
            var yBounding = joints.Max(j => j.yInitial) - joints.Min(j => j.yInitial);
            /* the maximum leads to some problems - even for our fair little pendulum "starting block"
             * if all the joints lie along a line, then it's no surprise that things will run afowl.
             * Even with the following adjustment may be overly conservative. */
            if (xBounding < yBounding / Constants.BoundingBoxAspectRatio)
                xBounding = yBounding / Constants.BoundingBoxAspectRatio;
            if (yBounding < xBounding / Constants.BoundingBoxAspectRatio)
                yBounding = xBounding / Constants.BoundingBoxAspectRatio;
            maximumDeltaX = Constants.XRangeLimitFactor * xBounding;
            maximumDeltaY = Constants.YRangeLimitFactor * yBounding;
            minimumDeltaX = Constants.XMinimumFactor * xBounding;
            minimumDeltaY = Constants.YMinimumFactor * yBounding;
        }

        internal Boolean DefineNewPositions(double positionChange)
        {
            posResult = PositionAnalysisResults.Normal;
            InitializeJointsAndLinks(positionChange);
            if (posResult == PositionAnalysisResults.InvalidPosition) return false;
            var numUnknownJoints = joints.Count(j => j.positionKnown != KnownState.Fully);

            while (posResult != PositionAnalysisResults.NoSolvableDyadFound && numUnknownJoints > 0)
            {
                posResult = PositionAnalysisResults.NoSolvableDyadFound;
                foreach (var j in joints.Where(j => j.Link2 != null))
                {
                    if (j.positionKnown == KnownState.Fully) continue;
                    joint knownJoint1;
                    joint knownJoint2;
                    GearData gData;
                    double angleChange;
                    switch (j.jointType)
                    {
                        case JointTypes.R:

                            #region R-R-R

                            if (FindKnownPositionOnLink(j, j.Link1, out knownJoint1) &&
                                FindKnownPositionOnLink(j, j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaCircleIntersection(j, knownJoint1, knownJoint2);
                                assignJointPosition(j, sJPoint, j.Link1);
                                setLinkPositionFromRotate(j, j.Link1);
                                setLinkPositionFromRotate(j, j.Link2);
                            }
                            #endregion
                            #region R-R-P

                            else if (FindKnownPositionOnLink(j, j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j, j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaCircleAndLineIntersection(j, knownJoint1, knownJoint2,
                                    out angleChange);
                                assignJointPosition(j, sJPoint, j.Link1);
                                setLinkPositionFromRotate(j, j.Link1, angleChange);
                                setLinkPositionFromRotate(j, j.Link2);
                            }
                            #endregion
                            #region P-R-R

                            else if (FindKnownSlopeOnLink(j, j.Link1, out knownJoint1)
                                     && FindKnownPositionOnLink(j, j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaCircleAndLineIntersection(j, knownJoint2, knownJoint1,
                                    out angleChange);
                                assignJointPosition(j, sJPoint, j.Link2);
                                setLinkPositionFromRotate(j, j.Link2, angleChange);
                                setLinkPositionFromRotate(j, j.Link1);
                            }
                            #endregion
                            #region P-R-P

                            else if (FindKnownSlopeOnLink(j, j.Link1, out knownJoint1)
                                     && FindKnownSlopeOnLink(j, j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaIntersectingLines(j, knownJoint1, knownJoint2);
                                assignJointPosition(j, sJPoint, j.Link1);
                                setLinkPositionFromRotate(j, j.Link1);
                                setLinkPositionFromRotate(j, j.Link2);
                            }
                            #endregion
                            #region R-R-GG

                            else if (FindKnownPositionOnLink(j, j.Link1, out knownJoint1) &&
                                     GearData.FindKnownGearAngleOnLink(j, j.Link1, j.Link2, joints, links, gearsData,
                                         out gData))
                            {
                                gData.SolveGearCenterFromKnownGearAngle(j, knownJoint1, links, out angleChange);
                                setLinkPositionFromRotate(knownJoint1, j.Link1, angleChange);
                            }
                            #endregion
                            #region GG-R-R

                            else if (FindKnownPositionOnLink(j, j.Link2, out knownJoint1) &&
                                     GearData.FindKnownGearAngleOnLink(j, j.Link2, j.Link1, joints, links, gearsData,
                                         out gData))
                            {
                                gData.SolveGearCenterFromKnownGearAngle(j, knownJoint1, links, out angleChange);
                                setLinkPositionFromRotate(knownJoint1, j.Link2, angleChange);
                            }

                            #endregion

                            break;
                        case JointTypes.P:

                            #region R-P-R

                            if (FindKnownPositionOnLink(j, j.Link1, out knownJoint1) &&
                                FindKnownPositionOnLink(j, j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveRPRIntersection(j, knownJoint1, knownJoint2, out angleChange);
                                assignJointPosition(j, sJPoint, j.Link2);
                                setLinkPositionFromRotate(knownJoint1, j.Link1, angleChange);
                                setLinkPositionFromRotate(j, j.Link2, angleChange);
                            }
                            #endregion
                            #region P-P-R

                            else if (FindKnownSlopeOnLink(j, j.Link1, out knownJoint1)
                                     && FindKnownPositionOnLink(j, j.Link2, out knownJoint2))
                            {
                                /* in this case, the block is on the rotating link and the slide is on the sliding link */
                                var sJPoint = solveViaSlopeToCircleIntersectionPPR(j, knownJoint1, knownJoint2);
                                assignJointPosition(j, sJPoint, j.Link2);
                                setLinkPositionFromRotate(j, j.Link1);
                                setLinkPositionFromRotate(j, j.Link2);
                                //setLinkPositionFromTranslation(knownJoint1, j.Link1, sJPoint.x - j.xLast, sJPoint.y - j.yLast,
                                //    j.SlideAngle);
                                //setLinkPositionFromTranslation(j, j.Link2, sJPoint.x - j.xLast, sJPoint.y - j.yLast);
                            }
                            #endregion
                            #region R-P-P

                            else if (FindKnownPositionOnLink(j, j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j, j.Link2, out knownJoint2))
                            {
                                /* in this case, the slide is on the rotating link and the block is on the sliding link */
                                point sJPoint = solveViaSlopeToCircleIntersectionRPP(j, knownJoint1, knownJoint2);
                                assignJointPosition(j, sJPoint, j.Link2);
                                setLinkPositionFromRotate(j, j.Link1);
                                setLinkPositionFromRotate(j, j.Link2);
                                //setLinkPositionFromTranslation(j, j.Link2, sJPoint.x - j.xLast, sJPoint.y - j.yLast);
                            }
                            #endregion
                            #region P-P-P

                            else if (FindKnownSlopeOnLink(j, j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j, j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveViaIntersectingLines(j, knownJoint1, knownJoint2);
                                assignJointPosition(j, sJPoint, j.Link1);
                                setLinkPositionFromRotate(j, j.Link1);
                                setLinkPositionFromRotate(j, j.Link2);
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

                            if (FindKnownPositionAndSlopeOnLink(j, j.Link1, out knownJoint1) &&
                                FindKnownPositionOnLink(j, j.Link2, out knownJoint2))
                            {
                                var sJPoint = solveRotatePinToSlot(j, knownJoint2, out angleChange);
                                assignJointPosition(j, sJPoint, j.Link2);
                                setLinkPositionFromRotate(j, j.Link2, angleChange);
                            }
                            #endregion
                            #region R&P-RP-P

                            else if (FindKnownPositionAndSlopeOnLink(j, j.Link1, out knownJoint1) &&
                                     FindKnownSlopeOnLink(j, j.Link2, out knownJoint2))
                            {
                                /* not sure this is right, but j must be partially known so it has enough
                             * information to define the first line. */
                                var sJPoint = solveViaIntersectingLines(j, j, knownJoint2);
                                assignJointPosition(j, sJPoint, j.Link1);
                                setLinkPositionFromTranslation(j, j.Link2, sJPoint.x - j.xLast, sJPoint.y - j.yLast,
                                    j.SlideAngle);
                            }

                            #endregion

                            break;
                        case JointTypes.G:
                            gData = gearsData[joints.IndexOf(j)];

                            #region R-G-R

                            if (gData.IsGearSolvableRGR(joints, links))
                            {
                                gData.SolveGearPositionAndAnglesRGR(joints, links);
                                posResult = PositionAnalysisResults.Normal;
                            }
                            #endregion
                            #region P-G-R

                            else if (gData.IsGearSolvablePGR(joints, links))
                            {
                                gData.SolveGearPositionAndAnglesPGR(joints, links);
                                posResult = PositionAnalysisResults.Normal;
                            }
                            #endregion
                            #region R-G-P

                            else if (gData.IsGearSolvableRGP(joints, links))
                            {
                                gData.SolveGearPositionAndAnglesRGP(joints, links);
                                posResult = PositionAnalysisResults.Normal;
                            }

                            #endregion

                            break;
                    }
                    if (posResult == PositionAnalysisResults.InvalidPosition) return false;
                }
                numUnknownJoints = joints.Count(j => j.positionKnown != KnownState.Fully);
            }
            if (posResult == PositionAnalysisResults.NoSolvableDyadFound && numUnknownJoints > 0)
            {
                try
                {
                    if (NDPS == null)
                        NDPS = new NonDyadicPositionSolver(this);
                    var NDPSError = 0.0;
                    NDPS.Run_PositionsAreClose(out NDPSError);
                    PositionError = Math.Sqrt(NDPSError);


                }
                catch (Exception e)
                {
                    Debug.WriteLine("Error in setting up and running NonDyadicPositionSolver.");
                }
            }
            if (posResult == PositionAnalysisResults.InvalidPosition) return false;
            if (joints.Any(j => Math.Abs(j.x - j.xInitial) > maximumDeltaX || Math.Abs(j.y - j.yInitial) > maximumDeltaY))
                return false;
            if (joints.All(j => Math.Abs(j.x - j.xLast) < minimumDeltaX && Math.Abs(j.y - j.yLast) < minimumDeltaY
                                && links.All(c => Math.Abs(c.Angle - c.AngleLast) < Constants.AngleMinimumFactor)))
                return false;

            UpdateSliderPosition();
            return true;
        }


        internal void UpdateSliderPosition()
        {
            foreach (var j in joints.Where(jt => jt.jointType == JointTypes.P || jt.jointType == JointTypes.RP
                                                 || (jt.jointType == JointTypes.G && !double.IsNaN(jt.OffsetSlideAngle)))
                )
            {
                var refJoint = j.Link1.ReferenceJoint1;

                var refVector = new[] { refJoint.x - j.x, refJoint.y - j.y };
                var slideVector = new[] { Math.Cos(j.SlideAngle), Math.Sin(j.SlideAngle) };
                j.SlidePosition = StarMath.dotProduct(slideVector, refVector);
            }
        }

        private void InitializeJointsAndLinks(double positionChange)
        {
            /* reset known states and error accumulator. */
            _posError = 0.0;
            foreach (var j in joints) j.positionKnown = KnownState.Unknown;
            foreach (var l in links) l.AngleIsKnown = KnownState.Unknown;

            /* reset ground link and joints */
            groundLink.AngleIsKnown = KnownState.Fully;
            foreach (var j in groundLink.joints)
                assignJointPosition(j, j.xInitial, j.yInitial, groundLink);
            foreach (var j in groundLink.joints.Where(j => j.jointType == JointTypes.P))
                setLinkPositionFromRotate(j, j.OtherLink(groundLink), 0.0);

            /* now, set input link. */
            if (inputJoint.jointType == JointTypes.R)
                setLinkPositionFromRotate(inputJoint, inputLink, positionChange);
            else if (inputJoint.jointType == JointTypes.P)
                setLinkPositionFromTranslation(inputJoint, inputLink, positionChange, positionChange,
                    inputJoint.SlideAngle);
            else
                throw new Exception(
                    "Input is not of type R or P (as currently required at the beginning of DefineNewPositions");
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

        private point solveViaCircleAndLineIntersection(joint j, joint circleCenterJoint, joint lineJoint,
            out double angleChange)
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


        private point solveViaIntersectingLines(joint j, joint knownJointA, joint knownJointB)
        {
            var ptA = defineParallelLineThroughJoint(j, knownJointA, j.Link1);
            var ptB = defineParallelLineThroughJoint(j, knownJointB, j.Link2);
            return Constants.solveViaIntersectingLines(Math.Tan(knownJointA.SlideAngle), ptA,
                Math.Tan(knownJointB.SlideAngle), ptB);
        }

        private point defineParallelLineThroughJoint(joint positionJoint, joint slopeJoint, link thisLink)
        {
            var length = thisLink.DistanceBetweenSlides(slopeJoint, positionJoint);
            var angle = slopeJoint.SlideAngle + Math.PI / 2;
            return new point(slopeJoint.x + length * Math.Cos(angle),
                slopeJoint.y + length * Math.Sin(angle));
        }

        // the basis of R-P-R dyad determination method is the complex little function
        private point solveRPRIntersection(joint j, joint knownJoint1, joint knownJoint2, out double angleChange)
        {
            var rAC = j.Link2.lengthBetween(j, knownJoint2);
            var rBC = j.Link1.DistanceBetweenSlides(j, knownJoint1);

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
            if (distthetaIntNegSquared < distthetaIntPosSquared)
            {
                angleChange = thetaIntNeg - oldTheta;
                return new point(xIntNeg, yIntNeg);
            }
            angleChange = thetaIntPos - oldTheta;
            return new point(xIntPos, yIntPos);
            /**** for the longest time, I thought there were 4 cases for this one. This led to some uncontrollable
             * cases. Now I konw (like all the rest) that it is two. I am leaving the code only because I am unsure
             * of a case where these might work out as true. **/
            /* first, the External positive case */
            //beta = Math.Asin((rBC - rAC * Math.Sin(alpha)) / lAB);
            //var thetaExtPos = Math.PI - alpha + beta + phi;
            //var xExtPos = knownJoint2.x + rAC * Math.Cos(thetaExtPos);
            //var yExtPos = knownJoint2.y + rAC * Math.Sin(thetaExtPos);
            //var distthetaExtPosSquared = Constants.distanceSqared(xExtPos, yExtPos, numPt.x, numPt.y);

            ///* second, the External negative case */
            //var thetaExtNeg = phi - beta - alpha;
            //var xExtNeg = knownJoint2.x + rAC * Math.Cos(thetaExtNeg);
            //var yExtNeg = knownJoint2.y + rAC * Math.Sin(thetaExtNeg);
            //var distthetaExtNegSquared = Constants.distanceSqared(xExtNeg, yExtNeg, numPt.x, numPt.y);

            //var distance = new List<double>
            //                   {
            //                       distthetaIntPosSquared, distthetaIntNegSquared,
            //                      // distthetaExtPosSquared, distthetaExtNegSquared
            //                   };
            //if (distance.All(double.IsNaN))
            //    return new point(double.NaN, double.NaN);
            //var minDist = distance.Where(x => !double.IsNaN(x)).Min();
            //
            // if (minDist > rAC *rAC) return new point(double.NaN, double.NaN);
            //point jPoint;
            //switch (distance.IndexOf(minDist))
            //{
            //    case 0:
            //        angleChange = thetaIntPos - oldTheta;
            //        jPoint = new point(xIntPos, yIntPos);
            //        break;
            //    case 1:
            //        angleChange = thetaIntNeg - oldTheta;
            //        jPoint = new point(xIntNeg, yIntNeg);
            //        break;
            //    case 2:
            //        angleChange = thetaExtPos - oldTheta;
            //        jPoint = new point(xExtPos, yExtPos);
            //        break;
            //    default:
            //        angleChange = thetaExtNeg - oldTheta;
            //        jPoint = new point(xExtNeg, yExtNeg);
            //        break;
            //}
            //return jPoint;
        }

        private point solveViaSlopeToCircleIntersectionPPR(joint j, joint slideJoint, joint circCenterJoint)
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
                return new point(xNeg, yNeg);
            return new point(xPos, yPos);
        }

        private point solveViaSlopeToCircleIntersectionRPP(joint j, joint circCenterJoint, joint slideJoint)
        {
            /* in this case, the slide is on the rotating link and the block is on the sliding link */
            double slopeB = Math.Tan(slideJoint.SlideAngle);
            var ptB = defineParallelLineThroughJoint(j, slideJoint, j.Link2);

            var actualSlideAngle = j.SlideAngle;
            var thetaNeg = actualSlideAngle - Math.PI / 2;
            var rAC = j.Link1.DistanceBetweenSlides(j, circCenterJoint);
            var orthoPt = new point(circCenterJoint.x + rAC * Math.Cos(thetaNeg),
                circCenterJoint.y + rAC * Math.Sin(thetaNeg));
            var slopeA = Math.Tan(actualSlideAngle);
            var ptNeg = Constants.solveViaIntersectingLines(slopeA, orthoPt, slopeB, ptB);
            var distNegSquared = Constants.distanceSqared(ptNeg.x, ptNeg.y, j.xNumerical, j.yNumerical);

            var thetaPos = actualSlideAngle + Math.PI / 2;
            orthoPt = new point(circCenterJoint.x + rAC * Math.Cos(thetaPos), circCenterJoint.y + rAC * Math.Sin(thetaPos));
            slopeA = Math.Tan(actualSlideAngle);
            var ptPos = Constants.solveViaIntersectingLines(slopeA, orthoPt, slopeB, ptB);
            var distPosSquared = Constants.distanceSqared(ptPos.x, ptPos.y, j.xNumerical, j.yNumerical);

            if (distNegSquared < distPosSquared)
                return ptNeg;
            return ptPos;
        }

        #endregion


        #region RP Solving Methods

        private point solveRotatePinToSlot(joint j, joint circleCenterJoint, out double angleChange)
        {
            var circleLink = j.Link2;
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
            var ptNeg = new point(circleCenterJoint.x + r1 * Math.Cos(thetaNeg),
                circleCenterJoint.y + r1 * Math.Sin(thetaNeg));
            var distNegSquared = Constants.distanceSqared(numPt, ptNeg);

            var thetaPos = thetaBase + alpha;
            var ptPos = new point(circleCenterJoint.x + r1 * Math.Cos(thetaPos),
                circleCenterJoint.y + r1 * Math.Sin(thetaPos));
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

        private double solveRotateSlotToPin(joint fixedJoint, joint slideJoint, link thisLink, double j2j_angle)
        {
            var dist2Slide = thisLink.DistanceBetweenSlides(slideJoint, fixedJoint);
            if (Constants.sameCloseZero(dist2Slide)) return j2j_angle;
            var distanceBetweenJoints = Constants.distance(fixedJoint, slideJoint);
            if (Math.Abs(dist2Slide) > distanceBetweenJoints)
            {
                posResult = PositionAnalysisResults.InvalidPosition;
                return double.NaN;
            }
            var changeInSlideAngle = Math.Asin(dist2Slide / distanceBetweenJoints);

            var slideAnglePos = j2j_angle + changeInSlideAngle;
            var slideAngleNeg = j2j_angle - changeInSlideAngle;
            var lastSlideAngle = slideJoint.OffsetSlideAngle + thisLink.AngleLast;
            var deltaAnglePos = slideAnglePos - lastSlideAngle;
            while (deltaAnglePos > Math.PI / 2) deltaAnglePos -= Math.PI;
            while (deltaAnglePos < -Math.PI / 2) deltaAnglePos += Math.PI;
            var deltaAngleNeg = slideAngleNeg - lastSlideAngle;
            while (deltaAngleNeg > Math.PI / 2) deltaAngleNeg -= Math.PI;
            while (deltaAngleNeg < -Math.PI / 2) deltaAngleNeg += Math.PI;

            var deltaAngleNum = thisLink.AngleNumerical - thisLink.AngleLast;
            while (deltaAngleNum > Math.PI) deltaAngleNum -= Constants.FullCircle;
            while (deltaAngleNum < -Math.PI) deltaAngleNum += Constants.FullCircle;

            var errorPos = Math.Abs(deltaAnglePos - deltaAngleNum);
            var errorNeg = Math.Abs(deltaAngleNeg - deltaAngleNum);

            if (errorPos < errorNeg)
            {
                PositionError = distanceBetweenJoints * errorPos;
                return deltaAnglePos;
            }
            PositionError = distanceBetweenJoints * errorNeg;
            return deltaAngleNeg;
        }

        #endregion

        #region set & find link position and angles

        private void assignJointPosition(joint j, point newPoint, link thisLink) // = null)
        {
            assignJointPosition(j, newPoint.x, newPoint.y, thisLink);
        }

        private void assignJointPosition(joint j, double xNew, double yNew, link thisLink)
        {
            if (double.IsInfinity(xNew) || double.IsInfinity(yNew) ||
                double.IsNaN(xNew) || double.IsNaN(yNew))
                posResult = PositionAnalysisResults.InvalidPosition;
            else
            {
                j.x = xNew;
                j.y = yNew;
                posResult = PositionAnalysisResults.Normal;
                if (j.FixedWithRespectTo(thisLink))
                {
                    j.positionKnown = KnownState.Fully;
                    xNew -= j.xNumerical;
                    yNew -= j.yNumerical;
                    PositionError = Math.Sqrt(xNew * xNew + yNew * yNew);
                }
                else j.positionKnown = KnownState.Partially;
            }
        }

        /* ugh, this function is taking the most time.
         * knownJoint does not have to be KnownState.Fully - rather it is never Unknown, but can be Partially
         * in the case of coming from a slide joint.*/

        internal void setLinkPositionFromRotate(joint knownJoint, link thisLink, double angleChange = double.NaN)
        {
            if (thisLink == null) return;
            //if (thisLink.AngleIsKnown == KnownState.Fully) return; //this sometimes happen as the process recurses, esp. around RP and G joints   
            // if (double.IsNaN(angleChange) && thisLink.AngleIsKnown == KnownState.Fully)
            if (thisLink.AngleIsKnown == KnownState.Fully)
                angleChange = thisLink.Angle - thisLink.AngleLast;
            if (double.IsNaN(angleChange))
            {
                if (!knownJoint.FixedWithRespectTo(thisLink))
                {
                    knownJoint =
                        thisLink.joints.FirstOrDefault(j => j != knownJoint && j.positionKnown == KnownState.Fully
                                                            && j.FixedWithRespectTo(thisLink));
                    if (knownJoint == null) return;
                }
                var j2 = (thisLink.joints.FirstOrDefault(j => j != knownJoint && j.positionKnown == KnownState.Fully
                                                              && j.FixedWithRespectTo(thisLink))
                          ?? thisLink.joints.FirstOrDefault(j => j != knownJoint && j.positionKnown == KnownState.Fully))
                         ??
                         thisLink.joints.FirstOrDefault(j => j != knownJoint && j.positionKnown == KnownState.Partially
                                                             && !j.FixedWithRespectTo(thisLink));
                if (j2 == null) return;
                var new_j2j_Angle = Constants.angle(knownJoint, j2);
                var old_j2j_Angle = Constants.angle(knownJoint.xLast, knownJoint.yLast, j2.xLast, j2.yLast);
                angleChange = new_j2j_Angle - old_j2j_Angle;
                if (j2.SlidingWithRespectTo(thisLink))
                    angleChange = solveRotateSlotToPin(knownJoint, j2, thisLink, new_j2j_Angle);
                if (posResult == PositionAnalysisResults.InvalidPosition) return;
            }
            thisLink.Angle = thisLink.AngleLast + angleChange;
            thisLink.AngleIsKnown = KnownState.Fully;

            /* now update other joints on this link that might be determined now that the angle is known */
            knownJoint =
                thisLink.joints.FirstOrDefault(
                    j => j.FixedWithRespectTo(thisLink) && j.positionKnown == KnownState.Fully);
            foreach (var j in thisLink.joints)
            {
                if (j.positionKnown == KnownState.Fully)
                {
                    if (j.OtherLink(thisLink).AngleIsKnown != KnownState.Fully && j.jointType == JointTypes.P)
                        setLinkPositionFromRotate(j, j.OtherLink(thisLink), angleChange);
                }
                else
                {
                    if (j.positionKnown != KnownState.Fully && knownJoint != null)
                    {
                        //if (j.jointType == JointTypes.G)
                        //    assignJointPosition(j, j.x, j.y, thisLink);
                        if (j.FixedWithRespectTo(thisLink))
                        {
                            var length = thisLink.lengthBetween(j, knownJoint);
                            var angle = Constants.angle(knownJoint.xLast, knownJoint.yLast, j.xLast, j.yLast);
                            angle += angleChange;
                            assignJointPosition(j, knownJoint.x + length * Math.Cos(angle),
                                knownJoint.y + length * Math.Sin(angle), thisLink);
                        }
                        else if (j.jointType != JointTypes.G)
                        {
                            var length = thisLink.DistanceBetweenSlides(j, knownJoint);
                            var angle = j.SlideAngle - Math.PI / 2;
                            angle += angleChange;
                            //while (angle < -Math.PI / 2) angle += Math.PI;
                            assignJointPosition(j, knownJoint.x + length * Math.Cos(angle),
                                knownJoint.y + length * Math.Sin(angle), thisLink);
                        }
                    }    
                    var otherLink = j.OtherLink(thisLink);
                    if (otherLink == null) continue;
                    if (j.jointType == JointTypes.G
                        && gearsData[joints.IndexOf(j)].SetGearRotation(thisLink, otherLink, links, joints))
                    {
                        var otherKnownJoint =
                            otherLink.joints.FirstOrDefault(
                                jj => jj.FixedWithRespectTo(otherLink) && jj.positionKnown == KnownState.Fully);
                        if (otherKnownJoint != null) setLinkPositionFromRotate(otherKnownJoint, otherLink);
                    }
                    else if (otherLink.AngleIsKnown != KnownState.Fully && j.jointType == JointTypes.P)
                        setLinkPositionFromRotate(j, otherLink, angleChange);
                    else if (j.positionKnown == KnownState.Fully)
                        setLinkPositionFromRotate(j, otherLink);
                }
            }
        }


        private void setLinkPositionFromTranslation(joint knownJoint, link thisLink,
                double deltaX, double deltaY, double angle = double.NaN)
        {
            if (thisLink == null) return;
            // if (thisLink.AngleIsKnown == KnownState.Fully) return;
            foreach (var j in thisLink.joints.Where(j => j.positionKnown != KnownState.Fully))
            {
                if (knownJoint.FixedWithRespectTo(thisLink))
                {
                    if (double.IsNaN(angle))
                        assignJointPosition(j, j.xLast + deltaX, j.yLast + deltaY, thisLink);
                    else
                        assignJointPosition(j, j.xLast + deltaX * Math.Cos(angle),
                            j.yLast + deltaY * Math.Sin(angle), thisLink);
                }
                if (!j.FixedWithRespectTo(thisLink)) continue;
                if (j.jointType == JointTypes.P)
                    setLinkPositionFromTranslation(j, j.OtherLink(thisLink), deltaX, deltaY, j.SlideAngle);
                else setLinkPositionFromRotate(j, j.OtherLink(thisLink));
            }
        }



        private static bool FindKnownPositionAndSlopeOnLink(joint unkJoint, link link, out joint knownJoint)
        {
            knownJoint = null;
            if (link.AngleIsKnown == KnownState.Unknown) return false;
            return FindKnownPositionOnLink(unkJoint, link, out knownJoint);
        }
        private static bool FindKnownPositionOnLink(joint unkJoint, link link, out joint knownJoint)
        {
            knownJoint = link.joints.FirstOrDefault(j => j != unkJoint && j.positionKnown == KnownState.Fully && j.FixedWithRespectTo(link));
            return knownJoint != null;
        }
        private static bool FindKnownSlopeOnLink(joint unkJoint, link link, out joint knownJoint)
        {
            knownJoint = null;
            if (link.AngleIsKnown == KnownState.Unknown) return false;
            knownJoint = link.joints.FirstOrDefault(j => j != unkJoint && j.positionKnown != KnownState.Unknown);
            return knownJoint != null;
        }
        #endregion

    }
}