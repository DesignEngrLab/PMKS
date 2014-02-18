using System;
using System.Linq;
using System.Collections.Generic;
using StarMathLib;

namespace PlanarMechanismSimulator
{
    public class link
    {
        /// <summary>
        /// 
        /// </summary>
        public List<joint> joints { get; private set; }

        private int numJoints;
        /// <summary>
        /// Is the link the ground-link?
        /// </summary>
        public readonly Boolean isGround;

        internal KnownState AngleIsKnown;

        /// <summary>
        /// The lengths between joints that are fixed with respect to this link
        /// </summary>
        private Dictionary<int, double> lengths;

        /// <summary>
        /// The shortest distance between a joint and the line of a sliding joint
        /// </summary>
        private Dictionary<int, double> distanceToSlideLine;

        /// <summary>
        /// The shortest distance between a joint and the line of a sliding joint
        /// </summary>
        private Dictionary<int, double> angleFromBlockToJoint;

        /// <summary>
        /// Gets the maximum length.
        /// </summary>
        /// <value>
        /// The maximum length.
        /// </value>
        public double MaxLength
        {
            get
            {
                return lengths.Values.Max(v => Math.Abs(v));
            }
        }

        /// <summary>
        /// Gets the sum of all the lengths between the pivots.
        /// </summary>
        /// <value>
        /// The total length.
        /// </value>
        public double TotalLength
        {
            get
            {
                return lengths.Values.Sum(v => Math.Abs(v));
            }
        }
        public string name { get; private set; }

        public double AngleInitial { get; set; }
        public double Angle { get; set; }
        public double AngleLast { get; set; }
        public double AngleNumerical { get; set; }

        public double Velocity { get; set; }
        public double VelocityLast { get; set; }
        public double Acceleration { get; set; }

        internal link(string name, List<joint> Joints, Boolean IsGround)
        {
            this.name = name;
            joints = Joints;
            isGround = IsGround;
        }

        private link() { }

        internal void DetermineLengthsAndReferences()
        {
            numJoints = joints.Count;
            var fixedJoints = joints.Where(j => j.FixedWithRespectTo(this)).ToList();
            if (fixedJoints.Count < 2 || fixedJoints.Count(j => j.isGround) > 1)
                Angle = AngleInitial = AngleNumerical = AngleLast = 0.0;
            else if (fixedJoints.Count(j => j.isGround) == 1)
            {
                var ground = fixedJoints.FirstOrDefault(j => j.isGround);
                var notGround = fixedJoints.FirstOrDefault(j => !j.isGround);
                Angle = AngleInitial = AngleNumerical = AngleLast = Constants.angle(ground, notGround);
            }
            else Angle = AngleInitial = AngleNumerical = AngleLast = Constants.angle(fixedJoints[0], fixedJoints[1]);
            foreach (var j in joints)
            {
                if (j.jointType == JointTypes.R) j.InitSlideAngle = Double.NaN;
                else if (j.SlidingWithRespectTo(this))
                {
                    j.InitSlideAngle -= AngleInitial;
                    while (j.InitSlideAngle < -Math.PI / 2) j.InitSlideAngle += Math.PI;
                    while (j.InitSlideAngle > Math.PI / 2) j.InitSlideAngle -= Math.PI;
                    j.SlideLimits = null;
                }
            }
            lengths = new Dictionary<int, double>();
            distanceToSlideLine = new Dictionary<int, double>();
            angleFromBlockToJoint = new Dictionary<int, double>();
            for (int i = 0; i < joints.Count - 1; i++)
                for (int j = i + 1; j < joints.Count; j++)
                {
                    var iJoint = joints[i];
                    var jJoint = joints[j];
                    var key = numJoints * i + j;
                    double distance = 0.0;
                    int signForDistance = 1;
                    if (!iJoint.SlidingWithRespectTo(this) && !jJoint.SlidingWithRespectTo(this))
                    {
                        distance = Constants.distance(iJoint.xInitial, iJoint.yInitial, jJoint.xInitial, jJoint.yInitial);
                        lengths.Add(key, distance);
                    }
                    if (iJoint.SlidingWithRespectTo(this) && jJoint.SlidingWithRespectTo(this))
                    {
                        if (!Constants.sameCloseZero(iJoint.SlideAngle, jJoint.SlideAngle))
                        {
                            distanceToSlideLine.Add(key, 0.0);
                            angleFromBlockToJoint.Add(key, iJoint.SlideAngle - jJoint.SlideAngle);
                        }
                        else
                        {
                            distance = Constants.distance(iJoint.xInitial, iJoint.yInitial, jJoint.xInitial,
                                jJoint.yInitial);
                            var theta = Constants.angle(iJoint.xInitial, iJoint.yInitial, jJoint.xInitial,
                                jJoint.yInitial) - Math.PI / 2 - iJoint.SlideAngle;
                            distanceToSlideLine.Add(key, Math.Abs(distance * Math.Cos(theta)));
                            angleFromBlockToJoint.Add(key, 0.0);
                        }
                    }
                    else if (iJoint.SlidingWithRespectTo(this))// && iJoint.jointType == JointTypes.P)
                    {
                        Boolean belowLinePositiveSlope;
                        var orthoPt = findOrthoPoint(jJoint, iJoint, iJoint.SlideAngle,
                            out belowLinePositiveSlope);
                        if (belowLinePositiveSlope) signForDistance = -1;
                        distance = Constants.distance(orthoPt.x, orthoPt.y, jJoint.xInitial, jJoint.yInitial);
                        distanceToSlideLine.Add(key, signForDistance * distance);

                        if (iJoint.SlideLimits == null)
                        {
                            var a = new[] { orthoPt.x - jJoint.xInitial, orthoPt.y - jJoint.yInitial };
                            a = StarMath.normalize(a);
                            var b = new[] { iJoint.xInitial - orthoPt.x, iJoint.yInitial - orthoPt.y };
                            var cross = StarMath.crossProduct2(a, b);
                            iJoint.SlideLimits = new[] {j, cross, cross, cross };
                        }
                    }
                    else if (jJoint.SlidingWithRespectTo(this))// && jJoint.jointType == JointTypes.P)
                    {
                        var reversekey = numJoints * j + i;
                        Boolean belowLinePositiveSlope;
                        var orthoPt = findOrthoPoint(iJoint, jJoint, jJoint.SlideAngle,
                            out belowLinePositiveSlope);
                        if (belowLinePositiveSlope) signForDistance = -1;
                        distance = Constants.distance(orthoPt.x, orthoPt.y, iJoint.xInitial, iJoint.yInitial);
                        distanceToSlideLine.Add(reversekey, signForDistance * distance);

                        if (jJoint.SlideLimits == null)
                        {
                            var a = new[] { orthoPt.x - iJoint.xInitial, orthoPt.y - iJoint.yInitial };
                            a = StarMath.normalize(a);
                            var b = new[] { jJoint.xInitial - orthoPt.x, jJoint.yInitial - orthoPt.y };
                            var cross = StarMath.crossProduct2(a, b);
                            jJoint.SlideLimits = new[] {i, cross, cross, cross };
                        }
                    }
                    if (iJoint.jointType == JointTypes.P)
                    {
                        var result = iJoint.SlideAngle -
                                     Constants.angle(iJoint.xInitial, iJoint.yInitial, jJoint.xInitial, jJoint.yInitial);
                        while (result < -Math.PI / 2) result += Math.PI;
                        while (result > Math.PI / 2) result -= Math.PI;
                        angleFromBlockToJoint.Add(key, result);
                    }
                    if (jJoint.jointType == JointTypes.P)
                    {
                        var reversekey = numJoints * j + i;
                        var result = jJoint.SlideAngle -
                                     Constants.angle(jJoint.xInitial, jJoint.yInitial, iJoint.xInitial, iJoint.yInitial);
                        while (result < -Math.PI / 2) result += Math.PI;
                        while (result > Math.PI / 2) result -= Math.PI;
                        angleFromBlockToJoint.Add(reversekey, result);
                    }
                }
        }

        internal double lengthBetween(joint joint1, joint joint2)
        {
            if (joint1 == joint2) return 0.0;
            var i = joints.IndexOf(joint1);
            var j = joints.IndexOf(joint2);
            if (i > j) return lengths[numJoints * j + i];
            return lengths[numJoints * i + j];
        }

        internal double DistanceBetweenSlides(joint slidingJoint, joint referenceJoint)
        {
            if (slidingJoint == referenceJoint) return 0.0;
            var i = joints.IndexOf(slidingJoint);
            var j = joints.IndexOf(referenceJoint);

            // if (i > j) return distanceToSlideLine[numJoints * j + i];
            return distanceToSlideLine[numJoints * i + j];
        }

        internal void setLength(joint joint1, joint joint2, double length)
        {
            if (joint1 == joint2)
                throw new Exception("link.setLength: cannot set the distance between joints because same joint provided as both joint1 and joint2.");
            var i = joints.IndexOf(joint1);
            var j = joints.IndexOf(joint2);
            if (i > j) lengths[numJoints * j + i] = length;
            else lengths[numJoints * i + j] = length;
        }

        internal double angleOfBlockToJoint(joint blockJoint, joint referenceJoint)
        {
            if (blockJoint == referenceJoint) return 0.0;
            var i = joints.IndexOf(blockJoint);
            var j = joints.IndexOf(referenceJoint);
            return angleFromBlockToJoint[numJoints * i + j];
        }
        /// <summary>
        /// Finds the orthogonal point on the line - the point closest to the reference Joint - the 
        /// point that makes a right angle (orthogonal angle) with the line. The boolean "belowLinePositiveeSlope"
        /// is true when:
        /// a. the slope is positive and the reference point is below the line (lower-y value) or
        /// b. when the slope is negative and the reference point is above the line or
        /// c. the slope is vertical (infinity or positive) and the reference point is to the right (higher x-value)
        /// d. the slope is horizontal (zero) and the reference point is BELOW.
        /// </summary>
        /// <param name="refJoint">The reference joint.</param>
        /// <param name="slideJoint">The slide joint.</param>
        /// <param name="lineAngle">The line angle.</param>
        /// <param name="belowLinePositiveSlope">if set to <c>true</c> [below line positive slope].</param>
        /// <returns></returns>
        public static point findOrthoPoint(joint refJoint, joint slideJoint, double lineAngle, out Boolean belowLinePositiveSlope)
        {
            if (Constants.sameCloseZero(lineAngle))
            {
                belowLinePositiveSlope = (refJoint.y < slideJoint.y);
                return new point(refJoint.x, slideJoint.y);
            }
            else if (Constants.sameCloseZero(Math.Abs(lineAngle), Math.PI / 2))
            {
                belowLinePositiveSlope = (refJoint.x > slideJoint.x);
                return new point(slideJoint.x, refJoint.y);
            }
            var slope = Math.Tan(lineAngle);
            var offset = (slideJoint.y - slope * slideJoint.x);
            var x = (refJoint.x + slope * (refJoint.y - offset)) / (slope * slope + 1);
            var y = slope * x + offset;
            belowLinePositiveSlope = (slope > 0) ? (refJoint.y - slope * refJoint.x) < offset :
                 (refJoint.y - slope * refJoint.x) > offset;

            return new point(x, y);
        }


        internal link copy(List<joint> oldJoints, List<joint> newJoints)
        {
            return new link
                {
                    Angle = Angle,
                    AngleInitial = AngleInitial,
                    AngleLast = AngleLast,
                    AngleIsKnown = AngleIsKnown,
                    AngleNumerical = AngleNumerical,
                    joints = joints.Select(j => newJoints[oldJoints.IndexOf(j)]).ToList(),
                    numJoints = numJoints,
                    lengths = new Dictionary<int, double>(lengths),
                    distanceToSlideLine = new Dictionary<int, double>(distanceToSlideLine),
                    name = name ,
                    angleFromBlockToJoint = new Dictionary<int, double>(angleFromBlockToJoint)
                    
                };
        }
    }
}