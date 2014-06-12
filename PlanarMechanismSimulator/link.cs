using System;
using System.Linq;
using System.Collections.Generic;
using PMKS;
using StarMathLib;

namespace PMKS
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

        private link()
        { joints = new List<joint>(); }

        internal void DetermineLengthsAndReferences()
        {
            numJoints = joints.Count;
            var linkAngle = double.NaN;
            var fixedJoints = joints.Where(j => j.FixedWithRespectTo(this)).OrderBy(j => j.yInitial).ToList();
            if (fixedJoints.Count < 2 || fixedJoints.Count(j => j.isGround) > 1)
                linkAngle = 0.0;
            else if (fixedJoints.Count(j => j.isGround) == 1)
            {
                var ground = fixedJoints.FirstOrDefault(j => j.isGround);
                var notGround = fixedJoints.FirstOrDefault(j => !j.isGround);
                linkAngle = Constants.angle(ground, notGround);
            }
            else linkAngle = Constants.angle(fixedJoints[0], fixedJoints[1]);
            while (linkAngle < -Math.PI / 2) linkAngle += Math.PI;
            while (linkAngle > Math.PI / 2) linkAngle -= Math.PI;
            Angle = AngleInitial = AngleNumerical = AngleLast = linkAngle;
            foreach (var j in joints.Where(j => j.SlidingWithRespectTo(this)))
                j.InitSlideAngle -= AngleInitial;

            lengths = new Dictionary<int, double>();

            distanceToSlideLine = new Dictionary<int, double>();
            angleFromBlockToJoint = new Dictionary<int, double>();
            for (int i = 0; i < joints.Count - 1; i++)
                for (int j = i + 1; j < joints.Count; j++)
                {
                    var iJoint = joints[i];
                    var jJoint = joints[j];
                    if (iJoint.SlidingWithRespectTo(this) && jJoint.SlidingWithRespectTo(this))
                    {
                        var key = numJoints * i + j;
                        angleFromBlockToJoint.Add(key, 0.0);
                        distanceToSlideLine.Add(key, 0.0);
                        key = numJoints * j + i;
                        angleFromBlockToJoint.Add(key, 0.0);
                        distanceToSlideLine.Add(key, 0.0);
                    }
                    else if (iJoint.SlidingWithRespectTo(this) && !jJoint.SlidingWithRespectTo(this))
                    {
                        addSlideDictionaryEntry(iJoint, jJoint, i, j);
                        addBlockAngleDictionaryEntry(iJoint, jJoint, i, j);
                        if (jJoint.jointType == JointTypes.P)
                        {
                            addBlockAngleDictionaryEntry(jJoint, iJoint, j, i);
                            addSlideDictionaryEntry(jJoint, iJoint, j, i);
                        }
                    }
                    else if (!iJoint.SlidingWithRespectTo(this) && jJoint.SlidingWithRespectTo(this))
                    {
                        addSlideDictionaryEntry(jJoint, iJoint, j, i);
                        addBlockAngleDictionaryEntry(jJoint, iJoint, j, i);
                        if (iJoint.jointType == JointTypes.P)
                        {
                            addBlockAngleDictionaryEntry(iJoint, jJoint, i, j);
                            addSlideDictionaryEntry(iJoint, jJoint, i, j);
                        }
                    }
                    else //    if (!iJoint.SlidingWithRespectTo(this) && !jJoint.SlidingWithRespectTo(this))
                    {
                        lengths.Add(numJoints * i + j,
                                Constants.distance(iJoint.xInitial, iJoint.yInitial, jJoint.xInitial, jJoint.yInitial));
                        if (iJoint.jointType == JointTypes.P)
                        {
                            addBlockAngleDictionaryEntry(iJoint, jJoint, i, j);
                            addSlideDictionaryEntry(iJoint, jJoint, i, j);
                        }
                        if (jJoint.jointType == JointTypes.P)
                        {
                            addBlockAngleDictionaryEntry(jJoint, iJoint, j, i);
                            addSlideDictionaryEntry(jJoint, iJoint, j, i);
                        }
                    }
                }
        }

        private void addSlideDictionaryEntry(joint slideJoint, joint fixedJoint, int slideIndex, int fixedIndex)
        {
            var key = numJoints * slideIndex + fixedIndex;
            var orthoPt = findOrthoPoint(fixedJoint, slideJoint);
            var slideUnitVector = new[] { Math.Cos(slideJoint.SlideAngle), Math.Sin(slideJoint.SlideAngle) };
            var fixedVector = new[] { fixedJoint.xInitial - orthoPt.x, fixedJoint.yInitial - orthoPt.y };
            distanceToSlideLine.Add(key, StarMath.crossProduct2(slideUnitVector, fixedVector));

            if (slideJoint.SlideLimits == null && slideJoint.SlidingWithRespectTo(this))
            {
                var cross = 0.0;
                var slideVector = new[] { slideJoint.xInitial - orthoPt.x, slideJoint.yInitial - orthoPt.y };
                if (Constants.sameCloseZero(fixedVector[0]) && Constants.sameCloseZero(fixedVector[1]))
                    cross = StarMath.dotProduct(slideUnitVector, slideVector);
                else
                {
                    var fixedUnitVector = StarMath.normalize(fixedVector);
                    cross = StarMath.crossProduct2(slideVector, fixedUnitVector);
                }
                slideJoint.SlideLimits = new[] { fixedIndex, cross, cross, cross };
            }
        }

        private void addBlockAngleDictionaryEntry(joint pJoint, joint refJoint, int pIndex, int refIndex)
        {
            var key = numJoints * pIndex + refIndex;
            var result = pJoint.SlideAngle -
                         Constants.angle(pJoint.xInitial, pJoint.yInitial, refJoint.xInitial, refJoint.yInitial);
            angleFromBlockToJoint.Add(key, result);
        }

        internal double lengthBetween(joint joint1, joint joint2)
        {
            if (joint1 == joint2) return 0.0;
            var i = joints.IndexOf(joint1);
            var j = joints.IndexOf(joint2);
            if (i > j) return lengths[numJoints * j + i];
            return lengths[numJoints * i + j];
        }

        /// <summary>
        /// returns the shortest distance between the sliding joint and the reference point.
        /// This is a signed distance! Given the unit vector created from the slide angle, 
        /// the positive distance is "on the left" or counter-clockwise, while a negative
        /// distance is "on the right" or clockwise.
        /// </summary>
        /// <param name="slidingJoint">The sliding joint.</param>
        /// <param name="referenceJoint">The reference joint.</param>
        /// <returns></returns>
        public double DistanceBetweenSlides(joint slidingJoint, joint referenceJoint)
        {
            if (slidingJoint == referenceJoint) return 0.0;
            var slideIndex = joints.IndexOf(slidingJoint);
            var fixedIndex = joints.IndexOf(referenceJoint);
            var index = numJoints * slideIndex + fixedIndex;
            if (distanceToSlideLine.ContainsKey(index))
                return distanceToSlideLine[index];
            else return -distanceToSlideLine[numJoints * fixedIndex + slideIndex];
        }
        /// <summary>
        /// returns the angle between the slide and the reference joint.
        /// </summary>
        /// <param name="blockJoint">The block joint.</param>
        /// <param name="referenceJoint">The reference joint.</param>
        /// <returns></returns>
        internal double angleOfBlockToJoint(joint blockJoint, joint referenceJoint)
        {
            if (blockJoint == referenceJoint) return 0.0;
            var i = joints.IndexOf(blockJoint);
            var j = joints.IndexOf(referenceJoint);
            return angleFromBlockToJoint[numJoints * i + j];
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
        public static point findOrthoPoint(joint refJoint, joint slideJoint)
        {
            var slideAngle = slideJoint.SlideAngle;
            if (Constants.sameCloseZero(slideAngle))
                return new point(refJoint.x, slideJoint.y);
            else if (Constants.sameCloseZero(Math.Abs(slideAngle), Math.PI / 2))
                return new point(slideJoint.x, refJoint.y);
            var slope1 = Math.Tan(slideAngle);
            var slope2 = -1 / slope1;
            var offset1 = slideJoint.y - slope1 * slideJoint.x;
            var offset2 = refJoint.y - slope2 * refJoint.x;
            var x = (offset2 - offset1) / (slope1 - slope2);
            var y = slope1 * x + offset1;
            return new point(x, y);
        }


        internal link copy()
        {
            return new link
                {
                    Angle = Angle,
                    AngleInitial = AngleInitial,
                    AngleLast = AngleLast,
                    AngleIsKnown = AngleIsKnown,
                    AngleNumerical = AngleNumerical,
                    numJoints = numJoints,
                    lengths = new Dictionary<int, double>(lengths),
                    distanceToSlideLine = new Dictionary<int, double>(distanceToSlideLine),
                    name = name,
                    angleFromBlockToJoint = new Dictionary<int, double>(angleFromBlockToJoint)

                };
        }
    }
}