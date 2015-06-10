using System;
using System.Linq;
using System.Collections.Generic;
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
        public Boolean IsGround { get; internal set; }

        internal KnownState AngleIsKnown;

        public joint ReferenceJoint1 { get; internal set; }
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
            this.IsGround = IsGround;
        }

        private link()
        { joints = new List<joint>(); }


        #region Determine Lengths And References and Initializing Functions
        internal void DetermineLengthsAndReferences()
        {
            numJoints = joints.Count;
            #region Define Initial Link Angle
            var fixedJoints = joints.Where(j => j.FixedWithRespectTo(this)).
                OrderBy(j => (j.isGround) ? 0 : 1).
                ThenBy(j => (j.Link2 != null) ? 0 : 1).ThenBy(j => j.xInitial).ToList();
            ReferenceJoint1 = fixedJoints[0];
            /* the linkAngle is defined from "the joint with the lowest initial x value
             * that is fixed to this link" to "the joint with the highest initial x value
             * that is fixed to this link. If the link has only one fixed joint (and
             * it will necessarily have one due to the addition of a joint added in
             * Simulator.addReferencePivotsToSlideOnlyLinks()) or is ground, then the 
             * initial angle is zero. */
            if (fixedJoints.Count == 2 && !this.IsGround)
                AngleInitial = Constants.angle(ReferenceJoint1, fixedJoints[1]);
            else AngleInitial = 0.0;
            Angle = AngleNumerical = AngleLast = AngleInitial;
            foreach (var j in joints.Where(j => j.SlidingWithRespectTo(this)))
                j.OffsetSlideAngle -= AngleInitial;
            #endregion
            #region Joint-to-Joint Dictionaries
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
            #endregion
        }


        /// <summary>
        /// Adds a slide dictionary entry.
        /// the shortest distance between the sliding joint and the reference point.
        /// This is a signed distance! Given the unit vector created from the slide angle, 
        /// the positive distance is "on the left" or counter-clockwise, while a negative
        /// distance is "on the right" or clockwise.
        /// </summary>
        /// <param name="slideJoint">The slide joint.</param>
        /// <param name="refJoint">The fixed joint.</param>
        /// <param name="slideIndex">Index of the slide.</param>
        /// <param name="fixedIndex">Index of the fixed.</param>
        private void addSlideDictionaryEntry(joint slideJoint, joint refJoint, int slideIndex, int fixedIndex)
        {
            var key = numJoints * slideIndex + fixedIndex;
            var slideUnitVector = new[] { Math.Cos(slideJoint.SlideAngle), Math.Sin(slideJoint.SlideAngle) };
            var refVector = new[] { refJoint.xInitial - slideJoint.xInitial, refJoint.yInitial - slideJoint.yInitial };
            var distance = (double.IsNaN(slideJoint.SlideAngle))
                ? StarMath.norm2(refVector)
                : StarMath.crossProduct2(slideUnitVector, refVector);
            distanceToSlideLine.Add(key, distance);
        }

        private void addBlockAngleDictionaryEntry(joint pJoint, joint refJoint, int pIndex, int refIndex)
        {
            var key = numJoints * pIndex + refIndex;
            var result = (pJoint.jointType == JointTypes.G) ? Math.PI / 2 : pJoint.SlideAngleInitial -
                         Constants.angle(pJoint.xInitial, pJoint.yInitial, refJoint.xInitial, refJoint.yInitial);
            angleFromBlockToJoint.Add(key, result);
        }
        #endregion

        #region Retrieve Joint-to-Joint data

        /// <summary>
        /// Returns the Lengths the between two joints that are fixed with respect to this link.
        /// </summary>
        /// <param name="joint1">joint1.</param>
        /// <param name="joint2">joint2.</param>
        /// <returns></returns>
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
            else //throw new Exception("link.DistanceBetweenSlides ->args wrong");
                return distanceToSlideLine[numJoints * fixedIndex + slideIndex];
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


        #endregion


        /// <summary>
        /// Sets the length between the two joints. This function is only called from Simulator.AssignLengths
        /// and is to be used only when the solving a problem of construct-ability (or whatever they call it in
        /// the literature).
        /// </summary>
        /// <param name="joint1">The joint1.</param>
        /// <param name="joint2">The joint2.</param>
        /// <param name="length">The length.</param>
        /// <exception cref="System.Exception">link.setLength: cannot set the distance between joints because same joint provided as both joint1 and joint2.</exception>
        internal void SetLength(joint joint1, joint joint2, double length)
        {
            if (joint1 == joint2)
                throw new Exception("link.setLength: cannot set the distance between joints because same joint provided as both joint1 and joint2.");
            var i = joints.IndexOf(joint1);
            var j = joints.IndexOf(joint2);
            if (i > j) lengths[numJoints * j + i] = length;
            else lengths[numJoints * i + j] = length;
        }
        /// <summary>
        /// Copies this instance.
        /// </summary>
        /// <returns></returns>
        internal link Copy()
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