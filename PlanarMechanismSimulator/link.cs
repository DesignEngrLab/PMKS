// ***********************************************************************
// Assembly         : PlanarMechanismKinematicSimulator
// Author           : Matt
// Created          : 06-10-2015
//
// Last Modified By : Matt
// Last Modified On : 06-28-2015
// ***********************************************************************
// <copyright file="link.cs" company="">
//     Copyright ©  2014
// </copyright>
// <summary></summary>
// ***********************************************************************

using System;
using System.Collections.Generic;
using System.Linq;
using StarMathLib;

namespace PMKS
{
    /// <summary>
    /// Class link.
    /// </summary>
    internal class Link
    {
        /// <summary>
        /// The shortest distance between a joint and the line of a sliding joint
        /// </summary>
        private Dictionary<int, double> angleFromBlockToJoint;

        /// <summary>
        /// The angle is known
        /// </summary>
        internal KnownState AngleIsKnown;

        /// <summary>
        /// The shortest distance between a joint and the line of a sliding joint
        /// </summary>
        private Dictionary<int, double> distanceToSlideLine;

        /// <summary>
        /// The lengths between joints that are fixed with respect to this link
        /// </summary>
        private Dictionary<int, double> lengths;

        /// <summary>
        /// The number joints
        /// </summary>
        private int numJoints;

        /// <summary>
        /// Initializes a new instance of the <see cref="Link"/> class.
        /// </summary>
        /// <param name="name">The name.</param>
        /// <param name="joints">The joints.</param>
        /// <param name="IsGround">The is ground.</param>
        internal Link(string name, IEnumerable<Joint> joints) : this()
        {
            this.Joints.AddRange(joints);
            if (Simulator.IsGroundLinkName(name))
            {
                this.Name = "ground";
                IsGround = true;
            }
            else this.Name = name;
        }

        /// <summary>
        /// Prevents a default instance of the <see cref="Link"/> class from being created.
        /// </summary>
        private Link()
        {
            Joints = new List<Joint>();
        }

        /// <summary>
        /// Gets the joints.
        /// </summary>
        /// <value>The joints.</value>
        internal List<Joint> Joints { get; private set; }

        /// <summary>
        /// Is the link the ground-link?
        /// </summary>
        /// <value>The is ground.</value>
        internal Boolean IsGround { get; set; }

        /// <summary>
        /// Gets or sets the reference joint1.
        /// </summary>
        /// <value>The reference joint1.</value>
        internal Joint ReferenceJoint1 { get; set; }

        /// <summary>
        /// Gets the maximum length.
        /// </summary>
        /// <value>The maximum length.</value>
        internal double MaxLength
        {
            get { return lengths.Values.Max(v => Math.Abs(v)); }
        }

        /// <summary>
        /// Gets the sum of all the lengths between the pivots.
        /// </summary>
        /// <value>The total length.</value>
        internal double TotalLength
        {
            get { return lengths.Values.Sum(v => Math.Abs(v)); }
        }

        /// <summary>
        /// Gets the name.
        /// </summary>
        /// <value>The name.</value>
        internal string Name { get; private set; }
        /// <summary>
        /// Gets the angle initial.
        /// </summary>
        /// <value>The angle initial.</value>
        internal double AngleInitial { get; set; }
        /// <summary>
        /// Gets or sets the angle.
        /// </summary>
        /// <value>The angle.</value>
        internal double Angle { get; set; }
        /// <summary>
        /// Gets or sets the angle last.
        /// </summary>
        /// <value>The angle last.</value>
        internal double AngleLast { get; set; }
        /// <summary>
        /// Gets or sets the angle numerical.
        /// </summary>
        /// <value>The angle numerical.</value>
        internal double AngleNumerical { get; set; }
        /// <summary>
        /// Gets or sets the velocity.
        /// </summary>
        /// <value>The velocity.</value>
        internal double Velocity { get; set; }
        /// <summary>
        /// Gets or sets the velocity last.
        /// </summary>
        /// <value>The velocity last.</value>
        internal double VelocityLast { get; set; }
        /// <summary>
        /// Gets or sets the acceleration.
        /// </summary>
        /// <value>The acceleration.</value>
        internal double Acceleration { get; set; }

        /// <summary>
        /// Sets the length between the two joints. This function is only called from Simulator.AssignLengths
        /// and is to be used only when the solving a problem of construct-ability (or whatever they call it in
        /// the literature).
        /// </summary>
        /// <param name="joint1">The joint1.</param>
        /// <param name="joint2">The joint2.</param>
        /// <param name="length">The length.</param>
        /// <exception cref="System.Exception">link.setLength: cannot set the distance between joints because same joint provided
        /// as both joint1 and joint2.</exception>
        internal void SetLength(Joint joint1, Joint joint2, double length)
        {
            if (joint1 == joint2)
                throw new Exception(
                    "link.setLength: cannot set the distance between joints because same joint provided as both joint1 and joint2.");
            var i = Joints.IndexOf(joint1);
            var j = Joints.IndexOf(joint2);
            if (i > j) lengths[numJoints * j + i] = length;
            else lengths[numJoints * i + j] = length;
        }

        /// <summary>
        /// Copies this instance.
        /// </summary>
        /// <returns>link.</returns>
        internal Link Copy()
        {
            return new Link
            {
                Angle = Angle,
                AngleInitial = AngleInitial,
                AngleLast = AngleLast,
                AngleIsKnown = AngleIsKnown,
                AngleNumerical = AngleNumerical,
                numJoints = numJoints,
                lengths = new Dictionary<int, double>(lengths),
                distanceToSlideLine = new Dictionary<int, double>(distanceToSlideLine),
                Name = Name,
                angleFromBlockToJoint = new Dictionary<int, double>(angleFromBlockToJoint)
            };
        }

        #region Determine Lengths And References and Initializing Functions

        /// <summary>
        /// Determines the lengths and references.
        /// </summary>
        internal void DetermineLengthsAndReferences()
        {
            numJoints = Joints.Count;

            #region Define Initial Link Angle

            var fixedJoints = Joints.Where(j => j.FixedWithRespectTo(this)).
                OrderBy(j => (j.IsGround) ? 0 : 1).
                ThenBy(j => (j.Link2 != null) ? 0 : 1).ThenBy(j => j.XInitial).ToList();
            ReferenceJoint1 = fixedJoints[0];
            /* the linkAngle is defined from "the joint with the lowest initial x value
             * that is fixed to this link" to "the joint with the highest initial x value
             * that is fixed to this link. If the link has only one fixed joint (and
             * it will necessarily have one due to the addition of a joint added in
             * Simulator.addReferencePivotsToSlideOnlyLinks()) or is ground, then the 
             * initial angle is zero. */
            if (fixedJoints.Count == 2 && !IsGround)
                AngleInitial = Constants.Angle(ReferenceJoint1, fixedJoints[1]);
            else AngleInitial = 0.0;
            Angle = AngleNumerical = AngleLast = AngleInitial;
            foreach (var j in Joints.Where(j => j.SlidingWithRespectTo(this)))
                j.OffsetSlideAngle -= AngleInitial;

            #endregion

            #region Joint-to-Joint Dictionaries

            lengths = new Dictionary<int, double>();
            distanceToSlideLine = new Dictionary<int, double>();
            angleFromBlockToJoint = new Dictionary<int, double>();

            for (var i = 0; i < Joints.Count - 1; i++)
                for (var j = i + 1; j < Joints.Count; j++)
                {
                    var iJoint = Joints[i];
                    var jJoint = Joints[j];
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
                        AddSlideDictionaryEntry(iJoint, jJoint, i, j);
                        AddBlockAngleDictionaryEntry(iJoint, jJoint, i, j);
                        if (jJoint.TypeOfJoint == JointType.P)
                        {
                            AddBlockAngleDictionaryEntry(jJoint, iJoint, j, i);
                            AddSlideDictionaryEntry(jJoint, iJoint, j, i);
                        }
                    }
                    else if (!iJoint.SlidingWithRespectTo(this) && jJoint.SlidingWithRespectTo(this))
                    {
                        AddSlideDictionaryEntry(jJoint, iJoint, j, i);
                        AddBlockAngleDictionaryEntry(jJoint, iJoint, j, i);
                        if (iJoint.TypeOfJoint == JointType.P)
                        {
                            AddBlockAngleDictionaryEntry(iJoint, jJoint, i, j);
                            AddSlideDictionaryEntry(iJoint, jJoint, i, j);
                        }
                    }
                    else //    if (!iJoint.SlidingWithRespectTo(this) && !jJoint.SlidingWithRespectTo(this))
                    {
                        lengths.Add(numJoints * i + j,
                            Constants.Distance(iJoint.XInitial, iJoint.YInitial, jJoint.XInitial, jJoint.YInitial));
                        if (iJoint.TypeOfJoint == JointType.P)
                        {
                            AddBlockAngleDictionaryEntry(iJoint, jJoint, i, j);
                            AddSlideDictionaryEntry(iJoint, jJoint, i, j);
                        }
                        if (jJoint.TypeOfJoint == JointType.P)
                        {
                            AddBlockAngleDictionaryEntry(jJoint, iJoint, j, i);
                            AddSlideDictionaryEntry(jJoint, iJoint, j, i);
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
        private void AddSlideDictionaryEntry(Joint slideJoint, Joint refJoint, int slideIndex, int fixedIndex)
        {
            var key = numJoints * slideIndex + fixedIndex;
            var slideUnitVector = new[] { Math.Cos(slideJoint.SlideAngle), Math.Sin(slideJoint.SlideAngle) };
            var refVector = new[] { refJoint.XInitial - slideJoint.XInitial, refJoint.YInitial - slideJoint.YInitial };
            var distance = (double.IsNaN(slideJoint.SlideAngle))
                ? StarMath.norm2(refVector)
                : StarMath.crossProduct2(slideUnitVector, refVector);
            distanceToSlideLine.Add(key, distance);
        }

        /// <summary>
        /// Adds the block angle dictionary entry.
        /// </summary>
        /// <param name="pJoint">The p joint.</param>
        /// <param name="refJoint">The reference joint.</param>
        /// <param name="pIndex">Index of the p.</param>
        /// <param name="refIndex">Index of the reference.</param>
        private void AddBlockAngleDictionaryEntry(Joint pJoint, Joint refJoint, int pIndex, int refIndex)
        {
            var key = numJoints * pIndex + refIndex;
            var result = (pJoint.TypeOfJoint == JointType.G)
                ? Constants.QuarterCircle
                : pJoint.SlideAngleInitial -
                  Constants.Angle(pJoint.XInitial, pJoint.YInitial, refJoint.XInitial, refJoint.YInitial);
            angleFromBlockToJoint.Add(key, result);
        }

        #endregion

        #region Retrieve Joint-to-Joint data

        /// <summary>
        /// Returns the Lengths the between two joints that are fixed with respect to this link.
        /// </summary>
        /// <param name="joint1">joint1.</param>
        /// <param name="joint2">joint2.</param>
        /// <returns>System.Double.</returns>
        internal double LengthBetween(Joint joint1, Joint joint2)
        {
            if (joint1 == joint2) return 0.0;
            var i = Joints.IndexOf(joint1);
            var j = Joints.IndexOf(joint2);
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
        /// <returns>System.Double.</returns>
        internal double DistanceBetweenSlides(Joint slidingJoint, Joint referenceJoint)
        {
            if (slidingJoint == referenceJoint) return 0.0;
            var slideIndex = Joints.IndexOf(slidingJoint);
            var fixedIndex = Joints.IndexOf(referenceJoint);
            var index = numJoints * slideIndex + fixedIndex;
            if (distanceToSlideLine.ContainsKey(index))
                return distanceToSlideLine[index];
            return distanceToSlideLine[numJoints * fixedIndex + slideIndex];
        }

        /// <summary>
        /// returns the angle between the slide and the reference joint.
        /// </summary>
        /// <param name="blockJoint">The block joint.</param>
        /// <param name="referenceJoint">The reference joint.</param>
        /// <returns>System.Double.</returns>
        internal double AngleOfBlockToJoint(Joint blockJoint, Joint referenceJoint)
        {
            if (blockJoint == referenceJoint) return 0.0;
            var i = Joints.IndexOf(blockJoint);
            var j = Joints.IndexOf(referenceJoint);
            return angleFromBlockToJoint[numJoints * i + j];
        }

        #endregion
    }
}