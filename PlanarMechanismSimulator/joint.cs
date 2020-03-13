// ***********************************************************************
// Assembly         : PlanarMechanismKinematicSimulator
// Author           : Matt
// Created          : 06-28-2015
//
// Last Modified By : Matt
// Last Modified On : 06-28-2015
// ***********************************************************************
// <copyright file="joint.cs" company="">
//     Copyright ©  2014
// </copyright>
// <summary></summary>
// ***********************************************************************
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;
using Newtonsoft.Json.Linq;
using System;
using System.Collections.Generic;
using System.Runtime.Serialization;

namespace PMKS
{
    /// <summary>
    /// The point class is used internally to easily convert joints to a simple 2D point.
    /// </summary>
    public struct Point
    {
        /// <summary>
        /// The x coordinate.
        /// </summary>
        public double X;

        /// <summary>
        /// The y coordinate.
        /// </summary>
        public double Y;

        /// <summary>
        /// Initializes a new instance of the <see cref="Point" /> struct.
        /// </summary>
        /// <param name="x">The x.</param>
        /// <param name="y">The y.</param>
        public Point(double x, double y)
        {
            X = x;
            Y = y;
        }
    }

    /// <summary>
    /// The joint class, which represents the relationship between 2 links.
    /// </summary>
    public class Joint
    {
        /// <summary>
        /// The _offset slide angle
        /// </summary>
        private double _offsetSlideAngle = double.NaN;
        /// <summary>
        /// The position known
        /// </summary>
        internal KnownState positionKnown;

        /// <summary>
        /// Initializes a new instance of the <see cref="Joint" /> class.
        /// </summary>
        /// <param name="isGround">if set to <c>true</c> [is ground].</param>
        /// <param name="jointType">Type of the joint.</param>
        /// <param name="currentJointPosition">The current joint position.</param>
        /// <param name="specifiedAs">The specified as.</param>
        /// <exception cref="System.Exception">Values for x and y must be provided for joint.</exception>
        internal Joint(bool isGround, JointType jointType, double[] currentJointPosition = null,
            JointSpecifiedAs specifiedAs = JointSpecifiedAs.Design)
        {
            IsGround = isGround;
            TypeOfJoint = jointType;
            JointSpecifiedAs = specifiedAs;

            if (currentJointPosition == null) return;
            if (currentJointPosition.GetLength(0) < 2)
                throw new Exception("Values for x and y must be provided for joint.");
            X = XInitial = XLast = XNumerical = currentJointPosition[0];
            Y = YInitial = YLast = YNumerical = currentJointPosition[1];
            if (currentJointPosition.GetLength(0) >= 3 && jointType != JointType.R)
                OffsetSlideAngle = currentJointPosition[2];
            else if (jointType == JointType.P || jointType == JointType.RP)
                OffsetSlideAngle = 0.0;

            while (OffsetSlideAngle > Constants.QuarterCircle) OffsetSlideAngle -= Math.PI;
            while (OffsetSlideAngle < -Constants.QuarterCircle) OffsetSlideAngle += Math.PI;
        }

        /// <summary>
        /// Prevents a default instance of the <see cref="Joint"/> class from being created.
        /// </summary>
        private Joint()
        {
        }
        
        /// <summary>
        /// Initializes a new instance of the <see cref="Joint"/> class.
        /// This is only used for specifying a second copy of an R-joint that joins more than two links.
        /// </summary>
        /// <param name="joint">The joint.</param>
        internal Joint(Joint joint)
            : this(joint.IsGround, joint.TypeOfJoint, new[] { joint.XInitial, joint.YInitial }, JointSpecifiedAs.TernaryJoint)
        {
            sameJointAs = joint;
        }

        /// <summary>
        /// Is the joint connected to ground
        /// </summary>
        /// <value>The is ground.</value>
        [JsonIgnore]
        public Boolean IsGround { get; internal set; }

        internal JointSpecifiedAs JointSpecifiedAs { get; private set; }
        internal Joint sameJointAs;

        /// <summary>
        /// The joint type
        /// </summary>
        /// <value>The type of joint.</value>
        [JsonConverter(typeof(StringEnumConverter))]
        public JointType TypeOfJoint { get; set; }

        /// <summary>
        /// The initial x-coordinate
        /// </summary>
        /// <value>The x initial.</value>
        [JsonProperty(PropertyName = "X")]
        public double XInitial { get; set; }

        /// <summary>
        /// The initial y-coordinate
        /// </summary>
        /// <value>The y initial.</value>
        [JsonProperty(PropertyName = "Y")]
        public double YInitial { get; set; }

        /// <summary>
        /// The initial slide angle
        /// </summary>
        /// <value>The offset slide angle.</value>
         [JsonProperty(PropertyName = "Angle")]
        public double OffsetSlideAngle
        {
            get { return _offsetSlideAngle; }
            set { _offsetSlideAngle = value; }
        }

        //internal double OffsetSlideAngle = 0.0;
        /// <summary>
        /// Gets the slide angle initial.
        /// </summary>
        /// <value>The slide angle initial.</value>
        [JsonIgnore]
        public double SlideAngleInitial
        {
            get { return Link1.AngleInitial + OffsetSlideAngle; }
        }

        /// <summary>
        /// Gets the slide angle.
        /// </summary>
        /// <value>The slide angle.</value>
        internal double SlideAngle
        {
            get { return Link1.Angle + OffsetSlideAngle; }
        }

        /// <summary>
        /// Gets or sets the slide position.
        /// </summary>
        /// <value>The slide position.</value>
        internal double SlidePosition { get; set; }
        /// <summary>
        /// Gets or sets the slide velocity.
        /// </summary>
        /// <value>The slide velocity.</value>
        internal double SlideVelocity { get; set; }
        /// <summary>
        /// Gets or sets the slide acceleration.
        /// </summary>
        /// <value>The slide acceleration.</value>
        internal double SlideAcceleration { get; set; }
        //internal double[] SlideLimits { get; internal set; }
        /// <summary>
        /// Gets or sets the x.
        /// </summary>
        /// <value>The x.</value>
        internal double X { get; set; }
        /// <summary>
        /// Gets or sets the y.
        /// </summary>
        /// <value>The y.</value>
        internal double Y { get; set; }
        /// <summary>
        /// Gets or sets the x numerical.
        /// </summary>
        /// <value>The x numerical.</value>
        internal double XNumerical { get; set; }
        /// <summary>
        /// Gets or sets the y numerical.
        /// </summary>
        /// <value>The y numerical.</value>
        internal double YNumerical { get; set; }
        /// <summary>
        /// Gets or sets the x last.
        /// </summary>
        /// <value>The x last.</value>
        internal double XLast { get; set; }
        /// <summary>
        /// Gets or sets the y last.
        /// </summary>
        /// <value>The y last.</value>
        internal double YLast { get; set; }
        /// <summary>
        /// Gets or sets the vx.
        /// </summary>
        /// <value>The vx.</value>
        internal double Vx { get; set; }
        /// <summary>
        /// Gets or sets the vy.
        /// </summary>
        /// <value>The vy.</value>
        internal double Vy { get; set; }
        /// <summary>
        /// Gets or sets the vx last.
        /// </summary>
        /// <value>The vx last.</value>
        internal double VxLast { get; set; }
        /// <summary>
        /// Gets or sets the vy last.
        /// </summary>
        /// <value>The vy last.</value>
        internal double VyLast { get; set; }
        /// <summary>
        /// Gets or sets the ax.
        /// </summary>
        /// <value>The ax.</value>
        internal double Ax { get; set; }
        /// <summary>
        /// Gets or sets the ay.
        /// </summary>
        /// <value>The ay.</value>
        internal double Ay { get; set; }

        /// <summary>
        /// Gets the link1.
        /// </summary>
        /// <value>The link1.</value>
        [JsonIgnore]
        public Link Link1 { get;  set; }

        /// <summary>
        /// Gets the link2.
        /// </summary>
        /// <value>The link2.</value>
        [JsonIgnore]
        public Link Link2 { get; set; }

        [JsonRequired]
        public string[] Links { get; private set; }

        [OnSerializing]
        internal void OnSerializingMethod(StreamingContext context)
        {
            if (Link2 == null) Links = new[] { Link1.Name.Trim().ToLower() };
            else Links = new[] { Link1.Name.Trim().ToLower(), Link2.Name.Trim().ToLower() };
        }

        /// <summary>
        /// Gets the minimum slide position.
        /// </summary>
        /// <value>The minimum slide position.</value>
        public double MinSlidePosition { get; internal set; }
        /// <summary>
        /// Gets the maximum slide position.
        /// </summary>
        /// <value>The maximum slide position.</value>
        public double MaxSlidePosition { get; internal set; }
        /// <summary>
        /// Gets the original slide position.
        /// </summary>
        /// <value>The original slide position.</value>
        public double OrigSlidePosition { get; internal set; }

        // everything else gets stored here
        [JsonExtensionData]
        private IDictionary<string, JToken> _additionalData;
        /// <summary>
        /// Gets the just a tracer.
        /// </summary>
        /// <value>The just a tracer.</value>
        public Boolean JustATracer
        {
            get { return (Link2 == null && this != Link1.ReferenceJoint1); }
        }

        /// <summary>
        /// Slidings the with respect to.
        /// </summary>
        /// <param name="link0">The link0.</param>
        /// <returns>Boolean.</returns>
        internal Boolean SlidingWithRespectTo(Link link0)
        {
            return !FixedWithRespectTo(link0);
            //return (Link1 == link0
            //        && (jointType == JointTypes.P || jointType == JointTypes.RP
            //            || (jointType == JointTypes.G && !Double.IsNaN(OffsetSlideAngle))));
        }

        /// <summary>
        /// Fixeds the with respect to.
        /// </summary>
        /// <param name="link0">The link0.</param>
        /// <returns>Boolean.</returns>
        public Boolean FixedWithRespectTo(Link link0)
        {
            if (link0 != Link1 && link0 != Link2) return false;
            //throw new Exception("link0 is not connected to joint (in joint.FixedWithRespectTo).");
            if (TypeOfJoint == JointType.R) return true;
            if (TypeOfJoint == JointType.G) return false;
            /* then joint is either P or RP, so... */
            return (Link2 == link0);
        }

        /// <summary>
        /// Others the link.
        /// </summary>
        /// <param name="thislink">The thislink.</param>
        /// <returns>link.</returns>
        /// <exception cref="System.Exception">the link provided to joint->OtherLink is not attached to this joint.</exception>
        internal Link OtherLink(Link thislink)
        {
            if (Link1 == thislink) return Link2;
            if (Link2 == thislink) return Link1;
            throw new Exception("the link provided to joint->OtherLink is not attached to this joint.");
        }

        /// <summary>
        /// Performs an implicit conversion from <see cref="Joint"/> to <see cref="Point"/>.
        /// </summary>
        /// <param name="j">The j.</param>
        /// <returns>The result of the conversion.</returns>
        public static implicit operator Point(Joint j)
        {
            return new Point(j.X, j.Y);
        }

        /// <summary>
        /// Copies this instance.
        /// </summary>
        /// <returns>joint.</returns>
        internal Joint Copy()
        {
            return new Joint
            {
                Link1 = Link1,
                Link2 = Link2,
                OffsetSlideAngle = OffsetSlideAngle,
                X = X,
                XInitial = XInitial,
                XLast = XLast,
                XNumerical = XNumerical,
                Y = Y,
                YInitial = YInitial,
                YLast = YLast,
                YNumerical = YNumerical,
                IsGround = IsGround,
                TypeOfJoint = TypeOfJoint,
                Ax = Ax,
                Ay = Ay,
                SlideAcceleration = SlideAcceleration,
                positionKnown = positionKnown,
                Vx = Vx,
                Vy = Vy,
                VxLast = VxLast,
                VyLast = VyLast,
                SlideVelocity = SlideVelocity
            };
        }
    }
}