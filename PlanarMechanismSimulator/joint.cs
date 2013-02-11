using System;

namespace PlanarMechanismSimulator
{
    public enum JointTypes
    {
        R,
        P,
        RP,
        G
        // non-slip roll, like rack and pinion - although this challenges the 2 DOF nature of just gear teeth
        // cabling or belt or chain
    };

    internal enum KnownState
    {
        Unknown,
        Partially,
        Fully
    };
    public class point
    {
        /// <summary>
        /// 
        /// </summary>
        public double x, y;
        public point(double x, double y)
        {
            this.x = x;
            this.y = y;
        }
    }

    public class joint
    {
        /// <summary>
        /// Is the joint connected to ground
        /// </summary>
        public Boolean isGround;
        /// <summary>
        /// The joint type
        /// </summary>
        public JointTypes jointType;
        /// <summary>
        /// The initial x-coordinate
        /// </summary>
        public double xInitial;
        /// <summary>
        /// The initial y-coordinate
        /// </summary>
        public double yInitial;
        /// <summary>
        /// The initial slide angle
        /// </summary>
       //public double InitSlideAngle = Double.NaN;
        public double InitSlideAngle = 0.0;
        internal double SlideAngle { get { return Link1.Angle + InitSlideAngle; } }
        internal double SlideVelocity { get; set; }
        internal double x { get; set; }
        internal double y { get; set; }
        internal double xNumerical { get; set; }
        internal double yNumerical { get; set; }
        internal double xLast { get; set; }
        internal double yLast { get; set; }

        internal double vx { get; set; }
        internal double vy { get; set; }
        internal double vxLast { get; set; }
        internal double vyLast { get; set; }

        internal double ax { get; set; }
        internal double ay { get; set; }



        /// <summary>
        /// Gets the link1.
        /// </summary>
        /// <value>
        /// The link1.
        /// </value>
        public link Link1 { get; internal set; }
        /// <summary>
        /// Gets the link2.
        /// </summary>
        /// <value>
        /// The link2.
        /// </value>
        public link Link2 { get; internal set; }


        internal KnownState positionKnown;


        internal joint(bool IsGround, string pTypeStr, double[] currentJointPosition = null)
        {
            isGround = IsGround;
            JointTypes pType;
            if (Enum.TryParse(pTypeStr, true, out pType)) jointType = pType;
            else throw new Exception("Unable to cast joint type " + pTypeStr + " as a recognized JointType.");

            if (currentJointPosition == null) return;
            if (currentJointPosition.GetLength(0) < 2)
                throw new Exception("Values for x and y must be provided for joint.");
            x = xInitial = xLast = xNumerical = currentJointPosition[0];
            y = yInitial = yLast = yNumerical = currentJointPosition[1];
            if (currentJointPosition.GetLength(0) >= 3 && jointType != JointTypes.R)
                InitSlideAngle = currentJointPosition[2];
            else if (jointType == JointTypes.P || jointType == JointTypes.RP)
            {
                
                InitSlideAngle = 0.0;
            }
                //throw new Exception("No slide angle provided for " + pTypeStr + " joint.");
        }

        private joint() { }

        public Boolean SlidingWithRespectTo(link link0)
        {
            return (Link1 == link0
                    && (jointType == JointTypes.P || jointType == JointTypes.RP
                        || (jointType == JointTypes.G && !Double.IsNaN(InitSlideAngle))));
        }

        public Boolean FixedWithRespectTo(link link0)
        {
            if (link0 != Link1 && link0 != Link2) return false;
            if (jointType == JointTypes.R) return true;
            if (jointType == JointTypes.G) return false;
            /* this business with Gear Connection is what prevents SlidingWithRespectTo and
             * FixedWithRespectTo being truly opposite. */

            /* then joint is either P or RP, so... */
            return (Link2 == link0);
        }

        internal link OtherLink(link thislink)
        {
            if (Link1 == thislink) return Link2;
            if (Link2 == thislink) return Link1;
            throw new Exception("the link provided to joint->OtherLink is not attached to this joint.");
        }

        public static implicit operator point(joint j)
        {
            return new point(j.x, j.y);
        }

        internal joint copy()
        {
            return new joint
            {
                InitSlideAngle = InitSlideAngle,
                Link1 = Link1,
                Link2 = Link2,
                x = x,
                xInitial = xInitial,
                xLast = xLast,
                xNumerical = xNumerical,
                y = y,
                yInitial = yInitial,
                yLast = yLast,
                yNumerical = yNumerical,
                isGround = isGround,
                jointType = jointType
            };
        }
    }

    internal enum PositionAnalysisResults { NoSolvableDyadFound, Normal, InvalidPosition, BranchingProbable }

}