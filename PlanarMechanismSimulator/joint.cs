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
        public Boolean isGround;
        public JointTypes jointType;
        public double xInitial;
        public double yInitial;
        public double InitSlideAngle = Double.NaN;
        internal double SlideAngle { get { return Link1.Angle + InitSlideAngle; } }

        internal double x { get; set; }
        internal double y { get; set; }
        internal double xNumerical { get; set; }
        internal double yNumerical { get; set; }
        internal double xLast { get; set; }
        internal double yLast { get; set; }
        public link Link1 { get; internal set; }
        public link Link2 { get; internal set; }
        // how to plan for future cam shapes
        internal KnownState knownState;
        // internal Boolean SlideLineIsUnknown = true;
        //   internal Boolean PositionIsUnknown = true;


        internal joint(bool IsGround, string pTypeStr, double[] currentJointPosition = null)
        {
            isGround = IsGround;
            JointTypes pType;
            if (Enum.TryParse(pTypeStr, true, out pType)) jointType = pType;
            else throw new Exception("Unable to cast joint type " + pTypeStr + " as a recognized JointType.");

            if (currentJointPosition == null) return;
            if (currentJointPosition.GetLength(0) < 2)
                throw new Exception("Values for x and y must be provided for joint.");
            xInitial = currentJointPosition[0];
            yInitial = currentJointPosition[1];
            if (currentJointPosition.GetLength(0) >= 3 && jointType != JointTypes.R)
                InitSlideAngle = currentJointPosition[2];
            else if (jointType == JointTypes.P || jointType == JointTypes.RP)
                throw new Exception("No slide angle provided for " + pTypeStr + " joint.");
        }

        private joint(){}

        public Boolean SlidingWithRespectTo(link link0)
        {
            return (Link1 == link0
                    && (jointType == JointTypes.P || jointType == JointTypes.RP
                        || (jointType == JointTypes.G && !Double.IsNaN(InitSlideAngle))));
        }

        public Boolean FixedWithRespectTo(link link0)
        {
            if (jointType == JointTypes.R) return true;
            if (jointType == JointTypes.G) return false;
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