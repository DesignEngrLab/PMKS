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
    public struct point
    {
        /// <summary>
        /// 
        /// </summary>
        public double X, Y;
        public point(double x, double y)
        {
            X = x;
            Y = y;
        }
    }
    public class joint
    {
        public readonly Boolean isGround;
        public readonly JointTypes jointType;
        public double initX;
        public double initY;

        public link Link1 { get; internal set; }
        public link Link2 { get; internal set; }
        public double SlideAngle = double.NaN;
        // how to plan for future cam shapes

        internal Boolean AngleIsUnknown;
        internal Boolean PositionIsUnknown;


        internal joint(bool IsGround, string pTypeStr, double[] currentJointPosition = null)
        {
            isGround = IsGround;
            JointTypes pType;
            if (Enum.TryParse(pTypeStr, true, out pType)) jointType = pType;
            else throw new Exception("Unable to cast joint type " + pTypeStr + " as a recognized JointType.");

            if (currentJointPosition == null) return;
            if (currentJointPosition.GetLength(0) < 2)
                throw new Exception("Values for x and y must be provided for joint.");
            initX = currentJointPosition[0];
            initY = currentJointPosition[1];
            if (currentJointPosition.GetLength(0) >= 3 && jointType != JointTypes.R)
                SlideAngle = currentJointPosition[2];
            else if (jointType == JointTypes.P || jointType == JointTypes.RP)
                throw new Exception("No slide angle provided for " + pTypeStr + " joint.");
        }

        public Boolean SlidingWithRespectToLink(link link0)
        {
            return (Link1 == link0
                && (jointType == JointTypes.P || jointType == JointTypes.RP
                || (jointType == JointTypes.G && double.IsNaN(SlideAngle))));
        }
        public Boolean FixedWithRespectToLink(link link0)
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
    }
}