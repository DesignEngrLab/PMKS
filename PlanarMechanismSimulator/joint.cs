using System;
using System.Linq;
using System.Collections.Generic;

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
    public class point
    {
        public double X { get; set; }
        public double Y { get; set; }
    }
    public class joint : point
    {
        public readonly Boolean isGround;
        public readonly JointTypes jointType;

        public link Link1 { get; internal set; }
        public link Link2 { get; internal set; }
        public double SlideAngle { get; set; }
        // how to plan for future cam shapes


        internal joint(bool IsGround, string pTypeStr, double[] currentJointPosition)
        {
            isGround = IsGround;
            JointTypes pType;
            if (Enum.TryParse(pTypeStr, true, out pType)) jointType = pType;
            else throw new Exception("Unable to cast joint type " + pTypeStr + " as a recognized JointType.");

            if (currentJointPosition != null)
            {
                if (jointType == JointTypes.P || jointType == JointTypes.RP)
                {
                    if (currentJointPosition.GetLength(0) == 0 || currentJointPosition.GetLength(0) == 2)
                        throw new Exception("No slide angle provided for " + pTypeStr + " joint.");
                    else SlideAngle = currentJointPosition[0];
                    if (currentJointPosition.GetLength(0) >= 3)
                    {
                        X = currentJointPosition[1];
                        Y = currentJointPosition[2];
                    }
                }
                else
                {
                    if (currentJointPosition.GetLength(0) >= 2)
                    {
                        X = currentJointPosition[0];
                        Y = currentJointPosition[1];
                    }
                }
            }
        }

        public Boolean LinkIsSlide(link link0)
        {
            return ((jointType == JointTypes.P || jointType == JointTypes.RP) && Link1 == link0);
        }
    }
}