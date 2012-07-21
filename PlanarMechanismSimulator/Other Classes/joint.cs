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
                    //if (currentJointPosition.GetLength(0) == 0 || currentJointPosition.GetLength(0) == 2)
                    if (currentJointPosition.GetLength(0) < 3)
                        throw new Exception("No slide angle provided for " + pTypeStr + " joint.");
                    initX = currentJointPosition[0];
                    initY = currentJointPosition[1];
                    SlideAngle = currentJointPosition[2];
                }
                else
                {
                    if (currentJointPosition.GetLength(0) >= 2)
                    {
                        initX = currentJointPosition[0];
                        initY = currentJointPosition[1];
                    }
                }
            }
        }

        public Boolean LinkIsSlide(link link0)
        {
            return (Link1 == link0
                && (jointType == JointTypes.P || jointType == JointTypes.RP
                || (jointType == JointTypes.G && double.IsNaN(SlideAngle))));
        }
    }
}