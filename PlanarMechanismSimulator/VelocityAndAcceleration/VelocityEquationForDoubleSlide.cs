using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PlanarMechanismSimulator.VelocityAndAcceleration
{
    internal class VelocityEquationForDoubleSlide : JointToJointEquation
    {
        private int slide1SpeedIndex;
        private int slide2SpeedIndex;

        internal VelocityEquationForDoubleSlide(joint slideJoint, joint fixedJoint, link link, bool slideJointIsKnown, bool fixedJointIsKnown)
            : base(slideJoint,fixedJoint,link,slideJointIsKnown,fixedJointIsKnown) { }

        internal override double[] GetRow1Coefficients()
        {
            for (int i = 0; i < unkLength; i++)
            {
                if (i == joint1XIndex) coefficients[i] = -1;
                else if (i == joint2XIndex) coefficients[i] = 1;
                else if (i == linkOmegaIndex) coefficients[i] = (joint2.y - joint1.y);
                else if (i == slide1SpeedIndex) coefficients[i] = Math.Cos(joint1.SlideAngle);
                else if (i == slide2SpeedIndex) coefficients[i] = -Math.Cos(joint2.SlideAngle);
                else coefficients[i] = 0;
            }
            return coefficients;
        }
        internal override double[] GetRow2Coefficients()
        {
            for (int i = 0; i < unkLength; i++)
            {
                if (i == joint1YIndex) coefficients[i] = -1;
                else if (i == joint2YIndex) coefficients[i] = 1;
                else if (i == linkOmegaIndex) coefficients[i] = (joint1.x - joint2.x);
                else if (i == slide1SpeedIndex) coefficients[i] = Math.Sin(joint1.SlideAngle);
                else if (i == slide2SpeedIndex) coefficients[i] = -Math.Sin(joint2.SlideAngle);
                else coefficients[i] = 0;
            }
            return coefficients;
        }

        internal override double GetRow1Constant()
        {
            if (joint1IsKnown) return joint1.vx;
            if (joint2IsKnown) return -1 * joint2.vx;
            return 0.0;
        }
        internal override double GetRow2Constant()
        {
            if (joint1IsKnown) return joint1.vy;
            if (joint2IsKnown) return -1 * joint2.vy;
            return 0.0;
        }
        internal override void CaptureUnknownIndicies(List<object> unknownObjects)
        {
            base.CaptureUnknownIndicies(unknownObjects);
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o is Tuple<link, joint>)
                {
                    if (((Tuple<link, joint>)o).Item1 == link)
                    {
                        if (((Tuple<link, joint>)o).Item2 == joint1)
                            slide1SpeedIndex = index;
                        else  if (((Tuple<link, joint>)o).Item2 == joint2)
                            slide2SpeedIndex = index;
                    }
                }
                if (o is joint) index += 2;
                else index++;
            }
        }
    }
}
