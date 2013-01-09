using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PlanarMechanismSimulator.VelocityAndAcceleration
{
    internal class AccelerationEquationForDoubleSlide : AccelerationJointToJoint
    {
        private int slide1SpeedIndex = -1;
        private int slide2SpeedIndex = -1;

        internal AccelerationEquationForDoubleSlide(joint slide1Joint, joint slide2Joint, link link, bool slideJointIsKnown, bool fixedJointIsKnown)
            : base(slide1Joint, slide2Joint, link, slideJointIsKnown, fixedJointIsKnown) { }

        internal override double[] GetRow1Coefficients()
        {
            var coefficients = new double[unkLength];
            for (int i = 0; i < unkLength; i++)
            {
                if (i == joint1XIndex) coefficients[i] = -1;
                else if (i == joint2XIndex) coefficients[i] = 1;
                else if (i == linkIndex) coefficients[i] = (joint2.y - joint1.y);
                else if (i == slide1SpeedIndex) coefficients[i] = Math.Cos(joint1.SlideAngle);
                else if (i == slide2SpeedIndex) coefficients[i] = -Math.Cos(joint2.SlideAngle);
                else coefficients[i] = 0;
            }
            return coefficients;
        }
        internal override double[] GetRow2Coefficients()
        {
            var coefficients = new double[unkLength];
            for (int i = 0; i < unkLength; i++)
            {
                if (i == joint1YIndex) coefficients[i] = -1;
                else if (i == joint2YIndex) coefficients[i] = 1;
                else if (i == linkIndex) coefficients[i] = (joint1.x - joint2.x);
                else if (i == slide1SpeedIndex) coefficients[i] = Math.Sin(joint1.SlideAngle);
                else if (i == slide2SpeedIndex) coefficients[i] = -Math.Sin(joint2.SlideAngle);
                else coefficients[i] = 0;
            }
            return coefficients;
        }

        internal override double GetRow1Constant()
        {
            return -2 * joint1.SlideVelocity * Math.Sin(joint1.SlideAngle) * link.Velocity
                + 2 * joint2.SlideVelocity * Math.Sin(joint2.SlideAngle) * link.Velocity 
                + base.GetRow1Constant();
        }
        internal override double GetRow2Constant()
        {
            // need to add in the coriolis term
            return 2 * joint1.SlideVelocity * Math.Cos(joint1.SlideAngle) * link.Velocity  
                -2 * joint2.SlideVelocity * Math.Cos(joint2.SlideAngle) * link.Velocity 
                + base.GetRow2Constant();
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
                        else if (((Tuple<link, joint>)o).Item2 == joint2)
                            slide2SpeedIndex = index;
                    }
                }
                if (o is joint) index += 2;
                else index++;
            }
        }
    }
}
