using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PlanarMechanismSimulator.VelocityAndAcceleration
{
    internal class AccelerationEquationForFixedToSlide : AccelerationJointToJoint
    {
        private int slideSpeedIndex = -1;
        internal AccelerationEquationForFixedToSlide(joint slideJoint, joint fixedJoint, link link, bool slideJointIsKnown, bool fixedJointIsKnown)
            : base(slideJoint, fixedJoint, link, slideJointIsKnown, fixedJointIsKnown) { }

        internal override double[] GetRow1Coefficients()
        {
            var coefficients = new double[unkLength];
            for (int i = 0; i < unkLength; i++)
            {
                if (i == joint1XIndex) coefficients[i] = -1;
                else if (i == joint2XIndex) coefficients[i] = 1;
                else if (i == linkIndex) coefficients[i] = (joint2.y - joint1.y);
                else if (i == slideSpeedIndex) coefficients[i] = Math.Cos(joint1.SlideAngle);
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
                else if (i == slideSpeedIndex) coefficients[i] = Math.Sin(joint1.SlideAngle);
                else coefficients[i] = 0;
            }
            return coefficients;
        }

        internal override double GetRow1Constant()
        {
            return -2 * joint1.SlideVelocity * Math.Sin(joint1.SlideAngle) * link.Velocity + base.GetRow1Constant();
        }
        internal override double GetRow2Constant()
        {
            // need to add in the coriolis term
            return 2 * joint1.SlideVelocity * Math.Cos(joint1.SlideAngle) * link.Velocity + base.GetRow2Constant();
        }
        internal override void CaptureUnknownIndicies(List<object> unknownObjects)
        {
            base.CaptureUnknownIndicies(unknownObjects);
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o is Tuple<link, joint>
                    && ((Tuple<link, joint>)o).Item1 == link
                    && ((Tuple<link, joint>)o).Item2 == joint1)
                    slideSpeedIndex = index;
                if (o is joint) index += 2;
                else index++;
            }
        }
    }
}
