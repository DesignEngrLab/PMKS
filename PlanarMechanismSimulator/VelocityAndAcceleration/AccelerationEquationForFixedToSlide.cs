using System;
using System.Collections.Generic;

namespace PMKS.VelocityAndAcceleration
{
    internal class AccelerationEquationForFixedToSlide : AccelerationJointToJoint
    {
        private int slideSpeedIndex = -1;
        internal AccelerationEquationForFixedToSlide(Joint slideJoint, Joint fixedJoint, Link link, bool slideJointIsKnown, bool fixedJointIsKnown)
            : base(slideJoint, fixedJoint, link, slideJointIsKnown, fixedJointIsKnown) { }

        internal override double[] GetRow1Coefficients()
        {
            var coefficients = new double[unkLength];
            for (int i = 0; i < unkLength; i++)
            {
                if (i == joint1XIndex) coefficients[i] = -1;
                else if (i == joint2XIndex) coefficients[i] = 1;
                else if (i == linkIndex) coefficients[i] = (joint2.Y - joint1.Y);
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
                else if (i == linkIndex) coefficients[i] = (joint1.X - joint2.X);
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
            return 2 * joint1.SlideVelocity * Math.Cos(joint1.SlideAngle) * link.Velocity + base.GetRow2Constant();
        }
        internal override void CaptureUnknownIndicies(List<object> unknownObjects)
        {
            base.CaptureUnknownIndicies(unknownObjects);
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o is Tuple<Link, Joint>
                    && ((Tuple<Link, Joint>)o).Item1 == link
                    && ((Tuple<Link, Joint>)o).Item2 == joint1)
                    slideSpeedIndex = index;
                if (o is Joint) index += 2;
                else index++;
            }
        }
        internal override List<int> GetRow1Indices()
        {
            var indices = base.GetRow1Indices();
            indices.Add(slideSpeedIndex);
            return indices;
        }


        internal override List<int> GetRow2Indices()
        {
            var indices = base.GetRow2Indices();
            indices.Add(slideSpeedIndex);
            return indices;
        }
    }
}
