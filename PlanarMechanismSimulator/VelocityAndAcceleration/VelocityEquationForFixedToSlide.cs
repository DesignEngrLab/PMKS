using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PlanarMechanismSimulator.VelocityAndAcceleration
{
    internal class VelocityEquationForFixedToSlide : VelocityJointToJoint
    {
        private int slideSpeedIndex = -1;
        internal VelocityEquationForFixedToSlide(joint slideJoint, joint fixedJoint, link link, bool slideJointIsKnown, bool fixedJointIsKnown, bool linkIsKnown)
            : base(slideJoint, fixedJoint, link, slideJointIsKnown, fixedJointIsKnown, linkIsKnown) { }

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
