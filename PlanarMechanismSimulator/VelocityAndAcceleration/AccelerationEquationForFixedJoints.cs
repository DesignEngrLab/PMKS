using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PlanarMechanismSimulator.VelocityAndAcceleration
{
    internal class AccelerationEquationForFixedJoints : AccelerationJointToJoint
    {

        internal AccelerationEquationForFixedJoints(joint joint1, joint joint2, link link, bool Joint1IsKnown, bool Joint2IsKnown)
            : base(joint1, joint2, link, Joint1IsKnown, Joint2IsKnown) { }

        internal override double[] GetRow1Coefficients()
        {
            var coefficients = new double[unkLength];
            for (int i = 0; i < unkLength; i++)
            {
                if (i == joint1XIndex) coefficients[i] = -1;
                else if (i == joint2XIndex) coefficients[i] = 1;
                else if (i == linkIndex) coefficients[i] = (joint2.y - joint1.y);
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
                else coefficients[i] = 0;
            }
            return coefficients;
        }

    }
}
