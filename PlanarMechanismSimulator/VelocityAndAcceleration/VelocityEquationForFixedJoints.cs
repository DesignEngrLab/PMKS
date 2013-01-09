using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PlanarMechanismSimulator.VelocityAndAcceleration
{
    internal class VelocityEquationForFixedJoints : JointToJointEquation
    {

        internal VelocityEquationForFixedJoints(joint joint1, joint joint2, link link, bool Joint1IsKnown, bool Joint2IsKnown)
            : base(joint1, joint2, link, Joint1IsKnown, Joint2IsKnown) { }

        internal override double[] GetRow1Coefficients()
        {
            for (int i = 0; i < unkLength; i++)
            {
                if (i == joint1XIndex) coefficients[i] = -1;
                else if (i == joint2XIndex) coefficients[i] = 1;
                else if (i == linkOmegaIndex) coefficients[i] = (joint2.y - joint1.y);
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
    }
}
