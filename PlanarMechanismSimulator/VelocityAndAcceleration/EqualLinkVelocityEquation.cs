using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PlanarMechanismSimulator.VelocityAndAcceleration
{
    internal class EqualLinkVelocityEquation : LinkToLinkEquation
    {
        internal EqualLinkVelocityEquation(link link1, link link2) : base(link1, link2)
        {
        }
        internal double[] GetRowCoefficients()
        {
            for (int i = 0; i < unkLength; i++)
            {
                if (i == link1OmegaIndex) coefficients[i] = -1;
                else if (i == link2OmegaIndex) coefficients[i] = 1;
                else coefficients[i] = 0;
            }
            return coefficients;
        }

    }
}
