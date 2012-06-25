using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PlanarMechanismSimulator
{
    static class Constants
    {
        /// <summary>
        ///   This is used below in the close enough to zero booleans to match points
        ///   (see below: sameCloseZero). In order to avoid strange round-off issues - 
        ///   even with doubles - I have implemented this function when comparing the
        ///   position of points (mostly in checking for a valid transformation (see
        ///   ValidTransformation) and if other nodes comply (see otherNodesComply).
        /// </summary>
        private const double epsilon = 0.000001;

        internal static Boolean sameCloseZero(double x1)
        {
            return Math.Abs(x1) < epsilon;
        }

        internal static Boolean sameCloseZero(double x1, double x2)
        {
            return sameCloseZero(x1 - x2);
        }
    }
}
