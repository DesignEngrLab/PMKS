using System;
using System.Collections.Generic;
using System.Linq;
using PlanarMechanismSimulator.VelocityAndAcceleration;
using StarMathLib;

namespace PlanarMechanismSimulator
{
    public static class Constants
    {
        /// <summary>
        ///   This is used below in the close enough to zero booleans to match points
        ///   (see below: sameCloseZero). In order to avoid strange round-off issues - 
        ///   even with doubles - I have implemented this function when comparing the
        ///   position of points (mostly in checking for a valid transformation (see
        ///   ValidTransformation) and if other nodes comply (see otherNodesComply).
        /// </summary>
        public const double epsilonSame = 10e-12;

        public const double epsilon = 10e-9;
        internal const double rangeMultiplier = 5.0;
        internal const int numberOfTries = 50;
        public const double SmallPerturbationFraction = 0.003;
        public const double DefaultStepSize = 0.5;
        public const double MinimumStepSize = 0.001;
        public const int MaxItersInPositionError = 10;
        public const double ConservativeErrorEstimation = 0.9;
        public const double ErrorEstimateInertia = 2.0;
        public const double ErrorSizeIncrease = 1.2;
        public const long MaxItersInNonDyadicSolver = 300;

        public static Boolean sameCloseZero(double x1)
        {
            return Math.Abs(x1) < epsilonSame;
        }

        public static Boolean sameCloseZero(double x1, double x2)
        {
            return sameCloseZero(x1 - x2);
        }


        #region DistanceSquared

        public static double distanceSqared(double x1, double y1, double x2 = 0, double y2 = 0)
        {
            return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
        }

        internal static double distanceSqared(point point1, point point2)
        {
            return distanceSqared(point1.x, point1.y, point2.x, point2.y);
        }

        #endregion

        #region Distance

        internal static double distance(double x1, double y1, double x2 = 0, double y2 = 0)
        {
            return Math.Sqrt(distanceSqared(x1, y1, x2, y2));
        }

        internal static double distance(point point1, point point2)
        {
            return distance(point1.x, point1.y, point2.x, point2.y);
        }

        #endregion

        #region Angle

        internal static double angle(point start, point end)
        {
            return angle(start.x, start.y, end.x, end.y);
        }


        internal static double angle(double startX, double startY, double endX, double endY)
        {
            return Math.Atan2(endY - startY, endX - startX);
        }

        #endregion

        public static point solveViaIntersectingLines(double slopeA, point ptA, double slopeB, point ptB)
        {
            if (sameCloseZero(ptA.x, ptB.x) && sameCloseZero(ptA.y, ptB.y)) return ptA;
            if (sameCloseZero(slopeA, slopeB)) return new point(Double.NaN, Double.NaN);
            var offsetA = ptA.y - slopeA * ptA.x;
            var offsetB = ptB.y - slopeB * ptB.x;
            if (Double.IsNaN(slopeA) || Double.IsInfinity(slopeA))
                return new point(ptA.x, slopeB * ptA.x + offsetB);
            if (Double.IsNaN(slopeB) || Double.IsInfinity(slopeB))
                return new point(ptB.x, slopeA * ptB.x + offsetA);

            var x = (offsetB - offsetA) / (slopeA - slopeB);
            var y = slopeA * x + offsetA;
            return new point(x, y);
        }
    }
}
