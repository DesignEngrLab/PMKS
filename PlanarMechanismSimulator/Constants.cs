using System;
using System.Collections.Generic;
using System.Linq;
using PMKS.VelocityAndAcceleration;
using PMKS;
using StarMathLib;

namespace PMKS
{
    /// <summary>
    /// Class of Constants and simple static functions used in PMKS.
    /// </summary>
    public static class Constants
    {

        /// <summary>
        ///   This is used below in the close enough to zero booleans to match points
        ///   (see below: sameCloseZero). In order to avoid strange round-off issues - 
        ///   even with doubles - I have implemented this function when comparing the
        ///   position of points (mostly in checking for a valid transformation (see
        ///   ValidTransformation) and if other nodes comply (see otherNodesComply).
        /// </summary>
        internal const double epsilonSame = 1e-12;

        internal const double epsilon = 1e-9 ;
        internal const double ErrorInDeterminingCompleteCycle = 0.001;
        internal const double rangeMultiplier = 5.0;
        internal const int numberOfTries = 50;
        internal const double SmallPerturbationFraction = 0.003;
        internal const double DefaultStepSize = 0.5;
        internal const double MinimumStepSize = 0.0005;
        internal const int MaxItersInPositionError = 10;
        internal const double ConservativeErrorEstimation = 0.9;
        internal const double ErrorEstimateInertia = 2.0;
        internal const double ErrorSizeIncrease = 1.2;
        internal const long MaxItersInNonDyadicSolver = 300;
        internal const double DefaultInputSpeed = 1.0;

        internal static TimeSpan MaxTimeToFindMatrixOrders = new TimeSpan((long)2000000);

        internal const double XRangeLimitFactor = 5.0;
        internal const double YRangeLimitFactor = 5.0;      
        internal const double BoundingBoxAspectRatio = 2.0;
        internal const double XMinimumFactor = 1e-8;
        internal const double YMinimumFactor = 1e-8;
        internal const double AngleMinimumFactor = 1e-6;

        internal const double JointAccelerationLimitFactor = 75.0;
        internal const double LinkAccelerationLimitFactor = 75.0;
        internal const double JointVelocityLimitFactor = 75.0;
        internal const double LinkVelocityLimitFactor = 75.0;
        internal const double FullCircle = 2 * Math.PI;
        internal const double MaxSlope = 10e9;
        internal const double SmoothingErrorRepeatFactor = 10.0;

        /// <summary>
        /// Is x1s the close zero?
        /// </summary>
        /// <param name="x1">The x1.</param>
        /// <returns>Boolean.</returns>
        public static Boolean sameCloseZero(double x1)
        {
            return Math.Abs(x1) < epsilonSame;
        }

        /// <summary>
        /// Are x1 and x2 the same?
        /// </summary>
        /// <param name="x1">The x1.</param>
        /// <param name="x2">The x2.</param>
        /// <returns>Boolean.</returns>
        public static Boolean sameCloseZero(double x1, double x2)
        {
            return sameCloseZero(x1 - x2);
        }


        #region DistanceSquared

        internal static double distanceSqared(double x1, double y1, double x2 = 0, double y2 = 0)
        {
            return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
        }

        internal static double distanceSqared(Point point1, Point point2)
        {
            return distanceSqared(point1.X, point1.Y, point2.X, point2.Y);
        }

        #endregion

        #region Distance

        internal static double distance(double x1, double y1, double x2 = 0, double y2 = 0)
        {
            return Math.Sqrt(distanceSqared(x1, y1, x2, y2));
        }

        internal static double distance(Point point1, Point point2)
        {
            return distance(point1.X, point1.Y, point2.X, point2.Y);
        }

        #endregion

        #region Angle

        /// <summary>
        /// Finds the angle on the vector from start to end.
        /// </summary>
        /// <param name="start">The start.</param>
        /// <param name="end">The end.</param>
        /// <returns>System.Double.</returns>
        public static double angle(Point start, Point end)
        {
            return angle(start.X, start.Y, end.X, end.Y);
        }


        /// <summary>
        /// Finds the angle on the vector from start to end.
        /// </summary>
        /// <param name="startX">The start x.</param>
        /// <param name="startY">The start y.</param>
        /// <param name="endX">The end x.</param>
        /// <param name="endY">The end y.</param>
        /// <returns>System.Double.</returns>
        public static double angle(double startX, double startY, double endX, double endY)
        {
            return Math.Atan2(endY - startY, endX - startX);
        }

        #endregion

        public static Point solveViaIntersectingLines(double slopeA, Point ptA, double slopeB, Point ptB)
        {
            if (sameCloseZero(ptA.X, ptB.X) && sameCloseZero(ptA.Y, ptB.Y)) return ptA;
            if (sameCloseZero(slopeA, slopeB)) return new Point(Double.NaN, Double.NaN);
            var offsetA = ptA.Y - slopeA * ptA.X;
            var offsetB = ptB.Y - slopeB * ptB.X;
            if (verticalSlope(slopeA))
                return new Point(ptA.X, slopeB * ptA.X + offsetB);
            if (verticalSlope(slopeB))
                return new Point(ptB.X, slopeA * ptB.X + offsetA);

            var x = (offsetB - offsetA) / (slopeA - slopeB);
            var y = slopeA * x + offsetA;
            return new Point(x, y);
        }

        private static Boolean verticalSlope(double slope)
        {
            return (Double.IsNaN(slope) || Double.IsInfinity(slope)
                    || Math.Abs(slope) > Constants.MaxSlope);
        }
    }
}
