using System;

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

        internal static TimeSpan MaxTimeToFindMatrixOrders = new TimeSpan(2000000);

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
        /// <summary>
        /// The full circle or rather 2pi
        /// </summary>
        public const double FullCircle = 2 * Math.PI;
        /// <summary>
        /// A quarter of a circle, or Pi divided by 2
        /// </summary>
        public const double QuarterCircle = Math.PI / 2.0;
        internal const double MaxSlope = 10e9;
        internal const double SmoothingErrorRepeatFactor = 10.0;
        public const double DefaultSpeed = 10.0;
        public const double DefaultError = 0.001;
        public const double DefaultAngleInc = 5.0;

        /// <summary>
        /// Is x1s the close zero?
        /// </summary>
        /// <param name="x1">The x1.</param>
        /// <returns>Boolean.</returns>
        internal static Boolean SameCloseZero(double x1)
        {
            return Math.Abs(x1) < epsilonSame;
        }

        /// <summary>
        /// Are x1 and x2 the same?
        /// </summary>
        /// <param name="x1">The x1.</param>
        /// <param name="x2">The x2.</param>
        /// <returns>Boolean.</returns>
        internal static Boolean SameCloseZero(double x1, double x2)
        {
            return SameCloseZero(x1 - x2);
        }


        #region DistanceSquared

        internal static double DistanceSqared(double x1, double y1, double x2 = 0, double y2 = 0)
        {
            return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
        }

        internal static double DistanceSqared(Point point1, Point point2)
        {
            return DistanceSqared(point1.X, point1.Y, point2.X, point2.Y);
        }

        #endregion

        #region Distance

        internal static double Distance(double x1, double y1, double x2 = 0, double y2 = 0)
        {
            return Math.Sqrt(DistanceSqared(x1, y1, x2, y2));
        }

        internal static double Distance(Point point1, Point point2)
        {
            return Distance(point1.X, point1.Y, point2.X, point2.Y);
        }

        #endregion

        #region Angle

        /// <summary>
        /// Finds the angle on the vector from start to end.
        /// </summary>
        /// <param name="start">The start.</param>
        /// <param name="end">The end.</param>
        /// <returns>System.Double.</returns>
        internal static double Angle(Point start, Point end)
        {
            return Angle(start.X, start.Y, end.X, end.Y);
        }


        /// <summary>
        /// Finds the angle on the vector from start to end.
        /// </summary>
        /// <param name="startX">The start x.</param>
        /// <param name="startY">The start y.</param>
        /// <param name="endX">The end x.</param>
        /// <param name="endY">The end y.</param>
        /// <returns>System.Double.</returns>
        public static double Angle(double startX, double startY, double endX, double endY)
        {
            return Math.Atan2(endY - startY, endX - startX);
        }

        #endregion

        /// <summary>
        /// Solves the via intersecting lines.
        /// </summary>
        /// <param name="slopeA">The slope a.</param>
        /// <param name="ptA">The pt a.</param>
        /// <param name="slopeB">The slope b.</param>
        /// <param name="ptB">The pt b.</param>
        /// <returns></returns>
        public static Point SolveViaIntersectingLines(double slopeA, Point ptA, double slopeB, Point ptB)
        {
            if (SameCloseZero(ptA.X, ptB.X) && SameCloseZero(ptA.Y, ptB.Y)) return ptA;
            if (SameCloseZero(slopeA, slopeB)) return new Point(Double.NaN, Double.NaN);
            var offsetA = ptA.Y - slopeA * ptA.X;
            var offsetB = ptB.Y - slopeB * ptB.X;
            if (VerticalSlope(slopeA))
                return new Point(ptA.X, slopeB * ptA.X + offsetB);
            if (VerticalSlope(slopeB))
                return new Point(ptB.X, slopeA * ptB.X + offsetA);

            var x = (offsetB - offsetA) / (slopeA - slopeB);
            var y = slopeA * x + offsetA;
            return new Point(x, y);
        }

        private static Boolean VerticalSlope(double slope)
        {
            return (Double.IsNaN(slope) || Double.IsInfinity(slope)
                    || Math.Abs(slope) > MaxSlope);
        }
    }
}
