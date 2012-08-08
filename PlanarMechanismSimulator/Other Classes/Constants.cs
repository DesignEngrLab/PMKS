using System;

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
        public const double epsilon = 0.000001;

        internal const double rangeMultiplier = 5.0;
        internal const int numberOfTries = 50;

        internal static Boolean sameCloseZero(double x1)
        {
            return Math.Abs(x1) < epsilon;
        }

        internal static Boolean sameCloseZero(double x1, double x2)
        {
            return sameCloseZero(x1 - x2);
        }


        #region DistanceSquared
        public static double distanceSqared(double x1, double y1, double x2=0, double y2=0)
        {
            return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
        }
        internal static double distanceSqared(point point1, point point2)
        {
            return distanceSqared(point1.x, point1.y, point2.x, point2.y);
        }
        #endregion

        #region Distance
        public static double distance(double x1, double y1, double x2 = 0, double y2 = 0)
        {
            return Math.Sqrt(distanceSqared(x1, y1, x2, y2));
        }
        public static double distance(point point1, point point2)
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
    }
}
