using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

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
            return distanceSqared(point1.X, point1.Y, point2.X, point2.Y);
        }
        internal static double distanceSqared(point point1, joint joint2)
        {
            return distanceSqared(point1.X, point1.Y, joint2.initX, joint2.initY);
        }
        internal static double distanceSqared(joint joint1, joint joint2)
        {
            return distanceSqared(joint1.initX, joint1.initY, joint2.initX, joint2.initY);
        }
        #endregion

        #region Distance
        public static double distance(double x1, double y1, double x2 = 0, double y2 = 0)
        {
            return Math.Sqrt(distanceSqared(x1, y1, x2, y2));
        }
        public static double distance(point point1, point point2)
        {
            return distance(point1.X, point1.Y, point2.X, point2.Y);
        }
        internal static double distance(point point1, joint joint2)
        {
            return distance(point1.X, point1.Y, joint2.initX, joint2.initY);
        }
        internal static double distance(joint joint1, joint joint2)
        {
            return distance(joint1.initX, joint1.initY, joint2.initX, joint2.initY);
        }
        #endregion

        #region Angle
        internal static double angle(point start, point end)
        {
            return angle(start.X, start.Y, end.X, end.Y);
        }

        internal static double angle(joint start, joint end)
        {
            return angle(start.initX, start.initY, end.initX, end.initY);
        }

        internal static double angle(double startX, double startY, double endX, double endY)
        {
            return Math.Atan2(endY - startY, endX - startX);
        }
        #endregion

    }
}
