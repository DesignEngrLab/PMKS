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
            return distanceSqared(point1.X, point1.Y, point2.X, point2.Y);
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
        #endregion

        #region Angle
        internal static double angle(point start, point end)
        {
            return angle(start.X, start.Y, end.X, end.Y);
        }


        internal static double angle(double startX, double startY, double endX, double endY)
        {
            return Math.Atan2(endY - startY, endX - startX);
        }
        #endregion

        #region point to line interactions
        internal static point findOrthoPoint(point p, point lineRef, double lineAngle)
        {
            if (sameCloseZero(lineAngle))
                return new point(p.X, lineRef.Y);
            if (sameCloseZero(Math.Abs(lineAngle), Math.PI / 2))
                return new point(lineRef.X, p.Y);
            var slope = Math.Tan(lineAngle);
            var offset = (lineRef.Y - slope * lineRef.X);
            var x = (p.X + slope * (p.Y - offset)) / (slope * slope + 1);
            var y = slope * x + offset;
            return new point(x, y);
        }

        internal static point findOrthoPoint(double pX, double pY, double lineRefX, double lineRefY, double lineAngle, out Boolean pointOnHigherOffset)
        {
            if (sameCloseZero(lineAngle))
            {
                pointOnHigherOffset = (pY > lineRefY);
                return new point(pX, lineRefY);
            }
            if (sameCloseZero(Math.Abs(lineAngle), Math.PI / 2))
            {
                pointOnHigherOffset = (pX * lineAngle < 0);
                return new point(lineRefX, pY);
            }
            var slope = Math.Tan(lineAngle);
            var offset = (lineRefY - slope*lineRefX);
            var x = (pX + slope * (pY - offset)) /(slope * slope + 1);
            var y = slope * x + offset;
            pointOnHigherOffset = ((pY - slope*pX) > offset);
            return new point(x, y);
        }
#endregion
    }
}
