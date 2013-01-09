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
        public const int MaxItersInPositionError = 10;
        public const double ConservativeErrorEstimation = 0.9;
        public const double ErrorEstimateInertia = 2.0;
        public const double ErrorSizeIncrease = 1.2;
        public static long MaxItersInNonDyadicSolver = 300;

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

        internal static point solveViaIntersectingLines(double slopeA, point ptA, double slopeB, point ptB)
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

        internal static Boolean CreateBestMatrixAndB(int numUnknowns, int numEquations, double[][] rows, double[,] A,
                                                double[] answers, double[] b)
        {
            if (numEquations < numUnknowns) return false;
            var rowData = new List<Tuple<List<int>, double, double[], double>>();
            for (int i = 0; i < numEquations; i++)
                rowData.Add(new Tuple<List<int>, double, double[], double>
                    (NonZeroColumns(rows[i], numUnknowns), DistanceFromOne(rows[i]), rows[i], answers[i]));
            var targetIndices = new List<Tuple<int, int>>();
            for (int i = 0; i < numUnknowns; i++)
                targetIndices.Add(new Tuple<int, int>(i, rowData.Count(r => r.Item1.Contains(i))));
            while (targetIndices.Count > 0)
            {
                var lowestOccurence = targetIndices.Min(t => t.Item2);
                //if (lowestOccurence == 0) return false;
                var targetIndex = targetIndices.First(t => t.Item2 == lowestOccurence);
                var index = targetIndex.Item1;
                var rowDatum = rowData.Where(r => r.Item1.Contains(index)).OrderBy(t => t.Item2).First();
                StarMath.SetRow(index, A, rowDatum.Item3);
                b[index] = rowDatum.Item4;
                targetIndices.Remove(targetIndex);
                rowData.Remove(rowDatum);
                var tuplesToUpdate = targetIndices.Where(tuple => rowDatum.Item1.Contains(tuple.Item1)).ToList();
                foreach (var tuple in tuplesToUpdate)
                {
                    targetIndices.Remove(tuple);
                    targetIndices.Add(new Tuple<int, int>(tuple.Item1, tuple.Item2 - 1));
                }
            }
            return true;
        }

        private static List<int> NonZeroColumns(double[] rows, int numUnknowns)
        {
            var result = new List<int>();
            for (int j = 0; j < numUnknowns; j++)
                if (rows[j] != 0.0) result.Add(j);
            return result;
        }

        private static double DistanceFromOne(double[] row)
        {
            var rowMax = row.Max();
            if (rowMax == 0.0) rowMax = 1.0;
            else if (Math.Abs(rowMax) < 1) rowMax = 1 / rowMax;
            var rowMin = row.Min();
            if (rowMin == 0.0) rowMin = 1.0;
            else if (Math.Abs(rowMin) < 1) rowMin = 1 / rowMin;
            return Math.Max(Math.Abs(rowMax), Math.Abs(rowMin));
        }
    }
}
