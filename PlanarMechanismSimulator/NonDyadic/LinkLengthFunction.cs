using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;
using StarMathLib;

namespace PlanarMechanismSimulator
{
    internal class LinkLengthFunction : IObjectiveFunction, IDifferentiable, ITwiceDifferentiable
    {
        private readonly int index1;
        private readonly int index2;
        private readonly double origLength;

        private double xIndex1;
        private double yIndex1;
        private double xIndex2;
        private double yIndex2;

        private double deltaX
        {
            get { return xIndex1 - xIndex2; }
        }

        private double deltaY
        {
            get { return yIndex1 - yIndex2; }
        }

        private double newLengthSqared
        {
            get { return deltaX * deltaX + deltaY * deltaY; }
        }

        private double newLength
        {
            get { return Math.Sqrt(newLengthSqared); }
        }

        public LinkLengthFunction(int index1, double XIndex1, double YIndex1, int index2, double XIndex2, double YIndex2)
        {
            xIndex1 = XIndex1;
            yIndex1 = YIndex1;
            xIndex2 = XIndex2;
            yIndex2 = YIndex2;
            this.index1 = index1;
            this.index2 = index2;
            origLength = Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
        }
        public LinkLengthFunction(int index1, int index2, double length)
        {
            this.index1 = index1;
            this.index2 = index2;
            origLength = length;
        }
        public LinkLengthFunction(int index1, double XIndex1, double YIndex1, int index2, double length)
        {
            xIndex1 = XIndex1;
            yIndex1 = YIndex1;
            this.index1 = index1;
            this.index2 = index2;
            origLength = length;
        }

        public double calculate(double[] x)
        {
            assignPositions(x);
            return newLengthSqared - 2 * origLength * newLength + origLength * origLength;
        }

        public double deriv_wrt_xi(double[] x, int i)
        {
            if (!(i == 2 * index1 || i == 2 * index1 + 1 || i == 2 * index2 || i == 2 * index2 + 1)) return 0;
            assignPositions(x);
            if (i == 2 * index1)
                // w.r.t. x1
                return -2 * deltaX * (origLength / newLength - 1);
            if (i == 2 * index1 + 1)
                // w.r.t. y1
                return -2 * deltaY * (origLength / newLength - 1);
            if (i == 2 * index2)
                // w.r.t. x2
                return 2 * deltaX * (origLength / newLength - 1);
            if (i == 2 * index2 + 1)
                // w.r.t. y2
                return 2 * deltaY * (origLength / newLength - 1);
            throw new Exception("Gradient:you shouldn't be seeing this! how did you get by the initial if-statement?");
        }

        public double second_deriv_wrt_ij(double[] x, int i, int j)
        {
            if ((!(i == 2 * index1 || i == 2 * index1 + 1 || i == 2 * index2 || i == 2 * index2 + 1))
                || (!(j == 2 * index1 || j == 2 * index1 + 1 || j == 2 * index2 || j == 2 * index2 + 1))) return 0;
            assignPositions(x);
            var firstTerm = 2 * (1 - origLength / newLength);
            var secondterm = 2 * origLength / Math.Pow(newLength, 3);
            if (i == j)
            {
                if ((i == 2 * index1) || (i == 2 * index2)) return firstTerm + deltaX * deltaX * secondterm;
                if ((i == 2 * index1 + 1) || (i == 2 * index2 + 1)) return firstTerm + deltaY * deltaY * secondterm;
            }
            if (i > j)
            {
                //switch so that i is always less than j. Hessians are always symmetric.
                var temp = i;
                i = j;
                j = temp;
            }
            if (i == 2 * index1)
            {
                if (j == 2 * index1 + 1) return deltaX * deltaY * secondterm;
                if (j == 2 * index2) return -firstTerm - deltaX * deltaX * secondterm;
                if (j == 2 * index2 + 1) return -deltaX * deltaY * secondterm;
            }
            if (i == 2 * index1 + 1)
            {
                if (j == 2 * index2) return -deltaY * deltaX * secondterm;
                if (j == 2 * index2 + 1) return -firstTerm - deltaY * deltaY * secondterm;
            }
            if (i == 2 * index2)
                if (j == 2 * index2 + 1) return deltaX * deltaY * secondterm;
            throw new Exception("Hessian:you shouldn't be seeing this! how did you get by the initial if-statement?");
        }

        private void assignPositions(double[] x)
        {
            if (x.GetLength(0) > 2 * index1 + 1)
            {
                xIndex1 = x[2 * index1];
                yIndex1 = x[2 * index1 + 1];
            }
            if (x.GetLength(0) > 2 * index2 + 1)
            {
                xIndex2 = x[2 * index2];
                yIndex2 = x[2 * index2 + 1];
            }
        }
        
        internal void MovePivot(int index, double x, double y)
        {
            if (index == index1)
            {
                xIndex1 = x;
                yIndex1 = y;
            }
            if (index == index2)
            {
                xIndex2 = x;
                yIndex2 = y;
            }
        }
    }
}
