using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;
using StarMathLib;

namespace PlanarMechanismSimulator
{
    internal class NonDyadicPositionFinder : IObjectiveFunction, IDifferentiable, ITwiceDifferentiable
    {
        private const double rangeMultiplier = 5.0;
        private const int numberOfTries = 50;

        private readonly List<LinkLengthFunction> linkFunctions;
        private readonly abstractConvergence ConvergedWithinLimit;
        private readonly abstractOptMethod optMethod;

        private readonly int numPivots;
        private readonly IEnumerable<link> links;
        private readonly IList<joint> joints;

        internal long NumEvals { get; private set; }

        internal NonDyadicPositionFinder(IEnumerable<link> links, IList<joint> pivots, int numPivots, double epsilon)
        {
            this.links = links;
            this.joints = pivots;
            this.numPivots = numPivots;
            linkFunctions = new List<LinkLengthFunction>();
            foreach (var c in links)
            {
                var p0 = c.joints[0];
                var p0Index = pivots.IndexOf(p0);
                var p1 = c.joints[1];
                var p1Index = pivots.IndexOf(p1);
                if ((!double.IsNaN(p0.X)) && (!double.IsNaN(p0.Y)) &&
                    (!double.IsNaN(p1.X)) && (!double.IsNaN(p1.Y)))
                    linkFunctions.Add(new LinkLengthFunction(p0Index, p0.X, p0.Y, p1Index, p1.X, p1.Y));
                else if ((!double.IsNaN(p0.X)) && (!double.IsNaN(p0.Y)) && (!double.IsNaN(c.lengths[0])))
                    linkFunctions.Add(new LinkLengthFunction(p0Index, p0.X, p0.Y, p1Index, c.lengths[0]));
                else if ((!double.IsNaN(p1.X)) && (!double.IsNaN(p1.Y)) && (!double.IsNaN(c.lengths[0])))
                    linkFunctions.Add(new LinkLengthFunction(p1Index, p1.X, p1.Y, p0Index, c.lengths[0]));
                else if (!double.IsNaN(c.lengths[0]))
                    linkFunctions.Add(new LinkLengthFunction(p0Index, p1Index, c.lengths[0]));
                else throw new Exception("Links is not well-specified (in constructor of NonDyadicPositionFinder).");
                //todo: note how all the lengths above are lengths[0] need to fix s.t. they correspond to proper values.
            }
            optMethod = new NewtonMethod();
            optMethod.Add(this);
            ConvergedWithinLimit = new ToKnownBestFConvergence(0, epsilon);
            optMethod.Add(ConvergedWithinLimit);
            optMethod.Add(new FixedOrGoldenSection(5 * epsilon, 0));
            optMethod.Add(new MaxIterationsConvergence(300));
            optMethod.Add(new DeltaFConvergence(5 * epsilon));
        }

        internal void MovePivot(int index, double x, double y)
        {
        }
        internal Boolean SolutionFound()
        {
            return optMethod.ConvergenceDeclaredBy.Contains(ConvergedWithinLimit);
        }
        internal Boolean Run_PositionsAreClose()
        {
            optMethod.ResetFunctionEvaluationDatabase();
            var xInit = new double[2 * numPivots]; //need to check if this is always true. If input is a ternary link it could be less.
            for (int i = 0; i < numPivots; i++)
            {
                xInit[2 * i] = joints[i].X;
                xInit[2 * i + 1] = joints[i].Y;
            }

            double[] xStar;
            var result = optMethod.Run(out xStar, xInit);
            if (SolutionFound())
            {
                for (int i = 0; i < numPivots; i++)
                {
                    joints[i].X = xStar[2 * i];
                    joints[i].Y = xStar[2 * i + 1];
                }
                return true;
            }
            return false;
        }

        internal double Run_PositionsAreUnknown()
        {
            var r = new Random();
            var fStar = double.PositiveInfinity;
            double[] xStar=null;
            long numFEvals = 0;
            int k = 0;
            var xMin = joints.Min(j => j.X);
            var xMax = joints.Max(j => j.X);
            var yMin = joints.Min(j => j.Y);
            var yMax = joints.Max(j => j.Y);
            var maxLength = links.Max(l0 => l0.lengths.Max());
            var range =rangeMultiplier* (new[] { xMax - xMin, yMax - yMin, maxLength }).Max();
            var offset = (xMin + xMax + yMin + yMax) / 4 - (range / 2);
            do
            {
                numFEvals += optMethod.numEvals;
                optMethod.ResetFunctionEvaluationDatabase();
                var xInit = new double[2 * numPivots]; //need to check if this is always true. If input is a ternary link it could be less.

                for (int i = 0; i < numPivots; i++)
                    xInit[i] = range * r.NextDouble() +offset;
                double[] xStarTemp;
                var fStarTemp = optMethod.Run(out xStarTemp, xInit);
                if (fStarTemp < fStar)
                {
                    xStar = xStarTemp;
                    fStar = fStarTemp;
                }
                //SearchIO.output("fStar = " + fStar);
            } while (!optMethod.ConvergenceDeclaredBy.Contains(ConvergedWithinLimit) && k++ < numberOfTries);

            for (int i = 0; i < numPivots; i++)
            {
                joints[i].X = xStar[2 * i];
                joints[i].Y = xStar[2 * i + 1];
            }
            return fStar;
        }


        public double calculate(double[] x)
        { return linkFunctions.Sum(a => a.calculate(x)); }


        public double deriv_wrt_xi(double[] x, int i)
        { return linkFunctions.Sum(a => a.deriv_wrt_xi(x, i)); }


        public double second_deriv_wrt_ij(double[] x, int i, int j)
        { return linkFunctions.Sum(a => a.second_deriv_wrt_ij(x, i, j)); }

    }
}
