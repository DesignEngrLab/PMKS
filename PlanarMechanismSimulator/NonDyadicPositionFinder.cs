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
        private readonly List<LinkLengthFunction> linkFunctions;
        private readonly abstractConvergence ConvergedWithinLimit;
        private readonly abstractOptMethod optMethod;
        
        private readonly int numPivots ;
        private readonly IEnumerable<link> links;
        private readonly IList<pivot> pivots;

        internal long NumEvals { get; private set; }

        internal NonDyadicPositionFinder(IEnumerable<link> links, IList<pivot> pivots, int numPivots, double epsilon)
        {
            this.links = links;
            this.pivots = pivots;
            this.numPivots = numPivots;
            linkFunctions = new List<LinkLengthFunction>();
            foreach (var c in links)
            {
                var p0 = c.Pivots[0];
                var p0Index = pivots.IndexOf(p0);
                var p1 = c.Pivots[1];
                var p1Index = pivots.IndexOf(p1);
                if ((!double.IsNaN(p0.X)) && (!double.IsNaN(p0.Y)) &&
                    (!double.IsNaN(p1.X)) && (!double.IsNaN(p1.Y)))
                    linkFunctions.Add(new LinkLengthFunction(p0Index, p0.X, p0.Y, p1Index, p1.X, p1.Y));
                else if ((!double.IsNaN(p0.X)) && (!double.IsNaN(p0.Y)) && (!double.IsNaN(c.length)))
                    linkFunctions.Add(new LinkLengthFunction(p0Index, p0.X, p0.Y, p1Index, c.length));
                else if ((!double.IsNaN(p1.X)) && (!double.IsNaN(p1.Y)) && (!double.IsNaN(c.length)))
                    linkFunctions.Add(new LinkLengthFunction(p1Index, p1.X, p1.Y, p0Index, c.length));
                else if (!double.IsNaN(c.length))
                    linkFunctions.Add(new LinkLengthFunction(p0Index, p1Index, c.length));
                else throw new Exception("Links is not well-specified (in constructor of NonDyadicPositionFinder).");
            }
            optMethod = new NewtonMethod();
            optMethod.Add(this);
            ConvergedWithinLimit = new ToKnownBestFConvergence(0, epsilon);
            optMethod.Add(ConvergedWithinLimit);
            optMethod.Add(new FixedOrGoldenSection(5*epsilon, 0));
            optMethod.Add(new MaxIterationsConvergence(300));
            optMethod.Add(new DeltaFConvergence(5*epsilon));
        }

        internal void MovePivot(int index, double x, double y)
        {
            foreach (var lf in linkFunctions) lf.MovePivot(index, x, y);
        }
        internal Boolean SolutionFound()
        {
            return optMethod.ConvergenceDeclaredBy.Contains(ConvergedWithinLimit);
        }
        internal double Run(out double[] xStar, double[] xInit=null)
        {
            NumEvals += optMethod.numEvals;
            optMethod.ResetFunctionEvaluationDatabase();
            var result = optMethod.Run(out xStar, xInit);
            if (SolutionFound())
                for (int i = 0; i < 2*numPivots; i=i+2)
                {
                    pivots[i/2].X = xStar[i];
                    pivots[i/2].Y = xStar[i+1];
                }
            foreach (var a in links)
                a.length = Math.Sqrt((a.Pivots[0].X - a.Pivots[1].X)*(a.Pivots[0].X - a.Pivots[1].X)
                                     + (a.Pivots[0].Y - a.Pivots[1].Y)*(a.Pivots[0].Y - a.Pivots[1].Y));
            return result;
        }

        public double calculate(double[] x)
        { return linkFunctions.Sum(a => a.calculate(x)); }


        public double deriv_wrt_xi(double[] x, int i)
        { return linkFunctions.Sum(a => a.deriv_wrt_xi(x, i)); }


        public double second_deriv_wrt_ij(double[] x, int i, int j)
        { return linkFunctions.Sum(a => a.second_deriv_wrt_ij(x, i, j)); }

    }
}
