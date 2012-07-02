using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;
using StarMathLib;

namespace PlanarMechanismSimulator
{
    internal class NonDyadicPositionSolver : IObjectiveFunction, IDifferentiable, ITwiceDifferentiable
    {
        private readonly List<LinkLengthFunction> linkFunctions;
        private readonly abstractConvergence ConvergedWithinLimit;
        private readonly abstractOptMethod optMethod;

        private readonly int numUnknownPivots;
        private readonly int beginGndJointsIndex;
        private readonly int numUnknownLinks;
        private readonly IList<link> links;
        private readonly IList<joint> joints;

        internal long NumEvals { get; private set; }

        internal NonDyadicPositionSolver(IList<link> links, IList<joint> joints, int numUnknownPivots,
            int beginGndJointsIndex, int numUnknownLinks, double epsilon)
        {
            this.links = links;
            this.joints = joints;
            this.numUnknownPivots = numUnknownPivots;
            this.beginGndJointsIndex = beginGndJointsIndex;
            this.numUnknownLinks = numUnknownLinks;
            linkFunctions = new List<LinkLengthFunction>();
            foreach (var c in links)
            {
                int lengthIndex = 0;
                for (int i = 0; i < c.joints.Count - 1; i++)
                    for (int j = i + 1; j < c.joints.Count; j++)
                    {
                        var p0 = c.joints[i];
                        var p0Index = joints.IndexOf(p0);
                        var p1 = c.joints[j];
                        var p1Index = joints.IndexOf(p1);
                        if ((!double.IsNaN(p0.initX)) && (!double.IsNaN(p0.initY)) &&
                            (!double.IsNaN(p1.initX)) && (!double.IsNaN(p1.initY)))
                            linkFunctions.Add(new LinkLengthFunction(p0Index, p0.initX, p0.initY, p1Index, p1.initX, p1.initY));
                        else if ((!double.IsNaN(p0.initX)) && (!double.IsNaN(p0.initY)) && (!double.IsNaN(c.lengths[lengthIndex])))
                            linkFunctions.Add(new LinkLengthFunction(p0Index, p0.initX, p0.initY, p1Index, c.lengths[lengthIndex]));
                        else if ((!double.IsNaN(p1.initX)) && (!double.IsNaN(p1.initY)) && (!double.IsNaN(c.lengths[lengthIndex])))
                            linkFunctions.Add(new LinkLengthFunction(p1Index, p1.initX, p1.initY, p0Index, c.lengths[lengthIndex]));
                        else if (!double.IsNaN(c.lengths[lengthIndex]))
                            linkFunctions.Add(new LinkLengthFunction(p0Index, p1Index, c.lengths[lengthIndex]));
                        else throw new Exception("Links is not well-specified (in constructor of NonDyadicPositionFinder).");
                    }
                //todo: note how all the lengths above are lengths[0] need to fix s.t. they correspond to proper values.
            }
            optMethod = new NewtonMethod();
            optMethod.Add(this);
            ConvergedWithinLimit = new ToKnownBestFConvergence(0, epsilon);
            optMethod.Add(ConvergedWithinLimit);
            optMethod.Add(new FixedOrGoldenSection(5 * epsilon, 0));
            optMethod.Add(new MaxIterationsConvergence(300));
            optMethod.Add(new DeltaFConvergence(0.5 * epsilon));
        }

        internal Boolean SolutionFound()
        {
            return optMethod.ConvergenceDeclaredBy.Contains(ConvergedWithinLimit);
        }
        internal bool Run_PositionsAreClose(double[,] newJointParams, double[,] newLinkParams, double[,] oldJointParams, double[,] oldLinkParams)
        {
            optMethod.ResetFunctionEvaluationDatabase();
            var xInit = new double[2 * numUnknownPivots]; //need to check if this is always true. If input is a ternary link it could be less.
            for (int i = 0; i < numUnknownPivots; i++)
            {
                xInit[2 * i] = newJointParams[i, 0];
                xInit[2 * i + 1] = newJointParams[i, 1];
            }
            for (int i = numUnknownPivots; i < beginGndJointsIndex; i++)
                foreach (var llf in linkFunctions)
                    llf.SetJointPosition(i, newJointParams[i, 0], newJointParams[i, 1]);

            double[] xStar;
            var result = optMethod.Run(out xStar, xInit);
            if (SolutionFound())
            {
                for (int i = 0; i < numUnknownPivots; i++)
                {
                    newJointParams[i, 0] = xStar[2 * i];
                    newJointParams[i, 1] = xStar[2 * i + 1];
                }
                for (int i = 0; i < numUnknownLinks; i++)
                {
                    var joint0Index = joints.IndexOf(links[i].joints[0]);
                    var joint1Index = joints.IndexOf(links[i].joints[1]);
                    newLinkParams[i, 0] = Math.Atan2(newJointParams[joint1Index, 1] - newJointParams[joint0Index, 1],
                        newJointParams[joint1Index, 0] - newJointParams[joint0Index, 0]);
                }
                return true;
            }
            return false;
        }

        internal double Run_PositionsAreUnknown(double[,] newJointParams, double[,] newLinkParams)
        {
            var r = new Random();
            var fStar = double.PositiveInfinity;
            double[] xStar = null;
            long numFEvals = 0;
            int k = 0;
            var xMin = joints.Min(j => j.initX);
            var xMax = joints.Max(j => j.initX);
            var yMin = joints.Min(j => j.initY);
            var yMax = joints.Max(j => j.initY);
            var maxLength = links.Max(l0 => l0.lengths.Max());
            var range = Constants.rangeMultiplier * (new[] { xMax - xMin, yMax - yMin, maxLength }).Max();
            var offset = (xMin + xMax + yMin + yMax) / 4 - (range / 2);
            do
            {
                numFEvals += optMethod.numEvals;
                optMethod.ResetFunctionEvaluationDatabase();
                var xInit = new double[2 * numUnknownPivots]; //need to check if this is always true. If input is a ternary link it could be less.

                for (int i = 0; i < numUnknownPivots; i++)
                    xInit[i] = range * r.NextDouble() + offset;
                double[] xStarTemp;
                var fStarTemp = optMethod.Run(out xStarTemp, xInit);
                if (fStarTemp < fStar)
                {
                    xStar = xStarTemp;
                    fStar = fStarTemp;
                }
                //SearchIO.output("fStar = " + fStar);
            } while (!optMethod.ConvergenceDeclaredBy.Contains(ConvergedWithinLimit) && k++ < Constants.numberOfTries);

            for (int i = 0; i < numUnknownPivots; i++)
            {
                newJointParams[i, 0] = xStar[2 * i];
                newJointParams[i, 1] = xStar[2 * i + 1];
            }
            for (int i = 0; i < numUnknownLinks; i++)
            {
                var joint0Index = joints.IndexOf(links[i].joints[0]);
                var joint1Index = joints.IndexOf(links[i].joints[1]);
                newLinkParams[i, 0] = Math.Atan2(newJointParams[joint1Index, 1] - newJointParams[joint0Index, 1],
                    newJointParams[joint1Index, 0] - newJointParams[joint0Index, 0]);
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
