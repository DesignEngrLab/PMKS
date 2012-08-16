using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;

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
        private readonly IList<int> unkJointIndices;
        internal long NumEvals { get; private set; }

        internal NonDyadicPositionSolver(IList<link> links, IList<joint> joints, double epsilon)
        {
            this.links = links;
            this.joints = joints;
            linkFunctions = new List<LinkLengthFunction>();
            unkJointIndices = new List<int>();
            for (int i = 0; i < joints.Count; i++)
            {
var                j = joints[i];
                if (j.knownState != KnownState.Fully)
                                    if (j.jointType != JointTypes.R)
                        throw new Exception("Cannot currently handle non R-joints in Non-Dyadic Analysis.");
                unkJointIndices.Add(i);
                }
            numUnknownPivots = unkJointIndices.Count;
            for (int i = 0; i < numUnknownPivots; i++)
            {
                var p0 = joints[unkJointIndices[i]];
                var p0Index = joints.IndexOf(p0);
                foreach (var p1 in p0.Link1.joints)
                {
                    if (p1 == p0) continue;
                    var p1Index = joints.IndexOf(p1);
                    var p1UnkListIndex = unkJointIndices.IndexOf(p1Index);
                    linkFunctions.Add(new LinkLengthFunction(i, p0.xInitial, p0.yInitial, p1UnkListIndex, p1.xInitial, p1.yInitial));
                }
                foreach (var p1 in p0.Link2.joints)
                {
                    if (p1 == p0) continue;
                    var p1Index = joints.IndexOf(p1);
                    var p1UnkListIndex = unkJointIndices.IndexOf(p1Index);
                    linkFunctions.Add(new LinkLengthFunction(i, p0.xInitial, p0.yInitial, p1UnkListIndex, p1.xInitial, p1.yInitial));
                }
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
        internal bool Run_PositionsAreClose()
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
            optMethod.Run(out xStar, xInit);
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
                    newLinkParams[i, 0] = Constants.angle(newJointParams[joint0Index, 0], newJointParams[joint0Index, 1],
                       newJointParams[joint1Index, 0], newJointParams[joint1Index, 1]);
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
            NumEvals = 0;
            int k = 0;
            var xMin = joints.Min(j => j.xInitial);
            var xMax = joints.Max(j => j.xInitial);
            var yMin = joints.Min(j => j.yInitial);
            var yMax = joints.Max(j => j.yInitial);
            var maxLength = links.Max(l0 => l0.Lengths.Max());
            var range = Constants.rangeMultiplier * (new[] { xMax - xMin, yMax - yMin, maxLength }).Max();
            var offset = (xMin + xMax + yMin + yMax) / 4 - (range / 2);
            do
            {
                NumEvals += optMethod.numEvals;
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
                newLinkParams[i, 0] = Constants.angle(newJointParams[joint0Index, 0], newJointParams[joint0Index, 1],
                    newJointParams[joint1Index, 0], newJointParams[joint1Index, 1]);
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
