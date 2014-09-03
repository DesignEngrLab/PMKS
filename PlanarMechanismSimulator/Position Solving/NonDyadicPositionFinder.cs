using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;
using PMKS;

namespace PMKS.PositionSolving
{
    internal class NonDyadicPositionSolver : IObjectiveFunction, IDifferentiable, ITwiceDifferentiable
    {
        private readonly List<ILinkFunction> linkFunctions;
        private readonly abstractConvergence ConvergedWithinLimit;
        private readonly abstractOptMethod optMethod;

        private readonly int numUnknownPivots;
        private readonly int beginGndJointsIndex;
        private readonly int numUnknownLinks;
        private readonly IList<link> links;
        private readonly IList<joint> joints;
        private readonly IList<joint> unkJoints;
        private PositionFinder posFinder;
        internal long NumEvals { get; private set; }


        public NonDyadicPositionSolver(PositionFinder posFinder)
        {
            this.posFinder = posFinder;
            this.links = posFinder.links;
            this.joints = posFinder.joints;
            linkFunctions = new List<ILinkFunction>();
            unkJoints = new List<joint>();
            foreach (var j in joints.Where(j => j.positionKnown != KnownState.Fully))
                unkJoints.Add(j);

            numUnknownPivots = unkJoints.Count;
            foreach (var c in links)
            {
                for (int i = 0; i < c.joints.Count - 1; i++)
                    for (int j = i + 1; j < c.joints.Count; j++)
                    {
                        var p0 = c.joints[i];
                        var p1 = c.joints[j];
                        var p0Index = unkJoints.IndexOf(p0);
                        var p1Index = unkJoints.IndexOf(p1);
                        if (p0Index == -1 && p1Index == -1) continue; //if both joints are known, then no need to add an objective function term   
                        if (p0.SlidingWithRespectTo(c) && p1.SlidingWithRespectTo(c)) continue;  //if both joints are sliding on link c, then no useful constraint info is extracted   
                        if (p0.FixedWithRespectTo(c) && p1.FixedWithRespectTo(c))
                            linkFunctions.Add(new LinkLengthFunction(p0Index, joints.IndexOf(p0), p0.xInitial,
                                p0.yInitial, p1Index, joints.IndexOf(p1), p1.xInitial, p1.yInitial));
                        else
                        {
                            if (p0.SlidingWithRespectTo(c))
                            {  // the LinkSliderFunction assumes the first joint is the fixed one. Since this is not the case, reverse here.
                                var tempJoint = p0; var tempIndex = p0Index;
                                p0 = p1; p0Index = p1Index;
                                p1 = tempJoint; p1Index = tempIndex;
                            }
                            linkFunctions.Add(new LinkSliderFunction(p0Index, joints.IndexOf(p0), p0.xInitial,p0.yInitial, 
                                p1Index, joints.IndexOf(p1), p1.xInitial, p1.yInitial,p1.InitSlideAngle,c.AngleInitial));
                            if (p1.jointType == JointTypes.P)
                                linkFunctions.Add(new LinkSameAngleFunction(p0Index, joints.IndexOf(p0), p0.xInitial, p0.yInitial,
                                    p1Index, joints.IndexOf(p1), p1.xInitial, p1.yInitial, p1.InitSlideAngle, c.AngleInitial));
                        }
                    }
            }
            optMethod = new NewtonMethod();
            optMethod.Add(this);
            ConvergedWithinLimit = new ToKnownBestFConvergence(0, Constants.epsilon);
            optMethod.Add(ConvergedWithinLimit);
            optMethod.Add(new DeltaFConvergence(1e-5));
            // optMethod.Add(new FixedOrGoldenSection(1e-2, 0));
            optMethod.Add(new FixedOrGoldenSection(0.1 * Constants.epsilonSame, 0));
            optMethod.Add(new MaxIterationsConvergence(Constants.MaxItersInNonDyadicSolver));
            //  optMethod.Add(new DeltaFConvergence(0.01 * Constants.epsilonSame));
        }

        internal Boolean SolutionFound()
        {
            return optMethod.ConvergenceDeclaredBy.Contains(ConvergedWithinLimit);
        }
        internal bool Run_PositionsAreClose(out double posError)
        {
            posError = 0.0;
            var xInit = new double[2 * numUnknownPivots];
            for (int i = 0; i < joints.Count; i++)
            {
                var j = joints[i];
                if (j.positionKnown == KnownState.Fully)
                    foreach (var llf in linkFunctions)
                        llf.SetInitialJointPosition(i, j.x, j.y);
                else
                {
                    var xPosStart = j.xNumerical;
                    var yPosStart = j.yNumerical;
                    foreach (var llf in linkFunctions)
                        llf.SetInitialJointPosition(i, xPosStart, yPosStart);
                    var index = unkJoints.IndexOf(j);
                    xInit[2 * index] = xPosStart;
                    xInit[2 * index + 1] = yPosStart;
                }
            }

            double[] xStar;
            optMethod.Run(out xStar, xInit);
            if (!SolutionFound()) return false;

            for (int i = 0; i < numUnknownPivots; i++)
            {
                var j = unkJoints[i];
                j.x = xStar[2 * i];
                j.y = xStar[2 * i + 1];
                j.positionKnown = KnownState.Fully;

                var tempError = (xStar[2 * i] - xInit[2 * i]) * (xStar[2 * i] - xInit[2 * i]) +
                    (xStar[2 * i + 1] - xInit[2 * i + 1]) * (xStar[2 * i + 1] - xInit[2 * i + 1]);
                if (posError < tempError) posError = tempError;
            }
            foreach (var c in links)
                if (c.AngleIsKnown == KnownState.Unknown)
                    posFinder.setLinkPositionFromRotate(c.joints.First(j => j.FixedWithRespectTo(c)), c);
            return true;
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
            var maxLength = links.Max(l0 => l0.MaxLength);
            var range = Constants.rangeMultiplier * (new[] { xMax - xMin, yMax - yMin, maxLength }).Max();
            var offset = (xMin + xMax + yMin + yMax) / 4 - (range / 2);
            do
            {
                NumEvals += optMethod.numEvals;
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
