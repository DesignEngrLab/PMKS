using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;

namespace PMKS.PositionSolving
{
    internal class NonDyadicPositionSolver : IObjectiveFunction
    {
        private readonly List<NonDyadicObjFunctionTerm> linkFunctions;
        private readonly abstractConvergence ConvergedWithinLimit;
        private readonly abstractOptMethod optMethod;

        private readonly int numUnknownJoints;
        private readonly IList<Link> links;
        private readonly IList<Joint> joints;
        private readonly IList<Joint> unkJoints;
        private PositionFinder posFinder;
        internal long NumEvals { get; private set; }


        internal NonDyadicPositionSolver(PositionFinder posFinder)
        {
            this.posFinder = posFinder;
            links = posFinder.links;
            joints = posFinder.joints;
            linkFunctions = new List<NonDyadicObjFunctionTerm>();
            unkJoints = new List<Joint>();
            foreach (var j in joints.Where(j => j.positionKnown != KnownState.Fully && !j.JustATracer))
                /* we'll solve these tracer points later, hence the j.Link2 !=null. */
                unkJoints.Add(j);
            foreach (var j in joints)
            {
                if (j.TypeOfJoint == JointType.R) continue;
                var slideLink = j.Link1;
                var blockLink = j.Link2;
                if (j.TypeOfJoint == JointType.RP)
                {
                    var slideJoint = slideLink.ReferenceJoint1;
                    var refJoint = slideLink.Joints.FirstOrDefault(jt => jt != slideJoint
                                                                       && jt.Link2 != null &&
                                                                       jt.FixedWithRespectTo(slideLink));
                    if (refJoint == null) continue;
                    if (unkJoints.Contains(j) || unkJoints.Contains(slideJoint))
                        linkFunctions.Add(new SameSlideAcrossRPJointLinks(unkJoints.IndexOf(j), joints.IndexOf(j),
                            unkJoints.IndexOf(slideJoint), joints.IndexOf(slideJoint),
                            unkJoints.IndexOf(refJoint), joints.IndexOf(refJoint),
                            j.OffsetSlideAngle, slideLink.DistanceBetweenSlides(j, slideJoint)));
                }
                else if (j.TypeOfJoint == JointType.P)
                {
                    var refJoint = (blockLink.ReferenceJoint1 != j) ? blockLink.ReferenceJoint1 :
                        blockLink.Joints.FirstOrDefault(jt => jt != j && jt.Link2 != null && jt.FixedWithRespectTo(blockLink));
                    if (refJoint == null) continue;
                    var slideJoint = slideLink.ReferenceJoint1;
                    if (unkJoints.Contains(j) || unkJoints.Contains(slideJoint))
                        linkFunctions.Add(new SameSlideAcrossPJointLinks(unkJoints.IndexOf(j), joints.IndexOf(j),
                            unkJoints.IndexOf(slideJoint), joints.IndexOf(slideJoint),
                            unkJoints.IndexOf(refJoint), joints.IndexOf(refJoint),
                            blockLink.angleOfBlockToJoint(j, refJoint), slideLink.DistanceBetweenSlides(j, slideJoint)));

                    //var sJ2 = slideLink.joints.FirstOrDefault(jt => jt != slideJoint
                    //                                               && jt.Link2 != null &&
                    //                                               jt.FixedWithRespectTo(slideLink));
                    //if ((sJ2 != null)
                    //    && ((unkJoints.Contains(j) && unkJoints.Contains(refJoint)) ||
                    //    (unkJoints.Contains(slideJoint) && unkJoints.Contains(sJ2))))
                    //    linkFunctions.Add(new SameAngleAcrossPJointLinks(unkJoints.IndexOf(j), joints.IndexOf(j),
                    //        j.XInitial, j.YInitial,
                    //        unkJoints.IndexOf(refJoint), joints.IndexOf(refJoint), refJoint.XInitial, refJoint.YInitial,
                    //        unkJoints.IndexOf(slideJoint), joints.IndexOf(slideJoint), slideJoint.XInitial, slideJoint.YInitial,
                    //        unkJoints.IndexOf(sJ2), joints.IndexOf(sJ2), sJ2.XInitial, sJ2.YInitial, j.OffsetSlideAngle,
                    //        blockLink.angleOfBlockToJoint(j, refJoint)));
                }
            }
            foreach (var c in links)
            {
                for (int i = 0; i < c.Joints.Count - 1; i++)
                    for (int j = i + 1; j < c.Joints.Count; j++)
                    {
                        var p0 = c.Joints[i];
                        var p1 = c.Joints[j];
                        if (p0.Link2 == null || p1.Link2 == null) continue;
                        var p0Index = unkJoints.IndexOf(p0);
                        var p1Index = unkJoints.IndexOf(p1);
                        if (p0Index == -1 && p1Index == -1)
                            continue; //if both joints are known, then no need to add an objective function term   
                        if (p0.FixedWithRespectTo(c) && p1.FixedWithRespectTo(c))
                        {
                            var deltX = p0.XInitial - p1.XInitial;
                            var deltY = p0.YInitial - p1.YInitial;
                            var origLengthSquared = deltX * deltX + deltY * deltY;
                            linkFunctions.Add(new LinkLengthFunction(p0Index, joints.IndexOf(p0), p1Index, joints.IndexOf(p1), origLengthSquared));
                        }
                    }
            }
            numUnknownJoints = unkJoints.Count;
            optMethod = new NewtonMethod();
            optMethod.Add(this);
            ConvergedWithinLimit = new ToKnownBestFConvergence(0, Constants.epsilon);
            optMethod.Add(ConvergedWithinLimit);
            optMethod.Add(new DeltaFConvergence(Constants.epsilon));
            // optMethod.Add(new FixedOrGoldenSection(1e-2, 0));
            optMethod.Add(new FixedOrGoldenSection(0.1 * Constants.epsilonSame, 0));
            optMethod.Add(new MaxIterationsConvergence(Constants.MaxItersInNonDyadicSolver));
            //  optMethod.Add(new DeltaFConvergence(0.01 * Constants.epsilonSame));
        }

        internal Boolean SolutionFound()
        {
            return optMethod.ConvergenceDeclaredBy.Contains(ConvergedWithinLimit);
        }
        internal void Run_PositionsAreClose(out double posError)
        {
            posError = 0.0;
            var xInit = new double[2 * numUnknownJoints];
            for (int i = 0; i < joints.Count; i++)
            {
                var j = joints[i];
                if (j.positionKnown == KnownState.Fully)
                    foreach (var llf in linkFunctions)
                        llf.SetInitialJointPosition(i, j.x, j.y);
                else if (j.Link2 != null) //once, again tracer points are done at the end
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
            if (!SolutionFound())
            {
                posFinder.posResult = PositionAnalysisResults.InvalidPosition;
                return;
            }

            for (int i = 0; i < numUnknownJoints; i++)
            {
                var j = unkJoints[i];
                j.x = xStar[2 * i];
                j.y = xStar[2 * i + 1];
                j.positionKnown = KnownState.Fully;

                var tempError = (xStar[2 * i] - xInit[2 * i]) * (xStar[2 * i] - xInit[2 * i]) +
                    (xStar[2 * i + 1] - xInit[2 * i + 1]) * (xStar[2 * i + 1] - xInit[2 * i + 1]);
                if (posError < tempError) posError = tempError;
            }
            /* this recurses through to fix all the tracer points    */
            foreach (var c in links)
                //   if (c.AngleIsKnown == KnownState.Unknown)
                posFinder.setLinkPositionFromRotate(c.Joints.First(j => j.FixedWithRespectTo(c)), c);
        }

        internal double Run_PositionsAreUnknown(double[,] newJointParams, double[,] newLinkParams)
        {
            var r = new Random();
            var fStar = double.PositiveInfinity;
            double[] xStar = null;
            NumEvals = 0;
            int k = 0;
            var xMin = joints.Min(j => j.XInitial);
            var xMax = joints.Max(j => j.XInitial);
            var yMin = joints.Min(j => j.YInitial);
            var yMax = joints.Max(j => j.YInitial);
            var maxLength = links.Max(l0 => l0.MaxLength);
            var range = Constants.rangeMultiplier * (new[] { xMax - xMin, yMax - yMin, maxLength }).Max();
            var offset = (xMin + xMax + yMin + yMax) / 4 - (range / 2);
            do
            {
                NumEvals += optMethod.numEvals;
                var xInit = new double[2 * numUnknownJoints]; //need to check if this is always true. If input is a ternary link it could be less.

                for (int i = 0; i < numUnknownJoints; i++)
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

            for (int i = 0; i < numUnknownJoints; i++)
            {
                newJointParams[i, 0] = xStar[2 * i];
                newJointParams[i, 1] = xStar[2 * i + 1];
            }
            foreach (var c in links)
                if (c.AngleIsKnown == KnownState.Unknown)
                    posFinder.setLinkPositionFromRotate(c.Joints.First(j => j.FixedWithRespectTo(c)), c);

            return fStar;
        }


        internal override double calculate(double[] x)
        {
            double sum = 0;
            foreach (NonDyadicObjFunctionTerm a in linkFunctions)
                sum += a.calculate(x);
            return sum;
        }


        internal override double deriv_wrt_xi(double[] x, int i)
        {
            double sum = 0;
            foreach (NonDyadicObjFunctionTerm a in linkFunctions)
                sum += a.deriv_wrt_xi(x, i);
            return sum;
        }


        internal override double second_deriv_wrt_ij(double[] x, int i, int j)
        { return linkFunctions.Sum(a => a.second_deriv_wrt_ij(x, i, j)); }


    }
}
