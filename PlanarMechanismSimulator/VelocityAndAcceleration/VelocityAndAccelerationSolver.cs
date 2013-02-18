#region
using System;
using System.Linq;
using System.Collections.Generic;
using StarMathLib;
#endregion

namespace PlanarMechanismSimulator.VelocityAndAcceleration
//at time t=0; all acceleration and velocity are zero
{
    /// <summary>
    /// Acceleration Solver works by creating equations corresponding to the relative velocity equations.
    /// </summary>
    internal class AccelerationSolver : VelocityAndAccelerationSolver
    {
        internal AccelerationSolver(List<joint> joints, List<link> links, int firstInputJointIndex, int inputJointIndex,
                              int inputLinkIndex,
                              double InputSpeed)
            : base(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, InputSpeed)
        {
        }
        protected override void SetInitialInputAndGroundLinkStates(link groundLink)
        {
            groundLink.Acceleration = 0.0;
            inputLink.Acceleration = 0.0;
        }

        protected override void SetInitialInputAndGroundJointStates()
        {
            if (inputJoint.jointType == JointTypes.R)
            {
                var xGnd = joints[inputJointIndex].x;
                var yGnd = joints[inputJointIndex].y;
                for (int i = firstInputJointIndex; i < inputJointIndex; i++)
                {
                    if (joints[i].SlidingWithRespectTo(inputLink)) continue;
                    joints[i].ax = inputSpeed * inputSpeed * (xGnd - joints[i].x);
                    joints[i].ay = inputSpeed * inputSpeed * (yGnd - joints[i].y);
                }
            }
            else if (inputJoint.jointType != JointTypes.P)
                throw new Exception("Currently only R or P can be the input joints.");
        }
        protected override void PutStateVarsBackInJointsAndLinks(double[] x)
        {
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o is link) ((link)o).Acceleration = x[index++];
                else if (o is joint)
                {
                    ((joint)o).ax = x[index++];
                    ((joint)o).ay = x[index++];
                }
            }
        }

        protected override JointToJointEquation MakeJointToJointEquations(joint kJoint, joint jJoint, link l, bool jointJIsKnown, bool jointKIsKnown, bool linkIsKnown)
        {
            if (jJoint.SlidingWithRespectTo(l) && kJoint.SlidingWithRespectTo(l))
                return new AccelerationEquationForDoubleSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown);
            if (jJoint.SlidingWithRespectTo(l))
                return new AccelerationEquationForFixedToSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown);
            if (kJoint.SlidingWithRespectTo(l))
                return new AccelerationEquationForFixedToSlide(kJoint, jJoint, l, jointJIsKnown, jointKIsKnown);
            return new AccelerationEquationForFixedJoints(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown);
        }
    }
    /// <summary>
    /// Velocity Solver works by creating equations corresponding to the relative velocity equations.
    /// </summary>
    internal class VelocitySolver : VelocityAndAccelerationSolver
    {
        internal VelocitySolver(List<joint> joints, List<link> links, int firstInputJointIndex, int inputJointIndex,
                              int inputLinkIndex,
                              double InputSpeed)
            : base(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, InputSpeed)
        {
        }
        protected override void SetInitialInputAndGroundLinkStates(link groundLink)
        {
            if (inputJoint.jointType == JointTypes.R)
                inputLink.Velocity = inputSpeed;
            else if (inputJoint.jointType == JointTypes.P)
            {
                inputLink.Velocity = 0.0;
                var vx = inputSpeed * Math.Cos(inputJoint.SlideAngle);
                var vy = inputSpeed * Math.Sin(inputJoint.SlideAngle);
                for (int j = firstInputJointIndex; j <= inputJointIndex; j++)
                {
                    if (!joints[j].SlidingWithRespectTo(inputLink))
                    {
                        joints[j].vx = vx;
                        joints[j].vy = vy;
                    }
                }
            }
            else throw new Exception("Currently only R or P can be the input joints.");

            groundLink.Velocity = 0.0;
        }
        protected override void SetInitialInputAndGroundJointStates()
        {
            if (inputJoint.jointType == JointTypes.R)
            {
                var xGnd = joints[inputJointIndex].x;
                var yGnd = joints[inputJointIndex].y;
                for (int i = firstInputJointIndex; i < inputJointIndex; i++)
                {
                    if (joints[i].SlidingWithRespectTo(inputLink)) continue;
                    joints[i].vx = inputSpeed * (yGnd - joints[i].y);
                    joints[i].vy = inputSpeed * (joints[i].x - xGnd);
                }
            }
            else if (inputJoint.jointType != JointTypes.P)
                throw new Exception("Currently only R or P can be the input joints.");
        }
        protected override void PutStateVarsBackInJointsAndLinks(double[] x)
        {
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o is link) ((link)o).Velocity = x[index++];
                else if (o is joint)
                {
                    ((joint)o).vx = x[index++];
                    ((joint)o).vy = x[index++];
                }
            }
        }
        protected override JointToJointEquation MakeJointToJointEquations(joint kJoint, joint jJoint, link l, bool jointJIsKnown, bool jointKIsKnown, bool linkIsKnown)
        {
            if (jJoint.SlidingWithRespectTo(l) && kJoint.SlidingWithRespectTo(l))
                return new VelocityEquationForDoubleSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown, linkIsKnown);
            if (jJoint.SlidingWithRespectTo(l))
                return new VelocityEquationForFixedToSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown, linkIsKnown);
            if (kJoint.SlidingWithRespectTo(l))
                return new VelocityEquationForFixedToSlide(kJoint, jJoint, l, jointJIsKnown, jointKIsKnown, linkIsKnown);
            return new VelocityEquationForFixedJoints(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown, linkIsKnown);
        }
    }

    /// <summary>
    /// Velocity Solver works by creating equations corresponding to the relative velocity equations.
    /// </summary>
    internal abstract class VelocityAndAccelerationSolver
    {
        protected readonly List<joint> joints;
        protected readonly int firstInputJointIndex;
        protected readonly int inputJointIndex;
        protected readonly double inputSpeed;

        protected readonly joint inputJoint;
        protected readonly List<EquationBase> equations;
        protected readonly link inputLink;
        protected readonly int numUnknowns;
        protected readonly double[,] A;
        protected readonly double[] b;
        protected readonly List<object> unknownObjects;
        private readonly List<int>[] possibleRowsPerIndex;
        private int[][] matrixOrders;

        /// <summary>
        /// Initializes a new instance of the <see cref="VelocitySolver" /> class.
        /// </summary>
        /// <param name="joints">The joints.</param>
        /// <param name="links">The links.</param>
        /// <param name="firstInputJointIndex">First index of the input joint.</param>
        /// <param name="inputJointIndex">Index of the input joint.</param>
        /// <param name="inputLinkIndex">Index of the input link.</param>
        /// <param name="InputSpeed">The input speed.</param>
        /// <exception cref="System.Exception">Currently only R or P can be the input joints.</exception>
        public VelocityAndAccelerationSolver(List<joint> joints, List<link> links, int firstInputJointIndex,
                                             int inputJointIndex,
                                             int inputLinkIndex,
                                             double InputSpeed)
        {
            /************ Initialization ************/
            this.joints = joints;
            this.inputJointIndex = inputJointIndex;
            inputJoint = joints[inputJointIndex];
            this.firstInputJointIndex = firstInputJointIndex;
            inputLink = links[inputLinkIndex];
            this.joints = joints;
            inputSpeed = InputSpeed;
            equations = new List<EquationBase>();

            unknownObjects = new List<object>();
            /************ Set up Equations ************/
            SetInitialInputAndGroundLinkStates(links[inputLinkIndex + 1]);
            for (int i = 0; i < links.Count; i++)
            {
                var l = links[i];
                if (i < inputLinkIndex) unknownObjects.Add(l);
                var refJoint = l.joints.FirstOrDefault(j => jointIsKnownState(j, l));
                if (refJoint != null)
                {
                    foreach (var j in l.joints.Where(j => j != refJoint && !jointIsKnownState(j, l)))
                        equations.Add(MakeJointToJointEquations(j, refJoint, l, true, false, (i >= inputLinkIndex)));
                }
                else
                {
                    refJoint = l.joints[0];
                    for (int k = 1; k < l.joints.Count; k++)
                        equations.Add(MakeJointToJointEquations(l.joints[k], refJoint, l, false, false,
                                                                (i >= inputLinkIndex)));
                }
            }
            /**** Then there is the matter of the sliding velocities.. ****/
            foreach (joint j in joints)
                if (j.jointType == JointTypes.P || j.jointType == JointTypes.RP)
                    unknownObjects.Add(new Tuple<link, joint>(j.Link1, j));
            for (int i = 0; i < firstInputJointIndex; i++)
            {
                var j = joints[i];
                unknownObjects.Add(j);
                if (j.jointType == JointTypes.P)
                    equations.Add(new EqualLinkToLinkStateVarEquation(j.Link1, j.Link2));
            }
            /**** The velocities of any P-joints or RP-joints connected to ground are unknown. ****/
            for (int i = inputJointIndex + 1; i < joints.Count; i++)
                if (joints[i].jointType == JointTypes.P || joints[i].jointType == JointTypes.RP)
                    unknownObjects.Add(joints[i]);
            /**** Set velocity of any P-links connected to input and remove link from unknowns ****/
            for (int i = firstInputJointIndex; i < inputJointIndex; i++)
                if (joints[i].jointType == JointTypes.P)
                {
                    var otherLink = joints[i].OtherLink(links[inputJointIndex]);
                    otherLink.Velocity = links[inputLinkIndex].Velocity;
                    unknownObjects.Remove(otherLink);
                }
            /**** Set up equations. Number of unknowns is 2*unknown-joints + 1*unknown links. ****/
            foreach (var unknownObject in unknownObjects)
            {
                if (unknownObject is joint && ((joint)unknownObject).jointType != JointTypes.P)
                    numUnknowns += 2;
                else numUnknowns++;
            }
            A = new double[numUnknowns, numUnknowns];
            b = new double[numUnknowns];
            /* While unknownObjects is saved, the data for their positions in the matrix is also stored in the equation objects*/
            var rowNonZeroes = new List<List<int>>();
            var numEquations = 0;
            foreach (var eq in equations)
            {
                eq.CaptureUnknownIndicies(unknownObjects);
                eq.unkLength = numUnknowns;
                if (eq is JointToJointEquation)
                {
                    rowNonZeroes.Add(((JointToJointEquation)eq).GetRow1Indices());
                    rowNonZeroes.Add(((JointToJointEquation)eq).GetRow2Indices());
                    numEquations += 2;
                }
                else
                {
                    rowNonZeroes.Add(((EqualLinkToLinkStateVarEquation)eq).GetRowIndices());
                    numEquations++;
                }
            }
            if (numEquations != numUnknowns) throw new Exception("The number of equations, " + numEquations + ", must be the same as the " +
                      "number of unknowns, " + numUnknowns);
            DefineTwoMatrixOrders(rowNonZeroes);
        }

        private void DefineTwoMatrixOrders(List<List<int>> rowNonZeroes)
        {
            matrixOrders = new int[2][];
            for (int m = 0; m < 2; m++)
            {
                matrixOrders[m] = new int[numUnknowns];
                var rowNonZeroesTemp = new List<List<int>>(rowNonZeroes);
                var targetIndices = new List<Tuple<int, int>>();
                for (int i = 0; i < numUnknowns; i++)
                    targetIndices.Add(new Tuple<int, int>(i, rowNonZeroesTemp.Count(r => r.Contains(i))));
                var j = 0;
                while (targetIndices.Count > 0)
                {
                    var lowestOccurence = targetIndices.Min(t => t.Item2);
                    var lowestOccurringVariable = targetIndices.First(t => t.Item2 == lowestOccurence);
                    var rowsWithlowestOccuringVar =
                        rowNonZeroesTemp.Where(r => r.Contains(lowestOccurringVariable.Item1));
                    var rowWithlowestOccuringVar = (rowsWithlowestOccuringVar.Count() == 1)
                                                       ? rowsWithlowestOccuringVar.First()
                                                       : rowsWithlowestOccuringVar.ToArray()[m];
                    matrixOrders[m][lowestOccurringVariable.Item1] = rowNonZeroes.IndexOf(rowWithlowestOccuringVar);
                    targetIndices.Remove(lowestOccurringVariable);
                    rowNonZeroesTemp.Remove(rowWithlowestOccuringVar);
                    var tuplesToUpdate =
                        targetIndices.Where(tuple => rowWithlowestOccuringVar.Contains(tuple.Item1)).ToList();
                    foreach (var tuple in tuplesToUpdate)
                    {
                        targetIndices.Remove(tuple);
                        targetIndices.Add(new Tuple<int, int>(tuple.Item1, tuple.Item2 - 1));
                    }
                    j++;
                }
            }
        }


        private bool jointIsKnownState(joint j, link l)
        {
            return ((j.isGround && (j.jointType == JointTypes.R || j.jointType == JointTypes.G))
                || ((l == inputLink || j.OtherLink(l) == inputLink) && !j.SlidingWithRespectTo(inputLink)));
        }

        protected abstract JointToJointEquation MakeJointToJointEquations(joint kJoint, joint jJoint, link l, bool jointJIsKnown,
                                                          bool jointKIsKnown, bool linkIsKnown);

        protected abstract void SetInitialInputAndGroundLinkStates(link groundLink);
        protected abstract void SetInitialInputAndGroundJointStates();
        protected abstract void PutStateVarsBackInJointsAndLinks(double[] x);


        internal Boolean Solve()
        {
            //return false;
            SetInitialInputAndGroundJointStates();
            var rows = new double[numUnknowns][];
            var answers = new double[numUnknowns];
            var i = 0;
            foreach (var eq in equations)
                if (eq is JointToJointEquation)
                {
                    rows[i] = ((JointToJointEquation)eq).GetRow1Coefficients();
                    answers[i] = ((JointToJointEquation)eq).GetRow1Constant();
                    i++;
                    rows[i] = ((JointToJointEquation)eq).GetRow2Coefficients();
                    answers[i] = ((JointToJointEquation)eq).GetRow2Constant();
                    i++;
                }
                else
                {
                    rows[i] = ((EqualLinkToLinkStateVarEquation)eq).GetRowCoefficients();
                    answers[i] = 0.0;
                    i++;
                }
            var rowOrdering = ChooseBestRowOrder(rows);
            for (int j = 0; j < numUnknowns; j++)
            {
                StarMath.SetRow(j, A, rows[rowOrdering[j]]);
                b[j] = answers[rowOrdering[j]];
            }
            var x = StarMath.solve(A, b);
            if (x.Any(value => Double.IsInfinity(value) || Double.IsNaN(value))) return false;
            PutStateVarsBackInJointsAndLinks(x);
            return true;
        }

        private int[] ChooseBestRowOrder(double[][] rows)
        {
            var order0Value = 1.0;
            var order1Value = 1.0;

            for (int i = 0; i < numUnknowns; i++)
            {
                var value = MultiplicativeDistanceToOne(rows[matrixOrders[0][i]][i]);
                if (value < order0Value) order0Value = value;
                value = MultiplicativeDistanceToOne(rows[matrixOrders[1][i]][i]);
                if (value < order1Value) order1Value = value;
            }
            if (order0Value >= order1Value) return matrixOrders[0];
            return matrixOrders[1];
        }
        public double MultiplicativeDistanceToOne(double x)
        {
            if (double.IsInfinity(x) || double.IsNaN(x) || x == 0.0) return 0;
            return (Math.Abs(x) > 1) ? 1 / Math.Abs(x) : Math.Abs(x);

        }
    }
}

