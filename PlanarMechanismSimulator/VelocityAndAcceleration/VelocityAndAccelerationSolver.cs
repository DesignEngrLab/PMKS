#region
using System;
using System.Diagnostics;
using System.Linq;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
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
                              int inputLinkIndex, double InputSpeed, Dictionary<int, gearData> gearsData)
            : base(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, InputSpeed, gearsData)
        {
        }
        protected override void SetInitialInputAndGroundLinkStates()
        {
            groundLink.Acceleration = 0.0;
            inputLink.Acceleration = 0.0;
        }

        protected override void SetInitialInputAndGroundJointStates()
        {
            if (inputJoint.jointType == JointTypes.R)
            {
                var xGnd = inputJoint.x;
                var yGnd = inputJoint.y;
                foreach (var j in inputLink.joints)
                {
                    if (j.FixedWithRespectTo(inputLink))
                    {
                        j.ax = inputSpeed * inputSpeed * (xGnd - j.x);
                        j.ay = inputSpeed * inputSpeed * (yGnd - j.y);
                    }
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
                else if (o is Tuple<link, joint>)
                    ((Tuple<link, joint>)o).Item2.SlideAcceleration = x[index++];
            }
            if (gearsData == null) return;
            foreach (var gearData in gearsData.Values)
            {
                var j = joints[gearData.gearTeethIndex];
                j.ax = (joints[gearData.gearCenter1Index].ax * gearData.radius1
                    + joints[gearData.gearCenter2Index].ax * gearData.radius2) / (gearData.radius1 + gearData.radius2);
                j.ay = (joints[gearData.gearCenter1Index].ay * gearData.radius1
                    + joints[gearData.gearCenter2Index].ay * gearData.radius2) / (gearData.radius1 + gearData.radius2);
            }
        }

        protected override JointToJointEquation MakeJointToJointEquations(joint kJoint, joint jJoint, link l, bool jointKIsKnown, bool jointJIsKnown, bool linkIsKnown)
        {
            if (jJoint.SlidingWithRespectTo(l) && kJoint.SlidingWithRespectTo(l))
                return new AccelerationEquationForDoubleSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown);
            if (jJoint.SlidingWithRespectTo(l))
                return new AccelerationEquationForFixedToSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown);
            if (kJoint.SlidingWithRespectTo(l))
                return new AccelerationEquationForFixedToSlide(kJoint, jJoint, l, jointKIsKnown, jointJIsKnown);
            return new AccelerationEquationForFixedJoints(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown);
        }
    }
    /// <summary>
    /// Velocity Solver works by creating equations corresponding to the relative velocity equations.
    /// </summary>
    internal class VelocitySolver : VelocityAndAccelerationSolver
    {
        internal VelocitySolver(List<joint> joints, List<link> links, int firstInputJointIndex, int inputJointIndex,
            int inputLinkIndex, double InputSpeed, Dictionary<int, gearData> gearsData)
            : base(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, InputSpeed, gearsData)
        {
        }
        protected override void SetInitialInputAndGroundLinkStates()
        {
            if (inputJoint.jointType == JointTypes.R)
                inputLink.Velocity = inputSpeed;
            else if (inputJoint.jointType == JointTypes.P)
            {
                inputLink.Velocity = 0.0;
                var vx = inputSpeed * Math.Cos(inputJoint.SlideAngle);
                var vy = inputSpeed * Math.Sin(inputJoint.SlideAngle);
                foreach (var j in inputLink.joints)
                {
                    if (j.SlidingWithRespectTo(inputLink)) continue;
                    j.vx = vx;
                    j.vy = vy;
                }
            }
            else throw new Exception("Currently only R or P can be the input joints.");

            groundLink.Velocity = 0.0;
        }
        protected override void SetInitialInputAndGroundJointStates()
        {
            if (inputJoint.jointType == JointTypes.R)
            {
                var xGnd = inputJoint.x;
                var yGnd = inputJoint.y;
                foreach (var j in inputLink.joints)
                {
                    if (j.SlidingWithRespectTo(inputLink)) continue;
                    j.vx = inputSpeed * (yGnd - j.y);
                    j.vy = inputSpeed * (j.x - xGnd);
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
                else if (o is Tuple<link, joint>)
                    ((Tuple<link, joint>)o).Item2.SlideVelocity = x[index++];
            }
            if (gearsData == null) return;
            foreach (var gearData in gearsData.Values)
            {
                var j = joints[gearData.gearTeethIndex];
                j.vx = (joints[gearData.gearCenter1Index].vx * gearData.radius1
                    + joints[gearData.gearCenter2Index].vx * gearData.radius2) / (gearData.radius1 + gearData.radius2);
                j.vy = (joints[gearData.gearCenter1Index].vy * gearData.radius1
                    + joints[gearData.gearCenter2Index].vy * gearData.radius2) / (gearData.radius1 + gearData.radius2);
            }
        }
        protected override JointToJointEquation MakeJointToJointEquations(joint kJoint, joint jJoint, link l, bool jointKIsKnown, bool jointJIsKnown, bool linkIsKnown)
        {
            if (jJoint.SlidingWithRespectTo(l) && kJoint.SlidingWithRespectTo(l))
                return new VelocityEquationForDoubleSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown, linkIsKnown);
            if (jJoint.SlidingWithRespectTo(l))
                return new VelocityEquationForFixedToSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown, linkIsKnown);
            if (kJoint.SlidingWithRespectTo(l))
                return new VelocityEquationForFixedToSlide(kJoint, jJoint, l, jointKIsKnown, jointJIsKnown, linkIsKnown);
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
        protected readonly joint groundReferenceJoint;
        protected readonly List<EquationBase> equations;
        protected readonly link inputLink;
        protected readonly link groundLink;
        protected readonly int numUnknowns;
        protected readonly double[,] A;
        protected readonly double[] b;
        protected readonly List<object> unknownObjects;
        private int[][] matrixOrders;
        protected readonly Dictionary<int, gearData> gearsData;

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
        public VelocityAndAccelerationSolver(List<joint> joints, List<link> links, int firstInputJointIndex, int inputJointIndex,
                                             int inputLinkIndex, double InputSpeed, Dictionary<int, gearData> gearsData)
        {
            /************ Initialization ************/
            this.joints = joints;
            this.inputJointIndex = inputJointIndex;
            inputJoint = joints[inputJointIndex];
            this.firstInputJointIndex = firstInputJointIndex;
            groundReferenceJoint = joints.Last();
            inputLink = links[inputLinkIndex];
            groundLink = links[inputLinkIndex + 1];
            this.joints = joints;
            inputSpeed = InputSpeed;
            this.gearsData = gearsData;
            equations = new List<EquationBase>();

            unknownObjects = new List<object>();
            /************ Set up Equations ************/
            SetInitialInputAndGroundLinkStates();
            #region go through the links to identify equations from joint-to-joint on a particular link
            /* first address all the links at the beginning of the list (the ones that are not input or ground) */
            for (int i = 0; i < inputLinkIndex; i++)
            {
                var l = links[i];
                unknownObjects.Add(l);
                var refJoint = l.joints.FirstOrDefault(j => j.isGround && !j.SlidingWithRespectTo(groundLink));
                if (refJoint == null)
                    refJoint = l.joints.FirstOrDefault(j => jointIsKnownState(j, l));
                if (refJoint == null)
                {
                    refJoint = l.joints[0];
                    for (int k = 1; k < l.joints.Count; k++)
                        equations.Add(MakeJointToJointEquations(l.joints[k], refJoint, l, false, false, false));
                }
                else foreach (var j in l.joints.Where(j => j != refJoint))
                        equations.Add(MakeJointToJointEquations(j, refJoint, l, jointIsKnownState(j, l), true, false));
            }
            /* now address the input link - well, the only unknown is the joints that slide on this input. */
            foreach (var j in inputLink.joints.Where(j => j.SlidingWithRespectTo(inputLink)))
                equations.Add(MakeJointToJointEquations(j, inputJoint, inputLink, false, true, true));

            /* now address the input link - again, only the joints that slide w.r.t. ground. */
            foreach (var j in groundLink.joints.Where(j => j.SlidingWithRespectTo(groundLink)))
                equations.Add(MakeJointToJointEquations(j, groundReferenceJoint, groundLink, false, true, true));
            /* Since links connected by a P joint rotate at same speed, we need to remove those from the unknown list. */
            /**** Set velocity of any links connected to input by a P joint and remove link from unknowns ****/
            foreach (var pJoint in inputLink.joints.Where(j => j.jointType == JointTypes.P))
            {
                var otherLink = pJoint.OtherLink(inputLink);
                otherLink.Velocity = inputLink.Velocity;
                unknownObjects.Remove(otherLink);
            }
            /**** Set velocity of any links connected to ground by a P joint and remove link from unknowns ****/
            foreach (var pJoint in groundLink.joints.Where(j => j.jointType == JointTypes.P))
            {
                var otherLink = pJoint.OtherLink(groundLink);
                otherLink.Velocity = 0.0;
                unknownObjects.Remove(otherLink);
            }
            #endregion
            #region go through the joints to identify equations from slip velocities and link-to-link relationships
            /**** Then there is the matter of the sliding velocities.. ****/
            for (int i = 0; i < firstInputJointIndex; i++)
            {
                var j = joints[i];
                if (j.jointType == JointTypes.R) unknownObjects.Add(j);
                else if (j.jointType == JointTypes.G && j.Link1 != inputLink && j.Link1 != groundLink &&
                             j.Link2 != inputLink && j.Link2 != groundLink)
                    unknownObjects.Add(j);
                else if (j.jointType == JointTypes.P)
                {
                    unknownObjects.Add(new Tuple<link, joint>(j.Link1, j));
                    if (j.Link2 != inputLink && j.Link2 != groundLink)
                    {
                        unknownObjects.Add(j);
                        if (j.Link1 != inputLink && j.Link1 != groundLink)
                            equations.Add(new EqualLinkToLinkStateVarEquation(j.Link1, j.Link2));
                    }
                }
                else if (j.jointType == JointTypes.RP)
                {
                    unknownObjects.Add(new Tuple<link, joint>(j.Link1, j));
                    if (j.Link2 != inputLink && j.Link2 != groundLink)
                        unknownObjects.Add(j);
                }
            }
            #endregion

            /**** Set up equations. Number of unknowns is 2*unknown-joints + 1*unknown links. ****/
            foreach (var unknownObject in unknownObjects)
            {
                if (unknownObject is joint) numUnknowns += 2;
                else numUnknowns++;
            }
            if (numUnknowns > 200)
            {
                SkipMatrixInversionUntilSparseSolverIsDefined = true;
                return;
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
            if (numEquations < numUnknowns) throw new Exception("The number of equations, " + numEquations + ", must be as big as the " +
                      "number of unknowns, " + numUnknowns);
            DefineTwoMatrixOrders(rowNonZeroes);
        }

        private void DefineTwoMatrixOrders(List<List<int>> rowNonZeroes)
        {
            matrixOrders = new int[2][];
            orderFound = new[] { false, false };
            var now = DateTime.Now;

#if SILVERLIGHT
            var forwardThread = new Thread(() =>
            {
                DepthFirstToFindOrder(rowNonZeroes, new List<int>(), 0, now);
                forwardDone.Set();
            });
            var backwardThread = new Thread(() =>
            {
                DepthFirstToFindOrder(rowNonZeroes, new List<int>(), 1, now);
                backwardDone.Set();
            });
            forwardThread.Start();
            backwardThread.Start();
            if (forwardDone.WaitOne() && backwardDone.WaitOne())
            {
                //forwardDone = new AutoResetEvent(false);
                //backwardDone = new AutoResetEvent(false);
            }
#else
            Parallel.For(0, 2, i => DepthFirstToFindOrder(rowNonZeroes, new List<int>(), i, now));
 #endif
            if (matrixOrders[0] == null & matrixOrders[1] == null)
                throw new Exception("No valid matrix formulations found.");
            if (matrixOrders[0] == null)
            {
                matrixOrders[0] = matrixOrders[1];
                matrixOrders[1] = null;
            }

        }


        private static AutoResetEvent forwardDone = new AutoResetEvent(false);
        private static AutoResetEvent backwardDone = new AutoResetEvent(false);

        private Boolean[] orderFound;
        private void DepthFirstToFindOrder(List<List<int>> rowNonZeroes, List<int> order, int rowIndex, DateTime start)
        {
            if (orderFound[rowIndex]) return;
            if (DateTime.Now - start > Constants.MaxTimeToFindMatrixOrders)
            {
                orderFound[rowIndex] = true;
                return;
            }
            var index = order.Count;
            if (index == numUnknowns)
            {
                matrixOrders[rowIndex] = order.ToArray();
                orderFound[rowIndex] = true;
                return;
            }
            var possibleChoices = (rowIndex == 0) ? rowNonZeroes.Where((r, j) => r.Contains(index) && !order.Contains(j)) :
             rowNonZeroes.Where((r, j) => r.Contains(index) && !order.Contains(j)).Reverse();
            if (!possibleChoices.Any()) return;
            if (possibleChoices.Count() == 1)
            {
                order.Add(rowNonZeroes.IndexOf(possibleChoices.First()));
                DepthFirstToFindOrder(rowNonZeroes, order, rowIndex, start);
            }
            else
            {
                foreach (var choice in possibleChoices)
                {
                    var orderCopy = new List<int>(order);
                    orderCopy.Add(rowNonZeroes.IndexOf(choice));
                    DepthFirstToFindOrder(rowNonZeroes, orderCopy, rowIndex, start);
                }
            }
        }



        private bool jointIsKnownState(joint j, link l)
        {
            return ((j.isGround && (j.jointType == JointTypes.R || j.jointType == JointTypes.G))
                || ((l == inputLink || j.OtherLink(l) == inputLink) && !j.SlidingWithRespectTo(inputLink)));
        }

        protected abstract JointToJointEquation MakeJointToJointEquations(joint kJoint, joint jJoint, link l, bool jointKIsKnown, bool jointJIsKnown, bool linkIsKnown);

        protected abstract void SetInitialInputAndGroundLinkStates();
        protected abstract void SetInitialInputAndGroundJointStates();
        protected abstract void PutStateVarsBackInJointsAndLinks(double[] x);


        internal Boolean Solve()
        {
            if (SkipMatrixInversionUntilSparseSolverIsDefined) return false;
            try
            {
                SetInitialInputAndGroundJointStates();
                var rows = new List<double[]>();
                var answers = new List<double>();
                foreach (var eq in equations)
                    if (eq is JointToJointEquation)
                    {
                        rows.Add(((JointToJointEquation)eq).GetRow1Coefficients());
                        answers.Add(((JointToJointEquation)eq).GetRow1Constant());
                        rows.Add(((JointToJointEquation)eq).GetRow2Coefficients());
                        answers.Add(((JointToJointEquation)eq).GetRow2Constant());
                    }
                    else
                    {
                        rows.Add(((EqualLinkToLinkStateVarEquation)eq).GetRowCoefficients());
                        answers.Add(0.0);
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
            catch { return false; }
        }

        private int[] ChooseBestRowOrder(List<double[]> rows)
        {
            if (matrixOrders[0] == null && matrixOrders[1] == null) return null;
            if (matrixOrders[0] == null) return matrixOrders[1];
            if (matrixOrders[1] == null) return matrixOrders[0];
            var order0Value = 1.0;
            var order1Value = 1.0;

            for (int i = 0; i < numUnknowns; i++)
            {
                var value = MultiplicativeDistanceToOne(rows[matrixOrders[0][i]][i]);
                if (value < order0Value) order0Value = value;
                value = MultiplicativeDistanceToOne(rows[matrixOrders[1][i]][i]);
                if (value < order1Value) order1Value = value;
            }
            if (order0Value >= order1Value)
                return matrixOrders[0];
            return matrixOrders[1];
        }
        public double MultiplicativeDistanceToOne(double x)
        {
            if (double.IsInfinity(x) || double.IsNaN(x) || x == 0.0) return 0;
            return (Math.Abs(x) > 1) ? 1 / Math.Abs(x) : Math.Abs(x);

        }

        public bool SkipMatrixInversionUntilSparseSolverIsDefined { get; private set; }
    }
}

