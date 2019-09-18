#region

using StarMathLib;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
#endregion

namespace PMKS.VelocityAndAcceleration
//at time t=0; all acceleration and velocity are zero
{
    /// <summary>
    /// Acceleration Solver works by creating equations corresponding to the relative velocity equations.
    /// </summary>
    internal class AccelerationSolver : VelocityAndAccelerationSolver
    {
        internal AccelerationSolver(List<Joint> joints, List<Link> links, int firstInputJointIndex, int inputJointIndex,
                              int inputLinkIndex, double inputSpeed, Dictionary<int, GearData> gearsData, double averageLength)
            : base(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, inputSpeed, gearsData)
        {
            maximumJointValue = Constants.JointAccelerationLimitFactor * averageLength * inputSpeed * inputSpeed;
            maximumLinkValue = Constants.LinkAccelerationLimitFactor * inputSpeed * inputSpeed;
        }

        protected override void SetInitialInputAndGroundStates()
        {
            RecursivelySetLinkVelocityThroughPJoints(groundLink, new List<Link>(), 0.0, false);
            RecursivelySetLinkVelocityThroughPJoints(inputLink, new List<Link>(), 0.0, false);
            if (inputJoint.TypeOfJoint == JointType.R)
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
            else if (inputJoint.TypeOfJoint == JointType.P)
            {
                inputJoint.SlideAcceleration = 0.0;
                foreach (var j in inputLink.joints)
                {
                    if (j.FixedWithRespectTo(inputLink))
                        j.ax = j.ay = 0.0;
                }
            }
            else throw new Exception("Currently only R or P can be the input joints.");
        }

        protected override Boolean PutStateVarsBackInJointsAndLinks(double[] x)
        {
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o is Link)
                {
                    var a = x[index++];
                    if (Math.Abs(a) > maximumLinkValue) return false;
                    ((Link)o).Acceleration = a;
                }
                else if (o is Joint)
                {
                    var ax = x[index++];
                    var ay = x[index++];
                    if (Math.Abs(ax) > maximumJointValue || Math.Abs(ay) > maximumJointValue) return false;
                    ((Joint)o).ax = ax;
                    ((Joint)o).ay = ay;
                }
                else if (o is Tuple<Link, Joint>)
                    ((Tuple<Link, Joint>)o).Item2.SlideAcceleration = x[index++];
            }
            if (gearsData != null)
            {
                foreach (var gearData in gearsData.Values)
                {
                    var j = joints[gearData.gearTeethIndex];
                    j.ax = (joints[gearData.gearCenter1Index].ax * gearData.radius1
                            + joints[gearData.gearCenter2Index].ax * gearData.radius2) /
                           (gearData.radius1 + gearData.radius2);
                    j.ay = (joints[gearData.gearCenter1Index].ay * gearData.radius1
                            + joints[gearData.gearCenter2Index].ay * gearData.radius2) /
                           (gearData.radius1 + gearData.radius2);
                }
            }
            foreach (var j in joints)
            {
                if (j.Link2 == null && !unknownObjects.Contains(j))
                {
                    var refJoint = j.Link1.ReferenceJoint1;
                    j.ax = refJoint.ax + (refJoint.x - j.x) * j.Link1.Velocity * j.Link1.Velocity
                        + (refJoint.y - j.y) * j.Link1.Acceleration;
                    j.ay = refJoint.ay + (refJoint.y - j.y) * j.Link1.Velocity * j.Link1.Velocity
                        + (j.x - refJoint.x) * j.Link1.Acceleration;
                }
            }
            return true;
        }

        protected override double[] GetInitialGuess(int length)
        {
            var x = new double[length];
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o is Link)
                    x[index++] = ((Link)o).Acceleration;
                else if (o is Joint)
                {
                    x[index++] = ((Joint)o).ax;
                    x[index++] = ((Joint)o).ay;
                }
                else if (o is Tuple<Link, Joint>)
                    x[index++] = ((Tuple<Link, Joint>)o).Item2.SlideAcceleration;
            }
            return x;
        }


        protected override JointToJointEquation MakeJointToJointEquations(Joint kJoint, Joint jJoint, Link l, bool jointKIsKnown, bool jointJIsKnown, bool linkIsKnown)
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
        internal VelocitySolver(List<Joint> joints, List<Link> links, int firstInputJointIndex, int inputJointIndex,
            int inputLinkIndex, double inputSpeed, Dictionary<int, GearData> gearsData, double averageLength)
            : base(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, inputSpeed, gearsData)
        {
            maximumJointValue = Constants.JointVelocityLimitFactor * averageLength * Math.Abs(inputSpeed);
            maximumLinkValue = Constants.LinkVelocityLimitFactor * Math.Abs(inputSpeed);
        }
        protected override void SetInitialInputAndGroundStates()
        {
            RecursivelySetLinkVelocityThroughPJoints(groundLink, new List<Link>(), 0.0, true);
            if (inputJoint.TypeOfJoint == JointType.R)
            {
                RecursivelySetLinkVelocityThroughPJoints(inputLink, new List<Link>(), inputSpeed, true);
                var xGnd = inputJoint.x;
                var yGnd = inputJoint.y;
                foreach (var j in inputLink.joints)
                {
                    if (j.SlidingWithRespectTo(inputLink)) continue;
                    j.vx = inputSpeed * (yGnd - j.y);
                    j.vy = inputSpeed * (j.x - xGnd);
                }
            }
            else if (inputJoint.TypeOfJoint == JointType.P)
            {
                var vx = inputSpeed * Math.Cos(inputJoint.SlideAngle);
                var vy = inputSpeed * Math.Sin(inputJoint.SlideAngle);
                inputJoint.SlideVelocity = inputSpeed;
                foreach (var j in inputLink.joints)
                {
                    if (j.SlidingWithRespectTo(inputLink)) continue;
                    j.vx = vx;
                    j.vy = vy;
                }
            }
            else throw new Exception("Currently only R or P can be the input joints.");
        }


        protected override Boolean PutStateVarsBackInJointsAndLinks(double[] x)
        {
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o is Link)
                {
                    var v = x[index++];
                    if (Math.Abs(v) > maximumJointValue) return false;
                    ((Link)o).Velocity = v;
                }
                else if (o is Joint)
                {
                    var vx = x[index++];
                    var vy = x[index++];
                    if (Math.Abs(vx) > maximumJointValue || Math.Abs(vy) > maximumJointValue) return false;
                    ((Joint)o).vx = vx;
                    ((Joint)o).vy = vy;
                }
                else if (o is Tuple<Link, Joint>)
                    ((Tuple<Link, Joint>)o).Item2.SlideVelocity = x[index++];
            }
            if (gearsData != null)
            {
                foreach (var gearData in gearsData.Values)
                {
                    var j = joints[gearData.gearTeethIndex];
                    j.vx = (joints[gearData.gearCenter1Index].vx * gearData.radius1
                            + joints[gearData.gearCenter2Index].vx * gearData.radius2) /
                           (gearData.radius1 + gearData.radius2);
                    j.vy = (joints[gearData.gearCenter1Index].vy * gearData.radius1
                            + joints[gearData.gearCenter2Index].vy * gearData.radius2) /
                           (gearData.radius1 + gearData.radius2);
                }
            }
            foreach (var j in joints)
            {
                if (j.Link2 == null && !unknownObjects.Contains(j))
                {
                    var refJoint = j.Link1.ReferenceJoint1;
                    j.vx = refJoint.vx + (refJoint.y - j.y) * j.Link1.Velocity;
                    j.vy = refJoint.vy + (j.x - refJoint.x) * j.Link1.Velocity;
                }
            }
            return true;
        }
        protected override double[] GetInitialGuess(int length)
        {
            var x = new double[length];
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o is Link)
                    x[index++] = ((Link)o).VelocityLast;
                else if (o is Joint)
                {
                    x[index++] = ((Joint)o).vxLast;
                    x[index++] = ((Joint)o).vyLast;
                }
                else if (o is Tuple<Link, Joint>)
                    x[index++] = ((Tuple<Link, Joint>)o).Item2.SlideVelocity;
            }
            return x;
        }
        protected override JointToJointEquation MakeJointToJointEquations(Joint kJoint, Joint jJoint, Link l, bool jointKIsKnown, bool jointJIsKnown,
            bool linkIsKnown)
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
        protected readonly List<Joint> joints;
        protected readonly int firstInputJointIndex;
        protected readonly int inputJointIndex;
        protected readonly double inputSpeed;
        protected double maximumJointValue;
        protected double maximumLinkValue;
        protected readonly Joint inputJoint;
        protected readonly Joint groundReferenceJoint;
        protected readonly List<EquationBase> equations;
        protected readonly Link inputLink;
        protected readonly Link groundLink;
        protected readonly int numUnknowns;
        protected readonly double[,] A;
        protected readonly double[] b;
        protected readonly List<object> unknownObjects;
        private int[][] matrixOrders;
        protected readonly Dictionary<int, GearData> gearsData;

        /// <summary>
        /// Initializes a new instance of the <see cref="VelocitySolver" /> class.
        /// </summary>
        /// <param name="joints">The joints.</param>
        /// <param name="links">The links.</param>
        /// <param name="firstInputJointIndex">First index of the input joint.</param>
        /// <param name="inputJointIndex">Index of the input joint.</param>
        /// <param name="inputLinkIndex">Index of the input link.</param>
        /// <param name="inputSpeed">The input speed.</param>
        /// <param name="gearsData">The gears data.</param>
        /// <exception cref="Exception">The number of equations, " + numEquations + ", must be as big as the " +
        ///                       "number of unknowns, " + numUnknowns</exception>
        /// <exception cref="System.Exception">Currently only R or P can be the input joints.</exception>
        internal VelocityAndAccelerationSolver(List<Joint> joints, List<Link> links, int firstInputJointIndex, int inputJointIndex,
                                             int inputLinkIndex, double inputSpeed, Dictionary<int, GearData> gearsData)
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
            this.inputSpeed = inputSpeed;
            this.gearsData = gearsData;
            equations = new List<EquationBase>();

            /************ Set up unknown objects ************/
            unknownObjects = new List<object>();
            unknownObjects.AddRange(links);
            unknownObjects.AddRange(joints);
            foreach (var j in joints)
            {
                if (j.JustATracer) unknownObjects.Remove(j);
                if (j.TypeOfJoint == JointType.RP || j.TypeOfJoint == JointType.P)
                    unknownObjects.Add(new Tuple<Link, Joint>(j.Link1, j));
            }
            /* Since links connected by a P joint rotate at same speed, we need to remove those from the unknown list. */
            /**** Set velocity of any links connected to input by a P joint and remove link from unknowns ****/
            RecursivelyRemoveKnownLinks(groundLink);
            /**** Set velocity of any links connected to ground by a P joint and remove link from unknowns ****/
            RecursivelyRemoveKnownLinks(inputLink);
            unknownObjects.RemoveAll(o => groundLink.joints.Contains(o) && ((Joint)o).FixedWithRespectTo(groundLink));
            unknownObjects.RemoveAll(o => inputLink.joints.Contains(o) && ((Joint)o).FixedWithRespectTo(inputLink));
            if (inputJoint.TypeOfJoint == JointType.P)
                unknownObjects.RemoveAll(o => o is Tuple<Link, Joint> && ((Tuple<Link, Joint>)o).Item2 == inputJoint);
            /************ Set up Equations ************/
            #region go through the links to identify equations from joint-to-joint on a particular link
            /* first address all the links at the beginning of the list (the ones that are not input or ground) */
            foreach (var l in links)
            {
                var r = l.ReferenceJoint1;
                var l_is_unknown = unknownObjects.Contains(l);
                var r_is_unknown = unknownObjects.Contains(r);
                foreach (var j in l.joints.Where(j => j != r && j.Link2 != null))
                {
                    var j_is_unknown = unknownObjects.Contains(j);
                    if (l_is_unknown || r_is_unknown || j_is_unknown)
                        equations.Add(MakeJointToJointEquations(j, r, l, !j_is_unknown, !r_is_unknown, !l_is_unknown));
                    if (!r_is_unknown && !j_is_unknown) l_is_unknown = false; // not sure this will ever be used, but if both joints are
                    // known, then the first pass will provide a means for finding
                    // link angular velocity/acceleration. 
                }
            }
            #endregion
            #region go through the joints to identify any P-joint link-to-link relationships
            /* Then there is the matter of P-joint additional equations. Notice above that we remove any chains of links 
             * connected to ground or input via P-joints as these have the known angular velocities and accelerations. 
             * Well, we may need to add equations for clusters of links connected by P-joints that are not on ground
             * e.g. R-R-P-R. */
            foreach (var j in joints)
                if (j.TypeOfJoint == JointType.P && unknownObjects.Contains(j.Link1))
                    equations.Add(new EqualLinkToLinkStateVarEquation(j.Link1, j.Link2));
            #endregion

            /**** Set up equations. Number of unknowns is 2*unknown_joints + 1*unknown_links. ****/
            foreach (var unknownObject in unknownObjects)
            {
                if (unknownObject is Joint) numUnknowns += 2;
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
            if (numEquations < numUnknowns) throw new Exception("The number of equations, " + numEquations + ", must be as big as the " +
                      "number of unknowns, " + numUnknowns);
            DefineTwoMatrixOrders(rowNonZeroes);
        }

        private void RecursivelyRemoveKnownLinks(Link l)
        {
            if (!unknownObjects.Contains(l)) return;
            unknownObjects.Remove(l);
            foreach (var pJoint in l.joints.Where(j => j.TypeOfJoint == JointType.P))
                RecursivelyRemoveKnownLinks(pJoint.OtherLink(l));
        }

        protected void RecursivelySetLinkVelocityThroughPJoints(Link l, List<Link> LinksAlreadySet,
            double value, Boolean thisIsVelocity)
        {
            if (LinksAlreadySet.Contains(l)) return;
            LinksAlreadySet.Add(l);
            if (thisIsVelocity) l.Velocity = value;
            else l.Acceleration = value;
            foreach (var pJoint in l.joints.Where(j => j.TypeOfJoint == JointType.P))
                RecursivelySetLinkVelocityThroughPJoints(pJoint.OtherLink(l), LinksAlreadySet, value, thisIsVelocity);
        }

        private void DefineTwoMatrixOrders(List<List<int>> rowNonZeroes)
        {
            matrixOrders = new int[2][];
            orderFound = new[] { false, false };
            var now = DateTime.Now;

            var forwardTask = Task.Factory.StartNew(() => DepthFirstToFindOrder(rowNonZeroes, new List<int>(), 0, now));
            var backwardTask = Task.Factory.StartNew(() => DepthFirstToFindOrder(rowNonZeroes, new List<int>(), 1, now));
            Task.WaitAll(forwardTask, backwardTask);

            if (matrixOrders[0] == null & matrixOrders[1] == null) return;
            //throw new Exception("No valid matrix formulations found.");
            if (matrixOrders[0] == null)
            {
                matrixOrders[0] = matrixOrders[1];
                matrixOrders[1] = null;
            }
        }


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

        protected abstract JointToJointEquation MakeJointToJointEquations(Joint kJoint, Joint jJoint, Link l, bool jointKIsKnown, bool jointJIsKnown,
            bool linkIsKnown);

        protected abstract void SetInitialInputAndGroundStates();
        protected abstract Boolean PutStateVarsBackInJointsAndLinks(double[] x);
        protected abstract double[] GetInitialGuess(int length);


        internal Boolean Solve()
        {
            SetInitialInputAndGroundStates();
            try
            {
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
                if (rowOrdering == null) return false;
                for (int j = 0; j < numUnknowns; j++)
                {
                    StarMath.SetRow(j, A, rows[rowOrdering[j]]);
                    b[j] = answers[rowOrdering[j]];
                }

                var x = StarMath.solve(A, b);
                if (x.Any(value => Double.IsInfinity(value) || Double.IsNaN(value))) return false;
                if (x.Any() && x.All(Constants.sameCloseZero)) return false;
                return PutStateVarsBackInJointsAndLinks(x);

            }
            catch { return false; }
        }



        private int[] ChooseBestRowOrder(List<double[]> rows)
        {
            if (matrixOrders[0] == null && matrixOrders[1] == null) return null;
            if (matrixOrders[0] == null) return matrixOrders[1];
            if (matrixOrders[1] == null) return matrixOrders[0];
            var worstOrder0Value = 1.0;
            var worstOrder1Value = 1.0;

            for (int i = 0; i < numUnknowns; i++)
            {
                var value = MultiplicativeDistanceToOne(rows[matrixOrders[0][i]][i]);
                if (value == 0.0) return matrixOrders[1];
                if (worstOrder0Value > value) worstOrder0Value = value;
                value = MultiplicativeDistanceToOne(rows[matrixOrders[1][i]][i]);
                if (value == 0.0) return matrixOrders[0];
                if (worstOrder1Value > value) worstOrder1Value = value;
            }
            if (worstOrder0Value >= worstOrder1Value)
                return matrixOrders[0];
            return matrixOrders[1];
        }
        internal double MultiplicativeDistanceToOne(double x)
        {
            if (double.IsInfinity(x) || double.IsNaN(x) || x == 0.0) return 0;
            return (Math.Abs(x) > 1) ? 1 / Math.Abs(x) : Math.Abs(x);

        }
    }
}

