#region

using System;
using System.Linq;
using System.Collections.Generic;
using OptimizationToolbox;
using System.Collections;
using StarMathLib;

#endregion

namespace PlanarMechanismSimulator.VelocityAndAcceleration
//at time t=0; all acceleration and velocity are zero
{
    public class AccelerationSolver
    {
        private readonly List<joint> joints;
        private readonly List<link> links;
        private readonly int firstInputJointIndex;
        private readonly int inputJointIndex;
        private readonly int inputLinkIndex;
        private readonly double inputSpeed;

        private readonly joint inputJoint;
        private readonly List<EquationBase> equations;
        private readonly link inputLink;
        private readonly link groundLink;
        private readonly int numUnknowns;
        private readonly double[,] A;
        private readonly double[] b;
        private readonly List<object> unknownObjects;

        /// <summary>
        /// Initializes a new instance of the <see cref="AccelerationSolver" /> class.
        /// </summary>
        /// <param name="joints">The joints.</param>
        /// <param name="links">The links.</param>
        /// <param name="firstInputJointIndex">First index of the input joint.</param>
        /// <param name="inputJointIndex">Index of the input joint.</param>
        /// <param name="inputLinkIndex">Index of the input link.</param>
        /// <param name="InputSpeed">The input speed.</param>
        /// <exception cref="System.Exception">Currently only R or P can be the input joints.</exception>
        public AccelerationSolver(List<joint> joints, List<link> links, int firstInputJointIndex, int inputJointIndex,
                              int inputLinkIndex,
                              double InputSpeed)
        {
            /************ Initialization ************/
            this.joints = joints;
            this.inputJointIndex = inputJointIndex;
            inputJoint = joints[inputJointIndex];
            this.links = links;
            this.firstInputJointIndex = firstInputJointIndex;
            this.inputLinkIndex = inputLinkIndex;
            inputLink = links[inputLinkIndex];
            groundLink = links[links.Count - 1];
            this.joints = joints;
            this.links = links;
            inputSpeed = InputSpeed;
            equations = new List<EquationBase>();

            unknownObjects = new List<object>();
            /************ Set up Equations ************/
            for (int i = 0; i < links.Count; i++)
            {
                var l = links[i];
                if (i < inputLinkIndex) unknownObjects.Add(l);
                else if (i == inputLinkIndex)
                {
                    if (inputJoint.jointType == JointTypes.R)
                        l.Acceleration = inputSpeed;
                    else if (inputJoint.jointType == JointTypes.P)
                    {
                        l.Acceleration = 0.0;
                        var vx = inputSpeed * Math.Cos(inputJoint.SlideAngle);
                        var vy = inputSpeed * Math.Sin(inputJoint.SlideAngle);
                        for (int j = firstInputJointIndex; j <= inputJointIndex; j++)
                        {
                            if (!joints[j].SlidingWithRespectTo(l))
                            {
                                joints[j].vx = vx;
                                joints[j].vy = vy;
                            }
                        }
                    }
                    else throw new Exception("Currently only R or P can be the input joints.");
                }
                else if (i == inputLinkIndex + 1) l.Acceleration = 0.0;
                for (int j = 0; j < l.joints.Count - 1; j++)
                    for (int k = j + 1; k < l.joints.Count; k++)
                    {
                        var jJoint = l.joints[j];
                        var kJoint = l.joints[k];
                        var jointJIsKnown = (j >= firstInputJointIndex &&
                                             (jJoint.jointType == JointTypes.R || jJoint.jointType == JointTypes.G));
                        var jointKIsKnown = (k >= firstInputJointIndex &&
                                             (kJoint.jointType == JointTypes.R || kJoint.jointType == JointTypes.G));
                        if (!jointJIsKnown || !jointKIsKnown)
                        {
                            if (jJoint.SlidingWithRespectTo(l) && kJoint.SlidingWithRespectTo(l))
                                equations.Add(new AccelerationEquationForDoubleSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown));
                            else if (joints[j].SlidingWithRespectTo(l))
                                equations.Add(new AccelerationEquationForFixedToSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown));
                            else if (joints[k].SlidingWithRespectTo(l))
                                equations.Add(new AccelerationEquationForFixedToSlide(kJoint, jJoint, l, jointJIsKnown, jointKIsKnown));
                            else
                                equations.Add(new AccelerationEquationForFixedJoints(l.joints[j], l.joints[k], l,
                                                                                 jointJIsKnown, jointKIsKnown));
                        }
                    }
            }
            for (int i = 0; i < inputJointIndex; i++)
            {
                var j = joints[i];
                unknownObjects.Add(j);
                if (j.jointType == JointTypes.P)
                {
                    numUnknowns--;
                    var l1Known = (j.Link1 == inputLink || j.Link1 == groundLink);
                    var l2Known = (j.Link2 == inputLink || j.Link2 == groundLink);
                    if (!l1Known && !l2Known)
                        equations.Add(new EqualLinkAccelerationEquation(j.Link1, j.Link2));
                }
            }
            /**** Set velocity of any P-links connected to input and remove link from unknowns ****/
            for (int i = firstInputJointIndex; i < inputJointIndex; i++)
                if (joints[i].jointType == JointTypes.P)
                {
                    var otherLink = joints[i].OtherLink(links[inputJointIndex]);
                    otherLink.Acceleration = links[inputLinkIndex].Acceleration;
                    unknownObjects.Remove(otherLink);
                }
            /**** But the velocities of any P-joints or RP-joints connected to ground are unknown. ****/
            for (int i = inputJointIndex + 1; i < joints.Count; i++)
                if (joints[i].jointType == JointTypes.P || joints[i].jointType == JointTypes.RP)
                    unknownObjects.Add(joints[i]);
            /**** Then there is the matter of the sliding velocities.. ****/
            for (int i = 0; i < joints.Count; i++)
                if (joints[i].jointType == JointTypes.P || joints[i].jointType == JointTypes.RP)
                    unknownObjects.Add(new Tuple<link, joint>(joints[i].Link1, joints[i]));
            /**** Set up equations. Number of unknowns is 2*unknown-joints + 1*unknown links. ****/
            numUnknowns = unknownObjects.Count + unknownObjects.Count(o => o is joint);
            A = new double[numUnknowns, numUnknowns];
            b = new double[numUnknowns];
            /* While unknownObjects is saved, the data for their positions in the matrix is also stored in the equation objects*/
            foreach (var eq in equations)
            {
                eq.CaptureUnknownIndicies(unknownObjects);
                eq.unkLength = numUnknowns;
            }
        }

        internal Boolean Solve()
        {
            InitializeGroundAndInputSpeedAndAcceleration();
            var rows = new List<Tuple<double, double[], double>>();
            foreach (var eq in equations)
            {
                if (eq is JointToJointEquation)
                {
                    var row = ((JointToJointEquation)eq).GetRow1Coefficients();
                    rows.Add(new Tuple<double, double[], double>(DistanceFromOne(row), row,
                                                                 ((JointToJointEquation)eq).GetRow1Constant()));

                    row = ((JointToJointEquation)eq).GetRow2Coefficients();
                    rows.Add(new Tuple<double, double[], double>(DistanceFromOne(row), row,
                                                                 ((JointToJointEquation)eq).GetRow2Constant()));
                }
                else
                {
                    var row = ((EqualLinkAccelerationEquation)eq).GetRowCoefficients();
                    rows.Add(new Tuple<double, double[], double>(DistanceFromOne(row), row, 0.0));
                }
            }
            /* there are usually more equations than unknowns. So, we search for the equations that would cause the
             * matrix to be poorly scaled and remove them. However, we cannot remove row-equations that have are 
             * solving an unknown that no other row-equations are solving. */
            rows = rows.OrderByDescending(f => f.Item1).ToList();
            while (rows.Count > numUnknowns)
            {
                for (int i = 0; i < rows.Count; i++)
                    if (!HasUniqueNonZeroColumn(rows, i))
                    {
                        rows.RemoveAt(i);
                        break;
                    }
            }
            for (int i = 0; i < numUnknowns; i++)
            {
                StarMath.SetRow(i, A, rows[i].Item2);
                b[i] = rows[i].Item3;
            }
            var x = StarMath.solve(A, b);
            if (x.Any(value => double.IsInfinity(value) || double.IsNaN(value))) return false;
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o is link) ((link)o).Acceleration = x[index++];
                else if (o is joint)
                {
                    ((joint)o).vx = x[index++];
                    ((joint)o).vy = x[index++];
                }
            }
            return true;
        }

        private bool HasUniqueNonZeroColumn(List<Tuple<double, double[], double>> rows, int i)
        {
            for (int j = 0; j < numUnknowns; j++)
            {
                if (rows[i].Item2[j] == 0.0) continue;
                if (!rows.Any(r => r != rows[i] && r.Item2[j] != 0.0)) return true;
            }
            return false;

        }
        private double DistanceFromOne(double[] row)
        {
            var rowMax = row.Max();
            if (rowMax == 0.0) rowMax = 1.0;
            else if (Math.Abs(rowMax) < 1) rowMax = 1 / rowMax;
            var rowMin = row.Min();
            if (rowMin == 0.0) rowMin = 1.0;
            else if (Math.Abs(rowMin) < 1) rowMin = 1 / rowMin;
            return Math.Max(Math.Abs(rowMax), Math.Abs(rowMin));
        }
        private void InitializeGroundAndInputSpeedAndAcceleration()
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

    }

}

