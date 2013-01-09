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
    /// Velocity Solver works by creating equations corresponding to the relative velocity equations.
    /// </summary>
    public class VelocitySolver
    {
        private readonly List<joint> joints;
        private readonly int firstInputJointIndex;
        private readonly int inputJointIndex;
        private readonly double inputSpeed;

        private readonly joint inputJoint;
        private readonly List<EquationBase> equations;
        private readonly link inputLink;
        private readonly link groundLink;
        private readonly int numUnknowns;
        private readonly double[,] A;
        private readonly double[] b;
        private readonly List<object> unknownObjects;
        private readonly int numEquations;

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
        public VelocitySolver(List<joint> joints, List<link> links, int firstInputJointIndex, int inputJointIndex,
                              int inputLinkIndex,
                              double InputSpeed)
        {
            /************ Initialization ************/
            this.joints = joints;
            this.inputJointIndex = inputJointIndex;
            inputJoint = joints[inputJointIndex];
            this.firstInputJointIndex = firstInputJointIndex;
            inputLink = links[inputLinkIndex];
            groundLink = links[links.Count - 1];
            this.joints = joints;
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
                        l.Velocity = inputSpeed;
                    else if (inputJoint.jointType == JointTypes.P)
                    {
                        l.Velocity = 0.0;
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
                else if (i == inputLinkIndex + 1) l.Velocity = 0.0;
                for (int j = 0; j < l.joints.Count - 1; j++)
                    for (int k = j + 1; k < l.joints.Count; k++)
                    {
                        var jJoint = l.joints[j];
                        var kJoint = l.joints[k];
                        var jointJIsKnown = ((jJoint.isGround && (jJoint.jointType == JointTypes.R || jJoint.jointType == JointTypes.G))
                            || ((l == inputLink || jJoint.OtherLink(l)==inputLink)&& !jJoint.SlidingWithRespectTo(inputLink)));
                        var jointKIsKnown = ((kJoint.isGround && (kJoint.jointType == JointTypes.R || kJoint.jointType == JointTypes.G))
                            || ((l == inputLink || kJoint.OtherLink(l) == inputLink) && !kJoint.SlidingWithRespectTo(inputLink)));
                        if (!jointJIsKnown || !jointKIsKnown)
                        {
                            if (jJoint.SlidingWithRespectTo(l) && kJoint.SlidingWithRespectTo(l))
                                equations.Add(new VelocityEquationForDoubleSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown,
                                    (i >= inputJointIndex)));
                            else if (joints[j].SlidingWithRespectTo(l))
                                equations.Add(new VelocityEquationForFixedToSlide(jJoint, kJoint, l, jointJIsKnown, jointKIsKnown,
                                    (i >= inputJointIndex)));
                            else if (joints[k].SlidingWithRespectTo(l))
                                equations.Add(new VelocityEquationForFixedToSlide(kJoint, jJoint, l, jointJIsKnown, jointKIsKnown,
                                    (i >= inputJointIndex)));
                            else
                                equations.Add(new VelocityEquationForFixedJoints(l.joints[j], l.joints[k], l,
                                                                                 jointJIsKnown, jointKIsKnown, (i >= inputJointIndex)));
                        }
                    }
            }
            for (int i = 0; i < firstInputJointIndex; i++)
            {
                var j = joints[i];
                unknownObjects.Add(j);
                if (j.jointType == JointTypes.P)
                    equations.Add(new EqualLinkToLinkStateVarEquation(j.Link1, j.Link2));
            }
            /**** Set velocity of any P-links connected to input and remove link from unknowns ****/
            for (int i = firstInputJointIndex; i < inputJointIndex; i++)
                if (joints[i].jointType == JointTypes.P)
                {
                    var otherLink = joints[i].OtherLink(links[inputJointIndex]);
                    otherLink.Velocity = links[inputLinkIndex].Velocity;
                    unknownObjects.Remove(otherLink);
                }
            /**** But the velocities of any P-joints or RP-joints connected to ground are unknown. ****/
            for (int i = inputJointIndex + 1; i < joints.Count; i++)
                if (joints[i].jointType == JointTypes.P || joints[i].jointType == JointTypes.RP)
                    unknownObjects.Add(joints[i]);
            /**** Then there is the matter of the sliding velocities.. ****/
            foreach (joint j in joints)
                if (j.jointType == JointTypes.P || j.jointType == JointTypes.RP)
                    unknownObjects.Add(new Tuple<link, joint>(j.Link1, j));
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
            numEquations = 0;
            foreach (var eq in equations)
            {
                if (eq is JointToJointEquation) numEquations += 2;
                else numEquations++;
                eq.CaptureUnknownIndicies(unknownObjects);
                eq.unkLength = numUnknowns;
            }
        }

        internal Boolean Solve()
        {
            InitializeInputSpeed();
            var rows = new double[numEquations][];
            var answers = new double[numEquations];
            var i = 0;
            foreach (var eq in equations)
            {
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
            }
            if (!Constants.CreateBestMatrixAndB(numUnknowns, numEquations, rows, A,answers, b)) return false;
            var x = StarMath.solve(A, b);
            if (x.Any(value => double.IsInfinity(value) || double.IsNaN(value))) return false;
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
            return true;
        }


        private void InitializeInputSpeed()
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

