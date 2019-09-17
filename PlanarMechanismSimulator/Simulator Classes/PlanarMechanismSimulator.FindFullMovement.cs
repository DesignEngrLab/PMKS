// ***********************************************************************
// Assembly         : PlanarMechanismKinematicSimulator
// Author           : Matt
// Created          : 06-10-2015
//
// Last Modified By : Matt
// Last Modified On : 06-28-2015
// ***********************************************************************
// <copyright file="PlanarMechanismSimulator.FindFullMovement.cs" company="">
//     Copyright ©  2014
// </copyright>
// <summary></summary>
// ***********************************************************************

using OptimizationToolbox;
using PMKS.PositionSolving;
using PMKS.VelocityAndAcceleration;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace PMKS
{
    /// <summary>
    /// Class Simulator.
    /// </summary>
    public partial class Simulator : IDependentAnalysis
    {
        /// <summary>
        /// The use error method
        /// </summary>
        private bool useErrorMethod;

        /// <summary>Finds the full movement.</summary>
        /// <returns>
        ///   <c>true</c> if simulation was successful, <c>false</c> otherwise.</returns>
        /// <exception cref="Exception">Either the smoothing error angle delta or the time step must be specified.</exception>
        /// <exception cref="System.Exception">Either the smoothing error angle delta or the time step must be specified.</exception>
        public bool FindFullMovement()
        {
            if (double.IsNaN(DeltaAngle) && double.IsNaN(FixedTimeStep) && double.IsNaN(MaxSmoothingError))
            {
                throw new Exception(
                     "Either the smoothing error angle delta or the time step must be specified.");
                return false;
            }
            useErrorMethod = (!double.IsNaN(MaxSmoothingError) && MaxSmoothingError > 0);

            #region Set up initial point parameters (x, x-dot, x-double-dot, etc.)

            double[,] initJointParams, initLinkParams;

            SetInitialVelocityAndAcceleration(SimulationJoints, SimulationLinks, out initJointParams, out initLinkParams);

            JointParameters = new TimeSortedList { { 0.0, initJointParams } };
            LinkParameters = new TimeSortedList { { 0.0, initLinkParams } };

            InputRange = new[] { inputLink.AngleInitial, inputLink.AngleInitial };
            if (double.IsNaN(InputSpeed) || InputSpeed == 0.0) InputSpeed = Constants.DefaultInputSpeed;

            #endregion

#if DEBUGSERIAL
            Simulate(SimulationJoints, SimulationLinks, true);
#else
            List<Joint> backwardJoints;
            List<Link> backwardLinks;
            CopyJointsAndLinksForBackwards(SimulationJoints, SimulationLinks, out backwardJoints, out backwardLinks);

            /*** Stepping Forward in Time ***/
            var forwardTask = Task.Factory.StartNew(() => Simulate(SimulationJoints, SimulationLinks, true));

            /*** Stepping Backward in Time ***/
            var backwardTask = Task.Factory.StartNew(() => Simulate(backwardJoints, backwardLinks, false));
            Task.WaitAll(forwardTask, backwardTask);
#endif
            if (this.JointParameters.Count < 2) return false;
            DefineMovementCharacteristics();
            if (invertedToSolveNonGroundInput)
                TransposeLinkAndJointParameters();
            return true;
        }

        /// <summary>
        /// Copies the joints and links for backwards.
        /// </summary>
        /// <param name="AllJoints">All joints.</param>
        /// <param name="AllLinks">All links.</param>
        /// <param name="backwardJoints">The backward joints.</param>
        /// <param name="backwardLinks">The backward links.</param>
        private static void CopyJointsAndLinksForBackwards(List<Joint> AllJoints, List<Link> AllLinks,
            out List<Joint> backwardJoints, out List<Link> backwardLinks)
        {
            backwardJoints = AllJoints.Select(j => j.copy()).ToList();
            backwardLinks = AllLinks.Select(c => c.Copy()).ToList();
            foreach (var j in backwardJoints)
            {
                j.Link1 = backwardLinks[AllLinks.IndexOf(j.Link1)];
                if (j.Link2 != null)
                    j.Link2 = backwardLinks[AllLinks.IndexOf(j.Link2)];
            }
            for (var i = 0; i < AllLinks.Count; i++)
            {
                var backwardLink = backwardLinks[i];
                var forwardLink = AllLinks[i];
                foreach (var j in forwardLink.joints)
                {
                    var jointIndex = AllJoints.IndexOf(j);
                    backwardLink.joints.Add(backwardJoints[jointIndex]);
                    if (j == forwardLink.ReferenceJoint1)
                        backwardLink.ReferenceJoint1 = backwardJoints[jointIndex];
                }
            }
        }

        /// <summary>
        /// Defines the movement characteristics.
        /// </summary>
        private void DefineMovementCharacteristics()
        {
            BeginTime = JointParameters.Times[0];
            EndTime = JointParameters.Times.Last();
            InitializeQueryVars();
            for (var i = 0; i < NumAllJoints; i++)
            {
                var j = SimulationJoints[i];
                if (j.TypeOfJoint == JointType.R) continue;
                j.OrigSlidePosition = JointParameters[0.0][oIOSJ[i], 6];
                j.MinSlidePosition = JointParameters.Parameters.Min(jp => jp[oIOSJ[i], 6]);
                j.MaxSlidePosition = JointParameters.Parameters.Max(jp => jp[oIOSJ[i], 6]);
            }
            if (lessThanFullRotation())
            {
                //if the crank input couldn't rotate all the way around,then this is easy... 
                CycleType = CycleTypes.LessThanFullCycle;
                return;
            }
            // if the simulation did go all the way around then, we should be careful to ensure is connects correctly.
            var cyclePeriodTime = Constants.FullCircle / Math.Abs(InputSpeed);
            if (DoesMechanismRepeatOnCrankCycle(cyclePeriodTime))
            {
                BeginTime = 0.0;
                EndTime = cyclePeriodTime;
                CycleType = CycleTypes.OneCycle;
                while (JointParameters.Times.Last() >= EndTime)
                {
                    var time = JointParameters.Times.Last();
                    var parameters = JointParameters.Parameters.Last();
                    JointParameters.RemoveAt(JointParameters.LastIndex);
                    time -= cyclePeriodTime;
                    JointParameters.AddNearBegin(time, parameters);

                    parameters = LinkParameters.Parameters.Last();
                    LinkParameters.RemoveAt(LinkParameters.LastIndex);
                    LinkParameters.AddNearEnd(time, parameters);
                }
                while (JointParameters.Times[0] < 0.0)
                {
                    var time = JointParameters.Times[0];
                    var parameters = JointParameters.Parameters[0];
                    JointParameters.RemoveAt(0);
                    time += cyclePeriodTime;
                    JointParameters.AddNearEnd(time, parameters);

                    parameters = LinkParameters.Parameters[0];
                    LinkParameters.RemoveAt(0);
                    LinkParameters.AddNearEnd(time, parameters);
                }
            }
            else
            {
                CycleType = CycleTypes.MoreThanOneCycle;
            }
            InitializeQueryVars();
        }

        /// <summary>
        /// Transposes the link and joint parameters.
        /// </summary>
        private void TransposeLinkAndJointParameters()
        {
            var fixedGroundJoints = GroundLink.joints.Where(j => j.FixedWithRespectTo(GroundLink)).ToList();
            var gndJointIndex = SimulationJoints.IndexOf(fixedGroundJoints[0]);
            var gndLinkIndex = SimulationLinks.IndexOf(GroundLink);
            for (var i = 0; i <= LinkParameters.LastIndex; i++)
                LinkParameters[i].Value[gndLinkIndex, 0] -= GroundLink.AngleInitial;
            TransposeJointPosition(gndJointIndex, gndLinkIndex);
#if DEBUGSERIAL
            TransposeJointVelocity(gndJointIndex, gndLinkIndex);
            TransposeJointAcceleration(gndJointIndex, gndLinkIndex);
            
            TransposeLinkStateVariables(gndLinkIndex, 0);
            TransposeLinkStateVariables(gndLinkIndex, 1);
            TransposeLinkStateVariables(gndLinkIndex, 2);
#else
            /* These joint velocity and acceleration functions don't depend on each other, but they
                 * require the position to be complete and that the link state variables stay untouched. */
            var task1 = Task.Factory.StartNew(() => TransposeJointVelocity(gndJointIndex, gndLinkIndex));
            var task2 = Task.Factory.StartNew(() => TransposeJointAcceleration(gndJointIndex, gndLinkIndex));
            Task.WaitAll(task1, task2);

            var task3 = Task.Factory.StartNew(() => TransposeLinkStateVariables(gndLinkIndex, 0));
            var task4 = Task.Factory.StartNew(() => TransposeLinkStateVariables(gndLinkIndex, 1));
            var task5 = Task.Factory.StartNew(() => TransposeLinkStateVariables(gndLinkIndex, 2));
            Task.WaitAll(task3, task4, task5);
#endif
        }

        /// <summary>
        /// Transposes the link state variables.
        /// </summary>
        /// <param name="gndLinkIndex">Index of the GND link.</param>
        /// <param name="varIndex">Index of the variable.</param>
        private void TransposeLinkStateVariables(int gndLinkIndex, int varIndex)
        {
            for (var i = 0; i <= LinkParameters.LastIndex; i++)
            {
                var lParams = LinkParameters[i].Value;
                var offset = lParams[oIOSL[gndLinkIndex], varIndex];
                for (var j = 0; j < NumLinks; j++)
                    lParams[oIOSL[j], varIndex] -= offset;
            }
        }

        /// <summary>
        /// Transposes the joint position.
        /// </summary>
        /// <param name="gndJointIndex">Index of the GND joint.</param>
        /// <param name="gndLinkIndex">Index of the GND link.</param>
        private void TransposeJointPosition(int gndJointIndex, int gndLinkIndex)
        {
            var xOffset = SimulationJoints[gndJointIndex].xInitial;
            var yOffset = SimulationJoints[gndJointIndex].yInitial;
            for (var i = 0; i <= JointParameters.LastIndex; i++)
            {
                var jParams = JointParameters[i].Value;
                var tx = jParams[oIOSJ[gndJointIndex], 0];
                var ty = jParams[oIOSJ[gndJointIndex], 1];
                var newAngle = -LinkParameters[i].Value[oIOSL[gndLinkIndex], 0];
                var cosAngle = Math.Cos(newAngle);
                var sinAngle = Math.Sin(newAngle);
                for (var j = 0; j < NumAllJoints; j++)
                {
                    var xNew = (jParams[oIOSJ[j], 0] - tx) * cosAngle - (jParams[oIOSJ[j], 1] - ty) * sinAngle + xOffset;
                    jParams[oIOSJ[j], 1] = (jParams[oIOSJ[j], 0] - tx) * sinAngle + (jParams[oIOSJ[j], 1] - ty) * cosAngle + yOffset;
                    jParams[oIOSJ[j], 0] = xNew;
                }
            }
        }

        /// <summary>
        /// Transposes the joint velocity.
        /// </summary>
        /// <param name="gndJointIndex1">The GND joint index1.</param>
        /// <param name="gndLinkIndex">Index of the GND link.</param>
        private void TransposeJointVelocity(int gndJointIndex1, int gndLinkIndex)
        {
            for (var i = 0; i <= JointParameters.LastIndex; i++)
            {
                var jParams = JointParameters[i].Value;
                var VxGnd = jParams[oIOSJ[gndJointIndex1], 2];
                var VyGnd = jParams[oIOSJ[gndJointIndex1], 3];
                var newAngle = -LinkParameters[i].Value[oIOSL[gndLinkIndex], 0];
                var cosAngle = Math.Cos(newAngle);
                var sinAngle = Math.Sin(newAngle);
                var angularVelocity = LinkParameters[i].Value[oIOSL[gndLinkIndex], 1];
                for (var j = 0; j < NumAllJoints; j++)
                {
                    var xNew = (jParams[oIOSJ[j], 2] - VxGnd) * cosAngle - (jParams[oIOSJ[j], 3] - VyGnd) * sinAngle
                               + angularVelocity * (jParams[oIOSJ[j], 1] - jParams[oIOSJ[gndJointIndex1], 1]);
                    jParams[oIOSJ[j], 3] = (jParams[oIOSJ[j], 2] - VxGnd) * sinAngle + (jParams[oIOSJ[j], 3] - VyGnd) * cosAngle
                                    - angularVelocity * (jParams[oIOSJ[j], 0] - jParams[oIOSJ[gndJointIndex1], 0]);
                    jParams[oIOSJ[j], 2] = xNew;
                }
            }
        }

        /// <summary>
        /// Transposes the joint acceleration.
        /// </summary>
        /// <param name="gndJointIndex1">The GND joint index1.</param>
        /// <param name="gndLinkIndex">Index of the GND link.</param>
        private void TransposeJointAcceleration(int gndJointIndex1, int gndLinkIndex)
        {
            for (var i = 0; i <= JointParameters.LastIndex; i++)
            {
                var jParams = JointParameters[i].Value;
                var ax = jParams[oIOSJ[gndJointIndex1], 4];
                var ay = jParams[oIOSJ[gndJointIndex1], 5];
                var newAngle = -LinkParameters[i].Value[gndLinkIndex, 0];
                var cosAngle = Math.Cos(newAngle);
                var sinAngle = Math.Sin(newAngle);
                var angularVelocity = LinkParameters[i].Value[oIOSL[gndLinkIndex], 1];
                var angularAcceleration = LinkParameters[i].Value[oIOSL[gndLinkIndex], 2];
                for (var j = 0; j < NumAllJoints; j++)
                {
                    var rx = (jParams[oIOSJ[j], 0] - jParams[oIOSJ[gndJointIndex1], 0]);
                    var ry = (jParams[oIOSJ[j], 1] - jParams[oIOSJ[gndJointIndex1], 1]);
                    var xNew = (jParams[oIOSJ[j], 4] - ax) * cosAngle - (jParams[oIOSJ[j], 5] - ay) * sinAngle
                               + angularVelocity * angularVelocity * rx + angularAcceleration * ry;
                    jParams[oIOSJ[j], 5] = (jParams[oIOSJ[j], 4] - ax) * sinAngle + (jParams[oIOSJ[j], 5] - ay) * cosAngle
                                    + angularVelocity * angularVelocity * ry - angularAcceleration * rx;
                    jParams[oIOSJ[j], 4] = xNew;
                }
            }
        }

        /// <summary>
        /// Doeses the mechanism repeat on crank cycle.
        /// </summary>
        /// <param name="cyclePeriodTime">The cycle period time.</param>
        /// <returns><c>true</c> if XXXX, <c>false</c> otherwise.</returns>
        private bool DoesMechanismRepeatOnCrankCycle(double cyclePeriodTime)
        {
            // Does the mechanism return to the same place?
            // even though the crank goes all the way around, other joints of the mechanism may be in different places. 
            var initJointState = JointParameters.Parameters[0];
            var initLinkState = LinkParameters.Parameters[0];
            var topTime = JointParameters.Times[0] + cyclePeriodTime;
            var maxAngleError = Constants.ErrorInDeterminingCompleteCycle;
            var maxLengthError = Constants.ErrorInDeterminingCompleteCycle * AverageLength;
            if (!double.IsNaN(MaxSmoothingError) && MaxSmoothingError > 0)
            {
                maxAngleError *= Constants.SmoothingErrorRepeatFactor * Math.Sqrt(MaxSmoothingError);
                maxLengthError *= Constants.SmoothingErrorRepeatFactor * Math.Sqrt(MaxSmoothingError) * AverageLength;
            }
            for (var i = 0; i < NumLinks; i++)
            {
                var deltaLinkAngle = FindLinkAngleAtTime(topTime, oIOSL[i]) - initLinkState[oIOSL[i], 0];
                while (deltaLinkAngle > Math.PI) deltaLinkAngle -= Constants.FullCircle;
                while (deltaLinkAngle < -Math.PI) deltaLinkAngle += Constants.FullCircle;
                if (deltaLinkAngle > maxAngleError) return false;
            }
            for (var i = 0; i < NumAllJoints; i++)
            {
                var topJointLocation = FindJointPositionAtTime(topTime, oIOSJ[i]);
                var initJointLocationX = initJointState[oIOSJ[i], 0];
                var initJointLocationY = initJointState[oIOSJ[i], 1];
                if (Math.Abs(topJointLocation[0] - initJointLocationX) > maxLengthError
                    || Math.Abs(topJointLocation[1] - initJointLocationY) > maxLengthError)
                    return false;
            }
            return true;
        }

        /// <summary>
        /// Sets the initial velocity and acceleration.
        /// </summary>
        /// <param name="joints">The joints.</param>
        /// <param name="links">The links.</param>
        /// <param name="initJointParams">The initialize joint parameters.</param>
        /// <param name="initLinkParams">The initialize link parameters.</param>
        private void SetInitialVelocityAndAcceleration(List<Joint> joints, List<Link> links,
            out double[,] initJointParams, out double[,] initLinkParams)
        {
            var posFinder = new PositionFinder(joints, links, gearsData, drivingIndex);
            posFinder.UpdateSliderPosition();
            var velSolver = new VelocitySolver(joints, links, firstInputJointIndex, drivingIndex, inputLinkIndex,
                InputSpeed, gearsData, AverageLength);
            var accelSolver = new AccelerationSolver(joints, links, firstInputJointIndex, drivingIndex,
                inputLinkIndex, InputSpeed, gearsData, AverageLength);

            initJointParams = WriteJointStatesVariablesToMatrixAndToLast(joints);
            initLinkParams = WriteLinkStatesVariablesToMatrixAndToLast(links);
            var smallTimeStep = (double.IsNaN(FixedTimeStep))
                ? Constants.SmallPerturbationFraction
                : Constants.SmallPerturbationFraction * FixedTimeStep;
            if (velSolver.Solve())
            {
                for (var i = 0; i <= drivingIndex; i++)
                {
                    initJointParams[oIOSJ[i], 2] = joints[i].vxLast = joints[i].vx;
                    initJointParams[oIOSJ[i], 3] = joints[i].vyLast = joints[i].vy;
                }
                for (var i = 0; i <= inputLinkIndex; i++)
                    initLinkParams[oIOSL[i], 1] = links[i].VelocityLast = links[i].Velocity;
                if (accelSolver.Solve())
                {
                    for (var i = 0; i <= drivingIndex; i++)
                    {
                        initJointParams[oIOSJ[i], 4] = joints[i].ax;
                        initJointParams[oIOSJ[i], 5] = joints[i].ay;
                    }
                    for (var i = 0; i <= inputLinkIndex; i++)
                        initLinkParams[oIOSL[i], 2] = links[i].Acceleration;
                }
                else
                {
                    /* velocity was successfully found, but not acceleration. */
                    if (posFinder.DefineNewPositions(smallTimeStep * InputSpeed) &&
                        velSolver.Solve())
                    {
                        /* forward difference on velocities to create accelerations. */
                        for (var i = 0; i <= drivingIndex; i++)
                        {
                            initJointParams[oIOSJ[i], 4] = joints[i].ax = (joints[i].vx - joints[i].vxLast) / smallTimeStep;
                            initJointParams[oIOSJ[i], 5] = joints[i].ay = (joints[i].vy - joints[i].vyLast) / smallTimeStep;
                        }
                        for (var i = 0; i <= inputLinkIndex; i++)
                            initLinkParams[oIOSL[i], 2] = links[i].Acceleration = (links[i].Velocity - links[i].VelocityLast) / smallTimeStep;

                        /* since the position solving wrote values to joints[i].x and .y, we need to reset them, for further work. */
                        foreach (var joint in SimulationJoints)
                        {
                            joint.x = joint.xInitial;
                            joint.y = joint.yInitial;
                        }
                        foreach (var link in SimulationLinks)
                            link.Angle = link.AngleInitial;
                    }
                }
                return;
            }
            var ForwardJointParams = new double[NumAllJoints, 2];
            var ForwardLinkParams = new double[NumLinks];
            /*** Stepping Forward in Time ***/
            var forwardSuccess = posFinder.DefineNewPositions(smallTimeStep * InputSpeed);
            if (forwardSuccess)
            {
                for (var i = 0; i < NumAllJoints; i++)
                {
                    ForwardJointParams[oIOSJ[i], 0] = joints[i].x;
                    ForwardJointParams[oIOSJ[i], 1] = joints[i].y;
                }
                for (var i = 0; i < NumLinks; i++)
                    ForwardLinkParams[oIOSL[i]] = links[i].Angle;
            }
            /*** Stepping Backward in Time ***/
            var BackwardJointParams = new double[NumAllJoints, 2];
            var BackwardLinkParams = new double[NumLinks];
            var backwardSuccess = posFinder.DefineNewPositions(-smallTimeStep * InputSpeed);
            if (backwardSuccess)
            {
                for (var i = 0; i < NumAllJoints; i++)
                {
                    BackwardJointParams[oIOSJ[i], 0] = joints[i].x;
                    BackwardJointParams[oIOSJ[i], 1] = joints[i].y;
                }
                for (var i = 0; i < NumLinks; i++)
                    BackwardLinkParams[oIOSL[i]] = links[i].Angle;
            }
            if (forwardSuccess && backwardSuccess)
            {
                /* central difference puts values in init parameters. */
                for (var i = 0; i < NumAllJoints; i++)
                {
                    /* first-order central finite difference */
                    initJointParams[i, 2] = (ForwardJointParams[i, 0] - BackwardJointParams[i, 0]) / (2 * smallTimeStep);
                    initJointParams[i, 3] = (ForwardJointParams[i, 1] - BackwardJointParams[i, 1]) / (2 * smallTimeStep);
                    /* second-order central finite difference */
                    initJointParams[i, 4] = (ForwardJointParams[i, 0] - 2 * initJointParams[i, 0] +
                                             BackwardJointParams[i, 0]) / (smallTimeStep * smallTimeStep);
                    initJointParams[i, 5] = (ForwardJointParams[i, 1] - 2 * initJointParams[i, 1] +
                                             BackwardJointParams[i, 1]) / (smallTimeStep * smallTimeStep);
                }
                for (var i = 0; i < NumAllLinks; i++)
                {
                    /* first-order central finite difference */
                    initLinkParams[i, 1] = (ForwardLinkParams[i] - BackwardLinkParams[i]) / (2 * smallTimeStep);
                    /* second-order central finite difference */
                    initLinkParams[i, 2] = (ForwardLinkParams[i] - 2 * initLinkParams[i, 0] + BackwardLinkParams[i])
                                           / (smallTimeStep * smallTimeStep);
                }
            }
            else if (forwardSuccess)
            {
                /* forward difference puts values in init parameters. */
                for (var i = 0; i < NumAllJoints; i++)
                {
                    /* first-order forward finite difference */
                    initJointParams[i, 2] = (ForwardJointParams[i, 0] - initJointParams[i, 0]) / smallTimeStep;
                    initJointParams[i, 3] = (ForwardJointParams[i, 1] - initJointParams[i, 1]) / smallTimeStep;
                }
                for (var i = 0; i < NumAllLinks; i++)
                    /* first-order forward finite difference */
                    initLinkParams[i, 1] = (ForwardLinkParams[i] - initLinkParams[i, 0]) / smallTimeStep;
            }
            else if (backwardSuccess)
            {
                /* backward difference puts values in init parameters. */
                for (var i = 0; i < NumAllJoints; i++)
                {
                    /* first-order backward finite difference */
                    initJointParams[i, 2] = (initJointParams[i, 0] - BackwardJointParams[i, 0]) / smallTimeStep;
                    initJointParams[i, 3] = (initJointParams[i, 1] - BackwardJointParams[i, 1]) / smallTimeStep;
                }
                for (var i = 0; i <= NumAllLinks; i++)
                    /* first-order backward finite difference */
                    initLinkParams[i, 1] = (initLinkParams[i, 0] - BackwardLinkParams[i]) / smallTimeStep;
            }
            /* since the position solving wrote values to joints[i].x and .y, we need to reset them, for further work. */
            foreach (var joint in SimulationJoints)
            {
                joint.x = joint.xInitial;
                joint.y = joint.yInitial;
            }
            foreach (var link in SimulationLinks)
                link.Angle = link.AngleInitial;
        }

        /// <summary>
        /// Simulates the specified joints.
        /// </summary>
        /// <param name="joints">The joints.</param>
        /// <param name="links">The links.</param>
        /// <param name="Forward">The forward.</param>
        private void Simulate(List<Joint> joints, List<Link> links, Boolean Forward)
        {
            var timeStep = (Forward == (InputSpeed > 0)) ? FixedTimeStep : -FixedTimeStep;
            var startingPosChange = (Forward == (InputSpeed > 0))
                ? Constants.DefaultStepSize
                : -Constants.DefaultStepSize;
            if (inputJoint.TypeOfJoint == JointType.P) startingPosChange *= AverageLength;
            var maxLengthError = MaxSmoothingError * AverageLength;
            var currentTime = 0.0;
            Boolean validPosition;
            var posFinder = new PositionFinder(joints, links, gearsData, drivingIndex);
            var velSolver = new VelocitySolver(joints, links, firstInputJointIndex, drivingIndex, inputLinkIndex,
                InputSpeed, gearsData, AverageLength);
            var accelSolver = new AccelerationSolver(joints, links, firstInputJointIndex, drivingIndex,
                inputLinkIndex, InputSpeed, gearsData, AverageLength);
            do
            {
                #region Find Next Positions

                if (useErrorMethod)
                {
                    var k = 0;
                    double upperError;
                    do
                    {
                        timeStep = startingPosChange / InputSpeed;
                        NumericalPosition(timeStep, joints, links);
                        validPosition = posFinder.DefineNewPositions(startingPosChange);
                        upperError = posFinder.PositionError - maxLengthError;
                        if (validPosition && upperError < 0)
                        {
                            startingPosChange *= Constants.ErrorSizeIncrease;
                            // startingPosChange = startingPosChange * maxLengthError / (maxLengthError + upperError);
                        }
                        else
                        {
                            if (Math.Abs(startingPosChange * Constants.ConservativeErrorEstimation * 0.5) <
                                Constants.MinimumStepSize)
                                validPosition = false;
                            else startingPosChange *= Constants.ConservativeErrorEstimation * 0.5;
                        }
                    } while ((!validPosition || upperError > 0) && k++ < Constants.MaxItersInPositionError
                             &&
                             (Math.Abs(startingPosChange * Constants.ConservativeErrorEstimation * 0.5) >=
                              Constants.MinimumStepSize));
                    //var tempStep = startingPosChange;
                    //startingPosChange = (Constants.ErrorEstimateInertia * prevStep + startingPosChange) / (1 + Constants.ErrorEstimateInertia);
                    //prevStep = tempStep;
                }
                else
                {
                    // this next function puts the xNumerical and yNumerical values in the joints
                    NumericalPosition(timeStep, joints, links);
                    var delta = InputSpeed * timeStep;
                    // this next function puts the x and y values in the joints
                    validPosition = posFinder.DefineNewPositions(delta);
                }

                #endregion

                if (validPosition)
                {
                    if (Forward == (InputSpeed > 0))
                        lock (InputRange)
                        {
                            InputRange[1] = links[inputLinkIndex].Angle;
                        }
                    else
                        lock (InputRange)
                        {
                            InputRange[0] = links[inputLinkIndex].Angle;
                        }

                    #region Find Velocities for Current Position

                    // this next functions puts the vx and vy values as well as the vx_unit and vy_unit in the joints
                    if (!velSolver.Solve())
                    {
                        Status += "Instant Centers could not be found at" + currentTime + ".";
                        NumericalVelocity(timeStep, joints, links);
                    }

                    #endregion

                    #region Find Accelerations for Current Position

                    // this next functions puts the ax and ay values in the joints
                    if (!accelSolver.Solve())
                    {
                        Status += "Analytical acceleration could not be found at" + currentTime + ".";
                        NumericalAcceleration(timeStep, joints, links);
                    }

                    #endregion

                    currentTime += timeStep;
                    var jointParams = WriteJointStatesVariablesToMatrixAndToLast(joints);
                    var linkParams = WriteLinkStatesVariablesToMatrixAndToLast(links);
                    if (Forward == (InputSpeed > 0))
                    {
                        lock (JointParameters)
                            JointParameters.AddNearEnd(currentTime, jointParams);
                        lock (LinkParameters)
                            LinkParameters.AddNearEnd(currentTime, linkParams);
                    }
                    else
                    {
                        lock (JointParameters)
                            JointParameters.AddNearBegin(currentTime, jointParams);
                        lock (LinkParameters)
                            LinkParameters.AddNearBegin(currentTime, linkParams);
                    }
                }
            } while (validPosition && lessThanFullRotation());
        }

        /// <summary>
        /// Writes the joint states variables to matrix and to last.
        /// </summary>
        /// <param name="joints">The joints.</param>
        /// <returns>System.Double[].</returns>
        private double[,] WriteJointStatesVariablesToMatrixAndToLast(List<Joint> joints)
        {
            var jointParams = new double[NumAllJoints, maxJointParamLengths];
            for (var i = 0; i < NumAllJoints; i++)
            {
                var j = joints[i];
                jointParams[oIOSJ[i], 0] = j.x;
                jointParams[oIOSJ[i], 1] = j.y;
                jointParams[oIOSJ[i], 2] = j.vx;
                jointParams[oIOSJ[i], 3] = j.vy;
                jointParams[oIOSJ[i], 4] = j.ax;
                jointParams[oIOSJ[i], 5] = j.ay;
                if (j.TypeOfJoint != JointType.R)
                {
                    jointParams[oIOSJ[i], 6] = j.SlidePosition;
                    jointParams[oIOSJ[i], 7] = j.SlideVelocity;
                    jointParams[oIOSJ[i], 8] = j.SlideAcceleration;
                }
                j.xLast = j.x;
                j.yLast = j.y;
                j.vxLast = j.vx;
                j.vyLast = j.vy;
            }
            return jointParams;
        }

        /// <summary>
        /// Writes the link states variables to matrix and to last.
        /// </summary>
        /// <param name="links">The links.</param>
        /// <returns>System.Double[].</returns>
        private double[,] WriteLinkStatesVariablesToMatrixAndToLast(List<Link> links)
        {
            var linkParams = new double[NumLinks, 3];
            for (var i = 0; i < NumLinks; i++)
            {
                var l = links[i];
                linkParams[oIOSL[i], 0] = l.Angle;
                linkParams[oIOSL[i], 1] = l.Velocity;
                linkParams[oIOSL[i], 2] = l.Acceleration;
                l.AngleLast = l.Angle;
                l.VelocityLast = l.Velocity;
            }
            return linkParams;
        }
    }
}