using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using OptimizationToolbox;
using PlanarMechanismSimulator.PositionSolving;
using PlanarMechanismSimulator.VelocityAndAcceleration;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        private static AutoResetEvent forwardDone = new AutoResetEvent(false);
        private static AutoResetEvent backwardDone = new AutoResetEvent(false);

        public void FindFullMovement()
        {
            if (double.IsNaN(DeltaAngle) && double.IsNaN(FixedTimeStep) && double.IsNaN(MaxSmoothingError))
                throw new Exception(
                    "Either the smoothing error angle delta or the time step must be specified.");
            bool useErrorMethod = (!double.IsNaN(MaxSmoothingError) && MaxSmoothingError > 0);

            #region Set up initial point parameters (x, x-dot, x-double-dot, etc.)

            double[,] initJointParams, initLinkParams;
            SetInitialVelocityAndAcceleration(AllJoints, AllLinks, out initJointParams, out initLinkParams);

            JointParameters = new TimeSortedList { { 0.0, initJointParams } };
            LinkParameters = new TimeSortedList { { 0.0, initLinkParams } };

            InputRange = new[] { inputLink.AngleInitial, inputLink.AngleInitial };

            #endregion
#if DEBUGSERIAL
            if (useErrorMethod)
            {
                if (MaxSmoothingError == 0) throw new Exception("The MaxSmoothingError must not be zero.");

                SimulateWithinError(AllJoints, AllLinks, true);
            }
            else
            {
                SimulateWithFixedDelta(AllJoints, AllLinks, true);
            }
#else
            var newJoints = AllJoints.Select(j => j.copy()).ToList();
            var newLinks = AllLinks.Select(c => c.copy(AllJoints, newJoints)).ToList();
            foreach (var j in newJoints)
            {
                j.Link1 = newLinks[AllLinks.IndexOf(j.Link1)];
                if (j.Link2 != null)
                    j.Link2 = newLinks[AllLinks.IndexOf(j.Link2)];
            }
            if (useErrorMethod)
            {

#if SILVERLIGHT
                var forwardThread = new Thread(() =>
                    {
                        SimulateWithinError(AllJoints, AllLinks, true);
                        forwardDone.Set();
                    });
                var backwardThread = new Thread(() =>
                    {
                        SimulateWithinError(newJoints, newLinks, false);
                        backwardDone.Set();
                    });
                forwardThread.Start();
                backwardThread.Start();
                if (forwardDone.WaitOne() && backwardDone.WaitOne())
                {
                    forwardDone = new AutoResetEvent(false);
                    backwardDone = new AutoResetEvent(false);
                }
#else
                Parallel.Invoke(
                    /*** Stepping Forward in Time ***/
                    () => SimulateWithinError(AllJoints, AllLinks, true),
                    /*** Stepping Backward in Time ***/
                    () => SimulateWithinError(newJoints, newLinks, false));
#endif
            }
            else
            {
#if SILVERLIGHT
                var forwardThread = new Thread(() =>
                    {
                        SimulateWithFixedDelta(AllJoints, AllLinks, true);
                        forwardDone.Set();
                    });
                var backwardThread = new Thread(() =>
                    {
                        SimulateWithFixedDelta(newJoints, newLinks, false);
                        backwardDone.Set();
                    });
                forwardThread.Start();
                backwardThread.Start();
                if (forwardDone.WaitOne() && backwardDone.WaitOne())
                {
                    forwardDone = new AutoResetEvent(false);
                    backwardDone = new AutoResetEvent(false);
                }
#else
                Parallel.Invoke(
                    /*** Stepping Forward in Time ***/
                     () => SimulateWithFixedDelta(AllJoints, AllLinks, true),
                    /*** Stepping Backward in Time ***/
                     () => SimulateWithFixedDelta(newJoints, newLinks, false));
#endif
            }
#endif
            DefineMovementBooleans();
        }


        public void DefineMovementBooleans()
        {
            if (lessThanFullRotation())
            {
                AdditionalGearCycling = false;
                CompleteCycle = false;
                Time_Span = JointParameters.Times.Last() - JointParameters.Times[0];
                return;
            }
            Time_Span = CyclePeriodTime = 2 * Math.PI / InputSpeed;
            var forwardTime = CyclePeriodTime + JointParameters.Times[0];
            var initState = JointParameters.Parameters[0];
            var indexTop = JointParameters.LastIndex - 1;
            while (JointParameters.Times[indexTop] > forwardTime)
            {
                indexTop--;
            }
            var maxRMSError = 0.0;
            var tau = forwardTime - JointParameters.Times[indexTop];
            var deltaTime = JointParameters.Times[indexTop + 1] - JointParameters.Times[indexTop];
            var previousState = JointParameters.Parameters[indexTop];
            var nextState = JointParameters.Parameters[indexTop + 1];
            for (int i = 0; i < numJoints; i++)
            {
                var xCycle = FindPositionatTime(tau, deltaTime, previousState[i, 0], nextState[i, 0],
                    previousState[i, 2], nextState[i, 2], previousState[i, 4],
                    nextState[i, 4]);
                var error = (xCycle - initState[i, 0]) * (xCycle - initState[i, 0]);
                if (maxRMSError < error) maxRMSError = error;
                var yCycle = FindPositionatTime(tau, deltaTime, previousState[i, 1], nextState[i, 1],
                    previousState[i, 3], nextState[i, 3], previousState[i, 5],
                    nextState[i, 5]);
                error = (yCycle - initState[i, 1]) * (yCycle - initState[i, 1]);
                if (maxRMSError < error) maxRMSError = error;
            }
            CompleteCycle = (maxRMSError < Constants.ErrorInDeterminingCompleteCycle);
            AdditionalGearCycling = AllJoints.Any(j => j.jointType == JointTypes.G);
            if (CompleteCycle)
            {
                while (indexTop < JointParameters.LastIndex)
                {
                    JointParameters.RemoveAt(indexTop + 1);
                    LinkParameters.RemoveAt(indexTop + 1);
                }
                while (JointParameters.Times[0] < 0.0)
                {
                    var time = JointParameters.Times[0];
                    var parameters = JointParameters.Parameters[0];
                    JointParameters.RemoveAt(0);
                    JointParameters.AddNearEnd(time + CyclePeriodTime, parameters);

                    parameters = LinkParameters.Parameters[0];
                    LinkParameters.RemoveAt(0);
                    LinkParameters.AddNearEnd(time + CyclePeriodTime, parameters);
                }
            }
        }


        private void SetInitialVelocityAndAcceleration(List<joint> joints, List<link> links, out double[,] initJointParams, out double[,] initLinkParams)
        {
            initJointParams = WriteJointStatesVariablesToMatrixAndToLast(joints);
            initLinkParams = WriteLinkStatesVariablesToMatrixAndToLast(links);

            var posFinder = new PositionFinder(joints, links, gearsData, inputJointIndex);
            var velSolver = new VelocitySolver(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, InputSpeed, gearsData);
            var accelSolver = new AccelerationSolver(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, InputSpeed, gearsData);

            double smallTimeStep = Constants.SmallPerturbationFraction * FixedTimeStep;
            if (velSolver.Solve())
            {
                for (int i = 0; i <= inputJointIndex; i++)
                {
                    initJointParams[i, 2] = joints[i].vx;
                    initJointParams[i, 3] = joints[i].vy;
                }
                for (int i = 0; i <= inputLinkIndex; i++)
                    initLinkParams[i, 1] = links[i].Velocity;
                if (accelSolver.Solve())
                {
                    for (int i = 0; i <= inputJointIndex; i++)
                    {
                        initJointParams[i, 4] = joints[i].ax;
                        initJointParams[i, 5] = joints[i].ay;
                    }
                    for (int i = 0; i <= inputLinkIndex; i++)
                        initLinkParams[i, 2] = links[i].Acceleration;
                }
                else
                {
                    /* velocity was successfully found, but not acceleration. */
                    if (posFinder.DefineNewPositions(smallTimeStep * InputSpeed) &&
                        velSolver.Solve())
                    {
                        /* forward difference on velocities to create accelerations. */
                        for (int i = 0; i <= inputJointIndex; i++)
                        {
                            initJointParams[i, 4] = (joints[i].vx - joints[i].vxLast) / smallTimeStep;
                            initJointParams[i, 5] = (joints[i].vy - joints[i].vyLast) / smallTimeStep;
                        }
                        for (int i = 0; i <= inputLinkIndex; i++)
                            initLinkParams[i, 2] = (links[i].Velocity - links[i].VelocityLast) / smallTimeStep;

                        /* since the position solving wrote values to joints[i].x and .y, we need to reset them, for further work. */
                        for (int i = 0; i <= inputJointIndex; i++)
                        {
                            joints[i].x = initJointParams[i, 0];
                            joints[i].y = initJointParams[i, 1];
                            joints[i].vx = initJointParams[i, 2];
                            joints[i].vy = initJointParams[i, 3];
                            joints[i].ax = initJointParams[i, 4];
                            joints[i].ay = initJointParams[i, 5];
                        }
                        for (int i = 0; i <= inputLinkIndex; i++)
                        {
                            links[i].Angle = initLinkParams[i, 0];
                            links[i].Velocity = initLinkParams[i, 1];
                            links[i].Acceleration = initLinkParams[i, 2];
                        }
                    }
                }
                return;
            }
            var ForwardJointParams = new double[numJoints, 2];
            var ForwardLinkParams = new double[numLinks];
            /*** Stepping Forward in Time ***/
            bool forwardSuccess = posFinder.DefineNewPositions(smallTimeStep * InputSpeed);
            if (forwardSuccess)
            {
                NumericalVelocity(smallTimeStep, joints, links);
                for (int i = 0; i < numJoints; i++)
                {
                    ForwardJointParams[i, 0] = joints[i].x;
                    ForwardJointParams[i, 0] = joints[i].y;
                }
                for (int i = 0; i < numLinks; i++)
                    ForwardLinkParams[i] = links[i].Angle;
            }
            /*** Stepping Backward in Time ***/
            var BackwardJointParams = new double[numJoints, 2];
            var BackwardLinkParams = new double[numLinks];
            bool backwardSuccess = posFinder.DefineNewPositions(-smallTimeStep * InputSpeed);
            if (backwardSuccess)
            {
                NumericalVelocity(-smallTimeStep, joints, links);
                for (int i = 0; i < numJoints; i++)
                {
                    BackwardJointParams[i, 0] = joints[i].x;
                    BackwardJointParams[i, 0] = joints[i].y;
                }
                for (int i = 0; i < numLinks; i++)
                    BackwardLinkParams[i] = links[i].Angle;
            }
            if (forwardSuccess && backwardSuccess)
            {
                /* central difference puts values in init parameters. */
                for (int i = 0; i <= inputJointIndex; i++)
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
                for (int i = 0; i <= inputLinkIndex; i++)
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
                for (int i = 0; i <= inputJointIndex; i++)
                {
                    /* first-order forward finite difference */
                    initJointParams[i, 2] = (ForwardJointParams[i, 0] - initJointParams[i, 0]) / smallTimeStep;
                    initJointParams[i, 3] = (ForwardJointParams[i, 1] - initJointParams[i, 1]) / smallTimeStep;
                }
                for (int i = 0; i <= inputLinkIndex; i++)
                    /* first-order forward finite difference */
                    initLinkParams[i, 1] = (ForwardLinkParams[i] - initLinkParams[i, 0]) / smallTimeStep;
            }
            else if (backwardSuccess)
            {
                /* backward difference puts values in init parameters. */
                for (int i = 0; i <= inputJointIndex; i++)
                {
                    /* first-order backward finite difference */
                    initJointParams[i, 2] = (initJointParams[i, 0] - BackwardJointParams[i, 0]) / smallTimeStep;
                    initJointParams[i, 3] = (initJointParams[i, 1] - BackwardJointParams[i, 1]) / smallTimeStep;
                }
                for (int i = 0; i <= inputLinkIndex; i++)
                    /* first-order backward finite difference */
                    initLinkParams[i, 1] = (initLinkParams[i, 0] - BackwardLinkParams[i]) / smallTimeStep;
            }
            /* since the position solving wrote values to joints[i].x and .y, we need to reset them, for further work. */
            for (int i = 0; i <= inputJointIndex; i++)
            {
                joints[i].x = initJointParams[i, 0];
                joints[i].y = initJointParams[i, 1];
                joints[i].vx = initJointParams[i, 2];
                joints[i].vy = initJointParams[i, 3];
                joints[i].ax = initJointParams[i, 4];
                joints[i].ay = initJointParams[i, 5];
            }
            for (int i = 0; i <= inputLinkIndex; i++)
            {
                links[i].Angle = initLinkParams[i, 0];
                links[i].Velocity = initLinkParams[i, 1];
                links[i].Acceleration = initLinkParams[i, 2];
            }
        }


        private void SimulateWithFixedDelta(List<joint> joints, List<link> links, Boolean Forward)
        {
            double timeStep = Forward ? FixedTimeStep : -FixedTimeStep;
            double currentTime = 0.0;
            Boolean validPosition;
            var posFinder = new PositionFinder(joints, links, gearsData, inputJointIndex);
            var velSolver = new VelocitySolver(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, InputSpeed, gearsData);
            var accelSolver = new AccelerationSolver(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, InputSpeed, gearsData);
            do
            {
                #region Find Next Positions

                // this next function puts the xNumerical and yNumerical values in the joints
                NumericalPosition(timeStep, joints, links);
                double delta = InputSpeed * timeStep;
                // this next function puts the x and y values in the joints
                validPosition = posFinder.DefineNewPositions(delta);

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
                    double[,] jointParams = WriteJointStatesVariablesToMatrixAndToLast(joints);
                    double[,] linkParams = WriteLinkStatesVariablesToMatrixAndToLast(links);
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

        private void SimulateWithinError(List<joint> joints, List<link> links, Boolean Forward)
        {
            double startingPosChange = Forward ? Constants.DefaultStepSize : -Constants.DefaultStepSize;
            if (inputJoint.jointType == JointTypes.P) startingPosChange *= AverageLength;
            double maxLengthError = MaxSmoothingError * AverageLength;
            double currentTime = 0.0;
            Boolean validPosition;
            var posFinder = new PositionFinder(joints, links, gearsData, inputJointIndex);
            var velSolver = new VelocitySolver(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, InputSpeed, gearsData);
            var accelSolver = new AccelerationSolver(joints, links, firstInputJointIndex, inputJointIndex, inputLinkIndex, InputSpeed, gearsData);
            do
            {
                #region Find Next Positions
                int k = 0;
                double upperError, timeStep;
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
                        startingPosChange *= Constants.ConservativeErrorEstimation * 0.5;
                        startingPosChange = Math.Sign(startingPosChange) * Math.Max(Math.Abs(startingPosChange), Constants.MinimumStepSize);
                    }
                } while ((!validPosition || upperError > 0) && k++ < Constants.MaxItersInPositionError);
                //var tempStep = startingPosChange;
                //startingPosChange = (Constants.ErrorEstimateInertia * prevStep + startingPosChange) / (1 + Constants.ErrorEstimateInertia);
                //prevStep = tempStep;

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
                    double[,] jointParams = WriteJointStatesVariablesToMatrixAndToLast(joints);
                    double[,] linkParams = WriteLinkStatesVariablesToMatrixAndToLast(links);
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


        private double[,] WriteJointStatesVariablesToMatrixAndToLast(List<joint> joints)
        {
            var jointParams = new double[numJoints, 6];
            for (int i = 0; i < numJoints; i++)
            {
                joint j = joints[i];
                jointParams[i, 0] = j.x;
                jointParams[i, 1] = j.y;
                jointParams[i, 2] = j.vx;
                jointParams[i, 3] = j.vy;
                jointParams[i, 4] = j.ax;
                jointParams[i, 5] = j.ay;
                j.xLast = j.x;
                j.yLast = j.y;
                j.vxLast = j.vx;
                j.vyLast = j.vy;
            }
            return jointParams;
        }

        private double[,] WriteLinkStatesVariablesToMatrixAndToLast(List<link> links)
        {
            var linkParams = new double[numLinks, 3];
            for (int i = 0; i < numLinks; i++)
            {
                link l = links[i];
                linkParams[i, 0] = l.Angle;
                linkParams[i, 1] = l.Velocity;
                linkParams[i, 2] = l.Acceleration;
                l.AngleLast = l.Angle;
                l.VelocityLast = l.Velocity;
            }
            return linkParams;
        }
    }
}