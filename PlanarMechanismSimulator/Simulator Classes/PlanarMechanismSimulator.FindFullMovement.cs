using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        public void FindFullMovement()
        {
            if (double.IsNaN(DeltaAngle) && double.IsNaN(FixedTimeStep) && double.IsNaN(MaxSmoothingError))
                throw new Exception(
                    "Either the smoothing error angle delta or the time step must be specified.");
            var useErrorMethod = !double.IsNaN(MaxSmoothingError);
            #region Set up initial point parameters (x, x-dot, x-double-dot, etc.)

            double[,] initJointParams, initLinkParams;
            SetInitialVelocityAndAcceleration(DefineVelocitiesAnalytically(), DefineAccelerationsAnalytically(),
              out  initJointParams, out initLinkParams);

            var posFinder = new PositionFinder(joints, links, gearsData, inputJointIndex);
            JointParameters = new TimeSortedList();
            JointParameters.Add(0.0, initJointParams);
            LinkParameters = new TimeSortedList();
            LinkParameters.Add(0.0, initLinkParams);

            InputRange = new[] { inputLink.AngleInitial, inputLink.AngleInitial };

            #endregion
            if (useErrorMethod)
            {
#if DEBUGSERIAL
                SimulateWithinError(true, posFinder);
#elif SILVERLIGHT
                var posFinderBackwards = setUpNewPositionFinder();
                var forwardThread = new Thread(delegate()
                    {
                        SimulateWithinError( true, posFinder);
                        forwardDone.Set();
                    });
                var backwardThread = new Thread(delegate()
                    {
                        SimulateWithinError(false, posFinderBackwards);
                        backwardDone.Set();
                    });
                forwardThread.Start(); backwardThread.Start();
                if (forwardDone.WaitOne() && backwardDone.WaitOne())
                {
                    forwardDone = new AutoResetEvent(false);
                    backwardDone = new AutoResetEvent(false);
                }
#else
                var posFinderBackwards = setUpNewPositionFinder();
                Parallel.Invoke(
                    /*** Stepping Forward in Time ***/
                    () => SimulateWithinError(true, posFinder),
                    /*** Stepping Backward in Time ***/
                    () => SimulateWithinError(false, posFinderBackwards));
#endif
            }
            else
            {
#if DEBUGSERIAL
                SimulateWithFixedDelta(true, posFinder);
#elif SILVERLIGHT
                var posFinderBackwards = setUpNewPositionFinder();
                var forwardThread = new Thread(delegate()
                {
                    SimulateWithFixedDelta(true, posFinder);
                    forwardDone.Set();
                });
                var backwardThread = new Thread(delegate()
                {
                    SimulateWithFixedDelta(false, posFinderBackwards);
                    backwardDone.Set();
                });
                forwardThread.Start(); backwardThread.Start();
                if (forwardDone.WaitOne() && backwardDone.WaitOne())
                {
                    forwardDone = new AutoResetEvent(false);
                    backwardDone = new AutoResetEvent(false);
                }
#else
                var posFinderBackwards = setUpNewPositionFinder();
                Parallel.Invoke(
                    /*** Stepping Forward in Time ***/
                    () => SimulateWithFixedDelta(true, posFinder),
                    /*** Stepping Backward in Time ***/
                    () => SimulateWithFixedDelta(false, posFinderBackwards));
#endif
            }
        }

        private void SetInitialVelocityAndAcceleration(bool velocitySuccessful, bool accelSuccessful,
           out double[,] initJointParams, out double[,] initLinkParams)
        {
            initJointParams = WriteJointStatesVariablesToMatrixAndToLast();
            initLinkParams = WriteLinkStatesVariablesToMatrixAndToLast();
            if (velocitySuccessful && accelSuccessful)
                /* good news, both velocity and acceleration were found analytically! */
                return;
                var posFinder = new PositionFinder(joints, links, gearsData, inputJointIndex);
                var smallTimeStep = Constants.SmallPerturbationFraction * FixedTimeStep;
            if (velocitySuccessful)
            { /* velocity was successfully found, but not acceleration. */
                if (posFinder.DefineNewPositions(smallTimeStep * InputSpeed) && DefineVelocitiesAnalytically())
                {
                    /* forward difference on velocities to create accelerations. */
                    for (int i = 0; i < firstInputJointIndex; i++)
                    {
                        initJointParams[i, 4] = (joints[i].vx - joints[i].vxLast) / smallTimeStep;
                        initJointParams[i, 5] = (joints[i].vy - joints[i].vyLast) / smallTimeStep;
                    }
                    for (int i = 0; i < inputLinkIndex; i++)
                        initLinkParams[i, 2] = (links[i].Velocity - links[i].VelocityLast) / smallTimeStep;

                    /* since the position solving wrote values to joints[i].x and .y, we need to reset them, for further work. */
                    for (int i = 0; i < numJoints; i++)
                    {
                        joints[i].x = initJointParams[i, 0];
                        joints[i].y = initJointParams[i, 1];
                        joints[i].vx = initJointParams[i, 2];
                        joints[i].vy = initJointParams[i, 3];
                        joints[i].ax = initJointParams[i, 4];
                        joints[i].ay = initJointParams[i, 5];
                    }
                    for (int i = 0; i < numLinks; i++)
                    {
                        links[i].Angle = initLinkParams[i, 0];
                        links[i].Velocity = initLinkParams[i, 1];
                        links[i].Acceleration = initLinkParams[i, 2];
                    }
                    return;
                }
            }
            var ForwardJointParams = new double[numJoints, 2];
            var ForwardLinkParams = new double[numLinks];
            /*** Stepping Forward in Time ***/
            var forwardSuccess = posFinder.DefineNewPositions(smallTimeStep * InputSpeed);
            if (forwardSuccess)
            {
                NumericalVelocity(smallTimeStep);
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
            var backwardSuccess = posFinder.DefineNewPositions(-smallTimeStep * InputSpeed);
            if (backwardSuccess)
            {
                NumericalVelocity(-smallTimeStep);
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
                for (int i = 0; i < firstInputJointIndex; i++)
                {
                    /* first-order central finite difference */
                    initJointParams[i, 2] = (ForwardJointParams[i, 0] - BackwardJointParams[i, 0]) / (2 * smallTimeStep);
                    initJointParams[i, 3] = (ForwardJointParams[i, 1] - BackwardJointParams[i, 1]) / (2 * smallTimeStep);
                    /* second-order central finite difference */
                    initJointParams[i, 4] = (ForwardJointParams[i, 0] - 2 * initJointParams[i, 0] + BackwardJointParams[i, 0]) / (smallTimeStep * smallTimeStep);
                    initJointParams[i, 5] = (ForwardJointParams[i, 1] - 2 * initJointParams[i, 1] + BackwardJointParams[i, 1]) / (smallTimeStep * smallTimeStep);
                }
                for (int i = 0; i < inputLinkIndex; i++)
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
                for (int i = 0; i < firstInputJointIndex; i++)
                {
                    /* first-order forward finite difference */
                    initJointParams[i, 2] = (ForwardJointParams[i, 0] - initJointParams[i, 0]) / smallTimeStep;
                    initJointParams[i, 3] = (ForwardJointParams[i, 1] - initJointParams[i, 1]) / smallTimeStep;
                }
                for (int i = 0; i < inputLinkIndex; i++)
                    /* first-order forward finite difference */
                    initLinkParams[i, 1] = (ForwardLinkParams[i] - initLinkParams[i, 0]) / smallTimeStep;
            }
            else if (backwardSuccess)
            {
                /* backward difference puts values in init parameters. */
                for (int i = 0; i < firstInputJointIndex; i++)
                {
                    /* first-order backward finite difference */
                    initJointParams[i, 2] = (initJointParams[i, 0] - BackwardJointParams[i, 0]) / smallTimeStep;
                    initJointParams[i, 3] = (initJointParams[i, 1] - BackwardJointParams[i, 1]) / smallTimeStep;
                }
                for (int i = 0; i < inputLinkIndex; i++)
                    /* first-order backward finite difference */
                    initLinkParams[i, 1] = (initLinkParams[i, 0] - BackwardLinkParams[i]) / smallTimeStep;
            }
            /* since the position solving wrote values to joints[i].x and .y, we need to reset them, for further work. */
            for (int i = 0; i < numJoints; i++)
            {
                joints[i].x = initJointParams[i, 0];
                joints[i].y = initJointParams[i, 1];
                joints[i].vx = initJointParams[i, 2];
                joints[i].vy = initJointParams[i, 3];
                joints[i].ax = initJointParams[i, 4];
                joints[i].ay = initJointParams[i, 5];
            }
            for (int i = 0; i < numLinks; i++)
            {
                links[i].Angle = initLinkParams[i, 0];
                links[i].Velocity = initLinkParams[i, 1];
                links[i].Acceleration = initLinkParams[i, 2];
            }
        }



        private PositionFinder setUpNewPositionFinder()
        {
            var newJoints = joints.Select(j => j.copy()).ToList();
            var newLinks = links.Select(c => c.copy(joints, newJoints)).ToList();
            foreach (var j in newJoints)
            {
                j.Link1 = newLinks[links.IndexOf(j.Link1)];
                if (j.Link2 != null)
                    j.Link2 = newLinks[links.IndexOf(j.Link2)];
            }
            return new PositionFinder(newJoints, newLinks, gearsData, inputJointIndex);
        }

        private static AutoResetEvent forwardDone = new AutoResetEvent(false);
        private static AutoResetEvent backwardDone = new AutoResetEvent(false);


        private void SimulateWithFixedDelta(Boolean Forward, PositionFinder posFinder)
        {
            var timeStep = Forward ? FixedTimeStep : -FixedTimeStep;
            var currentTime = 0.0;
            Boolean validPosition;
            do
            {
                #region Find Next Positions
                // this next function puts the xNumerical and yNumerical values in the joints
                NumericalPosition(timeStep);
                var delta = InputSpeed * timeStep;
                // this next function puts the x and y values in the joints
                validPosition = posFinder.DefineNewPositions(delta);
                #endregion

                if (validPosition)
                {
                    if (Forward)
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
                    if (!DefineVelocitiesAnalytically())
                    {
                        Status += "Instant Centers could not be found at" + currentTime + ".";
                        NumericalVelocity(timeStep);
                    }

                    #endregion

                    #region Find Accelerations for Current Position

                    // this next functions puts the ax and ay values in the joints
                    if (!DefineAccelerationsAnalytically())
                    {
                        Status += "Analytical acceleration could not be found at" + currentTime + ".";
                        NumericalAcceleration(timeStep);
                    }

                    #endregion

                    currentTime += timeStep;
                    var linkParams = WriteLinkStatesVariablesToMatrixAndToLast();
                    var jointParams = WriteJointStatesVariablesToMatrixAndToLast();
                    if (Forward)
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

        private void SimulateWithinError(Boolean Forward, PositionFinder posFinder)
        {
            var startingPosChange = Forward ? Constants.DefaultStepSize : -Constants.DefaultStepSize;
            var prevStep = startingPosChange;
            if (inputJoint.jointType == JointTypes.P) startingPosChange *= AverageLength;
            var maxLengthError = MaxSmoothingError * AverageLength;
            var currentTime = 0.0;
            double timeStep;
            Boolean validPosition;
            do
            {
                #region Find Next Positions
                var upperError = double.PositiveInfinity;
                var k = 0;
                do
                {
                    timeStep = startingPosChange / InputSpeed;
                    NumericalPosition(timeStep);
                    validPosition = posFinder.DefineNewPositions(startingPosChange);
                    upperError = posFinder.PositionError - maxLengthError;
                    if (validPosition && upperError < 0)
                    {
                        startingPosChange *= Constants.ErrorSizeIncrease;
                        // startingPosChange = startingPosChange * maxLengthError / (maxLengthError + upperError);
                    }
                    else startingPosChange *= Constants.ConservativeErrorEstimation * 0.5;

                } while (upperError > 0 && k++ < Constants.MaxItersInPositionError);
                //var tempStep = startingPosChange;
                //startingPosChange = (Constants.ErrorEstimateInertia * prevStep + startingPosChange) / (1 + Constants.ErrorEstimateInertia);
                //prevStep = tempStep;
                #endregion

                if (validPosition)
                {
                    if (Forward)
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
                    if (!DefineVelocitiesAnalytically())
                    {
                        Status += "Instant Centers could not be found at" + currentTime + ".";
                        NumericalVelocity(timeStep);
                    }

                    #endregion

                    #region Find Accelerations for Current Position

                    // this next functions puts the ax and ay values in the joints
                    if (!DefineAccelerationsAnalytically())
                    {
                        Status += "Analytical acceleration could not be found at" + currentTime + ".";
                        NumericalAcceleration(timeStep);
                    }

                    #endregion

                    currentTime += timeStep;
                    var linkParams = WriteLinkStatesVariablesToMatrixAndToLast();
                    var jointParams = WriteJointStatesVariablesToMatrixAndToLast();
                    if (Forward)
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


        private double[,] WriteJointStatesVariablesToMatrixAndToLast()
        {
            var jointParams = new double[numJoints, 6];
            for (int i = 0; i < numJoints; i++)
            {
                var j = joints[i];
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

        private double[,] WriteLinkStatesVariablesToMatrixAndToLast()
        {
            var linkParams = new double[numLinks, 3];
            for (int i = 0; i < numLinks; i++)
            {
                var l = links[i];
                linkParams[i, 0] = l.Angle;
                linkParams[i, 0] = l.Velocity;
                linkParams[i, 0] = l.Acceleration;
                l.AngleLast = l.Angle;
                l.VelocityLast = l.Velocity;
            }
            return linkParams;
        }

    }
}
