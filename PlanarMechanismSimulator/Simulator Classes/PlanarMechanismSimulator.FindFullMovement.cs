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

            JointParameters = new TimeSortedList();
            LinkParameters = new TimeSortedList();

            var initPivotParams = new double[numJoints, 6];
            for (int i = 0; i < numJoints; i++)
            {
                initPivotParams[i, 0] = joints[i].xInitial;
                initPivotParams[i, 1] = joints[i].yInitial;
            }
            var initLinkParams = new double[numLinks, 3];
            for (int i = 0; i < numLinks; i++)
                initLinkParams[i, 0] = links[i].AngleInitial;

            InputRange = new[] { inputLink.AngleInitial, inputLink.AngleInitial };
            JointParameters.Add(0.0, initPivotParams);
            LinkParameters.Add(0.0, initLinkParams);
            posFinder = new PositionFinder(joints, links, gearsData, inputJointIndex);
            var posFinderBackwards = setUpNewPositionFinder();
            /* attempt to find velocities and accelerations at initial point analytically
             * there is no point in trying numerically as this is the first point and the numerical methods
             * perform finite difference of current and last time steps. */

            // at this point. the values are in x_initial and y_initial, but the velocity analysis will look at only
            // x and y.
            if (!(DefineVelocities() && findAccelerationAnalytically(0.0, true)))
            {
                var ForwardJointParams = (double[,])initPivotParams.Clone();
                var ForwardLinkParams = (double[,])initLinkParams.Clone();
                bool forwardSuccess = false;
                var BackwardJointParams = (double[,])initPivotParams.Clone();
                var BackwardLinkParams = (double[,])initLinkParams.Clone();
                bool backwardSuccess = false;
                var smallTimeStep = Constants.SmallPerturbationFraction * FixedTimeStep;
                forwardSuccess = posFinder.DefineNewPositions(Constants.SmallPerturbationFraction * FixedTimeStep * InputSpeed,
                ForwardJointParams, ForwardLinkParams, initPivotParams, initLinkParams);
                if (forwardSuccess)
                    NumericalVelocity(smallTimeStep, ForwardJointParams, ForwardLinkParams, initPivotParams, initLinkParams);
                /*** Stepping Backward in Time ***/
                backwardSuccess = posFinder.DefineNewPositions(-Constants.SmallPerturbationFraction * FixedTimeStep * InputSpeed,
                BackwardJointParams, BackwardLinkParams, initPivotParams, initLinkParams);
                if (backwardSuccess)
                    NumericalVelocity(-smallTimeStep, BackwardJointParams, BackwardLinkParams, initPivotParams, initLinkParams);

                if (forwardSuccess && backwardSuccess)
                {
                    /* central difference puts values in init parameters. */
                    for (int i = 0; i < firstInputJointIndex; i++)
                    {
                        initPivotParams[i, 2] = (ForwardJointParams[i, 2] + BackwardJointParams[i, 2]) / 2;
                        initPivotParams[i, 3] = (ForwardJointParams[i, 3] + BackwardJointParams[i, 3]) / 2;
                        initPivotParams[i, 4] = (ForwardJointParams[i, 2] - BackwardJointParams[i, 2]) / (2 * smallTimeStep);
                        initPivotParams[i, 5] = (ForwardJointParams[i, 3] - BackwardJointParams[i, 3]) / (2 * smallTimeStep);
                    }
                    for (int i = 0; i < inputLinkIndex; i++)
                    {
                        initLinkParams[i, 1] = (ForwardLinkParams[i, 1] + BackwardLinkParams[i, 1]) / 2;
                        initLinkParams[i, 2] = (ForwardLinkParams[i, 1] - BackwardLinkParams[i, 1]) / (2 * smallTimeStep);
                    }
                }
                else if (forwardSuccess)
                {
                    for (int i = 0; i < firstInputJointIndex; i++)
                    {
                        initPivotParams[i, 2] = ForwardJointParams[i, 2];
                        initPivotParams[i, 3] = ForwardJointParams[i, 3];
                    }
                    for (int i = 0; i < inputLinkIndex; i++)
                        initLinkParams[i, 1] = ForwardLinkParams[i, 1];
                }
                else if (backwardSuccess)
                {
                    for (int i = 0; i < firstInputJointIndex; i++)
                    {
                        initPivotParams[i, 2] = BackwardJointParams[i, 2];
                        initPivotParams[i, 3] = BackwardJointParams[i, 3];
                    }
                    for (int i = 0; i < inputLinkIndex; i++)
                        initLinkParams[i, 1] = BackwardLinkParams[i, 1];
                }
            }

            #endregion
            if (useErrorMethod)
            {
#if DEBUGSERIAL
            SimulateWithinError(initPivotParams, initLinkParams, true, posFinder);
#elif SILVERLIGHT
            var forwardThread = new Thread(delegate()
                {
                    SimulateWithinError( initPivotParams, initLinkParams, true, posFinder);
                    forwardDone.Set();
                });
            var backwardThread = new Thread(delegate()
                {
                    SimulateWithinError(initPivotParams, initLinkParams, false, posFinderBackwards);
                    backwardDone.Set();
                });
            forwardThread.Start(); backwardThread.Start();
            if (forwardDone.WaitOne() && backwardDone.WaitOne())
            {
                forwardDone = new AutoResetEvent(false);
                backwardDone = new AutoResetEvent(false);
            }
#else
                Parallel.Invoke(
                    /*** Stepping Forward in Time ***/
                    () => SimulateWithinError(initPivotParams, initLinkParams, true, posFinder),
                    /*** Stepping Backward in Time ***/
                    () => SimulateWithinError(initPivotParams, initLinkParams, false, posFinderBackwards));
#endif
            }
            else
            {
#if DEBUGSERIAL
            SimulateWithFixedDelta( initPivotParams, initLinkParams, true, posFinder);
#elif SILVERLIGHT
            var forwardThread = new Thread(delegate()
                {
                    SimulateWithFixedDelta( initPivotParams, initLinkParams, true, posFinder);
                    forwardDone.Set();
                });
            var backwardThread = new Thread(delegate()
                {
                    SimulateWithFixedDelta(initPivotParams, initLinkParams, false, posFinderBackwards);
                    backwardDone.Set();
                });
            forwardThread.Start(); backwardThread.Start();
            if (forwardDone.WaitOne() && backwardDone.WaitOne())
            {
                forwardDone = new AutoResetEvent(false);
                backwardDone = new AutoResetEvent(false);
            }
#else
                Parallel.Invoke(
                    /*** Stepping Forward in Time ***/
                    () => SimulateWithFixedDelta(initPivotParams, initLinkParams, true, posFinder),
                    /*** Stepping Backward in Time ***/
                    () => SimulateWithFixedDelta(initPivotParams, initLinkParams, false, posFinderBackwards));
#endif
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


        private void SimulateWithFixedDelta(double[,] lastPivotParams, double[,] lastLinkParams, Boolean Forward, PositionFinder posFinder)
        {
            var timeStep = Forward ? FixedTimeStep : -FixedTimeStep;
            var currentTime = 0.0;
            Boolean validPosition;
            do
            {
                var currentLinkParams = new double[numLinks, 3];
                var currentPivotParams = new double[numJoints, 6];
                #region Find Next Positions
                NumericalPosition(timeStep, currentPivotParams, currentLinkParams,
                                  lastPivotParams, lastLinkParams);
                var delta = InputSpeed * timeStep;
                validPosition = posFinder.DefineNewPositions(delta, currentPivotParams, currentLinkParams,
                                                   lastPivotParams, lastLinkParams);
                #endregion

                if (validPosition)
                {
                    if (Forward)
                        lock (InputRange)
                        {
                            InputRange[1] = currentLinkParams[inputLinkIndex, 0];
                        }
                    else
                        lock (InputRange)
                        {
                            InputRange[0] = currentLinkParams[inputLinkIndex, 0];
                        }

                    #region Find Velocities for Current Position

                    if (!DefineVelocities())
                    {
                        Status += "Instant Centers could not be found at" + currentTime + ".";
                        NumericalVelocity(timeStep, currentPivotParams, currentLinkParams,
                                          lastPivotParams, lastLinkParams);
                    }

                    #endregion

                    #region Find Accelerations for Current Position

                    if (!findAccelerationAnalytically(currentTime, true))
                    {
                        Status += "Analytical acceleration could not be found at" + currentTime + ".";
                        NumericalAcceleration(timeStep, currentPivotParams, currentLinkParams,
                                              lastPivotParams, lastLinkParams);
                    }

                    #endregion

                    currentTime += timeStep;
                    if (Forward)
                    {
                        lock (JointParameters)
                            JointParameters.AddNearEnd(currentTime, currentPivotParams);
                        lock (LinkParameters)
                            LinkParameters.AddNearEnd(currentTime, currentLinkParams);
                    }
                    else
                    {
                        lock (JointParameters)
                            JointParameters.AddNearBegin(currentTime, currentPivotParams);
                        lock (LinkParameters)
                            LinkParameters.AddNearBegin(currentTime, currentLinkParams);
                    }
                    lastPivotParams = currentPivotParams;
                    lastLinkParams = currentLinkParams;
                }
            } while (validPosition && lessThanFullRotation());
        }


        private void SimulateWithinError(double[,] lastPivotParams, double[,] lastLinkParams, Boolean Forward, PositionFinder posFinder)
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
                var currentLinkParams = new double[numLinks, 3];
                var currentPivotParams = new double[numJoints, 6];

                #region Find Next Positions
                var upperError = double.PositiveInfinity;
                var k = 0;
                do
                {
                    timeStep = startingPosChange / InputSpeed;
                    NumericalPosition(timeStep, currentPivotParams, currentLinkParams,
                                      lastPivotParams, lastLinkParams);
                    validPosition = posFinder.DefineNewPositions(startingPosChange, currentPivotParams, currentLinkParams,
                                                       lastPivotParams, lastLinkParams);
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
                            InputRange[1] = currentLinkParams[inputLinkIndex, 0];
                        }
                    else
                        lock (InputRange)
                        {
                            InputRange[0] = currentLinkParams[inputLinkIndex, 0];
                        }

                    #region Find Velocities for Current Position

                    if (!DefineVelocities())
                    {
                        Status += "Instant Centers could not be found at" + currentTime + ".";
                        NumericalVelocity(timeStep, currentPivotParams, currentLinkParams,
                                          lastPivotParams, lastLinkParams);
                    }

                    #endregion

                    #region Find Accelerations for Current Position

                    if (!findAccelerationAnalytically(currentTime, true))
                    {
                        Status += "Analytical acceleration could not be found at" + currentTime + ".";
                        NumericalAcceleration(timeStep, currentPivotParams, currentLinkParams,
                                              lastPivotParams, lastLinkParams);
                    }

                    #endregion

                    currentTime += timeStep;
                    if (Forward)
                    {
                        lock (JointParameters)
                            JointParameters.AddNearEnd(currentTime, currentPivotParams);
                        lock (LinkParameters)
                            LinkParameters.AddNearEnd(currentTime, currentLinkParams);
                    }
                    else
                    {
                        lock (JointParameters)
                            JointParameters.AddNearBegin(currentTime, currentPivotParams);
                        lock (LinkParameters)
                            LinkParameters.AddNearBegin(currentTime, currentLinkParams);
                    }
                    lastPivotParams = currentPivotParams;
                    lastLinkParams = currentLinkParams;
                }
            } while (validPosition && lessThanFullRotation());
        }


    }
}
