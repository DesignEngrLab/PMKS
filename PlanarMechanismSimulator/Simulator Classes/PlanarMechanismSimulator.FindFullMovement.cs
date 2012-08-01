using System;
using System.Threading;
using System.Threading.Tasks;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        public void FindFullMovement()
        {
            if ((double.IsNaN(DeltaAngle)) && (double.IsNaN(FixedTimeStep)))
                throw new Exception(
                    "Either the angle delta or the time step must be specified.");

            #region Set up initial point parameters (x, x-dot, x-double-dot, etc.)
            SetUpDyadicVelocityObjects();
            JointParameters = new TimeSortedList();
            LinkParameters = new TimeSortedList();

            var initPivotParams = new double[numJoints, 6];
            for (int i = 0; i < numJoints; i++)
            {
                initPivotParams[i, 0] = joints[i].initX;
                initPivotParams[i, 1] = joints[i].initY;
            }
            var initLinkParams = new double[numLinks, 3];
            for (int i = 0; i < numLinks; i++)
                initLinkParams[i, 0] = links[i].Angle;
            InitializeGroundAndInputSpeedAndAcceleration(initPivotParams, initLinkParams);
            InputRange = new[] { inputLink.Angle, inputLink.Angle };
            JointParameters.Add(0.0, initPivotParams);
            LinkParameters.Add(0.0, initLinkParams);
            /* attempt to find velocities and accelerations at initial point analytically
             * there is no point in trying numerically as this is the first point and the numerical methods
             * perform finite difference of current and last time steps. */
            if (!(findVelocitiesThroughICMethod(0.0, true) && findAccelerationAnalytically(0.0, true)))
            {
                var ForwardJointParams = (double[,])initPivotParams.Clone();
                var ForwardLinkParams = (double[,])initLinkParams.Clone();
                bool forwardSuccess = false;
                var BackwardJointParams = (double[,])initPivotParams.Clone();
                var BackwardLinkParams = (double[,])initLinkParams.Clone();
                bool backwardSuccess = false;
                var smallTimeStep = 0.003 * InputSpeed * FixedTimeStep;
#if SILVERLIGHT
                    forwardSuccess =
                          microPerturbForFiniteDifferenceOfVelocityAndAcceleration(smallTimeStep,
                          ForwardJointParams,ForwardLinkParams,initPivotParams, initLinkParams);
                    /*** Stepping Backward in Time ***/
                   backwardSuccess = microPerturbForFiniteDifferenceOfVelocityAndAcceleration(-smallTimeStep,
                      BackwardJointParams,BackwardLinkParams,initPivotParams,initLinkParams);

#else
                Parallel.Invoke(
                    /*** Stepping Forward in Time ***/
                    () => forwardSuccess =
                          microPerturbForFiniteDifferenceOfVelocityAndAcceleration(smallTimeStep,
                          ForwardJointParams, ForwardLinkParams, initPivotParams, initLinkParams),
                    /*** Stepping Backward in Time ***/
                    () => backwardSuccess = microPerturbForFiniteDifferenceOfVelocityAndAcceleration(-smallTimeStep,
                      BackwardJointParams, BackwardLinkParams, initPivotParams, initLinkParams));
#endif
                if (forwardSuccess && backwardSuccess)
                { /* central difference puts values in init parameters. */
                    for (int i = 0; i < numJoints; i++)
                        for (int j = 2; j < 6; j++)
                            initPivotParams[i, j] = (ForwardJointParams[i, j] + BackwardJointParams[i, j]) / 2;
                    for (int i = 0; i < numLinks; i++)
                        for (int j = 1; j < 3; j++)
                            initLinkParams[i, j] = (ForwardLinkParams[i, j] + BackwardLinkParams[i, j]) / 2;
                }
                else if (forwardSuccess)
                { /* forward difference puts values in init parameters. */
                    for (int i = 0; i < numJoints; i++)
                        for (int j = 2; j < 6; j++)
                            initPivotParams[i, j] = ForwardJointParams[i, j];
                    for (int i = 0; i < numLinks; i++)
                        for (int j = 1; j < 3; j++)
                            initLinkParams[i, j] = ForwardLinkParams[i, j];
                }
                else if (backwardSuccess)
                { /* Backward difference puts values in init parameters. */
                    for (int i = 0; i < numJoints; i++)
                        for (int j = 2; j < 6; j++)
                            initPivotParams[i, j] = BackwardJointParams[i, j];
                    for (int i = 0; i < numLinks; i++)
                        for (int j = 1; j < 3; j++)
                            initLinkParams[i, j] = BackwardLinkParams[i, j];
                }
            }
            #endregion
#if SILVERLIGHT

            loopWithFixedDelta(FixedTimeStep, initPivotParams, initLinkParams, true); 
            //var forwardThread = new Thread(delegate()
            //    {
            //        loopWithFixedDelta(FixedTimeStep, initPivotParams, initLinkParams, true);
            //        forwardDone.Set();
            //    });
            //var backwardThread = new Thread(delegate()
            //    {
            //        loopWithFixedDelta(-FixedTimeStep, initPivotParams, initLinkParams, false);
            //        backwardDone.Set();
            //    });
            //forwardThread.Start(); backwardThread.Start();  
            //if (forwardDone.WaitOne() && backwardDone.WaitOne())
            //{
            //    forwardDone = new AutoResetEvent(false);
            //    backwardDone = new AutoResetEvent(false);
            //}

#else
            Parallel.Invoke(
                /*** Stepping Forward in Time ***/
                () => loopWithFixedDelta(FixedTimeStep, initPivotParams, initLinkParams, true),
                /*** Stepping Backward in Time ***/
                () => loopWithFixedDelta(-FixedTimeStep, initPivotParams, initLinkParams, false));
#endif
        }
        private static AutoResetEvent forwardDone = new AutoResetEvent(false);
        private static AutoResetEvent backwardDone = new AutoResetEvent(false);

        private Boolean microPerturbForFiniteDifferenceOfVelocityAndAcceleration(double smallTimeStep, double[,] currentJointParams, double[,] currentLinkParams,
            double[,] initPivotParams, double[,] initLinkParams)
        {
            if (!DefineNewPositions(smallTimeStep * InputSpeed, currentJointParams, currentLinkParams, initPivotParams,
                                    initLinkParams))
                return false;
            InitializeGroundAndInputSpeedAndAcceleration(currentJointParams, currentLinkParams);
            NumericalVelocity(smallTimeStep, currentJointParams, currentLinkParams, initPivotParams, initLinkParams);
            NumericalAcceleration(smallTimeStep, currentJointParams, currentLinkParams, initPivotParams,
                                  initLinkParams);
            return true;
        }

        private void loopWithFixedDelta(double timeStep, double[,] lastPivotParams, double[,] lastLinkParams, Boolean Forward)
        {
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
                validPosition = DefineNewPositions(delta, currentPivotParams, currentLinkParams,
                    lastPivotParams, lastLinkParams);
                #endregion
                if (validPosition)
                {
                    if (Forward)
                        lock (InputRange) { InputRange[1] = currentLinkParams[inputLinkIndex, 0]; }
                    else lock (InputRange) { InputRange[0] = currentLinkParams[inputLinkIndex, 0]; }
                    InitializeGroundAndInputSpeedAndAcceleration(currentPivotParams, currentLinkParams);

                    #region Find Velocities for Current Position
                    if (!findVelocitiesThroughICMethod(currentTime, true))
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
