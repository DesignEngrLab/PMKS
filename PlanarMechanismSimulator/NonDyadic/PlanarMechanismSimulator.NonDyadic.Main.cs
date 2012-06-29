using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using OptimizationToolbox;
using StarMathLib;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        private const int numberOfTries = 100;
        private void FindFullMovementNonDyadic()
        {
            #region Set up initial point parameters (x, x-dot, x-double-dot, etc.)
            var initPivotParams = new double[p, 6];
            for (int i = 0; i < p; i++)
            {
                initPivotParams[i, 0] = joints[i].X;
                initPivotParams[i, 1] = joints[i].Y;
            }
            var initLinkParams = new double[n, 3];
            for (int i = 0; i < n; i++)
                initLinkParams[i, 0] = links[i].Angle;
            angleRange = new[] { links[inputLinkIndex].Angle, links[inputLinkIndex].Angle };
            JointParameters.Add(0.0, initPivotParams);
            LinkParameters.Add(0.0, initLinkParams);
            MoveInputToNextPosition(0.0, initPivotParams, initLinkParams, initPivotParams, initLinkParams);
            var lastForwardPivotParams = initPivotParams;
            var lastForwardLinkParams = initLinkParams;
            var lastBackwardPivotParams = initPivotParams;
            var lastBackwardLinkParams = initLinkParams;
            /* attempt to find velocities and accelerations at initial point analytically
             * there is no point in trying numerically as this is the first point and the numerical methods
             * perform finite difference of current and last time steps. */
            if (!(findVelocitiesThroughICMethod(0.0, true) && findAccelerationAnalytically(0.0, true)))
            {
                var smallBackwardStepJointParams = (double[,])initPivotParams.Clone();
                var dummyLinkParams = new double[n, 3];
                MoveInputToNextPosition(-0.01 * InputSpeed * FixedTimeStep, smallBackwardStepJointParams,
                    dummyLinkParams, initPivotParams, initLinkParams);
                var ndPosFinder = new NonDyadicPositionFinder(links, joints, firstInputJointIndex,
                    inputJointIndex, inputLinkIndex, epsilon);
                if (ndPosFinder.Run_PositionsAreClose(smallBackwardStepJointParams, dummyLinkParams))
                {
                    JointParameters.Add(-0.01 * FixedTimeStep, smallBackwardStepJointParams);
                    LinkParameters.Add(-0.01 * FixedTimeStep, dummyLinkParams);
                    NumericalVelocity(-0.01 * FixedTimeStep, initPivotParams, initLinkParams,
                        smallBackwardStepJointParams, dummyLinkParams);
                    NumericalAcceleration(-0.01 * FixedTimeStep, initPivotParams, initLinkParams,
                        smallBackwardStepJointParams, dummyLinkParams);
                    JointParameters.Remove(-0.01 * FixedTimeStep);
                    LinkParameters.Remove(-0.01 * FixedTimeStep);
                }
                else throw new Exception("Unable to establish micro-perturbation to initial position.");
            }
            #endregion

            Parallel.Invoke(
            #region *** Stepping Forward in Time ***
delegate
{
    var currentTime = 0.0;
    Boolean validPosition;
    var ndForwardPosFinder = new NonDyadicPositionFinder(links, joints, firstInputJointIndex,
        inputJointIndex, inputLinkIndex, epsilon);
    do
    {
        var currentLinkParams = new double[n, 3];
        var currentPivotParams = new double[p, 6];
        #region Find Next Positions
        /* this time, NumericalPosition is called first and instead of
         * updating currentPivotParams (which is already full at this point)
         * we update the X, Y positions of the joints, which are global to the method. */
        NumericalPosition(FixedTimeStep, currentPivotParams, currentLinkParams,
            lastForwardPivotParams, lastForwardLinkParams);
        /* First, we analytically set the input pivot.*/
        MoveInputToNextPosition(InputSpeed * FixedTimeStep, currentPivotParams, currentLinkParams,
            lastForwardPivotParams, lastForwardLinkParams);
        /* Based upon the numerical approximation, we analytically update the remaining
         * joints. */
        validPosition = ndForwardPosFinder.Run_PositionsAreClose(currentPivotParams, currentLinkParams);
        /* create new currentPivotParams based on these updated positions of the joints */
        #endregion
        if (validPosition)
        {
            lock (angleRange) { angleRange[0] = currentLinkParams[inputLinkIndex, 0]; }

            #region Find Velocities for Current Position
            if (!findVelocitiesThroughICMethod(currentTime, true))
            {
                Status += "Instant Centers could not be found at" + currentTime + ".";
                NumericalVelocity(FixedTimeStep, currentPivotParams, currentLinkParams,
                    lastForwardPivotParams, lastForwardLinkParams);
            }
            #endregion
            #region Find Accelerations for Current Position
            if (!findAccelerationAnalytically(currentTime, true))
            {
                Status += "Analytical acceleration could not be found at" + currentTime + ".";
                NumericalAcceleration(FixedTimeStep, currentPivotParams, currentLinkParams,
                    lastForwardPivotParams, lastForwardLinkParams);
            }
            #endregion
            currentTime += FixedTimeStep;
            lock (JointParameters)
                JointParameters.Add(currentTime, currentPivotParams);
            lock (LinkParameters)
                LinkParameters.Add(currentTime, currentLinkParams);
            lastForwardPivotParams = currentPivotParams;
            lastForwardLinkParams = currentLinkParams;
        }
    } while (validPosition && lessThanFullRotation());
}
            #endregion
            #region *** Stepping Backward in Time ***
, delegate
                 {
                     var currentTime = 0.0;
                     Boolean validPosition;
                     var ndBackwardPosFinder = new NonDyadicPositionFinder(links, joints, firstInputJointIndex,
                         inputJointIndex, inputLinkIndex, epsilon);
                     do
                     {
                         var currentLinkParams = new double[n, 3];
                         var currentPivotParams = new double[p, 6];
                         #region Find Next Positions
                         /* this time, NumericalPosition is called first and instead of
                          * updating currentPivotParams (which is already full at this point)
                          * we update the X, Y positions of the joints, which are global to the method. */
                         NumericalPosition(-FixedTimeStep, currentPivotParams, currentLinkParams,
                             lastBackwardPivotParams, lastBackwardLinkParams);
                         /* First, we analytically set the input pivot.*/
                         MoveInputToNextPosition(-InputSpeed * FixedTimeStep, currentPivotParams, currentLinkParams,
                lastBackwardPivotParams, lastBackwardLinkParams);
                         /* Based upon the numerical approximation, we analytically update the remaining
                          * joints. */
                         validPosition = ndBackwardPosFinder.Run_PositionsAreClose(currentPivotParams, currentLinkParams);
                         /* create new currentPivotParams based on these updated positions of the joints */
                         #endregion
                         if (validPosition)
                         {
                             lock (angleRange) { angleRange[1] = currentLinkParams[inputLinkIndex, 0]; }

                             #region Find Velocities for Current Position
                             if (!findVelocitiesThroughICMethod(currentTime, true))
                             {
                                 Status += "Instant Centers could not be found at" + currentTime + ".";
                                 NumericalVelocity(-FixedTimeStep, currentPivotParams, currentLinkParams,
                             lastBackwardPivotParams, lastBackwardLinkParams);
                             }
                             #endregion
                             #region Find Accelerations for Current Position
                             if (!findAccelerationAnalytically(currentTime, true))
                             {
                                 Status += "Analytical acceleration could not be found at" + currentTime + ".";
                                 NumericalAcceleration(-FixedTimeStep, currentPivotParams, currentLinkParams,
                             lastBackwardPivotParams, lastBackwardLinkParams);
                             }
                             #endregion
                             currentTime -= FixedTimeStep; lock (JointParameters)
                                 JointParameters.Add(currentTime, currentPivotParams);
                             lock (LinkParameters)
                                 LinkParameters.Add(currentTime, currentLinkParams);
                             lastBackwardPivotParams = currentPivotParams;
                             lastBackwardLinkParams = currentLinkParams;
                         }
                     } while (validPosition && lessThanFullRotation());
            #endregion
                 }
);
        }
    }
}

