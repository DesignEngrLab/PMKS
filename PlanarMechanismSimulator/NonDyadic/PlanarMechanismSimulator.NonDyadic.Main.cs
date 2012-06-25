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
            var ndPosFinder = new NonDyadicPositionFinder(links, joints, inputJointIndex, epsilon);
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
            JointParameters.Add(0.0, initPivotParams);
            LinkParameters.Add(0.0, initLinkParams);
            /* attempt to find velocities and accelerations at initial point analytically
             * there is no point in trying numerically as this is the first point and the numerical methods
             * perform finite difference of current and last time steps. */
            var smallBackwardStepJointParams = (double[,])initPivotParams.Clone();
            var dummyLinkParams = new double[n, 3];
            MoveInputToNextPosition(0.0, -0.01 * FixedTimeStep, smallBackwardStepJointParams, dummyLinkParams);
            if (ndPosFinder.Run_PositionsAreClose())
            {
                JointParameters.Add(-0.01 * FixedTimeStep, smallBackwardStepJointParams);
                NumericalVelocity(0.0, true);
                NumericalAcceleration(0.0, true);
                JointParameters.Remove(-0.01 * FixedTimeStep);
            }
            else throw new Exception("Unable to establish micro-perturbation to initial position.");
            #endregion
            #region Work Simultaneously in both rotation directions
            Parallel.Invoke(
                delegate()
                {
                    var currentTime = 0.0;
                    var currentLinkParams = new double[n, 2];
                    var currentPivotParams = new double[p, 6];
                    Boolean validPosition;
                    do /*** Stepping Forward in Time **/
                    {
                        #region Find Next Positions
                        /* First, we analytically set the input pivot.*/
                        MoveInputToNextPosition(currentTime, FixedTimeStep, currentPivotParams, currentLinkParams);
                        /* this time, NumericalPosition is called first and instead of
                         * updating currentPivotParams (which is already full at this point)
                         * we update the X, Y positions of the joints, which are global to the method. */
                        NumericalPosition(currentTime, FixedTimeStep, currentPivotParams, currentLinkParams);
                        /* Based upon the numerical approximation, we analytically update the remaining
                         * joints. */
                        validPosition = ndPosFinder.Run_PositionsAreClose();
                        /* create new currentPivotParams based on these updated positions of the joints */
                        if (validPosition)
                        {
                            currentTime += FixedTimeStep;
                            JointParameters.Add(currentTime, currentPivotParams);
                            LinkParameters.Add(currentTime, currentLinkParams);
                            lock (angleRange) { angleRange[0] = currentLinkParams[inputJointIndex, 0]; }
                        #endregion
                            NumericalVelocity(currentTime, true);
                            NumericalAcceleration(currentTime, true);
                        }
                    } while (validPosition && lessThanFullRotation());
                },
                delegate()
                {
                    var currentTime = 0.0;
                    var currentLinkParams = new double[n, 2];
                    var currentPivotParams = new double[p, 6];
                    Boolean validPosition;
                    do /*** Stepping Backward in Time **/
                    {
                        #region Find Next Positions
                        /* First, we analytically set the input pivot.*/
                        MoveInputToNextPosition(currentTime, -FixedTimeStep, currentPivotParams, currentLinkParams);
                        /* this time, NumericalPosition is called first and instead of
                         * updating currentPivotParams (which is already full at this point)
                         * we update the X, Y positions of the joints, which are global to the method. */
                        NumericalPosition(currentTime, -FixedTimeStep, currentPivotParams, currentLinkParams);
                        /* Based upon the numerical approximation, we analytically update the remaining
                         * joints. */
                        validPosition = ndPosFinder.Run_PositionsAreClose();
                        /* create new currentPivotParams based on these updated positions of the joints */
                        if (validPosition)
                        {
                            currentTime -= FixedTimeStep;
                            JointParameters.Add(currentTime, currentPivotParams);
                            LinkParameters.Add(currentTime, currentLinkParams);
                            lock (angleRange) { angleRange[1] = currentLinkParams[inputJointIndex, 0]; }
                        #endregion
                            NumericalVelocity(currentTime, false);
                            NumericalAcceleration(currentTime, false);
                        }
                    } while (validPosition && lessThanFullRotation());
                });
            #endregion
        }
    }
}

