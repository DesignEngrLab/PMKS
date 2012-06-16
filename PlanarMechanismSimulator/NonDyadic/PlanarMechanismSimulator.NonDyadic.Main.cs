using System;
using System.Collections.Generic;
using OptimizationToolbox;
using StarMathLib;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        private const int numberOfTries = 100;
        private void FindFullMovementNonDyadic()
        {
            var currentTime = 0.0;
            var currentPivotParams = new double[p, 6];
            for (int i = 0; i < p; i++)
            {
                currentPivotParams[i, 0] = joints[i].X;
                currentPivotParams[i, 1] = joints[i].Y;
            }
            JointParameters.Add(currentTime, currentPivotParams);
            var currentLinkParams = new double[n, 2];
            LinkParameters.Add(currentTime, currentLinkParams);
            bool validPosition;

            var ndPosFinder = new NonDyadicPositionFinder(links, joints, inputIndex, epsilon);

            do /*** Stepping Forward in Time **/
            {
                NumericalVelocity(currentTime, true);
                NumericalAcceleration(currentTime, true);
                #region Find Next Positions
                /* this time, NumericalPosition is called first and instead of
                 * updating currentPivotParams (which is already full at this point)
                 * we update the X, Y positions of the joints, which are global to the method. */
                NumericalPosition(currentTime, FixedTimeStep);
                /* Based upon the numerical approximation, we analytically update the remaining
                 * joints. First, we analytically set the input pivot.*/
                MoveInputToNextPosition(currentTime, FixedTimeStep);
                validPosition = ndPosFinder.Run_PositionsAreClose();
                /* create new currentPivotParams based on these updated positions of the joints */
                if (validPosition)
                {
                    currentPivotParams = new double[p, 6];
                    for (int i = 0; i < p; i++)
                    {
                        currentPivotParams[i, 0] = joints[i].X;
                        currentPivotParams[i, 1] = joints[i].Y;
                    }
                    currentTime += FixedTimeStep;
                    JointParameters.Add(currentTime, currentPivotParams);
                    currentLinkParams = new double[n, 2];
                    LinkParameters.Add(currentTime, currentLinkParams);
                }
                #endregion
            } while (validPosition && lessThanFullRotation());
            if (!lessThanFullRotation()) return;

            #region Reset to t=0

            currentPivotParams = JointParameters.Values[0];
            currentTime = JointParameters.Keys[0];
            for (int i = 0; i < p; i++)
            {
                joints[i].X = currentPivotParams[i, 0];
                joints[i].Y = currentPivotParams[i, 1];
            }

            #endregion

            #region Find first backwards step positions
            NumericalPosition(currentTime, -FixedTimeStep);
            /* Based upon the numerical approximation, we analytically update the remaining
             * joints. First, we analytically set the input pivot.*/
            MoveInputToNextPosition(currentTime, -FixedTimeStep);
            validPosition = ndPosFinder.Run_PositionsAreClose();
            /* create new currentPivotParams based on these updated positions of the joints */
            if (!validPosition) return;
            currentPivotParams = new double[p, 6];
            for (int i = 0; i < p; i++)
            {
                currentPivotParams[i, 0] = joints[i].X;
                currentPivotParams[i, 1] = joints[i].Y;
            }
            currentTime -= FixedTimeStep;
            JointParameters.Add(currentTime, currentPivotParams);
            currentLinkParams = new double[n, 2];
            LinkParameters.Add(currentTime, currentLinkParams);
            #endregion

            do /*** Stepping Backward in Time **/
            {
                NumericalVelocity(currentTime, false);
                NumericalAcceleration(currentTime, false);
                #region Find Next Positions

                /* this time, NumericalPosition is called first and instead of
                 * updating currentPivotParams (which is already full at this point)
                 * we update the X, Y positions of the joints, which are global to the method. */
                NumericalPosition(currentTime, -FixedTimeStep);
                /* Based upon the numerical approximation, we analytically update the remaining
                 * joints. First, we analytically set the input pivot.*/
                MoveInputToNextPosition(currentTime, -FixedTimeStep);
                validPosition = ndPosFinder.Run_PositionsAreClose();
                /* create new currentPivotParams based on these updated positions of the joints */
                if (validPosition)
                {
                    currentPivotParams = new double[p, 6];
                    for (int i = 0; i < p; i++)
                    {
                        currentPivotParams[i, 0] = joints[i].X;
                        currentPivotParams[i, 1] = joints[i].Y;
                    }
                    currentTime += FixedTimeStep;
                    JointParameters.Add(currentTime, currentPivotParams);
                    currentLinkParams = new double[n, 2];
                    LinkParameters.Add(currentTime, currentLinkParams);
                }
                #endregion
            } while (validPosition && lessThanFullRotation());
        }

    }
}
