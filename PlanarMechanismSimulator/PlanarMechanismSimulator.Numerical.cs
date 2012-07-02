using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        //todo:update LinkParameters in Velocity and Acceleration
        private Boolean NumericalVelocity(double deltaTime, double[,] newJointParams, double[,] newLinkParams, double[,] lastJointParams, double[,] lastLinkParams)
        {
            try
            {
                /* first set the ground joints to zero velocity */
                for (int i = inputJointIndex + 1; i < numJoints; i++)
                    newJointParams[i, 2] = newJointParams[i, 3] = 0.0;
                for (int i = 0; i < firstInputJointIndex; i++)
                    {
                        newJointParams[i, 2] = (newJointParams[i, 0] - lastJointParams[i, 0]) / deltaTime;
                        newJointParams[i, 3] = (newJointParams[i, 1] - lastJointParams[i, 1]) / deltaTime;
                    }
                for (int i = 0; i < inputLinkIndex; i++)
                    newLinkParams[i, 1] = (newLinkParams[i, 0] - lastLinkParams[i, 0]) / deltaTime;
                return true;
            }
            catch (Exception e)
            {
                throw;
            }
        }
        private Boolean NumericalAcceleration(double deltaTime, double[,] newJointParams, double[,] newLinkParams, double[,] lastJointParams, double[,] lastLinkParams)
        {
            try
            {
                /* first set the ground joints to zero acceleration */
                for (int i = inputJointIndex + 1; i < numJoints; i++)
                    newJointParams[i, 4] = newJointParams[i, 5] = 0.0;
                for (int i = 0; i < firstInputJointIndex; i++)
                    {
                        newJointParams[i, 4] = (newJointParams[i, 2] - lastJointParams[i, 2]) / deltaTime;
                        newJointParams[i, 5] = (newJointParams[i, 3] - lastJointParams[i, 3]) / deltaTime;
                    }
                for (int i = 0; i < inputLinkIndex; i++)
                    newLinkParams[i, 2] = (newLinkParams[i, 1] - lastLinkParams[i, 1]) / deltaTime;
                return true;
            }
            catch (Exception e)
            {
                { }
                throw;
            }
        }
        private void NumericalPosition(double deltaTime, double[,] newJointParams, double[,] newLinkParams, double[,] lastJointParams, double[,] lastLinkParams)
        {
            try
            {
                for (int i = inputJointIndex + 1; i < numJoints; i++)
                {
                    newJointParams[i, 0] = lastJointParams[i, 0];
                    newJointParams[i, 1] = lastJointParams[i, 1];
                }
                /* The input link's joints should already have been updated, so don't overwrite those values.
                 * For gear teeth, the position is of joint stays fixed even though the gear teeth are
                 * flying by. Have to do these at the end. */
                for (int i = 0; i < firstInputJointIndex; i++)
                    {
                        newJointParams[i, 0] = lastJointParams[i, 0] + lastJointParams[i, 2] * deltaTime +
              0.5 * lastJointParams[i, 4] * deltaTime * deltaTime;
                        newJointParams[i, 1] = lastJointParams[i, 1] + lastJointParams[i, 3] * deltaTime +
                                      0.5 * lastJointParams[i, 5] * deltaTime * deltaTime;
                    }
                for (int i = 0; i < inputLinkIndex; i++)
                    newLinkParams[i, 0] = lastLinkParams[i, 0] + lastLinkParams[i, 1] * deltaTime +
              0.5 * lastLinkParams[i, 2] * deltaTime * deltaTime;

            }
            catch (Exception e)
            {
                throw new Exception("Failed to find numerical position stepping " + deltaTime + " seconds.");
            }
        }
    }
}

