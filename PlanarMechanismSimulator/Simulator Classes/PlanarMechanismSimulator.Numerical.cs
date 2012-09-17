using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        private void NumericalPosition(double deltaTime, double[,] newJointParams, double[,] newLinkParams, double[,] lastJointParams, double[,] lastLinkParams)
        {
            for (int i = 0; i < numJoints; i++)
            {
                newJointParams[i, 0] = lastJointParams[i, 0] + lastJointParams[i, 2] * deltaTime +
                    0.5 * lastJointParams[i, 4] * deltaTime * deltaTime;
                newJointParams[i, 1] = lastJointParams[i, 1] + lastJointParams[i, 3] * deltaTime +
                              0.5 * lastJointParams[i, 5] * deltaTime * deltaTime;
            }
            for (int i = 0; i < inputLinkIndex; i++)
            {
                newLinkParams[i, 0] = lastLinkParams[i, 0] + lastLinkParams[i, 1] * deltaTime +
                    0.5 * lastLinkParams[i, 2] * deltaTime * deltaTime;

            }
        }
        private void NumericalVelocity(double deltaTime, double[,] newJointParams, double[,] newLinkParams, double[,] lastJointParams, double[,] lastLinkParams)
        {
            for (int i = 0; i < firstInputJointIndex; i++)
            {
                newJointParams[i, 2] = (newJointParams[i, 0] - lastJointParams[i, 0]) / deltaTime;
                newJointParams[i, 3] = (newJointParams[i, 1] - lastJointParams[i, 1]) / deltaTime;
            }
            for (int i = 0; i < inputLinkIndex; i++)
                newLinkParams[i, 1] = (newLinkParams[i, 0] - lastLinkParams[i, 0]) / deltaTime;
        }
        private void NumericalAcceleration(double deltaTime, double[,] newJointParams, double[,] newLinkParams, double[,] lastJointParams, double[,] lastLinkParams)
        {
            for (int i = 0; i < firstInputJointIndex; i++)
            {
                newJointParams[i, 4] = (newJointParams[i, 2] - lastJointParams[i, 2]) / deltaTime;
                newJointParams[i, 5] = (newJointParams[i, 3] - lastJointParams[i, 3]) / deltaTime;
            }
            for (int i = 0; i < inputLinkIndex; i++)
                newLinkParams[i, 2] = (newLinkParams[i, 1] - lastLinkParams[i, 1]) / deltaTime;
        }
    }
}

