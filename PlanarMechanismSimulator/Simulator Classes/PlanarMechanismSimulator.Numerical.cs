using System.Collections.Generic;
using OptimizationToolbox;
using System;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        private void NumericalPosition(double deltaTime, List<joint> joints, List<link> links)
        {
            for (int i = 0; i < numJoints; i++)
            {
                joints[i].xNumerical = joints[i].xLast + joints[i].vx * deltaTime + 0.5 * joints[i].ax * deltaTime * deltaTime;
                joints[i].yNumerical = joints[i].yLast + joints[i].vy * deltaTime + 0.5 * joints[i].ay * deltaTime * deltaTime;
            }
            for (int i = 0; i < inputLinkIndex; i++)
                links[i].Angle = links[i].AngleLast + links[i].Velocity * deltaTime + 0.5 * links[i].Acceleration * deltaTime * deltaTime;
        }

        private void NumericalVelocity(double deltaTime, List<joint> joints, List<link> links)
        {
            for (int i = 0; i < firstInputJointIndex; i++)
            {
                joints[i].vx = (joints[i].x - joints[i].xLast) / deltaTime;
                joints[i].vy = (joints[i].y - joints[i].yLast) / deltaTime;
                var magnitude = Math.Sqrt(joints[i].vx * joints[i].vx + joints[i].vy * joints[i].vy);
            }
            for (int i = 0; i < inputLinkIndex; i++)
                links[i].Velocity = (links[i].Angle - links[i].AngleLast) / deltaTime;
        }

        private void NumericalAcceleration(double deltaTime, List<joint> joints, List<link> links)
        {
            for (int i = 0; i < firstInputJointIndex; i++)
            {
                joints[i].ax = (joints[i].vx - joints[i].vxLast) / deltaTime;
                joints[i].ay = (joints[i].vy - joints[i].vyLast) / deltaTime;
            }
            for (int i = 0; i < inputLinkIndex; i++)
                links[i].Acceleration = (links[i].Velocity - links[i].VelocityLast) / deltaTime;
        }
    }
}

