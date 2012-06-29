﻿using System;
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
                for (int i = inputJointIndex + 1; i < p; i++)
                    newJointParams[i, 2] = newJointParams[i, 3] = 0.0;
                for (int i = 0; i < firstInputJointIndex; i++)
                    if (joints[i].jointType != JointTypes.G)
                    {
                        newJointParams[i, 2] = (newJointParams[i, 0] - lastJointParams[i, 0]) / deltaTime;
                        newJointParams[i, 3] = (newJointParams[i, 1] - lastJointParams[i, 1]) / deltaTime;
                    }
                /* now that these have updated, we have to go back a fix any gears to be interpolations of their pivots. */
                for (int i = 0; i < firstInputJointIndex; i++)
                    if (joints[i].jointType == JointTypes.G)
                    {
                        var gear1 = joints[i].Link1;
                        var gear2 = joints[i].Link2;
                        var g1center = gear1.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[i].Link1));
                        var g2center = gear2.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[i].Link2));
                        newJointParams[i, 2] = (lastLinkParams[links.IndexOf(gear1), 0] * (g1center.Y - joints[i].Y) +
                        lastLinkParams[links.IndexOf(gear2), 0] * (g2center.Y - joints[i].Y)) / 2.0;
                        newJointParams[i, 3] = (lastLinkParams[links.IndexOf(gear1), 0] * (joints[i].X - g1center.X) +
                        lastLinkParams[links.IndexOf(gear2), 0] * (joints[i].X - g2center.X)) / 2.0;
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
                for (int i = inputJointIndex + 1; i < p; i++)
                    newJointParams[i, 4] = newJointParams[i, 5] = 0.0;
                for (int i = 0; i < firstInputJointIndex; i++)
                    if (joints[i].jointType != JointTypes.G)
                    {
                        newJointParams[i, 4] = (newJointParams[i, 2] - lastJointParams[i, 2]) / deltaTime;
                        newJointParams[i, 5] = (newJointParams[i, 3] - lastJointParams[i, 3]) / deltaTime;
                    }
                /* now that these have updated, we have to go back a fix any gears to be interpolations of their pivots. */
                for (int i = 0; i < firstInputJointIndex; i++)
                    if (joints[i].jointType == JointTypes.G)
                    {
                        var gear1 = joints[i].Link1;
                        var gear2 = joints[i].Link2;
                        var g1center = gear1.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[i].Link1));
                        var g2center = gear2.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[i].Link2));
                        newJointParams[i, 4] = (lastLinkParams[links.IndexOf(gear1), 2] * (g1center.Y - joints[i].Y) +
                                            lastLinkParams[links.IndexOf(gear2), 2] * (g2center.Y - joints[i].Y)) / 2.0;
                        newJointParams[i, 5] = (lastLinkParams[links.IndexOf(gear1), 3] * (joints[i].X - g1center.X) +
                        lastLinkParams[links.IndexOf(gear2), 3] * (joints[i].X - g2center.X)) / 2.0;
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
                for (int i = inputJointIndex + 1; i < p; i++)
                {
                    newJointParams[i, 0] = lastJointParams[i, 0];
                    newJointParams[i, 1] = lastJointParams[i, 1];
                }
                /* The input link's joints should already have been updated, so don't overwrite those values.
                 * For gear teeth, the position is of joint stays fixed even though the gear teeth are
                 * flying by. Have to do these at the end. */
                for (int i = 0; i < firstInputJointIndex; i++)
                    if (joints[i].jointType != JointTypes.G)
                    {
                        newJointParams[i, 0] = lastJointParams[i, 0] + lastJointParams[i, 2] * deltaTime +
              0.5 * lastJointParams[i, 4] * deltaTime * deltaTime;
                        newJointParams[i, 1] = lastJointParams[i, 1] + lastJointParams[i, 3] * deltaTime +
                                      0.5 * lastJointParams[i, 5] * deltaTime * deltaTime;
                    }
                /* now that these have updated, we have to go back a fix any gears to be interpolations of their pivots. */
                for (int i = 0; i < firstInputJointIndex; i++)
                    if (joints[i].jointType == JointTypes.G)
                    {
                        var link1center = joints[i].Link1.referencePts[0];
                        var link2center = joints[i].Link2.referencePts[0];
                        //***************debug need to remove references to joints[i].X and .Y
                        var distToL1Center = Math.Sqrt((link1center.X - joints[i].X) * (link1center.X - joints[i].X)
                                                       + (link1center.Y - joints[i].Y) * (link1center.Y - joints[i].Y));
                        var distCenter2Center = Math.Sqrt((link1center.X - link2center.X) * (link1center.X - link2center.X)
                                                       + (link1center.Y - link2center.Y) * (link1center.Y - link2center.Y));
                        //***************
                        newJointParams[i, 0] = link1center.X + (link2center.X - link1center.X) * (distToL1Center / distCenter2Center);
                        newJointParams[i, 1] = link1center.Y + (link2center.Y - link1center.Y) * (distToL1Center / distCenter2Center);
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

