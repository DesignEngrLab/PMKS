using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        //todo:update LinkParameters in Velocity and Acceleration
        private Boolean NumericalVelocity(double currentTime, Boolean forwardInTime)
        {
            try
            {
                int currentIndex = JointParameters.IndexOfKey(currentTime);
                //find the position in the JointParameters list
                var currentJointParams = JointParameters.Values[currentIndex];
                //could get values through iterator, but assuming 
                // this is faster since we already have position.
                var lastIndex = forwardInTime ? currentIndex - 1 : currentIndex + 1; //establish last index
                var lastJointParams = currentJointParams; //as a default, initialize is the same as currentParams
                var deltaTime = JointParameters.Keys[currentIndex] - JointParameters.Keys[lastIndex];
                if (lastIndex >= 0 && lastIndex < JointParameters.Count) // but ideally it is distinct, so long as it is
                    lastJointParams = JointParameters.Values[lastIndex]; // not at the end of the list.
                var currentLinkParams = LinkParameters.Values[currentIndex];
                var lastLinkParams = LinkParameters.Values[lastIndex];
                for (int i = 0; i < p; i++)
                {
                    if (joints[i].isGround)
                    {
                        currentJointParams[i, 2] = currentJointParams[i, 3] = 0.0;
                        continue;
                    }
                    if (inputLink.joints.Contains(joints[i]) || joints[i].jointType == JointTypes.G) continue;
                    currentJointParams[i, 2] = (currentJointParams[i, 0] - lastJointParams[i, 0]) / deltaTime;
                    currentJointParams[i, 3] = (currentJointParams[i, 1] - lastJointParams[i, 1]) / deltaTime;
                }
                /* now that these have updated, we have to go back a fix any gears to be interpolations of their pivots. */
                for (int i = 0; i < p; i++)
                {
                    if (joints[i].isGround || inputLink.joints.Contains(joints[i]) || joints[i].jointType != JointTypes.G) continue;
                    var gear1 = joints[i].Link1;
                    var gear2 = joints[i].Link2;
                    var g1center = gear1.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[i].Link1));
                    var g2center = gear2.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[i].Link2));
                    currentJointParams[i, 2] = (lastLinkParams[links.IndexOf(gear1), 0] * (g1center.Y - joints[i].Y) +
                    lastLinkParams[links.IndexOf(gear2), 0] * (g2center.Y - joints[i].Y)) / 2.0;
                    currentJointParams[i, 3] = (lastLinkParams[links.IndexOf(gear1), 0] * (joints[i].X - g1center.X) +
                    lastLinkParams[links.IndexOf(gear2), 0] * (joints[i].X - g2center.X)) / 2.0;
                }
                /* todo: now need to update currentLinkParams based on values from currentPivotParams. Is there a better
                 * way to do gears? */
                for (int i = 0; i < n; i++)
                {
                    /* will need cases for each joint type that a link connects to*/
                    var attachedJoints = links[i].joints;
                    var xC = attachedJoints.Sum(j => j.X) / attachedJoints.Count;
                    var yC = attachedJoints.Sum(j => j.Y) / attachedJoints.Count;
                }
                return true;
            }
            catch (Exception e)
            {
                throw;
            }
        }
        private Boolean NumericalAcceleration(double currentTime, Boolean forwardInTime)
        {
            try
            {
                var currentIndex = JointParameters.IndexOfKey(currentTime);
                //find the position in the JointParameters list
                var currentJointParams = JointParameters.Values[currentIndex];
                //could get values through iterator, but assuming 
                // this is faster since we already have position.
                var lastIndex = forwardInTime ? currentIndex - 1 : currentIndex + 1; //establish last index
                var lastJointParams = currentJointParams; //as a default, initialize is the same as currentParams
                var deltaTime = JointParameters.Keys[currentIndex] - JointParameters.Keys[lastIndex];
                if (lastIndex >= 0 && lastIndex < JointParameters.Count) // but ideally it is distinct, so long as it is
                    lastJointParams = JointParameters.Values[lastIndex]; // not at the end of the list.
                var currentLinkParams = LinkParameters.Values[currentIndex];
                var lastLinkParams = LinkParameters.Values[lastIndex];
                for (int i = 0; i < p; i++)
                {
                    if (joints[i].isGround)
                    {
                        currentJointParams[i, 4] = currentJointParams[i, 5] = 0.0;
                        continue;
                    }
                    if (inputLink.joints.Contains(joints[i]) || joints[i].jointType == JointTypes.G) continue;
                    currentJointParams[i, 4] = (currentJointParams[i, 2] - lastJointParams[i, 2]) / deltaTime;
                    currentJointParams[i, 5] = (currentJointParams[i, 3] - lastJointParams[i, 3]) / deltaTime;
                }
                /* now that these have updated, we have to go back a fix any gears to be interpolations of their pivots. */
                for (int i = 0; i < p; i++)
                {
                    if (joints[i].isGround || inputLink.joints.Contains(joints[i]) || joints[i].jointType != JointTypes.G) continue;
                    var gear1 = joints[i].Link1;
                    var gear2 = joints[i].Link2;
                    var g1center = gear1.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[i].Link1));
                    var g2center = gear2.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[i].Link2));
                    currentJointParams[i, 4] = (lastLinkParams[links.IndexOf(gear1), 2] * (g1center.Y - joints[i].Y) +
                                        lastLinkParams[links.IndexOf(gear2), 2] * (g2center.Y - joints[i].Y)) / 2.0;
                    currentJointParams[i, 5] = (lastLinkParams[links.IndexOf(gear1), 3] * (joints[i].X - g1center.X) +
                    lastLinkParams[links.IndexOf(gear2), 3] * (joints[i].X - g2center.X)) / 2.0;
                }
                /* todo: now need to update currentLinkParams based on values from currentPivotParams. Is there a better
                 * way to do gears? */
                for (int i = 0; i < n; i++)
                {
                    /* will need cases for each joint type that a link connects to*/
                    var attachedJoints = links[i].joints;
                    var xC = attachedJoints.Sum(j => j.X) / attachedJoints.Count;
                    var yC = attachedJoints.Sum(j => j.Y) / attachedJoints.Count;
                }
                return true;
            }
            catch (Exception e)
            {
                { }
                throw;
            }
        }
        private void NumericalPosition(double currentTime, double time_step, double[,] newPivotParams, double[,] newLinkParams)
        {
            try
            {
                var lastParams = JointParameters[currentTime]; //find the position in the JointParameters list

                for (int i = 0; i < p; i++)
                {
                    if (joints[i].isGround)
                    {
                        newPivotParams[i, 0] = lastParams[i, 0];
                        newPivotParams[i, 1] = lastParams[i, 1];
                        continue;
                    }
                    if (inputLink.joints.Contains(joints[i]) || joints[i].jointType == JointTypes.G)
                        /* The input link's joints should already have been updated, so don't overwrite those values.
                         * For gear teeth, the position is of joint stays fixed even though the gear teeth are
                         * flying by. Have to do these at the end. */
                        continue;
                    newPivotParams[i, 0] = lastParams[i, 0] + lastParams[i, 2] * time_step +
                                  0.5 * lastParams[i, 4] * time_step * time_step;
                    newPivotParams[i, 1] = lastParams[i, 1] + lastParams[i, 3] * time_step +
                                  0.5 * lastParams[i, 5] * time_step * time_step;
                }
                /* now that these have updated, we have to go back a fix any gears to be interpolations of their pivots. */
                for (int i = 0; i < p; i++)
                {
                    if (joints[i].isGround || inputLink.joints.Contains(joints[i]) || joints[i].jointType != JointTypes.G) continue;
                    var link1center = joints[i].Link1.referencePts[0];
                    var link2center = joints[i].Link2.referencePts[0];
                    //***************debug need to remove references to joints[i].X and .Y
                    var distToL1Center = Math.Sqrt((link1center.X - joints[i].X) * (link1center.X - joints[i].X)
                                                   + (link1center.Y - joints[i].Y) * (link1center.Y - joints[i].Y));
                    var distCenter2Center = Math.Sqrt((link1center.X - link2center.X) * (link1center.X - link2center.X)
                                                   + (link1center.Y - link2center.Y) * (link1center.Y - link2center.Y));
                    //***************
                    newPivotParams[i, 0] = link1center.X + (link2center.X - link1center.X) * (distToL1Center / distCenter2Center);
                    newPivotParams[i, 1] = link1center.Y + (link2center.Y - link1center.Y) * (distToL1Center / distCenter2Center);
                }
            }
            catch (Exception e)
            {
                throw new Exception("Failed to find numerical position stepping " + time_step + " seconds from t = " + currentTime);
            }
        }
    }
}

