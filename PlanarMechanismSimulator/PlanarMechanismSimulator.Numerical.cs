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
                    if (p == inputIndex || joints[p].jointType == JointTypes.G) continue;
                    currentJointParams[i, 2] = (currentJointParams[i, 0] - lastJointParams[i, 0]) / deltaTime;
                    currentJointParams[i, 3] = (currentJointParams[i, 1] - lastJointParams[i, 1]) / deltaTime;
                }
                /* now that these have updated, we have to go back a fix any gears to be interpolations of their pivots. */
                for (int i = 0; i < p; i++)
                {
                    if (i == inputIndex || joints[i].jointType != JointTypes.G) continue;
                    var gear1 = joints[p].Link1;
                    var gear2 = joints[p].Link2;
                    var g1center = gear1.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[p].Link1));
                    var g2center = gear2.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[p].Link2));
                    currentJointParams[i, 2] = (lastLinkParams[links.IndexOf(gear1), 0] * (g1center.Y - joints[p].Y) +
                    lastLinkParams[links.IndexOf(gear2), 0] * (g2center.Y - joints[p].Y)) / 2.0;
                    currentJointParams[i, 3] = (lastLinkParams[links.IndexOf(gear1), 0] * (joints[p].X - g1center.X) +
                    lastLinkParams[links.IndexOf(gear2), 0] * (joints[p].X - g2center.X)) / 2.0;
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
            catch (Exception)
            {
                { }
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
                    if (p == inputIndex || joints[p].jointType == JointTypes.G) continue;
                    currentJointParams[i, 4] = (currentJointParams[i, 2] - lastJointParams[i, 2]) / deltaTime;
                    currentJointParams[i, 5] = (currentJointParams[i, 3] - lastJointParams[i, 3]) / deltaTime;
                }
                /* now that these have updated, we have to go back a fix any gears to be interpolations of their pivots. */
                for (int i = 0; i < p; i++)
                {
                    if (i == inputIndex || joints[i].jointType != JointTypes.G) continue;
                    var gear1 = joints[p].Link1;
                    var gear2 = joints[p].Link2;
                    var g1center = gear1.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[p].Link1));
                    var g2center = gear2.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[p].Link2));
                    currentJointParams[i, 4] = (lastLinkParams[links.IndexOf(gear1), 2] * (g1center.Y - joints[p].Y) +
                                        lastLinkParams[links.IndexOf(gear2), 2] * (g2center.Y - joints[p].Y)) / 2.0;
                    currentJointParams[i, 5] = (lastLinkParams[links.IndexOf(gear1), 3] * (joints[p].X - g1center.X) +
                    lastLinkParams[links.IndexOf(gear2), 3] * (joints[p].X - g2center.X)) / 2.0;
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
            catch (Exception)
            {
                { }
                throw;
            }
        }
        private Boolean NumericalPosition(double lastTime, double time_step)
        {
            try
            {
                var lastParams = JointParameters[lastTime]; //find the position in the JointParameters list

                /* this one DOES NOT put new values in the currentPivotParams, which is already full.
                 * This is to predict position of the next time step. Results are put into the joints
                 * directly. */
                for (int i = 0; i < p; i++)
                {
                    if (i == inputIndex || joints[i].jointType == JointTypes.G)
                        /* The input joint should already have been updated, so don't overwrite those values.
                         * For gear teeth, the position is of joint stays fixed even though the gear teeth are
                         * flying by. Have to do these at the end. */
                        continue;
                    joints[i].X = lastParams[i, 0] + lastParams[i, 2] * time_step +
                                  0.5 * lastParams[i, 4] * time_step * time_step;
                    joints[i].Y = lastParams[i, 1] + lastParams[i, 3] * time_step +
                                  0.5 * lastParams[i, 5] * time_step * time_step;

                    if (joints[i].LinkIsSlide(joints[i].Link1))
                    {
                    }
                    else if (joints[i].LinkIsSlide(joints[i].Link2))
                    {
                    }
                }
                /* now that these have updated, we have to go back a fix any gears to be interpolations of their pivots. */
                for (int i = 0; i < p; i++)
                {
                    if (i == inputIndex || joints[i].jointType != JointTypes.G) continue;
                    var link1center =
                        joints[p].Link1.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[p].Link1));
                    var link2center =
                        joints[p].Link2.joints.First(j => j.jointType != JointTypes.G && j.LinkIsSlide(joints[p].Link2));
                    var distToL1Center = Math.Sqrt((link1center.X - joints[p].X) * (link1center.X - joints[p].X)
                                                   + (link1center.Y - joints[p].Y) * (link1center.Y - joints[p].Y));
                    var distCenter2Center = Math.Sqrt((link1center.X - link2center.X) * (link1center.X - link2center.X)
                                                   + (link1center.Y - link2center.Y) * (link1center.Y - link2center.Y));
                    joints[p].X = link1center.X + (link2center.X - link1center.X) * (distToL1Center / distCenter2Center);
                    joints[p].Y = link1center.Y + (link2center.Y - link1center.Y) * (distToL1Center / distCenter2Center);
                }
                return true;
            }
            catch (Exception e)
            {
                Status += "Failed to find numerical position stepping " + time_step + " seconds from t = " + lastTime;
                return false;
            }
        }
    }
}

