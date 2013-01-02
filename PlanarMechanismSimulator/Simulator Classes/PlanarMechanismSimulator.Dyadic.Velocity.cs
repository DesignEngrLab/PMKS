#region

using System;
using System.Linq;
using System.Collections.Generic;
using OptimizationToolbox;
using System.Collections;

#endregion

namespace PlanarMechanismSimulator
//at time t=0; all acceleration and velocity are zero
{
    public partial class Simulator : IDependentAnalysis
    {
        private void InitializeGroundAndInputSpeedAndAcceleration()
        {
            for (int i = 0; i < inputJointIndex; i++)
            {
                joints[i].velocityKnown = KnownState.Unknown;
            }
            /* these are the ground joints, which are not moving. */
            for (int i = inputJointIndex + 1; i < numJoints; i++)
            {
                joints[i].vx = joints[i].vx_unit = 0.0;
                joints[i].vy = joints[i].vy_unit = 0.0;
                joints[i].velocityKnown = KnownState.Fully;
            }

            for (int i = 0; i < inputLinkIndex; i++)
            {
                var groundJoint = links[i].joints.FirstOrDefault(j => j.isGround);
                if (groundJoint == null)
                    links[i].velocityKnown = KnownState.Unknown;
                else
                {
                    links[i].velocityKnown = KnownState.Partially;
                    if (groundJoint.jointType == JointTypes.P)
                        links[i].InstantCenter = new point(double.PositiveInfinity, double.PositiveInfinity);
                    else links[i].InstantCenter = new point(groundJoint.x, groundJoint.y);

                }
            }
            links[inputLinkIndex + 1].Velocity = 0.0;
            links[inputLinkIndex + 1].velocityKnown = KnownState.Fully;
            links[inputLinkIndex + 1].InstantCenter = new point(0, 0);

            if (inputJoint.jointType == JointTypes.R)
            {
                links[inputLinkIndex].Velocity = InputSpeed;
                links[inputLinkIndex].Acceleration = 0.0;
                links[inputLinkIndex].InstantCenter = new point(inputJoint.x, inputJoint.y);
                links[inputLinkIndex].velocityKnown = KnownState.Fully;

                joints[inputJointIndex].vx = joints[inputJointIndex].vx_unit = 0.0;
                joints[inputJointIndex].vy = joints[inputJointIndex].vy_unit = 0.0;
                joints[inputJointIndex].velocityKnown = KnownState.Fully;
            }
            else if (inputJoint.jointType == JointTypes.P)
            {
                links[inputLinkIndex].Velocity = 0.0;
                links[inputLinkIndex].Acceleration = 0.0;
                links[inputLinkIndex].InstantCenter = new point(double.PositiveInfinity, double.PositiveInfinity);
                links[inputLinkIndex].velocityKnown = KnownState.Fully;

                var angle = inputJoint.SlideAngle;
                joints[inputJointIndex].vx_unit = Math.Cos(angle);
                joints[inputJointIndex].vy_unit = Math.Sin(angle);
                joints[inputJointIndex].vx = InputSpeed * joints[inputJointIndex].vx_unit;
                joints[inputJointIndex].vy = InputSpeed * joints[inputJointIndex].vy_unit;
                joints[inputJointIndex].velocityKnown = KnownState.Fully;
            }
            else throw new Exception("Currently only R or P can be the input joints.");
        }

        private Boolean DefineVelocitiesAnalytically()
        {
            InitializeGroundAndInputSpeedAndAcceleration();

            int numUnknownJoints;
            Boolean successfulLinkUpdate, successfulJointUpdate;
            do
            {
                successfulLinkUpdate = UpdateVelocitiesAndICsOfLinks();
                successfulJointUpdate = UpdateVelocitiesOfJoints();
                numUnknownJoints = joints.Count(j => j.velocityKnown != KnownState.Fully);
            } while (numUnknownJoints > 0 && (successfulJointUpdate || successfulLinkUpdate));
            if (numUnknownJoints == 0)
            {
                UpdateVelocitiesAndICsOfLinks();
                return true;
            }
            return false;
        }

        #region Update Velocities and ICs of Links
        private bool UpdateVelocitiesAndICsOfLinks()
        {
            var a_successful_update = false;
            foreach (var thisLink in links)
            {
                if (thisLink.velocityKnown == KnownState.Fully) continue;
                if (thisLink.velocityKnown == KnownState.Unknown)
                {
                    var jointsWithKnownVelDirs = thisLink.joints.Where(j => j.velocityKnown != KnownState.Unknown);
                    if (jointsWithKnownVelDirs.Count() >= 2)
                    {
                        thisLink.InstantCenter = findInstantCenter(jointsWithKnownVelDirs.ToList());
                        thisLink.velocityKnown = KnownState.Partially;
                        a_successful_update = true;
                    }
                }
                if (thisLink.velocityKnown == KnownState.Partially)
                {
                    var knownJoint = thisLink.joints.FirstOrDefault(j => j.velocityKnown == KnownState.Fully
                        && !(Constants.sameCloseZero(j.x, thisLink.InstantCenter.x)
                        && Constants.sameCloseZero(j.y, thisLink.InstantCenter.y)));
                    if (knownJoint != null)
                    {
                        setAngularVelocity(thisLink, knownJoint);
                        a_successful_update = true;
                    }
                }
            }
            return a_successful_update;
        }

        private void setAngularVelocity(link thisLink, joint knownJoint)
        {
            if (Double.IsNaN(thisLink.InstantCenter.x) || Double.IsInfinity(thisLink.InstantCenter.x)
                || Double.IsNaN(thisLink.InstantCenter.y) || Double.IsInfinity(thisLink.InstantCenter.y))
                thisLink.Velocity = 0.0;
            else
            {
                var negRy = -(knownJoint.y - thisLink.InstantCenter.y);
                var Rx = knownJoint.x - thisLink.InstantCenter.x;
                if (Constants.sameCloseZero(negRy))
                    thisLink.Velocity = knownJoint.vy / Rx;
                else if (Constants.sameCloseZero(Rx))
                    thisLink.Velocity = knownJoint.vx / negRy;
                else thisLink.Velocity = (knownJoint.vy / Rx + knownJoint.vx / negRy) / 2;
            }
            thisLink.velocityKnown = KnownState.Fully;
            // todo: need to recurse on all connected P joints
        }

        private point findInstantCenter(List<joint> jointsWithKnownVelDirs)
        {
            var lines = new SortedList<double, point>(new slopeSort());
            foreach (var j in jointsWithKnownVelDirs)
                lines.Add(-j.vx_unit / j.vy_unit, new point(j.x, j.y));
            while (lines.Count > 2)
            {
                var removeCand = lines.FirstOrDefault(line => (double.IsInfinity(line.Key) && double.IsInfinity(line.Value.x) && double.IsInfinity(line.Value.y))).Value;
                if (removeCand == null)
                    removeCand = lines.FirstOrDefault(line => (double.IsInfinity(line.Key) && (double.IsInfinity(line.Value.x) || double.IsInfinity(line.Value.y)))).Value;
                if (removeCand == null)
                    removeCand = lines.FirstOrDefault(line => (double.IsInfinity(line.Value.x) || double.IsInfinity(line.Value.y))).Value;
                var removeIndex = (removeCand == null) ? 1 : lines.IndexOfValue(removeCand);
                lines.RemoveAt(removeIndex);
            };
            return Constants.solveViaIntersectingLines(lines.Keys[0], lines.Values[0], lines.Keys[1], lines.Values[1]);
        }
        #endregion

        #region Update Velocities of Joints
        private bool UpdateVelocitiesOfJoints()
        {
            var a_successful_update = false;
            foreach (var j in joints)
            {
                if (j.velocityKnown == KnownState.Fully) continue;
                if (j.velocityKnown == KnownState.Unknown)
                {
                    if ((j.Link1.velocityKnown != KnownState.Unknown)
                        && (j.jointType == JointTypes.R || j.jointType == JointTypes.G))
                    {
                        updateJointUnitVectors(j, j.Link1);
                        a_successful_update = true;
                    }
                    else if (j.Link2.velocityKnown != KnownState.Unknown)
                    {
                        updateJointUnitVectors(j, j.Link2);
                        a_successful_update = true;
                    }
                }
                if (j.velocityKnown == KnownState.Partially)
                {
                    if ((j.Link1.velocityKnown == KnownState.Fully)
                        && (j.jointType == JointTypes.R || j.jointType == JointTypes.G))
                    {
                        updateJointVelocityVector(j, j.Link1);
                        a_successful_update = true;
                    }
                    else if (j.Link2.velocityKnown == KnownState.Fully)
                    {
                        updateJointVelocityVector(j, j.Link2);
                        a_successful_update = true;
                    }
                }
            }
            return a_successful_update;
        }

        private void updateJointVelocityVector(joint j, link l)
        {
            if (Double.IsNaN(l.InstantCenter.x) || Double.IsInfinity(l.InstantCenter.x)
                || Double.IsNaN(l.InstantCenter.y) || Double.IsInfinity(l.InstantCenter.y))
            {
                var knownJoint = l.joints.First(jj => jj != j && jj.velocityKnown == KnownState.Fully
                    && jj.FixedWithRespectTo(l));
                j.vx = knownJoint.vx;
                j.vy = knownJoint.vy;
                var magnitude = Math.Sqrt(j.vx * j.vx + j.vy * j.vy);
                j.vx_unit = j.vx / magnitude;
                j.vy_unit = j.vy / magnitude;
            }
            else
            {
                // todo: need to add radius!
                j.vx = j.vx_unit * l.Velocity;
                j.vy = j.vy_unit * l.Velocity;
            }
            j.velocityKnown = KnownState.Fully;
        }

        private void updateJointUnitVectors(joint j, link l)
        {
            var vx = j.x;
            var vy = j.y;
            vx -= l.InstantCenter.x;
            vy -= l.InstantCenter.y;
            var magnitude = Math.Sqrt(vx * vx + vy * vy);
            j.vx_unit = -vy / magnitude;
            j.vy_unit = vx / magnitude;
            j.velocityKnown = KnownState.Partially;
        }
        #endregion
    }

    /// <summary>
    /// A comparer for optimization that can be used for either 
    /// minimization or maximization.
    /// </summary>
    internal class slopeSort : IComparer<double>
    {
        public int Compare(double x, double y)
        {
            if (Math.Abs(x) > Math.Abs(y)) return -1;
            else return 1;
        }
    }

}