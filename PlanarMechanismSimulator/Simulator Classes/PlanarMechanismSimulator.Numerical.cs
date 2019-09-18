// ***********************************************************************
// Assembly         : PlanarMechanismKinematicSimulator
// Author           : Matt
// Created          : 06-10-2015
//
// Last Modified By : Matt
// Last Modified On : 06-28-2015
// ***********************************************************************
// <copyright file="PlanarMechanismSimulator.Numerical.cs" company="">
//     Copyright ©  2014
// </copyright>
// <summary></summary>
// ***********************************************************************

using System;
using System.Collections.Generic;
using OptimizationToolbox;

namespace PMKS
{
    /// <summary>
    /// Class Simulator.
    /// </summary>
    public partial class Simulator : IDependentAnalysis
    {
        /// <summary>
        /// 
        /// Finds the numerical position.
        /// </summary>
        /// <param name="deltaTime">The delta time.</param>
        /// <param name="joints">The joints.</param>
        /// <param name="links">The links.</param>
        private void NumericalPosition(double deltaTime, List<Joint> joints, List<Link> links)
        {
            for (int i = 0; i < NumAllJoints; i++)
            {
                joints[i].xNumerical = joints[i].xLast + joints[i].vx * deltaTime + 0.5 * joints[i].ax * deltaTime * deltaTime;
                joints[i].yNumerical = joints[i].yLast + joints[i].vy * deltaTime + 0.5 * joints[i].ay * deltaTime * deltaTime;
            }
            for (int i = 0; i < inputLinkIndex; i++)
            {
                links[i].AngleNumerical = links[i].AngleLast + links[i].Velocity * deltaTime + 0.5 * links[i].Acceleration * deltaTime * deltaTime;

                //while (links[i].AngleNumerical > Math.PI) links[i].AngleNumerical -= Constants.FullCircle;
                //while (links[i].AngleNumerical < -Math.PI) links[i].AngleNumerical += Constants.FullCircle;
            }
        }

        /// <summary>
        /// Numericals the velocity.
        /// </summary>
        /// <param name="deltaTime">The delta time.</param>
        /// <param name="joints">The joints.</param>
        /// <param name="links">The links.</param>
        private void NumericalVelocity(double deltaTime, List<Joint> joints, List<Link> links)
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

        /// <summary>
        /// Numericals the acceleration.
        /// </summary>
        /// <param name="deltaTime">The delta time.</param>
        /// <param name="joints">The joints.</param>
        /// <param name="links">The links.</param>
        private void NumericalAcceleration(double deltaTime, List<Joint> joints, List<Link> links)
        {
            for (int i = 0; i < firstInputJointIndex; i++)
            {
                joints[i].ax = (joints[i].vx - joints[i].vxLast) / deltaTime;
                joints[i].ay = (joints[i].vy - joints[i].vyLast) / deltaTime;
            }
            for (int i = 0; i < inputLinkIndex; i++)
                links[i].Acceleration = (links[i].Velocity - links[i].VelocityLast) / deltaTime;
        }

        /// <summary>
        /// Calculates a scaler for forces.
        /// </summary>
        private void CalculateForceScaler()
        {
            double maxLinkLength = 0;
            for (int l = 0; l < NumLinks; l++)
            {

                if (this.Links[l].lengths.Count == 0 ? false : this.Links[l].MaxLength > maxLinkLength && !this.Links[l].IsGround)
                {
                    maxLinkLength = Links[l].MaxLength;
                }
            }
            double maxForceMag = 0;
            double curForceMag = 0;
            for (int f = 0; f < NumForces; f++)
            {
                curForceMag = Math.Sqrt(Math.Pow(Forces[f].xmag, 2) + Math.Pow(Forces[f].ymag, 2));
                if (curForceMag > maxForceMag)
                {
                    maxForceMag = curForceMag;
                }
            }
            forceScaler = maxLinkLength / maxForceMag;
            if (forceScaler > 1.0)
            {
                forceScaler = 1.0;
            }
        }


        #region Retrieve State Variable Methods at a particular time

        /// <summary>
        /// The last query time
        /// </summary>
        private double lastQueryTime, prevQueryTime, nextQueryTime, nextToPrevTime;
        /// <summary>
        /// The previous query index
        /// </summary>
        private int prevQueryIndex, nextQueryIndex;
        /// <summary>
        /// The tau
        /// </summary>
        protected double tau;
        /// <summary>
        /// Finds the joint position at time.
        /// </summary>
        /// <param name="queryTime">The query time.</param>
        /// <param name="JointIndex">Index of the joint. This is not the simulation index, but the designer specified index.</param>
        /// <returns>System.Double[].</returns>
        public double[] FindJointPositionAtTime(double queryTime, int JointIndex)
        {
            setTimeIndices(queryTime);
            if (Math.Abs(tau) < Constants.epsilonSame)
                return new[]          
                {
                JointParameters.Parameters[prevQueryIndex][JointIndex, 0],
                JointParameters.Parameters[prevQueryIndex][JointIndex, 1]     
                };

            return new[]
                        {
                            FindPositionatTime(tau, nextToPrevTime,
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 0],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 0],
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 2],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 2],
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 4],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 4]),
                            FindPositionatTime(tau, nextToPrevTime,
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 1],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 1],
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 3],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 3],
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 5],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 5])
                        };
        }

        /// <summary>
        /// Finds the position of the given force at a specific time relative to the given time.
        /// </summary>
        /// <param name="queryTime"> The query time. </param>
        /// <param name="ForceIndex"> Index of the force. </param>
        public double[] FindForcePositionAtTime(double queryTime, int ForceIndex)
        {
            int joint = oIOSJ[(int)ForceParameters.Parameters[prevQueryIndex][ForceIndex, 5]];
            int link = (int)ForceParameters.Parameters[prevQueryIndex][ForceIndex, 8];
            double forceAngle = ForceParameters.Parameters[prevQueryIndex][ForceIndex, 9];
            setTimeIndices(queryTime);
            double[] jointPos = FindJointPositionAtTime(queryTime, joint);
            if (Math.Abs(tau) < Constants.epsilonSame)
            {
                return new[] { ForceParameters.Parameters[prevQueryIndex][ForceIndex,0],
                              ForceParameters.Parameters[prevQueryIndex][ForceIndex,1]};
            }
            return new[] { jointPos[0] + ForceParameters.Parameters[prevQueryIndex][ForceIndex,6] * Math.Cos(FindLinkAngleAtTime(queryTime, link) + forceAngle),
                           jointPos[1] + ForceParameters.Parameters[prevQueryIndex][ForceIndex,6] * Math.Sin(FindLinkAngleAtTime(queryTime, link) + forceAngle)};
        }

        public double[] FindICPositionAtTime(double queryTime, int ICIndex)
        {
            setTimeIndices(queryTime);
            return new[] {JointParameters.SecICLoc[prevQueryIndex][ICIndex, 2],
                          JointParameters.SecICLoc[prevQueryIndex][ICIndex, 3]};
        }

        /// <summary>
        /// Finds the joint velocity at time.
        /// </summary>
        /// <param name="queryTime">The query time.</param>
        /// <param name="JointIndex">Index of the joint. This is not the simulation index, but the designer specified index.</param>
        /// <returns>System.Double[].</returns>
        public double[] FindJointVelocityAtTime(double queryTime, int JointIndex)
        {
            setTimeIndices(queryTime);
            if (Math.Abs(tau) < Constants.epsilonSame)
                return new[]          
                {
                JointParameters.Parameters[prevQueryIndex][JointIndex, 2],
                JointParameters.Parameters[prevQueryIndex][JointIndex, 3]     
                };

            return new[]
                        {
                            FindVelocityatTime(tau, nextToPrevTime,
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 2],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 2],
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 4],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 4]),
                            FindVelocityatTime(tau, nextToPrevTime,
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 3],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 3],
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 5],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 5])
                        };
        }

        /// <summary>
        /// Finds the joint acceleration at time.
        /// </summary>
        /// <param name="queryTime">The query time.</param>
        /// <param name="JointIndex">Index of the joint. This is not the simulation index, but the designer specified index.</param>
        /// <returns>System.Double[].</returns>
        public double[] FindJointAccelerationAtTime(double queryTime, int JointIndex)
        {
            setTimeIndices(queryTime);
            if (Math.Abs(tau) < Constants.epsilonSame)
                return new[]          
                {
                JointParameters.Parameters[prevQueryIndex][JointIndex, 4],
                JointParameters.Parameters[prevQueryIndex][JointIndex, 5]     
                };

            return new[]
                        {
                            FindAccelerationatTime(tau, nextToPrevTime,
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 4],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 4]),
                            FindAccelerationatTime(tau, nextToPrevTime,
                                                         JointParameters.Parameters[prevQueryIndex][JointIndex, 5],
                                                         JointParameters.Parameters[nextQueryIndex][JointIndex, 5])
                        };
        }


        /// <summary>
        /// Finds the link angle at time.
        /// </summary>
        /// <param name="queryTime">The query time.</param>
        /// <param name="LinkIndex">Index of the link.</param>
        /// <returns>System.Double.</returns>
        public double FindLinkAngleAtTime(double queryTime, int LinkIndex)
        {
            setTimeIndices(queryTime);
            if (Math.Abs(tau) < Constants.epsilonSame)
                return LinkParameters.Parameters[prevQueryIndex][LinkIndex, 0];
            double posPrevious = LinkParameters.Parameters[prevQueryIndex][LinkIndex, 0];
            double posNext = LinkParameters.Parameters[nextQueryIndex][LinkIndex, 0];
            double vPrevious = LinkParameters.Parameters[prevQueryIndex][LinkIndex, 1];
            double vNext = LinkParameters.Parameters[nextQueryIndex][LinkIndex, 1];
            double aPrevious = LinkParameters.Parameters[prevQueryIndex][LinkIndex, 2];
            double aNext = LinkParameters.Parameters[nextQueryIndex][LinkIndex, 2];

            while (posPrevious < 0) { posPrevious += Constants.FullCircle; }
            while (posNext < 0) { posNext += Constants.FullCircle; }
            while (posPrevious >= Constants.FullCircle) { posPrevious -= Constants.FullCircle; }
            while (posNext >= Constants.FullCircle) { posNext -= Constants.FullCircle; }

            var positiveVel = (FindVelocityatTime(tau, nextToPrevTime, vPrevious, vNext, aPrevious, aNext) >= 0);
            if (positiveVel && posPrevious > posNext && Math.Abs(posPrevious - posNext) > Math.PI)
            {
                posPrevious -= Constants.FullCircle;
            }
            while (!positiveVel && posPrevious < posNext && Math.Abs(posPrevious - posNext) > Math.PI)
            {
                posNext -= Constants.FullCircle;
            }
            return FindPositionatTime(tau, nextToPrevTime, posPrevious, posNext, vPrevious, vNext, aPrevious, aNext);
        }


        /// <summary>
        /// Finds the link velocity at time.
        /// </summary>
        /// <param name="queryTime">The query time.</param>
        /// <param name="LinkIndex">Index of the link.  This is not the simulation index, but the designer specified index.</param>
        /// <returns>System.Double.</returns>
        public double FindLinkVelocityAtTime(double queryTime, int LinkIndex)
        {
            setTimeIndices(queryTime);
            if (Math.Abs(tau) < Constants.epsilonSame)
                return LinkParameters.Parameters[prevQueryIndex][LinkIndex, 1];
            return FindVelocityatTime(tau, nextToPrevTime,
                LinkParameters.Parameters[prevQueryIndex][LinkIndex, 1],
                LinkParameters.Parameters[nextQueryIndex][LinkIndex, 1],
                LinkParameters.Parameters[prevQueryIndex][LinkIndex, 2],
                LinkParameters.Parameters[nextQueryIndex][LinkIndex, 2]);

        }

        /// <summary>
        /// Finds the link acceleration at time.
        /// </summary>
        /// <param name="queryTime">The query time.</param>
        /// <param name="LinkIndex">Index of the link.  This is not the simulation index, but the designer specified index.</param>
        /// <returns>System.Double.</returns>
        public double FindLinkAccelerationAtTime(double queryTime, int LinkIndex)
        {
            setTimeIndices(queryTime);
            if (Math.Abs(tau) < Constants.epsilonSame)
                return LinkParameters.Parameters[prevQueryIndex][LinkIndex, 2];
            return FindAccelerationatTime(tau, nextToPrevTime,
                LinkParameters.Parameters[prevQueryIndex][LinkIndex, 2],
                LinkParameters.Parameters[nextQueryIndex][LinkIndex, 2]);

        }

        /// <summary>
        /// Finds the force magnitude at a certain time.
        /// </summary>
        /// <param name="queryTime"> The query time. </param>
        /// <param name="ForceIndex"> Index of the force. </param>
        public double[] FindForceMagnitudeAtTime(double queryTime, int ForceIndex)
        {
            setTimeIndices(queryTime);
            int link = (int)ForceParameters.Parameters[prevQueryIndex][ForceIndex, 8];
            if (System.Convert.ToBoolean(ForceParameters.Parameters[prevQueryIndex][ForceIndex, 4]))
            {
                double mag = Math.Sqrt(Math.Pow(Forces[ForceIndex].xmag, 2) + Math.Pow(Forces[ForceIndex].ymag, 2));
                return new[] { forceScaler * mag * Math.Cos(FindLinkAngleAtTime(queryTime, link) + ForceParameters.Parameters[prevQueryIndex][ForceIndex, 7]),
                               forceScaler * mag * Math.Sin(FindLinkAngleAtTime(queryTime, link) + ForceParameters.Parameters[prevQueryIndex][ForceIndex, 7])};
            }
            else
            {

                return new[] { forceScaler * ForceParameters.Parameters[prevQueryIndex][ForceIndex,2],
                               forceScaler * ForceParameters.Parameters[prevQueryIndex][ForceIndex,3]};
            }
        }

        /// <summary>
        /// Finds the position of the end of the force arrow at a time to help draw the arrows.
        /// </summary>
        /// <param name="queryTime"> The query time. </param>
        /// <param name="ForceIndex"> Index of the force. </param>
        public double[] FindForceArrowBaseAtTime(double queryTime, int ForceIndex)
        {
            double[] forcePos = FindForcePositionAtTime(queryTime, ForceIndex);
            double[] forceMag = FindForceMagnitudeAtTime(queryTime, ForceIndex);
            setTimeIndices(queryTime);
            return new[] {forcePos[0] + forceMag[0],
                          forcePos[1] + forceMag[1]};
        }

        /// <summary>
        /// Finds the position of the left arrow at a time.
        /// </summary>
        /// <param name="queryTime"> The query time. </param>
        /// <param name="ForceIndex"> Index of the force. </param>
        public double[] FindArrowLeftAtTime(double queryTime, int ForceIndex)
        {
            int link = (int)ForceParameters.Parameters[prevQueryIndex][ForceIndex, 8];
            setTimeIndices(queryTime);
            if (System.Convert.ToBoolean(ForceParameters.Parameters[prevQueryIndex][ForceIndex, 4]))
            {
                return new[] {(2.0 * Math.Cos(FindLinkAngleAtTime(queryTime, link) + ForceParameters.Parameters[prevQueryIndex][ForceIndex, 7] + (3 * Math.PI / 4))),
                              (2.0 * Math.Sin(FindLinkAngleAtTime(queryTime, link) + ForceParameters.Parameters[prevQueryIndex][ForceIndex, 7] + (3 * Math.PI / 4)))};
            }
            else
            {
                double angle = Math.Atan2(ForceParameters.Parameters[prevQueryIndex][ForceIndex, 3], ForceParameters.Parameters[prevQueryIndex][ForceIndex, 2]);
                return new[] {(2.0 * Math.Cos(angle + (3 * Math.PI / 4))),
                              (2.0 * Math.Sin(angle + (3 * Math.PI / 4)))};
            }
        }

        /// <summary>
        /// Finds the position of the right arrow at a time.
        /// </summary>
        /// <param name="queryTime"> The query time. </param>
        /// <param name="ForceIndex"> Index of the force. </param>    
        public double[] FindArrowRightAtTime(double queryTime, int ForceIndex)
        {
            int link = (int)ForceParameters.Parameters[prevQueryIndex][ForceIndex, 8];
            setTimeIndices(queryTime);
            if (System.Convert.ToBoolean(ForceParameters.Parameters[prevQueryIndex][ForceIndex, 4]))
            {
                return new[] {(2.0 * Math.Cos(FindLinkAngleAtTime(queryTime, link) + ForceParameters.Parameters[prevQueryIndex][ForceIndex, 7] - (3 * Math.PI / 4))),
                              (2.0 * Math.Sin(FindLinkAngleAtTime(queryTime, link) + ForceParameters.Parameters[prevQueryIndex][ForceIndex, 7] - (3 * Math.PI / 4)))};
            }
            else
            {
                double angle = Math.Atan2(ForceParameters.Parameters[prevQueryIndex][ForceIndex, 3], ForceParameters.Parameters[prevQueryIndex][ForceIndex, 2]);
                return new[] {(2.0 * Math.Cos(angle - (3 * Math.PI / 4))),
                              (2.0 * Math.Sin(angle - (3 * Math.PI / 4)))};
            }

        }


        /// <summary>
        /// Initializes the query vars.
        /// </summary>
        private void InitializeQueryVars()
        {
            lastQueryTime = prevQueryTime = nextQueryTime = nextToPrevTime = 0.0;
            prevQueryIndex = nextQueryIndex = 0;
            tau = 0.0;
        }


        /// <summary>
        /// Sets the time indices.
        /// </summary>
        /// <param name="queryTime">The query time.</param>
        private void setTimeIndices(double queryTime)
        {
                while (queryTime < BeginTime) queryTime += Time_Span;
                while (queryTime > EndTime) queryTime -= Time_Span;

                if (queryTime == lastQueryTime)
                    return; /* you are at a same time step - no need to change static vars. */
                /* you are at a new time step */
                // if (queryTime > JointParameters.Times[0] + Time_Span) setTimeIndices(queryTime - Time_Span);         // if the time is more than the span, recurse with lower time
                if (queryTime >= prevQueryTime && (queryTime <= nextQueryTime ||
                    prevQueryIndex == LinkParameters.LastIndex))
                {
                    /* cool. You're still in the same time step. This means we just need to change prevDeltaTime and nextDeltaTime. */
                }
                else
                {
                    while (queryTime < prevQueryTime)
                    {
                        nextQueryIndex = prevQueryIndex;
                        prevQueryIndex--;
                        if (prevQueryIndex < 0)
                        {
                            prevQueryIndex = LinkParameters.LastIndex;
                            prevQueryTime = LinkParameters.Times[prevQueryIndex];
                            nextQueryTime = LinkParameters.Times[nextQueryIndex];
                            break;
                        }
                        prevQueryTime = LinkParameters.Times[prevQueryIndex];
                        nextQueryTime = LinkParameters.Times[nextQueryIndex];
                    }
                    while (queryTime >= nextQueryTime)
                    {
                        prevQueryIndex = nextQueryIndex;
                        nextQueryIndex++;
                        if (nextQueryIndex > LinkParameters.LastIndex)
                        {
                            nextQueryIndex = 0;
                            prevQueryTime = LinkParameters.Times[prevQueryIndex];
                            nextQueryTime = LinkParameters.Times[nextQueryIndex];
                            break;
                        }

                        prevQueryTime = LinkParameters.Times[prevQueryIndex];
                        nextQueryTime = LinkParameters.Times[nextQueryIndex];
                    }
                }
                tau = queryTime - prevQueryTime;
                nextToPrevTime = nextQueryTime - prevQueryTime;
                if (nextToPrevTime < 0) nextToPrevTime += Time_Span;
                lastQueryTime = queryTime;
            }

        /// <summary>
        /// Finds the positionat time.
        /// </summary>
        /// <param name="tau">The tau.</param>
        /// <param name="deltaTime">The delta time.</param>
        /// <param name="posPrevious">The position previous.</param>
        /// <param name="posNext">The position next.</param>
        /// <param name="vPrevious">The v previous.</param>
        /// <param name="vNext">The v next.</param>
        /// <param name="aPrevious">a previous.</param>
        /// <param name="aNext">a next.</param>
        /// <returns>System.Double.</returns>
        static double FindPositionatTime(double tau, double deltaTime, double posPrevious, double posNext, double vPrevious,
               double vNext, double aPrevious, double aNext)
        {
            var tauSquared = tau * tau;
            var tauCubed = tau * tauSquared;
            var tauToThe4th = tau * tauCubed;
            var tauToThe5th = tau * tauToThe4th;
            var deltaTimeSquared = deltaTime * deltaTime;
            var deltaTimeCubed = deltaTime * deltaTimeSquared;
            return posPrevious + vPrevious * tau + aPrevious * tauSquared / 2
                   + tauCubed * (10 * (posNext - posPrevious) / deltaTimeSquared - 6 * vPrevious / deltaTime - 4 * vNext / deltaTime
                   - 1.5 * aPrevious + aNext / 2) / deltaTime
                   + tauToThe4th * (15 * (posPrevious - posNext) / deltaTimeSquared + 8 * vPrevious / deltaTime + 7 * vNext / deltaTime
                   + 1.5 * aPrevious - aNext) / deltaTimeSquared
                   + tauToThe5th * (6 * (posPrevious - posNext) / deltaTimeSquared + 3 * (vPrevious + vNext) / deltaTime + 0.5 * (aPrevious - aNext))
                   / deltaTimeCubed;
        }

        /// <summary>
        /// Finds the velocity (either x or y) at any time value.
        /// </summary>
        /// <param name="tau">tau is the differnce between the arbitrary time value and the previous time value.</param>
        /// <param name="deltaTime">The delta time is the difference between the next time value and the previous.</param>
        /// <param name="vPrevious">previous x or y-velocity</param>
        /// <param name="vNext">next x or y-velocity</param>
        /// <param name="aPrevious">previous x or y-acceleration</param>
        /// <param name="aNext">next x or y-acceleration</param>
        /// <returns>System.Double.</returns>
        static double FindVelocityatTime(double tau, double deltaTime, double vPrevious, double vNext, double aPrevious, double aNext)
        {
            if (deltaTime == 0.0) return vPrevious;
            var tauSquared = tau * tau;
            var tauCubed = tau * tauSquared;
            var deltaTimeSquared = deltaTime * deltaTime;
            var deltaTimeCubed = deltaTime * deltaTimeSquared;
            return vPrevious
                   + (vPrevious - vNext) * (2 * tauCubed / deltaTimeCubed - 3 * tauSquared / deltaTimeSquared)
                   + aNext * (tauCubed / deltaTimeSquared - tauSquared / deltaTime)
                   + aPrevious * (tauCubed / deltaTimeSquared - 2 * tauSquared / deltaTime + tau);
        }


        /// <summary>
        /// Finds the acceleration (X or Y) at any time value.
        /// </summary>
        /// <param name="tau">tau is the differnce between the arbitrary time value and the previous time value.</param>
        /// <param name="deltaTime">The delta time is the difference between the next time value and the previous.</param>
        /// <param name="aPrevious">previous x or y-acceleration</param>
        /// <param name="aNext">next x or y-acceleration</param>
        /// <returns>System.Double.</returns>
        static double FindAccelerationatTime(double tau, double deltaTime, double aPrevious, double aNext)
        {
            if (deltaTime == 0.0) return aPrevious;
            return aPrevious + (aNext - aPrevious) * tau / deltaTime;
        }

        #endregion

    }
}

