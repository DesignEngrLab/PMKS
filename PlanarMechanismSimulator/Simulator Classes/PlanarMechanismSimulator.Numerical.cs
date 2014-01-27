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



        #region Retrieve State Variable Methods at a particular time

        private double lastQueryTime, prevQueryTime, nextQueryTime, nextToPrevTime;
        private int prevQueryIndex, nextQueryIndex;
        protected static double tau;

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


        public double FindLinkPositionAtTime(double queryTime, int LinkIndex)
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
            const double fullCircle = 2 * Math.PI;
            while (posPrevious < 0) { posPrevious += fullCircle; }
            while (posNext < 0) { posNext += fullCircle; }
            while (posPrevious >= fullCircle) { posPrevious -= fullCircle; }
            while (posNext >= fullCircle) { posNext -= fullCircle; }

            var positiveVel = (FindVelocityatTime(tau, nextToPrevTime, vPrevious, vNext, aPrevious, aNext) >= 0);
            if (positiveVel && posPrevious > posNext && Math.Abs(posPrevious - posNext) > Math.PI)
            {
                posPrevious -= fullCircle;
            }
            while (!positiveVel && posPrevious < posNext && Math.Abs(posPrevious - posNext) > Math.PI)
            {
                posNext -= fullCircle;
            }
            return FindPositionatTime(tau, nextToPrevTime, posPrevious, posNext, vPrevious, vNext, aPrevious, aNext);
        }


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

        public double FindLinkAccelerationAtTime(double queryTime, int LinkIndex)
        {
            setTimeIndices(queryTime);
            if (Math.Abs(tau) < Constants.epsilonSame)
                return LinkParameters.Parameters[prevQueryIndex][LinkIndex, 2];
            return FindAccelerationatTime(tau, nextToPrevTime,
                LinkParameters.Parameters[prevQueryIndex][LinkIndex, 2],
                LinkParameters.Parameters[nextQueryIndex][LinkIndex, 2]);
      
        }



        private void setTimeIndices(double queryTime)
        {
             if (queryTime == lastQueryTime) return; /* you are at a same time step - no need to change static vars. */
            /* you are at a new time step */
            if (queryTime >= prevQueryTime && queryTime <= nextQueryTime)
            {
                /* cool. You're still in the same time step. This means we just need to change prevDeltaTime and nextDeltaTime. */
            }
            else
            {
                while (queryTime < prevQueryTime)
                {
                    if (prevQueryIndex == 0)
                    {
                        if (!CompleteCycle) break;
                        queryTime += Time_Span;
                        nextQueryIndex = LinkParameters.LastIndex;
                        nextQueryTime = LinkParameters.Times[nextQueryIndex];
                        prevQueryIndex = LinkParameters.LastIndex - 1;
                        prevQueryTime = LinkParameters.Times[prevQueryIndex];
                    }
                    else
                    {
                        nextQueryIndex = prevQueryIndex;
                        nextQueryTime = prevQueryTime;
                        prevQueryIndex--;
                        prevQueryTime = LinkParameters.Times[prevQueryIndex];
                    }
                }
                while (queryTime > nextQueryTime)
                {
                    if (nextQueryIndex == LinkParameters.LastIndex)
                    {
                        if (!CompleteCycle) break;
                        queryTime += Time_Span;
                        nextQueryIndex = 1;
                        nextQueryTime = LinkParameters.Times[1];
                        prevQueryIndex = 0;
                        prevQueryTime = LinkParameters.Times[0];
                    }
                    else
                    {
                        prevQueryIndex = nextQueryIndex;
                        prevQueryTime = nextQueryTime;
                        nextQueryIndex++;
                        nextQueryTime = LinkParameters.Times[nextQueryIndex];
                    }
                }
            }
            tau = queryTime - prevQueryTime;
            nextToPrevTime = nextQueryTime - prevQueryTime;
            lastQueryTime = queryTime;
        }

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
        /// <returns></returns>
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
        /// <returns></returns>
        static double FindAccelerationatTime(double tau, double deltaTime, double aPrevious, double aNext)
        {
            if (deltaTime == 0.0) return aPrevious;
            return aPrevious + (aNext - aPrevious) * tau / deltaTime;
        }

        #endregion

    }
}

