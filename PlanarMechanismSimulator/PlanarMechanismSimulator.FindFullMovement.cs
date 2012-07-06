using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        public void FindFullMovement()
        {
            if ((double.IsNaN(this.DeltaAngle)) && (double.IsNaN(this.FixedTimeStep)))
                throw new Exception(
                    "Either the angle delta or the time step must be specified.");

            #region Set up initial point parameters (x, x-dot, x-double-dot, etc.)
            SetUpDyadicVelocityObjects();
            JointParameters = new SortedList<double, double[,]>();
            LinkParameters = new SortedList<double, double[,]>();

            var initPivotParams = new double[numJoints, 6];
            for (int i = 0; i < numJoints; i++)
            {
                initPivotParams[i, 0] = joints[i].initX;
                initPivotParams[i, 1] = joints[i].initY;
            }
            var initLinkParams = new double[numLinks, 3];
            for (int i = 0; i < numLinks; i++)
                initLinkParams[i, 0] = links[i].Angle;
            InitializeGroundAndInputSpeedAndAcceleration(initPivotParams, initLinkParams);
            InputRange = new[] { inputLink.Angle, inputLink.Angle };
            JointParameters.Add(0.0, initPivotParams);
            LinkParameters.Add(0.0, initLinkParams);
            var lastForwardPivotParams = initPivotParams;
            var lastForwardLinkParams = initLinkParams;
            var lastBackwardPivotParams = initPivotParams;
            var lastBackwardLinkParams = initLinkParams;
            /* attempt to find velocities and accelerations at initial point analytically
             * there is no point in trying numerically as this is the first point and the numerical methods
             * perform finite difference of current and last time steps. */
            if (!(findVelocitiesThroughICMethod(0.0, true) && findAccelerationAnalytically(0.0, true)))
            {
                var smallBackwardStepJointParams = (double[,])initPivotParams.Clone();
                var dummyLinkParams = (double[,])initLinkParams.Clone();
                var delta = -0.01 * InputSpeed * FixedTimeStep;
                if (DefineNewPositions(delta, smallBackwardStepJointParams, dummyLinkParams, initPivotParams, initLinkParams))
                {
                    JointParameters.Add(-0.01 * FixedTimeStep, smallBackwardStepJointParams);
                    LinkParameters.Add(-0.01 * FixedTimeStep, dummyLinkParams);
                    InitializeGroundAndInputSpeedAndAcceleration(smallBackwardStepJointParams, dummyLinkParams);
                    NumericalVelocity(-0.01 * FixedTimeStep, initPivotParams, initLinkParams,
                        smallBackwardStepJointParams, dummyLinkParams);
                    NumericalAcceleration(-0.01 * FixedTimeStep, initPivotParams, initLinkParams,
                        smallBackwardStepJointParams, dummyLinkParams);
                    JointParameters.Remove(-0.01 * FixedTimeStep);
                    LinkParameters.Remove(-0.01 * FixedTimeStep);
                }
                else throw new Exception("Unable to establish micro-perturbation to initial position.");
            }
            #endregion

            Parallel.Invoke(
            #region *** Stepping Forward in Time ***
delegate
{
    var currentTime = 0.0;
    Boolean validPosition;
    do
    {
        var currentLinkParams = new double[numLinks, 3];
        var currentPivotParams = new double[numJoints, 6];
        #region Find Next Positions
        /* this time, NumericalPosition is called first and instead of
         * updating currentPivotParams (which is already full at this point)
         * we update the X, Y positions of the joints, which are global to the method. */
        NumericalPosition(FixedTimeStep, currentPivotParams, currentLinkParams,
            lastForwardPivotParams, lastForwardLinkParams);
        /* First, we analytically set the input pivot.*/
        var delta = InputSpeed * FixedTimeStep;
        /* Based upon the numerical approximation, we analytically update the remaining
         * joints. */
        validPosition = DefineNewPositions(delta, currentPivotParams, currentLinkParams,
            lastForwardPivotParams, lastForwardLinkParams);
        /* create new currentPivotParams based on these updated positions of the joints */
        #endregion
        if (validPosition)
        {
            lock (InputRange) { InputRange[0] = currentLinkParams[inputLinkIndex, 0]; }
            InitializeGroundAndInputSpeedAndAcceleration(currentPivotParams, currentLinkParams);

            #region Find Velocities for Current Position
            if (!findVelocitiesThroughICMethod(currentTime, true))
            {
                Status += "Instant Centers could not be found at" + currentTime + ".";
                NumericalVelocity(FixedTimeStep, currentPivotParams, currentLinkParams,
                    lastForwardPivotParams, lastForwardLinkParams);
            }
            #endregion
            #region Find Accelerations for Current Position
            if (!findAccelerationAnalytically(currentTime, true))
            {
                Status += "Analytical acceleration could not be found at" + currentTime + ".";
                NumericalAcceleration(FixedTimeStep, currentPivotParams, currentLinkParams,
                    lastForwardPivotParams, lastForwardLinkParams);
            }
            #endregion
            currentTime += FixedTimeStep;
            lock (JointParameters)
                JointParameters.Add(currentTime, currentPivotParams);
            lock (LinkParameters)
                LinkParameters.Add(currentTime, currentLinkParams);
            lastForwardPivotParams = currentPivotParams;
            lastForwardLinkParams = currentLinkParams;
        }
    } while (validPosition && lessThanFullRotation());
}
            #endregion
            #region *** Stepping Backward in Time ***
, delegate
 {
     var currentTime = 0.0;
     Boolean validPosition;
     do
     {
         var currentLinkParams = new double[numLinks, 3];
         var currentPivotParams = new double[numJoints, 6];
         #region Find Next Positions
         NumericalPosition(-FixedTimeStep, currentPivotParams, currentLinkParams,
             lastBackwardPivotParams, lastBackwardLinkParams);
         var delta = -InputSpeed * FixedTimeStep;
         validPosition = DefineNewPositions(delta, currentPivotParams, currentLinkParams,
             lastForwardPivotParams, lastForwardLinkParams);
         #endregion
         if (validPosition)
         {
             lock (InputRange) { InputRange[1] = currentLinkParams[inputLinkIndex, 0]; }
             InitializeGroundAndInputSpeedAndAcceleration(currentPivotParams, currentLinkParams);

             #region Find Velocities for Current Position
             if (!findVelocitiesThroughICMethod(currentTime, true))
             {
                 Status += "Instant Centers could not be found at" + currentTime + ".";
                 NumericalVelocity(-FixedTimeStep, currentPivotParams, currentLinkParams,
                     lastBackwardPivotParams, lastBackwardLinkParams);
             }
             #endregion
             #region Find Accelerations for Current Position
             if (!findAccelerationAnalytically(currentTime, true))
             {
                 Status += "Analytical acceleration could not be found at" + currentTime + ".";
                 NumericalAcceleration(-FixedTimeStep, currentPivotParams, currentLinkParams,
                     lastBackwardPivotParams, lastBackwardLinkParams);
             }
             #endregion
             currentTime -= FixedTimeStep; 
             lock (JointParameters)
                 JointParameters.Add(currentTime, currentPivotParams);
             lock (LinkParameters)
                 LinkParameters.Add(currentTime, currentLinkParams);
             lastBackwardPivotParams = currentPivotParams;
             lastBackwardLinkParams = currentLinkParams;
         }
     } while (validPosition && lessThanFullRotation());
            #endregion
 }
        );
        }


    }
}
