using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class PlanarMechanismSimulator : IDependentAnalysis
    {
        private void NumericalPosition()
        {
            if (timeRow >= PivotParameters.GetLength(1) - 1)
                throw new Exception("At last position already in Numerical Position.");
            for (int i = 0; i < p; i++)
            {
                {
                    PivotParameters[i, timeRow + 1, 0] = PivotParameters[i, timeRow, 2] * timeStep
                                                         + 0.5 * PivotParameters[i, timeRow, 4] * timeStep * timeStep
                                                         + PivotParameters[i, timeRow, 0];
                    PivotParameters[i, timeRow + 1, 1] = PivotParameters[i, timeRow, 3] * timeStep
                                                         + 0.5 * PivotParameters[i, timeRow, 5] * timeStep * timeStep
                                                         + PivotParameters[i, timeRow, 1];
                }

            }

        }

        private void NumericalVelocity(double lastTime)
        {
            double[,] closeParams;
            double closeTime = PivotParameters.Keys.Min(t => Math.Abs(lastTime - t));
            // problem lastTime should already be on list --> answer will always be zero!
            if (PivotParameters.Count <= 1 || closeTime > timeStep)
            {
                var smallStep = timeStep / 10;
                closeParams = FindNextPosition(PivotParameters[lastTime], smallStep);
            }
            else closeParams = PivotParameters.First(kvp => Math.Abs(kvp.Key - lastTime) == closeTime).Value;
            var currentParams = PivotParameters[lastTime];
            for (int i = 0; i < p; i++)
            {
                currentParams[i, 2] = (closeParams[i, 0] - currentParams[i, 0]) / timeStep;
                currentParams[i, 3] = (closeParams[i, 1] - currentParams[i, 1]) / timeStep;
            }
        }

        private void NumericalAccelerationNew(double lastTime)
        {
            double[,] closeParams;
            double closeTime = PivotParameters.Keys.Min(t => Math.Abs(lastTime - t));
            if (PivotParameters.Count <= 1 || closeTime > timeStep)
            {
                var smallStep = timeStep / 10;
                closeParams = FindNextPosition(PivotParameters[lastTime], smallStep);
            }
            else closeParams = PivotParameters.First(kvp => Math.Abs(kvp.Key - lastTime) == closeTime).Value;
            var currentParams = PivotParameters[lastTime];
            for (int i = 0; i < p; i++)
            {
                currentParams[i, 2] = (closeParams[i, 0] - currentParams[i, 0]) / timeStep;
                currentParams[i, 3] = (closeParams[i, 1] - currentParams[i, 1]) / timeStep;
            }
        }


        private void NumericalAcceleration(double lastTime)
        {
            if (PivotParameters.Count <= 1 || PivotParameters.Keys.Min(t => Math.Abs(lastTime - t)) > timeStep)
                throw new NotImplementedException();
            else
                for (int i = 0; i < p; i++)
                {
                    PivotParameters[i, timeRow, 4] = (PivotParameters[i, timeRow, 2] - PivotParameters[i, timeRow - 1, 2]) / timeStep;
                    PivotParameters[i, timeRow, 5] = (PivotParameters[i, timeRow, 3] - PivotParameters[i, timeRow - 1, 3]) / timeStep;

                }
        }
    }
}