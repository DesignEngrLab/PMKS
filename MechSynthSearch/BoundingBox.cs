using System;
using OptimizationToolbox;
using PlanarMechanismSimulator;


namespace MechSynth
{
    class BoundingBox : IInequality
    {
        MechSimulation sim;
        double maxWidth, maxHeight;


        //constructor
        public BoundingBox(MechSimulation ev, double maxWidth, double maxHeight)
        {
            this.sim = ev;
            this.maxHeight = maxHeight;
            this.maxWidth = maxWidth;
        }

        public double calculate(double[] x)
        {
            double minX = double.PositiveInfinity;
            double minY = double.PositiveInfinity;
            double maxX = double.NegativeInfinity;
            double maxY = double.NegativeInfinity;

            for (int i = 0; i < sim.PivotParameters.GetLength(0); i++)
            {
                for (int j = 0; j < sim.PivotParameters.GetLength(1); j++)
                {
                    if (sim.PivotParameters[i, j, 0] < minX) minX = sim.PivotParameters[i, j, 0];
                    if (sim.PivotParameters[i, j, 0] > maxX) maxX = sim.PivotParameters[i, j, 0];
                    if (sim.PivotParameters[i, j, 1] < minY) minY = sim.PivotParameters[i, j, 1];
                    if (sim.PivotParameters[i, j, 1] > maxY) maxY = sim.PivotParameters[i, j, 1];
                }
            }
            return Math.Max((maxX - minX - maxWidth), (maxY - minY - maxHeight));

        }
    }
}
