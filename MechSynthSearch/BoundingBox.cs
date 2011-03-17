using System;
using OptimizationToolbox;
using PlanarMechanismSimulator;


namespace MechSynth
{
    class BoundingBox : IInequality
    {
        MechSimulation ev;
        double maxWidth, maxHeight;


        //constructor
        public BoundingBox(MechSimulation ev, double maxWidth, double maxHeight)
        {
            this.ev = ev;
            this.maxHeight = maxHeight;
            this.maxWidth = maxWidth;
        }

        public double calculate(double[] x)
        {
            double minX = double.PositiveInfinity;
            double minY = double.PositiveInfinity;
            double maxX = double.NegativeInfinity;
            double maxY = double.NegativeInfinity;

            for (int i = 0; i < ev.PivotParameters.GetLength(0); i++)
            {
                for (int j = 0; j < ev.PivotParameters.GetLength(1); j++)
                {
                    if (ev.PivotParameters[i, j, 0] < minX) minX = ev.PivotParameters[i, j, 0];
                    if (ev.PivotParameters[i, j, 0] > maxX) maxX = ev.PivotParameters[i, j, 0];
                    if (ev.PivotParameters[i, j, 1] < minY) minY = ev.PivotParameters[i, j, 1];
                    if (ev.PivotParameters[i, j, 1] > maxY) maxY = ev.PivotParameters[i, j, 1];
                }
            }
            return Math.Max((maxX - minX - maxWidth), (maxY - minY - maxHeight));

        }
    }
}
