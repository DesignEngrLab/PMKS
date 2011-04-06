using System;
using OptimizationToolbox;
using PlanarMechanismSimulator;
using StarMathLib;


namespace MechSynth
{
    class GrashofCriteria : IInequality
    {
        MechSimulation sim;
        int i;


        //constructor
        public GrashofCriteria(MechSimulation ev, int i)
        {
            this.sim = ev;
            this.i = i;
        }

        public double calculate(double[] x)
        {
            double maxlength, minlength,other1=0.0,other2=0.0;

            double[] l = new double[4];

            l[0] = Math.Sqrt(Math.Pow(6.0-0.0,2)+Math.Pow(0.0-0.0,2));
            l[1] = Math.Sqrt(Math.Pow(6.0 - x[0], 2) + Math.Pow(0.0 - x[1], 2));
            l[2]= Math.Sqrt(Math.Pow(0.0- x[2], 2) + Math.Pow(0.0 - x[3], 2));
            l[3] = Math.Sqrt(Math.Pow(x[2] - x[0], 2) + Math.Pow(x[3] - x[1], 2));

            maxlength = StarMath.Max(l);
            minlength = StarMath.Min(l);

            if (l[0] != maxlength || l[0] != minlength)
                other1 = l[0];
            if (l[1] != maxlength || l[1] != minlength)
                other1 = l[1];
            if (l[2] != maxlength || l[2] != minlength)
                other1 = l[2];
            if (l[3] != maxlength || l[3] != minlength)
                other1 = l[3];

            if (l[0] != maxlength || l[0] != minlength || l[0] != other1)
                other2 = l[0];
            if (l[1] != maxlength || l[1] != minlength || l[1] != other1)
                other2 = l[1];
            if (l[2] != maxlength || l[2] != minlength || l[2] != other1)
                other2 = l[2];
            if (l[3] != maxlength || l[3] != minlength || l[3] != other1)
                other2 = l[3];


            

            //for (int i = 0; i < sim.PivotParameters.GetLength(0); i++)
            //{
            //    for (int j = 0; j < sim.PivotParameters.GetLength(1); j++)
            //    {
            //        if (sim.PivotParameters[i, j, 0] < minX) minX = sim.PivotParameters[i, j, 0];
            //        if (sim.PivotParameters[i, j, 0] > maxX) maxX = sim.PivotParameters[i, j, 0];
            //        if (sim.PivotParameters[i, j, 1] < minY) minY = sim.PivotParameters[i, j, 1];
            //        if (sim.PivotParameters[i, j, 1] > maxY) maxY = sim.PivotParameters[i, j, 1];
            //    }
            //}
            return (maxlength+minlength-(other1+other2));

        }

    }
}
