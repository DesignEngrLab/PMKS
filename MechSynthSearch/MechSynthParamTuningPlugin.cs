using System;
using System.Collections.Generic;
using GraphSynth;
using GraphSynth.Representation;
using GraphSynth.Search;
using OptimizationToolbox;
using PlanarMechanismSimulator;
using SearchIO = GraphSynth.SearchIO;
using System.IO;
using StarMathLib;


namespace MechSynth
{
    public class MechSynthParamTuningPlugin : SearchProcess
    {
        public MechSynthParamTuningPlugin(ISettings settings) : base(settings) { }
        public override string text
        {
            get { return "MechSynth ParamTuning Plugin"; }
        }
        protected override void Run()
        {
            Random r = new Random(); //1);



            double[,] desiredPath ={{1.87,8},{2.93,8.46},{2.80,8.41},
                                       {1.99,8.06},{0.96,7.46},{0,6.71},{-0.77,5.93},{-1.3,5.26},{-1.60,4.81},{-1.65,4.75},{-1.25,5.33},{0,6.71}};
            double startAngle = 0;
            double endAngle = 2 * Math.PI;
            double iOmega = 2;
            double iAlpha = 0;
            MechSimulation sim = new MechSimulation();


            //Below is a relation for bounding box and also the first point 
            double bb_min, bb_max;

            bb_min = StarMath.Min(desiredPath);
            bb_max = StarMath.Max(desiredPath);

            //now that min and max are obtained - we will form a bounding box using these max and min values

            

            sim.Graph = seedGraph;
            //  designGraph testGraph = this.seedGraph;
            //   ev.c = new candidate(testGraph, 0);
            //  ev.c = this.seedGraph;

            //bounding box - trying to contain the solutions within a particular box
            BoundingBox bb = new BoundingBox(sim, bb_max-bb_min, bb_max-bb_min);
            GrashofCriteria cc = new GrashofCriteria(sim, 0);

            //adding a new objective function which can be taken by the optimization program
            var pathObjFun = new ComparePathWithDesired(seedCandidate, desiredPath, sim);


            //initializing the optimization program 
            var optMethod = new NelderMead();
            //var optMethod = new GradientBasedOptimization();

            optMethod.Add(new PowellMethod());
            //PowellsOpt.Add(new DSCPowell(0.00001, .5, 200));
            optMethod.Add(new GoldenSection(0.001, 1.0, 100));

            //adding simulation
            optMethod.Add(sim);

            //adding objective function to this optimization routine
            optMethod.Add(pathObjFun);

            //we are removing this since we do not have a merit function defined
            optMethod.Add(new squaredExteriorPenalty(optMethod, 2.0));
            optMethod.Add(bb);
            optMethod.Add(cc);

            // convergence 
            optMethod.Add(new MaxFnEvalsConvergence(10000));
            //optMethod.Add(new DeltaXConvergence(0.01));
            optMethod.Add(new ToKnownBestFConvergence(0.0, 0.1));

            //generating random x,y values
            double[] x0 = new double[6];
            for (int i = 0; i < x0.GetLength(0); i++) //since I am going to assign ground pivots as they are
                x0[i] =  10*r.NextDouble();


            //sim.calculate(x0);

            double[] xStar;
            double fStar = optMethod.Run(out xStar, x0);
           // double fStar = optMethod.Run(out xStar, 8);
            

            SearchIO.output("***Converged by" + optMethod.ConvergenceDeclaredByTypeString, 0);


        }
        protected void RunFullTPSquared()
        {
            Random r = new Random();

            double[,] desiredPath ={{1.87,8},{2.93,8.46},{2.80,8.41},
                                       {1.99,8.06},{0.96,7.46},{0,6.71},{-0.77,5.93},{-1.3,5.26},{-1.60,4.81},{-1.65,4.75},{-1.25,5.33},{0,6.71}};
            double startAngle = 0;
            double endAngle = 2 * Math.PI;
            double iOmega = 2;
            double iAlpha = 0;
            MechSimulation sim = new MechSimulation();
            BoundingBox bb = new BoundingBox(sim, 10, 10);
            GrashofCriteria cc = new GrashofCriteria(sim, 0);

            List<candidate> candidates = new List<candidate>();
            while (true) //notConverged())
            {
                // 1. Generate topologies - calling rulesets - this adds candidates to the candidates list.

                //2. Evaluate  & Param Tuning
                foreach (candidate c in candidates)
                {
                    if (double.IsNaN(c.f0))
                    {
                        sim.Graph = c.graph;
                        NelderMead NMOpt = new NelderMead();
                        NMOpt.Add(sim);
                        //gbu.Add(new GoldenSection(.001, 20));
                        //gbu.Add(new BFGSDirection());
                        NMOpt.Add(new MaxIterationsConvergence(100));
                        double[] x0 = new double[8];
                        for (int i = 0; i < x0.GetLength(0); i++) //since I am going to assign ground pivots as they are
                            x0[i] = r.NextDouble();

                        double[] xStar;
                        double fStar = NMOpt.Run(out xStar, x0);
                        //   double fStar = NMOpt.Run(out xStar,8);
                        c.f0 = fStar;
                    }
                }

                //3. Pruning
                // throw out topologies (candidates) that have bad/large values of f0.

                //4. Guide?

            }
            SearchIO.output("***Completed!***");
        }

    }


}
