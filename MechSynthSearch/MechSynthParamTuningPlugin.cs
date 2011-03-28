using System;
using System.Collections.Generic;
using GraphSynth;
using GraphSynth.Representation;
using GraphSynth.Search;
using OptimizationToolbox;
using PlanarMechanismSimulator;
using SearchIO = GraphSynth.SearchIO;

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
            Random r = new Random();

            double[,] desiredPath ={{1.87,8},{2.93,8.46},{2.80,8.41},
                                       {1.99,8.06},{0.96,7.46},{0,6.71},{-0.77,5.93},{-1.3,5.26},{-1.60,4.81},{-1.65,4.75},{-1.25,5.33},{0,6.71}};
            double startAngle = 0;
            double endAngle = 2 * Math.PI;
            double iOmega = 2;
            double iAlpha = 0; 
            MechSimulation sim = new MechSimulation();
            sim.c = seedGraph;
          //  designGraph testGraph = this.seedGraph;
         //   ev.c = new candidate(testGraph, 0);
          //  ev.c = this.seedGraph;
            BoundingBox bb = new BoundingBox(sim, 200, 200);



            NelderMead NMOpt = new NelderMead();
            NMOpt.Add(sim);
            //gbu.Add(new GoldenSection(.001, 20));
            //gbu.Add(new BFGSDirection());
            NMOpt.Add(new MaxIterationsConvergence(400));

            double[] xStar;
            double fStar = NMOpt.Run(out xStar,10);

            SearchIO.output("***Completed!***");


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
            BoundingBox bb = new BoundingBox(sim, 200, 200);

            List<candidate> candidates = new List<candidate>();
            while (true) //notConverged())
            {
                // 1. Generate topologies - calling rulesets - this adds candidates to the candidates list.

                //2. Evaluate  & Param Tuning
                foreach (candidate c in candidates)
                {
                    if (double.IsNaN(c.f0))
                    {
                        sim.c = c;
                        NelderMead NMOpt = new NelderMead();
                        NMOpt.Add(sim);
                        //gbu.Add(new GoldenSection(.001, 20));
                        //gbu.Add(new BFGSDirection());
                        NMOpt.Add(new MaxIterationsConvergence(100));


                        double[] xStar;
                        double fStar = NMOpt.Run(out xStar,10);
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
