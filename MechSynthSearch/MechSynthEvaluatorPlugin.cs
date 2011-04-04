using System;
using GraphSynth;
using GraphSynth.Representation;
using GraphSynth.Search;
using PlanarMechanismSimulator;

namespace MechSynth
{
    public class MechSynthEvaluatorPlugin : SearchProcess
    {
        public MechSynthEvaluatorPlugin(ISettings settings)
            : base(settings)
        {
            this.AutoPlay = true;
            this.RequiredNumRuleSets = 0;
        }
        public override string text
        {
            get { return "MechSynth Evaluator Plugin"; }
        }
        protected override void Run()
        {
            /* When the bolded "Run Search Process" is clicked from the
             * pulldown window, this function is initiated. It can be removed 
             * so that any of the included functions can be called directly, or
             * other code can fill this entire file. --> YOUR RESEARCH GOES HERE.
             */
            //  designGraph testGraph = designGraph.openGraphFromXml(settings.inputDirectory + "c1.gxml");]
            designGraph testGraph = this.seedGraph;
            double[,] desiredPath ={{1.87,8},{2.93,8.46},{2.80,8.41},
                                       {1.99,8.06},{0.96,7.46},{0,6.71},{-0.77,5.93},{-1.3,5.26},{-1.60,4.81},{-1.65,4.75},{-1.25,5.33},{0,6.71}};
            double startAngle = 0;
            double endAngle = 2 * Math.PI;
            double iOmega = 2;
            MechSimulation ev = new MechSimulation();
            ev.Graph = testGraph;

            ev.calculate(new[] { 0.0, 0.0, 6.0, 0.0, 1.874099, 7.998559, 1.73, 1 });
            //ev.saveParameterData(settings.outputDirectory + "outputPathC1");
            SearchIO.output("***Completed!***");
            //
            // c now needs values for the positions of the pivots unless of course it's set in the xml
            // otherwise you could set by calling calc(x).
            //
            //double[,] path = ev.returnPath();
            //
            // save double as csv or something (see Albert and Dagu).
            //
            //
            //runSearchProcessBFS();
            //runSearchProcessDFS();
            //runSearchProcessBESTFIRST();

            
        }


    }
}

