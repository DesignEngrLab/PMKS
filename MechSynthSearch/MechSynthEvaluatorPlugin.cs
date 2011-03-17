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
            double[,] desiredPath ={{60,50},{54,44},{49,43},{52,47},{54,49},
                                       {56,51},{57,53},{58,55},{59,57},{61,55}};
            double startAngle = 0;
            double endAngle = 2 * Math.PI;
            double iOmega = 2;
            MechSimulation ev = new MechSimulation(startAngle, endAngle, iOmega);
            ev.C = new candidate(testGraph, 0);
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

