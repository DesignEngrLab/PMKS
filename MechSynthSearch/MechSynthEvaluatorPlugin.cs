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

            /** reciprocating follower arc ***/
            //double[,] desiredPath ={{1.87,8},{2.93,8.46},{2.80,8.41},
            //                           {1.99,8.06},{0.96,7.46},{0,6.71},{-0.77,5.93},{-1.3,5.26},{-1.60,4.81},{-1.65,4.75},{-1.25,5.33},{0,6.71}};
          /** bean shape **/
            double[,] desiredPath = { { 158.65, 161.92 }, { 109.38, 135.30 }, { 57.997, 101.69 }, { 24.59, 82.07 }, { 0.33, 76.90 }, { -17.03, 91.46 },
                                    { -13.92, 129.10 }, { -0.74, 155.01 }, { 20.73, 180.91 }, { 53.78, 205.65 }, { 88.17, 219.90 },
                                    { 125, 225 }, { 165.44, 217.76 }, { 189.57, 200.42 }, { 185.89, 178.49 } };
          
            double startAngle = 0;
            double endAngle = 2 * Math.PI;
            double iOmega = 2;
            MechSimulation ev = new MechSimulation();
            ev.Graph = testGraph;

            ev.calculate(); //new[] { 0.0, 0.0, 6.0, 0.0, 1.874099, 7.998559, 1.73, 1 });
            var CPWD = new ComparePathWithDesired(seedCandidate, desiredPath, ev);
            CPWD.calculate(new double[1]);
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

