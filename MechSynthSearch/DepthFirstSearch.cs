using System;
using System.Collections.Generic;
using GraphSynth.Representation;
using GraphSynth.Search;
using GraphSynth;

namespace MechSynth
{
    public class DepthFirstSearch : SearchProcess
    {
        public DepthFirstSearch(ISettings settings) : base(settings) { }
        public override string text
        {
            get { return "Depth First Search"; }
        }
        protected override void Run()
        {
            Stack<candidate> candidates = new Stack<candidate>();
            candidate current = null;
            Boolean found = false;
            candidates.Push(seedCandidate);

            while (!found && (candidates.Count != 0) && !SearchIO.terminateRequest)
            {
                current = candidates.Pop();
                SearchIO.iteration = current.recipe.Count;
                SearchIO.miscObject = candidates.Count;

                if (isCurrentTheGoal(current))
                {
                    found = true;
                }
                else
                {
                    int rsIndex = current.activeRuleSetIndex;
                    List<option> ruleChoices = rulesets[rsIndex].recognize(current.graph);

                    foreach (option opt in ruleChoices)
                    {
                        candidate child = current.copy();
                        transferLmappingToChild(child.graph, current.graph, opt.location);
                        opt.apply(child.graph, null);
                        child.addToRecipe(opt);
                        candidates.Push(child);
                    }
                }
            }
            SearchIO.addAndShowGraphWindow(current.graph, "cand1");
            Save(settings.outputDirectory+"\\DFScandidate.xml", current);
        }
        private Boolean isCurrentTheGoal(candidate m)
        {
            throw new NotImplementedException();
        }
    }
}

