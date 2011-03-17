using System;
using System.Collections.Generic;
using GraphSynth.Representation;
using System.Text;

namespace PlanarMechanismSimulator
{
    public class circleDiagramItem
    {
        public node link1 = null;
        public node link2 = null;
        public node pivot = null;
        public double x = double.NaN;
        public double y = double.NaN;
        public double speed = double.NaN;
        public double alpha = double.NaN;//angular acceleration
        //public double lVelocity = double.NaN;
        //public node otherNodes = null;//nodes/points of interest in a plate-like link!

    }
}
