#region

using System;
using System.Linq;
using System.Collections.Generic;
using OptimizationToolbox;
using System.Collections;

#endregion

namespace PlanarMechanismSimulator.VelocityAndAcceleration
//at time t=0; all acceleration and velocity are zero
{
    public class VelocitySolver
    {
        private readonly List<joint> joints;
        private readonly List<link> links;

        private readonly int inputLinkIndex;
        private readonly link inputLink;
        private readonly joint inputJoint;
        private readonly link groundLink;
        private readonly joint inputJoint;
        private readonly List<EquationBase> equations;

        public VelocitySolver(List<joint> joints, List<link> links, int inputLinkIndex, joint inputJoint)
        {
            equations = new List<EquationBase>();
            this.joints = joints;
            this.inputLinkIndex = inputLinkIndex;
            this.links = links;
            inputLink = links[links.Count - 2];
            groundLink = links[links.Count - 1];
            for (int k = 0; k < inputLinkIndex; k++)
                for (int i = 0; i < links[k].joints.Count - 1; i++)
                    for (int j = i + 1; j < links[k].joints.Count; j++)
                        equations.Add(new RelativeVelocityEquation(links[k].joints[i], links[k].joints[j], links[k]));
            
        }

    }
}
