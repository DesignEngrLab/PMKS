using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;

namespace PMKS.PositionSolving
{
    internal interface ILinkFunction : IObjectiveFunction, IDifferentiable, ITwiceDifferentiable
    {
        void SetInitialJointPosition(int index, double x, double y);

    }
}
