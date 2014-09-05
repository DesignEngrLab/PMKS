using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;

namespace PMKS.PositionSolving
{
    internal abstract class NonDyadicObjFunctionTerm : IObjectiveFunction, IDifferentiable, ITwiceDifferentiable
    {
        internal abstract void SetInitialJointPosition(int index, double x, double y);

        public abstract double calculate(double[] x);

        public abstract double deriv_wrt_xi(double[] x, int i);
        public abstract double second_deriv_wrt_ij(double[] x, int i, int j);
    }
}
