using OptimizationToolbox;

namespace PMKS.PositionSolving
{
    internal abstract class NonDyadicObjFunctionTerm : IObjectiveFunction
    {
        internal abstract void SetInitialJointPosition(int index, double x, double y);

    }
}
