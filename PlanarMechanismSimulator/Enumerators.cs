using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PMKS
{
    /// <summary>
    /// The mechanism will have a characteristic repeat cycle.
    /// </summary>
    public enum CycleTypes
    {
        LessThanFullCycle = -1,
        OneCycle = 0,
        MoreThanOneCycle = 1
    }

    /// <summary>
    /// the accepted joint types used in this software
    /// </summary>
    public enum JointType
    {
        unspecified,
        R,
        P,
        RP,
        G
        // non-slip roll, like rack and pinion - although this challenges the 2 DOF nature of just gear teeth
        // cabling or belt or chain
    };

    internal enum KnownState
    {
        Unknown,
        Partially,
        Fully
    };


    internal enum PositionAnalysisResults { NoSolvableDyadFound, Normal, InvalidPosition, BranchingProbable }
}
