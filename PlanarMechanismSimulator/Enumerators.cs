// ***********************************************************************
// Assembly         : PlanarMechanismKinematicSimulator
// Author           : Matt
// Created          : 06-10-2015
//
// Last Modified By : Matt
// Last Modified On : 06-28-2015
// ***********************************************************************
// <copyright file="Enumerators.cs" company="">
//     Copyright ©  2014
// </copyright>
// <summary></summary>
// ***********************************************************************
namespace PMKS
{
    /// <summary>
    /// The mechanism will have a characteristic repeat cycle.
    /// </summary>
    public enum CycleTypes
    {
        /// <summary>
        /// Less than a full cycle
        /// </summary>
        LessThanFullCycle = -1,

        /// <summary>
        /// Exactly one cycle (one rotation) of the input drive
        /// </summary>
        OneCycle = 0,

        /// <summary>
        /// More than one cycle
        /// </summary>
        MoreThanOneCycle = 1
    }

    /// <summary>
    /// the accepted joint types used in this software
    /// </summary>
    public enum JointType
    {
        /// <summary>
        /// The unspecified
        /// </summary>
        unspecified,

        /// <summary>
        /// The 1 DOF revolute joint.
        /// </summary>
        R,

        /// <summary>
        /// The 1 DOF prismatic (sliding) joint.
        /// </summary>
        P,

        /// <summary>
        /// The pin-in-slot joint, RP. Essentially, an R joint on top of a P joint.
        /// This is a 2 DOF joint.
        /// </summary>
        RP,

        /// <summary>
        /// The gear joint, G, is a 2 DOF joint.
        /// </summary>
        G
        // non-slip roll, like rack and pinion - although this challenges the 2 DOF nature of just gear teeth
        // cabling or belt or chain
    }

    /// <summary>
    /// Enum KnownState
    /// </summary>
    internal enum KnownState
    {
        /// <summary>
        /// The unknown
        /// </summary>
        Unknown,
        /// <summary>
        /// The partially
        /// </summary>
        Partially,
        /// <summary>
        /// The fully
        /// </summary>
        Fully
    }


    /// <summary>
    /// Enum PositionAnalysisResults
    /// </summary>
    internal enum PositionAnalysisResults
    {
        /// <summary>
        /// The no solvable dyad found
        /// </summary>
        NoSolvableDyadFound,
        /// <summary>
        /// The normal
        /// </summary>
        Normal,
        /// <summary>
        /// The invalid position
        /// </summary>
        InvalidPosition,
        /// <summary>
        /// The branching probable
        /// </summary>
        BranchingProbable
    }

    internal enum JointSpecifiedAs
    {
        Design,
        TernaryJoint,
        FixedPointForLink,
        GearSupport
    }
}