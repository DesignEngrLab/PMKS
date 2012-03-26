using System;
using System.Collections.Generic;

namespace PlanarMechanismSimulator
{
    internal class link
    {
        internal int index { get; private set; }
        internal double length { get;  set; }
        internal List<pivot> Pivots;
        internal Boolean IsGround;

        internal link(int i, double d, Boolean isGnd)
        {
            index = i;
            length = d;
            IsGround = isGnd;
        }
    }
    internal class pivot
    {
        internal int index { get; private set; }
        internal Boolean IsGround { get; private set; }
        internal double X { get; set; }
        internal double Y { get; set; }
        internal List<link> Links = new List<link>();
        internal PivotTypes PivotType { get; private set; }


        internal pivot(int i, Boolean isGnd, string pTypeStr)
        {
            index = i;
            IsGround = isGnd;
            PivotTypes pType;
            if (Enum.TryParse(pTypeStr, true, out pType)) PivotType = pType;
            else throw new Exception("Unable to cast pivot type " + pTypeStr + " as a recognized PivotType.");
        }

        public bool newp { get; set; }
    }

    internal enum PivotTypes
    {
        R,
        PX,
        PY,
        RPX,
        RPY
    };
}