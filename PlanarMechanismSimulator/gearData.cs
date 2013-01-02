using System;

namespace PlanarMechanismSimulator
{
    internal class gearData
    {
        internal readonly double radius1;
        internal readonly double radius2;
        internal readonly int connectingRodIndex;
        internal readonly int gearCenter1Index;
        internal readonly int gearCenter2Index;

        public gearData(joint j, int connectingRodIndex, joint gearCenter1,
                        int gearCenter1Index, joint gearCenter2, int gearCenter2Index)
        {
            this.connectingRodIndex = connectingRodIndex;
            this.gearCenter1Index = gearCenter1Index;
            this.gearCenter2Index = gearCenter2Index;
            var dx1 = gearCenter1.xInitial - j.xInitial;
            var dy1 = gearCenter1.yInitial - j.yInitial;
            var dx2 = gearCenter2.xInitial - j.xInitial;
            var dy2 = gearCenter2.yInitial - j.yInitial;
            if (gearCenter1.jointType == JointTypes.P)
            {
                radius1 = Constants.distance(j, gearCenter1);
                radius2 = Constants.distance(j, gearCenter2);
                gearCenter1.InitSlideAngle = j.InitSlideAngle = Math.Atan2(-dx2, dy2);
            }
            else
            {
                gearCenter1.InitSlideAngle = j.InitSlideAngle = double.NaN;
                if (dx1 != 0 && dx2 != 0)
                {
                    if (!Constants.sameCloseZero(dy1 / dx1, dy2 / dx2))
                        throw new Exception("Gear teeth between gears " + j.Link1.name + " and " + j.Link2.name
                                            + " do not pivot orthogonal to teeth.");
                }
                else if (dx1 != 0 || dx2 != 0)
                    throw new Exception("Gear teeth between gears " + j.Link1.name + " and " + j.Link2.name
                                        + " do not pivot orthogonal to teeth.");
                radius1 = Math.Sqrt(dx1 * dx1 + dy1 * dy1);
                radius2 = Math.Sqrt(dx2 * dx2 + dy2 * dy2);
                if (dx1 * dx2 >= 0 && dy1 * dy2 >= 0)
                {
                    //then they're both on the same side of the gear teeth
                    // and one is inverted (the bigger one to be precise).
                    if (radius1 > radius2) radius1 *= -1;
                    else radius2 *= -1;
                }
            }
        }
    }
}
