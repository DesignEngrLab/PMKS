using System;
using System.Collections.Generic;
using PMKS;

namespace PMKS
{
    internal class gearData
    {
        internal readonly double radius1;
        internal readonly double radius2;
        internal readonly int gearTeethIndex;
        internal readonly int connectingRodIndex;
        internal readonly int gearCenter1Index;
        internal readonly int gearCenter2Index;
        internal readonly int gear1LinkIndex;
        internal readonly int gear2LinkIndex;

        public gearData(joint gearTeethJoint, int gearTeethIndex, joint gearCenter1, int gearCenter1Index, int gear1LinkIndex,
            joint gearCenter2, int gearCenter2Index, int gear2LinkIndex,
            int connectingRodIndex, double initialGearAngle)
        {
            this.gearTeethIndex = gearTeethIndex;
            this.gearCenter1Index = gearCenter1Index;
            this.gear1LinkIndex = gear1LinkIndex;
            this.gearCenter2Index = gearCenter2Index;
            this.gear2LinkIndex = gear2LinkIndex;
            this.connectingRodIndex = connectingRodIndex;
            var dx1 = gearCenter1.xInitial - gearTeethJoint.xInitial;
            var dy1 = gearCenter1.yInitial - gearTeethJoint.yInitial;
            var dx2 = gearCenter2.xInitial - gearTeethJoint.xInitial;
            var dy2 = gearCenter2.yInitial - gearTeethJoint.yInitial;
            radius1 = Constants.distance(gearTeethJoint, gearCenter1);
            radius2 = Constants.distance(gearTeethJoint, gearCenter2);
            gearCenter1.OffsetSlideAngle = initialGearAngle;

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

        internal bool IsGearSolvableRP_G_R(List<joint> joints, List<link> links)
        {
            return (links[gear1LinkIndex].AngleIsKnown == KnownState.Fully
                    && joints[gearCenter1Index].positionKnown == KnownState.Fully
                    && joints[gearCenter2Index].jointType == JointTypes.R
                    && joints[gearCenter2Index].positionKnown == KnownState.Fully);
        }
        internal bool IsGearSolvableR_G_RP(List<joint> joints, List<link> links)
        {
            return (links[gear2LinkIndex].AngleIsKnown == KnownState.Fully
                    && joints[gearCenter2Index].positionKnown == KnownState.Fully
                    && joints[gearCenter1Index].jointType == JointTypes.R
                    && joints[gearCenter1Index].positionKnown == KnownState.Fully);
        }
        internal bool IsGearSolvablePGR(List<joint> joints, List<link> links)
        {
            throw new NotImplementedException();
        }
        internal bool IsGearSolvableRGP(List<joint> joints, List<link> links)
        {
            throw new NotImplementedException();
        }

        internal point SolveGearPositionAndAnglesRPGR(List<joint> joints, List<link> links,
            out double angleChange)
        {
            var knownlink = links[gear1LinkIndex];
            var rKnownGear = this.radius1;
            var gearCenterKnown = joints[gearCenter1Index];
            var rUnkGear = this.radius2;
            var gearCenterUnknown = joints[gearCenter2Index];
            angleChange = findUnknownGearAngleChange(knownlink, rKnownGear, gearCenterKnown, rUnkGear, gearCenterUnknown);       
            return findGearTeethPoint(gearCenterKnown, rKnownGear, gearCenterUnknown, rUnkGear);
        }


        internal point SolveGearPositionAndAnglesRGRP(List<joint> joints, List<link> links,
            out double angleChange)
        {
            var knownlink = links[gear2LinkIndex];
            var rKnownGear = radius2;
            var gearCenterKnown = joints[gearCenter2Index];
            var rUnkGear = radius1;
            var gearCenterUnknown = joints[gearCenter1Index];  
            angleChange = findUnknownGearAngleChange(knownlink, rKnownGear, gearCenterKnown, rUnkGear, gearCenterUnknown);
            return findGearTeethPoint(gearCenterKnown, rKnownGear, gearCenterUnknown, rUnkGear);
        }

        private static double findUnknownGearAngleChange(link knownlink, double rKnownGear, joint gearCenterKnown, double rUnkGear, joint gearCenterUnknown)
        {
            return -(rKnownGear / rUnkGear) * (
                 (knownlink.Angle - knownlink.AngleLast)
                 + findAngleChangeBetweenJoints(gearCenterKnown, gearCenterUnknown));
        }      
        private static point findGearTeethPoint(joint center1, double rGear1, joint center2, double rGear2)
        {
            var x = center1.x + rGear1 * (center2.x - center1.x) / (rGear2 + rGear1);
            var y = center1.y + rGear1 * (center2.y - center1.y) / (rGear2 + rGear1);
            return new point(x,y);

        }

        internal static double findAngleChangeBetweenJoints(joint From, joint To)
        {
            return Constants.angle(From.x, From.y, To.x, To.y) -
                   Constants.angle(From.xLast, From.yLast, To.xLast, To.yLast);
        }



        internal void SolveGearPositionAndAnglesRGP(List<joint> joints, List<link> links)
        {
            throw new NotImplementedException();
        }

        internal void SolveGearPositionAndAnglesPGR(List<joint> joints, List<link> links)
        {
            throw new NotImplementedException();
        }
    }

}

