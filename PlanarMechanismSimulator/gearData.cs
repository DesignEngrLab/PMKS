﻿using System;
using System.Collections.Generic;
using StarMathLib;

namespace PMKS
{
    internal class GearData
    {
        internal readonly double radius1;
        internal readonly double radius2;
        internal readonly int gearTeethIndex;
        internal readonly int connectingRodIndex;
        internal readonly int gearCenter1Index;
        internal readonly int gearCenter2Index;
        internal readonly int gear1LinkIndex;
        internal readonly int gear2LinkIndex;

        internal GearData(Joint gearTeethJoint, int gearTeethIndex, Joint gearCenter1, int gearCenter1Index,
            int gear1LinkIndex,
            Joint gearCenter2, int gearCenter2Index, int gear2LinkIndex,
            int connectingRodIndex, double initialGearAngle)
        {
            this.gearTeethIndex = gearTeethIndex;
            this.gearCenter1Index = gearCenter1Index;
            this.gear1LinkIndex = gear1LinkIndex;
            this.gearCenter2Index = gearCenter2Index;
            this.gear2LinkIndex = gear2LinkIndex;
            this.connectingRodIndex = connectingRodIndex;
            var dx1 = gearCenter1.XInitial - gearTeethJoint.XInitial;
            var dy1 = gearCenter1.YInitial - gearTeethJoint.YInitial;
            var dx2 = gearCenter2.XInitial - gearTeethJoint.XInitial;
            var dy2 = gearCenter2.YInitial - gearTeethJoint.YInitial;
            radius1 = Constants.Distance(gearTeethJoint, gearCenter1);
            radius2 = Constants.Distance(gearTeethJoint, gearCenter2);
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

        internal bool IsGearSolvableRP_G_R(List<Joint> joints, List<Link> links)
        {
            return (links[gear1LinkIndex].AngleIsKnown == KnownState.Fully
                    && joints[gearCenter1Index].positionKnown == KnownState.Fully
                    && joints[gearCenter2Index].TypeOfJoint == JointType.R
                    && joints[gearCenter2Index].positionKnown == KnownState.Fully);
        }

        internal bool IsGearSolvableR_G_RP(List<Joint> joints, List<Link> links)
        {
            return (links[gear2LinkIndex].AngleIsKnown == KnownState.Fully
                    && joints[gearCenter2Index].positionKnown == KnownState.Fully
                    && joints[gearCenter1Index].TypeOfJoint == JointType.R
                    && joints[gearCenter1Index].positionKnown == KnownState.Fully);
        }

        internal Point SolveGearPositionAndAnglesRPGR(List<Joint> joints, List<Link> links,
            out double angleChange)
        {
            var knownlink = links[gear1LinkIndex];
            var rKnownGear = radius1;
            var gearCenterKnown = joints[gearCenter1Index];

            var unknownlink = links[gear2LinkIndex];
            var rUnkGear = radius2;
            var gearCenterUnknown = joints[gearCenter2Index];
            angleChange = findUnknownGearAngleChange(rKnownGear, gearCenterKnown, rUnkGear, gearCenterUnknown, knownlink,
                unknownlink);
            return findGearTeethPointAlongConnectingRod(gearCenterKnown, rKnownGear, gearCenterUnknown, rUnkGear);
        }


        internal Point SolveGearPositionAndAnglesRGRP(List<Joint> joints, List<Link> links,
            out double angleChange)
        {
            var knownlink = links[gear2LinkIndex];
            var rKnownGear = radius2;
            var gearCenterKnown = joints[gearCenter2Index];
            var rUnkGear = radius1;
            var gearCenterUnknown = joints[gearCenter1Index];
            angleChange = findUnknownGearAngleChange(rKnownGear, gearCenterKnown, rUnkGear, gearCenterUnknown, knownlink);
            return findGearTeethPointAlongConnectingRod(gearCenterKnown, rKnownGear, gearCenterUnknown, rUnkGear);
        }

        private static double findUnknownGearAngleChange(double rKnownGear, Joint gearCenterKnown, double rUnkGear,
            Joint gearCenterUnknown,
            Link knownlink, Link unknownlink = null)
        {
            var linkAngle = (knownlink.Angle - knownlink.AngleLast);
            linkAngle *= -(rKnownGear / rUnkGear) *
                         (linkAngle + findAngleChangeBetweenOfConnectingRod(gearCenterKnown, gearCenterUnknown));
            if (unknownlink == null) return linkAngle;
            var change = linkAngle - unknownlink.AngleNumerical;
            while (change > Math.PI)
            {
                linkAngle -= Constants.FullCircle;
                change = linkAngle - unknownlink.AngleNumerical;
            }
            while (change < -Math.PI)
            {
                linkAngle += Constants.FullCircle;
                change = linkAngle - unknownlink.AngleNumerical;
            }
            return linkAngle;
        }

        internal static Point findGearTeethPointAlongConnectingRod(Joint center1, double rGear1, Joint center2,
            double rGear2)
        {
            var x = center1.X + rGear1 * (center2.X - center1.X) / (rGear2 + rGear1);
            var y = center1.Y + rGear1 * (center2.Y - center1.Y) / (rGear2 + rGear1);
            return new Point(x, y);
        }

        internal static double findAngleChangeBetweenOfConnectingRod(Joint From, Joint To)
        {
            return Constants.Angle(From.X, From.Y, To.X, To.Y) -
                   Constants.Angle(From.XLast, From.YLast, To.XLast, To.YLast);
        }

        internal double FindNominalGearRotation(List<Link> links, Link knownlink)
        {
            var rKnownGear = radiusOfLink(links.IndexOf(knownlink));
            var rUnkGear = radiusOfOtherLink(links.IndexOf(knownlink));
            return -(rKnownGear / rUnkGear) * (knownlink.Angle - knownlink.AngleLast);

        }

        internal double radiusOfLink(int linkIndex)
        {
            if (linkIndex == gear1LinkIndex) return radius1;
            if (linkIndex == gear2LinkIndex) return radius2;
            return double.NaN;
        }

        internal double radiusOfOtherLink(int linkIndex)
        {
            if (linkIndex == gear1LinkIndex) return radius2;
            if (linkIndex == gear2LinkIndex) return radius1;
            return double.NaN;
        }

        internal int gearCenterIndex(int linkIndex)
        {
            if (linkIndex == gear1LinkIndex) return gearCenter1Index;
            if (linkIndex == gear2LinkIndex) return gearCenter2Index;
            return -1;
        }

        #region for R-R-G/G

        internal static Boolean FindKnownGearAngleOnLink(Joint gearCenter, Link connectingRod, Link gearLink, List<Joint> joints,
            List<Link> links, Dictionary<int, GearData> gearsData, out GearData gData)
        {
            if (gearLink.AngleIsKnown == KnownState.Fully)
            {
                foreach (var gearData in gearsData.Values)
                {
                    if (connectingRod == links[gearData.connectingRodIndex])
                    {
                        var otherGear = (gearLink == links[gearData.gear1LinkIndex])
                            ? links[gearData.gear2LinkIndex]
                            : links[gearData.gear1LinkIndex];
                        if (otherGear.AngleIsKnown == KnownState.Fully)
                        {
                            gData = gearData;
                            return true;
                        }
                    }
                }
            }
            gData = null;
            return false;
        }

        internal void SolveGearCenterFromKnownGearAngle(Joint gearCenter, Joint armPivot, List<Link> links, out double angleChange)
        {
            var angle1Change = links[gear1LinkIndex].Angle - links[gear1LinkIndex].AngleLast;
            var angle2Change = links[gear2LinkIndex].Angle - links[gear2LinkIndex].AngleLast;
            angleChange = (angle1Change + (radius2 / radius1) * angle2Change) / (1 + radius2 / radius1);
        }

        #endregion
        #region for R-G-R

        internal bool IsGearSolvableRGR(List<Joint> joints, List<Link> links)
        {
            return (joints[gearCenter1Index].positionKnown == KnownState.Fully &&
                joints[gearCenter2Index].positionKnown == KnownState.Fully);
        }

        internal void SolveGearPositionAndAnglesRGR(List<Joint> joints, List<Link> links)
        {
            var gearTeeth = joints[gearTeethIndex];
            var center1 = joints[gearCenter1Index];
            var center2 = joints[gearCenter2Index];
            gearTeeth.X = center1.X + radius1 * (center2.X - center1.X) / (radius2 + radius1);
            gearTeeth.Y = center1.Y + radius1 * (center2.Y - center1.Y) / (radius2 + radius1);

            gearTeeth.positionKnown = KnownState.Fully;
        }

        #endregion
        #region for R-G-P or P-G-R
        internal bool IsGearSolvableRGP(List<Joint> joints, List<Link> links)
        {
            return (joints[gearCenter2Index].positionKnown != KnownState.Unknown
                && links[gear2LinkIndex].AngleIsKnown == KnownState.Fully
                && joints[gearCenter1Index].positionKnown == KnownState.Fully);
        }
        internal void SolveGearPositionAndAnglesRGP(List<Joint> joints, List<Link> links)
        {
            var gearTeeth = joints[gearTeethIndex];
            var angle = links[gear2LinkIndex].Angle;
            var Pcenter = joints[gearCenter2Index];
            var Rcenter = joints[gearCenter1Index];
            SolveGearPositionAndAnglesPGR(gearTeeth, Pcenter, Rcenter, angle, radius1);
        }
        internal bool IsGearSolvablePGR(List<Joint> joints, List<Link> links)
        {
            return (joints[gearCenter1Index].positionKnown != KnownState.Unknown
                && links[gear1LinkIndex].AngleIsKnown == KnownState.Fully
                && joints[gearCenter2Index].positionKnown == KnownState.Fully);
        }
        internal void SolveGearPositionAndAnglesPGR(List<Joint> joints, List<Link> links)
        {
            var gearTeeth = joints[gearTeethIndex];
            var angle = links[gear1LinkIndex].Angle;
            var Pcenter = joints[gearCenter1Index];
            var Rcenter = joints[gearCenter2Index];
            SolveGearPositionAndAnglesPGR(gearTeeth, Pcenter, Rcenter, angle, radius2);
        }
        private void SolveGearPositionAndAnglesPGR(Joint gearTeeth, Joint Pcenter, Joint Rcenter, double angle, double gearRadius)
        {
            angle += Constants.QuarterCircle;
            var angleUnitVector = new[] { Math.Cos(angle), Math.Sin(angle) };
            var toSlideVector = new[] { (Pcenter.X - Rcenter.X), (Pcenter.Y - Rcenter.Y) };
            if (StarMath.dotProduct(angleUnitVector, toSlideVector) < 0)
            {
                angleUnitVector[0] = -angleUnitVector[0];
                angleUnitVector[1] = -angleUnitVector[1];
            }
            gearTeeth.X = Rcenter.X + gearRadius * angleUnitVector[0];
            gearTeeth.Y = Rcenter.Y + gearRadius * angleUnitVector[1];
            gearTeeth.positionKnown = KnownState.Fully;
        }




        #endregion



        /// <summary>
        /// Sets the gear rotation only from setLinkPositionFromRotate
        /// </summary>
        /// <param name="knownGearLink">The known gear link.</param>
        /// <param name="unknownGearLink">The unknown gear link.</param>
        /// <param name="links">The links.</param>
        /// <param name="joints">The joints.</param>
        internal Boolean SetGearRotation(Link knownGearLink, Link unknownGearLink, List<Link> links, List<Joint> joints)
        {
            if (unknownGearLink.AngleIsKnown == KnownState.Fully) return false;
            var rKnownGear = radiusOfLink(links.IndexOf(knownGearLink));
            var rUnkGear = radiusOfOtherLink(links.IndexOf(knownGearLink));
            var gearAngleChange = -(rKnownGear / rUnkGear) * (knownGearLink.Angle - knownGearLink.AngleLast);

            if (joints[gearCenter1Index].positionKnown == KnownState.Fully &&
                joints[gearCenter2Index].positionKnown == KnownState.Fully)
            {
                var From = joints[gearCenter1Index];
                var To = joints[gearCenter2Index];
                var connectingRodAngleChange = Constants.Angle(From.X, From.Y, To.X, To.Y) -
                                               Constants.Angle(From.XLast, From.YLast, To.XLast, To.YLast);
                unknownGearLink.Angle += connectingRodAngleChange * (1 + rKnownGear / rUnkGear) +
                                         gearAngleChange;
                unknownGearLink.AngleIsKnown = KnownState.Fully;
                return true;
            }
            if (unknownGearLink.AngleIsKnown == KnownState.Unknown)
            {
                unknownGearLink.Angle = gearAngleChange + unknownGearLink.AngleLast;
                unknownGearLink.AngleIsKnown = KnownState.Partially;
                return false;
            }
            var angleTemp = gearAngleChange + unknownGearLink.AngleLast;
            unknownGearLink.Angle = (unknownGearLink.Angle + angleTemp) / 2.0;
            unknownGearLink.AngleIsKnown = KnownState.Fully;
            return true;
        }

    }
}

