using System;
using System.Linq;
using System.Collections.Generic;

namespace PlanarMechanismSimulator
{
    public class link
    {
        /// <summary>
        /// 
        /// </summary>
        public List<joint> joints { get; private set; }

        private int numJoints;
        /// <summary>
        /// 
        /// </summary>
        public readonly Boolean isGround;

        internal KnownState AngleIsKnown;

        private Dictionary<int, double> lengths;
        //private Dictionary<int, point> orthoPoints;

        public double[] Lengths
        {
            get
            {
                var result = new double[numJoints * (numJoints - 1) / 2];
                int i = 0;
                foreach (var length in lengths.Values)
                    result[i++] = Math.Abs(length);
                while (i < result.GetLength(0))
                    result[i++] = Double.NaN;
                return result;
            }
        }
        public string name { get; private set; }

        public double AngleInitial { get; set; }
        public double Angle { get; set; }
        public double AngleLast { get; set; }
        public double AngleNumerical { get; set; }

        internal KnownState velocityKnown;

        public double Velocity { get; set; }
        public double VelocityLast { get; set; }
        public double Acceleration { get; set; }
        public point InstantCenter { get; set; }

        internal link(string name, List<joint> Joints, Boolean IsGround)
        {
            this.name = name;
            joints = Joints;
            isGround = IsGround;
        }

        private link() { }

        internal void DetermineLengthsAndReferences()
        {
            numJoints = joints.Count;
            var fixedJoints = joints.Where(j => j.FixedWithRespectTo(this)).ToList();
            if (fixedJoints.Count < 2 || fixedJoints.Count(j => j.isGround) > 1)
                Angle = AngleInitial =AngleNumerical=AngleLast= 0.0;
            else if (fixedJoints.Count(j => j.isGround) == 1)
            {
                var ground = fixedJoints.FirstOrDefault(j => j.isGround);
                var notGround = fixedJoints.FirstOrDefault(j => !j.isGround);
                Angle = AngleInitial = AngleNumerical = AngleLast = Constants.angle(ground, notGround);
            }
            else Angle = AngleInitial = AngleNumerical = AngleLast = Constants.angle(fixedJoints[0], fixedJoints[1]);
            lengths = new Dictionary<int, double>();
            //orthoPoints = new Dictionary<int, point>();
            for (int i = 0; i < joints.Count - 1; i++)
                for (int j = i + 1; j < joints.Count; j++)
                {
                    var iJoint = joints[i];
                    var jJoint = joints[j];
                    var key = numJoints * i + j;
                    double distance;
                    int signForDistance = 1;
                    if (iJoint.SlidingWithRespectTo(this) && !jJoint.SlidingWithRespectTo(this))
                    {
                        if (Constants.sameCloseZero(iJoint.InitSlideAngle, jJoint.InitSlideAngle))
                            distance = Constants.distance(iJoint.xInitial, iJoint.yInitial, jJoint.xInitial, jJoint.yInitial);
                        else distance = 0.0;
                    }
                    else if (iJoint.SlidingWithRespectTo(this))
                    {
                        Boolean belowLinePositiveSlope;
                        var orthoPt = findOrthoPoint(jJoint, iJoint, iJoint.InitSlideAngle, out  belowLinePositiveSlope);
                        if (belowLinePositiveSlope) signForDistance = -1;
                        distance = Constants.distance(orthoPt.x, orthoPt.y, jJoint.xInitial, jJoint.yInitial);
                    }
                    else if (jJoint.SlidingWithRespectTo(this))
                    {
                        Boolean belowLinePositiveSlope;
                        var orthoPt = findOrthoPoint(iJoint, jJoint, jJoint.InitSlideAngle, out  belowLinePositiveSlope);
                        if (belowLinePositiveSlope) signForDistance = -1;
                        distance = Constants.distance(orthoPt.x, orthoPt.y, iJoint.xInitial, iJoint.yInitial);
                    }
                    else distance = Constants.distance(iJoint.xInitial, iJoint.yInitial, jJoint.xInitial, jJoint.yInitial);
                    lengths.Add(key, signForDistance * distance);
                }
            foreach (var j in joints)
            {  /* this comes at the end s.t. the findOrthoPoint calls do not have to re-adjust */
                if (j.SlidingWithRespectTo(this)) j.InitSlideAngle -= AngleInitial;
                while (j.InitSlideAngle < -Math.PI / 2) j.InitSlideAngle += Math.PI;
                while (j.InitSlideAngle > Math.PI / 2) j.InitSlideAngle -= Math.PI;
            }
        }

        internal double lengthBetween(joint joint1, joint joint2)
        {
            return Math.Abs(signedLengthBetween(joint1, joint2));
        }

        internal double signedLengthBetween(joint joint1, joint joint2)
        {
            if (joint1 == joint2) return 0.0;
            var i = joints.IndexOf(joint1);
            var j = joints.IndexOf(joint2);

            if (i > j) return lengths[numJoints * j + i];
            return lengths[numJoints * i + j];
        }

        internal void setLength(joint joint1, joint joint2, double length)
        {
            if (joint1 == joint2)
                throw new Exception("link.setLength: cannot set the distance between joints because same joint provided as both joint1 and joint2.");
            var i = joints.IndexOf(joint1);
            var j = joints.IndexOf(joint2);

            if (i > j) lengths[numJoints * j + i] = length;
            else lengths[numJoints * i + j] = length;
        }

        private static point findOrthoPoint(joint p, joint lineRef, double lineAngle, out Boolean belowLinePositiveSlope)
        {
            if (Constants.sameCloseZero(lineAngle))
            {
                belowLinePositiveSlope = (p.yInitial < lineRef.yInitial);
                return new point(p.xInitial, lineRef.yInitial);
            }
            else if (Constants.sameCloseZero(Math.Abs(lineAngle), Math.PI / 2))
            {
                belowLinePositiveSlope = (p.xInitial > lineRef.xInitial);
                return new point(lineRef.xInitial, p.yInitial);
            }
            var slope = Math.Tan(lineAngle);
            var offset = (lineRef.yInitial - slope * lineRef.xInitial);
            var x = (p.xInitial + slope * (p.yInitial - offset)) / (slope * slope + 1);
            var y = slope * x + offset;
            belowLinePositiveSlope = (slope > 0) ? (p.yInitial - slope * p.xInitial) < offset :
                 (p.yInitial - slope * p.xInitial) > offset;

            return new point(x, y);
        }

        internal double angleOfBlockToJoint(joint blockjoint, joint referenceJoint)
        {
            // todo: this should be stored from constructor calls - at the beginning. It will also be used to set up constraints for the
            //       nondyadic solver.
            var result = blockjoint.SlideAngle - Constants.angle(blockjoint.xLast, blockjoint.yLast, referenceJoint.xLast, referenceJoint.yLast);
            while (result < 0) result += Math.PI;
            while (result > Math.PI) result -= Math.PI;
            return result;
        }

        internal link copy(List<joint> oldJoints, List<joint> newJoints)
        {
            return new link
                {
                    Angle = Angle,
                    AngleInitial = AngleInitial,
                    AngleLast = AngleLast,
                    AngleIsKnown = AngleIsKnown,
                    AngleNumerical = AngleNumerical,
                    joints = joints.Select(j => newJoints[oldJoints.IndexOf(j)]).ToList(),
                    numJoints = numJoints,
                    lengths = new Dictionary<int, double>(lengths),
                    name = name
                };
        }
    }
}