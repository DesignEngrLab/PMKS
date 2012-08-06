using System;
using System.Linq;
using System.Collections.Generic;

namespace PlanarMechanismSimulator
{
    //internal enum LinkPointType
    //{
    //    Pin,
    //    Center,
    //    GearTooth,
    //    OrthoSlideReference
    //};

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

        internal Boolean AngleIsKnown;

        // internal List<point> referencePts;
        //  private List<LinkPointType> pointTypes;


        private Dictionary<int, double> lengths;

        public double[] Lengths
        {
            get
            {
                var result = new double[numJoints * (numJoints - 1) / 2];
                int i = 0;
                foreach (var length in lengths.Values)
                    result[i++] = length;
                while (i< result.GetLength(0))
                    result[i++] = double.NaN;
                return result;
            }
        }
        public string name { get; private set; }

        public double Angle { get; set; }

        internal link(string name, List<joint> Joints, Boolean IsGround)
        {
            this.name = name;
            joints = Joints;
            isGround = IsGround;
        }

        internal void DetermineLengthsAndReferences()
        {
            numJoints = joints.Count;
            var fixedJoints = joints.Where(j => j.FixedWithRespectTo(this)).ToList();
            if (fixedJoints.Count < 2 || fixedJoints.Count(j => j.isGround) > 1) Angle = 0.0;
            else if (fixedJoints.Count(j => j.isGround) == 1)
            {
                var ground = fixedJoints.FirstOrDefault(j => j.isGround);
                var notGround = fixedJoints.FirstOrDefault(j => !j.isGround);
                Angle = Constants.angle(ground, notGround);
            }
            else Angle = Constants.angle(fixedJoints[0], fixedJoints[1]);
            lengths = new Dictionary<int, double>();
            for (int i = 0; i < joints.Count - 1; i++)
                for (int j = i + 1; j < joints.Count; j++)
                {
                    var iJoint = joints[i];
                    var jJoint = joints[j];
                    var key = numJoints * i + j;
                    if (iJoint.SlidingWithRespectTo(this) && jJoint.SlidingWithRespectTo(this))
                        lengths.Add(key,
                                    Constants.sameCloseZero(iJoint.SlideAngle, jJoint.SlideAngle)
                                        ? Constants.distance(Constants.findOrthoPoint(iJoint, jJoint, jJoint.SlideAngle), iJoint)
                                        : 0.0);
                    else if (iJoint.SlidingWithRespectTo(this))
                        lengths.Add(key, Constants.distance(Constants.findOrthoPoint(jJoint, iJoint, iJoint.SlideAngle), jJoint));
                    else if (jJoint.SlidingWithRespectTo(this))
                        lengths.Add(key, Constants.distance(Constants.findOrthoPoint(iJoint, jJoint, jJoint.SlideAngle), iJoint));
                    else lengths.Add(key, Constants.distance(iJoint, jJoint));
                }
            foreach (var j in joints)
            {  /* this comes at the end s.t. the findOrthoPoint calls do not have to re-adjust */
                if (j.SlidingWithRespectTo(this)) j.SlideAngle -= Angle;
                while (j.SlideAngle < -Math.PI / 2) j.SlideAngle += Math.PI;
                while (j.SlideAngle > Math.PI / 2) j.SlideAngle -= Math.PI;
            }
        }

        internal double lengthBetween(joint joint1, joint joint2)
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

    }
}