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

        /// <summary>
        /// 
        /// </summary>
        public readonly Boolean isGround;

        internal Boolean AngleIsUnknown;

        // internal List<point> referencePts;
        //  private List<LinkPointType> pointTypes;


        /// <summary>
        /// Gets or sets the lengths between the joints. If there are more than 2 joints, then the 
        /// number of lengths must be, at a minimum,  2*(n-2) + 1. (1 for 2 joints; 3 for 3; 5 for 4; etc.).
        /// This defines a lattice of triangles and is linear with n. Various triangulations could occur, 
        /// here we used: (0, 1), (0, 2), (1, 2), (1, 3), (2, 3), (2, 4), (3, 4), ..., (n, n+1), (n, n+2).
        /// HOWEVER, I become worried that there would be problems with multiple solutions involving internal
        /// reflections (imagine a quaternary plate folding on top of itself). And, it was easier
        /// to change this to the lengths of all points w.r.t. one another. This is n*(n-1)/2 (or order n^2
        /// instead of linear). Could this explode? Well, one rarely sees more than quaternary plates anyway.
        /// </summary>
        /// <value>
        /// The lengths.
        /// </value>
        public double[] lengths { get; private set; }

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
            var fixedJoints = joints.Where(j => j.FixedWithRespectToLink(this)).ToList();
            if (fixedJoints.Count < 2 || fixedJoints.Count(j => j.isGround) > 1) Angle = 0.0;
            else if (fixedJoints.Count(j => j.isGround) == 1)
            {
                var ground = fixedJoints.FirstOrDefault(j => j.isGround);
                var notGround = fixedJoints.FirstOrDefault(j => !j.isGround);
                Angle = Constants.angle(ground, notGround);
            }
            else Angle = Constants.angle(fixedJoints[0], fixedJoints[1]);

            //** see comments under lengths declaration for reason why this is commented. **
            //var numLengths = 2 * (joints.Count - 2) + 1;
            var numLengths = joints.Count * (joints.Count - 1) / 2;
            lengths = new double[numLengths];
            int lengthIndex = 0;
            for (int i = 0; i < joints.Count - 1; i++)
                for (int j = i + 1; j < joints.Count; j++)
                {
                    var iJoint = joints[i];
                    var jJoint = joints[j];
                    if (iJoint.SlidingWithRespectToLink(this) && jJoint.SlidingWithRespectToLink(this))
                        if (Constants.sameCloseZero(iJoint.SlideAngle, jJoint.SlideAngle))
                            lengths[lengthIndex++] = Constants.distance(findOrthoPoint(iJoint, jJoint), jJoint);
                        else lengths[lengthIndex++] = 0.0;
                    else if (iJoint.SlidingWithRespectToLink(this))
                        lengths[lengthIndex++] = Constants.distance(findOrthoPoint(iJoint, jJoint), jJoint);
                    else if (jJoint.SlidingWithRespectToLink(this))
                        lengths[lengthIndex++] = Constants.distance(findOrthoPoint(jJoint, iJoint), iJoint);
                    else lengths[lengthIndex++] = Constants.distance(iJoint, jJoint);
                }
            // ** see comments under lengths declaration for reason why this is commented.**
            //int linkIndex = 0;
            //while (lengthIndex < numLengths)
            //{
            //    lengths[lengthIndex++] = Math.Sqrt(
            //        (joints[linkIndex].X - joints[linkIndex + 1].X)
            //        * (joints[linkIndex].X - joints[linkIndex + 1].X)
            //        + (joints[linkIndex].Y - joints[linkIndex + 1].Y)
            //        * (joints[linkIndex].Y - joints[linkIndex + 1].Y));
            //    if (lengthIndex < numLengths)
            //        lengths[lengthIndex++] = Math.Sqrt(
            //            (joints[linkIndex].X - joints[linkIndex + 2].X)
            //            * (joints[linkIndex].X - joints[linkIndex + 2].X)
            //            + (joints[linkIndex].Y - joints[linkIndex + 2].Y)
            //            * (joints[linkIndex].Y - joints[linkIndex + 2].Y));
            //    linkIndex++;
            //}
            foreach (var j in joints)
            {  /* this comes at the end s.t. the findOrthoPoint calls do not have to re-adjust */
                if (j.SlidingWithRespectToLink(this)) j.SlideAngle -= Angle;
                while (j.SlideAngle < -Math.PI / 2) j.SlideAngle += Math.PI;
                while (j.SlideAngle > Math.PI / 2) j.SlideAngle -= Math.PI;
            }
        }

        internal static point findOrthoPoint(joint slideJoint, joint fixedJoint)
        {
            if (Constants.sameCloseZero(slideJoint.SlideAngle))
                return new point(fixedJoint.initX, slideJoint.initY);
            if (Constants.sameCloseZero(Math.Abs(slideJoint.SlideAngle), Math.PI / 2))
                return new point(slideJoint.initX, fixedJoint.initY);

            var slope = Math.Tan(slideJoint.SlideAngle);
            var x = (slope * slope * slideJoint.initX + fixedJoint.initX + slope * (fixedJoint.initX - slideJoint.initX)) /
                    (slope * slope + 1);
            var y = slope * x + (slideJoint.initY - slope * slideJoint.initX);
            return new point(x, y);
        }
        internal double lengthBetween(joint joint1, joint joint2)
        {
            return lengthBetween(joints.IndexOf(joint1), joints.IndexOf(joint2));
        }

        private double lengthBetween(int i, int j)
        {
            if (i == j) return 0.0;
            if (i > j)
            {
                var temp = i;
                i = j;
                j = temp;
            }
            var index = j - i - 1;
            for (int k = 0; k < i; k++)
                index += joints.Count - k - 1;
            return lengths[index];
        }
    }
}