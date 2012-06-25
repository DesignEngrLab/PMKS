using System;
using System.Linq;
using System.Collections.Generic;
using StarMathLib;

namespace PlanarMechanismSimulator
{
    internal enum LinkPointType
    {
        Pin,
        Center,
        GearTooth,
        OrthoSlideReference
    };

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

        private double offsetAngle;

        internal List<point> referencePts;
        private List<LinkPointType> pointTypes;


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
        public double[] lengths { get; set; }

        public string name { get; private set; }

        public double Angle
        {
            get
            {
                return Math.Atan2(joints[1].Y - joints[0].Y, joints[1].X - joints[0].X) - offsetAngle;
            }
        }

        internal link(string name, List<joint> Joints, Boolean IsGround)
        {
            this.name = name;
            joints = Joints;
            isGround = IsGround;
        }

        internal void DetermineLengthsAndReferences()
        {
            // first put the pin joints on the list
            referencePts = joints.Where(j => j.jointType == JointTypes.R).Cast<point>().ToList();
            // next put P and RP that are not the slide components.
            referencePts.AddRange(
                joints.Where(j => (j.jointType == JointTypes.P || j.jointType == JointTypes.RP) && !j.LinkIsSlide(this)));
            // now add point for the gear teeth interactions. note these will move around the gear, they will not be at the contact point for long.
            var newJoints = new List<joint>(referencePts.Cast<joint>());
            var numPins = newJoints.Count;

            pointTypes = new List<LinkPointType>();
            for (int i = 0; i < numPins; i++) pointTypes.Add(LinkPointType.Pin);


            // next add the orthogonal points for the prismatic slides
            foreach (var j in joints.Where(j => j.jointType == JointTypes.P && j.LinkIsSlide(this)))
            {
                newJoints.Add(j);
                if (referencePts.Count > 0) referencePts.Add(findOrthoPoint(j, referencePts[0]));
                else referencePts.Add(new point { X = j.X, Y = j.Y }); //else, just make this the first reference point    
                pointTypes.Add(LinkPointType.OrthoSlideReference);
            }
            // add gear tooth points that spin with gear
            foreach (var j in joints.Where(j => j.jointType == JointTypes.G))
            {
                newJoints.Add(j);
                referencePts.Add(new point { X = j.X, Y = j.Y });
                pointTypes.Add(LinkPointType.GearTooth);
            }
            // next add the orthogonal points for the slides
            foreach (var j in joints.Where(j => j.jointType == JointTypes.RP && j.LinkIsSlide(this)))
            {
                newJoints.Add(j);
                if (referencePts.Count > 0) referencePts.Add(findOrthoPoint(j, referencePts[0]));
                else throw new Exception("Link, " + name + ", cannot be comprise of only of slide-side RP joints (intrinsically 2 or more DOF).");
                pointTypes.Add(LinkPointType.OrthoSlideReference);
            }
            joints = newJoints;
            offsetAngle = Math.Atan2(joints[1].Y - joints[0].Y, joints[1].X - joints[0].X);
            //** see comments under lengths declaration for reason why this is commented. **
            //var numLengths = 2 * (joints.Count - 2) + 1;
            var numLengths = referencePts.Count * (referencePts.Count - 1) / 2;
            lengths = new double[numLengths];
            int lengthIndex = 0;
            for (int i = 0; i < referencePts.Count - 1; i++)
                for (int j = i + 1; j < referencePts.Count; j++)
                {
                    lengths[lengthIndex++] = Math.Sqrt(
                        (referencePts[i].X - referencePts[j].X)
                        * (referencePts[i].X - referencePts[j].X)
                        + (referencePts[i].Y - referencePts[j].Y)
                        * (referencePts[i].Y - referencePts[j].Y));
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
        }

        private point findOrthoPoint(joint j, point point)
        {
            var piRemainder = j.SlideAngle % Math.PI;
            if (Constants.sameCloseZero(piRemainder))
                return new point { X = point.X, Y = j.Y };
            if (Constants.sameCloseZero(Math.Abs(piRemainder), Math.PI / 2))
                return new point { X = j.X, Y = point.Y };

            var slope = Math.Tan(j.SlideAngle);
            var x = (slope * slope * j.X + point.X + slope * (point.X - j.X)) / (slope * slope + 1);
            var y = slope * x + (j.Y - slope * j.X);
            return new point { X = x, Y = y };
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