using System;
using System.Linq;
using System.Collections.Generic;

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

        private List<point> referencePts;
        private List<LinkPointType> pointTypes;


        /// <summary>
        /// Gets or sets the lengths between the joints. If there are more than 2 joints, then the 
        /// length is 2*(n-2) + 1. (1 for 2 joints; 3 for 3; 5 for 4; etc.)
        /// there are various triangulations that could occur, here we used: (0, 1), (0, 2), 
        /// (1, 2), (1, 3), (2, 3), (2, 4), (3, 4), ..., (n, n+1), (n, n+2)
        /// </summary>
        /// <value>
        /// The lengths.
        /// </value>
        public double[] lengths { get; set; }

        public string name { get; private set; }

        public double Angle
        {
            get { throw new NotImplementedException(); }
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
            // new add point for the gear teeth interactions. note these will move around the gear, they will not be at the contact point for long.
            var centerPt =new point { X = joints.Sum(j => j.X) / joints.Count, Y = joints.Sum(j => j.Y) / joints.Count };
            var numPins = joints.RemoveAll(j => referencePts.Contains(j));
            var newJoints = new List<joint>(referencePts.Cast<joint>());

            pointTypes = new List<LinkPointType>();
            for (int i = 0; i < numPins; i++) pointTypes.Add(LinkPointType.Pin);
            pointTypes.Add(LinkPointType.Center);
            referencePts.Add(centerPt);

            // add gear tooth points that spin with gear
            foreach (var j in joints.Where(j => (j.jointType == JointTypes.G)))
            {
                newJoints.Add(j);
                referencePts.Add(new point {X = j.X, Y = j.Y});
                pointTypes.Add(LinkPointType.GearTooth);
            }
            // next add the orthogonal points for the slides
            foreach (var j in joints.Where(j => (j.jointType == JointTypes.P || j.jointType == JointTypes.RP) && j.LinkIsSlide(this)))
            {
                newJoints.Add(j);
                referencePts.Add(findOrthoPoint(j));
                pointTypes.Add(LinkPointType.OrthoSlideReference);
            }
            joints = newJoints;

            var numLengths = 2*(joints.Count - 2) + 1;
            lengths = new double[numLengths];
            int lengthIndex = 0;
            int linkIndex = 0;
            while (lengthIndex < numLengths)
            {
                lengths[lengthIndex++] = Math.Sqrt(
                    (joints[linkIndex].X - joints[linkIndex + 1].X)
                    *(joints[linkIndex].X - joints[linkIndex + 1].X)
                    + (joints[linkIndex].Y - joints[linkIndex + 1].Y)
                    *(joints[linkIndex].Y - joints[linkIndex + 1].Y));
                if (lengthIndex < numLengths)
                    lengths[lengthIndex++] = Math.Sqrt(
                        (joints[linkIndex].X - joints[linkIndex + 2].X)
                        *(joints[linkIndex].X - joints[linkIndex + 2].X)
                        + (joints[linkIndex].Y - joints[linkIndex + 2].Y)
                        *(joints[linkIndex].Y - joints[linkIndex + 2].Y));
                linkIndex++;
            }

        }

        private point findOrthoPoint(joint j)
        {
            throw new NotImplementedException();
        }
    }
}