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
        public readonly List<joint> joints;
        /// <summary>
        /// 
        /// </summary>
        public readonly Boolean isGround;

        private readonly double offsetAngle;

        private List<point> referencePts;

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
            get
            {
                throw new NotImplementedException();
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
            referencePts=joints.Where(j =>j.jointType==JointTypes.R).Cast<point>().ToList();
            // next put P and RP that are not the slide components.
            referencePts.AddRange(joints.Where(j => (j.jointType==JointTypes.P || j.jointType==JointTypes.RP) && !j.LinkIsSlide(this)).Cast<point>());
            // new add point for the gear teeth interactions. note these will move around the gear, they will not be at the contact point for long.
            if (referencePts.Count==0)
            referencePts.AddRange(joints.Where(j => (j.jointType==JointTypes.G)).Select(j=>new point{X=j.X, Y=j.Y}));
            // next add the orthogonal points for the slides
            referencePts.AddRange(joints.Where(j => (j.jointType==JointTypes.P || j.jointType==JointTypes.RP) && j.LinkIsSlide(this)).Select(findOrthoPoint));
            var numLengths = 2 * (joints.Count - 2) + 1;
            lengths = new double[numLengths];
                    int lengthIndex = 0;
                    int linkIndex = 0;
                    while (lengthIndex < numLengths)
                    {
                        lengths[lengthIndex++] = Math.Sqrt(
                            (joints[linkIndex].X - joints[linkIndex + 1].X)
                            * (joints[linkIndex].X - joints[linkIndex + 1].X)
                            + (joints[linkIndex].Y - joints[linkIndex + 1].Y)
                            * (joints[linkIndex].Y - joints[linkIndex + 1].Y));
                        if (lengthIndex < numLengths)
                            lengths[lengthIndex++] = Math.Sqrt(
                                (joints[linkIndex].X - joints[linkIndex + 2].X)
                                * (joints[linkIndex].X - joints[linkIndex + 2].X)
                                + (joints[linkIndex].Y - joints[linkIndex + 2].Y)
                                * (joints[linkIndex].Y - joints[linkIndex + 2].Y));
                        linkIndex++;
                    }

        }

        private point findOrthoPoint(joint j)
        {
            throw new NotImplementedException();
        }
    }
    public class point
    {
        public double X { get; set; }
        public double Y { get; set; }
    }
    public class joint : point
    {
        public readonly Boolean isGround;
        public readonly JointTypes jointType;

        public link Link1 { get; internal set; }
        public link Link2 { get; internal set; }
        public double SlideAngle { get; set; }
        // how to plan for future cam shapes


        internal joint(bool IsGround, string pTypeStr, double[] currentJointPosition)
        {
            isGround = IsGround;
            JointTypes pType;
            if (Enum.TryParse(pTypeStr, true, out pType)) jointType = pType;
            else throw new Exception("Unable to cast joint type " + pTypeStr + " as a recognized JointType.");

            if (currentJointPosition != null)
            {
                if (jointType == JointTypes.P || jointType == JointTypes.RP)
                {
                    if (currentJointPosition.GetLength(0) == 0 || currentJointPosition.GetLength(0) == 2)
                        throw new Exception("No slide angle provided for " + pTypeStr + " joint.");
                    else SlideAngle = currentJointPosition[0];
                    if (currentJointPosition.GetLength(0) >= 3)
                    {
                        X = currentJointPosition[1];
                        Y = currentJointPosition[2];
                    }
                }
                else
                {
                    if (currentJointPosition.GetLength(0) >= 2)
                    {
                        X = currentJointPosition[0];
                        Y = currentJointPosition[1];
                    }
                }
            }
        }

        public Boolean LinkIsSlide(link link0)
        {
            return ((jointType == JointTypes.P || jointType == JointTypes.RP) && Link1 == link0);
        }
    }

    public enum JointTypes
    {
        R,
        P,
        RP,
        G
        // non-slip roll, like rack and pinion - although this challenges the 2 DOF nature of just gear teeth
        // cabling or belt or chain

    };
}