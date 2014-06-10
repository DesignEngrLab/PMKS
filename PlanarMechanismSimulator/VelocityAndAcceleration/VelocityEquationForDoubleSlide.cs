using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using PMKS;

namespace PMKS.VelocityAndAcceleration
{
    internal class VelocityEquationForDoubleSlide : VelocityJointToJoint
    {
        private int slide1SpeedIndex = -1;
        private int slide2SpeedIndex = -1;

        internal VelocityEquationForDoubleSlide(joint slide1Joint, joint slide2Joint, link link, bool slideJoint1IsKnown,
             bool slideJoint2Known, bool linkIsKnown)
            : base(slide1Joint, slide2Joint, link, slideJoint1IsKnown, slideJoint2Known, linkIsKnown) { }

        internal override double[] GetRow1Coefficients()
        {
            var coefficients = new double[unkLength];
            for (int i = 0; i < unkLength; i++)
            {
                if (i == joint1XIndex) coefficients[i] = -1;
                else if (i == joint2XIndex) coefficients[i] = 1;
                else if (i == linkIndex) coefficients[i] = (joint2.y - joint1.y);
                else if (i == slide1SpeedIndex) coefficients[i] = Math.Cos(joint1.SlideAngle);
                else if (i == slide2SpeedIndex) coefficients[i] = -Math.Cos(joint2.SlideAngle);
                else coefficients[i] = 0;
            }
            return coefficients;
        }
        internal override double[] GetRow2Coefficients()
        {
            var coefficients = new double[unkLength];
            for (int i = 0; i < unkLength; i++)
            {
                if (i == joint1YIndex) coefficients[i] = -1;
                else if (i == joint2YIndex) coefficients[i] = 1;
                else if (i == linkIndex) coefficients[i] = (joint1.x - joint2.x);
                else if (i == slide1SpeedIndex) coefficients[i] = Math.Sin(joint1.SlideAngle);
                else if (i == slide2SpeedIndex) coefficients[i] = -Math.Sin(joint2.SlideAngle);
                else coefficients[i] = 0;
            }
            return coefficients;
        }

        internal override void CaptureUnknownIndicies(List<object> unknownObjects)
        {
            base.CaptureUnknownIndicies(unknownObjects);
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o is Tuple<link, joint>)
                {
                    if (((Tuple<link, joint>)o).Item1 == link)
                    {
                        if (((Tuple<link, joint>)o).Item2 == joint1)
                            slide1SpeedIndex = index;
                        else if (((Tuple<link, joint>)o).Item2 == joint2)
                            slide2SpeedIndex = index;
                    }
                }
                if (o is joint) index += 2;
                else index++;
            }
        }
        internal override List<int> GetRow1Indices()
        {
            var indices = base.GetRow1Indices();
            indices.Add(slide1SpeedIndex);
            indices.Add(slide2SpeedIndex);
            return indices;
        }


        internal override List<int> GetRow2Indices()
        {
            var indices = base.GetRow2Indices();
            indices.Add(slide1SpeedIndex);
            indices.Add(slide2SpeedIndex);
            return indices;
        }
    }
}
