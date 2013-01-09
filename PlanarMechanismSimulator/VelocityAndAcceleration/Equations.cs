using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PlanarMechanismSimulator.VelocityAndAcceleration
{
    internal abstract class EquationBase
    {
        public int unkLength
        {
            protected get { return _unkLength; }
            set
            {
                _unkLength = value;
                coefficients = new double[_unkLength];
            }
        }

        protected double[] coefficients;
        private int _unkLength;

        /// <summary>
        /// Captures the unknown indicies.
        /// </summary>
        /// <param name="unknownObjects">The unknown objects.</param>
        internal abstract void CaptureUnknownIndicies(List<object> unknownObjects);
    }

    internal abstract class JointToJointEquation : EquationBase
    {
        protected readonly joint joint1;
        protected int joint1XIndex;
        protected int joint1YIndex;
        protected readonly joint joint2;
        protected int joint2XIndex;
        protected int joint2YIndex;
        protected readonly link link;
        protected int linkOmegaIndex;
        protected readonly bool joint1IsKnown;
        protected readonly bool joint2IsKnown;


        internal JointToJointEquation(joint joint1, joint joint2, link link, bool Joint1IsKnown, bool Joint2IsKnown)
        {
            this.joint1 = joint1;
            this.joint2 = joint2;
            this.link = link;
            this.joint1IsKnown = Joint1IsKnown;
            this.joint2IsKnown = Joint2IsKnown;
        }


        internal override void CaptureUnknownIndicies(List<object> unknownObjects)
        {
            if (joint1IsKnown) joint1XIndex = joint1YIndex = -1;
            if (joint2IsKnown) joint2XIndex = joint2YIndex = -1;
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (!joint1IsKnown && o == joint1)
                {
                    joint1XIndex = index;
                    joint1YIndex = index + 1;
                }
                else if (!joint2IsKnown && o == joint2)
                {
                    joint2XIndex = index;
                    joint2YIndex = index + 1;
                }
                else if (o == link) linkOmegaIndex = index;
                if (o is joint) index += 2;
                else index++;
            }
        }

        internal abstract double[] GetRow1Coefficients();
        internal abstract double[] GetRow2Coefficients();
        internal abstract double GetRow1Constant();
        internal abstract double GetRow2Constant();

    }

    internal abstract class LinkToLinkEquation : EquationBase
    {
        protected readonly link link1;
        protected int link1OmegaIndex;
        protected readonly link link2;
        protected int link2OmegaIndex;


        internal LinkToLinkEquation(link link1, link link2)
        {
            this.link1 = link1;
            this.link2 = link2;
        }

        internal override void CaptureUnknownIndicies(List<object> unknownObjects)
        {
            var index = 0;
            foreach (var o in unknownObjects)
            {
                if (o == link1) link1OmegaIndex = index;
                else if (o == link2) link2OmegaIndex = index;
                if (o is joint) index += 2;
                else index++;
            }
        }
    }
}