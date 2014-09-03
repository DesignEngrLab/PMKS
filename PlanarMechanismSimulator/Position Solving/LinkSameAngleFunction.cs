using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OptimizationToolbox;

namespace PMKS.PositionSolving
{
    internal class LinkSameAngleFunction : ILinkFunction
    {
        private readonly int jointListIndexRef;
        private readonly int jointListIndexSlide;
        private readonly int varListIndexRef;
        private readonly int varListIndexSlide;
        private readonly double jointInitAngle;
        private readonly double linkInitAngle;

        private double xRef;
        private double yRef;
        private double xSlide;
        private double ySlide;


        public LinkSameAngleFunction(int varListIndexRef, int jointListIndexRef, double xRef, double yRef, int varListIndexSlide, int jointListIndexSlide, double xSlide, double ySlide,
              double jointInitAngle, double linkInitAngle)
        {
            this.jointListIndexRef = jointListIndexRef;
            this.jointListIndexSlide = jointListIndexSlide;
            this.varListIndexRef = varListIndexRef;
            this.varListIndexSlide = varListIndexSlide;

            this.xRef = xRef;
            this.yRef = yRef;
            this.xSlide = xSlide;
            this.ySlide = ySlide;
            this.jointInitAngle = jointInitAngle;
            this.linkInitAngle = linkInitAngle;
        }

        public double calculate(double[] x)
        {
            assignPositions(x);
            return double.NaN;
        }

        public double deriv_wrt_xi(double[] x, int i)
        {

            return double.NaN;
        }

        public double second_deriv_wrt_ij(double[] x, int i, int j)
        {
            return double.NaN;

        }

        private void assignPositions(double[] x)
        {
            if (varListIndexRef >= 0)// && x.GetLength(0) > 2 * varListIndex1 + 1)
            {
                xRef = x[2 * varListIndexRef];
                yRef = x[2 * varListIndexRef + 1];
            }
            if (varListIndexSlide >= 0)// && x.GetLength(0) > 2 * varListIndex2 + 1)
            {
                xSlide = x[2 * varListIndexSlide];
                ySlide = x[2 * varListIndexSlide + 1];
            }
        }

        public void SetInitialJointPosition(int index, double x, double y)
        {
            if (index == jointListIndexRef)
            {
                xRef = x;
                yRef = y;
            }
            if (index == jointListIndexSlide)
            {
                xSlide = x;
                ySlide = y;
            }
        }

    }
}