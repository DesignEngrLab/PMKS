using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using OptimizationToolbox;
using StarMathLib;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        private const int numberOfTries = 100;
        private void FindFullMovementNonDyadic()
        {
            var ndPosFinder = new NonDyadicPositionFinder(links, joints, inputJointIndex, epsilon);
          
        }
    }
}

