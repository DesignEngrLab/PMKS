/*************************************************************************
 *     This file & class is part of the Object-Oriented Optimization
 *     Toolbox (or OOOT) Project
 *     Copyright 2010 Matthew Ira Campbell, PhD.
 *
 *     OOOT is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General internal License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *  
 *     OOOT is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General internal License for more details.
 *  
 *     You should have received a copy of the GNU General internal License
 *     along with OOOT.  If not, see <http://www.gnu.org/licenses/>.
 *     
 *     Please find further details and contact information on OOOT
 *     at http://ooot.codeplex.com/.
 *************************************************************************/

using System;
using System.Collections.Generic;

namespace OptimizationToolbox
{
    /// <summary>
    /// Given a value Kmax, this criteria will return true if the process reaches this many iterations.
    /// </summary>
    internal class MaxIterationsConvergence : abstractConvergence
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="MaxIterationsConvergence"/> class.
        /// </summary>
        internal MaxIterationsConvergence()
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="MaxIterationsConvergence"/> class.
        /// </summary>
        /// <param name="maxIterations">The max iterations.</param>
        internal MaxIterationsConvergence(long maxIterations)
        {
            this.maxIterations = maxIterations;
        }

        /// <summary>
        /// Gets or sets the maximum number of iterations.
        /// </summary>
        /// <value>The max iterations.</value>
        internal long maxIterations { get; set; }

        /// <summary>
        /// Given a value Kmax, this criteria will return true if the process reaches this many iterations.
        /// </summary>
        /// <param name="iteration">The number of iterations.</param>
        /// <param name="numFnEvals">The number of function evaluations (not used).</param>
        /// <param name="fBest">The best f (not used).</param>
        /// <param name="xBest">The best x (not used).</param>
        /// <param name="population">The population of candidates (not used).</param>
        /// <param name="gradF">The gradient of F (not used).</param>
        /// <returns>
        /// true or false - has the process converged?
        /// </returns>
        internal override bool converged(long iteration, long numFnEvals, double fBest = double.NaN, IList<double> xBest = null, IList<double[]> population = null, IList<double> gradF = null)
        {
            if (iteration < 0)
                throw new Exception(
                    "MaxIterationsConvergence expected a positive value for the first argument, YInteger");
            return (iteration >= maxIterations);
        }
    }
}