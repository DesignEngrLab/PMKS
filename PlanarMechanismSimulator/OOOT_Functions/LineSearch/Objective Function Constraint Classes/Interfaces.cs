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

namespace OptimizationToolbox
{


    internal abstract class IOptFunction
    {
        internal abstract double calculate(double[] x);
        internal abstract double deriv_wrt_xi(double[] x, int i);
        internal abstract double second_deriv_wrt_ij(double[] x, int i, int j);
    }
    internal abstract class IObjectiveFunction : IOptFunction { }
    internal abstract class IConstraint : IOptFunction { }
    internal abstract class IEquality : IConstraint { }
    internal abstract class IInequality : IConstraint { }

    interface IDependentAnalysis
    {
        void calculate(double[] x);
    }
}
