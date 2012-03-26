/*************************************************************************
 *     This file & class is part of the Object-Oriented Optimization
 *     Toolbox (or OOOT) Project
 *     Copyright 2010 Matthew Ira Campbell, PhD.
 *
 *     OOOT is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *  
 *     OOOT is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with OOOT.  If not, see <http://www.gnu.org/licenses/>.
 *     
 *     Please find further details and contact information on OOOT
 *     at http://ooot.codeplex.com/.
 *************************************************************************/
using System;
using System.Collections.Generic;
using OptimizationToolbox;
using StarMathLib;

namespace MechanismPositionExample
{
    class Program
    {
        private static void Main()
        {
            // stephsonIIapproach();
            //genericApproachStephenson();
            genericApproachDblButterfly();
        }

        #region old sample Stephenson II
        private static void stephsonIIapproach()
        {
            SearchIO.verbosity = 5;
            var optMethod = new NewtonMethod();
            #region define initial points
            var xInGnd = 0.0;
            var yInGnd = -6.65;
            var xCrank = 0.0;
            var yCrank = -2.751;
            var x34 = 1.3970; var y34 = 0.2370;
            var x35 = 2.7960; var y35 = -2.6260;
            var x46 = 8.1040; var y46 = 1.3580;
            var x56 = 5.5810; var y56 = 0.0;

            var xOutGnd = 10.1390; var yOutGnd = -5.9360;

            //distance calculation
            //var l16=Math.Sqrt((xInGnd-x61)^2+(y12-y61)^2);
            var lCrank = Math.Sqrt(Math.Pow(xInGnd - xCrank, 2) + Math.Pow(yInGnd - yCrank, 2));
            var l235 = Math.Sqrt(Math.Pow(xCrank - x35, 2) + Math.Pow(yCrank - y35, 2));
            var l35 = Math.Sqrt(Math.Pow(x56 - x35, 2) + Math.Pow(y56 - y35, 2));
            var l56 = Math.Sqrt(Math.Pow(x56 - xOutGnd, 2) + Math.Pow(y56 - yOutGnd, 2));
            var l23 = Math.Sqrt(Math.Pow(xCrank - x34, 2) + Math.Pow(yCrank - y34, 2));
            var l345 = Math.Sqrt(Math.Pow(x35 - x34, 2) + Math.Pow(y35 - y34, 2));
            var l34 = Math.Sqrt(Math.Pow(x34 - x46, 2) + Math.Pow(y34 - y46, 2));
            var l645 = Math.Sqrt(Math.Pow(x56 - x46, 2) + Math.Pow(y56 - y46, 2));
            var l46 = Math.Sqrt(Math.Pow(x46 - xOutGnd, 2) + Math.Pow(y46 - yOutGnd, 2));
            var objfun = new StephensonII(xOutGnd, yOutGnd, xCrank, yCrank, l235, l35, l56, l23, l345, l34, l645, l46);
            SearchIO.output("value before move: " + objfun.calculate(new[] { x34, y34, x35, y35, x46, y46, x56, y56 }));
            #endregion
            /* new crank position */
            var theta = 10.0;
            theta = (90 - theta) * Math.PI / 180;
            xCrank = xInGnd + lCrank * Math.Cos(theta);
            yCrank = yInGnd + lCrank * Math.Sin(theta);

            objfun = new StephensonII(xOutGnd, yOutGnd, xCrank, yCrank, l235, l35, l56, l23, l345, l34, l645, l46);
            SearchIO.output("value after move: " + objfun.calculate(new[] { x34, y34, x35, y35, x46, y46, x56, y56 }));

            optMethod.Add(objfun);
            /* At least one convergence method is required for NelderMead.
             * Since we know the optimal is 0 (@ {1, 1}) we can use the 
             * "ToKnownBestFConvergence" with a tolerance of 0.0001. */
            optMethod.Add(new MaxIterationsConvergence(3));
            optMethod.Add(new ToKnownBestFConvergence(0, 1e-8));
            optMethod.Add(new FixedOrGoldenSection(1e-8, 0));
            /* Let us start the search from a specific point. */
            double[] xInit = new[] { x34, y34, x35, y35, x46, y46, x56, y56 };
            double[] xStar;

            /* the next line is where the optimization actually occurs. 
             * Note that the Run command requires the "out double[]" as it's
             * first argument. In this way, the optmization can return a single double
             * equal to the value of the optimal objective function, and an optimizer, 
             * a vector of optimal decision variables. There are two other overloads
             * for the Run command. You can simply specify the out vector and nothing
             * else, or you can provide the out vector and the number of decision 
             * variables. */
            var fStar = optMethod.Run(out xStar, xInit);

            /* output is provided from the optimization. Since the optMethod is an object
             * we can probe it following the run to get at important data like how the 
             * process converged. */
            Console.WriteLine("Convergence Declared by " + optMethod.ConvergenceDeclaredByTypeString);
            Console.WriteLine("X* = " + StarMath.MakePrintString(xStar));
            Console.WriteLine("F* = " + fStar, 1);
            Console.WriteLine("NumEvals = " + optMethod.numEvals);
            /* Since there is no randomness in the process, you should get the following
             * response:
             * No inequalities specified.
             * Convergence Declared by ToKnownBestFConvergence
             * X* = {   1.079   ,  1.159    }
             * F* = 0.00772036716239199
             * NumEvals = 245
             */
            Console.ReadKey();
        }
        #endregion
        private static void genericApproachStephenson()
        {
            SearchIO.verbosity = 2;
            var optMethod = new NewtonMethod();
            //  var optMethod = new GradientBasedOptimization();
            var x = new[]
                        {
                            //intermediate pivots
                            1.3970, 0.2370, //pivot0
                            2.7960, -2.6260, //pivot1
                            8.1040, 1.3580, //pivot2
                            5.5810, 0.0 //pivot3
                        };
            var numVar = x.GetLength(0);
            var knowns = new[]
                             {
                                 0.0, -2.751, //crank
                                 0.0, -6.65, //input ground
                                 10.1390, -5.9360 //output ground
                             };
            var allPositions = new double[numVar + knowns.GetLength(0)];
            x.CopyTo(allPositions, 0);
            knowns.CopyTo(allPositions, numVar);
            var connections = new List<Tuple<int, int>>
                                  {
                                      new Tuple<int, int>(0, 1),
                                      new Tuple<int, int>(0, 2),
                                      new Tuple<int, int>(0, 4),
                                      new Tuple<int, int>(1, 3),
                                      new Tuple<int, int>(1, 4),
                                      new Tuple<int, int>(2, 3),
                                      new Tuple<int, int>(2, 6),
                                      new Tuple<int, int>(3, 6),
                                      new Tuple<int, int>(4, 5), //crank link
                                      new Tuple<int, int>(5, 6)
                                  };
            var objfun = new NonDyadicPositionFinder(4, allPositions, connections);


            //check calculation
            //SearchIO.output("init objfun = " + objfun.calculate(x));
            //var initGradient = new double[numVar];
            //var initHessian = new double[numVar, numVar];
            //for (int i = 0; i < 8; i++)
            //{
            //    initGradient[i] = objfun.deriv_wrt_xi(x, i);
            //    for (int j = 0; j < 8; j++)
            //        initHessian[i, j] = objfun.second_deriv_wrt_ij(x, i, j);
            //}
            //SearchIO.output("init grad = " + StarMath.MakePrintString(initGradient));
            //SearchIO.output("init H = " + StarMath.MakePrintString(initHessian));

            /* new crank position */
            var theta = 0.0;
            var lCrank = Math.Sqrt(Math.Pow(knowns[0] - knowns[2], 2) + Math.Pow(knowns[1] - knowns[3], 2));
            theta = (90 - theta) * Math.PI / 180;
            knowns[0] = knowns[2] + lCrank * Math.Cos(theta);
            knowns[1] = knowns[3] + lCrank * Math.Sin(theta);
            objfun.MoveCrank(knowns[0], knowns[1]);
            SearchIO.output("value after move: " + objfun.calculate(x));

            optMethod.Add(objfun);
            /*optMethod.Add(new FletcherReevesDirection()); */
            optMethod.Add(new MaxIterationsConvergence(300));
            optMethod.Add(new DeltaXConvergence(1e-4));
            var converge = new ToKnownBestFConvergence(0, 1e-10);
            optMethod.Add(converge);
            optMethod.Add(new FixedOrGoldenSection(1e-10, 0));
            double[] xStar;
            var r = new Random();
            var fStar = double.PositiveInfinity;
            do
            {
                for (int i = 0; i < x.GetLength(0); i++)
                    x[i] = r.NextDouble() / (r.NextDouble() * r.NextDouble());
                fStar = optMethod.Run(out xStar, x);
                SearchIO.output("fStar = " + fStar);
            } while (!optMethod.ConvergenceDeclaredBy.Contains(converge));

            Console.WriteLine("Convergence Declared by " + optMethod.ConvergenceDeclaredByTypeString);
            Console.WriteLine("theta " + theta);
            Console.WriteLine("X* = " + StarMath.MakePrintString(xStar));
            Console.WriteLine("F* = " + fStar, 1);
            Console.WriteLine("NumEvals = " + optMethod.numEvals);

            Console.ReadKey();
        }
        private static void genericApproachDblButterfly()
        {
            SearchIO.verbosity = 1;
            var optMethod = new NewtonMethod();
            //var optMethod = new GradientBasedOptimization();
            var x = new[]
                        {
                            //intermediate pivots
                            -4.5,6.5, //pivot0
                            -5.9,9.3, //pivot1
                            -5.0,4.0, //pivot2
                            0.0,4.0, //pivot3
                            -0.134,8.717,//pivot 4
                            5.268,10.782//pivot 5
                        };
            var numVar = x.GetLength(0);
            var knowns = new[]
                             {
                                 -9.24, 4.24, //crank
                                 -5.0, 0.0, //input ground
                                 0.0, 0.0, //output ground
                                 -2.5, 2.5//intermediate ground
                             };
            var allPositions = new double[numVar + knowns.GetLength(0)];
            x.CopyTo(allPositions, 0);
            knowns.CopyTo(allPositions, numVar);
            var connections = new List<Tuple<int, int>>
                                  {
                                      new Tuple<int, int>(0, 1),
                                      new Tuple<int, int>(0, 5),
                                      new Tuple<int, int>(0, 6),
                                      new Tuple<int, int>(0, 9),
                                      new Tuple<int, int>(1, 9),
                                      new Tuple<int, int>(1, 2),
                                      new Tuple<int, int>(1, 3),
                                      new Tuple<int, int>(2, 3),
                                      new Tuple<int, int>(2, 4),
                                      new Tuple<int, int>(3, 4),
                                      new Tuple<int, int>(3, 8),
                                      new Tuple<int, int>(4, 5),
                                      new Tuple<int, int>(5, 6),
                                      new Tuple<int, int>(6, 7),
                                      new Tuple<int, int>(7, 8),
                                      new Tuple<int, int>(8, 9) //input crank
                                  };
            var objfun = new NonDyadicPositionFinder(6, allPositions, connections);


            //check calculation
            SearchIO.output("init objfun = " + objfun.calculate(x));
            var initGradient = new double[numVar];
            var initHessian = new double[numVar, numVar];
            for (int i = 0; i < 12; i++)
            {
                initGradient[i] = objfun.deriv_wrt_xi(x, i);
                for (int j = 0; j < 12; j++)
                    initHessian[i, j] = objfun.second_deriv_wrt_ij(x, i, j);
            }
            SearchIO.output("init grad = " + StarMath.MakePrintString(initGradient));
            SearchIO.output("init H = " + StarMath.MakePrintString(initHessian));

            /* new crank position */
            var theta = 0.0;
            var lCrank = Math.Sqrt(Math.Pow(knowns[0] - knowns[2], 2) + Math.Pow(knowns[1] - knowns[3], 2));
            theta = (135 - theta) * Math.PI / 180;
            knowns[0] = knowns[2] + lCrank * Math.Cos(theta);
            knowns[1] = knowns[3] + lCrank * Math.Sin(theta);
            objfun.MoveCrank(knowns[0], knowns[1]);
            SearchIO.output("value after move: " + objfun.calculate(x));

            optMethod.Add(objfun);
            var converge = new ToKnownBestFConvergence(0, 1e-4);
            optMethod.Add(converge);
            //optMethod.Add(new FletcherReevesDirection());
            // optMethod.Add(new MaxFnEvalsConvergence(300));
            optMethod.Add(new MaxIterationsConvergence(30));
            optMethod.Add(new DeltaFConvergence(1e-2));
            //optMethod.Add(new FixedOrGoldenSection(1e-2, 0));
            optMethod.Add(new GoldenSection(1e-2, 0));
            double[] xStar;
            var r = new Random();
            var fStar = double.PositiveInfinity;
            long numFEvals = 0;
            do
            {
                numFEvals += optMethod.numEvals;
                optMethod.ResetFunctionEvaluationDatabase();
                for (int i = 0; i < x.GetLength(0); i++)
                    x[i] = 20 * r.NextDouble() - 10;
                fStar = optMethod.Run(out xStar, x);
                //SearchIO.output("fStar = " + fStar);
            } while (!optMethod.ConvergenceDeclaredBy.Contains(converge));

            Console.WriteLine("Convergence Declared by " + optMethod.ConvergenceDeclaredByTypeString);
            Console.WriteLine("X* = " + StarMath.MakePrintString(xStar));
            Console.WriteLine("F* = " + fStar, 1);
            Console.WriteLine("NumEvals = " + numFEvals);

            Console.ReadKey();
        }
    }
}