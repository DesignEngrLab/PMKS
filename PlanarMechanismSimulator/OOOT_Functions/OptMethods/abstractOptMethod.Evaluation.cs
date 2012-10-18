using System;
using System.Collections.Generic;
using System.Linq;
using StarMathLib;

namespace OptimizationToolbox
{
    internal abstract partial class abstractOptMethod
    {

        /// <summary>
        /// Gets or sets the number of active constraints.
        /// </summary>
        /// <value>The number of active constraints, m.</value>
        internal int m { get; set; }

        /// <summary>
        /// Gets or sets the number of equality constraints.
        /// </summary>
        /// <value>The number of equality constraints, p.</value>
        internal int p { get; set; }

        /// <summary>
        /// Gets or sets the number of inequality constraints.
        /// </summary>
        /// <value>The number of inequality constraints, q.</value>
        internal int q { get; set; }

        /// <summary>
        /// Gets the number of function evaluations. This is actually the max of
        /// all functions (objective functions, equalities and inequalities) from
        /// the optimization run.
        /// </summary>
        /// <value>The num evals.</value>
        internal long numEvals { get; private set; }
        internal List<IObjectiveFunction> f { get; private set; }
        internal List<IEquality> h { get; private set; }
        internal List<IInequality> g { get; private set; }
        internal List<IConstraint> active { get; private set; }
        internal IDependentAnalysis dependentAnalysis { get; private set; }
        private double[] lastDependentAnalysis;


        private void calc_dependent_Analysis(double[] point)
        {
            if (dependentAnalysis == null) return;
            dependentAnalysis.calculate(point);
            lastDependentAnalysis = (double[])point.Clone();
        }


        #region Calculate f, g, h helper functions
        internal double calculate(IOptFunction function, double[] point)
        {
            return function.calculate(point);
        }


        // the reason this function is internal but the remainder are not, is because
        // this is called from other classes. Most notably, the line search methods, 
        // and the initial sampling in SA to get a temperature.
        /// <summary>
        /// Calculates the value of f at the specified point (assuming single-objective).
        /// </summary>
        /// <param name="point">The point.</param>
        /// <param name="includeMeritPenalty">if set to <c>true</c> [include merit penalty].</param>
        /// <returns></returns>
        internal double calc_f(double[] point, Boolean includeMeritPenalty = false)
        {
            var penalty = ((g.Count + h.Count > 0) && (ConstraintsSolvedWithPenalties || includeMeritPenalty))
                ? meritFunction.calcPenalty(point) : 0.0;
            numEvals++;
            return calculate(f[0], point) + penalty;
        }


        /// <summary>
        /// Calculates the f vector (multi-objective) at the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <param name="includeMeritPenalty">if set to <c>true</c> [include merit penalty].</param>
        /// <returns></returns>
        internal double[] calc_f_vector(double[] point, Boolean includeMeritPenalty = false)
        { return f.Select(fi => calculate(fi, point)).ToArray(); }
        /// <summary>
        /// Calculates the h vector at the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns></returns>
        protected double[] calc_h_vector(double[] point)
        { return h.Select(h0 => calculate(h0, point)).ToArray(); }
        /// <summary>
        /// Calculates the g vector at the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns></returns>
        protected double[] calc_g_vector(double[] point)
        { return g.Select(g0 => calculate(g0, point)).ToArray(); }
        /// <summary>
        /// Calculates the active vector at the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns></returns>
        protected double[] calc_active_vector(double[] point)
        { return active.Select(a => calculate(a, point)).ToArray(); }

        /// <summary>
        /// Calculates the gradient of f vector at the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <param name="includeMeritPenalty">if set to <c>true</c> [include merit penalty].</param>
        /// <returns></returns>
        protected double[] calc_f_gradient(double[] point, Boolean includeMeritPenalty = false)
        {
            var grad = new double[n];
            for (var i = 0; i != n; i++)
                grad[i] = deriv_wrt_xi(f[0], point, i);
            if (!feasible(point) && (ConstraintsSolvedWithPenalties || includeMeritPenalty))
                return StarMath.add(grad, meritFunction.calcGradientOfPenalty(point));
            return grad;
        }




        /// <summary>
        ///Calculates the gradient of h vector at the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns></returns>
        protected double[,] calc_h_gradient(double[] point)
        {
            var result = new double[p, n];
            for (var i = 0; i != p; i++)
                for (var j = 0; j != n; j++)
                    result[i, j] = deriv_wrt_xi(h[i], point, j);
            return result;
        }

        /// <summary>
        /// Calculates the gradient of g vector at the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns></returns>
        protected double[,] calc_g_gradient(double[] point)
        {
            var result = new double[q, n];
            for (var i = 0; i != q; i++)
                for (var j = 0; j != n; j++)
                    result[i, j] = deriv_wrt_xi(g[i], point, j);
            return result;
        }

        /// <summary>
        /// Calculates the gradient of active vector at  the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns></returns>
        protected double[,] calc_active_gradient(double[] point)
        {
            var result = new double[m, n];
            for (var i = 0; i != m; i++)
                for (var j = 0; j != n; j++)
                    result[i, j] = deriv_wrt_xi(active[i], point, j);
            return result;
        }

        /// <summary>
        /// Calculates the gradient of h vector at the specified point
        /// at the specified indices.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <param name="Indices">The indices.</param>
        /// <returns></returns>
        protected double[,] calc_h_gradient(double[] point, List<int> Indices)
        {
            var size = Indices.Count;
            var result = new double[p, size];
            for (var i = 0; i != p; i++)
                for (var j = 0; j != size; j++)
                    result[i, j] = deriv_wrt_xi(h[i], point, Indices[j]);
            return result;
        }
        /// <summary>
        /// Calculates the gradient of g vector at the specified point
        /// at the specified indices.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <param name="Indices">The indices.</param>
        /// <returns></returns>
        protected double[,] calc_g_gradient(double[] point, List<int> Indices)
        {
            var size = Indices.Count;
            var result = new double[q, size];
            for (var i = 0; i != q; i++)
                for (var j = 0; j != size; j++)
                    result[i, j] = deriv_wrt_xi(g[i], point, Indices[j]);
            return result;
        }
        /// <summary>
        /// Calculates the gradient of active vector at the specified point
        /// at the specified indices.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <param name="Indices">The indices.</param>
        /// <returns></returns>
        protected double[,] calc_active_gradient(double[] point, List<int> Indices)
        {
            var size = Indices.Count;
            var result = new double[m, size];
            for (var i = 0; i != m; i++)
                for (var j = 0; j != size; j++)
                    result[i, j] = deriv_wrt_xi(active[i], point, Indices[j]);
            return result;
        }


        #endregion

        /// <summary>
        /// Determines if the specified point is feasible.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns></returns>
        internal Boolean feasible(double[] point)
        {
            if (h.Any(a => !feasible(a, point)))
                return false;

            if (g.Any(a => !feasible(a, point)))
                return false;
            return true;
        }

        /// <summary>
        /// Determines if the specified point is feasible
        /// for the inequality, c.
        /// </summary>
        /// <param name="c">The c.</param>
        /// <param name="point">The point.</param>
        /// <returns></returns>
        private bool feasible(IInequality c, double[] point)
        {
            return (calculate(c, point) <= 0);
        }
        /// <summary>
        /// Determines if the specified point is feasible
        /// for the equality, c.
        /// </summary>
        /// <param name="c">The c.</param>
        /// <param name="point">The point.</param>
        /// <returns></returns>
        private bool feasible(IEquality c, double[] point)
        {
            return (calculate(c, point) == 0);
        }
        /// <summary>
        /// Determines if the specified point is feasible
        /// for the constraint, c.
        /// </summary>
        /// <param name="c">The c.</param>
        /// <param name="point">The point.</param>
        /// <returns></returns>
        internal bool feasible(IConstraint c, double[] point)
        {
            if (c is IEquality)
                return feasible((IEquality)c, point);
            if (c is IInequality)
                return feasible((IInequality)c, point);
            throw new Exception("IConstraint is neither IInequality or IEquality?!?");
        }


        /// <summary>
        /// Calculates the derivative with respect to variable xi
        /// for the specified function.
        /// </summary>
        /// <param name="function">The function.</param>
        /// <param name="point">The point.</param>
        /// <param name="i">The index of the variable in x.</param>
        /// <returns></returns>
        internal double deriv_wrt_xi(IOptFunction function, double[] point, int i)
        {
                    return ((IDifferentiable)function).deriv_wrt_xi(point, i);
        }


        #region finite difference

        private double calcBack1(IOptFunction function, double stepSize, double[] point, int i)
        {
            var backStep = (double[])point.Clone();
            backStep[i] -= stepSize;
            return (calculate(function, point) - calculate(function, backStep)) / stepSize;
        }
        private double calcForward1(IOptFunction function, double stepSize, double[] point, int i)
        {
            var forStep = (double[])point.Clone();
            forStep[i] += stepSize;
            return (calculate(function, forStep) - calculate(function, point)) / stepSize;
        }
        private double calcCentral2(IOptFunction function, double stepSize, double[] point, int i)
        {
            var forStep = (double[])point.Clone();
            var backStep = (double[])point.Clone();
            forStep[i] += stepSize;
            backStep[i] -= stepSize;
            return (calculate(function, forStep) - calculate(function, backStep)) / (2 * stepSize);
        }



        private double calcBack2(IOptFunction function, double stepSize, double[] point, int i)
        {
            var backStep1 = (double[])point.Clone();
            backStep1[i] -= stepSize;

            var backStep2 = (double[])point.Clone();
            backStep2[i] -= 2 * stepSize;
            return (calculate(function, backStep2) - 4 * calculate(function, backStep1) + 3 * calculate(function, point))
                / (2 * stepSize);
        }

        private double calcForward2(IOptFunction function, double stepSize, double[] point, int i)
        {
            var forStep1 = (double[])point.Clone();
            forStep1[i] += stepSize;

            var forStep2 = (double[])point.Clone();
            forStep2[i] += 2 * stepSize;
            return (-3 * calculate(function, point) + 4 * calculate(function, forStep1) - calculate(function, forStep2))
                / (2 * stepSize);
        }

        private double calcCentral4(IOptFunction function, double stepSize, double[] point, int i)
        {
            var forStep1 = (double[])point.Clone();
            forStep1[i] += stepSize;
            var forStep2 = (double[])point.Clone();
            forStep2[i] += 2 * stepSize;
            var backStep1 = (double[])point.Clone();
            backStep1[i] -= stepSize;
            var backStep2 = (double[])point.Clone();
            backStep2[i] -= 2 * stepSize;
            return (calculate(function, backStep2) - 8 * calculate(function, backStep1)
                    + 8 * calculate(function, forStep1) - calculate(function, forStep2)) / (12 * stepSize);
        }

        #endregion
    }
}
