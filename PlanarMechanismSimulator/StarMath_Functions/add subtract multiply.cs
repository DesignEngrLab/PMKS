/*************************************************************************
 *     This file & class is part of the StarMath Project
 *     Copyright 2010, 2011 Matthew Ira Campbell, PhD.
 *
 *     StarMath is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General internal License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *  
 *     StarMath is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General internal License for more details.
 *  
 *     You should have received a copy of the GNU General internal License
 *     along with StarMath.  If not, see <http://www.gnu.org/licenses/>.
 *     
 *     Please find further details and contact information on StarMath
 *     at http://starmath.codeplex.com/.
 *************************************************************************/

using System;
using System.Collections.Generic;
using System.Linq;

namespace StarMathLib
{
    internal static partial class StarMath
    {
        /// <summary>
        /// Multiplies all elements of a 1D double array with the double value.
        /// </summary>
        /// <param name = "a">The double value to be multiplied</param>
        /// <param name = "B">The double vector to be multiplied with</param>
        /// <returns>A 1D double array that contains the product</returns>
        internal static double[] multiply(double a, IList<double> B)
        { return multiply(a, B, B.Count()); }
        /// <summary>
        /// Multiplies all elements of a 1D double array with the double value.
        /// </summary>
        /// <param name="a">The double value to be multiplied</param>
        /// <param name="B">The double vector to be multiplied with</param>
        /// <param name="length">The length of the vector B. This is an optional argument, but if it is already known 
        /// - there is a slight speed advantage to providing it.</param>
        /// <returns>
        /// A 1D double array that contains the product
        /// </returns>
        internal static double[] multiply(double a, IList<double> B, int length)
        {
            // scale vector B by the amount of scalar a
            var c = new double[length];
            for (var i = 0; i != length; i++)
                c[i] = a * B[i];
            return c;
        }
        /* Note: We do not need need a scalar integer multiplied by a double vector
         * because the scalar integer can be automatically cast to a double and use 
         * the two functions above. */

        /// <summary>
        /// Multiplies all elements of a 1D integer array with the double value.
        /// </summary>
        /// <param name = "a">The double value to be multiplied</param>
        /// <param name = "B">The integer vector to be multiplied with</param>
        /// <returns>A 1D double array that contains the product</returns>
        internal static double[] multiply(double a, IList<int> B)
        { return multiply(a, B, B.Count()); }
        /// <summary>
        /// Multiplies all elements of a 1D integer array with the double value.
        /// </summary>
        /// <param name="a">The double value to be multiplied</param>
        /// <param name="B">The integer vector to be multiplied with</param>
        /// <param name="length">The length of the vector B. This is an optional argument, but if it is already known 
        /// - there is a slight speed advantage to providing it.</param>
        /// <returns>
        /// A 1D double array that contains the product
        /// </returns>
        internal static double[] multiply(double a, IList<int> B, int length)
        {
            // scale vector B by the amount of scalar a
            var c = new double[length];
            for (var i = 0; i != length; i++)
                c[i] = a * B[i];
            return c;
        }
        /// <summary>
        /// Multiplies all elements of a 1D integer array with the integer value.
        /// </summary>
        /// <param name = "a">The integer value to be multiplied</param>
        /// <param name = "B">The integer vector to be multiplied with</param>
        /// <returns>A 1D integer array that contains the product</returns>
        internal static int[] multiply(int a, IList<int> B)
        { return multiply(a, B, B.Count()); }
        /// <summary>
        /// Multiplies all elements of a 1D integer array with the integer value.
        /// </summary>
        /// <param name="a">The integer value to be multiplied</param>
        /// <param name="B">The integer vector to be multiplied with</param>
        /// <param name="length">The length of the vector B. This is an optional argument, but if it is already known 
        /// - there is a slight speed advantage to providing it.</param>
        /// <returns>
        /// A 1D integer array that contains the product
        /// </returns>
        internal static int[] multiply(int a, IList<int> B, int length)
        {
            // scale vector B by the amount of scalar a
            var c = new int[length];
            for (var i = 0; i != length; i++)
                c[i] = a * B[i];
            return c;
        }

        /// <summary>
        /// Divides all elements of a 1D double array by the double value.
        /// </summary>
        /// <param name = "B">The vector to be divided</param>
        /// <param name = "a">The double value to be divided by, the divisor.</param>
        /// <returns>A 1D double array that contains the product</returns>
        internal static double[] divide(IList<double> B, double a)
        { return multiply((1 / a), B); }
        /// <summary>
        /// Divides all elements of a 1D double array by the double value.
        /// </summary>
        /// <param name="B">The vector to be divided</param>
        /// <param name="a">The double value to be divided by, the divisor.</param>
        /// <param name="length">The length of the vector B. This is an optional argument, but if it is already known 
        /// - there is a slight speed advantage to providing it.</param>
        /// <returns>
        /// A 1D double array that contains the product
        /// </returns>
        internal static double[] divide(IList<double> B, double a, int length)
        { return multiply((1 / a), B, length); }

        #region Dot-product of vectors to vectors
        /// <summary>
        /// The dot product of the two 1D double vectors A and B
        /// </summary>
        /// <param name = "A">1D double Array, A</param>
        /// <param name = "B">1D double Array, B</param>
        /// <returns>A double value that contains the dot product</returns>
        internal static double dotProduct(IList<double> A, IList<double> B)
        {
            var length = A.Count();
            if (length != B.Count())
                throw new Exception("Matrix sizes do not match");
            return dotProduct(A, B, length);
        }
        /// <summary>
        /// The dot product of the two 1D double vectors A and B
        /// </summary>
        /// <param name="A">1D double Array, A</param>
        /// <param name="B">1D double Array, B</param>
        /// <param name="length">The length of both vectors A and B. This is an optional argument, but if it is already known 
        /// - there is a slight speed advantage to providing it.</param>
        /// <returns>
        /// A double value that contains the dot product
        /// </returns>
        internal static double dotProduct(IList<double> A, IList<double> B, int length)
        {
            var c = 0.0;
            for (var i = 0; i != length; i++)
                c += A[i] * B[i];
            return c;
        }
        #endregion

        
        /// <summary>
        /// The cross product of two double vectors, A and B, which are of length, 2.
        /// In actuality, there is no cross-product for 2D. This is shorthand for 2D systems 
        /// that are really simplifications of 3D. The returned scalar is actually the value in 
        /// the third (read: z) direction.
        /// </summary>
        /// <param name = "A">1D double Array, A</param>
        /// <param name = "B">1D double Array, B</param>
        /// <returns></returns>
        internal static double crossProduct2(IList<double> A, IList<double> B)
        {
            if (((A.Count() == 2) && (B.Count() == 2))
                || ((A.Count() == 3) && (B.Count() == 3) && A[2] == 0.0 && B[2] == 0.0))
                return A[0] * B[1] - B[0] * A[1];
            throw new Exception("This cross product \"shortcut\" is only used with 2D vectors to get the single value in the,"
                                + "would be, Z-direction.");
        }

        #region Multiply vector by transpose of another vector.
        /// <summary>
        /// Multiply vector by transpose of another vector to create a matrix.
        /// Product of each element of array-1 (1D double) with each element of array-2 (1D double)
        /// C[i,j] = A[i] * B[j]
        /// </summary>
        /// <param name = "A">1D double array - column vector (1 element per row)</param>
        /// <param name = "B">1D double array - row vector (1 element column)</param>
        /// <returns>2D double array product matrix, value of element [i,j] = A[i] * B[j]</returns>
        internal static double[,] multiplyVectorsIntoAMatrix(IList<double> A, IList<double> B)
        { return multiplyVectorsIntoAMatrix(A, B, A.Count(), B.Count()); }
        /// <summary>
        /// Multiply vector by transpose of another vector to create a matrix.
        /// Product of each element of array-1 (1D int) with each element of array-2 (1D double)
        /// C[i,j] = A[i] * B[j]
        /// </summary>
        /// <param name = "A">1D integer array - column vector (1 element per row)</param>
        /// <param name = "B">1D double array - row vector (1 element column)</param>
        /// <returns>2D double array product matrix, value of element [i,j] = A[i] * B[j]</returns>
        internal static double[,] multiplyVectorsIntoAMatrix(IList<int> A, IList<double> B)
        { return multiplyVectorsIntoAMatrix(A, B, A.Count(), B.Count()); }
        /// <summary>
        /// Multiply vector by transpose of another vector to create a matrix.
        /// Product of each element of array-1 (1D int) with each element of array-2 (1D double)
        /// C[i,j] = A[i] * B[j]
        /// </summary>
        /// <param name = "A">1D double array - column vector (1 element per row)</param>
        /// <param name = "B">1D integer array - row vector (1 element column)</param>
        /// <returns>2D double array product matrix, value of element [i,j] = A[i] * B[j]</returns>
        internal static double[,] multiplyVectorsIntoAMatrix(IList<double> A, IList<int> B)
        { return multiplyVectorsIntoAMatrix(A, B, A.Count(), B.Count()); }
        /// <summary>
        /// Multiply vector by transpose of another vector to create a matrix.
        /// Product of each element of array-1 (1D int) with each element of array-2 (1D int)
        /// C[i,j] = A[i] * B[j]
        /// </summary>
        /// <param name = "A">1D integer array - column vector (1 element per row)</param>
        /// <param name = "B">1D integer array - row vector (1 element column)</param>
        /// <returns>2D double array product matrix, value of element [i,j] = A[i] * B[j]</returns>
        internal static int[,] multiplyVectorsIntoAMatrix(IList<int> A, IList<int> B)
        { return multiplyVectorsIntoAMatrix(A, B, A.Count(), B.Count()); }
        /// <summary>
        /// Multiply vector by transpose of another vector to create a matrix.
        /// Product of each element of array-1 (1D double) with each element of array-2 (1D double)
        /// C[i,j] = A[i] * B[j]
        /// </summary>
        /// <param name = "A">1D double array - column vector (1 element per row)</param>
        /// <param name = "B">1D double array - row vector (1 element column)</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of colimns.</param>
        /// <returns>2D double array product matrix, value of element [i,j] = A[i] * B[j]</returns>
        internal static double[,] multiplyVectorsIntoAMatrix(IList<double> A, IList<double> B,
            int numRows, int numCols)
        {
            var C = new double[numRows, numCols];

            for (var i = 0; i != numRows; i++)
                for (var j = 0; j != numCols; j++)
                    C[i, j] = A[i] * B[j];
            return C;
        }
        /// <summary>
        /// Multiply vector by transpose of another vector to create a matrix.
        /// Product of each element of array-1 (1D int) with each element of array-2 (1D double)
        /// C[i,j] = A[i] * B[j]
        /// </summary>
        /// <param name = "A">1D integer array - column vector (1 element per row)</param>
        /// <param name = "B">1D double array - row vector (1 element column)</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of colimns.</param>
        /// <returns>2D double array product matrix, value of element [i,j] = A[i] * B[j]</returns>
        internal static double[,] multiplyVectorsIntoAMatrix(IList<int> A, IList<double> B,
            int numRows, int numCols)
        {
            var C = new double[numRows, numCols];

            for (var i = 0; i != numRows; i++)
                for (var j = 0; j != numCols; j++)
                    C[i, j] = A[i] * B[j];
            return C;
        }
        /// <summary>
        /// Multiply vector by transpose of another vector to create a matrix.
        /// Product of each element of array-1 (1D int) with each element of array-2 (1D double)
        /// C[i,j] = A[i] * B[j]
        /// </summary>
        /// <param name = "A">1D double array - column vector (1 element per row)</param>
        /// <param name = "B">1D integer array - row vector (1 element column)</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of colimns.</param>
        /// <returns>2D double array product matrix, value of element [i,j] = A[i] * B[j]</returns>
        internal static double[,] multiplyVectorsIntoAMatrix(IList<double> A, IList<int> B,
            int numRows, int numCols)
        {
            var C = new double[numRows, numCols];

            for (var i = 0; i != numRows; i++)
                for (var j = 0; j != numCols; j++)
                    C[i, j] = A[i] * B[j];
            return C;
        }
        /// <summary>
        /// Multiply vector by transpose of another vector to create a matrix.
        /// Product of each element of array-1 (1D int) with each element of array-2 (1D int)
        /// C[i,j] = A[i] * B[j]
        /// </summary>
        /// <param name="A">1D integer array - column vector (1 element per row)</param>
        /// <param name="B">1D integer array - row vector (1 element column)</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of colimns.</param>
        /// <returns>
        /// 2D double array product matrix, value of element [i,j] = A[i] * B[j]
        /// </returns>
        internal static int[,] multiplyVectorsIntoAMatrix(IList<int> A, IList<int> B,
            int numRows, int numCols)
        {
            var C = new int[numRows, numCols];

            for (var i = 0; i != numRows; i++)
                for (var j = 0; j != numCols; j++)
                    C[i, j] = A[i] * B[j];
            return C;
        }
        #endregion

        #region Matrix(2D) to matrix(2D) multiplication
        /// <summary>
        /// Product of two matrices (2D double)
        /// </summary>
        /// <param name = "A">2D double Array, A</param>
        /// <param name = "B">2D double Array, A</param>
        /// <returns>A 2D double array that is the product of the two matrices A and B</returns>
        internal static double[,] multiply(double[,] A, double[,] B)
        {
            if (A.GetLength(1) != B.GetLength(0))
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(A, B, A.GetLength(0), B.GetLength(1));
        }

        /// <summary>
        /// Product of two matrices (2D double)
        /// </summary>
        /// <param name = "A">2D int Array, A</param>
        /// <param name = "B">2D double Array, A</param>
        /// <returns>A 2D double array that is the product of the two matrices A and B</returns>
        internal static double[,] multiply(int[,] A, double[,] B)
        {
            if (A.GetLength(1) != B.GetLength(0))
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(A, B, A.GetLength(0), B.GetLength(1));
        }

        /// <summary>
        /// Product of two matrices (2D double)
        /// </summary>
        /// <param name = "A">2D double Array, A</param>
        /// <param name = "B">2D int Array, A</param>
        /// <returns>A 2D double array that is the product of the two matrices A and B</returns>
        internal static double[,] multiply(double[,] A, int[,] B)
        {
            if (A.GetLength(1) != B.GetLength(0))
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(A, B, A.GetLength(0), B.GetLength(1));
        }


        /// <summary>
        /// Product of two matrices (2D double)
        /// </summary>
        /// <param name = "A">2D int Array, A</param>
        /// <param name = "B">2D int Array, A</param>
        /// <returns>A 2D int array that is the product of the two matrices A and B</returns>
        internal static int[,] multiply(int[,] A, int[,] B)
        {
            if (A.GetLength(1) != B.GetLength(0))
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(A, B, A.GetLength(0), B.GetLength(1));
        }

        /// <summary>
        /// Product of two matrices (2D double)
        /// </summary>
        /// <param name = "A">2D double Array, A</param>
        /// <param name = "B">2D double Array, A</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>A 2D double array that is the product of the two matrices A and B</returns>
        internal static double[,] multiply(double[,] A, double[,] B, int numRows, int numCols)
        {
            var C = new double[numRows, numCols];

            for (var i = 0; i != numRows; i++)
                for (var j = 0; j != numCols; j++)
                {
                    C[i, j] = 0.0;
                    for (var k = 0; k != A.GetLength(1); k++)
                        C[i, j] += A[i, k] * B[k, j];
                }
            return C;
        }

        /// <summary>
        /// Product of two matrices (2D double)
        /// </summary>
        /// <param name = "A">2D int Array, A</param>
        /// <param name = "B">2D double Array, A</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>A 2D double array that is the product of the two matrices A and B</returns>
        internal static double[,] multiply(int[,] A, double[,] B, int numRows, int numCols)
        {
            var C = new double[numRows, numCols];

            for (var i = 0; i != numRows; i++)
                for (var j = 0; j != numCols; j++)
                {
                    C[i, j] = 0.0;
                    for (var k = 0; k != A.GetLength(1); k++)
                        C[i, j] += A[i, k] * B[k, j];
                }
            return C;
        }
        /// <summary>
        /// Product of two matrices (2D double)
        /// </summary>
        /// <param name = "A">2D double Array, A</param>
        /// <param name = "B">2D int Array, A</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>A 2D double array that is the product of the two matrices A and B</returns>
        internal static double[,] multiply(double[,] A, int[,] B, int numRows, int numCols)
        {
            var C = new double[numRows, numCols];

            for (var i = 0; i != numRows; i++)
                for (var j = 0; j != numCols; j++)
                {
                    C[i, j] = 0.0;
                    for (var k = 0; k != A.GetLength(1); k++)
                        C[i, j] += A[i, k] * B[k, j];
                }
            return C;
        }

        /// <summary>
        /// Product of two matrices (2D double)
        /// </summary>
        /// <param name="A">2D int Array, A</param>
        /// <param name="B">2D int Array, A</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>
        /// A 2D int array that is the product of the two matrices A and B
        /// </returns>
        internal static int[,] multiply(int[,] A, int[,] B, int numRows, int numCols)
        {
            var C = new int[numRows, numCols];

            for (var i = 0; i != numRows; i++)
                for (var j = 0; j != numCols; j++)
                {
                    C[i, j] = 0;
                    for (var k = 0; k != A.GetLength(1); k++)
                        C[i, j] += A[i, k] * B[k, j];
                }
            return C;
        }
        #endregion

        #region Multiply matrix to a vector (and vice versa)
        /// <summary>
        /// Product of a matrix and a vector (2D double and 1D double)
        /// </summary>
        /// <param name = "A">2D double Array</param>
        /// <param name = "B">1D double array - column vector (1 element row)</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(double[,] A, IList<double> B)
        {
            // this is B dot term_i multiplication
            var numRows = A.GetLength(0);
            var numCols = A.GetLength(1);
            if (numCols != B.Count())
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(A, B, numRows, numCols);
        }

        /// <summary>
        /// Product of a matrix and a vector (2D double and 1D double)
        /// </summary>
        /// <param name = "A">2D int Array</param>
        /// <param name = "B">1D double array - column vector (1 element row)</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(int[,] A, IList<double> B)
        {
            // this is B dot term_i multiplication
            var numRows = A.GetLength(0);
            var numCols = A.GetLength(1);
            if (numCols != B.Count())
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(A, B, numRows, numCols);
        }

        /// <summary>
        /// Product of two matrices (2D double and 1D double)
        /// </summary>
        /// <param name = "A">2D double Array</param>
        /// <param name = "B">1D int array - column vector (1 element row)</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(double[,] A, IList<int> B)
        {
            // this is B dot term_i multiplication
            var numRows = A.GetLength(0);
            var numCols = A.GetLength(1);
            if (numCols != B.Count())
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(A, B, numRows, numCols);
        }

        /// <summary>
        /// Product of two matrices (2D double and 1D double)
        /// </summary>
        /// <param name = "A">2D int Array</param>
        /// <param name = "B">1D int array - column vector (1 element row)</param>
        /// <returns>A 1D int array that is the product of the two matrices A and B</returns>
        internal static int[] multiply(int[,] A, IList<int> B)
        {
            // this is B dot term_i multiplication
            var numRows = A.GetLength(0);
            var numCols = A.GetLength(1);
            if (numCols != B.Count())
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(A, B, numRows, numCols);
        }

        /// <summary>
        /// Product of two matrices (1D double and 2D double)
        /// </summary>
        /// <param name = "A">1D double array - row vector (1 element column)</param>
        /// <param name = "B">2D double Array</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(IList<double> B, double[,] A)
        {
            // this is B dot term_i multiplication
            var numRows = A.GetLength(0);
            var numCols = A.GetLength(1);
            if (numRows != B.Count())
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(B, A, numRows, numCols);
        }

        /// <summary>
        /// Product of two matrices (1D double and 2D double)
        /// </summary>
        /// <param name = "A">1D double array - row vector (1 element column)</param>
        /// <param name = "B">2D int Array</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(IList<double> B, int[,] A)
        {
            // this is B dot term_i multiplication
            var numRows = A.GetLength(0);
            var numCols = A.GetLength(1);
            if (numRows != B.Count())
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(B, A, numRows, numCols);
        }

        /// <summary>
        /// Product of two matrices (1D double and 2D double)
        /// </summary>
        /// <param name = "A">1D int array - row vector (1 element column)</param>
        /// <param name = "B">2D double Array</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(IList<int> B, double[,] A)
        {
            // this is B dot term_i multiplication
            var numRows = A.GetLength(0);
            var numCols = A.GetLength(1);
            if (numRows != B.Count())
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(B, A, numRows, numCols);
        }

        /// <summary>
        /// Product of two matrices (1D double and 2D double)
        /// </summary>
        /// <param name = "A">1D int array - row vector (1 element column)</param>
        /// <param name = "B">2D int Array</param>
        /// <returns>A 1D int array that is the product of the two matrices A and B</returns>
        internal static int[] multiply(IList<int> B, int[,] A)
        {
            // this is B dot term_i multiplication
            var numRows = A.GetLength(0);
            var numCols = A.GetLength(1);
            if (numRows != B.Count())
                throw new Exception("Column count in first matrix must be equal to row count in second matrix");
            return multiply(B, A, numRows, numCols);
        }

        /// <summary>
        /// Product of a matrix and a vector (2D double and 1D double)
        /// </summary>
        /// <param name = "A">2D double Array</param>
        /// <param name = "B">1D double array - column vector (1 element row)</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(double[,] A, IList<double> B, int numRows, int numCols)
        {
            var C = new double[numRows];

            for (var i = 0; i != numRows; i++)
            {
                C[i] = 0.0;
                for (var j = 0; j != numCols; j++)
                    C[i] += A[i, j] * B[j];
            }
            return C;
        }

        /// <summary>
        /// Product of a matrix and a vector (2D double and 1D double)
        /// </summary>
        /// <param name = "A">2D int Array</param>
        /// <param name = "B">1D double array - column vector (1 element row)</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(int[,] A, IList<double> B, int numRows, int numCols)
        {
            var C = new double[numRows];

            for (var i = 0; i != numRows; i++)
            {
                C[i] = 0.0;
                for (var j = 0; j != numCols; j++)
                    C[i] += A[i, j] * B[j];
            }
            return C;
        }

        /// <summary>
        /// Product of two matrices (2D double and 1D double)
        /// </summary>
        /// <param name = "A">2D double Array</param>
        /// <param name = "B">1D int array - column vector (1 element row)</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(double[,] A, IList<int> B, int numRows, int numCols)
        {
            var C = new double[numRows];

            for (var i = 0; i != numRows; i++)
            {
                C[i] = 0.0;
                for (var j = 0; j != numCols; j++)
                    C[i] += A[i, j] * B[j];
            }
            return C;
        }

        /// <summary>
        /// Product of two matrices (2D double and 1D double)
        /// </summary>
        /// <param name = "A">2D int Array</param>
        /// <param name = "B">1D int array - column vector (1 element row)</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>A 1D int array that is the product of the two matrices A and B</returns>
        internal static int[] multiply(int[,] A, IList<int> B, int numRows, int numCols)
        {
            var C = new int[numRows];

            for (var i = 0; i != numRows; i++)
            {
                C[i] = 0;
                for (var j = 0; j != numCols; j++)
                    C[i] += A[i, j] * B[j];
            }
            return C;
        }

        /// <summary>
        /// Product of two matrices (1D double and 2D double)
        /// </summary>
        /// <param name = "A">1D double array - row vector (1 element column)</param>
        /// <param name = "B">2D double Array</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(IList<double> B, double[,] A, int numRows, int numCols)
        {
            var C = new double[numCols];

            for (var i = 0; i != numCols; i++)
            {
                C[i] = 0.0;
                for (var j = 0; j != numRows; j++)
                    C[i] += B[j] * A[j, i];
            }
            return C;
        }
        /// <summary>
        /// Product of two matrices (1D double and 2D double)
        /// </summary>
        /// <param name = "A">1D double array - row vector (1 element column)</param>
        /// <param name = "B">2D int Array</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(IList<double> B, int[,] A, int numRows, int numCols)
        {
            var C = new double[numCols];

            for (var i = 0; i != numCols; i++)
            {
                C[i] = 0.0;
                for (var j = 0; j != numRows; j++)
                    C[i] += B[j] * A[j, i];
            }
            return C;
        }
        /// <summary>
        /// Product of two matrices (1D double and 2D double)
        /// </summary>
        /// <param name = "A">1D int array - row vector (1 element column)</param>
        /// <param name = "B">2D double Array</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>A 1D double array that is the product of the two matrices A and B</returns>
        internal static double[] multiply(IList<int> B, double[,] A, int numRows, int numCols)
        {
            var C = new double[numCols];

            for (var i = 0; i != numCols; i++)
            {
                C[i] = 0.0;
                for (var j = 0; j != numRows; j++)
                    C[i] += B[j] * A[j, i];
            }
            return C;
        }
        /// <summary>
        /// Product of two matrices (1D double and 2D double)
        /// </summary>
        /// <param name="B">2D int Array</param>
        /// <param name="A">1D int array - row vector (1 element column)</param>
        /// <param name="numRows">The number of rows.</param>
        /// <param name="numCols">The number of columns.</param>
        /// <returns>
        /// A 1D int array that is the product of the two matrices A and B
        /// </returns>
        internal static int[] multiply(IList<int> B, int[,] A, int numRows, int numCols)
        {
            var C = new int[numCols];

            for (var i = 0; i != numCols; i++)
            {
                C[i] = 0;
                for (var j = 0; j != numRows; j++)
                    C[i] += B[j] * A[j, i];
            }
            return C;
        }

        #endregion

        #region Add Vector-to-Vector and Matrix-to-Matrix

        /// <summary>
        /// Adds arrays A and B
        /// </summary>
        /// <param name = "A">1D double array 1</param>
        /// <param name = "B">1D double array 2</param>
        /// <returns>1D double array that contains sum of vectros A and B</returns>
        internal static double[] add(IList<double> A, IList<double> B)
        {
            var length = A.Count();
            if (length != B.Count()) throw new Exception("Matrix sizes do not match");
            return add(A, B, length);
        }
        /// <summary>
        /// Adds arrays A and B
        /// </summary>
        /// <param name = "A">1D double array 1</param>
        /// <param name = "B">1D double array 2</param>
        /// <param name="length">The length of the array.</param>
        /// <returns>1D double array that contains sum of vectros A and B</returns>
        internal static double[] add(IList<double> A, IList<double> B, int length)
        {
            var c = new double[length];
            for (var i = 0; i != length; i++)
                c[i] = A[i] + B[i];
            return c;
        }
        #endregion

    }
}