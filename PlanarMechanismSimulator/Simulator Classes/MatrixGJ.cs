using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PMKS
{
    public class MatrixGJ
    {
        double[,] matrix;

        public MatrixGJ()
        {
            this.matrix = new double[0, 0];
        }

        public MatrixGJ(double[,] matrix)
        {
            this.matrix = matrix;
        }

        public double[,] Matrix
        {
            get
            {
                return this.matrix;
            }
            set
            {
                this.matrix = value;
            }
        }

        public double[,] Inverse()
        {
            double[,] extendedMatrix = new double[1, 2];
            int size = this.matrix.GetUpperBound(0) + 1;

            if (this.matrix.GetUpperBound(0) == this.matrix.GetUpperBound(1))
            {
                extendedMatrix = new double[size, 2 * size];
            }
            else
            {
                System.Diagnostics.Debug.WriteLine("The given matrix is not square");
                return new double[0, 0];
            }

            for (int i = 0; i < size; i++)
            {
                for (int j = 0; j < size; j++)
                {
                    extendedMatrix[i, j] = this.matrix[i, j];
                    if (i != j)
                    {
                        extendedMatrix[i, j + size] = 0;
                    }
                    else
                    {
                        extendedMatrix[i, j + size] = 1;
                    }
                }
            }
            for (int i = 0; i < size; i++)
            {
                if (extendedMatrix[i, i] != 0.0)
                {
                    double divideForOne = extendedMatrix[i, i];
                    for (int j = 0; j < 2 * size; j++)
                    {
                        extendedMatrix[i, j] /= divideForOne;
                    }
                }
                else
                {
                    for (int j = i + 1; j < size; j++)
                    {
                        if (extendedMatrix[j, i] != 0.0)
                        {
                            double[] holdRow = new double[2 * size];
                            for (int h = 0; h < 2 * size; h++)
                            {
                                holdRow[h] = extendedMatrix[i, h];
                                extendedMatrix[i, h] = extendedMatrix[j, h];
                                extendedMatrix[j, h] = holdRow[h];
                            }
                            break;
                        }
                    }
                    i -= 1;
                }

                for (int k = 0; k < size; k++)
                {
                    if (k != i && extendedMatrix[k, i] != 0.0)
                    {
                        double valToBeZero = extendedMatrix[k, i];
                        for (int l = 0; l < 2 * size; l++)
                        {
                            extendedMatrix[k, l] -= (valToBeZero * extendedMatrix[i, l]);
                        }
                    }
                }

                /*
                //for printing purposes
                for (int q = 0; q < size; q++)
                {
                    String extendedLine = "";
                    for (int j = 0; j < 2*size; j++)
                    {
                        extendedLine += extendedMatrix[q, j].ToString();
                        extendedLine += "\t";
                        //Console.Write(extendedMatrix[q, j] + "\t");
                    }
                    //Console.WriteLine();
                    System.Diagnostics.Debug.WriteLine(extendedLine);
                }
                //Console.WriteLine();
                System.Diagnostics.Debug.WriteLine("");
                */

            }
            double[,] output = new double[size, size];
            for (int i = 0; i < size; i++)
            {
                for (int j = 0; j < size; j++)
                {
                    output[i, j] = extendedMatrix[i, j + size];
                }
            }
            /*
            Console.WriteLine("Final Inverted Matrix:");
            for (int q = 0; q < size; q++)
            {
                for (int j = 0; j < size; j++)
                {
                    Console.Write(output[q, j] + "\t");
                }
                Console.WriteLine();
            }
            */
            return output;
        }

        public double[,] Multiply(MatrixGJ otherMatrix)
        {
            int sizeA = this.matrix.GetUpperBound(0) + 1;
            int sizeB = this.matrix.GetUpperBound(1) + 1;
            int sizeC = otherMatrix.Matrix.GetUpperBound(1) + 1;
            double[,] output = new double[sizeA, sizeC];

            if (sizeB == otherMatrix.Matrix.GetUpperBound(0) + 1)
            {
                for (int a = 0; a < sizeA; a++)
                {
                    for (int c = 0; c < sizeC; c++)
                    {
                        output[a, c] = 0;
                        for (int b = 0; b < sizeB; b++)
                        {
                            output[a, c] += (this.matrix[a, b] * otherMatrix.Matrix[b, c]);
                        }
                    }
                }
            }
            return output;
        }

        public List<string> PrintMatrix()
        {
            List<string> outLoS = new List<string>();
            for (int i = 0; i < this.matrix.GetUpperBound(0) + 1; i++)
            {
                string row = "";
                for (int j = 0; j < this.matrix.GetUpperBound(1) + 1; j++)
                {
                    //Console.Write(this.matrix[i, j] + "\t");
                    string roundedNum = RoundToSignificantDigits(this.matrix[i, j], 5).ToString();
                    row += (roundedNum + "\t");
                }
                //Console.WriteLine();
                System.Diagnostics.Debug.WriteLine(row);
                outLoS.Add(row);
            }
            return outLoS;
        }

        public double RoundToSignificantDigits(double d, int digits)
        {
            if (d == 0)
                return 0;
            if (Math.Abs(d) < 0.00001)
                return 0;
            double scale = Math.Pow(10, Math.Floor(Math.Log10(Math.Abs(d))) + 1);
            return scale * Math.Round(d / scale, digits);
        }

        public List<string> PrintCleanMatrix()
        {
            List<string> outLoS = new List<string>();
            for (int i = 0; i < this.matrix.GetUpperBound(0) + 1; i++)
            {
                string row = "";
                for (int j = 0; j < this.matrix.GetUpperBound(1) + 1; j++)
                {
                    //Console.Write(this.matrix[i, j] + "\t");
                    string value = "";
                    if (this.matrix[i, j].ToString().Length <= 7)
                    {
                        value = this.matrix[i, j].ToString();
                    }
                    else
                    {
                        for (int k = 0; k < 7; k++)
                        {
                            value += this.matrix[i, j].ToString()[k];
                        }
                    }
                    row += (value + "\t");
                }
                //Console.WriteLine();
                System.Diagnostics.Debug.WriteLine(row);
                outLoS.Add(row);
            }
            return outLoS;
        }

        ~MatrixGJ()
        {
            //Console.WriteLine("MatrixGJ object collected");
        }
    }
}
