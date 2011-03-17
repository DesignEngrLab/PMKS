using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using GraphSynth.Representation;
using GraphSynth;
using OptimizationToolbox;
using StarMathLib;

namespace PlanarMechanismSimulator

    //at time t=0; all acceleration and velocity are zero
{
    public partial class MechSimulation : IDependentAnalysis
    {
        #region New Acceleration Determination
        private void findAccelerationNew(List<node> pivots, int timeRow, circleDiagramItem[] circleDiagram, List<node> links, DynamicMatrixTerm[] coriolis1, double[, ,] coriolis, DynamicMatrixTerm[] unknowns, DynamicMatrixTerm[] Omeg, double[, ,] slipacceleration, double iAlpha, double iOmega, double newt)
        {
           
            
            
            
            #region Filling up values in the different rows
            int maxCol = unknowns.GetLength(0);
            int maxRow = Omeg.GetLength(0);
            double[,] accelerationMatrix = new double[maxRow, maxCol];
            double[,] wsquareMatrix = new double[maxRow, 1];
            int no = 0;

            for (int i = 0; i < maxRow; i++) //shouldn't this be 2*no of equations taking into account X and Y
            {
                Direction d = Omeg[i].dir;
                node link = Omeg[i].belongsTo;
                node pivot0 = coriolis1[i].belongsFrom;
                node pivot1 = coriolis1[i].belongsTo;

                for (no = 0; no < n; no++)
                    if (links[no] == link)
                        break;
                //using "no" we can obtain coriolis component and 
                //also omega from linkparameters
                if (d == Direction.X)
                    wsquareMatrix[i, 0] = -(LinkParameters[no, timeRow, 0] * LinkParameters[no, timeRow, 0] * (-pivot1.X + pivot0.X)) + coriolis[no, timeRow, 0];
                if (d == Direction.Y)
                    wsquareMatrix[i, 0] = -(LinkParameters[no, timeRow, 0] * LinkParameters[no, timeRow, 0] * (-pivot1.Y + pivot0.Y)) + coriolis[no, timeRow, 1];

                //we also need to alpha X (link length)  - in terms of X and Y - but this will be in acceleration matrix as part of separate equation
                for (int j = 0; j < maxCol; j++)
                {
                    if (unknowns[j].dir == d)
                    //&& unknowns[j].defaultValue != 0.0
                    {
                        if (unknowns[j].belongsTo == pivot0)
                            accelerationMatrix[i, j] = 1;
                        if (unknowns[j].belongsTo == pivot1)
                            accelerationMatrix[i, j] = -1;

                    }

                    if (unknowns[j].type == DynamicType.angularAcceleration && unknowns[j].belongsTo == link)
                        // && unknowns[j].defaultValue != 0
                        if (d == Direction.X)
                            accelerationMatrix[i, j] = (-pivot1.Y + pivot0.Y);
                        else if (d == Direction.Y)
                            accelerationMatrix[i, j] = -(-pivot1.X + pivot0.X);
                    if (unknowns[j].type == DynamicType.radialAcceleration)
                    //&& unknowns[j].defaultValue != 0
                    {
                        //this is for a PIS // slider on a link

                        if (Omeg[i].belongsTo.localLabels.Contains("pis_conn"))
                            accelerationMatrix[i, j] = -1;
                    }


                }



            }
            #endregion

            #region Collapsing matrices - Not used any more

            //collect all indices where 

            //List<int> indexNos = new List<int>();

            //for(int i=0;i<maxCol;i++)
            //{
            //    if (unknowns[i].type == DynamicType.absoluteAcceleration)
            //    {
            //        if (unknowns[i].belongsTo.localLabels.Contains("ground"))
            //            indexNos.Add(i);
            //    }

            //    if (unknowns[i].type == DynamicType.angularAcceleration)
            //    {
            //        if (unknowns[i].belongsTo.localLabels.Contains("ground") || unknowns[i].belongsTo.localLabels.Contains("input"))
            //            indexNos.Add(i);
            //    }

            //    if (unknowns[i].type == DynamicType.radialAcceleration)
            //    {
            //        if (unknowns[i].defaultValue == 0)
            //            indexNos.Add(i);
            //    }

            //}  
            ////we shall collapse the columns in the acceleration matrix

            //int nooftermstoremove = indexNos.Count;
            //int w=0,v=0;

            //double[,] ColColapAMatrix = new double[2 * numEqs, maxCol - nooftermstoremove];

            //for (int i = 0; i < 2 * numEqs; i++)
            //{
            //    for (int j = 0; j < maxCol; j++)
            //    {
            //        foreach (int k in indexNos)
            //            if (j == k)
            //                break;
            //            else
            //            {
            //                ColColapAMatrix[w, v] = accelerationMatrix[i, j];
            //                v++;
            //            }
            //    }
            //    w++;
            //}

            ////row collapsing - collapsing ground link
            //List<int> rowindex = new List<int>();
            //for (int i = 0; i < 2 * numEqs; i++)
            //    if (Omeg[i].belongsTo.localLabels.Contains("ground"))
            //        rowindex.Add(i);

            //w = 0; v = 0;
            //double[,] FinalAMatrix = new double[2*numEqs-rowindex.Count,maxCol-nooftermstoremove];
            //double[,] FinalWMatrix = new double[2*numEqs-rowindex.Count,1];
            //for (int i = 0; i < 2 * numEqs; i++)
            //    foreach (int k in rowindex)
            //        if (i == k)
            //            break;
            //        else
            //        {
            //            for (int j = 0; j < maxCol - nooftermstoremove; j++)
            //                FinalAMatrix[w, j] = ColColapAMatrix[i, j];

            //            FinalWMatrix[w, 0] = wsquareMatrix[i, 0];

            //            w++;
            //        }
            #endregion

            #region Removing the input ang accln and adding it to the other omegaSquareMatrix

            //apart from removing input ang accln, we also need to remove slider's (X/Y) and pis_conn link should only
            //contain slip acceleration - why not remove in set up itself?


            //there is only input - which means there is only one column to be removed
            //in cases now, we have alpha = 0; which may not be the same

            //creating a new matrix for acceleration

            double[,] accelerationMatrix_New = new double[maxRow, maxCol - 1];
            double[,] inputAlphaColumn = new double[maxRow, 1];

            //getting the index of unknowns
            int indexno = 0;
            for (int i = 0; i < maxCol; i++)
                if (unknowns[i].belongsTo.localLabels.Contains("link"))

                    foreach (circleDiagramItem c in circleDiagram)
                        if (c.link1.localLabels.Contains("ground") && c.link1.localLabels.Contains("link") && c.speed == iOmega)
                            if (c.link2 == unknowns[i].belongsTo)
                            {
                                indexno = i;
                                break;
                            }


            //got to have new unknowns matrix
            DynamicMatrixTerm[] unknowns_New = new DynamicMatrixTerm[maxCol - 1];
            int z = 0;
            for (int i = 0; i < maxCol; i++)
            {
                if (i != indexno)
                {
                    unknowns_New[z] = unknowns[i];
                    z++;
                }
            }

            //adding the acceleration matrix to the new one and the lone column to inputAlphaColumn
            int w = 0, v = 0;
            for (int i = 0; i < maxRow; i++)
            {
                v = 0;
                for (int j = 0; j < maxCol; j++)
                {
                    if (j != indexno)
                    {
                        accelerationMatrix_New[w, v] = accelerationMatrix[i, j];
                        v++;
                    }
                    else
                        inputAlphaColumn[w, 0] = accelerationMatrix[i, j];



                }
                w++;
            }

            //adding the inputAlphaColumn to the wsquareMatrix

            double[,] New_wsquareMatrix = new double[maxRow, 1];

            for (int i = 0; i < maxRow; i++)
                New_wsquareMatrix[i, 0] = wsquareMatrix[i, 0] + iAlpha * inputAlphaColumn[i, 0];


            #endregion

            double[,] result = new double[maxRow, 1];

            #region Making row diagonal

            if (maxRow > 9)
            {

                while (diag(accelerationMatrix_New))
                {
                    int store_1 = 0;
                    for (int i = 0; i < accelerationMatrix_New.GetLength(0); i++)
                    {
                        for (int j = 0; j < accelerationMatrix_New.GetLength(0); j++)
                        {
                            double kk = accelerationMatrix_New.GetLength(0);


                            if (i == j && (accelerationMatrix_New[i, j] == 0))
                            {

                                //now i know that the diagonal in that particular element is 0
                                //search for another row that does not have zero in the same column 
                                //and make sure that this index does not show up again...

                                for (int k = 0; k < accelerationMatrix_New.GetLength(0); k++)
                                    if (accelerationMatrix_New[k, j] != 0)
                                    {
                                        //now store this row
                                        store_1 = k;
                                        break;
                                    }

                                //interchange both these two rows
                                //store these two values in two separate lists
                                //and then replace them in the matrix back
                                double[] list1 = StarMath.GetRow(i,accelerationMatrix_New);
                                double[] list2 = StarMath.GetRow(store_1,accelerationMatrix_New );
                                //now replacing them back to the acceleration matrix
                                StarMath.SetRow(i,accelerationMatrix_New, list2 );
                                StarMath.SetRow(store_1,accelerationMatrix_New, list1 );

                                //List<double> list1 = new List<double>();
                                //List<double> list2 = new List<double>();
                                //for (int m = 0; m < accelerationMatrix_New.GetLength(0); m++)
                                //    list1.Add(accelerationMatrix_New[i, m]);
                                //for (int m = 0; m < accelerationMatrix_New.GetLength(0); m++)
                                //    list2.Add(accelerationMatrix_New[store_1, m]);
                                //for (int m = 0; m < accelerationMatrix_New.GetLength(0); m++)
                                //    accelerationMatrix_New[i, m] = list2[m];
                                //for (int m = 0; m < accelerationMatrix_New.GetLength(0); m++)
                                //    accelerationMatrix_New[store_1, m] = list1[m];

                                //we also need to change the omega matrix and also the unknowns matrix too as we change the order
                                //of the acceleration matrix

                                double w_1 = New_wsquareMatrix[i, 0];
                                double w_2 = New_wsquareMatrix[store_1, 0];
                                DynamicMatrixTerm a1 = unknowns_New[i];
                                DynamicMatrixTerm a2 = unknowns_New[store_1];


                                //now replace

                                New_wsquareMatrix[i, 0] = w_2;
                                New_wsquareMatrix[store_1, 0] = w_1;

                                unknowns_New[i] = a2;
                                unknowns_New[store_1] = a1;

                            }

                        }

                    }






                }

                double[,] inversematrix = StarMath.inverse(accelerationMatrix_New);
                double[,] A =StarMath.multiply(accelerationMatrix_New, inversematrix) ;
                double error = StarMath.norm2(StarMath.subtract(A , StarMath.makeIdentity(A.GetLength(0))));
                result = StarMath.multiply(inversematrix, New_wsquareMatrix);



            #endregion


            }

            else
            {



                double[,] cofactorMatrix = new double[maxRow, maxRow];

                for (int i = 0; i < maxRow; i++)
                {
                    for (int j = 0; j < maxRow; j++)
                    {
                        cofactorMatrix[i, j] = Math.Pow((-1), (i + j)) * MatrixMinor(accelerationMatrix_New, i, j, maxRow);
                    }

                }




                double determinant = 0.0;

                determinant = MatrixDeterminant(accelerationMatrix_New, maxRow);//passing matrix and order

                if (determinant != 0)
                {

                }

                double[,] inversematrix = new double[maxRow, maxRow];

                inversematrix = StarMath.multiply((1 / determinant), (StarMath.transpose(cofactorMatrix)));


                double determinant1 = 0.0;

                determinant1 = MatrixDeterminant(inversematrix, maxRow);


                #region Equation Solving
                double[,] result1 = new double[maxRow, 1];

                //it is required to ensure that diagonals are not zero for the StarMath.Inverse routine to work
                //so we are going to interchange rows 
                //ref: http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?Effective_UBLAS/Matrix_Inversion
                //logic needs to be worked out with different types of matrices
                //and also determine whether multiple iterations are required

                double[,] rowPlaceHolders = new double[maxRow, 1];


                //for (int i = 0; i < maxRow; i++)
                //    for (int j = 0; j < maxRow; j++)
                //    {
                //        if (i == j)
                //        {
                //            if (accelerationMatrix_New[i, j] == 0)
                //            {
                //                //need to find another row that has that column non zero
                //                //add the new row in the place of this and the existing in the place of new
                //                int existingrow = i;
                //                int newRow = MatrixSwapFunction(accelerationMatrix_New, i, j, maxRow);
                //                rowPlaceHolders[i, 0] = newRow;
                //                rowPlaceHolders[newRow, 0] = i;
                //                double[,] iRow = new double[1, maxRow];
                //                double[,] pRow = new double[1, maxRow];
                //                double iWM = New_wsquareMatrix[i, 0];
                //                double pWM = New_wsquareMatrix[newRow, 0];


                //                for (int m = 0; m < maxRow; m++)
                //                {
                //                    iRow[0, m] = accelerationMatrix_New[i, m];
                //                    pRow[0, m] = accelerationMatrix_New[newRow, m];
                //                }

                //                for (int m = 0; m < maxRow; m++)
                //                {
                //                    accelerationMatrix_New[newRow, m] = iRow[0, m];
                //                    accelerationMatrix_New[i, m] = pRow[0, m];
                //                }

                //                New_wsquareMatrix[newRow, 0] = iWM;
                //                New_wsquareMatrix[i, 0] = pWM;


                //            }
                //            else
                //            {
                //                rowPlaceHolders[i, 0] = i;
                //            }
                //        }
                //    }

                result = StarMath.multiply(inversematrix, New_wsquareMatrix);



                #endregion
            }

            #region Storing values in the respective pivotparameters and link parameters

            //store results in the appropriate 
            //code below is typically for a four bar
            //need to correct it


            for (int i = 0; i < p; i++)
            {
                if (!pivots[i].localLabels.Contains("ground"))
                {
                    for (int b = 0; b < maxRow; b++)
                        if (unknowns_New[b].belongsTo.localLabels.Contains("pivot") && unknowns_New[b].belongsTo == pivots[i])
                        {
                            if (unknowns_New[b].dir == Direction.X)
                                PivotParameters[i, timeRow, 4] = PivotParameters[i, timeRow, 4] + result[b, 0];
                            else if (unknowns_New[b].dir == Direction.Y)
                                PivotParameters[i, timeRow, 5] = PivotParameters[i, timeRow, 5] + result[b, 0];
                        }
                }
            }

            for (int i = 0; i < n; i++)
            {
                if (!links[i].localLabels.Contains("ground"))
                {
                    for (int b = 0; b < maxRow; b++)
                        if (unknowns_New[b].belongsTo.localLabels.Contains("link") && unknowns_New[b].belongsTo == links[i])
                        {

                            LinkParameters[i, timeRow, 1] = result[b, 0];

                        }
                }
            }

            //also add the value to the slider too. this is not done in the previous code

            for (int i = 0; i < p; i++)
            {
                if (pivots[i].localLabels.Contains("slider_conn"))
                {
                    double accln1 = PivotParameters[i, timeRow, 4];
                    double accln2 = PivotParameters[i, timeRow, 5];

                    foreach (arc aa in pivots[i].arcs)
                        if (aa.localLabels.Contains("pivotarc"))
                            if (aa.otherNode(pivots[i]).localLabels.Contains("sliderh") ||
                                aa.otherNode(pivots[i]).localLabels.Contains("sliderv"))
                            {
                                node n_n = aa.otherNode(pivots[i]);
                                for (int j = 0; j < p; j++)
                                    if (pivots[j] == n_n)
                                    {
                                        PivotParameters[j, timeRow, 4] = accln1;
                                        PivotParameters[j, timeRow, 5] = accln2;
                                    }
                            }
                }

            }

            #endregion

        }



        #endregion
        #region MatrixMath Determinant
        private double MatrixDeterminant(double[,] accelerationMatrix_New, int maxRow)
        {
            //this is a general matrix determinant function for a nxn matrix
            //we have a square matrix which will be used
            //once the matrix is obtained we need to use a function which will recursively obtain the minor/cofactor of the particular element
            //once the cofactor size is 2 - i shall add find the determinant of a 2x2 matrix

            double det = 0.0;
            int i = 0, j;


            for (j = 0; j < maxRow; j++)
            {
                //passing the minor matrix too in this

                det += Math.Pow((-1), (i + j)) * accelerationMatrix_New[i, j] * MatrixMinor(accelerationMatrix_New, i, j, maxRow);

            }




            return (det);

        }

        private double MatrixMinor(double[,] accelerationMatrix_New, int i, int j, int maxRow)
        {
            double x = 0.0;
            //got to form a new matrix without the elements of ith row and jth column
            double[,] newmatrix = new double[maxRow - 1, maxRow - 1];
            int m = 0, n = 0;
            for (int k = 0; k < maxRow; k++)
            {
                if (k != i)
                {
                    n = 0;
                    for (int l = 0; l < maxRow; l++)
                    {
                        if (l != j)
                        {
                            newmatrix[m, n] = accelerationMatrix_New[k, l];
                            n++;
                        }

                    }
                    m++;
                }

            }
            //now we have the reduced matrix
            int order = maxRow - 1;
            if (order == 2)
            {
                x = newmatrix[0, 0] * newmatrix[1, 1] - newmatrix[0, 1] * newmatrix[1, 0];

            }
            else
            {

                int k = 0;
                for (int l = 0; l < n; l++)
                {
                    x += Math.Pow((-1), (k + l)) * newmatrix[k, l] * MatrixMinor(newmatrix, k, l, order);
                }

            }



            return x;
        }
        #endregion
        #region MatrixMath Row Swap Function
        private int MatrixSwapFunction(double[,] accelerationMatrix_New, int i, int j, int maxRow)
        {
            int p = 0;
            for (int k = 0; k < maxRow; k++)
                if (accelerationMatrix_New[k, j] != 0)
                {
                    p = k;
                    break;
                }
            //got to swap accelerationmatrix rows i and p
            //we can save these two rows in separate row matrices and then add them


            return p;
        }
        #endregion
        #region Function: Slip Velocities & Coriolis Component and update the respective velocities - completely horizontal/vertical PIS/slideronalink

        private void findLinearSlipVelocities(circleDiagramItem[] circleDiagram, List<node> pivots, int timeRow, List<node> links,  double[, ,] coriolis, double[, ,] slipvelocity)
        {
            double slipvelocityx = 0.0, slipvelocityy = 0.0;
            node n1 = null, n2 = null, n3 = null;
            double omega = 0.0;
            //slider on a link

            for (int i = 0; i < p; i++)
            {
                if (pivots[i].localLabels.Contains("slideronalink") || (pivots[i].localLabels.Contains("pis") && !(pivots[i].localLabels.Contains("ground"))))
                {

                    foreach (arc a1 in pivots[i].arcs)
                    {
                        if (a1.otherNode(pivots[i]).localLabels.Contains("slideronalink_conn") || a1.otherNode(pivots[i]).localLabels.Contains("pis_conn"))
                            n1 = a1.otherNode(pivots[i]);
                    }

                    foreach (arc a2 in n1.arcs)
                    {
                        if (!(a2.otherNode(n1).localLabels.Contains("slideronalink") && a2.otherNode(n1).localLabels.Contains("pivot")))
                            n2 = a2.otherNode(n1);
                    }

                    n3 = pivots[i];

                    double slope0 = 0.0, slope1 = 0.0;

                    slope0 = (n2.Y - n3.Y) / (n2.X - n3.Y);//use this in one equation -using n1

                    slope1 = -1 / slope0; //use this in other equation using n2;

                    //get the intersection point:

                    double x = PivotParameters[i, timeRow, 2];
                    double y = PivotParameters[i, timeRow, 3];

                    double B = y - slope0 * x;
                    double C = n3.Y - slope1 * n3.X;

                    double x1 = -(B - C) / (slope0 - slope1);
                    double y1 = slope0 * x1 + B;

                    //values to be used are X,Y and x1,y1

                    //the perpendicular distance between the two points should give the slip velocity in the case of slider on a link 
                    //the code works for pis too unless the slot is different from the link orientation


                    //using the X,x1 ; Y,y1 and omega of the particular "slider or pis on a link" link, calculate the velocity of slip and 
                    //add it to the velocity of the particular point

                    slipvelocityx = -(x - x1);
                    slipvelocityy = -(y - y1);

                    //we have got the X difference and Y difference
                    //we need to take cross product of omega and the distance


                    //the statement below is not required at all
                    foreach (circleDiagramItem c in circleDiagram)
                        if (c.link2 == n1 && !(double.IsNaN(c.speed)) && c.link1.localLabels.Contains("ground"))
                            omega = c.speed;


                    //     slipvelocity[i, timeRow, 0] = slipvelocityx;
                    //     slipvelocity[i, timeRow, 1] = slipvelocityy;


                    //      pivotParameters[i, timeRow, 2] += slipvelocityy;
                    //      pivotParameters[i, timeRow, 3] += slipvelocityx;

                    //calculating the coriolis componenet
                    //which is 2xOmegaxslip velocity

                    coriolis[i, timeRow, 0] = -2 * omega * PivotParameters[i, timeRow, 3];
                    coriolis[i, timeRow, 1] = 2 * omega * PivotParameters[i, timeRow, 2];


                }

            }

        }
        #endregion
        #region Diagonalization (not in real sense)
        private bool diag(double[,] accelerationMatrix_New)
        {

            for (int i = 0; i < accelerationMatrix_New.GetLength(0); i++)
            {
                if (accelerationMatrix_New[i, i] == 0)
                    return true;
            }
            return false;

        }

        #endregion

    }


}