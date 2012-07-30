#region
using System;
using OptimizationToolbox;

#endregion

namespace PlanarMechanismSimulator
    //at time t=0; all acceleration and velocity are zero
{
    public partial class Simulator : IDependentAnalysis
    {
        private Boolean findAccelerationAnalytically(double currentTime, bool forwardInTime)
        {
            return false;
            throw new NotImplementedException();
        }
        //#region New Acceleration Determination

        //private Boolean findAccelerationNew(double[,] LinkParams, double[,] PivotParams)
        //{
        //    double iAlpha = 0;

        //    #region Filling up values in the different rows

        //    var maxCol = unknowns.GetLength(0);
        //    var maxRow = Omeg.GetLength(0);
        //    var accelerationMatrix = new double[maxRow,maxCol];
        //    var wsquareMatrix = new double[maxRow,1];
        //    var no = 0;

        //    for (var i = 0; i < maxRow; i++) //shouldn't this be 2*no of equations taking into account X and Y
        //    {
        //        var d = Omeg[i].dir;
        //        var link = ((LinkDynamicMatrixTerm) Omeg[i]).belongsTo;
        //        var pivot0 = ((PivotDynamicMatrixTerm) coriolis1[i]).belongsFrom;
        //        var pivot1 = ((PivotDynamicMatrixTerm) coriolis1[i]).belongsTo;

        //        for (no = 0; no < n; no++)
        //            if (links[no] == link)
        //                break;
        //        //using "no" we can obtain coriolis component and 
        //        //also omega from linkparameters
        //        if (d == Direction.X)
        //            wsquareMatrix[i, 0] =
        //                -(LinkParams[no, 0]*LinkParams[no, 0]*(-pivot1.X + pivot0.X)) +
        //                coriolis[no, 0];
        //        if (d == Direction.Y)
        //            wsquareMatrix[i, 0] =
        //                -(LinkParams[no, 0]*LinkParams[no, 0]*(-pivot1.Y + pivot0.Y)) +
        //                coriolis[no, 1];

        //        //we also need to alpha X (link length)  - in terms of X and Y - but this will be in acceleration matrix as part of separate equation
        //        for (var j = 0; j < maxCol; j++)
        //        {
        //            if (unknowns[j].dir == d)
        //                //&& unknowns[j].defaultValue != 0.0
        //            {
        //                if (((PivotDynamicMatrixTerm) unknowns[j]).belongsTo == pivot0)
        //                    accelerationMatrix[i, j] = 1;
        //                if (((PivotDynamicMatrixTerm) unknowns[j]).belongsTo == pivot1)
        //                    accelerationMatrix[i, j] = -1;
        //            }

        //            if ((unknowns[j] is LinkDynamicMatrixTerm) &&
        //                unknowns[j].type == DynamicType.angularAcceleration &&
        //                ((LinkDynamicMatrixTerm) unknowns[j]).belongsTo == link)
        //                // && unknowns[j].defaultValue != 0
        //                if (d == Direction.X)
        //                    accelerationMatrix[i, j] = (-pivot1.Y + pivot0.Y);
        //                else if (d == Direction.Y)
        //                    accelerationMatrix[i, j] = -(-pivot1.X + pivot0.X);
        //            if (unknowns[j].type == DynamicType.radialAcceleration)
        //                //&& unknowns[j].defaultValue != 0
        //            {
        //                //this is for a PIS // slider on a link

        //                if (((PivotDynamicMatrixTerm) Omeg[i]).belongsTo.localLabels.Contains("pis_conn"))
        //                    accelerationMatrix[i, j] = -1;
        //            }
        //        }
        //    }

        //    #endregion

        //    #region Collapsing matrices - Not used any more

        //    //collect all indices where 

        //    //List<int> indexNos = new List<int>();

        //    //for(int i=0;i<maxCol;i++)
        //    //{
        //    //    if (unknowns[i].type == DynamicType.absoluteAcceleration)
        //    //    {
        //    //        if (unknowns[i].belongsTo.localLabels.Contains("ground"))
        //    //            indexNos.Add(i);
        //    //    }

        //    //    if (unknowns[i].type == DynamicType.angularAcceleration)
        //    //    {
        //    //        if (unknowns[i].belongsTo.localLabels.Contains("ground") || unknowns[i].belongsTo.localLabels.Contains("input"))
        //    //            indexNos.Add(i);
        //    //    }

        //    //    if (unknowns[i].type == DynamicType.radialAcceleration)
        //    //    {
        //    //        if (unknowns[i].defaultValue == 0)
        //    //            indexNos.Add(i);
        //    //    }

        //    //}  
        //    ////we shall collapse the columns in the acceleration matrix

        //    //int nooftermstoremove = indexNos.Count;
        //    //int w=0,v=0;

        //    //double[,] ColColapAMatrix = new double[2 * numEqs, maxCol - nooftermstoremove];

        //    //for (int i = 0; i < 2 * numEqs; i++)
        //    //{
        //    //    for (int j = 0; j < maxCol; j++)
        //    //    {
        //    //        foreach (int k in indexNos)
        //    //            if (j == k)
        //    //                break;
        //    //            else
        //    //            {
        //    //                ColColapAMatrix[w, v] = accelerationMatrix[i, j];
        //    //                v++;
        //    //            }
        //    //    }
        //    //    w++;
        //    //}

        //    ////row collapsing - collapsing ground link
        //    //List<int> rowindex = new List<int>();
        //    //for (int i = 0; i < 2 * numEqs; i++)
        //    //    if (Omeg[i].belongsTo.localLabels.Contains("ground"))
        //    //        rowindex.Add(i);

        //    //w = 0; v = 0;
        //    //double[,] FinalAMatrix = new double[2*numEqs-rowindex.Count,maxCol-nooftermstoremove];
        //    //double[,] FinalWMatrix = new double[2*numEqs-rowindex.Count,1];
        //    //for (int i = 0; i < 2 * numEqs; i++)
        //    //    foreach (int k in rowindex)
        //    //        if (i == k)
        //    //            break;
        //    //        else
        //    //        {
        //    //            for (int j = 0; j < maxCol - nooftermstoremove; j++)
        //    //                FinalAMatrix[w, j] = ColColapAMatrix[i, j];

        //    //            FinalWMatrix[w, 0] = wsquareMatrix[i, 0];

        //    //            w++;
        //    //        }

        //    #endregion

        //    #region Removing the input ang accln and adding it to the other omegaSquareMatrix

        //    //apart from removing input ang accln, we also need to remove slider's (X/Y) and pis_conn link should only
        //    //contain slip acceleration - why not remove in set up itself?


        //    //there is only input - which means there is only one column to be removed
        //    //in cases now, we have alpha = 0; which may not be the same

        //    //creating a new matrix for acceleration

        //    var accelerationMatrix_New = new double[maxRow,maxCol - 1];
        //    var inputAlphaColumn = new double[maxRow,1];

        //    //getting the index of unknowns
        //    var indexno = 0;
        //    for (var i = 0; i < maxCol; i++)
        //        if (unknowns[i] is LinkDynamicMatrixTerm)

        //            foreach (circleDiagramItem c in circleDiagram)
        //                if (c.link1.IsGround && c.speed == iOmega)
        //                    if (c.link2 == ((LinkDynamicMatrixTerm) unknowns[i]).belongsTo)
        //                    {
        //                        indexno = i;
        //                        break;
        //                    }


        //    //got to have new unknowns matrix
        //    var unknowns_New = new DynamicMatrixTerm[maxCol - 1];
        //    var z = 0;
        //    for (var i = 0; i < maxCol; i++)
        //    {
        //        if (i != indexno)
        //        {
        //            unknowns_New[z] = unknowns[i];
        //            z++;
        //        }
        //    }

        //    //adding the acceleration matrix to the new one and the lone column to inputAlphaColumn
        //    int w = 0, v = 0;
        //    for (var i = 0; i < maxRow; i++)
        //    {
        //        v = 0;
        //        for (var j = 0; j < maxCol; j++)
        //        {
        //            if (j != indexno)
        //            {
        //                accelerationMatrix_New[w, v] = accelerationMatrix[i, j];
        //                v++;
        //            }
        //            else
        //                inputAlphaColumn[w, 0] = accelerationMatrix[i, j];
        //        }
        //        w++;
        //    }

        //    //adding the inputAlphaColumn to the wsquareMatrix

        //    var New_wsquareMatrix = new double[maxRow,1];

        //    for (var i = 0; i < maxRow; i++)
        //        New_wsquareMatrix[i, 0] = wsquareMatrix[i, 0] + iAlpha*inputAlphaColumn[i, 0];

        //    #endregion

        //    var result = new double[maxRow,1];

        //    #region Making row diagonal

        //    if (maxRow > 9)
        //    {
        //        while (diag(accelerationMatrix_New))
        //        {
        //            var store_1 = 0;
        //            for (var i = 0; i < accelerationMatrix_New.GetLength(0); i++)
        //            {
        //                for (var j = 0; j < accelerationMatrix_New.GetLength(0); j++)
        //                {
        //                    double kk = accelerationMatrix_New.GetLength(0);


        //                    if (i == j && (accelerationMatrix_New[i, j] == 0))
        //                    {
        //                        //now i know that the diagonal in that particular element is 0
        //                        //search for another row that does not have zero in the same column 
        //                        //and make sure that this index does not show up again...

        //                        for (var k = 0; k < accelerationMatrix_New.GetLength(0); k++)
        //                            if (accelerationMatrix_New[k, j] != 0)
        //                            {
        //                                //now store this row
        //                                store_1 = k;
        //                                break;
        //                            }

        //                        //interchange both these two rows
        //                        //store these two values in two separate lists
        //                        //and then replace them in the matrix back
        //                        var list1 = StarMath.GetRow(i, accelerationMatrix_New);
        //                        var list2 = StarMath.GetRow(store_1, accelerationMatrix_New);
        //                        //now replacing them back to the acceleration matrix
        //                        StarMath.SetRow(i, accelerationMatrix_New, list2);
        //                        StarMath.SetRow(store_1, accelerationMatrix_New, list1);

        //                        //List<double> list1 = new List<double>();
        //                        //List<double> list2 = new List<double>();
        //                        //for (int m = 0; m < accelerationMatrix_New.GetLength(0); m++)
        //                        //    list1.Add(accelerationMatrix_New[i, m]);
        //                        //for (int m = 0; m < accelerationMatrix_New.GetLength(0); m++)
        //                        //    list2.Add(accelerationMatrix_New[store_1, m]);
        //                        //for (int m = 0; m < accelerationMatrix_New.GetLength(0); m++)
        //                        //    accelerationMatrix_New[i, m] = list2[m];
        //                        //for (int m = 0; m < accelerationMatrix_New.GetLength(0); m++)
        //                        //    accelerationMatrix_New[store_1, m] = list1[m];

        //                        //we also need to change the omega matrix and also the unknowns matrix too as we change the order
        //                        //of the acceleration matrix

        //                        var w_1 = New_wsquareMatrix[i, 0];
        //                        var w_2 = New_wsquareMatrix[store_1, 0];
        //                        var a1 = unknowns_New[i];
        //                        var a2 = unknowns_New[store_1];


        //                        //now replace

        //                        New_wsquareMatrix[i, 0] = w_2;
        //                        New_wsquareMatrix[store_1, 0] = w_1;

        //                        unknowns_New[i] = a2;
        //                        unknowns_New[store_1] = a1;
        //                    }
        //                }
        //            }
        //        }

        //        var inversematrix = StarMath.inverse(accelerationMatrix_New);
        //        var A = StarMath.multiply(accelerationMatrix_New, inversematrix);
        //        var error = StarMath.norm2(StarMath.subtract(A, StarMath.makeIdentity(A.GetLength(0))));
        //        // if (double.IsNaN(error) || error> epsilon) Campbell: add something like this
        //        result = StarMath.multiply(inversematrix, New_wsquareMatrix);

        //        #endregion
        //    }

        //    else
        //    {
        //        var cofactorMatrix = new double[maxRow,maxRow];

        //        for (var i = 0; i < maxRow; i++)
        //        {
        //            for (var j = 0; j < maxRow; j++)
        //            {
        //                cofactorMatrix[i, j] = Math.Pow((-1), (i + j))*MatrixMinor(accelerationMatrix_New, i, j, maxRow);
        //            }
        //        }


        //        var determinant = 0.0;

        //        determinant = MatrixDeterminant(accelerationMatrix_New, maxRow); //passing matrix and order

        //        if (determinant != 0)
        //        {
        //        }

        //        var inversematrix = new double[maxRow,maxRow];

        //        inversematrix = StarMath.multiply((1/determinant), (StarMath.transpose(cofactorMatrix)));


        //        var determinant1 = 0.0;

        //        determinant1 = MatrixDeterminant(inversematrix, maxRow);

        //        #region Equation Solving

        //        var result1 = new double[maxRow,1];

        //        //it is required to ensure that diagonals are not zero for the StarMath.Inverse routine to work
        //        //so we are going to interchange rows 
        //        //ref: http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?Effective_UBLAS/Matrix_Inversion
        //        //logic needs to be worked out with different types of matrices
        //        //and also determine whether multiple iterations are required

        //        var rowPlaceHolders = new double[maxRow,1];


        //        //for (int i = 0; i < maxRow; i++)
        //        //    for (int j = 0; j < maxRow; j++)
        //        //    {
        //        //        if (i == j)
        //        //        {
        //        //            if (accelerationMatrix_New[i, j] == 0)
        //        //            {
        //        //                //need to find another row that has that column non zero
        //        //                //add the new row in the place of this and the existing in the place of new
        //        //                int existingrow = i;
        //        //                int newRow = MatrixSwapFunction(accelerationMatrix_New, i, j, maxRow);
        //        //                rowPlaceHolders[i, 0] = newRow;
        //        //                rowPlaceHolders[newRow, 0] = i;
        //        //                double[,] iRow = new double[1, maxRow];
        //        //                double[,] pRow = new double[1, maxRow];
        //        //                double iWM = New_wsquareMatrix[i, 0];
        //        //                double pWM = New_wsquareMatrix[newRow, 0];


        //        //                for (int m = 0; m < maxRow; m++)
        //        //                {
        //        //                    iRow[0, m] = accelerationMatrix_New[i, m];
        //        //                    pRow[0, m] = accelerationMatrix_New[newRow, m];
        //        //                }

        //        //                for (int m = 0; m < maxRow; m++)
        //        //                {
        //        //                    accelerationMatrix_New[newRow, m] = iRow[0, m];
        //        //                    accelerationMatrix_New[i, m] = pRow[0, m];
        //        //                }

        //        //                New_wsquareMatrix[newRow, 0] = iWM;
        //        //                New_wsquareMatrix[i, 0] = pWM;


        //        //            }
        //        //            else
        //        //            {
        //        //                rowPlaceHolders[i, 0] = i;
        //        //            }
        //        //        }
        //        //    }

        //        result = StarMath.multiply(inversematrix, New_wsquareMatrix);

        //        #endregion
        //    }

        //    #region Storing values in the respective pivotparameters and link parameters

        //    //store results in the appropriate 
        //    //code below is typically for a four bar
        //    //need to correct it


        //    for (var i = 0; i < p; i++)
        //    {
        //        if (!joints[i].IsGround)
        //        {
        //            for (var b = 0; b < maxRow; b++)
        //                if (unknowns_New[b] is PivotDynamicMatrixTerm &&
        //                    ((PivotDynamicMatrixTerm) unknowns_New[b]).belongsTo == joints[i])
        //                {
        //                    if (unknowns_New[b].dir == Direction.X)
        //                        PivotParams[i, 4] = PivotParams[i, 4] + result[b, 0];
        //                    else if (unknowns_New[b].dir == Direction.Y)
        //                        PivotParams[i, 5] = PivotParams[i, 5] + result[b, 0];
        //                }
        //        }
        //    }

        //    for (var i = 0; i < n; i++)
        //    {
        //        if (!links[i].IsGround)
        //        {
        //            for (var b = 0; b < maxRow; b++)
        //                if (unknowns_New[b] is LinkDynamicMatrixTerm &&
        //                    ((LinkDynamicMatrixTerm) unknowns_New[b]).belongsTo == links[i])
        //                {
        //                    LinkParams[i, 1] = result[b, 0];
        //                }
        //        }
        //    }

        //    //also add the value to the slider too. this is not done in the previous code

        //    for (var i = 0; i < p; i++)
        //    {
        //        if (joints[i].Contains("slider_conn"))
        //        {
        //            var accln1 = PivotParams[i, 4];
        //            var accln2 = PivotParams[i, 5];

        //            foreach (var aa in joints[i].Links)
        //                foreach (var otherPivot in aa.Pivots.Where(op => op != joints[i]))
        //                {
        //                    if (otherPivot.PivotType == PivotTypes.PX || otherPivot.PivotType == PivotTypes.PY)
        //                    {
        //                        PivotParams[otherPivot.index, 4] = accln1;
        //                        PivotParams[otherPivot.index, 5] = accln2;
        //                    }
        //                }
        //        }
        //        #endregion
        //    }
        //}

        //#endregion

        //#region MatrixMath Determinant

        //private double MatrixDeterminant(double[,] accelerationMatrix_New, int maxRow)
        //{
        //    //this is a general matrix determinant function for a nxn matrix
        //    //we have a square matrix which will be used
        //    //once the matrix is obtained we need to use a function which will recursively obtain the minor/cofactor of the particular element
        //    //once the cofactor size is 2 - i shall add find the determinant of a 2x2 matrix

        //    var det = 0.0;
        //    int i = 0, j;


        //    for (j = 0; j < maxRow; j++)
        //    {
        //        //passing the minor matrix too in this

        //        det += Math.Pow((-1), (i + j))*accelerationMatrix_New[i, j]*
        //               MatrixMinor(accelerationMatrix_New, i, j, maxRow);
        //    }


        //    return (det);
        //}

        //private double MatrixMinor(double[,] accelerationMatrix_New, int i, int j, int maxRow)
        //{
        //    var x = 0.0;
        //    //got to form a new matrix without the elements of ith row and jth column
        //    var newmatrix = new double[maxRow - 1,maxRow - 1];
        //    int m = 0, n = 0;
        //    for (var k = 0; k < maxRow; k++)
        //    {
        //        if (k != i)
        //        {
        //            n = 0;
        //            for (var l = 0; l < maxRow; l++)
        //            {
        //                if (l != j)
        //                {
        //                    newmatrix[m, n] = accelerationMatrix_New[k, l];
        //                    n++;
        //                }
        //            }
        //            m++;
        //        }
        //    }
        //    //now we have the reduced matrix
        //    var order = maxRow - 1;
        //    if (order == 2)
        //    {
        //        x = newmatrix[0, 0]*newmatrix[1, 1] - newmatrix[0, 1]*newmatrix[1, 0];
        //    }
        //    else
        //    {
        //        var k = 0;
        //        for (var l = 0; l < n; l++)
        //        {
        //            x += Math.Pow((-1), (k + l))*newmatrix[k, l]*MatrixMinor(newmatrix, k, l, order);
        //        }
        //    }


        //    return x;
        //}

        //#endregion

        //#region MatrixMath Row Swap Function

        //private int MatrixSwapFunction(double[,] accelerationMatrix_New, int i, int j, int maxRow)
        //{
        //    var p = 0;
        //    for (var k = 0; k < maxRow; k++)
        //        if (accelerationMatrix_New[k, j] != 0)
        //        {
        //            p = k;
        //            break;
        //        }
        //    //got to swap accelerationmatrix rows i and p
        //    //we can save these two rows in separate row matrices and then add them


        //    return p;
        //}

        //#endregion

        //#region Function: Slip Velocities & Coriolis Component and update the respective velocities - completely horizontal/vertical PIS/slideronalink

        //private void findLinearSlipVelocities(double[,] PivotParams)
        //{
        //    double slipvelocityx = 0.0, slipvelocityy = 0.0;
        //    //node n1 = null, n2 = null, n3 = null;
        //    var omega = 0.0;
        //    //slider on a link

        //    for (var i = 0; i < p; i++)
        //    {
        //        var piv = joints[i];
        //        if (!piv.IsGround && piv.PivotType==PivotTypes.RPX || piv.PivotType==PivotTypes.RPY)
        //        {
        //            var link1 = piv.Links.Find(a => (a.Contains("slideronalink_conn") ||
        //                                                    a.Contains("pis_conn")));
        //            var otherPiv = link1.Pivots.Find(a => !(a.Contains("slideronalink")));
        //            double slope0 = 0.0, slope1 = 0.0;

        //            slope0 = (otherPiv.Y - piv.Y)/(otherPiv.X - piv.Y); //use this in one equation -using n1

        //            slope1 = -1/slope0; //use this in other equation using n2;

        //            //get the intersection point:

        //            var x = PivotParams[i,  2];
        //            var y = PivotParams[i,  3];

        //            var B = y - slope0*x;
        //            var C = piv.Y - slope1*piv.X;

        //            var x1 = -(B - C)/(slope0 - slope1);
        //            var y1 = slope0*x1 + B;

        //            //values to be used are X,Y and x1,y1

        //            //the perpendicular distance between the two points should give the slip velocity in the case of slider on a link 
        //            //the code works for pis too unless the slot is different from the link orientation


        //            //using the X,x1 ; Y,y1 and omega of the particular "slider or pis on a link" link, calculate the velocity of slip and 
        //            //add it to the velocity of the particular point

        //            slipvelocityx = -(x - x1);
        //            slipvelocityy = -(y - y1);

        //            //we have got the X difference and Y difference
        //            //we need to take cross product of omega and the distance


        //            //the statement below is not required at all
        //            foreach (circleDiagramItem c in circleDiagram)
        //                if (c.link2 == link1 && !(double.IsNaN(c.speed)) && c.link1.IsGround)
        //                    omega = c.speed;


        //            //     slipvelocity[i, 0] = slipvelocityx;
        //            //     slipvelocity[i, 1] = slipvelocityy;


        //            //      pivotParameters[i, 2] += slipvelocityy;
        //            //      pivotParameters[i, 3] += slipvelocityx;

        //            //calculating the coriolis componenet
        //            //which is 2xOmegaxslip velocity

        //            coriolis[i, 0] = -2*omega*PivotParams[i,  3];
        //            coriolis[i, 1] = 2*omega*PivotParams[i,  2];
        //        }
        //    }
        //}

        //#endregion

        //#region Diagonalization (not in real sense)

        //private bool diag(double[,] accelerationMatrix_New)
        //{
        //    for (var i = 0; i < accelerationMatrix_New.GetLength(0); i++)
        //    {
        //        if (accelerationMatrix_New[i, i] == 0)
        //            return true;
        //    }
        //    return false;
        //}

        //#endregion
    }
}