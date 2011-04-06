using System;
using System.Collections.Generic;
using System.IO;
using GraphSynth.Representation;
using OptimizationToolbox;

namespace PlanarMechanismSimulator

    //at time t=0; all acceleration and velocity are zero
{
    public partial class MechSimulation : IDependentAnalysis
    {

        #region Constructor
        /// <summary>
        /// Initializes a new instance of the <see cref="MechSimulation"/> class.
        /// </summary>
        /// <param name="inputSweepAngle">The input sweep angle.</param>
        /// <param name="numSteps">The number of steps.</param>
        /// <param name="graph">The graph.</param>
        public MechSimulation(double inputSweepAngle = 2 * Math.PI, int numSteps = 12, designGraph graph = null)
        {
            this.inputSweepAngle = inputSweepAngle;
            this.numSteps = numSteps;
            if (graph != null) Graph = graph;
        }

        #endregion

        #region Fields

        public double[, ,] PivotParameters { get; private set; }
        public double[, ,] LinkParameters { get; private set; }


        #region Given by the Design Problem (these are set only in the constructor)

        /// <summary>
        /// The total angle that the input is sweeped. It is to be specified in radians.
        /// </summary>
        private readonly double inputSweepAngle;
        /// <summary>
        /// The (minimum) number of steps between startAngle and endAngle.
        /// </summary>
        private readonly int numSteps;

        #endregion

        #region Given by the Configuration (these are set in updateInternalParameters)
        designGraph g;
        public designGraph Graph
        {
            set
            {
                if ((g != null) && (g.Equals(value))) return;
                g = value;
                SetUpFieldsForNewGraph();
            }
        }
        int n, p, numEqs;//no. of links, pivots, and equations needed
        int sizeofCDI;

        private List<node> links;
        private List<node> pivots;
        node groundLink;
        private node inputpivot;
        private circleDiagramItem[] circleDiagram;
        private double[, ,] coriolis;
        private double[, ,] slipvelocity;
        private double[, ,] slipacceleration;
        DynamicMatrixTerm[] unknowns;
        DynamicMatrixTerm[] coriolis_1;
        DynamicMatrixTerm[] omeg_1;
        private void SetUpFieldsForNewGraph()
        {
            #region Setup
            links = new List<node>(); //create a list of LINKS
            pivots = new List<node>();//create a list of pivots

            foreach (node nde in g.nodes)
            {
                if (nde.localLabels.Contains("link"))
                {
                    links.Add(nde);//adding all links to LINKS List
                    if (nde.localLabels.Contains("ground")) groundLink = nde;

                }
                else if (nde.localLabels.Contains("pivot")) pivots.Add(nde);//adding all pivots to PIVOTS List
                /* I'm throwing an exception (crashing the program) if I find a node that is neither
                 * link nor pivot. Later this can be removed, but the following code works on the assumption
                 * that all nodes fall into one category or the other. */
                else throw new Exception();
            }

            n = links.Count;//count the no of links in the system
            p = pivots.Count;//count the no of pivots in the system
            numEqs = 0;
            foreach (node l in links) numEqs += l.arcs.Count - 1;


            #region fill up link spots in CircleDiagram
            sizeofCDI = (n * (n - 1) / 2);
            circleDiagram = new circleDiagramItem[sizeofCDI];//size is equal to no of instant centers
            //code below gives the link indices for each circleDiagram Item and is assigning the instant 
            //center pairs - these may not be necessarily starting from input - but in the way the graph
            //and the particular linkage is created
            //also need not be in the order like 12 / 13 - it could be 31 / 21 and so on
            //the code below creates 12/13/14;23/24;34 - say for a 4 bar mechanism
            //creates an ordered pair of instant centers
            int q = 0;
            for (int ii = 0; ii != n - 1; ii++)
            {
                for (int j = ii + 1; j != n; j++)
                {
                    circleDiagram[q] = new circleDiagramItem();//coding requirement
                    if (links[j] == groundLink)
                    {
                        circleDiagram[q].link1 = links[j];
                        circleDiagram[q].link2 = links[ii];
                    }
                    else
                    {
                        circleDiagram[q].link1 = links[ii];
                        circleDiagram[q].link2 = links[j];
                    }
                    q++;
                }
            }

            #endregion

            /*Declaring pivotParameters - containing X,Y,Vx,Vy,Ax,Ay
             *Declaring linkParameters - containing Omega,Alpha   - not required now */

            //I changed numtimesteps to 1 - don't forget to change it back to numTimesteps
            PivotParameters = new double[p, numSteps, 6];
            LinkParameters = new double[n, numSteps, 2];
            coriolis = new double[p, numSteps, 2];

            #region Forming the acceleration node matrix

            // change to lists
            List<DynamicMatrixTerm> coriolis1 = new List<DynamicMatrixTerm>();
            List<DynamicMatrixTerm> Omeg = new List<DynamicMatrixTerm>();

            // DynamicMatrixTerm[] coriolis1 = new DynamicMatrixTerm[2 * numEqs];//shouldn't this be 2*no of equations?
            // DynamicMatrixTerm[] Omeg = new DynamicMatrixTerm[2 * numEqs];
            List<DynamicMatrixTerm> unknownsList = new List<DynamicMatrixTerm>();

            //List<int> locationofPivot0 = new List<int>();
            //List<int> locationofPivot1 = new List<int>();
            //int row = 0;
            //int row2 = 0;
            for (int i = 0; i < n; i++)
            {
                int NOP = links[i].arcs.Count;
                node link = links[i];
                node pivot0 = ((arc)link.arcs[0]).otherNode(link);

                //we are not adding slider to acceleration determination - this will be copied from the velocity of slider_conn pivot
                if (!pivot0.localLabels.Contains("sliderh") && !pivot0.localLabels.Contains("sliderv"))
                    addToUnknowns(pivot0, link, unknownsList);

                if (!link.localLabels.Contains("ground") && !link.localLabels.Contains("slider_conn"))
                {
                    unknownsList.Add(new DynamicMatrixTerm()
                    {
                        belongsTo = link,
                        type = DynamicType.angularAcceleration
                    });
                    //unknownsList[row2].belongsTo = link;
                    //unknownsList[row2].type = DynamicType.angularAcceleration;
                    //if (link.localLabels.Contains("ground")) unknownsList[row2].defaultValue = 0.0;
                    //else unknownsList[row2].defaultValue = double.NaN;
                    //row2++;
                }
                for (int j = 1; j < NOP; j++)
                {
                    node pivot1 = ((arc)link.arcs[j]).otherNode(link);

                    if (!link.localLabels.Contains("ground") && !link.localLabels.Contains("slider_conn"))
                    {
                        Omeg.Add(new DynamicMatrixTerm()
                        {
                            belongsTo = link,
                            type = DynamicType.angularVelocity,
                            dir = Direction.X
                        });

                        //if (link.localLabels.Contains("ground")) Omeg[row].defaultValue = 0.0;
                        //else Omeg[row].defaultValue = double.NaN;

                        if ((pivot0.localLabels.Contains("pis")) || (pivot1.localLabels.Contains("pis")) ||
                            (pivot0.localLabels.Contains("slideronalink")) || (pivot1.localLabels.Contains("slideronalink")))
                        {
                            if (link.localLabels.Contains("pis_conn"))

                                coriolis1.Add(new DynamicMatrixTerm()
                                {

                                    belongsFrom = pivot0,
                                    belongsTo = pivot1,
                                    type = DynamicType.radialVelocity,
                                    dir = Direction.X
                                });
                            else
                                coriolis1.Add(new DynamicMatrixTerm()
                                {

                                    belongsFrom = pivot0,
                                    belongsTo = pivot1,
                                    type = DynamicType.radialVelocity,
                                    dir = Direction.X,
                                    defaultValue = 0.0
                                });
                        }
                        else
                            coriolis1.Add(new DynamicMatrixTerm()
                            {

                                belongsFrom = pivot0,
                                belongsTo = pivot1,
                                type = DynamicType.radialVelocity,
                                dir = Direction.X,
                                defaultValue = 0.0
                            });
                    }

                    //if ((pivot0.localLabels.Contains("pis")) || (pivot1.localLabels.Contains("pis")) ||
                    //    (pivot0.localLabels.Contains("slideronalink")) || (pivot1.localLabels.Contains("slideronalink")))
                    //    coriolis1[row].defaultValue = double.NaN;
                    //else coriolis1[row].defaultValue = 0.0;

                    if (!link.localLabels.Contains("ground") && !link.localLabels.Contains("slider_conn"))
                    {
                        Omeg.Add(new DynamicMatrixTerm()
                        {
                            belongsTo = link,
                            type = DynamicType.angularVelocity,
                            dir = Direction.Y
                        });


                        //if (link.localLabels.Contains("ground")) Omeg[row].defaultValue = 0.0;
                        //else Omeg[row].defaultValue = double.NaN;

                        if ((pivot0.localLabels.Contains("pis")) || (pivot1.localLabels.Contains("pis")) ||
                            (pivot0.localLabels.Contains("slideronalink")) || (pivot1.localLabels.Contains("slideronalink")))
                        {
                            if (link.localLabels.Contains("pis_conn"))
                            {
                                coriolis1.Add(new DynamicMatrixTerm()
                                {
                                    belongsFrom = pivot0,
                                    belongsTo = pivot1,
                                    type = DynamicType.radialVelocity,
                                    dir = Direction.Y
                                });
                            }
                            else
                                coriolis1.Add(new DynamicMatrixTerm()
                                {

                                    belongsFrom = pivot0,
                                    belongsTo = pivot1,
                                    type = DynamicType.radialVelocity,
                                    dir = Direction.X,
                                    defaultValue = 0.0
                                });
                        }
                        else
                            coriolis1.Add(new DynamicMatrixTerm()
                            {
                                belongsFrom = pivot0,
                                belongsTo = pivot1,
                                type = DynamicType.radialVelocity,
                                dir = Direction.Y,
                                defaultValue = 0.0
                            });
                    }
                    //    coriolis1[row].defaultValue = double.NaN;
                    //else coriolis1[row].defaultValue = 0.0;
                    //row++;

                    if ((pivot0.localLabels.Contains("pis")) || (pivot1.localLabels.Contains("pis")) ||
                        (pivot0.localLabels.Contains("slideronalink")) || (pivot1.localLabels.Contains("slideronalink")))
                    {
                        if (link.localLabels.Contains("pis_conn"))
                            unknownsList.Add(new DynamicMatrixTerm()
                            {
                                belongsFrom = pivot0,
                                belongsTo = pivot1,
                                type = DynamicType.radialAcceleration
                            });
                    }


                    //if ((pivot0.localLabels.Contains("pis")) || (pivot1.localLabels.Contains("pis")) ||
                    //    (pivot0.localLabels.Contains("slideronalink")) || (pivot1.localLabels.Contains("slideronalink")))
                    //    unknownsList[row2].defaultValue = double.NaN;
                    //else unknownsList[row2].defaultValue = 0.0;
                    //row2++;
                    if (!pivot1.localLabels.Contains("sliderh") && !pivot1.localLabels.Contains("sliderv"))
                        addToUnknowns(pivot1, link, unknownsList);
                }
            }

            // cast others into arrays

            unknowns = unknownsList.ToArray();
            coriolis_1 = coriolis1.ToArray();
            omeg_1 = Omeg.ToArray();
            #endregion

            slipvelocity = new double[p, numSteps, 2];
            slipacceleration = new double[p, numSteps, 2];

            #endregion
        }
        #endregion

        private double inputSpeed = 1;
        public double InputSpeed
        {
            get
            {
                return inputSpeed;
            }
            set { inputSpeed = value; }
        }
        #endregion


        // this is the required overload for IDependentAnalysis. This is used in an
        // optimization when new values for the pivot locations are being generated.
        public void calculate(double[] x)
        {
            int k = 0;
            for (int ii = 0; ii < p; ii++)
            {
                if (pivots[ii].localLabels.Contains("ground") && pivots[ii].localLabels.Contains("input"))
                {
                    PivotParameters[ii, 0, 0] = pivots[ii].X = 0.0;
                    PivotParameters[ii, 0, 1] = pivots[ii].Y = 0.0;
                }
                else if (pivots[ii].localLabels.Contains("ground") && pivots[ii].localLabels.Contains("ip"))
                {
                    PivotParameters[ii, 0, 0] = pivots[ii].X = 6.0;
                    PivotParameters[ii, 0, 1] = pivots[ii].Y = 0.0;

                }
                ////else if (pivots[ii].localLabels.Contains("output"))
                ////{
                ////    PivotParameters[ii, 0, 0] = pivots[ii].X = 1.874099;
                ////    PivotParameters[ii, 0, 1] = pivots[ii].Y = 7.998559;

                ////}
                else
               {
                    PivotParameters[ii, 0, 0] = pivots[ii].X = x[k++] ;
                    PivotParameters[ii, 0, 1] = pivots[ii].Y = x[k++];
               }
            }
            simulate();
        }

        // If you are just simulating a given graph, you can skip right to here, 
        // and use the pivot positions that already exist.
        public void calculate()
        {
            #region Assign Pivot X and Y to the matrix pivotParameters

            for (int ii = 0; ii < p; ii++)
            {
                PivotParameters[ii, 0, 0] = pivots[ii].X;
                PivotParameters[ii, 0, 1] = pivots[ii].Y;
            }
            simulate();
        }

            #endregion


        private void simulate()
        {

            double[,] pivotLengths = findlinkLengths();
            Boolean success = true;

            findImmediatePivots();

            #region For loop functions
            //            for (sa = 0; sa <= endAngle; sa += ((endAngle) / numTimeSteps))
            for (int timeRow = 0; timeRow < numSteps; timeRow++)
            {
                //findImmediateICs();
                //success = findSecondaryICs();
                //if (success)
                //{
                //    findAngularVelocities(timeRow);
                //    findLinearVelocities(timeRow);
                //    //check slip velocities
                //    findLinearSlipVelocities(timeRow);
                //    //find slip velocities and update
                //}
                //else
                //{
                //    SearchIO.output("Instant Centers could not be found");
                //    break;
                //    // instead of breaking, we will introduce an approach like we solve for accelerations.
                //}
                ////acceleration determination: only when input acceleration is given
                ////findAccelerationMIC(pivots, linkParameters, pivotParameters, timeRow, circleDiagram, links, coriolis, slipacceleration);
                //findAccelerationNew(timeRow, coriolis, coriolis_1, unknowns, omeg_1, slipacceleration);
                success = findNewPositions(timeRow, pivotLengths);
                if (!success) {SearchIO.output("Rotatability not satisfied");break;}

                //Find new position and update Path matrix of the output pivot too
                SearchIO.output(timeRow);
                //PrintDetails();
            }
            
            #endregion


            //NumericalAcceleration(pivots, linkParameters, pivots, circleDiagram, numTimeSteps, pivotParameters, timet, newt);
            //  NumericalPosition(pivots, linkParameters, pivots, circleDiagram, numTimeSteps, pivotParameters, timet, newt);
        }

        //private enum status
        //{
        //    normal,
        //    ICsNotFound,
        //    PositionNotDyadic,
        //    PositionRotabilityViolated
        //} ;
        //#region Save Output Parameter Data
        //public void saveParameterData(string filename)
        //{
        //    testfunction();
        //    StreamWriter stream = File.AppendText(filename + ".xls");
        //    for (int i = 0; i < numTimeSteps; i++)
        //    {
        //        stream.Write(path[i, 0] + "\t");
        //        stream.Write(path[i, 1] + "\t");
        //        stream.WriteLine();

        //    }
        //}
        //#endregion


        #region Numerical Position
        private void NumericalPosition(List<node> pivots, List<node> pivots_3, circleDiagramItem[] circleDiagram, int numTimeSteps, double timet, double newt)
        {
            int i;
            for (int j = 0; j < numTimeSteps; j++)
            {
                for (i = 0; i < p; i++)
                {

                    if (j != numTimeSteps - 1)
                    {
                        PivotParameters[i, j + 1, 0] = PivotParameters[i, j, 2] * timet
                            + 0.5 * PivotParameters[i, j, 4] * timet * timet
                                + PivotParameters[i, j, 0];
                        PivotParameters[i, j + 1, 1] = PivotParameters[i, j, 3] * timet
                            + 0.5 * PivotParameters[i, j, 5] * timet * timet
                            + PivotParameters[i, j, 1];
                    }

                }
            }
        }
        #endregion
        #region Numerical Acceleration
        private void NumericalAcceleration(List<node> pivots, List<node> pivots_3, circleDiagramItem[] circleDiagram, int numTimeSteps, double timet, double newt)
        {
            //we know the complete analytical velocity 
            //actually I don't need to pass all these input parameters into the function

            for (int j = 0; j < numTimeSteps; j++)
            {


                for (int i = 0; i < p; i++)
                {
                    //we know the numtimesteps
                    if (j != numTimeSteps - 1)
                    {
                        PivotParameters[i, j, 4] = (PivotParameters[i, j + 1, 2] - PivotParameters[i, j, 2]) / timet;
                        PivotParameters[i, j, 5] = (PivotParameters[i, j + 1, 3] - PivotParameters[i, j, 3]) / timet;

                    }
                    else
                    {
                        PivotParameters[i, j, 4] = (PivotParameters[i, 0, 2] - PivotParameters[i, j, 2]) / timet;
                        PivotParameters[i, j, 5] = (PivotParameters[i, 0, 3] - PivotParameters[i, j, 3]) / timet;

                    }



                }



            }






        }
        #endregion
        //#region Function: Print Details
        //private void PrintDetails(int timeRow, List<node> pivots, List<node> links, StreamWriter file)
        //{
        //    //we shall output individual pivot parameters in the format below:
        //    // pivot --->  Labels, X, Y, velocityx, velocityy, acc X, acc Y
        //    //link ---> labels, ang v, ang acc
        //    //this will append the same file for all the time-steps

        //    for (int i = 0; i < p; i++)
        //    {
        //        if (pivots[i].localLabels.Contains("inputloc") /*&& pivots[i].localLabels.Contains("ip") && !pivots[i].localLabels.Contains("ground")*/)
        //        {
        //            // file.Write(pivots[i].localLabels + "\t");
        //            file.Write(PivotParameters[i, timeRow, 0] + "\t");
        //            file.Write(PivotParameters[i, timeRow, 1] + "\t");
        //            //file.Write(PivotParameters[i, timeRow, 2] + "\t");
        //            //file.Write(PivotParameters[i, timeRow, 3] + "\t");
        //            //file.Write(PivotParameters[i, timeRow, 4] + "\t");
        //            //file.Write(PivotParameters[i, timeRow, 5] + "\t");
        //            file.WriteLine();
        //        }
        //    }

        //    for (int j = 0; j < n; j++)
        //    {

        //        // file.Write(links[j].localLabels + "\t");
        //        // file.Write(linkParameters[j, timeRow, 0] + "\t");
        //        // file.Write(linkParameters[j, timeRow, 1] + "\t");
        //        //  file.WriteLine();

        //    }

        //}


        //private void PrintDetails()
        //{
        //    // Printing Positions of pivots 2 and 3


        //    int rowMax = PivotParameters.GetLength(1);
        //    int colMax = PivotParameters.GetLength(2);


        //    int rowMax1 = LinkParameters.GetLength(1);
        //    int colMax1 = LinkParameters.GetLength(2);

        //    int whichPivot = 1;
        //    int whichPivot1 = 2;

        //    FileStream fs = new FileStream("4bar_position.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);

        //    using (StreamWriter sw = new StreamWriter(fs))
        //    {
        //        for (int i = 0; i < rowMax; i++)
        //        {
        //            for (int j = 0; j < 2; j++)
        //                sw.Write(PivotParameters[whichPivot, i, j] + "\t");
        //            for (int j = 0; j < 2; j++)
        //                sw.Write(PivotParameters[whichPivot1, i, j] + "\t");

        //            //for (int j = 0; j < 2; j++)
        //            //  sw.Write(pivotParameters[4, i, j] + "\t");
        //            //for (int j = 0; j < 2; j++)
        //            //    sw.Write(pivotParameters[5, i, j] + "\t");

        //            sw.WriteLine();
        //        }
        //    }

        //    //shall get the angle of the input crank
        //    FileStream fs4 = new FileStream("4bar_angle.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);
        //    using (StreamWriter sw = new StreamWriter(fs4))
        //    {
        //        for (int i = 0; i < rowMax; i++)
        //        {
        //            double x_x = PivotParameters[whichPivot, i, 0];
        //            double y_y = PivotParameters[whichPivot, i, 1];



        //            //we know that the initial point is 0,0

        //            //     double rad_angle = Math.Atan2(y_y, x_x);
        //            double rad_angle = Math.Atan2(y_y, x_x);

        //            //we shall determine the angle of the other link

        //            double xx_1 = PivotParameters[1, i, 0];
        //            double yy_1 = PivotParameters[1, i, 1];
        //            double xx_2 = PivotParameters[3, i, 0];
        //            double yy_2 = PivotParameters[3, i, 1];

        //            double rad_angle2 = Math.Atan2((yy_1 - yy_2), (xx_1 - xx_2));

        //            sw.Write(rad_angle + "\t");
        //            sw.Write(rad_angle2 + "\t");

        //            sw.WriteLine();



        //        }


        //    }




        //    //Printing Velocities

        //    FileStream fs1 = new FileStream("4bar_velocity.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);

        //    using (StreamWriter sw = new StreamWriter(fs1))
        //    {
        //        for (int i = 0; i < rowMax; i++)
        //        {
        //            for (int j = 2; j < 4; j++)
        //                sw.Write(PivotParameters[whichPivot, i, j] + "\t");
        //            for (int j = 2; j < 4; j++)
        //                sw.Write(PivotParameters[whichPivot1, i, j] + "\t");
        //            //for (int j = 2; j < 4; j++)
        //            //  sw.Write(pivotParameters[4, i, j] + "\t");
        //            //for (int j = 0; j < 2; j++)
        //            //    sw.Write(pivotParameters[5, i, j] + "\t");
        //            sw.WriteLine();
        //        }
        //    }



        //    //Printing Acceleration

        //    FileStream fs2 = new FileStream("4bar_acceleration.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);

        //    using (StreamWriter sw = new StreamWriter(fs2))
        //    {
        //        for (int i = 0; i < rowMax; i++)
        //        {
        //            for (int j = 4; j < 6; j++)
        //                sw.Write(PivotParameters[whichPivot, i, j] + "\t");
        //            for (int j = 4; j < 6; j++)
        //                sw.Write(PivotParameters[whichPivot1, i, j] + "\t");
        //            //for (int j = 2; j < 4; j++)
        //            //    sw.Write(pivotParameters[4, i, j] + "\t");
        //            //for (int j = 0; j < 2; j++)
        //            //    sw.Write(pivotParameters[5, i, j] + "\t");
        //            sw.WriteLine();
        //        }
        //    }

        //    //printing omega

        //    FileStream fs3 = new FileStream("omega_alpha.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);


        //    using (StreamWriter sw = new StreamWriter(fs3))
        //    {
        //        for (int i = 0; i < rowMax1; i++)
        //        {
        //            for (int j = 0; j < 2; j++)
        //                sw.Write(LinkParameters[1, i, j] + "\t");
        //            for (int j = 0; j < 2; j++)
        //                sw.Write(LinkParameters[2, i, j] + "\t");
        //            //for (int j = 0; j < 2; j++)
        //            //   sw.Write(linkParameters[3, i, j] + "\t");
        //            sw.WriteLine();
        //        }
        //    }




        //}
        //#endregion

        //#region print details

        //public void PrintDetails()
        //{
            

        //        FileStream fs = new FileStream("4bar_2pivots.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);

        //        using (StreamWriter sw = new StreamWriter(fs))
        //        {
        //            sw.Write("Output X" + "\t");
        //            sw.Write("Output Y" + "\t");
        //            sw.Write("Other Pivot X" + "\t");
        //            sw.Write("Other Pivot Y" + "\t");
        //            sw.Write("RMS" + "\t");
        //            sw.WriteLine();
        //            for (int i = 0; i < 12; i++)
        //            {

        //                sw.Write(output[i, 0] + "\t");
        //                sw.Write(output[i, 1] + "\t");
        //                sw.Write(otherpivot[i, 0] + "\t");
        //                sw.Write(otherpivot[i, 0] + "\t");
        //                sw.Write(rm_s + "\t");

        //            }


                
        //    }

        //}


        //#endregion 

    }


}