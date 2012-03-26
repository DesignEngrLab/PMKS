#region

using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;

#endregion

namespace PlanarMechanismSimulator
//at time t=0; all acceleration and velocity are zero
{
    public partial class PlanarMechanismSimulator : IDependentAnalysis
    {
        #region Unknowns - Filling up Dynamic MatrixMath Term Function

        //private void addToUnknowns(node pivot, node link, int row2, List<DynamicMatrixTerm> unknownsList)
        //{
        //    if (!pivot.localLabels.Contains("ground"))
        //    {
        //        //got to check if pivot is already there in the dynamic matrix
        //        int foundIndex = unknownsList.FindIndex(delegate(DynamicMatrixTerm dmt)
        //        {
        //            return ((dmt.belongsTo == pivot)
        //                && (dmt.type == DynamicType.absoluteAcceleration));
        //        });
        //        // when FindIndex returns a negative one, this means it was not found
        //        if (foundIndex == -1)
        //        {
        //            unknownsList.Add(new DynamicMatrixTerm());
        //            unknownsList[row2].belongsTo = pivot;
        //            unknownsList[row2].type = DynamicType.absoluteAcceleration;
        //            unknownsList[row2].dir = Direction.X;
        //            row2++;

        //            unknownsList.Add(new DynamicMatrixTerm());
        //            unknownsList[row2].belongsTo = pivot;
        //            unknownsList[row2].type = DynamicType.absoluteAcceleration;
        //            unknownsList[row2].dir = Direction.Y;
        //            row2++;
        //        }
        //    }
        //}
        private void addToUnknowns(pivot pivot, List<DynamicMatrixTerm> unknownsList)
        {
            if (!pivot.IsGround)
            {
                //got to check if pivot is already there in the dynamic matrix
                var foundIndex = unknownsList.FindIndex(dmt =>
                                                        ((dmt is PivotDynamicMatrixTerm) &&
                                                         (dmt.type == DynamicType.absoluteAcceleration) &&
                                                         ((PivotDynamicMatrixTerm)dmt).belongsTo == pivot));
                // when FindIndex returns a negative one, this means it was not found
                if (foundIndex == -1)
                {
                    if (!pivot.Contains("slider_conn"))
                    {
                        unknownsList.Add(new PivotDynamicMatrixTerm
                                             {
                                                 belongsTo = pivot,
                                                 type = DynamicType.absoluteAcceleration,
                                                 dir = Direction.X
                                             });

                        unknownsList.Add(new PivotDynamicMatrixTerm
                                             {
                                                 belongsTo = pivot,
                                                 type = DynamicType.absoluteAcceleration,
                                                 dir = Direction.Y
                                             });
                    }
                    else
                    {
                        foreach (var aa in pivot.Links)
                            foreach (var otherPivot in aa.Pivots.Where(b=> b!=aa))
                            {
                                if (otherPivot.PivotType == PivotTypes.PX)
                                {
                                    unknownsList.Add(new PivotDynamicMatrixTerm
                                                         {
                                                             belongsTo = pivot,
                                                             type = DynamicType.absoluteAcceleration,
                                                             dir = Direction.X
                                                         });
                                }
                                else if (otherPivot.PivotType == PivotTypes.PY)
                                {
                                    unknownsList.Add(new PivotDynamicMatrixTerm
                                                         {
                                                             belongsTo = pivot,
                                                             type = DynamicType.absoluteAcceleration,
                                                             dir = Direction.Y
                                                         });
                                }
                            }
                    }
                }
            }
        }

        #endregion

        #region Function: Find Linear Velocities

        //this code works for normal, sliderv,slider h, pis
        //also for those pivots connected to sliders, we need to add velocities - but this point is taken into account. So that's good
        private void findLinearVelocities(double[,] PivotParameters)
        {
            double ICx = 0.0, ICy = 0.0, omega = 0.0;
            for (var i = 0; i != p; i++)
            {
                if (pivots[i].IsGround)
                {
                    PivotParameters[i,  2] = 0.0;
                    PivotParameters[i,  3] = 0.0;
                }
                else if (!(pivots[i].localLabels.Contains("slider"))) //check for PIS and slider on a link
                {
                    link attachedLink = null;
                    /* find the first link it's connected to, and call that attachedLink */
                    foreach (var a in pivots[i].Links)
                    {
                        if (!a.Contains("slider_conn") && !a.IsGround)
                        {
                            attachedLink = a;
                            break;
                        }
                    }
                    /* next, find from cycling through the CD, the angular velocity w.r.t. ground
                     * and the IC between pivots[i] and ground. */
                    foreach (circleDiagramItem cdi in circleDiagram)
                    {
                        if ((cdi.link2 == attachedLink) && (!double.IsNaN(cdi.speed)) &&
                            cdi.link1.IsGround)
                        {
                            omega = cdi.speed;
                            ICx = cdi.x;
                            ICy = cdi.y;
                            break;
                        }
                    }
                }

                else if (pivots[i].PivotType == PivotTypes.PX || pivots[i].PivotType == PivotTypes.PY)
                    {
                    //var nsample = new node();
                    //var nsample2 = new node();
                    //var nsample3 = new node();
                    //nsample = null;
                    //nsample2 = null;
                    //nsample3 = null;
                        foreach (var nsample in pivots[i].Links)
                    {
                        if (nsample.Contains("slider_conn"))
                        {
                            foreach (var piv1 in nsample.Pivots)
                            {
                                if (!piv1.Contains("slider"))
                                {
                                    var link1 = piv1.Links.Find(a => (!a.Contains("slider_conn") || !ar.IsGround));
                                    foreach (circleDiagramItem cd in circleDiagram)
                                    {
                                        if ((cd.link2 == link1) && (!double.IsNaN(cd.speed)) &&
                                            cd.link1.IsGround)
                                        {
                                            omega = cd.speed;
                                            ICx = cd.x;
                                            ICy = cd.y;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }


                //omega X distance between the points
                //sliderh will only have the X component 
                //sliderv will only have the Y component

                if (pivots[i].IsGround)
                {
                    PivotParameters[i,  2] = 0.0;
                    PivotParameters[i,  3] = 0.0;
                }
                else if (!pivots[i].localLabels.Contains("slider"))
                {
                    PivotParameters[i,  2] = -omega * (-ICy + pivots[i].Y); //changed - to +
                    PivotParameters[i,  3] = omega * (-ICx + pivots[i].X);
                }
                else
                {


                    {
                        if (pivots[i].localLabels.Contains("sliderh"))
                        {
                            PivotParameters[i,  2] = -omega * (-ICy + pivots[i].Y);
                            PivotParameters[i,  3] = 0;
                        }
                        if (pivots[i].localLabels.Contains("sliderv"))
                        {
                            PivotParameters[i,  2] = 0;
                            PivotParameters[i,  3] = omega * (-ICx + pivots[i].X);
                        }
                    }
                }
            }
        }

        #endregion

        #region Function: Find Angular Velocities

        /// <summary>
        ///   find angular velocities (relative to gnd) - updated 9/1/08
        /// </summary>
        /// <param name="circleDiagram"> The circle diagram. </param>
        /// <param name="timeRow"> The time row. </param>
        /// <param name="links"> The links. </param>
        private double[,] findAngularVelocities()
        {
            var linkParams = new double[n,2];
            //angular velocity with respect to the position of the instant center
            //concept is to include the direction as well as magnitude of the velocities
            //22 July 2008
            int inputSpeedIndex;
            for (inputSpeedIndex = 0; inputSpeedIndex < sizeofCDI; inputSpeedIndex++)
                if (!double.IsNaN(circleDiagram[inputSpeedIndex].speed) &&
                    circleDiagram[inputSpeedIndex].speed == inputSpeed
                    && circleDiagram[inputSpeedIndex].link1.IsGround)
                {
                    break;
                }

            for (var i = 0; i < n; i++)
            {
                if (links[i] == circleDiagram[inputSpeedIndex].link2)
                    linkParams[i,  0] = circleDiagram[inputSpeedIndex].speed;
                if (links[i].Contains("slider_conn"))
                {
                    linkParams[i,  0] = 0.0;
                    linkParams[i,  1] = 0.0;
                }

                if (links[i].IsGround)
                {
                    linkParams[i,  0] = 0.0;
                    linkParams[i,  1] = 0.0;
                }
            }


            //code below is to determine the individual-link angular velocities
            //the relative angular velocities are not determined
            //the angular velocities are determined with respect to the input which has one of the links as ground
            //so the first step is to determine which of rows the ground label

            //linkParameters[inputSpeedIndex, timeRow, 0] = iOmega; //storing the value of input velocity

            int newInc;
            //why not just compute angular velocity

            for (newInc = 0; newInc < sizeofCDI; newInc++)
            {
                if (inputSpeedIndex != newInc)
                {
                    circleDiagram[newInc].speed = inputSpeed * findDistance(circleDiagram, inputSpeedIndex, newInc)
                        /*findDirectionSign(circleDiagram, inputSpeedIndex, newInc, groundLink)*/;
                }
            }


            for (var index = 0; index < sizeofCDI; index++)
            {
                for (var k = 0; k < n; k++)
                {
                    if (links[k] == circleDiagram[index].link2 &&
                        circleDiagram[index].link1.IsGround)
                        linkParams[k, 0] = circleDiagram[index].speed; //store link velocities
                }
            }

            //now got to replace slideronalink and pisonalink angular velocity with the other link based on which it rotates

            //for (int l = 0; l < n; l++)
            //{
            //    if (links[l].localLabels.Contains("slideronalink_conn") || links[l].localLabels.Contains("pis_conn"))
            //    {
            //        node n1 = null;
            //        node n2 = null;
            //        foreach (arc a in links[l].arcs)
            //        {
            //            if (a.otherNode(links[l]).localLabels.Contains("slideronalink") || a.otherNode(links[l]).localLabels.Contains("pis"))
            //                n1 = a.otherNode(links[l]);
            //        }
            //        foreach (arc a2 in n1.arcs)
            //            if (!(a2.otherNode(n1).localLabels.Contains("slideronalink_conn")) || !(a2.otherNode(n1).localLabels.Contains("pis_conn")))
            //                n2 = a2.otherNode(n1);
            //        for (int mm = 0; mm < n; mm++)
            //        {

            //            if (links[mm] == n2)
            //                linkParameters[l, timeRow, 0] = linkParameters[mm, timeRow, 0];
            //        }
            //    }
            //}
            return linkParams;
        }

        #endregion

        #region Function: Find all the other remaining ICs

        /// <summary>
        ///   find the tough IC's
        /// </summary>
        /// <param name="circleDiagram"> The circle diagram. </param>
        /// <param name="NaNtracker"> The na ntracker. </param>
        private Boolean findSecondaryICs()
        {
            throw new NotImplementedException();
        }

        #endregion

        #region Function: Find Immediate Pivots

        /// <summary>
        ///   find immediate pivots and their positions (IC's)
        /// </summary>
        /// <param name="circleDiagram"> </param>
        private void findImmediatePivots()
        {
            /* ok, now the rows of the circleDiagram have been given the unique pairings of
                     * links (link1 and link2). next go through each row of circle diagram, and find
                     * the Level 1 immediate instant centers that exist at common pivot points.
             The IC X and Y are determined in the next function*/


            for (var i = 0; i != circleDiagram.GetLength(0); i++)
            {
                var row = circleDiagram[i];
                foreach (var candidatePivot in row.link1.Pivots)
                {
                    foreach (var b in row.link2.Pivots)
                        if (b == candidatePivot)
                        {
                            //if it is a slider moving on a link,h-slider,v-slider or PIS, there is a slight modification to the code below
                            //if PIS, that particular node is actually calculated like the slider moving on a link.

                            if (candidatePivot.localLabels.Contains("pis") &&
                                !(candidatePivot.localLabels.Contains("slider")))
                                row.p = null;
                            //the above statement takes into account pis and not infy length pis
                            else
                                row.p = candidatePivot;
                            if (row.p != null && row.p.localLabels.Contains("input"))
                            {
                                // row.speed = row.pivot.localVariables[0];
                                //row.alpha = row.pivot.localVariables[1];

                                row.speed = inputSpeed;
                                row.alpha = 0.0;
                            }
                        }
                }
            }
        }

        #endregion

        #region Function: ICs located on Pivot Points

        /// <summary>
        ///   find immediate pivots and their positions (IC's)
        /// </summary>
        /// <param name="circleDiagram"> </param>
        private void findImmediateICs()
        {
            foreach (circleDiagramItem row in circleDiagram)

                if (row.p != null)
                {
                    if (row.p.localLabels.Contains("slider") || row.p.localLabels.Contains("slideronalink"))
                    {
                        //row.X = double.MaxValue;
                        //row.Y = double.MaxValue;
                        if (row.p.localLabels.Contains("sliderh"))
                        {
                            row.x = row.p.X;
                            row.y = double.MaxValue;
                        }

                        else if (row.p.localLabels.Contains("sliderv"))
                        {
                            row.x = double.MaxValue;
                            row.y = row.p.X;
                        }
                    }
                    else
                    {
                        row.x = row.p.X;
                        row.y = row.p.Y;
                    }
                }
        }

        #endregion

        #region Function: Find Distance Btw Instant Centers for Angular Velocity Computation with Direction Sign

        //function to determine the common instant center and also the distance between them as required in the formula written above
        private double findDistance(circleDiagramItem[] circleDiagram, int inputSpeedIndex, int newInc)
        {
            double inputX, inputY, c1X, c1Y, c2X, c2Y;
            inputX = circleDiagram[inputSpeedIndex].x;
            inputY = circleDiagram[inputSpeedIndex].y;
            c2X = circleDiagram[newInc].x;
            c2Y = circleDiagram[newInc].y;
            var findLink = circleDiagram[newInc].link2;
            var inputLink = circleDiagram[inputSpeedIndex].link2;


            for (var nInc = 0; nInc < sizeofCDI; nInc++)
                if ((circleDiagram[nInc].link1 == inputLink && circleDiagram[nInc].link2 == findLink)
                    || (circleDiagram[nInc].link2 == inputLink && circleDiagram[nInc].link1 == findLink))
                {
                    c1X = circleDiagram[nInc].x;
                    c1Y = circleDiagram[nInc].y;
                    //check for presence of vertical and horizontal sliders - return angular velocity as zero for that particular link
                    //since for slider h or v, the angular velocity corresponds to the link which is link-pivot-link2-slider.
                    //in this case link2 becomes zero.
                    //but we need to see that whenever link2 velocity or slider velocity is required, then we need to take the velocities 
                    //of pivot and link and add them to link2 and slider
                    //for pin-in-slot and slider on link, the angular velocity is not of a concern since angular velocities are linked 
                    //to links and linear velocities are linked to pivots concerned or points of interest


                    //if (circleDiagram[newInc].link2.localLabels.Contains("slideronalink_conn") || circleDiagram[newInc].link2.localLabels.Contains("pis_conn"))
                    //    return 0;

                    //else if (circleDiagram[newInc].link1.localLabels.Contains("slideronalink_conn") || circleDiagram[newInc].link1.localLabels.Contains("pis_conn"))
                    //    return 0; //this statement is not useful, need to verify the angular velocity calculation

                    //the statement below is used to return angular velocity of zero is the links in consideration are parallel
                    if ((inputX == double.PositiveInfinity && inputY == double.PositiveInfinity) ||
                        (c2X == double.PositiveInfinity && c2Y == double.PositiveInfinity) ||
                        (c1X == double.PositiveInfinity && c1Y == double.PositiveInfinity))
                    {
                        return 0.0;
                    }

                    else
                    {
                        double dist1, dist2, dist3;
                        //distance between two points
                        dist1 = Math.Sqrt((c1X - inputX) * (c1X - inputX) + (c1Y - inputY) * (c1Y - inputY));
                        //distance between IC & other point
                        dist2 = Math.Sqrt((c2X - inputX) * (c2X - inputX) + (c2Y - inputY) * (c2Y - inputY));
                        //distance between IC & one point
                        dist3 = Math.Sqrt((c1X - c2X) * (c1X - c2X) + (c1Y - c2Y) * (c1Y - c2Y));

                        //if (dist1 < (dist2) && dist1 < dist3)
                        if (dist1 < dist3)
                            return (-dist1 / dist3);
                        else
                            return (dist1 / dist3);
                    }
                }

            return double.NaN;
        }

        #endregion

        #region Function: Common & Fulfilled rows

        private List<circleDiagramItem> findCommonAndFulfilledRows(link link, circleDiagramItem[] circleDiagram)
        {
            var commonRows = new List<circleDiagramItem>();
            for (var i = 0; i != circleDiagram.GetLength(0); i++)
                if ((!double.IsNaN(circleDiagram[i].x)) //this checks to see if the row has been fulfilled
                    && ((circleDiagram[i].link1 == link) || ((circleDiagram[i].link2 == link))))
                    // this checks to see if the row has the same link as the input to the function - meaning it is common.
                    commonRows.Add(circleDiagram[i]);
            return commonRows;
        }

        #endregion

        #region Function: Unknown Instant centers

        private bool unknownInstantCenters(circleDiagramItem[] circleDiagram)
        {
            for (var i = 0; i != circleDiagram.GetLength(0); i++)
                if (double.IsNaN(circleDiagram[i].x))
                    return true;
            return false;
        }

        #endregion

        #region Function: Find instant Center

        private double[] findInstantCenter(int[,] CDIRows, circleDiagramItem[] list1, circleDiagramItem[] list2,
                                           circleDiagramItem[] circleDiagram, link link1, link link2)
        {
            // !!!!!!!!!!!!!!!!!!!!!
            //
            // check for divide by zero. where in the equations below does the denominator go
            // to zero. This is in fact natural in the case of sliders. How to handle this?
            //
            // !!!!!!!!!!!!!!!!!!!!!

            //for PIS, there needs to be a separate if-else code
            //since there is only one row of CDI
            //so need to identify that it is only row and then take the computation to the stage below
            //if the length of CDI is only 2, then we can as well know it might be a PIS case.
            var arr = new circleDiagramItem();
            var brr = new circleDiagramItem();
            var crr = new circleDiagramItem();
            var drr = new circleDiagramItem();

            var Slope0Nr = 0.0;
            var Slope0Dr = 0.0;
            var Slope1Nr = 0.0;
            var Slope1Dr = 0.0;

            double Slope0, Slope1;

            var searchnode = new List<pivot>();


            double xc = double.NaN, yc = double.NaN;

            var x00 = 0.0;
            var y00 = 0.0;
            var x01 = 0.0;
            var y01 = 0.0;


            // second row - two common points on a line
            var x10 = 0.0;
            var y10 = 0.0;
            var x11 = 0.0;
            var y11 = 0.0;

            double xar, yar, xbr, ybr, xcr, ycr, xdr, ydr;

            var indexno = 0;
            if (list1.GetLength(0) == 1 || list2.GetLength(0) == 1)
            {
                indexno = 3;
            }

            if (CDIRows.GetLength(0) == 2 && indexno != 3)
            {
                #region first get the index nos

                if (list1.GetLength(0) <= list2.GetLength(0))
                {
                    // first row - two common points on a line
                    x00 = list1[(CDIRows[0, 0])].x;
                    y00 = list1[(CDIRows[0, 0])].y;
                    x01 = list2[(CDIRows[0, 1])].x;
                    y01 = list2[(CDIRows[0, 1])].y;


                    // second row - two common points on a line
                    x10 = list1[(CDIRows[1, 0])].x;
                    y10 = list1[(CDIRows[1, 0])].y;
                    x11 = list2[(CDIRows[1, 1])].x;
                    y11 = list2[(CDIRows[1, 1])].y;


                    indexno = 1;
                }
                else
                {
                    x00 = list1[(CDIRows[0, 0])].x;
                    y00 = list1[(CDIRows[0, 0])].y;
                    x01 = list2[(CDIRows[0, 1])].x;
                    y01 = list2[(CDIRows[0, 1])].y;


                    // second row - two common points on a line
                    x10 = list1[(CDIRows[1, 0])].x;
                    y10 = list1[(CDIRows[1, 0])].y;
                    x11 = list2[(CDIRows[1, 1])].x;
                    y11 = list2[(CDIRows[1, 1])].y;

                    indexno = 2;
                }

                #endregion

                Slope0Nr = (y01 - y00);
                Slope0Dr = (x01 - x00);
                Slope1Nr = (y11 - y10);
                Slope1Dr = (x11 - x10);

                //the statement below is used to identify if there are parallel links
                //if links are parallel then the instant center is at infinity
                //if this happens, then the angular velocity of that particular link is also 0

                if (Slope0Nr == 0 && Slope0Dr == 0 && Slope1Nr == 0 && Slope1Dr == 0)
                {
                    xc = double.PositiveInfinity;
                    yc = double.PositiveInfinity;
                }

                #region for indexno==1

                else if (indexno == 1)
                {
                    if (list1[(CDIRows[1, 0])].x == double.MaxValue ||
                        list1[(CDIRows[0, 0])].x == double.MaxValue ||
                        list2[(CDIRows[0, 1])].x == double.MaxValue
                        || list2[(CDIRows[1, 1])].x == double.MaxValue)
                    {
                        //4 if statements below basically check which one has got Maxvalue - this will help in identifying which one has a slider/slideronalink/not PIS
                        if (list1[(CDIRows[1, 0])].x == double.MaxValue)
                        {
                            arr = list1[(CDIRows[1, 0])];

                            brr = list2[(CDIRows[1, 1])];

                            crr = list1[(CDIRows[0, 0])];

                            drr = list2[(CDIRows[0, 1])];
                        }

                        if (list1[(CDIRows[0, 0])].x == double.MaxValue)
                        {
                            arr = list1[(CDIRows[0, 0])];
                            brr = list2[(CDIRows[0, 1])];
                            crr = list1[(CDIRows[1, 0])];
                            drr = list1[(CDIRows[1, 1])];
                        }
                        if (list2[(CDIRows[0, 1])].x == double.MaxValue)
                        {
                            arr = list2[(CDIRows[0, 1])];
                            brr = list1[(CDIRows[0, 0])];
                            crr = list1[(CDIRows[1, 0])];
                            drr = list2[(CDIRows[1, 1])];
                        }
                        if (list2[(CDIRows[1, 1])].x == double.MaxValue)
                        {
                            arr = list2[(CDIRows[1, 1])];
                            brr = list1[(CDIRows[1, 0])];
                            crr = list1[(CDIRows[0, 0])];

                            drr = list2[(CDIRows[0, 1])];
                        }

                        //find out what element exists between the two links in circlediagram a from g.nodes 
                        //basically looking for slider/slideronalink/PIS


                        foreach (var ar in arr.link1.Pivots)
                        {
                            if (ar.PivotType == PivotTypes.PX || ar.PivotType == PivotTypes.PY)
                              //  || ar.localLabels.Contains("slideronalink"))
                                //i have actually removed the pis checking in these statements
                                //which were earlier included
                                searchnode.Add(ar);
                        }


                        foreach (var ar in arr.link2.Pivots)
                        {
                            if (arr.link1.Contains("slideronalink_conn")
                                || arr.link1.Contains("slider_conn"))
                            {
                                foreach (var ar1 in arr.link1.Pivots)
                                {
                                    if (ar1 != searchnode[0])
                                        searchnode.Add(ar1);
                                    if (searchnode[0] != ar)
                                        searchnode.Add(ar);
                                }
                            }

                            else if (arr.link2.Contains("slideronalink_conn")
                                     || arr.link1.Contains("slider_conn"))
                            {
                                foreach (var ar1 in arr.link2.Pivots)
                                {
                                    if (ar1 != searchnode[0])
                                        searchnode.Add(ar1);
                                    if (searchnode[0] != ar)
                                        searchnode.Add(ar);
                                }
                            }
                        }

                        //now that searchnode[0] consists of sliderh,sliderv,pis,slideronalink X&Y, searchnode[1]&[2] - contains the other X's & Y's typically used for slideronalink

                        if (searchnode[0].PivotType == PivotTypes.PX)
                        {
                            xbr = brr.x;
                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);
                            var B0 = ycr - Slope1 * xcr;
                            xc = xbr;
                            yc = Slope1 * xc + B0;
                        }
                        else if (searchnode[0].PivotType==PivotTypes.PY)
                        {
                            ybr = brr.x;
                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);
                            var B0 = ycr - Slope1 * xcr;
                            yc = ybr;
                            xc = (yc - B0) / Slope1;
                        }
                        else if (searchnode[0].Contains("slideronalink"))
                        {
                            xar = searchnode[1].X;
                            yar = searchnode[1].Y;
                            xbr = searchnode[2].X;
                            ybr = searchnode[2].Y;
                            var Slope10 = (ybr - yar) / (xbr - xar);
                            Slope0 = -1 / Slope10;


                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);

                            var B0 = searchnode[2].Y - Slope0 * searchnode[2].X;
                            var B1 = searchnode[1].Y - Slope1 * searchnode[1].X;
                            xc = (B1 - B0) / (Slope0 - Slope1);
                            yc = Slope0 * xc + B0;
                        }

                        //add code for horizontal and vertical lines - since slope determination results in infinity for vertical lines
                        //if slope0=0 meaning horizontal line - which will result in equation as Y=A
                        //if slope = infinity meaning vertical line; equation of line X=B;
                        //conditions to check are
                        //horizontal-slant;vertical-slant;horizontal-vertical
                        //so this code(the two else-if statements check if it is vertical alone;horizontal lines are taken care of.
                    }

                    else if (Slope0Dr == 0 && Slope1Dr == 0 && indexno != 3)
                    {
                        // xc = double.PositiveInfinity;
                        //yc = double.PositiveInfinity;
                        xc = double.MaxValue;
                        yc = double.MaxValue;
                    }

                    else
                    {
                        if (Slope0Dr == 0 && Slope1Dr != 0 && indexno != 3)
                        {
                            xc = x01;
                            Slope1 = Slope1Nr / Slope1Dr;
                            yc = Slope1 * xc;
                        }

                        else if (Slope1Dr == 0 && Slope0Dr != 0 && indexno != 3)
                        {
                            xc = x10;
                            Slope0 = Slope0Nr / Slope0Dr;
                            yc = Slope0 * xc;
                        }
                        else
                        {
                            Slope0 = Slope0Nr / Slope0Dr;
                            Slope1 = Slope1Nr / Slope1Dr;
                            var B0 = y00 - Slope0 * x00;
                            var B1 = y10 - Slope1 * x10;
                            xc = (B1 - B0) / (Slope0 - Slope1);
                            yc = Slope0 * xc + B0;
                        }
                    }
                }
                #endregion

                #region for indexno==2

                else if (indexno == 2)
                {
                    if (list1[(CDIRows[1, 0])].x == double.MaxValue ||
                        list1[(CDIRows[0, 0])].x == double.MaxValue ||
                        list2[(CDIRows[0, 1])].x == double.MaxValue
                        || list2[(CDIRows[1, 1])].x == double.MaxValue)
                    {
                        //4 if statements below basically check which one has got Maxvalue - this will help in identifying which one has a slider/slideronalink/not PIS
                        if (list1[(CDIRows[1, 0])].x == double.MaxValue)
                        {
                            arr = list1[(CDIRows[1, 0])];

                            brr = list2[(CDIRows[1, 1])];

                            crr = list1[(CDIRows[0, 0])];

                            drr = list2[(CDIRows[0, 1])];
                        }

                        if (list1[(CDIRows[0, 0])].x == double.MaxValue)
                        {
                            arr = list1[(CDIRows[0, 0])];
                            brr = list2[(CDIRows[0, 1])];
                            crr = list1[(CDIRows[1, 0])];
                            drr = list2[(CDIRows[1, 1])];
                        }
                        if (list2[(CDIRows[0, 1])].x == double.MaxValue)
                        {
                            arr = list2[(CDIRows[0, 1])];
                            brr = list1[(CDIRows[0, 0])];
                            crr = list1[(CDIRows[1, 0])];
                            drr = list2[(CDIRows[1, 1])];
                        }
                        if (list2[(CDIRows[1, 1])].x == double.MaxValue)
                        {
                            arr = list2[(CDIRows[1, 1])];
                            brr = list1[(CDIRows[1, 0])];
                            crr = list1[(CDIRows[0, 0])];

                            drr = list2[(CDIRows[0, 1])];
                        }

                        //find out what element exists between the two links in circlediagram a from g.nodes 
                        //basically looking for slider/slideronalink/PIS


                        foreach (var ar in arr.link1.Pivots)
                        {
                            if (ar.PivotType == PivotTypes.PX || ar.PivotType == PivotTypes.PY 
                              || ar.Contains("slideronalink"))
                                //i have actually removed the pis checking in these statements
                                //which were earlier included
                                searchnode.Add(ar);
                        }


                        foreach (var ar in arr.link2.Pivots)
                        {
                            if (arr.link1.Contains("slideronalink_conn")
                                || arr.link1.Contains("slider_conn"))
                            {
                                foreach (var ar1 in arr.link1.Pivots)
                                {
                                    if (ar1 != searchnode[0])
                                        searchnode.Add(ar1);
                                    if (searchnode[0] != ar)
                                        searchnode.Add(ar);
                                }
                            }

                            else if (arr.link2.Contains("slideronalink_conn")
                                     || arr.link1.Contains("slider_conn"))
                            {
                                foreach (var ar1 in arr.link2.Pivots)
                                {
                                    if (ar1 != searchnode[0])
                                        searchnode.Add(ar1);
                                    if (searchnode[0] != ar)
                                        searchnode.Add(ar);
                                }
                            }
                        }

                        //now that searchnode[0] consists of sliderh,sliderv,pis,slideronalink X&Y, searchnode[1]&[2] - contains the other X's & Y's typically used for slideronalink

                        if (searchnode[0].PivotType == PivotTypes.PX)
                        {
                            xbr = brr.x;
                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);
                            var B0 = ycr - Slope1 * xcr;
                            xc = xbr;
                            yc = Slope1 * xc + B0;
                        }
                        else if (searchnode[0].PivotType == PivotTypes.PY)
                        {
                            ybr = brr.x;
                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);
                            var B0 = ycr - Slope1 * xcr;
                            yc = ybr;
                            xc = (yc - B0) / Slope1;
                        }
                        else if (searchnode[0].Contains("slideronalink"))
                        {
                            xar = searchnode[1].X;
                            yar = searchnode[1].Y;
                            xbr = searchnode[2].X;
                            ybr = searchnode[2].Y;
                            var Slope10 = (ybr - yar) / (xbr - xar);
                            Slope0 = -1 / Slope10;


                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);

                            var B0 = searchnode[2].Y - Slope0 * searchnode[2].X;
                            var B1 = searchnode[1].Y - Slope1 * searchnode[1].X;
                            xc = (B1 - B0) / (Slope0 - Slope1);
                            yc = Slope0 * xc + B0;
                        }

                        //add code for horizontal and vertical lines - since slope determination results in infinity for vertical lines
                        //if slope0=0 meaning horizontal line - which will result in equation as Y=A
                        //if slope = infinity meaning vertical line; equation of line X=B;
                        //conditions to check are
                        //horizontal-slant;vertical-slant;horizontal-vertical
                        //so this code(the two else-if statements check if it is vertical alone;horizontal lines are taken care of.
                    }

                    else if (Slope0Dr == 0 && Slope1Dr == 0 && indexno != 3)
                    {
                        xc = double.PositiveInfinity;
                        yc = double.PositiveInfinity;
                    }

                    else
                    {
                        if (Slope0Dr == 0 && Slope1Dr != 0 && indexno != 3)
                        {
                            xc = x01;
                            Slope1 = Slope1Nr / Slope1Dr;
                            yc = Slope1 * xc + (y01 - Slope1 * x01);
                        }

                        else if (Slope1Dr == 0 && Slope0Dr != 0 && indexno != 3)
                        {
                            xc = x10;
                            Slope0 = Slope0Nr / Slope0Dr;

                            yc = Slope0 * xc + (y00 - Slope0 * x00);
                        }
                        else
                        {
                            Slope0 = Slope0Nr / Slope0Dr;
                            Slope1 = Slope1Nr / Slope1Dr;
                            var B0 = y00 - Slope0 * x00;
                            var B1 = y10 - Slope1 * x10;
                            xc = (B1 - B0) / (Slope0 - Slope1);
                            yc = Slope0 * xc + B0;
                        }
                    }
                }

                #endregion
            }

                //the if-else statement is to be used for PIS
            //what needs to be done? 
            //we already know that one link connection
            //the other is PIS
            //which is connected to a link that has two pivots
            //we need to get the slope of that link and use the pis X&Y


            #region PIS section essentially

            else
            {
                if (link1.Contains("pis_conn") || link2.Contains("pis_conn"))
                {
                    x00 = list2[(CDIRows[0, 0])].x;
                    y00 = list2[(CDIRows[0, 0])].y;
                    x01 = list1[(CDIRows[0, 1])].x;
                    y01 = list1[(CDIRows[0, 1])].y;

                    var slope0Dr = x01 - x00;
                    var slope0Nr = y01 - y00;

                    Slope0 = (y01 - y00) / (x01 - x00);


                    if (!(list1[(CDIRows[0, 0])].link2.Contains("pis_conn")))
                        foreach (var a in list1[(CDIRows[0, 0])].link2.Pivots)
                        {
                            if (a.Contains("pis"))
                            {
                                searchnode.Add(a);
                                searchnode.Add(list1[(CDIRows[0, 0])].link2);
                                searchnode.Add(list2[(CDIRows[0, 0])].link2);
                            }
                        }
                    if (!(list2[(CDIRows[0, 0])].link2.Contains("pis_conn")))
                        foreach (var a in list2[(CDIRows[0, 0])].link2.Pivots)
                        {
                            if (a.Contains("pis"))
                            {
                                searchnode.Add(a);
                                searchnode.Add(list2[(CDIRows[0, 0])].link2);
                                searchnode.Add(list1[(CDIRows[0, 0])].link2);
                            }
                        }
                    //now we know which is connected to PIS
                    //from searchnode[2] we also find out what is the other link to which pis is connected
                    //that is - we know the link which has the label pis_conn
                    //this section works for straight line type PIS and not plate like PIS

                    if (searchnode.Count >= 2)
                    {
                        foreach (var b in searchnode[2].Pivots)
                            if (!b.Contains("pis"))
                                searchnode.Add(b);

                        //the above statement might not work if the pis is connected to a triangular plate or quad node?
                        //will it work for circular slot? may not - as of now this work only for straight line paths


                        x10 = searchnode[3].X;
                        y10 = searchnode[3].Y;
                        x11 = searchnode[4].X;
                        y11 = searchnode[4].Y;


                        Slope1Nr = y11 - y10;
                        Slope1Dr = x11 - x10;

                        if (Slope1Dr == 0)
                            //this statement also takes into account the vertical position of the PIS slot link
                            Slope1 = 0;
                        else
                            Slope1 = -1 / (Slope1Nr / Slope1Dr);


                        double x22, y22;

                        x22 = searchnode[0].X;
                        y22 = searchnode[0].Y;

                        //equation for solving

                        double B0 = 0.0, B1 = 0.0;

                        B0 = y22 - Slope1 * x22;
                        B1 = y01 - Slope0 * x01;

                        if (slope0Dr != 0)
                        {
                            xc = (B1 - B0) / (Slope1 - Slope0);
                            yc = Slope1 * xc + B0;
                        }
                        else
                        {
                            xc = list1[(CDIRows[0, 0])].x;
                            yc = Slope1 * xc + B0;
                        }
                    }
                }
            }

            #endregion

            return new[] { xc, yc };
        }

        #endregion

        #region Function: New IC Found In Last Pass

        //this has been taken from Position
        private readonly List<Boolean> ICsFound = new List<Boolean>();
        // Campbell: this is only used in function below. Can it be made a local variable?

        private bool NewICFoundInLastPass(circleDiagramItem[] circleDiagram)
        {
            /* assume, at first, that no change has occured (result = false),
             * then if any change is found switch it to true. */
            var result = false;
            /* a change is search in the following for-loop. A change means
             * that the position (or just the X-coord in this case) is an
             * actual number - no longer NaN, but if the corresponding 
             * boolean in PositionsFound is false, then last pass it was
             * NaN. Therefore a change has occured - set result to true. */
            for (var i = 0; i < circleDiagram.GetLength(0); i++)
                if (!double.IsNaN(circleDiagram[i].x) &&
                    ((ICsFound.Count <= i) || !ICsFound[i]))
                {
                    result = true;
                    break;
                }
            /* Update PositionsFound
             * If true then we now need to change any PositionsFound to true
             * that were previously false but which now have a value. */
            if (result)
            {
                for (var i = 0; i < circleDiagram.GetLength(0); i++)
                {
                    if (ICsFound.Count <= i)
                        ICsFound.Add(false);
                    if (!double.IsNaN(circleDiagram[i].x))
                        ICsFound[i] = true;
                }
            }
            return result;
        }

        #endregion
    }
}