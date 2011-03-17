using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using GraphSynth.Representation;
using GraphSynth;
using OptimizationToolbox;

namespace PlanarMechanismSimulator

    //at time t=0; all acceleration and velocity are zero
{
    public partial class MechSimulation : IDependentAnalysis
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
        private void addToUnknowns(node pivot, node link, List<DynamicMatrixTerm> unknownsList)
        {
            if (!pivot.localLabels.Contains("ground"))
            {
                //got to check if pivot is already there in the dynamic matrix
                int foundIndex = unknownsList.FindIndex(delegate(DynamicMatrixTerm dmt)
                {
                    return ((dmt.belongsTo == pivot)
                        && (dmt.type == DynamicType.absoluteAcceleration));
                });
                // when FindIndex returns a negative one, this means it was not found
                if (foundIndex == -1)
                {
                    if (!pivot.localLabels.Contains("slider_conn"))
                    {
                        unknownsList.Add(new DynamicMatrixTerm()
                        {
                            belongsTo = pivot,
                            type = DynamicType.absoluteAcceleration,
                            dir = Direction.X
                        });

                        unknownsList.Add(new DynamicMatrixTerm()
                        {
                            belongsTo = pivot,
                            type = DynamicType.absoluteAcceleration,
                            dir = Direction.Y
                        });
                    }
                    else
                    {
                        foreach(arc aa in pivot.arcs)
                            if (aa.localLabels.Contains("pivotarc"))
                            {
                                if (aa.otherNode(pivot).localLabels.Contains("sliderh"))
                                {
                                    unknownsList.Add(new DynamicMatrixTerm()
                                    {
                                        belongsTo = pivot,
                                        type = DynamicType.absoluteAcceleration,
                                        dir = Direction.X
                                    });

                                }
                                else if (aa.otherNode(pivot).localLabels.Contains("sliderv"))
                                {
                                    unknownsList.Add(new DynamicMatrixTerm()
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
        private void findLinearVelocities(circleDiagramItem[] circleDiagram, List<node> pivots, int timeRow, double newt)
        {
            double ICx = 0.0, ICy = 0.0, omega = 0.0;
            for (int i = 0; i != p; i++)
            {
                if (pivots[i].localLabels.Contains("ground"))
                {
                    PivotParameters[i, timeRow, 2] = 0.0;
                    PivotParameters[i, timeRow, 3] = 0.0;
                }
                else if (!(pivots[i].localLabels.Contains("slider"))) //check for PIS and slider on a link
                {

                    node attachedLink = null;
                    /* find the first link it's connected to, and call that attachedLink */
                    foreach (arc a in pivots[i].arcs)
                    {
                        if (a.otherNode(pivots[i]).localLabels.Contains("link") && !(a.otherNode(pivots[i]).localLabels.Contains("slider_conn"))

                             && !(a.otherNode(pivots[i]).localLabels.Contains("ground")))
                        {

                            attachedLink = a.otherNode(pivots[i]);
                            break;
                        }
                    }
                    /* next, find from cycling through the CD, the angular velocity w.r.t. ground
                     * and the IC between pivots[i] and ground. */
                    foreach (circleDiagramItem cdi in circleDiagram)
                    {
                        if ((cdi.link2 == attachedLink) && (!double.IsNaN(cdi.speed)) && cdi.link1.localLabels.Contains("ground"))
                        {
                            omega = cdi.speed;
                            ICx = cdi.x;
                            ICy = cdi.y;
                            break;
                        }
                    }
                }

                else if (pivots[i].localLabels.Contains("sliderh") || pivots[i].localLabels.Contains("sliderv"))
                {
                    node nsample = new node();
                    node nsample2 = new node();
                    node nsample3 = new node();
                    nsample = null;
                    nsample2 = null; nsample3 = null;
                    foreach (arc a in pivots[i].arcs)
                    {
                        if (a.otherNode(pivots[i]).localLabels.Contains("slider_conn"))
                        {


                            nsample = a.otherNode(pivots[i]);

                            foreach (arc a1 in nsample.arcs)
                            {
                                if (!a1.otherNode(nsample).localLabels.Contains("slider"))
                                {


                                    nsample2 = a1.otherNode(nsample);

                                    //ICx = nsample2.X;
                                    //ICy = nsample2.Y;

                                    foreach (arc ar in nsample2.arcs)
                                    {
                                        if (ar.otherNode(nsample2).localLabels.Contains("link") && (!(ar.otherNode(nsample2).localLabels.Contains("slider_conn"))
                                         || !(ar.otherNode(nsample2).localLabels.Contains("ground"))))
                                        {
                                            nsample3 = ar.otherNode(nsample2);

                                            if (nsample3 != null)
                                                break;
                                        }
                                    }

                                    foreach (circleDiagramItem cd in circleDiagram)
                                    {
                                        if ((cd.link2 == nsample3) && (!double.IsNaN(cd.speed)) && cd.link1.localLabels.Contains("ground"))
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

                if (pivots[i].localLabels.Contains("ground"))
                {
                    PivotParameters[i, timeRow, 2] = 0.0;
                    PivotParameters[i, timeRow, 3] = 0.0;
                }
                else if (!pivots[i].localLabels.Contains("slider"))
                {


                    PivotParameters[i, timeRow, 2] = -omega * (-ICy + pivots[i].Y);//changed - to +
                    PivotParameters[i, timeRow, 3] = omega * (-ICx + pivots[i].X);

                }
                else
                {


                    {
                        if (pivots[i].localLabels.Contains("sliderh"))
                        {
                            PivotParameters[i, timeRow, 2] = -omega * (-ICy + pivots[i].Y);
                            PivotParameters[i, timeRow, 3] = 0;
                        }
                        if (pivots[i].localLabels.Contains("sliderv"))
                        {
                            PivotParameters[i, timeRow, 2] = 0;
                            PivotParameters[i, timeRow, 3] = omega * (-ICx + pivots[i].X);
                        }
                    }
                }

            }
        }






        #endregion
 
        #region Function: Find Angular Velocities
        /// <summary>
        /// find angular velocities (relative to gnd)   - updated 9/1/08
        /// </summary>
        /// <param name="circleDiagram"></param>
        /// <param name="sizeofCDI"></param>
        private void findAngularVelocities(circleDiagramItem[] circleDiagram, int timeRow, List<node> links, double newt)
        {
            //angular velocity with respect to the position of the instant center
            //concept is to include the direction as well as magnitude of the velocities
            //22 July 2008

            for (inputSpeedIndex = 0; inputSpeedIndex < sizeofCDI; inputSpeedIndex++)
                if (!double.IsNaN(circleDiagram[inputSpeedIndex].speed) && circleDiagram[inputSpeedIndex].speed == iOmega
                    && circleDiagram[inputSpeedIndex].link1.localLabels.Contains("ground"))
                {
                    break;
                }

            for (int i = 0; i < n; i++)
            {
                if (links[i] == circleDiagram[inputSpeedIndex].link2)
                    LinkParameters[i, timeRow, 0] = circleDiagram[inputSpeedIndex].speed;
                if (links[i].localLabels.Contains("slider_conn"))
                {
                    LinkParameters[i, timeRow, 0] = 0.0;
                    LinkParameters[i, timeRow, 1] = 0.0;
                }

                if (links[i].localLabels.Contains("ground"))
                {
                    LinkParameters[i, timeRow, 0] = 0.0;
                    LinkParameters[i, timeRow, 1] = 0.0;
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
                    circleDiagram[newInc].speed = iOmega * findDistance(circleDiagram, inputSpeedIndex, newInc)
                        /*findDirectionSign(circleDiagram, inputSpeedIndex, newInc, groundLink)*/;

                }
            }



            for (int index = 0; index < sizeofCDI; index++)
            {
                for (int k = 0; k < n; k++)
                {
                    if (links[k] == circleDiagram[index].link2 && circleDiagram[index].link1.localLabels.Contains("ground"))
                        LinkParameters[k, timeRow, 0] = circleDiagram[index].speed;//store link velocities
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




        }
        #endregion
        #region Function: Find all the other remaining ICs
        /// <summary>
        /// find the tough IC's
        /// </summary>
        /// <param name="circleDiagram"></param>
        /// <param name="cDLength"></param>
        private void findSecondaryICs(circleDiagramItem[] circleDiagram, candidate c, double NaNtracker)
        {
            

            //code for double butterfly linkage
            double x1=0.0, x2=0.0, x3=0.0, x4=0.0, x5=0.0, x6=0.0, x7=0.0, x8=0.0, x9=0.0, x10=0.0, y1=0.0, y2=0.0, y3=0.0, y4=0.0, y5=0.0,y6=0.0, y7=0.0, y8=0.0, y9=0.0, y10=0.0;
            foreach (node nd in g.nodes)
            {
                if (nd.name=="IP")
                {
                    x1 = nd.X;
                    y1 = nd.Y;
                }
                if (nd.name=="IP7")
                {
                    x2 = nd.X;
                    y2 = nd.Y;
                }
                if (nd.name=="Input")
                {
                    x3 = nd.X;
                    y3 = nd.Y;
                }
                if (nd.name=="Output")
                {
                    x4 = nd.X;
                    y4 = nd.Y;
                }
                if (nd.name=="IP8")
                {
                    x5 = nd.X;
                    y5 = nd.Y;
                }
                if (nd.name=="IP21")
                {
                    x6 = nd.X;
                    y6 = nd.Y;
                }
                if (nd.name=="n1")
                {
                    x7 = nd.X;
                    y7 = nd.Y;
                }
                if(nd.name=="n4")
                {
                    x8 = nd.X;
                    y8 = nd.Y;
                }
                if (nd.name=="n2")
                {
                    x9 = nd.X;
                    y9 = nd.Y;
                }
                if (nd.name == "IP2")
                {
                    x10 = nd.X;
                    y10 = nd.Y;
                }                          
            }


            //one xy point 

            double slope = (y4 - y3) / (x4 - x3);

            double a = -slope, b = 1, e = -slope * x5 + y5, c1 = -(y2 - y1), d = x2 - x1, f = c1 * x1 + d * y1;

            double newY = (c1 * e - f * a) / (b * c1 - a * d);

            double newX = (e - b * newY) / a;

            //now obtain the intersection point between link 2 and link4

            a = (y2 - y1); b = (x2 - x1); c1 = (y4 - y3); d = (x4 - x3);

            double newY1 = (-c1 * (a * x1 - b * y1) + a * (c1 * x3 - d * y3)) / (c1 * b - a * d);

            double newX1 = (a * x1 - b * y1 + b * newY1) / a;

            //now slope between B and newX1,newY1

            double slope_1 = (y10 - newY1) / (x10 - newX1);

            //----------------------------------------------------

            //another point

             slope = (y9 - y8) / (x9 - x8);

            a = -slope; b = 1; e = -slope * x5 + y5; c1 = -(y7 - y6); d = (x7 - x6); f = c1 * x6 + d * y6;

            double newY2 = (c1 * e - f * a) / (b * c1 - a * d);

            double newX2 = (e - b * newY2) / a;

            //now obtain the intersection point between link 6 and link8

            a = (y9 - y8); b = (x9 - x8); c1 = (y7 - y6); d = (x7 - x6);

            double newX3 = (b * d * (y8 - y6) - a * d * x8 + b * c1 * x6) / (b * c1 - a * d);

            double newY3 = (a*newX3-a*x8+b*y8)/b;


            //now slope between B and newX3,newY3

            double slope_2 = (y10 - newY3) / (x10 - newX3);


            //now there are two slopes, two points - find intersection point

            double new_pointX = (-slope_2 * newX2 + slope_1 * newX - newY + newY)/(slope_1-slope_2);
            double new_pointY = -slope_1 * newX + newY + slope_1 * new_pointX;
            
            //now point Q is obtained 

            //connect point between Q and O5(IP8)
            //now draw a line through B parallel to Q-O5. 
            //intersection of the above line with link2-gives I-13
            //intersection of the line with link8-gives I-17

            double new_slope = (y5 - new_pointY) / (x5 - new_pointX);
            
            //point x10,y10; 

            //point x3,y3,x4,y4

            //the point of intersection will give one ic

            a = -new_slope; b = 1; e = -new_slope * x10 + y10; c1 = -(y4 - y3); d = (x4 - x3); f = c1 * x3 + d * y3;

            double IC_1y = (c1 * e - f * a) / (b * c1 - a * d);
            double IC_1x = (e - b * IC_1y) / a;

            //other point that needs to taken - x8,x9

            a = -new_slope; b = 1; e = -new_slope * x10 + y10; c1 = -(y9 - y8); d = (x9 - x8); f = c1 * x8 + d * y8;

            double IC_2y = (c1 * e - f * a) / (b * c1 - a * d);
            double IC_2x = (e - b * IC_2y) / a;

            //the two instant centers are obtained

            //how to fill these two into the appropriate space
            //one could form a for-loop - searching for the appropriate position in the circle-diagram

            //filling the space

            for (int i = 0; i != sizeofCDI; i++)
            {
                if (circleDiagram[i].link1.name == "Ground" && circleDiagram[i].link2.name == "n0")
                { circleDiagram[i].x = IC_1x; circleDiagram[i].y = IC_1y; }
                if (circleDiagram[i].link1.name == "Ground" && circleDiagram[i].link2.name == "link11")
                { circleDiagram[i].x = IC_2x; circleDiagram[i].y = IC_2y; }

            }


            /* first pass is now complete. Now, we have to go through the circleDiagram a few
         * more times to fill up the secondary and tertiary (perhaps higher) instant centers. */
            do
            {
                for (int m = 0; m != sizeofCDI; m++)
                    /* if it's unfulfilled - meaning X and Y are not known, then attempt to solve it. */
                    if (double.IsNaN(circleDiagram[m].x))
                    {
                        node link1 = circleDiagram[m].link1;
                        node link2 = circleDiagram[m].link2;
                        List<circleDiagramItem> list1 = findCommonAndFulfilledRows(link1, circleDiagram);
                        List<circleDiagramItem> list2 = findCommonAndFulfilledRows(link2, circleDiagram);

                       


                        // if either list1 or list2 is null then skip this row for now - you'll need to find another IC
                        // further down the list to solve this in the next outer loop (next time you get to while).

                        // find at least two common links in rows of list1 and list2
                        // else skip for now
                        int[,] CDIRows = new int[2, 2];//two paths needed as in the circle diagram[0 1;1 1]

                        int k = 0;//row index for CDIRows - starts with 0,goes to 1
                        // when it increments to 2, then we've successfully filled up CDIRows. At this point, we've
                        // found four existing IC's to help us find the IC for this row. There is no room for k=2, but
                        // it is an indication to us that we can solve this row. It may find a third path (set of ICs)
                        // but we don't need it.

                        //in the case of pis, we may end up with just three values since where-ever pis is there, we need to get the 
                        //perpendicular slope and use coordinate of pis in the equation determination
                        //
                        //will this affect the general program? may not I guess....
                        //can it be made that - just one pair and the other pis equation could be determined from the instant center function

                        //updated tuesday 17th - the code needs to be reworked 
                       // if (list1.Count < list2.Count || list2.Count < list1.Count)
                        if(list1.Count==1 || list2.Count==1)
                        {
                            if (list1.Count < list2.Count)
                            {
                                
                                for (int i = 0; i != list1.Count; i++)
                                {
                                    node otherlink = (link1 == list1[i].link1) ? list1[i].link2 : list1[i].link1;
                                    for (int j = 0; j != list2.Count; j++)
                                    {
                                        if ((otherlink == list2[j].link1) || (otherlink == list2[j].link2))
                                        {
                                            if (k < 2)
                                            {
                                                CDIRows[k, 0] = i; //only one row is filled
                                                CDIRows[k, 1] = j;
                                                k++;
                                            }
                                        }
                                    }
                                }

                            }
                            else
                            {
                                for (int i = 0; i != list2.Count; i++)
                                {
                                    node otherlink = (link1 == list2[i].link1) ? list2[i].link2 : list2[i].link1;
                                    for (int j = 0; j != list1.Count; j++)
                                    {
                                        if ((otherlink == list1[j].link1) || (otherlink == list1[j].link2))
                                        {
                                            if (k < 2)
                                            {
                                                CDIRows[k, 0] = i; //only one row is filled
                                                CDIRows[k, 1] = j;
                                                k++;
                                            }
                                        }
                                    }
                                }
                            }
                            //so one row is filled
                            //this condition is only for Pin-in-slot
                            //now we will call findinstantcenter function as below
                            
                            {
                                double[] ic = new double[2];
                                ic = findInstantCenter(CDIRows, list1.ToArray(), list2.ToArray(), circleDiagram, c,link1,link2);
                                circleDiagram[m].x = ic[0];
                                circleDiagram[m].y = ic[1];
                            }
                        }
                        else
                        {

                            for (int i = 0; i != list1.Count; i++)
                            {
                                node otherlink = (link1 == list1[i].link1) ? list1[i].link2 : list1[i].link1;
                                for (int j = 0; j != list2.Count; j++)
                                {
                                    if ((otherlink == list2[j].link1) || (otherlink == list2[j].link2))
                                    {
                                        if (k == 2)
                                        
                                            if (list1.Count > 2 || list2.Count > 2)
                                            {
                                                double s1nr, s1dr, s2nr, s2dr;


                                                // first row - two common points on a line
                                                double x00 = list1[(CDIRows[0, 0])].x;
                                                double y00 = list1[(CDIRows[0, 0])].y;
                                                double x01 = list2[(CDIRows[0, 1])].x;
                                                double y01 = list2[(CDIRows[0, 1])].y;


                                                // second row - two common points on a line
                                                double x110 = list1[(CDIRows[1, 0])].x;
                                                double y110 = list1[(CDIRows[1, 0])].y;
                                                double x11 = list2[(CDIRows[1, 1])].x;
                                                double y11 = list2[(CDIRows[1, 1])].y;

                                                s1nr = (y01 - y00);
                                                s1dr = (x01 - x00);
                                                s2nr = (y11 - y110);
                                                s2dr = (x11 - x110);

                                                if ((s1dr)==double.MaxValue || s1dr == 0 || double.IsInfinity(s1dr))
                                                {
                                                    if (double.IsInfinity(s2dr) || (s2dr)==double.MaxValue || s2dr == 0)
                                                        k--;
                                                }
                                                else if (s1nr == 0 || s2nr == 0)
                                                    k--;
                                            }



                                        

                                        if (k < 2)
                                        {

                                            CDIRows[k, 0] = i;
                                            CDIRows[k, 1] = j;
                                            k++;
                                        }
                                    }
                                }
                            }


                            if (k == 2) /* only when k=2 will we have found a set of four rows to extract an intstant center from. */
                            {
                                double[] ic = new double[2];
                                ic = findInstantCenter(CDIRows, list1.ToArray(), list2.ToArray(), circleDiagram, c, link1,link2);
                                circleDiagram[m].x = ic[0];
                                circleDiagram[m].y = ic[1];
                            }
                        }

                    }
            } while (unknownInstantCenters(circleDiagram) && NewICFoundInLastPass(circleDiagram));

            //the statement below is used to check 
            if (NewICFoundInLastPass(circleDiagram) == false)
                NaNtracker = 2.0;
        }
        #endregion
        #region Function: Find Immediate Pivots
        /// <summary>
        /// find immediate pivots and their positions (IC's) 
        /// </summary>
        /// <param name="circleDiagram"></param>
        private void findImmediatePivots(circleDiagramItem[] circleDiagram)
        {
            /* ok, now the rows of the circleDiagram have been given the unique pairings of
                     * links (link1 and link2). next go through each row of circle diagram, and find
                     * the Level 1 immediate instant centers that exist at common pivot points.
             The IC X and Y are determined in the next function*/


            for (int i = 0; i != circleDiagram.GetLength(0); i++)
            {
                circleDiagramItem row = circleDiagram[i];
                foreach (arc a in row.link1.arcs)
                {
                    node candidatePivot = a.otherNode(row.link1);
                    foreach (arc b in row.link2.arcs)
                        if (b.otherNode(row.link2) == candidatePivot)
                        {   //if it is a slider moving on a link,h-slider,v-slider or PIS, there is a slight modification to the code below
                            //if PIS, that particular node is actually calculated like the slider moving on a link.

                            if (candidatePivot.localLabels.Contains("pis") && !(candidatePivot.localLabels.Contains("slider")))
                                row.pivot = null;
                            //the above statement takes into account pis and not infy length pis
                            else
                                row.pivot = candidatePivot;
                            if (row.pivot!=null && row.pivot.localLabels.Contains("input"))
                            {
                                // row.speed = row.pivot.localVariables[0];
                                //row.alpha = row.pivot.localVariables[1];

                                row.speed = iOmega;
                                row.alpha = 0.0;
                            }

                        }
                }
            }

        }
        #endregion
        #region Function: ICs located on Pivot Points
        /// <summary>
        /// find immediate pivots and their positions (IC's)  
        /// </summary>
        /// <param name="circleDiagram"></param>
        private void findImmediateICs(circleDiagramItem[] circleDiagram)
        {
            foreach (circleDiagramItem row in circleDiagram)

                if (row.pivot != null)
                {
                    if (row.pivot.localLabels.Contains("slider") || row.pivot.localLabels.Contains("slideronalink"))
                    {
                        //row.X = double.MaxValue;
                        //row.Y = double.MaxValue;
                        if(row.pivot.localLabels.Contains("sliderh"))
                        {
                            row.x=row.pivot.X;
                            row.y=double.MaxValue;

                        }

                        else if (row.pivot.localLabels.Contains("sliderv"))
                        {
                            row.x = double.MaxValue;
                            row.y = row.pivot.X;

                        }


                    }
                    else
                    {

                        row.x = row.pivot.X;
                        row.y = row.pivot.Y;
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
            node findLink = circleDiagram[newInc].link2;
            node inputLink = circleDiagram[inputSpeedIndex].link2;


            for (int nInc = 0; nInc < sizeofCDI; nInc++)
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
        private List<circleDiagramItem> findCommonAndFulfilledRows(node link, circleDiagramItem[] circleDiagram)
        {
            List<circleDiagramItem> commonRows = new List<circleDiagramItem>();
            for (int i = 0; i != circleDiagram.GetLength(0); i++)
                if ((!double.IsNaN(circleDiagram[i].x))  //this checks to see if the row has been fulfilled
                    && ((circleDiagram[i].link1 == link) || ((circleDiagram[i].link2 == link))))
                    // this checks to see if the row has the same link as the input to the function - meaning it is common.
                    commonRows.Add(circleDiagram[i]);
            return commonRows;
        }
        #endregion
        #region Function: Unknown Instant centers
        private bool unknownInstantCenters(circleDiagramItem[] circleDiagram)
        {
            for (int i = 0; i != circleDiagram.GetLength(0); i++)
                if (double.IsNaN(circleDiagram[i].x))
                    return true;
            return false;
        }
        #endregion
        #region Function: Find instant Center
        private double[] findInstantCenter(int[,] CDIRows, circleDiagramItem[] list1, circleDiagramItem[] list2, circleDiagramItem[] circleDiagram, candidate c, node link1, node link2)
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
            circleDiagramItem arr = new circleDiagramItem();
            circleDiagramItem brr = new circleDiagramItem();
            circleDiagramItem crr = new circleDiagramItem();
            circleDiagramItem drr = new circleDiagramItem();

            double Slope0Nr = 0.0;
            double Slope0Dr = 0.0;
            double Slope1Nr = 0.0;
            double Slope1Dr = 0.0;

            double Slope0, Slope1;

            List<node> searchnode = new List<node>();



            double xc = double.NaN, yc = double.NaN;

            double x00 = 0.0;
            double y00 = 0.0;
            double x01 = 0.0;
            double y01 = 0.0;


            // second row - two common points on a line
            double x10 = 0.0;
            double y10 = 0.0;
            double x11 = 0.0;
            double y11 = 0.0;

            double xar, yar, xbr, ybr, xcr, ycr, xdr, ydr;

            int indexno=0;
            if (list1.GetLength(0) == 1 || list2.GetLength(0)==1)
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



                        foreach (arc ar in arr.link1.arcs)
                        {
                            if (ar.otherNode(arr.link1).localLabels.Contains("sliderh") || ar.otherNode(arr.link1).localLabels.Contains("sliderv")
                                    || ar.otherNode(arr.link1).localLabels.Contains("slideronalink"))//i have actually removed the pis checking in these statements
                                //which were earlier included
                                searchnode.Add(ar.otherNode(arr.link1));
                        }



                        foreach (arc ar in arr.link2.arcs)
                        {

                            if (arr.link1.localLabels.Contains("slideronalink_conn")
                                 || arr.link1.localLabels.Contains("slider_conn"))
                            {

                                foreach (arc ar1 in arr.link1.arcs)
                                {
                                    if (ar1.otherNode(arr.link1) != searchnode[0])
                                        searchnode.Add(ar1.otherNode(arr.link1));
                                    if (searchnode[0] != ar.otherNode(arr.link2))
                                        searchnode.Add(ar.otherNode(arr.link2));
                                }

                            }

                            else if (arr.link2.localLabels.Contains("slideronalink_conn")
                                 || arr.link1.localLabels.Contains("slider_conn"))
                            {
                                foreach (arc ar1 in arr.link2.arcs)
                                {
                                    if (ar1.otherNode(arr.link2) != searchnode[0])
                                        searchnode.Add(ar1.otherNode(arr.link2));
                                    if (searchnode[0] != ar.otherNode(arr.link1))
                                        searchnode.Add(ar.otherNode(arr.link1));
                                }

                            }
                        }

                        //now that searchnode[0] consists of sliderh,sliderv,pis,slideronalink X&Y, searchnode[1]&[2] - contains the other X's & Y's typically used for slideronalink

                        if (searchnode[0].localLabels.Contains("sliderh"))
                        {
                            xbr = brr.x;
                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);
                            double B0 = ycr - Slope1 * xcr;
                            xc = xbr;
                            yc = Slope1 * xc + B0;

                        }
                        else if (searchnode[0].localLabels.Contains("sliderv"))
                        {
                            ybr = brr.x;
                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);
                            double B0 = ycr - Slope1 * xcr;
                            yc = ybr;
                            xc = (yc - B0) / Slope1;
                        }
                        else if (searchnode[0].localLabels.Contains("slideronalink"))
                        {

                            xar = searchnode[1].X;
                            yar = searchnode[1].Y;
                            xbr = searchnode[2].X;
                            ybr = searchnode[2].Y;
                            double Slope10 = (ybr - yar) / (xbr - xar);
                            Slope0 = -1 / Slope10;


                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);

                            double B0 = searchnode[2].Y - Slope0 * searchnode[2].X;
                            double B1 = searchnode[1].Y - Slope1 * searchnode[1].X;
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
                            double B0 = y00 - Slope0 * x00;
                            double B1 = y10 - Slope1 * x10;
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



                        foreach (arc ar in arr.link1.arcs)
                        {
                            if (ar.otherNode(arr.link1).localLabels.Contains("sliderh") || ar.otherNode(arr.link1).localLabels.Contains("sliderv")
                                    || ar.otherNode(arr.link1).localLabels.Contains("slideronalink"))//i have actually removed the pis checking in these statements
                                //which were earlier included
                                searchnode.Add(ar.otherNode(arr.link1));
                        }



                        foreach (arc ar in arr.link2.arcs)
                        {

                            if (arr.link1.localLabels.Contains("slideronalink_conn")
                                 || arr.link1.localLabels.Contains("slider_conn"))
                            {

                                foreach (arc ar1 in arr.link1.arcs)
                                {
                                    if (ar1.otherNode(arr.link1) != searchnode[0])
                                        searchnode.Add(ar1.otherNode(arr.link1));
                                    if (searchnode[0] != ar.otherNode(arr.link2))
                                        searchnode.Add(ar.otherNode(arr.link2));
                                }

                            }

                            else if (arr.link2.localLabels.Contains("slideronalink_conn")
                                 || arr.link1.localLabels.Contains("slider_conn"))
                            {
                                foreach (arc ar1 in arr.link2.arcs)
                                {
                                    if (ar1.otherNode(arr.link2) != searchnode[0])
                                        searchnode.Add(ar1.otherNode(arr.link2));
                                    if (searchnode[0] != ar.otherNode(arr.link1))
                                        searchnode.Add(ar.otherNode(arr.link1));
                                }

                            }
                        }

                        //now that searchnode[0] consists of sliderh,sliderv,pis,slideronalink X&Y, searchnode[1]&[2] - contains the other X's & Y's typically used for slideronalink

                        if (searchnode[0].localLabels.Contains("sliderh"))
                        {
                            xbr = brr.x;
                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);
                            double B0 = ycr - Slope1 * xcr;
                            xc = xbr;
                            yc = Slope1 * xc + B0;

                        }
                        else if (searchnode[0].localLabels.Contains("sliderv"))
                        {
                            ybr = brr.x;
                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);
                            double B0 = ycr - Slope1 * xcr;
                            yc = ybr;
                            xc = (yc - B0) / Slope1;
                        }
                        else if (searchnode[0].localLabels.Contains("slideronalink"))
                        {

                            xar = searchnode[1].X;
                            yar = searchnode[1].Y;
                            xbr = searchnode[2].X;
                            ybr = searchnode[2].Y;
                            double Slope10 = (ybr - yar) / (xbr - xar);
                            Slope0 = -1 / Slope10;


                            xcr = crr.x;
                            ycr = crr.y;
                            xdr = drr.x;
                            ydr = drr.y;
                            Slope1 = (ydr - ycr) / (xdr - xcr);

                            double B0 = searchnode[2].Y - Slope0 * searchnode[2].X;
                            double B1 = searchnode[1].Y - Slope1 * searchnode[1].X;
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
                            double B0 = y00 - Slope0 * x00;
                            double B1 = y10 - Slope1 * x10;
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
                if (link1.localLabels.Contains("pis_conn") || link2.localLabels.Contains("pis_conn"))
                {

                    x00 = list2[(CDIRows[0, 0])].x;
                    y00 = list2[(CDIRows[0, 0])].y;
                    x01 = list1[(CDIRows[0, 1])].x;
                    y01 = list1[(CDIRows[0, 1])].y;

                    double slope0Dr = x01 - x00;
                    double slope0Nr = y01 - y00;

                    Slope0 = (y01 - y00) / (x01 - x00);


                    if (!(list1[(CDIRows[0, 0])].link2.localLabels.Contains("pis_conn")))
                        foreach (arc a in list1[(CDIRows[0, 0])].link2.arcs)
                        {
                            if (a.otherNode(list1[(CDIRows[0, 0])].link2).localLabels.Contains("pis"))
                            {
                                searchnode.Add(a.otherNode(list1[(CDIRows[0, 0])].link2));
                                searchnode.Add(list1[(CDIRows[0, 0])].link2);
                                searchnode.Add(list2[(CDIRows[0, 0])].link2);
                            }

                        }
                    if (!(list2[(CDIRows[0, 0])].link2.localLabels.Contains("pis_conn")))
                        foreach (arc a in list2[(CDIRows[0, 0])].link2.arcs)
                        {
                            if (a.otherNode(list2[(CDIRows[0, 0])].link2).localLabels.Contains("pis"))
                            {
                                searchnode.Add(a.otherNode(list2[(CDIRows[0, 0])].link2));
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

                        foreach (arc b in searchnode[2].arcs)
                            if (!b.otherNode(searchnode[2]).localLabels.Contains("pis"))
                                searchnode.Add(b.otherNode(searchnode[2]));

                        //the above statement might not work if the pis is connected to a triangular plate or quad node?
                        //will it work for circular slot? may not - as of now this work only for straight line paths



                        x10 = searchnode[3].X;
                        y10 = searchnode[3].Y;
                        x11 = searchnode[4].X;
                        y11 = searchnode[4].Y;


                        Slope1Nr = y11 - y10;
                        Slope1Dr = x11 - x10;

                        if (Slope1Dr == 0) //this statement also takes into account the vertical position of the PIS slot link
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


            return new double[] { xc, yc };


        }
        #endregion
        #region Function: New IC Found In Last Pass
        //this has been taken from Position
        List<Boolean> ICsFound = new List<Boolean>(); // this is only used in function below. Can it be made a local variable?
        private bool NewICFoundInLastPass(circleDiagramItem[] circleDiagram)
        {
            /* assume, at first, that no change has occured (result = false),
             * then if any change is found switch it to true. */
            Boolean result = false;
            /* a change is search in the following for-loop. A change means
             * that the position (or just the X-coord in this case) is an
             * actual number - no longer NaN, but if the corresponding 
             * boolean in PositionsFound is false, then last pass it was
             * NaN. Therefore a change has occured - set result to true. */
            for (int i = 0; i < circleDiagram.GetLength(0); i++)
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
                for (int i = 0; i < circleDiagram.GetLength(0); i++)
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