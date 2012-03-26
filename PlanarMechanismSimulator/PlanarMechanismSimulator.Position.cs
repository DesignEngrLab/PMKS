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
        #region New Position Found in Last Pass

        private readonly List<Boolean> PositionsFound = new List<Boolean>();
        //Campbell: I'm worried about this list. All calls occur in this first method, except for a clear on line 579.

        private bool NewPositionFoundInLastPass(List<pivot> pivots1)
        {
            /* assume, at first, that no change has occured (result = false),
             * then if any change is found switch it to true. */
            var result = false;
            /* a change is search in the following for-loop. A change means
             * that the position (or just the X-coord in this case) is an
             * actual number - no longer NaN, but if the corresponding 
             * boolean in PositionsFound is false, then last pass it was
             * NaN. Therefore a change has occured - set result to true. */
            for (var i = 0; i < pivots1.Count; i++)
                if (!double.IsNaN(pivots1[i].X) &&
                    ((PositionsFound.Count <= i) || !PositionsFound[i]))
                {
                    result = true;
                    break;
                }
            /* Update PositionsFound
             * If true then we now need to change any PositionsFound to true
             * that were previously false but which now have a value. */
            if (result)
            {
                for (var i = 0; i < pivots1.Count; i++)
                {
                    if (PositionsFound.Count <= i)
                        PositionsFound.Add(false);
                    if (!double.IsNaN(pivots1[i].X))
                        PositionsFound[i] = true;
                }
            }
            return result;
        }

        #endregion

        #region Find Link Lengths - this is a static value - BUT GOT TO ADD NAN TO POSITIONS THAT DO NOT HAVE A DIRECT LINK


        #endregion

        #region Function: Find New Positions

        private Boolean findNewPositions(int timeRow, double[,] pivotLengths)
        {
            #region Perform Numerical integration

            //int i;
            //for (i = 0; i < p; i++)
            //{

            //    if (timeRow != numTimeSteps-1)
            //    {
            //        pivotParameters[i, timeRow + 1, 0] = pivotParameters[i, timeRow, 2] * newt
            //            + 0.5 * pivotParameters[i, timeRow, 4] * newt * newt
            //                + pivotParameters[i, timeRow, 0];
            //        pivotParameters[i, timeRow + 1, 1] = pivotParameters[i, timeRow, 3] * newt
            //            + 0.5 * pivotParameters[i, timeRow, 5] * newt * newt
            //            + pivotParameters[i, timeRow, 1];
            //    }

            //}

            #endregion

            #region Analytical Postion via. Circle intersection / circle line intersection

            var pivots1 = new List<pivot>(pivots);

            //now adding pivots

            //need to copy the pivots position to a matrix [p,2]
            //we shall compare the straightline position between the new position(from values) and the old position
            //depending on which is less, we shall select

            var oldpivotpositions = new double[p, 2];

            for (var pp_1 = 0; pp_1 < p; pp_1++)
            {
                oldpivotpositions[pp_1, 0] = pivots1[pp_1].X;
                oldpivotpositions[pp_1, 1] = pivots1[pp_1].Y;
            }

            //shall do the input pivot link
            //we shall do it for each pivot connected to the input link

            #region Input Link New Position

            for (var x = 0; x < p; x++)
                if (pivots[x].IsGround && pivots[x] == inputpivot)
                {
                    foreach (var cc in pivots[x].Links)
                        foreach (var otherPiv in cc.Pivots.Where(piv => piv != pivots[x]))
                        {
                            if (!otherPiv.IsGround && otherPiv.PivotType != PivotTypes.PX && otherPiv.PivotType != PivotTypes.PY)
                            {
                                findInputLink_NewPositions(otherPiv.index, pivots1, pivotLengths, v, x);
                            }
                        }
                }

            #endregion

            foreach (var pp in pivots1)
                if ((!pp.IsGround) && (!pp.newp) && ((pp.PivotType != PivotTypes.RPX) || (pp.PivotType != PivotTypes.RPY)))
                {
                    pp.X = double.NaN;
                    pp.Y = double.NaN;
                }


            //if there is a pin-in-slot (or slideronalink) that is connected to the input link
            //then find the positions of the other pivot that is connected to the Pis by intersection
            //should think about merging pis and slideronalink

            #region PIS connected to the input

            foreach (var pp in pivots1)
                if (pp.newp && pp.localLabels.Contains("pis"))
                {
                    foreach (arc aa in pp.arcs)
                        if (aa.localLabels.Contains("pivotarc"))
                        {
                            if (!aa.otherNode(pp).localLabels.Contains("input") && !double.IsNaN(aa.otherNode(pp).X))
                            {
                                //try to determine the gradient

                                //determine to which other node is this connected - not ground for sure
                                //determine the length between the two nodes
                                //intersect on this gradient the length and that forms the new position

                                var c1 = aa.otherNode(pp);
                                node c2 = null;

                                //get the other node which has a NAN and which is directly to this node and the pis

                                foreach (arc ab in c1.arcs)
                                {
                                    if (ab.localLabels.Contains("pivotarc"))
                                        if (double.IsNaN(ab.otherNode(c1).X))
                                        {
                                            foreach (arc ac in ab.otherNode(c1).arcs)
                                                if (ac.localLabels.Contains("pivotarc"))
                                                    if (ac.otherNode(ab.otherNode(c1)) == pp)
                                                        c2 = ab.otherNode(c1);
                                        }
                                }

                                //get the slope between pp & node c1;

                                var SlopeDr = pp.X - c1.X;
                                var SlopeNr = pp.Y - c1.Y;


                                //get the length between c1 and c2

                                int bb1, bb2;

                                for (bb1 = 0; bb1 < p; bb1++)
                                    if (pivots1[bb1] == c1)
                                        break;

                                for (bb2 = 0; bb2 < p; bb2++)
                                    if (pivots1[bb2] == c2)
                                        break;


                                var length = pivotLengths[bb1, bb2];

                                //determine whether the slope is 0 or infinity or normal
                                // intersect the circle and the line
                                if (SlopeDr == 0)
                                {
                                    pivots1[bb2].X = c1.X;
                                    pivots1[bb2].Y = c1.Y + length;
                                }

                                else if (SlopeNr == 0)
                                {
                                    pivots1[bb2].X = c1.X + length;
                                    pivots1[bb2].Y = c1.Y;
                                }

                                else
                                {
                                    var Actual_Slope = SlopeNr / SlopeDr;

                                    var x_1 = c1.X;
                                    var y_1 = c1.Y;

                                    //double x_value_1 = (-2.0 * x_1 + Math.Sqrt((2.0 * x_1) * (2.0 * x_1) + 4.0 * 1.0 * (length * length / (1.0 + Actual_Slope * Actual_Slope))))/(2.0);
                                    //double x_value_2 = (-2.0 * x_1 - Math.Sqrt((2.0 * x_1) * (2.0 * x_1) + 4.0 * 1.0 * (length * length / (1.0 + Actual_Slope * Actual_Slope))))/2.0;


                                    var x_value_1 = Math.Sqrt((length * length) / (1 + Actual_Slope * Actual_Slope)) + x_1;
                                    ;
                                    var x_value_2 = -Math.Sqrt((length * length) / (1 + Actual_Slope * Actual_Slope)) + x_1;
                                    ;

                                    var y_value_1 = Actual_Slope * (x_value_1 - x_1) + y_1;
                                    var y_value_2 = Actual_Slope * (x_value_2 - x_1) + y_1;

                                    //depending on which one - add it to the c2.X term;

                                    if ((pp.Y < 0 && pp.X > 0) || (pp.X > 0 && pp.Y > 0))
                                    {
                                        pivots1[bb2].X = x_value_1;
                                        pivots1[bb2].Y = y_value_1;
                                    }
                                    else
                                    {
                                        pivots1[bb2].X = x_value_2;
                                        pivots1[bb2].Y = y_value_2;
                                    }
                                }
                            }
                        }
                }

            #endregion

            while (ContainsUnknownPositions(pivots1) && NewPositionFoundInLastPass(pivots1))
            {
                //one would be the pivot which would have the new position
                //which means we should add a label to the pivot once we determine the new position
                //this would help in selecting one circle and other would be the one that is connected to the ground
                //and this particular pivot

                //step - select the pivots that has new position and has another pivot 

                for (var y = 0; y < p; y++)
                {
                    if (double.IsNaN(pivots1[y].X))
                    {
                        if (pivots1[y].PivotType == PivotTypes.PX || pivots1[y].PivotType == PivotTypes.PY
                            || pivots1[y].Contains("slider_conn") || pivots1[y].Contains("slider"))
                        {
                            #region slider:Circle - Line intersection

                            node c1 = null;
                            node cc = null;

                            if (pivots1[y].localLabels.Contains("slider"))
                            {
                                foreach (arc ar in pivots1[y].arcs)
                                    if (ar.localLabels.Contains("pivotarc"))
                                        if (!double.IsNaN(ar.otherNode(pivots1[y]).X) &&
                                            ar.otherNode(pivots1[y]).localLabels.Contains("slider_conn"))
                                        {
                                            cc = ar.otherNode(pivots1[y]);

                                            foreach (arc arc1 in cc.arcs)
                                                if (arc1.localLabels.Contains("pivotarc"))
                                                    if (!double.IsNaN(arc1.otherNode(cc).X) &&
                                                        !arc1.otherNode(cc).localLabels.Contains("slider"))
                                                        c1 = arc1.otherNode(cc);
                                        }
                            }


                            int no1 = 0, no12 = 0;

                            //find where exactly lies point lies in pivots

                            for (no1 = 0; no1 < p; no1++)
                                if (pivots[no1] == c1)
                                    break;

                            for (no12 = 0; no12 < p; no12++)
                                if (pivots[no12] == cc)
                                    break;

                            //get the length

                            var length1 = pivotLengths[no12, no1];

                            //find out whether the other end is a slider h or slider v

                            string whatSlider = null;

                            if (pivots1[y].localLabels.Contains("sliderh"))
                                whatSlider = "h";
                            if (pivots1[y].localLabels.Contains("sliderv"))
                                whatSlider = "v";

                            var values = new List<double>();

                            FindCircleLineIntersection(cc, c1, whatSlider, length1, values);

                            //we also have the index no - but we also need to determine index no of c2 node

                            //add to values to pivots

                            //now depending on the sign convention, we need to choose values from values.list


                            if ((cc.X >= 0 && cc.Y >= 0) || (cc.X >= 0 && cc.Y <= 0))
                            {
                                pivots1[y].X = values[0];
                                pivots1[y].Y = values[1];


                                pivots1[no12].X = values[0];
                                pivots1[no12].Y = values[1];
                            }
                            else if ((cc.X <= 0 && cc.Y <= 0) || (cc.X <= 0 && cc.Y <= 0))
                            {
                                pivots1[y].X = values[2];
                                pivots1[y].Y = values[3];


                                pivots1[no12].X = values[2];
                                pivots1[no12].Y = values[3];
                            }

                            #endregion
                        }
                        else if (!pivots1[y].localLabels.Contains("slider_conn") &&
                                 !pivots1[y].localLabels.Contains("slider") &&
                                 !pivots1[y].localLabels.Contains("pis"))
                        {
                            #region Circle Circle Intersection - For general Case - not sliders

                            var N_for_intersection = new List<node>();
                            foreach (arc ar in pivots1[y].arcs)
                                if (ar.localLabels.Contains("pivotarc"))
                                    if (!double.IsNaN(ar.otherNode(pivots1[y]).X))
                                        N_for_intersection.Add(ar.otherNode(pivots1[y]));

                            var values = new List<double>();
                            if (N_for_intersection.Count >= 2)
                            {
                                //need to determine lengths between pivot points

                                int no1, no2;
                                for (no1 = 0; no1 < p; no1++)
                                    if (pivots[no1] == N_for_intersection[0])
                                        break;

                                for (no2 = 0; no2 < p; no2++)
                                    if (pivots[no2] == N_for_intersection[1])
                                        break;

                                var length1 = pivotLengths[y, no1];
                                var length2 = pivotLengths[y, no2];


                                Find_New_Intersection_points(length1, length2, N_for_intersection[0],
                                                             N_for_intersection[1], values);

                                //we know which pivot and also the two different possibilities from values.list
                                //we shall use the straight-line distance determination technique to get the right value. 

                                double st1_x = 0.0, st1_y = 0.0;

                                st1_x = oldpivotpositions[y, 0];
                                st1_y = oldpivotpositions[y, 1];

                                if (double.IsNaN(values[0]) && double.IsNaN(values[1]) && double.IsNaN(values[2]) &&
                                    double.IsNaN(values[3]))
                                {
                                    pivots1[y].X = values[0];
                                    pivots1[y].Y = values[1];
                                }
                                else
                                {
                                    //straight-line distance

                                    double st_d1 = 0.0, st_d2 = 0.0;

                                    st_d1 =
                                        Math.Sqrt((st1_x - values[0]) * (st1_x - values[0]) +
                                                  (st1_y - values[1]) * (st1_y - values[1]));
                                    st_d2 =
                                        Math.Sqrt((st1_x - values[2]) * (st1_x - values[2]) +
                                                  (st1_y - values[3]) * (st1_y - values[3]));

                                    if (st_d1 < st_d2)
                                    {
                                        pivots1[y].X = values[0];
                                        pivots1[y].Y = values[1];
                                    }

                                    else
                                    {
                                        pivots1[y].X = values[2];
                                        pivots1[y].Y = values[3];
                                    }
                                }
                            }

                            #endregion
                        }

                        else if (pivots1[y].localLabels.Contains("pis"))
                        {
                            #region PIS not connected to the input

                            //if it is a pis, we know that one end could be connected to a ground
                            //so this would help in determining the distance between pis and ground pivot
                            //and also the position of pis on the link initially

                            //this problem is also a circle-line intersection


                            //first determine the other ground

                            node abab = null;
                            foreach (arc ab in pivots1[y].arcs)
                                if (ab.localLabels.Contains("pivotarc") &&
                                    ab.otherNode(pivots1[y]).localLabels.Contains("ground"))
                                    abab = ab.otherNode(pivots1[y]);

                            //determine the index of abab and length from the pis pivot
                            int x;
                            for (x = 0; x < p; x++)
                                if (pivots1[x] == abab)
                                    break;

                            var length1 = pivotLengths[y, x];

                            //now determine the line on which the pis is located

                            foreach (arc ab in pivots1[y].arcs)
                                if (ab.otherNode(pivots1[y]).localLabels.Contains("link") &&
                                    ab.otherNode(pivots1[y]).localLabels.Contains("pis_conn"))
                                {
                                    var pis_link = ab.otherNode(pivots1[y]);

                                    //now determine the two pivots on this link other than pis

                                    //list of nodes
                                    var pis_link_nodes = new List<node>();

                                    foreach (arc ab1 in pis_link.arcs)
                                        if (ab1.otherNode(pis_link).localLabels.Contains("pivot") &&
                                            !ab1.otherNode(pis_link).localLabels.Contains("pis"))
                                            pis_link_nodes.Add(ab1.otherNode(pis_link));


                                    //now to get the circle-line intersection working


                                    double xx1, yy1, xx2, yy2;

                                    xx1 = pis_link_nodes[0].X;
                                    yy1 = pis_link_nodes[0].Y;
                                    xx2 = pis_link_nodes[1].X;
                                    yy2 = pis_link_nodes[1].Y;

                                    var slope = (yy2 - yy1) / (xx2 - xx1);

                                    var C = yy1 - slope * xx1;

                                    double Delta;

                                    Delta = Math.Pow((-2 * abab.X + 2 * (C - abab.Y) * (slope)), 2) -
                                            4 * (slope * slope + 1) *
                                            (abab.X * abab.X + (C - abab.Y) * (C - abab.Y) - length1 * length1);
                                    double value_x1 = 0.0, value_x2 = 0.0, value_y_1 = 0.0, value_y_2 = 0.0;

                                    if (Delta > 0)
                                    {
                                        value_x1 = (-(-2 * abab.X + 2 * (C - abab.Y) * slope) + Math.Sqrt(Delta)) /
                                                   (2 * (slope * slope + 1));
                                        value_x2 = (-(-2 * abab.X + 2 * (C - abab.Y) * slope) - Math.Sqrt(Delta)) /
                                                   (2 * (slope * slope + 1));
                                        value_y_1 = slope * value_x1 + C;
                                        value_y_2 = slope * value_x2 + C;
                                    }

                                    else
                                    {
                                    }

                                    //perhaps compare with the previous point
                                    //not sure if this is fool proof

                                    //find the minimum distance and then go about

                                    var dist1 =
                                        Math.Sqrt((oldpivotpositions[y, 0] - value_x1) *
                                                  (oldpivotpositions[y, 0] - value_x1) +
                                                  (oldpivotpositions[y, 1] - value_y_1) *
                                                  (oldpivotpositions[y, 0] - value_y_1));


                                    var dist2 =
                                        Math.Sqrt((oldpivotpositions[y, 0] - value_x2) *
                                                  (oldpivotpositions[y, 0] - value_x2) +
                                                  (oldpivotpositions[y, 1] - value_y_2) *
                                                  (oldpivotpositions[y, 0] - value_y_2));


                                    if (dist1 < dist2)
                                    {
                                        pivots1[y].X = value_x1;
                                        pivots1[y].X = value_y_1;
                                    }
                                    else
                                    {
                                        pivots1[y].X = value_x2;
                                        pivots1[y].Y = value_y_2;
                                    }
                                }

                            #endregion
                        }

                        #region Stephenson II

                        #endregion
                    }
                }


                //removing "newp" label from all pivots (here it is used only for input link
                foreach (var a in pivots1) a.newp = false;
            }
            /* After the while-loop terminates, we reset the PositionsFound list. */
            PositionsFound.Clear();

            #endregion

            #region Copy values to Pivots X and Y

            for (var t = 0; t < p; t++)
            {
                if (timeRow != numSteps - 1)
                {
                    if (!double.IsNaN(pivots1[t].X))
                    {
                        PivotParameters[t, timeRow + 1, 0] = pivots1[t].X;
                        PivotParameters[t, timeRow + 1, 1] = pivots1[t].Y;
                    }

                    pivots[t].X = PivotParameters[t, timeRow + 1, 0];
                    pivots[t].Y = PivotParameters[t, timeRow + 1, 1];
                }
            }

            //for (i = 0; i < p; i++)
            //    if (pivots[i].localLabels.Contains("output"))
            //    {
            //        path[generalcounter, 0] = pivots[i].X;
            //        path[generalcounter, 1] = pivots[i].Y;
            //    }
            // make sure to put positions both in pivotparameters as well as back into the graph (just in pivots list)

            #endregion

            return checkrotatability(pivots1);
        }

        #region Check if there is any NAN in the pivots

        private bool checkrotatability(List<pivot> pivots1)
        {
            //if there is any NAN for any pivots.x / pivots.y , then system.output and also exit this candidate
            var result = true;
            for (var tempi = 0; tempi < p; tempi++)
                if (pivots1[tempi].X == double.NaN || pivots1[tempi].Y == double.NaN)
                {
                    SearchIO.output("Failed Candidate - Rotatability not satisfied");
                    result = false;
                    return result;


                    //how to exit this particular candidate????
                    //can I have a variable which is defined in setup and which will track this condition?
                    //which can be used even for circle diagram too. 
                    //if NaNtracker is 1.0; then there is NaN and the system is no good.
                }
            return result;
        }

        #endregion

        #endregion

        #region Circle - Line Intersection for Sliders

        private void FindCircleLineIntersection(pivot c1, pivot c2, string whatSlider, double length1, List<double> values)
        {
            //calculating the intersection points
            //first is determining b^2 - 4*a*c term

            //X Y coordinates of the point c1

            var xx = c2.X;
            var yy = c2.Y;

            double horz = double.NaN, vert = double.NaN;

            if (whatSlider == "h")
                horz = c1.Y;
            if (whatSlider == "v")
                vert = c1.X;


            //write a if statement to account for both horz and vert sliders

            if (!double.IsNaN(horz))
            {
                var delta = (2 * xx) * (2 * xx) - 4 * 1 * (xx * xx + (horz - yy) * (horz - yy) - length1 * length1);

                if (delta > 0)
                {
                    values.Add((2 * xx + Math.Sqrt(delta)) / 2);
                    values.Add(horz);
                    values.Add((2 * xx - Math.Sqrt(delta)) / 2);
                    values.Add(horz);
                }

                else if (delta < 0 || delta == 0)
                {
                    values.Add(double.NaN);
                    values.Add(double.NaN);
                    values.Add(double.NaN);
                    values.Add(double.NaN);
                }
            }

            if (!double.IsNaN(vert))
            {
                var delta = (2 * yy) * (2 * yy) - 4 * 1 * (yy * yy + (vert - xx) * (vert - xx) - length1 * length1);

                if (delta > 0)
                {
                    values.Add(vert);
                    values.Add((2 * yy + Math.Sqrt(delta)) / 2);
                    values.Add(vert);
                    values.Add((2 * yy - Math.Sqrt(delta)) / 2);
                }
                else if (delta < 0 || delta == 0)
                {
                    values.Add(double.NaN);
                    values.Add(double.NaN);
                    values.Add(double.NaN);
                    values.Add(double.NaN);
                }
            }
        }

        #endregion

        #region New Intersection Points

        private void Find_New_Intersection_points(double length1, double length2, pivot c1, pivot c2, List<double> values)
        {
            //code obtained from 
            //http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/
            //Intersection of two circles
            //Written by Paul Bourke
            //April 1997
            //Contributions:
            //C source code example by Tim Voght

            //vertical and horizontal distances between the centers of the circles

            double x_1, x_2, y_1, y_2;
            double vertx = 0.0, verty = 0.0;
            vertx = -c1.X + c2.X;
            verty = -c1.Y + c2.Y;

            //straightline distance between the centers
            double d;
            d = Math.Sqrt(vertx * vertx + verty * verty);

            //shall add conditions for solvability - this is important
            //this shall be done once the code is working for a 4 bar mechanism


            if (d > (length1 + length2))
            {
                values.Add(double.NaN);
                values.Add(double.NaN);
                values.Add(double.NaN);
                values.Add(double.NaN);
            }
            else if (d < Math.Abs(length1 - length2))
            {
                values.Add(double.NaN);
                values.Add(double.NaN);
                values.Add(double.NaN);
                values.Add(double.NaN);
            }

            else
            {
                //determine distance from point 0 to point 2

                double a;
                a = ((length1 * length1) - (length2 * length2) + d * d) / (2.0 * d);

                //determine the coordinates

                double x2, y2;

                x2 = c1.X + (vertx * a / d);
                y2 = c1.Y + (verty * a / d);

                //distance from either of the intersection points

                double h;

                h = Math.Sqrt(length1 * length1 - a * a);

                double rx, ry;

                rx = -verty * (h / d);
                ry = vertx * (h / d);

                //determine absolute intersection points

                x_1 = x2 + rx;
                x_2 = x2 - rx;
                y_1 = y2 + ry;
                y_2 = y2 - ry;

                values.Add(x_1);
                values.Add(y_1);
                values.Add(x_2);
                values.Add(y_2);
            }
        }

        #endregion

        #region Input Link New Position Determination

        private void findInputLink_NewPositions(pivot qw, List<pivot> pivots1, double[,] pivotlengths, int v, int x)
        {
            double x_pos, y_pos;
            double linklength;
            linklength = pivotlengths[x, v];
            var xx = qw.X;
            var yy = qw.Y;
            double x2 = 0.0, y2 = 0.0;

            for (var i = 0; i < p; i++)
                if (pivots1[i].localLabels.Contains("input"))
                {
                    x2 = pivots1[i].X;
                    y2 = pivots1[i].Y;
                }


            var x_distance = xx - x2;
            var y_distance = yy - y2;

            var theta = Math.Atan2(y_distance, x_distance) + (inputSweepAngle / numSteps);

            x_pos = linklength * Math.Cos(theta);
            y_pos = linklength * Math.Sin(theta);

            var actualx = x_pos + x2;
            var actualy = y_pos + y2;
            pivots1[v].X = actualx;
            pivots1[v].Y = actualy;
            pivots1[v].newp = true;
        }

        #endregion

        #region To check if pivots have been given new positions

        private Boolean ContainsUnknownPositions(List<pivot> pivots1)
        {
            foreach (var p in pivots1)
            {
                if (double.IsNaN(p.X))
                    return true;
            }

            return false;
        }

        #endregion

        void IDependentAnalysis.calculate(double[] x)
        {
            throw new NotImplementedException();
        }
    }
}