#region

using System;
using System.Linq;
using System.Collections.Generic;
using OptimizationToolbox;
using System.Collections;

#endregion

namespace PlanarMechanismSimulator
//at time t=0; all acceleration and velocity are zero
{
    public partial class Simulator : IDependentAnalysis
    {
        private circleDiagramItem[] circleDiagram;
        private int sizeOfCircDiag;

        private void SetUpDyadicVelocityObjects()
        {
            #region fill up link spots in CircleDiagram
            sizeOfCircDiag = numLinks * (numLinks - 1) / 2;
            circleDiagram = new circleDiagramItem[sizeOfCircDiag];
            for (int i = 0; i <= inputJointIndex; i++)
                circleDiagram[0] = new circleDiagramItem(joints[i].Link1, joints[i].Link2, joints[i]);
            /* so, the position on the input joint in the circle diagram is in the same spot as the inputJoint
             * in the list of joints. This is because we add all the immediate instant centers first. So the two lists
             * (circleDiagram and joints) start the same. */
            int k = inputJointIndex + 1;
            for (int i = 0; i < numLinks - 1; i++)
                for (int j = i + 1; j < numLinks; j++)
                    if (!links[i].joints.Intersect(links[j].joints).Any())
                        /* if there are not any common joints between the two links then we add them (if there were
                         * they would have been added in the first for-loop. */
                        circleDiagram[k++] = new circleDiagramItem(links[i], links[j], null);
            #endregion

            #region

            #region Forming the acceleration node matrix

            // change to lists
            var coriolis1 = new List<DynamicMatrixTerm>();
            var Omeg = new List<DynamicMatrixTerm>();
            var unknownsList = new List<DynamicMatrixTerm>();

            for (int i = 0; i < inputLinkIndex; i++)
            {
                var thisLink = links[i];
                var firstJoint = thisLink.joints[0];
                unknownsList.Add(new LinkDynamicMatrixTerm()
                                     {
                                         belongsTo = thisLink,
                                         type = DynamicType.angularAcceleration
                                     });

                //we are not adding slider to acceleration determination - this will be copied from the velocity of slider_conn pivot
                if (firstJoint.jointType == JointTypes.R)
                    addToUnknowns(firstJoint, unknownsList);

                //unknownsList[row2].belongsTo = link;
                //unknownsList[row2].type = DynamicType.angularAcceleration;
                //if (link.localLabels.Contains("ground")) unknownsList[row2].defaultValue = 0.0;
                //else unknownsList[row2].defaultValue = double.NaN;
                //row2++;
            }
            for (int j = 1; j < thisLink.joints.Count; j++)
            {
                joint pivot1 = thisLink.joints[j];

                if (!thisLink.isGround && !thisLink.Contains("slider_conn"))
                {
                    Omeg.Add(new LinkDynamicMatrixTerm()
                                 {
                                     belongsTo = thisLink,
                                     type = DynamicType.angularVelocity,
                                     dir = Direction.X
                                 });

                    //if (link.localLabels.Contains("ground")) Omeg[row].defaultValue = 0.0;
                    //else Omeg[row].defaultValue = double.NaN;

                    if ((firstJoint.Contains("pis")) || (pivot1.localLabels.Contains("pis")) ||
                        (firstJoint.Contains("slideronalink")) ||
                        (pivot1.Contains("slideronalink")))
                    {
                        if (thisLink.Contains("pis_conn"))

                            coriolis1.Add(new PivotDynamicMatrixTerm()
                                              {

                                                  belongsFrom = firstJoint,
                                                  belongsTo = pivot1,
                                                  type = DynamicType.radialVelocity,
                                                  dir = Direction.X
                                              });
                        else
                            coriolis1.Add(new PivotDynamicMatrixTerm()
                                              {

                                                  belongsFrom = firstJoint,
                                                  belongsTo = pivot1,
                                                  type = DynamicType.radialVelocity,
                                                  dir = Direction.X,
                                                  defaultValue = 0.0
                                              });
                    }
                    else
                        coriolis1.Add(new PivotDynamicMatrixTerm()
                                          {

                                              belongsFrom = firstJoint,
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

                if (!thisLink.isGround && !thisLink.Contains("slider_conn"))
                {
                    Omeg.Add(new LinkDynamicMatrixTerm()
                                 {
                                     belongsTo = thisLink,
                                     type = DynamicType.angularVelocity,
                                     dir = Direction.Y
                                 });


                    //if (link.localLabels.Contains("ground")) Omeg[row].defaultValue = 0.0;
                    //else Omeg[row].defaultValue = double.NaN;

                    if ((firstJoint.Contains("pis")) || (pivot1.Contains("pis")) ||
                        (firstJoint.Contains("slideronalink")) ||
                        (pivot1.Contains("slideronalink")))
                    {
                        if (thisLink.Contains("pis_conn"))
                        {
                            coriolis1.Add(new PivotDynamicMatrixTerm()
                                              {
                                                  belongsFrom = firstJoint,
                                                  belongsTo = pivot1,
                                                  type = DynamicType.radialVelocity,
                                                  dir = Direction.Y
                                              });
                        }
                        else
                            coriolis1.Add(new PivotDynamicMatrixTerm()
                                              {

                                                  belongsFrom = firstJoint,
                                                  belongsTo = pivot1,
                                                  type = DynamicType.radialVelocity,
                                                  dir = Direction.X,
                                                  defaultValue = 0.0
                                              });
                    }
                    else
                        coriolis1.Add(new PivotDynamicMatrixTerm()
                                          {
                                              belongsFrom = firstJoint,
                                              belongsTo = pivot1,
                                              type = DynamicType.radialVelocity,
                                              dir = Direction.Y,
                                              defaultValue = 0.0
                                          });
                }
                //    coriolis1[row].defaultValue = double.NaN;
                //else coriolis1[row].defaultValue = 0.0;
                //row++;

                if ((firstJoint.Contains("pis")) || (pivot1.Contains("pis")) ||
                    (firstJoint.Contains("slideronalink")) ||
                    (pivot1.Contains("slideronalink")))
                {
                    if (thisLink.Contains("pis_conn"))
                        unknownsList.Add(new PivotDynamicMatrixTerm()
                                             {
                                                 belongsFrom = firstJoint,
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
                    addToUnknowns(pivot1, unknownsList);
            }


            // cast others into arrays

            unknowns = unknownsList.ToArray();
            coriolis_1 = coriolis1.ToArray();
            omeg_1 = Omeg.ToArray();

            #endregion

            slipvelocity = new double[p, numSteps, 2];
            slipacceleration = new double[p, numSteps, 2];
        }
        private void addToUnknowns(joint pivot, List<DynamicMatrixTerm> unknownsList)
        {
            if (pivot.isGround) return;

            //got to check if pivot is already there in the dynamic matrix
            var foundIndex = unknownsList.FindIndex(dmt =>
                                                    ((dmt is PivotDynamicMatrixTerm) &&
                                                     (dmt.type == DynamicType.absoluteAcceleration) &&
                                                     ((PivotDynamicMatrixTerm)dmt).belongsTo == pivot));
            // when FindIndex returns a negative one, this means it was not found
            if (foundIndex >= 0) return;

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
                    foreach (var otherPivot in aa.Pivots.Where(b => b != aa))
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

            #endregion

        #region Function: Find Linear Velocities

        //this code works for normal, sliderv,slider h, pis
        //also for those pivots connected to sliders, we need to add velocities - but this point is taken into account. So that's good
        private void findLinearVelocities(double[,] PivotParameters)
        {
            double ICx = 0.0, ICy = 0.0, omega = 0.0;
            for (var i = 0; i != p; i++)
            {
                if (joints[i].isGround)
                {
                    PivotParameters[i, 2] = 0.0;
                    PivotParameters[i, 3] = 0.0;
                }
                else if (!(joints[i].localLabels.Contains("slider"))) //check for PIS and slider on a link
                {
                    link attachedLink = null;
                    /* find the first link it's connected to, and call that attachedLink */
                    foreach (var a in joints[i].Links)
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

                else if (joints[i].PivotType == PivotTypes.PX || joints[i].PivotType == PivotTypes.PY)
                {
                    //var nsample = new node();
                    //var nsample2 = new node();
                    //var nsample3 = new node();
                    //nsample = null;
                    //nsample2 = null;
                    //nsample3 = null;
                    foreach (var nsample in joints[i].Links)
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

                if (joints[i].isGround)
                {
                    PivotParameters[i, 2] = 0.0;
                    PivotParameters[i, 3] = 0.0;
                }
                else if (joints[i].jointType != JointTypes.P)
                {
                    PivotParameters[i, 2] = -omega * (-ICy + joints[i].Y); //changed - to +
                    PivotParameters[i, 3] = omega * (-ICx + joints[i].X);
                }
                else
                {


                    {
                        if (joints[i].localLabels.Contains("sliderh"))
                        {
                            PivotParameters[i, 2] = -omega * (-ICy + joints[i].Y);
                            PivotParameters[i, 3] = 0;
                        }
                        if (joints[i].localLabels.Contains("sliderv"))
                        {
                            PivotParameters[i, 2] = 0;
                            PivotParameters[i, 3] = omega * (-ICx + joints[i].X);
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
            //code below is to determine the individual-link angular velocities
            //the relative angular velocities are not determined
            //the angular velocities are determined with respect to the input which has one of the links as ground
            //so the first step is to determine which of rows the ground label

            //linkParameters[inputSpeedIndex, timeRow, 0] = iOmega; //storing the value of input velocity

            int newInc;
            //why not just compute angular velocity

            for (newInc = 0; newInc < inputJointIndex; newInc++)
            {
                if (inputJointIndex != newInc)
                {
                    circleDiagram[newInc].speed = inputSpeed * findDistance(circleDiagram, inputJointIndex, newInc)
                        /*findDirectionSign(circleDiagram, inputSpeedIndex, newInc, groundLink)*/;
                }
            }


            for (var index = 0; index < circleDiagInputIndex; index++)
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


            for (var nInc = 0; nInc < inputJointIndex; nInc++)
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

        private bool findVelocitiesThroughICMethod(double[,] currentPivotParams, double[,] currentLinkParams, bool Forward)
        {
            findImmediateICs(currentLinkParams);
            if (findSecondaryICs())
            {
                currentLinkParams = findAngularVelocities();
                LinkParameters.Add(currentTime, currentLinkParams);
                findLinearVelocities(currentPivotParams);
                //check slip velocities
                findLinearSlipVelocities(currentPivotParams);
                //find slip velocities and update
            }
            return false;
            throw new NotImplementedException();
        }

        /// <summary>
        ///   find immediate pivots and their positions (IC's)
        /// </summary>
        /// <param name="circleDiagram"> </param>
        private void findImmediateICs(double[,] currentLink)
        {
            for (int i = 0; i <= inputJointIndex; i++)
            {
                var cdi = circleDiagram[i];
                cdi.found = true;
                cdi.location.x = cdi.immediateJoint.x;
                cdi.location.y = cdi.immediateJoint.y;
            }
            for (int i = inputJointIndex + 1; i < sizeOfCircDiag; i++)
                circleDiagram[i].found = false;
        }


        private Boolean findSecondaryICs()
        {
            /* first pass is now complete. Now, we have to go through the circleDiagram a few
             * more times to fill up the secondary and tertiary (perhaps higher) instant centers. */
            var numberOfICsToFind = circleDiagram.Count(cdi => !cdi.found);
            Boolean foundAnInstantCenter;
            do
            {
                foundAnInstantCenter = false;
                for (int i = inputJointIndex + 1; i < sizeOfCircDiag; i++)
                {
                    var cdi = circleDiagram[i];
                    /* if it's unfulfilled - meaning X and Y are not known, then attempt to solve it. */
                    if (cdi.found) continue;
                    List<circleDiagramItem> link1List = findCommonAndFulfilledRows(cdi.link1);
                    List<circleDiagramItem> link2List = findCommonAndFulfilledRows(cdi.link2);

                    /* if there aren't any elements in list1 or list2 then return. you'll need to find another IC
                     * further down the list to solve this in the next outer loop (next time you get to while). */
                    if (!link1List.Any() || !link2List.Any()) continue;

                    var connectedCDIs = new List<Tuple<circleDiagramItem, circleDiagramItem>>();
                    foreach (var leftCDI in link1List)
                        foreach (var rightCDI in link2List)
                        {
                            var otherLeftLink = (cdi.link1 == leftCDI.link1) ? leftCDI.link2 : leftCDI.link1;
                            var otherRightLink = (cdi.link2 == rightCDI.link1) ? rightCDI.link2 : rightCDI.link1;
                            if (otherLeftLink == otherRightLink)
                                connectedCDIs.Add(new Tuple<circleDiagramItem, circleDiagramItem>(leftCDI, rightCDI));
                        }
                    if (findInstantCenter(cdi, connectedCDIs))
                    {
                        foundAnInstantCenter = true;
                        numberOfICsToFind--;
                    }
                }
            } while (foundAnInstantCenter && numberOfICsToFind > 0);
            return (numberOfICsToFind == 0);
        }

        private List<circleDiagramItem> findCommonAndFulfilledRows(link thisLink)
        {
            return circleDiagram.Where(cdi => cdi.found && (cdi.link1 == thisLink || cdi.link2 == thisLink)).ToList();
        }

        private point findInstantCenter(circleDiagramItem cdi, List<Tuple<circleDiagramItem, circleDiagramItem>> connectedCDIs)
        {
            var lines = new SortedList<double, point>(new noEqualSort());
            foreach (var cCDI in connectedCDIs)
            {
                var refPoint = (double.IsInfinity(cCDI.Item1.location.x)||double.IsInfinity(cCDI.Item1.location.y))
                    ? new point(cCDI.Item2.location.x, cCDI.Item2.location.y)
                    : new point(cCDI.Item1.location.x, cCDI.Item1.location.y);
                var slope = (Constants.sameCloseZero(cCDI.Item1.location.x , cCDI.Item2.location.x) ||double.IsInfinity(cCDI.Item1.location.y)
                    ||double.IsInfinity(cCDI.Item2.location.y))
                    ? double.PositiveInfinity
                    : (cCDI.Item1.location.y - cCDI.Item2.location.y) / (cCDI.Item1.location.x - cCDI.Item2.location.x);
                lines.Add(slope, refPoint);
            }
            while (lines.Count > 2)
            {
                var removeCand = lines.FirstOrDefault(line=>(double.IsInfinity(line.Key) && double.IsInfinity(line.Value.x) && double.IsInfinity(line.Value.y)));
                if (removeCand==null)
                     removeCand = lines.FirstOrDefault(line=>(double.IsInfinity(line.Key) && (double.IsInfinity(line.Value.x) || double.IsInfinity(line.Value.y))));
                if (removeCand==null)
                     removeCand = lines.FirstOrDefault(line=>(double.IsInfinity(line.Value.x) || double.IsInfinity(line.Value.y)));
                var removeIndex = (removeCand==null)?1:lines.IndexOfValue(removeCand);
                lines.RemoveAt(removeIndex);
              
            };
          var ICpoint =  Constants.solveViaIntersectingLines(lines.Keys[0], lines.Values[0],lines.Keys[1], lines.Values[1]);
        
            // here!!! what to return true or false?
        }

        private double[] findInstantCenter(int[,] CDIRows, circleDiagramItem[] list1, circleDiagramItem[] list2,
                                            link link1, link link2)
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
            else if (link1.Contains("pis_conn") || link2.Contains("pis_conn"))
            {
                #region PIS section essentially
                //the if-else statement is to be used for PIS
                //what needs to be done? 
                //we already know that one link connection
                //the other is PIS
                //which is connected to a link that has two pivots
                //we need to get the slope of that link and use the pis X&Y

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
                #endregion
            }



            return new[] { xc, yc };
        }







        void temp()
        {
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
            if (link1List.Count == 1 || link2List.Count == 1)
            {
                if (link1List.Count < link2List.Count)
                {

                    for (int i = 0; i != link1List.Count; i++)
                    {
                        node otherlink = (link1 == link1List[i].link1) ? link1List[i].link2 : link1List[i].link1;
                        for (int j = 0; j != link2List.Count; j++)
                        {
                            if ((otherlink == link2List[j].link1) || (otherlink == link2List[j].link2))
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
                    for (int i = 0; i != link2List.Count; i++)
                    {
                        node otherlink = (link1 == link2List[i].link1) ? link2List[i].link2 : link2List[i].link1;
                        for (int j = 0; j != link1List.Count; j++)
                        {
                            if ((otherlink == link1List[j].link1) || (otherlink == link1List[j].link2))
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
                    ic = findInstantCenter(CDIRows, link1List.ToArray(), link2List.ToArray(), circleDiagram, c, link1, link2);
                    circleDiagram[m].x = ic[0];
                    circleDiagram[m].y = ic[1];
                }
            }
            else
            {

                for (int i = 0; i != link1List.Count; i++)
                {
                    node otherlink = (link1 == link1List[i].link1) ? link1List[i].link2 : link1List[i].link1;
                    for (int j = 0; j != link2List.Count; j++)
                    {
                        if ((otherlink == link2List[j].link1) || (otherlink == link2List[j].link2))
                        {
                            if (k == 2)

                                if (link1List.Count > 2 || link2List.Count > 2)
                                {
                                    double s1nr, s1dr, s2nr, s2dr;


                                    // first row - two common points on a line
                                    double x00 = link1List[(CDIRows[0, 0])].x;
                                    double y00 = link1List[(CDIRows[0, 0])].y;
                                    double x01 = link2List[(CDIRows[0, 1])].x;
                                    double y01 = link2List[(CDIRows[0, 1])].y;


                                    // second row - two common points on a line
                                    double x110 = link1List[(CDIRows[1, 0])].x;
                                    double y110 = link1List[(CDIRows[1, 0])].y;
                                    double x11 = link2List[(CDIRows[1, 1])].x;
                                    double y11 = link2List[(CDIRows[1, 1])].y;

                                    s1nr = (y01 - y00);
                                    s1dr = (x01 - x00);
                                    s2nr = (y11 - y110);
                                    s2dr = (x11 - x110);

                                    if ((s1dr) == double.MaxValue || s1dr == 0 || double.IsInfinity(s1dr))
                                    {
                                        if (double.IsInfinity(s2dr) || (s2dr) == double.MaxValue || s2dr == 0)
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


                if (k == 2)
                {
                    double[] ic = new double[2];
                    ic = findInstantCenter(CDIRows, link1List.ToArray(), link2List.ToArray(), circleDiagram, c, link1, link2);
                    circleDiagram[m].x = ic[0];
                    circleDiagram[m].y = ic[1];
                }
            }

        }
    }
}