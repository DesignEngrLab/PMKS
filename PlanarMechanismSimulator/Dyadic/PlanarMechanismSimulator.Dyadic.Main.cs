﻿using System;
using System.Collections.Generic;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        private int sizeofCDI;
        private circleDiagramItem[] circleDiagram;

        private void FindFullMovementDyadic()
        {
            var currentTime = 0.0;
            var currentPivotParams = new double[p, 6];
            for (int i = 0; i < p; i++)
            {
                currentPivotParams[i, 0] = joints[i].X;
                currentPivotParams[i, 1] = joints[i].Y;
            }
            JointParameters.Add(currentTime, currentPivotParams);
            var currentLinkParams = new double[n, 2];
            LinkParameters.Add(currentTime, currentLinkParams);
            bool validPosition;

            SetUpDyadicObjects();

            do /*** Stepping Forward in Time **/
            {
                #region Find Velocities for Current Position
                if (!findVelocitiesThroughICMethod(currentTime, true))
                {
                    Status +="Instant Centers could not be found at" + currentTime + ".";
                    NumericalVelocity(currentTime, true);
                }
                #endregion
                #region Find Accelerations for Current Position
                if (!findAccelerationAnalytically(currentTime, true))
                {
                    Status +="Analytical acceleration could not be found at" + currentTime + ".";
                    NumericalAcceleration(currentTime, true);
                }
                #endregion
                #region Find Next Positions
                /* this time, NumericalPosition is called first and instead of
                 * updating currentPivotParams (which is already full at this point)
                 * we update the X, Y positions of the joints, which are global to the method. */
                NumericalPosition(currentTime, FixedTimeStep);
                /* Based upon the numerical approximation, we analytically update the remaining
                 * joints. First, we analytically set the input pivot.*/
                MoveInputToNextPosition(currentTime, FixedTimeStep);
                validPosition = AnalyticallyCorrectPositionsDyadic();
                /* create new currentPivotParams based on these updated positions of the joints */
                if (validPosition)
                {
                    currentPivotParams = new double[p, 6];
                    for (int i = 0; i < p; i++)
                    {
                        currentPivotParams[i, 0] = joints[i].X;
                        currentPivotParams[i, 1] = joints[i].Y;
                    }
                    currentTime += FixedTimeStep;
                    JointParameters.Add(currentTime, currentPivotParams);
                    currentLinkParams = new double[n, 2];
                    LinkParameters.Add(currentTime, currentLinkParams);
                }
                #endregion
            } while (validPosition && lessThanFullRotation());
            if (!lessThanFullRotation()) return;

            #region Reset to t=0

            currentPivotParams = JointParameters.Values[0];
            currentTime = JointParameters.Keys[0];
            for (int i = 0; i < p; i++)
            {
                joints[i].X = currentPivotParams[i, 0];
                joints[i].Y = currentPivotParams[i, 1];
            }

            #endregion

            #region Find first backwards step positions
            NumericalPosition(currentTime, -FixedTimeStep);
            /* Based upon the numerical approximation, we analytically update the remaining
             * joints. First, we analytically set the input pivot.*/
            MoveInputToNextPosition(currentTime, -FixedTimeStep);
            validPosition = AnalyticallyCorrectPositionsDyadic();
            /* create new currentPivotParams based on these updated positions of the joints */
            if (!validPosition) return;
            currentPivotParams = new double[p, 6];
            for (int i = 0; i < p; i++)
            {
                currentPivotParams[i, 0] = joints[i].X;
                currentPivotParams[i, 1] = joints[i].Y;
            }
            currentTime -= FixedTimeStep;
            JointParameters.Add(currentTime, currentPivotParams);
            currentLinkParams = new double[n, 2];
            LinkParameters.Add(currentTime, currentLinkParams);
            #endregion

            do /*** Stepping Backward in Time **/
            {
                if (!findVelocitiesThroughICMethod(currentTime, false))
                {
                    Status +="Instant Centers could not be found at"+currentTime+".";
                    NumericalVelocity(currentTime, false);
                }
                if (!findAccelerationAnalytically(currentTime, false))
                {
                    Status +="Analytical acceleration could not be found at" + currentTime + ".";
                    NumericalAcceleration(currentTime, false);
                }
                #region Find Next Positions

                /* this time, NumericalPosition is called first and instead of
                 * updating currentPivotParams (which is already full at this point)
                 * we update the X, Y positions of the joints, which are global to the method. */
                NumericalPosition(currentTime, -FixedTimeStep);
                /* Based upon the numerical approximation, we analytically update the remaining
                 * joints. First, we analytically set the input pivot.*/
                MoveInputToNextPosition(currentTime, -FixedTimeStep);
                validPosition = AnalyticallyCorrectPositionsDyadic();
                /* create new currentPivotParams based on these updated positions of the joints */
                if (validPosition)
                {
                    currentPivotParams = new double[p, 6];
                    for (int i = 0; i < p; i++)
                    {
                        currentPivotParams[i, 0] = joints[i].X;
                        currentPivotParams[i, 1] = joints[i].Y;
                    }
                    currentTime += FixedTimeStep;
                    JointParameters.Add(currentTime, currentPivotParams);
                    currentLinkParams = new double[n, 2];
                    LinkParameters.Add(currentTime, currentLinkParams);
                }
                #endregion
            } while (validPosition && lessThanFullRotation());
        }
        private void SetUpDyadicObjects()
        {
            //var numEqs = 0;
            //foreach (var l in links) numEqs += l.Pivots.Count - 1;


            //#region fill up link spots in CircleDiagram

            //sizeofCDI = (n * (n - 1) / 2);
            //circleDiagram = new circleDiagramItem[sizeofCDI]; //size is equal to no of instant centers
            ////code below gives the link indices for each circleDiagram Item and is assigning the instant 
            ////center pairs - these may not be necessarily starting from input - but in the way the graph
            ////and the particular linkage is created
            ////also need not be in the order like 12 / 13 - it could be 31 / 21 and so on
            ////the code below creates 12/13/14;23/24;34 - say for a 4 bar mechanism
            ////creates an ordered pair of instant centers
            //int q = 0;
            //for (int ii = 0; ii != n - 1; ii++)
            //{
            //    for (int j = ii + 1; j != n; j++)
            //    {
            //        circleDiagram[q] = new circleDiagramItem(); //coding requirement
            //        if (links[j] == groundLink)
            //        {
            //            circleDiagram[q].link1 = links[j];
            //            circleDiagram[q].link2 = links[ii];
            //        }
            //        else
            //        {
            //            circleDiagram[q].link1 = links[ii];
            //            circleDiagram[q].link2 = links[j];
            //        }
            //        q++;
            //    }
            //}

            //#endregion

            //#region Forming the acceleration node matrix

            //// change to lists
            //var coriolis1 = new List<DynamicMatrixTerm>();
            //var Omeg = new List<DynamicMatrixTerm>();

            //// DynamicMatrixTerm[] coriolis1 = new DynamicMatrixTerm[2 * numEqs];//shouldn't this be 2*no of equations?
            //// DynamicMatrixTerm[] Omeg = new DynamicMatrixTerm[2 * numEqs];
            //var unknownsList = new List<DynamicMatrixTerm>();

            ////List<int> locationofPivot0 = new List<int>();
            ////List<int> locationofPivot1 = new List<int>();
            ////int row = 0;
            ////int row2 = 0;
            //for (int i = 0; i < n; i++)
            //{
            //    var link = links[i];
            //    int NOP = link.joints.Count;
            //    var pivot0 = link.joints[0];

            //    //we are not adding slider to acceleration determination - this will be copied from the velocity of slider_conn pivot
            //    if (pivot0.jointType != jointType.P && pivot0 != jointType.RP)
            //        addToUnknowns(pivot0, unknownsList);

            //    if (!link.isGround && !link.Contains("slider_conn"))
            //    {
            //        unknownsList.Add(new LinkDynamicMatrixTerm()
            //                             {
            //                                 belongsTo = link,
            //                                 type = DynamicType.angularAcceleration
            //                             });
            //        //unknownsList[row2].belongsTo = link;
            //        //unknownsList[row2].type = DynamicType.angularAcceleration;
            //        //if (link.localLabels.Contains("ground")) unknownsList[row2].defaultValue = 0.0;
            //        //else unknownsList[row2].defaultValue = double.NaN;
            //        //row2++;
            //    }
            //    for (int j = 1; j < NOP; j++)
            //    {
            //        joint pivot1 = link.joints[j];

            //        if (!link.isGround && !link.Contains("slider_conn"))
            //        {
            //            Omeg.Add(new LinkDynamicMatrixTerm()
            //                         {
            //                             belongsTo = link,
            //                             type = DynamicType.angularVelocity,
            //                             dir = Direction.X
            //                         });

            //            //if (link.localLabels.Contains("ground")) Omeg[row].defaultValue = 0.0;
            //            //else Omeg[row].defaultValue = double.NaN;

            //            if ((pivot0.Contains("pis")) || (pivot1.localLabels.Contains("pis")) ||
            //                (pivot0.Contains("slideronalink")) ||
            //                (pivot1.Contains("slideronalink")))
            //            {
            //                if (link.Contains("pis_conn"))

            //                    coriolis1.Add(new PivotDynamicMatrixTerm()
            //                                      {

            //                                          belongsFrom = pivot0,
            //                                          belongsTo = pivot1,
            //                                          type = DynamicType.radialVelocity,
            //                                          dir = Direction.X
            //                                      });
            //                else
            //                    coriolis1.Add(new PivotDynamicMatrixTerm()
            //                                      {

            //                                          belongsFrom = pivot0,
            //                                          belongsTo = pivot1,
            //                                          type = DynamicType.radialVelocity,
            //                                          dir = Direction.X,
            //                                          defaultValue = 0.0
            //                                      });
            //            }
            //            else
            //                coriolis1.Add(new PivotDynamicMatrixTerm()
            //                                  {

            //                                      belongsFrom = pivot0,
            //                                      belongsTo = pivot1,
            //                                      type = DynamicType.radialVelocity,
            //                                      dir = Direction.X,
            //                                      defaultValue = 0.0
            //                                  });
            //        }

            //        //if ((pivot0.localLabels.Contains("pis")) || (pivot1.localLabels.Contains("pis")) ||
            //        //    (pivot0.localLabels.Contains("slideronalink")) || (pivot1.localLabels.Contains("slideronalink")))
            //        //    coriolis1[row].defaultValue = double.NaN;
            //        //else coriolis1[row].defaultValue = 0.0;

            //        if (!link.isGround && !link.Contains("slider_conn"))
            //        {
            //            Omeg.Add(new LinkDynamicMatrixTerm()
            //                         {
            //                             belongsTo = link,
            //                             type = DynamicType.angularVelocity,
            //                             dir = Direction.Y
            //                         });


            //            //if (link.localLabels.Contains("ground")) Omeg[row].defaultValue = 0.0;
            //            //else Omeg[row].defaultValue = double.NaN;

            //            if ((pivot0.Contains("pis")) || (pivot1.Contains("pis")) ||
            //                (pivot0.Contains("slideronalink")) ||
            //                (pivot1.Contains("slideronalink")))
            //            {
            //                if (link.Contains("pis_conn"))
            //                {
            //                    coriolis1.Add(new PivotDynamicMatrixTerm()
            //                                      {
            //                                          belongsFrom = pivot0,
            //                                          belongsTo = pivot1,
            //                                          type = DynamicType.radialVelocity,
            //                                          dir = Direction.Y
            //                                      });
            //                }
            //                else
            //                    coriolis1.Add(new PivotDynamicMatrixTerm()
            //                                      {

            //                                          belongsFrom = pivot0,
            //                                          belongsTo = pivot1,
            //                                          type = DynamicType.radialVelocity,
            //                                          dir = Direction.X,
            //                                          defaultValue = 0.0
            //                                      });
            //            }
            //            else
            //                coriolis1.Add(new PivotDynamicMatrixTerm()
            //                                  {
            //                                      belongsFrom = pivot0,
            //                                      belongsTo = pivot1,
            //                                      type = DynamicType.radialVelocity,
            //                                      dir = Direction.Y,
            //                                      defaultValue = 0.0
            //                                  });
            //        }
            //        //    coriolis1[row].defaultValue = double.NaN;
            //        //else coriolis1[row].defaultValue = 0.0;
            //        //row++;

            //        if ((pivot0.Contains("pis")) || (pivot1.Contains("pis")) ||
            //            (pivot0.Contains("slideronalink")) ||
            //            (pivot1.Contains("slideronalink")))
            //        {
            //            if (link.Contains("pis_conn"))
            //                unknownsList.Add(new PivotDynamicMatrixTerm()
            //                                     {
            //                                         belongsFrom = pivot0,
            //                                         belongsTo = pivot1,
            //                                         type = DynamicType.radialAcceleration
            //                                     });
            //        }


            //        //if ((pivot0.localLabels.Contains("pis")) || (pivot1.localLabels.Contains("pis")) ||
            //        //    (pivot0.localLabels.Contains("slideronalink")) || (pivot1.localLabels.Contains("slideronalink")))
            //        //    unknownsList[row2].defaultValue = double.NaN;
            //        //else unknownsList[row2].defaultValue = 0.0;
            //        //row2++;
            //        if (!pivot1.localLabels.Contains("sliderh") && !pivot1.localLabels.Contains("sliderv"))
            //            addToUnknowns(pivot1, unknownsList);
            //    }
            //}

            //// cast others into arrays

            //unknowns = unknownsList.ToArray();
            //coriolis_1 = coriolis1.ToArray();
            //omeg_1 = Omeg.ToArray();

            //#endregion

            //slipvelocity = new double[p, numSteps, 2];
            //slipacceleration = new double[p, numSteps, 2];
        }
    }
}