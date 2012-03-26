using System;
using System.Collections.Generic;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class PlanarMechanismSimulator : IDependentAnalysis
    {
        private void SetUpDyadicObjects()
        {
            var numEqs = 0;
            foreach (var l in links) numEqs += l.Pivots.Count - 1;


            #region fill up link spots in CircleDiagram

            sizeofCDI = (n * (n - 1) / 2);
            circleDiagram = new circleDiagramItem[sizeofCDI]; //size is equal to no of instant centers
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
                    circleDiagram[q] = new circleDiagramItem(); //coding requirement
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

            #region Forming the acceleration node matrix

            // change to lists
            var coriolis1 = new List<DynamicMatrixTerm>();
            var Omeg = new List<DynamicMatrixTerm>();

            // DynamicMatrixTerm[] coriolis1 = new DynamicMatrixTerm[2 * numEqs];//shouldn't this be 2*no of equations?
            // DynamicMatrixTerm[] Omeg = new DynamicMatrixTerm[2 * numEqs];
            var unknownsList = new List<DynamicMatrixTerm>();

            //List<int> locationofPivot0 = new List<int>();
            //List<int> locationofPivot1 = new List<int>();
            //int row = 0;
            //int row2 = 0;
            for (int i = 0; i < n; i++)
            {
                var link = links[i];
                int NOP = link.Pivots.Count;
                var pivot0 = link.Pivots[0];

                //we are not adding slider to acceleration determination - this will be copied from the velocity of slider_conn pivot
                if (pivot0.PivotType != PivotTypes.PX && pivot0 != PivotTypes.PY)
                    addToUnknowns(pivot0, unknownsList);

                if (!link.IsGround && !link.Contains("slider_conn"))
                {
                    unknownsList.Add(new LinkDynamicMatrixTerm()
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
                    pivot pivot1 = link.Pivots[j];

                    if (!link.IsGround && !link.Contains("slider_conn"))
                    {
                        Omeg.Add(new LinkDynamicMatrixTerm()
                                     {
                                         belongsTo = link,
                                         type = DynamicType.angularVelocity,
                                         dir = Direction.X
                                     });

                        //if (link.localLabels.Contains("ground")) Omeg[row].defaultValue = 0.0;
                        //else Omeg[row].defaultValue = double.NaN;

                        if ((pivot0.Contains("pis")) || (pivot1.localLabels.Contains("pis")) ||
                            (pivot0.Contains("slideronalink")) ||
                            (pivot1.Contains("slideronalink")))
                        {
                            if (link.Contains("pis_conn"))

                                coriolis1.Add(new PivotDynamicMatrixTerm()
                                                  {

                                                      belongsFrom = pivot0,
                                                      belongsTo = pivot1,
                                                      type = DynamicType.radialVelocity,
                                                      dir = Direction.X
                                                  });
                            else
                                coriolis1.Add(new PivotDynamicMatrixTerm()
                                                  {

                                                      belongsFrom = pivot0,
                                                      belongsTo = pivot1,
                                                      type = DynamicType.radialVelocity,
                                                      dir = Direction.X,
                                                      defaultValue = 0.0
                                                  });
                        }
                        else
                            coriolis1.Add(new PivotDynamicMatrixTerm()
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

                    if (!link.IsGround && !link.Contains("slider_conn"))
                    {
                        Omeg.Add(new LinkDynamicMatrixTerm()
                                     {
                                         belongsTo = link,
                                         type = DynamicType.angularVelocity,
                                         dir = Direction.Y
                                     });


                        //if (link.localLabels.Contains("ground")) Omeg[row].defaultValue = 0.0;
                        //else Omeg[row].defaultValue = double.NaN;

                        if ((pivot0.Contains("pis")) || (pivot1.Contains("pis")) ||
                            (pivot0.Contains("slideronalink")) ||
                            (pivot1.Contains("slideronalink")))
                        {
                            if (link.Contains("pis_conn"))
                            {
                                coriolis1.Add(new PivotDynamicMatrixTerm()
                                                  {
                                                      belongsFrom = pivot0,
                                                      belongsTo = pivot1,
                                                      type = DynamicType.radialVelocity,
                                                      dir = Direction.Y
                                                  });
                            }
                            else
                                coriolis1.Add(new PivotDynamicMatrixTerm()
                                                  {

                                                      belongsFrom = pivot0,
                                                      belongsTo = pivot1,
                                                      type = DynamicType.radialVelocity,
                                                      dir = Direction.X,
                                                      defaultValue = 0.0
                                                  });
                        }
                        else
                            coriolis1.Add(new PivotDynamicMatrixTerm()
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

                    if ((pivot0.Contains("pis")) || (pivot1.Contains("pis")) ||
                        (pivot0.Contains("slideronalink")) ||
                        (pivot1.Contains("slideronalink")))
                    {
                        if (link.Contains("pis_conn"))
                            unknownsList.Add(new PivotDynamicMatrixTerm()
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
                        addToUnknowns(pivot1, unknownsList);
                }
            }

            // cast others into arrays

            unknowns = unknownsList.ToArray();
            coriolis_1 = coriolis1.ToArray();
            omeg_1 = Omeg.ToArray();

            #endregion

            slipvelocity = new double[p, numSteps, 2];
            slipacceleration = new double[p, numSteps, 2];
        }
    }
}
