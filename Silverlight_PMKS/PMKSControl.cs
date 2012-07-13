using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public static class PMKSControl
    {
        public static JointsViewModel JointsInfo { get; set; }

        public static bool IsAngleInsteadOfError { get; set; }

        private static Simulator pmks;
        private static readonly List<List<string>> LinkIDs = new List<List<string>>();
        private static readonly List<string> JointTypes = new List<string>();
        private static readonly List<double[]> InitPositions = new List<double[]>();
        private static int numJoints;

        internal static void ParseData()
        {
            if (JointsInfo == null) return;
            numJoints = TrimEmptyJoints();
            if (SameTopology() && SameParameters()) return;

            if (SameTopology())
            {
                DefinePositions();
                pmks.AssignPositions(InitPositions);
                if (pmks.DegreesOfFreedom == 1) pmks.FindFullMovement();
                UpdateVisuals();
            }
            else
            {
                if (!(DefineLinkIDS() && DefinePositions() && DefineJointTypeList())) return;

                try
                {
                    RunSimulation();
                }
                catch (Exception e)
                {
                    status(e.Message);
                }
            }

        }

        public static void RunSimulation()
        {
            if (pmks == null) pmks = new Simulator(LinkIDs, JointTypes, InitPositions);

            if (pmks.IsDyadic) status("The mechanism is comprised of only of dyads.");
            else status("The mechanism has non-dyadic loops.");
            int dof = pmks.DegreesOfFreedom;
            status("Degrees of freedom = " + dof);
            if (dof == 1)
            {
                pmks.DeltaAngle = AngleIncrement;
                pmks.InputSpeed = Speed;
                pmks.FindFullMovement();
                UpdateVisuals();
            }

        }

        private static int TrimEmptyJoints()
        {
            for (int i = 0; i < JointsInfo.Data.Count; i++)
                if (JointsInfo.Data[i].JointType == null) return i;
            return -1;
        }

        private static bool SameParameters()
        {
            if (numJoints != JointTypes.Count) return false;
            for (int i = 0; i < numJoints; i++)
            {
                if (Math.Abs(InitPositions[i][0] - Double.Parse(JointsInfo.Data[i].XPos)) > Constants.epsilon) return false;
                if (Math.Abs(InitPositions[i][1] - Double.Parse(JointsInfo.Data[i].YPos)) > Constants.epsilon) return false;
                if (!(InitPositions[i].GetLength(0) == 2 && string.IsNullOrWhiteSpace(JointsInfo.Data[i].Angle))
                    && Math.Abs(InitPositions[i][2] - Double.Parse(JointsInfo.Data[i].Angle)) > Constants.epsilon) return false;
            }
            return true;
        }

        private static bool SameTopology()
        {
            if (numJoints != JointTypes.Count) return false;
            for (int i = 0; i < numJoints; i++)
            {
                if (!JointsInfo.Data[i].JointType.StartsWith(JointTypes[i])) return false;
                var newLinkIDS = new List<string>(JointsInfo.Data[i].LinkNames.Split(new[] { ',', ' ' },
                    StringSplitOptions.RemoveEmptyEntries));
                if (LinkIDs[i].Any(linkID => !newLinkIDS.Remove(linkID)))
                    return false;
                if (newLinkIDS.Count > 0) return false;
            }
            return true;
        }

        private static void status(string p)
        {
            StatusBox.Text += "\n" + p;
        }

        private static Boolean DefineJointTypeList()
        {
            JointTypes.Clear();
            for (int i = 0; i < numJoints; i++)
            {
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].JointType)) return false;
                JointTypes.Add(JointsInfo.Data[i].JointType.Substring(0, 2).Trim(' '));
            }
            return true;
        }

        private static Boolean DefineLinkIDS()
        {
            LinkIDs.Clear();
            for (int i = 0; i < numJoints; i++)
            {
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].LinkNames)) return false;
                var linkNames = new List<string>(JointsInfo.Data[i].LinkNames.Split(new[] { ',', ' ' },
                    StringSplitOptions.RemoveEmptyEntries));
                if (linkNames.Count == 0) return false;
                LinkIDs.Add(linkNames);
            }
            return true;
        }

        private static Boolean DefinePositions()
        {
            InitPositions.Clear();
            for (int i = 0; i < numJoints; i++)
            {
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].XPos) ||
                    string.IsNullOrWhiteSpace(JointsInfo.Data[i].YPos))
                    return false;
                InitPositions.Add(string.IsNullOrWhiteSpace(JointsInfo.Data[i].Angle)
                                      ? new[] { Double.Parse(JointsInfo.Data[i].XPos), Double.Parse(JointsInfo.Data[i].YPos) }
                                      : new[]
                                          {
                                              Double.Parse(JointsInfo.Data[i].XPos),
                                              Double.Parse(JointsInfo.Data[i].YPos),
                                              Double.Parse(JointsInfo.Data[i].Angle)
                                          });
            }
            return true;
        }

        public static void UpdateVisuals()
        {
            if (pmks == null || pmks.JointParameters == null) return;
            canvas.Children.Clear();
            var minima = new double[4];
            var maxima = new double[4];

            defineJointParamLimits(minima, maxima);
            var biggerDim = Math.Max(maxima[0] - minima[0], maxima[1] - minima[1]);
            var penThick = 0.001 * biggerDim;
            var velocityFactor = 0.1 * biggerDim / maxima[2];
            var accelFactor = 0.1 * biggerDim / maxima[3];

            for (int i = 0; i < pmks.inputJointIndex; i++)
            {
                var pCollect = new PointCollection();
                for (int j = 0; j < pmks.JointParameters.Size; j++)
                {
                    var x = pmks.JointParameters.Parameters[j][i, 0];
                    var y = pmks.JointParameters.Parameters[j][i, 1];
                    pCollect.Add(new Point(x, y));
                    canvas.Children.Add(new Line
                    {
                        X1 = x,
                        Y1 = y,
                        X2 = x + velocityFactor * pmks.JointParameters.Parameters[j][i, 2],
                        Y2 = y + velocityFactor * pmks.JointParameters.Parameters[j][i, 3],
                        Stroke = new SolidColorBrush { Color = Colors.Brown },
                        StrokeThickness = penThick
                    });
                    canvas.Children.Add(new Line
                    {
                        X1 = x,
                        Y1 = y,
                        X2 = x + accelFactor * pmks.JointParameters.Parameters[j][i, 4],
                        Y2 = y + accelFactor * pmks.JointParameters.Parameters[j][i, 5],
                        Stroke = new SolidColorBrush { Color = Colors.Orange },
                        StrokeThickness = penThick
                    });
                    canvas.Children.Add(new Ellipse
                        {
                            Width = 5 * penThick,
                            Height = 5 * penThick,
                            RenderTransform = new TranslateTransform { X = x-2.5*penThick, Y = y-2.5*penThick },
                            Fill = new SolidColorBrush { Color = Colors.Green }
                        });
                }
                var start = pCollect[0];
                pCollect.RemoveAt(0);
                canvas.Children.Add(new Path
                    {
                        Data = new PathGeometry
                            {
                                Figures =
                                    new PathFigureCollection
                                        {
                                            new PathFigure
                                                {
                                                    StartPoint = start,
                                                    Segments =
                                                        new PathSegmentCollection
                                                            {new PolyQuadraticBezierSegment {Points = pCollect}},
                                                            IsClosed = false
                                                }
                                        }
                            },
                        StrokeThickness = penThick,
                        Stroke = new SolidColorBrush { Color = Colors.Green }
                    });
                var ScaleFactor = Math.Min((BigGrid.ActualWidth - 2 * buffer) / (maxima[0] - minima[0]),
                                           (BigGrid.ActualHeight - 2 * buffer) / (maxima[1] - minima[1]));
                canvas.RenderTransform = new MatrixTransform
                    {
                        Matrix =
                            new Matrix(ScaleFactor, 0, 0, -ScaleFactor, buffer - ScaleFactor * minima[0],
                                       ScaleFactor * maxima[1] + buffer)
                    };
                canvas.Margin = new Thickness(buffer);
            }
        }

        private static void defineJointParamLimits(double[] minima, double[] maxima)
        {
            for (int k = 0; k < minima.GetLength(0); k++)
            {
                minima[k] = double.PositiveInfinity;
                maxima[k] = double.NegativeInfinity;
            }
            for (int j = 0; j < pmks.JointParameters.Size; j++)
                for (int i = 0; i < pmks.numJoints; i++)
                {
                    if (minima[0] > pmks.JointParameters.Parameters[j][i, 0])
                        minima[0] = pmks.JointParameters.Parameters[j][i, 0];
                    if (maxima[0] < pmks.JointParameters.Parameters[j][i, 0])
                        maxima[0] = pmks.JointParameters.Parameters[j][i, 0];
                    if (minima[1] > pmks.JointParameters.Parameters[j][i, 1])
                        minima[1] = pmks.JointParameters.Parameters[j][i, 1];
                    if (maxima[1] < pmks.JointParameters.Parameters[j][i, 1])
                        maxima[1] = pmks.JointParameters.Parameters[j][i, 1];
                    var velSize = Constants.distanceSqared(pmks.JointParameters.Parameters[j][i, 2],
                                                     pmks.JointParameters.Parameters[j][i, 3]);
                    if (minima[2] > velSize) minima[2] = velSize;
                    if (maxima[2] < velSize) maxima[2] = velSize;
                    var accelSize = Constants.distanceSqared(pmks.JointParameters.Parameters[j][i, 4],
                                                     pmks.JointParameters.Parameters[j][i, 5]);
                    if (minima[3] > accelSize) minima[3] = accelSize;
                    if (maxima[3] < accelSize) maxima[3] = accelSize;
                }
            minima[2] = Math.Sqrt(minima[2]);
            maxima[2] = Math.Sqrt(maxima[2]);
            minima[3] = Math.Sqrt(minima[3]);
            maxima[3] = Math.Sqrt(maxima[3]);
        }



        internal static void SettingsUpdated()
        {
            if (pmks == null || pmks.JointParameters == null) return;
            //throw new NotImplementedException();
        }

        public static double Speed { get; set; }

        public static double AngleIncrement { get; set; }

        public static double Error { get; set; }

        public static bool IsMetric { get; set; }

        public static bool IsRadians { get; set; }

        public static TextBox StatusBox { get; set; }

        public static Canvas canvas { get; set; }


        public static Grid BigGrid { get; set; }

        public static double buffer = 50.0;
    }
}
