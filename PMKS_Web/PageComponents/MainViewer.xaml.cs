using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public partial class MainViewer : UserControl
    {
        public MainViewer()
        {
            InitializeComponent();
        }
        public void UpdateVisuals(TimeSortedList JointParameters, TimeSortedList LinkParameters, int inputJointIndex)
        {
            if (LinkParameters == null || JointParameters == null) return;
            MainCanvas.Children.Clear();
            var minima = new double[4];
            var maxima = new double[4];

            defineJointParamLimits(JointParameters, minima, maxima);
            var biggerDim = Math.Max(maxima[0] - minima[0], maxima[1] - minima[1]);
            var penThick = DisplayConstants.PenThicknessRatio * biggerDim;
            var velocityFactor = DisplayConstants.VelocityLengthRatio * biggerDim / maxima[2];
            var accelFactor = DisplayConstants.AccelLengthRatio * biggerDim / maxima[3];

            for (int i = 0; i < inputJointIndex; i++)
            {
                var pCollect = new PointCollection();
                for (int j = 0; j < JointParameters.Size; j++)
                {
                    var x = JointParameters.Parameters[j][i, 0];
                    var y = JointParameters.Parameters[j][i, 1];
                    pCollect.Add(new Point(x, y));
                    //new LineGeometry
                    MainCanvas.Children.Add(new Line
                    {
                        X1 = x,
                        Y1 = y,
                        X2 = x + velocityFactor * JointParameters.Parameters[j][i, 2],
                        Y2 = y + velocityFactor * JointParameters.Parameters[j][i, 3],
                        Stroke = new SolidColorBrush { Color = Colors.Brown },
                        StrokeThickness = penThick
                    });
                    MainCanvas.Children.Add(new Line
                    {
                        X1 = x,
                        Y1 = y,
                        X2 = x + accelFactor * JointParameters.Parameters[j][i, 4],
                        Y2 = y + accelFactor * JointParameters.Parameters[j][i, 5],
                        Stroke = new SolidColorBrush { Color = Colors.Orange },
                        StrokeThickness = penThick
                    });
                    MainCanvas.Children.Add(new Ellipse
                    {
                        Width = 5 * penThick,
                        Height = 5 * penThick,
                        RenderTransform = new TranslateTransform { X = x - 2.5 * penThick, Y = y - 2.5 * penThick },
                        Fill = new SolidColorBrush { Color = Colors.Green }
                    });
                }
                var start = pCollect[0];
                pCollect.RemoveAt(0);
                MainCanvas.Children.Add(new Path
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
                                                            {new PolyLineSegment() {Points = pCollect}},
                                                            IsClosed = false
                                                }
                                        }
                            },
                        StrokeThickness = penThick,
                        Stroke = new SolidColorBrush { Color = Colors.Green }
                    });
            }
            var ScaleFactor = Math.Min((((Grid)Parent).ActualWidth - 2 * DisplayConstants.Buffer) / (maxima[0] - minima[0]),
                                       (((Grid)Parent).ActualHeight - 2 * DisplayConstants.Buffer) / (maxima[1] - minima[1]));
            MainCanvas.RenderTransform = new MatrixTransform
            {
                Matrix =
                    new Matrix(ScaleFactor, 0, 0, -ScaleFactor, DisplayConstants.Buffer - ScaleFactor * minima[0],
                               ScaleFactor * maxima[1] + DisplayConstants.Buffer)
            };
            MainCanvas.Margin = new Thickness(DisplayConstants.Buffer);
        }

        private static void defineJointParamLimits(TimeSortedList JointParameters, double[] minima, double[] maxima)
        {
            var numJoints = JointParameters.Parameters[0].GetLength(0);
            for (int k = 0; k < minima.GetLength(0); k++)
            {
                minima[k] = double.PositiveInfinity;
                maxima[k] = double.NegativeInfinity;
            }
            maxima[2] = maxima[3] =Constants.epsilonSame;
            for (int j = 0; j < JointParameters.Size; j++)
                for (int i = 0; i < numJoints; i++)
                {
                    if (minima[0] > JointParameters.Parameters[j][i, 0])
                        minima[0] = JointParameters.Parameters[j][i, 0];
                    if (maxima[0] < JointParameters.Parameters[j][i, 0])
                        maxima[0] = JointParameters.Parameters[j][i, 0];
                    if (minima[1] > JointParameters.Parameters[j][i, 1])
                        minima[1] = JointParameters.Parameters[j][i, 1];
                    if (maxima[1] < JointParameters.Parameters[j][i, 1])
                        maxima[1] = JointParameters.Parameters[j][i, 1];
                    var velSize = Constants.distanceSqared(JointParameters.Parameters[j][i, 2],
                                                     JointParameters.Parameters[j][i, 3]);
                    if (minima[2] > velSize) minima[2] = velSize;
                    if (maxima[2] < velSize) maxima[2] = velSize;
                    var accelSize = Constants.distanceSqared(JointParameters.Parameters[j][i, 4],
                                                     JointParameters.Parameters[j][i, 5]);
                    if (minima[3] > accelSize) minima[3] = accelSize;
                    if (maxima[3] < accelSize) maxima[3] = accelSize;
                }
            minima[2] = Math.Sqrt(minima[2]);
            maxima[2] = Math.Sqrt(maxima[2]);
            minima[3] = Math.Sqrt(minima[3]);
            maxima[3] = Math.Sqrt(maxima[3]);
        }
    }
}
