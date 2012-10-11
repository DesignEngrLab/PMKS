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
using System.Collections.ObjectModel;
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public partial class MainViewer : UserControl
    {
        public MainViewer()
        {
            InitializeComponent();
        }
        public void UpdateVisuals(TimeSortedList JointParameters, TimeSortedList LinkParameters, int inputJointIndex, List<joint> joints, ObservableCollection<JointData> JointData)
        {
            if (LinkParameters == null || JointParameters == null) return;

            MainCanvas.Children.Clear();
            double penThick, velocityFactor, accelFactor;
            defineDisplayConstants(JointParameters, out penThick, out velocityFactor, out accelFactor);
            
            for (int i = 0; i < inputJointIndex; i++)
            {
                /* make a shape (i.e. Path shape) for each joint for each of the 3:position, velocity, and acceleration */
                MainCanvas.Children.Add(new AccelerationPath(i, JointParameters, JointData[i], accelFactor) { StrokeThickness = penThick });
                MainCanvas.Children.Add(new VelocityPath(i, JointParameters, JointData[i], velocityFactor) { StrokeThickness = penThick });
                MainCanvas.Children.Add(new PositionPath(i, JointParameters, JointData[i]) { StrokeThickness = penThick });
            }
        }

        private void defineDisplayConstants(TimeSortedList JointParameters, out double penThick, out double velocityFactor, out double accelFactor)
        {
            var minima = new double[4];
            var maxima = new double[4];
            var numJoints = JointParameters.Parameters[0].GetLength(0);
            for (int k = 0; k < minima.GetLength(0); k++)
            {
                minima[k] = double.PositiveInfinity;
                maxima[k] = double.NegativeInfinity;
            }
            maxima[2] = maxima[3] = Constants.epsilonSame;
            for (int j = 0; j < JointParameters.Count; j++)
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


            var ScaleFactor = Math.Min((((Grid)Parent).ActualWidth - 2 * DisplayConstants.Buffer) / (maxima[0] - minima[0]),
                                       (((Grid)Parent).ActualHeight - 2 * DisplayConstants.Buffer) / (maxima[1] - minima[1]));
            var biggerDim = Math.Max(maxima[0] - minima[0], maxima[1] - minima[1]);
            penThick = DisplayConstants.PenThicknessRatio / ScaleFactor;
            velocityFactor = DisplayConstants.VelocityLengthRatio * biggerDim / maxima[2];
            accelFactor = DisplayConstants.AccelLengthRatio * biggerDim / maxima[3];
            /* this function is going to impact pan and zoom. I'm not sure it is right. */
            /* Plus there is the matter of the cropping, which causes problems if the shapes have negative parts. */
            MainCanvas.RenderTransform = new MatrixTransform
            {
                Matrix =
                    new Matrix(ScaleFactor, 0, 0, -ScaleFactor, DisplayConstants.Buffer - ScaleFactor * minima[0],
                               ScaleFactor * maxima[1] + DisplayConstants.Buffer)
            };
            /* this is just to debug the problem with the positioning and cropping. */
            MainCanvas.Background = new SolidColorBrush(Colors.LightGray);
            MainCanvas.Margin = new Thickness(DisplayConstants.Buffer);
        }
    }
}
