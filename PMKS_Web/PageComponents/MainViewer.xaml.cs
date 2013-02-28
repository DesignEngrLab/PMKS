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
using Binding_Classes;

namespace PMKS_Silverlight_App
{
    public partial class MainViewer : UserControl
    {
        private double ScaleFactor, minX, minY;
        private bool Panning;
        private Point ScreenStartPoint = new Point(0, 0);
        private Point startOffset;

        public MainViewer()
        {
            InitializeComponent();
        }

        #region Properties


        #endregion



        internal void UpdateVisuals(Simulator pmks, ObservableCollection<JointData> jointData, TimeSlider timeSlider)
        {
            if (pmks.LinkParameters == null || pmks.JointParameters == null) return;
            MainCanvas.Children.Clear();
            if (pmks.JointParameters.Count < 2) return;
            double velocityFactor, accelFactor, width, height;

            defineDisplayConstants(pmks.JointParameters, out  width, out height, out velocityFactor, out accelFactor);
            MainCanvas.Width = 3 * width;
            MainCanvas.Height = 3 * height;
            ScaleFactor = Math.Min(((FrameworkElement)Parent).ActualWidth / (3 * width),
                                      ((FrameworkElement)Parent).ActualHeight / (3 * height));
            if (ScaleFactor > 100) ScaleFactor = 100;
            if (ScaleFactor < 0.01) ScaleFactor = 0.01;

            double penThick = DisplayConstants.PenThicknessRatio / ScaleFactor;

            for (int i = 0; i < pmks.inputJointIndex; i++)
            {
                /* make a shape (i.e. Path shape) for each joint for each of the 3:position, velocity, and acceleration */
                MainCanvas.Children.Add(new AccelerationPath(i, pmks.JointParameters, jointData[i], accelFactor, width - minX, height - minY) { StrokeThickness = penThick });
                MainCanvas.Children.Add(new VelocityPath(i, pmks.JointParameters, jointData[i], velocityFactor, width - minX, height - minY) { StrokeThickness = penThick });
                MainCanvas.Children.Add(new PositionPath(i, pmks.JointParameters, jointData[i], width - minX, height - minY) { StrokeThickness = penThick });
            }
            /************experimenting with joint shape classes ***************/
            MainCanvas.Children.Add(new JointBaseShape(pmks.AllJoints[0],timeSlider,pmks,0.5,0.5,minX, minY));
            /******************************************************************/
            for(int i = 0; i < jointData.Count; i++) {
                double x;
                double y;

                x = jointData[i].getXPos();
                y = jointData[i].getYPos();

                Ellipse ellipse = new Ellipse();
                ellipse.Height = .5;
                ellipse.Width = .5;
                SolidColorBrush b = new SolidColorBrush();
                SolidColorBrush blueBrush = new SolidColorBrush();
                blueBrush.Color = Colors.Blue;
                ellipse.Fill = blueBrush;
                ellipse.StrokeThickness = .5;
                ellipse.Stroke = blueBrush;

                //ellipse.MouseEnter += new MouseEventHandler(ellipseHover);
                MainCanvas.Children.Add(ellipse);
                Canvas.SetLeft(ellipse, width - minX + x - .25);
                Canvas.SetTop(ellipse, height - minY + y - .25);

            }
            startOffset = new Point((((FrameworkElement)Parent).ActualWidth - (3 * ScaleFactor * width)) / 2,
           (((FrameworkElement)Parent).ActualHeight - (3 * ScaleFactor * height)) / 2);
            ScreenStartPoint = new Point(0.0, ((FrameworkElement)Parent).ActualHeight);
            transform();
        }

        private void defineDisplayConstants(TimeSortedList JointParameters, out double width, out double height,
            out double velocityFactor, out double accelFactor)
        {
            var minima = new double[6];
            var maxima = new double[6];
            var numJoints = JointParameters.Parameters[0].GetLength(0);
            for (int k = 0; k < minima.GetLength(0); k++)
            {
                minima[k] = double.PositiveInfinity;
                maxima[k] = double.NegativeInfinity;
            }
            //maxima[2] = maxima[3] = Constants.epsilonSame;
            for (int j = 0; j < JointParameters.Count; j++)
                for (int i = 0; i < numJoints; i++)
                    for (int k = 0; k < 6; k++)
                    {
                        if (minima[k] > JointParameters.Parameters[j][i, k])
                            minima[k] = JointParameters.Parameters[j][i, k];
                        if (maxima[k] < JointParameters.Parameters[j][i, k])
                            maxima[k] = JointParameters.Parameters[j][i, k];

                    }
            minX = minima[0];
            minY = minima[1];
            width = maxima[0] - minX;
            height = maxima[1] - minY;
            var vxMax = Math.Max(Math.Abs(minima[2]), Math.Abs(maxima[2]));
            var vyMax = Math.Max(Math.Abs(minima[3]), Math.Abs(maxima[3]));
            velocityFactor = Math.Min(width / vxMax, height / vyMax);

            var axMax = Math.Max(Math.Abs(minima[4]), Math.Abs(maxima[4]));
            var ayMax = Math.Max(Math.Abs(minima[5]), Math.Abs(maxima[5]));
            accelFactor = Math.Min(width / axMax, height / ayMax);
        }

        private void transform()
        {
            MainCanvas.RenderTransform = new MatrixTransform
            {
                Matrix = new Matrix(ScaleFactor, 0, 0, -ScaleFactor, startOffset.X,
                    ((FrameworkElement)Parent).ActualHeight - startOffset.Y)
            };

        }

        #region Events

        internal void OnMouseWheel(object sender, MouseWheelEventArgs e)
        {
            /* commented out for now, not working properly, but the zooming mechanism should take place in this handler*/
            if (e.Delta > 1)
                ScaleFactor *= 1.05;
            else ScaleFactor /= 1.05;
            transform();
        }

        internal void OnLostMouseCapture(object sender, MouseEventArgs e)
        {
            Panning = false;
        }

        internal void OnMouseEnter(object sender, MouseEventArgs e)
        {
            if (Panning) return;
            Panning = true;
            // Save starting point, used later when determining how much to scroll.
            ScreenStartPoint = e.GetPosition((UIElement) Parent);
        }

        internal void OnMouseMove(object sender, MouseEventArgs e)
        {
            if (!Panning) return;
            startOffset = new Point(startOffset.X + (e.GetPosition((UIElement)Parent).X - ScreenStartPoint.X) / ScaleFactor,
                                    startOffset.Y - (e.GetPosition((UIElement)Parent).Y - ScreenStartPoint.Y) / ScaleFactor);

            transform();
        }
        #endregion


    }
}
