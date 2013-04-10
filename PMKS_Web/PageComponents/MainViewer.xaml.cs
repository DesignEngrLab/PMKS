using System.Linq;
using PlanarMechanismSimulator;
using System;
using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;

namespace PMKS_Silverlight_App
{
    public partial class MainViewer : UserControl
    {
        private double ScaleFactor, minX, minY;
        private bool Panning;
        private Point ScreenStartPoint = new Point(0, 0);
        private Point startOffset;
        private double accelFactor, velocityFactor, width, height;

        public MainViewer()
        {
            InitializeComponent();
        }

        #region Properties
        /// <summary>
        /// Gets the X offset.
        /// </summary>
        /// <value>
        /// The X offset.
        /// </value>
        public double XOffset { get; private set; }
        /// <summary>
        /// Gets the Y offset.
        /// </summary>
        /// <value>
        /// The Y offset.
        /// </value>
        public double YOffset { get; private set; }

        /// <summary>
        /// Gets or sets whether or not one needs to recalcuate limits.
        /// </summary>
        /// <value>
        /// The recalcuate limits.
        /// </value>
        public Boolean RecalculateLimits = true;

        #endregion



        internal void UpdateVisuals(Simulator pmks, ObservableCollection<JointData> jointData, Slider timeSlider)
        {
            if (pmks.LinkParameters == null || pmks.JointParameters == null || pmks.JointParameters.Count < 2) return;
            UpdateRangeScaleAndCenter(pmks);
            #region remove old shapes
            for (int i = MainCanvas.Children.Count - 1; i >= 0; i--)
            {
                var child = MainCanvas.Children[i];
                if (child is AccelerationVector || child is VelocityVector  || child is PositionPath || child is JointBaseShape)
                    MainCanvas.Children.RemoveAt(i);
            }
            #endregion
            #region draw position, velocity, and acceleration curves
            double penThick = DisplayConstants.PenThicknessRatio / ScaleFactor;

            timeSlider.Maximum = pmks.JointParameters.Times.Last();
            timeSlider.Minimum = pmks.JointParameters.Times[0];
            timeSlider.LargeChange = (timeSlider.Maximum - timeSlider.Minimum) * DisplayConstants.TickDistance / timeSlider.ActualHeight;
            timeSlider.SmallChange = (timeSlider.Maximum - timeSlider.Minimum) / 1000;
            timeSlider.Value = 0.0;


            /************ creating the shapes ***************/
            for (int i = 0; i < jointData.Count; i++)
            {
                var j = pmks.JointReOrdering[i];
                if (pmks.AllJoints[j].FixedWithRespectTo(pmks.groundLink)) continue;
                MainCanvas.Children.Add(new PositionPath(j, pmks.JointParameters, jointData[i], XOffset, YOffset) { StrokeThickness = penThick });
                JointBaseShape displayJoint; 
                switch (pmks.AllJoints[j].jointType)
                {
                    default:displayJoint=new RJointShape(pmks.AllJoints[j], timeSlider, pmks, DisplayConstants.JointSize/ScaleFactor, penThick, XOffset, YOffset);
                        break;
                }
                MainCanvas.Children.Add(displayJoint);
                MainCanvas.Children.Add(new AccelerationVector(pmks.AllJoints[j], timeSlider, pmks, accelFactor, penThick, XOffset, YOffset, displayJoint, jointData[i]));
                MainCanvas.Children.Add(new VelocityVector(pmks.AllJoints[j], timeSlider, pmks, velocityFactor, penThick, XOffset, YOffset, displayJoint, jointData[i]));
            }
            /******************************************************************/
            #endregion
        }

        public void UpdateRangeScaleAndCenter(Simulator pmks)
        {
            if (pmks == null) return;
            if (RecalculateLimits)
            {
                var minima = new double[6];
                var maxima = new double[6];
                var numJoints = pmks.JointParameters.Parameters[0].GetLength(0);
                for (int k = 0; k < minima.GetLength(0); k++)
                {
                    minima[k] = double.PositiveInfinity;
                    maxima[k] = double.NegativeInfinity;
                }
                //maxima[2] = maxima[3] = Constants.epsilonSame;
                for (int j = 0; j < pmks.JointParameters.Count; j++)
                    for (int i = 0; i < numJoints; i++)
                        for (int k = 0; k < 6; k++)
                        {
                            if (minima[k] > pmks.JointParameters.Parameters[j][i, k])
                                minima[k] = pmks.JointParameters.Parameters[j][i, k];
                            if (maxima[k] < pmks.JointParameters.Parameters[j][i, k])
                                maxima[k] = pmks.JointParameters.Parameters[j][i, k];

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

                XOffset = width - minX;
                YOffset = height - minY;
                MainCanvas.Width = 3 * width;
                MainCanvas.Height = 3 * height;
                ScaleFactor = Math.Min(((FrameworkElement)Parent).ActualWidth / (3 * width),
                                       ((FrameworkElement)Parent).ActualHeight / (3 * height));
                if (ScaleFactor > 100) ScaleFactor = 100;
                if (ScaleFactor < 0.01) ScaleFactor = 0.01;
                RecalculateLimits = false;
            }
            startOffset = new Point((((FrameworkElement)Parent).ActualWidth - (3 * ScaleFactor * width)) / 2,
           (((FrameworkElement)Parent).ActualHeight - (3 * ScaleFactor * height)) / 2);
            ScreenStartPoint = new Point(0.0, ((FrameworkElement)Parent).ActualHeight);
            transform();
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
            ScreenStartPoint = e.GetPosition((UIElement)Parent);
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
