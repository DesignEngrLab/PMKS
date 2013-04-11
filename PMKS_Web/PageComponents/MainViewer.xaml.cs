using System.Collections.Generic;
using System.Linq;
using PlanarMechanismSimulator;
using System;
using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using Silverlight_PMKS.Shapes.Static_Shapes;

namespace PMKS_Silverlight_App
{
    public partial class MainViewer : UserControl
    {
        #region Fields
        private Point ScreenStartPoint = new Point(0, 0);
        private Point startOffset;
        private double width;
        private double height;
        private Boolean Panning;
        private double penThick;
        private double jointSize;
        #endregion

        public MainViewer()
        {
            InitializeComponent();
        }

        #region Properties
        /// <summary>
        /// Gets the scale factor.
        /// </summary>
        /// <value>
        /// The scale factor.
        /// </value>
        public double ScaleFactor { get; private set; }
        /// <summary>
        /// Gets the accel factor.
        /// </summary>
        /// <value>
        /// The accel factor.
        /// </value>
        public double AccelFactor { get; private set; }
        /// <summary>
        /// Gets the velocity factor.
        /// </summary>
        /// <value>
        /// The velocity factor.
        /// </value>
        public double VelocityFactor { get; private set; }
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


        private double ParentHeight
        {
            get
            {
                return (((FrameworkElement)Parent).ActualHeight == 0)
                           ? Application.Current.Host.Content.ActualHeight
                           : ActualHeight;
            }
        }

        private double ParentWidth
        {
            get
            {
                return (((FrameworkElement)Parent).ActualWidth == 0)
                           ? Application.Current.Host.Content.ActualWidth
                           : ActualWidth;
            }
        }
        #endregion

        internal void Clear()
        {
            foreach (var child in MainCanvas.Children)
            {
                if (!(child is DisplayVectorBaseShape) && !(child is PositionPath) && !(child is JointBaseShape))
                    continue;
                if (child is DisplayVectorBaseShape)
                    ((DisplayVectorBaseShape)child).ClearBindings();
                else if (child is JointBaseShape)
                    ((JointBaseShape)child).ClearBindings();
                else child.ClearValue(OpacityProperty);
            }
            MainCanvas.Children.Clear();
        }

        internal void UpdateVisuals(Simulator pmks, ObservableCollection<JointData> jointData, Slider timeSlider)
        {
            #region draw position, velocity, and acceleration curves
            timeSlider.Maximum = pmks.JointParameters.Times.Last();
            timeSlider.Minimum = pmks.JointParameters.Times[0];
            var h = (timeSlider.Height == 0) ? ParentHeight : timeSlider.Height;
            timeSlider.LargeChange = (timeSlider.Maximum - timeSlider.Minimum) * DisplayConstants.TickDistance / h;
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
                    default: displayJoint = new RJointShape(pmks.AllJoints[j], timeSlider, pmks, jointSize, penThick, XOffset, YOffset);
                        break;
                }
                MainCanvas.Children.Add(displayJoint);
                MainCanvas.Children.Add(new AccelerationVector(pmks.AllJoints[j], timeSlider, pmks, AccelFactor, penThick, XOffset, YOffset, displayJoint, jointData[i]));
                MainCanvas.Children.Add(new VelocityVector(pmks.AllJoints[j], timeSlider, pmks, VelocityFactor, penThick, XOffset, YOffset, displayJoint, jointData[i]));
            }
            /******************************************************************/
            #endregion
        }

        internal void UpdateRangeScaleAndCenter(Simulator pmks)
        {
            if (pmks == null) return;
            var minima = new[]
                {
                    -DisplayConstants.AxesBuffer, -DisplayConstants.AxesBuffer, double.PositiveInfinity,
                    double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity
                };
            var maxima = new[]
                {
                    DisplayConstants.AxesBuffer, DisplayConstants.AxesBuffer, double.NegativeInfinity,
                    double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity
                };
            var numJoints = pmks.JointParameters.Parameters[0].GetLength(0);

            for (int j = 0; j < pmks.JointParameters.Count; j++)
                for (int i = 0; i < numJoints; i++)
                    for (int k = 0; k < 6; k++)
                    {
                        if (minima[k] > pmks.JointParameters.Parameters[j][i, k])
                            minima[k] = pmks.JointParameters.Parameters[j][i, k];
                        if (maxima[k] < pmks.JointParameters.Parameters[j][i, k])
                            maxima[k] = pmks.JointParameters.Parameters[j][i, k];

                    }
            var minX = minima[0];
            var minY = minima[1];
            width = maxima[0] - minX;
            height = maxima[1] - minY;
            var vxMax = Math.Max(Math.Abs(minima[2]), Math.Abs(maxima[2]));
            var vyMax = Math.Max(Math.Abs(minima[3]), Math.Abs(maxima[3]));
            VelocityFactor = Math.Min(width / vxMax, height / vyMax);

            var axMax = Math.Max(Math.Abs(minima[4]), Math.Abs(maxima[4]));
            var ayMax = Math.Max(Math.Abs(minima[5]), Math.Abs(maxima[5]));
            AccelFactor = Math.Min(width / axMax, height / ayMax);

            XOffset = width - minX;
            YOffset = height - minY;
            MainCanvas.Width = 3 * width;
            MainCanvas.Height = 3 * height;

            ScaleFactor = Math.Min(ParentWidth / (3 * width), ParentHeight / (3 * height));
            if (ScaleFactor > 100) ScaleFactor = 100;
            if (ScaleFactor < 0.01) ScaleFactor = 0.01;
            startOffset = new Point((ParentWidth - (3 * ScaleFactor * width)) / 2,
                                    (ParentHeight - (3 * ScaleFactor * height)) / 2);
            ScreenStartPoint = new Point(0.0, ParentHeight);
            penThick = DisplayConstants.PenThicknessRatio / ScaleFactor;
            jointSize = DisplayConstants.JointSize / ScaleFactor;
            transform();
        }

        public void DrawStaticShapes(List<List<string>> linkIDs, List<string> jointTypes, List<double[]> initPositions)
        {
            MainCanvas.Children.Remove(MainCanvas.Children.FirstOrDefault(a => (a is Axes)));
            MainCanvas.Children.Add(new Axes(DisplayConstants.PenThicknessRatio / ScaleFactor, XOffset, YOffset));


        }

        internal void UpdateRangeScaleAndCenter(List<double[]> initPositions)
        {
            var minX = -DisplayConstants.AxesBuffer;
            var minY = -DisplayConstants.AxesBuffer;
            var maxX = DisplayConstants.AxesBuffer;
            var maxY = DisplayConstants.AxesBuffer;
            foreach (var initPosition in initPositions)
            {
                if (initPosition[0] < minX) minX = initPosition[0];
                else if (initPosition[0] > maxX) maxX = initPosition[0];
                if (initPosition[1] < minY) minY = initPosition[1];
                else if (initPosition[1] > maxY) maxY = initPosition[2];
            }
            if (initPositions.Count == 0)
            {
                maxX = maxY = 5 * DisplayConstants.AxesBuffer;
            }
            width = maxX - minX;
            height = maxY - minY;
            XOffset = width - minX;
            YOffset = height - minY;
            MainCanvas.Width = 3 * width;
            MainCanvas.Height = 3 * height;
            ScaleFactor = Math.Min(ParentWidth / (3 * width),
                                   ParentHeight / (3 * height));
            if (ScaleFactor > 100) ScaleFactor = 100;
            if (ScaleFactor < 0.01) ScaleFactor = 0.01;

            startOffset = new Point((ParentWidth - (3 * ScaleFactor * width)) / 2, (ParentHeight - (3 * ScaleFactor * height)) / 2);
            ScreenStartPoint = new Point(0.0, ParentHeight);
            penThick = DisplayConstants.PenThicknessRatio / ScaleFactor;
            jointSize = DisplayConstants.JointSize / ScaleFactor;
            transform();
        }

        private void transform()
        {
            MainCanvas.RenderTransform = new MatrixTransform
            {
                Matrix = new Matrix(ScaleFactor, 0, 0, -ScaleFactor, startOffset.X, ParentHeight - startOffset.Y)
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
