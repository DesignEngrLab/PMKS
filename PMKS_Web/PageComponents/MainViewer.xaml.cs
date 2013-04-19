using System.Collections.Generic;
using System.Linq;
using System.Windows.Controls.Primitives;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
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
        internal Point PanningAnchor;
        private double width, height, minX, maxX, minY, maxY;
        private Boolean Panning;
        private double penThick;
        private double jointSize;
        public MainPage main;
        public Storyboard storyBoard { get; private set; }
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
        public double ScaleFactor { get; set; }
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
                return Application.Current.Host.Content.ActualHeight;
                return (((FrameworkElement)Parent).ActualHeight == 0)
                           ? Application.Current.Host.Content.ActualHeight
                           : ActualHeight;
            }
        }

        private double ParentWidth
        {
            get
            {
                return Application.Current.Host.Content.ActualWidth;
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
                if (!(child is DisplayVectorBaseShape) && !(child is PositionPath) && !(child is DynamicJointBaseShape) && !(child is LinkShape))
                    continue;
                if (child is DisplayVectorBaseShape)
                    ((DisplayVectorBaseShape)child).ClearBindings();
                else if (child is DynamicJointBaseShape)
                    ((DynamicJointBaseShape)child).ClearBindings();
                else if (child is LinkShape)
                    ((LinkShape)child).ClearBindings();
                else child.ClearValue(OpacityProperty);
            }
            MainCanvas.Children.Clear();
        }

        internal void DrawDynamicShapes(Simulator pmks, ObservableCollection<JointData> jointData, Slider timeSlider)
        {
            #region draw position, velocity, and acceleration curves
            timeSlider.Maximum = pmks.JointParameters.Times.Last();
            main.maxTimeText.Text = timeSlider.Maximum.ToString("F");
            timeSlider.Minimum = pmks.JointParameters.Times[0];
            main.minTimeText.Text = timeSlider.Minimum.ToString("F");
            var h = (timeSlider.ActualHeight == 0) ? ParentHeight : timeSlider.ActualHeight;
            timeSlider.LargeChange = (timeSlider.Maximum - timeSlider.Minimum) * DisplayConstants.TickDistance / h;
            timeSlider.SmallChange = (timeSlider.Maximum - timeSlider.Minimum) / 1000;
            timeSlider.Value = 0.0;


            /************ creating the shapes ***************/
            foreach (var child in MainCanvas.Children)
                if (child is LinkShape)
                    ((LinkShape)child).SetBindings(timeSlider, pmks, XOffset, YOffset);

            for (int i = 0; i < jointData.Count; i++)
            {
                var j = pmks.JointReOrdering[i];
                if (pmks.AllJoints[j].FixedWithRespectTo(pmks.groundLink)) continue;
                MainCanvas.Children.Add(new PositionPath(j, pmks.JointParameters, jointData[i], XOffset, YOffset) { StrokeThickness = penThick });
                DynamicJointBaseShape displayJoint;
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

            var duration = new Duration(TimeSpan.FromSeconds(timeSlider.Maximum - timeSlider.Minimum));
            var timeAnimation = new DoubleAnimation
                {
                    From = timeSlider.Minimum,
                    To = timeSlider.Maximum,
                    Duration = duration,
                    AutoReverse = !pmks.CompleteCycle,
                    RepeatBehavior = RepeatBehavior.Forever
                };
            storyBoard = new Storyboard
                {
                    Duration = duration,
                    AutoReverse = !pmks.CompleteCycle,
                    RepeatBehavior = RepeatBehavior.Forever
                };
            storyBoard.Children.Add(timeAnimation);
            Storyboard.SetTarget(timeAnimation, timeSlider);
            Storyboard.SetTargetProperty(timeAnimation, new PropertyPath(RangeBase.ValueProperty));
            if ((bool)main.PlayButton.IsChecked)
                storyBoard.Begin();
        }

        public void DrawStaticShapes(List<List<string>> linkIDs, List<string> jointTypes, List<double[]> initPositions,
                                     List<string> distinctLinkNames, bool FilledIn)
        {
            MainCanvas.Children.Remove(MainCanvas.Children.FirstOrDefault(a => (a is Axes)));
            MainCanvas.Children.Add(new Axes(penThick, XOffset, YOffset));
            for (int i = 0; i < distinctLinkNames.Count; i++)
            {
                if (distinctLinkNames[i] == "ground") continue;
                MainCanvas.Children.Add(new LinkShape(i, distinctLinkNames[i], linkIDs, jointTypes, initPositions,
                                                      XOffset, YOffset, penThick, null,
                                                      DisplayConstants.DefaultLinkThickness/ScaleFactor));
            }
            for (int i = 0; i < linkIDs.Count; i++)
            {
                MainCanvas.Children.Add(new InputRJointShape(jointSize, penThick, initPositions[i][0] + XOffset,
                                                             initPositions[i][1] + YOffset,
                                                             linkIDs[i].Contains("ground"), FilledIn));
            }
            if (TargetPath != null)
            {
                MainCanvas.Children.Remove(TargetPath);
                TargetPath.RenderTransform
                    = new TranslateTransform {X = XOffset, Y = YOffset};
                MainCanvas.Children.Add(TargetPath);
            }
        }

        internal void UpdateRanges(List<double[]> initPositions)
        {
            minX = -DisplayConstants.AxesBuffer;
            minY = -DisplayConstants.AxesBuffer;
            maxX = DisplayConstants.AxesBuffer;
            maxY = DisplayConstants.AxesBuffer;
            foreach (var initPosition in initPositions)
            {
                if (initPosition[0] < minX) minX = initPosition[0];
                else if (initPosition[0] > maxX) maxX = initPosition[0];
                if (initPosition[1] < minY) minY = initPosition[1];
                else if (initPosition[1] > maxY) maxY = initPosition[1];
            }
            if (initPositions.Count == 0)
            {
                maxX = maxY = 5 * DisplayConstants.AxesBuffer;
            }
            width = maxX - minX;
            height = maxY - minY;
        }
        internal void UpdateRanges(Simulator pmks)
        {
            if (pmks == null) return;
            minX = -DisplayConstants.AxesBuffer;
            minY = -DisplayConstants.AxesBuffer;
            maxX = DisplayConstants.AxesBuffer;
            maxY = DisplayConstants.AxesBuffer;
            var numJoints = pmks.JointParameters.Parameters[0].GetLength(0);

            for (int j = 0; j < pmks.JointParameters.Count; j++)
                for (int i = 0; i < numJoints; i++)
                {
                    if (minX > pmks.JointParameters.Parameters[j][i, 0])
                        minX = pmks.JointParameters.Parameters[j][i, 0];
                    if (minY > pmks.JointParameters.Parameters[j][i, 1])
                        minY = pmks.JointParameters.Parameters[j][i, 1];
                    if (maxX < pmks.JointParameters.Parameters[j][i, 0])
                        maxX = pmks.JointParameters.Parameters[j][i, 0];
                    if (maxY < pmks.JointParameters.Parameters[j][i, 1])
                        maxY = pmks.JointParameters.Parameters[j][i, 1];
                }
            width = maxX - minX;
            height = maxY - minY;
        }

        internal void FindVelocityAndAccelerationScalers(Simulator pmks)
        {
            if (pmks == null) return;
            var maxVx = 0.0;
            var maxVy = 0.0;
            var maxAx = 0.0;
            var maxAy = 0.0;
            var numJoints = pmks.JointParameters.Parameters[0].GetLength(0);

            for (int j = 0; j < pmks.JointParameters.Count; j++)
                for (int i = 0; i < numJoints; i++)
                {
                    var vx = Math.Abs(pmks.JointParameters.Parameters[j][i, 2]);
                    var vy = Math.Abs(pmks.JointParameters.Parameters[j][i, 3]);
                    var ax = Math.Abs(pmks.JointParameters.Parameters[j][i, 4]);
                    var ay = Math.Abs(pmks.JointParameters.Parameters[j][i, 5]);
                    if (maxVx < vx) maxVx = vx;
                    if (maxVy < vy) maxVy = vy;
                    if (maxAx < ax) maxAx = ax;
                    if (maxAy < ay) maxAy = ay;
                }

            VelocityFactor = Math.Min(width / maxVx, height / maxVy);

            AccelFactor = Math.Min(width / maxAx, height / maxAy);

        }


        internal void UpdateScaleAndCenter()
        {
            XOffset = DisplayConstants.DefaultBufferMultipler * width - minX;
            YOffset = DisplayConstants.DefaultBufferMultipler * height - minY;
            // for the meantime, we decided to not change the size of the canvas. Keep fixed size.
            //XOffset = YOffset = -DisplayConstants.MostNegativeAllowableCoordinate;
            //MainCanvas.Width = DisplayConstants.TotalSize;
            //MainCanvas.Height = DisplayConstants.TotalSize;
            ///////////////////////
            MainCanvas.Width = width + 2 * DisplayConstants.DefaultBufferMultipler * width;
            MainCanvas.Height = height + 2 * DisplayConstants.DefaultBufferMultipler * height;

            var newScaleFactor = Math.Min(ParentWidth / (MainCanvas.Width), ParentHeight / (MainCanvas.Height));
            //if (Math.Abs(newScaleFactor - ScaleFactor) / Math.Max(newScaleFactor, ScaleFactor) >
            //    DisplayConstants.DeltaChangeInScaleToStaySame)
            //{
            //    ScaleFactor = newScaleFactor;
            if (newScaleFactor > 10) newScaleFactor = 10;
            if (newScaleFactor < 0.1) newScaleFactor = 0.1;
            //}
            var newPanAnchor = new Point((ParentWidth - MainCanvas.Width) / 2, (ParentHeight - MainCanvas.Height) / 2);
            //if ((Math.Abs(newPanAnchor.X - startOffset.X) + Math.Abs(newPanAnchor.Y - startOffset.Y))
            //  / Math.Max(startOffset.X, startOffset.Y) > DisplayConstants.DeltaChangeInScaleToStaySame)
            //startOffset = newPanAnchor;

            MoveScaleCanvas(newScaleFactor, newPanAnchor, true);
        }

        internal void MoveScaleCanvas(double newScaleFactor, Point newPanAnchor, Boolean animate = false)
        {
            var px = (newPanAnchor.X > 0) ? 0.0 : newPanAnchor.X;
            var py = (newPanAnchor.Y > 0) ? 0.0 : newPanAnchor.Y;

            if (animate)
            {
                //    var duration = new Duration(new TimeSpan(0, 0, 1));
                //var timeAnimation = new DoubleAnimationUsingKeyFrames{}
                //{
                //    Duration = duration,
                //    AutoReverse = false,EasingFunction = 
                //};
                //storyBoard = new Storyboard
                //{
                //    Duration = duration,
                //    AutoReverse = !pmks.CompleteCycle,
                //    RepeatBehavior = RepeatBehavior.Forever
                //};
                //storyBoard.Children.Add(timeAnimation);
                //Storyboard.SetTarget(timeAnimation, this);
                //Storyboard.SetTargetProperty(timeAnimation, new PropertyPath(UIElement.RenderTransformProperty));


                MainCanvas.RenderTransform = new MatrixTransform
                {
                    Matrix = new Matrix(ScaleFactor, 0, 0, -ScaleFactor, px, ParentHeight - py)
                };
            }
            else
                MainCanvas.RenderTransform = new MatrixTransform
                {
                    Matrix = new Matrix(ScaleFactor, 0, 0, -ScaleFactor, px, ParentHeight - py)
                };
            ScaleFactor = newScaleFactor;
            PanningAnchor = new Point(px, ParentHeight - py);
            penThick = DisplayConstants.PenThicknessRatio / ScaleFactor;
            jointSize = DisplayConstants.JointSize / ScaleFactor;
        }

        public Path TargetPath { get; set; }
    }
}
