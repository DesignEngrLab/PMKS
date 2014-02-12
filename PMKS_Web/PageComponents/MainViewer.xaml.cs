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
using Silverlight_PMKS;
using Silverlight_PMKS.Shapes.Static_Shapes;

namespace PMKS_Silverlight_App
{
    public partial class MainViewer : UserControl
    {

        #region Fields
        private double canvasWidth, canvasHeight, minX, maxX, minY, maxY;
        private double penThick;
        private double jointSize;
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
            get { return (double.IsNaN(Height)) ? Application.Current.Host.Content.ActualHeight : Height; }
        }

        private double ParentWidth
        {
            get
            {
                return (double.IsNaN(Width)) ? Application.Current.Host.Content.ActualWidth : Width;
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
            App.main.maxTimeText.Text = timeSlider.Maximum.ToString("F");
            timeSlider.Minimum = pmks.JointParameters.Times[0];
            App.main.minTimeText.Text = timeSlider.Minimum.ToString("F");
            var h = (timeSlider.ActualHeight == 0) ? ParentHeight : timeSlider.ActualHeight;
            timeSlider.LargeChange = (timeSlider.Maximum - timeSlider.Minimum) * DisplayConstants.TickDistance / h;
            timeSlider.SmallChange = (timeSlider.Maximum - timeSlider.Minimum) / 1000;
            timeSlider.Value = 0.0;


            /************ creating the shapes ***************/
            foreach (var child in MainCanvas.Children)
                if (child is LinkShape)
                    ((LinkShape)child).SetBindings(timeSlider, pmks, XOffset, YOffset);

            for (int i = 0; i < pmks.numJoints; i++)
            {
                var j = pmks.JointNewIndexFromOriginal[i];
                if (pmks.AllJoints[j].FixedWithRespectTo(pmks.groundLink)) continue;
                MainCanvas.Children.Add(new PositionPath(j, pmks.JointParameters, jointData[i], XOffset, YOffset) { StrokeThickness = penThick });
                DynamicJointBaseShape displayJoint;
                switch (pmks.AllJoints[j].jointType)
                {
                    case JointTypes.P:
                        var fixedLink = pmks.AllJoints[j].Link2;
                        displayJoint = new PJointShape(pmks.AllJoints[j], fixedLink, timeSlider, pmks, jointSize, penThick, XOffset, YOffset);
                        break;
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
                    AutoReverse = pmks.CycleType!=CycleTypes.OneCycle,
                    RepeatBehavior = RepeatBehavior.Forever
                };
            storyBoard = new Storyboard
                {
                    Duration = duration,
                    AutoReverse = pmks.CycleType != CycleTypes.OneCycle,
                    RepeatBehavior = RepeatBehavior.Forever
                };
            storyBoard.Children.Add(timeAnimation);
            Storyboard.SetTarget(timeAnimation, timeSlider);
            Storyboard.SetTargetProperty(timeAnimation, new PropertyPath(RangeBase.ValueProperty));
            if ((bool)App.main.PlayButton.IsChecked)
                storyBoard.Begin();
        }

        public void DrawStaticShapes(List<List<string>> linkIDs, List<string> jointTypes, List<double[]> initPositions,
                                     List<string> distinctLinkNames, bool FilledIn)
        {
            MainCanvas.Children.Remove(MainCanvas.Children.FirstOrDefault(a => (a is Axes)));
            MainCanvas.Children.Add(new Axes(penThick, XOffset, YOffset, MainCanvas.Width, MainCanvas.Height));
            for (int i = 0; i < distinctLinkNames.Count; i++)
            {
                if (distinctLinkNames[i] == "ground") continue;
                MainCanvas.Children.Add(new LinkShape(i, distinctLinkNames[i], linkIDs, jointTypes, initPositions,
                                                      XOffset, YOffset, penThick, null,
                                                      DisplayConstants.DefaultLinkThickness / ScaleFactor));
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
                    = new TranslateTransform { X = XOffset, Y = YOffset };
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
            canvasWidth = maxX - minX;
            canvasHeight = maxY - minY;
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
            canvasWidth = maxX - minX;
            canvasHeight = maxY - minY;
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

            VelocityFactor = Math.Min(canvasWidth / maxVx, canvasHeight / maxVy);

            AccelFactor = Math.Min(canvasWidth / maxAx, canvasHeight / maxAy);

        }


        internal void UpdateScaleAndCenter()
        {
            XOffset = DisplayConstants.DefaultBufferMultipler * canvasWidth - minX;
            YOffset = DisplayConstants.DefaultBufferMultipler * canvasHeight - minY;
            // for the meantime, we decided to not change the size of the canvas. Keep fixed size.
            //XOffset = YOffset = -DisplayConstants.MostNegativeAllowableCoordinate;
            //MainCanvas.Width = DisplayConstants.TotalSize;
            //MainCanvas.Height = DisplayConstants.TotalSize;
            ///////////////////////
            MainCanvas.Width = canvasWidth + 2 * DisplayConstants.DefaultBufferMultipler * canvasWidth;
            MainCanvas.Height = canvasHeight + 2 * DisplayConstants.DefaultBufferMultipler * canvasHeight;

            var newScaleFactor = Math.Min(ParentWidth / (MainCanvas.Width), ParentHeight / (MainCanvas.Height));
            //if (Math.Abs(newScaleFactor - ScaleFactor) / Math.Max(newScaleFactor, ScaleFactor) >
            //    DisplayConstants.DeltaChangeInScaleToStaySame)
            //{
            //    ScaleFactor = newScaleFactor;
            if (newScaleFactor > DisplayConstants.MaxZoomIn) newScaleFactor = DisplayConstants.MaxZoomIn;
            if (newScaleFactor < DisplayConstants.MaxZoomOut) newScaleFactor = DisplayConstants.MaxZoomOut;
            //}
            var newPanAnchor = new Point((ParentWidth - newScaleFactor * MainCanvas.Width) / 2,
                (ParentHeight - newScaleFactor * MainCanvas.Height) / 2);
            //if ((Math.Abs(newPanAnchor.X - startOffset.X) + Math.Abs(newPanAnchor.Y - startOffset.Y))
            //  / Math.Max(startOffset.X, startOffset.Y) > DisplayConstants.DeltaChangeInScaleToStaySame)
            //startOffset = newPanAnchor;

            MoveScaleCanvas(newScaleFactor, newPanAnchor, true);
        }

        internal void MoveScaleCanvas(double newScaleFactor, Point newPanAnchor, Boolean animate = false)
        {
            double px = newPanAnchor.X ;
            double py = newPanAnchor.Y;
            //if (ParentWidth < newScaleFactor * MainCanvas.Width)
            //{
            //    if (px > 0) px = 0;
            //    else if (px < (ParentWidth - newScaleFactor * MainCanvas.Width))
            //        px = ParentWidth - newScaleFactor * MainCanvas.Width;
            //}
            //else
            //{
            //    if (px > (ParentWidth - (newScaleFactor * MainCanvas.Width) / 2))
            //        px = ParentWidth - (newScaleFactor * MainCanvas.Width) / 2;
            //    else if (px < 0) px = 0;
            //}
            //if (ParentHeight < newScaleFactor * MainCanvas.Width)
            //{
            //    if (py > 0) py = 0;
            //    else if (py < (ParentHeight - newScaleFactor * MainCanvas.Height))
            //        py = ParentHeight - newScaleFactor * MainCanvas.Height;
            //}
            //else
            //{
            //    if (py > (ParentHeight - (newScaleFactor * MainCanvas.Height) / 2))
            //        py = ParentHeight - (newScaleFactor * MainCanvas.Height) / 2;
            //    else if (py < 0) py = 0;
            //}
            ScaleFactor = newScaleFactor;
            penThick = DisplayConstants.PenThicknessRatio / ScaleFactor;
            jointSize = DisplayConstants.JointSize / ScaleFactor;
            MainCanvas.RenderTransform = new CompositeTransform
            {
                ScaleX = ScaleFactor,
                ScaleY = ScaleFactor,
                TranslateX = px,
                TranslateY = py
            }; 
        }

        public Path TargetPath { get; set; }
    }
}
