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
        /* the canvas width and height are the distances in the coordinates of the space.
         * _kinematicSpaceWidth = maxX - minX & _kinematicSpaceHeight = maxY - minY
         * minX, maxX are the bounds (negative or positive in this coord. space)
         * they are based on the amount of movement in the mechanism plus a border set by 
         *  DisplayConstants.AxesBuffer which is set to 1/4 in. or 24 pixels */
        private double _kinematicSpaceWidth, _kinematicSpaceHeight, minX, maxX, minY, maxY;

        /// <summary>
        /// Gets the x axis offset.
        /// </summary>
        /// <value>
        /// The x axis offset.
        /// </value>
        public double XAxisOffset { get; private set; }

        public double YAxisOffset { get; private set; }

        private double penThick;
        private double jointSize;
        private GroundLinkShape groundLinkShape;
        private Axes axes;
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
        public double ScaleFactor = 1.0;
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


        internal void DrawDynamicShapes(Simulator pmks, ObservableCollection<JointData> jointData, Slider timeSlider)
        {
            #region draw position, velocity, and acceleration curves

            timeSlider.Maximum = pmks.EndTime;
            App.main.maxTimeText.Text = timeSlider.Maximum.ToString("F");
            timeSlider.Minimum = pmks.BeginTime;
            App.main.minTimeText.Text = timeSlider.Minimum.ToString("F");
            var h = (timeSlider.ActualHeight == 0) ? ParentHeight : timeSlider.ActualHeight;
            timeSlider.LargeChange = (timeSlider.Maximum - timeSlider.Minimum) * DisplayConstants.TickDistance / h;
            timeSlider.SmallChange = (timeSlider.Maximum - timeSlider.Minimum) / 1000;
            timeSlider.Value = 0.0;


            /************ creating the shapes ***************/
            foreach (var child in MainCanvas.Children)
                if (child is LinkShape)
                    ((LinkShape)child).SetBindings(timeSlider, pmks, XAxisOffset, YAxisOffset);

            for (int i = 0; i < pmks.numJoints; i++)
            {
                var j = pmks.JointNewIndexFromOriginal[i];
                if (pmks.AllJoints[j].FixedWithRespectTo(pmks.groundLink)) continue;
                MainCanvas.Children.Add(new PositionPath(j, pmks.JointParameters, jointData[i], XAxisOffset, YAxisOffset, pmks.CycleType == CycleTypes.OneCycle,
                     penThick));
                DynamicJointBaseShape displayJoint;
                switch (pmks.AllJoints[j].jointType)
                {
                    case JointTypes.P:
                        var fixedLink = pmks.AllJoints[j].Link2;
                        displayJoint = new PJointShape(pmks.AllJoints[j], fixedLink, timeSlider, pmks, jointSize, penThick, XAxisOffset, YAxisOffset);
                        break;
                    default: displayJoint = new RJointShape(pmks.AllJoints[j], timeSlider, pmks, jointSize, penThick, XAxisOffset, YAxisOffset);
                        break;
                }
                MainCanvas.Children.Add(displayJoint);
                MainCanvas.Children.Add(new AccelerationVector(pmks.AllJoints[j], timeSlider, pmks, AccelFactor, penThick, XAxisOffset, YAxisOffset, displayJoint, jointData[i]));
                MainCanvas.Children.Add(new VelocityVector(pmks.AllJoints[j], timeSlider, pmks, VelocityFactor, penThick, XAxisOffset, YAxisOffset, displayJoint, jointData[i]));
            }
            /******************************************************************/
            #endregion

            var duration = new Duration(TimeSpan.FromSeconds(timeSlider.Maximum - timeSlider.Minimum));
            var timeAnimation = new DoubleAnimation
                {
                    From = timeSlider.Minimum,
                    To = timeSlider.Maximum,
                    Duration = duration,
                    AutoReverse = pmks.CycleType != CycleTypes.OneCycle,
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

        public void DrawStaticShapes(Simulator pmks, ObservableCollection<JointData> jointData, bool sameTopology)
        {
            if (!sameTopology)
            {
                MainCanvas.Children.Clear();
                initialPositionIcons.Clear();
                for (int index = 0; index < pmks.AllJoints.Count; index++)
                {
                    var j = pmks.AllJoints[pmks.JointNewIndexFromOriginal[index]];

                    InputJointBaseShape inputJointBaseShape = null;
                    switch (j.jointType)
                    {
                        case JointTypes.R:
                            inputJointBaseShape =
                                new InputRJointShape(jointSize, penThick, j.xInitial + XAxisOffset,
                                    j.yInitial + YAxisOffset, j.isGround, jointData[index]);
                            break;
                        case JointTypes.P:
                            inputJointBaseShape =
                                new InputPJointShape(jointSize, penThick, j.xInitial + XAxisOffset,
                                    j.yInitial + YAxisOffset,
                                    j.InitSlideAngle + j.Link1.AngleInitial, j.isGround, jointData[index]);
                            break;
                    }
                    MainCanvas.Children.Add(inputJointBaseShape);
                    initialPositionIcons.Add(inputJointBaseShape);
                }
            }
            else
            {
                //for (int index = 0; index < pmks.AllJoints.Count; index++)
                //{
                //    var j = pmks.AllJoints[pmks.JointNewIndexFromOriginal[index]];
                //    var icon = initialPositionIcons[index];
                //    icon.SetPosition(j.xInitial + XAxisOffset,
                //        j.yInitial + YAxisOffset,
                //        j.InitSlideAngle + j.Link1.AngleInitial);
                //}
            }
            /* remove old Axes */
            MainCanvas.Children.Remove(axes);
            /* add new ones (based on XOffset and YOffest */
            axes = new Axes(penThick, XAxisOffset, YAxisOffset, MainCanvas.Width, MainCanvas.Height);
            MainCanvas.Children.Add(axes);
                                             
            /* remove old ground shapes */
            MainCanvas.Children.Remove(groundLinkShape);

            groundLinkShape = new GroundLinkShape(pmks.groundLink, XAxisOffset, YAxisOffset, penThick, jointSize,
                                                 DisplayConstants.DefaultBufferRadius / ScaleFactor);
            MainCanvas.Children.Add(groundLinkShape);

            /* now add the link shapes */
            for (int i = 0; i < pmks.numLinks; i++)
            {
                if (!pmks.AllLinks[i].isGround)
                    MainCanvas.Children.Add(new LinkShape(i, pmks.AllLinks[i], XAxisOffset, YAxisOffset, penThick, jointSize, null,
                                                            DisplayConstants.DefaultBufferRadius / ScaleFactor));
            }
            if (TargetPath != null)
            {
                MainCanvas.Children.Remove(TargetPath);
                TargetPath.RenderTransform
                    = new TranslateTransform { X = XAxisOffset, Y = YAxisOffset };
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
            _kinematicSpaceWidth = maxX - minX;
            _kinematicSpaceHeight = maxY - minY;
        }
        /// <summary>
        /// Updates the _kinematicSpaceWidth, _kinematicSpaceHeight, minX, maxX, minY, maxY
        /// </summary>
        /// <param name="pmks">The PMKS.</param>
        internal void UpdateRanges(Simulator pmks)
        {
            if (pmks == null) return;
            var origMinX = minX = -DisplayConstants.AxesBuffer;
            var origMinY = minY = -DisplayConstants.AxesBuffer;
            var origMaxX = maxX = DisplayConstants.AxesBuffer;
            var origMaxY = maxY = DisplayConstants.AxesBuffer;

            for (int i = 0; i < pmks.numJoints; i++)
            {
                if (origMinX > pmks.JointParameters.Parameters[0][i, 0])
                    origMinX = pmks.JointParameters.Parameters[0][i, 0];
                if (origMinY > pmks.JointParameters.Parameters[0][i, 1])
                    origMinY = pmks.JointParameters.Parameters[0][i, 1];
                if (origMaxX < pmks.JointParameters.Parameters[0][i, 0])
                    origMaxX = pmks.JointParameters.Parameters[0][i, 0];
                if (origMaxY < pmks.JointParameters.Parameters[0][i, 1])
                    origMaxY = pmks.JointParameters.Parameters[0][i, 1];
            }
            for (int j = 0; j < pmks.JointParameters.Count; j++)
                for (int i = 0; i < pmks.numJoints; i++)
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
            if ((maxX - minX) > 3 * (origMaxX - origMinX))
            {
                _kinematicSpaceWidth = 3 * (origMaxX - origMinX);
                minX = origMinX - (origMaxX - origMinX);
                maxX = origMaxX + (origMaxX - origMinX);
            }
            else _kinematicSpaceWidth = (maxX - minX);
            if ((maxY - minY) > 3 * (origMaxY - origMinY))
            {
                _kinematicSpaceHeight = 3 * (origMaxY - origMinY);
                minY = origMinY - (origMaxY - origMinY);
                maxY = origMaxY + (origMaxY - origMinY);

            }
            else _kinematicSpaceHeight = (maxY - minY);
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

            VelocityFactor = Math.Min(_kinematicSpaceWidth / maxVx, _kinematicSpaceHeight / maxVy);

            AccelFactor = Math.Min(_kinematicSpaceWidth / maxAx, _kinematicSpaceHeight / maxAy);

        }
        List<InputJointBaseShape> initialPositionIcons = new List<InputJointBaseShape>();
        protected override void OnMouseMove(MouseEventArgs e)
        {
            if (multiSelect) return;
            var jointMoved = false;
            foreach (var inputJointBaseShape in initialPositionIcons)
                jointMoved = (inputJointBaseShape.OnMouseMove(e) || jointMoved);
            if (jointMoved)
                App.main.ParseData(false, false);
            base.OnMouseMove(e);
        }

        protected override void OnMouseLeftButtonDown(MouseButtonEventArgs e)
        {
            foreach (var inputJointBaseShape in initialPositionIcons)
                if (!e.Handled)
                    inputJointBaseShape.OnMouseLeftButtonDown(e, multiSelect);
            base.OnMouseLeftButtonDown(e);
        }

        public Boolean multiSelect;
        private double xPanAnchor, yPanAnchor;

        protected override void OnMouseLeftButtonUp(MouseButtonEventArgs e)
        {
            foreach (var inputJointBaseShape in initialPositionIcons)
                //if (!e.Handled)
                inputJointBaseShape.OnMouseLeftButtonUp(e, multiSelect);
           // if (e.Handled) App.main.ParseData();
            base.OnMouseLeftButtonUp(e);
        }

        internal void UpdateScaleAndCenter()
        {
            var oldXOffset = XAxisOffset;
            var oldYOffset = YAxisOffset;
            XAxisOffset = DisplayConstants.DefaultBufferMultipler * _kinematicSpaceWidth - minX;
            YAxisOffset = DisplayConstants.DefaultBufferMultipler * _kinematicSpaceHeight - minY;

            MainCanvas.Width = _kinematicSpaceWidth + 2 * DisplayConstants.DefaultBufferMultipler * _kinematicSpaceWidth;
            MainCanvas.Height = _kinematicSpaceHeight + 2 * DisplayConstants.DefaultBufferMultipler * _kinematicSpaceHeight;

            var newScaleFactor = Math.Min(DisplayConstants.DefaultBufferMultipler * ParentWidth / (MainCanvas.Width), 
                DisplayConstants.DefaultBufferMultipler * ParentHeight / (MainCanvas.Height));
          //  newScaleFactor = ScaleFactor;


            if (newScaleFactor > DisplayConstants.MaxZoomIn) newScaleFactor = DisplayConstants.MaxZoomIn;
            if (newScaleFactor < DisplayConstants.MaxZoomOut) newScaleFactor = DisplayConstants.MaxZoomOut;
            var oldXPanAnchor = xPanAnchor;
            var oldYPanAnchor = yPanAnchor;
            xPanAnchor = (ParentWidth - newScaleFactor * MainCanvas.Width) / 2;
            yPanAnchor = (ParentHeight - newScaleFactor * MainCanvas.Height) / 2;
            MoveScaleCanvasFromMouse(newScaleFactor, new Point(xPanAnchor, yPanAnchor));

            //var duration = new Duration(TimeSpan.FromSeconds(DisplayConstants.ZoomTime));
            //var easing = new QuadraticEase { EasingMode = EasingMode.EaseOut };
            //var oldCx = newScaleFactor / ScaleFactor * (oldXOffset + oldXPanAnchor - XAxisOffset);
            //var oldCy = newScaleFactor / ScaleFactor * (oldYOffset + oldYPanAnchor - YAxisOffset);
            //var animateScaleX = new DoubleAnimation
            //{
            //    EasingFunction = easing,
            //    Duration = duration,
            //    From = ScaleFactor,
            //    To = newScaleFactor
            //};
            //var animateScaleY = new DoubleAnimation
            //{
            //    EasingFunction = easing,
            //    Duration = duration,
            //    From = ScaleFactor,
            //    To = newScaleFactor
            //};
            //var animateTranslateX = new DoubleAnimation
            //{
            //    EasingFunction = easing,
            //    Duration = duration,
            //    To = xPanAnchor,
            //    From = oldCx
            //};
            //var animateTranslateY = new DoubleAnimation
            //{
            //    EasingFunction = easing,
            //    Duration = duration,
            //    To = yPanAnchor,
            //    From = oldCy
            //};
            //var transform = new CompositeTransform
            //{
            //    ScaleX = ScaleFactor,
            //    ScaleY = ScaleFactor,
            //    TranslateX = oldCx,
            //    TranslateY = oldCy

            //};
            //MainCanvas.RenderTransform = transform;
            //Storyboard.SetTarget(animateScaleX, transform);
            //Storyboard.SetTargetProperty(animateScaleX, new PropertyPath(CompositeTransform.ScaleXProperty));
            //Storyboard.SetTarget(animateScaleY, transform);
            //Storyboard.SetTargetProperty(animateScaleY, new PropertyPath(CompositeTransform.ScaleYProperty));
            //Storyboard.SetTarget(animateTranslateX, transform);
            //Storyboard.SetTargetProperty(animateTranslateX, new PropertyPath(CompositeTransform.TranslateXProperty));
            //Storyboard.SetTarget(animateTranslateY, transform);
            //Storyboard.SetTargetProperty(animateTranslateY, new PropertyPath(CompositeTransform.TranslateYProperty));

            //var zoomSb = new Storyboard { Duration = duration };
            //zoomSb.Children.Add(animateScaleX);
            //zoomSb.Children.Add(animateScaleY);
            //zoomSb.Children.Add(animateTranslateX);
            //zoomSb.Children.Add(animateTranslateY);

            //zoomSb.Begin();

            ScaleFactor = newScaleFactor;
            penThick = DisplayConstants.PenThicknessRatio / ScaleFactor;
            jointSize = DisplayConstants.JointSize / ScaleFactor;
        }
        internal void MoveScaleCanvasFromMouse(double newScaleFactor, Point newPanAnchor)
        {
            MainCanvas.RenderTransform = new CompositeTransform
            {
                ScaleX = newScaleFactor,
                ScaleY = newScaleFactor,
                TranslateX = newPanAnchor.X,
                TranslateY = newPanAnchor.Y
            };
            ScaleFactor = newScaleFactor;
        }

        public Path TargetPath { get; set; }

        public void ClearDynamicShapesAndBindings(bool sameTopology)
        {
            for (int index = MainCanvas.Children.Count - 1; index >= 0; index--)
            {
                var child = MainCanvas.Children[index];
                if (child is DisplayVectorBaseShape)
                {
                    ((DisplayVectorBaseShape)child).ClearBindings();
                    MainCanvas.Children.RemoveAt(index);
                }
                else if (child is DynamicJointBaseShape)
                {
                    ((DynamicJointBaseShape)child).ClearBindings();
                    MainCanvas.Children.RemoveAt(index);
                }
                else if (child is LinkShape)
                {
                    ((LinkShape)child).ClearBindings();
                    MainCanvas.Children.RemoveAt(index);
                }
                else if (child is PositionPath)
                {
                    child.ClearValue(OpacityProperty);
                    MainCanvas.Children.RemoveAt(index);
                }
            }
            if (!sameTopology) MainCanvas.Children.Clear();
        }
    }
}
