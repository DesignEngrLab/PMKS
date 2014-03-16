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

        List<InputJointBaseShape> initialPositionIcons = new List<InputJointBaseShape>();

        public Boolean multiSelect;
        internal Point startMovingReference;

        private double xPanAnchor, yPanAnchor;

        private double penThick;
        private double jointSize;
        private GroundLinkShape groundLinkShape;
        private Axes axes;
        private double _scaleFactor;
        private bool inTheMidstMoving;

        #endregion

        #region Constructor
        public MainViewer()
        {
            InitializeComponent();
            MainCanvas.Width = MainCanvas.Height = DisplayConstants.UltimateWindowWidth / DisplayConstants.MaxZoomOut;
            XAxisOffset = DisplayConstants.UltimateWindowWidth / (2 * DisplayConstants.MaxZoomOut);
            YAxisOffset = DisplayConstants.UltimateWindowWidth / (2 * DisplayConstants.MaxZoomOut);

            axes = new Axes(penThick, XAxisOffset, YAxisOffset, MainCanvas.Width, MainCanvas.Height);
            Children.Add(axes);
        }
        #endregion

        #region Properties
        public UIElementCollection Children { get { return MainCanvas.Children; } }
        /// <summary>
        /// Gets or sets the target path.
        /// </summary>
        /// <value>
        /// The target path.
        /// </value>
        public Path TargetPath { get; set; }
        /// <summary>
        /// Gets the x axis offset.
        /// </summary>
        /// <value>
        /// The x axis offset.
        /// </value>
        public double XAxisOffset { get; private set; }

        /// <summary>
        /// Gets the y axis offset.
        /// </summary>
        /// <value>
        /// The y axis offset.
        /// </value>
        public double YAxisOffset { get; private set; }

        /// <summary>
        /// Gets the story board.
        /// </summary>
        /// <value>
        /// The story board.
        /// </value>
        public Storyboard animateMechanismStoryBoard { get; private set; }

        /// <summary>
        /// Gets the scale factor.
        /// </summary>
        /// <value>
        /// The scale factor.
        /// </value>
        public double ScaleFactor
        {
            get { return _scaleFactor; }
            set
            {
                _scaleFactor = value;
                penThick = DisplayConstants.PenThicknessRatio / _scaleFactor;
                jointSize = DisplayConstants.JointSize / _scaleFactor;
                axes.StrokeThickness = penThick;
            }
        }

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

        #region Methods
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
            foreach (var child in Children)
                if (child is LinkShape)
                    ((LinkShape)child).SetBindings(timeSlider, pmks, XAxisOffset, YAxisOffset);

            for (int i = 0; i < pmks.numJoints; i++)
            {
                var j = pmks.JointNewIndexFromOriginal[i];
                if (pmks.AllJoints[j].FixedWithRespectTo(pmks.groundLink)) continue;
                Children.Add(new PositionPath(j, pmks.JointParameters, jointData[i], XAxisOffset, YAxisOffset, pmks.CycleType == CycleTypes.OneCycle,
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
                Children.Add(displayJoint);
                Children.Add(new AccelerationVector(pmks.AllJoints[j], timeSlider, pmks, AccelFactor, penThick, XAxisOffset, YAxisOffset, displayJoint, jointData[i]));
                Children.Add(new VelocityVector(pmks.AllJoints[j], timeSlider, pmks, VelocityFactor, penThick, XAxisOffset, YAxisOffset, displayJoint, jointData[i]));
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
            animateMechanismStoryBoard = new Storyboard
                {
                    Duration = duration,
                    AutoReverse = pmks.CycleType != CycleTypes.OneCycle,
                    RepeatBehavior = RepeatBehavior.Forever
                };
            animateMechanismStoryBoard.Children.Add(timeAnimation);
            Storyboard.SetTarget(timeAnimation, timeSlider);
            Storyboard.SetTargetProperty(timeAnimation, new PropertyPath(RangeBase.ValueProperty));
            if ((bool)App.main.PlayButton.IsChecked)
                animateMechanismStoryBoard.Begin();
        }

        public void DrawStaticShapes(Simulator pmks, ObservableCollection<JointData> jointData, bool sameTopology)
        {
            //if (!sameTopology)
            //{
                Children.Clear();
                Children.Add(axes);
                initialPositionIcons.Clear();
                for (int index = 0; index < pmks.AllJoints.Count; index++)
                {
                    var j = pmks.AllJoints[pmks.JointNewIndexFromOriginal[index]];

                    InputJointBaseShape inputJointBaseShape = null;
                    switch (j.jointType)
                    {
                        case JointTypes.R:
                            inputJointBaseShape =
                                new InputRJointShape(jointSize, penThick, j.xInitial, j.yInitial, XAxisOffset,
                                    YAxisOffset, j.isGround, jointData[index]);
                            break;
                        case JointTypes.P:
                            inputJointBaseShape =
                                new InputPJointShape(jointSize, penThick, j.xInitial, j.yInitial, XAxisOffset,
                                    YAxisOffset, j.InitSlideAngle + j.Link1.AngleInitial, j.isGround, jointData[index]);
                            break;
                    }
                    Children.Add(inputJointBaseShape);
                    initialPositionIcons.Add(inputJointBaseShape);
                }
            //}
            //else
            //{
            //    for (int index = 0; index < pmks.AllJoints.Count; index++)
            //    {
            //        var j = pmks.AllJoints[pmks.JointNewIndexFromOriginal[index]];
            //        var icon = initialPositionIcons[index];
            //        icon.SetPosition(j.xInitial,j.yInitial, XAxisOffset, YAxisOffset,
            //            j.InitSlideAngle + j.Link1.AngleInitial);
            //    }
            //}

            /* remove old ground shapes */
            Children.Remove(groundLinkShape);

            groundLinkShape = new GroundLinkShape(pmks.groundLink, XAxisOffset, YAxisOffset, penThick, jointSize,
                                                 DisplayConstants.DefaultBufferRadius / ScaleFactor);
            Children.Add(groundLinkShape);

            /* now add the link shapes */
            for (int i = 0; i < pmks.numLinks; i++)
            {
                if (!pmks.AllLinks[i].isGround)
                    Children.Add(new LinkShape(i, pmks.AllLinks[i], XAxisOffset, YAxisOffset, penThick, jointSize, null,
                                                            DisplayConstants.DefaultBufferRadius / ScaleFactor));
            }
            if (TargetPath != null)
            {
                Children.Remove(TargetPath);
                TargetPath.RenderTransform
                    = new TranslateTransform { X = XAxisOffset, Y = YAxisOffset };
                Children.Add(TargetPath);
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



        internal void UpdateScaleAndCenter()
        {
            var w = _kinematicSpaceWidth + 2 * DisplayConstants.DefaultBufferMultipler * _kinematicSpaceWidth;
            var h = _kinematicSpaceHeight + 2 * DisplayConstants.DefaultBufferMultipler * _kinematicSpaceHeight;
            var newScaleFactor = Math.Min(ParentWidth / w, ParentHeight / h);
            if (newScaleFactor > DisplayConstants.MaxZoomIn) newScaleFactor = DisplayConstants.MaxZoomIn;
            if (newScaleFactor < DisplayConstants.MaxZoomOut) newScaleFactor = DisplayConstants.MaxZoomOut;
            var oldXPanAnchor = xPanAnchor;
            var oldYPanAnchor = yPanAnchor;
            xPanAnchor = (ParentWidth - newScaleFactor * MainCanvas.Width) / 2;
            yPanAnchor = (ParentHeight - newScaleFactor * MainCanvas.Height) / 2;

            //var duration = new Duration(TimeSpan.FromSeconds(DisplayConstants.ZoomTime));
            //var easing = new QuadraticEase { EasingMode = EasingMode.EaseOut };
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
            //    To = oldXPanAnchor,
            //    From = xPanAnchor
            //};
            //var animateTranslateY = new DoubleAnimation
            //{
            //    EasingFunction = easing,
            //    Duration = duration,
            //    To = oldYPanAnchor,
            //    From = yPanAnchor
            //};
            //var transform = new CompositeTransform
            //{
            //    ScaleX = ScaleFactor,
            //    ScaleY = ScaleFactor,
            //    TranslateX = oldXPanAnchor,
            //    TranslateY = oldYPanAnchor

            //};
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
            var transform = new CompositeTransform
            {
                ScaleX = ScaleFactor,
                ScaleY = ScaleFactor,
                TranslateX = xPanAnchor,
                TranslateY = yPanAnchor

            };
            MainCanvas.RenderTransform = transform;
        }
        public void ClearDynamicShapesAndBindings(bool sameTopology)
        {
            for (int index = Children.Count - 1; index >= 0; index--)
            {
                var child = Children[index];
                if (child is DisplayVectorBaseShape)
                {
                    ((DisplayVectorBaseShape)child).ClearBindings();
                    Children.RemoveAt(index);
                }
                else if (child is DynamicJointBaseShape)
                {
                    ((DynamicJointBaseShape)child).ClearBindings();
                    Children.RemoveAt(index);
                }
                else if (child is LinkShape)
                {
                    ((LinkShape)child).ClearBindings();
                    Children.RemoveAt(index);
                }
                else if (child is PositionPath)
                {
                    child.ClearValue(OpacityProperty);
                    Children.RemoveAt(index);
                }
            }
            // if (!sameTopology) {Children.Clear();}
        }

        #endregion

        #region Events: Overrides

        protected override void OnMouseMove(MouseEventArgs e)
        {                                                
            var mousePos = e.GetPosition(App.main.mainViewer.MainCanvas);

            if (!multiSelect && !inTheMidstMoving) startMovingReference = mousePos;

            var jointMoved = false;
            foreach (var inputJointBaseShape in initialPositionIcons)
                jointMoved = (inputJointBaseShape.OnMouseMove(mousePos,startMovingReference, multiSelect) || jointMoved);
            //if (jointMoved)
            //    App.main.ParseData(false, false);
            base.OnMouseMove(e);
        }

        protected override void OnMouseLeftButtonDown(MouseButtonEventArgs e)
        {
            foreach (var inputJointBaseShape in initialPositionIcons)
                if (!e.Handled)
                    inputJointBaseShape.OnMouseLeftButtonDown(e, multiSelect);
            inTheMidstMoving = e.Handled;
            base.OnMouseLeftButtonDown(e);
        }

        protected override void OnMouseLeftButtonUp(MouseButtonEventArgs e)
        {
            if (multiSelect) return;
            inTheMidstMoving = false;
            foreach (var inputJointBaseShape in initialPositionIcons)
                //if (!e.Handled)
                inputJointBaseShape.OnMouseLeftButtonUp(e, multiSelect);
            if (e.Handled) App.main.ParseData();
            base.OnMouseLeftButtonUp(e);
        }

        internal void MoveScaleCanvasFromMouse(double newScaleFactor, Point newPanAnchor)
        {
            ScaleFactor = newScaleFactor;
            MainCanvas.RenderTransform = new CompositeTransform
              {
                  ScaleX = ScaleFactor,
                  ScaleY = ScaleFactor,
                  TranslateX = newPanAnchor.X,
                  TranslateY = newPanAnchor.Y
              };
        }
        #endregion
    }
}
