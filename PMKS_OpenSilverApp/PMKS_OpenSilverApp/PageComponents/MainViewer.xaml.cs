using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Windows.Controls.Primitives;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using PMKS;
using System;
using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using Silverlight_PMKS;
using Silverlight_PMKS.Shapes.Static_Shapes;
using Point = System.Windows.Point;


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
        private double _kinematicSpaceWidth, _kinematicSpaceHeight;
        public double minX, maxX, minY, maxY;

        List<InputJointBaseShape> initialPositionIcons = new List<InputJointBaseShape>();

        public Boolean multiSelect;
        internal Point startMovingReference;

        private double xPanAnchor, yPanAnchor;

        private double penThick;
        private double jointSize;
        private GroundLinkShape groundLinkShape;
        private Axes axes;
        private double _scaleFactor;
        public bool inTheMidstMoving;

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

            SimulateOnMove = new BackgroundWorker
            {
                WorkerSupportsCancellation = true
            };
            SimulateOnMove.DoWork += SimulateOnMove_DoWork;
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
                if (Constants.sameCloseZero(_scaleFactor, value)) return;
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

            for (int i = 0; i < pmks.NumJoints; i++)
            {
                var jt = pmks.Joints[i];
                if (jt.FixedWithRespectTo(pmks.GroundLink)) continue;
                Children.Add(new PositionPath(i, pmks.JointParameters, jointData[i], XAxisOffset, YAxisOffset, pmks.CycleType == CycleTypes.OneCycle,
                     penThick));
                DynamicJointBaseShape displayJoint;
                switch (jt.TypeOfJoint)
                {
                    case JointType.P:
                        var fixedLink = pmks.Joints[i].Link2;
                        displayJoint = new PJointShape(pmks.Joints[i], fixedLink, timeSlider, pmks, jointSize, penThick, XAxisOffset, YAxisOffset);
                        break;
                    default: displayJoint = new RJointShape(pmks.Joints[i], timeSlider, pmks, jointSize, penThick, XAxisOffset, YAxisOffset);
                        break;
                }
                if (App.main.drivingIndex == i)
                {
                    displayJoint.Stroke = new SolidColorBrush {Color = Color.FromArgb(180, 40, 255,0)};
                    displayJoint.StrokeThickness = 30.0;
                }
                Children.Add(displayJoint);
                Children.Add(new AccelerationVector(pmks.Joints[i], timeSlider, pmks, AccelFactor, penThick, XAxisOffset, YAxisOffset, displayJoint, jointData[i]));
                Children.Add(new VelocityVector(pmks.Joints[i], timeSlider, pmks, VelocityFactor, penThick, XAxisOffset, YAxisOffset, displayJoint, jointData[i]));
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

        internal void DrawStaticShapes(Simulator pmks, ObservableCollection<JointData> jointData)
        {
            Children.Clear();
            Children.Add(axes);
            initialPositionIcons.Clear();
            for (int index = 0; index < pmks.Joints.Count; index++)
            {
                var j = pmks.Joints[index];
                var isDriver = (index == App.main.drivingIndex);
                JointData jointRowData = (index < jointData.Count) ? jointData[index] : null;
                InputJointBaseShape inputJointBaseShape = null;
                switch (j.TypeOfJoint)
                {
                    case JointType.R:
                        inputJointBaseShape =
                            new InputRJointShape(jointSize, penThick, j.xInitial, j.yInitial, XAxisOffset,
                                YAxisOffset, j.IsGround, isDriver,index,jointRowData);
                        break;
                    case JointType.P:
                        inputJointBaseShape =
                            new InputPJointShape(jointSize, penThick, j.xInitial, j.yInitial, XAxisOffset,
                                YAxisOffset, j.SlideAngleInitial, j.IsGround, isDriver, index, jointRowData);
                        break;
                    case JointType.RP:
                        inputJointBaseShape =
                            new InputRPJointShape(jointSize, penThick, j.xInitial, j.yInitial, XAxisOffset,
                                YAxisOffset, j.SlideAngleInitial, j.IsGround, index, jointRowData);
                        break;
                    case JointType.G:
                        inputJointBaseShape = new InputGJointShape(jointSize, penThick, j.xInitial, j.yInitial, XAxisOffset,
                                YAxisOffset, j.SlideAngleInitial, j.IsGround,index, jointRowData);
                        break;
                }
                Children.Add(inputJointBaseShape);
                initialPositionIcons.Add(inputJointBaseShape);
            }
            /* remove old ground shapes */
            Children.Remove(groundLinkShape);
            groundLinkShape = new GroundLinkShape(pmks.GroundLink, XAxisOffset, YAxisOffset, penThick, jointSize,
                                                 DisplayConstants.DefaultBufferRadius / ScaleFactor);
            Children.Add(groundLinkShape);
            /* now add the link shapes */
            for (int i = 0; i < pmks.NumLinks; i++)
            {
                if (!pmks.Links[i].name.Equals("ground"))
                    Children.Add(new LinkShape(i, pmks.Links[i], XAxisOffset, YAxisOffset, penThick, jointSize, null,
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

        private List<UIElement> shapesCreatedDuringMove;

        private void UpdateShapesDuringMove(List<bool> changedJoints, ObservableCollection<JointData> data, DoWorkEventArgs doWorkEventArgs)
        {
            try
            {
                if (shapesCreatedDuringMove == null || !shapesCreatedDuringMove.Any())
                    if (SimulateOnMove.CancellationPending) { doWorkEventArgs.Cancel = true; return; }
                {
                    var linkNames = new List<string>();
                    shapesCreatedDuringMove = new List<UIElement>(Children.Where(s => s is PositionPath));
                    for (int i = 0; i < changedJoints.Count; i++)
                    {
                        if (!changedJoints[i]) continue;
                        foreach (var linkName in data[i].LinkNamesList.Where(ln => !linkNames.Contains(ln)))
                        {
                            linkNames.Add(linkName);
                            shapesCreatedDuringMove.Add(linkName.Equals("ground")
                                ? Children.First(s => s is GroundLinkShape)
                                : Children.First(s => (s is LinkShape && linkName.Equals(((LinkShape)s).Name))));
                        }
                    }
                }

                for (int index = shapesCreatedDuringMove.Count - 1; index >= 0; index--)
                {
                    if (SimulateOnMove.CancellationPending) { doWorkEventArgs.Cancel = true; return; }
                    var child = shapesCreatedDuringMove[index];
                    shapesCreatedDuringMove.RemoveAt(index);
                    Children.Remove(child);

                    if (child is LinkShape)
                    {
                        ((LinkShape)child).ClearBindings();
                        var thisLink = movingPMKS.Links.Contains(((LinkShape)child).thisLink)
                            ? ((LinkShape)child).thisLink
                            : movingPMKS.Links.First(c => c.name == ((LinkShape)child).thisLink.name);
                        child = new LinkShape(((LinkShape)child).linkNum, thisLink, XAxisOffset, YAxisOffset, penThick,
                            jointSize, null, DisplayConstants.DefaultBufferRadius / ScaleFactor);
                    }
                    else if (child is GroundLinkShape)
                        child = new GroundLinkShape(movingPMKS.GroundLink, XAxisOffset, YAxisOffset, penThick, jointSize,
                            DisplayConstants.DefaultBufferRadius / ScaleFactor);
                    else if (child is PositionPath)
                    {
                        child.ClearValue(OpacityProperty);
                        var i = ((PositionPath)child).index;
                        child = new PositionPath(i, movingPMKS.JointParameters, data[i],
                            XAxisOffset, YAxisOffset, movingPMKS.CycleType == CycleTypes.OneCycle,
                            penThick);
                    }
                    shapesCreatedDuringMove.Add(child);
                    Children.Add(child);
                }
            }
            catch (Exception exc)
            {
                App.main.status("Error in UpdateShapesDuringMove: " + exc);
                App.main.ParseData(true);
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

            for (int i = 0; i < pmks.NumJoints; i++)
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
                for (int i = 0; i < pmks.NumJoints; i++)
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
            // todo: something is still wrong with this equation - you will notice when you create a really big mechanism
            xPanAnchor = (ParentWidth - (maxX + minX) / 2 - newScaleFactor * MainCanvas.Width) / 2;
            yPanAnchor = (ParentHeight - (maxY + minY) / 2 - newScaleFactor * MainCanvas.Height) / 2;
            MoveScaleCanvas(newScaleFactor, xPanAnchor, yPanAnchor, DisplayConstants.ZoomTimeOnRedraw);
        }

        internal void MoveScaleCanvas(double newScaleFactor, double xPanAnchor, double yPanAnchor, double zoomTime)
        {
            if (stillZooming) return;
            var transform = MainCanvas.RenderTransform as CompositeTransform;
            if (transform == null || zoomTime == 0.0)
            {
                ScaleFactor = newScaleFactor;
                MainCanvas.RenderTransform = new CompositeTransform
                    {
                        ScaleX = ScaleFactor,
                        ScaleY = ScaleFactor,
                        TranslateX = xPanAnchor,
                        TranslateY = yPanAnchor
                    };
                return;
            }
            var oldXPanAnchor = transform.TranslateX;
            var oldYPanAnchor = transform.TranslateY;

            var duration = new Duration(TimeSpan.FromSeconds(zoomTime));
            var easing = new QuadraticEase { EasingMode = EasingMode.EaseOut };
            var animateScaleX = new DoubleAnimation
            {
                EasingFunction = easing,
                Duration = duration,
                From = ScaleFactor,
                To = newScaleFactor
            };
            var animateScaleY = new DoubleAnimation
            {
                EasingFunction = easing,
                Duration = duration,
                From = ScaleFactor,
                To = newScaleFactor
            };
            var animateTranslateX = new DoubleAnimation
            {
                EasingFunction = easing,
                Duration = duration,
                From = oldXPanAnchor,
                To = xPanAnchor
            };
            var animateTranslateY = new DoubleAnimation
            {
                EasingFunction = easing,
                Duration = duration,
                From = oldYPanAnchor,
                To = yPanAnchor
            };
            Storyboard.SetTarget(animateScaleX, transform);
            Storyboard.SetTargetProperty(animateScaleX, new PropertyPath(CompositeTransform.ScaleXProperty));
            Storyboard.SetTarget(animateScaleY, transform);
            Storyboard.SetTargetProperty(animateScaleY, new PropertyPath(CompositeTransform.ScaleYProperty));
            Storyboard.SetTarget(animateTranslateX, transform);
            Storyboard.SetTargetProperty(animateTranslateX, new PropertyPath(CompositeTransform.TranslateXProperty));
            Storyboard.SetTarget(animateTranslateY, transform);
            Storyboard.SetTargetProperty(animateTranslateY, new PropertyPath(CompositeTransform.TranslateYProperty));

            zoomSb = new Storyboard { Duration = duration };
            zoomSb.Completed += zoomSb_Completed;
            zoomSb.Children.Add(animateScaleX);
            zoomSb.Children.Add(animateScaleY);
            zoomSb.Children.Add(animateTranslateX);
            zoomSb.Children.Add(animateTranslateY);
            stillZooming = true;
            zoomSb.Begin();

            ScaleFactor = newScaleFactor;

        }

        private Boolean stillZooming;

        void zoomSb_Completed(object sender, EventArgs e)
        {
            stillZooming = false;
        }

        public void ClearDynamicShapesAndBindings()
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
        }
        #endregion

        #region Events: Overrides

        protected override void OnMouseMove(MouseEventArgs e)
        {
            var mousePos = e.GetPosition(App.main.mainViewer.MainCanvas);
            if (!multiSelect && !inTheMidstMoving) startMovingReference = mousePos;
            var aJointHasChanged = false;
            var coordinates = new List<double[]>();
            var changedJoints = new List<Boolean>();
            foreach (var inputJointBaseShape in initialPositionIcons)
            {
                var changed = inputJointBaseShape.OnMouseMove(mousePos, startMovingReference, multiSelect);
                aJointHasChanged = (aJointHasChanged || changed);
                changedJoints.Add(changed);
                coordinates.Add(
                    //double.IsNaN(inputJointBaseShape.angle)
                    //? new[] { inputJointBaseShape.xCoord, inputJointBaseShape.yCoord }         :
                    new[] { inputJointBaseShape.xCoord, inputJointBaseShape.yCoord, inputJointBaseShape.angle });
            }
            if (aJointHasChanged && !SimulateOnMove.IsBusy)
            {
                App.main.PlayButton_Unchecked(null, null);
                SimulateOnMove.RunWorkerAsync(new object[] { coordinates, changedJoints });
            }
            base.OnMouseMove(e);
        }

        private BackgroundWorker SimulateOnMove;
        private Simulator movingPMKS;
        private Storyboard zoomSb;


        protected override void OnMouseLeftButtonDown(MouseButtonEventArgs e)
        {
            foreach (var inputJointBaseShape in initialPositionIcons)
                if (!e.Handled)
                    inputJointBaseShape.OnMouseLeftButtonDown(e, multiSelect);
            inTheMidstMoving = e.Handled;
            base.OnMouseLeftButtonDown(e);
        }

        void SimulateOnMove_DoWork(object sender, DoWorkEventArgs e)
        {
            var coordinates = (List<double[]>)((object[])e.Argument)[0];
            var changedJoints = (List<Boolean>)((object[])e.Argument)[1];
            if (SimulateOnMove.CancellationPending) { e.Cancel = true; return; }
            if (movingPMKS == null)
            {
                movingPMKS = new Simulator(App.main.LinkIDs, App.main.JointTypes,
                  App.main.drivingIndex, coordinates) { MaxSmoothingError = 0.1 };
            }
            else movingPMKS.AssignPositions(coordinates);
            if (SimulateOnMove.CancellationPending) { e.Cancel = true; return; }
            if (movingPMKS.DegreesOfFreedom != 1)
                return;
            movingPMKS.FindFullMovement();
            App.main.Dispatcher.BeginInvoke(() => UpdateShapesDuringMove(changedJoints, App.main.JointsInfo.Data, e));
        }

        protected override void OnMouseLeftButtonUp(MouseButtonEventArgs e)
        {
            SimulateOnMove.CancelAsync();
            App.main.Panning = false;
            movingPMKS = null;
            shapesCreatedDuringMove = null;
            if (multiSelect) return;
            inTheMidstMoving = false;
            foreach (var inputJointBaseShape in initialPositionIcons)
                //if (!e.Handled)
                inputJointBaseShape.OnMouseLeftButtonUp(e, multiSelect);
            if (e.Handled) App.main.ParseData();
            base.OnMouseLeftButtonUp(e);
        }

        #endregion



    }
}
