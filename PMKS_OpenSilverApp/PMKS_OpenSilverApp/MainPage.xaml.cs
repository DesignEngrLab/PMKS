using System.Windows.Media;
using System.Windows.Shapes;
using PMKS;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Input;
using Silverlight_PMKS;
using Point = System.Windows.Point;

namespace PMKS_Silverlight_App
{
    public partial class MainPage : UserControl
    {
        #region Fields
        public Simulator pmks;
        public readonly List<string[]> LinkIDs = new List<string[]>();
        public readonly List<JointType> JointTypes = new List<JointType>();
        private readonly List<double[]> InitPositions = new List<double[]>();
        private int numJoints;
        public int drivingIndex;
        public JointsViewModel JointsInfo;
        public LinksViewModel LinksInfo;
        #endregion
        #region Properties
        private static void GlobalSettingChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            ((MainPage)d).ParseData(true);
        }
        public static readonly DependencyProperty SpeedProperty
            = DependencyProperty.Register("Speed", typeof(double), typeof(MainPage),
                                          new PropertyMetadata(DisplayConstants.DefaultSpeed,
                                              GlobalSettingChanged));

        public double Speed
        {
            get { return (double)GetValue(SpeedProperty); }
            set { SetValue(SpeedProperty, value); }
        }
        public static readonly DependencyProperty ErrorProperty
            = DependencyProperty.Register("Error", typeof(double), typeof(MainPage),
                                 new PropertyMetadata(DisplayConstants.DefaultError, GlobalSettingChanged));
        public double Error
        {
            get { return (double)GetValue(ErrorProperty); }
            set { SetValue(ErrorProperty, value); }
        }
        public static readonly DependencyProperty AngleIncrementProperty
            = DependencyProperty.Register("AngleIncrement", typeof(double), typeof(MainPage),
                                 new PropertyMetadata(DisplayConstants.DefaultAngleInc, GlobalSettingChanged));
        public double AngleIncrement
        {
            get { return (double)GetValue(AngleIncrementProperty); }
            set { SetValue(AngleIncrementProperty, value); }
        }

        public static readonly DependencyProperty AngleUnitsProperty
            = DependencyProperty.Register("AngleUnits", typeof(AngleType), typeof(MainPage),
                                 new PropertyMetadata(AngleType.Degrees, GlobalSettingChanged));
        public AngleType AngleUnits
        {
            get { return (AngleType)GetValue(AngleUnitsProperty); }
            set { SetValue(AngleUnitsProperty, value); }
        }

        public static readonly DependencyProperty LengthUnitsProperty
            = DependencyProperty.Register("LengthUnits", typeof(LengthType), typeof(MainPage),
                                 new PropertyMetadata(LengthType.mm, GlobalSettingChanged));
        public LengthType LengthUnits
        {
            get { return (LengthType)GetValue(LengthUnitsProperty); }
            set { SetValue(LengthUnitsProperty, value); }
        }
        public static readonly DependencyProperty AnalysisStepProperty
            = DependencyProperty.Register("AnalysisStep", typeof(AnalysisType), typeof(MainPage),
                                 new PropertyMetadata(AnalysisType.error, GlobalSettingChanged));
        public AnalysisType AnalysisStep
        {
            get { return (AnalysisType)GetValue(AnalysisStepProperty); }
            set { SetValue(AnalysisStepProperty, value); }
        }

        #endregion

        public MainPage()
        {
            LinksInfo = new LinksViewModel();
            JointsInfo = new JointsViewModel();
            InitializeComponent();
            InputJointBaseShape.LoadShapes();
            App.Current.Host.Content.Resized += Content_Resized;
        }

        private void MainPage_Loaded_1(object sender, RoutedEventArgs e)
        {
            /****** bind joint info to datagrid ******/
            var binding = new Binding
            {
                Source = JointsInfo,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(JointsViewModel.DataCollectionProperty)
            };
            fileAndEditPanel.JointDataGrid.SetBinding(DataGrid.ItemsSourceProperty, binding);

            /****** bind link info to link datagrid ******/
            binding = new Binding
            {
                Source = LinksInfo,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(LinksViewModel.DataCollectionProperty)
            };
            linkInputTable.linkDataGrid.SetBinding(DataGrid.ItemsSourceProperty, binding);

            /****** bind joint info to link info ******/
            //binding = new Binding
            //{
            //    Source = JointsInfo,
            //    Mode = BindingMode.OneWay,
            //    Path = new PropertyPath(LinksViewModel.DataCollectionProperty),
            //    Converter = new JointDataToLinkListConverter(LinksInfo)
            //};
            //BindingOperations.SetBinding(LinksInfo, LinksViewModel.DataCollectionProperty, binding);

            /****** bind speed to speedBox ******/
            binding = new Binding
            {
                Source = this,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(SpeedProperty),
                Converter = new TextToDoubleConverter(),
                ConverterParameter = Speed
            };
            globalSettings.speedBox.SetBinding(TextBox.TextProperty, binding);

            /****** bind angle increment to anglebox******/
            binding = new Binding
            {
                Source = this,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(AngleIncrementProperty),
                Converter = new TextToAngleConverter(this),
                ConverterParameter = AngleIncrement
            };
            globalSettings.AngleBox.SetBinding(TextBox.TextProperty, binding);

            /****** bind error percentage to errorbox******/
            binding = new Binding
            {
                Source = this,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(ErrorProperty),
                Converter = new TextToPercentageConverter(),
                ConverterParameter = Error
            };
            globalSettings.ErrorBox.SetBinding(TextBox.TextProperty, binding);

            /****** bind angle units  to radians toggle ******/
            binding = new Binding
            {
                Source = this,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(AngleUnitsProperty),
                Converter = new BooleanToAngleTypeConverter()
            };
            globalSettings.RadiansCheckBox.SetBinding(ToggleButton.IsCheckedProperty, binding);

            /****** bind length units to metric checkbox******/
            binding = new Binding
            {
                Source = this,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(LengthUnitsProperty),
                Converter = new BooleanToLengthTypeConverter()
            };
            globalSettings.MetricCheckBox.SetBinding(ToggleButton.IsCheckedProperty, binding);
            /****** bind accuracy type to error checkbox******/
            binding = new Binding
            {
                Source = this,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(AnalysisStepProperty),
                Converter = new BooleanToAnalysisStepConverter()
            };
            globalSettings.ErrorCheckBox.SetBinding(ToggleButton.IsCheckedProperty, binding);
            ParseData();
        }

        #region PMKS Controller Functions



        internal void ParseData(Boolean ForceRerunOfSimulation = false)
        {

            try
            {
                #region table validation
                if (JointsInfo == null) return;
                numJoints = TrimEmptyJoints();
                if (pmks != null && !ForceRerunOfSimulation && SameTopology() && SameParameters()) return;
                DefineInputDriver();
                DefineLinkIDs();
                if (!(GroundLinkFound() && DuplicateLinkNames() && DefinePositions() && DefineJointTypeList() && DataListsSameLength())) return;
                pmks = new Simulator(LinkIDs, JointTypes, drivingIndex, InitPositions);
                JointData.UpdateRandomRange(InitPositions);
                mainViewer.ClearDynamicShapesAndBindings();
                PlayButton_Unchecked(null, null);

                if (pmks.IsDyadic) status("The mechanism is comprised of only of dyads.");
                else status("The mechanism has non-dyadic loops.");
                int dof = pmks.DegreesOfFreedom;
                App.main.fileAndEditPanel.ReportDOF(dof);
                status("Degrees of freedom = " + dof);
                if (dof == 1)
                {
                    if (JointsInfo.Data[drivingIndex].JointTypeString.Equals("P", StringComparison.InvariantCultureIgnoreCase))
                    {
                        pmks.InputSpeed = Speed;
                        globalSettings.SpeedHeaderTextBlock.Text = "Speed (unit/sec)";
                        if (AnalysisStep == AnalysisType.error)
                            pmks.MaxSmoothingError = Error;
                        else
                            pmks.DeltaAngle = AngleIncrement;
                    }
                    else
                    {
                        pmks.InputSpeed = DisplayConstants.RadiansPerSecToRPM * Speed;
                        globalSettings.SpeedHeaderTextBlock.Text = "Speed (rpm)";
                        if (AnalysisStep == AnalysisType.error)
                            pmks.MaxSmoothingError = Error;
                        else
                            pmks.DeltaAngle = AngleIncrement / DisplayConstants.RadiansToDegrees;
                    }
                }
                else
                {
                    mainViewer.UpdateRanges(InitPositions);
                    mainViewer.UpdateScaleAndCenter();
                    mainViewer.DrawStaticShapes(pmks, JointsInfo.Data);
                    return;
                }

            }
            catch (Exception e)
            {
                status("Incomplete or incorrect data: \n" + e.InnerException);
                return;
            }
                #endregion

            try
            {
                #region Simulation of mechanism
                status("Analyzing...");
                var now = DateTime.Now;
                pmks.FindFullMovement();
                status("...done (" + (DateTime.Now - now).TotalMilliseconds.ToString() + "ms).");
                switch (pmks.CycleType)
                {
                    case CycleTypes.LessThanFullCycle:
                        status("Input cannot rotate a full 360 degrees.");
                        break;
                    case CycleTypes.OneCycle:
                        status("Mechanism is completely cyclic with input.");
                        break;
                    case CycleTypes.MoreThanOneCycle:
                        status(
                            "Input rotates a full 360 degrees but motion is not yet cyclic (more rotations are required).");
                        break;
                }
                #endregion
                #region draw curves
                status("Drawing...");
                now = DateTime.Now;
                if (pmks.LinkParameters == null || pmks.JointParameters == null || pmks.JointParameters.Count < 2)
                {
                    status("The mechanism does not move.");
                    // return;
                }
                mainViewer.UpdateRanges(pmks);
                mainViewer.FindVelocityAndAccelerationScalers(pmks);
                mainViewer.UpdateScaleAndCenter();
                mainViewer.DrawStaticShapes(pmks, JointsInfo.Data);
                mainViewer.DrawDynamicShapes(pmks, JointsInfo.Data, timeSlider);
                status("...done (" + (DateTime.Now - now).TotalMilliseconds + "ms).");
                PlayButton_Checked(null, null);
                #endregion

            }
            catch (Exception e)
            {
                status(e.Message);
            }
        }

        private void DefineLinkIDs()
        {
            LinkIDs.Clear();
            for (int i = 0; i < numJoints; i++)
                LinkIDs.Add(JointsInfo.Data[i].LinkNamesList);
        }

        private void DefineInputDriver()
        {
            var inputDriver = JointsInfo.Data.FirstOrDefault(jd => jd.DrivingInput);
            if (inputDriver == null)
                inputDriver = JointsInfo.Data.FirstOrDefault(jd => jd.CanBeDriver);
            drivingIndex = JointsInfo.Data.IndexOf(inputDriver);
        }

        private bool GroundLinkFound()
        {
            var flatListOfLinkNames = LinkIDs.SelectMany(linkID => linkID).ToList();
            int numGroundLinks = flatListOfLinkNames.Count(s => s.Equals("ground"));

            if (numGroundLinks == 0)
            {
                status("There are no links named ground. There must be at least one ground link.");
                return false;
            }
            if (numGroundLinks == 1)
                status("There is only one connection to ground. This only makes sense for the one move link.");
            return true;
        }

        private bool DuplicateLinkNames()
        {
            foreach (var linklist in LinkIDs)
                foreach (string mystring in linklist)

                    if (linklist.Count(s => s.Equals(mystring)) > 1)
                    {
                        status("No link should be referenced twice in the same joint. " + mystring.ToString());
                        return false;
                    }
            return true;
        }


        private int TrimEmptyJoints()
        {
            for (int i = 0; i < JointsInfo.Data.Count; i++)
            {
                var row = JointsInfo.Data[i];
                if (row.LinkNamesList == null || string.IsNullOrWhiteSpace(row.LinkNames)) return i;
            }
            return JointsInfo.Data.Count;
        }

        private bool SameParameters()
        {
            double angle, xpos, ypos;
            if (numJoints != JointTypes.Count) return false;
            for (int i = 0; i < numJoints; i++)
            {
                /* fixed a problem that was in this logic -- mc 10/11/2012 */
                if (Double.TryParse(JointsInfo.Data[i].XPos, out xpos) && !Constants.sameCloseZero(InitPositions[i][0], xpos)) return false;
                if (Double.TryParse(JointsInfo.Data[i].YPos, out ypos) && !Constants.sameCloseZero(InitPositions[i][1], ypos)) return false;
                if (InitPositions[i].GetLength(0) == 2 && Double.TryParse(JointsInfo.Data[i].Angle, out angle)) return false;
                if (InitPositions[i].GetLength(0) == 3 && Double.TryParse(JointsInfo.Data[i].Angle, out angle))
                    if (!Constants.sameCloseZero(DisplayConstants.RadiansToDegrees * InitPositions[i][2], angle)) return false;
            }
            return true;
        }

        private bool SameTopology()
        {
            if (numJoints != JointTypes.Count) return false;
            for (int i = 0; i < numJoints; i++)
            {
                if (i == drivingIndex && !JointsInfo.Data[i].DrivingInput) return false;
                if (JointsInfo.Data[i].TypeOfJoint != JointTypes[i]) return false;
                var newLinkIDS = JointsInfo.Data[i].LinkNamesList;
                if (i >= LinkIDs.Count) return false;
                if (newLinkIDS.GetLength(0) != LinkIDs[i].GetLength(0)) return false;
                if (newLinkIDS.Where((t, j) => t != LinkIDs[i][j]).Any())
                    return false;
            }
            return true;
        }

        public void status(string p)
        {
            outputStatus.StatusBox.Text += "\n" + p;
            outputStatus.StatusBox.SelectionStart = outputStatus.StatusBox.Text.Length;

        }

        private Boolean DefineJointTypeList()
        {
            JointTypes.Clear();
            for (int i = 0; i < numJoints; i++)
            {
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].JointTypeString)) return false;
                JointTypes.Add(JointsInfo.Data[i].TypeOfJoint);
            }
            return true;
        }

        protected override void OnKeyDown(KeyEventArgs e)
        {
            if (e.Key == Key.Ctrl || e.Key == Key.Shift) mainViewer.multiSelect = true;
            else if (e.Key == Key.PageUp || e.Key == Key.Home || e.Key == Key.Add || e.Key == Key.Insert)
                ZoomIn();
            else if (e.Key == Key.PageDown || e.Key == Key.End || e.Key == Key.Subtract || e.Key == Key.Delete)
                ZoomOut();
            else if (e.Key == Key.Up) KeyboardPan(0, -1);
            else if (e.Key == Key.Down) KeyboardPan(0, 1);
            else if (e.Key == Key.Left) KeyboardPan(-1, 0);
            else if (e.Key == Key.Right) KeyboardPan(1, 0);
            else if (e.Key == Key.Escape)
            {
                Panning = mainViewer.multiSelect = mainViewer.inTheMidstMoving = false;
            }
            base.OnKeyDown(e);
        }


        protected override void OnKeyUp(KeyEventArgs e)
        {
            if (e.Key == Key.Shift || e.Key == Key.Ctrl) mainViewer.multiSelect = false;
            base.OnKeyUp(e);
        }

        public bool Panning;
        private Point panStartReference, mousePositionWRTPage, mousePositionWRTCanvas;

        private Boolean DefinePositions()
        {
            InitPositions.Clear();
            for (int i = 0; i < numJoints; i++)
            {
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].XPos) ||
                    string.IsNullOrWhiteSpace(JointsInfo.Data[i].YPos) ||
                    string.Equals(JointsInfo.Data[i].Angle, "REQUIRED", StringComparison.InvariantCultureIgnoreCase))
                    return false;
                InitPositions.Add(string.IsNullOrWhiteSpace(JointsInfo.Data[i].Angle)
                                      ? new[] { Double.Parse(JointsInfo.Data[i].XPos), Double.Parse(JointsInfo.Data[i].YPos) }
                                      : new[]
                                          {
                                              Double.Parse(JointsInfo.Data[i].XPos),
                                              Double.Parse(JointsInfo.Data[i].YPos),
                                              Double.Parse(JointsInfo.Data[i].Angle)/DisplayConstants.RadiansToDegrees
                                          });
            }
            return true;
        }

        private bool DataListsSameLength()
        {
            var numRows = LinkIDs.Count;
            if (numRows != InitPositions.Count || numRows != JointTypes.Count) return false;
            for (int i = 0; i < numRows; i++)
            {
                var jStr = JointTypes[i];
                if (InitPositions[i].GetLength(0) != 3 && (jStr == JointType.P || jStr == JointType.RP))
                    return false;
            }
            return true;
        }

        #endregion

        internal void OnMouseWheel(object sender, MouseWheelEventArgs e)
        {
            var newScaleFactor = (e.Delta > 1)
                ? mainViewer.ScaleFactor * DisplayConstants.ZoomStep
                : mainViewer.ScaleFactor / DisplayConstants.ZoomStep;
            double delta = mainViewer.ScaleFactor - newScaleFactor;
            var oldTx = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateX;
            var oldTy = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateY;
            mainViewer.MoveScaleCanvas(newScaleFactor, delta * mousePositionWRTCanvas.X + oldTx,
                      delta * mousePositionWRTCanvas.Y + oldTy, DisplayConstants.ZoomTimeOnMouseWheel);
        }


        internal void ZoomIn()
        {
            double newScaleFactor = mainViewer.ScaleFactor * DisplayConstants.ZoomStep;
            double delta = mainViewer.ScaleFactor - newScaleFactor;
            var oldTx = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateX;
            var oldTy = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateY;
            mainViewer.MoveScaleCanvas(newScaleFactor, delta * mousePositionWRTCanvas.X + oldTx,
                      delta * mousePositionWRTCanvas.Y + oldTy, DisplayConstants.ZoomTimeOnKey);
        }

        private void KeyboardPan(int x, int y)
        {
            var oldTx = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateX;
            var oldTy = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateY;
            mainViewer.MoveScaleCanvas(mainViewer.ScaleFactor, oldTx + x * DisplayConstants.PanFactorForArrowKeys / mainViewer.ScaleFactor,
                oldTy + y * DisplayConstants.PanFactorForArrowKeys / mainViewer.ScaleFactor, DisplayConstants.PanTimeOnArrowKeys);

        }
        internal void ZoomOut()
        {
            double newScaleFactor = mainViewer.ScaleFactor / DisplayConstants.ZoomStep;
            double delta = mainViewer.ScaleFactor - newScaleFactor;
            var oldTx = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateX;
            var oldTy = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateY;
            mainViewer.MoveScaleCanvas(newScaleFactor, delta * mousePositionWRTCanvas.X + oldTx,
                      delta * mousePositionWRTCanvas.Y + oldTy, DisplayConstants.ZoomTimeOnKey);
        }

        private void ZoomInButton_OnClick(object sender, RoutedEventArgs e)
        {
            double newScaleFactor = mainViewer.ScaleFactor * DisplayConstants.ZoomStep;
            double delta = mainViewer.ScaleFactor - newScaleFactor;
            var oldTx = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateX;
            var oldTy = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateY;
            mainViewer.MoveScaleCanvas(newScaleFactor, DisplayConstants.ZoomStep * oldTx + delta * (ActualWidth / 2) / newScaleFactor,
                DisplayConstants.ZoomStep * oldTy + delta * (ActualHeight / 2) / newScaleFactor, DisplayConstants.ZoomTimeOnBtn);
        }

        private void ZoomOutButton_OnClick(object sender, RoutedEventArgs e)
        {
            double newScaleFactor = mainViewer.ScaleFactor / DisplayConstants.ZoomStep;
            double delta = mainViewer.ScaleFactor - newScaleFactor;
            var oldTx = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateX;
            var oldTy = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateY;
            mainViewer.MoveScaleCanvas(newScaleFactor, oldTx / DisplayConstants.ZoomStep + delta * (ActualWidth / 2) / newScaleFactor,
              oldTy / DisplayConstants.ZoomStep + delta * (ActualHeight / 2) / newScaleFactor, DisplayConstants.ZoomTimeOnBtn);
        }
        internal void MouseUpStopPanning(object sender, MouseEventArgs e)
        {
            Panning = false;
        }

        internal void MouseDownStartPan(object sender, MouseEventArgs e)
        {
            if (Panning) return;
            Panning = true;
            // Save starting point, used later when determining how much to scroll.
            panStartReference = e.GetPosition(this);
            var oldTx = 0.0;
            var oldTy = 0.0;
            if (mainViewer.MainCanvas.RenderTransform is CompositeTransform)
            {
                oldTx = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateX;
                oldTy = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateY;
            }
            else if (mainViewer.MainCanvas.RenderTransform is MatrixTransform)
            {
                oldTx = ((MatrixTransform)mainViewer.MainCanvas.RenderTransform).Matrix.OffsetX;
                oldTy = ((MatrixTransform)mainViewer.MainCanvas.RenderTransform).Matrix.OffsetY;
            }
            panStartReference.X -= oldTx;
            panStartReference.Y += oldTy;
        }


        internal void OnMouseMove(object sender, MouseEventArgs e)
        {
            mousePositionWRTPage = e.GetPosition(this);
            mousePositionWRTCanvas = e.GetPosition(mainViewer.MainCanvas);
            if (!Panning) return;
            mainViewer.MoveScaleCanvas(mainViewer.ScaleFactor, mousePositionWRTPage.X - panStartReference.X,
                                    panStartReference.Y - mousePositionWRTPage.Y, DisplayConstants.PanTimeOnMouseMove);
        }

        private void PlayButton_Checked(object sender, RoutedEventArgs e)
        {
            if (mainViewer.animateMechanismStoryBoard == null) return;
            SlideShape1.Opacity = SlideShape2.Opacity = 0;
            mainViewer.animateMechanismStoryBoard.Begin();
            if (pmks.CycleType == CycleTypes.OneCycle)
            {
                PlayFowardBackShape1.Opacity = PlayFowardBackShape2.Opacity = 0;
                PlayForwardShape.Opacity = 1;
            }
            else
            {
                PlayFowardBackShape1.Opacity = PlayFowardBackShape2.Opacity = 1;
                PlayForwardShape.Opacity = 0;
            }
        }

        internal void PlayButton_Unchecked(object sender, RoutedEventArgs e)
        {
            if (mainViewer.animateMechanismStoryBoard == null) return;
            mainViewer.animateMechanismStoryBoard.Stop();
            PlayFowardBackShape1.Opacity = PlayFowardBackShape2.Opacity = PlayForwardShape.Opacity = 0;
            SlideShape1.Opacity = SlideShape2.Opacity = 1;
        }

        private void timeSlider_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            PlayButton_Unchecked(sender, e);
            if (e.ClickCount == 2)
                timeSlider.Value = 0.0;
        }

        private void timeSlider_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            e.Handled = true;
            PlayButton_Unchecked(sender, e);
            if (e.Delta > 0) timeSlider.Value += timeSlider.LargeChange;
            else timeSlider.Value -= timeSlider.LargeChange;
        }

        private void RefreshButton_OnClick(object sender, RoutedEventArgs e)
        {
            Panning = mainViewer.multiSelect = mainViewer.inTheMidstMoving = false;
            mainViewer.UpdateRanges(pmks);
            mainViewer.FindVelocityAndAccelerationScalers(pmks);
            mainViewer.UpdateScaleAndCenter();
            mainViewer.DrawStaticShapes(pmks, JointsInfo.Data);
            mainViewer.DrawDynamicShapes(pmks, JointsInfo.Data, timeSlider);
        }

        void Content_Resized(object sender, EventArgs e)
        {
            mainViewer.Width = Application.Current.Host.Content.ActualWidth /
                               Application.Current.Host.Content.ZoomFactor;
            mainViewer.Height = Application.Current.Host.Content.ActualHeight /
                               Application.Current.Host.Content.ZoomFactor;
            mainViewer.UpdateScaleAndCenter();
        }

    }
}
