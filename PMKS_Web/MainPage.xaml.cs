using System.Windows.Media;
using System.Windows.Shapes;
using PlanarMechanismSimulator;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Input;
using Silverlight_PMKS;

namespace PMKS_Silverlight_App
{
    public partial class MainPage : UserControl
    {
        #region Fields
        public Simulator pmks;
        public readonly List<List<string>> LinkIDs = new List<List<string>>();
        public readonly List<string> JointTypes = new List<string>();
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
                                          new PropertyMetadata(1.0, GlobalSettingChanged));

        public double Speed
        {
            get { return (double)GetValue(SpeedProperty); }
            set { SetValue(SpeedProperty, value); }
        }
        public static readonly DependencyProperty ErrorProperty
            = DependencyProperty.Register("Error", typeof(double), typeof(MainPage),
                                 new PropertyMetadata(0.001, GlobalSettingChanged));
        public double Error
        {
            get { return (double)GetValue(ErrorProperty); }
            set { SetValue(ErrorProperty, value); }
        }
        public static readonly DependencyProperty AngleIncrementProperty
            = DependencyProperty.Register("AngleIncrement", typeof(double), typeof(MainPage),
                                 new PropertyMetadata(0.0087266462599716477, GlobalSettingChanged));
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
            JointsInfo = new JointsViewModel();
            LinksInfo = new LinksViewModel();
            InitializeComponent();
            InputJointBaseShape.LoadShapes();
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
                Converter = new BooleanToLengthTypeConverter()
            };
            globalSettings.ErrorCheckBox.SetBinding(ToggleButton.IsCheckedProperty, binding);
            ParseData();
        }

        #region PMKS Controller Functions

     

        internal void ParseData(Boolean ForceRerunOfSimulation = false)
        {
#if trycatch
            try
            {
#endif
                #region table validation
                if (JointsInfo == null) return;
                numJoints = TrimEmptyJoints();
                if (pmks != null && !ForceRerunOfSimulation && SameTopology() && SameParameters()) return;
                DefineInputDriver();

                if (pmks != null && SameTopology() && DataListsSameLength() && drivingIndex==pmks.DrivingIndex)
                {
                    DefinePositions();
                    pmks.AssignPositions(InitPositions);
                }
                else
                {
                    if (!validLinks()) return;

                    if (!(DefineLinkIDS() && DefinePositions() && DefineJointTypeList() && DataListsSameLength())) return;
                    pmks = new Simulator(LinkIDs, JointTypes, drivingIndex, InitPositions);
                }
                mainViewer.ClearDynamicShapesAndBindings();
                PlayButton_Unchecked(null, null);

                if (pmks.IsDyadic) status("The mechanism is comprised of only of dyads.");
                else status("The mechanism has non-dyadic loops.");
                int dof = pmks.DegreesOfFreedom;
                App.main.fileAndEditPanel.ReportDOF(dof);
                status("Degrees of freedom = " + dof);
                if (dof == 1)
                {
                    pmks.InputSpeed = Speed;
                    if (globalSettings.ErrorCheckBox.IsChecked != null && (bool)globalSettings.ErrorCheckBox.IsChecked)
                        pmks.MaxSmoothingError = Error;
                    else
                        pmks.DeltaAngle = AngleIncrement;
                }
                else
                {
                    mainViewer.UpdateRanges(InitPositions);
                    mainViewer.UpdateScaleAndCenter();
                    mainViewer.DrawStaticShapes(pmks, JointsInfo.Data);
                    return;
                }
#if trycatch
            }
            catch (Exception e)
            {
                status("Incomplete or incorrect data: \n" + e.InnerException);
                return;
            }
#endif
                #endregion
#if trycatch
            try
            {
#endif
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
                    status("...nothing to draw. Error in simulation.");
                    return;
                }
                mainViewer.UpdateRanges(pmks);
                mainViewer.FindVelocityAndAccelerationScalers(pmks);
                mainViewer.UpdateScaleAndCenter();
                mainViewer.DrawStaticShapes(pmks, JointsInfo.Data);
                mainViewer.DrawDynamicShapes(pmks, JointsInfo.Data, timeSlider);
                status("...done (" + (DateTime.Now - now).TotalMilliseconds + "ms).");
                PlayButton_Checked(null, null);
                #endregion
#if trycatch
            }
            catch (Exception e)
            {
                status(e.Message);
            }
#endif
        }

        private void DefineInputDriver()
        {
            var inputDriver = JointsInfo.Data.FirstOrDefault(jd => jd.DrivingInput);
            if (inputDriver == null)
                inputDriver = JointsInfo.Data.FirstOrDefault(jd => jd.CanBeDriver);
            drivingIndex = JointsInfo.Data.IndexOf(inputDriver);
        }

        private bool validLinks()
        {
            List<string> flatList = new List<string>();
            if (!DefineLinkIDS())
            {
                return true;
            }
            //generates a flat list of strings
            foreach (List<string> linklist in LinkIDs)
            {
                foreach (string s in linklist)
                {
                    flatList.Add(s);
                }
            }
            foreach (string s in flatList)
            {
                int count = 0;
                for (int i = 0; i < flatList.Count; i++)
                {
                    if (s.Equals(flatList.ElementAt(i)))
                    {
                        count++;
                    }
                }
                if (count < 2)
                {
                    //status("Only one Link named " + s.ToString());
                    //return false;
                    //this should not return false if it is simply one connected to ground.
                }
            }
            int groundlinks = 0;
            foreach (string s in flatList)
            {
                if (s.ToLower().Equals("0") || s.ToLower().Equals("grnd") || s.ToLower().Equals("ground") || s.ToLower().Equals("gnd") || s.ToLower().Equals("grd") || s.ToLower().Equals("zero"))
                {
                    groundlinks++;
                }
            }
            if (groundlinks == 0)
            {
                status("There are no links named ground. There must be at least one ground link.");
                return false;
            }


            foreach (List<string> linklist in LinkIDs)
            {
                foreach (string mystring in linklist)
                {
                    int stringcount = 0;
                    foreach (string s in linklist)
                    {
                        if (mystring.Equals(s))
                        {
                            stringcount++;
                        }
                    }
                    if (stringcount > 1)
                    {
                        status("No link should be referenced twice in the same joint. " + mystring.ToString());
                    }
                }
            }

            return true;
        }


        private int TrimEmptyJoints()
        {
            for (int i = 0; i < JointsInfo.Data.Count; i++)
            {
                var row = JointsInfo.Data[i];
                double dummy;
                if (row.LinkNamesList.Count() == 1 && double.TryParse(row.XPos, out dummy) && double.TryParse(row.YPos, out dummy))
                    row.JointType = "R (pin joint)";
                if (string.IsNullOrWhiteSpace(row.JointType)) return i;
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
                    if (!Constants.sameCloseZero(InitPositions[i][2], angle)) return false;
            }
            return true;
        }

        private bool SameTopology()
        {
            if (numJoints != JointTypes.Count) return false;
            for (int i = 0; i < numJoints; i++)
            {
                if (i == drivingIndex && !JointsInfo.Data[i].DrivingInput) return false;
                if (JointsInfo.Data[i].JointType != JointTypes[i]) return false;
                var newLinkIDS = new List<string>(JointsInfo.Data[i].LinkNames.Split(new[] { ',', ' ' },
                    StringSplitOptions.RemoveEmptyEntries));
                if (i >= LinkIDs.Count) return false;
                if (newLinkIDS.Count != LinkIDs[i].Count) return false;
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
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].JointType)) return false;
                JointTypes.Add(JointsInfo.Data[i].JointType);
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


        private List<string> distinctLinkNames;
        public bool Panning;
        private Point panStartReference, mousePositionWRTPage, mousePositionWRTCanvas;

        private Boolean DefineLinkIDS()
        {
            distinctLinkNames = new List<string>();
            LinkIDs.Clear();
            for (int i = 0; i < numJoints; i++)
            {
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].LinkNames)) return false;
                var linkNames = new List<string>(JointsInfo.Data[i].LinkNames.Split(new[] { ',', ' ' },
                    StringSplitOptions.RemoveEmptyEntries));
                if (linkNames.Count == 0)
                    throw new Exception("it doesn't seem you should ever be able to get here");
                //return false;
                LinkIDs.Add(linkNames);
                distinctLinkNames.AddRange(linkNames);
            }
            distinctLinkNames = distinctLinkNames.Distinct().ToList();
            return true;
        }

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
                                              Double.Parse(JointsInfo.Data[i].Angle)
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
                if (InitPositions[i].GetLength(0) != 3 &&
                    (jStr.Split(',', ' ')[0].Equals("p", StringComparison.InvariantCultureIgnoreCase) ||
                    jStr.Split(',', ' ')[0].Equals("rp", StringComparison.InvariantCultureIgnoreCase)))
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
                                    panStartReference.Y - mousePositionWRTPage.Y, DisplayConstants.ZoomTimeOnPan);
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

        private void UserControl_SizeChanged(object sender, SizeChangedEventArgs e)
        {

            mainViewer.Width = Application.Current.Host.Content.ActualWidth;
            mainViewer.Height = Application.Current.Host.Content.ActualHeight;
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

    }
}
