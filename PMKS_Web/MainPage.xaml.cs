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
        private readonly List<List<string>> LinkIDs = new List<List<string>>();
        private readonly List<string> JointTypes = new List<string>();
        private readonly List<double[]> InitPositions = new List<double[]>();
        private int numJoints;
        private int drivingIndex;
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
                                 new PropertyMetadata(0.0001, GlobalSettingChanged));
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
            fileAndEditPanel.dataGrid.SetBinding(DataGrid.ItemsSourceProperty, binding);

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
        internal void ParseData(Boolean SettingChanged = false)
        {
            #region table validation
            if (JointsInfo == null) return;
            numJoints = TrimEmptyJoints();
            if (pmks != null && !SettingChanged && SameTopology() && SameParameters()) return;


            if (pmks != null && SameTopology() && DataListsSameLength())
            {
                DefinePositions();
                pmks.AssignPositions(InitPositions);
            }
            if (!validLinks()) return;

            if (!(DefineLinkIDS() && DefinePositions() && DefineJointTypeList() && DataListsSameLength())) return;
            #endregion

            #region Just Draw Axes and Joints
            mainViewer.Clear();
            mainViewer.UpdateRanges(InitPositions);
            mainViewer.UpdateScaleAndCenter();
            mainViewer.DrawStaticShapes(LinkIDs, JointTypes, InitPositions, distinctLinkNames, true);
            #endregion

            #region Setting Up PMKS
            try
            {
                PlayButton_Unchecked(null, null);
                DefineInputDriver();
                pmks = new Simulator(LinkIDs, JointTypes, drivingIndex, InitPositions);

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
                else return;
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
                if (pmks.AdditionalGearCycling)
                    status(pmks.CompleteCycle
                               ? "Input rotates a full 360 degrees but motion is not yet cyclic (more rotations are required)."
                               : "Input cannot rotate a full 360 degrees.");
                else
                    status(pmks.CompleteCycle
                               ? "Completes a full cycle and repeats"
                               : "Input cannot rotate a full 360 degrees.");
                #endregion
                #region draw curves
                status("Drawing...");
                now = DateTime.Now;
                if (pmks.LinkParameters == null || pmks.JointParameters == null || pmks.JointParameters.Count < 2)
                {
                    status("...nothing to draw. Error in simulation.");
                    return;
                }
                mainViewer.Clear();
                mainViewer.UpdateRanges(pmks);
                mainViewer.FindVelocityAndAccelerationScalers(pmks);
                mainViewer.UpdateScaleAndCenter();
                mainViewer.DrawStaticShapes(LinkIDs, JointTypes, InitPositions, distinctLinkNames, false);
                mainViewer.DrawDynamicShapes(pmks, JointsInfo.Data, timeSlider);
                status("...done (" + (DateTime.Now - now).TotalMilliseconds.ToString() + "ms).");
                PlayButton_Checked(null, null);
                #endregion
            }
            catch (Exception e)
            {
                status(e.Message);
            }
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

        private List<string> distinctLinkNames;
        private bool Panning;
        private Point ScreenStartPoint;
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
            double newScaleFactor = e.Delta > 1
                ? mainViewer.ScaleFactor * 1.05
                : mainViewer.ScaleFactor / 1.05;

            double delta = mainViewer.ScaleFactor - newScaleFactor;
            ScreenStartPoint = e.GetPosition(mainViewer.MainCanvas);
             var oldTx = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateX;
            var oldTy = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateY;
            mainViewer.MoveScaleCanvas(newScaleFactor, new Point(delta * ScreenStartPoint.X +oldTx,
                      delta * ScreenStartPoint.Y+oldTy));
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
            ScreenStartPoint = e.GetPosition(this);
            var oldTx = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateX;
            var oldTy = ((CompositeTransform)mainViewer.MainCanvas.RenderTransform).TranslateY;
            ScreenStartPoint.X -= oldTx;
            ScreenStartPoint.Y -= oldTy;
        }


        internal void OnMouseMove(object sender, MouseEventArgs e)
        {
            if (!Panning) return;
            var newPanAnchor = new Point(e.GetPosition(this).X - ScreenStartPoint.X,
                                    ScreenStartPoint.Y - e.GetPosition(this).Y);
            mainViewer.MoveScaleCanvas(mainViewer.ScaleFactor, newPanAnchor);
        }

        private void PlayButton_Checked(object sender, RoutedEventArgs e)
        {
            if (mainViewer.storyBoard == null) return;
            SlideShape1.Opacity = SlideShape2.Opacity = 0;
            mainViewer.storyBoard.Begin();
            if (pmks.CompleteCycle)
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

        private void PlayButton_Unchecked(object sender, RoutedEventArgs e)
        {
            if (mainViewer.storyBoard == null) return;
            mainViewer.storyBoard.Stop();
            PlayFowardBackShape1.Opacity = PlayFowardBackShape2.Opacity = PlayForwardShape.Opacity = 0;
            SlideShape1.Opacity = SlideShape2.Opacity = 1;
        }

        private void timeSlider_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            PlayButton_Unchecked(sender, e);
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

    }
}
