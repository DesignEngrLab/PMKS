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

namespace PMKS_Silverlight_App
{
    public partial class MainPage : UserControl
    {
        #region Fields
        private Simulator pmks;
        private readonly List<List<string>> LinkIDs = new List<List<string>>();
        private readonly List<string> JointTypes = new List<string>();
        private readonly List<double[]> InitPositions = new List<double[]>();
        private int numJoints;
        #endregion
        #region Properties
        public static readonly DependencyProperty SpeedProperty
            = DependencyProperty.Register("Speed", typeof(double), typeof(MainPage),
                                          new PropertyMetadata(1.0));
        public double Speed
        {
            get { return (double)GetValue(SpeedProperty); }
            set { SetValue(SpeedProperty, value); }
        }
        public static readonly DependencyProperty ErrorProperty
            = DependencyProperty.Register("Error", typeof(double), typeof(MainPage),
                                 new PropertyMetadata(1.0));
        public double Error
        {
            get { return (double)GetValue(ErrorProperty); }
            set { SetValue(ErrorProperty, value); }
        }
        public static readonly DependencyProperty AngleIncrementProperty
            = DependencyProperty.Register("AngleIncrement", typeof(double), typeof(MainPage),
                                 new PropertyMetadata(0.05));
        public double AngleIncrement
        {
            get { return (double)GetValue(AngleIncrementProperty); }
            set { SetValue(AngleIncrementProperty, value); }
        }

        public static readonly DependencyProperty AngleUnitsProperty
            = DependencyProperty.Register("AngleUnits", typeof(AngleType), typeof(MainPage),
                                 new PropertyMetadata(AngleType.Degrees));
        public AngleType AngleUnits
        {
            get { return (AngleType)GetValue(AngleUnitsProperty); }
            set { SetValue(AngleUnitsProperty, value); }
        }

        public static readonly DependencyProperty LengthUnitsProperty
            = DependencyProperty.Register("LengthUnits", typeof(LengthType), typeof(MainPage),
                                 new PropertyMetadata(LengthType.mm));
        public LengthType LengthUnits
        {
            get { return (LengthType)GetValue(LengthUnitsProperty); }
            set { SetValue(LengthUnitsProperty, value); }
        }

        #endregion

        public MainPage()
        {
            InitializeComponent();
            jointInputTable.main = editButtons.main = linkInputTable.main = this;
            //jointInputTable.main = editButtons.main = linkInputTable.main = timeSlider.main = this;
            
        }

        private void MainPage_Loaded_1(object sender, RoutedEventArgs e)
        {
            var binding = new Binding
            {
                Source = JointsInfo,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(JointsViewModel.DataCollectionProperty)
            };
            jointInputTable.dataGrid.SetBinding(DataGrid.ItemsSourceProperty, binding);

            binding = new Binding
            {
                Source = LinksInfo,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(LinksViewModel.DataCollectionProperty)
            };
            linkInputTable.linkDataGrid.SetBinding(DataGrid.ItemsSourceProperty, binding);

            //binding = new Binding
            //{
            //    Source = JointsInfo,
            //    Mode = BindingMode.OneWay,
            //    Path = new PropertyPath(LinksViewModel.DataCollectionProperty),
            //    Converter = new JointDataToLinkListConverter(LinksInfo)
            //};
            //BindingOperations.SetBinding(LinksInfo, LinksViewModel.DataCollectionProperty, binding);

            binding = new Binding
            {
                Source = this,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(MainPage.SpeedProperty),
                Converter = new TextToDoubleConverter()
            };
            globalSettings.speedBox.SetBinding(TextBox.TextProperty, binding);

            binding = new Binding
            {
                Source = this,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(MainPage.AngleIncrementProperty),
                Converter = new TextToDoubleConverter()
            };
            globalSettings.AngleErrorBox.SetBinding(TextBox.TextProperty, binding);

            binding = new Binding
            {
                Source = this,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(MainPage.AngleUnitsProperty),
                Converter = new BooleanToAngleTypeConverter()
            };
            globalSettings.RadiansCheckBox.SetBinding(ToggleButton.IsCheckedProperty, binding);

            binding = new Binding
            {
                Source = this,
                Mode = BindingMode.TwoWay,
                Path = new PropertyPath(MainPage.LengthUnitsProperty),
                Converter = new BooleanToLengthTypeConverter()
            };
            globalSettings.MetricCheckBox.SetBinding(ToggleButton.IsCheckedProperty, binding);
            ParseData();
        }
        #region PMKS Controller Functions
        internal void ParseData()
        {
            #region table validation
            if (JointsInfo == null) return;
            numJoints = TrimEmptyJoints();
            if (pmks != null && SameTopology() && SameParameters() && SameGlobalSettings() && SameRadioSettings())
                return;


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
            mainViewer.UpdateRangeScaleAndCenter(InitPositions);
            mainViewer.DrawStaticShapes(LinkIDs, JointTypes, InitPositions);
            #endregion

            #region Setting Up PMKS
            try
            {
                pmks = new Simulator(LinkIDs, JointTypes, InitPositions);

                if (pmks.IsDyadic) status("The mechanism is comprised of only of dyads.");
                else status("The mechanism has non-dyadic loops.");
                int dof = pmks.DegreesOfFreedom;
                status("Degrees of freedom = " + dof);
                if (dof == 1)
                {
                    pmks.InputSpeed = Speed;
                    if (globalSettings.ErrorCheckBox.IsChecked != null && (bool)globalSettings.ErrorCheckBox.IsChecked)
                        pmks.MaxSmoothingError = AngleIncrement;
                    else
                        pmks.DeltaAngle = AngleIncrement;
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
                mainViewer.UpdateRangeScaleAndCenter(pmks);
                mainViewer.DrawStaticShapes(LinkIDs, JointTypes, InitPositions);
                mainViewer.UpdateVisuals(pmks, JointsInfo.Data, timeSlider);
                status("...done (" + (DateTime.Now - now).TotalMilliseconds.ToString() + "ms).");
                #endregion
            }
            catch (Exception e)
            {
                status(e.Message);
            }
        }

        private bool SameRadioSettings()
        {
            if (pmks == null)
            {
                return false;
            }
            if (pmks.DeltaAngle == AngleIncrement && (globalSettings.ErrorCheckBox.IsChecked != null && (bool)globalSettings.ErrorCheckBox.IsChecked))
            {
                return false;
            }
            if (pmks.MaxSmoothingError == AngleIncrement && (globalSettings.AngleCheckBox.IsChecked != null && (bool)globalSettings.AngleCheckBox.IsChecked))
            {
                return false;
            }
            return true;
        }

        private bool SameGlobalSettings()
        {
            if (pmks == null)
            {
                return false;
            }
            if (pmks.MaxSmoothingError == AngleIncrement || pmks.DeltaAngle == AngleIncrement)
            {
                return true;
            }
            return false;

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

        private void status(string p)
        {
            outputStatus.StatusBox.Text += "\n" + p;
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

        private Boolean DefineLinkIDS()
        {
            LinkIDs.Clear();
            for (int i = 0; i < numJoints; i++)
            {
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].LinkNames)) return false;
                var linkNames = new List<string>(JointsInfo.Data[i].LinkNames.Split(new[] { ',', ' ' },
                    StringSplitOptions.RemoveEmptyEntries));
                if (linkNames.Count == 0) return false;
                LinkIDs.Add(linkNames);
            }
            return true;
        }

        private Boolean DefinePositions()
        {
            InitPositions.Clear();
            for (int i = 0; i < numJoints; i++)
            {
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].XPos) ||
                    string.IsNullOrWhiteSpace(JointsInfo.Data[i].YPos))
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
                {
                    jointInputTable.HighlightMissingAngle(i);
                    return false;
                }
            }
            return true;
        }

        #endregion

        private void OnMouseWheel(object sender, MouseWheelEventArgs e)
        {
            mainViewer.OnMouseWheel(sender, e);
        }

        private void OnLostMouseCapture(object sender, MouseEventArgs e)
        {
            mainViewer.OnLostMouseCapture(sender, e);
        }

        private void OnMouseEnter(object sender, MouseEventArgs e)
        {
            mainViewer.OnMouseEnter(sender, e);
        }

        private void OnMouseMove(object sender, MouseEventArgs e)
        {
            mainViewer.OnMouseMove(sender, e);
        }
    }
}
