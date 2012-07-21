using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using PlanarMechanismSimulator;
using Silverlight_PMKS;

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
            jointInputTable.main = editButtons.main = this;
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

            binding = new Binding
            {
                Source = JointsInfo,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(LinksViewModel.DataCollectionProperty),
                Converter = new JointDataToLinkListConverter(LinksInfo)
            };
            BindingOperations.SetBinding(LinksInfo, LinksViewModel.DataCollectionProperty, binding);

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
        }
        #region PMKS Controller Functions
        internal void ParseData()
        {
            if (JointsInfo == null) return;
            numJoints = TrimEmptyJoints();
            if (SameTopology() && SameParameters())
            {
                if (pmks != null)
                    mainViewer.UpdateVisuals(pmks.JointParameters, pmks.LinkParameters, pmks.inputJointIndex);
                return;
            }

            if (SameTopology())
            {
                DefinePositions();
                pmks.AssignPositions(InitPositions);
            }
            else if (!(DefineLinkIDS() && DefinePositions() && DefineJointTypeList())) return;
            //try
            //{
            pmks = new Simulator(LinkIDs, JointTypes, InitPositions);

            if (pmks.IsDyadic) status("The mechanism is comprised of only of dyads.");
            else status("The mechanism has non-dyadic loops.");
            int dof = pmks.DegreesOfFreedom;
            status("Degrees of freedom = " + dof);
            if (dof == 1)
            {
                pmks.DeltaAngle = AngleIncrement;
                pmks.InputSpeed = Speed;
                status("Analyzing...");
                var now = DateTime.Now;
                pmks.FindFullMovement();
                status("...done (" + (DateTime.Now - now).TotalMilliseconds.ToString() + "ms).");
                status("Drawing...");
                now = DateTime.Now;
                mainViewer.UpdateVisuals(pmks.JointParameters, pmks.LinkParameters, pmks.inputJointIndex);
                status("...done (" + (DateTime.Now - now).TotalMilliseconds.ToString() + "ms).");
            }
            //}
            //catch (Exception e)
            //{
            //    status(e.Message);
            //}
        }

        private int TrimEmptyJoints()
        {
            for (int i = 0; i < JointsInfo.Data.Count; i++)
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].JointType)) return i;
            return JointsInfo.Data.Count;
        }

        private bool SameParameters()
        {
            double angle;
            if (numJoints != JointTypes.Count) return false;
            for (int i = 0; i < numJoints; i++)
            {
                if (Math.Abs(InitPositions[i][0] - Double.Parse(JointsInfo.Data[i].XPos)) > Constants.epsilon) return false;
                if (Math.Abs(InitPositions[i][1] - Double.Parse(JointsInfo.Data[i].YPos)) > Constants.epsilon) return false;
                if (InitPositions[i].GetLength(0) == 2 && Double.TryParse(JointsInfo.Data[i].Angle, out angle)) return false;
                if (InitPositions[i].GetLength(0) == 3 && Double.TryParse(JointsInfo.Data[i].Angle, out angle))
                    if (Math.Abs(InitPositions[i][2] - angle) > Constants.epsilon) return false;
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
                if (LinkIDs[i].Any(linkID => !newLinkIDS.Remove(linkID)))
                    return false;
                if (newLinkIDS.Count > 0) return false;
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
            double angle;
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


        #endregion

    }
}
