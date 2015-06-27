using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Globalization;
using System.Linq;
using System.Windows;
using PMKS;
using Silverlight_PMKS;

namespace PMKS_Silverlight_App
{
    public class JointData : DependencyObject, INotifyPropertyChanged
    {

        public double X = double.NaN;
        public double Y = double.NaN;
        public double AngleDegrees = double.NaN;
        public JointType TypeOfJoint;
        private string _linkNames;

        public string LinkNames
        {
            get { return _linkNames; }
            set
            {
                _linkNames = value;
                _linkNames = _linkNames.Replace("gnd", "ground");
                _linkNames = _linkNames.Replace("grnd", "ground");
                _linkNames = _linkNames.Replace("grond", "ground");
                _linkNames = _linkNames.Replace("gound", "ground");
                _linkNames = _linkNames.Replace("groud", "ground");
                _linkNames = _linkNames.Replace("0", "ground");
            }
        }

        public string JointTypeString
        {
            get
            {
                if (TypeOfJoint == JointType.unspecified) return "";
                return TypeOfJoint.ToString();
            }
            set
            {
                string jointTypeString;
                if (string.IsNullOrWhiteSpace(value)) jointTypeString = "";
                else jointTypeString = value.Split(',', ' ')[0];
                Enum.TryParse(jointTypeString, true, out TypeOfJoint);
            }
        }

        public string[] LinkNamesList
        {
            set
            {
                var tempList = "";
                foreach (var s in value)
                {
                    tempList += s;
                    tempList += ",";
                }
                _linkNames = tempList.Remove(tempList.Length - 1);
                if (Application.Current.RootVisual != null)
                    App.main.linkInputTable.UpdateLinksTable();
            }
            get
            {
                if (_linkNames == null || string.IsNullOrWhiteSpace(_linkNames)) return new string[0];
                return _linkNames.Split(new char[] { ',', ' ' }, StringSplitOptions.RemoveEmptyEntries);
            }
        }

        public string XPos
        {
            get { return (double.IsNaN(X)) ? "" : X.ToString("F3", CultureInfo.InvariantCulture); }
            set
            {
                if (!double.TryParse(value, out X))
                    X = double.NaN;
            }
        }

        public string YPos
        {
            get { return (double.IsNaN(Y)) ? "" : Y.ToString("F3", CultureInfo.InvariantCulture); }
            set
            {
                if (!double.TryParse(value, out Y))
                    Y = double.NaN;
            }
        }
        // Declare the PropertyChanged event
        public event PropertyChangedEventHandler PropertyChanged;

        // OnPropertyChanged will raise the PropertyChanged event passing the
        // source property that is being updated.
        private void onPropertyChanged(object sender, string propertyName)
        {
            if (this.PropertyChanged != null)
            {
                PropertyChanged(sender, new PropertyChangedEventArgs(propertyName));
            }
        }

        public string Angle
        {
            get
            {
                if (double.IsNaN(AngleDegrees))
                {
                    if (TypeOfJoint == JointType.R || TypeOfJoint == JointType.G || TypeOfJoint == JointType.unspecified)
                        return "";
                    return "required";
                }
                return AngleDegrees.ToString("F3", CultureInfo.InvariantCulture);
            }
            set
            {
                if (!double.TryParse(value, out AngleDegrees))
                {
                    AngleDegrees = double.NaN;
                }
                while (AngleDegrees > 90) AngleDegrees -= 180.0;
                while (AngleDegrees < -90) AngleDegrees += 180.0;
            }
        }


        public Boolean PosVisible
        {
            get { return (Boolean)GetValue(PosVisibleProperty); }
            set { SetValue(PosVisibleProperty, value); }
        }

        public static readonly DependencyProperty PosVisibleProperty
            = DependencyProperty.Register("PosVisible",
                                          typeof(Boolean), typeof(JointData),
                                          new PropertyMetadata(true));

        public Boolean VelocityVisible
        {
            get { return (Boolean)GetValue(VelVisibleProperty); }
            set { SetValue(VelVisibleProperty, value); }
        }

        public static readonly DependencyProperty VelVisibleProperty = DependencyProperty.Register("VelVisible",
                                          typeof(Boolean), typeof(JointData),
                                          new PropertyMetadata(false));

        public Boolean AccelerationVisible
        {
            get { return (Boolean)GetValue(AccelVisibleProperty); }
            set { SetValue(AccelVisibleProperty, value); }
        }

        public static readonly DependencyProperty AccelVisibleProperty = DependencyProperty.Register("AccelVisible",
                                          typeof(Boolean), typeof(JointData),
                                          new PropertyMetadata(false));
        public Boolean DrivingInput
        {
            get { return (Boolean)GetValue(DrivingInputProperty); }
            set { SetValue(DrivingInputProperty, value); }
        }

        public static readonly DependencyProperty DrivingInputProperty = DependencyProperty.Register("DrivingInput",
                                          typeof(Boolean), typeof(JointData),
                                          new PropertyMetadata(false));


        public Boolean CanBeDriver
        {
            get
            {
                return ((TypeOfJoint == JointType.R || TypeOfJoint == JointType.P)
                    && LinkNamesList != null && LinkNamesList.Count() > 1);
            }
        }

        internal static string ConvertDataToText(char jointSepChar)
        {
            var text = "";
            foreach (var jInfo in App.main.JointsInfo.Data)
            {
                if (string.IsNullOrWhiteSpace(jInfo.LinkNames) || (string.IsNullOrWhiteSpace(jInfo.JointTypeString)))
                    continue;
                text += jInfo.LinkNames + ",";
                text += jInfo.JointTypeString + ",";
                text += jInfo.XPos + ",";
                text += jInfo.YPos;
                text += (!string.IsNullOrWhiteSpace(jInfo.Angle)) ? "," + jInfo.Angle : "";
                var boolStr = ","
                              + (jInfo.PosVisible ? 't' : 'f')
                              + (jInfo.VelocityVisible ? 't' : 'f')
                              + (jInfo.AccelerationVisible ? 't' : 'f')
                              + (jInfo.DrivingInput ? 't' : 'f');
                text += boolStr + jointSepChar;
            }
            return text;
        }


        public void RefreshTablePositions()
        {
            //if (App.main != null)
            //    App.main.fileAndEditPanel.dataGrid.InvalidateMeasure();      
            onPropertyChanged(this, "XPos");
            onPropertyChanged(this, "YPos");
            onPropertyChanged(this, "Angle");
        }
    }
}

