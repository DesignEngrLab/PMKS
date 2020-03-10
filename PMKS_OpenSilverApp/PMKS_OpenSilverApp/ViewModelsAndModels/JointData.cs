using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Dynamic;
using System.Globalization;
using System.Linq;
using System.Text.RegularExpressions;
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
                _linkNames = value.ToLower();
                _linkNames = _linkNames.Replace("gnd", "ground");
                _linkNames = _linkNames.Replace("grnd", "ground");
                _linkNames = _linkNames.Replace("grond", "ground");
                _linkNames = _linkNames.Replace("gound", "ground");
                _linkNames = _linkNames.Replace("groud", "ground");
                if (Regex.Match(_linkNames[0].ToString(), @"[0,1]").Success
                    && Regex.Match(_linkNames[1].ToString(), @"[^a-z,^0-9]").Success)
                {
                    _linkNames = _linkNames.Remove(0, 1);
                    _linkNames = "ground" + _linkNames;
                }
                var lastIndex = _linkNames.Length - 1;
                if (Regex.Match(_linkNames[lastIndex].ToString(), @"[0,1]").Success
                    && Regex.Match(_linkNames[lastIndex - 1].ToString(), @"[^a-z,^0-9]").Success)
                {
                    _linkNames = _linkNames.Remove(lastIndex);
                    _linkNames += "ground";
                }
                _linkNames = Regex.Replace(_linkNames, @"[^a-z,^0-9][0,1][^a-z,^0-9]", " ground ");
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
                return _linkNames.Split(new[] { ',', ' ' }, StringSplitOptions.RemoveEmptyEntries);
                //var tempList = _linkNames.Split(new [] { ',', ' ' }, StringSplitOptions.RemoveEmptyEntries);
                //for (int i = 0; i < tempList.Length; i++)
                //{
                //    var s = tempList[i];
                //    if (s.Equals("0") || s.Equals("1")) tempList[i] = "ground";
                //}
                //return tempList;
            }
        }

        public string XPos
        {
            get { return (double.IsNaN(X)) ? "" : X.ToString("F3", CultureInfo.InvariantCulture); }
            set
            {
                if (!double.TryParse(value, out X))
                    X = GetRandomNewXCoord(Y);
            }
        }

        public string YPos
        {
            get { return (double.IsNaN(Y)) ? "" : Y.ToString("F3", CultureInfo.InvariantCulture); }
            set
            {
                if (!double.TryParse(value, out Y))
                    Y = GetRandomNewYCoord(X);
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
        public double CanPlotStateVars
        {
            get
            {
                if (LinkNames.Contains("ground")) return 0.0;
                else return 1.0;
            }
        }

        internal static string ConvertDataToText(char jointSepChar)
        {
            var nameTrimChars = new[] { ' ', ',', ';', ':', '.', '|' };
            var text = "";
            foreach (var jInfo in App.main.JointsInfo.Data)
            {
                if (string.IsNullOrWhiteSpace(jInfo.LinkNames) || (string.IsNullOrWhiteSpace(jInfo.JointTypeString)))
                    continue;
                var linkNames = jInfo.LinkNames;
                var linkNamesList = linkNames.Split(nameTrimChars);
                linkNames = "";
                foreach (var name in linkNamesList)
                {
                    var trimmedName = name.Trim(nameTrimChars);
                    if (!string.IsNullOrWhiteSpace(trimmedName))
                        linkNames += name.Trim(nameTrimChars) + ",";
                }
                text += linkNames;
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



        private static double xRangeCenter, yRangeCenter, radiusRange;
        private static Random random = new Random();
        private static double GetRandomNewXCoord(double yCoord = double.NaN)
        {
            if (double.IsNaN(yCoord))
                return xRangeCenter + random.NextDouble() * radiusRange;
            else
            {
                var angle = Math.Asin((yCoord - yRangeCenter) / radiusRange);
                if (random.NextDouble() > 0.5)
                    return xRangeCenter + radiusRange * Math.Cos(angle);
                else return xRangeCenter - radiusRange * Math.Cos(angle);
            }
        }
        private static double GetRandomNewYCoord(double xCoord = double.NaN)
        {
            if (double.IsNaN(xCoord))
                return yRangeCenter + random.NextDouble() * radiusRange;
            else
            {
                var angle = Math.Acos((xCoord - xRangeCenter) / radiusRange);
                if (random.NextDouble() > 0.5)
                    return yRangeCenter + radiusRange * Math.Sin(angle);
                else return yRangeCenter - radiusRange * Math.Sin(angle);
            }
        }

        internal static void UpdateRandomRange(List<double[]> initPositions)
        {
            var xMin = initPositions.Min(coord => coord[0]);
            var xMax = initPositions.Max(coord => coord[0]);
            var yMin = initPositions.Min(coord => coord[1]);
            var yMax = initPositions.Max(coord => coord[1]);
            xRangeCenter = (xMax + xMin) / 2;
            yRangeCenter = (yMax + yMin) / 2;
            radiusRange = Math.Max((xMax - xMin), (yMax - yMin)) / 2;
        }
    }
}

