using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;

namespace PMKS_Silverlight_App
{
    public class JointData
    {
        private bool _posVisible = true;
        private bool _velocityVisible = true;
        private bool _accelerationVisible = true;
        private double _xPos = double.NaN;
        private double _yPos = double.NaN;
        private double _angle = double.NaN;
        private string _jointType;

        public string JointType
        {
            get { return _jointType; }
            set
            {
                if (string.IsNullOrWhiteSpace(value)) _jointType = "";
                else _jointType = value.Split(',', ' ')[0];
            }
        }

        public string LinkNames { get; set; }

        public string XPos
        {
            get { return (double.IsNaN(_xPos)) ? "" : _xPos.ToString(); }
            set
            {
                if (!double.TryParse(value, out _xPos))
                    _xPos = double.NaN;
            }
        }

        public string YPos
        {
            get { return (double.IsNaN(_yPos)) ? "" : _yPos.ToString("N"); }
            set
            {
                if (!double.TryParse(value, out _yPos))
                    _yPos = double.NaN;
            }
        }

        public string Angle
         {
            get { return (double.IsNaN(_angle)) ? "" : _angle.ToString("N"); }
            set
            {
                if (!double.TryParse(value, out _angle))
                    _angle = double.NaN;
            }
        }

        public Boolean PosVisible
        {
            get { return _posVisible; }
            set
            {
                _posVisible = value;
            }
        }

        public Boolean VelocityVisible
        {
            get { return _velocityVisible; }
            set
            {
                _velocityVisible = value;
            }
        }

        public Boolean AccelerationVisible
        {
            get { return _accelerationVisible; }
            set
            {
                _accelerationVisible = value;
            }
        }

        internal static bool ConvertTextToData(string text, out List<JointData> jointsInfo)
        {
            jointsInfo = new List<JointData>();
            var pivotSentences = text.Split('\n').ToList();
            pivotSentences.RemoveAll(string.IsNullOrWhiteSpace);
            if (pivotSentences.Count == 0) return false;
            foreach (var pivotSentence in pivotSentences)
            {
                var words = pivotSentence.Split(',', ' ').ToList();
                words.RemoveAll(string.IsNullOrWhiteSpace);
                var lastJointType = words.LastOrDefault(s => s.Equals("R", StringComparison.InvariantCultureIgnoreCase)
                                                             ||
                                                             s.Equals("P", StringComparison.InvariantCultureIgnoreCase)
                                                             ||
                                                             s.Equals("RP", StringComparison.InvariantCultureIgnoreCase)
                                                             ||
                                                             s.Equals("G", StringComparison.InvariantCultureIgnoreCase));
                var jointTypeIndex = words.LastIndexOf(lastJointType);
                var index = jointTypeIndex;
                if (index <= 0) return false;
                var typeString = words[index];
                string angleTemp = "";
                double temp;
                if (words.Count() < index + 3) return false;
                if (!double.TryParse(words[index + 1], out temp) || !double.TryParse(words[index + 2], out temp))
                    return false;
                string Xtemp = words[++index];
                string Ytemp = words[++index];
                if ((words.Count() > ++index) && double.TryParse(words[index], out temp))
                {
                    angleTemp = words[index];
                    index++;
                }
                bool posV = true, velV = true, accelV = true;
                if (words.Count() > index && Boolean.TryParse(words[index], out posV))
                    if (words.Count() > ++index && Boolean.TryParse(words[index], out velV))
                        if (words.Count() > ++index && Boolean.TryParse(words[index], out accelV))
                        {
                        }
                words.RemoveRange(jointTypeIndex, words.Count - jointTypeIndex);
                var linkIDStr = words.Aggregate("", (current, s) => current + ("," + s));
                linkIDStr = linkIDStr.Remove(0, 1);
                jointsInfo.Add(new JointData
                    {
                        JointType = typeString,
                        XPos = Xtemp,
                        YPos = Ytemp,
                        Angle = angleTemp,
                        LinkNames = linkIDStr,
                        PosVisible = posV,
                        VelocityVisible = velV,
                        AccelerationVisible = accelV
                    });
            }
            return true;
        }

        internal static string ConvertDataToText(ObservableCollection<JointData> collection)
        {
            var text = "";
            foreach (var jInfo in collection)
            {
                if (string.IsNullOrWhiteSpace(jInfo.LinkNames) || (string.IsNullOrWhiteSpace(jInfo.JointType)))
                    continue;
                text += jInfo.LinkNames + ",";
                text += jInfo.JointType + ",";
                text += jInfo.XPos + ",";
                text += jInfo.YPos;
                text += (!string.IsNullOrWhiteSpace(jInfo.Angle)) ? "," + jInfo.Angle : "";
                var boolStr = "";
                if (!jInfo.AccelerationVisible) boolStr = ",false";
                if (!jInfo.VelocityVisible || !jInfo.AccelerationVisible)
                    boolStr = "," + jInfo.VelocityVisible.ToString() + boolStr;
                if (!jInfo.PosVisible || !jInfo.VelocityVisible || !jInfo.AccelerationVisible)
                    boolStr = "," + jInfo.PosVisible.ToString() + boolStr;
                text += boolStr + "\n";
            }
            return text;
        }
    }
}

