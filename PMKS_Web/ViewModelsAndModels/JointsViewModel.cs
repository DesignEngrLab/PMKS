using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class JointsViewModel : ViewModelBase
    {
        ObservableCollection<JointData> data;

        public JointsViewModel()
        {
            Data = new ObservableCollection<JointData>()
            {
                new JointData {JointType = "R (pin joint)", XPos = "0.0", YPos = "0.0", LinkNames="ground, input"},
                new JointData {JointType = "R (pin joint)", XPos = "0.0", YPos = "8.0", LinkNames="input coupler"},
                new JointData {JointType = "R (pin joint)", XPos = "10.0", YPos = "12.0", LinkNames="coupler,output "},
                new JointData {JointType = "R (pin joint)", XPos = "10.0", YPos = "0.0", LinkNames="ground, outpu"},
                new JointData(),new JointData(),new JointData(),new JointData(),new JointData()
            };
        }

        public ObservableCollection<JointData> Data
        {
            get
            {
                return data;
            }
            set
            {
                data = value;
                OnPropertyChanged("Data");
            }
        }

        public static Boolean ConvertTextToData(string text, out JointsViewModel jointsInfo)
        {
            jointsInfo = null;
            var data = new ObservableCollection<JointData>();
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
                        if (words.Count() > ++index && Boolean.TryParse(words[index], out accelV)) { }
                words.RemoveRange(jointTypeIndex, words.Count - jointTypeIndex);
                var linkIDStr = words.Aggregate("", (current, s) => current + ("," + s));
                linkIDStr = linkIDStr.Remove(0, 1);
                data.Add(new JointData
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
            jointsInfo = new JointsViewModel { Data = data };
            return true;
        }
        internal static string ConvertDataToText(JointsViewModel jointsViewModel)
        {
            var text = "";
            foreach (var jInfo in jointsViewModel.Data)
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
    public class JointTypeProvider
    {
        public List<string> jointTypeList
        {
            get { return new List<string> { "R (pin joint)", "P (sliding block)", "RP (pin in slot)", "G (gear teeth)" }; }
        }
    }


}

