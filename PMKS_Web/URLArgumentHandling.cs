using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Windows.Browser;
using System.Windows.Controls;
using Silverlight_PMKS;

namespace PMKS_Silverlight_App
{
    public static class IOStringFunctions
    {
        private static string debugString = // "";
      //  "mech=ground input R 0 0 tfft|input output G 25 0 0 tfff|output ground R 35 0 0 tfff|";
        "mech=ground input R 10.86328125 -0.63671875 0 tfff|input c R 7 7 0 tfff|c r P 14 0 60 tfff|r h R 11.59375 -10.818359375 1 tfff|c out R -1.1015625 7.3359375 0 tfff|out ground R -7.359375 -4.017578125 0 tfff|h ground R 0 0 45 tfft|";
         public const string GlobalSettingString = "set=";
        public const string TargetShapeString = "ts=";
        public const string MechanismString = "mech=";
       
        internal static Boolean UrlToGlobalSettings(MainPage main)
        {
            var globalSettingsString = getString(GlobalSettingString.TrimEnd('='));
            if (string.IsNullOrWhiteSpace(globalSettingsString)) return false;
            return StringToGlobalSettings(globalSettingsString, main);
        }

        internal static bool StringToGlobalSettings(string globalSettingsString, MainPage main)
        {
            var settingsList = globalSettingsString.Split('|');
            foreach (var setting in settingsList)
            {
                double value;
                var valString = setting.Substring(1);
                if (!double.TryParse(valString, out value)) continue;
                switch (setting[0])
                {
                    case 's':
                        main.Speed = value;
                        break;
                    case 'e':
                        main.Error = value;
                        main.AnalysisStep = AnalysisType.error;
                        break;
                    case 'f':
                        main.AngleIncrement = value;
                        main.AnalysisStep = AnalysisType.fixedDelta;
                        break;
                }
            }
            return true;
        }

        private static string getString(string p)
        {
            if (HtmlPage.Document.QueryString.ContainsKey(p))
                return HtmlPage.Document.QueryString[p];
            var index = debugString.IndexOf(p);
            if (index == -1) return "";
            var endIndex = debugString.IndexOf("&", index + p.Length);
            if (endIndex <= index) endIndex = debugString.Length;
            return debugString.Substring(index + p.Length + 1, endIndex - index - p.Length - 1);
        }

        internal static Boolean UrlToTargetShape(MainPage main)
        {
            var targetShapeString = getString(TargetShapeString.TrimEnd('='));
            return StringToTargetShape(targetShapeString, main);
        }

        internal static bool StringToTargetShape(string targetShapeString, MainPage main)
        {
            if (string.IsNullOrWhiteSpace(targetShapeString)) return false;
            main.fileAndEditPanel.TargetShapeStream.Text = targetShapeString;
            main.fileAndEditPanel.TargetShapeStream_OnTextChanged(null, null);
            return true;
        }


        internal static Boolean UrlToMechanism(MainPage main)
        {
            var initMechString = getString(MechanismString.TrimEnd('='));
            if (string.IsNullOrWhiteSpace(initMechString)) return false;
            initMechString = initMechString.Replace("|", "\n");
            return IOStringFunctions.ConvertTextToData(initMechString);
        }
        internal static string MechanismToUrl(MainPage main)
        {
            var result = JointData.ConvertDataToText(main.JointsInfo.Data);
            result = result.Replace('\n', '|');
            result = result.Replace(',', ' ');
            result = result.Replace("  ", " ");
            // I thought I ought to call this last one twice just to be sure...
            result = result.Replace("  ", " ");
            return MechanismString + result;
        }

        internal static string TargetShapeToUrl(TextBox TargetShapeStream)
        {
            var result = TargetShapeStream.Text;
            if (string.IsNullOrWhiteSpace(result) || result.Trim().Equals(DisplayConstants.TargetShapeQueryText))
                return "";
            result = result.Replace(',', ' ');
            return TargetShapeString + result;
        }
        internal static string GlobslSettingsToUrl(MainPage main)
        {
            var result = "";
            if (main.Speed != DisplayConstants.DefaultSpeed)
                result += "s" + main.Speed + "|";
            if (main.AnalysisStep != AnalysisType.error || main.Error != DisplayConstants.DefaultError)
            {
                if (main.AnalysisStep == AnalysisType.error)
                    result += "e" + main.Error + "|";
                else result += "f" + main.AngleIncrement + "|";

            }
            result = result.TrimEnd('|');
            if (string.IsNullOrWhiteSpace(result)) return "";
            return GlobalSettingString + result;
        }

        internal static bool OpenConfigFromTextFile(string fileText)
        {
            var startIndex = fileText.IndexOf(IOStringFunctions.GlobalSettingString);
            App.main.globalSettings.ResetToDefault();
            if (startIndex >= 0) //
            {
                startIndex += IOStringFunctions.GlobalSettingString.Length;
                var endIndex = fileText.IndexOf("\n", startIndex);
                var settingString = fileText.Substring(startIndex, endIndex - startIndex);
                IOStringFunctions.StringToGlobalSettings(settingString, App.main);
            }
            startIndex = fileText.IndexOf(IOStringFunctions.TargetShapeString);
            if (startIndex >= 0) //
            {
                startIndex += IOStringFunctions.TargetShapeString.Length;
                var endIndex = fileText.IndexOf("\n", startIndex);
                var settingString = fileText.Substring(startIndex, endIndex - startIndex);
                IOStringFunctions.StringToTargetShape(settingString, App.main);
            }
            startIndex = fileText.IndexOf(IOStringFunctions.MechanismString);
            if (startIndex >= 0) return ConvertTextToData(fileText.Substring(startIndex
                + IOStringFunctions.MechanismString.Length + 1));
            else return ConvertTextToData(fileText);
        }


        public static Boolean ConvertTextToData(string mechString)
        {
            List<JointData> jointDataList = null;
            if (JointData.ConvertTextToData(mechString, out jointDataList))
            {
                App.main.JointsInfo.Data.Clear();
                App.main.LinksInfo.Data.Clear();
                foreach (var j in jointDataList)
                {
                    App.main.JointsInfo.Data.Add(j);
                    App.main.linkInputTable.UpdateLinksTableAfterAdd(j);
                }
                return true;
            }
            return false;
        }


        internal static bool ConvertTextToData(string text, out List<JointData> jointsInfo)
        {
            jointsInfo = new List<JointData>();
            var pivotSentences = text.Split('\n').ToList();
            pivotSentences.RemoveAll(string.IsNullOrWhiteSpace);
            if (pivotSentences.Count == 0) return false;
            foreach (var pivotSentence in pivotSentences)
            {
                var words = pivotSentence.Split(',', ' ', '|').ToList();
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
                var bools = new bool[4];
                if (words.Count() > index && (words[index].Contains("t") || words[index].Contains("f")))
                {
                    var plusMinusString = words[index];
                    int i = 0;
                    while (i < plusMinusString.Length)
                    {
                        if (plusMinusString[i].Equals('t')) bools[i] = true;
                        i++;
                    }
                }
                else
                {
                    int i = 0;
                    while (index + i < words.Count)
                    {
                        Boolean.TryParse(words[index], out bools[i]);
                        i++;
                    }
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
                        PosVisible = bools[0],
                        VelocityVisible = bools[1],
                        AccelerationVisible = bools[2],
                        DrivingInput = bools[3]
                    });
            }
            jointsInfo.Add(new JointData());
            if (jointsInfo.All(jd => !jd.DrivingInput))
                jointsInfo.First(jd => jd.CanBeDriver).DrivingInput = true;
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
                var boolStr = ","
                              + (jInfo.PosVisible ? 't' : 'f')
                              + (jInfo.VelocityVisible ? 't' : 'f')
                              + (jInfo.AccelerationVisible ? 't' : 'f')
                              + (jInfo.DrivingInput ? 't' : 'f');
                //var boolStr = "," + jInfo.PosVisible
                //    + "," + jInfo.VelocityVisible
                //    + "," + jInfo.AccelerationVisible
                //    + "," + jInfo.DrivingInput;
                //while (boolStr.EndsWith(",False"))
                //{
                //    boolStr = boolStr.Remove(boolStr.Length - 6);
                //}
                text += boolStr + "\n";
            }
            return text;
        }

    }
}


