using System;
using System.Windows.Browser;
using System.Windows.Controls;

namespace PMKS_Silverlight_App
{
    public static class UrlArgumentHandling
    {
        private static string debugString = //"";
        "mech=ground input R -29.9751570727451 39.6305466723673 0 ffft|input output R -41.4332984863479 48.5516072917578 0 ffff|output ground RP -13.2095 8.9821 1 ffff|output R -4.55078125 37.451171875 0 tfff|";
        internal static Boolean UrlToGlobalSettings(MainPage main)
        {
            var globalSettingsString = getString("set");
            if (string.IsNullOrWhiteSpace(globalSettingsString)) return false;
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
            var targetShapeString = getString("ts");
            if (string.IsNullOrWhiteSpace(targetShapeString)) return false;
            main.fileAndEditPanel.TargetShapeStream.Text = targetShapeString;
            main.fileAndEditPanel.TargetShapeStream_OnTextChanged(null, null);
            return true;
        }
        internal static Boolean UrlToMechanism(MainPage main)
        {
            var initMechString = getString("mech");
            if (string.IsNullOrWhiteSpace(initMechString)) return false;
            initMechString = initMechString.Replace("|", "\n");
            return main.fileAndEditPanel.ConvertTextToData(initMechString);
        }
        internal static string MechanismToUrl(MainPage main)
        {
            var result = JointData.ConvertDataToText(main.JointsInfo.Data);
            result = result.Replace('\n', '|');
            result = result.Replace(',', ' ');
            result = result.Replace("  ", " ");
            // I thought I ought to call this last one twice just to be sure...
            result = result.Replace("  ", " ");
            return "mech=" + result;
        }

        internal static string TargetShapeToUrl(TextBox TargetShapeStream)
        {
            var result = TargetShapeStream.Text;
            if (string.IsNullOrWhiteSpace(result) || result.Trim().Equals(DisplayConstants.TargetShapeQueryText))
                return "";
            result = result.Replace(',', ' ');
            return "ts=" + result + "&";
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
            return "set=" + result + "&";
        }
    }
}
