using System.Text;
using System.Windows.Browser;
using PlanarMechanismSimulator;
using System;
using System.IO;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using Silverlight_PMKS;

namespace PMKS_Silverlight_App
{
    public static class UrlArgumentHandling
    {
        internal static void UrlToTargetShape(MainPage main)
        {
            if (HtmlPage.Document.QueryString.ContainsKey("ts"))
            {
                main.fileAndEditPanel.TargetShapeStream.Text = HtmlPage.Document.QueryString["ts"];
                main.fileAndEditPanel.TargetShapeStream_OnTextChanged(null, null);
            }
        }
        internal static void UrlToMechanism(MainPage main)
        {
            //var initMechString = "";
            var initMechString =
                "ground input R 7 5 1 0001|coupler input R -2.3 3.98 1.6 0110|output coupler P 10 0 0 0000|ground output R 8 3 0 0110|coupler R 0 0 0000|";

            if (string.IsNullOrWhiteSpace(initMechString) && HtmlPage.Document.QueryString.ContainsKey("mech"))
                initMechString = HtmlPage.Document.QueryString["mech"];
            if (string.IsNullOrWhiteSpace(initMechString)) return;
            initMechString = initMechString.Replace("|", "\n");
            main.fileAndEditPanel.ConvertTextToData(initMechString);
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
    }
}
