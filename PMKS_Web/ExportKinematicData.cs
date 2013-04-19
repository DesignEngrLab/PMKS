using System;
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
    public static class ExportKinematicData
    {

        internal static void ExportToCSV(PlanarMechanismSimulator.Simulator simulator)
        {
            var saveFileDialog = new SaveFileDialog
                {
                    DefaultFileName =
                        "KinematicDatafromPMKS" + DateTime.Now.ToShortTimeString() + "_" +
                        DateTime.Now.ToShortDateString(),
                    DefaultExt = ".csv",
                    Filter = "Comma Separated Values (*.csv)|*.txt|Text Files (*.txt)|*.txt|All Files (*.*)|*.*",
                };
if ((bool) saveFileDialog.ShowDialog())
{
    ;
}
        }
    }
}
