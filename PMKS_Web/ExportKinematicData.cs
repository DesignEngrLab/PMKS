using System.Text;
using PMKS;
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
    public static class ExportKinematicData
    {
        private const int cellLength = 16;
        internal static void ExportToCSV()
        {
            var saveFileDialog = new SaveFileDialog
            {
                DefaultFileName =
                    "KinematicDatafromPMKS."
                    + DateTime.Now.Second + "." + DateTime.Now.Minute + "."
                    + DateTime.Now.Hour + "." + DateTime.Now.Day + "."
                    + DateTime.Now.Month + "." + DateTime.Now.Year,
                DefaultExt = ".txt",
                Filter =
                    "Tab-Delimited text file (*.txt)|*.txt|Comma Separated Values file (*.csv)|*.csv|All Files (*.*)|*.*",
            };
            var result = saveFileDialog.ShowDialog();

            try
            {
                if (result == true)
                {
                    App.main.status("Exporting Data...");
                   var now = DateTime.Now;
                    var fileStream = saveFileDialog.OpenFile();
                    var sw = new StreamWriter(fileStream, Encoding.Unicode);
                    if (System.IO.Path.GetExtension(saveFileDialog.SafeFileName) == ".csv")
                        sw.Write(ConvertPMKSDataToString(App.main.pmks, ","));
                    else sw.Write(ConvertPMKSDataToString(App.main.pmks, "\t"));
                    sw.Flush();
                    sw.Close();

                    App.main.status("...done (" + (DateTime.Now - now).TotalMilliseconds + "ms).");
                }

            }
            catch (Exception exc)
            {
                App.main.status("********** Unable to write to file " + saveFileDialog.SafeFileName + ". ********");
                App.main.status(exc.Message);
                App.main.status("********** Unable to write to file " + saveFileDialog.SafeFileName + ". ********");
            }
        }

        private static string ConvertPMKSDataToString(Simulator pmks, string sep)
        {
            var timeSteps = pmks.JointParameters.Count;
            var stringBuilder = new StringBuilder((timeSteps + 1) * cellLength * (pmks.numJoints * 6 + pmks.numLinks * 3));
            stringBuilder.Append("TimeSteps");
            for (int i = 0; i < pmks.numJoints; i++)
                stringBuilder.Append(
                    sep + "x_" + i + sep + "y_" + i + sep + "Vx_" + i + sep + "Vy_" + i + sep + "Ax_" + i + sep + "Ay_" +
                    i);
            for (int i = 0; i < pmks.numLinks; i++)
                stringBuilder.Append(
                    sep + "angle_" + pmks.AllLinks[i].name + sep + "angVel_" + pmks.AllLinks[i].name + sep + "angAccel_" +
                    pmks.AllLinks[i].name);
            stringBuilder.AppendLine();

            var times = pmks.JointParameters.Times;
            var jParams = pmks.JointParameters.Parameters;
            var lParams = pmks.LinkParameters.Parameters;
            var lastLink = pmks.numLinks - 1;
            for (int i = 0; i < timeSteps; i++)
            {
                stringBuilder.Append(times[i] + sep);
                for (int j = 0; j < pmks.numJoints; j++)
                    for (int k = 0; k < 6; k++)
                        stringBuilder.Append(jParams[i][pmks.JointNewIndexFromOriginal[j], k] + sep);
                for (int j = 0; j < pmks.numLinks - 1; j++)
                    stringBuilder.Append(lParams[i][j, 0] + sep + lParams[i][j, 1] + sep + lParams[i][j, 2] + sep);
                stringBuilder.AppendLine(lParams[i][lastLink, 0] + sep + lParams[i][lastLink, 1] + sep + lParams[i][lastLink, 2]);
            }
            return stringBuilder.ToString();
        }
    }
}
