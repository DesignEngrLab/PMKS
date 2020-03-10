// ***********************************************************************
// Assembly         : Silverlight_PMKS
// Author           : Matt
// Created          : 06-10-2015
//
// Last Modified By : Matt
// Last Modified On : 06-27-2015
// ***********************************************************************
// <copyright file="ExportKinematicData.cs" company="">
//     Copyright ©  2012
// </copyright>
// <summary></summary>
// ***********************************************************************
using System;
using System.IO;
using System.Text;
using System.Windows.Controls;
using PMKS;
using Silverlight_PMKS;

/// <summary>
/// The PMKS_Silverlight_App namespace.
/// </summary>
namespace PMKS_Silverlight_App
{
    /// <summary>
    /// Class ExportKinematicData.
    /// </summary>
    internal static class ExportKinematicData
    {
        /// <summary>
        /// The cell length
        /// </summary>
        private const int cellLength = 16;

        /// <summary>
        /// Exports to CSV.
        /// </summary>
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
                    "Tab-Delimited text file (*.txt)|*.txt|Comma Separated Values file (*.csv)|*.csv|All Files (*.*)|*.*"
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
                    if (Path.GetExtension(saveFileDialog.SafeFileName) == ".csv")
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

        /// <summary>
        /// Converts the PMKS data to string.
        /// </summary>
        /// <param name="pmks">The PMKS.</param>
        /// <param name="sep">The sep.</param>
        /// <returns>System.String.</returns>
        private static string ConvertPMKSDataToString(Simulator pmks, string sep)
        {
            var timeSteps = pmks.JointParameters.Count;
            var stringBuilder = new StringBuilder((timeSteps + 1)*cellLength*(pmks.NumJoints*6 + pmks.NumLinks*3));
            stringBuilder.Append("TimeSteps");
            for (var i = 0; i < pmks.NumJoints; i++)
                stringBuilder.Append(
                    sep + "x_" + i + sep + "y_" + i + sep + "Vx_" + i + sep + "Vy_" + i + sep + "Ax_" + i + sep + "Ay_" +
                    i);
            for (var i = 0; i < pmks.NumLinks; i++)
                stringBuilder.Append(
                    sep + "angle_" + pmks.Links[i].name + sep + "angVel_" + pmks.Links[i].name + sep + "angAccel_" +
                    pmks.Links[i].name);
            stringBuilder.AppendLine();

            var times = pmks.JointParameters.Times;
            var jParams = pmks.JointParameters.Parameters;
            var lParams = pmks.LinkParameters.Parameters;
            var lastLink = pmks.NumLinks - 1;
            for (var i = 0; i < timeSteps; i++)
            {
                stringBuilder.Append(times[i] + sep);
                for (var j = 0; j < pmks.NumJoints; j++)
                    for (var k = 0; k < 6; k++)
                        stringBuilder.Append(jParams[i][j, k] + sep);
                for (var j = 0; j < pmks.NumLinks - 1; j++)
                    stringBuilder.Append(lParams[i][j, 0] + sep + lParams[i][j, 1] + sep + lParams[i][j, 2] + sep);
                stringBuilder.AppendLine(lParams[i][lastLink, 0] + sep + lParams[i][lastLink, 1] + sep +
                                         lParams[i][lastLink, 2]);
            }
            return stringBuilder.ToString();
        }
    }
}