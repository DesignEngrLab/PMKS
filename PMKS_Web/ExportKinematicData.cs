﻿using System.Text;
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

namespace PMKS_Silverlight_App
{
    public static class ExportKinematicData
    {

        internal static void ExportToCSV(Simulator pmks, MainPage main)
        {
            var saveFileDialog = new SaveFileDialog
                {
                    DefaultFileName =
                        "KinematicDatafromPMKS."+ DateTime.Now.ToOADate(),
                    DefaultExt = ".csv",
                    Filter = "Comma Separated Values file (*.csv)|*.csv|Tab-Delimited text file (*.txt)|*.txt|All Files (*.*)|*.*",
                };
            bool? result = saveFileDialog.ShowDialog();
            FileInfo fileInfo;
            try
            {
                if (result == true)
                {
                    var fileStream = saveFileDialog.OpenFile();
                    var sw = new StreamWriter(fileStream, Encoding.Unicode);
                    if (System.IO.Path.GetExtension(saveFileDialog.SafeFileName) == ".csv")
                        sw.Write(ConvertPMKSDataToCSV(pmks));
                    else sw.Write(ConvertPMKSDataToTabDelimitedTxt(pmks));
                    sw.Flush();
                    sw.Close();
                }
            }
            catch (Exception exc)
            {
                main.status("********** Unable to write to file " + saveFileDialog.SafeFileName + ". ********");
                main.status(exc.Message);
                main.status("********** Unable to write to file " + saveFileDialog.SafeFileName + ". ********");
            }
        }

        private static string ConvertPMKSDataToCSV(Simulator pmks)
        {
            var csv = "TimeSteps,Joints";
            for (int i = 0; i < 6 * pmks.numJoints; i++) csv += ",";
            csv += "Links";
            for (int i = 0; i < 3 * pmks.numLinks; i++) csv += ",";
            csv += "\n,";
            for (int i = 0; i < pmks.numJoints; i++) csv +=
                "x_"+i+",y_"+i+",Vx_"+i+",Vy_"+i+",Ax_"+i+",Ay_"+i+",";
            for (int i = 0; i < pmks.numLinks; i++) csv +=
                "angle_" + pmks.AllLinks[i].name + ",angVel_" + pmks.AllLinks[i].name + ",angAccel_" + pmks.AllLinks[i].name;
            csv += "\n";
            var timeSteps = pmks.JointParameters.Count;
            var times = pmks.JointParameters.Times;
            var jParams = pmks.JointParameters.Parameters;
            var lParams = pmks.LinkParameters.Parameters;
            var lastLink = pmks.numLinks - 1;
            for (int i = 0; i < timeSteps; i++)
            {
                csv += times[i] + ",";
                for (int j = 0; j < pmks.numJoints; j++)
                    for (int k = 0; k < 6; k++)
                        csv += jParams[i][pmks.JointReOrdering[j], k] + ",";
                for (int j = 0; j < pmks.numLinks - 1; j++)
                    csv += lParams[i][j, 0] + "," + lParams[i][j, 1] + "," + lParams[i][j, 2] + ",";
                csv += lParams[i][lastLink, 0] + "," + lParams[i][lastLink, 1] + "," + lParams[i][lastLink, 2] + "\n";
            }
            return csv;
        }
        private static string ConvertPMKSDataToTabDelimitedTxt(Simulator pmks)
        {
            var tabtxt = "TimeSteps\tJoints";
            for (int i = 0; i < 6 * pmks.numJoints; i++) tabtxt += "\t";
            tabtxt += "Links";
            for (int i = 0; i < 3 * pmks.numLinks; i++) tabtxt += "\t";
            tabtxt += "\n";
            for (int i = 0; i < pmks.numJoints; i++) tabtxt +=
                "\tx_" + i + "\ty_" + i + "\tVx_" + i + "\tVy_" + i + "\tAx_" + i + "\tAy_" + i;
            for (int i = 0; i < pmks.numLinks; i++) tabtxt +=
                "\tangle_" + pmks.AllLinks[i].name + "\tangVel_" + pmks.AllLinks[i].name + "\tangAccel_" + pmks.AllLinks[i].name;
            tabtxt += "\n";
            var timeSteps = pmks.JointParameters.Count;
            var times = pmks.JointParameters.Times;
            var jParams = pmks.JointParameters.Parameters;
            var lParams = pmks.LinkParameters.Parameters;
            var lastLink = pmks.numLinks - 1;
            for (int i = 0; i < timeSteps; i++)
            {
                tabtxt += times[i] + "\t";
                for (int j = 0; j < pmks.numJoints; j++)
                    for (int k = 0; k < 6; k++)
                        tabtxt += jParams[i][pmks.JointReOrdering[j], k] + "\t";
                for (int j = 0; j < pmks.numLinks - 1; j++)
                    tabtxt += lParams[i][j, 0] + "\t" + lParams[i][j, 1] + "\t" + lParams[i][j, 2] + "\t";
                tabtxt += lParams[i][lastLink, 0] + "\t" + lParams[i][lastLink, 1] + "\t" + lParams[i][lastLink, 2] + "\n";
            }
            return tabtxt;
        }

    }
}
