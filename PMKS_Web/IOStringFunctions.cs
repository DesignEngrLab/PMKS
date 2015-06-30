// ***********************************************************************
// Assembly         : Silverlight_PMKS
// Author           : Matt
// Created          : 06-10-2015
//
// Last Modified By : Matt
// Last Modified On : 06-26-2015
// ***********************************************************************
// <copyright file="IOStringFunctions.cs" company="">
//     Copyright ©  2012
// </copyright>
// <summary></summary>
// ***********************************************************************
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Windows.Browser;
using System.Windows.Controls;
using PMKS;
using Silverlight_PMKS;

/// <summary>
/// The PMKS_Silverlight_App namespace.
/// </summary>
namespace PMKS_Silverlight_App
{
    /// <summary>
    /// Class IOStringFunctions.
    /// </summary>
    public static class IOStringFunctions
    {
        // "set=s50|f0.5&ts=h60v4M46 78v-60h4M183 41l-30 52l3.5 2&mech=ground input R 97.194 -5.099 0.000 tfff|input leg1 R -0.088 0.000 0.000 tfff|leg1 output R 60.055 -0.012 0.000 tfff|output ground R 104.674 48.209 0.000 tfff|input leg2 R 53.490 -2.745 0.000 ffff|leg2 input1 R 159.370 -6.979 0.000 tfff|input1 ground P 222.793 -34.952 45.000 ffft|";
        /// <summary>
        /// The global setting string
        /// </summary>
        public const string GlobalSettingString = "set=";
        /// <summary>
        /// The target shape string
        /// </summary>
        public const string TargetShapeString = "ts=";
        /// <summary>
        /// The mechanism string
        /// </summary>
        public const string MechanismString = "mech=";
        /// <summary>
        /// The debug string
        /// </summary>
        private static readonly string debugString = "";

        /// <summary>
        /// Gets the string.
        /// </summary>
        /// <param name="p">The p.</param>
        /// <returns>System.String.</returns>
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

        /// <summary>
        /// Opens the configuration from text file.
        /// </summary>
        /// <param name="fileText">The file text.</param>
        /// <returns><c>true</c> if XXXX, <c>false</c> otherwise.</returns>
        internal static bool OpenConfigFromTextFile(string fileText)
        {
            var startIndex = fileText.IndexOf(GlobalSettingString);
            App.main.globalSettings.ResetToDefault();
            if (startIndex >= 0) //
            {
                startIndex += GlobalSettingString.Length;
                var endIndex = fileText.IndexOf("\n", startIndex);
                var settingString = fileText.Substring(startIndex, endIndex - startIndex);
                StringToGlobalSettings(settingString);
            }
            startIndex = fileText.IndexOf(TargetShapeString);
            if (startIndex >= 0)
            {
                startIndex += TargetShapeString.Length;
                var endIndex = fileText.IndexOf("\n", startIndex);
                var settingString = fileText.Substring(startIndex, endIndex - startIndex);
                StringToTargetShape(settingString);
            }
            startIndex = fileText.IndexOf(MechanismString);
            if (startIndex >= 0)
                return StringToMechanism(fileText.Substring(startIndex + MechanismString.Length + 1));
            return StringToMechanism(fileText);
        }

        #region Global Settings

        /// <summary>
        /// URLs to global settings.
        /// </summary>
        /// <returns>Boolean.</returns>
        internal static Boolean UrlToGlobalSettings()
        {
            var globalSettingsString = getString(GlobalSettingString.TrimEnd('='));
            if (string.IsNullOrWhiteSpace(globalSettingsString)) return false;
            return StringToGlobalSettings(globalSettingsString);
        }

        /// <summary>
        /// Strings to global settings.
        /// </summary>
        /// <param name="globalSettingsString">The global settings string.</param>
        /// <returns><c>true</c> if XXXX, <c>false</c> otherwise.</returns>
        internal static bool StringToGlobalSettings(string globalSettingsString)
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
                        App.main.Speed = value;
                        break;
                    case 'e':
                        App.main.Error = value;
                        App.main.AnalysisStep = AnalysisType.error;
                        break;
                    case 'f':
                        App.main.AngleIncrement = value;
                        App.main.AnalysisStep = AnalysisType.fixedDelta;
                        break;
                }
            }
            return true;
        }

        /// <summary>
        /// Globals the settings to URL.
        /// </summary>
        /// <returns>System.String.</returns>
        internal static string GlobalSettingsToUrl()
        {
            var result = "";
            if (App.main.Speed != DisplayConstants.DefaultSpeed)
                result += "s" + App.main.Speed + "|";
            if (App.main.AnalysisStep != AnalysisType.error || App.main.Error != DisplayConstants.DefaultError)
            {
                if (App.main.AnalysisStep == AnalysisType.error)
                    result += "e" + App.main.Error + "|";
                else result += "f" + App.main.AngleIncrement + "|";
            }
            result = result.TrimEnd('|');
            if (string.IsNullOrWhiteSpace(result)) return "";
            return GlobalSettingString + result;
        }

        #endregion

        #region TargetShape

        /// <summary>
        /// URLs to target shape.
        /// </summary>
        /// <returns>Boolean.</returns>
        internal static Boolean UrlToTargetShape()
        {
            var targetShapeString = getString(TargetShapeString.TrimEnd('='));
            return StringToTargetShape(targetShapeString);
        }

        /// <summary>
        /// Strings to target shape.
        /// </summary>
        /// <param name="targetShapeString">The target shape string.</param>
        /// <returns><c>true</c> if XXXX, <c>false</c> otherwise.</returns>
        internal static bool StringToTargetShape(string targetShapeString)
        {
            if (string.IsNullOrWhiteSpace(targetShapeString)) return false;
            App.main.fileAndEditPanel.TargetShapeStream.Text = targetShapeString;
            App.main.fileAndEditPanel.TargetShapeStream_OnTextChanged(null, null);
            return true;
        }

        /// <summary>
        /// Targets the shape to URL.
        /// </summary>
        /// <param name="TargetShapeStream">The target shape stream.</param>
        /// <returns>System.String.</returns>
        internal static string TargetShapeToUrl(TextBox TargetShapeStream)
        {
            var result = TargetShapeStream.Text;
            if (string.IsNullOrWhiteSpace(result) || result.Trim().Equals(DisplayConstants.TargetShapeQueryText))
                return "";
            result = result.Replace(',', ' ');
            return TargetShapeString + result;
        }

        #endregion

        #region Mechanism

        /// <summary>
        /// URLs to mechanism.
        /// </summary>
        /// <returns>Boolean.</returns>
        internal static Boolean UrlToMechanism()
        {
            var initMechString = getString(MechanismString.TrimEnd('='));
            if (string.IsNullOrWhiteSpace(initMechString)) return false;
            initMechString = initMechString.Replace("|", "\n");
            return StringToMechanism(initMechString);
        }

        /// <summary>
        /// Mechanisms to URL.
        /// </summary>
        /// <returns>System.String.</returns>
        internal static string MechanismToUrl()
        {
            var result = JointData.ConvertDataToText('|');
            result = result.Replace(" ", "");
            return MechanismString + result;
        }


        /// <summary>
        /// Strings to mechanism.
        /// </summary>
        /// <param name="text">The text.</param>
        /// <returns>Boolean.</returns>
        internal static Boolean StringToMechanism(string text)
        {
            List<string[]> linkIDs;
            List<JointType> jointTypes;
            int driverIndex;
            List<double[]> initPositions;
            List<Boolean[]> displayBools;
            if (!Simulator.ConvertTextToData(text, out linkIDs, out jointTypes, out driverIndex, out initPositions
                , out displayBools))
            {
                App.main.status("Unable to read string as a mechanism: \"" + text + ".\"");
                return false;
            }
            App.main.JointsInfo.Data.Clear();
            App.main.LinksInfo.Data.Clear();
            for (var i = 0; i < linkIDs.Count; i++)
            {
                App.main.JointsInfo.Data.Add(new JointData
                {
                    TypeOfJoint = jointTypes[i],
                    X = initPositions[i][0],
                    Y = initPositions[i][1],
                    AngleDegrees = initPositions[i][2],
                    LinkNamesList = linkIDs[i],
                    PosVisible = displayBools[i][0],
                    VelocityVisible = displayBools[i][1],
                    AccelerationVisible = displayBools[i][2],
                    DrivingInput = displayBools[i][3]
                });
                App.main.linkInputTable.UpdateLinksTable();
            }
            return true;
        }

        /// <summary>
        /// Mechanisms to string.
        /// </summary>
        /// <param name="collection">The collection.</param>
        /// <returns>System.String.</returns>
        internal static string MechanismToString(ObservableCollection<JointData> collection)
        {
            var text = "";
            foreach (var jInfo in collection)
            {
                if (string.IsNullOrWhiteSpace(jInfo.LinkNames) || (string.IsNullOrWhiteSpace(jInfo.JointTypeString)))
                    continue;
                text += jInfo.LinkNames + ",";
                text += jInfo.JointTypeString + ",";
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

        #endregion
    }
}