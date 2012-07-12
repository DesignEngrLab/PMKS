using System;
using System.Collections.Generic;
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
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public static class PMKSControl
    {
        public static JointsViewModel JointsInfo { get; set; }

        public static bool IsAngleInsteadOfError { get; set; }

        private static Simulator pmks;
        private static readonly List<List<string>> LinkIDs = new List<List<string>>();
        private static readonly List<string> JointTypes = new List<string>();
        private static readonly List<double[]> InitPositions = new List<double[]>();
        private static int numJoints;

        internal static void ParseData()
        {
            if (JointsInfo == null) return;
            numJoints = TrimEmptyJoints();
            if (SameTopology() && SameParameters()) return;

            if (SameTopology())
            {
                DefinePositions();
                pmks.AssignPositions(InitPositions);
                if (pmks.DegreesOfFreedom == 1) pmks.FindFullMovement();
                UpdateVisuals();
            }
            else
            {
                if (!(DefineLinkIDS() && DefinePositions() && DefineJointTypeList())) return;

                try
                { RunSimulation();
                }
                catch (Exception e)
                {
                    status(e.Message);
                }
            }

        }

        public static void RunSimulation()
        {
            if (pmks == null) pmks = new Simulator(LinkIDs, JointTypes, InitPositions);

            if (pmks.IsDyadic) status("The mechanism is comprised of only of dyads.");
            else status("The mechanism has non-dyadic loops.");
            int dof = pmks.DegreesOfFreedom;
            status("Degrees of freedom = " + dof);
            if (dof == 1)
            {
                pmks.DeltaAngle = AngleIncrement;
                pmks.InputSpeed = Speed;
                pmks.FindFullMovement();
                UpdateVisuals();
            }

        }

        private static int TrimEmptyJoints()
        {
            for (int i = 0; i < JointsInfo.Data.Count; i++)
                if (JointsInfo.Data[i].JointType == null) return i;
            return -1;
        }

        private static bool SameParameters()
        {
            if (numJoints != JointTypes.Count) return false;
            for (int i = 0; i < numJoints; i++)
            {
                if (Math.Abs(InitPositions[i][0] - Double.Parse(JointsInfo.Data[i].XPos)) > Constants.epsilon) return false;
                if (Math.Abs(InitPositions[i][1] - Double.Parse(JointsInfo.Data[i].YPos)) > Constants.epsilon) return false;
                if (!(InitPositions[i].GetLength(0) == 2 && string.IsNullOrWhiteSpace(JointsInfo.Data[i].Angle))
                    && Math.Abs(InitPositions[i][2] - Double.Parse(JointsInfo.Data[i].Angle)) > Constants.epsilon) return false;
            }
            return true;
        }

        private static bool SameTopology()
        {
            if (numJoints != JointTypes.Count) return false;
            for (int i = 0; i < numJoints; i++)
            {
                if (!JointsInfo.Data[i].JointType.StartsWith(JointTypes[i])) return false;
                var newLinkIDS = new List<string>(JointsInfo.Data[i].LinkNames.Split(new[] { ',', ' ' },
                    StringSplitOptions.RemoveEmptyEntries));
                if (LinkIDs[i].Any(linkID => !newLinkIDS.Remove(linkID)))
                    return false;
                if (newLinkIDS.Count > 0) return false;
            }
            return true;
        }

        private static void status(string p)
        {
            StatusBox.Text += "\n" + p;
        }

        private static Boolean DefineJointTypeList()
        {
            JointTypes.Clear();
            for (int i = 0; i < numJoints; i++)
            {
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].JointType)) return false;
                JointTypes.Add(JointsInfo.Data[i].JointType.Substring(0, 2).Trim(' '));
            }
            return true;
        }

        private static Boolean DefineLinkIDS()
        {
            LinkIDs.Clear();
            for (int i = 0; i < numJoints; i++)
            {
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].LinkNames)) return false;
                var linkNames = new List<string>(JointsInfo.Data[i].LinkNames.Split(new[] { ',', ' ' },
                    StringSplitOptions.RemoveEmptyEntries));
                if (linkNames.Count == 0) return false;
                LinkIDs.Add(linkNames);
            }
            return true;
        }

        private static Boolean DefinePositions()
        {
            InitPositions.Clear();
            for (int i = 0; i < numJoints; i++)
            {
                if (string.IsNullOrWhiteSpace(JointsInfo.Data[i].XPos) ||
                    string.IsNullOrWhiteSpace(JointsInfo.Data[i].YPos))
                    return false;
                InitPositions.Add(string.IsNullOrWhiteSpace(JointsInfo.Data[i].Angle)
                                      ? new[] { Double.Parse(JointsInfo.Data[i].XPos), Double.Parse(JointsInfo.Data[i].YPos) }
                                      : new[]
                                          {
                                              Double.Parse(JointsInfo.Data[i].XPos),
                                              Double.Parse(JointsInfo.Data[i].YPos),
                                              Double.Parse(JointsInfo.Data[i].Angle)
                                          });
            }
            return true;
        }

        public static void UpdateVisuals()
        {
            if (pmks == null || pmks.JointParameters == null) return;
            Canvas.Children.Clear();
            foreach (var jointParam in pmks.JointParameters.Parameters)
            {
                for (int i = 0; i < pmks.numJoints; i++)
                {
                    Canvas.Children.Add(new Ellipse
                        {
                            Width = 500,
                            Height = 500,
                            RenderTransform = new TranslateTransform { X = 100*jointParam[i, 0], Y = 100*jointParam[i, 1] },
                            Fill = new SolidColorBrush { Color = Colors.Green }
                        });

                }
            }
        }

        internal static void SettingsUpdated()
        {
            if (pmks == null || pmks.JointParameters == null) return;
            throw new NotImplementedException();
        }

        public static double Speed { get; set; }

        public static double AngleIncrement { get; set; }

        public static double Error { get; set; }

        public static bool IsMetric { get; set; }

        public static bool IsRadians { get; set; }

        public static TextBox StatusBox { get; set; }

        public static Canvas Canvas { get; set; }
    }
}
