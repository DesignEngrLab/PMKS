﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using System.Collections.ObjectModel;

namespace PMKS_Silverlight_App
{
    public partial class EditButtons : UserControl
    {
        public MainPage main { private get; set; }
        public EditButtons()
        {
            InitializeComponent();
        }

        private void OpenButton_Click(object sender, RoutedEventArgs e)
        {
            var dialog = new OpenFileDialog
                {
                    Filter = "Text Files (*.txt)|*.txt|Comma-Separated Values (.csv)|*.csv"
                };
            if (dialog.ShowDialog() == true)
                using (var reader = dialog.File.OpenText())
                {
                    List<JointData> jointDataList = null;
                    if (JointData.ConvertTextToData(reader.ReadToEnd(), out jointDataList))
                    {
                        ClearButton_Click(null, null);
                        foreach (var j in jointDataList) main.JointsInfo.Data.Add(j);
                    }
                }
        }
        private void SaveButton_Click(object sender, RoutedEventArgs e)
        {
            var dialog = new SaveFileDialog()
            {
                Filter = "Text Files (*.txt)|*.txt|Comma-Separated Values (.csv)|*.csv"
            };
            if (dialog.ShowDialog() == true)
                using (var stream = dialog.OpenFile())
                using (var writer = new StreamWriter(stream))
                    writer.Write(JointData.ConvertDataToText(main.JointsInfo.Data));
        }

        internal void AddButton_Click(object sender, RoutedEventArgs e)
        {
            main.JointsInfo.Data.Add(new JointData());
        }
        private void ClearButton_Click(object sender, RoutedEventArgs e)
        {
            main.JointsInfo.Data.Clear();

        }
        private void RemoveButton_Click(object sender, RoutedEventArgs e)
        {
            var jointData = main.JointsInfo.Data;
            var LinkData = main.LinksInfo.Data;
            var Jointstable = main.jointInputTable.dataGrid;
            JointData removedJoint;
            if (Jointstable.SelectedItem == null)
            {
                removedJoint = jointData[jointData.Count - 1];
                jointData.RemoveAt(jointData.Count - 1);
            }
            else
            {
                removedJoint = jointData[Jointstable.SelectedIndex];
                jointData.RemoveAt(Jointstable.SelectedIndex);
            }
            UpdateLinksTable(jointData, LinkData, removedJoint);
        }

        private void UpdateLinksTable(ObservableCollection<JointData> JointData, ObservableCollection<LinkData> LinksData, JointData removedJoint)
        {
            foreach (string link in removedJoint.LinkNamesList)
            {
                bool found = false;
                int index;
                for (index = 0; index < JointData.Count && !found; index++)
                    found = JointData[index].LinkNamesList.Contains(link);
                if (!found)
                    RemoveLink(LinksData, link);
            }
            
        }

        private void RemoveLink(ObservableCollection<LinkData> LinksData, string link)
        {
            for (int index = 0; index < LinksData.Count; index++)
                if (LinksData[index].Name.Equals(link))
                {
                    LinksData.RemoveAt(index);
                    return;
                }
        }

        private void SimulateButton_Click(object sender, RoutedEventArgs e)
        {
            main.ParseData();
        }

    }
}