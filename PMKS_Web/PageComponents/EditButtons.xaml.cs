using System;
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

namespace PMKS_Silverlight_App
{
    public partial class EditButtons : UserControl
    {
        public EditButtons()
        {
            InitializeComponent();
        } 

        private void OpenButton_Click(object sender, RoutedEventArgs e)
        {
            string data = "";
            var dialog = new OpenFileDialog
                {
                    Filter = "Text Files (*.txt)|*.txt|Comma-Separated Values (.csv)|*.csv"
                };
            if (dialog.ShowDialog() == true)
                using (var reader = dialog.File.OpenText())
                {
                    JointsViewModel jInfo = null;
                    if (JointsViewModel.ConvertTextToData(reader.ReadToEnd(), out jInfo))
                    {
                        PMKSControl.JointsInfo = jInfo;
                        PMKSControl.ParseData();
                    }
                }
        }
        private void SaveButton_Click(object sender, RoutedEventArgs e)
        {
            string data = "";
            var dialog = new SaveFileDialog()
            {
                Filter = "Text Files (*.txt)|*.txt|Comma-Separated Values (.csv)|*.csv"
            };
            if (dialog.ShowDialog() == true)
                using (var stream = dialog.OpenFile())
                using (var writer = new StreamWriter(stream))
                    writer.Write(JointsViewModel.ConvertDataToText(PMKSControl.JointsInfo));
        }

        private void AddButton_Click(object sender, RoutedEventArgs e)
        {
            PMKSControl.JointsInfo.Data.Add(new JointData());
        }
        private void ClearButton_Click(object sender, RoutedEventArgs e)
        {

        }
        private void RemoveButton_Click(object sender, RoutedEventArgs e)
        {

        }

        private void SimulateButton_Click(object sender, RoutedEventArgs e)
        {
            PMKSControl.ParseData();
        }

    }
}