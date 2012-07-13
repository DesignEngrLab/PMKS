using System;
using System.Collections.Generic;
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
    public partial class GlobalSettings : UserControl
    {
        public GlobalSettings()
        {
            InitializeComponent();
            MetricCheckBox.IsChecked = true;
            RadiansCheckBox.IsChecked = true;
            AngleCheckBox.IsChecked = true;
            AngleErrorBox.Text = "0.1";
            AngleErrorBox_LostFocus(null,null);
            speedBox.Text = "1.0";
            speedBox_LostFocus(null,null);
        }
        
        private void AngleCheckBox_Checked(object sender, RoutedEventArgs e)
        {
            PMKSControl.IsAngleInsteadOfError = true;
            AngleErrorBox.Text = "";
        }

        private void ErrorCheckBox_Checked(object sender, RoutedEventArgs e)
        {
            PMKSControl.IsAngleInsteadOfError = false;
            AngleErrorBox.Text = "";
        }
   
        private void MetricCheckBox_Checked(object sender, RoutedEventArgs e)
        {
            PMKSControl.IsMetric = true;
            PMKSControl.UpdateVisuals();
        }

        private void InchesCheckBox_Checked(object sender, RoutedEventArgs e)
        {
            PMKSControl.IsMetric = false;
            PMKSControl.UpdateVisuals();
        }

        private void RadiansCheckBox_Checked(object sender, RoutedEventArgs e)
        {
            PMKSControl.IsRadians = true;
            if(PMKSControl.IsAngleInsteadOfError)
            PMKSControl.UpdateVisuals();
        }

        private void DegreesCheckBox_Checked(object sender, RoutedEventArgs e)
        {
            PMKSControl.IsRadians = true;
            PMKSControl.UpdateVisuals();
        }

        private void speedBox_LostFocus(object sender, RoutedEventArgs e)
        {

            double temp;
            if(double.TryParse(speedBox.Text,out temp))
            {
                PMKSControl.Speed = temp;
                PMKSControl.SettingsUpdated();
            }
        }

        private void AngleErrorBox_LostFocus(object sender, RoutedEventArgs e)
        {
            double temp;
            if (double.TryParse(AngleErrorBox.Text, out temp))
            {
                if (PMKSControl.IsAngleInsteadOfError)
                    PMKSControl.AngleIncrement = temp;
                else PMKSControl.Error = temp;
                PMKSControl.SettingsUpdated();
            }

        }
    }
}
