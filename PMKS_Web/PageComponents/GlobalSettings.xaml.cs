using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public partial class GlobalSettings : UserControl
    {
        public MainPage main;
        public GlobalSettings()
        {
            InitializeComponent();
            ErrorCheckBox.IsChecked = MetricCheckBox.IsChecked = DegreesCheckBox.IsChecked = true;
        }

        private void ErrorCheckBox_Checked(object sender, RoutedEventArgs e)
        {
            ErrorBox.Foreground = ErrorCheckBox.Foreground = new SolidColorBrush(Colors.Black);
            AngleBox.Foreground = AngleCheckBox.Foreground = new SolidColorBrush(Colors.Gray);
            GlobalSettingChanged();
        }

        private void ErrorCheckBox_Unchecked(object sender, RoutedEventArgs e)
        {
            ErrorBox.Foreground = ErrorCheckBox.Foreground = new SolidColorBrush(Colors.Gray);
            AngleBox.Foreground = AngleCheckBox.Foreground = new SolidColorBrush(Colors.Black);
            GlobalSettingChanged();
        }

        private void GlobalSettingChanged(object sender = null, RoutedEventArgs e = null)
        {
            if (main == null) return;
            main.ParseData(true);
        }

    }

}
