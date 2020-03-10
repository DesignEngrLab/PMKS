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
using Silverlight_PMKS;

namespace PMKS_Silverlight_App
{
    public partial class GlobalSettings : UserControl
    {

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
            if (App.main == null) return;
            App.main.ParseData(true);
            foreach (var jointData in App.main.JointsInfo.Data)
                jointData.RefreshTablePositions();
        }


        internal void ResetToDefault()
        {
            App.main.Speed = DisplayConstants.DefaultSpeed;
            App.main.Error = DisplayConstants.DefaultError;
            App.main.AngleIncrement = DisplayConstants.DefaultAngleInc;
            App.main.AngleUnits = AngleType.Degrees;
            App.main.LengthUnits = LengthType.mm;
            App.main.AnalysisStep = AnalysisType.error;
            ErrorCheckBox.IsChecked = MetricCheckBox.IsChecked = DegreesCheckBox.IsChecked = true;
        }

        private void TexBox_KeyUp(object sender, KeyEventArgs e)
        {
            var textBox = (TextBox)sender;
            if (e.Key == Key.Escape)
            {
                textBox.Text = "";
            }
            if (e.Key == Key.Down || e.Key == Key.End || e.Key == Key.Enter || e.Key == Key.PageDown || e.Key == Key.PageUp || e.Key == Key.Right || e.Key == Key.Space || e.Key == Key.Tab || e.Key == Key.Up)
            {
                BindingExpression be = textBox.GetBindingExpression(TextBox.TextProperty);
                if (be != null) be.UpdateSource();
            }

        }
    }

}
