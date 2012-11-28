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
    public partial class TimeSlider : UserControl
    {
        public MainPage main
        { get; set; }

        public TimeSlider()
        {
            InitializeComponent();
        }

        private void Slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            Console.WriteLine("Old value is {0}, New value is {1}", e.OldValue, e.NewValue);
        }

        private void Slider_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            //MessageBox.Show(String.Format(" New value is {0}", TimeSliderGUI.Value),  "mouse left up", MessageBoxButton.OKCancel);
        }
    }
}
