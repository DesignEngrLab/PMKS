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
        public MainPage main { private get; set; }
        public GlobalSettings()
        {
            InitializeComponent();
            ErrorCheckBox.IsChecked = true;
        }

        private void lostFocus(object sender, RoutedEventArgs e)
        {
            main.ParseData();
        }

        private void onLoad(object sender, RoutedEventArgs e)
        {
            main = (MainPage)Application.Current.RootVisual;
        }

        private void radioChanged(object sender, RoutedEventArgs e)
        {
            main.ParseData();
        }
    }

}
