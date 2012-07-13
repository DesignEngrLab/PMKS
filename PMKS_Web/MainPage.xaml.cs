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
    public partial class MainPage : UserControl
    {
        public MainPage()
        {
            InitializeComponent();
        }

        private void MainPage_Loaded_1(object sender, RoutedEventArgs e)
        {
            JointsInfo =(JointsViewModel)jointInputTable.Resources["JointsInfo"];

            PMKSControl.BigGrid = BigGrid;
        }

        private void LayoutRoot_Drop_1(object sender, DragEventArgs e)
        {
            var t = e.OriginalSource;
        }


    }
}
