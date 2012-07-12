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
    public partial class JointInputTable : UserControl
    {
        public JointInputTable()
        {
            InitializeComponent();
        }
        private void JointInputTable_Loaded_1(object sender, RoutedEventArgs e)
        {
            PMKSControl.JointsInfo = (JointsViewModel) Resources["JointsInfo"];

        }
    }
}