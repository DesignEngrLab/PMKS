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
            Loaded += JointInputTable_Loaded;
        }

        private void JointInputTable_Loaded(object sender, RoutedEventArgs e)
        {
            this.dataGrid.ItemsSource = new List<JointData>()
                {
                    new JointData {JointType = "R (pin joint)", XPos = "1", YPos = "3.5"},
                    new JointData {JointType ="P (sliding block)", XPos = "3.1", YPos = "37.5"},
                    new JointData {JointType =  "RP (pin in slot)", XPos = "6.1", YPos = "13.5"}
                };
        }

        private void dataGrid_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }
    }

}