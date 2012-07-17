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
        public MainPage main { private get; set; }
        public JointInputTable()
        {
            InitializeComponent();
        }
        private void dataGrid_RowEditEnded(object sender, DataGridRowEditEndedEventArgs e)
        {
            main.editButtons.AddButton_Click(null, null);
        }

        private void dataGrid_CellEditEnded(object sender, DataGridCellEditEndedEventArgs e)
        {
            main.ParseData();
        }
    }
}