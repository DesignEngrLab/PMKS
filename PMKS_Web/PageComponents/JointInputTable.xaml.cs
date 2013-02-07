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
        //angle is useless for r joints
        private const string RJOINT = "r";
        //angle is required for p and rp joints
        private const string PJOINT = "p";
        private const string RPJOINT = "rp";
        private const string TYPEOFJOINTCOLUMN= "type of joint";

        public MainPage main { private get; set; }
        public JointInputTable()
        {
            InitializeComponent();
        }
        private void dataGrid_RowEditEnded(object sender, DataGridRowEditEndedEventArgs e)
        {
            //main.editButtons.AddButton_Click(null, null);
        }

        private void dataGrid_CellEditEnded(object sender, DataGridCellEditEndedEventArgs e)
        {
            main.ParseData();
            if (e.Column.DisplayIndex == 0 || e.Column.Header.ToString().ToLower().Equals("links")) // in case if header is changed, we check the display index and vice versa
            {
                DataGridCell cell = e.Column.GetCellContent(e.Row.DataContext).Parent as DataGridCell;
                JointData datacontext = cell.DataContext as JointData;
                if (datacontext.LinkNamesList.Length > 0)
                {
                    main.linkInputTable.UpdateLinksTableAterAdd(datacontext);
                }
                
            }

            if (e.Column.DisplayIndex == 1 || e.Column.Header.ToString().ToLower().Equals(TYPEOFJOINTCOLUMN)) // in case if header is changed, we check the display index and vice versa
            {
                DataGridCell cell = e.Column.GetCellContent(e.Row.DataContext).Parent as DataGridCell;
                JointData datacontext = cell.DataContext as JointData;
                if (datacontext.JointType.ToLower().Equals(RJOINT))
                {
                    //disable the column "angle" here
                }
                else if (datacontext.JointType.ToLower().Equals(PJOINT) || datacontext.JointType.ToLower().Equals(RPJOINT))
                {
                    //enable the column "angle" here
                    //ensure that an angle is entered
                }
                else
                {
                    //enable the column "angle" here
                }
            }
        }

        internal void HighlightMissingAngle(int i)
        {
            //dataGrid.row
            //throw new NotImplementedException();
        }

        private void ClickMe_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            System.Diagnostics.Debug.WriteLine("Click Count: " + e.ClickCount);

            if (e.ClickCount == 2)
            {
                main.editButtons.AddButton_Click(sender, e);
            }

        }

    }
}