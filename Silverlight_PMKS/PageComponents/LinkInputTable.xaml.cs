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
    public partial class LinkInputTable : UserControl
    {
        public LinkInputTable()
        {
            InitializeComponent();
            Loaded += LinkInputTable_Loaded;
        }

        private void LinkInputTable_Loaded(object sender, RoutedEventArgs e)
        {
            this.linkDataGrid.ItemsSource = new List<LinkData>()
                {
                    new LinkData {Name = "ground", Visible = true},
                };
        }
    }


    public class LinkData
    {
        public string Name { get; set; }
        public Boolean Visible { get; set; }
    }

}