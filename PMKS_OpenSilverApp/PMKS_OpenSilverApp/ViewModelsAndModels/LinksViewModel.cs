using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class LinksViewModel : DependencyObject
    {
        public LinksViewModel()
        {
            Data = new ObservableCollection<LinkData>();
        }

        public static readonly DependencyProperty DataCollectionProperty
            = DependencyProperty.Register("Data",
                                          typeof(ObservableCollection<LinkData>), typeof(LinksViewModel),
                                          new PropertyMetadata(null));

        public ObservableCollection<LinkData> Data
        {
            get { return (ObservableCollection<LinkData>)GetValue(DataCollectionProperty); }
            set { SetValue(DataCollectionProperty, value); }
        }

    }
    public class LinkData
    {
        private bool _visible = true;
        public string Name { get; set; }
        public Boolean Visible
        {
            get { return _visible; }
            set { _visible = value; }
        }

    }

}

