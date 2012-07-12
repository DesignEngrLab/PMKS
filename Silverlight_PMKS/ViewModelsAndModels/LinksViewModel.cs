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
    public class LinksViewModel : ViewModelBase
    {
        ObservableCollection<LinkData> data;

        public LinksViewModel()
        {
            Data = new ObservableCollection<LinkData>()
            {
                new LinkData {Name="ground", Visible=true},
                new LinkData()
            };
        }

        public ObservableCollection<LinkData> Data
        {
            get
            {
                return data;
            }
            set
            {
                data = value;
                OnPropertyChanged("Data");
            }
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

