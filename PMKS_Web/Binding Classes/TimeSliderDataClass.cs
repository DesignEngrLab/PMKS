using System;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using System.ComponentModel;

namespace Binding_Classes
{
    public class TimeSliderDataClass: INotifyPropertyChanged
    {
        private double _val;
        public double Val
        {
            get 
            {
                return _val;
            }
            set 
            {
                if (_val != value)
                {
                    _val = value;
                    RaisePropertyChanged("Val");
                }
            }
        }

        private int _numberOfPoints;
        public int NumberOfPoints
        {
            get
            {
                return _numberOfPoints;
            }
            set
            {
                if (_numberOfPoints != value)
                {
                    _numberOfPoints = value;
                    RaisePropertyChanged("NumberOfPoints");
                }
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;
        void RaisePropertyChanged(string propertyName)
        {
            var handler = PropertyChanged;
            if (handler != null)
            {
                handler(this, new PropertyChangedEventArgs(propertyName));
            }
        } 
    }
}
