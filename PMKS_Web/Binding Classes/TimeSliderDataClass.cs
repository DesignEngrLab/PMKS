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
using PMKS_Silverlight_App;

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
                    RaisePropertyChanged("Val");
                    if (_pathCollection != null)
                        setNewStateFromSliderValue(_val, value);
                    _val = value;
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

        private double _penThick;
        public double PenThick
        {
            get { return _penThick; }
            set { _penThick = value; }
        }

        private void setNewStateFromSliderValue(double oldValue, double newValue)
        {
            int oldIndex = (int)(oldValue/ 10.0 /NumberOfPoints);
            int newIndex = (int)(newValue / 10.0 / NumberOfPoints);
            (PathCollection[oldIndex] as PositionPath).StrokeThickness = _penThick;
            (PathCollection[oldIndex] as PositionPath).Stroke = new SolidColorBrush { Color = Colors.Green };
            (PathCollection[newIndex] as PositionPath).StrokeThickness = _penThick * 2;
            (PathCollection[newIndex] as PositionPath).Stroke = new SolidColorBrush { Color = Colors.Black };
        }

        private UIElementCollection _pathCollection;
        public UIElementCollection PathCollection
        {
            get { return _pathCollection; }
            set { _pathCollection = value; }
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
