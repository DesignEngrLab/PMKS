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
using System.ComponentModel;
using Binding_Classes;

namespace PMKS_Silverlight_App
{
    public partial class TimeSlider : UserControl
    {
        public MainPage main { private get; set; }

        public static readonly DependencyProperty CurrentTimeProperty
            = DependencyProperty.Register("CurrentTime",
                                          typeof(double), typeof(TimeSlider), new PropertyMetadata(0.0));

        public double CurrentTime
        {
            get { return (double)GetValue(CurrentTimeProperty); }
            set { SetValue(CurrentTimeProperty, value); }
        }
    }
}
