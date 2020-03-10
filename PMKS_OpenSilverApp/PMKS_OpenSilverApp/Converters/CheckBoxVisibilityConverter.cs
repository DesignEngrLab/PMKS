using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Data;

namespace PMKS_Silverlight_App
{
    class CheckBoxVisibilityConverter : IValueConverter
    {
        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            bool? result = null;
            if ((Visibility)value == Visibility.Visible)  result = true;
            else if ((Visibility)value == Visibility.Collapsed)  result= false;
            return result;
        }

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if ((Boolean)value) return 1.0;
            else return 0.0;
        }

    }
}