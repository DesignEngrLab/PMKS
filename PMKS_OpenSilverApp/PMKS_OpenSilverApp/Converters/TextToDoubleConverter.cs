using System;
using System.Globalization;
using System.Windows.Data;

namespace PMKS_Silverlight_App
{


    public class TextToDoubleConverter : IValueConverter
    {
        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var val = value.ToString();
            double Value;
            if (string.IsNullOrWhiteSpace(val) || !Double.TryParse(val, out Value))
                return (double)parameter;
            return Value;
        }

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return ((double) value).ToString("F");
        }

    }
}