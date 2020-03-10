using System;
using System.Globalization;
using System.Windows;
using System.Windows.Data;

namespace PMKS_Silverlight_App
{
    public class TextToAngleConverter : IValueConverter
    {
        private MainPage main;
        public TextToAngleConverter(MainPage main)
        {
            this.main = main;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var angleType = main.AngleUnits;

            var val = value.ToString();
            double Value;
            if (string.IsNullOrWhiteSpace(val) || !Double.TryParse(val, out Value))
                return (double)parameter;
            //if (angleType == AngleType.Radians)
            return Value;
            //else
            //    return Value / DisplayConstants.RadiansToDegrees;
        }

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var angleType = main.AngleUnits;

            //if (angleType == AngleType.Radians)
                return ((double)value).ToString("F");
           // else return (((double)value) * DisplayConstants.RadiansToDegrees).ToString("F");

        }

    }
}