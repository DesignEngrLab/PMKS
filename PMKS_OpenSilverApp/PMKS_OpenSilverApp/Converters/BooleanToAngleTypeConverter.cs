using System;
using System.Globalization;
using System.Windows.Data;

namespace PMKS_Silverlight_App
{
    public class BooleanToAngleTypeConverter : IValueConverter
    {
        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if ((Boolean)value) return AngleType.Radians;
            return AngleType.Degrees;
        }

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return ((AngleType)value == AngleType.Radians);
        }

    }
}