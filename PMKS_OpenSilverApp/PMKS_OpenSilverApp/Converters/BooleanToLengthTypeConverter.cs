using System;
using System.Globalization;
using System.Windows.Data;

namespace PMKS_Silverlight_App
{
    public class BooleanToLengthTypeConverter : IValueConverter
    {
        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if ((Boolean)value) return LengthType.mm;
            else return LengthType.inches;
        }

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return ((LengthType)value == LengthType.mm);
        }

    }
}