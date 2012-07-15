﻿using System;
using System.Globalization;
using System.Windows.Data;

namespace PMKS_Silverlight_App
{
    public class BooleanToAngleTypeConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            Boolean temp;
            if ((Boolean)value) return AngleType.Radians;
            else return AngleType.Degrees;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return ((AngleType) value==AngleType.Radians);
        }

    }
}