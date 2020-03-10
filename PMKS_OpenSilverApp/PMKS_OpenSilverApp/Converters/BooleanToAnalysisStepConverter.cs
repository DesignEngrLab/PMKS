using System;
using System.Globalization;
using System.Windows.Data;

namespace PMKS_Silverlight_App
{
    public class BooleanToAnalysisStepConverter : IValueConverter
    {
        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if ((Boolean)value) return AnalysisType.error;
            else return AnalysisType.fixedDelta;
        }

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return ((AnalysisType)value == AnalysisType.error);
        }

    }
}