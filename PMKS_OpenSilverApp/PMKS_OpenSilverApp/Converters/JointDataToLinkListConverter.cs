using System;
using System.Linq;
using System.Globalization;
using System.Windows.Data;
using System.Collections.Generic;
using System.Collections.ObjectModel;

namespace PMKS_Silverlight_App
{
    class JointDataToLinkListConverter : IValueConverter
    {
        private LinksViewModel LinksInfo;

        public JointDataToLinkListConverter(LinksViewModel LinksInfo)
        {
            this.LinksInfo = LinksInfo;
        }
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            string nameString = "";
            var jointsInfo = value as ObservableCollection<JointData>;
            if (jointsInfo == null) return null;
            foreach (var s in jointsInfo)
                nameString += s.LinkNames + " ";
            var nameList = new List<string>(nameString.Split(',', ' ').Distinct());
            for (int i = LinksInfo.Data.Count - 1; i >= 0; i--)
            {
                var linkName = LinksInfo.Data[i].Name;
                if (nameList.Contains(linkName)) nameList.Remove(linkName);
                else LinksInfo.Data.RemoveAt(i);
            }
            foreach (var s in nameList)
                LinksInfo.Data.Add(new LinkData { Name = s, Visible = true });
            return LinksInfo;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotSupportedException();
        }

    }
}
