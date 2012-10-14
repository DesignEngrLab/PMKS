using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public partial class LinkInputTable : UserControl
    {
        public MainPage main { private get; set; }

        public LinkInputTable()
        {
            InitializeComponent();
            DataContext = new LinksViewModel();
        }

        internal void UpdateLinksTableAfterDeletion(string[] linksToRemove)
        {
            ObservableCollection<JointData> jointData = main.JointsInfo.Data;
            ObservableCollection<LinkData> linkData = main.LinksInfo.Data;
            foreach (string link in linksToRemove)
            {
                if (!linkValid(jointData, link))
                    RemoveLink(linkData, link);
            }

        }

        private bool linkValid( ObservableCollection<JointData> jointData, string link)
        {
            bool found = false;
                int index;
                for (index = 0; index < jointData.Count && !found; index++)
                    found = jointData[index].LinkNamesList.Contains(link);
            return found;
        }

        private void RemoveLink(ObservableCollection<LinkData> LinksData, string link)
        {
            for (int index = 0; index < LinksData.Count; index++)
                if (LinksData[index].Name.Equals(link))
                {
                    LinksData.RemoveAt(index);
                    return;
                }
        }
        
        internal void UpdateLinksTableAterAdd(JointData joint)
        {
            ObservableCollection<LinkData> linkData = main.LinksInfo.Data;
            ObservableCollection<JointData> jointData = main.JointsInfo.Data;
            int index = 0;
            while (index < linkData.Count)
            {
                if (!linkValid(jointData, linkData[index].Name))
                    RemoveLink(linkData, linkData[index].Name);
                else
                    index++;
            }

            foreach (string link in joint.LinkNamesList)
                AddLink(linkData, link);
        }

        private void AddLink(ObservableCollection<LinkData> LinksData, string link)
        {
            for (int index = 0; index < LinksData.Count; index++)
                if (LinksData[index].Name.Equals(link))
                    return;
            LinksData.Add(new LinkData { Name = link, Visible = true });
        }

    }

}