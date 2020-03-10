using System.Collections.Generic;
using System.Linq;
using System.Windows.Controls;
using Silverlight_PMKS;

namespace PMKS_Silverlight_App
{
    public partial class LinkInputTable : UserControl
    {
        public LinkInputTable()
        {
            InitializeComponent();
            //  DataContext = App.main.LinksInfo;
        }


        internal void UpdateLinksTable()
        {
            var linkData = App.main.LinksInfo.Data;
            // make list of unique link names in joints (could be simpler Linq code, but problems exist when jData.LinkNamesList hasn't been initialized
            LinkNamesInJoints =
               App.main.JointsInfo.Data.Where(jData => jData.LinkNamesList != null)
                   .SelectMany(jData => jData.LinkNamesList)
                   .Distinct()
                   .ToList();
            for (int index = linkData.Count - 1; index >= 0; index--)
                if (!LinkNamesInJoints.Contains(linkData[index].Name))
                    linkData.RemoveAt(index);
            var linkNamesInLinkTable = linkData.Select(ld => ld.Name).ToList();
            foreach (string linkName in LinkNamesInJoints)
                if (!linkNamesInLinkTable.Contains(linkName))
                    linkData.Add(new LinkData { Name = linkName, Visible = true });
        }

        public List<string> LinkNamesInJoints { get; private set; }
    }

}