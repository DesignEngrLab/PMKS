using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Markup;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using System.Collections.ObjectModel;

namespace PMKS_Silverlight_App
{
    public partial class EditButtons : UserControl
    {
        public MainPage main { private get; set; }
        public EditButtons()
        {
            InitializeComponent();
        }

        private void OpenButton_Click(object sender, RoutedEventArgs e)
        {
            var dialog = new OpenFileDialog
                {
                    Filter = "Text Files (*.txt)|*.txt|Comma-Separated Values (.csv)|*.csv"
                };
            if (dialog.ShowDialog() == true)
                using (var reader = dialog.File.OpenText())
                {
                    List<JointData> jointDataList = null;
                    if (JointData.ConvertTextToData(reader.ReadToEnd(), out jointDataList))
                    {
                        ClearButton_Click(null, null);
                        foreach (var j in jointDataList)
                        {
                            main.JointsInfo.Data.Add(j);
                            main.linkInputTable.UpdateLinksTableAterAdd(j);
                        }
                    }
                }
            main.ParseData();
        }
        private void SaveButton_Click(object sender, RoutedEventArgs e)
        {
            var dialog = new SaveFileDialog()
            {
                Filter = "Text Files (*.txt)|*.txt|Comma-Separated Values (.csv)|*.csv"
            };
            if (dialog.ShowDialog() == true)
                using (var stream = dialog.OpenFile())
                using (var writer = new StreamWriter(stream))
                    writer.Write(JointData.ConvertDataToText(main.JointsInfo.Data));
        }

        internal void AddButton_Click(object sender, RoutedEventArgs e)
        {
            main.JointsInfo.Data.Add(new JointData());
        }
        private void ClearButton_Click(object sender, RoutedEventArgs e)
        {
            main.JointsInfo.Data.Clear();
            main.LinksInfo.Data.Clear();
            main.ParseData();
        }
        private void RemoveButton_Click(object sender, RoutedEventArgs e)
        {
            var jointData = main.JointsInfo.Data;
            var Jointstable = main.jointInputTable.dataGrid;
            if (jointData.Count > 0)
            {
                JointData removedJoint;
                if (Jointstable.SelectedItem == null)
                {
                    removedJoint = jointData[jointData.Count - 1];
                    jointData.RemoveAt(jointData.Count - 1);
                }
                else
                {
                    removedJoint = jointData[Jointstable.SelectedIndex];
                    jointData.RemoveAt(Jointstable.SelectedIndex);
                }
                main.linkInputTable.UpdateLinksTableAfterDeletion(removedJoint.LinkNamesList);
            }
            main.ParseData();
        }


        private void TargetShapeStream_OnTextChanged(object sender, TextChangedEventArgs e)
        {
            main.mainViewer.MainCanvas.Children.Remove(main.mainViewer.TargetPath);
            if (TargetShapeStream.Text.Length != 0 &&
                TargetShapeStream.Text != "Enter Target Shape Stream Here.")
            {
                try
                {
                    main.mainViewer.TargetPath = (System.Windows.Shapes.Path)XamlReader.Load(
                        DisplayConstants.TargetPathStreamFront
                        + TargetShapeStream.Text
                        + DisplayConstants.TargetPathStreamEnd);
                    TargetShapeStream.FontStyle = FontStyles.Normal;
                    TargetShapeStream.Foreground = new SolidColorBrush(Colors.Black);
                    main.mainViewer.TargetPath.RenderTransform
                        = new TranslateTransform
                        {
                            X = main.mainViewer.XOffset,
                            Y = main.mainViewer.YOffset
                        };
                    main.mainViewer.MainCanvas.Children.Add(main.mainViewer.TargetPath);
                    return;
                }
                catch (Exception exc)
                {
                    main.status(exc.ToString());
                }
            }
            if (string.IsNullOrWhiteSpace(TargetShapeStream.Text))
            {
                TargetShapeStream.Text = "Enter Target Shape Stream Here.";
                TargetShapeStream.FontStyle = FontStyles.Italic;
                TargetShapeStream.Foreground = new SolidColorBrush(Colors.Black);
            }
        }

        private void TargetShapeStream_OnMouseEnter(object sender, MouseEventArgs e)
        {
            TargetShapeStream.SelectAll();
        }

    }
}