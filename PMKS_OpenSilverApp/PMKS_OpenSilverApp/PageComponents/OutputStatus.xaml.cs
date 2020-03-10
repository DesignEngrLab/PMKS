using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using System.IO;

namespace PMKS_Silverlight_App
{
    public partial class OutputStatus : UserControl
    {
        public OutputStatus()
        {
            InitializeComponent();
        }

        private void clearButton_Click(object sender, RoutedEventArgs e)
        {
            StatusBox.Text = "";
        }

        private void saveAsFileButton_Click(object sender, RoutedEventArgs e)
        {
            SaveFileDialog dialog = new SaveFileDialog();
            dialog.DefaultExt = ".txt";
            bool? result = dialog.ShowDialog();
            if (result == true)
            {
                using (Stream fs = (Stream)dialog.OpenFile())
                {
                    fs.Write(GetBytes(StatusBox.Text), 0, StatusBox.Text.Length);
                    fs.Close();
                }
            }
        }

        private byte[] GetBytes(string str)
        {
            byte[] bytes = new byte[str.Length * sizeof(char)];
            System.Buffer.BlockCopy(str.ToCharArray(), 0, bytes, 0, bytes.Length);
            return bytes;
        }

    }

}