using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public partial class GlobalSettings : UserControl
    {
        public GlobalSettings()
        {
            InitializeComponent();
            // todo:Disable the error approach (which automatically adjusts step size) has yet to be implemented, 
            // we add these two lines.
            ErrorCheckBox.IsEnabled = false;
            AngleCheckBox.IsChecked = true;
            //////////////////////////////////////////////////////////////
        }
    }

}
