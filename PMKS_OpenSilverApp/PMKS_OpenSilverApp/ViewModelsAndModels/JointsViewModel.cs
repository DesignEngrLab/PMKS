using System.Collections.ObjectModel;
using System.Windows;

namespace PMKS_Silverlight_App
{
    public class JointsViewModel : DependencyObject
    {
        public static readonly DependencyProperty DataCollectionProperty
            = DependencyProperty.Register("Data",
                                          typeof(ObservableCollection<JointData>), typeof(JointsViewModel),
                                          new PropertyMetadata(null));

        public ObservableCollection<JointData> Data
        {
            get { return (ObservableCollection<JointData>)GetValue(DataCollectionProperty); }
            set { SetValue(DataCollectionProperty, value); }
        }

        public JointsViewModel()
        {
            Data = new ObservableCollection<JointData>()
                {
                    new JointData {JointTypeString = "R (pin joint)", XPos = "0.0", YPos = "0.0", LinkNames = "ground, input",DrivingInput = true},
                    new JointData {JointTypeString = "R (pin joint)", XPos = "25.0", YPos = "0.0", LinkNames = "input"}
                };
        }
    }
}

