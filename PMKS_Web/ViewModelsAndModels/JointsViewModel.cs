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
                    //new JointData {JointType = "R (pin joint)", XPos = "0.0", YPos = "0.0", LinkNames = "ground, input"},
                    //new JointData {JointType = "r", XPos = "2", YPos = "0.0", Angle="0", LinkNames = "input coupler"},
                    //new JointData {JointType = "R (pin joint)", XPos = "5.0", YPos = "0.0", LinkNames = "coupler output "},
                    //new JointData
                    //    {JointType = "p", XPos = "4.0", YPos = "4.0", LinkNames ="coupler"}
                        
                    //new JointData {JointType = "R (pin joint)", XPos = "0.0", YPos = "0.0", LinkNames = "ground, input"},
                    //new JointData {JointType = "p", XPos = "20", YPos = "5.0", Angle="0",  LinkNames = "input coupler"},
                    //new JointData {JointType = "r", XPos = "20.0", YPos = "0.0",LinkNames = "coupler output "},
                    //new JointData{JointType = "r", XPos = "21.0", YPos = "10", LinkNames ="output ground"}
                    //,new JointData()
                    new JointData {JointType = "R (pin joint)", XPos = "0.0", YPos = "0.0", LinkNames = "ground, input",DrivingInput = true},
                    new JointData {JointType = "R (pin joint)", XPos = "25.0", YPos = "0.0", LinkNames = "input"}
                };
        }
    }
}

