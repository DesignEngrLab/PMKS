using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class JointsViewModel : ViewModelBase
    {
        ObservableCollection<JointData> data;

        public JointsViewModel()
        {
            Data = new ObservableCollection<JointData>()
            {
                new JointData {JointType = "R (pin joint)", XPos = "0.0", YPos = "0.0", LinkNames="ground, input"},
                new JointData {JointType = "R (pin joint)", XPos = "0.0", YPos = "8.0", LinkNames="input coupler"},
                new JointData {JointType = "R (pin joint)", XPos = "10.0", YPos = "12.0", LinkNames="coupler,output "},
                new JointData {JointType = "R (pin joint)", XPos = "10.0", YPos = "0.0", LinkNames="ground, outpu"},
                new JointData(),new JointData(),new JointData(),new JointData(),new JointData()
            };
        }

        public ObservableCollection<JointData> Data
        {
            get
            {
                return data;
            }
            set
            {
                data = value;
                OnPropertyChanged("Data");
            }
        }

        //internal bool SameTopology(JointsViewModel JointsInfo)
        //{
        //    throw new NotImplementedException();
        //}

        //internal bool SameParameters(JointsViewModel JointsInfo)
        //{
        //    throw new NotImplementedException();
        //}
    }
    public class JointTypeProvider
    {
        public List<string> jointTypeList
        {
            get { return new List<string> { "R (pin joint)", "P (sliding block)", "RP (pin in slot)", "G (gear teeth)" }; }
        }
    }


}

