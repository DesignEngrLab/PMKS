using System;
using System.Collections.Generic;
using System.Globalization;
using System.Windows.Data;
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public enum StateVariableType
    {
        Position,
        Velocity,
        Acceleration
    };

    public class TimeToJointParameterConverter : IValueConverter
    {
        protected  int stateVariableTypeIndex;
        protected  int JointIndex;
        protected  Simulator pmks;
        protected Boolean includeAngle;
        protected double initSlideAngle;
        protected int slideLinkIndex;

        public TimeToJointParameterConverter(joint j, StateVariableType jointState, Simulator pmks)
        {
            stateVariableTypeIndex = (int)jointState;
            JointIndex = pmks.AllJoints.IndexOf(j);
            if (j.jointType == JointTypes.P || j.jointType == JointTypes.RP)
            {
                includeAngle = true;
                initSlideAngle = j.InitSlideAngle;
                slideLinkIndex = pmks.AllLinks.IndexOf(j.Link1);
            }
            this.pmks = pmks;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }


        public virtual object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var currentTime = (double)value;
            double[] result;
            switch (stateVariableTypeIndex)
            {
                case 0: result = pmks.FindJointPositionAtTime(currentTime, JointIndex);
                    break;
                case 1: result = pmks.FindJointVelocityAtTime(currentTime, JointIndex);
                    break;
                default: result = pmks.FindJointAccelerationAtTime(currentTime, JointIndex);
                    break;
            }
            if (includeAngle && stateVariableTypeIndex==0)
            {
                var angle =DisplayConstants.RadiansToDegrees*(initSlideAngle + pmks.FindLinkAngleAtTime(currentTime, slideLinkIndex));
                result = new[] {result[0], result[1], angle};
            }
            return result;

        }

    }
}
