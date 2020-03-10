using System;
using System.Collections.Generic;
using System.Globalization;
using System.Windows.Data;
using PMKS;

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
        protected double offsetSlideAngle;
        protected int slideLinkIndex;

        public TimeToJointParameterConverter(Joint j, StateVariableType jointState, Simulator pmks)
        {
            stateVariableTypeIndex = (int)jointState;
            JointIndex = pmks.Joints.IndexOf(j);
            if (j.TypeOfJoint == JointType.P || j.TypeOfJoint == JointType.RP)
            {
                includeAngle = true;
                offsetSlideAngle = j.OffsetSlideAngle;
                slideLinkIndex = pmks.Links.IndexOf(j.Link1);
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
                var angle =DisplayConstants.RadiansToDegrees*(offsetSlideAngle + pmks.FindLinkAngleAtTime(currentTime, slideLinkIndex));
                result = new[] {result[0], result[1], angle};
            }
            return result;

        }

    }
}
