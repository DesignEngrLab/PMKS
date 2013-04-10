using System;
using System.Collections.Generic;
using System.Globalization;
using System.Windows.Data;
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public enum JointState
    {
        XCoord,
        YCoord,
        XVelocity,
        YVelocity,
        XAcceleration,
        YAcceleration
    };

    public class TimeToJointParameterConverter : TimeToParameterBaseConverter
    {
        public TimeToJointParameterConverter(joint j, JointState jointState, Simulator pmks)
            : base(pmks)
        {
            ColIndex = (int) jointState;
            RowIndex = pmks.AllJoints.IndexOf(j);
        }
        public override object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var currentTime = (double)value;
            SetTimeIndices(currentTime);
            switch (ColIndex)
            {
                case 0:
                case 1:
                    return Simulator.FindPositionatTime(tau, deltaTime,
                        parameters[prevIndex][RowIndex, ColIndex],
                        parameters[nextIndex][RowIndex, ColIndex],
                        parameters[prevIndex][RowIndex, ColIndex + 2],
                        parameters[nextIndex][RowIndex, ColIndex + 2],
                        parameters[prevIndex][RowIndex, ColIndex + 4],
                        parameters[nextIndex][RowIndex, ColIndex + 4]);
                case 2:
                case 3:
                    return Simulator.FindVelocityatTime(tau, deltaTime,
                        parameters[prevIndex][RowIndex, ColIndex],
                        parameters[nextIndex][RowIndex, ColIndex],
                        parameters[prevIndex][RowIndex, ColIndex + 2],
                        parameters[nextIndex][RowIndex, ColIndex + 2]);
                default:
                    return Simulator.FindAccelerationatTime(tau, deltaTime,
                        parameters[prevIndex][RowIndex, ColIndex],
                        parameters[nextIndex][RowIndex, ColIndex]);
            }
        }
    }
}
