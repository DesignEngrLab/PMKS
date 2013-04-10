using System;
using System.Collections.Generic;
using System.Globalization;
using System.Windows.Data;
using PMKS_Silverlight_App;
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public enum LinkState
    {
        Angle,
        Velocity,
        Acceleration
    };

    public class TimeToLinkParameterConverter : TimeToParameterBaseConverter
    {
        public TimeToLinkParameterConverter(link l, JointState linkState, Simulator pmks)
            : base(pmks)
        {
            ColIndex = (int)linkState;
            RowIndex = pmks.AllLinks.IndexOf(l);
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
