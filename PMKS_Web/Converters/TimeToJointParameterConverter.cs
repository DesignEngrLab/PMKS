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
    }
}
