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
    }
}
