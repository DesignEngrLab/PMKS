using System.Collections.Generic;
using System.Windows;

namespace PMKS_Silverlight_App
{
    public enum AngleType
    {
        Radians, Degrees
    }
    public enum LengthType
    {
        mm, inches
    }
    public static class DisplayConstants
    {
        public const double Buffer = 50.0;
        public const double PenThicknessRatio = 0.001;
        public const double VelocityLengthRatio = 0.3;
        public const double AccelLengthRatio = 0.5;
    }
    public class JointTypeProvider
    {
        public List<string> jointTypeList
        {
            get { return new List<string> { "R (pin joint)", "P (sliding block)", "RP (pin in slot)", "G (gear teeth)" }; }
        }
    }
}