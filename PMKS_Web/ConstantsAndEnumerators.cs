using System;
using System.Collections.Generic;
using System.Windows;

namespace PMKS_Silverlight_App
{
    public enum AngleType
    { Radians, Degrees }
    public enum LengthType
    { mm, inches }
    public enum AnalysisType
    { error, fixedDelta }

    public static class DisplayConstants
    {
        public const double PenThicknessRatio = 1;
        public const double VelocityLengthRatio = 0.3;
        public const double AccelLengthRatio = 0.5;
        public const double TickDistance = 24.0; // one-quarter of an inch
        public const double JointSize = 4.0; // one-sixteenth of an inch
        public const double AxesBuffer = 10; // one inch
        public const double UnCroppedDimension = 999999;
        public const double LinkFillOpacity = 0.5;
        public const double LinkHueMultiplier = 180.0 / Math.PI;
        public const double LinkFillLuminence = 0.8;
        public const double LinkFillSaturation = 0.5;
        public const double LinkStrokeOpacity = 0.75;
        public const double LinkStrokeLuminence = 0.3;
        public const double LinkStrokeSaturation = 0.7;
        public const double DefaultLinkThickness = 8.0;
        public const double InitialSlidingJointLengthMultiplier = 3.0;
        public const double SingleJointLinkRadiusMultipler = 10;
        public const double RadiansToDegrees = 180.0 / Math.PI;
        public const double TotalSize = 2000.0;
        public const double MostNegativeAllowableCoordinate = -TotalSize / 2;
        public const double DeltaChangeInScaleToStaySame = 0.2;
        public const double DefaultBufferMultipler = 0.5;
        public const string TargetPathStreamFront =
            "<Path xmlns=\"http://schemas.microsoft.com/winfx/2006/xaml/presentation\""
            + " Stroke=\"#FFC7C7C7\" StrokeThickness=\"2\"  Data=\"";

        public const string TargetPathStreamEnd = "\"/>";
        public const double SliderRectangleAspectRatio = 1.25;
        public static double MaxZoomIn = 25;
        public static double MaxZoomOut = 0.04;

    }
    public class JointTypeProvider
    {
        public List<string> jointTypeList
        {
            get { return new List<string> { "R (pin joint)"
                , "P (sliding block)", "RP (pin in slot)", "G (gear teeth)" 
            }; }
        }
    }
}