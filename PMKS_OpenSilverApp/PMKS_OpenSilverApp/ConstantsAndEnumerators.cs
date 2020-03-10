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
        public const double AxesBuffer = 6;//24; // one inch
        public const double UnCroppedDimension = 999999;
        public const double LinkFillOpacity = 0.5;
        public const double LinkFillOpacityForOneJointLinks = 0.35;
        public const double LinkHueMultiplier = 180.0 / Math.PI;
        public const double LinkFillLuminence = 0.8;
        public const double LinkFillSaturation = 0.5;
        public const double LinkStrokeOpacity = 0.75;
        public const double LinkStrokeLuminence = 0.3;
        public const double LinkStrokeSaturation = 0.7;
        public const double DefaultBufferRadius = 8.0;
        public const double InitialSlidingJointLengthMultiplier = 1.0;
        public const double SingleJointLinkRadiusMultipler = 7.0;
        public const double RadiansToDegrees = 180.0 / Math.PI;
        public const double DeltaChangeInScaleToStaySame = 0.2;
        public const double DefaultBufferMultipler = 0.5;
        public const string TargetPathStreamFront =
            "<Path xmlns=\"http://schemas.microsoft.com/winfx/2006/xaml/presentation\""
            + " Stroke=\"#AAC7C7C7\" StrokeThickness=\"2\"  Data=\"";

        public const string TargetShapeQueryText = "Enter Target Shape Stream Here.";
        public const string TargetPathStreamEnd = "\"/>";
        public const double SliderRectangleWidthIncrease = 1.5;
        public const double MaxZoomIn = 50;
        public const double MaxZoomOut = 0.02;
        public const double UltimateWindowWidth = 960;
        public const double MaxUnselectedOpacity = 0.7;
        public const double IconIncreaseRadiusFactor = 1.8;
        public const double ZoomStep = 1.05;
        public const double ZoomTimeOnBtn = 0.15;//25;
        public const double ZoomTimeOnKey = 0.0;//1;
        public const double ZoomTimeOnMouseWheel = 0.0;//5;
        public const double ZoomTimeOnRedraw = 0.75;
        public const double PanTimeOnMouseMove = 0.0;
        public const double PanTimeOnArrowKeys = 0.1;
        public const double PanFactorForArrowKeys = 70.0;
        public const double ExtraAxesLengthFactor = 0.0;
        public const double RadiansPerSecToRPM = Math.PI / 30.0;
        public const double DefaultSpeed = 10.0;
        public const double DefaultError = 0.001;
        public const double DefaultAngleInc = 5.0;
        public const double MinAxisPenThickness = 0.012;


    }
    public class JointTypeProvider
    {
        public List<string> jointTypeList
        {
            get
            {
                return new List<string> { "R (pin joint)"
                , "P (sliding block)", "RP (pin in slot)" , "G (gear teeth)" 
            };
            }
        }
    }
}