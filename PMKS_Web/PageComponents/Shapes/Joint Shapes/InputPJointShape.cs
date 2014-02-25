using System;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public class InputPJointShape : FixedJointBaseShape
    {
        public InputPJointShape(double jointSize, double strokeThickness, double xPosition, double yPosition, double angle, bool isGround)
            : base(jointSize, strokeThickness, xPosition, yPosition, isGround, false)
        {
            var w = DisplayConstants.PJointSizeIncrease * jointSize * DisplayConstants.SliderRectangleAspectRatioSqareRoot;
            var h = DisplayConstants.PJointSizeIncrease * jointSize / DisplayConstants.SliderRectangleAspectRatioSqareRoot;
            Data = new RectangleGeometry
            {
                Rect = new Rect(new Point(-w / 2,-h / 2), new Point(w/2, h/2)),
                Transform = new CompositeTransform
                {
                    Rotation = DisplayConstants.RadiansToDegrees*angle,
                    TranslateX = xPosition,
                    TranslateY = yPosition  
                }
            };
            Width = Height = DisplayConstants.UnCroppedDimension;
            Fill = null;
        }

    }
}
