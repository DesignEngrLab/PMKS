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
using System.Windows.Shapes;
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public class InputRJointShape : FixedJointBaseShape
    {
        public InputRJointShape(double radius, double strokeThickness, double xPosition, double yPosition, bool isGround)
            : base(radius, strokeThickness, xPosition, yPosition, isGround, false)
        {
            Data = new EllipseGeometry
            {
                Center = new Point(xPosition, yPosition),
                RadiusX = radius,
                RadiusY = radius
            };
            Width = Height = DisplayConstants.UnCroppedDimension;
            //RenderTransform = new TranslateTransform
            //{
            //    X = xOffset,
            //    Y = yOffset
            //};
        }

    }
}
