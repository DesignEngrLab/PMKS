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
    public class RJointShape : DynamicJointBaseShape
    {
        public RJointShape(joint j, Slider timeSlider, Simulator pmks, double radius, double strokeThickness, double xOffset, double yOffset)
       :base(j,timeSlider,pmks,radius,strokeThickness,xOffset,yOffset) 
        {
            Data = new EllipseGeometry
                {
                    RadiusX = radius,
                    RadiusY = radius,
                    Transform = new TranslateTransform { X = radius, Y = radius }
                };

            RenderTransform = new TranslateTransform
            {
                X = XCoord + xOffset - radius,
                Y = YCoord + yOffset - radius
            };
        }

        public override void Redraw()
        {
            RenderTransform = new TranslateTransform
            {
                X = XCoord + xOffset - radius,
                Y = YCoord + yOffset - radius
            };
        }

        internal override void ClearBindings()
        {
            ClearValue(XCoordProperty);
            ClearValue(YCoordProperty);
        }
    }
}
