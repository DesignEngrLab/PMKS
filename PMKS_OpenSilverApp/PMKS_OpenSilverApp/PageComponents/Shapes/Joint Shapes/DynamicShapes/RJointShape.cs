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
using PMKS;

namespace PMKS_Silverlight_App
{
    public class RJointShape : DynamicJointBaseShape
    {
        public RJointShape(Joint j, Slider timeSlider, Simulator pmks, double radius, double strokeThickness, double xOffset, double yOffset)
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
                X = Coordinates[0] + xOffset - radius,
                Y = Coordinates[1] + yOffset - radius
            };
        }

        public override void Redraw()
        {
            if (Coordinates == null) return;
            RenderTransform = new TranslateTransform
            {
                X = Coordinates[0] + xOffset - radius,
                Y = Coordinates[1] + yOffset - radius
            };
        }

    }
}
