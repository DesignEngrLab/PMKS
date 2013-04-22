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
    public class PJointShape : DynamicJointBaseShape
    {
        private readonly double h;
        private readonly double w;
        private readonly link fixedLink;

        public PJointShape(joint j, link fixedLink, Slider timeSlider, Simulator pmks, double radius, double strokeThickness, double xOffset, double yOffset)
       :base(j,timeSlider,pmks,radius,strokeThickness,xOffset,yOffset)
        {
            this.fixedLink = fixedLink;
            w = radius*Math.Sqrt(DisplayConstants.SliderRectangleAspectRatio);
            h = radius/Math.Sqrt(DisplayConstants.SliderRectangleAspectRatio);
            Data = new RectangleGeometry
            {
                    Rect = new Rect(new Point(w,h),new Point(2*w,2*h) )
                };

            RenderTransform = new TranslateTransform
            {
                X = Coordinates[0] + xOffset - 1.5*w,
                Y = Coordinates[1] + yOffset - 1.5*h
            };
        }

        public override void Redraw()
        {
            if (Coordinates == null) return;
            RenderTransform = new TranslateTransform
            {
                X = Coordinates[0] + xOffset - 1.5 * w,
                Y = Coordinates[1] + yOffset - 1.5 * h
            };
        }

        internal override void ClearBindings()
        {
            ClearValue(CoordinatesProperty);
        }
    }
}
