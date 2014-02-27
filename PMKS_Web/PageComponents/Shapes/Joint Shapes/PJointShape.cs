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

        public PJointShape(joint j, link fixedLink, Slider timeSlider, Simulator pmks, double radius, double strokeThickness, double xOffset, double yOffset)
            : base(j, timeSlider, pmks, radius, strokeThickness, xOffset, yOffset)
        {
            w = DisplayConstants.PJointSizeIncrease * radius * DisplayConstants.SliderRectangleAspectRatioSqareRoot;
            h = DisplayConstants.PJointSizeIncrease * radius / DisplayConstants.SliderRectangleAspectRatioSqareRoot;
            Data = new RectangleGeometry
            {
                Rect = new Rect(new Point(0, 0), new Point(w, h))
                //Rect = new Rect(new Point(w, h), new Point(2*w, 2*h))
                //Rect = new Rect(new Point(-w, -h), new Point(w, h))
            };

        }

        public override void Redraw()
        {
            if (Coordinates == null) return;

            RenderTransform = new CompositeTransform
            {
                CenterX = 0.5 * w,
                CenterY = 0.5 * h,
                TranslateX = Coordinates[0] + xOffset - 0.5 * w,
                TranslateY = Coordinates[1] + yOffset - 0.5 * h,
                Rotation = Coordinates[2]
            };
        }

        internal override void ClearBindings()
        {
            ClearValue(CoordinatesProperty);
        }
    }
}
