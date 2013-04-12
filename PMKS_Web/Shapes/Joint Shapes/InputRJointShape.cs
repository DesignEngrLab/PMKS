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
        public InputRJointShape(double radius, double strokeThickness, double xOffset, double yOffset, bool isGround, bool isFilled)
            : base(radius, strokeThickness, xOffset, yOffset, isGround, isFilled)
        {
            var geom = new GeometryGroup
                {
                    FillRule = FillRule.Nonzero,
                    Children =
                        new GeometryCollection
                            {
                                new EllipseGeometry {Center = new Point(xOffset,yOffset),
                                    RadiusX = radius, RadiusY = radius}
                            }
                };
            if (isGround)
            {
                var triangleSideLength = 2.5 * radius;
                geom.Children.Add(new PathGeometry
                {
                    Figures = new PathFigureCollection
                    {
                        new PathFigure
                        {
                            IsFilled = true,IsClosed = true,
                            StartPoint = new Point(xOffset,yOffset),
                            Segments = new PathSegmentCollection
                            {
                                new PolyLineSegment
                                {
                                    Points = new PointCollection
                                    {
                                        //new Point(-triangleSideLength,-triangleSideLength),
                                        //new Point(triangleSideLength,-triangleSideLength)
                                        new Point(xOffset-triangleSideLength,yOffset-triangleSideLength),
                                        new Point(xOffset+triangleSideLength,yOffset-triangleSideLength)
                                    }}}}}
                });
            }
            Data = geom;
            Width = Height = DisplayConstants.UnCroppedDimension;
            //RenderTransform = new TranslateTransform
            //{
            //    X = xOffset,
            //    Y = yOffset
            //};
        }

    }
}
