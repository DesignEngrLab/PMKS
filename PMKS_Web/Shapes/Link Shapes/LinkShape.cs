using System;
using System.Linq;
using PlanarMechanismSimulator;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class LinkShape : Path
    {
        protected readonly double yOffset;
        protected readonly double xOffset;
        private readonly string name;
        private readonly Point center;

        public LinkShape(int linkNum, string name, List<List<string>> linkIDs, List<string> jointTypes,
                         List<double[]> initPositions,
                         double strokeThickness, double startingBufferRadius, double xOffset, double yOffset)
        {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.name = name;
            Fill = new SolidColorBrush(AHSLtoARGBColor.Convert(DisplayConstants.LinkFillOpacity,
                                                               DisplayConstants.LinkHueMultiplier * linkNum,
                                                               DisplayConstants.LinkFillSaturation,
                                                               DisplayConstants.LinkFillLuminence));
            Stroke = new SolidColorBrush(AHSLtoARGBColor.Convert(DisplayConstants.LinkStrokeOpacity,
                                                                 DisplayConstants.LinkHueMultiplier * linkNum,
                                                                 DisplayConstants.LinkStrokeSaturation,
                                                                 DisplayConstants.LinkStrokeLuminence));
            StrokeThickness = strokeThickness;
            Height = Width = DisplayConstants.UnCroppedDimension;

            var connectedPositions = new PointCollection();
            for (int j = 0; j < linkIDs.Count; j++)
                if (linkIDs[j].Contains(name))
                    connectedPositions.Add(new Point(initPositions[j][0], initPositions[j][1]));
            center = new Point(connectedPositions.Average(n => n.X),
                               connectedPositions.Average(n => n.Y));
            if (connectedPositions.Count == 1)
                Data = new EllipseGeometry
                        {
                            Center = connectedPositions[0],
                            RadiusX = startingBufferRadius,
                            RadiusY = startingBufferRadius
                        };
            else Data = RedrawWithNewBufferRadius(MIConvexHull.Find(connectedPositions), startingBufferRadius);
            RenderTransform = new TranslateTransform {X = xOffset, Y = yOffset};
        }
        private PathGeometry RedrawWithNewBufferRadius(PointCollection points, double radius)
        {
            Point start;
            var segments = new PathSegmentCollection();

            if (radius <= 0)
            {
                start = points[0];
                points.RemoveAt(0);
                segments.Add(new PolyLineSegment { Points = points });
            }
            else
            {
                start = findNextPoint(points[points.Count - 1], points[0], radius);
                var size = new Size(radius, radius);

                for (int i = 0; i < points.Count - 1; i++)
                {
                    segments.Add(new ArcSegment
                    {
                        IsLargeArc = false,
                        Point = findThisPoint(points[i], points[i + 1], radius),
                        SweepDirection = SweepDirection.Clockwise,
                        Size = size
                    });
                    segments.Add(new LineSegment { Point = findNextPoint(points[i], points[i + 1], radius) });
                }
                segments.Add(new ArcSegment
                {
                    IsLargeArc = false,
                    Point = findThisPoint(points[points.Count - 1], points[0], radius),
                    SweepDirection = SweepDirection.Clockwise,
                    Size = size
                });
            }
            return new PathGeometry
                {
                    FillRule = FillRule.Nonzero,
                    Figures = new PathFigureCollection
                        {
                            new PathFigure
                                {
                                    IsClosed = true,
                                    IsFilled = true,
                                    StartPoint = start,
                                    Segments = segments
                                }
                        }
                };
        }

        Point findThisPoint(Point thisPt, Point nextPt, double radius)
        {
            var outVector = makeOutVector(thisPt, nextPt, radius);
            return new Point(thisPt.X + outVector.X, thisPt.Y + outVector.Y);
        }

        Point findNextPoint(Point thisPt, Point nextPt, double radius)
        {
            var outVector = makeOutVector(thisPt, nextPt, radius);
            return new Point(nextPt.X + outVector.X, nextPt.Y + outVector.Y);
        }
        private Point makeOutVector(Point thisPt, Point nextPt, double radius)
        {
            var vX = nextPt.Y - thisPt.Y;
            var vY = thisPt.X - nextPt.X;
            var mag = Math.Sqrt(vX * vX + vY * vY);
            vX /= mag;
            vY /= mag;
            return new Point(radius * vX, radius * vY);
        }




        internal void SetBindings(Slider timeSlider, Slider bufferRadiusSlider, Simulator pmks, link thisJoint, joint fixedJoint)
        {
            var binding = new Binding
            {
                Source = timeSlider,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(RangeBase.ValueProperty),
                Converter = new TimeToJointParameterConverter(fixedJoint, JointState.XCoord, pmks)
            };
            SetBinding(XCoordProperty, binding);

            binding = new Binding
            {
                Source = timeSlider,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(RangeBase.ValueProperty),
                Converter = new TimeToJointParameterConverter(fixedJoint, JointState.YCoord, pmks)
            };
            SetBinding(YCoordProperty, binding);

            binding = new Binding
           {
               Source = timeSlider,
               Mode = BindingMode.OneWay,
               Path = new PropertyPath(RangeBase.ValueProperty),
               Converter = new TimeToLinkParameterConverter(thisJoint, JointState.XCoord, pmks)
           };
            SetBinding(AngleProperty, binding);

            binding = new Binding
            {
                Source = bufferRadiusSlider,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(RangeBase.ValueProperty),
            };
            SetBinding(BufferRadiusProperty, binding);
        }

        internal void ClearBindings()
        {
            ClearValue(XCoordProperty);
            ClearValue(YCoordProperty);
            ClearValue(AngleProperty);
            ClearValue(BufferRadiusProperty);
        }
        #region Dependency Properties
        public static readonly DependencyProperty XCoordProperty
            = DependencyProperty.Register("XCoord",
                                          typeof(double), typeof(LinkShape),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));

        public double XCoord
        {
            get { return (double)GetValue(XCoordProperty); }
            set { SetValue(XCoordProperty, value); }
        }
        public static readonly DependencyProperty YCoordProperty
            = DependencyProperty.Register("YCoord",
                                          typeof(double), typeof(LinkShape),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));

        public double YCoord
        {
            get { return (double)GetValue(YCoordProperty); }
            set { SetValue(YCoordProperty, value); }
        }
        public static readonly DependencyProperty AngleProperty
            = DependencyProperty.Register("Angle",
                                          typeof(double), typeof(LinkShape),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));

        public double Angle
        {
            get { return (double)GetValue(AngleProperty); }
            set { SetValue(AngleProperty, value); }
        }
        public static readonly DependencyProperty BufferRadiusProperty
            = DependencyProperty.Register("BufferRadius",
                                          typeof(double), typeof(LinkShape),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));



        public double BufferRadius
        {
            get { return (double)GetValue(BufferRadiusProperty); }
            set { SetValue(BufferRadiusProperty, value); }
        }
        #endregion
        protected static void OnTimeChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            ((LinkShape)d).Redraw();
        }

        private void Redraw()
        {
            throw new System.NotImplementedException();
        }



    }

}
