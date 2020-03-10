using System;
using System.Linq;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;
using PMKS;
using Point = System.Windows.Point;

namespace PMKS_Silverlight_App
{
    public class LinkShape : Path
    {
        #region Fields
        public readonly double MinimumBufferRadius;
        private readonly List<Point> cvxCenters;
        public Link thisLink { get; private set; }
        public int linkNum { get; private set; }
        private Joint fixedJoint;
        private int updatedStateVars = 0;
        private GeometryGroup slideBorders, slideHoles;
        #endregion

        #region Constructor

        public LinkShape(int linkNum, Link thisLink, double xOffset, double yOffset, double strokeThickness, double jointSize, Slider bufferRadiusSlider,
            double startingBufferRadius)
        {
            this.linkNum = linkNum;
            this.thisLink = thisLink;
            Name = thisLink.name;
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
            var centers = new List<Point>();
            foreach (var j in thisLink.joints)
            {
                if (j.SlidingWithRespectTo(thisLink))
                {                      
                    var slideAngle = j.SlideAngleInitial;
                    var dx = DisplayConstants.InitialSlidingJointLengthMultiplier * startingBufferRadius *
                             Math.Cos(slideAngle);
                    var dy = DisplayConstants.InitialSlidingJointLengthMultiplier * startingBufferRadius *
                             Math.Sin(slideAngle);
                    centers.Add(new Point(j.xInitial + xOffset + dx, j.yInitial + yOffset + dy));
                    centers.Add(new Point(j.xInitial + xOffset - dx, j.yInitial + yOffset - dy));

                    if (slideHoles == null)
                    {
                        slideHoles = new GeometryGroup { FillRule = FillRule.Nonzero, Children = new GeometryCollection() };
                        slideBorders = new GeometryGroup { FillRule = FillRule.Nonzero, Children = new GeometryCollection() };
                    }
                    if (j.TypeOfJoint == JointType.P)
                        slideHoles.Children.Add(SlideShapeMaker.MakePSlotHole(j, thisLink, xOffset, yOffset, jointSize, startingBufferRadius));
                    else slideHoles.Children.Add(SlideShapeMaker.MakeRPSlotHole(j, thisLink, xOffset, yOffset, jointSize, startingBufferRadius));
                    slideBorders.Children.Add(SlideShapeMaker.MakePSlotBorder(j, thisLink, xOffset, yOffset, jointSize, startingBufferRadius));

                }
                else centers.Add(new Point(j.xInitial + xOffset, j.yInitial + yOffset));
            }
            if (centers.Count == 1)
            {
                MinimumBufferRadius = DisplayConstants.SingleJointLinkRadiusMultipler * startingBufferRadius;
                Opacity = DisplayConstants.LinkFillOpacityForOneJointLinks;
                cvxCenters = centers;
            }
            else
            {
                MinimumBufferRadius = 0.0;
                cvxCenters = MIConvexHull.Find(centers);
            }
            // the next line can be removed one the binding is established.
            BufferRadius = startingBufferRadius;
            //var binding = new Binding
            //   {
            //       Source = bufferRadiusSlider,
            //       Mode = BindingMode.TwoWay,
            //       Path = new PropertyPath(RangeBase.ValueProperty),
            //   };
            //SetBinding(BufferRadiusProperty, binding);
            Data = RedrawWithNewBufferRadius();
        }
        #endregion
        private static void OnRadiusChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            ((LinkShape)d).RedrawWithNewBufferRadius();
        }
        private Geometry RedrawWithNewBufferRadius()
        {
            if (BufferRadius < MinimumBufferRadius) BufferRadius = MinimumBufferRadius;
            if (cvxCenters.Count == 1)
            {
                Stroke.Opacity = 0.0;
                return new EllipseGeometry
                      {
                          Center = cvxCenters[0],
                          RadiusX = BufferRadius,
                          RadiusY = BufferRadius
                      };
            }
            Point start;
            var segments = new PathSegmentCollection();

            if (BufferRadius <= 0)
            {
                start = cvxCenters[0];
                var pointCollection = new PointCollection();
                for (int i = 1; i < cvxCenters.Count - 1; i++) pointCollection.Add(cvxCenters[i]);
                segments.Add(new PolyLineSegment { Points = pointCollection });
            }
            else
            {
                start = findNextPoint(cvxCenters[cvxCenters.Count - 1], cvxCenters[0], BufferRadius);
                var size = new Size(BufferRadius, BufferRadius);

                for (int i = 0; i < cvxCenters.Count - 1; i++)
                {
                    segments.Add(new ArcSegment
                    {
                        IsLargeArc = false,
                        Point = findThisPoint(cvxCenters[i], cvxCenters[i + 1], BufferRadius),
                        SweepDirection = SweepDirection.Clockwise,
                        Size = size
                    });
                    segments.Add(new LineSegment { Point = findNextPoint(cvxCenters[i], cvxCenters[i + 1], BufferRadius) });
                }
                segments.Add(new ArcSegment
                {
                    IsLargeArc = false,
                    Point = findThisPoint(cvxCenters[cvxCenters.Count - 1], cvxCenters[0], BufferRadius),
                    SweepDirection = SweepDirection.Clockwise,
                    Size = size
                });
            }
            var basicBodyGeometry = new PathGeometry
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
            if (slideHoles == null)
                return basicBodyGeometry;
            slideBorders.Children.Add(basicBodyGeometry);
            return new GeometryGroup
            {
                FillRule = FillRule.EvenOdd,
                Children = new GeometryCollection { slideBorders, slideHoles }
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
            if (mag == 0) return thisPt;
            vX /= mag;
            vY /= mag;
            return new Point(radius * vX, radius * vY);
        }



        public void SetBindings(Slider timeSlider, Simulator pmks, double xOffset, double yOffset)
        {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            fixedJoint = thisLink.joints.FirstOrDefault(j => j.IsGround && j.FixedWithRespectTo(thisLink));
            if (fixedJoint == null) fixedJoint = thisLink.joints.FirstOrDefault(j => j.FixedWithRespectTo(thisLink));
            if (fixedJoint == null) throw new Exception("Cannot display links that lack a fixed joint.");
            xFixedJoint = fixedJoint.xInitial + xOffset;
            yFixedJoint = fixedJoint.yInitial + yOffset;
            startingAngle = thisLink.AngleInitial;
            var binding = new Binding
              {
                  Source = timeSlider,
                  Mode = BindingMode.OneWay,
                  Path = new PropertyPath(RangeBase.ValueProperty),
                  Converter = new TimeToLinkParameterConverter(thisLink, fixedJoint, StateVariableType.Position, pmks)
              };
            SetBinding(CoordinatesProperty, binding);

        }

        internal void ClearBindings()
        {
            ClearValue(CoordinatesProperty);
            ClearValue(BufferRadiusProperty);
        }
        #region Dependency Properties
        public static readonly DependencyProperty CoordinatesProperty
            = DependencyProperty.Register("Coordinates",
                                          typeof(double[]), typeof(LinkShape),
                                          new PropertyMetadata(null, OnTimeChanged));

        public double[] Coordinates
        {
            get { return (double[])GetValue(CoordinatesProperty); }
            set { SetValue(CoordinatesProperty, value); }
        }
        public static readonly DependencyProperty BufferRadiusProperty
            = DependencyProperty.Register("BufferRadius",
                                          typeof(double), typeof(LinkShape),
                                          new PropertyMetadata(double.NaN, OnRadiusChanged));
        public double BufferRadius { get; set; }
        //{
        //    get { return (double)GetValue(BufferRadiusProperty); }
        //    set { SetValue(BufferRadiusProperty, value); }
        //}
        #endregion

        private double xFixedJoint, yFixedJoint, startingAngle;
        private double yOffset;
        private double xOffset;
        private static void OnTimeChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            var linkShape = ((LinkShape)d);
            if (linkShape.Coordinates == null || linkShape.Coordinates.Contains(double.NaN)) return;
            linkShape.RenderTransform = new TransformGroup
            {
                Children = new TransformCollection
                {
                    new TranslateTransform{ X=-linkShape.xFixedJoint, Y = -linkShape.yFixedJoint },
                    new RotateTransform{ Angle = DisplayConstants.RadiansToDegrees * (linkShape.Coordinates[2] - linkShape.startingAngle) },
                    new TranslateTransform{ X = linkShape.Coordinates[0] + linkShape.xOffset, Y = linkShape.Coordinates[1] + linkShape.yOffset }
                }
            };
        }


    }

}
