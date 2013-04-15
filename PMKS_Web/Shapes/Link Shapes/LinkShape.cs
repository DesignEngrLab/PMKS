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
        #region Fields
        public readonly double MinimumBufferRadius;
        private readonly List<Point> cvxCenters;
        private readonly string name;
        private link thisLink;
        private joint fixedJoint;
        private int updatedStateVars = 0;
        #endregion

        #region Constructor
        public LinkShape(int linkNum, string name, List<List<string>> linkIDs, List<string> jointTypes,
                         List<double[]> initPositions, double xOffset, double yOffset, double strokeThickness, Slider bufferRadiusSlider, double startingBufferRadius)
        {
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
            // Height = Width = DisplayConstants.UnCroppedDimension;
            var centers = new List<Point>();
            for (int j = 0; j < linkIDs.Count; j++)
                if (linkIDs[j].Contains(name))
                {
                    var jPoint = new Point(initPositions[j][0] + xOffset, initPositions[j][1] + yOffset);
                    centers.Add(jPoint);
                    // need to add points for sliding joints that are ? distance from the joint along the angle.
                    if (jointTypes[j][0].Equals('p') || jointTypes[j][0].Equals('P') || (jointTypes[j].Length > 1 &&
                        (jointTypes[j][1].Equals('p') || jointTypes[j][1].Equals('P'))))
                    {
                        var dx = DisplayConstants.InitialSlidingJointLengthMultiplier * startingBufferRadius *
                                 Math.Cos(initPositions[j][2]);
                        var dy = DisplayConstants.InitialSlidingJointLengthMultiplier * startingBufferRadius *
                                 Math.Sin(initPositions[j][2]);
                        centers.Add(new Point(jPoint.X + dx, jPoint.Y + dy));
                        centers.Add(new Point(jPoint.X - dx, jPoint.Y - dy));
                    }
                }
            if (centers.Count == 1)
            {
                MinimumBufferRadius = DisplayConstants.SingleJointLinkRadiusMultipler * startingBufferRadius;
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
        private PathGeometry RedrawWithNewBufferRadius()
        {
            if (BufferRadius < MinimumBufferRadius) BufferRadius = MinimumBufferRadius;
            if (cvxCenters.Count == 1)
                Data = new EllipseGeometry
                        {
                            Center = cvxCenters[0],
                            RadiusX = BufferRadius,
                            RadiusY = BufferRadius
                        };
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
            if (mag == 0) return thisPt;
            vX /= mag;
            vY /= mag;
            return new Point(radius * vX, radius * vY);
        }




        internal void SetBindings(Slider timeSlider, Simulator pmks)
        {
            thisLink = pmks.AllLinks.First(l => l.name.Equals(name));
            fixedJoint = thisLink.joints.FirstOrDefault(j => j.isGround && j.FixedWithRespectTo(thisLink));
            if (fixedJoint == null) fixedJoint = thisLink.joints.FirstOrDefault(j => j.FixedWithRespectTo(thisLink));
            if (fixedJoint == null) throw new Exception("Cannot display links that lack a fixed joint.");
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
               Converter = new TimeToLinkParameterConverter(thisLink, LinkState.Angle, pmks)
           };
            SetBinding(AngleProperty, binding);

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
                                          new PropertyMetadata(double.NaN, OnRadiusChanged));
        public double BufferRadius { get; set; }
        //{
        //    get { return (double)GetValue(BufferRadiusProperty); }
        //    set { SetValue(BufferRadiusProperty, value); }
        //}
        #endregion
        private static void OnTimeChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            ((LinkShape)d).MoveLink();
        }

        private double xPrev, yPrev, anglePrev;
        private void MoveLink()
        {
            if (double.IsNaN(XCoord) || double.IsNaN(YCoord) || double.IsNaN(Angle)) return;
            if ((xPrev==XCoord)&&(yPrev==YCoord)&&())
            //if (++updatedStateVars < 3) return;
            //updatedStateVars = 0;
            //RenderTransformOrigin = new Point(0.5,0.5);
            //var cosAngle = Math.Cos(Angle);
            //var sinAngle = Math.Sin(Angle);
            RenderTransform = new CompositeTransform { CenterX = XCoord,CenterY = YCoord, Rotation = Angle};
            //RenderTransform = new MatrixTransform { Matrix = new Matrix { M11 = cosAngle, M12 = -sinAngle, M21 = sinAngle, M22 = cosAngle, OffsetX = XCoord, OffsetY = YCoord } };

        }



    }

}
