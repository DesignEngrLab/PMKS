using System;
using System.Net;
using System.Windows;
using System.Windows.Controls;
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
    public class JointBaseShape : Path
    {
        public static readonly DependencyProperty XCoordProperty
            = DependencyProperty.Register("XCoord",
                                          typeof(double), typeof(JointBaseShape), new PropertyMetadata(double.NaN));

        public double XCoord
        {
            get { return (double)GetValue(XCoordProperty); }
            set
            {
                SetValue(XCoordProperty, value);
                _GeometryTransform = new TranslateTransform { X = XCoord + minX, Y = YCoord + minY };
            }
        }
        public static readonly DependencyProperty YCoordProperty
            = DependencyProperty.Register("YCoord",
                                          typeof(double), typeof(JointBaseShape), new PropertyMetadata(double.NaN));

        protected Transform _GeometryTransform;
        private readonly double minY;
        private readonly double minX;

        public double YCoord
        {
            get { return (double)GetValue(YCoordProperty); }
            set
            {
                SetValue(YCoordProperty, value);
                _GeometryTransform = new TranslateTransform { X = XCoord + minX, Y = YCoord + minY };
            }
        }

        public JointBaseShape(joint j, TimeSlider timeSlider, Simulator pmks, double radius, double strokeThickness, double minX, double minY)
        {
            var binding = new Binding
                              {
                                  Source = timeSlider,
                                  Mode = BindingMode.OneWay,
                                  Path = new PropertyPath(TimeSlider.CurrentTimeProperty),
                                  Converter = new TimeToJointParameterConverter(j, JointState.XCoord, pmks)
                              };
            SetBinding(XCoordProperty, binding);

            binding = new Binding
                             {
                                 Source = timeSlider,
                                 Mode = BindingMode.OneWay,
                                 Path = new PropertyPath(TimeSlider.CurrentTimeProperty),
                                 Converter = new TimeToJointParameterConverter(j, JointState.YCoord, pmks)
                             };
            SetBinding(YCoordProperty, binding);
            XCoord = j.xInitial;
            YCoord = j.yInitial;
            this.minX = minX;
            this.minY = minY;
            Data = new EllipseGeometry
                {
                    RadiusX = radius,
                    RadiusY = radius,
                    Transform = new TranslateTransform { X = XCoord + minX, Y = YCoord + minY }
                };
            Stroke = new SolidColorBrush(Colors.Black);
            StrokeThickness = strokeThickness;
        }

        public override Transform GeometryTransform
        {
            get
            {
                return _GeometryTransform;
            }
        }
    }
}
