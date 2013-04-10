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
    public abstract class JointBaseShape : Path
    {
        public static readonly DependencyProperty XCoordProperty
            = DependencyProperty.Register("XCoord",
                                          typeof(double), typeof(JointBaseShape),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));

        public double XCoord
        {
            get { return (double)GetValue(XCoordProperty); }
            set { SetValue(XCoordProperty, value); }
        }
        public static readonly DependencyProperty YCoordProperty
            = DependencyProperty.Register("YCoord",
                                          typeof(double), typeof(JointBaseShape),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double YCoord
        {
            get { return (double)GetValue(YCoordProperty); }
            set { SetValue(YCoordProperty, value); }
        }
        protected static void OnTimeChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            ((JointBaseShape) d).Redraw();
        }


        protected readonly double yOffset;
        protected readonly double xOffset;
        protected readonly double radius;
        protected abstract void Redraw();

        protected JointBaseShape(joint j, Slider timeSlider, Simulator pmks, double radius, double strokeThickness, double xOffset, double yOffset)

        {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            
            this.radius = radius;

            Stroke = new SolidColorBrush(Colors.Black);
            StrokeThickness = strokeThickness;
            Height = Width = 2 * (radius + strokeThickness);
            var binding = new Binding
                              {
                                  Source = timeSlider,
                                  Mode = BindingMode.OneWay,
                                  Path = new PropertyPath(RangeBase.ValueProperty),
                                  Converter = new TimeToJointParameterConverter(j, JointState.XCoord, pmks)
                              };
            SetBinding(XCoordProperty, binding);

            binding = new Binding
                             {
                                 Source = timeSlider,
                                 Mode = BindingMode.OneWay,
                                 Path = new PropertyPath(RangeBase.ValueProperty),
                                 Converter = new TimeToJointParameterConverter(j, JointState.YCoord, pmks)
                             };
            SetBinding(YCoordProperty, binding);
            RenderTransform = new TranslateTransform
            {
                X = XCoord + xOffset - radius,
                Y = YCoord + yOffset - radius
            };
        }

    }
}
