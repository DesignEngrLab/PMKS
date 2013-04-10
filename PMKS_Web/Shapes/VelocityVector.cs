using System;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using PlanarMechanismSimulator;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class VelocityVector : Path
    {
        #region Fields
        private readonly double yOffset;
        private readonly double xOffset;
        private readonly double factor;
        #endregion

        #region Dependency Properties

        public static readonly DependencyProperty XStartProperty
            = DependencyProperty.Register("XStart",
                                          typeof(double), typeof(VelocityVector),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double XStart
        {
            get { return (double)GetValue(XStartProperty); }
            set { SetValue(XStartProperty, value); }
        }
        public static readonly DependencyProperty YStartProperty
            = DependencyProperty.Register("YStart",
                                          typeof(double), typeof(VelocityVector),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double YStart
        {
            get { return (double)GetValue(YStartProperty); }
            set { SetValue(YStartProperty, value); }
        }

        public static readonly DependencyProperty XVelocityLengthProperty
            = DependencyProperty.Register("XVelocityLength",
                                          typeof(double), typeof(VelocityVector),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double XVelocityLength
        {
            get { return (double)GetValue(XVelocityLengthProperty); }
            set { SetValue(XVelocityLengthProperty, value); }
        }
        public static readonly DependencyProperty YVelocityLengthProperty
            = DependencyProperty.Register("YVelocityLength",
                                          typeof(double), typeof(VelocityVector),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double YVelocityLength
        {
            get { return (double)GetValue(YVelocityLengthProperty); }
            set { SetValue(YVelocityLengthProperty, value); }
        }

        #endregion


        public VelocityVector(joint j, Slider timeSlider, Simulator pmks, double factor, double strokeThickness, double xOffset, double yOffset, JointBaseShape displayJoint, JointData jData)
        {
            Height = Width = 999999;
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.factor = factor;
            Data = new LineGeometry();
            Stroke = new SolidColorBrush(Colors.Brown);
            
            StrokeThickness = strokeThickness;
            //RenderTransform = new TranslateTransform { X = offsetX, Y = offsetY };

            var binding = new Binding
              {
                  Source = jData,
                  Mode = BindingMode.OneWay,
                  Path = new PropertyPath(JointData.VelVisibleProperty),
                  Converter = new CheckBoxVisibilityConverter()
              };
            SetBinding(OpacityProperty, binding);


            binding = new Binding
            {
                Source = displayJoint,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(JointBaseShape.XCoordProperty),
            };
            SetBinding(XStartProperty, binding);

            binding = new Binding
            {
                Source = displayJoint,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(JointBaseShape.YCoordProperty),
            };
            SetBinding(YStartProperty, binding);
            binding = new Binding
            {
                Source = timeSlider,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(RangeBase.ValueProperty),
                Converter = new TimeToJointParameterConverter(j, JointState.XVelocity, pmks)
            };
            SetBinding(XVelocityLengthProperty, binding);

            binding = new Binding
            {
                Source = timeSlider,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(RangeBase.ValueProperty),
                Converter = new TimeToJointParameterConverter(j, JointState.YVelocity, pmks)
            };
            SetBinding(YVelocityLengthProperty, binding);

        }


        private static void OnTimeChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            var vector = ((VelocityVector)d);
            var xStart = vector.XStart + vector.xOffset;
            var yStart = vector.YStart + vector.yOffset;
            ((LineGeometry)vector.Data).StartPoint = new Point(xStart, yStart);
            ((LineGeometry)vector.Data).EndPoint = new Point(xStart + vector.factor * vector.XVelocityLength, yStart + vector.factor * vector.YVelocityLength);
            //vector.Width =  10000*vector.factor * Math.Abs(vector.XVelocityLength);
            //vector.Height = 10000 * vector.factor * Math.Abs(vector.YVelocityLength);
        }
    }
}
