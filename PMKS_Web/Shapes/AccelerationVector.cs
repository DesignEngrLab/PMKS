using System;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Ink;
using PlanarMechanismSimulator;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class AccelerationVector : Path
    {
        #region Fields
        private readonly double yOffset;
        private readonly double xOffset;
        private readonly double factor;
        #endregion

        #region Dependency Properties

        public static readonly DependencyProperty XStartProperty
            = DependencyProperty.Register("XStart",
                                          typeof(double), typeof(AccelerationVector),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double XStart
        {
            get { return (double)GetValue(XStartProperty); }
            set { SetValue(XStartProperty, value); }
        }
        public static readonly DependencyProperty YStartProperty
            = DependencyProperty.Register("YStart",
                                          typeof(double), typeof(AccelerationVector),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double YStart
        {
            get { return (double)GetValue(YStartProperty); }
            set { SetValue(YStartProperty, value); }
        }

        public static readonly DependencyProperty XAccelerationLengthProperty
            = DependencyProperty.Register("XAccelerationLength",
                                          typeof(double), typeof(AccelerationVector),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double XAccelerationLength
        {
            get { return (double)GetValue(XAccelerationLengthProperty); }
            set { SetValue(XAccelerationLengthProperty, value); }
        }
        public static readonly DependencyProperty YAccelerationLengthProperty
            = DependencyProperty.Register("YAccelerationLength",
                                          typeof(double), typeof(AccelerationVector),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double YAccelerationLength
        {
            get { return (double)GetValue(YAccelerationLengthProperty); }
            set { SetValue(YAccelerationLengthProperty, value); }
        }

        #endregion


        public AccelerationVector(joint j, Slider timeSlider, Simulator pmks, double factor, double strokeThickness, double xOffset, double yOffset, JointBaseShape displayJoint, JointData jData)
        {
            Height = Width = 999999;
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.factor = factor;
            Data = new LineGeometry();
            Stroke = new SolidColorBrush(Colors.Orange);
            StrokeThickness = strokeThickness;

            var binding = new Binding
            {
                Source = jData,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(JointData.AccelVisibleProperty),
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
                Converter = new TimeToJointParameterConverter(j, JointState.XAcceleration, pmks)
            };
            SetBinding(XAccelerationLengthProperty, binding);

            binding = new Binding
            {
                Source = timeSlider,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(RangeBase.ValueProperty),
                Converter = new TimeToJointParameterConverter(j, JointState.YAcceleration, pmks)
            };
            SetBinding(YAccelerationLengthProperty, binding);

        }


        private static void OnTimeChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            var vector = ((AccelerationVector)d);
            var xStart = vector.XStart + vector.xOffset;
            var yStart = vector.YStart + vector.yOffset;
            ((LineGeometry)vector.Data).StartPoint = new Point(xStart, yStart);
            ((LineGeometry)vector.Data).EndPoint = new Point(xStart + vector.factor * vector.XAccelerationLength, yStart + vector.factor * vector.YAccelerationLength);
            //vector.Width = vector.factor*Math.Abs(vector.XAccelerationLength);
            //vector.Height = vector.factor*Math.Abs(vector.YAccelerationLength);
        }
    }
}
