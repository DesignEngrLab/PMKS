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
using PMKS;

namespace PMKS_Silverlight_App
{
    public abstract class DynamicJointBaseShape : JointBaseShape
    {
        public abstract void Redraw();
        internal void ClearBindings()
        {
            ClearValue(CoordinatesProperty);
        }
        public static readonly DependencyProperty CoordinatesProperty
            = DependencyProperty.Register("Coordinates",
                                          typeof(double[]), typeof(DynamicJointBaseShape),
                                          new PropertyMetadata(null, OnTimeChanged));

        public double[] Coordinates
        {
            get { return (double[])GetValue(CoordinatesProperty); }
            set { SetValue(CoordinatesProperty, value); }
        }
        protected static void OnTimeChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            ((DynamicJointBaseShape)d).Redraw();
        }

        protected DynamicJointBaseShape(Joint j, Slider timeSlider, Simulator pmks, double radius, double strokeThickness, double xOffset, double yOffset)
            : base(radius, strokeThickness, xOffset, yOffset)
        {
            Height = Width = DisplayConstants.UnCroppedDimension;
            var binding = new Binding
                              {
                                  Source = timeSlider,
                                  Mode = BindingMode.OneWay,
                                  Path = new PropertyPath(RangeBase.ValueProperty),
                                  Converter = new TimeToJointParameterConverter(j, StateVariableType.Position, pmks)
                              };
            SetBinding(CoordinatesProperty, binding);

            RenderTransform = new TranslateTransform
            {
                X = Coordinates[0] + xOffset - radius,
                Y = Coordinates[1] + yOffset - radius
            };
            Fill = new SolidColorBrush(Colors.Black);
        }


    }

}
