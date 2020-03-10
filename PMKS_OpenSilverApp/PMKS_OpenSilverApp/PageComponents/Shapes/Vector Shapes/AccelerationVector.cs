using System;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Ink;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;
using PMKS;

namespace PMKS_Silverlight_App
{
    public class AccelerationVector : DisplayVectorBaseShape
    {
        public AccelerationVector(Joint j, Slider timeSlider, Simulator pmks, double factor, double strokeThickness, double xOffset, double yOffset, 
            DynamicJointBaseShape displayJoint, JointData jData)
            : base(factor, strokeThickness, xOffset, yOffset)
        {
            Stroke = new SolidColorBrush(Colors.Orange);

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
                Path = new PropertyPath(DynamicJointBaseShape.CoordinatesProperty),
            };
            SetBinding(StartProperty, binding);

            binding = new Binding
            {
                Source = timeSlider,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(RangeBase.ValueProperty),
                Converter = new TimeToJointParameterConverter(j, StateVariableType.Acceleration, pmks)
            };
            SetBinding(EndProperty, binding);


        }

    }
}
