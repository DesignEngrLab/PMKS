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
    public class VelocityVector : DisplayVectorBaseShape
    {
        public VelocityVector(joint j, Slider timeSlider, Simulator pmks, double factor, double strokeThickness, double xOffset, double yOffset, JointBaseShape displayJoint, JointData jData)
            : base(factor, strokeThickness, xOffset, yOffset)
        {
            Data = new LineGeometry();
            Stroke = new SolidColorBrush(Colors.Brown);
 
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
            SetBinding(XLengthProperty, binding);

            binding = new Binding
            {
                Source = timeSlider,
                Mode = BindingMode.OneWay,
                Path = new PropertyPath(RangeBase.ValueProperty),
                Converter = new TimeToJointParameterConverter(j, JointState.YVelocity, pmks)
            };
            SetBinding(YLengthProperty, binding);

        }
        
    }
}
