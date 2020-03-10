using System;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class InputPJointShape : InputJointBaseShape
    {

        public InputPJointShape(double jointSize, double strokeThickness, double xPosition, double yPosition, double xAxisOffset,
            double yAxisOffset, double angle, bool isGround, bool isDriver, int jointNum, JointData jointData)
            : base(2 * jointSize * DisplayConstants.SliderRectangleWidthIncrease, 2 * jointSize, strokeThickness, xPosition,
            yPosition, xAxisOffset, yAxisOffset, angle, "PMoveArrows", "PRotateArrows", jointData)
        {

            jointShape = new Rectangle
           {
               Width = 2 * jointSize * DisplayConstants.SliderRectangleWidthIncrease,
               Height = 2 * jointSize,
               Fill = (isGround) ? new SolidColorBrush(Colors.White) : new SolidColorBrush(Colors.Transparent),
               Stroke = (isDriver) ? new SolidColorBrush(Colors.Green) : new SolidColorBrush(Colors.Black),
               StrokeThickness = (isDriver) ? 3 * strokeThickness : strokeThickness,
               RenderTransformOrigin = new Point(0.5, 0.5),
               RenderTransform = new TranslateTransform
               {
                   X = -jointSize * DisplayConstants.SliderRectangleWidthIncrease,
                   Y = -jointSize
               }
           };
            Children.Add(jointShape);
        }

    }
}
