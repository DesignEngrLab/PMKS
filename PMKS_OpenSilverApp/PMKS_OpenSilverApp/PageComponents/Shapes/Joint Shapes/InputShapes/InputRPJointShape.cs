using System;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class InputRPJointShape : InputJointBaseShape
    {

        public InputRPJointShape(double jointSize, double strokeThickness, double xPosition, double yPosition, double xAxisOffset,
            double yAxisOffset, double angle, bool isGround, int jointNum, JointData jointData)
            : base(2 * jointSize , 2 * jointSize, strokeThickness, xPosition,
            yPosition, xAxisOffset, yAxisOffset, angle, "PMoveArrows", "PRotateArrows", jointData)
        {

            jointShape = new Ellipse
           {
               Width = 2 * jointSize,
               Height = 2 * jointSize,
               Fill = new SolidColorBrush(Colors.Transparent),
               Stroke = new SolidColorBrush(Colors.Black),
               StrokeThickness = strokeThickness,
               RenderTransformOrigin = new Point(0.5, 0.5),
               RenderTransform = new TranslateTransform
               {
                   X = -jointSize,
                   Y = -jointSize
               }
           };
            Children.Add(jointShape);
        }

    }
}
