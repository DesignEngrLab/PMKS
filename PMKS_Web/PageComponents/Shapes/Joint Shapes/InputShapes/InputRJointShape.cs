using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class InputRJointShape : InputJointBaseShape
    {
        public InputRJointShape(double radius, double strokeThickness, double xPosition, double yPosition, double xAxisOffset,
            double yAxisOffset, bool isGround, JointData jointData)
            : base(2 * radius, 2 * radius, strokeThickness, xPosition, yPosition, xAxisOffset, yAxisOffset, 0, "MoveArrows","", jointData)
        {
            /* now draw the actual joint */
            var fillBrush = /*isGround ? new SolidColorBrush(Colors.Black) :*/ new SolidColorBrush(Colors.Transparent);
            jointShape = new Ellipse
            {
                Width = 2 * radius,
                Height = 2 * radius,
                Stroke = new SolidColorBrush(Colors.Black),
                Fill = fillBrush,
                StrokeThickness = strokeThickness,
                RenderTransform = new TranslateTransform
                {
                    X = -radius,
                    Y = -radius
                }
            };
            Children.Add(jointShape);
        }

    }
}
