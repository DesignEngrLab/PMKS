using System.Windows;
using System.Windows.Media;
using System.Windows.Shapes;
using PMKS_Silverlight_App;

namespace Silverlight_PMKS.Shapes.Static_Shapes
{
    public class Axes : Path
    {
        public Axes(double strokeThickness, double xOffset, double yOffset, double width, double height)
        {
            Data = new GeometryGroup()
                {
                    Children = new GeometryCollection()
                        {
                            new LineGeometry()
                                {
                                    StartPoint = new Point(xOffset, 0 - DisplayConstants.ExtraAxesLengthFactor * height),
                                    EndPoint = new Point(xOffset, height + DisplayConstants.ExtraAxesLengthFactor * height)
                                },
                            new LineGeometry()
                                {
                                    StartPoint = new Point(0 - DisplayConstants.ExtraAxesLengthFactor * width, yOffset),
                                    EndPoint = new Point(width + DisplayConstants.ExtraAxesLengthFactor * width, yOffset)
                                }
                        }
                };
            Stroke = new SolidColorBrush(Color.FromArgb(255, 80, 80, 80));
            StrokeThickness = strokeThickness;
        }

    }
}
