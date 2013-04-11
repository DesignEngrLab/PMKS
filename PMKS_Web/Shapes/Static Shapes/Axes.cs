using System.Windows;
using System.Windows.Media;
using System.Windows.Shapes;

namespace Silverlight_PMKS.Shapes.Static_Shapes
{
    public class Axes :Path
    {
        public Axes(double strokeThickness, double xOffset, double yOffset)

        {
            Data = new GeometryGroup()
                {
                    Children = new GeometryCollection()
                        {
                            new LineGeometry()
                                {
                                    StartPoint = new Point(xOffset, -1000),
                                    EndPoint = new Point(xOffset, 1000)
                                },
                            new LineGeometry()
                                {
                                    StartPoint = new Point(-1000, yOffset),
                                    EndPoint = new Point(1000, yOffset)
                                }
                        }
                };
            Stroke = new SolidColorBrush(Color.FromArgb(255,80,80,80));
            StrokeThickness = strokeThickness;
        }
    }
}
