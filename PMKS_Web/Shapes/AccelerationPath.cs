using PlanarMechanismSimulator;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class AccelerationPath : Path
    {
        public AccelerationPath(int index, TimeSortedList JointParameters, JointData jData, double accelFactor, double minX, double minY)
        {
            var lines = new PathFigureCollection(); 
            for (int j = 0; j < JointParameters.Count; j++)
            {
                var x = JointParameters.Parameters[j][index, 0];
                var y = JointParameters.Parameters[j][index, 1];
                lines.Add(new PathFigure
                    {
                        StartPoint = new Point(x, y),
                        Segments =
                            {
                                new LineSegment
                                    {
                                        Point = new Point(x + accelFactor*JointParameters.Parameters[j][index, 4],
                                                          y + accelFactor*JointParameters.Parameters[j][index, 5])
                                    }
                            }
                    });
            }
            Data = new PathGeometry{Figures = lines};
            Stroke = new SolidColorBrush { Color = Colors.Orange };
            RenderTransform = new TranslateTransform { X = -minX, Y = -minY };

            var binding = new Binding
              {
                  Source = jData,
                  Mode = BindingMode.OneWay,
                  Path = new PropertyPath(JointData.AccelVisibleProperty),
                  Converter = new CheckBoxVisibilityConverter()
              };
            SetBinding(OpacityProperty, binding);

        }

    }
}
