using PlanarMechanismSimulator;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class VelocityPath : Path
    {
        public VelocityPath(int index, TimeSortedList JointParameters, JointData jData, double velocityFactor, double offsetX, double offsetY)
        {
            var lines = new PathFigureCollection();
            for (int j = 0; j < JointParameters.Count; j++)
            {
                var x = JointParameters.Parameters[j][index, 0];
                var y = JointParameters.Parameters[j][index, 1];
                lines.Add(new PathFigure
                    {
                        StartPoint = new Point(x + offsetX, y + offsetY),
                        Segments =
                            {
                                new LineSegment
                                    {
                                        Point = new Point(x + offsetX+ velocityFactor*JointParameters.Parameters[j][index, 2],
                                                          y + offsetY+ velocityFactor*JointParameters.Parameters[j][index, 3])
                                    }
                            }
                    });
            }
            Data = new PathGeometry { Figures = lines };
            Stroke = new SolidColorBrush { Color = Colors.Brown };
            //RenderTransform = new TranslateTransform { X = offsetX, Y = offsetY };

            var binding = new Binding
              {
                  Source = jData,
                  Mode = BindingMode.OneWay,
                  Path = new PropertyPath(JointData.VelVisibleProperty),
                  Converter = new CheckBoxVisibilityConverter()
              };
            SetBinding(OpacityProperty, binding);

        }

    }
}
