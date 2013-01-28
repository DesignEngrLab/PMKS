using PlanarMechanismSimulator;
using System;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class PositionPath : Path
    {
        public PositionPath(int index, TimeSortedList JointParameters, JointData jData, double offsetX, double offsetY)
        {
            Data = new PathGeometry
                {
                    Figures = new PathFigureCollection
                        {
                            LinearPath(index, JointParameters, jData, offsetX, offsetY)
                            //,
                            //QuadraticPath(index, JointParameters, jData,  offsetX,  offsetY)
                        }
                };

            Stroke = new SolidColorBrush { Color = Colors.Green };
            //RenderTransform = new TranslateTransform { X = offsetX, Y = offsetY };
            var binding = new Binding
                {
                    Source = jData,
                    Mode = BindingMode.OneWay,
                    Path = new PropertyPath(JointData.PosVisibleProperty),
                    Converter = new CheckBoxVisibilityConverter()
                };
            SetBinding(OpacityProperty, binding);

        }


        public PathFigure LinearPath(int index, TimeSortedList JointParameters, JointData jData, double offsetX, double offsetY)
        {
            var start = new Point(JointParameters[0].Value[index, 0] + offsetX, JointParameters[0].Value[index, 1] + offsetY);
            var points = new PointCollection();
            for (int i = 1; i < JointParameters.Count; i++)
            {
                var x = JointParameters[i].Value[index, 0] + offsetX;
                var y = JointParameters[i].Value[index, 1] + offsetY;
                points.Add(new Point(x, y));
            }
            #region see if path should be closed
            var last_i = JointParameters.Count - 1;
            //find a good time step value.
            var timeStepLast = JointParameters.Times[last_i] - JointParameters.Times[last_i - 1];
            timeStepLast += JointParameters.Times[1] - JointParameters.Times[0];
            timeStepLast /= 2;

            var xLast = JointParameters[last_i].Value[index, 2] * timeStepLast +
                        JointParameters[last_i].Value[index, 4] * timeStepLast * timeStepLast / 2;
            var yLast = JointParameters[last_i].Value[index, 3] * timeStepLast +
                       JointParameters[last_i].Value[index, 5] * timeStepLast * timeStepLast / 2;
            var closePath = (Math.Abs(xLast - start.X) + Math.Abs(yLast - start.Y) < 100 * Constants.epsilon);

            #endregion

            return new PathFigure
            {
                StartPoint = start,
                Segments = new PathSegmentCollection { new PolyLineSegment { Points = points } },
                IsClosed = closePath
            };
        }

        public PathFigure QuadraticPath(int index, TimeSortedList JointParameters, JointData jData, double offsetX, double offsetY)
        {
            var start = new Point(JointParameters[0].Value[index, 0] + offsetX, JointParameters[0].Value[index, 1] + offsetY);
            var points = new PointCollection();
            for (int i = 1; i < JointParameters.Count; i++)
            {
                var timeStep = JointParameters.Times[i] - JointParameters.Times[i - 1];
                var x = JointParameters[i - 1].Value[index, 0] + offsetX;
                var y = JointParameters[i - 1].Value[index, 1] + offsetY;
                var v_x = (JointParameters[i].Value[index, 2] + JointParameters[i - 1].Value[index, 2]) / 2;
                var v_y = (JointParameters[i].Value[index, 3] + JointParameters[i - 1].Value[index, 3]) / 2;
                points.Add(new Point(x + (v_x * timeStep) / 2, y + (v_y * timeStep) / 2));
                x = JointParameters[i].Value[index, 0] + offsetX;
                y = JointParameters[i].Value[index, 1] + offsetY;
                points.Add(new Point(x, y));
            }
            #region see if path should be closed
            var last_i = JointParameters.Count - 1;
            //find a good time step value.
            var timeStepLast = JointParameters.Times[last_i] - JointParameters.Times[last_i - 1];
            timeStepLast += JointParameters.Times[1] - JointParameters.Times[0];
            timeStepLast /= 2;

            var xLast = JointParameters[last_i].Value[index, 2] * timeStepLast +
                        JointParameters[last_i].Value[index, 4] * timeStepLast * timeStepLast / 2;
            var yLast = JointParameters[last_i].Value[index, 3] * timeStepLast +
                       JointParameters[last_i].Value[index, 5] * timeStepLast * timeStepLast / 2;
            var closePath = (Math.Abs(xLast - start.X) + Math.Abs(yLast - start.Y) < 100 * Constants.epsilon);

            #endregion

            return new PathFigure
                {
                    StartPoint = start,
                    Segments = new PathSegmentCollection { new PolyQuadraticBezierSegment { Points = points } },
                    IsClosed = closePath
                };
        }
    }
}
