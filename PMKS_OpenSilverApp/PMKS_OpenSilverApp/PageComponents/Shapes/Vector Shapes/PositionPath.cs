using System;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;
using PMKS;
using Point = PMKS.Point;

namespace PMKS_Silverlight_App
{
    public class PositionPath : Path
    {
        private readonly bool _isClosed; 
        public int index { get;private set; }

        public PositionPath(int index, TimeSortedList JointParameters, JointData jData, double offsetX, double offsetY, Boolean isClosed,
            double penThick)
        {
            this.index = index;
            _isClosed = isClosed;
            Data = new PathGeometry
                {
                    Figures = new PathFigureCollection
                        {
                            //LinearPath(index, JointParameters, jData, offsetX, offsetY)
                            //,
                            QuadraticPath(index, JointParameters, jData,  offsetX,  offsetY)
                        }
                };
            StrokeThickness = penThick;

            Stroke = new SolidColorBrush { Color = Colors.Green };
            Width = Height = DisplayConstants.UnCroppedDimension;
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
            var start = new System.Windows.Point(JointParameters[0].Value[index, 0] + offsetX, JointParameters[0].Value[index, 1] + offsetY);
            var points = new PointCollection();
            for (int i = 1; i < JointParameters.Count; i++)
            {
                var x = JointParameters[i].Value[index, 0] + offsetX;
                var y = JointParameters[i].Value[index, 1] + offsetY;
                points.Add(new System.Windows.Point(x, y));
            }
            #region see if path should be closed
            //var last_i = JointParameters.Count - 1;
            ////find a good time step value.
            //var timeStepLast = JointParameters.Times[last_i] - JointParameters.Times[last_i - 1];
            //timeStepLast += JointParameters.Times[1] - JointParameters.Times[0];
            //timeStepLast /= 2;

            //var xLast = JointParameters[last_i].Value[index, 2] * timeStepLast +
            //            JointParameters[last_i].Value[index, 4] * timeStepLast * timeStepLast / 2;
            //var yLast = JointParameters[last_i].Value[index, 3] * timeStepLast +
            //           JointParameters[last_i].Value[index, 5] * timeStepLast * timeStepLast / 2;
            //var closePath = (Math.Abs(xLast - start.X) + Math.Abs(yLast - start.Y) < 100 * Constants.epsilon);

            #endregion

            return new PathFigure
            {
                StartPoint = start,
                Segments = new PathSegmentCollection { new PolyLineSegment { Points = points } },
                IsClosed = _isClosed
            };
        }

        public PathFigure QuadraticPath(int index, TimeSortedList JointParameters, JointData jData, double offsetX, double offsetY)
        {
            var start = new System.Windows.Point(JointParameters[0].Value[index, 0] + offsetX, JointParameters[0].Value[index, 1] + offsetY);
            var points = new PointCollection();
            for (int i = 1; i < JointParameters.Count; i++)
            {
                var x1 = JointParameters[i - 1].Value[index, 0] + offsetX;
                var y1 = JointParameters[i - 1].Value[index, 1] + offsetY;
                var v_1x = JointParameters[i - 1].Value[index, 2];
                var v_1y = JointParameters[i - 1].Value[index, 3];
                var x2 = JointParameters[i].Value[index, 0] + offsetX;
                var y2 = JointParameters[i].Value[index, 1] + offsetY;
                var v_2x = JointParameters[i].Value[index, 2];
                var v_2y = JointParameters[i].Value[index, 3];
                var interPt = Constants.solveViaIntersectingLines(v_1y / v_1x, new Point(x1, y1),
                      v_2y / v_2x, new Point(x2, y2));
                if (double.IsNaN(interPt.X) || double.IsNaN(interPt.Y))
                    points.Add(new System.Windows.Point((x1 + x2) / 2, (y1 + y2) / 2));
                else
                    points.Add(new System.Windows.Point(interPt.X, interPt.Y));
                points.Add(new System.Windows.Point(x2, y2));
            }
            if (_isClosed)
            {
                var x1 = JointParameters[JointParameters.LastIndex].Value[index, 0] + offsetX;
                var y1 = JointParameters[JointParameters.LastIndex].Value[index, 1] + offsetY;
                var v_1x = JointParameters[JointParameters.LastIndex].Value[index, 2];
                var v_1y = JointParameters[JointParameters.LastIndex].Value[index, 3];
                var x2 = JointParameters[0].Value[index, 0] + offsetX;
                var y2 = JointParameters[0].Value[index, 1] + offsetY;
                var v_2x = JointParameters[0].Value[index, 2];
                var v_2y = JointParameters[0].Value[index, 3];
                var interPt = Constants.solveViaIntersectingLines(v_1y / v_1x, new Point(x1, y1),
                      v_2y / v_2x, new Point(x2, y2));
                if (double.IsNaN(interPt.X) || double.IsNaN(interPt.Y))
                    points.Add(new System.Windows.Point((x1 + x2) / 2, (y1 + y2) / 2));
                else
                    points.Add(new System.Windows.Point(interPt.X, interPt.Y));
                points.Add(new System.Windows.Point(x2, y2));
            }
            #region see if path should be closed
            //var last_i = JointParameters.Count - 1;
            ////find a good time step value.
            //var timeStepLast = JointParameters.Times[last_i] - JointParameters.Times[last_i - 1];
            //timeStepLast += JointParameters.Times[1] - JointParameters.Times[0];
            //timeStepLast /= 2;

            //var xLast = JointParameters[last_i].Value[index, 2] * timeStepLast +
            //            JointParameters[last_i].Value[index, 4] * timeStepLast * timeStepLast / 2;
            //var yLast = JointParameters[last_i].Value[index, 3] * timeStepLast +
            //           JointParameters[last_i].Value[index, 5] * timeStepLast * timeStepLast / 2;
            //var closePath = (Math.Abs(xLast - start.X) + Math.Abs(yLast - start.Y) < 100 * Constants.epsilon);

            #endregion

            return new PathFigure
                {
                    StartPoint = start,
                    Segments = new PathSegmentCollection { new PolyQuadraticBezierSegment { Points = points } },
                    IsClosed = _isClosed
                };
        }
    }
}
