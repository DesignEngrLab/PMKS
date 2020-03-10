using System;
using System.Linq;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using PMKS;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;
using Point = System.Windows.Point;

namespace PMKS_Silverlight_App
{
    public abstract class DisplayVectorBaseShape : Path
    {
        #region Fields
        protected readonly double yOffset;
        protected readonly double xOffset;
        protected readonly double factor;
        #endregion

        #region Dependency Properties

        public static readonly DependencyProperty StartProperty
            = DependencyProperty.Register("Start",
                                          typeof(double[]), typeof(DisplayVectorBaseShape),
                                          new PropertyMetadata(null, OnTimeChanged));
        public double[] Start
        {
            get { return (double[])GetValue(StartProperty); }
            set { SetValue(StartProperty, value); }
        }

        public static readonly DependencyProperty EndProperty
            = DependencyProperty.Register("End",
                                          typeof(double[]), typeof(DisplayVectorBaseShape),
                                          new PropertyMetadata(null, OnTimeChanged));
        public double[] End
        {
            get { return (double[])GetValue(EndProperty); }
            set { SetValue(EndProperty, value); }
        }

        #endregion

        protected DisplayVectorBaseShape(double factor, double strokeThickness, double xOffset, double yOffset)
        {
            Height = Width = DisplayConstants.UnCroppedDimension;
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.factor = factor;
            Data = new LineGeometry();
            StrokeThickness = strokeThickness;

        }


        protected static void OnTimeChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            var vector = ((DisplayVectorBaseShape)d);
            if (vector.End == null || vector.Start == null || vector.Start.Contains(double.NaN)
                || vector.End.Contains(double.NaN)) return;
            var xStart = vector.Start[0] + vector.xOffset;
            var yStart = vector.Start[1] + vector.yOffset;
            ((LineGeometry)vector.Data).StartPoint = new Point(xStart, yStart);
            ((LineGeometry)vector.Data).EndPoint = new Point(xStart + vector.factor * vector.End[0], yStart + vector.factor * vector.End[1]);
        }
        public void ClearBindings()
        {
            ClearValue(StartProperty);
            ClearValue(EndProperty);
            ClearValue(OpacityProperty);
        }
    }
}
