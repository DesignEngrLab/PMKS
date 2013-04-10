using System;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using PlanarMechanismSimulator;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;

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

        public static readonly DependencyProperty XStartProperty
            = DependencyProperty.Register("XStart",
                                          typeof(double), typeof(DisplayVectorBaseShape),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double XStart
        {
            get { return (double)GetValue(XStartProperty); }
            set { SetValue(XStartProperty, value); }
        }
        public static readonly DependencyProperty YStartProperty
            = DependencyProperty.Register("YStart",
                                          typeof(double), typeof(DisplayVectorBaseShape),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double YStart
        {
            get { return (double)GetValue(YStartProperty); }
            set { SetValue(YStartProperty, value); }
        }

        public static readonly DependencyProperty XLengthProperty
            = DependencyProperty.Register("XLength",
                                          typeof(double), typeof(DisplayVectorBaseShape),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double XLength
        {
            get { return (double)GetValue(XLengthProperty); }
            set { SetValue(XLengthProperty, value); }
        }
        public static readonly DependencyProperty YLengthProperty
            = DependencyProperty.Register("YLength",
                                          typeof(double), typeof(DisplayVectorBaseShape),
                                          new PropertyMetadata(double.NaN, OnTimeChanged));
        public double YLength
        {
            get { return (double)GetValue(YLengthProperty); }
            set { SetValue(YLengthProperty, value); }
        }

        #endregion

        protected DisplayVectorBaseShape(double factor, double strokeThickness, double xOffset, double yOffset)
        {
            Height = Width = 999999;
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.factor = factor;
            Data = new LineGeometry();
            StrokeThickness = strokeThickness;

        }


        protected static void OnTimeChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            var vector = ((DisplayVectorBaseShape)d);
            var xStart = vector.XStart + vector.xOffset;
            var yStart = vector.YStart + vector.yOffset;
            ((LineGeometry)vector.Data).StartPoint = new Point(xStart, yStart);
            ((LineGeometry)vector.Data).EndPoint = new Point(xStart + vector.factor * vector.XLength, yStart + vector.factor * vector.YLength);
        }
        public void ClearBindings()
        {
            ClearValue(XStartProperty);
            ClearValue(YStartProperty);
            ClearValue(XLengthProperty);
            ClearValue(YLengthProperty);
            ClearValue(OpacityProperty);
        }
    }
}
