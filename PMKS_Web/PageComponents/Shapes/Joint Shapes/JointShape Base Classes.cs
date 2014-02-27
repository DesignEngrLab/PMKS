using System;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public abstract class JointBaseShape : Path
    {
        protected JointBaseShape(double radius, double strokeThickness, double xOffset, double yOffset)
        {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.radius = radius;
            Stroke = new SolidColorBrush(Colors.Black);
            StrokeThickness = strokeThickness;
        }

        protected readonly double yOffset;
        protected readonly double xOffset;
        protected readonly double radius;



    }
    public abstract class InputJointBaseShape : JointBaseShape
    {
        protected Boolean isGround;
        private bool mouseMoving;
        private Point moveReference;

        protected InputJointBaseShape(double radius, double strokeThickness, double xOffset, double yOffset, Boolean isGround)
            : base(radius, strokeThickness, xOffset, yOffset)
        {
            this.isGround = isGround;
            if (isGround) Fill = new SolidColorBrush(Colors.Black);
            else Fill = new SolidColorBrush(Colors.Transparent);
            MouseLeftButtonDown += FixedJointBaseShape_MouseLeftButtonDown;
            MouseLeftButtonUp += FixedJointBaseShape_MouseLeftButtonUp;
            MouseMove += FixedJointBaseShape_MouseMove;
            MouseEnter += InputJointBaseShape_MouseEnter;
        }

        void InputJointBaseShape_MouseEnter(object sender, MouseEventArgs e)
        {
        }

        private void FixedJointBaseShape_MouseMove(object sender, MouseEventArgs e)
        {
            if (mouseMoving)
            {
                var delta = e.GetPosition((UIElement)this.Parent);
                this.RenderTransform = new TranslateTransform
                {
                    X = delta.X - moveReference.X,
                    Y = delta.Y - moveReference.Y
                };
               
            }

        }

        void FixedJointBaseShape_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            mouseMoving = false;
            ReleaseMouseCapture();
            e.Handled = true;
        }

        void FixedJointBaseShape_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            mouseMoving = true;
            CaptureMouse();
            moveReference = e.GetPosition((UIElement)this.Parent);
            e.Handled = true;
        }


    }



    public abstract class DynamicJointBaseShape : JointBaseShape
    {
        public abstract void Redraw();
        internal abstract void ClearBindings();
        public static readonly DependencyProperty CoordinatesProperty
            = DependencyProperty.Register("Coordinates",
                                          typeof(double[]), typeof(DynamicJointBaseShape),
                                          new PropertyMetadata(null, OnTimeChanged));

        public double[] Coordinates
        {
            get { return (double[])GetValue(CoordinatesProperty); }
            set { SetValue(CoordinatesProperty, value); }
        }
        protected static void OnTimeChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            ((DynamicJointBaseShape)d).Redraw();
        }

        protected DynamicJointBaseShape(joint j, Slider timeSlider, Simulator pmks, double radius, double strokeThickness, double xOffset, double yOffset)
            : base(radius, strokeThickness, xOffset, yOffset)
        {
            Height = Width = DisplayConstants.UnCroppedDimension;
            var binding = new Binding
                              {
                                  Source = timeSlider,
                                  Mode = BindingMode.OneWay,
                                  Path = new PropertyPath(RangeBase.ValueProperty),
                                  Converter = new TimeToJointParameterConverter(j, StateVariableType.Position, pmks)
                              };
            SetBinding(CoordinatesProperty, binding);

            RenderTransform = new TranslateTransform
            {
                X = Coordinates[0] + xOffset - radius,
                Y = Coordinates[1] + yOffset - radius
            };
            Fill = new SolidColorBrush(Colors.Black);
        }


    }

}
