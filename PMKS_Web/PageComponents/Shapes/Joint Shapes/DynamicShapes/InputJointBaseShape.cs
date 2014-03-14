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
using Silverlight_PMKS;

namespace PMKS_Silverlight_App
{

    public abstract class InputJointBaseShape : Canvas
    {
        protected Shape jointShape;
        protected Shape translateIcon;
        protected Shape rotateIcon;
        protected Boolean isGround;
        private bool mouseMoving;
        private Point moveReference;
        protected double yCoord;
        protected double xCoord;
        protected double angle;
        protected static ResourceDictionary shapeResourceDictionary;
        private readonly double iconOpacityRadius;

        private double xPosition;
        private double yPosition;
        private JointData jointData;




        protected InputJointBaseShape(double iconWidth, double iconHeight, double strokeThickness,
            double xPosition, double yPosition, double angle, string translateIconKey, JointData jointData)
        {
            /* make the move arrows translate icon that is initialized in base class constructor */
            this.jointData = jointData;
            var translateIconDataTemplate = (DataTemplate)shapeResourceDictionary[translateIconKey];
            translateIcon = (Path)translateIconDataTemplate.LoadContent();
            var iconDimensionsStr = ((string)translateIcon.Tag).Split(',');
            var widthFromTemplate = double.Parse(iconDimensionsStr[0]);
            var heightFromTemplate = double.Parse(iconDimensionsStr[1]);
            translateIcon.Width = iconWidth * (translateIcon.Width - translateIcon.StrokeThickness) / widthFromTemplate;
            translateIcon.Height = iconHeight * (translateIcon.Height - translateIcon.StrokeThickness) / heightFromTemplate;
            iconOpacityRadius = DisplayConstants.IconIncreaseRadiusFactor * (translateIcon.Width + translateIcon.Height);
            translateIcon.StrokeThickness *= strokeThickness;
            translateIcon.Opacity = 0.0;
            MouseEnter += InputJointBaseShape_MouseEnter;
            MouseLeave += InputJointBaseShape_MouseLeave;
            Children.Add(translateIcon);
            translateIcon.RenderTransform = new TranslateTransform
              {
                  X = -translateIcon.ActualWidth / 2,
                  Y = -translateIcon.ActualHeight / 2
              };
            SetPosition(xPosition, yPosition, angle);

        }

        public void SetPosition(double xPosition, double yPosition, double angle)
        {
            xCoord = xPosition;
            yCoord = yPosition;

            this.angle = (double.IsNaN(angle)) ? 0.0 : angle;
            SetZIndex(this, 32766);
            RenderTransformOrigin = new Point(0.5, 0.5);

            //  RenderTransform =     new TranslateTransform{X = xPosition,Y=yPosition};
            RenderTransform = new CompositeTransform
            {
                TranslateX = xPosition,
                TranslateY = yPosition,
                Rotation = DisplayConstants.RadiansToDegrees * angle
            };
        }

        private Boolean mouseIsContained;
        void InputJointBaseShape_MouseLeave(object sender, MouseEventArgs e)
        {
            mouseIsContained = false;
        }

        void InputJointBaseShape_MouseEnter(object sender, MouseEventArgs e)
        {
            mouseIsContained = true;
        }


        public static void LoadShapes()
        {

            shapeResourceDictionary = new ResourceDictionary
            {
                Source = new Uri("/Silverlight_PMKS;component/Properties/ShapeResourceDictionary.xaml", UriKind.Relative)
            };

        }


        internal Boolean OnMouseMove(MouseEventArgs e)
        {
            var delta = e.GetPosition(App.main.mainViewer.MainCanvas);
            if (mouseMoving)
            {
                this.RenderTransform = new TranslateTransform
                {
                    X = delta.X,
                    Y = delta.Y
                };
                var deltaX = delta.X - moveReference.X;
                var deltaY = delta.Y - moveReference.Y;
                jointData._xPos += deltaX;
                jointData._yPos += deltaY;
                xCoord += deltaX;
                yCoord += deltaY;
                moveReference = delta;
                jointData.RefreshTablePositions();
                return true;
            }

            var xDifference = Math.Abs(delta.X - xCoord);
            var yDifference = Math.Abs(delta.Y - yCoord);
            var radialDifference = xDifference + yDifference;
            if (iconOpacityRadius < radialDifference)
                translateIcon.Opacity = 0.0;
            else translateIcon.Opacity = DisplayConstants.MaxUnselectedOpacity * (1 - radialDifference / iconOpacityRadius);
            return false;
        }

        internal void OnMouseLeftButtonDown(MouseButtonEventArgs e, bool multiSelect)
        {
            if (!mouseIsContained) return;
            mouseMoving = true;
            translateIcon.Opacity = 1.0;
            //CaptureMouse();
            if (multiSelect) return;
            moveReference = e.GetPosition(App.main.mainViewer.MainCanvas);
            e.Handled = true;
        }

        internal void OnMouseLeftButtonUp(MouseButtonEventArgs e, bool multiSelect)
        {
            if (!mouseIsContained || multiSelect) return;
            


            mouseMoving = false;
            //ReleaseMouseCapture();      
            e.Handled = true;

        }
    }
}



