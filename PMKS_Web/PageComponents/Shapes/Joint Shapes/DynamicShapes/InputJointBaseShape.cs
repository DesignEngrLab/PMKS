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
        protected double yCoord, xCoord, xAxisOffset, yAxisOffset, angle, oldXCoord, oldYCoord;
        protected static ResourceDictionary shapeResourceDictionary;
        private readonly double iconOpacityRadius;
        private JointData jointData;




        protected InputJointBaseShape(double iconWidth, double iconHeight, double strokeThickness,
            double xPosition, double yPosition, double xAxisOffset, double yAxisOffset, double angle, string translateIconKey, JointData jointData)
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
            SetPosition(xPosition, yPosition, xAxisOffset, yAxisOffset, angle);
        }

        public void SetPosition(double xPosition, double yPosition, double xAxisOffset, double yAxisOffset, double angle)
        {
            this.xAxisOffset = xAxisOffset;
            this.yAxisOffset = yAxisOffset;
            xCoord = xPosition;
            yCoord = yPosition;

            this.angle = (double.IsNaN(angle)) ? 0.0 : angle;
            SetZIndex(this, 32766);
            RenderTransformOrigin = new Point(0.5, 0.5);

            RenderTransform = new CompositeTransform
            {
                TranslateX = xCoord + xAxisOffset,
                TranslateY = yCoord + yAxisOffset,
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


        internal bool OnMouseMove(Point mousePos, Point startMovingReference, bool multiSelect)
        {
            if (mouseMoving && !multiSelect)
            {
                xCoord = jointData._xPos = oldXCoord + mousePos.X - startMovingReference.X;
                yCoord = jointData._yPos = oldYCoord + mousePos.Y - startMovingReference.Y;
                jointData.RefreshTablePositions();
                this.RenderTransform = new TranslateTransform
                {
                    X = xCoord + App.main.mainViewer.XAxisOffset,
                    Y = yCoord + App.main.mainViewer.XAxisOffset
                };
                return true;
            }
             if (!mouseMoving)
            {
                var xDifference = Math.Abs(mousePos.X - App.main.mainViewer.XAxisOffset - xCoord);
                var yDifference = Math.Abs(mousePos.Y - App.main.mainViewer.YAxisOffset - yCoord);
                var radialDifference = xDifference + yDifference;
                if (iconOpacityRadius < radialDifference)
                    translateIcon.Opacity = 0.0;
                else
                    translateIcon.Opacity = DisplayConstants.MaxUnselectedOpacity*
                                            (1 - radialDifference/iconOpacityRadius);
            }
            return false;
        }

        internal void OnMouseLeftButtonDown(MouseButtonEventArgs e, bool multiSelect)
        {
            if (!mouseIsContained) return;
            mouseMoving = true;
            oldXCoord = xCoord;
            oldYCoord = yCoord;
            translateIcon.Opacity = 1.0;
            //CaptureMouse();
            if (multiSelect) return;
            e.Handled = true;
        }

        internal void OnMouseLeftButtonUp(MouseButtonEventArgs e, bool multiSelect)
        {
            if (multiSelect) return;
                                           
            mouseMoving = false;
            //ReleaseMouseCapture();      
            e.Handled = true;

        }
    }
}



