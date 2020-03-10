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
using PMKS;
using Silverlight_PMKS;
using Point = System.Windows.Point;

namespace PMKS_Silverlight_App
{

    public abstract class InputJointBaseShape : Canvas
    {
        protected Shape jointShape;
        protected Shape translateIcon;
        protected Shape rotateIcon;
        public double yCoord, xCoord, xAxisOffset, yAxisOffset, angle, oldXCoord, oldYCoord, oldAngle;
        protected static ResourceDictionary shapeResourceDictionary;
        private readonly double iconOpacityRadius;
        private JointData jointData;

        private Boolean mouseIsContained, mouseIsOverRotate, mouseMoving, mouseRotating;





        protected InputJointBaseShape(double iconWidth, double iconHeight, double strokeThickness, double xPosition,
            double yPosition, double xAxisOffset, double yAxisOffset, double angle, string translateIconKey,
            string rotateIconKey, JointData jointData)
        {
            /* make the move arrows translate icon that is initialized in base class constructor */
            this.jointData = jointData;

            #region Make Translate icon

            var IconDataTemplate = (DataTemplate)shapeResourceDictionary[translateIconKey];
            translateIcon = (Path)IconDataTemplate.LoadContent();
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
            #endregion

            #region Make Rotate icon
            if (!string.IsNullOrWhiteSpace(rotateIconKey))
            {
                IconDataTemplate = (DataTemplate)shapeResourceDictionary[rotateIconKey];

                rotateIcon = (Path)IconDataTemplate.LoadContent();
                iconDimensionsStr = ((string)rotateIcon.Tag).Split(',');
                widthFromTemplate = double.Parse(iconDimensionsStr[0]);
                heightFromTemplate = double.Parse(iconDimensionsStr[1]);
                rotateIcon.Width = iconWidth * (rotateIcon.Width - rotateIcon.StrokeThickness) / widthFromTemplate;
                rotateIcon.Height = iconHeight * (rotateIcon.Height - rotateIcon.StrokeThickness) / heightFromTemplate;
                iconOpacityRadius = DisplayConstants.IconIncreaseRadiusFactor * (rotateIcon.Width + rotateIcon.Height);
                rotateIcon.StrokeThickness *= strokeThickness;
                rotateIcon.Opacity = 0.0;
                MouseEnter += InputJointBaseShape_MouseEnter;
                MouseLeave += InputJointBaseShape_MouseLeave;
                Children.Add(rotateIcon);
                rotateIcon.RenderTransform = new TranslateTransform
                {
                    X = -rotateIcon.ActualWidth / 2,
                    Y = -rotateIcon.ActualHeight / 2
                };
                rotateIcon.MouseEnter += rotateIcon_MouseEnter;
                rotateIcon.MouseLeave += rotateIcon_MouseLeave;
            }
            #endregion
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
        void InputJointBaseShape_MouseLeave(object sender, MouseEventArgs e)
        {
            mouseIsContained = false;
        }

        void InputJointBaseShape_MouseEnter(object sender, MouseEventArgs e)
        {
            mouseIsContained = true;
        }
        void rotateIcon_MouseLeave(object sender, MouseEventArgs e)
        {
            mouseIsOverRotate = false;
        }

        void rotateIcon_MouseEnter(object sender, MouseEventArgs e)
        {
            mouseIsOverRotate = true;
        }



        public static void LoadShapes()
        {
            shapeResourceDictionary = new ResourceDictionary
            {
                Source = new Uri("/Silverlight_PMKS;component/Properties/ShapeResourceDictionary.xaml", UriKind.Relative)
            };

        }


        private void AdjustOpacity(Point mousePos)
        {
            var xDifference = Math.Abs(mousePos.X - App.main.mainViewer.XAxisOffset - xCoord);
            var yDifference = Math.Abs(mousePos.Y - App.main.mainViewer.YAxisOffset - yCoord);
            var radialDifference = xDifference + yDifference;
            if (iconOpacityRadius < radialDifference)
            {
                translateIcon.Opacity = 0.0;
                if (rotateIcon != null) rotateIcon.Opacity = 0.0;
            }
            else
            {
                if (rotateIcon != null)
                    rotateIcon.Opacity = translateIcon.Opacity = DisplayConstants.MaxUnselectedOpacity *
                                             (1 - radialDifference / iconOpacityRadius);
                else translateIcon.Opacity = DisplayConstants.MaxUnselectedOpacity *
                              (1 - radialDifference / iconOpacityRadius);
            }
        }

        internal void OnMouseLeftButtonDown(MouseButtonEventArgs e, bool multiSelect)
        {
            if (!mouseIsContained) return;
            if (mouseIsOverRotate && rotateIcon != null)
            {
                mouseRotating = true;
                oldXCoord = xCoord;
                oldYCoord = yCoord;
                oldAngle = angle;
                rotateIcon.Opacity = 1.0;
            }
            else
            {
                mouseMoving = true;
                oldXCoord = xCoord;
                oldYCoord = yCoord;
                translateIcon.Opacity = 1.0;
            }
            if (multiSelect) return;
            e.Handled = true;
        }

        internal void OnMouseLeftButtonUp(MouseButtonEventArgs e, bool multiSelect)
        {
            if (multiSelect) return;
            mouseRotating = mouseMoving = false;
            e.Handled = true;
        }

        internal bool OnMouseMove(Point mousePos, Point startMovingReference, bool multiSelect)
        {
            if (!mouseMoving && !mouseRotating) AdjustOpacity(mousePos);
            if (mouseMoving && !multiSelect)
            {
                xCoord = jointData.X = oldXCoord + mousePos.X - startMovingReference.X;
                yCoord = jointData.Y = oldYCoord + mousePos.Y - startMovingReference.Y;
                jointData.RefreshTablePositions();

                RenderTransformOrigin = new Point(0.5, 0.5);

                RenderTransform = new CompositeTransform
                {
                    TranslateX = xCoord + xAxisOffset,
                    TranslateY = yCoord + yAxisOffset,
                    Rotation = DisplayConstants.RadiansToDegrees * angle
                };
                return true;
            }
            if (mouseRotating && !multiSelect)
            {
                var tempAngle = oldAngle + Constants.angle(startMovingReference.X, startMovingReference.Y, mousePos.X, mousePos.Y);

                while (tempAngle > Math.PI / 2) tempAngle -= Math.PI;
                while (tempAngle < -Math.PI / 2) tempAngle += Math.PI;
                angle =  tempAngle;
                jointData.AngleDegrees =DisplayConstants.RadiansToDegrees* tempAngle;
                jointData.RefreshTablePositions();

                RenderTransformOrigin = new Point(0.5, 0.5);

                RenderTransform = new CompositeTransform
                {
                    TranslateX = xCoord + xAxisOffset,
                    TranslateY = yCoord + yAxisOffset,
                    Rotation = DisplayConstants.RadiansToDegrees * angle
                };
                return true;
            }

            return false;
        }
    }
}



