using System;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using PMKS;
using Point = System.Windows.Point;

namespace PMKS_Silverlight_App
{
    public static class SlideShapeMaker
    {
        internal static RectangleGeometry MakeRPSlotHole(Joint j, Link thisLink, double xOffset, double yOffset, double jointSize, double startingBufferRadius)
        {
            var slideAngle = j.SlideAngleInitial + Math.PI;
            var blockWidth = 2 * jointSize * DisplayConstants.SliderRectangleWidthIncrease;
            var beforeSimulation = (j.MaxSlidePosition - j.MinSlidePosition < blockWidth);
            var blockHeight = 2 * jointSize;
            var slideWidth = (beforeSimulation) ? 3 * blockWidth : j.MaxSlidePosition - j.MinSlidePosition + blockWidth;
            var origX = (beforeSimulation) ? slideWidth / 2 : j.OrigSlidePosition - j.MinSlidePosition + blockWidth / 2;

            var holeShape = new RectangleGeometry
            {
                Rect = new Rect(new Point(-origX, -blockHeight / 2), new Size(slideWidth, blockHeight)),
                Transform = new CompositeTransform
                {
                    Rotation = DisplayConstants.RadiansToDegrees * slideAngle,
                    TranslateX = j.xInitial + xOffset,
                    TranslateY = j.yInitial + yOffset
                },
                RadiusX = blockHeight / 2,
                RadiusY = blockHeight / 2
            };
            return holeShape;

        }

        internal static RectangleGeometry MakePSlotHole(Joint j, Link thisLink, double xOffset, double yOffset, double jointSize, double startingBufferRadius)
        {
            var slideAngle = j.SlideAngleInitial + Math.PI;
            var blockWidth = 2 * jointSize * DisplayConstants.SliderRectangleWidthIncrease;
            var beforeSimulation = (j.MaxSlidePosition - j.MinSlidePosition < blockWidth);
            var blockHeight = 2 * jointSize;
            var slideWidth = (beforeSimulation) ? 3 * blockWidth : j.MaxSlidePosition - j.MinSlidePosition + blockWidth;
            var origX = (beforeSimulation) ? slideWidth / 2 : j.OrigSlidePosition - j.MinSlidePosition + blockWidth / 2;

            var holeShape = new RectangleGeometry
            {
                Rect = new Rect(new Point(-origX, -blockHeight / 2), new Size(slideWidth, blockHeight)),
                Transform = new CompositeTransform
                {
                    Rotation = DisplayConstants.RadiansToDegrees * slideAngle,
                    TranslateX = j.xInitial + xOffset,
                    TranslateY = j.yInitial + yOffset
                }
            };
            return holeShape;

        }

        internal static RectangleGeometry MakePSlotBorder(Joint j, Link thisLink, double xOffset, double yOffset, double jointSize,
            double startingBufferRadius, Boolean showNoSlot = false)
        {
            var slideAngle = j.SlideAngleInitial + Math.PI;
            var blockWidth = 2 * jointSize * DisplayConstants.SliderRectangleWidthIncrease;
            showNoSlot = showNoSlot || (j.MaxSlidePosition - j.MinSlidePosition < blockWidth);
            var blockHeight = 2 * jointSize;
            var slideWidth = (showNoSlot) ? 3 * blockWidth : j.MaxSlidePosition - j.MinSlidePosition + blockWidth;
            var origX = (showNoSlot) ? slideWidth / 2 : j.OrigSlidePosition - j.MinSlidePosition + blockWidth / 2;

            var borderShape = new RectangleGeometry
            {
                RadiusX = startingBufferRadius,
                RadiusY = startingBufferRadius,
                Rect =
                    new Rect(new Point(-origX - startingBufferRadius, -blockHeight / 2 - startingBufferRadius),
                        new Size(slideWidth + 2 * startingBufferRadius, blockHeight + 2 * startingBufferRadius)),
                Transform = new CompositeTransform
                {
                    Rotation = DisplayConstants.RadiansToDegrees * slideAngle,
                    TranslateX = j.xInitial + xOffset,
                    TranslateY = j.yInitial + yOffset
                }
            };
            return borderShape;
        }
    }
}
