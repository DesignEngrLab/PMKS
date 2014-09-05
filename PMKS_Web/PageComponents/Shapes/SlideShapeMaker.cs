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

namespace PMKS_Silverlight_App
{
    public static class SlideShapeMaker
    {


        internal static RectangleGeometry MakeRPSlotHole(joint j, link thisLink, double xOffset, double yOffset, double jointSize, double startingBufferRadius)
        {
            var slideAngle = j.InitSlideAngle + thisLink.AngleInitial;     
            var blockWidth = 2 * jointSize * DisplayConstants.SliderRectangleWidthIncrease;  
            var beforeSimulation = (j.MaxSlidePosition - j.MinSlidePosition <blockWidth);
            var blockHeight = 2 * jointSize;
            var slideWidth = (beforeSimulation) ? 3 * blockWidth : j.MaxSlidePosition - j.MinSlidePosition + blockWidth;
            var origX = (beforeSimulation) ? slideWidth / 2 : j.OrigSlidePosition - j.MinSlidePosition + blockWidth / 2;
            if (!beforeSimulation && thisLink.DistanceBetweenSlides(j, thisLink.ReferenceJoint1) < 0)
               slideAngle += Math.PI;
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

        internal static RectangleGeometry MakePSlotHole(joint j, link thisLink, double xOffset, double yOffset, double jointSize, double startingBufferRadius)
        {
            var slideAngle = j.InitSlideAngle + thisLink.AngleInitial;
            var blockWidth = 2 * jointSize * DisplayConstants.SliderRectangleWidthIncrease;
            var beforeSimulation = (j.MaxSlidePosition - j.MinSlidePosition < blockWidth);
            var blockHeight = 2 * jointSize;
            var slideWidth = (beforeSimulation) ? 3 * blockWidth : j.MaxSlidePosition - j.MinSlidePosition + blockWidth;
            var origX = (beforeSimulation) ? slideWidth / 2 : j.OrigSlidePosition - j.MinSlidePosition + blockWidth / 2;
            if (!beforeSimulation && thisLink.DistanceBetweenSlides(j, thisLink.ReferenceJoint1) < 0)
                slideAngle += Math.PI;
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

        internal static RectangleGeometry MakePSlotBorder(joint j, link thisLink, double xOffset, double yOffset, double jointSize, double startingBufferRadius)
        {
            var slideAngle = j.InitSlideAngle + thisLink.AngleInitial;
            var blockWidth = 2 * jointSize * DisplayConstants.SliderRectangleWidthIncrease;
            var beforeSimulation = (j.MaxSlidePosition - j.MinSlidePosition < blockWidth);
            var blockHeight = 2 * jointSize;
            var slideWidth = (beforeSimulation) ? 3 * blockWidth : j.MaxSlidePosition - j.MinSlidePosition + blockWidth;
            var origX = (beforeSimulation) ? slideWidth / 2 : j.OrigSlidePosition - j.MinSlidePosition + blockWidth / 2;
            if (!beforeSimulation && thisLink.DistanceBetweenSlides(j, thisLink.ReferenceJoint1) < 0)
                slideAngle += Math.PI;

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
