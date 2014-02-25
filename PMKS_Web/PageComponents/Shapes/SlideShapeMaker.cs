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
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public static class SlideShapeMaker
    {


        internal static RectangleGeometry MakeRPSlotHole(joint j, link thisLink, double xOffset, double yOffset, double jointSize, double startingBufferRadius)
        {
            var slideAngle = j.InitSlideAngle + thisLink.AngleInitial;


            var blockWidth = DisplayConstants.PJointSizeIncrease * jointSize * DisplayConstants.SliderRectangleAspectRatioSqareRoot;
            var blockHeight = DisplayConstants.PJointSizeIncrease * jointSize / DisplayConstants.SliderRectangleAspectRatioSqareRoot;
            var slideWidth = (j.SlideLimits != null)
                ? j.SlideLimits[3] - j.SlideLimits[1] + blockWidth
                : 3 * blockWidth;
            var origX = (j.SlideLimits != null)
                ? j.SlideLimits[2] - j.SlideLimits[1] + blockWidth / 2
                : slideWidth / 2;
            if (j.SlideLimits != null && thisLink.DistanceBetweenSlides(j, thisLink.joints[(int)j.SlideLimits[0]]) < 0)
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


            var blockWidth = DisplayConstants.PJointSizeIncrease * jointSize * DisplayConstants.SliderRectangleAspectRatioSqareRoot;
            var blockHeight = DisplayConstants.PJointSizeIncrease * jointSize / DisplayConstants.SliderRectangleAspectRatioSqareRoot;
            var slideWidth = (j.SlideLimits != null)
                ? j.SlideLimits[3] - j.SlideLimits[1] + blockWidth
                : 3 * blockWidth;
            var origX = (j.SlideLimits != null)
                ? j.SlideLimits[2] - j.SlideLimits[1] + blockWidth / 2
                : slideWidth / 2;
            if (j.SlideLimits != null && thisLink.DistanceBetweenSlides(j, thisLink.joints[(int)j.SlideLimits[0]]) < 0)
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
            var blockWidth = DisplayConstants.PJointSizeIncrease * jointSize * DisplayConstants.SliderRectangleAspectRatioSqareRoot;
            var blockHeight = DisplayConstants.PJointSizeIncrease * jointSize / DisplayConstants.SliderRectangleAspectRatioSqareRoot;
            var slideWidth = (j.SlideLimits != null)
                ? j.SlideLimits[3] - j.SlideLimits[1] + blockWidth
                : 3 * blockWidth;
            var origX = (j.SlideLimits != null)
                ? j.SlideLimits[2] - j.SlideLimits[1] + blockWidth / 2
                : slideWidth / 2;
            if (thisLink.DistanceBetweenSlides(j, thisLink.joints[(int)j.SlideLimits[0]]) < 0)
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
