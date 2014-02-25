using System;
using System.Linq;
using System.Windows.Media.Imaging;
using PlanarMechanismSimulator;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;
using PMKS_Silverlight_App;

namespace PMKS_Silverlight_App
{
    public class GroundLinkShape : Shape
    {
        public List<Shape> Shapes { get; private set; }


        #region Constructor
        public GroundLinkShape(link groundLink, double xOffset, double yOffset, double strokeThickness, double jointSize,
                            double startingBufferRadius)
        {
            Shapes = new List<Shape>();
            double triangleSideLength = 2.5 * jointSize;
            Shape innerShape, outerShape;
            foreach (var j in groundLink.joints)
            {
                switch (j.jointType)
                {
                    case JointTypes.R:
                        innerShape = new Ellipse
                        {
                            Stroke = new SolidColorBrush(Colors.Black),
                            Fill = new SolidColorBrush(Colors.Black),
                            StrokeThickness = strokeThickness,
                            Tag = "ground",
                            Height = jointSize,
                            Width = jointSize,
                            RenderTransform = new TranslateTransform { X = j.xInitial + xOffset, Y = j.yInitial + yOffset }
                        };
                        // innerShape = null;
                        outerShape = new Polygon
                        {
                            Stroke = new SolidColorBrush(Colors.Black),
                            Fill = new SolidColorBrush(Colors.Black),
                            StrokeThickness = strokeThickness,
                            Tag = "ground",
                            Points = new PointCollection
                            {
                                new Point(j.xInitial+xOffset,j.yInitial+ yOffset),
                                new Point(j.xInitial+xOffset - triangleSideLength,j.yInitial+ yOffset - triangleSideLength),
                                new Point(j.xInitial+xOffset + triangleSideLength,j.yInitial+ yOffset - triangleSideLength)    
                     
                            },
                            //RenderTransform = new TranslateTransform { X = xPosition, Y = yPosition }
                            //,Width = DisplayConstants.UnCroppedDimension,
                            //Height = DisplayConstants.UnCroppedDimension
                        };
                        break;
                    case JointTypes.P:
                        innerShape = new Path
                        {
                            Tag = "ground",
                            Data =
                                SlideShapeMaker.MakePSlotHole(j, groundLink,  xOffset, yOffset, jointSize,
                                    startingBufferRadius),
                            Stroke = new SolidColorBrush(Colors.Black),
                            Fill = new SolidColorBrush(Colors.White),
                            StrokeThickness = strokeThickness
                        };
                        outerShape = new Path
                        {
                            Tag = "ground",
                            Data =
                                SlideShapeMaker.MakePSlotBorder(j, groundLink, xOffset,  yOffset, jointSize,
                                    startingBufferRadius),
                            Fill = new ImageBrush
                            {
                                ImageSource = new BitmapImage(new Uri("../groundhashMED.png", UriKind.Relative)),
                                Stretch = Stretch.UniformToFill
                                //RelativeTransform = new ScaleTransform{ScaleX = 1.0,ScaleY = 1.0}
                            // in order to do this, you will need code to handle the tiling - not native to Silverlight
                            // use Shazzam (http://shazzam-tool.com/) to make the shader fx
                            // then use http://silverscratch.blogspot.com/2010/09/tiled-image-brush-for-silverlight.html
                            // or 
                            }   
                            //Fill = new SolidColorBrush(Colors.DarkGray)
                        };
                        break;
                    case JointTypes.RP:
                        if (j.SlidingWithRespectTo(groundLink))
                        {
                            innerShape = new Path
                            {
                                Tag = "ground",
                                Data =
                                    SlideShapeMaker.MakeRPSlotHole(j, groundLink,  xOffset,  yOffset, jointSize,
                                        startingBufferRadius),
                                Stroke = new SolidColorBrush(Colors.Black),
                                Fill = new SolidColorBrush(Colors.White),
                                StrokeThickness = strokeThickness
                            };
                            outerShape = new Path
                            {
                                Tag = "ground",
                                Data =
                                    SlideShapeMaker.MakePSlotBorder(j, groundLink, xOffset,  yOffset, jointSize,
                                        startingBufferRadius),
                                Fill = new ImageBrush
                                {
                                    ImageSource = new BitmapImage()
                                }
                            };
                        }
                        else
                        {
                            innerShape = new Ellipse
                            {
                                Stroke = new SolidColorBrush(Colors.Black),
                                Fill = new SolidColorBrush(Colors.Black),
                                StrokeThickness = strokeThickness,
                                Name = "ground",
                                Height = jointSize,
                                Width = jointSize,
                                RenderTransform = new TranslateTransform { X = j.xInitial + xOffset, Y = j.yInitial + yOffset }

                            };
                            outerShape = new Polygon
                            {
                                Stroke = new SolidColorBrush(Colors.Black),
                                Fill = new SolidColorBrush(Colors.Black),
                                StrokeThickness = strokeThickness,
                                Name = "ground",
                                Points = new PointCollection
                            {
                                new Point(j.xInitial+xOffset, j.yInitial+yOffset),
                                new Point(j.xInitial+xOffset - triangleSideLength,j.yInitial+ yOffset - triangleSideLength),
                                new Point(j.xInitial+xOffset + triangleSideLength,j.yInitial+ yOffset - triangleSideLength)
                            },
                                //Width = DisplayConstants.UnCroppedDimension,
                                //Height = DisplayConstants.UnCroppedDimension
                            };
                        }
                        break;
                    default: //this would be gear
                        throw new NotImplementedException();
                }
                Shapes.Add(outerShape);
                Shapes.Add(innerShape);
            }
        }
        #endregion


    }

}
