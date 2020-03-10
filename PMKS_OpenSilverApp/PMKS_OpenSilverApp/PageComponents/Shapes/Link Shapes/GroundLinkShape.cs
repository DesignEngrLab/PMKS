using System;
using System.Linq;
using System.Windows.Media.Imaging;
using PMKS;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Shapes;
using PMKS_Silverlight_App;
using Point = System.Windows.Point;

namespace PMKS_Silverlight_App
{
    public class GroundLinkShape : Panel
    {
        private const int grayscale = 180;
        private const int opacity = 180;
        #region Constructor
        public GroundLinkShape(Link groundLink, double xOffset, double yOffset, double strokeThickness, double jointSize,
                            double startingBufferRadius)
        {
            Name = "ground";
            Canvas.SetZIndex(this, 0);
            double triangleSideLength = 2.5 * jointSize;
            foreach (var j in groundLink.joints)
            {
                switch (j.TypeOfJoint)
                {
                    case JointType.R:

                        Children.Add(new Line
                        {
                            Stroke = new SolidColorBrush(Colors.Black),
                            StrokeThickness = strokeThickness,
                            X1 = j.xInitial + xOffset - 2 * triangleSideLength,
                            Y1 = j.yInitial + yOffset - triangleSideLength,
                            X2 = j.xInitial + xOffset + 2 * triangleSideLength,
                            Y2 = j.yInitial + yOffset - triangleSideLength,
                        });
                        Children.Add(new Polygon
                        {
                            Stroke = new SolidColorBrush(Colors.Black),
                            Fill = new SolidColorBrush(Colors.Black),
                            StrokeThickness = strokeThickness,
                            Points = new PointCollection
                            {
                                new Point(j.xInitial+xOffset,j.yInitial+ yOffset),
                                new Point(j.xInitial+xOffset - triangleSideLength,j.yInitial+ yOffset - triangleSideLength),
                                new Point(j.xInitial+xOffset + triangleSideLength,j.yInitial+ yOffset - triangleSideLength)    
                     
                            }
                        });
                        break;
                    case JointType.P:
                        if (j.SlidingWithRespectTo(groundLink))
                        {
                            Children.Add(new Path
                            {
                                Data =
                                    SlideShapeMaker.MakePSlotBorder(j, groundLink, xOffset, yOffset, jointSize,
                                        startingBufferRadius),
                                //Fill = new ImageBrush
                                //{
                                //    ImageSource = new BitmapImage(new Uri("../Properties/groundhashMED.png", UriKind.Relative)),
                                //    Stretch = Stretch.UniformToFill
                                //    //RelativeTransform = new ScaleTransform{ScaleX = 1.0,ScaleY = 1.0}
                                //    // in order to do this, you will need code to handle the tiling - not native to Silverlight
                                //    // use Shazzam (http://shazzam-tool.com/) to make the shader fx
                                //    // then use http://silverscratch.blogspot.com/2010/09/tiled-image-brush-for-silverlight.html
                                //    // or 
                                //}
                                Fill =
                                    new SolidColorBrush(new Color
                                    {
                                        A = opacity,
                                        B = grayscale,
                                        G = grayscale,
                                        R = grayscale
                                    })
                            });
                            Children.Add(new Path
                            {
                                Data =
                                    SlideShapeMaker.MakePSlotHole(j, groundLink, xOffset, yOffset, jointSize,
                                        startingBufferRadius),
                                Stroke = new SolidColorBrush(Colors.Black),
                                Fill = new SolidColorBrush(Colors.White),
                                StrokeThickness = strokeThickness
                            });
                        }
                        else
                        {
                            Children.Add(new Path
                            {
                                Data =
                                    SlideShapeMaker.MakePSlotBorder(j, groundLink, xOffset, yOffset, jointSize,
                                        startingBufferRadius,true),
                                Fill =
                                    new SolidColorBrush(new Color
                                    {
                                        A = opacity,
                                        B = grayscale,
                                        G = grayscale,
                                        R = grayscale
                                    })
                            });
                        }

                        break;
                    case JointType.RP:
                        if (j.SlidingWithRespectTo(groundLink))
                        {
                            Children.Add(new Path
                              {
                                  Data =
                                      SlideShapeMaker.MakePSlotBorder(j, groundLink, xOffset, yOffset, jointSize,
                                          startingBufferRadius),
                                  Fill = new SolidColorBrush(new Color { A = opacity, B = grayscale, G = grayscale, R = grayscale })
                                  //Fill = new ImageBrush
                                  //{
                                  //    ImageSource = new BitmapImage(new Uri("../Properties/groundhashMED.png", UriKind.Relative)),
                                  //    Stretch = Stretch.UniformToFill
                                  //    //RelativeTransform = new ScaleTransform{ScaleX = 1.0,ScaleY = 1.0}
                                  //    // in order to do this, you will need code to handle the tiling - not native to Silverlight
                                  //    // use Shazzam (http://shazzam-tool.com/) to make the shader fx
                                  //    // then use http://silverscratch.blogspot.com/2010/09/tiled-image-brush-for-silverlight.html
                                  //    // or 
                                  //}
                              });
                            Children.Add(new Path
                              {
                                  Data =
                                      SlideShapeMaker.MakeRPSlotHole(j, groundLink, xOffset, yOffset, jointSize,
                                          startingBufferRadius),
                                  Stroke = new SolidColorBrush(Colors.Black),
                                  Fill = new SolidColorBrush(Colors.White),
                                  StrokeThickness = strokeThickness
                              });
                        }
                        else
                        {
                            Children.Add(new Ellipse
                            {
                                Stroke = new SolidColorBrush(Colors.Black),
                                Fill = new SolidColorBrush(Colors.Black),
                                StrokeThickness = strokeThickness,
                                Height = jointSize,
                                Width = jointSize,
                                RenderTransform = new TranslateTransform { X = j.xInitial + xOffset, Y = j.yInitial + yOffset }

                            });
                            Children.Add(new Polygon
                            {
                                Stroke = new SolidColorBrush(Colors.Black),
                                Fill = new SolidColorBrush(Colors.Black),
                                StrokeThickness = strokeThickness,
                                Points = new PointCollection
                            {
                                new Point(j.xInitial+xOffset, j.yInitial+yOffset),
                                new Point(j.xInitial+xOffset - triangleSideLength,j.yInitial+ yOffset - triangleSideLength),
                                new Point(j.xInitial+xOffset + triangleSideLength,j.yInitial+ yOffset - triangleSideLength)
                            }
                            });
                        }
                        break;
                    default: //this would be gear
                        throw new NotImplementedException();
                }
            }
        }
        #endregion


    }

}
