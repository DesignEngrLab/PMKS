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
}
