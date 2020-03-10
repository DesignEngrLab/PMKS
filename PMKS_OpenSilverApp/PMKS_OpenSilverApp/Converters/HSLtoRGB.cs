using System;
using System.Globalization;
using System.Windows.Data;
using System.Windows.Media;

namespace PMKS_Silverlight_App
{
    public class AHSLtoARGBColor 
    {
        /// <summary>
        /// Converts the a, h, s, l values into a color.
        /// </summary>
       /// <param name="a">Opacity, must be in [0, 1].</param>
        /// <param name="h">Hue, must be in [0, 360].</param>
        /// <param name="s">Saturation, must be in [0, 1].</param>
        /// <param name="l">Luminance, must be in [0, 1].</param>
        /// <returns></returns>
        public static Color Convert(double a, double h, double s, double l)
        {
            if (s == 0)// achromatic color (gray scale)
                return Color.FromArgb((byte)((int)(a * 255.0)), (byte)((int)(l * 255.0)), (byte)((int)(l * 255.0)), (byte)((int)(l * 255.0)));


            double q = (l < 0.5) ? (l * (1.0 + s)) : (l + s - (l * s));
            double p = (2.0 * l) - q;

            double Hk = (h % 360.0) / 360.0;
            var T = new double[3];
            T[0] = Hk + (1.0 / 3.0);    // Tr
            T[1] = Hk;                // Tb
            T[2] = Hk - (1.0 / 3.0);    // Tg

            for (int i = 0; i < 3; i++)
            {
                if (T[i] < 0) T[i] += 1.0;
                if (T[i] > 1) T[i] -= 1.0;

                if ((T[i] * 6) < 1)
                {
                    T[i] = p + ((q - p) * 6.0 * T[i]);
                }
                else if ((T[i] * 2.0) < 1) //(1.0/6.0)<=T[i] && T[i]<0.5
                {
                    T[i] = q;
                }
                else if ((T[i] * 3.0) < 2) // 0.5<=T[i] && T[i]<(2.0/3.0)
                {
                    T[i] = p + (q - p) * ((2.0 / 3.0) - T[i]) * 6.0;
                }
                else T[i] = p;
            }

            return Color.FromArgb((byte)((int)(a * 255.0)), (byte)((int)(T[0] * 255.0)), (byte)((int)(T[1] * 255.0)), (byte)((int)(T[2] * 255.0)));
        }

    }
}