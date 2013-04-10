using System;
using System.Collections.Generic;
using System.Globalization;
using System.Windows.Data;
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public abstract class TimeToParameterBaseConverter : IValueConverter
    {
        protected int ColIndex;
        protected int RowIndex;

        private static int lastIndex;
        protected static double lastTime;
        protected static int prevIndex;
        protected static int nextIndex;
        protected static double prevTime;
        protected static double nextTime;
        protected static double tau;
        protected static double deltaTime;

        protected static List<double[,]> parameters;
        protected static List<double> times;
        private static double timePeriod;
        private static bool cyclic;

        protected TimeToParameterBaseConverter(Simulator pmks)
        {
            cyclic = pmks.CompleteCycle && !pmks.AdditionalGearCycling;
            times = pmks.JointParameters.Times;
            parameters = pmks.JointParameters.Parameters;
            lastIndex = times.Count - 1;
            lastTime = times[lastIndex];
            prevIndex = nextIndex = 0;
            prevTime = nextTime = times[0];
            tau = deltaTime = 0.0;
            timePeriod = 2 * Math.PI / pmks.InputSpeed;
        }


        public abstract object Convert(object value, Type targetType, object parameter, CultureInfo culture);

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }

        protected static void SetTimeIndices(double currentTime)
        {
            if (currentTime == lastTime) return; /* you are at a same time step - no need to change static vars. */
            /* you are at a new time step */
            if (currentTime >= prevTime && currentTime <= nextTime)
            {
                /* cool. You're still in the same time step. This means we just need to change prevDeltaTime and nextDeltaTime. */
            }
            else
            {
                while (currentTime < prevTime)
                {
                    if (prevIndex == 0)
                    {
                        if (!cyclic) break;
                        currentTime += timePeriod;
                        nextIndex = lastIndex;
                        nextTime = times[nextIndex];
                        prevIndex = lastIndex - 1;
                        prevTime = times[prevIndex];
                    }
                    else
                    {
                        nextIndex = prevIndex;
                        nextTime = prevTime;
                        prevIndex--;
                        prevTime = times[prevIndex];
                    }
                }
                while (currentTime > nextTime)
                {
                    if (nextIndex == lastIndex)
                    {
                        if (!cyclic) break;
                        currentTime += timePeriod;
                        nextIndex = 1;
                        nextTime = times[1];
                        prevIndex = 0;
                        prevTime = times[0];
                    }
                    else
                    {
                        prevIndex = nextIndex;
                        prevTime = nextTime;
                        nextIndex++;
                        nextTime = times[nextIndex];
                    }
                }
            }
            tau = currentTime - prevTime;
            deltaTime = nextTime - prevTime;
            lastTime = currentTime;
        }
    }
}
