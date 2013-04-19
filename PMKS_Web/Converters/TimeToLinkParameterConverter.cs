using System;
using System.Collections.Generic;
using System.Globalization;
using System.Windows.Data;
using PMKS_Silverlight_App;
using PlanarMechanismSimulator;

namespace PMKS_Silverlight_App
{
    public class TimeToLinkParameterConverter : TimeToJointParameterConverter
    {
        private readonly int linkRowIndex;
        private static List<double[,]> linkParameters;

        public TimeToLinkParameterConverter(link l, joint j, StateVariableType linkState, Simulator pmks)
            : base(j, linkState, pmks)
        {
            linkRowIndex = pmks.AllLinks.IndexOf(l);
            linkParameters = pmks.LinkParameters.Parameters;
            result = new double[3];
        }

        public override object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
           var result2 = (double[])base.Convert(value, targetType, parameter, culture);
            result = new[] {result2[0], result2[1], double.NaN};
            switch (ColIndex)
            {
                case 0:
                    result[2] = Simulator.FindAngleAtTime(tau, deltaTime,
                                                             linkParameters[prevIndex][linkRowIndex, 0],
                                                             linkParameters[nextIndex][linkRowIndex, 0],
                                                             linkParameters[prevIndex][linkRowIndex, 1],
                                                             linkParameters[nextIndex][linkRowIndex, 1],
                                                             linkParameters[prevIndex][linkRowIndex, 2],
                                                             linkParameters[nextIndex][linkRowIndex, 2]);
                    break;
                case 1:
                    result[2] = Simulator.FindVelocityatTime(tau, deltaTime,
                                                             linkParameters[prevIndex][linkRowIndex, 1],
                                                             linkParameters[nextIndex][linkRowIndex, 1],
                                                             linkParameters[prevIndex][linkRowIndex, 2],
                                                             linkParameters[nextIndex][linkRowIndex, 2]);
                    break;
                default:
                    result[2] = Simulator.FindAccelerationatTime(tau, deltaTime,
                                                                 linkParameters[prevIndex][linkRowIndex, 2],
                                                                 linkParameters[nextIndex][linkRowIndex, 2]);
                    break;
            }
            return result;
        }
    }
}

