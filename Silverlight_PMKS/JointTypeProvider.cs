using System;
using System.Collections.Generic;

namespace PMKS_Silverlight_App
{
    public class JointTypeProvider
    {
        public List<string> jointTypeList
        {
            get { return new List<string> { "R (pin joint)", "P (sliding block)", "RP (pin in slot)", "G (gear teeth)" }; }
        }
    }

    public class JointData
    {
        public string JointType { get; set; }
        public string LinkNames { get; set; }
        public string XPos { get; set; }
        public string YPos { get; set; }
        public string Angle { get; set; }
        public Boolean PosVisible { get; set; }
        public Boolean VelocityVisible { get; set; }
        public Boolean AccelerationVisible { get; set; }
    }
}