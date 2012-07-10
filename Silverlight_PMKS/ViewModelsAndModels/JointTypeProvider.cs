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
        private bool _posVisible=true;
        private bool _velocityVisible=true;
        private bool _accelerationVisible=true;
        public string JointType { get; set; }
        public string LinkNames { get; set; }
        public string XPos { get; set; }
        public string YPos { get; set; }
        public string Angle { get; set; }
        public Boolean PosVisible
        {
            get { return _posVisible; }
            set { _posVisible = value; }
        }

        public Boolean VelocityVisible
        {
            get { return _velocityVisible; }
            set { _velocityVisible = value; }
        }

        public Boolean AccelerationVisible
        {
            get { return _accelerationVisible; }
            set { _accelerationVisible = value; }
        }
    }
}