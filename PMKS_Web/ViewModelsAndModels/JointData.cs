using System;
using System.Collections.Generic;

namespace PMKS_Silverlight_App
{
    public class JointData
    {
        private bool _posVisible = true;
        private bool _velocityVisible = true;
        private bool _accelerationVisible = true;
        private double _xPos = double.NaN;
        private string _yPos;
        private string _angle;
        private string _jointType;
        private string _linkNames;

        public string JointType
        {
            get { return _jointType; }
            set
            {
                if (string.IsNullOrWhiteSpace(value)) _jointType = "";
                else _jointType = value.Split(',', ' ')[0];
                PMKSControl.ParseData();
            }
        }

        public string LinkNames
        {
            get { return _linkNames; }
            set
            {
                _linkNames = value;
                PMKSControl.ParseData();
            }
        }

        public string XPos
        {
            get { return (double.IsNaN(_xPos)) ? "" : _xPos.ToString(); }
            set
            {
                if (!double.TryParse(value, out _xPos))
                    _xPos = double.NaN;
                PMKSControl.ParseData();
            }
        }

        public string YPos
        {
            get { return _yPos; }
            set
            {
                _yPos = value;
                PMKSControl.ParseData();
            }
        }

        public string Angle
        {
            get { return _angle; }
            set
            {
                _angle = value;
                PMKSControl.ParseData();
            }
        }

        public Boolean PosVisible
        {
            get { return _posVisible; }
            set
            {
                _posVisible = value;
                PMKSControl.ParseData();
            }
        }

        public Boolean VelocityVisible
        {
            get { return _velocityVisible; }
            set
            {
                _velocityVisible = value;
                PMKSControl.ParseData();
            }
        }

        public Boolean AccelerationVisible
        {
            get { return _accelerationVisible; }
            set
            {
                _accelerationVisible = value;
                PMKSControl.ParseData();
            }
        }
    }
}