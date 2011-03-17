using System;
using System.Collections.Generic;
using System.Text;
using GraphSynth.Representation;

namespace PlanarMechanismSimulator
{
    class DynamicMatrixTerm
    {
        public double defaultValue;
        public double value;
        public DynamicType type;
        public Direction dir;
        /* belongs to is predominantly used when pointing back to the node that 
         * is either the pivot or the link. When pointing back to the pivot, this
         * term will be of type absoluteXVelocity, absoluteYVelocity, absoluteXAcceleration,
         * or absoluteYAcceleration. When pointing back to the link it will be of type
         * angularAcceleration, or angularVelocity. */
        public node belongsTo = null;
        /* belongsFrom is only used for the radialAcceleration, radialVelocity in which
         * we are defining the relative velocity and relative acceleration between two 
         * pivots - these are also known as (used for): coriolis acceleration and slip
         * acceleration. The name is a play on words of the fact that we are really like
         * an arc here connecting to nodes. */
        public node belongsFrom = null;

        /* borrowed from the example at http://msdn.microsoft.com/en-us/library/z5z9kes2.aspx */
        public DynamicMatrixTerm(double d) { value = d; }

        public DynamicMatrixTerm() { }

        // User-defined conversion from Digit to double
        public static implicit operator double(DynamicMatrixTerm d)
        {
            return d.value;
        }
        //  User-defined conversion from double to Digit
        public static implicit operator DynamicMatrixTerm(double d)
        {
            return new DynamicMatrixTerm(d);
        }

        public void Reset()
        {
            value = defaultValue;
        }
    }
    enum DynamicType
    {
        angularAcceleration, angularVelocity, radialAcceleration, radialVelocity,
        absoluteVelocity, absoluteAcceleration
    };
    enum Direction
    {
        X, Y, Z, relative
    };
}
