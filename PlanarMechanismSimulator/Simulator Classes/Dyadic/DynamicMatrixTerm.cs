namespace PlanarMechanismSimulator
{
    internal class PivotDynamicMatrixTerm : DynamicMatrixTerm
    {
        internal joint belongsFrom;
        internal joint belongsTo;
    }

    internal  class LinkDynamicMatrixTerm:DynamicMatrixTerm
        {
            internal link belongsTo;
        }

        internal abstract class DynamicMatrixTerm
    {
        internal double defaultValue;
        internal Direction dir;
        internal DynamicType type;
        internal double value;
        /* belongs to is predominantly used when pointing back to the node that 
         * is either the pivot or the link. When pointing back to the pivot, this
         * term will be of type absoluteXVelocity, absoluteYVelocity, absoluteXAcceleration,
         * or absoluteYAcceleration. When pointing back to the link it will be of type
         * angularAcceleration, or angularVelocity. */

        /* borrowed from the example at http://msdn.microsoft.com/en-us/library/z5z9kes2.aspx */

        internal DynamicMatrixTerm(double d)
        {
            value = d;
        }

        internal DynamicMatrixTerm()
        {
        }

        // User-defined conversion from Digit to double
            public static implicit operator double(DynamicMatrixTerm d)
        {
            return d.value;
        }

        internal void Reset()
        {
            value = defaultValue;
        }
    }

    internal enum DynamicType
    {
        angularAcceleration,
        angularVelocity,
        radialAcceleration,
        radialVelocity,
        absoluteVelocity,
        absoluteAcceleration
    };

    internal enum Direction
    {
        X,
        Y,
        Z,
        relative
    };
}