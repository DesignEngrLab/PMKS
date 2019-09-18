using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PMKS
{

    /// <summary>
    /// The force class, which represents a force on the mechanism.
    /// </summary>
    public class Force
    {
        public double xloc;
        public double yloc;
        public double xmag;
        public double ymag;
        public int onlink;
        public bool isfixed;

        public struct Point
        {
            /// <summary>
            /// The x coordinate.
            /// </summary>
            public double X;

            /// <summary>
            /// The y coordinate.
            /// </summary>
            public double Y;



            /// <summary>
            /// Initializes a new instance of the <see cref="Point" /> struct.
            /// </summary>
            /// <param name="x">The x.</param>
            /// <param name="y">The y.</param>
            public Point(double x, double y)
            {
                this.X = x;
                this.Y = y;
            }
        }

        public Force()
        {
            this.onlink = 999;
        }

        public Force(double xloc, double yloc, double xmag, double ymag, bool isfixed)
        {
            this.xloc = xloc;
            this.yloc = yloc;
            this.xmag = xmag;
            this.ymag = ymag;
            this.isfixed = isfixed;
        }

        public double xInitial { get; set; }
        public double yInitial { get; set; }
        public double Xmag { get; set; }
        public double Ymag { get; set; }
        public int Onlink { get; set; }
        public bool Isfixed { get; set; }
        /// Gets or sets the x.
        /// </summary>
        /// <value>The x.</value>
        internal double x { get; set; }
        /// <summary>
        /// Gets or sets the y.
        /// </summary>
        /// <value>The y.</value>
        internal double y { get; set; }

        internal Force Copy()
        {
            return new Force
            {
                xloc = xloc,
                yloc = yloc,
                xInitial = xInitial,
                yInitial = yInitial,
                xmag = xmag,
                ymag = ymag,
                isfixed = isfixed,
                Onlink = Onlink,
                onlink = onlink,
                Xmag = Xmag,
                Ymag = Ymag,
                Isfixed = Isfixed

            };
        }
    }
}
