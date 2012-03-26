using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class PlanarMechanismSimulator : IDependentAnalysis
    {
        #region Properties
        /// <summary>
        /// Gets the pivot parameters.
        /// </summary>
        //public double[, ,] PivotParameters { get; private set; }
        public SortedList<double, double[,]> PivotParameters;
        /// <summary>
        /// Gets the link parameters.
        /// </summary>
        //public double[, ,] LinkParameters { get; private set; }
        public SortedList<double, double[,]> LinkParameters;

        private double _eps = 1e-9;
        public double epsilon
        {
            get { return _eps; }
            set { _eps = value; }
        }

        public double DeltaAngle { get; set; }
        public double InputAngularSpeed { get; set; }
        public double timeStep
        {
            get { return DeltaAngle / InputAngularSpeed; }
        }

        #endregion

        #region Set by the Topology (from the Constructor)
        private readonly int n; //no. of links
        private readonly int p; //no. of  pivots
        private readonly link[] links;
        private readonly pivot[] pivots;
        private readonly int inputIndex;
        private readonly pivot inputpivot;
        #endregion

        #region Constructor - requires the topology of the mechanism to be provided.
        /// <summary>
        /// Initializes a new instance of the <see cref="PlanarMechanismSimulator"/> class.
        /// </summary>
        /// <param name="InputIndex">Index of the input.</param>
        /// <param name="Connections">The connections.</param>
        /// <param name="PivotTypes">The pivot types.</param>
        public PlanarMechanismSimulator(int InputIndex, int[,] Connections, IList<string> PivotTypes)
        {
            try
            {
                inputIndex = InputIndex;
                n = Connections.GetLength(1); //count the no of links in the system
                p = PivotTypes.Count; //count the no of pivots in the system
                links = new link[n]; //create a array of LINKS
                pivots = new pivot[p]; //create a arry of pivots
                for (int i = 0; i < p; i++)
                    pivots[i] = new pivot(i, i > inputIndex, PivotTypes[i]) { X = double.NaN, Y = double.NaN };
                inputpivot = pivots[inputIndex];
                for (int i = 0; i < n; i++)
                {
                    var p0 = pivots[Connections[i, 0]];
                    var p1 = pivots[Connections[i, 1]];
                    links[i] = new link(i, double.NaN, (p0.IsGround && p1.IsGround));
                    links[i].Pivots = new List<pivot> { p0, p1 };
                    p0.Links.Add(links[i]);
                    p1.Links.Add(links[i]);
                }
            }
            catch (Exception e)
            {
                throw new Exception(
                    "Failed to construct Planar Mechanism Simulator from topology data (InputIndex, Connections, PivotTypes).",
                    e);
            }
        }

        #endregion

        /// <summary>
        /// Assigns the initial positions of all the pivots.
        /// </summary>
        /// <param name="InitPositions">The init positions.</param>
        public void AssignPositions(double[,] InitPositions)
        {
            try
            {
                for (int i = 0; i < p; i++)
                {
                    pivots[i].X = InitPositions[i, 0];
                    pivots[i].Y = InitPositions[i, 1];
                }
                for (int i = 0; i < n; i++)
                    links[i].length = Math.Sqrt((links[i].Pivots[0].X - links[i].Pivots[1].X) * (links[i].Pivots[0].X - links[i].Pivots[1].X)
                        + (links[i].Pivots[0].Y - links[i].Pivots[1].Y) * (links[i].Pivots[0].Y - links[i].Pivots[1].Y));
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign positions to topology (see inner exeception).", e);
            }
        }
        /// <summary>
        /// Assigns the lengths of the links.
        /// </summary>
        /// <param name="Lengths">The lengths.</param>
        public void AssignLengths(double[] Lengths)
        {
            try
            {
                for (int i = 0; i < n; i++)
                    links[i].length = Lengths[i];
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign lengths to topology (see inner exeception).", e);
            }
        }
        /// <summary>
        /// Finds the initial positions from lengths.
        /// </summary>
        /// <param name="inputX">The input X.</param>
        /// <param name="inputY">The input Y.</param>
        /// <param name="gnd1X">The GND1 X.</param>
        /// <param name="gnd1Y">The GND1 Y.</param>
        /// <param name="gnd2X">The GND2 X.</param>
        /// <returns></returns>
        public Boolean FindInitialPositionsFromLengths(double inputX, double inputY, double gnd1X, double gnd1Y, double gnd2X)
        {
            try
            {
                if (links.Any(a => double.IsNaN(a.length))) throw new Exception("Link lengths for all links need to be set first. Use AssignLengths method.");
                var inputLink = links.First(a => a.Pivots.Contains(inputpivot) && a.Pivots.Contains(pivots[inputIndex + 1]));
                if (Math.Abs(inputLink.length - Math.Sqrt((inputX - gnd1X) * (inputX - gnd1X) + (inputY - gnd1Y) * (inputY - gnd1Y))) > epsilon)
                    throw new Exception("Input and first ground position do not match expected lenght of " + inputLink.length);
                inputpivot.X = inputX;
                inputpivot.Y = inputY;
                pivots[inputIndex + 1].X = gnd1X;
                pivots[inputIndex + 1].Y = gnd1Y;
                pivots[inputIndex + 2].X = gnd2X;
                var g1_to_g2Link = links.First(a => a.Pivots.Contains(pivots[inputIndex + 1]) && a.Pivots.Contains(pivots[inputIndex + 2]));
                pivots[inputIndex + 2].Y = gnd1Y + Math.Sqrt(g1_to_g2Link.length * g1_to_g2Link.length - ((gnd1X - gnd2X) * (gnd1X - gnd2X)));
                return epsilon > FindInitialPositionMain();
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign positions from lengths (see inner exeception).", e);
            }
        }

        /// <summary>
        /// Finds the initial positions from lengths.
        /// </summary>
        /// <param name="inputX">The input X.</param>
        /// <param name="inputY">The input Y.</param>
        /// <param name="AngleToGnd1">The angle to GND1.</param>
        /// <param name="AngleToGnd2">The angle to GND2.</param>
        /// <returns></returns>
        public Boolean FindInitialPositionsFromLengths(double inputX, double inputY, double AngleToGnd1, double AngleToGnd2)
        {
            try
            {
                if (links.Any(a => double.IsNaN(a.length))) throw new Exception("Link lengths for all links need to be set first. Use AssignLengths method.");
                var inputLink = links.First(a => a.Pivots.Contains(inputpivot) && a.Pivots.Contains(pivots[inputIndex + 1]));
                inputpivot.X = inputX;
                inputpivot.Y = inputY;
                pivots[inputIndex + 1].X = inputX + Math.Cos(AngleToGnd1) * inputLink.length;
                pivots[inputIndex + 1].Y = inputY + Math.Sin(AngleToGnd1) * inputLink.length;
                var g1_to_g2Link = links.First(a => a.Pivots.Contains(pivots[inputIndex + 1]) && a.Pivots.Contains(pivots[inputIndex + 2]));
                pivots[inputIndex + 2].X = pivots[inputIndex + 1].X + Math.Cos(AngleToGnd2) * g1_to_g2Link.length;
                pivots[inputIndex + 2].Y = pivots[inputIndex + 1].Y + +Math.Cos(AngleToGnd2) * g1_to_g2Link.length;
                return epsilon > FindInitialPositionMain();
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign positions from lengths (see inner exeception).", e);
            }
        }
        double FindInitialPositionMain()
        {
            var newPivots = new List<pivot>(pivots);
            var newInputIndex = inputIndex;
            while (newInputIndex < p - 3)
            {
                var lastGnd = newPivots[p - 1];
                newPivots.RemoveAt(p - 1);
                newPivots.Insert(0, lastGnd);
                newInputIndex++;
            }
            var nonDyadicPositionFinder = new NonDyadicPositionFinder(links, newPivots, newInputIndex, epsilon);
            double[] x, xStar;
            var r = new Random();
            var fStar = double.PositiveInfinity;
            long numFEvals = 0;
            do
            {
                numFEvals += nonDyadicPositionFinder.NumEvals;
                for (int i = 0; i < x.GetLength(0); i++)
                    x[i] = 20 * r.NextDouble() - 10;
                fStar = nonDyadicPositionFinder.Run(out xStar, x);
                //SearchIO.output("fStar = " + fStar);
            } while (!nonDyadicPositionFinder.SolutionFound());

            return fStar;

        }

        /// <summary>
        /// Finds the next position.
        /// </summary>
        /// <param name="X">The X.</param>
        /// <param name="Y">The Y.</param>
        /// <param name="InputIndex">Index of the input.</param>
        public void FindNextPosition(double X, double Y, int InputIndex = -1)
        {

        }

        /// <summary>
        /// Finds the next position.
        /// </summary>
        /// <param name="DeltaAngle">The delta angle.</param>
        /// <param name="InputIndex">Index of the input.</param>
        public Boolean FindNextPosition(double DeltaAngle = double.NaN, int InputIndex = -1)
        {

        }


        private double[,] FindNextPosition(double[,] currentPivotParams, double timeStep)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Finds the full movement.
        /// </summary>
        /// <param name="DeltaAngle">The delta angle.</param>
        /// <param name="InputAngularSpeed">The input angular speed.</param>
        public void FindFullMovement(double DeltaAngle = double.NaN, double InputAngularSpeed = double.NaN)
        {
            if (!double.IsNaN(DeltaAngle)) this.DeltaAngle = DeltaAngle;
            if (!double.IsNaN(InputAngularSpeed)) this.InputAngularSpeed = InputAngularSpeed;
            if ((double.IsNaN(this.DeltaAngle)) && (double.IsNaN(this.InputAngularSpeed)))
                throw new Exception("The angle delta and the input angular speed were not specified in FindFullMovement.");
            if (double.IsNaN(this.DeltaAngle))
                throw new Exception("The angle delta was not specified in FindFullMovement.");
            if (double.IsNaN(this.InputAngularSpeed))
                throw new Exception("The input angular speed was not specified in FindFullMovement.");

            PivotParameters = new SortedList<double, double[,]>();
            LinkParameters = new SortedList<double, double[,]>();

            if (isDyadic()) FindFullMovementDyadic();
            else FindFullMovementNonDyadic();
        }

        void FindFullMovementDyadic()
        {
            var currentTime = 0.0;
            var currentPivotParams = new double[p, 6];
            for (int i = 0; i < p; i++)
            {
                currentPivotParams[i, 0] = pivots[i].X;
                currentPivotParams[i, 1] = pivots[i].Y;
            }
            PivotParameters.Add(currentTime, currentPivotParams);
            LinkParameters.Add(currentTime, new double[n, 2]);
            bool validPosition;
            findImmediatePivots();

            do /*** Stepping Forward in Time **/
            {
                #region Find Velocities for Current Position
                findImmediateICs();
                double[,] currentLinkParams;
                if (findSecondaryICs())
                {
                    currentLinkParams = findAngularVelocities();
                    LinkParameters.Add(currentTime, currentLinkParams);
                    findLinearVelocities(currentPivotParams);
                    //check slip velocities
                    findLinearSlipVelocities(currentPivotParams);
                    //find slip velocities and update
                }
                else
                {
                    SearchIO.output("Instant Centers could not be found");
                    NumericalVelocity(currentTime);
                }
                #endregion
                #region Find Accelerations for Current Position
                if (!findAcceleration(currentLinkParams, currentPivotParams))
                    NumericalAcceleration(currentTime);
                #endregion
                #region Find Next Positions
                validPosition = FindNextPosition(currentPivotParams, timeStep);
                if (validPosition)
                {
                    currentPivotParams = new double[p, 6];
                    for (int i = 0; i < p; i++)
                    {
                        currentPivotParams[i, 0] = pivots[i].X;
                        currentPivotParams[i, 1] = pivots[i].Y;
                    }
                    currentTime += timeStep;
                    PivotParameters.Add(currentTime, currentPivotParams);
                    LinkParameters.Add(currentTime, new double[n, 2]);
                }
                #endregion
            } while (validPosition && lessThanFullRotation());
            if (!lessThanFullRotation()) return;
            #region Reset to t=0
            currentPivotParams = PivotParameters.Values[0];
            currentTime = PivotParameters.Keys[0];
            for (int i = 0; i < p; i++)
            {
                pivots[i].X = currentPivotParams[i, 0];
                pivots[i].Y = currentPivotParams[i, 1];
            }
            #endregion
            #region Find first backwards step positions
            validPosition = FindNextPosition(currentPivotParams, -timeStep);
            if (validPosition)
            {
                currentPivotParams = new double[p, 6];
                for (int i = 0; i < p; i++)
                {
                    currentPivotParams[i, 0] = pivots[i].X;
                    currentPivotParams[i, 1] = pivots[i].Y;
                }
                currentTime += timeStep;
                PivotParameters.Add(currentTime, currentPivotParams);
                LinkParameters.Add(currentTime, new double[n, 2]);
            }
            if (!validPosition) return;
            #endregion
            do /*** Stepping Backward in Time **/
            {
                #region Find Velocities for Current Position
                findImmediateICs();
                double[,] currentLinkParams;
                if (findSecondaryICs())
                {
                    currentLinkParams = findAngularVelocities();
                    LinkParameters.Add(currentTime, currentLinkParams);
                    findLinearVelocities(currentPivotParams);
                    //check slip velocities
                    findLinearSlipVelocities(currentPivotParams);
                    //find slip velocities and update
                }
                else
                {
                    SearchIO.output("Instant Centers could not be found");
                    NumericalVelocity(currentTime);
                }
                #endregion
                #region Find Accelerations for Current Position
                if (!findAcceleration(currentLinkParams, currentPivotParams))
                    NumericalAcceleration(currentTime);
                #endregion
                #region Find Next Positions
                validPosition = FindNextPosition(currentPivotParams, -timeStep);
                if (validPosition)
                {
                    currentPivotParams = new double[p, 6];
                    for (int i = 0; i < p; i++)
                    {
                        currentPivotParams[i, 0] = pivots[i].X;
                        currentPivotParams[i, 1] = pivots[i].Y;
                    }
                    currentTime += timeStep;
                    PivotParameters.Add(currentTime, currentPivotParams);
                    LinkParameters.Add(currentTime, new double[n, 2]);
                }
                #endregion
            } while (validPosition && lessThanFullRotation());
        }

        private bool lessThanFullRotation()
        {
            var timeSpan = PivotParameters.Keys[PivotParameters.Values.Count - 1]
                           - PivotParameters.Keys[0];
            return (timeSpan + timeStep < 2 * Math.PI / InputAngularSpeed);
        }




        private void FindFullMovementNonDyadic()
               {
            var currentTime = 0.0;
            var currentPivotParams = new double[p, 6];
            for (int i = 0; i < p; i++)
            {
                currentPivotParams[i, 0] = pivots[i].X;
                currentPivotParams[i, 1] = pivots[i].Y;
            }
            PivotParameters.Add(currentTime, currentPivotParams);
            LinkParameters.Add(currentTime, new double[n, 2]);
            bool validPosition;

            do /*** Stepping Forward in Time **/
            {
                    NumericalVelocity(currentTime);
                    NumericalAcceleration(currentTime);
                #region Find Next Positions
                validPosition = FindNextPosition(currentPivotParams, timeStep);
                if (validPosition)
                {
                    currentPivotParams = new double[p, 6];
                    for (int i = 0; i < p; i++)
                    {
                        currentPivotParams[i, 0] = pivots[i].X;
                        currentPivotParams[i, 1] = pivots[i].Y;
                    }
                    currentTime += timeStep;
                    PivotParameters.Add(currentTime, currentPivotParams);
                    LinkParameters.Add(currentTime, new double[n, 2]);
                }
                #endregion
            } while (validPosition && lessThanFullRotation());
            if (!lessThanFullRotation()) return;
            #region Reset to t=0
            currentPivotParams = PivotParameters.Values[0];
            currentTime = PivotParameters.Keys[0];
            for (int i = 0; i < p; i++)
            {
                pivots[i].X = currentPivotParams[i, 0];
                pivots[i].Y = currentPivotParams[i, 1];
            }
            #endregion
            #region Find first backwards step positions
            validPosition = FindNextPosition(currentPivotParams, -timeStep);
            if (validPosition)
            {
                currentPivotParams = new double[p, 6];
                for (int i = 0; i < p; i++)
                {
                    currentPivotParams[i, 0] = pivots[i].X;
                    currentPivotParams[i, 1] = pivots[i].Y;
                }
                currentTime -= timeStep;
                PivotParameters.Add(currentTime, currentPivotParams);
                LinkParameters.Add(currentTime, new double[n, 2]);
            }
            if (!validPosition) return;
            #endregion
            do /*** Stepping Backward in Time **/
            {
                    NumericalVelocity(currentTime);
                    NumericalAcceleration(currentTime);
                #region Find Next Positions
                validPosition = FindNextPosition(currentPivotParams, -timeStep);
                if (validPosition)
                {
                    currentPivotParams = new double[p, 6];
                    for (int i = 0; i < p; i++)
                    {
                        currentPivotParams[i, 0] = pivots[i].X;
                        currentPivotParams[i, 1] = pivots[i].Y;
                    }
                    currentTime += timeStep;
                    PivotParameters.Add(currentTime, currentPivotParams);
                    LinkParameters.Add(currentTime, new double[n, 2]);
                }
                #endregion
            } while (validPosition && lessThanFullRotation());
        }


        /// <summary>
        /// Calculates the full movement for a set of new initial positions stored in x.
        /// this is nearly the same as the function above it.
        /// </summary>
        /// <param name="x">The x.</param>
        public void calculate(double[] x)
        {
            throw new NotImplementedException();
        }

        private Boolean isDyadic()
        {
            throw new NotImplementedException();
        }
        /// <summary>
        /// Finds the difference from target path.
        /// </summary>
        /// <param name="TargetPath">The target path.</param>
        /// <returns></returns>
        public double FindDifferenceFromTargetPath(double[, ,] TargetPath)
        {
            return 0;
        }

        /// <summary>
        /// Optimizes the positions for target path.
        /// </summary>
        /// <param name="TargetPath">The target path.</param>
        /// <returns></returns>
        public double OptimizePositionsForTargetPath(double[, ,] TargetPath)
        {
            return 0;
        }

        /// <summary>
        /// Optimizes the input crank.
        /// </summary>
        /// <returns></returns>
        public double OptimizeInputCrank()
        {
            return 0;
        }

    }
}
