using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class Simulator : IDependentAnalysis
    {
        #region Properties

        /// <summary>
        /// Gets the pivot parameters.
        /// </summary>
        //public double[, ,] PivotParameters { get; private set; }
        public SortedList<double, double[,]> JointParameters;

        /// <summary>
        /// Gets the link parameters.
        /// </summary>
        //public double[, ,] LinkParameters { get; private set; }
        public SortedList<double, double[,]> LinkParameters;


        /// <summary>
        /// Gets the status.
        /// </summary>
        public string Status { get; private set; }
        public double[] angleRange;

        private double _eps = double.NaN;
        private double _deltaAngle = double.NaN;
        private double _fixedTimeStep = double.NaN;

        /// <summary>
        /// Gets or sets the epsilon.
        /// </summary>
        /// <value>
        /// The epsilon.
        /// </value>
        public double epsilon
        {
            get { return _eps; }
            set
            {
                _eps = value;
                _deltaAngle = double.NaN;
                InputSpeed = double.NaN;
                _fixedTimeStep = double.NaN;
            }
        }

        /// <summary>
        /// Gets or sets the delta angle.
        /// </summary>
        /// <value>
        /// The delta angle.
        /// </value>
        public double DeltaAngle
        {
            get { return _deltaAngle; }
            set
            {
                _deltaAngle = value;
                _eps = double.NaN;
                _fixedTimeStep = _deltaAngle / InputSpeed;
            }
        }

        /// <summary>
        /// Gets or sets the fixed time step.
        /// </summary>
        /// <value>
        /// The fixed time step.
        /// </value>
        public double FixedTimeStep
        {
            get { return _fixedTimeStep; }
            set
            {
                _fixedTimeStep = value;
                _eps = double.NaN;
                _deltaAngle = InputSpeed * _fixedTimeStep;
            }
        }

        /// <summary>
        /// Gets or sets the input angular speed.
        /// </summary>
        /// <value>
        /// The input angular speed.
        /// </value>
        public double InputSpeed { get; set; }

        #endregion

        #region Set by the Topology (from the Constructor)

        public int n { get; private set; }
        public int p { get; private set; }
        public List<link> links { get; private set; }
        public List<joint> joints { get; private set; }
        private int inputJointIndex;
        private joint inputpivot;
        private int inputLinkIndex;
        private link inputLink;

        /// <summary>
        /// Gets a value indicating whether this instance is dyadic.
        /// </summary>
        /// <value>
        ///   <c>true</c> if this instance is dyadic; otherwise, <c>false</c>.
        /// </value>
        public Boolean IsDyadic
        {
            get
            {
                var knownPositions = joints.Where(j => j.isGround || inputLink.joints.Contains(j)).ToList();
                var unknownPositions = joints.Where(j => !knownPositions.Contains(j) && j.Link2 != null).ToList();
                do
                {
                    var determined = unknownPositions.FirstOrDefault(j =>
                        (j.Link1.joints.Count(jj => knownPositions.Contains(jj))
                        + j.Link2.joints.Count(jj => knownPositions.Contains(jj)) >= 2));
                    if (determined == null) return false;
                    knownPositions.Add(determined);
                    unknownPositions.Remove(determined);
                } while (unknownPositions.Count > 0);
                return true;
            }
        }

        #endregion

        #region Constructor - requires the topology of the mechanism to be provided.

        /// <summary>
        /// Initializes a new instance of the <see cref="Simulator"/> class.
        /// </summary>
        /// <param name="LinkIDs">The link I ds.</param>
        /// <param name="JointTypes">The pivot types.</param>
        /// <param name="InitPositions">The init positions.</param>
        public Simulator(IList<List<string>> LinkIDs, IList<string> JointTypes, IList<double[]> InitPositions = null)
        {
            InputSpeed = 1.0;
            CreateLinkAndPositionDetails(LinkIDs, JointTypes, InitPositions);
        }

        public Simulator(string data)
        {
            InputSpeed = 1.0;
            var positions = new List<double[]>();
            var jointTypes = new List<string>();
            var pivotSentences = data.Split('\n').ToList();
            pivotSentences.RemoveAll(string.IsNullOrWhiteSpace);
            var linkIDs = new List<List<string>>();
            foreach (var pivotSentence in pivotSentences)
            {
                var words = pivotSentence.Split(' ').ToList();
                words.RemoveAll(string.IsNullOrWhiteSpace);
                var jointTypeIndex = words.FindLastIndex(s => s.Equals("R", StringComparison.InvariantCultureIgnoreCase)
                                                              ||
                                                              s.Equals("P", StringComparison.InvariantCultureIgnoreCase)
                                                              ||
                                                              s.Equals("RP", StringComparison.InvariantCultureIgnoreCase)
                                                              ||
                                                              s.Equals("G", StringComparison.InvariantCultureIgnoreCase));
                if (jointTypeIndex == -1) throw new Exception("No joint type found in: " + pivotSentence);
                jointTypes.Add(words[jointTypeIndex]);

                double Xtemp, Ytemp, angleTemp;
                if (words.Count() == jointTypeIndex + 1)
                    positions.Add(null);
                if ((words.Count() == jointTypeIndex + 3) && double.TryParse(words[jointTypeIndex + 1], out Xtemp) &&
                    double.TryParse(words[jointTypeIndex + 2], out Ytemp))
                    positions.Add(new[] { Xtemp, Ytemp });
                else if ((words.Count() == jointTypeIndex + 4) && double.TryParse(words[jointTypeIndex + 1], out Xtemp)
                         && double.TryParse(words[jointTypeIndex + 2], out Ytemp)
                         && double.TryParse(words[jointTypeIndex + 2], out angleTemp))
                    positions.Add(new[] { angleTemp, Xtemp, Ytemp });

                words.RemoveRange(jointTypeIndex, words.Count - jointTypeIndex);
                linkIDs.Add(words);
            }
            CreateLinkAndPositionDetails(linkIDs, jointTypes, positions);
        }


        private void CreateLinkAndPositionDetails(IList<List<string>> LinkIDs, IList<string> JointTypeStrings,
                                                  IList<double[]> Positions = null)
        {
            try
            {
                if (JointTypeStrings.Count != LinkIDs.Count)
                    throw new Exception("The number of PivotTypes (which is " + p + ") must be the"
                                        + "same as the number of LinkID pairs (which is " + LinkIDs.Count + ")");

                foreach (var linkID in LinkIDs)
                    for (int i = linkID.Count - 1; i >= 0; i--)
                        if (string.IsNullOrWhiteSpace(linkID[i])) linkID.RemoveAt(i);
                        else if (linkID[i].Equals("0", StringComparison.InvariantCultureIgnoreCase)
                                 || linkID[i].Equals("gnd", StringComparison.InvariantCultureIgnoreCase)
                                 || linkID[i].Equals("grnd", StringComparison.InvariantCultureIgnoreCase)
                                 || linkID[i].Equals("grond", StringComparison.InvariantCultureIgnoreCase)
                                 || linkID[i].Equals("gound", StringComparison.InvariantCultureIgnoreCase)
                                 || linkID[i].Equals("groud", StringComparison.InvariantCultureIgnoreCase)
                                 || linkID[i].StartsWith("ground", true, CultureInfo.InvariantCulture))
                            linkID[i] = "ground";
                var linkNames = LinkIDs.SelectMany(a => a).Distinct().ToList();

                n = linkNames.Count; //count the number of links in the system
                var newLinkIDs = new List<List<string>>();
                /* create the pivots */
                joints = new List<joint>(); //create an arry of pivots
                for (int i = 0; i < JointTypeStrings.Count; i++)
                {
                    double[] currentJointPosition = null;
                    if (Positions != null) currentJointPosition = Positions[i];
                    if (LinkIDs[i].Count == 1)
                    {
                        joints.Add(new joint((LinkIDs[i][0] == "ground"), JointTypeStrings[i], currentJointPosition));
                        newLinkIDs.Add(new List<string> { LinkIDs[i][0] });
                    }
                    else
                        for (int j = 0; j < LinkIDs[i].Count - 1; j++)
                        {
                            if (j > 0 && JointTypeStrings[i].Equals("rp", StringComparison.InvariantCultureIgnoreCase))
                                joints.Add(new joint((LinkIDs[i][j] == "ground" || LinkIDs[i][j + 1] == "ground"),
                                                     "r", currentJointPosition));
                            else if (j > 0 && (JointTypeStrings[i].Equals("g", StringComparison.InvariantCultureIgnoreCase)
                                 || JointTypeStrings[i].Equals("p", StringComparison.InvariantCultureIgnoreCase)))
                                throw new Exception("More than two links is not allowed for " + JointTypeStrings[i] + " joints.");
                            else joints.Add(new joint((LinkIDs[i][j] == "ground" || LinkIDs[i][j + 1] == "ground"),
                                             JointTypeStrings[i], currentJointPosition));
                            newLinkIDs.Add(new List<string> { LinkIDs[i][j], LinkIDs[i][j + 1] });
                        }
                }
                p = joints.Count; //count the number of pivots in the system
                /* now onto the links */
                links = new List<link>(); //create an array of LINKS
                for (int k = 0; k < n; k++)
                {
                    var pivotIndices =
                        newLinkIDs.Where(lid => lid.Contains(linkNames[k])).Select(lid => newLinkIDs.IndexOf(lid));
                    var pivotsForThisLink = pivotIndices.Select(i => joints[i]).ToList();
                    links.Add(new link(linkNames[k], pivotsForThisLink, pivotsForThisLink.Count(piv => piv.isGround) >= 2));
                }
                /* now that links have been created, need to add these to pivots */
                for (int i = 0; i < newLinkIDs.Count; i++)
                {
                    joints[i].Link1 = links[linkNames.IndexOf(newLinkIDs[i][0])];
                    if (newLinkIDs[i].Count > 1)
                        joints[i].Link2 = links[linkNames.IndexOf(newLinkIDs[i][1])];
                }

                /* reorder pivots to ease additional computation. put ground pivots at end, move input to just before those. */
                inputpivot = joints[0];
                if (inputpivot.jointType == JointTypes.G) throw new Exception("Input cannot be gear teeth.");
                if (inputpivot.jointType == JointTypes.RP) throw new Exception("Input cannot be an RP joint (2 DOF inputs are not allowed).");
                if (!inputpivot.Link1.isGround)
                {
                    if (!inputpivot.Link2.isGround) throw new Exception("Input must be connected to ground (2 DOF inputs are not allowed).");
                    var tempLinkRef = inputpivot.Link1;
                    inputpivot.Link1 = inputpivot.Link2;
                    inputpivot.Link2 = tempLinkRef;
                }
                inputLink = inputpivot.Link2;
                inputLinkIndex = links.IndexOf(inputLink);
                joints.Remove(inputpivot);
                var groundPivots = joints.FindAll(piv => piv.isGround);
                joints.RemoveAll(piv => piv.isGround);
                inputJointIndex = joints.Count;
                joints.Add(inputpivot);
                joints.AddRange(groundPivots);
                foreach (var eachLink in links) eachLink.DetermineLengthsAndReferences();
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
        public void AssignPositions(IList<double[]> InitPositions)
        {
            try
            {
                for (int i = 0; i < p; i++)
                {
                    if (InitPositions[i] != null)
                    {
                        joints[i].X = InitPositions[i][0];
                        joints[i].Y = InitPositions[i][1];
                    }
                }
                foreach (var eachLink in links) eachLink.DetermineLengthsAndReferences();
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
        /// todo: unclear the format of these lengths - how to relate to the link rigid body constraint 
        public void AssignLengths(IList<double[]> Lengths)
        {
            try
            {
                for (int i = 0; i < Lengths.Count(); i++)
                {
                    var p1 = joints[(int)Lengths[i][0]];
                    var p2 = joints[(int)Lengths[i][1]];
                    var link0 = links.Find(l => l.joints.Contains(p1) && l.joints.Contains(p2));
                    var pivotIndices = new List<int> { link0.joints.IndexOf(p1), link0.joints.IndexOf(p2) };
                    pivotIndices.Sort();
                    link0.lengths[pivotIndices[0] * 2 + pivotIndices[1] - 1] = Lengths[i][2];
                }
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
        /// <returns></returns>
        public Boolean FindInitialPositionsFromLengths(double inputX, double inputY, double gnd1X, double gnd1Y)
        {
            try
            {
                if (links.Any(a => a.lengths.Any(double.IsNaN)))
                    throw new Exception("Link lengths for all links need to be set first.");
                var inputLink = links.Find(a => a.isGround && a.joints.Contains(inputpivot)) ??
                                links.Find(a => a.joints.Contains(inputpivot));
                if (
                    Math.Abs(inputLink.lengths[0] -
                             Math.Sqrt((inputX - gnd1X) * (inputX - gnd1X) + (inputY - gnd1Y) * (inputY - gnd1Y))) > epsilon)
                    throw new Exception("Input and first ground position do not match expected length of " +
                                        inputLink.lengths[0]);
                inputpivot.X = inputX;
                inputpivot.Y = inputY;
                joints[inputJointIndex + 1].X = gnd1X;
                joints[inputJointIndex + 1].Y = gnd1Y;
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
        /// <returns></returns>
        public Boolean FindInitialPositionsFromLengths(double inputX, double inputY, double AngleToGnd1)
        {
            try
            {
                if (links.Any(a => a.lengths.Any(double.IsNaN)))
                    throw new Exception("Link lengths for all links need to be set first. Use AssignLengths method.");
                var inputLink = links.Find(a => a.isGround && a.joints.Contains(inputpivot)) ??
                                links.Find(a => a.joints.Contains(inputpivot));
                inputpivot.X = inputX;
                inputpivot.Y = inputY;
                joints[inputJointIndex + 1].X = inputX + Math.Cos(AngleToGnd1) * inputLink.lengths[0];
                joints[inputJointIndex + 1].Y = inputY + Math.Sin(AngleToGnd1) * inputLink.lengths[0];
                return epsilon > FindInitialPositionMain();
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign positions from lengths (see inner exeception).", e);
            }
        }

        private double FindInitialPositionMain()
        {
            var newPivots = new List<joint>(joints);
            var newInputIndex = inputJointIndex;
            while (newInputIndex < p - 3)
            {
                var lastGnd = newPivots[p - 1];
                newPivots.RemoveAt(p - 1);
                newPivots.Insert(0, lastGnd);
                newInputIndex++;
            }
            var nonDyadicPositionFinder = new NonDyadicPositionFinder(links, newPivots, newInputIndex, epsilon);
            return nonDyadicPositionFinder.Run_PositionsAreUnknown();
        }

        /// <summary>
        /// Finds the next position.
        /// </summary>
        /// <param name="time">The time.</param>
        /// <returns></returns>
        public Boolean FindPositionAtTime(double time)
        {
            return false;
        }
        /// <summary>
        /// Finds the position at crank angle.
        /// </summary>
        /// <param name="angle">The angle.</param>
        /// <returns></returns>
        public Boolean FindPositionAtCrankAngle(double angle)
        {
            return false;
        }



        /// <summary>
        /// Finds the full movement.
        /// </summary>
        public void FindFullMovement()
        {
            if ((double.IsNaN(this.DeltaAngle)) && (double.IsNaN(this.FixedTimeStep)))
                throw new Exception(
                    "Either the angle delta or the time step must be specified.");

            JointParameters = new SortedList<double, double[,]>();
            LinkParameters = new SortedList<double, double[,]>();

            if (IsDyadic) FindFullMovementDyadic();
            else FindFullMovementNonDyadic();
        }




        private Boolean lessThanFullRotation()
        {
            if (inputpivot.jointType == JointTypes.P) return true; // if the input is a sliding block, then there is no limit 
            // eventually, links will reach invalid positions in both directions. 
            double range;
            lock (angleRange)
            {
                range = angleRange[0] - angleRange[1];
            }
            return range < 2 * Math.PI;
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


        /// <summary>
        /// Finds the difference from target path.
        /// </summary>
        /// <param name="TargetPath">The target path.</param>
        /// <returns></returns>
        public double FindDifferenceFromTargetPath(double[, ,] TargetPath)
        {
            //todo
            throw new NotImplementedException();
        }

        /// <summary>
        /// Optimizes the positions for target path.
        /// </summary>
        /// <param name="TargetPath">The target path.</param>
        /// <returns></returns>
        public double OptimizePositionsForTargetPath(double[, ,] TargetPath)
        {
            //todo
            throw new NotImplementedException();
        }

        /// <summary>
        /// Optimizes the input crank.
        /// </summary>
        /// <returns></returns>
        public double OptimizeInputCrank()
        {
            //todo
            throw new NotImplementedException();
        }

        /// <summary>
        /// Gets the degrees of freedom.
        /// </summary>
        public int DegreesOfFreedom
        {
            get
            {
                var oneDOFJoints =
                    joints.Count(j => j.Link2 != null && (j.jointType == JointTypes.P || j.jointType == JointTypes.R));
                var twoDOFJoints =
                    joints.Count(j => j.Link2 != null && (j.jointType == JointTypes.G || j.jointType == JointTypes.RP));
                return 3 * (links.Count - 1) - 2 * oneDOFJoints - twoDOFJoints;
            }
        }

       // private void MoveInputToNextPosition(double currentTime, double timeStep, double[,] newJointParams, double[,] newLinkParams)

        private void MoveInputToNextPosition(double delta, double[,] newJointParams, double[,] newLinkParams, double[,] oldJointParams, double[,] oldLinkParams)
        {
            if (inputpivot.jointType == JointTypes.R)
            {
                var lastAngle = oldLinkParams[inputLinkIndex, 0];
                var newAngle = lastAngle + delta;
                newLinkParams[inputLinkIndex, 0] = newAngle;
                newLinkParams[inputLinkIndex, 1] = InputSpeed;
                newLinkParams[inputLinkIndex, 2] = 0.0;
                var xGnd = newJointParams[inputJointIndex, 0] = oldJointParams[inputJointIndex, 0];
                var yGnd = newJointParams[inputJointIndex, 1] = oldJointParams[inputJointIndex, 1];

                foreach (var j in inputLink.joints)
                {
                    if (inputpivot == j) continue;
                    var jIndex = joints.IndexOf(j);
                    var length = inputLink.lengthBetween(inputpivot, j);
                    var theta = Math.Atan2(j.Y - yGnd, j.X - xGnd) + delta;
                    newJointParams[jIndex, 0] = xGnd + length * Math.Cos(theta);
                    newJointParams[jIndex, 1] = xGnd + length * Math.Sin(theta);
                    newJointParams[jIndex, 2] = -InputSpeed * length * Math.Sin(theta);
                    newJointParams[jIndex, 3] = InputSpeed * length * Math.Cos(theta);
                    newJointParams[jIndex, 4] = -InputSpeed * InputSpeed * length * Math.Cos(theta);
                    newJointParams[jIndex, 5] = -InputSpeed * InputSpeed * length * Math.Sin(theta);
                }
            }
            else /*else, the input is a prismatic slide */
            {
                newLinkParams[inputLinkIndex, 0] = newLinkParams[inputLinkIndex, 1] = newLinkParams[inputLinkIndex, 2] = 0.0;
                /* the block input does not rotate therefore the angle, angular velocity, and angular accelerations are all zero. */
                var xDelta = delta * Math.Cos(inputpivot.SlideAngle);
                var yDelta = delta * Math.Sin(inputpivot.SlideAngle);
                foreach (var j in inputLink.joints)
                {
                    var jIndex = joints.IndexOf(j);
                    newJointParams[jIndex, 0] = oldJointParams[jIndex, 0] + xDelta;
                    newJointParams[jIndex, 1] = oldJointParams[jIndex, 1] + yDelta;
                }
            }
        }
    }
}
