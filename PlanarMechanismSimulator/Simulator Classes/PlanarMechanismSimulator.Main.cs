using System;
using System.Collections.Generic;
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
        public TimeSortedList JointParameters;
        /// <summary>
        /// Gets the link parameters.
        /// </summary>
        public TimeSortedList LinkParameters;
        /// <summary>
        /// Gets the status.
        /// </summary>
        public string Status { get; private set; }

        private PositionFinder posFinder;

        public double[] InputRange;
        private double _deltaAngle = double.NaN;
        private double _fixedTimeStep = double.NaN;
        private double _maxSmoothingError = double.NaN;
        /// <summary>
        /// Gets or sets the epsilon.
        /// </summary>
        /// <value>
        /// The epsilon.
        /// </value>
        public double epsilon
        {
            get { return Constants.epsilonSame; }
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
                _deltaAngle = InputSpeed * _fixedTimeStep;
            }
        }


        /// <summary>
        /// Gets or sets the fixed time step.
        /// </summary>
        /// <value>
        /// The fixed time step.
        /// </value>
        public double MaxSmoothingError
        {
            get { return _maxSmoothingError; }
            set
            {
                _fixedTimeStep = _deltaAngle = double.NaN;
                _maxSmoothingError = value;
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

        public int numLinks { get; private set; }
        public int numJoints { get; private set; }
        public List<link> links { get; private set; }
        public List<joint> joints { get; private set; }
        private int firstInputJointIndex;
        public int inputJointIndex;
        private joint inputJoint;
        private int inputLinkIndex;
        private link inputLink;
        private link groundLink;
        private List<joint> additionalRefjoints;
        private const string nameBaseForGearConnector = "auto_generated_gear_connect_";

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
                        (j.Link1.joints.Count(knownPositions.Contains)
                        + j.Link2.joints.Count(knownPositions.Contains) >= 2));
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
            // InputSpeed = 1.0;
            CreateLinkAndPositionDetails(LinkIDs, JointTypes, InitPositions);
        }

        public Simulator(string data)
        {
            //  InputSpeed = 1.0;
            var positions = new List<double[]>();
            var jointTypes = new List<string>();
            var pivotSentences = data.Split('\n').ToList();
            pivotSentences.RemoveAll(string.IsNullOrWhiteSpace);
            var linkIDs = new List<List<string>>();
            foreach (var pivotSentence in pivotSentences)
            {
                var words = pivotSentence.Split(' ').ToList();
                words.RemoveAll(string.IsNullOrWhiteSpace);
                var lastJointType = words.LastOrDefault(s => s.Equals("R", StringComparison.InvariantCultureIgnoreCase)
                                                              ||
                                                              s.Equals("P", StringComparison.InvariantCultureIgnoreCase)
                                                              ||
                                                              s.Equals("RP", StringComparison.InvariantCultureIgnoreCase)
                                                              ||
                                                              s.Equals("G", StringComparison.InvariantCultureIgnoreCase));
                var jointTypeIndex = words.LastIndexOf(lastJointType);
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
                         && double.TryParse(words[jointTypeIndex + 3], out angleTemp))
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
                    throw new Exception("The number of PivotTypes (which is " + numJoints + ") must be the"
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
                                 || linkID[i].StartsWith("ground", StringComparison.InvariantCultureIgnoreCase))
                            linkID[i] = "ground";
                var linkNames = LinkIDs.SelectMany(a => a).Distinct().ToList();

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
                /* now onto the links */
                links = new List<link>(); //create an array of LINKS
                for (int k = 0; k < linkNames.Count; k++)
                {
                    var pivotIndices =
                        newLinkIDs.Where(lid => lid.Contains(linkNames[k])).Select(lid => newLinkIDs.IndexOf(lid));
                    var pivotsForThisLink = pivotIndices.Select(i => joints[i]).ToList();
                    links.Add(new link(linkNames[k], pivotsForThisLink, pivotsForThisLink.All(piv => piv.isGround)));
                }
                /* now that links have been created, need to add these to pivots */
                for (int i = 0; i < newLinkIDs.Count; i++)
                {
                    joints[i].Link1 = links[linkNames.IndexOf(newLinkIDs[i][0])];
                    if (newLinkIDs[i].Count > 1)
                        joints[i].Link2 = links[linkNames.IndexOf(newLinkIDs[i][1])];
                }
                inputJoint = joints[0];
                if (inputJoint.jointType == JointTypes.G) throw new Exception("Input cannot be gear teeth.");
                if (inputJoint.jointType == JointTypes.RP) throw new Exception("Input cannot be an RP joint (2 DOF inputs are not allowed).");
                if (!inputJoint.Link1.isGround)
                {
                    if (!inputJoint.Link2.isGround) throw new Exception("Input must be connected to ground (2 DOF inputs are not allowed).");
                    var tempLinkRef = inputJoint.Link1;
                    inputJoint.Link1 = inputJoint.Link2;
                    inputJoint.Link2 = tempLinkRef;
                }
                inferAdditionalGearLinks();
                addReferencePivotsToSlideOnlyLinks();
                numLinks = links.Count; //count the number of links in the system
                numJoints = joints.Count; //count the number of pivots in the system
                /* reorder links, move input link and ground link to back of list */
                inputLink = (inputJoint.Link2.isGround) ? inputJoint.Link1 : inputJoint.Link2;
                links.Remove(inputLink); links.Add(inputLink); //move inputLink to back of list
                groundLink = links.First(c => c.isGround);
                links.Remove(groundLink); links.Add(groundLink); //move ground to back of list
                inputLinkIndex = numLinks - 2;
                /* reorder pivots to ease additional computation. put ground pivots at end, move input to just before those. */
                var origOrder = new List<joint>(joints);
                joints.Remove(inputJoint);
                var groundPivots = joints.Where(j => j.isGround).ToList();
                joints.RemoveAll(j => j.isGround);
                var connectedInputJoints = joints.Where(inputLink.joints.Contains).ToList();
                joints.RemoveAll(inputLink.joints.Contains);
                firstInputJointIndex = joints.Count;
                joints.AddRange(connectedInputJoints);
                inputJointIndex = joints.Count;
                joints.Add(inputJoint);
                joints.AddRange(groundPivots);
                JointReOrdering = new int[numJoints];
                for (int i = 0; i < numJoints; i++)
                    JointReOrdering[i] = joints.IndexOf(origOrder[i]);
                setAdditionalReferencePositions();
                setGearData();
                foreach (var eachLink in links) eachLink.DetermineLengthsAndReferences();
            }
            catch (Exception e)
            {
                throw new Exception(
                    "Failed to construct Planar Mechanism Simulator from topology data (InputIndex, Connections, PivotTypes).",
                    e);
            }
        }

        private void addReferencePivotsToSlideOnlyLinks()
        {
            if (links.All(c => c.joints.Count(j => j.FixedWithRespectTo(c)) > 0)) return;
            additionalRefjoints = new List<joint>();
            foreach (var c in links)
            {
                if (c.joints.Count(j => j.FixedWithRespectTo(c)) > 0) continue;
                var newJoint = new joint(false, "r");
                newJoint.Link1 = c;
                c.joints.Add(newJoint);
                joints.Add(newJoint);
                additionalRefjoints.Add(newJoint);
            }
        }
        private void setAdditionalReferencePositions()
        {
            if (additionalRefjoints == null) return;
            foreach (var thisAdditionalJoint in additionalRefjoints)
            {
                var xSum = 0.0;
                var ySum = 0.0;
                foreach (var otherJoint in thisAdditionalJoint.Link1.joints)
                {
                    if (otherJoint == thisAdditionalJoint) continue;
                    xSum += otherJoint.xInitial;
                    ySum += otherJoint.yInitial;
                }
                thisAdditionalJoint.xInitial = xSum / thisAdditionalJoint.Link1.joints.Count - 1;
                thisAdditionalJoint.yInitial = ySum / thisAdditionalJoint.Link1.joints.Count - 1;
            }
        }

        private void inferAdditionalGearLinks()
        {
            if (joints.All(j => j.jointType != JointTypes.G)) return;
            var counter = 0;
            foreach (var j in joints)
            {
                if (j.jointType != JointTypes.G) continue;
                var link1Neighbors = j.Link1.joints.Select(jj => jj.OtherLink(j.Link1)).ToList();
                if (j.Link2.joints.Any(
                           jj => jj != j && link1Neighbors.Contains(jj.OtherLink(j.Link2)))) continue;
                if (double.IsNaN(j.InitSlideAngle))
                    throw new Exception("No link connects between gears: " + j.Link1.name + " and " + j.Link2.name);
                var newJoint1 = new joint(false, "p", new[] { j.xInitial, j.yInitial, j.InitSlideAngle });
                var gearCenter2 = j.Link2.joints.FirstOrDefault(jj => jj != j && jj.jointType == JointTypes.R);
                if (gearCenter2 == null) throw new Exception("No pivot (R joint) for " + j.Link2.name);
                var newJoint2 = new joint(false, "r", new[] { gearCenter2.xInitial, gearCenter2.yInitial });
                var connectLink = new link(nameBaseForGearConnector + (counter++), new List<joint> { newJoint1, newJoint2 }, false);
                newJoint1.Link1 = j.Link1;
                newJoint2.Link1 = j.Link2;
                newJoint1.Link2 = newJoint2.Link2 = connectLink;
                joints.Add(newJoint1); joints.Add(newJoint2); links.Add(connectLink);
            }
        }


        private void setGearData()
        {
            if (joints.All(j => j.jointType != JointTypes.G)) return;
            gearsData = new Dictionary<int, gearData>();
            int index = 0;
            foreach (var gearTeethJoint in joints)
            {
                if (gearTeethJoint.jointType == JointTypes.G)
                {
                    var gear1 = gearTeethJoint.Link1;
                    var gear2 = gearTeethJoint.Link2;
                    var otherGear1Joints = gear1.joints.Where(j => j != gearTeethJoint && j.jointType != JointTypes.G
                        && j.Link2 != null).ToList();
                    var neighboringGear1Links = otherGear1Joints.Select(j => new List<link> { j.OtherLink(gear1) }).ToList();
                    for (int i = 0; i < otherGear1Joints.Count; i++)
                        neighboringGear1Links[i].AddRange(LinksFromSharedJoints(otherGear1Joints[i], neighboringGear1Links[i][0]));
                    var otherGear2Joints = gear2.joints.Where(j => j != gearTeethJoint && j.jointType != JointTypes.G
                        && j.Link2 != null).ToList();
                    var neighboringGear2Links = otherGear2Joints.Select(j => new List<link> { j.OtherLink(gear2) }).ToList();
                    for (int i = 0; i < otherGear2Joints.Count; i++)
                        neighboringGear2Links[i].AddRange(LinksFromSharedJoints(otherGear2Joints[i], neighboringGear2Links[i][0]));


                    var connectingRod =
                        neighboringGear1Links.SelectMany(c => c).Intersect(neighboringGear2Links.SelectMany(c => c)).
                            First();
                    int k = 0;
                    while (!neighboringGear1Links[k].Contains(connectingRod)) k++;
                    var gearCenter1 = otherGear1Joints[k];
                    k = 0;
                    while (!neighboringGear2Links[k].Contains(connectingRod)) k++;
                    var gearCenter2 = otherGear2Joints[k];
                    if (connectingRod.name.StartsWith(nameBaseForGearConnector))
                    {
                        gearCenter1.xInitial = gearTeethJoint.xInitial;
                        gearCenter1.yInitial = gearTeethJoint.yInitial;
                        var trueGearCenter2 = gearTeethJoint.Link2.joints.First(jj => jj != gearTeethJoint && jj.jointType == JointTypes.R);
                        gearCenter2.xInitial = trueGearCenter2.xInitial;
                        gearCenter2.yInitial = trueGearCenter2.yInitial;
                    }
                    gearsData.Add(index,
                                  new gearData(gearTeethJoint, connectingRod, links.IndexOf(connectingRod), gearCenter1,
                                               joints.IndexOf(gearCenter1),
                                               gearCenter2, joints.IndexOf(gearCenter2)));
                }
                index++;
            }
        }

        private IEnumerable<link> LinksFromSharedJoints(joint joint, link link)
        {
            var samePositionJoints =
                link.joints.Where(j => j != joint && j.jointType == JointTypes.R &&
                    Constants.sameCloseZero(j.xInitial, joint.xInitial) &&
                    Constants.sameCloseZero(j.yInitial, joint.yInitial)).ToList();
            if (samePositionJoints.Count == 0) return new link[0];
            var newLinks = samePositionJoints.Select(j => j.OtherLink(link)).ToList();
            for (int i = samePositionJoints.Count - 1; i >= 0; i--)
                newLinks.AddRange(LinksFromSharedJoints(samePositionJoints[i], samePositionJoints[i].OtherLink(link)));
            return newLinks;
        }

        private Dictionary<int, gearData> gearsData;

        #endregion

        /// <summary>
        /// Assigns the initial positions of all the pivots.
        /// </summary>
        /// <param name="InitPositions">The init positions.</param>
        public void AssignPositions(IList<double[]> InitPositions)
        {
            try
            {
                for (int i = 0; i < numJoints; i++)
                {
                    if (InitPositions[i] != null)
                    {
                        joints[JointReOrdering[i]].xInitial = InitPositions[i][0];
                        joints[JointReOrdering[i]].yInitial = InitPositions[i][1];
                    }
                }
                setAdditionalReferencePositions();
                setGearData();
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
                    var link0 = links.FirstOrDefault(l => l.joints.Contains(p1) && l.joints.Contains(p2));
                    var pivotIndices = new List<int> { link0.joints.IndexOf(p1), link0.joints.IndexOf(p2) };
                    pivotIndices.Sort();
                    link0.setLength(p1, p2, Lengths[i][2]);
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
                if (links.Any(a => a.Lengths.Any(double.IsNaN)))
                    throw new Exception("Link lengths for all links need to be set first.");
                var inputLink = links.FirstOrDefault(a => a.isGround && a.joints.Contains(inputJoint)) ??
                                links.FirstOrDefault(a => a.joints.Contains(inputJoint));
                if (!Constants.sameCloseZero(inputLink.lengthBetween(inputJoint, joints[inputJointIndex + 1]),
                    Constants.distance(inputX, inputY, gnd1X, gnd1Y)))
                    throw new Exception("Input and first ground position do not match expected length of " +
                                        inputLink.lengthBetween(inputJoint, joints[inputJointIndex + 1]));
                inputJoint.xInitial = inputX;
                inputJoint.yInitial = inputY;
                joints[inputJointIndex + 1].xInitial = gnd1X;
                joints[inputJointIndex + 1].yInitial = gnd1Y;
                // todo: put values in JointPositions and LinkAngles (like the 4 preceding lines)
                double[,] JointPositions, LinkAngles;
                return epsilon > FindInitialPositionMain(out JointPositions, out LinkAngles);
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
                if (links.Any(a => a.Lengths.Any(double.IsNaN)))
                    throw new Exception("Link lengths for all links need to be set first. Use AssignLengths method.");
                var inputLink = links.FirstOrDefault(a => a.isGround && a.joints.Contains(inputJoint)) ??
                                links.FirstOrDefault(a => a.joints.Contains(inputJoint));
                inputJoint.xInitial = inputX;
                inputJoint.yInitial = inputY;
                joints[inputJointIndex + 1].xInitial = inputX + Math.Cos(AngleToGnd1) * inputLink.lengthBetween(inputJoint, joints[inputJointIndex + 1]);
                joints[inputJointIndex + 1].yInitial = inputY + Math.Sin(AngleToGnd1) * inputLink.lengthBetween(inputJoint, joints[inputJointIndex + 1]);
                // todo: put values in JointPositions and LinkAngles (like the 4 preceding lines)
                double[,] JointPositions, LinkAngles;
                return epsilon > FindInitialPositionMain(out JointPositions, out LinkAngles);
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign positions from lengths (see inner exeception).", e);
            }
        }

        private double FindInitialPositionMain(out double[,] JointPositions, out double[,] LinkAngles)
        {
            LinkAngles = new double[numLinks, 1];
            JointPositions = new double[numJoints, 2];
            var NDPS = new NonDyadicPositionSolver(posFinder);
            return NDPS.Run_PositionsAreUnknown(JointPositions, LinkAngles);
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


        private Boolean lessThanFullRotation()
        {
            if (inputJoint.jointType == JointTypes.P) return true; // if the input is a sliding block, then there is no limit 
            // eventually, links will reach invalid positions in both directions. 
            double range;
            lock (InputRange)
            {
                range = Math.Abs(InputRange[0] - InputRange[1]);
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


        private void InitializeGroundAndInputSpeedAndAcceleration(double[,] jointParams, double[,] linkParams)
        {
            /* these are the ground joints, which are not moving. */
            for (int i = inputJointIndex + 1; i < numJoints; i++)
            {
                jointParams[i, 2] = jointParams[i, 3] = 0.0;
                jointParams[i, 4] = jointParams[i, 5] = 0.0;
            }
            /* and, of course, the ground link is not rotating at all. */
            linkParams[inputLinkIndex + 1, 1] = 0.0;
            linkParams[inputLinkIndex + 1, 2] = 0.0;

            if (!inputJoint.isGround) return;
            /* uh-oh, if the input is not connected to ground, we can't assume any values for
             * the x&y velocities or accelerations. */
            if (inputJoint.jointType == JointTypes.R)
            {
                /* otherwise, input is rotating at a constant speed. */
                linkParams[inputLinkIndex, 1] = InputSpeed;
                linkParams[inputLinkIndex, 2] = 0.0;

                var xInputJoint = jointParams[inputJointIndex, 0];
                var yInputJoint = jointParams[inputJointIndex, 1];
                jointParams[inputJointIndex, 2] = jointParams[inputJointIndex, 3] = 0.0;
                jointParams[inputJointIndex, 4] = jointParams[inputJointIndex, 5] = 0.0;

                for (int i = firstInputJointIndex; i < inputJointIndex; i++)
                {
                    var length = inputLink.lengthBetween(inputJoint, joints[i]);
                    var theta = Constants.angle(xInputJoint, yInputJoint, jointParams[i, 0], jointParams[i, 1]);
                    jointParams[i, 2] = -InputSpeed * length * Math.Sin(theta);
                    jointParams[i, 3] = InputSpeed * length * Math.Cos(theta);
                    jointParams[i, 4] = -InputSpeed * InputSpeed * length * Math.Cos(theta);
                    jointParams[i, 5] = -InputSpeed * InputSpeed * length * Math.Sin(theta);
                }
            }
            else if (inputJoint.jointType == JointTypes.P)
            {
                /* the block input does not rotate therefore the angular velocity, and angular accelerations are all zero. */
                linkParams[inputLinkIndex, 1] = 0.0;
                linkParams[inputLinkIndex, 2] = 0.0;

                var angle = inputJoint.SlideAngle;
                for (int i = firstInputJointIndex; i <= inputJointIndex; i++)
                {
                    /* position will be changed by the setLinkPosition function. */
                    jointParams[i, 2] = InputSpeed * Math.Cos(angle);
                    jointParams[i, 3] = InputSpeed * Math.Sin(angle);
                    jointParams[i, 4] = jointParams[i, 5] = 0.0;
                }
            }
            else throw new Exception("Currently only R or P can be the input joints.");
        }


        public int[] JointReOrdering { get; private set; }
    }
}
