using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;
using PlanarMechanismSimulator.PositionSolving;
using PlanarMechanismSimulator.VelocityAndAcceleration;

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

        public double[] InputRange { get; private set; }

        public int DrivingIndex { get; private set; }

        /// <summary>
        /// Gets the new index of a joint (value in the cell) given the original position.
        /// </summary>
        /// <value>
        /// The joint new index from original.
        /// </value>
        public int[] JointNewIndexFromOriginal { get; private set; }
        private double _deltaAngle = Constants.DefaultStepSize;

        private double _maxSmoothingError = double.NaN;
        private Dictionary<int, gearData> gearsData;

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
                _maxSmoothingError = double.NaN;
                // _fixedTimeStep = _deltaAngle / InputSpeed;
            }
        }

        /// <summary>
        /// Gets or sets the fixed time step.
        /// </summary>
        /// <value>
        /// The fixed time step.
        /// </value>
        public double FixedTimeStep { get { return _deltaAngle / InputSpeed; } }
        //{
        //    get { return _fixedTimeStep; }
        //    set
        //    {
        //        _fixedTimeStep = value;
        //        _deltaAngle = InputSpeed * _fixedTimeStep;
        //    }
        //}



        /// <summary>
        /// Gets or sets the maximum smoothing error.
        /// </summary>
        /// <value>
        /// The maximum smoothing error.
        /// </value>
        public double MaxSmoothingError
        {
            get { return _maxSmoothingError; }
            set
            {
                _maxSmoothingError = value;
                _deltaAngle = double.NaN;
                // _fixedTimeStep = _deltaAngle / InputSpeed;
            }
        }

        /// <summary>
        /// Gets or sets the input angular speed in radians/second.
        /// </summary>
        /// <value>
        /// The input angular speed.
        /// </value>
        public double InputSpeed { get; set; }

        internal double AverageLength { get; private set; }

        /// <summary>
        /// Gets whether or not the mechanism completes a cycle and repeats.
        /// </summary>
        /// <value>
        /// The complete cycle.
        /// </value>
        public CycleTypes CycleType { get; private set; }


        /// <summary>
        /// Gets the begin time.
        /// </summary>
        /// <value>
        /// The begin time.
        /// </value>
        public double BeginTime { get; private set; }


        /// <summary>
        /// Gets the end time.
        /// </summary>
        /// <value>
        /// The end time.
        /// </value>
        public double EndTime { get; private set; }

        /// <summary>
        /// Gets the time span.
        /// </summary>
        /// <value>
        /// The time span.
        /// </value>
        public double Time_Span
        {
            get { return EndTime - BeginTime; }
        }


        #endregion

        #region Set by the Topology (from the Constructor)

        public int numLinks { get; private set; }
        public int numJoints { get; private set; }
        public List<link> AllLinks { get; private set; }
        public List<joint> AllJoints { get; private set; }
        public int firstInputJointIndex { get; private set; }
        public int inputJointIndex { get; private set; }
        public joint inputJoint { get; private set; }
        public int inputLinkIndex { get; private set; }
        public link inputLink { get; private set; }
        public link groundLink { get; private set; }


        /// <summary>
        /// Gets the indices of joints (in the new PMKS order, not the order originially given)
        /// that are not to be updated or assigned from the AssignPositions function.
        /// </summary>
        /// <value>
        /// The index of the output joint.
        /// </value>
        public int[] FixedJointIndices { get; private set; }
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
                var knownPositions = AllJoints.Where(j => j.isGround || inputLink.joints.Contains(j)).ToList();
                var unknownPositions = AllJoints.Where(j => !knownPositions.Contains(j) && j.Link2 != null).ToList();
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
        /// <param name="LinkIDs">The link IDs.</param>
        /// <param name="JointTypes">The pivot types.</param>
        /// <param name="InitPositions">The init positions.</param>
        public Simulator(IList<List<string>> LinkIDs, IList<string> JointTypes, int DriverIndex = 0, IList<double[]> InitPositions = null)
        {
            CreateLinkAndPositionDetails(LinkIDs, JointTypes, DriverIndex, InitPositions);
            FixedJointIndices = new int[0];
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Simulator" /> class.
        /// </summary>
        /// <param name="LinkIDs">The link IDs.</param>
        /// <param name="JointTypes">The pivot types.</param>
        /// <param name="DriverIndex">Index of the driver.</param>
        /// <param name="InitPositions">The init positions.</param>
        /// <param name="FixedJointIndices">The fixed or unassignable joint indices.</param>
        public Simulator(IList<List<string>> LinkIDs, IList<string> JointTypes, int DriverIndex = 0, IList<double[]> InitPositions = null,
            int[] FixedJointIndices = null)
        {
            CreateLinkAndPositionDetails(LinkIDs, JointTypes, DriverIndex, InitPositions);
            this.FixedJointIndices = FixedJointIndices.Select(index => JointNewIndexFromOriginal[index]).ToArray();
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
                var words = pivotSentence.Split(new[] { ' ', ',', '\t', '|' }).ToList();
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
            CreateLinkAndPositionDetails(linkIDs, jointTypes, 0, positions);
        }


        private void CreateLinkAndPositionDetails(IList<List<string>> LinkIDs, IList<string> JointTypeStrings,
                                                int drivingIndex, IList<double[]> Positions = null)
        {
#if trycatch
            try
            {
#endif
            DrivingIndex = drivingIndex;
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
            AllJoints = new List<joint>(); //create an arry of pivots
            for (int i = 0; i < JointTypeStrings.Count; i++)
            {
                double[] currentJointPosition = null;
                if (Positions != null) currentJointPosition = Positions[i];
                if (LinkIDs[i].Count == 1)
                {
                    /* if the specified joint is only a point to be traced on a given link, then we make it
                     * an "R" joint. */
                    AllJoints.Add(new joint((LinkIDs[i][0] == "ground"), "r", currentJointPosition));
                    newLinkIDs.Add(new List<string> { LinkIDs[i][0] });
                }
                else /* if there are 2 or more links at this joint, then it's actually multiple co-located
                          * joints. This really only make sense for "R" joints. We assume that if it is typed as an RP
                          * joint then between 0 and 1 there is an RP and between 1 and 2, 2 and 3 and so on, it is an R joint. */
                    for (int j = 0; j < LinkIDs[i].Count - 1; j++)
                    {
                        if (j > 0 && JointTypeStrings[i].Equals("rp", StringComparison.InvariantCultureIgnoreCase))
                            AllJoints.Add(new joint((LinkIDs[i][j] == "ground" || LinkIDs[i][j + 1] == "ground"),
                                                 "r", currentJointPosition));
                        /* if there are more than 2 links and the joint is typed G or P then we throw an error. */
                        else if (j > 0 && (JointTypeStrings[i].Equals("g", StringComparison.InvariantCultureIgnoreCase)
                             || JointTypeStrings[i].Equals("p", StringComparison.InvariantCultureIgnoreCase)))
                            throw new Exception("More than two links is not allowed for " + JointTypeStrings[i] + " joints.");
                        /* else...this is the normal case of a joint between two links. */
                        else AllJoints.Add(new joint((LinkIDs[i][j] == "ground" || LinkIDs[i][j + 1] == "ground"),
                                         JointTypeStrings[i], currentJointPosition));
                        newLinkIDs.Add(new List<string> { LinkIDs[i][j], LinkIDs[i][j + 1] });
                    }
            }
            /* now onto the links */
            AllLinks = new List<link>(); //create an array of LINKS
            foreach (string linkName in linkNames)
            {
                var pivotIndices =
                    newLinkIDs.Where(lid => lid.Contains(linkName)).Select(lid => newLinkIDs.IndexOf(lid));
                var pivotsForThisLink = pivotIndices.Select(i => AllJoints[i]).ToList();
                AllLinks.Add(new link(linkName, pivotsForThisLink, pivotsForThisLink.All(piv => piv.isGround)));
            }
            /* now that links have been created, need to add these to pivots */
            for (int i = 0; i < newLinkIDs.Count; i++)
            {
                AllJoints[i].Link1 = AllLinks[linkNames.IndexOf(newLinkIDs[i][0])];
                if (newLinkIDs[i].Count > 1)
                    AllJoints[i].Link2 = AllLinks[linkNames.IndexOf(newLinkIDs[i][1])];
            }
            inputJoint = AllJoints[DrivingIndex];
            if (inputJoint.jointType == JointTypes.G) throw new Exception("Input cannot be gear teeth.");
            if (inputJoint.jointType == JointTypes.RP) throw new Exception("Input cannot be an RP joint (2 DOF inputs are not allowed).");
            if (!inputJoint.Link1.isGround)
            {
                if (!inputJoint.Link2.isGround) throw new Exception("Input must be connected to ground (2 DOF inputs are not allowed).");
                var tempLinkRef = inputJoint.Link1;
                inputJoint.Link1 = inputJoint.Link2;
                inputJoint.Link2 = tempLinkRef;
            }
            inputLink = inputJoint.Link2;
            inferAdditionalGearLinks();
            var additionalRefjoints = addReferencePivotsToSlideOnlyLinks();
            numLinks = AllLinks.Count; //count the number of links in the system
            numJoints = AllJoints.Count; //count the number of pivots in the system
            /* reorder links, move input link and ground link to back of list */
            AllLinks.Remove(inputLink); AllLinks.Add(inputLink); //move inputLink to back of list
            var groundLinks = AllLinks.Where(c => c.isGround).ToList();//move inputLink to back of list
            if (groundLinks.Count != 1) throw new Exception("There can only be one ground link. In this case, there are "
                  + groundLinks.Count);
            groundLink = groundLinks[0];
            AllLinks.Remove(groundLink); AllLinks.Add(groundLink); //move ground to back of list
            inputLinkIndex = numLinks - 2;
            /* reorder pivots to ease additional computation. put ground pivots at end, move input to just before those. */
            var origOrder = new List<joint>(AllJoints);
            var groundPivots = groundLink.joints.Where(j => j != inputJoint && j.FixedWithRespectTo(groundLink)).ToList();
            for (int i = groundPivots.Count - 1; i >= 0; i--)
            {
                /* this doesn't real change anything, but allows one to see the movement of P joints along ground. It only 
                 * applies to P-joints as RP-joints will behave differently. */
                if (groundPivots[i].jointType == JointTypes.P)
                {
                    var tempLinkRef = groundPivots[i].Link1;
                    groundPivots[i].Link1 = groundPivots[i].Link2;
                    groundPivots[i].Link2 = tempLinkRef;
                    groundPivots.RemoveAt(i);
                }
            }
            AllJoints.Remove(inputJoint);
            AllJoints.RemoveAll(groundPivots.Contains);

            var connectedInputJoints = inputLink.joints.Where(j => j != inputJoint && j.FixedWithRespectTo(inputLink)).ToList();
            AllJoints.RemoveAll(connectedInputJoints.Contains);
            firstInputJointIndex = AllJoints.Count;
            AllJoints.AddRange(connectedInputJoints);
            inputJointIndex = AllJoints.Count;
            AllJoints.Add(inputJoint);
            AllJoints.AddRange(groundPivots);

            JointNewIndexFromOriginal = new int[numJoints];
            for (int i = 0; i < numJoints; i++)
                JointNewIndexFromOriginal[i] = AllJoints.IndexOf(origOrder[i]);

            setAdditionalReferencePositions(additionalRefjoints);

#if trycatch
            }
            catch (Exception e)
            {
                throw new Exception(
                    "Failed to construct Planar Mechanism Simulator from topology data (InputIndex, Connections, PivotTypes).",
                    e);
            }
#endif
        }

        private List<joint> addReferencePivotsToSlideOnlyLinks()
        {
            if (AllLinks.All(c => c.joints.Any(j => j.FixedWithRespectTo(c)))) return null;
            var additionalRefjoints = new List<joint>();
            foreach (var c in AllLinks.Where(c => !c.joints.Any(j => j.FixedWithRespectTo(c))))
            {
                var newJoint = new joint(false, "r") { Link1 = c };
                c.joints.Add(newJoint);
                AllJoints.Add(newJoint);
                additionalRefjoints.Add(newJoint);
            }
            return additionalRefjoints;
        }
        private void setAdditionalReferencePositions(IEnumerable<joint> additionalRefjoints = null)
        {
            foreach (var j in AllJoints)
            {
                j.SlideLimits = null;
                if (j.jointType == JointTypes.R) j.InitSlideAngle = Double.NaN;
            }
            if (additionalRefjoints != null)
            {
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
                    thisAdditionalJoint.xInitial = xSum / (thisAdditionalJoint.Link1.joints.Count - 1);
                    thisAdditionalJoint.yInitial = ySum / (thisAdditionalJoint.Link1.joints.Count - 1);
                }
            }
            setGearData();
            var totalLength = 0.0;
            int numLengths = 0;
            foreach (var eachLink in AllLinks)
            {
                eachLink.DetermineLengthsAndReferences();
                totalLength += eachLink.TotalLength;
                numLengths += eachLink.joints.Count * (eachLink.joints.Count - 1) / 2;
            }
            AverageLength = totalLength / numLengths;
        }

        private void inferAdditionalGearLinks()
        {
            if (AllJoints.All(j => j.jointType != JointTypes.G)) return;
            var counter = 0;
            foreach (var j in AllJoints)
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
                AllJoints.Add(newJoint1); AllJoints.Add(newJoint2); AllLinks.Add(connectLink);
            }
        }


        private void setGearData()
        {
            if (AllJoints.All(j => j.jointType != JointTypes.G)) return;
            gearsData = new Dictionary<int, gearData>();
            int index = 0;
            foreach (var gearTeethJoint in AllJoints)
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
                                  new gearData(gearTeethJoint, AllJoints.IndexOf(gearTeethJoint), gearCenter1,
                                      AllJoints.IndexOf(gearCenter1), gearCenter2, AllJoints.IndexOf(gearCenter2),
                                      AllLinks.IndexOf(connectingRod)));
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


        #endregion

        /// <summary>
        /// Assigns the initial positions of all the pivots.
        /// </summary>
        /// <param name="InitPositions">The init positions.</param>
        public void AssignPositions(IList<double[]> InitPositions)
        {
#if trycatch
            try
            {
#endif
            for (int i = 0; i < numJoints; i++)
            {
                if (InitPositions[i] != null)
                {
                    var newIndex = JointNewIndexFromOriginal[i];
                    if (FixedJointIndices.Contains(newIndex)) continue;
                    var j = AllJoints[newIndex];
                    j.xInitial = j.xNumerical = j.xLast = j.x = InitPositions[i][0];
                    j.yInitial = j.yNumerical = j.yLast = j.y = InitPositions[i][1];
                    if (InitPositions[i].GetLength(0) > 2 && j.jointType != JointTypes.R)
                        j.InitSlideAngle = InitPositions[i][2];
                }
            }
            setAdditionalReferencePositions();
#if trycatch
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign positions to topology (see inner exeception).", e);
            }
#endif
        }
        /// <summary>
        /// Assigns the initial positions of all the pivots.
        /// </summary>
        /// <param name="InitPositions">The init positions.</param>
        private void AssignPositions(double[] InitPositions)
        {
#if trycatch
            try
            {
#endif
            var k = 0;
            for (int i = 0; i < numJoints; i++)
            {
                var newIndex = JointNewIndexFromOriginal[i];
                if (FixedJointIndices.Contains(newIndex)) continue;
                var j = AllJoints[newIndex];
                j.xInitial = j.xNumerical = j.xLast = j.x = InitPositions[k++];
                j.yInitial = j.yNumerical = j.yLast = j.y = InitPositions[k++];
                if (j.jointType != JointTypes.R)
                    j.InitSlideAngle = InitPositions[k++];
            }
            setAdditionalReferencePositions();
#if trycatch
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign positions to topology (see inner exeception).", e);
            }
#endif
        }


        /// <summary>
        /// Assigns the lengths of the links.
        /// </summary>
        /// <param name="Lengths">The lengths.</param>
        /// todo: unclear the format of these lengths - how to relate to the link rigid body constraint 
        public void AssignLengths(IList<double[]> Lengths)
        {
#if trycatch
            try
            {
#endif
            for (int i = 0; i < Lengths.Count(); i++)
            {
                var p1 = AllJoints[(int)Lengths[i][0]];
                var p2 = AllJoints[(int)Lengths[i][1]];
                var link0 = AllLinks.First(l => l.joints.Contains(p1) && l.joints.Contains(p2));
                var pivotIndices = new List<int> { link0.joints.IndexOf(p1), link0.joints.IndexOf(p2) };
                pivotIndices.Sort();
                link0.setLength(p1, p2, Lengths[i][2]);
            }
#if trycatch
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign lengths to topology (see inner exeception).", e);
            }
#endif
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
#if trycatch
            try
            {
#endif
            //if (AllLinks.Any(a => a.Lengths.Any(double.IsNaN)))
            //    throw new Exception("Link lengths for all links need to be set first.");
            if (!Constants.sameCloseZero(inputLink.lengthBetween(inputJoint, AllJoints[inputJointIndex + 1]),
                Constants.distance(inputX, inputY, gnd1X, gnd1Y)))
                throw new Exception("Input and first ground position do not match expected length of " +
                                    inputLink.lengthBetween(inputJoint, AllJoints[inputJointIndex + 1]));
            inputJoint.xInitial = inputX;
            inputJoint.yInitial = inputY;
            AllJoints[inputJointIndex + 1].xInitial = gnd1X;
            AllJoints[inputJointIndex + 1].yInitial = gnd1Y;
            // todo: put values in JointPositions and LinkAngles (like the 4 preceding lines)
            double[,] JointPositions, LinkAngles;
            return epsilon > FindInitialPositionMain(out JointPositions, out LinkAngles);
#if trycatch
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign positions from lengths (see inner exeception).", e);
            }
#endif
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
#if trycatch
            try
            {
#endif
            //if (AllLinks.Any(a => a.Lengths.Any(double.IsNaN)))
            //    throw new Exception("Link lengths for all links need to be set first. Use AssignLengths method.");
            inputJoint.xInitial = inputX;
            inputJoint.yInitial = inputY;
            AllJoints[inputJointIndex + 1].xInitial = inputX + Math.Cos(AngleToGnd1) * inputLink.lengthBetween(inputJoint, AllJoints[inputJointIndex + 1]);
            AllJoints[inputJointIndex + 1].yInitial = inputY + Math.Sin(AngleToGnd1) * inputLink.lengthBetween(inputJoint, AllJoints[inputJointIndex + 1]);
            // todo: put values in JointPositions and LinkAngles (like the 4 preceding lines)
            double[,] JointPositions, LinkAngles;
            return epsilon > FindInitialPositionMain(out JointPositions, out LinkAngles);
#if trycatch
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign positions from lengths (see inner exeception).", e);
            }
#endif
        }

        private double FindInitialPositionMain(out double[,] JointPositions, out double[,] LinkAngles)
        {
            LinkAngles = new double[numLinks, 1];
            JointPositions = new double[numJoints, 2];
            var NDPS = new NonDyadicPositionSolver(new PositionFinder(AllJoints, AllLinks, gearsData, inputJointIndex));
            return NDPS.Run_PositionsAreUnknown(JointPositions, LinkAngles);
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
            return range < Constants.FullCircle;
        }



        /// <summary>
        /// Calculates the full movement for a set of new initial positions stored in x.
        /// this is nearly the same as the function above it.
        /// </summary>
        /// <param name="x">The x.</param>
        public void calculate(double[] x = null)
        {
            if (x != null) // && x.GetLength(0) == numJoints)
                AssignPositions(x);
            FindFullMovement();
        }


        /// <summary>
        /// Gets the degrees of freedom.
        /// </summary>
        public int DegreesOfFreedom
        {
            get
            {
                var oneDOFJoints =
                    AllJoints.Count(j => j.Link2 != null && (j.jointType == JointTypes.P || j.jointType == JointTypes.R));
                var twoDOFJoints =
                    AllJoints.Count(j => j.Link2 != null && (j.jointType == JointTypes.G || j.jointType == JointTypes.RP));
                return 3 * (AllLinks.Count - 1) - 2 * oneDOFJoints - twoDOFJoints;
            }
        }
    }
}
