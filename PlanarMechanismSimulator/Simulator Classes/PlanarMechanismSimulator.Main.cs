// ***********************************************************************
// Assembly         : PlanarMechanismKinematicSimulator
// Author           : Matt
// Created          : 06-10-2015
//
// Last Modified By : Matt
// Last Modified On : 06-10-2015
// ***********************************************************************
// <copyright file="PlanarMechanismSimulator.Main.cs" company="">
//     Copyright ©  2014
// </copyright>
// <summary></summary>
// ***********************************************************************
using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;
using PMKS.PositionSolving;

namespace PMKS
{
    /// <summary>
    /// Class Simulator.
    /// </summary>
    public partial class Simulator : IDependentAnalysis
    {
        #region Properties and Fields
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
        /// <value>The status.</value>
        public string Status { get; private set; }

        /// <summary>
        /// Gets the input range.
        /// </summary>
        /// <value>The input range.</value>
        public double[] InputRange { get; private set; }

        /// <summary>
        /// Gets the new index of a joint (value in the cell) given the original position.
        /// </summary>
        /// <value>The joint new index from original.</value>
        public int[] JointNewIndexFromOriginal { get; private set; }
        /// <summary>
        /// The inverted to solve non ground input
        /// </summary>
        private Boolean invertedToSolveNonGroundInput;
        /// <summary>
        /// The _delta angle
        /// </summary>
        private double _deltaAngle = Constants.DefaultStepSize;
        /// <summary>
        /// The maximum joint parameter lengths
        /// </summary>
        private int maxJointParamLengths;
        /// <summary>
        /// The _max smoothing error
        /// </summary>
        private double _maxSmoothingError = double.NaN;
        /// <summary>
        /// The gears data
        /// </summary>
        private Dictionary<int, GearData> gearsData;

        /// <summary>
        /// Gets or sets the epsilon.
        /// </summary>
        /// <value>The epsilon.</value>
        public double epsilon
        {
            get { return Constants.epsilonSame; }
        }

        /// <summary>
        /// Gets or sets the delta angle.
        /// </summary>
        /// <value>The delta angle.</value>
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
        /// <value>The fixed time step.</value>
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
        /// <value>The maximum smoothing error.</value>
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
        /// <value>The input angular speed.</value>
        public double InputSpeed { get; set; }

        /// <summary>
        /// Gets the average length.
        /// </summary>
        /// <value>The average length.</value>
        internal double AverageLength { get; private set; }

        /// <summary>
        /// Gets whether or not the mechanism completes a cycle and repeats.
        /// </summary>
        /// <value>The complete cycle.</value>
        public CycleTypes CycleType { get; private set; }


        /// <summary>
        /// Gets the begin time.
        /// </summary>
        /// <value>The begin time.</value>
        public double BeginTime { get; private set; }


        /// <summary>
        /// Gets the end time.
        /// </summary>
        /// <value>The end time.</value>
        public double EndTime { get; private set; }

        /// <summary>
        /// Gets the time span.
        /// </summary>
        /// <value>The time span.</value>
        public double Time_Span
        {
            get { return EndTime - BeginTime; }
        }

        readonly List<joint> additionalRefjoints = new List<joint>();
        #endregion

        #region Set by the Topology (from the Constructor)

        /// <summary>
        /// Gets the number links.
        /// </summary>
        /// <value>The number links.</value>
        public int numLinks { get; private set; }
        /// <summary>
        /// Gets the number joints.
        /// </summary>
        /// <value>The number joints.</value>
        public int numJoints { get; private set; }
        /// <summary>
        /// Gets all links.
        /// </summary>
        /// <value>All links.</value>
        public List<link> AllLinks { get; private set; }
        /// <summary>
        /// Gets all joints.
        /// </summary>
        /// <value>All joints.</value>
        public List<joint> AllJoints { get; private set; }
        /// <summary>
        /// Gets the first index of the input joint.
        /// </summary>
        /// <value>The first index of the input joint.</value>
        public int firstInputJointIndex { get; private set; }
        /// <summary>
        /// Gets the index of the input joint.
        /// </summary>
        /// <value>The index of the input joint.</value>
        public int inputJointIndex { get; private set; }
        /// <summary>
        /// Gets the input joint.
        /// </summary>
        /// <value>The input joint.</value>
        public joint inputJoint { get; private set; }
        /// <summary>
        /// Gets the index of the input link.
        /// </summary>
        /// <value>The index of the input link.</value>
        public int inputLinkIndex { get; private set; }
        /// <summary>
        /// Gets the input link.
        /// </summary>
        /// <value>The input link.</value>
        public link inputLink { get; private set; }

        /// <summary>
        /// Gets the ground link.
        /// </summary>
        /// <value>The ground link.</value>
        private link groundLinkForSimulation;

        public link GroundLink { get; private set; }
        /// <summary>
        /// Gets the indices of joints (in the new PMKS order, not the order originially given)
        /// that are not to be updated or assigned from the AssignPositions function.
        /// </summary>
        /// <value>The index of the output joint.</value>
        public int[] ReadOnlyJointIndices { get; private set; }
        /// <summary>
        /// The name base for gear connector
        /// </summary>
        private const string nameBaseForGearConnector = "auto_generated_gear_connect_";

        /// <summary>
        /// Gets a value indicating whether this instance is dyadic.
        /// </summary>
        /// <value><c>true</c> if this instance is dyadic; otherwise, <c>false</c>.</value>
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
        /// Initializes a new instance of the <see cref="Simulator" /> class.
        /// </summary>
        /// <param name="linkIDs">The link IDs.</param>
        /// <param name="jointTypes">The pivot types.</param>
        /// <param name="driverIndex">Index of the driver.</param>
        /// <param name="initPositions">The init positions.</param>
        /// <param name="readOnlyJointIndices">The fixed or unassignable joint indices.</param>
        public Simulator(IList<string[]> linkIDs, IList<JointType> jointTypes, int driverIndex = 0, IList<double[]> initPositions = null,
            int[] readOnlyJointIndices = null)
        {
            CreateLinkAndPositionDetails(linkIDs, jointTypes, driverIndex, initPositions);
            RearrangeLinkAndJointLists();
            addReferencePivotsToSlideOnlyLinks();
            setAdditionalReferencePositions();
            ReadOnlyJointIndices = (readOnlyJointIndices == null)
                ? new int[0]
                : readOnlyJointIndices.Select(index => JointNewIndexFromOriginal[index]).ToArray();
        }
        /// <summary>
        /// Initializes a new instance of the <see cref="Simulator"/> class.
        /// </summary>
        /// <param name="data">The data.</param>
        /// <exception cref="Exception">No joint type found in:  + pivotSentence</exception>
        public Simulator(string data)
        {
            List<string[]> linkIDs;
            List<JointType> jointTypes;
            int driverIndex;
            List<double[]> initPositions;
            List<Boolean[]> unusedDisplayBools;
            ConvertTextToData(data, out linkIDs, out jointTypes, out driverIndex, out initPositions,
                out unusedDisplayBools);

            CreateLinkAndPositionDetails(linkIDs, jointTypes, driverIndex, initPositions);
            RearrangeLinkAndJointLists();
            addReferencePivotsToSlideOnlyLinks();
            setAdditionalReferencePositions();
        }


        /// <summary>
        /// Converts the text to data.
        /// </summary>
        /// <param name="text">The text.</param>
        /// <param name="linkIDs">The link i ds.</param>
        /// <param name="jointTypes">The joint types.</param>
        /// <param name="driverIndex">Index of the driver.</param>
        /// <param name="initPositions">The initialize positions.</param>
        /// <param name="displayBools">The display bools.</param>
        /// <returns><c>true</c> if XXXX, <c>false</c> otherwise.</returns>
        public static bool ConvertTextToData(string text, out List<string[]> linkIDs, out  List<JointType> jointTypes,
            out  int driverIndex, out  List<double[]> initPositions, out List<Boolean[]> displayBools)
        {
            linkIDs = new List<string[]>();
            jointTypes = new List<JointType>();
            initPositions = new List<double[]>();
            displayBools = new List<Boolean[]>();
            driverIndex = -1;
            var pivotSentences = text.Split('\n', '|').ToList();
            pivotSentences.RemoveAll(string.IsNullOrWhiteSpace);
            if (pivotSentences.Count == 0) return false;
            for (int jointIndex = 0; jointIndex < pivotSentences.Count; jointIndex++)
            {
                var pivotSentence = pivotSentences[jointIndex];
                var words = pivotSentence.Split(',', ' ').ToList();
                words.RemoveAll(string.IsNullOrWhiteSpace);
                var lastJointType = words.LastOrDefault(s => s.Equals("R", StringComparison.CurrentCultureIgnoreCase)
                                                             ||
                                                             s.Equals("P", StringComparison.CurrentCultureIgnoreCase)
                                                             ||
                                                             s.Equals("RP", StringComparison.CurrentCultureIgnoreCase)
                                                             ||
                                                             s.Equals("G", StringComparison.CurrentCultureIgnoreCase));
                JointType jType;
                if (!Enum.TryParse(lastJointType, true, out jType)) return false;
                jointTypes.Add(jType);
                var jointTypePosition = words.LastIndexOf(lastJointType);
                var position = jointTypePosition;
                double temp;
                if (words.Count() < position + 2) return false;
                if (!double.TryParse(words[++position], out temp)) return false;
                var Xcoord = temp;
                if (!double.TryParse(words[++position], out temp)) return false;
                var Ycoord = temp;
                var angle = 0.0;
                if ((words.Count() > ++position) && double.TryParse(words[position], out temp))
                {
                    angle = temp;
                    position++;
                }
                initPositions.Add(new[] { Xcoord, Ycoord, angle });
                var bools = new bool[4];
                if (words.Count() > position && (words[position].Contains("t") || words[position].Contains("f")))
                {
                    var plusMinusString = words[position];
                    for (int i = 0; i < plusMinusString.Length; i++)
                        if (plusMinusString[i].Equals('t') || plusMinusString[i].Equals('+')) bools[i] = true;
                }
                else
                {
                    for (int i = position; i < words.Count; i++)
                        Boolean.TryParse(words[position], out bools[i-position]);
                }
                if (bools.Count() >= 4 && bools[3]) driverIndex = jointIndex;
                displayBools.Add(bools);
                words.RemoveRange(jointTypePosition, words.Count - jointTypePosition);
                linkIDs.Add(words.ToArray());
            }
            if (driverIndex == -1)
            {
                for (int i = 0; i < linkIDs.Count; i++)
                {
                    if (linkIDs[i].GetLength(0) > 1
                        && (jointTypes[i] == JointType.R || jointTypes[i] == JointType.P))
                    {
                        driverIndex = i;
                        displayBools[i][3] = true;
                        break;
                    }
                }
            }
            return true;
        }


        /// <summary>
        /// Creates the link and position details.
        /// </summary>
        /// <param name="linkIDs">The link i ds.</param>
        /// <param name="jointTypes">The joint type strings.</param>
        /// <param name="drivingIndex">Index of the driving.</param>
        /// <param name="positions">The positions.</param>
        /// <exception cref="Exception">
        /// The number of PivotTypes (which is  + numJoints + ) must be the
        ///                                         + same as the number of LinkID pairs (which is  + LinkIDs.Count + )
        /// or
        /// More than two links is not allowed for  + JointTypeStrings[i] +  joints.
        /// or
        /// Input cannot be gear teeth.
        /// or
        /// Input cannot be an RP joint (2 DOF inputs are not allowed).
        /// or
        /// There can only be one ground link. In this case, there are 
        ///                                             + groundLinks.Count
        /// or
        /// Failed to construct Planar Mechanism Simulator from topology data (InputIndex, Connections, PivotTypes).
        /// </exception>
        private void CreateLinkAndPositionDetails(IList<string[]> linkIDs, IList<JointType> jointTypes,
            int drivingIndex, IList<double[]> positions = null)
        {

            try
            {
                inputJointIndex = drivingIndex;
                if (jointTypes.Count != linkIDs.Count)
                    throw new Exception("The number of PivotTypes (which is " + numJoints + ") must be the"
                                        + "same as the number of LinkID pairs (which is " + linkIDs.Count + ")");

                foreach (var linkID in linkIDs)
                    for (int i = linkID.GetLength(0) - 1; i >= 0; i--)
                        //if (string.IsNullOrWhiteSpace(linkID[i])) linkID.RemoveAt(i);
                        if (linkID[i].Equals("0", StringComparison.CurrentCultureIgnoreCase)
                                 || linkID[i].Equals("gnd", StringComparison.CurrentCultureIgnoreCase)
                                 || linkID[i].Equals("grnd", StringComparison.CurrentCultureIgnoreCase)
                                 || linkID[i].Equals("grond", StringComparison.CurrentCultureIgnoreCase)
                                 || linkID[i].Equals("gound", StringComparison.CurrentCultureIgnoreCase)
                                 || linkID[i].Equals("groud", StringComparison.CurrentCultureIgnoreCase)
                                 || linkID[i].StartsWith("ground", StringComparison.CurrentCultureIgnoreCase))
                            linkID[i] = "ground";
                var linkNames = linkIDs.SelectMany(a => a).Distinct().ToList();

                var newLinkIDs = new List<List<string>>();
                /* create the pivots */
                AllJoints = new List<joint>(); //create an arry of pivots
                for (int i = 0; i < jointTypes.Count; i++)
                {
                    double[] currentJointPosition = null;
                    if (positions != null) currentJointPosition = positions[i];
                    if (linkIDs[i].GetLength(0) == 1)
                    {
                        /* if the specified joint is only a point to be traced on a given link, then we make it
                         * an "R" joint. */
                        AllJoints.Add(new joint((linkIDs[i][0] == "ground"), JointType.R, currentJointPosition));
                        newLinkIDs.Add(new List<string> { linkIDs[i][0] });
                    }
                    else
                        for (int j = 0; j < linkIDs[i].GetLength(0) - 1; j++)
                        {
                            /* At the end of these conditions, the "else" statement handles the normal two-link joint. 
                             * If there are more than 2  links at this joint, then it's actually multiple co-located joints. 
                             * This really only make sense for "R" joints. We assume that if it is typed as an RP      
                             * joint then between 0 and 1 there is an RP; and between 1 and 2, 2 and 3 and so on, it is an R joint. */
                            if (j > 0 && jointTypes[i] == JointType.RP)
                                AllJoints.Add(new joint((linkIDs[i][j] == "ground" || linkIDs[i][j + 1] == "ground"),
                                    JointType.R, currentJointPosition));
                            /* if there are more than 2 links and the joint is typed G or P then we throw an error. */
                            else if (j > 0 && (jointTypes[i] == JointType.G || jointTypes[i] == JointType.P))
                                throw new Exception("More than two links is not allowed for " + jointTypes[i] + " joints.");
                            /* else...this is the normal case of a joint between two links. */
                            else
                                AllJoints.Add(new joint(
                                    (linkIDs[i][j] == "ground" || linkIDs[i][j + 1] == "ground"),
                                    jointTypes[i], currentJointPosition));
                            newLinkIDs.Add(new List<string> { linkIDs[i][j], linkIDs[i][j + 1] });
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
            }
            catch (Exception e)
            {
                throw new Exception(
                    "Failed to construct Planar Mechanism Simulator from topology data (InputIndex, Connections, PivotTypes).",
                    e);
            }
        }

        private void RearrangeLinkAndJointLists()
        {
            numLinks = AllLinks.Count; //count the number of links in the system
            numJoints = AllJoints.Count; //count the number of pivots in the system
            inputJoint = AllJoints[inputJointIndex];
            if (inputJoint.jointType == JointType.G) throw new Exception("Input cannot be gear teeth.");
            if (inputJoint.jointType == JointType.RP) throw new Exception("Input cannot be an RP joint (2 DOF inputs are not allowed).");
            var groundLinks = AllLinks.Where(c => c.IsGround).ToList(); //move inputLink to back of list
            if (groundLinks.Count != 1)
                throw new Exception("There can only be one ground link. In this case, there are "
                                    + groundLinks.Count);
            GroundLink = groundLinkForSimulation = groundLinks[0];
            if (inputJoint.Link2 == null)
                throw new Exception("The input driver must be between two links (cannot be applied to a single link as in the current description).");
            if (!inputJoint.Link1.IsGround && !inputJoint.Link2.IsGround)
            {
                invertedToSolveNonGroundInput = true;
                GroundLink = groundLinkForSimulation;
                var numFixedJointsOnLink1 = inputJoint.Link1.joints.Count(j => j.FixedWithRespectTo(inputJoint.Link1));
                if (numFixedJointsOnLink1 >= 2)
                {
                    inputLink = inputJoint.Link2;
                    groundLinkForSimulation = inputJoint.Link1;
                }
                else
                {
                    var numFixedJointsOnLink2 = inputJoint.Link2.joints.Count(j => j.FixedWithRespectTo(inputJoint.Link2));
                    if (numFixedJointsOnLink2 >= 2)
                    {
                        inputLink = inputJoint.Link1;
                        groundLinkForSimulation = inputJoint.Link2;
                    }
                    else
                    {
                        groundLinkForSimulation = numFixedJointsOnLink1 > 0 ? inputJoint.Link1 : inputJoint.Link2;
                        inputLink = numFixedJointsOnLink1 > 0 ? inputJoint.Link2 : inputJoint.Link1;
                        var newJoint = new joint(true, JointType.R) { Link1 = groundLinkForSimulation };
                        groundLinkForSimulation.joints.Add(newJoint);
                        AllJoints.Add(newJoint);
                        additionalRefjoints.Add(newJoint);
                    }
                }
                groundLinkForSimulation.IsGround = true;
                GroundLink.IsGround = false;
                if (GroundLink.joints.Count(j => j.FixedWithRespectTo(GroundLink)) < 2)
                {
                    var newJoint = new joint(false, JointType.R) { Link1 = GroundLink };
                    GroundLink.joints.Add(newJoint);
                    AllJoints.Add(newJoint);
                    additionalRefjoints.Add(newJoint);
                }
            }
            else
            {
                if (!inputJoint.Link1.IsGround)
                {
                    var tempLinkRef = inputJoint.Link1;
                    inputJoint.Link1 = inputJoint.Link2;
                    inputJoint.Link2 = tempLinkRef;
                }
                inputLink = inputJoint.Link2;
            }
            /* reorder pivots to ease additional computation. put ground pivots at end, move input to just before those. */
            var origOrder = new List<joint>(AllJoints);
            /* reorder links, move input link and ground link to back of list */
            AllLinks.Remove(inputLink); AllLinks.Add(inputLink); //move inputLink to back of list
            AllLinks.Remove(groundLinkForSimulation); AllLinks.Add(groundLinkForSimulation); //move ground to back of list
            inputLinkIndex = numLinks - 2;
            var groundPivots = groundLinkForSimulation.joints.Where(j => j != inputJoint && j.FixedWithRespectTo(groundLinkForSimulation)).ToList();
            AllJoints.Remove(inputJoint);
            AllJoints.RemoveAll(groundPivots.Contains);

            var connectedInputJoints = inputLink.joints.Where(j => j != inputJoint && j.FixedWithRespectTo(inputLink)).ToList();
            AllJoints.RemoveAll(connectedInputJoints.Contains);
            firstInputJointIndex = AllJoints.Count;
            AllJoints.AddRange(connectedInputJoints);
            inputJointIndex = AllJoints.Count;
            AllJoints.Add(inputJoint);
            AllJoints.AddRange(groundPivots);

            maxJointParamLengths = AllJoints.All(j => j.jointType == JointType.R) ? 6 : 9;
            JointNewIndexFromOriginal = new int[numJoints];
            for (int i = 0; i < numJoints; i++)
                JointNewIndexFromOriginal[i] = AllJoints.IndexOf(origOrder[i]);
        }


        /// <summary>
        /// Adds the reference pivots to slide only links.
        /// </summary>
        /// <returns>List&lt;joint&gt;.</returns>
        private void addReferencePivotsToSlideOnlyLinks()
        {
            foreach (var c in AllLinks.Where(c => !c.joints.Any(j => j.FixedWithRespectTo(c))))
            {
                var newJoint = new joint(c.IsGround, JointType.R) { Link1 = c };
                c.joints.Add(newJoint);
                AllJoints.Add(newJoint);
                additionalRefjoints.Add(newJoint);
            }
        }
        /// <summary>
        /// Sets the positions of additional reference positions.
        /// </summary>
        private void setAdditionalReferencePositions()
        {
            foreach (var j in AllJoints.Where(j => j.jointType == JointType.R))
                j.OffsetSlideAngle = Double.NaN;

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
                    thisAdditionalJoint.x = thisAdditionalJoint.xNumerical = thisAdditionalJoint.xLast = thisAdditionalJoint.xInitial
                        = xSum / (thisAdditionalJoint.Link1.joints.Count - 1);
                    thisAdditionalJoint.y = thisAdditionalJoint.yNumerical = thisAdditionalJoint.yLast = thisAdditionalJoint.yInitial
                        = ySum / (thisAdditionalJoint.Link1.joints.Count - 1);
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

        /// <summary>
        /// Infers the additional gear links.
        /// </summary>
        /// <exception cref="Exception">
        /// No link connects between gears:  + j.Link1.name +  and  + j.Link2.name
        /// or
        /// No pivot (R joint) for  + j.Link2.name
        /// </exception>
        private void inferAdditionalGearLinks()
        {
            if (AllJoints.All(j => j.jointType != JointType.G)) return;
            var counter = 0;
            foreach (var j in AllJoints)
            {
                if (j.jointType != JointType.G) continue;
                var link1Neighbors = j.Link1.joints.Select(jj => jj.OtherLink(j.Link1)).ToList();
                if (j.Link2.joints.Any(
                           jj => jj != j && link1Neighbors.Contains(jj.OtherLink(j.Link2)))) continue;
                if (double.IsNaN(j.OffsetSlideAngle))
                    throw new Exception("No link connects between gears: " + j.Link1.name + " and " + j.Link2.name);
                var newJoint1 = new joint(false, JointType.P, new[] { j.xInitial, j.yInitial, j.OffsetSlideAngle });
                var gearCenter2 = j.Link2.joints.FirstOrDefault(jj => jj != j && jj.jointType == JointType.R);
                if (gearCenter2 == null) throw new Exception("No pivot (R joint) for " + j.Link2.name);
                var newJoint2 = new joint(false, JointType.R, new[] { gearCenter2.xInitial, gearCenter2.yInitial });
                var connectLink = new link(nameBaseForGearConnector + (counter++), new List<joint> { newJoint1, newJoint2 }, false);
                newJoint1.Link1 = j.Link1;
                newJoint2.Link1 = j.Link2;
                newJoint1.Link2 = newJoint2.Link2 = connectLink;
                AllJoints.Add(newJoint1); AllJoints.Add(newJoint2); AllLinks.Add(connectLink);
            }
        }


        /// <summary>
        /// Sets the gear data.
        /// </summary>
        private void setGearData()
        {
            var index = 0;
            if (AllJoints.All(j => j.jointType != JointType.G)) return;
            gearsData = new Dictionary<int, GearData>();
            foreach (var gearTeethJoint in AllJoints)
            {
                if (gearTeethJoint.jointType == JointType.G)
                {
                    var gear1 = gearTeethJoint.Link1;
                    var gear2 = gearTeethJoint.Link2;
                    var possibleGear1Centers = gear1.joints.Where(j => j.jointType != JointType.G && j.Link2 != null);
                    var possibleGear2Centers =
                        gear2.joints.Where(j => j.jointType != JointType.G && j.Link2 != null).ToList();
                    var bestCenterOption = new Tuple<joint, joint, link, double>(null, null, null, double.NaN);
                    var bestError = double.PositiveInfinity;
                    foreach (var g1 in possibleGear1Centers)
                        foreach (var g2 in possibleGear2Centers)
                        {
                            if (g1.jointType == JointType.P && g2.jointType == JointType.P) continue;
                            if (g1.jointType == JointType.RP && g1.Link1 == gear1) continue;
                            if (g2.jointType == JointType.RP && g2.Link1 == gear2) continue;
                            var connectLink =
                                AllLinks.FirstOrDefault(c => c.joints.Contains(g1) && c.joints.Contains(g2));
                            if (connectLink == null) continue;
                            var angle1 = FindInitialGearAngle(g1, gearTeethJoint);
                            var angle2 = FindInitialGearAngle(g2, gearTeethJoint);
                            var error = Math.Abs(angle1 - angle2);
                            if (bestError > error)
                            {
                                bestError = error;
                                bestCenterOption = new Tuple<joint, joint, link, double>(g1, g2, connectLink, (angle1 + angle2) / 2);
                            }
                        }
                    var gearCenter1 = bestCenterOption.Item1;
                    var gearCenter2 = bestCenterOption.Item2;
                    var connectingRod = bestCenterOption.Item3;
                    var gearAngle = bestCenterOption.Item4;
                    gearsData.Add(index,
                        new GearData(gearTeethJoint, index, gearCenter1, AllJoints.IndexOf(gearCenter1), AllLinks.IndexOf(gear1),
                            gearCenter2, AllJoints.IndexOf(gearCenter2), AllLinks.IndexOf(gear2),
                                          AllLinks.IndexOf(connectingRod), gearAngle));
                }
                index++;
            }
        }

        /// <summary>
        /// Finds the initial gear angle.
        /// </summary>
        /// <param name="g1">The g1.</param>
        /// <param name="gearTeeth">The gear teeth.</param>
        /// <returns>System.Double.</returns>
        private double FindInitialGearAngle(joint g1, joint gearTeeth)
        {
            if (g1.jointType != JointType.R) return g1.SlideAngleInitial;
            var angle = Math.Atan2((gearTeeth.yInitial - g1.yInitial), (gearTeeth.xInitial - g1.xInitial))
                + Math.PI / 2;
            while (angle > Math.PI / 2) angle -= Math.PI;
            while (angle < -Math.PI / 2) angle += Math.PI;
            return angle;
        }
        #endregion

        /// <summary>
        /// Assigns the initial positions of all the pivots.
        /// </summary>
        /// <param name="InitPositions">The init positions.</param>
        public void AssignPositions(IList<double[]> InitPositions)
        {
            var initPosList = new List<double>();
            foreach (var position in InitPositions)
            {
                initPosList.AddRange(position);
            }
            AssignPositions(initPosList.ToArray());
        }
        /// <summary>
        /// Assigns the initial positions of all the pivots.
        /// </summary>
        /// <param name="InitPositions">The init positions.</param>
        /// <exception cref="Exception">Failed to assign positions to topology (see inner exeception).</exception>
        private void AssignPositions(double[] InitPositions)
        {
            try
            {
                var k = 0;
                for (int i = 0; i < numJoints; i++)
                {
                    var newIndex = JointNewIndexFromOriginal[i];
                    if (ReadOnlyJointIndices.Contains(newIndex)) continue;
                    var j = AllJoints[newIndex];
                    j.xInitial = j.xNumerical = j.xLast = j.x = InitPositions[k++];
                    j.yInitial = j.yNumerical = j.yLast = j.y = InitPositions[k++];
                    //if (j.jointType != JointTypes.R)
                    j.OffsetSlideAngle = InitPositions[k++];
                }
                setAdditionalReferencePositions();
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
        /// <exception cref="Exception">Failed to assign lengths to topology (see inner exeception).</exception>
        /// todo: unclear the format of these lengths - how to relate to the link rigid body constraint
        public void AssignLengths(IList<double[]> Lengths)
        {
            try
            {
                for (int i = 0; i < Lengths.Count(); i++)
                {
                    var p1 = AllJoints[(int)Lengths[i][0]];
                    var p2 = AllJoints[(int)Lengths[i][1]];
                    var link0 = AllLinks.First(l => l.joints.Contains(p1) && l.joints.Contains(p2));
                    var pivotIndices = new List<int> { link0.joints.IndexOf(p1), link0.joints.IndexOf(p2) };
                    pivotIndices.Sort();
                    link0.SetLength(p1, p2, Lengths[i][2]);
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
        /// <returns>Boolean.</returns>
        /// <exception cref="Exception">
        /// Input and first ground position do not match expected length of  +
        ///                                         inputLink.lengthBetween(inputJoint, AllJoints[inputJointIndex + 1])
        /// or
        /// Failed to assign positions from lengths (see inner exeception).
        /// </exception>
        public Boolean FindInitialPositionsFromLengths(double inputX, double inputY, double gnd1X, double gnd1Y)
        {

            try
            {
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
        /// <returns>Boolean.</returns>
        /// <exception cref="Exception">Failed to assign positions from lengths (see inner exeception).</exception>
        public Boolean FindInitialPositionsFromLengths(double inputX, double inputY, double AngleToGnd1)
        {

            try
            {
                //if (AllLinks.Any(a => a.Lengths.Any(double.IsNaN)))
                //    throw new Exception("Link lengths for all links need to be set first. Use AssignLengths method.");
                inputJoint.xInitial = inputX;
                inputJoint.yInitial = inputY;
                AllJoints[inputJointIndex + 1].xInitial = inputX + Math.Cos(AngleToGnd1) * inputLink.lengthBetween(inputJoint, AllJoints[inputJointIndex + 1]);
                AllJoints[inputJointIndex + 1].yInitial = inputY + Math.Sin(AngleToGnd1) * inputLink.lengthBetween(inputJoint, AllJoints[inputJointIndex + 1]);
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
        /// Finds the initial position main.
        /// </summary>
        /// <param name="JointPositions">The joint positions.</param>
        /// <param name="LinkAngles">The link angles.</param>
        /// <returns>System.Double.</returns>
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
        /// <returns>Boolean.</returns>
        public Boolean FindPositionAtCrankAngle(double angle)
        {
            return false;
        }


        /// <summary>
        /// Lesses the than full rotation.
        /// </summary>
        /// <returns>Boolean.</returns>
        private Boolean lessThanFullRotation()
        {
            if (inputJoint.jointType == JointType.P) return true; // if the input is a sliding block, then there is no limit 
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
        /// <value>The degrees of freedom.</value>
        public int DegreesOfFreedom
        {
            get
            {
                var numRJoints = AllJoints.Count(j => j.Link2 != null && (j.jointType == JointType.R));
                var numPJoints = AllJoints.Count(j => j.jointType == JointType.P);
                var numRPJoints = AllJoints.Count(j => j.jointType == JointType.RP);
                var numGJoints = AllJoints.Count(j => j.jointType == JointType.G);
                if (numLinks == 3 && numPJoints == 3 && numRJoints == 0 && numRPJoints == 0 && numGJoints == 0) return 1;
                if (numLinks == 4 && numPJoints == 4 && numRJoints == 0 && numRPJoints == 0 && numGJoints == 0) return 2;
                if (numLinks == 4 && numPJoints == 3 && numRJoints == 1 && numRPJoints == 0 && numGJoints == 0) return 0;
                return 3 * (AllLinks.Count - 1) - 2 * (numRJoints + numPJoints) - (numRPJoints + numGJoints);
            }
        }
    }
}
