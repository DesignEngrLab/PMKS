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

using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using OptimizationToolbox;
using PMKS.PositionSolving;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;

namespace PMKS
{
    /// <summary>
    ///     Class Simulator.
    /// </summary>
    public partial class Simulator : IDependentAnalysis
    {
        /// <summary>
        ///     Gets the degrees of freedom.
        /// </summary>
        /// <value>The degrees of freedom.</value>
        public int DegreesOfFreedom
        {
            get
            {
                var numRJoints = SimulationJoints.Count(j => j.Link2 != null && (j.TypeOfJoint == JointType.R));
                var numPJoints = SimulationJoints.Count(j => j.TypeOfJoint == JointType.P);
                var numRPJoints = SimulationJoints.Count(j => j.TypeOfJoint == JointType.RP);
                var numGJoints = SimulationJoints.Count(j => j.TypeOfJoint == JointType.G);
                if (NumLinks == 3 && numPJoints == 3 && numRJoints == 0 && numRPJoints == 0 && numGJoints == 0)
                    return 1;
                if (NumLinks == 4 && numPJoints == 4 && numRJoints == 0 && numRPJoints == 0 && numGJoints == 0)
                    return 2;
                if (NumLinks == 4 && numPJoints == 3 && numRJoints == 1 && numRPJoints == 0 && numGJoints == 0)
                    return 0;
                return 3 * (SimulationLinks.Count - 1) - 2 * (numRJoints + numPJoints) - (numRPJoints + numGJoints);
            }
        }

        /// <summary>
        ///     Calculates the full movement for a set of new initial positions stored in x.
        ///     this is nearly the same as the function above it.
        /// </summary>
        /// <param name="x">The x.</param>
        public void calculate(double[] x = null)
        {
            if (x != null) // && x.GetLength(0) == numJoints)
                AssignPositions(x);
            FindFullMovement();
        }

        /// <summary>
        ///     Assigns the initial positions of all the pivots.
        /// </summary>
        /// <param name="InitPositions">The init positions.</param>
        public void AssignPositions(IList<double[]> InitPositions)
        {
            try
            {
                int k = 0;
                for (int i = 0; i < NumJoints; i++)
                {
                    if (ReadOnlyJointIndices != null && ReadOnlyJointIndices.Contains(i)) continue;
                    var j = Joints[k];
                    ChangeJointPosition(InitPositions[k], j, false);
                    k++;
                }
                setAdditionalReferencePositions();
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign positions to topology (see inner exception).", e);
            }
        }

        /// <summary>Changes the initial positions of the provided joint.</summary>
        /// <param name="position"></param>
        /// <param name="j"></param>
        /// <param name="updateAdditionalPositions"></param>
        public void ChangeJointPosition(double[] position, Joint j, bool updateAdditionalPositions)
        {
            j.XInitial = j.xNumerical = j.xLast = j.x = position[0];
            j.YInitial = j.yNumerical = j.yLast = j.y = position[1];
            if (j.TypeOfJoint != JointType.R)
                j.OffsetSlideAngle = position[2];
            if (updateAdditionalPositions)
                setAdditionalReferencePositions();
        }

        /// <summary>
        /// Assigns the positions of all the joints but keeps the topology the same.
        /// </summary>
        /// <param name="initPositions">The newly assigned positions.</param>
        /// <param name="includesAnglesForAllJoints">The includes angles for all joints.</param>
        /// <exception cref="System.Exception">Failed to assign positions to topology (see inner exception).</exception>
        /// <exception cref="Exception">Failed to assign positions to topology (see inner exeception).</exception>
        public void AssignPositions(double[] initPositions, Boolean includesAnglesForAllJoints = false)
        {
            try
            {
                var numPositions = NumJoints - ReadOnlyJointIndices.GetLength(0);
                var positionArray = new double[numPositions][];
                var k = 0;
                var j = 0;
                for (var i = 0; i < numPositions; i++)
                {
                    while (ReadOnlyJointIndices.Contains(i)) j++;
                    if (Joints[j].TypeOfJoint != JointType.R || includesAnglesForAllJoints)
                        positionArray[i] = new[] { initPositions[k++], initPositions[k++], initPositions[k++] };
                    else
                        positionArray[i] = new[] { initPositions[k++], initPositions[k++] };
                    j++;
                }
            }
            catch (Exception e)
            {
                throw new Exception("Failed to assign positions to topology (see inner exception).", e);
            }
        }

        /// <summary>
        ///     Assigns the lengths of the links.
        /// </summary>
        /// <param name="Lengths">The lengths.</param>
        /// <exception cref="Exception">Failed to assign lengths to topology (see inner exeception).</exception>
        public void AssignLengths(IList<double[]> Lengths)
        {
            try
            {
                for (var i = 0; i < Lengths.Count(); i++)
                {
                    var p1 = SimulationJoints[(int)Lengths[i][0]];
                    var p2 = SimulationJoints[(int)Lengths[i][1]];
                    var link0 = SimulationLinks.First(l => l.Joints.Contains(p1) && l.Joints.Contains(p2));
                    var pivotIndices = new List<int> { link0.Joints.IndexOf(p1), link0.Joints.IndexOf(p2) };
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
        ///     Finds the initial positions from lengths.
        /// </summary>
        /// <param name="inputX">The input X.</param>
        /// <param name="inputY">The input Y.</param>
        /// <param name="gnd1X">The GND1 X.</param>
        /// <param name="gnd1Y">The GND1 Y.</param>
        /// <returns>Boolean.</returns>
        /// <exception cref="Exception">
        ///     Input and first ground position do not match expected length of  +
        ///     inputLink.lengthBetween(inputJoint, AllJoints[inputJointIndex + 1])
        ///     or
        ///     Failed to assign positions from lengths (see inner exeception).
        /// </exception>
        public Boolean FindInitialPositionsFromLengths(double inputX, double inputY, double gnd1X, double gnd1Y)
        {
            try
            {
                //if (AllLinks.Any(a => a.Lengths.Any(double.IsNaN)))
                //    throw new Exception("Link lengths for all links need to be set first.");
                if (!Constants.sameCloseZero(inputLink.lengthBetween(inputJoint, SimulationJoints[firstGroundJointIndex]),
                    Constants.distance(inputX, inputY, gnd1X, gnd1Y)))
                    throw new Exception("Input and first ground position do not match expected length of " +
                                        inputLink.lengthBetween(inputJoint, SimulationJoints[firstGroundJointIndex]));
                inputJoint.XInitial = inputX;
                inputJoint.YInitial = inputY;
                SimulationJoints[firstGroundJointIndex].XInitial = gnd1X;
                SimulationJoints[firstGroundJointIndex].YInitial = gnd1Y;
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
        ///     Finds the initial positions from lengths.
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
                inputJoint.XInitial = inputX;
                inputJoint.YInitial = inputY;
                SimulationJoints[firstGroundJointIndex].XInitial = inputX +
                                                          Math.Cos(AngleToGnd1) *
                                                          inputLink.lengthBetween(inputJoint,
                                                              SimulationJoints[firstGroundJointIndex]);
                SimulationJoints[firstGroundJointIndex].YInitial = inputY +
                                                          Math.Sin(AngleToGnd1) *
                                                          inputLink.lengthBetween(inputJoint,
                                                              SimulationJoints[firstGroundJointIndex]);
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
        ///     Finds the initial position main.
        /// </summary>
        /// <param name="JointPositions">The joint positions.</param>
        /// <param name="LinkAngles">The link angles.</param>
        /// <returns>System.Double.</returns>
        private double FindInitialPositionMain(out double[,] JointPositions, out double[,] LinkAngles)
        {
            LinkAngles = new double[NumLinks, 1];
            JointPositions = new double[NumAllJoints, 2];
            var NDPS = new NonDyadicPositionSolver(new PositionFinder(SimulationJoints, SimulationLinks, gearsData, simulationDriveIndex));
            return NDPS.Run_PositionsAreUnknown(JointPositions, LinkAngles);
        }

        /// <summary>
        ///     Finds the position at crank angle.
        /// </summary>
        /// <param name="angle">The angle.</param>
        /// <returns>Boolean.</returns>
        internal Boolean FindPositionAtCrankAngle(double angle)
        {
            return false;
        }

        /// <summary>
        ///     Lesses the than full rotation.
        /// </summary>
        /// <returns>Boolean.</returns>
        private Boolean lessThanFullRotation()
        {
            if (inputJoint.TypeOfJoint == JointType.P)
                return true; // if the input is a sliding block, then there is no limit 
            // eventually, links will reach invalid positions in both directions. 
            double range;
            lock (InputRange)
            {
                range = Math.Abs(InputRange[0] - InputRange[1]);
            }
            return range < Constants.FullCircle;
        }

        #region Properties and Fields
        /// <summary>
        /// Gets or sets the name of this simulation. This is not used
        /// internally, but may be necessary to applications wanting to keep track
        /// of different simulations.
        /// </summary>
        public string Name { get; set; }

        /// <summary>
        ///     Gets the pivot parameters.
        /// </summary>
        public TimeSortedList JointParameters { get; internal set; }

        /// <summary>
        ///     Gets the link parameters.
        /// </summary>
        public TimeSortedList LinkParameters { get; internal set; }

        /// <summary>
        ///     Gets the status.
        /// </summary>
        /// <value>The status.</value>
        public string Status { get; private set; }

        /// <summary>
        ///     Gets the input range.
        /// </summary>
        /// <value>The input range.</value>
        internal double[] InputRange { get; private set; }

        /// <summary>
        ///     The inverted to solve non ground input
        /// </summary>
        private Boolean invertedToSolveNonGroundInput;

        /// <summary>
        ///     The _delta angle
        /// </summary>
        private double _angleIncrement = Constants.DefaultStepSize;

        /// <summary>
        ///     The maximum joint parameter lengths
        /// </summary>
        private int maxJointParamLengths;

        /// <summary>
        ///     The _max smoothing error
        /// </summary>
        private double _maxSmoothingError = double.NaN;

        /// <summary>
        ///     The gears data
        /// </summary>
        private Dictionary<int, GearData> gearsData;

        /// <summary>
        ///     Gets or sets the epsilon.
        /// </summary>
        /// <value>The epsilon.</value>
        internal double epsilon
        {
            get { return Constants.epsilonSame; }
        }

        /// <summary>
        ///     Gets or sets the delta angle.
        /// </summary>
        /// <value>The delta angle.</value>
        public double AngleIncrement
        {
            get { return _angleIncrement; }
            set
            {
                _angleIncrement = value;
                _maxSmoothingError = double.NaN;
                // _fixedTimeStep = _angleIncrement / InputSpeed;
            }
        }

        /// <summary>
        ///     Gets or sets the fixed time step.
        /// </summary>
        /// <value>The fixed time step.</value>
        internal double FixedTimeStep
        {
            get { return _angleIncrement / InputSpeed; }
        }

        //{
        //    get { return _fixedTimeStep; }
        //    set
        //    {
        //        _fixedTimeStep = value;
        //        _angleIncrement = InputSpeed * _fixedTimeStep;
        //    }
        //}


        /// <summary>
        ///     Gets or sets the maximum smoothing error.
        /// </summary>
        /// <value>The maximum smoothing error.</value>
        public double MaxSmoothingError
        {
            get { return _maxSmoothingError; }
            set
            {
                _maxSmoothingError = value;
                _angleIncrement = double.NaN;
                // _fixedTimeStep = _angleIncrement / InputSpeed;
            }
        }

        /// <summary>
        ///     Gets or sets the input angular speed in radians/second.
        /// </summary>
        /// <value>The input angular speed.</value>
        public double InputSpeed { get; set; }

        /// <summary>
        ///     Gets the average length.
        /// </summary>
        /// <value>The average length.</value>
        [JsonIgnore]
        public double AverageLength { get; private set; }

        /// <summary>
        ///     Gets whether or not the mechanism completes a cycle and repeats.
        /// </summary>
        /// <value>The complete cycle.</value>
        public CycleTypes CycleType { get; private set; }


        /// <summary>
        ///     Gets the begin time.
        /// </summary>
        /// <value>The begin time.</value>
        public double StartTime { get; set; }


        /// <summary>
        ///     Gets the end time.
        /// </summary>
        /// <value>The end time.</value>
        public double EndTime { get; set; }

        /// <summary>
        ///     Gets the time span.
        /// </summary>
        /// <value>The time span.</value>
        [JsonIgnore]
        public double Time_Span
        {
            get { return EndTime - StartTime; }
        }

        private List<Joint> additionalRefjoints;

        #endregion

        #region Set by the Topology (from the Constructor)

        /// <summary>
        ///     Gets the number links.
        /// </summary>
        /// <value>The number links.</value>
        [JsonIgnore]
        public int NumLinks { get; private set; }

        /// <summary>
        ///     Gets the number of all links - including ones created to ease the analysis.
        /// </summary>
        /// <value>The number links.</value>
        internal int NumAllLinks { get; private set; }

        /// <summary>
        ///     Gets the number of all joints - including ones created to ease the analysis.
        /// </summary>
        /// <value>The number joints.</value>
        internal int NumAllJoints { get; private set; }

        /// <summary>
        /// Gets the number joints.
        /// </summary>
        /// <value>The number joints.</value>
        [JsonIgnore]
        public int NumJoints { get; private set; }

        /// <summary>
        ///     Gets the links as created in the constructor.
        /// </summary>
        /// <value>All links.</value>
        [JsonIgnore]
        public List<Link> Links { get; set; }

        /// <summary>
        ///     Gets all of the links, which includes the additional ones created for the simulation.
        /// </summary>
        /// <value>All joints.</value>
        private List<Link> SimulationLinks { get; set; }
        /// <summary>
        ///     Gets the joints as created in the constructor.
        /// </summary>
        /// <value>All joints.</value>
        public List<Joint> Joints { get; set; }

        /// <summary>
        ///     Gets all of the joints, which includes the additional ones created for the simulation.
        /// </summary>
        /// <value>All joints.</value>
        private List<Joint> SimulationJoints { get; set; }

        /// <summary>
        ///     Gets the first index of the input joint.
        /// </summary>
        /// <value>The first index of the input joint.</value>
        internal int firstInputJointIndex { get; private set; }
        internal int firstGroundJointIndex { get; private set; }
        
        /// <summary>
        ///     Gets the index of the input joint.
        /// </summary>
        /// <value>The index of the input joint.</value>
        public int DrivingIndex { get; set; }
        private int simulationDriveIndex;
        /// <summary>
        ///     Gets the input joint.
        /// </summary>
        /// <value>The input joint.</value>
        internal Joint inputJoint { get; private set; }

        /// <summary>
        ///     Gets the index of the input link.
        /// </summary>
        /// <value>The index of the input link.</value>
        internal int inputLinkIndex { get; private set; }

        /// <summary>
        ///     Gets the input link.
        /// </summary>
        /// <value>The input link.</value>
        internal Link inputLink { get; private set; }

        /// <summary>
        ///     Gets the ground link.
        /// </summary>
        /// <value>The ground link.</value>
        private Link groundLinkForSimulation;

        /// <summary>
        /// Gets the ground link.
        /// </summary>
        /// <value>
        /// The ground link.
        /// </value>
        [JsonIgnore]
        public Link GroundLink { get; private set; }

        /// <summary>
        ///     Gets the indices of joints (in the new PMKS order, not the order originially given)
        ///     that are not to be updated or assigned from the AssignPositions function.
        /// </summary>
        /// <value>The index of the output joint.</value>
        internal int[] ReadOnlyJointIndices { get; private set; }

        /// <summary>
        ///     The name base for gear connector
        /// </summary>
        private const string nameBaseForGearConnector = "auto_generated_gear_connect_";

        /// <summary>
        ///     Gets a value indicating whether this instance is dyadic.
        /// </summary>
        /// <value><c>true</c> if this instance is dyadic; otherwise, <c>false</c>.</value>
        public Boolean IsDyadic = true;



        // everything else gets stored here
        [JsonExtensionData]
        private IDictionary<string, JToken> _additionalData;
        #endregion

        #region Constructor - requires the topology of the mechanism to be provided.

        /// <summary>
        ///     Initializes a new instance of the <see cref="Simulator" /> class.
        /// </summary>
        /// <param name="linkIDs">The link IDs.</param>
        /// <param name="jointTypes">The pivot types.</param>
        /// <param name="driverIndex">Index of the driver.</param>
        /// <param name="initPositions">The init positions.</param>
        /// <param name="readOnlyJointIndices">The fixed or unassignable joint indices.</param>
        public Simulator(IList<string[]> linkIDs, IList<JointType> jointTypes, int driverIndex = 0,
            IList<double[]> initPositions = null,
            int[] readOnlyJointIndices = null)
        {
            CreateLinkAndPositionDetails(linkIDs, jointTypes, driverIndex, initPositions);
            RearrangeLinkAndJointLists();
            setAdditionalReferencePositions();
            ReadOnlyJointIndices = readOnlyJointIndices;
        }

        public void StoreJson(Stream stream)
        {
            JsonSerializer serializer = new JsonSerializer
            {
                //NullValueHandling = NullValueHandling.Ignore,
                //DefaultValueHandling = DefaultValueHandling.Ignore,
                TypeNameHandling = TypeNameHandling.Auto,
                ReferenceLoopHandling = ReferenceLoopHandling.Ignore,
                Formatting = Formatting.Indented
            };
            var sw = new StreamWriter(stream);
            using (var writer = new JsonTextWriter(sw))
                serializer.Serialize(writer, this);
        }

        public static Simulator CreateFromJsonStream(Stream stream)
        {
            var serializer = new JsonSerializer();
            var sr = new StreamReader(stream);
            using (var reader = new JsonTextReader(sr))
                return serializer.Deserialize<Simulator>(reader);
        }
        [OnDeserialized]
        internal void OnDeserializedMethod(StreamingContext context)
        {
            additionalRefjoints = new List<Joint>();
            var linkNames = new List<string>();
            foreach (var joint in Joints)
            {
                linkNames.Add(joint.Links[0]);
                if (joint.Links.Length > 1) linkNames.Add(joint.Links[1]);
            }
            for (var i = 0; i < linkNames.Count; i++)
                if (IsGroundLinkName(linkNames[i])) linkNames[i] = "ground";
            linkNames = linkNames.Distinct()
                .Where(s => !string.IsNullOrWhiteSpace(s)).ToList();

            Links = new List<Link>(); //create an array of LINKS
            foreach (var linkName in linkNames)
            {
                var joints = Joints.Where(j => j.Links.Contains(linkName));
                Links.Add(new Link(linkName, joints));
            }
            /* now that links have been created, need to add these to joints */
            foreach (var j in Joints)
            {
                j.Link1 = Links.First(a => j.Links[0].Equals(a.name));
                if (j.Links.Length > 1)
                    j.Link2 = Links.First(a => j.Links[1].Equals(a.name));
            }
            NumLinks = Links.Count; //count the number of links in the system
            NumJoints = Joints.Count; //count the number of pivots in the system
            maxJointParamLengths = Joints.All(j => j.TypeOfJoint == JointType.R) ? 6 : 9;
            RearrangeLinkAndJointLists();
            setAdditionalReferencePositions();
        }

        public static Simulator CreateFromTextStream(Stream stream)
        {
            var sr = new StreamReader(stream);
            var all = sr.ReadToEnd();
            return new Simulator(all);
        }
        /// <summary>
        /// This is the default constructor for serialization.
        /// </summary>
        public Simulator() { }

        /// <summary>
        ///     Initializes a new instance of the <see cref="Simulator" /> class.
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
            setAdditionalReferencePositions();
        }

        /// <summary>
        ///     Converts the text to data.
        /// </summary>
        /// <param name="text">The text.</param>
        /// <param name="linkIDs">The link i ds.</param>
        /// <param name="jointTypes">The joint types.</param>
        /// <param name="driverIndex">Index of the driver.</param>
        /// <param name="initPositions">The initialize positions.</param>
        /// <param name="displayBools">The display bools.</param>
        /// <returns><c>true</c> if XXXX, <c>false</c> otherwise.</returns>
        public static bool ConvertTextToData(string text, out List<string[]> linkIDs, out List<JointType> jointTypes,
            out int driverIndex, out List<double[]> initPositions, out List<Boolean[]> displayBools)
        {
            linkIDs = new List<string[]>();
            jointTypes = new List<JointType>();
            initPositions = new List<double[]>();
            displayBools = new List<Boolean[]>();
            driverIndex = -1;
            var pivotSentences = text.Split('\n', '|').ToList();
            pivotSentences.RemoveAll(string.IsNullOrWhiteSpace);
            if (pivotSentences.Count == 0) return false;
            for (var jointIndex = 0; jointIndex < pivotSentences.Count; jointIndex++)
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
                    for (var i = 0; i < plusMinusString.Length; i++)
                        if (plusMinusString[i].Equals('t') || plusMinusString[i].Equals('+')) bools[i] = true;
                }
                else
                {
                    for (var i = position; i < words.Count; i++)
                        Boolean.TryParse(words[position], out bools[i - position]);
                }
                if (bools.Count() >= 4 && bools[3]) driverIndex = jointIndex;
                displayBools.Add(bools);
                words.RemoveRange(jointTypePosition, words.Count - jointTypePosition);
                linkIDs.Add(words.ToArray());
            }
            if (driverIndex == -1)
            {
                for (var i = 0; i < linkIDs.Count; i++)
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
        ///     Creates the link and position details.
        /// </summary>
        /// <param name="linkIDs">The link i ds.</param>
        /// <param name="jointTypes">The joint type strings.</param>
        /// <param name="drivingIndex">Index of the driving.</param>
        /// <param name="positions">The positions.</param>
        /// <exception cref="Exception">
        ///     The number of PivotTypes (which is  + numJoints + ) must be the
        ///     + same as the number of LinkID pairs (which is  + LinkIDs.Count + )
        ///     or
        ///     More than two links is not allowed for  + JointTypeStrings[i] +  joints.
        ///     or
        ///     Input cannot be gear teeth.
        ///     or
        ///     Input cannot be an RP joint (2 DOF inputs are not allowed).
        ///     or
        ///     There can only be one ground link. In this case, there are
        ///     + groundLinks.Count
        ///     or
        ///     Failed to construct Planar Mechanism Simulator from topology data (InputIndex, Connections, PivotTypes).
        /// </exception>
        private void CreateLinkAndPositionDetails(IList<string[]> linkIDs, IList<JointType> jointTypes,
            int drivingIndex, IList<double[]> positions = null)
        {
            try
            {
                this.DrivingIndex = drivingIndex;
                if (jointTypes.Count != linkIDs.Count)
                    throw new Exception("The number of PivotTypes (which is " + NumAllJoints + ") must be the"
                                        + "same as the number of LinkID pairs (which is " + linkIDs.Count + ")");

                foreach (var linkID in linkIDs)
                    for (var i = linkID.GetLength(0) - 1; i >= 0; i--)
                        if (IsGroundLinkName(linkID[i])) linkID[i] = "ground";
                var linkNames = linkIDs.SelectMany(a => a).Distinct().ToList();

                var newLinkIDs = new List<List<string>>();
                /* create the pivots */
                Joints = new List<Joint>();
                additionalRefjoints = new List<Joint>();
                for (var i = 0; i < jointTypes.Count; i++)
                {
                    double[] currentJointPosition = null;
                    if (positions != null) currentJointPosition = positions[i];
                    if (linkIDs[i].GetLength(0) == 1)
                    {
                        /* if the specified joint is only a point to be traced on a given link, then we make it
                         * an "R" joint. */
                        Joints.Add(new Joint((linkIDs[i][0] == "ground"), JointType.R, currentJointPosition));
                        newLinkIDs.Add(new List<string> { linkIDs[i][0] });
                    }
                    else
                        for (var j = 0; j < linkIDs[i].GetLength(0) - 1; j++)
                        {
                            /* At the end of these conditions, the "else" statement handles the normal two-link joint. 
                             * If there are more than 2  links at this joint, then it's actually multiple co-located joints. 
                             * This really only make sense for "R" joints. We assume that if it is typed as an RP      
                             * joint then between 0 and 1 there is an RP; and between 1 and 2, 2 and 3 and so on, it is an R joint. */
                            if (j > 0 && jointTypes[i] == JointType.RP)
                                additionalRefjoints.Add(new Joint(Joints.Last()));
                            /* if there are more than 2 links and the joint is typed G or P then we throw an error. */
                            else if (j > 0 && (jointTypes[i] == JointType.G || jointTypes[i] == JointType.P))
                                throw new Exception("More than two links is not allowed for " + jointTypes[i] +
                                                    " joints.");
                            /* else...this is the normal case of a joint between two links. */
                            else
                                Joints.Add(new Joint(linkIDs[i].Contains("ground"), jointTypes[i], currentJointPosition));
                            newLinkIDs.Add(new List<string> { linkIDs[i][j], linkIDs[i][j + 1] });
                        }
                }
                maxJointParamLengths = Joints.All(j => j.TypeOfJoint == JointType.R) ? 6 : 9;
                /* now onto the links */
                Links = new List<Link>(); //create an array of LINKS
                foreach (var linkName in linkNames)
                {
                    var pivotIndices =
                        newLinkIDs.Where(lid => lid.Contains(linkName)).Select(lid => newLinkIDs.IndexOf(lid));
                    var pivotsForThisLink = pivotIndices.Select(i => Joints[i]);
                    Links.Add(new Link(linkName, pivotsForThisLink));
                }
                /* now that links have been created, need to add these to pivots */
                for (var i = 0; i < newLinkIDs.Count; i++)
                {
                    Joints[i].Link1 = Links[linkNames.IndexOf(newLinkIDs[i][0])];
                    if (newLinkIDs[i].Count > 1)
                        Joints[i].Link2 = Links[linkNames.IndexOf(newLinkIDs[i][1])];
                }
            }
            catch (Exception e)
            {
                throw new Exception(
                    "Failed to construct Planar Mechanism Simulator from topology data (InputIndex, Connections, PivotTypes).",
                    e);
            }
            NumLinks = Links.Count; //count the number of links in the system
            NumJoints = Joints.Count; //count the number of pivots in the system
        }

        public static bool IsGroundLinkName(string linkName)
        {
            if (linkName.Equals("0", StringComparison.CurrentCultureIgnoreCase)
                || linkName.Equals("gnd", StringComparison.CurrentCultureIgnoreCase)
                || linkName.Equals("grnd", StringComparison.CurrentCultureIgnoreCase)
                || linkName.Equals("grond", StringComparison.CurrentCultureIgnoreCase)
                || linkName.Equals("gound", StringComparison.CurrentCultureIgnoreCase)
                || linkName.Equals("groud", StringComparison.CurrentCultureIgnoreCase)
                || linkName.Equals("ground", StringComparison.CurrentCultureIgnoreCase)
                || linkName.StartsWith("ground", StringComparison.CurrentCultureIgnoreCase))
                return true;
            return false;
        }
        private void RearrangeLinkAndJointLists()
        {
            #region Define Input Joint and Link, and Ground Link
            inputJoint = Joints[DrivingIndex];
            if (inputJoint.TypeOfJoint == JointType.G) throw new Exception("Input cannot be gear teeth.");
            if (inputJoint.TypeOfJoint == JointType.RP)
                throw new Exception("Input cannot be an RP joint (2 DOF inputs are not allowed).");
            var groundLinks = Links.Where(c => c.IsGround).ToList(); //move inputLink to back of list
            if (groundLinks.Count != 1)
                throw new Exception("There can only be one ground link. In this case, there are "
                                    + groundLinks.Count);
            GroundLink = groundLinkForSimulation = groundLinks[0];
            if (inputJoint.Link2 == null)
                throw new Exception(
                    "The input driver must be between two links (cannot be applied to a single link as in the current description).");
            if (!inputJoint.Link1.IsGround && !inputJoint.Link2.IsGround)
            {
                invertedToSolveNonGroundInput = true;
                GroundLink = groundLinkForSimulation;
                var numFixedJointsOnLink1 = inputJoint.Link1.Joints.Count(j => j.FixedWithRespectTo(inputJoint.Link1));
                if (numFixedJointsOnLink1 >= 2)
                {
                    inputLink = inputJoint.Link2;
                    groundLinkForSimulation = inputJoint.Link1;
                }
                else
                {
                    var numFixedJointsOnLink2 =
                        inputJoint.Link2.Joints.Count(j => j.FixedWithRespectTo(inputJoint.Link2));
                    if (numFixedJointsOnLink2 >= 2)
                    {
                        inputLink = inputJoint.Link1;
                        groundLinkForSimulation = inputJoint.Link2;
                    }
                    else
                    {
                        groundLinkForSimulation = numFixedJointsOnLink1 > 0 ? inputJoint.Link1 : inputJoint.Link2;
                        inputLink = numFixedJointsOnLink1 > 0 ? inputJoint.Link2 : inputJoint.Link1;
                        var newJoint = new Joint(true, JointType.R, null, JointSpecifiedAs.FixedPointForLink) { Link1 = groundLinkForSimulation };
                        groundLinkForSimulation.Joints.Add(newJoint);
                        additionalRefjoints.Add(newJoint);
                    }
                }
                groundLinkForSimulation.IsGround = true;
                GroundLink.IsGround = false;
                if (GroundLink.Joints.Count(j => j.FixedWithRespectTo(GroundLink)) < 2)
                {
                    var newJoint = new Joint(false, JointType.R, null, JointSpecifiedAs.FixedPointForLink) { Link1 = GroundLink };
                    GroundLink.Joints.Add(newJoint);
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
            #endregion
            SimulationLinks = new List<Link>(Links);
            SimulationJoints = new List<Joint>(Joints);
            SimulationJoints.AddRange(additionalRefjoints);
            addReferenceJoints();
            #region Move Links around to ease Simulation
            /* reorder links to ease additional computation. put ground link at end, 
             * move input to just before those. */
            SimulationLinks.Remove(inputLink);
            SimulationLinks.Remove(groundLinkForSimulation);
            inputLinkIndex = SimulationLinks.Count;
            SimulationLinks.Add(inputLink); //move inputLink to back of list
            SimulationLinks.Add(groundLinkForSimulation); //move ground to back of list
            #endregion
            #region Set up SimulationJoints by moving Joints around to ease Simulation
            /* reorder links, move input link and ground link to back of list */
            var groundPivots =
                groundLinkForSimulation.Joints.Where(
                    j => j != inputJoint && j.FixedWithRespectTo(groundLinkForSimulation)).ToList();
            SimulationJoints.Remove(inputJoint);
            SimulationJoints.RemoveAll(groundPivots.Contains);

            var connectedInputJoints =
                inputLink.Joints.Where(j => j != inputJoint && j.FixedWithRespectTo(inputLink)).ToList();
            SimulationJoints.RemoveAll(connectedInputJoints.Contains);
            firstInputJointIndex = SimulationJoints.Count;
            SimulationJoints.AddRange(connectedInputJoints);
            simulationDriveIndex = SimulationJoints.Count;
            SimulationJoints.Add(inputJoint);
            firstGroundJointIndex = SimulationJoints.Count;
            SimulationJoints.AddRange(groundPivots);
            #endregion

            NumAllJoints = SimulationJoints.Count;
            NumAllLinks = SimulationLinks.Count;

            oIOSJ = new int[NumAllJoints];
            var nonDesignJointIndex = NumJoints;
            for (var i = 0; i < NumAllJoints; i++)
            {
                var index = Joints.IndexOf(SimulationJoints[i]);
                oIOSJ[i] = (index == -1) ? nonDesignJointIndex++ : index;
            }
            oIOSL = new int[NumAllLinks];
            var nonDesignLinkIndex = NumJoints;
            for (var i = 0; i < NumAllLinks; i++)
            {
                var index = Links.IndexOf(SimulationLinks[i]);
                oIOSL[i] = (index == -1) ? nonDesignLinkIndex++ : index;
            }
        }

        /// <summary>
        /// Gets or sets the O.I.O.S.J., the original indices of Simulated Joints.
        /// </summary>
        /// <value>The o iosj.</value>
        private int[] oIOSJ { get; set; }
        private int[] oIOSL { get; set; }


        /// <summary>
        ///     Adds the reference pivots to slide only links.
        /// </summary>
        /// <returns>List&lt;joint&gt;.</returns>
        private void addReferenceJoints()
        {
            foreach (var c in SimulationLinks.Where(c => !c.Joints.Any(j => j.FixedWithRespectTo(c))))
            {
                var newJoint = new Joint(c.IsGround, JointType.R) { Link1 = c };
                c.Joints.Add(newJoint);
                additionalRefjoints.Add(newJoint);
            }

            if (SimulationJoints.All(j => j.TypeOfJoint != JointType.G)) return;
            var counter = 0;
            foreach (var j in SimulationJoints)
            {
                if (j.TypeOfJoint != JointType.G) continue;
                // link1Neighbors are the links adjacent to this gear joint's link1
                var link1Neighbors = j.Link1.Joints.Select(jj => jj.OtherLink(j.Link1)).ToList();
                // if any of the joints on link2 (not including this gear joint) connect to the link1Neighbors,
                // then it is assumed that a support link and it's two joints are not needed
                if (j.Link2.Joints.Any(
                    jj => jj != j && link1Neighbors.Contains(jj.OtherLink(j.Link2)))) continue;
                if (double.IsNaN(j.OffsetSlideAngle))
                    throw new Exception("No link connects between gears: " + j.Link1.name + " and " + j.Link2.name);
                var newJoint1 = new Joint(j.Link1.IsGround, JointType.P, new[] { j.XInitial, j.YInitial, j.OffsetSlideAngle }, JointSpecifiedAs.GearSupport);
                var gearCenter2 = j.Link2.Joints.FirstOrDefault(jj => jj != j && jj.TypeOfJoint == JointType.R);
                if (gearCenter2 == null) throw new Exception("No pivot (R joint) for " + j.Link2.name);
                var newJoint2 = new Joint(j.Link2.IsGround, JointType.R, new[] { gearCenter2.XInitial, gearCenter2.YInitial }, JointSpecifiedAs.GearSupport);
                var connectLink = new Link(nameBaseForGearConnector + (counter++), new List<Joint> { newJoint1, newJoint2 });
                newJoint1.Link1 = j.Link1;
                newJoint2.Link1 = j.Link2;
                newJoint1.Link2 = newJoint2.Link2 = connectLink;
                additionalRefjoints.Add(newJoint1);
                additionalRefjoints.Add(newJoint2);
                SimulationLinks.Add(connectLink);
            }
        }

        /// <summary>
        ///     Sets the positions of additional reference positions.
        /// </summary>
        private void setAdditionalReferencePositions()
        {
            if (additionalRefjoints == null) return;
            foreach (var joint in additionalRefjoints)
            {
                switch (joint.jointSpecifiedAs)
                {
                    case JointSpecifiedAs.Design:
                        continue;
                    case JointSpecifiedAs.TernaryJoint:
                        joint.x =
                            joint.xNumerical = joint.xLast = joint.XInitial
                                = joint.sameJointAs.XInitial;
                        joint.y =
                            joint.yNumerical = joint.yLast = joint.YInitial
                                = joint.sameJointAs.YInitial;
                        break;
                    case JointSpecifiedAs.FixedPointForLink:
                        var xSum = 0.0;
                        var ySum = 0.0;
                        foreach (var otherJoint in joint.Link1.Joints)
                        {
                            if (otherJoint == joint) continue;
                            xSum += otherJoint.XInitial;
                            ySum += otherJoint.YInitial;
                        }
                        joint.x =
                            joint.xNumerical = joint.xLast = joint.XInitial
                                = xSum / (joint.Link1.Joints.Count - 1);
                        joint.y =
                            joint.yNumerical = joint.yLast = joint.YInitial
                                = ySum / (joint.Link1.Joints.Count - 1);
                        break;
                    case JointSpecifiedAs.GearSupport:
                        setGearData(); //todo: this function is kind of a mess.Need to revisit auto-created joints for gears
                        break;
                }
            }
            var totalLength = 0.0;
            var numLengths = 0;
            foreach (var eachLink in SimulationLinks)
            {
                eachLink.DetermineLengthsAndReferences();
                totalLength += eachLink.TotalLength;
                numLengths += eachLink.Joints.Count * (eachLink.Joints.Count - 1) / 2;
            }
            AverageLength = totalLength / numLengths;
        }


        /// <summary>
        ///     Sets the gear data.
        /// </summary>
        private void setGearData()
        {
            var index = 0;
            if (SimulationJoints.All(j => j.TypeOfJoint != JointType.G)) return;
            gearsData = new Dictionary<int, GearData>();
            foreach (var gearTeethJoint in SimulationJoints)
            {
                if (gearTeethJoint.TypeOfJoint == JointType.G)
                {
                    var gear1 = gearTeethJoint.Link1;
                    var gear2 = gearTeethJoint.Link2;
                    var possibleGear1Centers = gear1.Joints.Where(j => j.TypeOfJoint != JointType.G && j.Link2 != null);
                    var possibleGear2Centers =
                        gear2.Joints.Where(j => j.TypeOfJoint != JointType.G && j.Link2 != null).ToList();
                    var bestCenterOption = new Tuple<Joint, Joint, Link, double>(null, null, null, double.NaN);
                    var bestError = double.PositiveInfinity;
                    foreach (var g1 in possibleGear1Centers)
                        foreach (var g2 in possibleGear2Centers)
                        {
                            if (g1.TypeOfJoint == JointType.P && g2.TypeOfJoint == JointType.P) continue;
                            if (g1.TypeOfJoint == JointType.RP && g1.Link1 == gear1) continue;
                            if (g2.TypeOfJoint == JointType.RP && g2.Link1 == gear2) continue;
                            var connectLink =
                                SimulationLinks.FirstOrDefault(c => c.Joints.Contains(g1) && c.Joints.Contains(g2));
                            if (connectLink == null) continue;
                            var angle1 = FindInitialGearAngle(g1, gearTeethJoint);
                            var angle2 = FindInitialGearAngle(g2, gearTeethJoint);
                            var error = Math.Abs(angle1 - angle2);
                            if (bestError > error)
                            {
                                bestError = error;
                                bestCenterOption = new Tuple<Joint, Joint, Link, double>(g1, g2, connectLink,
                                    (angle1 + angle2) / 2);
                            }
                        }
                    var gearCenter1 = bestCenterOption.Item1;
                    var gearCenter2 = bestCenterOption.Item2;
                    var connectingRod = bestCenterOption.Item3;
                    var gearAngle = bestCenterOption.Item4;
                    gearsData.Add(index,
                        new GearData(gearTeethJoint, index, gearCenter1, SimulationJoints.IndexOf(gearCenter1),
                            SimulationLinks.IndexOf(gear1),
                            gearCenter2, SimulationJoints.IndexOf(gearCenter2), SimulationLinks.IndexOf(gear2),
                            SimulationLinks.IndexOf(connectingRod), gearAngle));
                }
                index++;
            }
        }

        /// <summary>
        ///     Finds the initial gear angle.
        /// </summary>
        /// <param name="g1">The g1.</param>
        /// <param name="gearTeeth">The gear teeth.</param>
        /// <returns>System.Double.</returns>
        private double FindInitialGearAngle(Joint g1, Joint gearTeeth)
        {
            if (g1.TypeOfJoint != JointType.R) return g1.SlideAngleInitial;
            var angle = Math.Atan2((gearTeeth.YInitial - g1.YInitial), (gearTeeth.XInitial - g1.XInitial))
                        + Constants.QuarterCircle;
            while (angle > Constants.QuarterCircle) angle -= Math.PI;
            while (angle < -Constants.QuarterCircle) angle += Math.PI;
            return angle;
        }

        #endregion
    }
}