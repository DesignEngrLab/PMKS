// ***********************************************************************
// Assembly         : PlanarMechanismKinematicSimulator
// Author           : Matt
// Created          : 06-10-2015
//
// Last Modified By : Matt
// Last Modified On : 06-28-2015
// ***********************************************************************
// <copyright file="PlanarMechanismSimulator.FindFullMovement.cs" company="">
//     Copyright ©  2014
// </copyright>
// <summary></summary>
// ***********************************************************************
using OptimizationToolbox;
using PMKS.PositionSolving;
using PMKS.VelocityAndAcceleration;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace PMKS
{
    /// <summary>
    /// Class Simulator.
    /// </summary>
    public partial class Simulator : IDependentAnalysis
    {
        /// <summary>
        /// The use error method
        /// </summary>
        private bool useErrorMethod;

        /// <summary>Finds the full movement.</summary>
        /// <returns>
        ///   <c>true</c> if simulation was successful, <c>false</c> otherwise.</returns>
        /// <exception cref="Exception">Either the smoothing error angle delta or the time step must be specified.</exception>
        /// <exception cref="System.Exception">Either the smoothing error angle delta or the time step must be specified.</exception>
        public bool FindFullMovement()
        {
            if (double.IsNaN(DeltaAngle) && double.IsNaN(FixedTimeStep) && double.IsNaN(MaxSmoothingError))
            {
                throw new Exception(
                     "Either the smoothing error angle delta or the time step must be specified.");
                return false;
            }
            useErrorMethod = (!double.IsNaN(MaxSmoothingError) && MaxSmoothingError > 0);

            #region Set up initial point parameters (x, x-dot, x-double-dot, etc.)

            double[,] initJointParams, initLinkParams, initForceParams;

            SetInitialVelocityAndAcceleration(SimulationJoints, SimulationLinks, out initJointParams, out initLinkParams);
            SetInitialForceParams(SimulationForces, initJointParams, SimulationLinks, out initForceParams);

            JointParameters = new TimeSortedList { { 0.0, initJointParams } };
            LinkParameters = new TimeSortedList { { 0.0, initLinkParams } };
            ForceParameters = new TimeSortedList { { 0.0, initForceParams } };

            InputRange = new[] { inputLink.AngleInitial, inputLink.AngleInitial };
            if (double.IsNaN(InputSpeed) || InputSpeed == 0.0) InputSpeed = Constants.DefaultInputSpeed;

            #endregion

#if DEBUGSERIAL
            Simulate(SimulationJoints, SimulationLinks, SimulationForces, true);
#else
            List<Joint> backwardJoints;
            List<Link> backwardLinks;
            List<Force> backwardForces;
            CopyJointsAndLinksForBackwards(SimulationJoints, SimulationLinks, SimulationForces, out backwardJoints, out backwardLinks, out backwardForces);

            /*** Stepping Forward in Time ***/
            var forwardTask = Task.Factory.StartNew(() => Simulate(SimulationJoints, SimulationLinks, SimulationForces,  true));

            /*** Stepping Backward in Time ***/
            var backwardTask = Task.Factory.StartNew(() => Simulate(backwardJoints, backwardLinks, backwardForces, false));
            Task.WaitAll(forwardTask, backwardTask);
#endif
            if (this.JointParameters.Count < 2) return false;
            DefineMovementCharacteristics();
            if (invertedToSolveNonGroundInput)
                TransposeLinkAndJointParameters();
            return true;
        }

        /// <summary>
        /// Turn a bool into a double
        /// </summary>
        /// <param name="b"> a bool </param>
        /// <returns> a double as a bool equivalent </returns>
        private double b2d(bool b)
        {
            return (b ? 1.0 : 0.0);
        }


        /// <summary>
        /// Finds the index of a given link.
        /// </summary>
        /// <returns>The link index.</returns>
        /// <param name="link">Link.</param>
        public int FindLinkIndex(Link link)
        {
            for (int i = 0; i < this.Links.Count(); i++)
            {
                if (link.Equals(this.Links[i]))
                {
                    return i;
                }
            }
            return -1;
        }

        public List<string> Analysis(bool isStatic, bool justOne, int jP)
        {
            List<string> output = new List<string>();
            double maxTorque = 0.0;

            VolumeCalculation(); //in native units
            MasslinkCalculation(); //in KG
            
            var initialLinks = this.LinkParameters.Parameters[0];

            //Initialize Centers of Mass
            for (int i = 0; i < this.Links.Count(); i++)
            {
                double jointXsum = 0;
                double jointYsum = 0;
                double numJoints = (double)this.Links[i].joints.Count();
                for (int j = 0; j < this.Links[i].joints.Count(); j++)
                {
                    jointXsum += this.Links[i].joints[j].xInitial;
                    jointYsum += this.Links[i].joints[j].yInitial;
                }
                initialLinks[i, 3] = jointXsum / numJoints;
                initialLinks[i, 4] = jointYsum / numJoints;
            }

            var initialForces = this.ForceParameters.Parameters[0];
            SetMoI();

            List<int> force_Link = new List<int>();
            int thelink = 0;
            for (int i = 0; i < NumForces; i++)
            {
                int linknumber = Determine_Link_With_Force(initialForces[i, 0], initialForces[i, 1]);
                if (linknumber != 999)
                {
                    force_Link.Add(linknumber);
                }
                thelink = linknumber;
            }

            #region outputFileHeader

            if (isStatic)
            {
                output.Add("Static Analysis\n");
            }
            else
            {
                output.Add("Dynamic Analysis\n");
            }

            //should probably display units... but only if the units matter -- if (gravity || dynamics) 

            output.Add("The mechanism to be analyzed is as follows:");
            output.Add("Joint:\tLinks attached:");

            for (int i = 0; i < this.Joints.Count(); i++)
            {
                string jointLine = "";
                jointLine += ("(" + this.Joints[i].xInitial.ToString() + ", " + this.Joints[i].yInitial.ToString() + ")");
                jointLine += "\t" + this.Joints[i].Link1.name + ", " + this.Joints[i].Link2.name;

                output.Add(jointLine);
            }

            AssignBasicLayers();
            //here is where we assign advanced layers "CollisionFreeLayers"

            output.Add("\nLinks and their layers:");
            for (int i = 0; i < this.Links.Count(); i++)
            {
                output.Add("Link:\t" + this.Links[i].name + "\tLayer: " + this.Links[i].Layer.ToString());
            }

            output.Add("\nForces:");
            if (force_Link.Count <= 0)
            {
                output.Add("No applied forces on links");
            }
            else
            {
                output.Add("Link:\tX Location:\tY Location:\tX Magnitude:\tY Magnitude:");

                for (int i = 0; i < force_Link.Count(); i++)
                {
                    if (this.Links[force_Link[i]].IsGround)
                    {
                        continue;
                    }

                    output.Add(this.Links[force_Link[i]].name + "\t" +
                        initialForces[i, 0].ToString() + "\t" +
                        initialForces[i, 1].ToString() + "\t" +
                        initialForces[i, 2].ToString() + "\t" +
                        initialForces[i, 3].ToString());
                }
            }

            if (!isStatic)
            {
                output.Add("\nLink Centers of Mass and Mass Moments of Inertia");
                output.Add("Link:\tCoM x:\tCoM y:\tMass Moment of Inertia:");
                for (int i = 0; i < this.Links.Count; i++)
                {
                    if (this.Links[i].IsGround)
                    {
                        continue;
                    }
                    output.Add(this.Links[i].name + "\t" + initialLinks[i, 3].ToString() + "\t" + initialLinks[i, 4].ToString() + "\t" + this.Links[i].MoI.ToString());
                }
            }

            #endregion

            List<int> jointOrderInMatrix = new List<int>();

            List<string> unknownStrMatrix = new List<string>(); //this is for output file purposes.
            List<string> coeffStrMatrix = new List<string>();
            List<string> knownStrMatrix = new List<string>();


            for (int jointInstance = 0; jointInstance < this.JointParameters.Count(); jointInstance++)
            {
                if (justOne) {
                    jointInstance = jP;
                }
                var currentJoints = this.JointParameters.Parameters[jointInstance];
                var currentForces = this.ForceParameters.Parameters[jointInstance];
                var currentLinks = this.LinkParameters.Parameters[jointInstance]; //used for dynamics

                //CoM stuff for dynamics
                List<double> comAccX = new List<double>();
                List<double> comAccY = new List<double>();
                List<double> comRx = new List<double>();
                List<double> comRy = new List<double>();

                if (!isStatic)
                {
                    for (int i = 0; i < this.Links.Count(); i++)
                    {
                        int primeJoint = FindJPIndex(this.Links[i].joints[0].xInitial, this.Links[i].joints[0].yInitial); //first joint of each link for reference
                        comRx.Add(currentLinks[i, 3] - currentJoints[primeJoint, 0]); //x component
                        comRy.Add(currentLinks[i, 4] - currentJoints[primeJoint, 1]); //y component
                        // need to cross current link's angular acc with r and then add omega crossed with (omega crossed with r)
                        // naming convention here is to put type output as the end of variable name
                        double angAccCrossRj = currentLinks[i, 2] * comRx[i]; //acc cross comRx
                        double angAccCrossRi = -1.0 * currentLinks[i, 2] * comRy[i]; //acc cross comRy
                        double angVelCrossRj = currentLinks[i, 1] * comRx[i]; //vel cross comRx
                        double angVelCrossRi = -1.0 * currentLinks[i, 1] * comRy[i]; //vel cross comRy
                        double angVelCrossJ2 = currentLinks[i, 1] * angVelCrossRi; //vel cross (vel cross comRy)
                        double angVelCrossI2 = -1.0 * currentLinks[i, 1] * angVelCrossRj; //vel cross (vel cross comRx)
                        double accX = angAccCrossRi + angVelCrossI2 + currentJoints[primeJoint, 4];
                        double accY = angAccCrossRj + angVelCrossJ2 + currentJoints[primeJoint, 5];
                        comAccX.Add(accX); //adding the x component acc at joint
                        comAccY.Add(accY); //adding the y component acc at joint
                        currentLinks[i, 5] = accX; //x com acc
                        currentLinks[i, 6] = accY; //y com acc
                    }
                }

                //the next step is to setup force equations for each joint. 

                List<double> reactionForceX = new List<double>();
                List<double> reactionForceY = new List<double>();

                //determine the actual number of joints
                int noOfJoints = 0;
                List<int> jointIndices = new List<int>();
                int ii = 0;
                foreach (var n in this.Joints)
                {
                    if (n.Link1 != null && n.Link2 != null)
                    {
                        noOfJoints++;
                        jointIndices.Add(ii);
                    }
                    ii++;
                }
                System.Diagnostics.Debug.WriteLine("number of joints with two links" + noOfJoints);

                //now determine the size of the matrix. 
                //this is assuming all are revolute joints. 

                int noOfUnknowns = noOfJoints * 2 + 1;

                var knownMatrix = new double[noOfUnknowns, 1];
                int[,] unknownMatrix = new int[noOfUnknowns, 1];
                ii = 0;
                for (int i = 0; i < jointIndices.Count; i++)
                {
                    reactionForceX.Add(1);
                    reactionForceY.Add(1);

                    //we can also populate the unknown matrix order 

                    unknownMatrix[ii++, 0] = jointIndices[i];
                    unknownMatrix[ii++, 0] = jointIndices[i];
                }
                System.Diagnostics.Debug.WriteLine("number of reaction forces" + reactionForceX.Capacity);
                //the final entry in the matrix will be the torque

                //try to add NAN or null to knownMatrix 

                for (int i = 0; i < noOfUnknowns; i++)
                    knownMatrix[i, 0] = 0.000000009;

                //now move everything to a function 

                //the code regions given below will be moved to a function and called successively for changing joint coordinates. 

                #region sum of forces

                //have an index, the index will make sure that the reaction force for that joint has been added. If it has already been added, if that joint appears in another link, then the reaction forces need to have equal but opposite reaction forces 

                List<int> jointIndexIntoMatrix = new List<int>();
                int groundLinkIndex = 0;
                //below I am basically adding the sum of forces = 0 equation 

                ii = 0;
                for (int i = 0; i < this.Links.Count; i++) //looking at every link
                {
                    List<int> covered = new List<int>(); //determines whether a force has already been covered

                    if (this.Links[i].IsGround == false) //eliminating ground link
                    {
                        int startPoint = 0;
                        foreach (var n in this.Links[i].joints) //every link has a joint 
                        {
                            if (n.Link1 != null && n.Link2 != null) //making sure there are two links making a joint, else that's not a joint
                            {
                                //now populate the matrix
                                //first let us do the sum of forces in the x-direction, y-direction, then the moments

                                //compare this joint to the joint indices. That is the order of the entry into the matrix. 


                                //HERE joints must be set to their current positions, but check the inputs of FindJointIndexRelatedToThisJoint
                                int valN = FindJPIndex(n.xInitial, n.yInitial);
                                var joint1_x = currentJoints[valN, 0];
                                var joint1_y = currentJoints[valN, 1];

                                int whichIndex = FindJointIndexRelatedToThisJoint(joint1_x, joint1_y, jointIndices, jointInstance);

                                //now that the index has been determined, add that index into the jointIndexintomatrix and then add that to the matrix
                                if (whichIndex != 999)
                                {
                                    if (!jointIndexIntoMatrix.Contains(whichIndex))
                                    {
                                        jointIndexIntoMatrix.Add(whichIndex);

                                        //add the applied forces to the known matrix 

                                        if (force_Link.Contains(i))
                                        {
                                            //if there is more than one force on each link, we need to have a for loop

                                            for (int k = 0; k < force_Link.Count; k++)
                                            {
                                                bool alreadyCovered = false;
                                                for (int ban = 0; ban < covered.Count(); ban++)
                                                {
                                                    if (covered[ban] == k)
                                                    {
                                                        alreadyCovered = true;
                                                        break;
                                                    }
                                                }
                                                if (alreadyCovered)
                                                {
                                                    continue;
                                                }

                                                if (force_Link[k] == i)
                                                {
                                                    covered.Add(k);
                                                    if (knownMatrix[ii, 0] == 0.000000009 || knownMatrix[ii + 1, 0] == 0.000000009)
                                                    {
                                                        knownMatrix[ii, 0] = currentForces[k, 2] * -1; //the minus 1 multiplication is to indicate that this is being moved from the left to the right side of the equation
                                                        knownMatrix[ii + 1, 0] = currentForces[k, 3] * -1;
                                                    }
                                                    else
                                                    {
                                                        knownMatrix[ii, 0] = knownMatrix[ii, 0] + currentForces[k, 2] * -1;
                                                        knownMatrix[ii + 1, 0] = knownMatrix[ii + 1, 0] + currentForces[k, 3] * -1;
                                                    }
                                                }
                                            }
                                            startPoint++;
                                        }
                                    }
                                    else
                                    {
                                        ii++;
                                    }
                                }

                            }
                        }

                        ii++;
                    }
                    else
                        groundLinkIndex = i;
                }

                #endregion

                #region adding input torque into the coefficient matrix

                var input_Link_Index = 0;
                for (int li = 0; li < this.NumLinks; li++)
                {
                    if (this.Links[li].name.Equals("input"))
                    {
                        input_Link_Index = li - 1;
                        break;
                    }
                }
                //I know the location of the input link index as well as the ground link index

                //after eliminating the ground link, determine the position of the input link in the list of links. 

                List<int> createLinkList = new List<int>();

                for (int i = 0; i < this.NumLinks; i++)
                    createLinkList.Add(i);

                //remove groundlink from the list. 

                createLinkList.Remove(groundLinkIndex);

                //now determine the location of the input link index in the createlinklist. 

                int positionofInput = 0;
                for (int i = 0; i < createLinkList.Count; i++)
                {
                    if (createLinkList[i] == input_Link_Index)
                    {
                        positionofInput = i;
                    }
                }

                var startingMomentIndex = (this.NumLinks - 1) * 2;

                #endregion

                #region sum of moments

                //need to compute the rxF of unknown reactions

                var length1 = 0.0;
                for (int i = 0; i < this.NumLinks; i++)
                {
                    List<double> rVectorX = new List<double>();
                    List<double> rVectorY = new List<double>();
                    List<double> rVectorForceX = new List<double>();
                    List<double> rVectorForceY = new List<double>();
                    List<int> jointIndicesMoment = new List<int>();
                    if (this.Links[i].IsGround == false)
                    {
                        //assuming force is never at the ground link 

                        for (int j = 0; j <= this.Links[i].joints.Count - 1; j++) //this doesn't need to be a for-loop; only need the first joint
                        {

                            for (int k = j + 1; k < this.Links[i].joints.Count; k++) //joint 1 to n-1
                            {
                                int valK = FindJPIndex(this.Links[i].joints[k].xInitial, this.Links[i].joints[k].yInitial);
                                int valJ = FindJPIndex(this.Links[i].joints[j].xInitial, this.Links[i].joints[j].yInitial);

                                var momentIndex = FindJointIndexRelatedToThisJoint(currentJoints[valK, 0], currentJoints[valK, 1], jointIndices, jointInstance);//this is assuming that the third point in the triangle/polygon link is never declared first.

                                if (momentIndex != -1)
                                {
                                    length1 = FindVectorLength(currentJoints[valJ, 0], currentJoints[valK, 0], currentJoints[valJ, 1], currentJoints[valK, 1]);
                                    rVectorX.Add(GetVectorIntercept(currentJoints[valK, 0], currentJoints[valJ, 0]));
                                    rVectorY.Add(GetVectorIntercept(currentJoints[valK, 1], currentJoints[valJ, 1]));

                                    jointIndicesMoment.Add(momentIndex);
                                }
                            }
                            break;

                        }

                        //now that the r vectors have been determined, do the cross product 

                        //for that I need to determine the force I am using the 

                        for (int pp = 0; pp < rVectorX.Count; pp++)
                        {
                            //need to determine if the location of this link in the matrix and then determine the insertion point of the moment expressions

                            positionofInput = 0; //positionofinput is just using the previously defined variable
                            for (int iii = 0; iii < createLinkList.Count; iii++)
                                if (createLinkList[iii] == i)
                                    positionofInput = iii;

                            //having determined the position of this link

                            //next step is to locate the moment row starting position

                            startingMomentIndex = (this.NumLinks - 1) * 2;
                        }

                        //need to compute the rXF of applied forces and add that to the known matrix 

                        //now find if there is a force on this link 

                        if (force_Link.Contains(i))
                        {
                            for (int xx = 0; xx < force_Link.Count; xx++)
                            {
                                if (force_Link[xx] == i)
                                {
                                    //here forces
                                    int valI = FindJPIndex(this.Links[i].joints[0].xInitial, this.Links[i].joints[0].yInitial);
                                    //int linkRef = (int)currentForces[xx, 8];
                                    //old:
                                    //length1 = FindVectorLength(       currentJoints[(int)currentForces[xx, 5], 0],                       currentForces[xx, 0],                 currentJoints[(int)currentForces[xx, 5], 1],              currentForces[xx, 1]       );
                                    //new:
                                    length1 = FindVectorLength(currentJoints[valI, 0], currentForces[xx, 0], currentJoints[valI, 1], currentForces[xx, 1]);
                                    var rVectorFX = GetVectorIntercept(currentForces[xx, 0], currentJoints[valI, 0]); //Is this really the first reference joint??
                                    var rVectorFY = GetVectorIntercept(currentForces[xx, 1], currentJoints[valI, 1]);
                                    var rCrossF = -rVectorFY * currentForces[xx, 2] + rVectorFX * currentForces[xx, 3];
                                    rCrossF *= -1; //because it belongs on the other side of the equation anyways? mod
                                    //identify the moment row corresponding to this link in the known matrix 

                                    //need to determine if the location of this link in the matrix and then determine the insertion point of the moment expressions

                                    positionofInput = 0; //positionofinput is just using the previously defined variable
                                    for (int iii = 0; iii < createLinkList.Count; iii++)
                                        if (createLinkList[iii] == i)
                                            positionofInput = iii;

                                    //having determined the position of this link

                                    //next step is to locate the moment row starting position

                                    startingMomentIndex = (this.NumLinks - 1) * 2; //10

                                    if (knownMatrix[startingMomentIndex + positionofInput, 0] == 0.000000009)
                                        knownMatrix[startingMomentIndex + positionofInput, 0] = rCrossF;
                                    else
                                        knownMatrix[startingMomentIndex + positionofInput, 0] = knownMatrix[startingMomentIndex + positionofInput, 0] + rCrossF;
                                }
                            }
                        }
                    }
                }
                System.Diagnostics.Debug.WriteLine("known matrix pretty much done");

                #endregion

                #region create final known matrix
                //now make sure all the remaining known matrix values are 0. 

                for (var n = 0; n < knownMatrix.Length; n++)
                {
                    if (knownMatrix[n, 0] == 0.000000009)
                    {
                        knownMatrix[n, 0] = 0.0;
                    }
                }

                if (!isStatic)
                {
                    for (int i = 0; i < this.Links.Count(); i++)
                    {
                        if (!this.Links[i].IsGround)
                        {
                            knownMatrix[2 * i - 2, 0] += (this.Links[i].Mass * comAccX[i]);
                            knownMatrix[2 * i - 1, 0] += (this.Links[i].Mass * comAccY[i]);
                            knownMatrix[i + (2 * noOfUnknowns / 3) - 1, 0] += (this.Links[i].MoI * currentLinks[i, 2]); //for this to work, ground link must be 0
                        }
                    }
                }

                #endregion

                #region append position specific information
                //finally the coefficient matrix(A) and the known matrix(B) have been populated 
                //The unknown matrix (X) = inv(A)*B 
                //That will give us the forces and the input torque required

                MatrixGJ knownMatrixGJ = new MatrixGJ(knownMatrix);
                System.Diagnostics.Debug.WriteLine("known matrix done");
                MatrixGJ coeffMatrixGJ = new MatrixGJ(this.FindCoeffMatrix(noOfUnknowns, jointInstance));
                System.Diagnostics.Debug.WriteLine("coeff matrix done");
                coeffMatrixGJ.PrintMatrix();
                MatrixGJ unknownMatrixGJ = new MatrixGJ(new MatrixGJ(coeffMatrixGJ.Inverse()).Multiply(knownMatrixGJ));
                System.Diagnostics.Debug.WriteLine("unknown matrix done");
                //double radianDenom = Math.PI / (this.Links[1].AngleInitial + jointInstance * this.DeltaAngle);

                if (Math.Abs(unknownMatrixGJ.Matrix[noOfUnknowns - 1, 0]) > Math.Abs(maxTorque))
                {
                    maxTorque = unknownMatrixGJ.Matrix[noOfUnknowns - 1, 0];
                }

                if (isStatic)
                {
                    output.Add("\n\nStatic Analysis of position number " + (jointInstance + 1) + ".\n");
                }
                else
                {
                    output.Add("\n\nDynamic Analysis of position number " + (jointInstance + 1) + ".\n");
                }

                string angDeg = (((this.Links[1].AngleInitial + (jointInstance * this.DeltaAngle)) * (180 / Math.PI)) % 360).ToString();
                string angRad = (this.Links[1].AngleInitial + jointInstance * this.DeltaAngle).ToString();
                output.Add("The input is at an angle of " + angDeg + " degrees (" + angRad + " radians).");

                if (force_Link.Count > 0)
                {
                    output.Add("\nCurrent Forces:");
                    output.Add("Link:\tX Location:\tY Location:\tX Magnitude:\tY Magnitude:");
                    for (int i = 0; i < force_Link.Count(); i++)
                    {
                        if (this.Links[force_Link[i]].IsGround)
                        {
                            continue;
                        }

                        List<string> vals = new List<string>();
                        vals.Add(currentForces[i, 0].ToString());
                        vals.Add(currentForces[i, 1].ToString());
                        vals.Add(currentForces[i, 2].ToString());
                        vals.Add(currentForces[i, 3].ToString());

                        output.Add(this.Links[force_Link[i]].name + "\t" + vals[0] + "\t" + vals[1] + "\t" + vals[2] + "\t" + vals[3]);
                    }
                }

                unknownStrMatrix = unknownMatrixGJ.PrintMatrix();

                coeffStrMatrix = coeffMatrixGJ.PrintMatrix();

                //coeffMatrixGJ.PrintCleanMatrix();

                knownStrMatrix = knownMatrixGJ.PrintMatrix();

                
                jointOrderInMatrix = jointIndexIntoMatrix;

                List<string> solutionKey = new List<string>();
                //This is useful information for the key
                for (int i = 0; i < this.Joints.Count(); i++)
                {
                    string jx = this.Joints[jointOrderInMatrix[i]].xInitial.ToString();
                    string jy = this.Joints[jointOrderInMatrix[i]].yInitial.ToString();
                    string l1 = this.Joints[jointOrderInMatrix[i]].Link1.name;

                    string l2 = this.Joints[jointOrderInMatrix[i]].Link2 == null ? "null" : this.Joints[jointOrderInMatrix[i]].Link2.name;
                    System.Diagnostics.Debug.WriteLine("Reaction Force X at ({0}, {1}) \t[{2}, {3}]", jx, jy, l1, l2);
                    solutionKey.Add("Reaction Force X at (" + jx + ", " + jy + ") \t[" + l1 + ", " + l2 + "]");
                    System.Diagnostics.Debug.WriteLine("Reaction Force Y at ({0}, {1}) \t[{2}, {3}]", jx, jy, l1, l2);
                    solutionKey.Add("Reaction Force Y at (" + jx + ", " + jy + ") \t[" + l1 + ", " + l2 + "]");
                }

                solutionKey.Add("Torque Required at input.");

                int solutionStartIndex = this.Joints.All(j => j.TypeOfJoint == JointType.R) ? 6 : 9;
                MatrixGJ tempSol = new MatrixGJ(new MatrixGJ(coeffMatrixGJ.Inverse()).Multiply(knownMatrixGJ));
                currentJoints[0, solutionStartIndex + 2] = tempSol.Matrix[noOfUnknowns - 1, 0];
                for (int i = 0; i < jointOrderInMatrix.Count(); i++) //0, 1, 2, 3, 4, 5, 6
                {
                    int jpIndex = FindJPIndex(this.Joints[jointOrderInMatrix[i]].xInitial, this.Joints[jointOrderInMatrix[i]].yInitial);
                    for (int j = 0; j < 2; j++)
                    {
                        currentJoints[jpIndex, solutionStartIndex + j] = tempSol.Matrix[2 * i + j, 0];
                    }
                }

                output.Add("\nHere is the solution reaction forces and input torque:");

                for (int i = 0; i < unknownStrMatrix.Count(); i++)
                {
                    output.Add(unknownStrMatrix[i] + "\t" + solutionKey[i]);
                }
                output.Add("\nHere is the coefficient matrix:");
                output.AddRange(coeffStrMatrix);

                output.Add("\nHere is the known matrix:");
                output.AddRange(knownStrMatrix);
                #endregion

                if (justOne)
                {
                    break;
                }
            }
            char cr = (char)169;
            output.Add("\n\n" + cr.ToString() + "JCA " + System.DateTime.Now.ToString("yyyy"));
            int RorP = this.Joints.All(j => j.TypeOfJoint == JointType.R) ? 6 : 9;
            this.JointParameters.Parameters[0][1, RorP + 2] = maxTorque;
            return output;
        }



        private double[,] FindCoeffMatrix(int noOfUnknowns, int jointInstance)
        {
            var initialForces = this.ForceParameters.Parameters[0];
            double[,] coefficientMatrix = new double[noOfUnknowns, noOfUnknowns]; //here coeff
            List<int> force_Link = new List<int>();
            int thelink = 0;
            //here forces
            for (int i = 0; i < this.NumForces; i++)
            {
                int linknumber = Determine_Link_With_Force(initialForces[i, 0], initialForces[i, 1]);
                if (linknumber != 999)
                {
                    force_Link.Add(linknumber);
                }
                thelink = linknumber;
            }

            List<int> jointOrderInMatrix = new List<int>();

            var currentJoints = this.JointParameters.Parameters[jointInstance];

            List<double> reactionForceX = new List<double>();
            List<double> reactionForceY = new List<double>();
            int noOfJoints = 0;
            List<int> jointIndices = new List<int>();
            int ii = 0;
            foreach (var n in this.Joints)
            {
                if (/*n.Link1 != null &&*/ n.Link2 != null)
                {
                    noOfJoints++;
                    jointIndices.Add(ii);
                }
                ii++;
            }
            ii = 0;
            for (int i = 0; i < jointIndices.Count; i++)
            {
                reactionForceX.Add(1);
                reactionForceY.Add(1);
            }

            #region sum of forces
            List<int> jointIndexIntoMatrix = new List<int>();
            int groundLinkIndex = 0;
            ii = 0;
            for (int i = 0; i < this.Links.Count; i++) //looking at every link
            {
                if (this.Links[i].IsGround == false) //eliminating ground link
                {
                    foreach (var n in this.Links[i].joints) //every link has a joint 
                    {
                        if (/*n.Link1 != null &&*/ n.Link2 != null) //making sure there are two links making a joint, else that's not a joint
                        {
                            int valN = FindJPIndex(n.xInitial, n.yInitial);
                            var joint1_x = currentJoints[valN, 0];
                            var joint1_y = currentJoints[valN, 1];
                            int whichIndex = FindJointIndexRelatedToThisJoint(joint1_x, joint1_y, jointIndices, jointInstance);
                            if (whichIndex != 999)
                            {
                                if (!jointIndexIntoMatrix.Contains(whichIndex))
                                {
                                    jointIndexIntoMatrix.Add(whichIndex);

                                    coefficientMatrix[ii, whichIndex * 2] = reactionForceX[whichIndex];
                                    coefficientMatrix[ii + 1, whichIndex * 2 + 1] = reactionForceY[whichIndex];
                                }
                                else
                                {
                                    ii++;
                                    coefficientMatrix[ii, whichIndex * 2] = reactionForceX[whichIndex] * -1;
                                    coefficientMatrix[ii + 1, whichIndex * 2 + 1] = reactionForceY[whichIndex] * -1;
                                }
                            }
                        }
                    }
                    ii++;
                }
                else
                {
                    groundLinkIndex = i;
                }
            }
            #endregion

            #region adding input torque into the coefficient matrix
            var input_Link_Index = 0;
            for (int li = 0; li < this.NumLinks; li++)
            {
                if (this.Links[li].name.Equals("input"))
                {
                    input_Link_Index = li - 1;
                    break;
                }
            }
            List<int> createLinkList = new List<int>();
            for (int i = 0; i < this.NumLinks; i++)
            {
                createLinkList.Add(i);
            }
            createLinkList.Remove(groundLinkIndex);
            int positionofInput = 0;
            for (int i = 0; i < createLinkList.Count; i++)
            {
                if (createLinkList[i] == input_Link_Index)
                {
                    positionofInput = i;
                }
            }
            var startingMomentIndex = (this.NumLinks - 1) * 2;
            coefficientMatrix[startingMomentIndex + positionofInput, noOfUnknowns - 1] = 1.0;
            #endregion

            #region sum of moments
            var length1 = 0.0;
            for (int i = 0; i < this.NumLinks; i++)
            {
                List<double> rVectorX = new List<double>();
                List<double> rVectorY = new List<double>();
                List<double> rVectorForceX = new List<double>();
                List<double> rVectorForceY = new List<double>();
                List<int> jointIndicesMoment = new List<int>();
                if (this.Links[i].IsGround == false)
                {
                    for (int j = 0; j <= this.Links[i].joints.Count - 1; j++)
                    {

                        for (int k = j + 1; k < this.Links[i].joints.Count; k++)
                        {
                            int valK = FindJPIndex(this.Links[i].joints[k].xInitial, this.Links[i].joints[k].yInitial);
                            int valJ = FindJPIndex(this.Links[i].joints[j].xInitial, this.Links[i].joints[j].yInitial);

                            var momentIndex = FindJointIndexRelatedToThisJoint(currentJoints[valK, 0], currentJoints[valK, 1], jointIndices, jointInstance);//this is assuming that the third point in the triangle/polygon link is never declared first.

                            if (momentIndex != -1)
                            {
                                length1 = FindVectorLength(currentJoints[valJ, 0], currentJoints[valK, 0], currentJoints[valJ, 1], currentJoints[valK, 1]);
                                rVectorX.Add(GetVectorIntercept(currentJoints[valK, 0], currentJoints[valJ, 0]));
                                rVectorY.Add(GetVectorIntercept(currentJoints[valK, 1], currentJoints[valJ, 1]));

                                jointIndicesMoment.Add(momentIndex);
                            }
                        }
                        break;
                    }
                    for (int pp = 0; pp < rVectorX.Count; pp++)
                    {
                        positionofInput = 0; //positionofinput is just using the previously defined variable
                        for (int iii = 0; iii < createLinkList.Count; iii++)
                        {
                            if (createLinkList[iii] == i)
                            {
                                positionofInput = iii;
                            }
                        }
                        startingMomentIndex = (this.NumLinks - 1) * 2;
                        coefficientMatrix[startingMomentIndex + positionofInput, jointIndicesMoment[pp] * 2] = -rVectorY[pp];
                        coefficientMatrix[startingMomentIndex + positionofInput, jointIndicesMoment[pp] * 2 + 1] = rVectorX[pp];
                    }
                }

            }
            #endregion

            //round to 5 sig figs
            for (int i = 0; i < noOfUnknowns; i++)
            {
                for (int j = 0; j < noOfUnknowns; j++)
                {
                    coefficientMatrix[i, j] = RoundToSignificantDigits(coefficientMatrix[i, j], 5);
                }
            }
            return coefficientMatrix;
        }

        public double RoundToSignificantDigits(double d, int digits)
        {
            if (d == 0)
                return 0;
            if (Math.Abs(d) < 0.00001)
                return 0;
            double scale = Math.Pow(10, Math.Floor(Math.Log10(Math.Abs(d))) + 1);
            return scale * Math.Round(d / scale, digits);
        }

        //only to be run once in the intialization of Static or Dynamic Analysis
        private void ApplyGravity(ref List<double> appliedForceX, ref List<double> appliedForceY, ref List<double> appliedForceValueX, ref List<double> appliedForceValueY, ref List<bool> fixedForces)
        {
            for (int i = 0; i < this.Links.Count(); i++)
            {
                if (!this.Links[i].IsGround)
                {
                    appliedForceX.Add(this.LinkParameters.Parameters[0][i, 3]);
                    appliedForceY.Add(this.LinkParameters.Parameters[0][i, 4]);
                    appliedForceValueX.Add(0.0);
                    appliedForceValueY.Add(-1 * this.Links[i].Mass * 386.09);
                    fixedForces.Add(false);
                }
            }
        }



        public void AssignBasicLayers()
        {
            for (int i = 0; i < this.Links.Count(); i++)
            {
                this.Links[i].Layer = -1;
            }
            bool[] assigned = new bool[this.Links.Count()];
            for (int i = 0; i < assigned.Count(); i++)
            {
                assigned[i] = false;
            }
            int groundIndex = -1;
            for (int i = 0; i < this.Links.Count() + 1; i++)
            {
                if (i < this.Links.Count())
                {
                    if (this.Links[i].IsGround)
                    {
                        groundIndex = i; //assume only one ground
                        continue;
                    }
                }
                bool allDoneExceptGround = true;
                for (int j = 0; j < assigned.Count(); j++)
                {
                    if (j != groundIndex)
                    {
                        allDoneExceptGround = allDoneExceptGround && assigned[j];
                    }
                }
                if (allDoneExceptGround)
                {
                    i = 0;
                }
                List<Link> neighbors = new List<Link>();
                for (int j = 0; j < this.Links[i].joints.Count(); j++)
                {
                    if (!this.Links[i].joints[j].Link1.Equals(this.Links[i]))
                    {
                        neighbors.Add(this.Links[i].joints[j].Link1);
                    }
                    if (this.Links[i].joints[j].Link2 != null)
                    {
                        if (!this.Links[i].joints[j].Link2.Equals(this.Links[i]))
                        {
                            neighbors.Add(this.Links[i].joints[j].Link2);
                        }
                    }
                }
                List<Link> setNeighbors = new List<Link>();
                for (int j = 0; j < neighbors.Count(); j++)
                {
                    if (!assigned[FindLinkIndex(neighbors[j])])
                    {
                        continue;
                    }
                    if (!setNeighbors.Contains(neighbors[j]))
                    {
                        setNeighbors.Add(neighbors[j]);
                    }
                }
                for (int layer = 0; layer < this.Links.Count(); layer++)
                {
                    if (setNeighbors.Count() == 0)
                    {
                        this.Links[i].Layer = 0;
                        assigned[i] = true;
                        break;
                    }
                    bool originalLayer = true;
                    for (int j = 0; j < setNeighbors.Count(); j++)
                    {
                        originalLayer = originalLayer && (layer != setNeighbors[j].Layer);
                    }
                    if (originalLayer)
                    {
                        this.Links[i].Layer = layer;
                        assigned[i] = true;
                        break;
                    }
                }
                if (allDoneExceptGround)
                { //meaning that now everything but ground -- AND -- ground are all set
                    break;
                }
            }
        }

        public void CollisionFreeLayers()
        {
            int maxLayer = 0;
            for (int i = 0; i < this.Links.Count(); i++)
            {
                if (maxLayer > this.Links[i].Layer)
                {
                    maxLayer = this.Links[i].Layer;
                }
            }
            //int[][,] linksPerLayer = new int[maxLayer][1, this.Links.Count()];
            for (int i = 0; i < maxLayer; i++)
            {
                List<int> linksInLayer = new List<int>();
                for (int j = 0; j < this.Links.Count(); j++)
                {
                    if (this.Links[j].Layer == i)
                    {
                        linksInLayer.Add(j);
                    }
                }
            }
        }

        public List<string> StressAnalysis(int units, bool isStatic) //this function should only store data and check if all conditions are passed
        {
            List<string> output = new List<string>();


            output.Add("Stress Analysis");
            output.Add("");
            output.Add("The mechanism to be analyzed is as follows:");

            output.Add("Joint:\tLinks attached:");

            for (int i = 0; i < this.Joints.Count(); i++)
            {
                string jointLine = "";
                jointLine += ("(" + this.Joints[i].xInitial.ToString() + ", " + this.Joints[i].yInitial.ToString() + ")");
                for (int j = 0; j < 20 - jointLine.Length; j++)
                {
                    jointLine += " ";
                }
                jointLine += "\t" + this.Joints[i].Link1.name + ", " + this.Joints[i].Link2.name;
                output.Add(jointLine);
            }

            int maxLayer = 0;
            for(int i = 0; i < this.Links.Count; i++) {
                if (this.Links[i].Layer > maxLayer) {
                    maxLayer = this.Links[i].Layer;
                }
            }


            //StaticAnalysis(); //replace with Analysis(bool)

            if (isStatic)
            {
                output.Add("\nStatic Stress");
            }
            else
            {
                output.Add("\nDynamic Stress");
            }
            List<double> magInfo = MaxReactionMagnitude(false, -1);

            output.Add("Max Force (Magnitude):\t\t" + magInfo[0]);
            output.Add("Crank Angle at this time:\t\t" + ((magInfo[1] * this.DeltaAngle + this.Links[1].AngleInitial) * 180.0 / Math.PI) % 360);
            output.Add("");
            /*
            int itFails = LinkFails(magInfo[0], maxLayer, units);
            output.Add("FAIL:\t" + (itFails < 0 ? "YES" : "NO"));
            if (itFails < 0)
            {
                List<string> options = new List<string> { "pin bending", "pin shear", "link tearout" };
                string reason = "Linkage fails beacuse of possible " + options[Math.Abs(itFails) - 1] + " when the crank is at " + ((this.DeltaAngle * magInfo[1] + this.Links[1].AngleInitial)* 180.0 / Math.PI).ToString() + " degrees.";
                output.Add(reason);
            }
            */

            StoreLinkTensions();
            StoreLinkStresses();

            output.Add("CrankAngle\tName\tTension\tStress");
            for (int i = 0; i < this.LinkParameters.Parameters.Count; i++)
            {
                output.Add((((i * this.DeltaAngle + this.Links[1].AngleInitial) * 180.0 / Math.PI) % 360).ToString());
                for (int j = 0; j < this.Links.Count; j++)
                {
                    if (this.Links[j].IsGround || this.Links[j].joints.Count > 2)
                    {
                        continue;
                    }
                    output.Add("\t" + this.Links[j].name + "\t" + this.LinkParameters.Parameters[i][j, 7] + "\t" + this.LinkParameters.Parameters[i][j, 8]);
                }
                output.Add("");
            }

            return output;
        }

        public List<double> MaxReactionMagnitude(bool isKnown, int jointParam)
        {
            List<double> output = new List<double>();
            double maxMag = 0.0;
            double jP = 0.0;
            for (int i = 0; i < this.JointParameters.Parameters.Count(); i++)
            {
                if (isKnown)
                {
                    i = jointParam;
                }
                var currentJoints = this.JointParameters.Parameters[i];
                for (int j = 0; j < this.Joints.Count(); j++)
                {
                    int val = this.FindJPIndex(this.Joints[j].xInitial, this.Joints[j].yInitial);
                    double thisMag = Math.Sqrt(Math.Pow(currentJoints[j, 6], 2) + Math.Pow(currentJoints[j, 7], 2));
                    if (thisMag > maxMag)
                    {
                        maxMag = thisMag;
                        jP = (double)i;
                    }
                }
                if (isKnown)
                {
                    break;
                }
            }
            output.Add(maxMag);
            output.Add(jP);
            return output;
        }

        //Abandoning this function, I think.
        public int LinkFails(double maxMag, int maxLayer, int units)
        {
            //check if it bends
            double numerator = maxMag * (units == 0 ? 1.0 : 4.44822) * maxLayer * this.depth * 2.0 * 64.0; //2x is to consider collar thickness
            double denominator = ((this.PinYieldStrength * 1000000.0) / 2.0) * Math.PI; 
            double cube = numerator / denominator;
            double pinDiam1 = Math.Pow(cube, 1.0 / 3.0); 

            //check if it shears
            numerator = 4.0 * maxMag * (units == 0 ? 1.0 : 4.44822);
            denominator = this.PinYieldStrength * 1000000.0 * 3.0 * Math.PI; 
            double square = numerator / denominator;
            double pinDiam2 = Math.Sqrt(square) * 2.0;

            //check if it tears out
            numerator = maxMag * (units == 0 ? 1.0 : 4.44822); 
            denominator = this.pinDiam * this.LinkYieldStrength * 1000000.0;
            double linkDepth = numerator / denominator;

            if (pinDiam1 > this.pinDiam && pinDiam2 > this.pinDiam && linkDepth > this.depth)
            {
                return -3;
            }
            return 1;
        }

        /// <summary>
        /// Sets the Link dimensions based off of the maximum stress in a system
        /// </summary>
        /// <param name="maxMag">Maximum reaction force magnitude (lbf) through the entire motion of the mechanism.</param>
        /// <param name="maxLayerDist">Max number of layers *between* two links connected by a pin.</param>
        /// <param name="yield">Yield strength of the pin material in psi.</param>
        public bool LinkDimensionsViaStress(double maxMag, int maxLayerDist, double yield, double linkYield, int units, bool shouldSet)
        {
            bool theOutcome = false; //false if the given params DONT fail
        // width, depth, pin diameter
        // set the pin diameter through this function
        // width is 3x pin diameter
        // depth is hard coded to 0.25 inches because it is the closest available stock size


            // BENDING pin minumum diameter (with factor of safety of 2)
        /**
         *   |-----------------------------------------------------------
         *   |     Max Applied * Layers * Depth (inches) * 2 * 64 * 2       
         *   |   ------------------------------------------------------    =  Pin Diameter
         * |3|                  (Yield Strength / 2) * PI
         *  V
         */
        TryAgain:
            double numerator = maxMag * maxLayerDist * this.Links[1].Depth * 2.0 * 64.0 * 2.0; //first 2x is to consider collar thickness
            double denominator = (yield / 2.0) * Math.PI;
            double cube = numerator / denominator;
            double pinDiam1 = Math.Pow(cube, 1.0 / 3.0);
            double roundedDiam1 = RoundToNearestQuarter(pinDiam1);

            // SHEAR pin minium diameter (with FOS of 2)
            numerator = 2.0 * 4.0 * maxMag;
            denominator = yield * 3.0 * Math.PI;
            double square = numerator / denominator;
            double pinDiam2 = Math.Sqrt(square) * 2.0;
            double roundedDiam2 = RoundToNearestQuarter(pinDiam2);

            if (roundedDiam1 > roundedDiam2)
            {
                for (int i = 0; i < this.Links.Count(); i++)
                {
                    this.Links[i].PinDiameter = roundedDiam1;
                    this.Links[i].Width = roundedDiam1 * 3.0;
                }
                this.width = roundedDiam1 * 3.0;
                this.pinDiam = roundedDiam1;

            }
            else
            {
                for (int i = 0; i < this.Links.Count(); i++)
                {
                    this.Links[i].PinDiameter = roundedDiam2;
                    this.Links[i].Width = roundedDiam2 * 3.0;
                }
                this.width = roundedDiam2 * 3.0;
                this.pinDiam = roundedDiam2;
            }

            numerator = maxMag * 2.0; //this 2 is a FOS
            denominator = this.Links[1].PinDiameter * linkYield;
            double linkDepth = numerator / denominator;
            linkDepth = RoundToNearestQuarter(linkDepth);
            if (linkDepth > this.Links[1].Depth)
            {
                for (int i = 0; i < this.Links.Count(); i++)
                {
                    this.Links[i].Depth = linkDepth;
                }
                this.depth = linkDepth;
                theOutcome = true; //fail, try again
                goto TryAgain;
            }
            return theOutcome;
        }

        //ASSUMING BINARY LINKS ONLY!!!
        public void StoreLinkTensions()
        {
            for (int i = 0; i < this.LinkParameters.Parameters.Count(); i++)
            {
                if (i >= this.JointParameters.Parameters.Count())
                {
                    continue;
                }
                var currentJoints = this.JointParameters.Parameters[i];
                var currentLinks = this.LinkParameters.Parameters[i];
                for (int j = 0; j < this.Links.Count(); j++)
                {

                    if (this.Links[j].IsGround)
                    {
                        continue;
                    }
                    if (this.Links[j].joints.Count > 2)
                    {
                        continue;
                    }

                    double tension = 0.0;
                    //this is hardcoded for binary links as they only have 2 joints
                    int joint1 = FindJPIndex(this.Links[j].joints[0].xInitial, this.Links[j].joints[0].yInitial);
                    int joint2 = FindJPIndex(this.Links[j].joints[1].xInitial, this.Links[j].joints[1].yInitial);

                    //find the angle of the link
                    //find the angle of the force
                    //find the magnitude of the force
                    double angDiff, mag;
                    if (!this.Links[j].joints[1].IsGround)
                    { //floating (coupler) case -or- first joint is grounded
                        angDiff = Math.Atan2(currentJoints[joint2, 7], currentJoints[joint2, 6]) - currentLinks[j, 0];
                        mag = Math.Sqrt(Math.Pow(currentJoints[joint2, 6], 2) + Math.Pow(currentJoints[joint2, 7], 2));

                        tension += (mag * Math.Cos(angDiff)); // this is the non-reference joint (free)

                        angDiff = Math.Atan2(currentJoints[joint1, 7], currentJoints[joint1, 6]) - currentLinks[j, 0];
                        mag = Math.Sqrt(Math.Pow(currentJoints[joint1, 6], 2) + Math.Pow(currentJoints[joint1, 7], 2));

                        tension -= (mag * Math.Cos(angDiff)); // this is the reference joint (ref)
                    }
                    else
                    {
                        angDiff = Math.Atan2(currentJoints[joint2, 7], currentJoints[joint2, 6]) - currentLinks[j, 0];
                        mag = Math.Sqrt(Math.Pow(currentJoints[joint2, 6], 2) + Math.Pow(currentJoints[joint2, 7], 2));

                        tension -= (mag * Math.Cos(angDiff)); // this is the reference joint (ref)

                        angDiff = Math.Atan2(currentJoints[joint1, 7], currentJoints[joint1, 6]) - currentLinks[j, 0];
                        mag = Math.Sqrt(Math.Pow(currentJoints[joint1, 6], 2) + Math.Pow(currentJoints[joint1, 7], 2));

                        tension += (mag * Math.Cos(angDiff)); // this is the non-reference joint (free) 
                    }

                    currentLinks[j, 7] = RoundToSignificantDigits(tension, 5);

                }
            }
        }

        public void StoreLinkStresses()
        {
            double mult = (this.uniIndex == 0 ? 100.0 : 39.3701);
            double crossArea = this.width * this.depth * mult * mult;
            for (int i = 0; i < this.LinkParameters.Parameters.Count(); i++)
            {
                var currentLinks = this.LinkParameters.Parameters[i];
                for (int j = 0; j < this.Links.Count(); j++)
                {
                    currentLinks[j, 8] = RoundToSignificantDigits(currentLinks[j, 7] / crossArea, 5);
                }
            }
        }


        public double InchToMeter(double inches)
        {
            return inches / 39.37;
        }

        public double RoundToNearestQuarter(double val)
        {
            return Math.Abs(((double)((int)(Math.Ceiling(4.0 * val)))) / 4.0);
        }

        public void VolumeCalculation()
        {
            int units = this.uniIndex;
            double multFactor = (units == 0 ? 100.0 : 39.3701);
            //double depth = 1;//cm 
            //double width = 1;//cm //y component for binary links 
            //double circlediam = .635;//cm
            for (int i = 0; i < this.NumLinks; i++)
            {
                //for ground links and link with only 2 joints
                if (this.Links[i].IsGround == true || this.Links[i].joints.Count < 3)
                {
                    if (this.Links[i].IsGround)
                    {
                        bool sliderDetected = false;
                        for (int j = 0; j < this.Links[i].joints.Count(); j++)
                        {
                            if (this.Links[i].joints[j].TypeOfJoint == JointType.P)
                            {
                                sliderDetected = true;
                                break;
                            }
                        }
                        if (sliderDetected)
                        {
                            continue; //Jon change this to actually figure out some info about the slider ground link (instead of ignoring it)   *** UPDATE: April 10 2018 -- just ignore it
                        }
                    }
                    //double x1 = this.Links[i].joints[0].xInitial;
                    //double y1 = this.Links[i].joints[0].yInitial;
                    //double x2 = this.Links[i].joints[1].xInitial;
                    //double y2 = this.Links[i].joints[1].yInitial;
                    //double distx = x2 - x1;
                    //double disty = y2 - y1;
                    //double dist = Math.Sqrt(Math.Pow(distx, 2) + Math.Pow(disty, 2));
                    double dist = this.Links[i].MaxLength;
                    
                    double volcalc = (dist * this.depth * this.width * multFactor * multFactor) + (Math.PI * this.depth * multFactor * (Math.Pow((this.width * multFactor) / 2.0, 2))) - 2.0 * (Math.PI * this.depth * multFactor * (Math.Pow(((this.pinDiam * multFactor) / 2.0), 2))); // subtraction for two holes because binary link
                    this.Links[i].Volume = volcalc;
                    this.Links[i].Depth = this.depth;
                    this.Links[i].Width = this.width;
                    this.Links[i].PinDiameter = this.pinDiam;
                }
                //for ternary links
                if (this.Links[i].joints.Count == 3 && this.Links[i].IsGround == false)
                {
                    double a = 0;
                    double c = 0;
                    double b = this.Links[i].MaxLength;
                    foreach (KeyValuePair<int, double> kvp in this.Links[i].lengths)
                    {
                        if (!(kvp.Value == b))
                        {
                            if (kvp.Value > a)
                            {
                                c = a;
                                a = kvp.Value;
                            }
                            else
                            {
                                c = kvp.Value;
                            }
                        }
                    }
                    double p = ((a + b + c) / 2);
                    double triArea = Math.Sqrt((p * (p - a) * (p - b) * (p - c)));
                    double area = triArea + (a + b + c) * ((this.width * multFactor)/ 2.0) + (Math.PI * Math.Pow((this.width * multFactor) / 2.0, 2)) - 3.0 * (Math.PI * this.depth * multFactor * (Math.Pow(((this.pinDiam * multFactor) / 2.0), 2)));
                    double volcalc = this.depth * multFactor * area;
                    this.Links[i].Volume = volcalc;
                    //double xa1 = this.Links[i].joints[0].xInitial;
                    //double ya1 = this.Links[i].joints[0].yInitial;
                    //double xa2 = this.Links[i].joints[1].xInitial;
                    //double ya2 = this.Links[i].joints[1].yInitial;
                    //double distxa = xa2 - xa1;
                    //double distya = ya2 - ya1;
                    //double dista = Math.Sqrt(Math.Pow(distxa, 2) + Math.Pow(distya, 2));
                    //double xb1 = this.Links[i].joints[0].xInitial;
                    //double yb1 = this.Links[i].joints[0].yInitial;
                    //double xb2 = this.Links[i].joints[2].xInitial;
                    //double yb2 = this.Links[i].joints[2].yInitial;
                    //double distxb = xb2 - xb1;
                    //double distyb = yb2 - yb1;
                    //double distb = Math.Sqrt(Math.Pow(distxb, 2) + Math.Pow(distyb, 2));
                    //double xc1 = this.Links[i].joints[1].xInitial;
                    //double yc1 = this.Links[i].joints[1].yInitial;
                    //double xc2 = this.Links[i].joints[2].xInitial;
                    //double yc2 = this.Links[i].joints[2].yInitial;
                    //double distxc = xc2 - xc1;
                    //double distyc = yc2 - yc1;
                    //double distc = Math.Sqrt(Math.Pow(distxc, 2) + Math.Pow(distyc, 2));
                    //double volcalc = .25 * depth * (Math.Sqrt(Math.Pow(-1 * dista, 4) + 2 * Math.Pow(dista * distb, 2) + 2 * Math.Pow(dista * distc, 2) - Math.Pow(distb, 4) + Math.Pow(distb * distc, 2) - Math.Pow(distc, 4))) + (Math.PI * depth * (Math.Pow(circlediam, 2)))
                }
                //for links with greater links WIP
                /* WIP if (pms.Links[i].joints.Count>3 && pms.Links[i].IsGround == false)
                  {
                      for (int j = 0; j <= pms.Links[i].joints.Count;j++)
                      {
                          double x = pms.Links[i].joints[j].xInitial;
                          double y = pms.Links[i].joints[j].yInitial;
                          greaterlinksx.Add(x);
                          greaterlinksy.Add(y);
                      }
                      for (int j = 0; j <= greaterlinksx.Count;j++)
                      {
                          for (int k = 0; k <= greaterlinksx.Count;k++)
                          {
                              double distx = greaterlinksx[j] - greaterlinksx[k];
                              double disty = greaterlinksy[j] - greaterlinksy[k];
                              distancesx.Add(distx);
                              distancesy.Add(disty);
                              for (int first = 1; first <= 2; first++)
                              {
                                  double val1x = distancesx.Min();
                              }
                          }
                      }
                  } WIP */
            }
        }


        public void MasslinkCalculation()
        {
            //double density = 1;//g/cm3
            int units = this.uniIndex;
            double multFactor = (units == 0 ? 0.001 : 0.453592); //volume is in cc or in^3 ; density * volume will return grams or lbs ; need to return kg

            List<double> linkmass = new List<double>();
            for (int i = 0; i < this.NumLinks; i++)
            {
                double vollink = this.Links[i].Volume;
                double mass = vollink * this.LinkDensity;
                this.Links[i].Mass = mass * multFactor; //now mass is stored in kg
            }
        }

        // Center of mass in relation to link
        public void CenterofMass()
        {
            //double width = 2;//cm //y component for binary links

            for (int i = 0; i < this.NumLinks; i++)
            {
                int ii = i + 1;

                // first to measure the CoM in x direction
                // if it is binary link, it is in the middle
                // if it is triangle, use centroid formula
                if (this.Links[i].IsGround == true || this.Links[i].joints.Count < 3)
                {
                    double x1 = this.JointParameters.Parameters[0][i, 0];
                    double y1 = this.JointParameters.Parameters[0][i, 1];
                    double x2 = this.JointParameters.Parameters[0][ii, 0];
                    double y2 = this.JointParameters.Parameters[0][ii, 1];
                    double distx = x2 - x1;
                    double disty = y2 - y1;
                    double length = Math.Sqrt(Math.Pow(distx, 2) + Math.Pow(disty, 2));
                    double xloc = length / 2;
                    double yloc = width / 2;
                    this.Links[i].CoMx = xloc;
                    this.Links[i].CoMy = yloc;
                }

                //for ternary links
                int iii = ii + 1;
                if (this.Links[i].joints.Count == 3 && this.Links[i].IsGround == false)
                {
                    double x1 = this.JointParameters.Parameters[0][i, 0];
                    double y1 = this.JointParameters.Parameters[0][i, 1];
                    double x2 = this.JointParameters.Parameters[0][ii, 0];
                    double y2 = this.JointParameters.Parameters[0][ii, 1];
                    double x3 = this.JointParameters.Parameters[0][iii - 3, 0];
                    double y3 = this.JointParameters.Parameters[0][iii - 3, 1];//coded fix
                    double centroidx = (x1 + x2 + x3) / 3;
                    double centroidy = (y1 + y2 + y3) / 3;
                    this.Links[i].CoMx = centroidx;
                    this.Links[i].CoMy = centroidy;

                }

            }
        }


        public int Determine_Link_With_Force(double xLoc, double yLoc)
        {
            //now need to determine the link on which this force exists

            bool onlyBinary = true;
            foreach (var n in this.Links)
                if (n.joints.Count > 2)
                    onlyBinary = false;

            //var xx = this.JointParameters.Parameters[0];

            #region determining the link where the force is acting if the force is on the edge

            List<double> linkVector_X = new List<double>();
            List<double> linkVector_Y = new List<double>();
            List<double> unitVector_Y = new List<double>();
            List<double> unitVector_X = new List<double>();
            List<double> unitVector_Force_X = new List<double>();
            List<double> unitVector_Force_Y = new List<double>();
            List<bool> length1SumLength2EqualsLength = new List<bool>();
            List<bool> length1SumLength2EqualsLength_Reduced = new List<bool>();
            List<int> linkIndex_Force = new List<int>();
            List<int> linkIndex = new List<int>();
            List<bool> edgeorNot = new List<bool>();
            var length = 0.0;
            for (int i = 0; i < this.NumLinks; i++)
            {
                if (this.Links[i].IsGround == false)
                {//assuming force is never at the ground link 

                    for (int j = 0; j <= this.Links[i].joints.Count - 1; j++)
                    {
                        for (int k = j + 1; k < this.Links[i].joints.Count; k++)
                        {

                            length = FindVectorLength(this.Links[i].joints[j].xInitial, this.Links[i].joints[k].xInitial, this.Links[i].joints[j].yInitial, this.Links[i].joints[k].yInitial);

                            linkVector_X.Add(GetVectorIntercept(this.Links[i].joints[j].xInitial, this.Links[i].joints[k].xInitial));
                            linkVector_Y.Add(GetVectorIntercept(this.Links[i].joints[j].yInitial, this.Links[i].joints[k].yInitial));
                            unitVector_X.Add(GetVectorIntercept(this.Links[i].joints[j].xInitial, this.Links[i].joints[k].xInitial) / length);
                            unitVector_Y.Add(GetVectorIntercept(this.Links[i].joints[j].yInitial, this.Links[i].joints[k].yInitial) / length);
                            linkIndex.Add(i);
                            var length1 = FindVectorLength(this.Links[i].joints[j].xInitial, xLoc, this.Links[i].joints[j].yInitial, yLoc);
                            var length2 = FindVectorLength(xLoc, this.Links[i].joints[k].xInitial, yLoc, this.Links[i].joints[k].yInitial);

                            if (length1 + length2 >= length - 0.000001 && length1 + length2 <= length + 0.000001)
                                length1SumLength2EqualsLength.Add(true);
                            else
                                length1SumLength2EqualsLength.Add(false);
                        }

                        length = FindVectorLength(this.Links[i].joints[j].xInitial, xLoc, this.Links[i].joints[j].yInitial, yLoc);
                        unitVector_Force_X.Add(GetVectorIntercept(this.Links[i].joints[j].xInitial, xLoc) / length);
                        unitVector_Force_Y.Add(GetVectorIntercept(this.Links[i].joints[j].yInitial, yLoc) / length);
                        linkIndex_Force.Add(i);
                    }

                }

            }
            //use cross product 

            List<int> comparative_indices = new List<int>();

            //for each link, compare the corresponding force-unit vector and unit vector. 

            for (int i = 0; i < linkIndex.Count; i++)
            {
                for (int j = 0; j < linkIndex_Force.Count; j++)
                {
                    if (linkIndex[i] == linkIndex_Force[j])
                        if (Math.Abs(Math.Asin(unitVector_X[i] * unitVector_Force_Y[j] - unitVector_Force_X[j] * unitVector_Y[i])) <= 0.00001)
                        {
                            comparative_indices.Add(linkIndex[i]);
                            length1SumLength2EqualsLength_Reduced.Add(length1SumLength2EqualsLength[i]);
                        }
                }

            }

            //look at Length1Sum... and find which has true and locate the corresponding link from comparative indices 

            for (int i = 0; i < length1SumLength2EqualsLength_Reduced.Count; i++)
            {

                if (length1SumLength2EqualsLength_Reduced[i])
                {
                    edgeorNot.Add(true);
                    return comparative_indices[i];
                }

            }

            //in case the force is not at the edge but at the center of a triangle or ternary link

            //https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon 

            for (int i = 0; i < this.NumLinks; i++)
            {
                if (this.Links[i].IsGround == false)
                    if (this.Links[i].joints.Count > 2)
                    {
                        bool inOrout = PointInTriangle(this.Links[i].joints, xLoc, yLoc);

                        if (inOrout)
                        {
                            edgeorNot.Add(false);
                            return i;
                        }
                    }

            }
            #endregion
            return 999;
        }

        private double FindVectorLength(double p1, double p2, double p3, double p4)
        {
            return Math.Sqrt(Math.Pow(p1 - p2, 2) + Math.Pow(p3 - p4, 2));
        }

        private static double GetVectorIntercept(double p1, double p2)
        {
            return (p1 - p2);
        }

        private bool PointInTriangle(List<PMKS.Joint> list, double p1, double p2)
        {

            List<bool> b = new List<bool>();
            double minX = list[0].xInitial;
            double maxX = list[0].xInitial;
            double minY = list[0].yInitial;
            double maxY = list[0].yInitial;

            for (int i = 1; i < list.Count; i++)
            {

                double q1 = list[i].xInitial;
                double q2 = list[i].yInitial;

                minX = Math.Min(q1, minX);
                maxX = Math.Max(q1, maxX);
                minY = Math.Min(q2, minY);
                maxY = Math.Max(q2, maxY);
            }


            if (p1 < minX || p1 > maxX || p2 < minY || p2 > maxY)
            {
                return false;
            }

            bool inside = false;
            for (int i = 0, j = list.Count - 1; i < list.Count; j = i++)
            {
                if ((list[i].yInitial > p2) != (list[j].yInitial > p2) &&
                     p1 < (list[j].xInitial - list[i].xInitial) * (p2 - list[i].yInitial) / (list[j].yInitial - list[i].yInitial) + list[i].xInitial)
                {
                    inside = !inside;
                }
            }

            return inside;

        }

        private int FindCircleCircleIntersections(
               double cx0, double cy0, double radius0,
               double cx1, double cy1, double radius1, out double ix1, out double iy1, out double ix2, out double iy2)
        {
            // Find the distance between the centers.
            double dx = cx0 - cx1;
            double dy = cy0 - cy1;
            double dist = Math.Sqrt(dx * dx + dy * dy);

            // See how many solutions there are.
            if (dist > radius0 + radius1)
            {
                // No solutions, the circles are too far apart.
                var intersection1 = double.NaN;
                var intersection2 = double.NaN;
                ix1 = double.NaN;
                iy1 = double.NaN;
                ix2 = double.NaN;
                iy2 = double.NaN;
                return 0;

            }
            else if (dist < Math.Abs(radius0 - radius1))
            {
                // No solutions, one circle contains the other.
                var intersection1 = double.NaN;
                var intersection2 = double.NaN;
                ix1 = double.NaN;
                iy1 = double.NaN;
                ix2 = double.NaN;
                iy2 = double.NaN;
                return 0;
            }
            else if ((dist == 0) && (radius0 == radius1))
            {
                // No solutions, the circles coincide.
                var intersection1 = double.NaN;
                var intersection2 = double.NaN;
                ix1 = double.NaN;
                iy1 = double.NaN;
                ix2 = double.NaN;
                iy2 = double.NaN;
                return 0;
            }
            else
            {
                // Find a and h.
                double a = (radius0 * radius0 -
                    radius1 * radius1 + dist * dist) / (2 * dist);
                double h = 0;
                if (Math.Abs(radius0 * radius0 - a * a) < 0.000001)
                    h = 0;
                else
                    h = Math.Sqrt(radius0 * radius0 - a * a);

                // Find P2.
                double cx2 = cx0 + a * (cx1 - cx0) / dist;
                double cy2 = cy0 + a * (cy1 - cy0) / dist;

                // Get the points P3.
                var intersection1 = (cx2 + h * (cy1 - cy0) / dist);
                var intersection2 = (cy2 - h * (cx1 - cx0) / dist);

                var intersection3 = (cx2 - h * (cy1 - cy0) / dist);
                var intersection4 = (cy2 + h * (cx1 - cx0) / dist);

                int solution = 0;

                // See if we have 1 or 2 solutions.
                if (dist == radius0 + radius1) { solution = 1; }
                else { solution = 2; }

                if (solution == 1)
                {
                    ix1 = intersection1;
                    iy1 = intersection2;
                    ix2 = intersection3;
                    iy2 = intersection4;
                    return 1;
                }

                else
                {
                    //if there are two solutions, how do we determine the right intersection point? 
                    ix1 = intersection1;
                    iy1 = intersection2;
                    ix2 = intersection3;
                    iy2 = intersection4;
                    return 2;
                }
            }
        }

        public int FindJPIndex(double xInit, double yInit)
        {
            /*
            System.Diagnostics.Debug.WriteLine("\t\tWithin FindJPIndex");
            for (int i = 0; i < this.JointParameters.Parameters.Count; i++)
            {
                System.Diagnostics.Debug.WriteLine("\t\t\t\tFFM Pos " + i.ToString());
                for (int j = 0; j < this.Joints.Count; j++)
                {
                    System.Diagnostics.Debug.WriteLine("\t\t\t\t\t(" + this.JointParameters.Parameters[i][j, 0].ToString() + ", " + this.JointParameters.Parameters[i][j, 1].ToString() + ")");
                }
            }
            */
            var initJoints = this.JointParameters.Parameters[0];

            /*
            for (int i = 0; i < this.Joints.Count; i++)
            {
                System.Diagnostics.Debug.WriteLine("\t\t\t(" + initJoints[i, 0].ToString() + ", " + initJoints[i, 1].ToString() + ")");
                System.Diagnostics.Debug.WriteLine("\t\t\t(" + this.JointParameters.Parameters[0][i, 0].ToString() + ", " + this.JointParameters.Parameters[0][i, 1].ToString() + ")");
            }
            */

            for (int i = 0; i < this.Joints.Count(); i++)
            {
                if (DoubleEquivalency(xInit, initJoints[i, 0]) && DoubleEquivalency(yInit, initJoints[i, 1]))
                {
                    return i;
                }
                /*
                if (xInit >= initJoints[i, 0] - 0.000001 && xInit <= initJoints[i, 0] + 0.000001 && 
                    yInit >= initJoints[i, 1] - 0.000001 && yInit <= initJoints[i, 1] + 0.000001)
                {
                    return i;
                }
                */
            }
            return -1;
        }

        public double SegmentMinDist(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
        {
            // (x1, y1), (x2, y2) [first, second]
            // (x3, y3), (x4, y4) [third, fourth]

            double m1 = FindSlope(x1, y1, x2, y2);
            double m2 = FindSlope(x3, y3, x4, y4);

            double b1original = FindYint(x1, y1, m1);
            double b2original = FindYint(x3, y3, m2);

            //if segments intersect, their min distance is zero
            //in order to intersect, the point of intersection must be within the x and y extremes of both segments
            double xi = Double.PositiveInfinity; //x intersection component
            double yi = Double.PositiveInfinity;

            if (!(Double.IsPositiveInfinity(m1) && Double.IsPositiveInfinity(m2)))
            {
                if (Double.IsPositiveInfinity(m1))
                {
                    xi = b1original;
                    yi = m2 * xi + b2original;
                }
                else if (Double.IsPositiveInfinity(m2))
                {
                    xi = b2original;
                    yi = m1 * xi + b1original;
                }
                else
                {
                    xi = (-1 * (b1original - b2original)) / (m1 - m2);
                    yi = m1 * xi + b1original;
                }
            }

            if (xi <= (x1 > x2 ? x1 : x2) && xi <= (x3 > x4 ? x3 : x4) && xi >= (x1 < x2 ? x1 : x2) && xi >= (x3 < x4 ? x3 : x4) &&
                yi <= (y1 > y2 ? y1 : y2) && yi <= (y3 > y4 ? y3 : y4) && yi >= (y1 < y2 ? y1 : y2) && yi >= (y3 < y4 ? y3 : y4))
            {
                return 0.0; //intersection confirmed, "distance" is zero.
            }
            //if segments are not within each others endpoint overlap range (debatable closest distance)
            bool nonTrivialDistance = false;
            List<bool> segsInRange = SegmentsInRange(x1, y1, x2, y2, x3, y3, x4, y4);
            for (int i = 0; i < segsInRange.Count(); i++)
            {
                nonTrivialDistance = nonTrivialDistance || segsInRange[i];
            }
            if (!nonTrivialDistance)
            {
                List<double> endpointDistances = new List<double>();
                endpointDistances.Add(Math.Sqrt(Math.Pow(x1 - x3, 2) + Math.Pow(y1 - y3, 2)));
                endpointDistances.Add(Math.Sqrt(Math.Pow(x1 - x4, 2) + Math.Pow(y1 - y4, 2)));
                endpointDistances.Add(Math.Sqrt(Math.Pow(x2 - x3, 2) + Math.Pow(y2 - y3, 2)));
                endpointDistances.Add(Math.Sqrt(Math.Pow(x2 - x4, 2) + Math.Pow(y2 - y4, 2)));

                double low = 1000000.0;
                for (int i = 0; i < endpointDistances.Count(); i++)
                {
                    if (endpointDistances[i] < low)
                    {
                        low = endpointDistances[i];
                    }
                }
                return low;
            }



            double m1inv = FindSlopeInverse(m1);
            double m2inv = FindSlopeInverse(m2);

            //need to create two y = mx + b functions for each line segment (one per vertex) for perpedicular rectangular limits
            double b1 = FindYint(x1, y1, m1inv);
            double b2 = FindYint(x2, y2, m1inv);
            double b3 = FindYint(x3, y3, m2inv);
            double b4 = FindYint(x4, y4, m2inv);

            List<double> dists = new List<double>();
            if (segsInRange[0])
            {
                dists.Add(PointLineDistance(x1, y1, m2, b2original));
            }
            if (segsInRange[1])
            {
                dists.Add(PointLineDistance(x2, y2, m2, b2original));
            }
            if (segsInRange[2])
            {
                dists.Add(PointLineDistance(x3, y3, m1, b1original));
            }
            if (segsInRange[3])
            {
                dists.Add(PointLineDistance(x4, y4, m1, b1original));
            }

            double lowest = 1000000.0;
            for (int i = 0; i < dists.Count(); i++)
            {
                if (dists[i] < lowest)
                {
                    lowest = dists[i];
                }
            }
            return lowest;
        }

        private double FindSlope(double x1, double y1, double x2, double y2)
        {
            if (DoubleEquivalency(x1, x2))
            {
                return Double.PositiveInfinity; //case where the x1 == x2 (vertical line segment)
            }
            else
            {
                return (y1 - y2) / (x1 - x2);
            }
        }

        private double PointLineDistance(double x, double y, double m, double b)
        {
            if (Double.IsPositiveInfinity(m))
            {
                return Math.Abs(x - b);
            }
            else if (DoubleEquivalency(m, 0.0))
            {
                return Math.Abs(y - b);
            }
            double slopeInv = FindSlopeInverse(m);
            double newB = y - slopeInv * x;
            double intX = (-1 * (b - newB)) / (m - slopeInv); //the x and y components of the intersection
            double intY = m * intX + b;

            return Math.Sqrt(Math.Pow(x - intX, 2) + Math.Pow(y - intY, 2));
        }

        // this function needs to determine if the endpoints of one segment are within the perpendicular rays
        // if they are not, then the shortest distance will be vertex to vertex
        private List<bool> SegmentsInRange(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
        {
            List<bool> output = new List<bool>();

            double m1 = FindSlope(x1, y1, x2, y2);
            double m2 = FindSlope(x3, y3, x4, y4);

            double m1inv = FindSlopeInverse(m1);
            double m2inv = FindSlopeInverse(m2);

            //need to create two y = mx + b functions for each line segment (one per vertex)
            double b1 = FindYint(x1, y1, m1inv);
            double b2 = FindYint(x2, y2, m1inv);
            double b3 = FindYint(x3, y3, m2inv);
            double b4 = FindYint(x4, y4, m2inv);

            //now determine if a point is between two lines
            //VERTEX NAME + In + SEGMENT NUMBER ['s range]
            output.Add(IsPointBetweenTwoLines(x1, y1, m2inv, b3, b4));
            output.Add(IsPointBetweenTwoLines(x2, y2, m2inv, b3, b4));
            output.Add(IsPointBetweenTwoLines(x3, y3, m1inv, b1, b2));
            output.Add(IsPointBetweenTwoLines(x4, y4, m1inv, b1, b2));

            return output;
        }

        /// <summary>
        /// Determines whether or not too doubles are equivalent to an accuracy of 0.000001
        /// </summary>
        /// <returns><c>true</c>, if two doubles are equivalent, <c>false</c> otherwise.</returns>
        /// <param name="n1">N1.</param>
        /// <param name="n2">N2.</param>
        private bool DoubleEquivalency(double n1, double n2)
        {
            return (n1 - n2 >= -0.000001 && n1 - n2 <= 0.000001);
        }

        private double FindSlopeInverse(double m)
        {
            if (Double.IsPositiveInfinity(m))
            {
                return 0.0;
            }
            else if (DoubleEquivalency(m, 0.0))
            {
                return Double.PositiveInfinity;
            }
            else
            {
                return -1.0 / m;
            }

        }

        private double FindYint(double x, double y, double m)
        {
            if (Double.IsPositiveInfinity(m))
            {
                return x; // this is not a true intercept (which would be NaN, but serves a purpose in other function
            }
            else
            {
                return (y - m * x);
            }
        }

        private bool IsPointBetweenTwoLines(double x, double y, double m, double b1, double b2)
        {
            if (Double.IsPositiveInfinity(m))
            {
                if (x <= (b1 > b2 ? b1 : b2) && x >= (b1 < b2 ? b1 : b2))
                {
                    return true;
                }
            }
            else if (DoubleEquivalency(m, 0.0))
            {
                if (y <= (b1 > b2 ? b1 : b2) && y >= (b1 < b2 ? b1 : b2))
                {
                    return true;
                }
            }
            else
            {
                double y1 = m * x + b1;
                double y2 = m * x + b2;
                if (y <= (y1 > y2 ? y1 : y2) && y >= (y1 < y2 ? y1 : y2))
                {
                    return true;
                }
            }
            return false;
        }

        private int FindJointIndexRelatedToThisJoint(double joint1_x, double joint1_y, List<int> jointIndices, int timeIndex)
        {

            int index = 0;
            var currentJoints = this.JointParameters.Parameters[timeIndex]; //This is added
            var initialJoints = this.JointParameters.Parameters[0];
            for (int j = 0; j < this.Joints.Count; j++)
            {
                if (joint1_x == currentJoints[j, 0] && joint1_y == currentJoints[j, 1])
                {
                    index = j;
                    break;
                }

                else
                {
                    index = 999;
                }
            }

            //must convert joint parameters index to normal this.joints index
            if (index < 999)
            {
                for (int j = 0; j < jointIndices.Count(); j++)
                {
                    if (this.Joints[jointIndices[j]].xInitial == initialJoints[index, 0] && this.Joints[jointIndices[j]].yInitial == initialJoints[index, 1])
                    {
                        index = j;
                        break;
                    }
                }
                return jointIndices.IndexOf(index);
            }
            else
                return index;
        }



        /// <summary>
        /// Copies the joints and links for backwards.
        /// </summary>
        /// <param name="AllJoints">All joints.</param>
        /// <param name="AllLinks">All links.</param>
        /// <param name="backwardJoints">The backward joints.</param>
        /// <param name="backwardLinks">The backward links.</param>
        private static void CopyJointsAndLinksForBackwards(List<Joint> AllJoints, List<Link> AllLinks, List<Force> AllForces,
            out List<Joint> backwardJoints, out List<Link> backwardLinks, out List<Force> backwardForces)
        {
            backwardJoints = AllJoints.Select(j => j.copy()).ToList();
            backwardLinks = AllLinks.Select(c => c.Copy()).ToList();
            backwardForces = AllForces.Select(f => f.Copy()).ToList();
            foreach (var j in backwardJoints)
            {
                j.Link1 = backwardLinks[AllLinks.IndexOf(j.Link1)];
                if (j.Link2 != null)
                    j.Link2 = backwardLinks[AllLinks.IndexOf(j.Link2)];
            }
            for (var i = 0; i < AllLinks.Count; i++)
            {
                var backwardLink = backwardLinks[i];
                var forwardLink = AllLinks[i];
                foreach (var j in forwardLink.joints)
                {
                    var jointIndex = AllJoints.IndexOf(j);
                    backwardLink.joints.Add(backwardJoints[jointIndex]);
                    if (j == forwardLink.ReferenceJoint1)
                        backwardLink.ReferenceJoint1 = backwardJoints[jointIndex];
                }
            }
        }

        /// <summary>
        /// Defines the movement characteristics.
        /// </summary>
        private void DefineMovementCharacteristics()
        {
            BeginTime = JointParameters.Times[0];
            EndTime = JointParameters.Times.Last();
            InitializeQueryVars();
            for (var i = 0; i < NumAllJoints; i++)
            {
                var j = SimulationJoints[i];
                if (j.TypeOfJoint == JointType.R) continue;
                j.OrigSlidePosition = JointParameters[0.0][oIOSJ[i], 6];
                j.MinSlidePosition = JointParameters.Parameters.Min(jp => jp[oIOSJ[i], 6]);
                j.MaxSlidePosition = JointParameters.Parameters.Max(jp => jp[oIOSJ[i], 6]);
            }
            if (lessThanFullRotation())
            {
                //if the crank input couldn't rotate all the way around,then this is easy... 
                CycleType = CycleTypes.LessThanFullCycle;
                return;
            }
            // if the simulation did go all the way around then, we should be careful to ensure is connects correctly.
            var cyclePeriodTime = Constants.FullCircle / Math.Abs(InputSpeed);
            if (DoesMechanismRepeatOnCrankCycle(cyclePeriodTime))
            {
                BeginTime = 0.0;
                EndTime = cyclePeriodTime;
                CycleType = CycleTypes.OneCycle;
                while (JointParameters.Times.Last() >= EndTime)
                {
                    var time = JointParameters.Times.Last();
                    var parameters = JointParameters.Parameters.Last();
                    JointParameters.RemoveAt(JointParameters.LastIndex);
                    time -= cyclePeriodTime;
                    JointParameters.AddNearBegin(time, parameters);

                    parameters = LinkParameters.Parameters.Last();
                    LinkParameters.RemoveAt(LinkParameters.LastIndex);
                    LinkParameters.AddNearEnd(time, parameters);
                    parameters = ForceParameters.Parameters.Last();
                    ForceParameters.RemoveAt(ForceParameters.LastIndex);
                }
                while (JointParameters.Times[0] < 0.0)
                {
                    var time = JointParameters.Times[0];
                    var parameters = JointParameters.Parameters[0];
                    JointParameters.RemoveAt(0);
                    time += cyclePeriodTime;
                    JointParameters.AddNearEnd(time, parameters);

                    parameters = LinkParameters.Parameters[0];
                    LinkParameters.RemoveAt(0);
                    LinkParameters.AddNearEnd(time, parameters);

                    parameters = ForceParameters.Parameters[0];
                    ForceParameters.RemoveAt(0);
                    ForceParameters.AddNearEnd(time, parameters);
                }
            }
            else
            {
                CycleType = CycleTypes.MoreThanOneCycle;
            }
            InitializeQueryVars();
        }

        /// <summary>
        /// Transposes the link and joint parameters.
        /// </summary>
        private void TransposeLinkAndJointParameters()
        {
            var fixedGroundJoints = GroundLink.joints.Where(j => j.FixedWithRespectTo(GroundLink)).ToList();
            var gndJointIndex = SimulationJoints.IndexOf(fixedGroundJoints[0]);
            var gndLinkIndex = SimulationLinks.IndexOf(GroundLink);
            for (var i = 0; i <= LinkParameters.LastIndex; i++)
                LinkParameters[i].Value[gndLinkIndex, 0] -= GroundLink.AngleInitial;
            TransposeJointPosition(gndJointIndex, gndLinkIndex);
#if DEBUGSERIAL
            TransposeJointVelocity(gndJointIndex, gndLinkIndex);
            TransposeJointAcceleration(gndJointIndex, gndLinkIndex);
            
            TransposeLinkStateVariables(gndLinkIndex, 0);
            TransposeLinkStateVariables(gndLinkIndex, 1);
            TransposeLinkStateVariables(gndLinkIndex, 2);
#else
            /* These joint velocity and acceleration functions don't depend on each other, but they
                 * require the position to be complete and that the link state variables stay untouched. */
            var task1 = Task.Factory.StartNew(() => TransposeJointVelocity(gndJointIndex, gndLinkIndex));
            var task2 = Task.Factory.StartNew(() => TransposeJointAcceleration(gndJointIndex, gndLinkIndex));
            Task.WaitAll(task1, task2);

            var task3 = Task.Factory.StartNew(() => TransposeLinkStateVariables(gndLinkIndex, 0));
            var task4 = Task.Factory.StartNew(() => TransposeLinkStateVariables(gndLinkIndex, 1));
            var task5 = Task.Factory.StartNew(() => TransposeLinkStateVariables(gndLinkIndex, 2));
            Task.WaitAll(task3, task4, task5);
#endif
        }

        /// <summary>
        /// Transposes the link state variables.
        /// </summary>
        /// <param name="gndLinkIndex">Index of the GND link.</param>
        /// <param name="varIndex">Index of the variable.</param>
        private void TransposeLinkStateVariables(int gndLinkIndex, int varIndex)
        {
            for (var i = 0; i <= LinkParameters.LastIndex; i++)
            {
                var lParams = LinkParameters[i].Value;
                var offset = lParams[oIOSL[gndLinkIndex], varIndex];
                for (var j = 0; j < NumLinks; j++)
                    lParams[oIOSL[j], varIndex] -= offset;
            }
        }

        /// <summary>
        /// Transposes the joint position.
        /// </summary>
        /// <param name="gndJointIndex">Index of the GND joint.</param>
        /// <param name="gndLinkIndex">Index of the GND link.</param>
        private void TransposeJointPosition(int gndJointIndex, int gndLinkIndex)
        {
            var xOffset = SimulationJoints[gndJointIndex].xInitial;
            var yOffset = SimulationJoints[gndJointIndex].yInitial;
            for (var i = 0; i <= JointParameters.LastIndex; i++)
            {
                var jParams = JointParameters[i].Value;
                var tx = jParams[oIOSJ[gndJointIndex], 0];
                var ty = jParams[oIOSJ[gndJointIndex], 1];
                var newAngle = -LinkParameters[i].Value[oIOSL[gndLinkIndex], 0];
                var cosAngle = Math.Cos(newAngle);
                var sinAngle = Math.Sin(newAngle);
                for (var j = 0; j < NumAllJoints; j++)
                {
                    var xNew = (jParams[oIOSJ[j], 0] - tx) * cosAngle - (jParams[oIOSJ[j], 1] - ty) * sinAngle + xOffset;
                    jParams[oIOSJ[j], 1] = (jParams[oIOSJ[j], 0] - tx) * sinAngle + (jParams[oIOSJ[j], 1] - ty) * cosAngle + yOffset;
                    jParams[oIOSJ[j], 0] = xNew;
                }
            }
        }

        /// <summary>
        /// Transposes the joint velocity.
        /// </summary>
        /// <param name="gndJointIndex1">The GND joint index1.</param>
        /// <param name="gndLinkIndex">Index of the GND link.</param>
        private void TransposeJointVelocity(int gndJointIndex1, int gndLinkIndex)
        {
            for (var i = 0; i <= JointParameters.LastIndex; i++)
            {
                var jParams = JointParameters[i].Value;
                var VxGnd = jParams[oIOSJ[gndJointIndex1], 2];
                var VyGnd = jParams[oIOSJ[gndJointIndex1], 3];
                var newAngle = -LinkParameters[i].Value[oIOSL[gndLinkIndex], 0];
                var cosAngle = Math.Cos(newAngle);
                var sinAngle = Math.Sin(newAngle);
                var angularVelocity = LinkParameters[i].Value[oIOSL[gndLinkIndex], 1];
                for (var j = 0; j < NumAllJoints; j++)
                {
                    var xNew = (jParams[oIOSJ[j], 2] - VxGnd) * cosAngle - (jParams[oIOSJ[j], 3] - VyGnd) * sinAngle
                               + angularVelocity * (jParams[oIOSJ[j], 1] - jParams[oIOSJ[gndJointIndex1], 1]);
                    jParams[oIOSJ[j], 3] = (jParams[oIOSJ[j], 2] - VxGnd) * sinAngle + (jParams[oIOSJ[j], 3] - VyGnd) * cosAngle
                                    - angularVelocity * (jParams[oIOSJ[j], 0] - jParams[oIOSJ[gndJointIndex1], 0]);
                    jParams[oIOSJ[j], 2] = xNew;
                }
            }
        }

        /// <summary>
        /// Transposes the joint acceleration.
        /// </summary>
        /// <param name="gndJointIndex1">The GND joint index1.</param>
        /// <param name="gndLinkIndex">Index of the GND link.</param>
        private void TransposeJointAcceleration(int gndJointIndex1, int gndLinkIndex)
        {
            for (var i = 0; i <= JointParameters.LastIndex; i++)
            {
                var jParams = JointParameters[i].Value;
                var ax = jParams[oIOSJ[gndJointIndex1], 4];
                var ay = jParams[oIOSJ[gndJointIndex1], 5];
                var newAngle = -LinkParameters[i].Value[gndLinkIndex, 0];
                var cosAngle = Math.Cos(newAngle);
                var sinAngle = Math.Sin(newAngle);
                var angularVelocity = LinkParameters[i].Value[oIOSL[gndLinkIndex], 1];
                var angularAcceleration = LinkParameters[i].Value[oIOSL[gndLinkIndex], 2];
                for (var j = 0; j < NumAllJoints; j++)
                {
                    var rx = (jParams[oIOSJ[j], 0] - jParams[oIOSJ[gndJointIndex1], 0]);
                    var ry = (jParams[oIOSJ[j], 1] - jParams[oIOSJ[gndJointIndex1], 1]);
                    var xNew = (jParams[oIOSJ[j], 4] - ax) * cosAngle - (jParams[oIOSJ[j], 5] - ay) * sinAngle
                               + angularVelocity * angularVelocity * rx + angularAcceleration * ry;
                    jParams[oIOSJ[j], 5] = (jParams[oIOSJ[j], 4] - ax) * sinAngle + (jParams[oIOSJ[j], 5] - ay) * cosAngle
                                    + angularVelocity * angularVelocity * ry - angularAcceleration * rx;
                    jParams[oIOSJ[j], 4] = xNew;
                }
            }
        }

        /// <summary>
        /// Doeses the mechanism repeat on crank cycle.
        /// </summary>
        /// <param name="cyclePeriodTime">The cycle period time.</param>
        /// <returns><c>true</c> if XXXX, <c>false</c> otherwise.</returns>
        private bool DoesMechanismRepeatOnCrankCycle(double cyclePeriodTime)
        {
            // Does the mechanism return to the same place?
            // even though the crank goes all the way around, other joints of the mechanism may be in different places. 
            var initJointState = JointParameters.Parameters[0];
            var initLinkState = LinkParameters.Parameters[0];
            var topTime = JointParameters.Times[0] + cyclePeriodTime;
            var maxAngleError = Constants.ErrorInDeterminingCompleteCycle;
            var maxLengthError = Constants.ErrorInDeterminingCompleteCycle * AverageLength;
            if (!double.IsNaN(MaxSmoothingError) && MaxSmoothingError > 0)
            {
                maxAngleError *= Constants.SmoothingErrorRepeatFactor * Math.Sqrt(MaxSmoothingError);
                maxLengthError *= Constants.SmoothingErrorRepeatFactor * Math.Sqrt(MaxSmoothingError) * AverageLength;
            }
            for (var i = 0; i < NumLinks; i++)
            {
                var deltaLinkAngle = FindLinkAngleAtTime(topTime, oIOSL[i]) - initLinkState[oIOSL[i], 0];
                while (deltaLinkAngle > Math.PI) deltaLinkAngle -= Constants.FullCircle;
                while (deltaLinkAngle < -Math.PI) deltaLinkAngle += Constants.FullCircle;
                if (deltaLinkAngle > maxAngleError) return false;
            }
            for (var i = 0; i < NumAllJoints; i++)
            {
                var topJointLocation = FindJointPositionAtTime(topTime, oIOSJ[i]);
                var initJointLocationX = initJointState[oIOSJ[i], 0];
                var initJointLocationY = initJointState[oIOSJ[i], 1];
                if (Math.Abs(topJointLocation[0] - initJointLocationX) > maxLengthError
                    || Math.Abs(topJointLocation[1] - initJointLocationY) > maxLengthError)
                    return false;
            }
            return true;
        }

        /// <summary>
        /// Sets the initial velocity and acceleration.
        /// </summary>
        /// <param name="joints">The joints.</param>
        /// <param name="links">The links.</param>
        /// <param name="initJointParams">The initialize joint parameters.</param>
        /// <param name="initLinkParams">The initialize link parameters.</param>
        private void SetInitialVelocityAndAcceleration(List<Joint> joints, List<Link> links,
            out double[,] initJointParams, out double[,] initLinkParams)
        {
            var posFinder = new PositionFinder(joints, links, gearsData, drivingIndex);
            posFinder.UpdateSliderPosition();
            var velSolver = new VelocitySolver(joints, links, firstInputJointIndex, drivingIndex, inputLinkIndex,
                InputSpeed, gearsData, AverageLength);
            var accelSolver = new AccelerationSolver(joints, links, firstInputJointIndex, drivingIndex,
                inputLinkIndex, InputSpeed, gearsData, AverageLength);

            initJointParams = WriteJointStatesVariablesToMatrixAndToLast(joints);
            initLinkParams = WriteLinkStatesVariablesToMatrixAndToLast(links);
            var smallTimeStep = (double.IsNaN(FixedTimeStep))
                ? Constants.SmallPerturbationFraction
                : Constants.SmallPerturbationFraction * FixedTimeStep;
            if (velSolver.Solve())
            {
                for (var i = 0; i <= drivingIndex; i++)
                {
                    initJointParams[oIOSJ[i], 2] = joints[i].vxLast = joints[i].vx;
                    initJointParams[oIOSJ[i], 3] = joints[i].vyLast = joints[i].vy;
                }
                for (var i = 0; i <= inputLinkIndex; i++)
                    initLinkParams[oIOSL[i], 1] = links[i].VelocityLast = links[i].Velocity;
                if (accelSolver.Solve())
                {
                    for (var i = 0; i <= drivingIndex; i++)
                    {
                        initJointParams[oIOSJ[i], 4] = joints[i].ax;
                        initJointParams[oIOSJ[i], 5] = joints[i].ay;
                    }
                    for (var i = 0; i <= inputLinkIndex; i++)
                        initLinkParams[oIOSL[i], 2] = links[i].Acceleration;
                }
                else
                {
                    /* velocity was successfully found, but not acceleration. */
                    if (posFinder.DefineNewPositions(smallTimeStep * InputSpeed) &&
                        velSolver.Solve())
                    {
                        /* forward difference on velocities to create accelerations. */
                        for (var i = 0; i <= drivingIndex; i++)
                        {
                            initJointParams[oIOSJ[i], 4] = joints[i].ax = (joints[i].vx - joints[i].vxLast) / smallTimeStep;
                            initJointParams[oIOSJ[i], 5] = joints[i].ay = (joints[i].vy - joints[i].vyLast) / smallTimeStep;
                        }
                        for (var i = 0; i <= inputLinkIndex; i++)
                            initLinkParams[oIOSL[i], 2] = links[i].Acceleration = (links[i].Velocity - links[i].VelocityLast) / smallTimeStep;

                        /* since the position solving wrote values to joints[i].x and .y, we need to reset them, for further work. */
                        foreach (var joint in SimulationJoints)
                        {
                            joint.x = joint.xInitial;
                            joint.y = joint.yInitial;
                        }
                        foreach (var link in SimulationLinks)
                            link.Angle = link.AngleInitial;
                    }
                }
                //SetCoM(links);
                return;
            }
            var ForwardJointParams = new double[NumAllJoints, 2];
            var ForwardLinkParams = new double[NumLinks];
            /*** Stepping Forward in Time ***/
            var forwardSuccess = posFinder.DefineNewPositions(smallTimeStep * InputSpeed);
            if (forwardSuccess)
            {
                for (var i = 0; i < NumAllJoints; i++)
                {
                    ForwardJointParams[oIOSJ[i], 0] = joints[i].x;
                    ForwardJointParams[oIOSJ[i], 1] = joints[i].y;
                }
                for (var i = 0; i < NumLinks; i++)
                    ForwardLinkParams[oIOSL[i]] = links[i].Angle;
            }
            /*** Stepping Backward in Time ***/
            var BackwardJointParams = new double[NumAllJoints, 2];
            var BackwardLinkParams = new double[NumLinks];
            var backwardSuccess = posFinder.DefineNewPositions(-smallTimeStep * InputSpeed);
            if (backwardSuccess)
            {
                for (var i = 0; i < NumAllJoints; i++)
                {
                    BackwardJointParams[oIOSJ[i], 0] = joints[i].x;
                    BackwardJointParams[oIOSJ[i], 1] = joints[i].y;
                }
                for (var i = 0; i < NumLinks; i++)
                    BackwardLinkParams[oIOSL[i]] = links[i].Angle;
            }
            if (forwardSuccess && backwardSuccess)
            {
                /* central difference puts values in init parameters. */
                for (var i = 0; i < NumAllJoints; i++)
                {
                    /* first-order central finite difference */
                    initJointParams[i, 2] = (ForwardJointParams[i, 0] - BackwardJointParams[i, 0]) / (2 * smallTimeStep);
                    initJointParams[i, 3] = (ForwardJointParams[i, 1] - BackwardJointParams[i, 1]) / (2 * smallTimeStep);
                    /* second-order central finite difference */
                    initJointParams[i, 4] = (ForwardJointParams[i, 0] - 2 * initJointParams[i, 0] +
                                             BackwardJointParams[i, 0]) / (smallTimeStep * smallTimeStep);
                    initJointParams[i, 5] = (ForwardJointParams[i, 1] - 2 * initJointParams[i, 1] +
                                             BackwardJointParams[i, 1]) / (smallTimeStep * smallTimeStep);
                }
                for (var i = 0; i < NumAllLinks; i++)
                {
                    /* first-order central finite difference */
                    initLinkParams[i, 1] = (ForwardLinkParams[i] - BackwardLinkParams[i]) / (2 * smallTimeStep);
                    /* second-order central finite difference */
                    initLinkParams[i, 2] = (ForwardLinkParams[i] - 2 * initLinkParams[i, 0] + BackwardLinkParams[i])
                                           / (smallTimeStep * smallTimeStep);
                }
            }
            else if (forwardSuccess)
            {
                /* forward difference puts values in init parameters. */
                for (var i = 0; i < NumAllJoints; i++)
                {
                    /* first-order forward finite difference */
                    initJointParams[i, 2] = (ForwardJointParams[i, 0] - initJointParams[i, 0]) / smallTimeStep;
                    initJointParams[i, 3] = (ForwardJointParams[i, 1] - initJointParams[i, 1]) / smallTimeStep;
                }
                for (var i = 0; i < NumAllLinks; i++)
                    /* first-order forward finite difference */
                    initLinkParams[i, 1] = (ForwardLinkParams[i] - initLinkParams[i, 0]) / smallTimeStep;
            }
            else if (backwardSuccess)
            {
                /* backward difference puts values in init parameters. */
                for (var i = 0; i < NumAllJoints; i++)
                {
                    /* first-order backward finite difference */
                    initJointParams[i, 2] = (initJointParams[i, 0] - BackwardJointParams[i, 0]) / smallTimeStep;
                    initJointParams[i, 3] = (initJointParams[i, 1] - BackwardJointParams[i, 1]) / smallTimeStep;
                }
                for (var i = 0; i <= NumAllLinks; i++)
                    /* first-order backward finite difference */
                    initLinkParams[i, 1] = (initLinkParams[i, 0] - BackwardLinkParams[i]) / smallTimeStep;
            }
            /* since the position solving wrote values to joints[i].x and .y, we need to reset them, for further work. */
            foreach (var joint in SimulationJoints)
            {
                joint.x = joint.xInitial;
                joint.y = joint.yInitial;
            }
            foreach (var link in SimulationLinks)
                link.Angle = link.AngleInitial;
        }

        /// <summary>
        /// Sets the initial force parameters.
        /// </summary>
        /// <param name="forces">The forces.</param>
        /// <param name="initForceParams">The initial force parameters.</param>
        private void SetInitialForceParams(List<Force> forces, double[,] initialJoints, List<Link> links, out double[,] initForceParams)
        {
            initForceParams = new double[NumForces, 10];
            List<int> force_Link_jointParam1 = new List<int>();
            List<int> force_Link_jointParam2 = new List<int>();
            List<double> appliedXoffset = new List<double>();
            List<double> appliedYoffset = new List<double>();
            List<double> forceLocationMag = new List<double>();
            List<double> forceLocationAngle = new List<double>();

            double forceAngle = 0.0;

            List<int> force_Link = new List<int>();
            int thelink = 0;
            for (int i = 0; i < forces.Count(); i++)
            {
                int linknumber = Determine_Link_With_Force(forces[i].xloc, forces[i].yloc);
                if (linknumber != 999)
                {
                    force_Link.Add(linknumber);
                }
                thelink = linknumber;
            }

            for (int i = 0; i < forces.Count; i++)
            {
                int holdJ1 = 0;
                int holdJ2 = 0;
                for (int j = 0; j < NumJoints; j++)
                {
                    if (initialJoints[oIOSJ[j], 0] == this.Links[force_Link[i]].joints[0].xInitial &&
                        initialJoints[oIOSJ[j], 1] == this.Links[force_Link[i]].joints[0].yInitial)
                    {
                        holdJ1 = j;
                        //break;
                    }
                    if (initialJoints[oIOSJ[j], 0] == this.Links[force_Link[i]].joints[1].xInitial &&
                        initialJoints[oIOSJ[j], 1] == this.Links[force_Link[i]].joints[1].yInitial)
                    {
                        holdJ2 = j;
                        //break;
                    }

                }
                if (this.Links[force_Link[i]].joints[1].IsGround)
                {
                    int temp = 0;
                    temp = holdJ1;
                    holdJ1 = holdJ2;
                    holdJ2 = temp;
                }
                force_Link_jointParam1.Add(holdJ1); //This stores the index of the primary   joint of a link with an applied force
                force_Link_jointParam2.Add(holdJ2); //This stores the index of the secondary joint of a link with an applied force

                //Find the vector (offset) from the first joint to the corresponding applied force
                appliedXoffset.Add(0.0 - initialJoints[oIOSJ[holdJ1], 0] + forces[i].xloc);
                appliedYoffset.Add(0.0 - initialJoints[oIOSJ[holdJ1], 1] + forces[i].yloc);

                forceLocationMag.Add(Math.Sqrt(Math.Pow(appliedXoffset[i], 2) + Math.Pow(appliedYoffset[i], 2))); // may have to change these two lines back to appliedForceXY
                forceLocationAngle.Add(Math.Atan2(appliedYoffset[i], appliedXoffset[i]));

            }


            for (var i = 0; i < forces.Count(); i++)
            {

                forceAngle = Math.Atan2(forces[i].ymag, forces[i].xmag);

                initForceParams[i, 0] = forces[i].xloc;
                initForceParams[i, 1] = forces[i].yloc;
                initForceParams[i, 2] = forces[i].xmag;
                initForceParams[i, 3] = forces[i].ymag;
                initForceParams[i, 4] = b2d(forces[i].isfixed);
                initForceParams[i, 5] = (double)force_Link_jointParam1[i];
                initForceParams[i, 6] = forceLocationMag[i];
                initForceParams[i, 7] = forceAngle - this.Links[force_Link[i]].AngleInitial;
                initForceParams[i, 8] = force_Link[i];
                initForceParams[i, 9] = forceLocationAngle[i] - this.Links[force_Link[i]].AngleInitial;
            }
        }

        /// <summary>
        /// Simulates the specified joints.
        /// </summary>
        /// <param name="joints">The joints.</param>
        /// <param name="links">The links.</param>
        /// <param name="Forward">The forward.</param>
        private void Simulate(List<Joint> joints, List<Link> links, List<Force> forces, Boolean Forward)
        {
            var timeStep = (Forward == (InputSpeed > 0)) ? FixedTimeStep : -FixedTimeStep;
            var startingPosChange = (Forward == (InputSpeed > 0))
                ? Constants.DefaultStepSize
                : -Constants.DefaultStepSize;
            if (inputJoint.TypeOfJoint == JointType.P) startingPosChange *= AverageLength;
            var maxLengthError = MaxSmoothingError * AverageLength;
            var currentTime = 0.0;
            Boolean validPosition;
            var posFinder = new PositionFinder(joints, links, gearsData, drivingIndex);
            var velSolver = new VelocitySolver(joints, links, firstInputJointIndex, drivingIndex, inputLinkIndex,
                InputSpeed, gearsData, AverageLength);
            var accelSolver = new AccelerationSolver(joints, links, firstInputJointIndex, drivingIndex,
                inputLinkIndex, InputSpeed, gearsData, AverageLength);
            do
            {
                #region Find Next Positions

                if (useErrorMethod)
                {
                    var k = 0;
                    double upperError;
                    do
                    {
                        timeStep = startingPosChange / InputSpeed;
                        NumericalPosition(timeStep, joints, links);
                        validPosition = posFinder.DefineNewPositions(startingPosChange);
                        upperError = posFinder.PositionError - maxLengthError;
                        if (validPosition && upperError < 0)
                        {
                            startingPosChange *= Constants.ErrorSizeIncrease;
                            // startingPosChange = startingPosChange * maxLengthError / (maxLengthError + upperError);
                        }
                        else
                        {
                            if (Math.Abs(startingPosChange * Constants.ConservativeErrorEstimation * 0.5) <
                                Constants.MinimumStepSize)
                                validPosition = false;
                            else startingPosChange *= Constants.ConservativeErrorEstimation * 0.5;
                        }
                    } while ((!validPosition || upperError > 0) && k++ < Constants.MaxItersInPositionError
                             &&
                             (Math.Abs(startingPosChange * Constants.ConservativeErrorEstimation * 0.5) >=
                              Constants.MinimumStepSize));
                    //var tempStep = startingPosChange;
                    //startingPosChange = (Constants.ErrorEstimateInertia * prevStep + startingPosChange) / (1 + Constants.ErrorEstimateInertia);
                    //prevStep = tempStep;
                }
                else
                {
                    // this next function puts the xNumerical and yNumerical values in the joints
                    NumericalPosition(timeStep, joints, links);
                    var delta = InputSpeed * timeStep;
                    // this next function puts the x and y values in the joints
                    validPosition = posFinder.DefineNewPositions(delta);
                }

                #endregion

                if (validPosition)
                {
                    if (Forward == (InputSpeed > 0))
                        lock (InputRange)
                        {
                            InputRange[1] = links[inputLinkIndex].Angle;
                        }
                    else
                        lock (InputRange)
                        {
                            InputRange[0] = links[inputLinkIndex].Angle;
                        }

                    #region Find Velocities for Current Position

                    // this next functions puts the vx and vy values as well as the vx_unit and vy_unit in the joints
                    if (!velSolver.Solve())
                    {
                        Status += "Instant Centers could not be found at" + currentTime + ".";
                        NumericalVelocity(timeStep, joints, links);
                    }

                    #endregion

                    #region Find Accelerations for Current Position

                    // this next functions puts the ax and ay values in the joints
                    if (!accelSolver.Solve())
                    {
                        Status += "Analytical acceleration could not be found at" + currentTime + ".";
                        NumericalAcceleration(timeStep, joints, links);
                    }

                    #endregion

                    SetCoM(links);

                    currentTime += timeStep;
                    var jointParams = WriteJointStatesVariablesToMatrixAndToLast(joints);
                    var linkParams = WriteLinkStatesVariablesToMatrixAndToLast(links);
                    var forceParams = WriteForceStateVariablesToMatrixAndToLast(forces, joints);
                    if (Forward == (InputSpeed > 0))
                    {
                        lock (JointParameters)
                            JointParameters.AddNearEnd(currentTime, jointParams);
                        lock (LinkParameters)
                            LinkParameters.AddNearEnd(currentTime, linkParams);
                        lock (ForceParameters)
                            ForceParameters.AddNearEnd(currentTime, forceParams);
                    }
                    else
                    {
                        lock (JointParameters)
                            JointParameters.AddNearBegin(currentTime, jointParams);
                        lock (LinkParameters)
                            LinkParameters.AddNearBegin(currentTime, linkParams);
                        lock (ForceParameters)
                            ForceParameters.AddNearBegin(currentTime, forceParams);
                    }
                }
            } while (validPosition && lessThanFullRotation());
        }

        /// <summary>
        /// Sets the center of mass of all the links in the mechanism.
        /// </summary>
        /// <param name="links"> The links in the mechanism. </param>
        public void SetCoM(List<Link> links){
            for (int i = 0; i < links.Count(); i++) {
                double jointXsum = 0;
                double jointYsum = 0;
                double numJoints = (double)links[i].joints.Count();
                for (int j = 0; j < links[i].joints.Count(); j++)
                {
                    jointXsum += links[i].joints[j].x;
                    jointYsum += links[i].joints[j].y;
                }
                links[i].CoMx = jointXsum / numJoints;
                links[i].CoMy = jointYsum / numJoints;
            }
        }


        /// <summary>
        /// Sets the mass moment of inertia of all the links in the mechanism.
        /// </summary>
        public void SetMoI()
        {
            double CoM_MoI = 0;
            double width = 0;
            double mass = 0;
            double length = 0;
            double totalLength = 0;
            for (int i = 0; i < this.Links.Count(); i++)
            {
                //determine if any joints are sliders for the given link
                bool isPrismatic = false;
                for (int j = 0; j < this.Links[i].joints.Count(); j++)
                {
                    if (this.Links[i].joints[j].TypeOfJoint == JointType.P)
                    {
                        isPrismatic = true;
                        break;
                    }
                }
                if (isPrismatic)
                {
                    continue;
                }

                mass = this.Links[i].Mass;
                if (this.Links[i].joints.Count() == 2)
                {
                    width = this.Links[i].Width;
                    length = this.Links[i].MaxLength;
                    totalLength = length + width;
                    CoM_MoI = ((mass / 12) * (Math.Pow(totalLength, 2) + Math.Pow(width, 2)));
                    this.Links[i].MoI = CoM_MoI + (mass * Math.Pow((length / 2), 2));
                }
                else if (this.Links[i].joints.Count() == 3)
                {
                    double b = this.Links[i].MaxLength;
                    double a = 0;
                    double c = 0;
                    foreach (KeyValuePair<int, double> kvp in this.Links[i].lengths)
                    {
                        if (!(kvp.Value == b))
                        {
                            if (kvp.Value > a)
                            {
                                c = a;
                                a = kvp.Value;
                            }
                            else
                            {
                                c = kvp.Value;
                            }
                        }
                    }
                    double cosB = (Math.Pow(c, 2) + Math.Pow(a, 2) - Math.Pow(b, 2)) / (2 * c * a);
                    double thetaB = Math.Acos(cosB);
                    double cosA = (Math.Pow(b, 2) + Math.Pow(c, 2) - Math.Pow(a, 2)) / (2 * b * c);
                    double thetaA = Math.Acos(cosA);
                    double thetaC = Math.PI - thetaA - thetaB;
                    double h = (c * Math.Sin(thetaA) + a * Math.Sin(thetaC)) / 2;
                    double x = c * Math.Cos(thetaA);
                    CoM_MoI = ((mass / 18) * ((Math.Pow(h, 2) + Math.Pow(b, 2) + Math.Pow(x, 2)) - b * x));

                    double x_dist = this.LinkParameters.Parameters[0][i, 3] - this.Links[i].joints[0].xInitial;
                    double y_dist = this.LinkParameters.Parameters[0][i, 4] - this.Links[i].joints[0].yInitial;
                    double d = Math.Pow(x_dist, 2) + Math.Pow(y_dist, 2); // need d^2 here so rather than sqrt followed by ^2 we just leave it

                    this.Links[i].MoI = CoM_MoI + mass * d;

                }
            }
        }

        /// <summary>
        /// Writes the joint states variables to matrix and to last.
        /// </summary>
        /// <param name="joints">The joints.</param>
        /// <returns>System.Double[].</returns>
        private double[,] WriteJointStatesVariablesToMatrixAndToLast(List<Joint> joints)
        {
            var jointParams = new double[NumAllJoints, maxJointParamLengths];
            for (var i = 0; i < NumAllJoints; i++)
            {
                var j = joints[i];
                jointParams[oIOSJ[i], 0] = j.x;
                jointParams[oIOSJ[i], 1] = j.y;
                jointParams[oIOSJ[i], 2] = j.vx;
                jointParams[oIOSJ[i], 3] = j.vy;
                jointParams[oIOSJ[i], 4] = j.ax;
                jointParams[oIOSJ[i], 5] = j.ay;
                if (j.TypeOfJoint != JointType.R)
                {
                    jointParams[oIOSJ[i], 6] = j.SlidePosition;
                    jointParams[oIOSJ[i], 7] = j.SlideVelocity;
                    jointParams[oIOSJ[i], 8] = j.SlideAcceleration;
                }
                j.xLast = j.x;
                j.yLast = j.y;
                j.vxLast = j.vx;
                j.vyLast = j.vy;
            }
            return jointParams;
        }

        /// <summary>
        /// Writes the link states variables to matrix and to last.
        /// </summary>
        /// <param name="links">The links.</param>
        /// <returns>System.Double[].</returns>
        private double[,] WriteLinkStatesVariablesToMatrixAndToLast(List<Link> links)
        {
            var linkParams = new double[NumLinks, 9]; //spaces 5 and 6 will be filled with CoMaccX and CoMaccY in dynamic analysis
            for (var i = 0; i < NumLinks; i++)
            {
                var l = links[i];
                linkParams[oIOSL[i], 0] = l.Angle;
                linkParams[oIOSL[i], 1] = l.Velocity;
                linkParams[oIOSL[i], 2] = l.Acceleration;
                linkParams[oIOSL[i], 3] = l.CoMx;
                linkParams[oIOSL[i], 4] = l.CoMy;
                l.AngleLast = l.Angle;
                l.VelocityLast = l.Velocity;
            }
            return linkParams;
        }

        /// <summary>
        /// Writes the force states variables to matrix.
        /// </summary>
        /// <param name="forces">The forces.</param>
        /// <param name="joints"> The joints. </param>
        /// <returns>System.Double[].</returns>
        private double[,] WriteForceStateVariablesToMatrixAndToLast(List<Force> forces, List<Joint> joints)
        {
            var forceParams = new double[NumForces, 10];

            for (var i = 0; i < NumForces; i++)
            {
                double linkDeltaAngle = 0.0;
                double xmag = this.ForceParameters.Parameters[0][i, 2];
                double ymag = this.ForceParameters.Parameters[0][i, 3];
                double forceMag = Math.Sqrt(Math.Pow(xmag, 2) + Math.Pow(ymag, 2));
                double initForceAngle = Math.Atan2(ymag, xmag);
                int refJointIndex = (int)this.ForceParameters.Parameters[0][i, 5];
                double distJoint = this.ForceParameters.Parameters[0][i, 6];
                double forceAngle = this.ForceParameters.Parameters[0][i, 7];
                double link = this.ForceParameters.Parameters[0][i, 8];
                double offsetAngle = this.ForceParameters.Parameters[0][i, 9];

                if (joints[refJointIndex].Link2 != null && !joints[refJointIndex].Link2.IsGround)
                {
                    forces[i].xloc = joints[refJointIndex].x + distJoint * Math.Cos(joints[refJointIndex].Link2.Angle + offsetAngle);
                    forces[i].yloc = joints[refJointIndex].y + distJoint * Math.Sin(joints[refJointIndex].Link2.Angle + offsetAngle);
                    linkDeltaAngle = joints[refJointIndex].Link2.Angle - joints[refJointIndex].Link2.AngleInitial;
                }
                else
                {
                    forces[i].xloc = joints[refJointIndex].x + distJoint * Math.Cos(joints[refJointIndex].Link1.Angle + offsetAngle);
                    forces[i].yloc = joints[refJointIndex].y + distJoint * Math.Sin(joints[refJointIndex].Link1.Angle + offsetAngle);
                    linkDeltaAngle = joints[refJointIndex].Link1.Angle - joints[refJointIndex].Link1.AngleInitial;
                }
                /*if (joints[refJointIndex].Link2.IsGround)
                {
                    forces[i].xloc = joints[refJointIndex].x + distJoint * Math.Cos(joints[refJointIndex].Link1.Angle + offsetAngle);
                    forces[i].yloc = joints[refJointIndex].y + distJoint * Math.Sin(joints[refJointIndex].Link1.Angle + offsetAngle);
                    linkDeltaAngle = joints[refJointIndex].Link1.Angle - joints[refJointIndex].Link1.AngleInitial;
                }
                else
                {
                    forces[i].xloc = joints[refJointIndex].x + distJoint * Math.Cos(joints[refJointIndex].Link2.Angle + offsetAngle);
                    forces[i].yloc = joints[refJointIndex].y + distJoint * Math.Sin(joints[refJointIndex].Link2.Angle + offsetAngle);
                    linkDeltaAngle = joints[refJointIndex].Link2.Angle - joints[refJointIndex].Link2.AngleInitial;
                }*/
                if (forces[i].isfixed)
                {
                    forces[i].xmag = forceMag * Math.Cos(initForceAngle + linkDeltaAngle);
                    forces[i].ymag = forceMag * Math.Sin(initForceAngle + linkDeltaAngle);
                }
                else
                {
                    forces[i].xmag = this.ForceParameters.Parameters[0][i, 2];
                    forces[i].ymag = this.ForceParameters.Parameters[0][i, 3];
                }

                forceParams[i, 0] = forces[i].xloc;
                forceParams[i, 1] = forces[i].yloc;
                forceParams[i, 2] = forces[i].xmag;
                forceParams[i, 3] = forces[i].ymag;
                forceParams[i, 4] = b2d(forces[i].isfixed);
                forceParams[i, 5] = (double)refJointIndex;
                forceParams[i, 6] = distJoint;
                forceParams[i, 7] = forceAngle;
                forceParams[i, 8] = link;
                forceParams[i, 9] = offsetAngle;

            }
            return forceParams;
        }

        private void obtainTwoLineIntersection(double x1, double y1, double x2, double y2, double x3, double y3, double slideAngle, out double x_1, out double y_1)
        {

            //line perpendicular to the angle of slide is moved to which ever coordinate is used in the circle diagram method 

            //here I am creating a line using the two known coordinates
            double slope1 = (y2 - y1) / (x2 - x1);

            double newSlopeBasedOnSlideAngle = Math.Tan(slideAngle + Math.PI / 2); //slope based on slideAngle +90 deg has been determined

            double x = Math.Round((y3 - y1 + slope1 * x1 - newSlopeBasedOnSlideAngle * x3) / (slope1 - newSlopeBasedOnSlideAngle), 4);
            double y = 0.0;

            if (Math.Abs(newSlopeBasedOnSlideAngle) < 1e10) //accounting for tan(90) to be infinity
                y = Math.Round(y3 + newSlopeBasedOnSlideAngle * (x - x3), 4);
            else
                y = Math.Round(slope1 * (x - x1) + y1, 4);


            x_1 = x;
            y_1 = y;
        }
        private void obtainTwoLineIntersection(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, out double x_1, out double y_1)
        {
            //find the slope 

            double slope1 = (y2 - y1) / (x2 - x1);
            double slope2 = (y4 - y3) / (x4 - x3);
            double a = 0.0, b = 0;
            if (Math.Pow(1 / slope2 - 1 / slope1, -1) == 0)
            {
                if (y1 == 0 && slope1 == 0)
                    a = 0;
                else
                    a = y1 / slope1;

                if (y3 == 0 && slope2 == 0)
                    b = 0;
                else
                    b = y3 / slope2;

            }
            else
            {
                a = y1 / slope1;
                b = y3 / slope2;

            }

            double y = Math.Pow(1 / slope2 - 1 / slope1, -1) * (x1 - x3 - (a) + (b));
            double x = 0;
            if (b == 0)
                x = ((y - y1) / slope1) + x1;
            else
                x = ((y - y3) / slope2) + x3;
            if (a == 0)
                x = ((y - y3) / slope2) + x3;
            else
                x = ((y - y1) / slope1) + x1;

            if (double.IsNaN(x))
                x = double.PositiveInfinity;
            if (double.IsInfinity(x))
                x = double.PositiveInfinity;
            if (double.IsNaN(y))
                y = double.PositiveInfinity;
            if (double.IsInfinity(y))
                y = double.PositiveInfinity;

            x_1 = x;
            y_1 = y;
        }
        //instant center method will be here 
        public List<string> InstantCenterMethod()
        {
            List<string> output = new List<string>();

            #region find instant centers
            //determine the actual number of joints
            int noOfJoints = 0;
            List<int> jointIndices = new List<int>();
            int ii = 0;
            foreach (var n in this.Joints)
            {

                if (n.Link1 != null && n.Link2 != null)
                {
                    noOfJoints++;
                    jointIndices.Add(ii);

                }
                ii++;
            }

            //now we know the actual joints. Now we can determine the total number of instant centers

            int numberOfInstantCenters = this.NumLinks * (this.NumLinks - 1) / 2;

            //now that the number of instant centers have been determined, we need to determine the total number of primary and secondary instant centers 

            //the number of primary instant centers are equal to the number of joints available. 

            int numberOfPrimaryInstantCenters = jointIndices.Count;
            int numberOfSecondaryInstantCenters = numberOfInstantCenters - numberOfPrimaryInstantCenters;


            //let us determine what those primary and secondary instant centers are in terms of link numbers. 

            List<double[,]> instantCenterIdentifiers1 = new List<double[,]>(); //link 1, link 2, jointx, joint y
            List<double[,]> SecinstantCenterIdentifiers = new List<double[,]>(); //link 1, link 2, jointx, joint y
            List<string[,]> Secnames = new List<string[,]>();
            List<double[]> ICVelocity = new List<double[]>(); //list of angular velocities computed using IC method
            double slideAngle = 0.0;


            for (int xxx = 0; xxx < this.JointParameters.Count; xxx++)
            {

                double[,] instantCenterIdentifiers = new double[numberOfInstantCenters, 4];
                double[,] secondaryics = new double[numberOfSecondaryInstantCenters, 4];
                string[,] secondarynames=new string[numberOfSecondaryInstantCenters,2];



                int iii = 0;



                for (int i = 0; i < this.NumLinks; i++)
                {
                    for (int j = i + 1; j < this.NumLinks; j++)
                    {

                        instantCenterIdentifiers[iii, 0] = i;
                        instantCenterIdentifiers[iii, 1] = j;
                        instantCenterIdentifiers[iii, 2] = 9999;
                        instantCenterIdentifiers[iii, 3] = 9999;
                        iii++;
                    }

                }
                //locations of primary instant centers are at the joints
                //of the list in instantcenteridentifiers, let us now identify all the primary and the secondary instant centers 

                //first identifying the primary instant centers 
                string param = System.Convert.ToString(xxx + 1);
                System.Diagnostics.Debug.WriteLine("\n\n\n*****     -----     Analysis " + param + "     -----     *****");
                output.Add("\n\n\n*****     -----     Analysis " + param + "     -----     *****");
                output.Add("Primary IC's:\t\t|X Location||Y Location|");
                #region primary instant centers
                for (int i = 0; i < jointIndices.Count; i++)
                {
                    var isJointP = this.Joints[jointIndices[i]].TypeOfJoint.ToString();
                    var link1_joint1 = this.Joints[jointIndices[i]].Link1;
                    var link2_joint1 = this.Joints[jointIndices[i]].Link2;
                    int link1_index = 999;
                    int link2_index = 999;
                    for (int j = 0; j < this.Links.Count; j++)
                    {
                        if (link1_joint1 == this.Links[j])
                            link1_index = j;
                        if (link2_joint1 == this.Links[j])
                            link2_index = j;
                    }

                    //now that the link indices have been determined, the step is to place the joint coordinates into the appropriate position in the instant center array - instantCenterIdentifiers 
                    for (int k = 0; k < iii; k++)
                    {
                        if (instantCenterIdentifiers[k, 0] == link1_index && instantCenterIdentifiers[k, 1] == link2_index)
                        {
                            //  instantCenterIdentifiers[k, 2] = pms.Joints[jointIndices[i]].xInitial;
                            //  instantCenterIdentifiers[k, 3] = pms.Joints[jointIndices[i]].yInitial;

                            //var x1 = pms.Joints[jointIndices[i]].xInitial;
                            //var x2 = pms.Joints[jointIndices[i]].yInitial;
                            if (isJointP == "R")
                            {
                                var xx = this.JointParameters.Parameters[xxx];
                                instantCenterIdentifiers[k, 2] = xx[jointIndices[i], 0];
                                instantCenterIdentifiers[k, 3] = xx[jointIndices[i], 1];


                                double xval = System.Math.Round(instantCenterIdentifiers[k, 2],2);
                                string xstring = System.Convert.ToString(xval);
                                double yval = System.Math.Round(instantCenterIdentifiers[k, 3], 2);
                                string ystring = System.Convert.ToString(yval);
                                output.Add(this.Joints[link1_index].Link1.name + "," + this.Joints[link1_index].Link2.name + ":    \t" + xstring + "\t\t" + ystring);
                            }
                            else
                            {
                                //for the prismatic joint
                                //find the angle of slide
                                //determine the perpendicular 
                                //that's where the primary instant center is located

                                //if the slide angle is horizontal, then x coordinate is 0,y coordinate is infinity
                                //if the slide angle is vertical, then x coordinate is infinity, y coordinate is 0.
                                //if the slide angle is other than the above two, then the x and y coordinates are infinity

                                slideAngle = this.Joints[jointIndices[i]].SlideAngleInitial;

                                var xx = this.JointParameters.Parameters[xxx];
                                if (slideAngle == 0.0)
                                {
                                    instantCenterIdentifiers[k, 2] = xx[jointIndices[i], 0];
                                    instantCenterIdentifiers[k, 3] = double.PositiveInfinity;

                                }
                                if (slideAngle == Math.PI / 2)
                                {
                                    instantCenterIdentifiers[k, 2] = double.PositiveInfinity;
                                    instantCenterIdentifiers[k, 3] = xx[jointIndices[i], 1];

                                }
                                else
                                {
                                    instantCenterIdentifiers[k, 2] = double.PositiveInfinity;
                                    instantCenterIdentifiers[k, 3] = double.PositiveInfinity;

                                }



                            }

                        }
                        if (instantCenterIdentifiers[k, 1] == link1_index && instantCenterIdentifiers[k, 0] == link2_index)
                        {
                            // instantCenterIdentifiers[k, 2] = pms.Joints[jointIndices[i]].xInitial;
                            // instantCenterIdentifiers[k, 3] = pms.Joints[jointIndices[i]].yInitial;
                            // var x1 = pms.Joints[jointIndices[i]].xInitial;
                            // var x2 = pms.Joints[jointIndices[i]].yInitial;
                            var xx = this.JointParameters.Parameters[xxx];
                            if (isJointP == "R")
                            {

                                instantCenterIdentifiers[k, 2] = xx[jointIndices[i], 0];
                                instantCenterIdentifiers[k, 3] = xx[jointIndices[i], 1];

                                double xval = System.Math.Round(instantCenterIdentifiers[k, 2], 2);
                                string xstring = System.Convert.ToString(xval);
                                double yval = System.Math.Round(instantCenterIdentifiers[k, 3], 2);
                                string ystring = System.Convert.ToString(yval);
                                output.Add(this.Joints[link1_index].Link1.name + "," + this.Joints[link1_index].Link2.name + ":    \t" + xstring + "\t\t" + ystring);
                            }
                            else
                            {

                                slideAngle = this.Joints[jointIndices[i]].SlideAngleInitial;
                                //for the prismatic joint
                                //find the angle of slide
                                //determine the perpendicular 
                                //that's where the primary instant center is located

                                //if the slide angle is horizontal, then x coordinate is 0,y coordinate is infinity
                                //if the slide angle is vertical, then x coordinate is infinity, y coordinate is 0.
                                //if the slide angle is other than the above two, then the x and y coordinates are infinity


                                if (slideAngle == 0.0)
                                {
                                    instantCenterIdentifiers[k, 2] = xx[jointIndices[i], 0];
                                    instantCenterIdentifiers[k, 3] = double.PositiveInfinity;

                                }
                                else if (slideAngle == Math.Round(Math.PI / 2, 2))
                                {
                                    instantCenterIdentifiers[k, 2] = double.PositiveInfinity;
                                    instantCenterIdentifiers[k, 3] = xx[jointIndices[i], 1];

                                }
                                else
                                {
                                    instantCenterIdentifiers[k, 2] = double.PositiveInfinity;
                                    instantCenterIdentifiers[k, 3] = double.PositiveInfinity;

                                }



                            }

                        }
                    }
                }
                #endregion
                output.Add("\n");
                output.Add("Secondary IC's:\t\t|X Location||Y Location|");
                #region secondary instant centers
            //now that the primary instant centers have all been determined, the next step is to detemrine the secondary instant centers 
            //this is where we need to incorporate the circle diagram method for instant centers

                //need to determine (0,2), (1,3)

                //paths to get to (0,2) -> 0-1 1-2;2-3 0-3 (1,3) -> 3-0,0-1

                //first step is to extract the secondary instant center from instantCenterIdentifiers 
                int numic = 0;

            loops: for (int i = 0; i < iii; i++) //goto statement is not good, we can use a do-while or something like that. 
                {

                    if (instantCenterIdentifiers[i, 2] == 9999)
                    {
                        //the first secondary instant center's location has been determined

                        var link1 = (int)instantCenterIdentifiers[i, 0];
                        var link2 = (int)instantCenterIdentifiers[i, 1];

                        double intermediateLink = 99999;


                        List<double> PointsX_2 = new List<double>();
                        List<double> PointsY_2 = new List<double>();
                        double x_1, y_1;

                        //now we need to find two paths between link1 and link2 

                        //if link1 is 0, link2 is 2, then we need to find 1, to use 01 and 12 
                        //we also need to find 3, to use 03 and 23

                        //also got to make sure the loop is continuous 01 12 or 04 42 and not 04 52 

                        for (int j = 0; j < this.NumLinks; j++)
                        {

                            if ((j > link1 && j < link2) || (j < link1 && j > link2) || (j > link1 && j > link2) || (j < link1 && j < link2))
                            {

                                //now that index has been determined, the next step is to find if the instant center location is actually available in the instantcenteridentifier array
                                intermediateLink = j;
                                List<double> PointsX_1 = new List<double>();
                                List<double> PointsY_1 = new List<double>();

                                for (int k = 0; k < iii; k++)
                                {

                                    if ((instantCenterIdentifiers[k, 0] == link1 && instantCenterIdentifiers[k, 1] == intermediateLink) || (instantCenterIdentifiers[k, 0] == intermediateLink && instantCenterIdentifiers[k, 1] == link1))
                                    {
                                        if (instantCenterIdentifiers[k, 2] != 9999)
                                            for (int kk = 0; kk < iii; kk++)
                                            {

                                                if ((instantCenterIdentifiers[kk, 0] == link2 && instantCenterIdentifiers[kk, 1] == intermediateLink) || (instantCenterIdentifiers[kk, 0] == intermediateLink && instantCenterIdentifiers[kk, 1] == link2))
                                                {
                                                    if (instantCenterIdentifiers[kk, 2] != 9999)

                                                        if (PointsX_1.Count < 2)
                                                        {
                                                            PointsX_1.Add(instantCenterIdentifiers[k, 2]);
                                                            PointsY_1.Add(instantCenterIdentifiers[k, 3]);
                                                        }


                                                }


                                            }


                                    }



                                    else if ((instantCenterIdentifiers[k, 0] == link2 && instantCenterIdentifiers[k, 1] == intermediateLink) || (instantCenterIdentifiers[k, 0] == intermediateLink && instantCenterIdentifiers[k, 1] == link2))
                                    {
                                        if (instantCenterIdentifiers[k, 2] != 9999)
                                            for (int kk = 0; kk < iii; kk++)
                                            {

                                                if ((instantCenterIdentifiers[kk, 0] == link1 && instantCenterIdentifiers[kk, 1] == intermediateLink) || (instantCenterIdentifiers[kk, 0] == intermediateLink && instantCenterIdentifiers[kk, 1] == link1))
                                                {
                                                    if (instantCenterIdentifiers[kk, 2] != 9999)

                                                        if (PointsX_1.Count < 2)
                                                        {
                                                            PointsX_1.Add(instantCenterIdentifiers[k, 2]);
                                                            PointsY_1.Add(instantCenterIdentifiers[k, 3]);
                                                        }


                                                }


                                            }


                                    }


                                }

                                if (PointsX_1.Count < 4)
                                {
                                    foreach (var m in PointsX_1)
                                        PointsX_2.Add(m);

                                    foreach (var m in PointsY_1)
                                        PointsY_2.Add(m);
                                }

                            }

                        }

                        //now that four points have been determined, let us determine the intersection point

                        if (PointsX_2.Count >= 4)
                        {
                            //if all PointsX_2 and PointsY_2 have regular points and no infinity, then use the obtainTwoLineIntersection
                            //if there is an infinity, then we have to modify the two line intersection to account for sliders. 
                            var infinityBool = false;
                            int findXIndex = 9999;
                            int findYIndex = 9999;
                            int iiii = 0;
                            foreach (var n in PointsX_2)
                            {
                                if (n == double.PositiveInfinity)
                                {
                                    infinityBool = true;
                                    findXIndex = iiii;
                                }
                                iiii++;
                            }
                            iiii = 0;
                            foreach (var n in PointsY_2)
                            {
                                if (n == double.PositiveInfinity)
                                {
                                    infinityBool = true;
                                    findYIndex = iiii;
                                }
                                iiii++;
                            }


                            if (!infinityBool)
                            { 
                                obtainTwoLineIntersection(PointsX_2[0], PointsY_2[0], PointsX_2[1], PointsY_2[1], PointsX_2[2], PointsY_2[2], PointsX_2[3], PointsY_2[3], out x_1, out y_1);
                               
                                instantCenterIdentifiers[i, 2] = x_1;
                                instantCenterIdentifiers[i, 3] = y_1;
                                double xval = System.Math.Round(instantCenterIdentifiers[i, 2], 2);
                                double yval = System.Math.Round(instantCenterIdentifiers[i, 3], 2);
                                string xloc = System.Convert.ToString(xval);
                                string yloc = System.Convert.ToString(yval);
                                output.Add(this.Joints[link1].Link1.name + "," + this.Joints[link2].Link1.name+":   \t"+xloc+"\t\t"+yloc);
                                secondaryics[numic, 0] = link1;
                                secondaryics[numic, 1] = link2;
                                secondaryics[numic, 2] = instantCenterIdentifiers[i, 2];
                                secondaryics[numic, 3] = instantCenterIdentifiers[i, 3];
                                secondarynames[numic, 0] = this.Joints[link1].Link1.name;
                                secondarynames[numic, 1] = this.Joints[link2].Link1.name;
                                numic++;  

                            }
                            else
                            {
                                //need to account for slider aspects here
                                //  obtainTwoLineIntersection(-3, 10,18,6, 0, 0, Math.PI / 2, out x_1, out y_1);
                                //   obtainTwoLineIntersection(0, 0, -3, 10, 18, 6, 0, out x_1, out y_1);
                                //   obtainTwoLineIntersection(-3, 10, 18, 6, 0, 0, 0, out x_1, out y_1);

                                //we now know which index has infinity; 
                                //if 0 or 1 has infinity, then 2,3 are regular line
                                //if 2 or 3 has infinity, then 0,1 is the regular line 

                                if (findXIndex != 9999 || findYIndex != 9999)
                                {
                                    if (findXIndex == 0 || findYIndex == 0)
                                    {
                                        obtainTwoLineIntersection(PointsX_2[2], PointsY_2[2], PointsX_2[3], PointsY_2[3], PointsX_2[1], PointsY_2[1], slideAngle, out x_1, out y_1);
                                       

                                        instantCenterIdentifiers[i, 2] = x_1;
                                        instantCenterIdentifiers[i, 3] = y_1;
                                        double xval = System.Math.Round(instantCenterIdentifiers[i, 2], 2);
                                        double yval = System.Math.Round(instantCenterIdentifiers[i, 3], 2);
                                        string xloc = System.Convert.ToString(xval);
                                        string yloc = System.Convert.ToString(yval);
                                        output.Add(this.Joints[link1].Link1.name + "," + this.Joints[link2].Link1.name + ":   \t" + xloc + "\t\t" + yloc);
                                        secondaryics[numic, 0] = link1;
                                        secondaryics[numic, 1] = link2;
                                        secondaryics[numic, 2] = instantCenterIdentifiers[i, 2];
                                        secondaryics[numic, 3] = instantCenterIdentifiers[i, 3];
                                        secondarynames[numic, 0] = this.Joints[link1].Link1.name;
                                        secondarynames[numic, 1] = this.Joints[link2].Link1.name;
                                        numic++;
                                    }
                                    else if (findXIndex == 1 || findYIndex == 1)
                                    {
                                        obtainTwoLineIntersection(PointsX_2[2], PointsY_2[2], PointsX_2[3], PointsY_2[3], PointsX_2[0], PointsY_2[0], slideAngle, out x_1, out y_1);
                                        instantCenterIdentifiers[i, 2] = x_1;
                                        instantCenterIdentifiers[i, 3] = y_1;
                                        double xval = System.Math.Round(instantCenterIdentifiers[i, 2], 2);
                                        double yval = System.Math.Round(instantCenterIdentifiers[i, 3], 2);
                                        string xloc = System.Convert.ToString(xval);
                                        string yloc = System.Convert.ToString(yval);
                                        output.Add(this.Joints[link1].Link1.name + "," + this.Joints[link2].Link1.name + ":   \t" + xloc + "\t\t" + yloc);
                                        secondaryics[numic, 0] = link1;
                                        secondaryics[numic, 1] = link2;
                                        secondaryics[numic, 2] = instantCenterIdentifiers[i, 2];
                                        secondaryics[numic, 3] = instantCenterIdentifiers[i, 3];
                                        secondarynames[numic, 0] = this.Joints[link1].Link1.name;
                                        secondarynames[numic, 1] = this.Joints[link2].Link1.name;
                                        numic++;

                                    }
                                    else if (findXIndex == 2 || findYIndex == 2)
                                    {
                                        obtainTwoLineIntersection(PointsX_2[0], PointsY_2[0], PointsX_2[1], PointsY_2[1], PointsX_2[3], PointsY_2[3], slideAngle, out x_1, out y_1);
                                       
                                        instantCenterIdentifiers[i, 2] = x_1;
                                        instantCenterIdentifiers[i, 3] = y_1;
                                        double xval = System.Math.Round(instantCenterIdentifiers[i, 2], 2);
                                        double yval = System.Math.Round(instantCenterIdentifiers[i, 3], 2);
                                        string xloc = System.Convert.ToString(xval);
                                        string yloc = System.Convert.ToString(yval);
                                        output.Add(this.Joints[link1].Link1.name + "," + this.Joints[link2].Link1.name + ":   \t" + xloc + "\t\t" + yloc);
                                        secondaryics[numic, 0] = link1;
                                        secondaryics[numic, 1] = link2;
                                        secondaryics[numic, 2] = instantCenterIdentifiers[i, 2];
                                        secondaryics[numic, 3] = instantCenterIdentifiers[i, 3];
                                        secondarynames[numic, 0] = this.Joints[link1].Link1.name;
                                        secondarynames[numic, 1] = this.Joints[link2].Link1.name;
                                        numic++;

                                    }
                                    else if (findXIndex == 3 || findYIndex == 3)
                                    {
                                        obtainTwoLineIntersection(PointsX_2[0], PointsY_2[0], PointsX_2[1], PointsY_2[1], PointsX_2[2], PointsY_2[2], slideAngle, out x_1, out y_1);
                                       
                                        instantCenterIdentifiers[i, 2] = x_1;
                                        instantCenterIdentifiers[i, 3] = y_1;
                                        double xval = System.Math.Round(instantCenterIdentifiers[i, 2], 2);
                                        double yval = System.Math.Round(instantCenterIdentifiers[i, 3], 2);
                                        string xloc = System.Convert.ToString(xval);
                                        string yloc = System.Convert.ToString(yval);
                                        output.Add(this.Joints[link1].Link1.name + "," + this.Joints[link2].Link1.name + ":   \t" + xloc + "\t\t" + yloc);
                                        secondaryics[numic, 0] = link1;
                                        secondaryics[numic, 1] = link2;
                                        secondaryics[numic, 2] = instantCenterIdentifiers[i, 2];
                                        secondaryics[numic, 3] = instantCenterIdentifiers[i, 3];
                                        secondarynames[numic, 0] = this.Joints[link1].Link1.name;
                                        secondarynames[numic, 1] = this.Joints[link2].Link1.name;

                                        numic++;

                                    }

                                }
                            }
                        }
                    }

                }

                for (int i = 0; i < iii; i++)
                    if (instantCenterIdentifiers[i, 2] == 9999)
                        goto loops;

                #endregion
                instantCenterIdentifiers1.Add(instantCenterIdentifiers);
                SecinstantCenterIdentifiers.Add(secondaryics);
                Secnames.Add(secondarynames);


                this.JointParameters.ICLoc=instantCenterIdentifiers1;
                this.JointParameters.SecICLoc = SecinstantCenterIdentifiers;
                this.JointParameters.SecICname = Secnames;
                #region velocities of links using instant centers

                // for each link there's going to an angular velocity. 
                //start with the known angular velocity - preferably the input link
                //ground links - angular velocities are going to be zero. 

                //we also need to know which link is the slider 

                int sliderLinkIndex = 9999;
                double[] VelocityLinks = new double[this.NumLinks];
                int groundLinkIndex = 9999;
                int inputlinkIndex = 9999;
                for (int i = 0; i < this.NumLinks; i++)
                {
                    if (this.Links[i].IsGround == false)
                    {
                        VelocityLinks[i] = 99999;

                        foreach (var n in this.Links[i].joints)
                        {
                            if (n.TypeOfJoint.ToString() == "P")
                            {
                                sliderLinkIndex = i;
                            }

                        }


                    }
                    else
                        groundLinkIndex = i;

                    if (this.Links[i].name == "input")
                    {
                        inputlinkIndex = i;
                        VelocityLinks[i] = this.InputSpeed;
                    }


                }




                //next step is to determine the angular velocity of each link
                //if there is a slider, then that is not angular velocity but linear velocity

                for (int i = 0; i < this.NumLinks; i++)
                {
                    if (VelocityLinks[i] == 99999)
                    {
                        //i is the link index whose angular velocity is required
                        //we have the ground link
                        //input links usually have name as as "input" 

                        double rx1 = 9999, ry1 = 9999, sx1 = 9999, sy1 = 9999, rx2 = 9999, ry2 = 9999, sx2 = 9999, sy2 = 9999;

                        for (int j = 0; j < iii; j++)
                        {

                            if ((instantCenterIdentifiers[j, 0] == groundLinkIndex && instantCenterIdentifiers[j, 1] == inputlinkIndex) || (instantCenterIdentifiers[j, 0] == inputlinkIndex && instantCenterIdentifiers[j, 1] == groundLinkIndex))
                            {
                                rx1 = instantCenterIdentifiers[j, 2];
                                ry1 = instantCenterIdentifiers[j, 3];
                            }

                            if ((instantCenterIdentifiers[j, 0] == groundLinkIndex && instantCenterIdentifiers[j, 1] == i) || (instantCenterIdentifiers[j, 0] == i && instantCenterIdentifiers[j, 1] == groundLinkIndex))
                            {
                                sx1 = instantCenterIdentifiers[j, 2];
                                sy1 = instantCenterIdentifiers[j, 3];

                            }

                            if ((instantCenterIdentifiers[j, 0] == inputlinkIndex && instantCenterIdentifiers[j, 1] == i) || (instantCenterIdentifiers[j, 0] == i && instantCenterIdentifiers[j, 1] == inputlinkIndex))
                            {
                                sx2 = instantCenterIdentifiers[j, 2];
                                sy2 = instantCenterIdentifiers[j, 3];

                            }
                        }

                        //now that the various values have been obtained. 
                        //next step is to calculate the angular velocities 

                        double xIntercept1 = sx2 - rx1;
                        double yIntercept1 = sy2 - ry1;
                        double xIntercept2 = sx2 - sx1;
                        double yIntercept2 = sy2 - sy1;
                        double angularvelocity1 = 0.0, sliderVelocity = 0.0;


                        if (sliderLinkIndex != i)
                        {
                            //got to account for infinity/infinity or 0/0 cases 

                            if ((double.IsInfinity(xIntercept1) && double.IsInfinity(xIntercept2)))
                                angularvelocity1 = this.InputSpeed;
                            else if ((xIntercept1 == 0.0 && xIntercept2 == 0.0) && (yIntercept1 != 0.0 && yIntercept2 != 0.0))
                                angularvelocity1 = this.InputSpeed * yIntercept1 / yIntercept2;
                            else if ((yIntercept1 == 0.0 && yIntercept2 == 0.0) && (xIntercept1 != 0.0 && xIntercept2 != 0.0))
                                angularvelocity1 = this.InputSpeed * xIntercept1 / xIntercept2;
                            else
                                angularvelocity1 = this.InputSpeed * xIntercept1 / xIntercept2;
                        }
                        else
                        {
                            //if slideangle is 0, then x component is required for slider velocity
                            //if slideangle is 90, then y component is required for slider velocity
                            //if slideangle is none of the above two, the x and y combined account for slider velocity

                            if (slideAngle == 0.0)
                            {
                                sliderVelocity = this.InputSpeed /*omega is k */ * yIntercept1; /*intercept should be j*/

                            }

                            else if (slideAngle == Math.PI / 2)
                            {
                                sliderVelocity = this.InputSpeed /*omega is k */ * xIntercept1; /*intercept should be j*/

                            }
                            else
                            {
                                //magnitude is only saved here

                                sliderVelocity = this.InputSpeed * Math.Sqrt(Math.Pow(xIntercept1, 2) + Math.Pow(yIntercept1, 2));


                            }

                        }


                        //angularvelocity2 = pms.InputSpeed * yIntercept1 / yIntercept2;

                        VelocityLinks[i] = angularvelocity1;

                    }
                }

                ICVelocity.Add(VelocityLinks);

                #endregion


                //find the velocity of a point

                //assume a point 

                //this function can be used to determine the 

                //double[] trialPoint = new double[2] { 6.0693, 8.2725 };

                // double[] trialPoint = new double[2] { 10, 18 };

                //   double[] velocityOfPoint = velocityAtAnyPoint(trialPoint, jointIndices, pms, VelocityLinks, xxx);
                //   System.Diagnostics.Debug.WriteLine(velocityOfPoint[0]);
                //   System.Diagnostics.Debug.WriteLine(velocityOfPoint[1]);



            }

            //assuming we have two coordinates for a line between two instant centers
            //another two coordinates for a line between two instant centers
            //we need to first determine the line between two points 
            //then determine the intersection point between two lines 

            //possibilities - two lines can have an intersection point 
            //two lines can be parallel, in that case, there is no intersection point - the instant center is at infinity


            //  double x1 = 0, y1 = 0, x2 = 0, y2 = 10, x3 = 10, y3 = 20, x4 = 20, y4 = 0;
            //  double x1 = 20, y1 = 0, x2 = 10, y2 = 20, x3 = 0, y3 = 0, x4 = 0, y4 = 10;
            //  double x1 = 0, y1 = 10, x2 = 0, y2 = 0, x3 = 10, y3 = 20, x4 = 20, y4 = 0;
            //   double x1 = 0, y1 = 10, x2 = 0, y2 = 0, x3 = 20, y3 = 0, x4 = 10, y4 = 20;
            //   double x1 = 10, y1 = 20, x2 = 0, y2 = 10, x3 = 20, y3 = 0, x4 = 0, y4 = 0;
            // double x1 = 20, y1 = 0, x2 = 0, y2 = 0, x3 = 0, y3 = 10, x4 = 10, y4 = 20;
            //double x1 = 0, y1 = 0, x2 = 0, y2 = 10, x3 = 20, y3 = 0, x4 = 20, y4 = 20;


            #endregion
            for (int i = 0; i < ICVelocity.Count; i++)
            {
                string param = System.Convert.ToString(i + 1);
                output.Add("\n\n\n*****     -----    Velocity Analysis " + param + "     -----     *****");
                for (int k = 1; k < this.NumLinks; k++)
                {
                    string icvel = System.Convert.ToString(ICVelocity[i][k]);
                    this.JointParameters.ICVel = ICVelocity;
                    output.Add(this.Joints[k].Link1.name + " Link" + " Velocity is " + icvel + " rads/s");
                }
            }

            return output;

        }

        public string ShinySets(List<double> ax, List<double> ay, List<double> bx, List<double> by)
        {
            string output = "";
            if (!(ax.Count == 3 && ay.Count() == 3 && bx.Count() == 3 && by.Count() == 3))
            {
                return "Error: Need exactly 3 positions.";
            }

            double m01a, m01ai, m01b, m01bi, m12a, m12ai, m12b, m12bi;
            double m01ax, m01ay, m12ax, m12ay, m01bx, m01by, m12bx, m12by;
            double b01a, b01b, b12a, b12b;
            double xaInt = 0.0;
            double yaInt = 0.0;
            double xbInt = 0.0;
            double ybInt = 0.0;

            m01a = FindSlope(ax[0], ay[0], ax[1], ay[1]);
            m01ai = FindSlopeInverse(m01a);
            m01ax = Avg(ax[0], ax[1]);
            m01ay = Avg(ay[0], ay[1]);
            b01a = FindYint(m01ax, m01ay, m01ai);

            m12a = FindSlope(ax[1], ay[1], ax[2], ay[2]);
            m12ai = FindSlopeInverse(m12a);
            m12ax = Avg(ax[1], ax[2]);
            m12ay = Avg(ay[1], ay[2]);
            b12a = FindYint(m12ax, m12ay, m12ai);

            //need intersection of y = m01ai * x + b01a and y = m12ai * x + b12a
            SolveIntersection(m01ai, m12ai, b01a, b12a, ref xaInt, ref yaInt);


            m01b = FindSlope(bx[0], by[0], bx[1], by[1]);
            m01bi = FindSlopeInverse(m01b);
            m01bx = Avg(bx[0], bx[1]);
            m01by = Avg(by[0], by[1]);
            b01b = FindYint(m01bx, m01by, m01bi);

            m12b = FindSlope(bx[1], by[1], bx[2], by[2]);
            m12bi = FindSlopeInverse(m12b);
            m12bx = Avg(bx[1], bx[2]);
            m12by = Avg(by[1], by[2]);
            b12b = FindYint(m12bx, m12by, m12bi);

            SolveIntersection(m01bi, m12bi, b01b, b12b, ref xbInt, ref ybInt);

            // now the joints should be:
            // Ground:  (xaInt, yaInt), (xbInt, ybInt)
            // Input:   (xaInt, yaInt), (ax[0], ay[0])
            // Coupler: (ax[0], ay[0]), (bx[0], by[0])
            // Rocker:  (bx[0], by[0]), (xbInt, ybInt)
            //https://designengrlab.github.io/PMKS/pmks.html?mech=ground,input,R,-10.120,10.880,tfft|input,coupler,R,0.000,4.000,tfff|coupler,rocker,R,0.000,8.000,tfff|rocker,ground,R,-10.470,8.640,tfff|
            output += "Link: https://designengrlab.github.io/PMKS/pmks.html?mech=";
            output += "ground,input,R," + xaInt.ToString() + "," + yaInt.ToString() + ",tfft|";
            output += "input,coupler,R," + ax[0].ToString() + "," + ay[0].ToString() + ",tfff|";
            output += "coupler,rocker,R," + bx[0].ToString() + "," + by[0].ToString() + ",tfff|";
            output += "rocker,ground,R," + xbInt.ToString() + "," + ybInt.ToString() + ",tfff| \n";

            output += "Ground:  (" + xaInt.ToString() + ", " + yaInt.ToString() + "), (" + xbInt.ToString() + ", " + ybInt.ToString() + ")\n";
            output += "Input:   (" + xaInt.ToString() + ", " + yaInt.ToString() + "), (" + ax[0].ToString() + ", " + ay[0].ToString() + ")\n";
            output += "Coupler: (" + ax[0].ToString() + ", " + ay[0].ToString() + "), (" + bx[0].ToString() + ", " + by[0].ToString() + ")\n";
            output += "Rocker:  (" + bx[0].ToString() + ", " + by[0].ToString() + "), (" + xbInt.ToString() + ", " + ybInt.ToString() + ")\n";

            return output;
        }

        private double Avg(double a, double b)
        {
            return (a + b) / 2.0;
        }

        private void SolveIntersection(double m1, double m2, double b1, double b2, ref double x, ref double y)
        {
            if (DoubleEquivalency(m1, m2))
            {
                if (DoubleEquivalency(b1, b2))
                {
                    x = Double.NegativeInfinity;
                    y = Double.NegativeInfinity;
                }
                else
                {
                    x = Double.NaN;
                    y = Double.NaN;
                }
            }
            else if (Double.IsPositiveInfinity(m1))
            {
                x = b1;
                y = m2 * x + b2;
            }
            else if (Double.IsPositiveInfinity(m2))
            {
                x = b2;
                y = m1 * x + b1;
            }
            else
            {
                x = (-1.0 * (b1 - b2)) / (m1 - m2);
                y = m1 * x + b1;
            }
        }

        public List<string> SWexcel()
        {
            AssignBasicLayers();
            CollisionFreeLayers();
            LengthTypeProvider ltp = new LengthTypeProvider();

            List<string> mechData = new List<string>();
            mechData.Add("Planar Mechanism");
            mechData.Add("\nLinks Material:\t" + this.LinkMaterialName); 
            mechData.Add("Pin Material:\t" + this.PinMaterialName);
            mechData.Add("Units:\t" + ltp.lengthTypeList[this.lenIndex]);
            mechData.Add("\nName\tLayer\tWidth\tDepth\tPin Diameter\tJoint X\tJoint Y\tJoint Type");
            for (int i = 0; i < this.Links.Count(); i++)
            {
                var l = this.Links[i];
                mechData.Add(l.name + "\t" + l.Layer.ToString() + "\t" + this.width.ToString() + "\t" + this.depth.ToString() + "\t" + this.pinDiam.ToString());
                for (int k = 0; k < l.joints.Count(); k++)
                {
                    var j = l.joints[k];
                    mechData.Add("\t\t\t\t\t" + j.xInitial.ToString() + "\t" + j.yInitial.ToString() + "\t" + j.TypeOfJoint.ToString());
                }
                mechData.Add(" ");
            }
            return mechData;
        }
    }
    public class LengthTypeProvider
    {
        public List<string> lengthTypeList
        {
            get
            {
                return new List<string> { "Meters (m)", "Centimeters (cm)", "Millimeters (mm)", "Feet (ft)", "Inches (in)" };
            }
        }
    }
    public class MassTypeProvider
    {
        public List<string> massTypeList
        {
            get
            {
                return new List<string> { "Kilograms (kg)", "grams (g)", "Pounds-Mass (lbm)" };
            }
        }
    }
    public class DensityTypeProvider
    {
        public List<string> densityTypeList
        {
            get
            {
                return new List<string> { "kg/m^3", "g/cm^3", "lbm/in^3" };
            }
        }
    }
    public class PressureTypeProvider
    {
        public List<string> pressureTypeList
        {
            get
            {
                return new List<string> { "Mega Pascals (MPa)", "Pounds-Force per Square Inch (PSI)" };
            }
        }
    }
    public class AngleTypeProvider
    {
        public List<string> angleTypeList
        {
            get
            {
                return new List<string> { "Radians (rad)", "Degrees (deg)" };
            }
        }
    }
}