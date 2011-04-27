
using System;
using System.Collections.Generic;
using System.Windows;
using GraphSynth.Representation;
using OptimizationToolbox;
using PlanarMechanismSimulator;
using System.IO;

namespace MechSynth
{
    public class ComparePathWithDesired : IObjectiveFunction
    {
        private readonly MechSimulation sim;
        private readonly double[,] desiredPath;
        private readonly candidate c;


        public ComparePathWithDesired(candidate c, double[,] desiredPath, MechSimulation sim)
        {
            this.c = c;
            this.desiredPath = desiredPath;
            this.sim = sim;

        }
        public double calculate(double[] x)
        {
            //double checkSum = x[0] + x[1]*10 + x[2]*100 + x[3]*1000;
            //SearchIO.output(checkSum, 0);
            //the code below will work for Single output conditions
            //now store the pivot parameters into an array or we could directly manipulate it
            //the program is going to directly compare from index 0 to the last without trying to identify the closest one
            //will this strategy work?
            //distance function required
            //that could be incorporated into the Math library
            double[,] output = new double[1000, 2];
            double[,] otherpivot = new double[15, 2];
            double[,] actualoutput = new double[15, 2];
            double[,] InputGround = new double[15, 2];
            double[,] OtherGround = new double[15, 2];
            double[,] Trial = new double[15, 2];
            double[,] Trial2 = new double[15, 2];
            //store pivots separately

            //compare each desired point in the given order with the output obtained
            //15 points to be compared with 1000 points

            List<node> pivot_compare = new List<node>();

            foreach (node n in c.graph.nodes)
                if (n.localLabels.Contains("pivot"))
                    pivot_compare.Add(n);
            int outputpivotindex = 0;
            int inputground = 0, otherground = 0, trial = 0, trial2 = 0;
            for (int i = 0; i < pivot_compare.Count; i++)
                if (pivot_compare[i].localLabels.Contains("output"))
                    outputpivotindex = i;
            for (int i = 0; i < pivot_compare.Count; i++)
                if (pivot_compare[i].localLabels.Contains("input") && pivot_compare[i].localLabels.Contains("ground"))
                    inputground = i;
            for (int i = 0; i < pivot_compare.Count; i++)
                if (!pivot_compare[i].localLabels.Contains("input") && pivot_compare[i].localLabels.Contains("ground"))
                    otherground = i;
            for (int i = 0; i < pivot_compare.Count; i++)
                if (pivot_compare[i].localLabels.Contains("trial"))
                    trial = i;
            for (int i = 0; i < pivot_compare.Count; i++)
                if (pivot_compare[i].localLabels.Contains("trial2"))
                    trial2 = i;

            for (int i = 0; i < sim.PivotParameters.GetLength(1); i++)
            {
                output[i, 0] = sim.PivotParameters[outputpivotindex, i, 0];
                output[i, 1] = sim.PivotParameters[outputpivotindex, i, 1];

            }


            //now we have the output pivot index
            //we also have the sim.PivotParameters from where we would be able to obtain the 1000 points

            //take the first point, search through the list of 1000 points, find whichever is closest -say within a particular tolerance
            //make a note of the point..say i
            //coz next desired path point will be searched from i



            int number = 0;
            int getnostoredinoutput = 0;
            

            for (int i = 0; i < desiredPath.GetLength(0); i++)
            {
                for (int j = number; j < output.GetLength(0); j++)
                {
                    
                    if ((desiredPath[i,0]/output[j,0]>=0.8 || desiredPath[i, 0] / output[j, 0] <= 1.3) && (desiredPath[i,1]/output[j,1]>=0.8 || desiredPath[i, 1] / output[j, 1] <= 1.3))
                    {
                        actualoutput[i, 0] = output[j, 0];
                        actualoutput[i, 1] = output[j, 1];
                        number = j+1;
                        getnostoredinoutput += 1;
                        break;
                    }

                }

                InputGround[i, 0] = sim.PivotParameters[inputground, number - 1, 0];
                InputGround[i, 1] = sim.PivotParameters[inputground, number - 1, 1];
                Trial[i, 0] = sim.PivotParameters[trial, number - 1, 0];
                Trial[i, 1] = sim.PivotParameters[trial, number - 1, 1];
                Trial2[i, 0] = sim.PivotParameters[trial2, number - 1, 0];
                Trial2[i, 1] = sim.PivotParameters[trial2, number - 1, 1];
                OtherGround[i, 0] = sim.PivotParameters[otherground, number - 1, 0];
                OtherGround[i, 1] = sim.PivotParameters[otherground, number - 1, 1];

            }

            //even if we get 15 or less, we can compute RMS Number

            double rm_s = rmsdistance(actualoutput);

            #region old code
            

            //trying to find the other pivot connected to the input

            //int otherpivotindex = 0;

            //for (int i = 0; i < pivot_compare.Count; i++)
            //    if (!pivot_compare[i].localLabels.Contains("ground") && !pivot_compare[i].localLabels.Contains("output") && !pivot_compare[i].localLabels.Contains("input"))
            //        otherpivotindex = i;

            //for (int i = 0; i < sim.PivotParameters.GetLength(1); i++)
            //{
            //    otherpivot[i, 0] = sim.PivotParameters[outputpivotindex, i, 0];
            //    otherpivot[i, 1] = sim.PivotParameters[outputpivotindex, i, 1];
            //}

            //for (int i = 0; i < sim.PivotParameters.GetLength(1); i++)
            //{
            //    output[i, 0] = sim.PivotParameters[outputpivotindex, i, 0];
            //    output[i, 1] = sim.PivotParameters[outputpivotindex, i, 1];

            //}
           

            //printing pivot positions for each cycle: 
            if (rm_s < 100)
            {
                FileStream fs = new FileStream(rm_s + ".txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);
                using (StreamWriter sw = new StreamWriter(fs))
                {
                    sw.Write("Output X" + "\t");
                    sw.Write("Output Y" + "\t");
                    sw.Write("InputGround X" + "\t");
                    sw.Write("InputGround Y" + "\t");
                    sw.Write("OtherGround X" + "\t");
                    sw.Write("OtherGround Y" + "\t");
                    sw.Write("Trial X" + "\t");
                    sw.Write("Trial Y" + "\t");
                    sw.Write("Trial2 X" + "\t");
                    sw.Write("Trial2 Y" + "\t");

                  //  sw.Write("Other Pivot X" + "\t");
                  //  sw.Write("Other Pivot Y" + "\t");
                 //   sw.Write("RMS" + "\t");
                    sw.WriteLine();
                    for (int i = 0; i < actualoutput.GetLength(0); i++)
                    {

                        sw.Write(actualoutput[i, 0] + "\t");
                        sw.Write(actualoutput[i, 1] + "\t");
                        sw.Write(InputGround[i, 0] + "\t");
                        sw.Write(InputGround[i, 1] + "\t");
                        sw.Write(OtherGround[i, 0] + "\t");
                        sw.Write(OtherGround[i, 1] + "\t");
                        sw.Write(Trial[i, 0] + "\t");
                        sw.Write(Trial[i, 1] + "\t");
                        sw.Write(Trial2[i, 0] + "\t");
                        sw.Write(Trial2[i, 1] + "\t");
                    //    sw.Write(otherpivot[i, 0] + "\t");
                    //    sw.Write(otherpivot[i, 0] + "\t");
                  //      sw.Write(rm_s + "\t");
                    }
                }
                
            }

            #endregion 
            return rm_s;
        }

        private double rmsdistance(double[,] output)
        {
            //now that there are two arrays of the same size, we shall determine the straightline distance between two points

            double rms = 0.0;

            for (int i = 0; i < desiredPath.GetLength(0); i++)
            {

                rms += Math.Pow((desiredPath[i, 0] - output[i, 0]), 2) + Math.Pow((desiredPath[i, 1] - output[i, 1]), 2);

            }

            rms /= desiredPath.GetLength(0);
            rms = Math.Sqrt(rms);

            SearchIO.output("rms = " + rms, 0);


            return rms;
        }

        //private double rmsdistance2(double[,] output)
        //{
        //    //now that there are two arrays of the same size, we shall determine the straightline distance between two points

        //    double rms = 0.0;
        //    int numOutRows = output.GetLength(0);
        //    for (int i = 0; i < desiredPath.GetLength(0); i++)
        //    {
        //        var desiredPt = new Point(desiredPath[i, 0], desiredPath[i, 1]);
        //        var pt1 = new Point(output[numOutRows - 1, 0], output[numOutRows - 1, 1]);
        //        var pt2 = new Point(output[0, 0], output[0, 1]);
        //        double MinDx = getShortestDistance(pt1, pt2, desiredPt);
        //        for (int j = 1; j < numOutRows; j++)
        //        {
        //            pt1 = new Point(output[j - 1, 0], output[j - 1, 1]);
        //            pt2 = new Point(output[j, 0], output[j, 1]);
        //            double tempMin = getShortestDistance(pt1, pt2, desiredPt);
        //            if (tempMin < MinDx) MinDx = tempMin;
        //        }
        //        rms += MinDx;
        //    }
        //    rms /= desiredPath.GetLength(0);
        //    rms = Math.Sqrt(rms);

        //    SearchIO.output("rms = " + rms, 0);


        //    return rms;
        //}

        private double getShortestDistance(Point pt1, Point pt2, Point desiredPt)
        {
            double minLength = (pt1 - desiredPt).LengthSquared;
            minLength = Math.Min(minLength, (pt2 - desiredPt).LengthSquared);

            Vector unitLeft = (desiredPt - pt1);
            unitLeft.Normalize();
            Vector unitRight = (pt2 - pt1);
            unitRight.Normalize();
            double theta = Math.Acos(unitLeft * unitRight);
            if (theta > Math.PI / 2) return minLength;

            unitLeft = (pt1 - pt2);
            unitLeft.Normalize();
            unitRight = (desiredPt - pt2);
            unitRight.Normalize();
            theta = Math.Acos(unitLeft * unitRight);
            if (theta > Math.PI / 2) return minLength;


            // then compare with the intermediate point
            // make sure that return length-squared not length
            return minLength;
        }

        //public double[,] returnPath()
        //{
        //    var path = new double[numTimeSteps, 2];
        //    testfunction();


        //    for (int j = 0; j < numTimeSteps; j++)
        //    {
        //        path[j, 0] = sim.pivotParameters[2, j, 0];
        //        path[j, 1] = sim.pivotParameters[2, j, 1];

        //    }

        //    return path;
        //}

    }
}
