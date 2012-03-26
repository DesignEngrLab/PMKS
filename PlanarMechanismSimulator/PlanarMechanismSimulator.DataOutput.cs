using System;
using System.Collections.Generic;
using System.IO;
using OptimizationToolbox;

namespace PlanarMechanismSimulator
{
    public partial class PlanarMechanismSimulator : IDependentAnalysis
    {
        private enum status
        {
            normal,
            ICsNotFound,
            PositionNotDyadic,
            PositionRotabilityViolated
        } ;
        #region Save Output Parameter Data
        internal void saveParameterData(string filename)
        {
            testfunction();
            StreamWriter stream = File.AppendText(filename + ".xls");
            for (int i = 0; i < numTimeSteps; i++)
            {
                stream.Write(path[i, 0] + "\t");
                stream.Write(path[i, 1] + "\t");
                stream.WriteLine();

            }
        }
        #endregion


        #region Function: Print Details
        private void PrintDetails(int timeRow, List<pivot> pivots, List<link> links, StreamWriter file)
        {
            //we shall output individual pivot parameters in the format below:
            // pivot --->  Labels, X, Y, velocityx, velocityy, acc X, acc Y
            //link ---> labels, ang v, ang acc
            //this will append the same file for all the time-steps

            for (int i = 0; i < p; i++)
            {
                if (pivots[i].localLabels.Contains("inputloc") /*&& pivots[i].localLabels.Contains("ip") && !pivots[i].localLabels.Contains("ground")*/)
                {
                    // file.Write(pivots[i].localLabels + "\t");
                   // file.Write(PivotParameters[i, timeRow, 0] + "\t");
                  //  file.Write(PivotParameters[i, timeRow, 1] + "\t");
                    //file.Write(PivotParameters[i, timeRow, 2] + "\t");
                    //file.Write(PivotParameters[i, timeRow, 3] + "\t");
                    //file.Write(PivotParameters[i, timeRow, 4] + "\t");
                    //file.Write(PivotParameters[i, timeRow, 5] + "\t");
                    file.WriteLine();
                }
            }

            for (int j = 0; j < n; j++)
            {

                // file.Write(links[j].localLabels + "\t");
                // file.Write(linkParameters[j, timeRow, 0] + "\t");
                // file.Write(linkParameters[j, timeRow, 1] + "\t");
                //  file.WriteLine();

            }

        }


        private void PrintDetails()
        {
            // Printing Positions of pivots 2 and 3


            int rowMax = PivotParameters.GetLength(1);
            int colMax = PivotParameters.GetLength(2);


            int rowMax1 = LinkParameters.GetLength(1);
            int colMax1 = LinkParameters.GetLength(2);

            int whichPivot = 1;
            int whichPivot1 = 2;

            FileStream fs = new FileStream("4bar_position.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);

            using (StreamWriter sw = new StreamWriter(fs))
            {
                for (int i = 0; i < rowMax; i++)
                {
                    for (int j = 0; j < 2; j++)
                        sw.Write(PivotParameters[whichPivot, i, j] + "\t");
                    for (int j = 0; j < 2; j++)
                        sw.Write(PivotParameters[whichPivot1, i, j] + "\t");

                    //for (int j = 0; j < 2; j++)
                    //  sw.Write(pivotParameters[4, i, j] + "\t");
                    //for (int j = 0; j < 2; j++)
                    //    sw.Write(pivotParameters[5, i, j] + "\t");

                    sw.WriteLine();
                }
            }

            //shall get the angle of the input crank
            FileStream fs4 = new FileStream("4bar_angle.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);
            using (StreamWriter sw = new StreamWriter(fs4))
            {
                for (int i = 0; i < rowMax; i++)
                {
                    double x_x = PivotParameters[whichPivot, i, 0];
                    double y_y = PivotParameters[whichPivot, i, 1];



                    //we know that the initial point is 0,0

                    //     double rad_angle = Math.Atan2(y_y, x_x);
                    double rad_angle = Math.Atan2(y_y, x_x);

                    //we shall determine the angle of the other link

                    double xx_1 = PivotParameters[1, i, 0];
                    double yy_1 = PivotParameters[1, i, 1];
                    double xx_2 = PivotParameters[3, i, 0];
                    double yy_2 = PivotParameters[3, i, 1];

                    double rad_angle2 = Math.Atan2((yy_1 - yy_2), (xx_1 - xx_2));

                    sw.Write(rad_angle + "\t");
                    sw.Write(rad_angle2 + "\t");

                    sw.WriteLine();



                }


            }




            //Printing Velocities

            FileStream fs1 = new FileStream("4bar_velocity.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);

            using (StreamWriter sw = new StreamWriter(fs1))
            {
                for (int i = 0; i < rowMax; i++)
                {
                    for (int j = 2; j < 4; j++)
                        sw.Write(PivotParameters[whichPivot, i, j] + "\t");
                    for (int j = 2; j < 4; j++)
                        sw.Write(PivotParameters[whichPivot1, i, j] + "\t");
                    //for (int j = 2; j < 4; j++)
                    //  sw.Write(pivotParameters[4, i, j] + "\t");
                    //for (int j = 0; j < 2; j++)
                    //    sw.Write(pivotParameters[5, i, j] + "\t");
                    sw.WriteLine();
                }
            }



            //Printing Acceleration

            FileStream fs2 = new FileStream("4bar_acceleration.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);

            using (StreamWriter sw = new StreamWriter(fs2))
            {
                for (int i = 0; i < rowMax; i++)
                {
                    for (int j = 4; j < 6; j++)
                        sw.Write(PivotParameters[whichPivot, i, j] + "\t");
                    for (int j = 4; j < 6; j++)
                        sw.Write(PivotParameters[whichPivot1, i, j] + "\t");
                    //for (int j = 2; j < 4; j++)
                    //    sw.Write(pivotParameters[4, i, j] + "\t");
                    //for (int j = 0; j < 2; j++)
                    //    sw.Write(pivotParameters[5, i, j] + "\t");
                    sw.WriteLine();
                }
            }

            //printing omega

            FileStream fs3 = new FileStream("omega_alpha.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);


            using (StreamWriter sw = new StreamWriter(fs3))
            {
                for (int i = 0; i < rowMax1; i++)
                {
                    for (int j = 0; j < 2; j++)
                        sw.Write(LinkParameters[1, i, j] + "\t");
                    for (int j = 0; j < 2; j++)
                        sw.Write(LinkParameters[2, i, j] + "\t");
                    //for (int j = 0; j < 2; j++)
                    //   sw.Write(linkParameters[3, i, j] + "\t");
                    sw.WriteLine();
                }
            }




        }
        #endregion

        #region print details

        private void PrintDetails4Bar()
        {


            FileStream fs = new FileStream("4bar_2pivots.txt", FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);

            using (StreamWriter sw = new StreamWriter(fs))
            {
                sw.Write("Output X" + "\t");
                sw.Write("Output Y" + "\t");
                sw.Write("Other Pivot X" + "\t");
                sw.Write("Other Pivot Y" + "\t");
                sw.Write("RMS" + "\t");
                sw.WriteLine();
                for (int i = 0; i < 12; i++)
                {

                    sw.Write(output[i, 0] + "\t");
                    sw.Write(output[i, 1] + "\t");
                    sw.Write(otherpivot[i, 0] + "\t");
                    sw.Write(otherpivot[i, 0] + "\t");
                    sw.Write(rm_s + "\t");

                }



            }

        }


        #endregion 

    }


}