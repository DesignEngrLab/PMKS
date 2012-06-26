using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Office.Tools.Ribbon;
using PlanarMechanismSimulator;

namespace ExcelPlanarMechSimulator
{
    public partial class MechSimRibbon
    {
        private void MechSimRibbon_Load(object sender, RibbonUIEventArgs e)
        {
            button_Simulate.Enabled = false;
        }

        protected Simulator pms { get; set; }

        private void button_Parse_Click(object sender, RibbonControlEventArgs e)
        {
            try
            {
                Globals.Sheet1.Range["j2"].Value2 = "?";
                Globals.Sheet1.Range["h4"].Value2 = "";
                button_Simulate.Enabled = true;
                var rng = Globals.Sheet1.Range["B3:F22"];
                object[,] data = rng.Value2;
                var numRows = data.GetLength(0);
                var LinkIDs = new List<List<string>>();
                var typList = new List<string>();
                var Positions = new List<double[]>();
                for (int i = 0; i < data.GetLength(0); i++)
                {
                    if ((data[i + 1, 1] == null) || (string.IsNullOrWhiteSpace(data[i + 1, 1].ToString()))) break;
                    typList.Add(data[i + 1, 1].ToString());
                    if (typList[i].Equals("p", StringComparison.InvariantCultureIgnoreCase)
                        || typList[i].Equals("rp", StringComparison.InvariantCultureIgnoreCase))
                    {
                        double Xtemp, Ytemp, angleTemp;
                        if (data[i + 1, 2] != null && double.TryParse(data[i + 1, 2].ToString(), out Xtemp)
                            && data[i + 1, 3] != null && double.TryParse(data[i + 1, 3].ToString(), out Ytemp)
                            && data[i + 1, 4] != null && double.TryParse(data[i + 1, 4].ToString(), out angleTemp))
                            Positions.Add(new[] { angleTemp, Xtemp, Ytemp });
                        else throw new Exception("Numerical data required (x, y, and angle) for joint at row " + (i + 1).ToString());
                    }
                    else
                    {
                        double Xtemp, Ytemp;
                        if (data[i + 1, 2] != null && double.TryParse(data[i + 1, 2].ToString(), out Xtemp)
                            && data[i + 1, 3] != null && double.TryParse(data[i + 1, 3].ToString(), out Ytemp))
                            Positions.Add(new[] { Xtemp, Ytemp });
                        else throw new Exception("Numerical data required (x, and y) for joint at row " + (i + 1).ToString());
                    }
                    if (data[i + 1, 5] != null) LinkIDs.Add(data[i + 1, 5].ToString().Split(',', ' ').ToList());
                    else throw new Exception("One or more link names required for joint at row " + (i + 1).ToString());

                }
                pms = new Simulator(LinkIDs, typList, Positions);

                if (pms.IsDyadic) Globals.Sheet1.Range["h3"].Value2 = "The mechanism is comprised of only of dyads.";
                else Globals.Sheet1.Range["h3"].Value2 = "The mechanism has non-dyadic loops.";
                int dof = pms.DegreesOfFreedom;
                Globals.Sheet1.Range["j2"].Value2 = dof;
                if (dof == 1)
                {
                    button_Simulate.Enabled = true;
                }
                else
                {
                    Globals.Sheet1.Range["h4"].Value2 = "Cannot simulate mechanisms with degrees of freedom greater than"
                                                        + " or less than 1. ";
                    button_Simulate.Enabled = false;
                    // Globals.Sheet1.Range["j2"].Style //trying to change background color.
                }

            }
            catch (Exception exc)
            {
                Globals.Sheet1.Range["h4"].Value2 += "Error in input data: " + exc.Message;
                button_Simulate.Enabled = false;
            }
            finally
            {
                if (button_Simulate.Enabled)
                    Globals.Sheet1.Range["h4"].Value2 += " Ready to simulate.";
            }
        }

        private void button_Simulate_Click(object sender, RibbonControlEventArgs e)
        {
            var rpm = double.NaN;
            if (Globals.Sheet1.Range["n2"].Value2 == null || !double.TryParse(Globals.Sheet1.Range["n2"].Value2.ToString(), out rpm))
                rpm = 10.0;
            Globals.Sheet1.Range["n2"].Value2 = rpm;
            pms.InputSpeed = 2 * Math.PI * rpm / 60.0;

            var deltaAngle = double.NaN;
            if (Globals.Sheet1.Range["r2"].Value2 == null || !double.TryParse(Globals.Sheet1.Range["r2"].Value2.ToString(), out deltaAngle))
                deltaAngle = 1.0;
            Globals.Sheet1.Range["r2"].Value2 = deltaAngle;
            pms.DeltaAngle = Math.PI * deltaAngle / 180.0;


            pms.FindFullMovement();
            button_Simulate.Enabled = false;
            SortedList<double, double[,]> linkParameters = pms.LinkParameters;
            SortedList<double, double[,]> jointParameters = pms.JointParameters;
            //todo: add to 2nd and 3rd sheets of spreadsheet
            Globals.Sheet2.Range["a1:c4"].Value = linkParameters[0.0];
            // hanxu
        }


        private void editBox_speed_TextChanged(object sender, RibbonControlEventArgs e)
        {

        }

        private void box_incrementType_TextChanged(object sender, RibbonControlEventArgs e)
        {

        }

        private void editBox_incrementValue_TextChanged(object sender, RibbonControlEventArgs e)
        {

        }

        private void editBox_numSteps_TextChanged(object sender, RibbonControlEventArgs e)
        {

        }

    }
}
