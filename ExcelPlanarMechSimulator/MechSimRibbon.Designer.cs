using System.Collections.Generic;
using Microsoft.Office.Tools.Ribbon;

namespace ExcelPlanarMechSimulator
{
    partial class MechSimRibbon : Microsoft.Office.Tools.Ribbon.RibbonBase
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        public MechSimRibbon()
            : base(Globals.Factory.GetRibbonFactory())
        {
            InitializeComponent();
        }

        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.tab1 = this.Factory.CreateRibbonTab();
            this.group2 = this.Factory.CreateRibbonGroup();
            this.editBox_speed = this.Factory.CreateRibbonEditBox();
            this.box_incrementType = this.Factory.CreateRibbonComboBox();
            this.editBox_incrementValue = this.Factory.CreateRibbonEditBox();
            this.editBox_numSteps = this.Factory.CreateRibbonEditBox();
            this.group1 = this.Factory.CreateRibbonGroup();
            this.button_Parse = this.Factory.CreateRibbonButton();
            this.button_Simulate = this.Factory.CreateRibbonButton();
            this.tab1.SuspendLayout();
            this.group2.SuspendLayout();
            this.group1.SuspendLayout();
            // 
            // tab1
            // 
            this.tab1.ControlId.ControlIdType = Microsoft.Office.Tools.Ribbon.RibbonControlIdType.Office;
            this.tab1.Groups.Add(this.group2);
            this.tab1.Groups.Add(this.group1);
            this.tab1.Label = "Mech Sim";
            this.tab1.Name = "tab1";
            // 
            // group2
            // 
            this.group2.Items.Add(this.editBox_speed);
            this.group2.Items.Add(this.box_incrementType);
            this.group2.Items.Add(this.editBox_incrementValue);
            this.group2.Items.Add(this.editBox_numSteps);
            this.group2.Label = "define input";
            this.group2.Name = "group2";
            this.group2.Visible = false;
            // 
            // editBox_speed
            // 
            this.editBox_speed.Label = "speed [rpm]:";
            this.editBox_speed.Name = "editBox_speed";
            this.editBox_speed.Text = "10.0";
            this.editBox_speed.TextChanged += new Microsoft.Office.Tools.Ribbon.RibbonControlEventHandler(this.editBox_speed_TextChanged);
            // 
            // box_incrementType
            // 
            this.box_incrementType.Label = "increment:";
            this.box_incrementType.Name = "box_incrementType";
            this.box_incrementType.Text = "angle";
            this.box_incrementType.TextChanged += new Microsoft.Office.Tools.Ribbon.RibbonControlEventHandler(this.box_incrementType_TextChanged);
            // 
            // editBox_incrementValue
            // 
            this.editBox_incrementValue.Label = "value:";
            this.editBox_incrementValue.Name = "editBox_incrementValue";
            this.editBox_incrementValue.Text = "0.1";
            this.editBox_incrementValue.TextChanged += new Microsoft.Office.Tools.Ribbon.RibbonControlEventHandler(this.editBox_incrementValue_TextChanged);
            // 
            // editBox_numSteps
            // 
            this.editBox_numSteps.Label = "number of steps";
            this.editBox_numSteps.Name = "editBox_numSteps";
            this.editBox_numSteps.Text = null;
            this.editBox_numSteps.TextChanged += new Microsoft.Office.Tools.Ribbon.RibbonControlEventHandler(this.editBox_numSteps_TextChanged);
            // 
            // group1
            // 
            this.group1.Items.Add(this.button_Parse);
            this.group1.Items.Add(this.button_Simulate);
            this.group1.Label = "Actions";
            this.group1.Name = "group1";
            // 
            // button_Parse
            // 
            this.button_Parse.ControlSize = Microsoft.Office.Core.RibbonControlSize.RibbonControlSizeLarge;
            this.button_Parse.Label = "Parse and Check";
            this.button_Parse.Name = "button_Parse";
            this.button_Parse.ShowImage = true;
            this.button_Parse.Click += new Microsoft.Office.Tools.Ribbon.RibbonControlEventHandler(this.button_Parse_Click);
            // 
            // button_Simulate
            // 
            this.button_Simulate.ControlSize = Microsoft.Office.Core.RibbonControlSize.RibbonControlSizeLarge;
            this.button_Simulate.Label = "Simulate";
            this.button_Simulate.Name = "button_Simulate";
            this.button_Simulate.ShowImage = true;
            this.button_Simulate.Click += new Microsoft.Office.Tools.Ribbon.RibbonControlEventHandler(this.button_Simulate_Click);
            // 
            // MechSimRibbon
            // 
            this.Name = "MechSimRibbon";
            this.RibbonType = "Microsoft.Excel.Workbook";
            this.Tabs.Add(this.tab1);
            this.Load += new Microsoft.Office.Tools.Ribbon.RibbonUIEventHandler(this.MechSimRibbon_Load);
            this.tab1.ResumeLayout(false);
            this.tab1.PerformLayout();
            this.group2.ResumeLayout(false);
            this.group2.PerformLayout();
            this.group1.ResumeLayout(false);
            this.group1.PerformLayout();

        }

        #endregion

        internal Microsoft.Office.Tools.Ribbon.RibbonTab tab1;
        internal Microsoft.Office.Tools.Ribbon.RibbonGroup group1;
        internal Microsoft.Office.Tools.Ribbon.RibbonButton button_Parse;
        internal Microsoft.Office.Tools.Ribbon.RibbonButton button_Simulate;
        internal Microsoft.Office.Tools.Ribbon.RibbonGroup group2;
        internal Microsoft.Office.Tools.Ribbon.RibbonEditBox editBox_incrementValue;
        internal Microsoft.Office.Tools.Ribbon.RibbonEditBox editBox_speed;
        internal Microsoft.Office.Tools.Ribbon.RibbonComboBox box_incrementType;
        internal Microsoft.Office.Tools.Ribbon.RibbonEditBox editBox_numSteps;
    }

    partial class ThisRibbonCollection
    {
        internal MechSimRibbon MechSimRibbon
        {
            get { return this.GetRibbon<MechSimRibbon>(); }
        }
    }
}
