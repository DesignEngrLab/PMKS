namespace PlanarMechanismSimulator
{
    internal class circleDiagramItem
    {
        internal readonly link link1;
        internal readonly link link2;
        internal readonly joint immediateJoint;

        internal double speed = double.NaN;
        internal double angularAccel = double.NaN;

        internal bool found; 
        internal point location;

        public circleDiagramItem(link link1, link link2, joint joint)
        {
            this.link1 = link1;
            this.link2 = link2;
            this.immediateJoint = joint;
        }
    }
}