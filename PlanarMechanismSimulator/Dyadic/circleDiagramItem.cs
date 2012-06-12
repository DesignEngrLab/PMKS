namespace PlanarMechanismSimulator
{
    internal class circleDiagramItem
    {
        internal double alpha = double.NaN; //angular acceleration
        internal link link1;
        internal link link2;
        internal joint p;
        internal double speed = double.NaN;
        internal double x = double.NaN;
        internal double y = double.NaN;
    }
}