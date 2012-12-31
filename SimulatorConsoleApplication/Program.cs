namespace SimulatorConsoleApplication
{
    class Program
    {
        static void Main(string[] args)
        {
            var data = "gnd input r 0.0 0.0 \n"
                       + "input coupler r 0.0 1.0 \n"
                       + "output coupler R 12.0 12.0 \n"
                       + "output ground r 12.0 0\n";
            var pms = new PlanarMechanismSimulator.Simulator(data);
            pms.InputSpeed = 123.0;
            pms.FindFullMovement();
        }
    }
}
