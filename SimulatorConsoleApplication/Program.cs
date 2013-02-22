using System;

namespace SimulatorConsoleApplication
{
    class Program
    {
        static void Main(string[] args)
        {
            var data = "gnd input r 0.0 0.0 \n"
                       + "input coupler rp 0.0 1.0 0\n"
                       + " coupler R 12.0 0.0 \n"
                       + "coupler ground r 5.0 0\n";
            var pms = new PlanarMechanismSimulator.Simulator(data);
            pms.InputSpeed = 123.0;
            pms.MaxSmoothingError = 0.01;
            Console.WriteLine("start");
            pms.FindFullMovement();
            
            Console.WriteLine("done");
            Console.ReadKey();
        }
    }
}
