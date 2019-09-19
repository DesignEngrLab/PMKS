using System;
using System.IO;

namespace SimulatorConsoleApplication
{
    class Program
    {
        static void Main(string[] args)
        {
            var data = "ground,input,R,0,350\n"
            + "input,coupler,R,0,450\n"
            + "coupler,follower,R,120, 370\n"
            + "follower,ground,P,120,300\n"
            + "d=0\n"
            + "e=0.01\n";
           
            var pms = new PMKS.Simulator(data);

            // properties, like InputSpeed can, of course, be set programmatically as well.
            pms.InputSpeed = 123.0;
                        
            Console.WriteLine("start");
            pms.FindFullMovement();
            Console.WriteLine("done");
            Console.ReadKey();
        }
    }
}
