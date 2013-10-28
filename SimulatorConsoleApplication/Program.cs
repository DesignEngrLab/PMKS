using System;

namespace SimulatorConsoleApplication
{
    class Program
    {
        static void Main(string[] args)
        {
            //            var data = "ground,input,R,0,0\n"+
            //"input,coupler,R,0,0.098\n"+
            //"coupler,follower,R,0.224,0.333\n"+
            //"follower,one,R,0.528,0.265\n"+
            //"one,ground,R,0.48,0.071\n"+
            //"coupler,a,R,0.165,0.603\n"+
            //"follower,a,R,0.376,0.469";


            //var data = "ground,input,R,0,0\n"
            //+ "input,coupler,R,0,0.98\n"
            //+ "coupler,follower,R,2.24,3.33\n"
            //+ "follower,one,R,5.28,2.65\n"
            //+ "one,ground,R,4.8,0.71\n"
            //+ "coupler,a,R,1.65,6.03\n"
            //+ "follower,a,R,3.76,4.69\n";
            var data = "ground,input,R,0,350\n"
            + "input,coupler,R,0,450\n"
            + "coupler,follower,R,120, 370\n"
            + "follower,ground,P,120,300";
            // http://people.oregonstate.edu/~campmatt/pmks.html?mech=ground input R 0 350 ---+|input coupler R 0 450 ----|coupler follower R 120 370 ----|follower ground P 120 300 0 ----|


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
