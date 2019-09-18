using PMKS;
using System;
using System.IO;
using System.Reflection;

namespace JsonImportExport
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("start");
            Simulator pms;
            var dirInfo = new DirectoryInfo(Assembly.GetExecutingAssembly().Location);
            var dir = dirInfo.Parent.Parent.Parent.Parent.FullName + Path.DirectorySeparatorChar;

            using (var stream = File.OpenRead(dir + "4bar1.pmks"))
            {
                pms = PMKS.Simulator.CreateFromJsonStream(stream);
                pms.FindFullMovement();
            }
            using (var stream = File.OpenWrite(dir + "4bar1.output.pmks"))
            {
                pms.StoreJson(stream);
            }
            Console.WriteLine("done");
            Console.ReadKey();
        }
    }
}
