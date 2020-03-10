using DotNetForHtml5;
using Microsoft.AspNetCore.Blazor.Hosting;
using System.Windows;

namespace PMKS_OpenSilverApp.Browser
{
    public class Program
    {
        public static void Main(string[] args)
        {
            Cshtml5Initializer.Initialize();
            IWebAssemblyHost host = CreateHostBuilder(args).Build();
            host.Run();
            host.Dispose();
            Application.RunApplication(() =>
            {
                var app = new PMKS_OpenSilverApp.App();
            });
        }

        public static IWebAssemblyHostBuilder CreateHostBuilder(string[] args) =>
            BlazorWebAssemblyHost.CreateDefaultBuilder()
                .UseBlazorStartup<Startup>();
    }
}
