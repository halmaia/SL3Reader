using System;
using System.IO;

namespace SL3Reader
{
    public static class Program
    {
        public static unsafe void Main(string[] args)
        {
            string input, output;
            if (args.Length != 2 || !File.Exists(input = Path.GetFullPath(args[0])))
            {
                Console.WriteLine(@"Usage example: SL3Reader.exe C:\input.sl3 D:\output.csv");
                return;
            }
            using SL3Reader sl3reader = new(input);
            // Experiment:
            //sl3reader.ExportSideScans(@"F:\SS");
            //sl3reader.ExportInterfereometricDataset(@"F:\SS");

            sl3reader.ExportToCSV(output = Path.GetFullPath(args[1]), false);
        }
    }
}