using System;
using static System.IO.Path;

namespace SL3Reader
{
    public static class Program
    {
        public static void Main(string[] args)
        {
            if (args.Length != 2)
            {
                Console.WriteLine(@"Usage example: SL3Reader.exe C:\input.sl3 D:\output.csv");
                return;
            }

            string input = GetFullPath(args[0]);
            if(!Exists(input))
            {
                Console.WriteLine("Input file not found!");
                return;
            }

            string output = GetFullPath(args[1]);
            if (!Exists(GetDirectoryName(output)))
            {
                Console.WriteLine("Directory not found for the output file!");
                return;
            }

            using SL3Reader sl3reader = new(input);
            //sl3reader.ExamineUnknown8Datasets();
            //sl3reader.Export3D(output);
            //sl3reader.ExportImagery(@"F:\SS\", SurveyType.DownScan);
            sl3reader.AugmentTrajectory();
            sl3reader.ExportToCSV(output, false);
        }
    }
}