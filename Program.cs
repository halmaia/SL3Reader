using System;
using static System.IO.Path;

namespace SL3Reader {

    public static class Program {
        public static void Main(string[] args) {
            if (args.Length != 3) {
                PrintUsage();
                return;
            }

            string input = GetFullPath(args[0]);
            if (!Exists(input)) {
                Console.WriteLine("Input file not found!");
                PrintUsage();
                return;
            }

            string output = GetFullPath(args[1]);
            if (!Exists(GetDirectoryName(output))) {
                Console.WriteLine("Directory not found for the output file!");
                PrintUsage();
                return;
            }

            using SL3Reader sl3reader = new(input);

            string expSelector = args[2].Trim().ToLowerInvariant();
            switch (expSelector) {
                case "-3d":
                    sl3reader.Export3D(output);
                    break;
                case "-route":
                    sl3reader.ExportToCSV(output);
                    break;
                default:
                    Console.WriteLine("Invalid third argument (" + expSelector + ").");
                    PrintUsage();
                    return;
            }

            PrintSummary();

            return;

            static void PrintUsage() {
                Console.WriteLine("Usage examples:");
                Console.WriteLine("To export 3D points: SL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -3d");
                Console.WriteLine("To export route:     SL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -route");
            }

            void PrintSummary() {
                Console.WriteLine("Number of frames: " + sl3reader.Frames.Count.ToString());
                Console.WriteLine("Number of 3D frames: " + sl3reader.IndexByType[SurveyType.ThreeDimensional].Count.ToString());
                Console.WriteLine("\nExport finished successfully.");
            }
        }
    }
}
