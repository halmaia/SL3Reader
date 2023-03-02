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
                case "-3dm":
                    sl3reader.Export3D(output, false, true);
                    break;
                case "-3dg":
                    sl3reader.Export3D(output, false, false);
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
                Console.WriteLine("Usage examples:\n");

                Console.WriteLine("To export route:");
                Console.WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -route\n");
                Console.WriteLine("To export 3D points with magnetic heading (e.g. measured with Precision-9):");
                Console.WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -3dm\n");
                Console.WriteLine("To export 3D points with GNSS heading (e.g. in-built GPS):");
                Console.WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -3dg");
            }

            void PrintSummary() {
                var indexByType = sl3reader.IndexByType;

                Console.WriteLine("File statistics:");
                Console.WriteLine("Number of frames: " + sl3reader.Frames.Count.ToString());
                Console.WriteLine("Number of primary frames: " + indexByType[SurveyType.Primary].Count.ToString());
                Console.WriteLine("Number of primary frames: " + indexByType[SurveyType.Secondary].Count.ToString());
                Console.WriteLine("Number of left sidescan frames: " + indexByType[SurveyType.LeftSidescan].Count.ToString());
                Console.WriteLine("Number of right sidescan frames: " + indexByType[SurveyType.RightSidescan].Count.ToString());
                Console.WriteLine("Number of sidescan frames: " + indexByType[SurveyType.SideScan].Count.ToString());
                Console.WriteLine("Number of downscan frames: " + indexByType[SurveyType.DownScan].Count.ToString());
                Console.WriteLine("Number of 3D frames: " + indexByType[SurveyType.ThreeDimensional].Count.ToString());
                
                Console.WriteLine();
                Console.ForegroundColor= ConsoleColor.Green;
                Console.WriteLine("\nExport finished successfully.");
                Console.ForegroundColor = ConsoleColor.White;
            }
        }
    }
}