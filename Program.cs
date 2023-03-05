using System;
using System.Collections.Generic;
using static System.IO.Path;
using static System.Console;

namespace SL3Reader {

    public static class Program {
        public static void Main(string[] args) {
            if (args.Length != 3) {
                PrintUsage();
                return;
            }

            string input = GetFullPath(args[0]);
            if (!Exists(input)) {
                WriteLine("Input file not found!");
                PrintUsage();
                return;
            }

            string output = GetFullPath(args[1]);
            if (!Exists(GetDirectoryName(output))) {
                WriteLine("Directory not found for the output file!");
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
                case "-ss":
                    sl3reader.ExportImagery(output);
                    break;
                default:
                    WriteLine("Invalid third argument (" + expSelector + ").");
                    PrintUsage();
                    return;
            }

            PrintSummary();

            return;

            static void PrintUsage() {
                WriteLine("Usage examples:\n");

                WriteLine("To export route:");
                WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -route\n");
                WriteLine("To export 3D points with magnetic heading (e.g. measured with Precision-9):");
                WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -3dm\n");
                WriteLine("To export 3D points with GNSS heading (e.g. in-built GPS):");
                WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -3dg");
            }

            void PrintSummary() {
                SortedDictionary<SurveyType, List<int>> indexByType = sl3reader.IndexByType;

                WriteLine("File statistics:");
                WriteLine("\tNumber of frames: " + sl3reader.Frames.Count.ToString());
                WriteLine("\tNumber of primary frames: " + indexByType[SurveyType.Primary].Count.ToString());
                WriteLine("\tNumber of secondary frames: " + indexByType[SurveyType.Secondary].Count.ToString());
                WriteLine("\tNumber of left sidescan frames: " + indexByType[SurveyType.LeftSidescan].Count.ToString());
                WriteLine("\tNumber of right sidescan frames: " + indexByType[SurveyType.RightSidescan].Count.ToString());
                WriteLine("\tNumber of sidescan frames: " + indexByType[SurveyType.SideScan].Count.ToString());
                WriteLine("\tNumber of downscan frames: " + indexByType[SurveyType.DownScan].Count.ToString());
                WriteLine("\tNumber of 3D frames: " + indexByType[SurveyType.ThreeDimensional].Count.ToString());
                
                WriteLine();
                ForegroundColor= ConsoleColor.Green;
                WriteLine("\nExport finished successfully.");
                ForegroundColor = ConsoleColor.White;
            }
        }
    }
}
