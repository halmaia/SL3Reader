using System;
using static System.IO.Path;
using static System.Console;
using System.Runtime.CompilerServices;

namespace SL3Reader
{
    public static class Program
    {
        public static void Main(string[] args)
        {
            if (args!.Length < 3) // Always non-null.
            {
                PrintUsage();
                return;
            }

            string input = GetFullPath(args![0]);
            if (!Exists(input))
            {
                WriteLine("Input file not found!");
                PrintUsage();
                return;
            }

            string output = GetFullPath(args![1]);
            if (!Exists(GetDirectoryName(output)))
            {
                WriteLine("Directory not found for the output file!");
                PrintUsage();
                return;
            }

            using SL3Reader sl3reader = new(input);

            string expSelector = args![2].Trim().ToLowerInvariant();
            switch (expSelector)
            {
                case "-3dm":
                    sl3reader.Export3D(output, false, true);
                    PrintGreen("3D content with magnetic heading exported successfully.\n");
                    break;
                case "-3dg":
                    sl3reader.Export3D(output, false, false);
                    PrintGreen("3D content with GNSS heading exported successfully.\n");
                    break;
                case "-route":
                    sl3reader.ExportToCSV(output);
                    PrintGreen("Route exported successfully.\n");
                    break;
                case "-ss":
                    sl3reader.ExportImagery(output);
                    PrintGreen("Side scan imagery exported successfully.\n");
                    break;
                case "-ps":
                    sl3reader.ExportImagery(output, SurveyType.Primary);
                    PrintGreen("Primary scan imagery exported successfully.\n");
                    break;
                case "-ses":
                    sl3reader.ExportImagery(output, SurveyType.Secondary);
                    PrintGreen("Secondary scan imagery exported successfully.\n");
                    break;
                case "-ds":
                    sl3reader.ExportImagery(output, SurveyType.DownScan);
                    PrintGreen("DownScan imagery exported successfully.\n");
                    break;
                case "-u7":
                    sl3reader.ExportImagery(output, SurveyType.Unknown7);
                    PrintGreen("Frame type Nr.7 imagery exported successfully.\n");
                    break;
                default:
                    WriteLine("Invalid third argument (" + expSelector + ").");
                    PrintUsage();
                    return;
            }

            PrintSummary();

            return;

            static void PrintUsage()
            {
                WriteLine("Usage examples:\n");

                WriteLine("To export route:");

                WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -route\n");
                WriteLine("To export 3D points with magnetic heading (e.g. measured with devices like Precision–9):");
                WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -3dm\n");
                WriteLine("To export 3D points with GNSS heading (e.g. in-built GPS):");
                WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -3dg\n");

                WriteLine("To export side scan imagery:");
                WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\OutputFolder\" -ss\n");
                WriteLine("To export primary scan imagery:");
                WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\OutputFolder\" -ps\n");
                WriteLine("To export secondary scan imagery:");
                WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\OutputFolder\" -ses\n");
                WriteLine("To export down scan imagery:");
                WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\OutputFolder\" -ds\n");
                WriteLine("To export frame type №7 imagery imagery:");
                WriteLine("SL3Reader.exe \"C:\\input.sl3\" \"D:\\OutputFolder\" -u7\n");

                WriteLine("If either the input file’s name/path or the output file name/path contains space(s) use double quotation mark (\") to enclose it, like \"D:\\My SL3 Files\\Best Catch.sl3\".\n");

                WriteLine("For details see & cite the following publication: Halmai, Ákos; Gradwohl–Valkay, Alexandra; Czigány, Szabolcs; Ficsor, Johanna; Liptay, Zoltán Árpád; Kiss, Kinga; Lóczy, Dénes and Pirkhoffer, Ervin. 2020. \"Applicability of a Recreational-Grade Interferometric Sonar for the Bathymetric Survey and Monitoring of the Drava River\" ISPRS International Journal of Geo-Information 9, no. 3: 149. https://doi.org/10.3390/ijgi9030149 https://www.mdpi.com/2220-9964/9/3/149.\n");
                WriteLine("https://github.com/halmaia/SL3Reader");
                WriteLine("For license details see: https://github.com/halmaia/SL3Reader/blob/master/LICENSE.\n");
                WriteLine();

            }

            [SkipLocalsInit]
            void PrintSummary()
            {
                WriteLine("File statistics:");
                WriteLine("\tNumber of frames: " + sl3reader.Frames.Count.ToString("# ##0"));
                
                System.Collections.ObjectModel.ReadOnlyDictionary<SurveyType, System.Collections.ObjectModel.ReadOnlyCollection<nuint>> indexByType = sl3reader.IndexByType;
                WriteLine("\tNumber of primary frames: " + indexByType[SurveyType.Primary].Count.ToString("# ##0"));
                WriteLine("\tNumber of secondary frames: " + indexByType[SurveyType.Secondary].Count.ToString("# ##0"));
                WriteLine("\tNumber of left sidescan frames: " + indexByType[SurveyType.LeftSidescan].Count.ToString("# ##0"));
                WriteLine("\tNumber of right sidescan frames: " + indexByType[SurveyType.RightSidescan].Count.ToString("# ##0"));
                WriteLine("\tNumber of sidescan frames: " + indexByType[SurveyType.SideScan].Count.ToString("# ##0"));
                WriteLine("\tNumber of downscan frames: " + indexByType[SurveyType.DownScan].Count.ToString("# ##0"));
                WriteLine("\tNumber of 3D frames: " + indexByType[SurveyType.ThreeDimensional].Count.ToString("# ##0"));

                WriteLine();
                PrintGreen("\nExport finished successfully.");
            }

            [SkipLocalsInit]
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void PrintGreen(string message)
            {
                ForegroundColor = ConsoleColor.Green;
                WriteLine(message);
                ForegroundColor = ConsoleColor.White;
            }

        }
    }
}