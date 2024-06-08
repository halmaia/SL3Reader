using System;
using static System.Console;
using System.Runtime.CompilerServices;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Collections.ObjectModel;
using System.IO;

namespace SL3Reader;

public static class Program
{
    public static void Main([DisallowNull] string[] args)
    {
        OutputEncoding = System.Text.Encoding.UTF8;
        if (args!.Length is not 3) // Always non-null.
        {
            PrintRed("Wrong number of arguments!");
            WriteLine();
            PrintUsage();
            return;
        }

        string input = args![0];

        if (!File.Exists(input))
        {
            PrintRed("Input file not found!");
            WriteLine();
            PrintUsage();
            return;
        }

        DirectoryInfo directoryInfo = new(args![1]);
        if (!(directoryInfo.Parent ?? directoryInfo.Root).Exists)
        {
            PrintRed("Directory not found for output!");
            WriteLine();
            PrintUsage();
            return;
        }
        string output = directoryInfo.FullName;

        using SL3Reader sl3reader = new(input!);

        PrintSummary(sl3reader!);

        string expSelector = args![2].Trim().ToLowerInvariant();
        switch (expSelector)
        {
            case "-3dm":
                sl3reader.Export3D(output!, false, true);
                PrintGreen("3D content with magnetic heading exported successfully.\n");
                break;
            case "-3dm+":
                sl3reader.Export3D(output!, true, true);
                PrintGreen("3D content with magnetic heading exported successfully. Unreliable points are included.\n");
                break;
            case "-3dg":
                sl3reader.Export3D(output!, false, false);
                PrintGreen("3D content with GNSS heading exported successfully.\n");
                break;
            case "-3dg+":
                sl3reader.Export3D(output!, true, false);
                PrintGreen("3D content with GNSS heading exported successfully. Unreliable points are included.\n");
                break;
            case "-route":
                sl3reader.ExportRoutePoints(output!);
                PrintGreen("Route exported successfully.\n");
                break;
            case "-ss":
                sl3reader.ExportImagery(output!);
                PrintGreen("Side scan imagery exported successfully.\n");
                break;
            case "-ssg0":
                sl3reader.ExportImagery(output!, SurveyType.SideScan, true);
                PrintGreen("Side scan imagery exported successfully.\n");
                break;
            case "-ssg1":
                // sl3reader.ExportImagery(output!);
                PrintGreen("Do not use! Preserved for future compatibility!\n");
                break;
            case "-ssg3":
                // sl3reader.ExportImagery(output!);
                PrintGreen("Do not use! Preserved for future compatibility!\n");
                break;
            case "-ps":
                sl3reader.ExportImagery(output!, SurveyType.Primary);
                PrintGreen("Primary scan imagery exported successfully.\n");
                break;
            case "-ses":
                sl3reader.ExportImagery(output!, SurveyType.Secondary);
                PrintGreen("Secondary scan imagery exported successfully.\n");
                break;
            case "-ds":
                sl3reader.ExportImagery(output!, SurveyType.DownScan);
                PrintGreen("DownScan imagery exported successfully.\n");
                break;
            case "-u7":
                sl3reader.ExportImagery(output!, SurveyType.Unknown7);
                PrintGreen("Frame type №7 imagery exported successfully.\n");
                break;
            case "-exp":
                sl3reader.ExportSS_Int_Pair(output!);
                PrintGreen("Experimental output generated successfully.\n");
                break;
            default:
                WriteLine("Invalid argument (" + expSelector + ").");
                PrintUsage();
                return;
        }

        return;

        static void PrintUsage()
        {
            WriteLine();

            WriteLine("Usage examples:\n");

            WriteLine("To export route:");
            WriteLine("\tSL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -route\n");
            WriteLine("To export 3D points with magnetic heading (e.g. measured with devices like Precision–9):");
            WriteLine("\tSL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -3dm\n");
            WriteLine("To export 3D points with magnetic heading (e.g. measured with devices like Precision–9) with unreliable points:");
            WriteLine("\tSL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -3dm+\n");
            WriteLine("To export 3D points with GNSS heading (e.g. in-built GPS):");
            WriteLine("\tSL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -3dg\n");
            WriteLine("To export 3D points with GNSS heading (e.g. in-built GPS) with unreliable points:");
            WriteLine("\tSL3Reader.exe \"C:\\input.sl3\" \"D:\\output.csv\" -3dg+\n");

            WriteLine("To export side scan imagery:");

            WriteLine("\tSL3Reader.exe \"C:\\input.sl3\" \"D:\\OutputFolder\" -ss\n");
            WriteLine("To export primary scan imagery:");
            WriteLine("\tSL3Reader.exe \"C:\\input.sl3\" \"D:\\OutputFolder\" -ps\n");
            WriteLine("To export secondary scan imagery:");
            WriteLine("\tSL3Reader.exe \"C:\\input.sl3\" \"D:\\OutputFolder\" -ses\n");
            WriteLine("To export down scan imagery:");
            WriteLine("\tSL3Reader.exe \"C:\\input.sl3\" \"D:\\OutputFolder\" -ds\n");
            WriteLine("To export frame type №7 imagery imagery:");
            WriteLine("\tSL3Reader.exe \"C:\\input.sl3\" \"D:\\OutputFolder\" -u7\n");

            WriteLine("If either the input file’s name/path or the output file name/path contains space(s) use double quotation mark (\") to enclose it, like \"D:\\My SL3 Files\\Best Catch.sl3\".\n");

            WriteLine("For details see & cite the following publication: Halmai, Ákos; Gradwohl–Valkay, Alexandra; Czigány, Szabolcs; Ficsor, Johanna; Liptay, Zoltán Árpád; Kiss, Kinga; Lóczy, Dénes and Pirkhoffer, Ervin. 2020. \"Applicability of a Recreational-Grade Interferometric Sonar for the Bathymetric Survey and Monitoring of the Drava River\" ISPRS International Journal of Geo-Information 9, no. 3: 149. https://doi.org/10.3390/ijgi9030149 https://www.mdpi.com/2220-9964/9/3/149.\n");
            WriteLine("https://github.com/halmaia/SL3Reader");
            WriteLine("For license details see: https://github.com/halmaia/SL3Reader/blob/master/LICENSE.\n");
            WriteLine();
        }

        [SkipLocalsInit]
        static void PrintSummary([DisallowNull] SL3Reader sl3reader)
        {
            ReadOnlyDictionary<SurveyType, ReadOnlyCollection<nuint>> indexByType = sl3reader.FrameByType;
            ReadOnlyCollection<nuint> frames = sl3reader.Frames;
            int len = frames.Count;
            System.Globalization.CultureInfo invariantCulture = System.Globalization.CultureInfo.InvariantCulture;

            WriteLine("File statistics:");
            WriteLine("\tNumber of frames: " + len.ToString("# ##0"));

            WriteLine("\tNumber of primary frames: " + indexByType[SurveyType.Primary].Count.ToString("# ##0"));
            WriteLine("\tNumber of secondary frames: " + indexByType[SurveyType.Secondary].Count.ToString("# ##0"));
            WriteLine("\tNumber of left sidescan frames: " + indexByType[SurveyType.LeftSideScan].Count.ToString("# ##0"));
            WriteLine("\tNumber of right sidescan frames: " + indexByType[SurveyType.RightSideScan].Count.ToString("# ##0"));
            WriteLine("\tNumber of sidescan frames: " + indexByType[SurveyType.SideScan].Count.ToString("# ##0"));
            WriteLine("\tNumber of downscan frames: " + indexByType[SurveyType.DownScan].Count.ToString("# ##0"));
            WriteLine("\tNumber of 3D frames: " + indexByType[SurveyType.ThreeDimensional].Count.ToString("# ##0"));
            WriteLine("\tDistance covered: " + sl3reader.AugmentedCoordinates.LastOrDefault().Distance.ToString("0.# m", invariantCulture));

            if (len is not 0)
            {
                unsafe
                {
                    WriteLine("\tBeginning of the survey: " + ((Frame*)frames[0])->Timestamp.ToString("yyyy'-'MM'-'dd HH':'mm':'ss.fff'Z'", invariantCulture));
                    WriteLine("\tEnd of the survey: " + ((Frame*)frames[len - 1])->Timestamp.ToString("yyyy'-'MM'-'dd HH':'mm':'ss.fff'Z'", invariantCulture));
                }
            }

            WriteLine();
        }

        [SkipLocalsInit]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void PrintGreen(string message)
        {
            ForegroundColor = ConsoleColor.Green;
            WriteLine(message);
            ForegroundColor = ConsoleColor.White;
        }

        [SkipLocalsInit]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void PrintRed(string message)
        {
            ForegroundColor = ConsoleColor.Red;
            WriteLine(message);
            ForegroundColor = ConsoleColor.White;
        };
    }
}