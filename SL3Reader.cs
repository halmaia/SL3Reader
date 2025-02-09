﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Runtime.CompilerServices;
using Microsoft.Win32.SafeHandles;
using System.Globalization;
using System.Diagnostics.CodeAnalysis;
using System.IO.MemoryMappedFiles;
using System.Collections.ObjectModel;
using System.Text;
using System.Runtime.InteropServices;

namespace SL3Reader;

public class SL3Reader : IDisposable
{
    #region Public properties
    public ReadOnlyCollection<nuint> Frames { get; }
    public ReadOnlyCollection<GeoPoint> AugmentedCoordinates { get; }
    public ReadOnlyDictionary<SurveyType, ReadOnlyCollection<nuint>> FrameByType { get; }
    #endregion End Public properties

    #region Private variables
    private readonly MemoryMappedFile memoryMappedFile;
    private readonly MemoryMappedViewAccessor viewAccessor;
    private readonly SafeMemoryMappedViewHandle viewHandle;
    private readonly ReadOnlyCollection<int> Coordinate3DHelper;
    #endregion End Private variables

    [SkipLocalsInit]
    public unsafe SL3Reader([DisallowNull] string path)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(path, nameof(path));
        long len = new FileInfo(path!).Length;

        if (len < (SLFileHeader.Size + Frame.BasicSize))
            throw new EndOfStreamException("The file is too short to be valid.");

        // Init Frames
        const long averageFrameSize = 2118L; // Empirically set value to avoid frequent resize of the underlying array.
        int estimatedCount = (int)(len / averageFrameSize);
        ReadOnlyCollectionBuilder<nuint> frames = new(estimatedCount);

        // Init index lists
        int p8 = estimatedCount / 8, p4 = estimatedCount / 4, p10 = estimatedCount / 10;
        ReadOnlyCollectionBuilder<nuint> Primary = new(p8),
                                         Secondary = new(p8),
                                         DownScan = new(p10),
                                         LeftSidescan = [],
                                         RightSidescan = [],
                                         SideScan = new(p10),
                                         Unknown6 = [],
                                         Unknown7 = new(p4),
                                         Unknown8 = new(p4),
                                         ThreeDimensional = new(p10),
                                         DebugDigital = [],
                                         DebugNoise = [];

        ReadOnlyCollectionBuilder<int> coordinate3DHelper = new(p10);
        ReadOnlyCollectionBuilder<int> coordinateSidescanHelper = new(p10);

        memoryMappedFile = MemoryMappedFile.CreateFromFile(
            File.OpenHandle(path!, FileMode.Open, FileAccess.Read, FileShare.Read, FileOptions.RandomAccess, 0L),
            null, len, MemoryMappedFileAccess.Read, HandleInheritability.None, false);
        viewAccessor = memoryMappedFile.CreateViewAccessor(0, len, MemoryMappedFileAccess.Read);
        viewHandle = viewAccessor.SafeMemoryMappedViewHandle;
        byte* ptr = null;
        viewHandle.AcquirePointer(ref ptr);
        byte* maxPtr = ptr + len;

        ((SLFileHeader*)ptr)->ThrowIfInvalidFormatDetected();
        ptr += SLFileHeader.Size; // Advance for the first frame
        Frame* currentFrame = (Frame*)ptr;
        Frame.InitTimestampBase(currentFrame->HardwareTime); // Init time

        bool unknownFrameDetected = false;

        // Load frames
        for (int i = 0;
            ptr < maxPtr && (ptr + currentFrame->LengthOfFrame) < maxPtr; // Use whole frames only!
            ptr += currentFrame->LengthOfFrame,
            currentFrame = (Frame*)ptr,
            i++)
        {
            nuint current = (nuint)currentFrame;
            frames.Add(current);
            switch (currentFrame->SurveyType)
            {
                case SurveyType.Primary:
                    Primary.Add(current);
                    break;
                case SurveyType.Secondary:
                    Secondary.Add(current);
                    break;
                case SurveyType.DownScan:
                    DownScan.Add(current);
                    break;
                case SurveyType.LeftSideScan:
                    LeftSidescan.Add(current);
                    break;
                case SurveyType.RightSideScan:
                    RightSidescan.Add(current);
                    break;
                case SurveyType.SideScan:
                    SideScan.Add(current);
                    break;
                case SurveyType.Unknown6:
                    Unknown6.Add(current);
                    break;
                case SurveyType.Unknown7:
                    Unknown7.Add(current);
                    break;
                case SurveyType.Unknown8:
                    Unknown8.Add(current);
                    break;
                case SurveyType.ThreeDimensional:
                    ThreeDimensional.Add(current);
                    coordinate3DHelper.Add(i);
                    break;
                case SurveyType.DebugDigital:
                    DebugDigital.Add(current);
                    break;
                case SurveyType.DebugNoise:
                    DebugNoise.Add(current);
                    break;
                default:
                    unknownFrameDetected = true;
                    break;
            }
        }

        if (unknownFrameDetected)
            Console.WriteLine("Warning! Unknown frame type detected!\nContact the developer for further analysis.");

        // Init AugmentedCoordinates
        AugmentedCoordinates = frames.Count is 0 ?
            new ReadOnlyCollectionBuilder<GeoPoint>().ToReadOnlyCollection() :
            AugmentTrajectory(frames);

        Frames = frames.ToReadOnlyCollection();

        // Populate major index
        FrameByType = new SortedDictionary<SurveyType, ReadOnlyCollection<nuint>>()
        {
            { SurveyType.Primary, Primary.ToReadOnlyCollection() },
            { SurveyType.Secondary, Secondary.ToReadOnlyCollection() },
            { SurveyType.DownScan, DownScan.ToReadOnlyCollection() },
            { SurveyType.LeftSideScan, LeftSidescan.ToReadOnlyCollection() },
            { SurveyType. RightSideScan, RightSidescan.ToReadOnlyCollection() },
            { SurveyType.SideScan, SideScan.ToReadOnlyCollection() },
            { SurveyType.Unknown6, Unknown6.ToReadOnlyCollection() },
            { SurveyType.Unknown7, Unknown7.ToReadOnlyCollection() },
            { SurveyType.Unknown8, Unknown8.ToReadOnlyCollection() },
            { SurveyType.ThreeDimensional, ThreeDimensional.ToReadOnlyCollection() },
            { SurveyType.DebugDigital, DebugDigital.ToReadOnlyCollection() },
            { SurveyType.DebugNoise, DebugNoise.ToReadOnlyCollection() }
        }.AsReadOnly();

        Coordinate3DHelper = coordinate3DHelper.ToReadOnlyCollection();
        // ExamineUnknown8Datasets();
        return;

        // Local functions:

        static ReadOnlyCollection<GeoPoint> AugmentTrajectory(ReadOnlyCollectionBuilder<nuint> frames)
        {
            const double lim = 1.2d;

            ReadOnlyCollectionBuilder<GeoPoint> coordinates = new(frames.Count);

            (double longitude0, double lattitude0, double z0, double v0, double t0, double d0) = ((Frame*)frames[0])->QueryPositionHeadingSpeedTime();
            coordinates.Add(new(longitude0, lattitude0, d0, z0, 0)); // The first one.
            double distance = 0, LongPrev = longitude0, LatPrev = lattitude0;

            for (int i = 1, frameCount = frames.Count; i != frameCount;)
            {
                Frame* frame = (Frame*)frames[i++];

                (double longitude1, double lattitude1, double z1, double v1, double t1, double d1) =
                    frame->QueryPositionHeadingSpeedTime();

                (double sin0, double cos0) = double.SinCos(d0);
                (double sin1, double cos1) = double.SinCos(d1);

                double vx0 = cos0 * v0,
                       vy0 = sin0 * v0,
                       vx1 = cos1 * v1,
                       vy1 = sin1 * v1,
                       dt = t1 - t0;

                lattitude0 += dt * .5d * (vy0 + vy1) * 180d / (double.Pi * 6356752.3142d);
                longitude0 += dt * .5d * (vx0 + vx1) * 180d / (double.Pi * double.Cos(double.DegreesToRadians(lattitude0)) * 6356752.3142d);

                d0 = d1; t0 = t1; v0 = v1;

                if (frame->SurveyType is SurveyType.Primary or SurveyType.Secondary or
                    SurveyType.Unknown7 or SurveyType.Unknown8)
                {
                    double dy = GetLattitudeDistance(lattitude0, lattitude1);
                    if (dy > lim)
                        lattitude0 = lattitude1 + double.CopySign(double.RadiansToDegrees(lim / 6356752.3142d), lattitude0 - lattitude1);

                    double dx = GetLongitudeDistance(longitude0, longitude1, lattitude0);
                    if (dx > lim)
                        longitude0 = longitude1 + double.CopySign(double.RadiansToDegrees(lim / (6356752.3142d * double.Cos(double.DegreesToRadians(lattitude0)))), longitude0 - longitude1);
                }
                else
                {
                    double dy = GetLattitudeDistance(lattitude0, lattitude1);
                    if (dy > 50d) // Detect serious errors.
                        lattitude0 = frame->Latitude;

                    double dx = GetLongitudeDistance(longitude0, longitude1, lattitude0);
                    if (dx > 50d) // Detect serious errors.
                        longitude0 = frame->Longitude;
                }

                coordinates.Add(new(longitude0, lattitude0, d0, z1, distance += double.Hypot(GetLongitudeDistance(longitude0, LongPrev, lattitude0), GetLattitudeDistance(lattitude0, LatPrev))));
                LongPrev = longitude0; LatPrev = lattitude0;

            }
            return coordinates.ToReadOnlyCollection();

            static double GetLattitudeDistance(double lat1, double lat2) => 
                double.DegreesToRadians(double.Abs(lat2 - lat1)) * 6356752.3142d;

            static double GetLongitudeDistance(double lon1, double lon2, double lat) => 
                double.DegreesToRadians(double.Abs(lon2 - lon1)) * 
                                       (6356752.3142d * double.Cos(double.DegreesToRadians(lat)));
        }
    }

    public unsafe void ExportRoutePoints(string path)
    {
        ReadOnlyCollection<nuint> frames = Frames;
        int count = frames.Count;
        using FileStream textWriter = new(path,
            new FileStreamOptions()
            {
                Access = FileAccess.Write,
                Mode = FileMode.Create,
                Share = FileShare.Read,
                Options = FileOptions.SequentialScan,
                PreallocationSize = 151 * count, // Empirical guess.
                BufferSize = 65536
            });

        textWriter.Write("CampaignID[#],DateTime[UTC],SurveyType,WaterDepth[Feet],Longitude[°WGS84],Latitude[°WGS84],GNSSAltitude[Feet_WGS84],GNSSHeading[rad_azimuth],GNSSSpeed[m/s],MagneticHeading[rad_azimuth],MinRange[Feet],MaxRange[Feet],WaterTemperature[°C],WaterSpeed[Feet],HardwareTime[ms],Frequency,Milliseconds[ms],AugmentedX[m],AugmentedY[m]\r\n"u8);

        // TODO: Filter is not working!
        //ReadOnlyCollection<nuint> frames = filter is SurveyType.All ? Frames : IndexByType[filter];

        ReadOnlyCollection<GeoPoint> augmentedCoordinates = AugmentedCoordinates;

        scoped Span<byte> buffer = stackalloc byte[256];
        for (int i = 0; i != count;)
        {
            textWriter.Write(((Frame*)frames[i])->Format(buffer));
            textWriter.Write(augmentedCoordinates[i++].Format(buffer));
        }
    }

    public unsafe void ExportImagery(string path, SurveyType surveyType = SurveyType.SideScan, bool exportGeoreference = false)
    {
        ReadOnlyCollection<nuint> imageFrames = FrameByType[surveyType];
        if (imageFrames.Count < 1) return; // Return when no imagery exists.

        ArgumentNullException.ThrowIfNull(path);

        CultureInfo invariantCulture = CultureInfo.InvariantCulture;

        List<int> breakpoints = GetBreakPoints(imageFrames, out int maxHeight);
        int numberOfColumns = (int)((Frame*)imageFrames[0])->LengthOfEchoData;
        byte[] buffer = BitmapHelper.CreateBuffer(maxHeight, numberOfColumns);

        for (int i = 0, maxIndex = breakpoints.Count - 1; i < maxIndex; i++)
        {
            int first = breakpoints[i], final = breakpoints[1 + i];
            BitmapHelper.UpdateBuffer(
                buffer, final - first, numberOfColumns,
                out int fullStride,
                out Span<byte> fileBuffer,
                out Span<byte> pixelData);

            byte* offset = (byte*)((Frame*)imageFrames[0])->HeaderSize;

            fixed (byte* pixelPtr = pixelData)
                for (int j = first, k = 0; j < final; j++)
                    Buffer.MemoryCopy(imageFrames[j] + offset, pixelPtr + (fullStride * k++), numberOfColumns, numberOfColumns);

            string prefix = GetPrefix(surveyType);
            string filePath = Path.Combine(path, prefix + final + ".bmp");
            using SafeFileHandle handle = File.OpenHandle(filePath!,
                FileMode.Create, FileAccess.Write, FileShare.None, FileOptions.SequentialScan, fileBuffer.Length);
            RandomAccess.Write(handle, fileBuffer, 0);
            handle.Dispose();

            // World file

            // TODO: Remove intermediate solution:
            GeoPoint firstStrip = AugmentedCoordinates[Frames.IndexOf(imageFrames[first])];
            GeoPoint lastStrip = AugmentedCoordinates[Frames.IndexOf(imageFrames[final - 1])];

            Frame* lastFrame = (Frame*)imageFrames[final - 1];

            if (!exportGeoreference)
            {
                double XSize = -(lastStrip.Distance - firstStrip.Distance) / (final - first - 1);
                double YSize = -10 * lastFrame->MaxRange * .3048 / numberOfColumns;
                string WorldString = string.Join("\r\n",
                        ["0",
                     YSize.ToString(invariantCulture),
                     XSize.ToString(invariantCulture),
                     "0",
                     lastStrip.Distance.ToString(invariantCulture),
                     lastFrame->SurveyType is SurveyType.SideScan ? (-1400d * YSize).ToString(): "0"], 0, 6);

                File.WriteAllText(Path.Combine(path, prefix + final + ".bpw"), WorldString);
            }
            // End world file

            // AUX file
            if (exportGeoreference)
            {

                double delta = .3084 * lastFrame->MaxRange;

                List<GeoPoint> sourceGCPs = new((final - first) / 10);
                List<GeoPoint> targetGCPs = new(sourceGCPs.Capacity);

                for (int s = first; s <= final - 1; s += 50)
                {

                    sourceGCPs.Add(new(1400.5, -(s - first) + .5, default, default, default));
                    sourceGCPs.Add(new(2799.5, -(s - first) + .5, default, default, default));
                    sourceGCPs.Add(new(.5, -(s - first) + .5, default, default, default));

                    GeoPoint Strip = AugmentedCoordinates[Frames.IndexOf(imageFrames[s])];
                    Frame* frame = (Frame*)imageFrames[s];
                    (double sin, double cos) =
                    double.SinCos(frame->GNSSHeading - .5 * double.Pi);

                    targetGCPs.AddRange([
                    Strip,
                    new(double.FusedMultiplyAdd(-delta, sin, Strip.X),
                        double.FusedMultiplyAdd(-delta, cos, Strip.Y),
                        0, 0, 0),
                    new(double.FusedMultiplyAdd(delta, sin, Strip.X),
                        double.FusedMultiplyAdd(delta, cos, Strip.Y),
                        0, 0, 0)]);
                }

                GeoReferenceHelper.WriteGeoreferencedPAM(Path.ChangeExtension(filePath, ".bmp.aux.xml"), sourceGCPs, targetGCPs);
            }
            else
            {
                WriteAUXFile(filePath);
            }
        }

        [SkipLocalsInit, MethodImpl(MethodImplOptions.AggressiveInlining)]
        static string GetPrefix(SurveyType surveyType) =>
            surveyType switch
            {
                SurveyType.SideScan => "SS_",
                SurveyType.DownScan => "DS_",
                SurveyType.Primary => "PS_",
                SurveyType.Secondary => "SES_",
                SurveyType.Unknown8 => "U8_",
                SurveyType.Unknown7 => "U7_",
                _ => "UNK_"
            };

        static unsafe List<int> GetBreakPoints(ReadOnlyCollection<nuint> framesToCheck, out int contiguousLength)
        {
            int frameCount = framesToCheck.Count;
            List<int> breakpoints = new(frameCount / 300) { 0 }; // ~300 empirical guess. 
                                                                 // 'i' have to be incremented to prevent double test.

            float previousRange = ((Frame*)framesToCheck[0])->MaxRange;

            int i = 1;
            for (; i != frameCount; i++)
            {
                float currentRange = ((Frame*)framesToCheck[i])->MaxRange;
                if (currentRange != previousRange)
                {
                    previousRange = currentRange;
                    breakpoints.Add(i);
                }
            }

            if (breakpoints[^1] != --i)
                breakpoints.Add(i); // There was no change in the range, so we have to add the last one.

            contiguousLength = 0;

            for (int j = 0,
                 maxIndex = breakpoints.Count - 1,
                 delta;
                 j < maxIndex;
                 j++)
            {
                if (contiguousLength < (delta = breakpoints[1 + j] - breakpoints[j]))
                    contiguousLength = delta;
            }
            return breakpoints;
        }
    }

    public unsafe void ExamineUnknown8Datasets()
    {
        ReadOnlyCollection<nuint> unknown8Frames = FrameByType[SurveyType.Unknown8];
        int unknown8FrameCount = unknown8Frames.Count;
        if (unknown8FrameCount < 1) return; // Return when no U8 exists

        using var writer = File.CreateText(@"F:\Sample.csv");
        for (int i = 0; i < unknown8FrameCount; i++)
        {

            Frame* currentFrame = (Frame*)unknown8Frames[i];
            byte* ptr = ((byte*)currentFrame) + currentFrame->HeaderSize;
            for (int j = 0; j < 512; j += 2, ptr += 2)
            {
                writer.Write((*(short*)ptr).ToString() + ',');
            }
            writer.WriteLine();
        }
        writer.Dispose();
    }

    public unsafe void Export3D(string path, bool includeUnreliable = false, bool magneticHeading = false, bool flip = false)
    {
        ArgumentNullException.ThrowIfNull(path);

        const string doubleFormat = "0.####";
        const int bufferSize = 16384;

        ReadOnlyCollection<nuint> frames3D = FrameByType[SurveyType.ThreeDimensional];
        int frames3DLength = frames3D.Count;
        if (frames3DLength < 1) return;

        CultureInfo invariantCulture = CultureInfo.InvariantCulture;
        ReadOnlyCollection<GeoPoint> augmentedCoordinates = AugmentedCoordinates;
        ReadOnlyCollection<int> coordinate3DHelper = Coordinate3DHelper;

        string[] stringArray = new string[8];
        StringBuilder stringBuilder = new(bufferSize);

        using StreamWriter streamWriter = new(path!, Encoding.UTF8,
            new FileStreamOptions()
            {
                Access = FileAccess.Write,
                BufferSize = bufferSize,
                Mode = FileMode.OpenOrCreate,
                Options = FileOptions.SequentialScan
            });
        streamWriter.BaseStream.Write("CampaignID,DateTime,X[Lowrance_m],Y[Lowrance_m],Z[m_WGS84],Depth[m],Angle[°],Distance[m],Reliable\r\n"u8);

        double leftConversionUnit = flip ? 0.3048d : -0.3048d;
        double rightConversionUnit = -leftConversionUnit;

        for (int i = 0; i < frames3DLength; i++)
        {
            Frame* frame = (Frame*)frames3D[i];
            ThreeDimensionalFrameHeader* header = (ThreeDimensionalFrameHeader*)((byte*)frame + frame->HeaderSize);

            GeoPoint augmentedCoordinate = augmentedCoordinates[coordinate3DHelper[i]];
            byte* measurements = (byte*)header + ThreeDimensionalFrameHeader.Size;

            double centralX = augmentedCoordinate.X,
                   centralY = augmentedCoordinate.Y,
                   centralZ = .3048 * augmentedCoordinate.Altitude;

            (double sin, double cos) =
                double.SinCos((magneticHeading ? frame->MagneticHeading : frame->GNSSHeading) - .5 * double.Pi);

            stringArray[0] = frame->CampaignID.ToString();
            stringArray[1] = frame->Timestamp.ToString("yyyy'-'MM'-'dd HH':'mm':'ss.fff'Z'", invariantCulture);

            // *************************** THIS IS TERRIBLE ***************************
            // *************************** Refactor    ASAP ***************************
            // Left side
            for (byte* limit = measurements + header->NumberOfLeftBytes;
                measurements < limit;
                measurements += InterferometricMeasurement.Size)
            {
                InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;
                double delta = leftConversionUnit * measurement->Delta, // Negative side
                       depth = 0.3048d * measurement->Depth;

                stringArray[2] = double.FusedMultiplyAdd(delta, sin, centralX).ToString(doubleFormat, invariantCulture); // Azimuthal direction
                stringArray[3] = double.FusedMultiplyAdd(delta, cos, centralY).ToString(doubleFormat, invariantCulture);
                stringArray[4] = (centralZ - depth).ToString(doubleFormat, invariantCulture);
                stringArray[5] = depth.ToString(doubleFormat, invariantCulture);
                stringArray[6] = measurement->AngleInDegrees.ToString(doubleFormat, invariantCulture);
                stringArray[7] = measurement->MetricDistance.ToString(doubleFormat, invariantCulture);

                stringBuilder.AppendJoin(',', stringArray).AppendLine(",Y");
            }

            // Right side
            for (byte* limit = measurements + header->NumberOfRightBytes;
                measurements < limit;
                measurements += InterferometricMeasurement.Size)
            {
                InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;
                double delta = rightConversionUnit * measurement->Delta, // Positive side 
                       depth = .3048d * measurement->Depth;

                stringArray[2] = double.FusedMultiplyAdd(delta, sin, centralX).ToString(doubleFormat, invariantCulture); // Azimuthal direction
                stringArray[3] = double.FusedMultiplyAdd(delta, cos, centralY).ToString(doubleFormat, invariantCulture);
                stringArray[4] = (centralZ - depth).ToString(doubleFormat, invariantCulture);
                stringArray[5] = depth.ToString(doubleFormat, invariantCulture);
                stringArray[6] = measurement->AngleInDegrees.ToString(doubleFormat, invariantCulture);
                stringArray[7] = measurement->MetricDistance.ToString(doubleFormat, invariantCulture);

                stringBuilder.AppendJoin(',', stringArray).AppendLine(",Y");
            }

            if (includeUnreliable)
            {
                for (byte* limit = measurements + header->NumberOfUnreliableLeftBytes;
                    measurements < limit;
                    measurements += InterferometricMeasurement.Size)
                {
                    InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;
                    if (measurement->IsValid)
                    {
                        double delta = leftConversionUnit * measurement->Delta, // Negative side
                               depth = 0.3048d * measurement->Depth;

                        stringArray[2] = double.FusedMultiplyAdd(delta, sin, centralX).ToString(doubleFormat, invariantCulture); // Azimuthal direction
                        stringArray[3] = double.FusedMultiplyAdd(delta, cos, centralY).ToString(doubleFormat, invariantCulture);
                        stringArray[4] = (centralZ - depth).ToString(doubleFormat, invariantCulture);
                        stringArray[5] = depth.ToString(doubleFormat, invariantCulture);
                        stringArray[6] = measurement->AngleInDegrees.ToString(doubleFormat, invariantCulture);
                        stringArray[7] = measurement->MetricDistance.ToString(doubleFormat, invariantCulture);

                        stringBuilder.AppendJoin(',', stringArray).AppendLine(",N");
                    }
                }

                for (byte* limit = measurements + header->NumberOfUnreliableRightBytes;
                    measurements < limit;
                    measurements += InterferometricMeasurement.Size)
                {
                    InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;
                    if (measurement->IsValid)
                    {
                        double delta = rightConversionUnit * measurement->Delta, // Positive side
                               depth = 0.3048d * measurement->Depth;

                        stringArray[2] = double.FusedMultiplyAdd(delta, sin, centralX).ToString(doubleFormat, invariantCulture); // Azimuthal direction
                        stringArray[3] = double.FusedMultiplyAdd(delta, cos, centralY).ToString(doubleFormat, invariantCulture);
                        stringArray[4] = (centralZ - depth).ToString(doubleFormat, invariantCulture);
                        stringArray[5] = depth.ToString(doubleFormat, invariantCulture);
                        stringArray[6] = measurement->AngleInDegrees.ToString(doubleFormat, invariantCulture);
                        stringArray[7] = measurement->MetricDistance.ToString(doubleFormat, invariantCulture);

                        stringBuilder.AppendJoin(',', stringArray).AppendLine(",N");
                    }
                }
            }
            streamWriter.Write(stringBuilder);
            stringBuilder.Clear();
        }
    }

    #region AUX support
    [SkipLocalsInit]
    private static void WriteAUXFile(string originalPath)
    {
        ReadOnlySpan<byte> fileBuffer = "<PAMDataset>\n  <Metadata>\n    <MDI key=\"DataType\">Processed</MDI>\n    <MDI key=\"SensorName\">Lowrance StructureScan3D</MDI>\n  </Metadata>\n  <PAMRasterBand band=\"1\">\n    <Histograms>\n      <HistItem>\n        <HistMin>-0.5</HistMin>\n        <HistMax>168.5</HistMax>\n        <BucketCount>169</BucketCount>\n        <IncludeOutOfRange>1</IncludeOutOfRange>\n        <Approximate>1</Approximate>\n        <HistCounts>194107|13838|0|0|0|0|0|0|0|0|13204|0|0|0|0|0|0|0|0|0|12328|0|0|0|11492|0|0|10383|0|0|0|9912|0|0|4621|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|4391|0|0|0|8479|0|0|7695|0|0|0|7038|0|0|6490|5879|5554|5039|4656|4477|4055|3763|3511|3250|3025|2814|2592|2443|2320|2267|1980|1861|3424|3258|2877|2640|2440|2289|2162|2879|1758|1709|1500|1480|1446|1320|2954|2051|1884|1743|1506|1425|1265|1123|1017|909|838|775|729|641|1114|938|852|731|632|547|495|441|427|364|363|317|306|292|255|361|509|315|314|314|360|252|241|204|293|178|206|185|262|294|175|271|217|139|212|182|126|155|160|80|124|1993|85|145|208|157|184|190|129|170|104|84|117|157|381|334|658|1195</HistCounts>\n      </HistItem>\n    </Histograms>\n    <Metadata>\n      <MDI key=\"STATISTICS_COVARIANCES\">1748.548089737012</MDI>\n      <MDI key=\"STATISTICS_MAXIMUM\">255</MDI>\n      <MDI key=\"STATISTICS_MEAN\">128</MDI>\n      <MDI key=\"STATISTICS_MEDIAN\">128</MDI>\n      <MDI key=\"STATISTICS_MINIMUM\">0</MDI>\n      <MDI key=\"STATISTICS_SKIPFACTORX\">1</MDI>\n      <MDI key=\"STATISTICS_SKIPFACTORY\">1</MDI>\n      <MDI key=\"STATISTICS_STDDEV\">16</MDI>\n    </Metadata>\n  </PAMRasterBand>\n</PAMDataset>"u8;
        using SafeFileHandle handle = File.OpenHandle(Path.ChangeExtension(originalPath, ".bmp.aux.xml"),
        FileMode.Create, FileAccess.Write, FileShare.None, FileOptions.SequentialScan, fileBuffer.Length);
        RandomAccess.Write(handle, fileBuffer, 0L);
    }
    #endregion

    #region PRJ support
    [SkipLocalsInit]
    private static void WritePRJFile(string originalPath)
    {
        using SafeFileHandle handle = File.OpenHandle(Path.ChangeExtension(originalPath, ".prj"),
        FileMode.Create, FileAccess.Write, FileShare.None, FileOptions.SequentialScan, 805);
        RandomAccess.Write(handle,
            "PROJCS[\"Lowrance_Mercator\",GEOGCS[\"Lowrance_Sphere\",DATUM[\"D_Lowrance_Sphere\",SPHEROID[\"Lowrance_Sphere\",6356752.31424518,0.0]],PRIMEM[\"Greenwich\",0.0,AUTHORITY[\"EPSG\",8901]],UNIT[\"Degree\",0.0174532925199433,AUTHORITY[\"EPSG\",9102]]],PROJECTION[\"Mercator\",AUTHORITY[\"Esri\",43004]],PARAMETER[\"False_Easting\",0.0,AUTHORITY[\"Esri\",100001]],PARAMETER[\"False_Northing\",0.0,AUTHORITY[\"Esri\",100002]],PARAMETER[\"Central_Meridian\",0.0,AUTHORITY[\"Esri\",100010]],PARAMETER[\"Standard_Parallel_1\",0.0,AUTHORITY[\"Esri\",100025]],UNIT[\"Meter\",1.0,AUTHORITY[\"EPSG\",9001]]],VERTCS[\"WGS_1984_Geoid\",VDATUM[\"WGS_1984_Geoid\",AUTHORITY[\"Esri\",105100]],PARAMETER[\"Vertical_Shift\",0.0,AUTHORITY[\"Esri\",100006]],PARAMETER[\"Direction\",1.0,AUTHORITY[\"Esri\",100007]],UNIT[\"Meter\",1.0,AUTHORITY[\"EPSG\",9001]],AUTHORITY[\"Esri\",105700]]"u8,
            0);
        handle.Close();
    }
    #endregion

    #region Dispose Pattern
    private bool disposedValue;
    public virtual void Dispose(bool disposing)
    {
        if (!disposedValue)
        {
            if (disposing)
            {
                viewHandle.ReleasePointer();
                viewHandle.Dispose();
                viewAccessor.Dispose();
                memoryMappedFile.Dispose();
            }
            disposedValue = true;
        }
    }
    internal unsafe void ExportSS_Inf_Pair(string outputFolder)
    {
        using var ssFile = File.OpenWrite(Path.Combine(outputFolder, "SS.txt"));
        using var ifFile = File.OpenWrite(Path.Combine(outputFolder, "IF.txt"));
        var SideScanFrames = FrameByType[SurveyType.SideScan];
        var InterferometricFrames = FrameByType[SurveyType.ThreeDimensional];
        var ssStringBuilder = new StringBuilder(2800 * 4 + 2 + 2800 * 19);
        var ifStringBuilder = new StringBuilder(2800 * 2 * 19);
        int n = 0;
        foreach (nuint ss in SideScanFrames)
        {
            ref readonly Frame ssFrame = ref Unsafe.AsRef<Frame>((void*)ss);
            ref Frame ifFrame = ref Unsafe.AsRef<Frame>((void*)InterferometricFrames[n]);
            var ssStride = (ssFrame.MaxRange / 2800d) * (.3048 * 100);
            var ssHalfStep = .5 * ssStride;

            ReadOnlySpan<byte> ssBytes = new(Unsafe.Add<byte>((void*)ss, ssFrame.HeaderSize), 2800);
            int i = -1399; bool left = true;
            foreach (byte item in ssBytes)
            {
                ssStringBuilder.Append(double.FusedMultiplyAdd(i, ssStride, ssHalfStep)).Append(' ').Append(item).Append(' ');
                if (left && i == 0)
                { left = false; }
                else i++;
            }
            ssStringBuilder.AppendLine();
            ssFile.Write(Encoding.UTF8.GetBytes(ssStringBuilder.ToString()));
            ssStringBuilder.Clear();

            ref ThreeDimensionalFrameHeader frame3DHeader = ref Unsafe.AsRef<ThreeDimensionalFrameHeader>
                (Unsafe.Add<byte>(Unsafe.AsPointer<Frame>(ref ifFrame), ifFrame.HeaderSize));

            Span<float> leftMeasurements = new(Unsafe.Add<byte>(Unsafe.AsPointer(ref frame3DHeader), frame3DHeader.HeaderSize), frame3DHeader.NumberOfLeftBytes / 4);
            Span<float> rightMeasurements = new(Unsafe.Add<byte>(Unsafe.AsPointer(ref frame3DHeader), frame3DHeader.HeaderSize + frame3DHeader.NumberOfLeftBytes), frame3DHeader.NumberOfRightBytes / 4);
            Span<(float x, float y)> leftPairs = MemoryMarshal.Cast<float, (float x, float y)>(leftMeasurements).ToArray().AsSpan();
            Span<(float x, float y)> rightPairs = MemoryMarshal.Cast<float, (float x, float y)>(rightMeasurements);

            MemoryExtensions.Reverse(leftPairs);

            for (int k = 0; k < 1400 - leftPairs.Length; k++)
            {
                ifStringBuilder.Append("-9999 -9999 ");
            }

            for (int k = 0; k < leftPairs.Length; k++)
            {
                ifStringBuilder.Append(leftPairs[k].x * (-.3048 * 100)).Append(' ').
                    Append(leftPairs[k].y * (.3048 * 100)).Append(' ');
            }

            for (int k = 0; k < rightPairs.Length; k++)
            {
                ifStringBuilder.Append(rightPairs[k].x * (.3048 * 100)).Append(' ').
                    Append(rightPairs[k].y * (.3048 * 100)).Append(' ');
            }

            for (int k = 0; k < 1400 - rightPairs.Length; k++)
            {
                ifStringBuilder.Append("-9999 -9999 ");
            }

            ifStringBuilder.AppendLine();
            ifFile.Write(Encoding.UTF8.GetBytes(ifStringBuilder.ToString()));
            ifStringBuilder.Clear();
            n++;
        }

    }
    internal unsafe void ExportSS_Inf_PairCNN(string outputFolder)
    {
        using var ssFile = File.OpenWrite(Path.Combine(outputFolder, "SS06.dat"));
        var SideScanFrames = FrameByType[SurveyType.SideScan];
        var ssStringBuilder = new StringBuilder(2800 * 4);

        var InterferometricFrames = FrameByType[SurveyType.ThreeDimensional];
        using var ifFile = File.OpenWrite(Path.Combine(outputFolder, "IF06.txt"));
        var ifStringBuilder = new StringBuilder(2800 * 2 * 19);

        int n = 0;
        foreach (nuint ss in SideScanFrames)
        {
            ref readonly Frame ssFrame = ref Unsafe.AsRef<Frame>((void*)ss);

            var ssStride = (ssFrame.MaxRange / 1400d);
            var ssHalfStep = .5 * ssStride;

            ref Frame ifFrame = ref Unsafe.AsRef<Frame>((void*)InterferometricFrames[n]);
            ref ThreeDimensionalFrameHeader frame3DHeader = ref Unsafe.AsRef<ThreeDimensionalFrameHeader>
             (Unsafe.Add<byte>(Unsafe.AsPointer<Frame>(ref ifFrame), ifFrame.HeaderSize));

            Span<float> leftMeasurements = (new Span<float>(Unsafe.Add<byte>(Unsafe.AsPointer(ref frame3DHeader), frame3DHeader.HeaderSize), frame3DHeader.NumberOfLeftBytes / 4))[..^5];
            Span<float> rightMeasurements = (new Span<float>(Unsafe.Add<byte>(Unsafe.AsPointer(ref frame3DHeader), frame3DHeader.HeaderSize + frame3DHeader.NumberOfLeftBytes), frame3DHeader.NumberOfRightBytes / 4))[..^5];
            Span<(float x, float y)> leftPairs = MemoryMarshal.Cast<float, (float x, float y)>(leftMeasurements);
            Span<(float x, float y)> rightPairs = MemoryMarshal.Cast<float, (float x, float y)>(rightMeasurements);

            double leftMaxDist = double.Hypot(leftPairs[^1].x, leftPairs[^1].y);
            double rightMaxDist = double.Hypot(rightPairs[^1].x, rightPairs[^1].y);

            for (int m = 0; m < leftPairs.Length; m++)
            {
                ifStringBuilder.Append(leftPairs[m].x).Append(',').
                    Append(leftPairs[m].y).Append(',');
            }

            ifStringBuilder.AppendLine();
            ifFile.Write(Encoding.UTF8.GetBytes(ifStringBuilder.ToString()));
            ifStringBuilder.Clear();

            for (int m = 0; m < rightPairs.Length; m++)
            {
                ifStringBuilder.Append(rightPairs[m].x).Append(',').
                    Append(rightPairs[m].y).Append(',');
            }

            ifStringBuilder.AppendLine();
            ifFile.Write(Encoding.UTF8.GetBytes(ifStringBuilder.ToString()));
            ifStringBuilder.Clear();


            ReadOnlySpan<byte> ssBytes = new(Unsafe.Add<byte>((void*)ss, ssFrame.HeaderSize), 2800);
            ssStringBuilder.Clear();
            ssStringBuilder.Append("255,").Append(ssFrame.WaterDepth).Append(',');
            int k = 0;
            double range = double.FusedMultiplyAdd(ssStride, k, ssHalfStep);

            for (int j = 1399; j >= 0 && range <= leftMaxDist; j--)
            {
                ssStringBuilder.Append(ssBytes[j]).Append(',').Append(range.ToString("0.######")).Append(',');
                k++;
                range = double.FusedMultiplyAdd(ssStride, k, ssHalfStep);
            }

            ssStringBuilder.AppendLine();
            ssFile.Write(Encoding.UTF8.GetBytes(ssStringBuilder.ToString()));
            ssStringBuilder.Clear();

            ssStringBuilder.Append("255,").Append(ssFrame.WaterDepth).Append(',');
            k = 0;
            range = double.FusedMultiplyAdd(ssStride, k, ssHalfStep);
            for (int j = 1400; j < 2800 && range <= rightMaxDist; j++)
            {
                ssStringBuilder.Append(ssBytes[j]).Append(',').Append(range.ToString("0.######")).Append(',');
                k++;
                range = double.FusedMultiplyAdd(ssStride, k, ssHalfStep);
            }

            ssStringBuilder.AppendLine();
            ssFile.Write(Encoding.UTF8.GetBytes(ssStringBuilder.ToString()));
            ssStringBuilder.Clear();

            n++;
        }


    }
    void IDisposable.Dispose()
    {
        // Do not change this code. Put cleanup code in 'Dispose(bool disposing)' method
        Dispose(disposing: true);
        GC.SuppressFinalize(this);
    }
    #endregion Dispose Pattern
};