using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Runtime.CompilerServices;
using Microsoft.Win32.SafeHandles;
using System.Globalization;
using System.Buffers;
using System.Diagnostics.CodeAnalysis;
using System.IO.MemoryMappedFiles;
using System.Collections.ObjectModel;

namespace SL3Reader
{

    [DebuggerDisplay("{Name}")]
    public class SL3Reader : IDisposable
    {
        #region Public properties
        public ReadOnlyCollection<nuint> Frames { get; }
        public ReadOnlyCollection<GeoPoint> AugmentedCoordinates { get; }
        public ReadOnlyDictionary<SurveyType, ReadOnlyCollection<nuint>> IndexByType { get; }
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

            if (len < (SLFileHeader.Size + Frame.MinimumInitSize))
                throw new EndOfStreamException("The file is too short to be valid.");

            memoryMappedFile = MemoryMappedFile.CreateFromFile(
                File.OpenHandle(path!, FileMode.Open, FileAccess.Read, FileShare.Read, FileOptions.RandomAccess, 0L),
                null, len, MemoryMappedFileAccess.Read, HandleInheritability.None, false);
            viewAccessor = memoryMappedFile.CreateViewAccessor(0, len, MemoryMappedFileAccess.Read);
            viewHandle = viewAccessor.SafeMemoryMappedViewHandle;
            byte* ptr = null;
            viewHandle.AcquirePointer(ref ptr);

            ((SLFileHeader*)ptr)->ThrowIfInvalidFormatDetected();
            byte* maxPtr = ptr + len;

            ptr += SLFileHeader.Size; // Advance for the first frame

            // Init Frames
            const long averageFrameSize = 2118L; // Empirically set value to avoid frequent resize of the underlying array.
            int estimatedCount = (int)(len / averageFrameSize);
            ReadOnlyCollectionBuilder<nuint> frames = new(estimatedCount);

            // Init index lists
            int p8 = estimatedCount / 8, p4 = estimatedCount / 4, p10 = estimatedCount / 10;
            ReadOnlyCollectionBuilder<nuint> Primary = new(p8),
                                             Secondary = new(p8),
                                             DownScan = new(p10),
                                             LeftSidescan = new(),
                                             RightSidescan = new(),
                                             SideScan = new(p10),
                                             Unknown6 = new(),
                                             Unknown7 = new(p4),
                                             Unknown8 = new(p4),
                                             ThreeDimensional = new(p10),
                                             DebugDigital = new(),
                                             DebugNoise = new();

            ReadOnlyCollectionBuilder<int> coordinate3DHelper = new(p10);
            ReadOnlyCollectionBuilder<int> coordinateSidescanHelper = new(p10);

            // Init time
            Frame* currentFrame = (Frame*)ptr;
            Frame.InitTimestampBase(currentFrame->HardwareTime);

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
                    case SurveyType.LeftSidescan:
                        LeftSidescan.Add(current);
                        break;
                    case SurveyType.RightSidescan:
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
                }
            }

            // Init AugmentedCoordinates
            AugmentedCoordinates = frames.Count is 0 ?
                new ReadOnlyCollectionBuilder<GeoPoint>().ToReadOnlyCollection() :
                AugmentTrajectory(frames);

            Frames = frames.ToReadOnlyCollection();

            // Populate major index
            IndexByType = new SortedDictionary<SurveyType, ReadOnlyCollection<nuint>>()
            {
                { SurveyType.Primary, Primary.ToReadOnlyCollection() },
                { SurveyType.Secondary, Secondary.ToReadOnlyCollection() },
                { SurveyType.DownScan, DownScan.ToReadOnlyCollection() },
                { SurveyType.LeftSidescan, LeftSidescan.ToReadOnlyCollection() },
                { SurveyType. RightSidescan, RightSidescan.ToReadOnlyCollection() },
                { SurveyType.SideScan, SideScan.ToReadOnlyCollection() },
                { SurveyType.Unknown6, Unknown6.ToReadOnlyCollection() },
                { SurveyType.Unknown7, Unknown7.ToReadOnlyCollection() },
                { SurveyType.Unknown8, Unknown8.ToReadOnlyCollection() },
                { SurveyType.ThreeDimensional, ThreeDimensional.ToReadOnlyCollection() },
                { SurveyType.DebugDigital, DebugDigital.ToReadOnlyCollection() },
                { SurveyType.DebugNoise, DebugNoise.ToReadOnlyCollection() }
            }.AsReadOnly();

            Coordinate3DHelper = coordinate3DHelper.ToReadOnlyCollection();

            return;

            // Local fuctions:

            static ReadOnlyCollection<GeoPoint> AugmentTrajectory(ReadOnlyCollectionBuilder<nuint> frames)
            {
                const double lim = 1.2d;
                const double C = 1.4326d; // Empirical, around √2.

                ReadOnlyCollectionBuilder<GeoPoint> coordinates = new(frames.Count);

                (double x0, double y0, double z0, double v0, double t0, double d0) = ((Frame*)frames[0])->QueryMetric();
                coordinates.Add(new(x0, y0, d0, z0, 0)); // The first one.
                double distance = 0, xprev = x0, yprev = y0;

                for (int i = 1, frameCount = frames.Count; i != frameCount;)
                {
                    Frame* frame = (Frame*)frames[i++];


                    (double x1, double y1, double z1, double v1, double t1, double d1) =
                        frame->QueryMetric();

                    (double sin0, double cos0) = double.SinCos(d0);
                    (double sin1, double cos1) = double.SinCos(d1);

                    double vx0 = cos0 * v0,
                           vy0 = sin0 * v0,
                           vx1 = cos1 * v1,
                           vy1 = sin1 * v1,
                           dt = t1 - t0;

                    x0 += C * .5d * (vx0 + vx1) * dt;
                    y0 += C * .5d * (vy0 + vy1) * dt;

                    d0 = d1; t0 = t1; v0 = v1;

                    if (frame->SurveyType is SurveyType.Primary or SurveyType.Secondary or
                        SurveyType.Unknown7 or SurveyType.Unknown8)
                    {
                        double dy = y0 - y1;
                        if (double.Abs(dy) > lim)
                            y0 = y1 + double.CopySign(lim, dy);

                        double dx = x0 - x1;
                        if (double.Abs(dx) > lim)
                            x0 = x1 + double.CopySign(lim, dx);
                    }
                    else
                    {
                        double dy = y0 - y1;
                        if (double.Abs(dy) > 50) // Detect seriuos errors.
                            y0 = frame->Y;

                        double dx = x0 - x1;
                        if (double.Abs(dx) > 50) // Detect seriuos errors.
                            x0 = frame->X;
                    }

                    coordinates.Add(new(x0, y0, d0, z1, distance += double.Hypot(x0 - xprev, y0 - yprev)));
                    xprev = x0; yprev = y0;
                }
                return coordinates.ToReadOnlyCollection();
            }
        }

        public unsafe void ExportRoutePoints(string path)
        {
            ReadOnlyCollection<nuint> frames = Frames;
            int count = frames.Count;
            using StreamWriter textWriter = new(path,
                new FileStreamOptions()
                {
                    Access = FileAccess.Write,
                    Mode = FileMode.Create,
                    Share = FileShare.Read,
                    Options = FileOptions.SequentialScan,
                    PreallocationSize = 151 * count, // Empirical guess.
                    BufferSize = 151 * count
                });

            textWriter.BaseStream.Write("CampaignID[#],DateTime[UTC],SurveyType,WaterDepth[Feet],Longitude[°WGS48],Latitude[°WGS84],GNSSAltitude[Feet_WGS84Ellipsoid],GNSSHeading[rad_azimuth],GNSSSpeed[m/s],MagneticHeading[rad_azimuth],MinRange[Feet],MaxRange[Feet],WaterTemperature[°C],WaterSpeed[Feet],HardwareTime[ms],Frequency,Milliseconds[ms],AugmentedX[m],AugmentedY[m]"u8);

            // TODO: Filter is not working!
            //ReadOnlyCollection<nuint> frames = filter is SurveyType.All ? Frames : IndexByType[filter];

            ReadOnlyCollection<GeoPoint> augmentedCoordinates = AugmentedCoordinates;

            scoped Span<char> buffer = stackalloc char[256];
            for (int i = 0; i != count; i++)
            {
                textWriter.Write(((Frame*)frames[i])->Format(buffer));
                textWriter.WriteLine(augmentedCoordinates[i].Format(buffer));
            }
            textWriter.Close();
        }

        public unsafe void ExportImagery(string path, SurveyType surveyType = SurveyType.SideScan)
        {
            ArgumentNullException.ThrowIfNull(nameof(path));

            ReadOnlyCollection<nuint> imageFrames = IndexByType[surveyType];
            if (imageFrames.Count < 1) return; // Return when no imagery exists.

            CultureInfo invariantCulture = CultureInfo.InvariantCulture;

            List<int> breakpoints = GetBreakPoints(imageFrames, out int maxHeight);
            int numberOfColumns = (int)((Frame*)imageFrames[0])->LengthOfEchoData;
            byte[] buffer = BitmapHelper.CreateBuffer(maxHeight, numberOfColumns);
            string[] worldJoin = new string[6] { "0", "", "", "0", "", "" };

            for (int i = 0, maxIndex = breakpoints.Count - 1; i < maxIndex; i++)
            {
                int first = breakpoints[i], final = breakpoints[1 + i];
                BitmapHelper.UpdateBuffer(
                    buffer, final - first, numberOfColumns,
                    out int fullStride,
                    out Span<byte> fileBuffer,
                    out Span<byte> pixelData);

                byte* offset = ((Frame*)imageFrames[0])->FrameType is FrameType.Basic ? (byte*)Frame.BasicSize : (byte*)Frame.ExtendedSizeV10;

                fixed (byte* pixelPtr = pixelData)
                    for (int j = first, k = 0; j < final; j++)
                        Buffer.MemoryCopy(imageFrames[j] + offset, pixelPtr + (fullStride * k++), numberOfColumns, numberOfColumns);

                string prefix = GetPrefix(surveyType);
                using SafeFileHandle handle = File.OpenHandle(Path.Combine(path, prefix + final + ".bmp"),
                    FileMode.Create, FileAccess.Write, FileShare.None, FileOptions.SequentialScan, fileBuffer.Length);
                RandomAccess.Write(handle, fileBuffer, 0);
                handle.Close();

                // World file

                // TODO: Remove intermediate solution:
                var firstStrip = AugmentedCoordinates[Frames.IndexOf(imageFrames[first])];
                var lastStrip = AugmentedCoordinates[Frames.IndexOf(imageFrames[final - 1])];
                var lastFrame = (Frame*)imageFrames[final - 1];

                // TODO: Nem jó!
                double XSize = -(lastStrip.Distance - firstStrip.Distance) / (final - first);
                double YSize = -10d * lastFrame->MaxRange * .3048d / numberOfColumns;
                worldJoin[1] = YSize.ToString(invariantCulture);
                worldJoin[2] = XSize.ToString(invariantCulture);
                worldJoin[4] = lastStrip.Distance.ToString(invariantCulture);
                worldJoin[5] = lastFrame->SurveyType is SurveyType.SideScan ? (-1400d * YSize).ToString(invariantCulture) : "0";

                File.WriteAllText(Path.Combine(path, prefix + final + ".bpw"), string.Join('\n', worldJoin!, 0, 6));
                // End world file
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
                float previousRange = ((Frame*)framesToCheck[0])->MaxRange;

                List<int> breakpoints = new(frameCount / 300) { 0 }; // ~300 empirical guess. 
                                                                     // 'i' have to be incremented to prevent double test.

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
            ReadOnlyCollection<nuint> unknown8Frames = IndexByType[SurveyType.Unknown8];
            int unknown8FrameCount = unknown8Frames.Count;
            if (unknown8FrameCount < 1) return; // Return when no U8 exists

            byte[] buffer = new byte[512];
            fixed (byte* p = buffer)
            {
                for (int i = 0; i < unknown8FrameCount - 1; i++)
                {
                    nuint ptr = unknown8Frames[i];
                    Buffer.MemoryCopy((byte*)(ptr + (nuint)((Frame*)ptr)->RelativeDataOffset), p, 512, 512);
                    for (int j = 0; j < 256; j += 2)
                    {
                        Debug.Print($"{buffer[j]}, {buffer[1 + j]}");
                    }
                }

            }
        }

        public unsafe void Export3D(string path, bool includeUnreliable = false, bool magneticHeading = false)
        {
            ArgumentNullException.ThrowIfNull(nameof(path));

            const string doubleFormat = "0.####";
            CultureInfo invariantCulture = CultureInfo.InvariantCulture;

            ReadOnlyCollection<nuint> frames3D = IndexByType[SurveyType.ThreeDimensional];
            int frames3DLength = frames3D.Count;
            if (frames3DLength < 1) return;
            ReadOnlyCollection<GeoPoint> augmentedCoordinates = AugmentedCoordinates;
            ReadOnlyCollection<int> coordinate3DHelper = Coordinate3DHelper;

            using StreamWriter streamWriter = File.CreateText(path!);
            streamWriter.BaseStream.Write("CampaignID,DateTime,X[Lowrance_m],Y[Lowrance_m],Z[m_WGS84Ellipsoid],Depth[m],Angle[°],Distance[m],Reliability\r\n"u8);

            string[] stringArray = GC.AllocateUninitializedArray<string>(9);

            for (int i = 0; i < frames3DLength; i++)
            {
                Frame* frame = (Frame*)frames3D[i];
                ThreeDimensionalFrameHeader* header = (ThreeDimensionalFrameHeader*)((byte*)frame + frame->RelativeDataOffset);

                // TODO: Remove intermediate solution:
                GeoPoint augmentedCoordinate = augmentedCoordinates[coordinate3DHelper[i]];
                byte* measurements = (byte*)header + ThreeDimensionalFrameHeader.Size;

                double centralX = augmentedCoordinate.X,
                       centralY = augmentedCoordinate.Y,
                       centralZ = .3048 * augmentedCoordinate.Altitude;

                (double sin, double cos) =
                    double.SinCos((magneticHeading ? frame->MagneticHeading : frame->GNSSHeading) - .5 * double.Pi);

                stringArray[0] = frame->CampaignID.ToString();
                stringArray[1] = frame->Timestamp.ToString("yyyy'-'MM'-'dd HH':'mm':'ss.fff'Z'", invariantCulture);
                stringArray[8] = "R";

                // *************************** THIS IS TERRIBLE ***************************
                // *************************** Refactor    ASAP ***************************
                // Left side
                for (byte* limit = measurements + header->NumberOfLeftBytes;
                    measurements < limit;
                    measurements += InterferometricMeasurement.Size)
                {
                    InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;
                    double delta = measurement->Delta,
                           depth = measurement->Depth;
                    double distance = .3048 * double.Hypot(delta, depth);
                    double angle = 90 - (360 / double.Tau) * double.Atan2(depth, delta);

                    delta *= -.3048; // Negative side 

                    stringArray[2] = double.FusedMultiplyAdd(delta, sin, centralX).ToString(doubleFormat, invariantCulture); // Azimuthal direction
                    stringArray[3] = double.FusedMultiplyAdd(delta, cos, centralY).ToString(doubleFormat, invariantCulture);
                    stringArray[4] = double.FusedMultiplyAdd(-.3048, depth, centralZ).ToString(doubleFormat, invariantCulture);
                    stringArray[5] = (.3048 * depth).ToString(doubleFormat, invariantCulture);
                    stringArray[6] = angle.ToString(doubleFormat, invariantCulture);
                    stringArray[7] = distance.ToString(doubleFormat, invariantCulture);

                    streamWriter.WriteLine(string.Join(',', stringArray));
                }

                // Right side
                for (byte* limit = measurements + header->NumberOfRightBytes;
                    measurements < limit;
                    measurements += InterferometricMeasurement.Size)
                {
                    InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;
                    double delta = measurement->Delta,
                           depth = measurement->Depth;
                    double distance = .3048 * double.Hypot(delta, depth);
                    double angle = 90 - (360 / double.Tau) * double.Atan2(depth, delta);

                    delta *= .3048; // Positive side 

                    stringArray[2] = double.FusedMultiplyAdd(delta, sin, centralX).ToString(doubleFormat, invariantCulture); // Azimuthal direction
                    stringArray[3] = double.FusedMultiplyAdd(delta, cos, centralY).ToString(doubleFormat, invariantCulture);
                    stringArray[4] = double.FusedMultiplyAdd(-.3048, depth, centralZ).ToString(doubleFormat, invariantCulture);
                    stringArray[5] = (.3048 * depth).ToString(doubleFormat, invariantCulture);
                    stringArray[6] = angle.ToString(doubleFormat, invariantCulture);
                    stringArray[7] = distance.ToString(doubleFormat, invariantCulture);

                    streamWriter.WriteLine(string.Join(',', stringArray));
                }

                if (includeUnreliable)
                {
                    stringArray[8] = "U";
                    for (byte* limit = measurements + header->NumberOfUnreliableLeftBytes;
                        measurements < limit;
                        measurements += InterferometricMeasurement.Size)
                    {
                        InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;
                        if (IsValidMeasurement(measurement, out double delta, out double depth))
                        {
                            double distance = .3048 * double.Hypot(delta, depth);
                            double angle = 90 - (360 / double.Tau) * double.Atan2(depth, delta);

                            delta *= -.3048; // Negative side 

                            stringArray[2] = double.FusedMultiplyAdd(delta, sin, centralX).ToString(doubleFormat, invariantCulture); // Azimuthal direction
                            stringArray[3] = double.FusedMultiplyAdd(delta, cos, centralY).ToString(doubleFormat, invariantCulture);
                            stringArray[4] = double.FusedMultiplyAdd(-.3048, depth, centralZ).ToString(doubleFormat, invariantCulture);
                            stringArray[5] = (.3048 * depth).ToString(doubleFormat, invariantCulture);
                            stringArray[6] = angle.ToString(doubleFormat, invariantCulture);
                            stringArray[7] = distance.ToString(doubleFormat, invariantCulture);

                            streamWriter.WriteLine(string.Join(',', stringArray));
                        }
                    }

                    for (byte* limit = measurements + header->NumberOfUnreliableRightBytes;
                        measurements < limit;
                        measurements += InterferometricMeasurement.Size)
                    {
                        InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;
                        if (IsValidMeasurement(measurement, out double delta, out double depth))
                        {
                            double distance = .3048 * double.Hypot(delta, depth);
                            double angle = 90 - (360 / double.Tau) * double.Atan2(depth, delta);

                            delta *= .3048; // Positive side 

                            stringArray[2] = double.FusedMultiplyAdd(delta, sin, centralX).ToString(doubleFormat, invariantCulture); // Azimuthal direction
                            stringArray[3] = double.FusedMultiplyAdd(delta, cos, centralY).ToString(doubleFormat, invariantCulture);
                            stringArray[4] = double.FusedMultiplyAdd(-.3048, depth, centralZ).ToString(doubleFormat, invariantCulture);
                            stringArray[5] = (.3048 * depth).ToString(doubleFormat, invariantCulture);
                            stringArray[6] = angle.ToString(doubleFormat, invariantCulture);
                            stringArray[7] = distance.ToString(doubleFormat, invariantCulture);

                            streamWriter.WriteLine(string.Join(',', stringArray));
                        }
                    }
                }
            }

            [SkipLocalsInit]
            static bool IsValidMeasurement(InterferometricMeasurement* measurement, out double delta, out double depth)
            {
                delta = measurement->Delta;
                depth = measurement->Depth;
                return delta is not < 0.001 and not > 5000.0 && depth is not < 1 and not > 600.0;
            }
        }

        #region Dispose Pattern
        private bool disposedValue;
        public virtual void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                if (disposing)
                {
                    viewHandle.ReleasePointer();
                    viewHandle.Close();
                    viewAccessor.Dispose();
                    memoryMappedFile.Dispose();
                }
                disposedValue = true;
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
}