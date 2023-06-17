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
        #endregion End Private variables

        [SkipLocalsInit]
        public unsafe SL3Reader(string path)
        {
            byte* ptr = null;
            long len = new FileInfo(path).Length;

            if (len < (SLFileHeader.Size + Frame.MinimumInitSize))
                throw new EndOfStreamException("The file is too short to be valid.");

            memoryMappedFile = MemoryMappedFile.CreateFromFile(
                File.OpenHandle(path, FileMode.Open, FileAccess.Read, FileShare.Read, FileOptions.RandomAccess, 0L),
                null, len, MemoryMappedFileAccess.Read, HandleInheritability.None, false);
            viewAccessor = memoryMappedFile.CreateViewAccessor(0, len, MemoryMappedFileAccess.Read);
            viewHandle = viewAccessor.SafeMemoryMappedViewHandle;
            viewHandle.AcquirePointer(ref ptr);

            ((SLFileHeader*)ptr)->ThrowIfInvalidFormatDetected();
            ptr += SLFileHeader.Size; // Advance for the first frame

            // Init Frames
            const long averageFrameSize = 2118L; // Empirically set value to avoid frequent resize of the underlying array.
            int estimatedCount = (int)(len / averageFrameSize);
            ReadOnlyCollectionBuilder<nuint> frames = new(estimatedCount);

            // Init index lists
            ReadOnlyCollectionBuilder<nuint> Primary = new(estimatedCount / 8),
                                             Secondary = new(estimatedCount / 8),
                                             DownScan = new(estimatedCount / 10),
                                             LeftSidescan = new(),
                                             RightSidescan = new(),
                                             SideScan = new(estimatedCount / 10),
                                             Unknown6 = new(),
                                             Unknown7 = new(estimatedCount / 4),
                                             Unknown8 = new(estimatedCount / 4),
                                             ThreeDimensional = new(estimatedCount / 10),
                                             DebugDigital = new(),
                                             DebugNoise = new();

            // Init time
            Frame* currentFrame = (Frame*)ptr; // Transfer to static private constructor!
            Frame.InitTimestampBase(currentFrame->HardwareTime);

            // Load frames


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

            AugmentTrajectory();
        }

        public void ExportToCSV(string path, [ConstantExpected] SurveyType filter = SurveyType.All)
        {
            File.WriteAllLines(path,
                EnumerateFrames(Frames, AugmentedCoordinates, filter is SurveyType.All ? null : IndexByType[filter]));

            static IEnumerable<string> EnumerateFrames(
                                                        IReadOnlyList<IFrame> frames,
                                                        List<GeoPoint> augmentedCoordinates,
                                                        IReadOnlyList<int>? typeList = null)
            {

                yield return "CampaignID[#],DateTime[UTC],SurveyType,WaterDepth[Feet],Longitude[°WGS48],Latitude[°WGS84],GNSSAltitude[Feet_WGS84Ellipsoid],GNSSHeading[rad_azimuth],GNSSSpeed[m/s],MagneticHeading[rad_azimuth],MinRange[Feet],MaxRange[Feet],WaterTemperature[°C],WaterSpeed[Feet],HardwareTime[ms],Frequency,Milliseconds[ms],AugmentedX[m],AugmentedY[m]";

                if (typeList is null)
                    for (int i = 0, count = frames.Count; i < count; i++)
                        yield return frames[i].ToString() + ',' + augmentedCoordinates[i];

                else
                    for (int i = 0, count = typeList.Count, j = count != 0 ? typeList![0] : 0; i < count; i++, j = typeList![i])
                        yield return frames[j].ToString() + ',' + augmentedCoordinates[j];
            }
        }

        private List<int> GetBreakPoints(List<int> framesToCheck, out int contiguousLength)
        {
            int i = 0;
            IReadOnlyList<IFrame> frames = Frames;
            int frameCount = framesToCheck.Count;
            float previousRange = frames[framesToCheck[i]].MaxRange;
            List<int> breakpoints = new(frameCount / 300) { i }; // ~300 empirical guess.

            for (; i < frameCount; i++)
            {
                float currentRange = frames[framesToCheck[i]].MaxRange;
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

        public unsafe void ExportImagery(string path, SurveyType surveyType = SurveyType.SideScan)
        {
            ArgumentNullException.ThrowIfNull(nameof(path));

            if (Directory.Exists(path!))
                Directory.Delete(path!, true);
            path = Directory.CreateDirectory(path!).FullName;

            ReadOnlyCollection<nuint> imageFrames = IndexByType[surveyType];
            if (imageFrames.Count < 1) return; // Return when no imagery exists.
            
            uint numberOfColumns = ((Frame*)imageFrames[0])->LengthOfEchoData;
            string prefix = GetPrefix(surveyType);

            List<int> breakpoints = GetBreakPoints(imageFrames, out int maxHeight);

            byte[] buffer = BitmapHelper.CreateBuffer(maxHeight, numberOfColumns);

            for (int i = 0, maxIndex = breakpoints.Count - 1; i < maxIndex; i++)
            {
                int first = breakpoints[i], final = breakpoints[1 + i];
                BitmapHelper.UpdateBuffer(
                    buffer, final - first, numberOfColumns,
                    out int fullStride,
                    out Span<byte> fileBuffer,
                    out Span<byte> pixelData);

                for (int j = first, k = 0; j < final; j++)
                {
                    long dataOffset = frames[imageFrames[j]].DataOffset;
                    SeekExactly(dataOffset);

                    ReadExactly(pixelData.Slice(fullStride * k++, numberOfColumns));
                }

                using SafeFileHandle handle = File.OpenHandle(Path.Combine(path, prefix + final + ".bmp"),
                    FileMode.CreateNew, FileAccess.Write, FileShare.None, FileOptions.SequentialScan);
                RandomAccess.Write(handle, fileBuffer, 0);

                // World file
                CultureInfo InvariantCulture = CultureInfo.InvariantCulture;
                var firstStrip = AugmentedCoordinates[imageFrames[first]];
                var lastStrip = AugmentedCoordinates[imageFrames[final - 1]];
                var lastFrame = frames[imageFrames[final - 1]];

                double XSize = -(lastStrip.Distance - firstStrip.Distance) / (final - first - 1);
                double YSize = -10 * lastFrame.MaxRange * .3048 / numberOfColumns;
                string WorldString = string.Join("\r\n",
                        new string[6]
                         {"0",
                         YSize.ToString(InvariantCulture),
                         XSize.ToString(InvariantCulture),
                         "0",
                         lastStrip.Distance.ToString(InvariantCulture),
                         lastFrame.SurveyType is SurveyType.SideScan ? (-1400*YSize).ToString(): "0"}, 0, 6);

                File.WriteAllText(Path.Combine(path, prefix + final + ".bpw"), WorldString);
                // End world file
            }
        }


        [SkipLocalsInit, MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static string GetPrefix(SurveyType surveyType) =>
            surveyType switch
            {
                SurveyType.SideScan => "SS_",
                SurveyType.DownScan => "DS_",
                SurveyType.Primary => "PS_",
                SurveyType.Secondary => "SecS_",
                SurveyType.Unknown8 => "U8_",
                SurveyType.Unknown7 => "U7_",
                _ => "UNKNOWN_"
            };

        public unsafe void ExamineUnknown8Datasets()
        {
            var frames = Frames;
            int frameCount = frames.Count; // Initialize the frames.
            if (frameCount < 1) return;

            List<int> unknown8Frames = IndexByType[SurveyType.Unknown8];
            int unknown8FrameCount = unknown8Frames.Count;
            if (frameCount < 1) return; // Return when no U8 exists

            var buffer = new byte[512];
            fixed (byte* p = &buffer[0])
            {
                for (int i = 0; i < unknown8FrameCount - 1; i++)
                {
                    IFrame frame = frames[unknown8Frames[i]];
                    SeekExactly(frame.DataOffset);
                    Read(new(p, 512));
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


            IReadOnlyList<IFrame> frames = Frames;
            int frameCount = frames.Count; // Initialize the frames.
            if (frameCount < 1) return;

            List<int> frames3D = IndexByType[SurveyType.ThreeDimensional];
            int frames3DLength = frames3D.Count;
            if (frames3DLength < 1) return;
            List<GeoPoint> augmentedCoordinates = AugmentedCoordinates;

            using StreamWriter streamWriter = File.CreateText(path);
            streamWriter.WriteLine("CampaignID,DateTime,X[Lowrance_m],Y[Lowrance_m],Z[m_WGS84Ellipsoid],Depth[m],Angle[°],Distance[m],Reliability");

            string[] stringArray = GC.AllocateUninitializedArray<string>(9);

            ThreeDimensionalFrameHeader header = new();
            Span<byte> sHeader = new(&header, ThreeDimensionalFrameHeader.Size);

            byte[] measurementsArray = ArrayPool<byte>.Shared.Rent(800 * InterferometricMeasurement.Size);
            fixed (byte* reset = &measurementsArray[0])
            {
                for (int i = 0; i < frames3DLength; i++)
                {
                    int frame3DIndex = frames3D[i];
                    IFrame frame = frames[frame3DIndex];
                    GeoPoint augmentedCoordinate = augmentedCoordinates[frame3DIndex];

                    SeekExactly(frame.DataOffset);
                    ReadExactly(sHeader);
                    ReadExactly(new(reset, header.NumberOfUsedBytes));
                    byte* measurements = reset;

                    double centralX = augmentedCoordinate.X,
                           centralY = augmentedCoordinate.Y,
                           centralZ = .3048 * augmentedCoordinate.Altitude;

                    (double sin, double cos) =
                        double.SinCos((magneticHeading ? frame.MagneticHeading : frame.GNSSHeading) - .5 * double.Pi);

                    stringArray[0] = frame.CampaignID.ToString();
                    stringArray[1] = frame.Timestamp.ToString("yyyy'-'MM'-'dd HH':'mm':'ss.fff'Z'", invariantCulture);
                    stringArray[8] = "R";

                    // *************************** THIS IS TERRIBLE ***************************
                    // *************************** Refactor    ASAP ***************************
                    // Left side
                    for (byte* limit = measurements + header.NumberOfLeftBytes;
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
                    for (byte* limit = measurements + header.NumberOfRightBytes;
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
                        for (byte* limit = measurements + header.NumberOfUnreliableLeftBytes;
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

                        for (byte* limit = measurements + header.NumberOfUnreliableRightBytes;
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
            }
            ArrayPool<byte>.Shared.Return(measurementsArray);

            [SkipLocalsInit]
            static bool IsValidMeasurement(InterferometricMeasurement* measurement, out double delta, out double depth)
            {
                delta = measurement->Delta;
                depth = measurement->Depth;
                return delta is not < 0.001 and not > 5000.0 && depth is not < 1 and not > 250.0;
            }
        }

        [SkipLocalsInit, MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void SeekExactly(long offset)
        {
            if (Seek(offset, SeekOrigin.Begin) != offset)
                throw new IOException("Unable to seek!");
        }

        internal void AugmentTrajectory()
        {
            const double lim = 1.2d;
            const double C = 1.4326d; // Empirical, around √2.

            List<GeoPoint> coordinates = AugmentedCoordinates;
            IReadOnlyList<IFrame> frames = Frames;
            int frameCount = frames.Count; // Initialize the frames.
            if (frameCount < 1) return;

            (double x0, double y0, double z0, double v0, double t0, double d0) = frames[0].QueryMetric();
            coordinates.Add(new(x0, y0, d0, z0, 0)); // The first one.
            double distance = 0, xprev = x0, yprev = y0;

            for (int i = 1; i < frameCount; i++)
            {
                IFrame frame = frames[i];

                unsafe
                {
                    var fr = (Frame)frame;
                    var pos = fr.PositionOfFirstByte;
                    byte* ptr = (byte*)&fr;
                    int T = *(int*)&ptr[54];
                }

                (double x1, double y1, double z1, double v1, double t1, double d1) =
                    frame.QueryMetric();

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

                if (frame.SurveyType is SurveyType.Primary or SurveyType.Secondary or
                    SurveyType.Unknown7 or SurveyType.Unknown8)
                {

                    double dy = y0 - y1;
                    if (double.Abs(dy) > lim)
                        y0 = y1 + double.CopySign(lim, dy);

                    double dx = x0 - x1;
                    if (double.Abs(dx) > lim)
                        x0 = x1 + double.CopySign(lim, dx);
                }

                coordinates.Add(new(x0, y0, d0, z1, distance += double.Hypot(x0 - xprev, y0 - yprev)));
                xprev = x0; yprev = y0;
            }
        }

        #region Dispose Pattern
        private bool disposedValue;
        protected virtual void Dispose(bool disposing)
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

                // TODO: free unmanaged resources (unmanaged objects) and override finalizer
                // TODO: set large fields to null
                disposedValue = true;
            }
        }

        // // TODO: override finalizer only if 'Dispose(bool disposing)' has code to free unmanaged resources
        // ~SL3Reader()
        // {
        //     // Do not change this code. Put cleanup code in 'Dispose(bool disposing)' method
        //     Dispose(disposing: false);
        // }

        void IDisposable.Dispose()
        {
            // Do not change this code. Put cleanup code in 'Dispose(bool disposing)' method
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }
        #endregion Dispose Pattern
    };
}