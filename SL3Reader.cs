using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Runtime.CompilerServices;
using System.Threading;
using static System.Runtime.InteropServices.NativeMemory;
using Microsoft.Win32.SafeHandles;
using System.Numerics;
using System.Globalization;
using System.Security.Cryptography;
using System.Diagnostics.Metrics;

namespace SL3Reader
{
    [DebuggerDisplay("{Name}")]
    public class SL3Reader :
        FileStream,
        IEnumerable<IFrame>,
        IEnumerable,
        IReadOnlyList<IFrame>,
        IReadOnlyCollection<IFrame>
    {
        #region Frame support
        private List<IFrame> frames;
        public IReadOnlyList<IFrame> Frames => frames ??= CreateNewFrameList(this);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private List<IFrame> CreateNewFrameList()
        {
            const long averageFrameSize = 2118L; // Empirically set value to avoid frequent resize of the underlying array.
            return new((int)(Length / averageFrameSize));
        }

        private List<IFrame> CreateNewFrameList(IEnumerable<IFrame> collection)
        {
            List<IFrame> localFrames = CreateNewFrameList();
            InitIndexSupport(localFrames.Capacity);

            SortedDictionary<SurveyType, List<int>> typeIndex = indexByType;
            SortedDictionary<uint, List<int>> campaignIndex = indexByCampaign;

            int n = 0;
            foreach (IFrame frame in collection)
            {
                localFrames.Add(frame);
                typeIndex[frame.SurveyType].Add(n);
                if (campaignIndex.TryGetValue(frame.CampaignID, out List<int> campaignList))
                    campaignList.Add(n);
                else
                    campaignIndex.Add(frame.CampaignID, new List<int>(9) { n });

                n++;
            }
            return localFrames;
        }

        public int Count => Frames.Count; // Can't be read-only hence the Frames could be initialized.

        public IFrame this[int index] => Frames[index]; // Can't be read-only hence the Frames could be initialized.

        #endregion Frame support

        #region Augmented Coordinates
        private List<GeoPoint> augmentedCoordinates;
        public List<GeoPoint> AugmentedCoordinates => augmentedCoordinates ??= CreateNewCoordinateList();
        private List<GeoPoint> CreateNewCoordinateList()
        {
            const long averageFrameSize = 2118L; // Empirically set value to avoid frequent resize of the underlying array.
            return new((int)(Length / averageFrameSize));
        }
        #endregion Augmented Coordinates

        #region Indices
        private SortedDictionary<SurveyType, List<int>> indexByType;
        private SortedDictionary<uint, List<int>> indexByCampaign;
        public SortedDictionary<SurveyType, List<int>> IndexByType => indexByType;
        // Maybe useless, due to the drifting in 3D.
        public SortedDictionary<uint, List<int>> IndexByCampaign => indexByCampaign;
        private void InitIndexSupport(int estimatedCount)
        {
            indexByType = new()
            {
                { SurveyType.Primary, new(estimatedCount / 8) },
                { SurveyType.Secondary, new(estimatedCount / 8) },
                { SurveyType.DownScan, new(estimatedCount / 10) },
                { SurveyType.LeftSidescan,new() },
                { SurveyType. RightSidescan, new() },
                { SurveyType.SideScan, new(estimatedCount / 10) },
                { SurveyType.Unknown6,new() },
                { SurveyType.Unknown7, new(estimatedCount / 4) },
                { SurveyType.Unknown8, new(estimatedCount / 4) },
                { SurveyType.ThreeDimensional, new(estimatedCount / 10) },
                { SurveyType.DebugDigital, new() },
                { SurveyType.DebugNoise,new() }
            };

            indexByCampaign = new();
        }
        #endregion Indices
        public unsafe SL3Reader(string path) :
           base(path, FileMode.Open, FileAccess.Read,
           FileShare.Read, 4096, FileOptions.RandomAccess)
        {
            if (Length < (SLFileHeader.Size + Frame.MinimumInitSize))
                throw new EndOfStreamException("The file is too short to be valid.");

            SLFileHeader fileHeader = new();
            ReadExactly(new(&fileHeader, SLFileHeader.Size));

            if (!fileHeader.IsValidFormat(LogFileFormat.SL3))
                throw new InvalidDataException("Unsupported file type. Expected type SL3.");

            AugmentTrajectory();
        }

        public unsafe void ExportToCSV(string path, SurveyType? filter = null)
        {
            // StreamWriter is set to ASCII. If you add non-ASCII chars, revert to UTF-8.
            const string CSVHeader = "DateTime,SurveyType,WaterDepth,Longitude,Latitude,GNSSAltitude,GNSSHeading,GNSSSpeed,MagneticHeading,MinRange,MaxRange,WaterTemperature,WaterSpeed,HardwareTime,Frequency,Milliseconds,AugmentedX,AugmentedY";


            FileStreamOptions fileStreamOptions = new() { Access = FileAccess.Write, Mode = FileMode.OpenOrCreate, Share = FileShare.Read, Options = FileOptions.SequentialScan };
            using StreamWriter streamWriter = new(path, System.Text.Encoding.ASCII, fileStreamOptions);// Delay file open
            streamWriter.WriteLine(CSVHeader);

            if (filter is SurveyType internalFilter)
            {
                List<int> typeList = indexByType[internalFilter];
                int count = typeList.Count;
                var frames = Frames;

                for (int i = 0; i < count; i++)
                {
                    streamWriter.WriteLine(frames[typeList[i]].ToString()); // Write & WriteLine call the same
                                                                            //  "private unsafe void WriteSpan(ReadOnlySpan<char> buffer, bool appendNewLine)"
                                                                            // so adding the '\n' manually has no effect just causing platform dependent issues. 
                }
            }
            else
            {
                for (int i = 0, frameCount = Frames.Count; i < frameCount; i++)
                {
                    GeoPoint augmentedCoordinate = AugmentedCoordinates[i];
                    streamWriter.WriteLine(Frames[i].ToString() + ',' +
                                           augmentedCoordinate.ToString());
                    // Write & WriteLine call the same
                    //  "private unsafe void WriteSpan(ReadOnlySpan<char> buffer, bool appendNewLine)"
                    // so adding the '\n' manually has no effect just causing platform dependent issues.                                                                               // so ading the '\n' manually has no effect just causing platform dependent issues. 
                }
            }
            streamWriter.Close();
        }

        private List<int> GetBreakPoints(List<int> framesToCheck, out int contiguousLength)
        {
            int i = 0;
            var frames = Frames;
            int frameCount = framesToCheck.Count;
            float previousRange = frames[framesToCheck[0]].MaxRange;
            List<int> breakpoints = new(frameCount / 300) { 0 }; // ~300 empirical guess.

            for (; i < frameCount; i++)
            {
                float currentRange = frames[framesToCheck[i]].MaxRange;
                if (currentRange != previousRange)
                {
                    previousRange = currentRange;
                    breakpoints.Add(i);
                }
            }

            if (breakpoints[^1] != (i -= 1))
                breakpoints.Add(i); // There was no change in the range, so we have to add the last one.

            contiguousLength = 0;

            for (int j = 0,
                 maxIndex = breakpoints.Count - 1,
                 delta;
                 j < maxIndex;
                 j += 2)
            {
                if (contiguousLength < (delta = breakpoints[1 + j] - breakpoints[j]))
                    contiguousLength = delta;
            }

            return breakpoints;
        }

        public void ExportImagery(string path, SurveyType surveyType = SurveyType.SideScan)
        {
            ArgumentNullException.ThrowIfNull(nameof(path));

            if (Directory.Exists(path))
                Directory.Delete(path, true);
            path = Directory.CreateDirectory(path).FullName;

            var frames = Frames;
            int frameCount = frames.Count; // Initialize the frames.
            if (frameCount < 1) return;

            List<int> imageFrames = IndexByType[surveyType];
            if (imageFrames.Count < 1) return; // Return when no sidescan exists
            int numberOfColumns = (int)frames[imageFrames[0]].LengthOfEchoData;
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
                    if (Seek(dataOffset, SeekOrigin.Begin) != dataOffset)
                        throw new IOException("Unable to seek.");

                    ReadExactly(pixelData.Slice(fullStride * k++, numberOfColumns));
                }

                using SafeFileHandle handle = File.OpenHandle(Path.Combine(path, prefix + final + ".bmp"),
                    FileMode.CreateNew, FileAccess.Write, FileShare.None, FileOptions.SequentialScan);
                RandomAccess.Write(handle, fileBuffer, 0);
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
                    Seek(frame.DataOffset, SeekOrigin.Begin);
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
            
            IReadOnlyList<IFrame> frames = Frames;
            int frameCount = frames.Count; // Initialize the frames.
            if (frameCount < 1) return;

            List<int> frames3D = IndexByType[SurveyType.ThreeDimensional];
            int frames3DLength = frames3D.Count;
            if (frames3DLength < 1) return;

            using StreamWriter streamWriter = File.CreateText(path);
            streamWriter.WriteLine("Campaign,X,Y,Z,Reliability");

            List<GeoPoint> augmentedCoordinates = AugmentedCoordinates;
            ThreeDimensionalFrameHeader header = new();
            Span<byte> sHeader = new(&header, ThreeDimensionalFrameHeader.Size);
            byte* measurements = stackalloc byte[400 * InterferometricMeasurement.Size],
                  reset = measurements;

            for (int i = 0; i < frames3DLength; i++)
            {
                int frame3DIndex = frames3D[i];
                IFrame frame = frames[frame3DIndex];
                GeoPoint augmentedCoordinate = augmentedCoordinates[frame3DIndex];
                long offset = frame.DataOffset;
                if (Seek(offset, SeekOrigin.Begin) != offset)
                    throw new IOException("Unable to seek!");

                ReadExactly(sHeader);

                ReadExactly(new(measurements, header.NumberOfUsedBytes));

                uint campaignID = frame.CampaignID;
                double centralX = augmentedCoordinate.X, centralY = augmentedCoordinate.Y, centralZ = .3048 * augmentedCoordinate.Altitude;
                (double sin, double cos) = double.SinCos((magneticHeading ? frame.MagneticHeading : frame.GNSSHeading) - .5 * double.Pi);

                // Left side
                byte* limit = measurements + header.NumberOfLeftBytes;
                for (; measurements < limit; measurements += InterferometricMeasurement.Size)
                {
                    InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;

                    double delta = -.3048 * measurement->Delta; // Negative side
                    double x = double.FusedMultiplyAdd(delta, sin, centralX);
                    double y = double.FusedMultiplyAdd(delta, cos, centralY);
                    double z = double.FusedMultiplyAdd(-.3048, measurement->Depth, centralZ);

                    streamWriter.WriteLine($"{campaignID},{x},{y},{z},R");
                }

                // Right side
                limit = measurements + header.NumberOfRightBytes;
                for (; measurements < limit; measurements += InterferometricMeasurement.Size)
                {
                    InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;

                    double delta = .3048 * measurement->Delta; // Positive side
                    double x = double.FusedMultiplyAdd(delta, sin, centralX);
                    double y = double.FusedMultiplyAdd(delta, cos, centralY);
                    double z = double.FusedMultiplyAdd(-.3048, measurement->Depth, centralZ);

                    streamWriter.WriteLine($"{campaignID},{x},{y},{z},R");
                }

                if (includeUnreliable)
                {
                    limit = measurements + header.NumberOfUnreliableLeftBytes;
                    for (; measurements < limit; measurements += InterferometricMeasurement.Size)
                    {
                        InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;
                        float delta = measurement->Delta;
                        if (delta is < 0.001f or > 5000.0f) continue;
                        float depth = measurement->Depth;
                        if (depth is < 1f or > 250.0f) continue;

                        streamWriter.WriteLine($"{-delta},{i},{depth},U");
                    }

                    limit = measurements + header.NumberOfUnreliableRightBytes;
                    for (; measurements < limit; measurements += InterferometricMeasurement.Size)
                    {
                        InterferometricMeasurement* measurement = (InterferometricMeasurement*)measurements;
                        float delta = measurement->Delta;
                        if (delta is < 0.001f or > 5000.0f) continue;
                        float depth = measurement->Depth;
                        if (depth is < 1f or > 250.0f) continue;

                        streamWriter.WriteLine($"{delta},{i},{depth},U");
                    }
                }

                measurements = reset;
            }
            streamWriter.Close();
        }

        internal void AugmentTrajectory()
        {
            const double lim = 1.2d;
            const double C = 1.4326d; // Empirical, around √2.

            List<GeoPoint> coordinates = AugmentedCoordinates;
            var frames = Frames;
            int frameCount = frames.Count; // Initialize the frames.
            if (frameCount < 1) return;

            (double x, double y, double v, double t, double d) =
                frames[0].UnpackNavParameters();
            coordinates.Add(new(x, y, d, frames[0].GNSSAltitude, 0));

            for (int i = 1; i < frameCount; i++)
            {
                IFrame frame = frames[i];
                (double nx, double ny, double nv, double nt, double nd) =
                    frame.UnpackNavParameters();
                double sv = nv + v,
                       dt = nt - t;
                double vec = C * .5d * sv * dt,
                       ad = (nv * nd + v * d) / sv;
                (double sin, double cos) = double.SinCos(ad);

                y = double.FusedMultiplyAdd(vec, sin, y);
                x = double.FusedMultiplyAdd(vec, cos, x);
                t = nt;
                v = nv;
                d = nd;

                if (frame.SurveyType is SurveyType.Primary or SurveyType.Secondary or
                    SurveyType.Unknown7 or SurveyType.Unknown8)
                {
                    double dy = y - ny;
                    if (double.Abs(dy) > lim)
                        y = ny + double.CopySign(lim, dy);

                    double dx = x - nx;
                    if (double.Abs(dx) > lim)
                        x = nx + double.CopySign(lim, dx);
                }
                coordinates.Add(new(x, y, d, frame.GNSSAltitude, 0));
            }
        }

        #region Enumerator support
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private IEnumerator<IFrame> GetSL3Enumerator() =>
            frames is null || frames.Count == 0 ? new SL3Reader.Enumerator(this) : frames.GetEnumerator();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        IEnumerator<IFrame> IEnumerable<IFrame>.GetEnumerator() => GetSL3Enumerator();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        IEnumerator IEnumerable.GetEnumerator() => GetSL3Enumerator();

        public readonly struct Enumerator : IEnumerator<IFrame>, IEnumerator
        {
            private readonly SL3Reader source;
            private readonly unsafe Frame* pCurrent;
            private readonly long fileLength;
            readonly unsafe IFrame IEnumerator<IFrame>.Current => *pCurrent;
            readonly unsafe object IEnumerator.Current => *pCurrent;

            public unsafe Enumerator(SL3Reader source)
            {
                bool lockTaken = false;

                Monitor.Enter(source, ref lockTaken);
                if (!lockTaken) throw new IOException("Unable to lock stream for single access use.");

                this.source = source;
                fileLength = source.Length;

                pCurrent = (Frame*)AlignedAlloc(Frame.ExtendedSize, 8 * (nuint)nuint.Size);

                // The state of the source is unknown, so we have to reset the stream.
                Reset();
                InitTimestamp();
                Reset();
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private unsafe void InitTimestamp()
            {
                source.ReadExactly(new(pCurrent, Frame.MinimumInitSize)); // We won't have to read the whole
                // frame: just till the end of pCurrent->HardwareTime.
                Frame.InitTimestampBase(pCurrent->HardwareTime);
            }

            unsafe bool IEnumerator.MoveNext()
            {
                Frame* currentFrame = pCurrent;
                SL3Reader stream = source;

                // We read always the extended size!
                return stream.Read(new(currentFrame, Frame.ExtendedSize)) == Frame.ExtendedSize && // If false: unable to read. It could be due to EOF or IO error.
                       stream.Seek(currentFrame->LengthOfFrame - Frame.ExtendedSize, SeekOrigin.Current) < fileLength; // If false: Avoid returning non-complete frame.
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Reset()
            {
                if (source.Seek(SLFileHeader.Size, SeekOrigin.Begin) != SLFileHeader.Size)
                    throw new IOException("Unable to seek.");
            }
            unsafe void IDisposable.Dispose()
            {
                Monitor.Exit(source);
                AlignedFree(pCurrent);
            }
        }
        #endregion End: enumerator support
    }
}