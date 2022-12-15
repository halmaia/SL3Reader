using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Drawing;
using System.IO;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Threading.Channels;
using static System.Runtime.InteropServices.NativeMemory;

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
            SortedDictionary<SurveyType, List<IFrame>> typeIndex = indexByType;
            SortedDictionary<uint, List<IFrame>> camapignIndex = indexByCampaign;

            foreach (IFrame frame in collection)
            {
                localFrames.Add(frame);
                typeIndex[frame.SurveyType].Add(frame);
                if (camapignIndex.TryGetValue(frame.CampaignID, out List<IFrame> camapignList))
                    camapignList.Add(frame);
                else
                    camapignIndex.Add(frame.CampaignID, new List<IFrame>(9) { frame });
            }
            return localFrames;
        }

        public int Count => Frames.Count; // Can't be readonly hence the Frames could be initialized.

        public IFrame this[int index] => Frames[index]; // Can't be readonly hence the Frames could be initialized.

        #endregion Frame support

        #region Indices
        private SortedDictionary<SurveyType, List<IFrame>> indexByType;
        private SortedDictionary<uint, List<IFrame>> indexByCampaign;
        public SortedDictionary<SurveyType, List<IFrame>> IndexByType => indexByType;
        public SortedDictionary<uint, List<IFrame>> IndexByCampaign => indexByCampaign;
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

            SLFileHeader* pFileHeader = stackalloc SLFileHeader[1];
            ReadExactly(new(pFileHeader, SLFileHeader.Size));

            if (!pFileHeader->IsValidFormat(LogFileFormat.SL3))
                throw new InvalidDataException("Unsupported file type. Expected type SL3.");
        }

        public unsafe void ExportToCSV(string path, bool reuseFrames = false)
        {
            const string CSVHeader = "SurveyType,WaterDepth,Longitude,Lattitude,GNSSAltitude,GNSSHeading,GNSSSpeed,MagneticHeading,MinRange,MaxRange,WaterTemperature,WaterSpeed,HardwareTime,Frequency,Milliseconds";

            using StreamWriter streamWriter = File.CreateText(path);
            streamWriter.WriteLine(CSVHeader);

            if (reuseFrames && frames is null)
            {
                foreach (IFrame frame in Frames)
                {
                    streamWriter.WriteLine(frame.ToString()); // Write & WriteLine call the same
                    //  "private unsafe void WriteSpan(ReadOnlySpan<char> buffer, bool appendNewLine)"
                    // so ading the '\n' manually has no effect just causing platform dependent issues. 
                }
            }
            else
            {
                foreach (IFrame frame in this)
                {
                    // Write & WriteLine call the same
                    //  "private unsafe void WriteSpan(ReadOnlySpan<char> buffer, bool appendNewLine)"
                    // so ading the '\n' manually has no effect just causing platform dependent issues. 
                    streamWriter.WriteLine(frame.ToString());
                }
            }
            streamWriter.Close();
        }

        private static List<int> GetBreakPoints(List<IFrame> frames, out int contigousLength)
        {
            int i = 0;
            int frameCount = frames.Count;
            var firstFrame = frames[0];
            List<int> breakpoints = new(512) { 0 };
            float previousRange = firstFrame.MaxRange;

            for (; i < frameCount; i++)
            {
                float currentRange = frames[i].MaxRange;
                if (currentRange != previousRange)
                {
                    previousRange = currentRange;
                    breakpoints.Add(i);
                }
            }

            if (breakpoints[^1] != (i -= 1)) breakpoints.Add(i);

            contigousLength = 0;

            for (i = 0; i < breakpoints.Count - 1; i += 2)
            {
                int delta;
                if (contigousLength < (delta = breakpoints[1 + i] - breakpoints[i]))
                    contigousLength = delta;
            }

            return breakpoints;
        }

        public void ExportSideScans(string path)
        {
            const int width = 2800; // Valid only for SL3 3200 files.

            ArgumentNullException.ThrowIfNull(nameof(path));

            if (Directory.Exists(path = Path.GetFullPath(path)))
                Directory.Delete(path, true);
            Directory.CreateDirectory(path);

            int frameCount = Frames.Count; // Initialize the frames.
            if (frameCount < 1) return;

            List<IFrame> sideScanFrames = IndexByType[SurveyType.SideScan];
            if (sideScanFrames.Count < 1) return; // Return when no sidescan exists

            List<int> breakpoints = GetBreakPoints(sideScanFrames, out int maxHeight);

            byte[] buffer = BitmapHelper.CreateBuffer(maxHeight, width);

            for (int i = 0; i < breakpoints.Count - 1; i += 2)
            {
                int final = breakpoints[1 + i], first = breakpoints[i];
                BitmapHelper.UpdateBuffer(buffer, final - first, width,
                out int fullStride,
                out Span<byte> fullBuffer,
                out Span<byte> dataBuffer);

                int k = 0;
                for (int j = first; j < final; j++)
                {
                    long dataOffset = sideScanFrames[j].DataOffset;
                    if (Seek(dataOffset, SeekOrigin.Begin) != dataOffset)
                    {
                        throw new IOException("Unable to seek.");
                    }

                    ReadExactly(dataBuffer.Slice(k++ * fullStride, width));
                }

                using FileStream stream = File.OpenWrite(Path.Combine(path, $"SS_{final}.bmp"));
                stream.Write(fullBuffer);
                stream.Close();
            }
        }

        public unsafe void ExamineUnknown8Datasets()
        {
            int frameCount = Frames.Count; // Initialize the frames.
            if (frameCount < 1) return;

            List<IFrame> unknown8Frames = IndexByType[SurveyType.Unknown8];
            int unknown8FrameCount = unknown8Frames.Count;
            if (frameCount < 1) return; // Return when no sidescan exists

            var buffer = new byte[512];
            fixed(byte* p =&buffer[0]) {
                for (int i = 0; i < unknown8FrameCount - 1; i++)
                {
                    IFrame frame = unknown8Frames[i];
                    Seek(frame.DataOffset, SeekOrigin.Begin);
                    Read(new(p,512));
                    for (int j = 0; j < 256; j += 2)
                    {
                        Debug.Print($"{buffer[j]}, {buffer[1 + j]}");
                    }
                }

            }
        }

        public unsafe void Export3D(string path)
        {
            ArgumentNullException.ThrowIfNull(nameof(path));

            path = Path.GetFullPath(path);

            int frameCount = Frames.Count; // Initialize the frames.
            if (frameCount < 1) return;

            using StreamWriter streamWriter = File.CreateText(path);

            List<IFrame> frames3D = IndexByType[SurveyType.ThreeDimensional];
            int frames3DLength = frames3D.Count;
            if (frames3DLength < 1) return;

            ThreeDimensionalFrameHeader* header = stackalloc ThreeDimensionalFrameHeader[1];
            byte* measurements = stackalloc byte[400 * InterferometricMeasuement.Size],
                  reset = measurements;

            for (int i = 0; i < frames3DLength; i++)
            {
                IFrame frame = frames3D[i];
                var offset = frame.DataOffset;
                if (Seek(offset, SeekOrigin.Begin) != offset)
                    throw new IOException("Unable to seek!");

                ReadExactly(new(header, ThreeDimensionalFrameHeader.Size));

                ReadExactly(new(measurements, header->NumberOfLeftBytes + 
                                              header->NumberOfRightBytes + 
                                              header->NumberOfUnreliableBytes));

                byte* limit = measurements + header->NumberOfLeftBytes;
                for (; measurements < limit; measurements += InterferometricMeasuement.Size)
                {
                    InterferometricMeasuement* measurement = (InterferometricMeasuement*)measurements;
                    streamWriter.WriteLine($"{-measurement->Delta},{i},{measurement->Depth},R");
                }

                limit = measurements + header->NumberOfRightBytes;
                for (; measurements < limit; measurements += InterferometricMeasuement.Size)
                {
                    InterferometricMeasuement* measurement = (InterferometricMeasuement*)measurements;
                    streamWriter.WriteLine($"{measurement->Delta},{i},{measurement->Depth},R");
                }

                limit = measurements + header->NumberOfUnreliableLeftBytes;
                for (; measurements < limit; measurements += InterferometricMeasuement.Size)
                {
                    InterferometricMeasuement* measurement = (InterferometricMeasuement*)measurements;
                    float delta = measurement->Delta;
                    if (delta is < 0.001f or > 5000.0f) continue;
                    float depth = measurement->Depth;
                    if(depth is < 1f or > 250.0f) continue;

                    streamWriter.WriteLine($"{-delta},{i},{depth},U");
                }

                limit = measurements + header->NumberOfUnreliableRightBytes;
                for (; measurements < limit; measurements += InterferometricMeasuement.Size)
                {
                    InterferometricMeasuement* measurement = (InterferometricMeasuement*)measurements;
                    float delta = measurement->Delta;
                    if (delta is < 0.001f or > 5000.0f) continue;
                    float depth = measurement->Depth;
                    if (depth is < 1f or > 250.0f) continue;

                    streamWriter.WriteLine($"{delta},{i},{depth},U");
                }

                measurements = reset;
            }
            streamWriter.Close();
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