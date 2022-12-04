using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Drawing;
using System.IO;
using System.Runtime.CompilerServices;
using System.Threading;
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
            List<IFrame> list = CreateNewFrameList();
            InitIndexSupport(list.Capacity);
            var index = indexByType;
            foreach (IFrame frame in collection)
            {
                list.Add(frame);
                index[frame.SurveyType].Add(frame);
            }
            return list;
        }

        public int Count => Frames.Count; // Can't be readonly hence the Frames could be initialized.

        public IFrame this[int index] => Frames[index]; // Can't be readonly hence the Frames could be initialized.

        #endregion Frame support

        #region Indices
        private SortedDictionary<SurveyType, List<IFrame>> indexByType;
        private SortedDictionary<uint, List<IFrame>> indexByCampaign;
        public SortedDictionary<SurveyType, List<IFrame>> IndexByType => indexByType;
        public SortedDictionary<uint, List<IFrame>> IndexbyCamaign => indexByCampaign;
        private void InitIndexSupport(int estimatedCount)
        {
            indexByType = new()
            {
                { SurveyType.Primary, new(estimatedCount / 8) },
                { SurveyType.Secondary, new(estimatedCount / 8) },
                {SurveyType.DownScan, new(estimatedCount / 10) },
                {SurveyType.LeftSidescan,new() },
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
                List<IFrame> localFrames = CreateNewFrameList();
                frames = localFrames;
                InitIndexSupport(frames.Capacity);
                SortedDictionary<SurveyType, List<IFrame>> typeIndex = indexByType;
                SortedDictionary<uint, List<IFrame>> camapignIndex = indexByCampaign;

                foreach (IFrame frame in this)
                {
                    streamWriter.WriteLine(frame.ToString()); // Write & WriteLine call the same
                    //  "private unsafe void WriteSpan(ReadOnlySpan<char> buffer, bool appendNewLine)"
                    // so ading the '\n' manually has no effect just causing platform dependent issues. 
                    localFrames.Add(frame);
                    typeIndex[frame.SurveyType].Add(frame);
                    if(camapignIndex.TryGetValue(frame.CampaignID, out List<IFrame>  camapignList))
                        camapignList.Add(frame);
                    else
                        camapignIndex.Add(frame.CampaignID, new List<IFrame>(9) { frame });
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

        public void ExportSideScans(string directory)
        {
            DirectoryInfo dir = new(directory);
            dir.Create();

            const int width = 2800;
            List<IFrame> sideScanFrames = IndexByType[SurveyType.SideScan];
            int frameCount = sideScanFrames.Count;
            if (frameCount < 1) return;
            int maxHeight = 1, currentHeiht = 0;
            int i = 0;
            List<int> breakpoints = new(); // TODO: add capacity
            float maxRange = sideScanFrames[0].MaxRange;

            for (; i < frameCount; i++)
            {
                float currentMaxRange = sideScanFrames[i].MaxRange;
                currentHeiht++; // Should be here due to the zero indexing.
                if (currentMaxRange != maxRange)
                {
                    maxRange = currentMaxRange;
                    breakpoints.Add(i);
                    if (currentHeiht > maxHeight)
                        maxHeight = currentHeiht;
                    currentHeiht = 0;
                    continue;
                }

            }
            if (currentHeiht > maxHeight)
                maxHeight = currentHeiht;

            byte[] buffer = BitmapHelper.CreateBuffer(maxHeight, width);

            i = 0;
            foreach (int breakpoint in breakpoints)
            {
                BitmapHelper.UpdateBuffer(buffer, breakpoint - i, width,
                out int fullStride,
                out Span<byte> fullBuffer,
                out Span<byte> dataBuffer);

                int k = 0;
                for (; i < breakpoint; i++)
                {
                    long dataOffset = sideScanFrames[i].DataOffset;
                    if (Seek(dataOffset, SeekOrigin.Begin) != dataOffset)
                    {
                        throw new IOException("Unable to seek.");
                    }

                    ReadExactly(dataBuffer.Slice(k++ * fullStride, width));
                }
                using FileStream stream = File.OpenWrite(Path.Combine(dir.FullName, $"SS_{breakpoint}.bmp"));
                stream.Write(fullBuffer);
                stream.Close();
            }
        }

        //public void ExportInterfereometricDataset(string directory)
        //{
        //    FrameList localFrames = Unknown8ScanFrames;
        //    int length = localFrames.Count;

        //    for (int i = 0; i < length - 1; i++)
        //    {
        //        BasicFrame FirstFrame = (BasicFrame)localFrames[i];
        //        Seek(FirstFrame.DataOffset, SeekOrigin.Begin);
        //        Span<ushort> First = (new ushort[512/2]).AsSpan();
        //        ReadExactly(System.Runtime.InteropServices.MemoryMarshal.AsBytes(First));


        //        BasicFrame SecondFrame = (BasicFrame)localFrames[i + 1];
        //        Seek(SecondFrame.DataOffset, SeekOrigin.Begin);
        //        Span<float> Second = (new float[512/4]).AsSpan();
        //        ReadExactly(System.Runtime.InteropServices.MemoryMarshal.AsBytes(Second));

        //        for (int j = 0; j < 512/4; j += 1)
        //        {
        //            System.Numerics.Complex complex = new(First[j], Second[j]);
        //            Debug.Print((complex.Phase * (360 / double.Tau)).ToString());
        //        }

        //    }
        //}

        public unsafe void Export3D(string path)
        {
            List<IFrame> localFrames = IndexByType[SurveyType.ThreeDimensional];
            int length = localFrames.Count;

            ThreeDimensionalFrameHeader* header = stackalloc ThreeDimensionalFrameHeader[1];
            InterferometricMeasuement* measurements = stackalloc InterferometricMeasuement[400];

            for (int i = 0; i < length; i++)
            {
                IFrame _3DFrame = (Frame)localFrames[i];
                Seek(_3DFrame.DataOffset, SeekOrigin.Begin);
                ReadExactly(new(header, ThreeDimensionalFrameHeader.Size));
                ReadExactly(new(measurements, header->NumberOfLeftBytes));
                var limit = measurements + (header->NumberOfLeftBytes / InterferometricMeasuement.Size);
                for (InterferometricMeasuement* measurement = measurements; measurement < limit; measurement++)
                {
                    Debug.Print(measurement->ToString());
                }
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