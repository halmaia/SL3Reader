using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
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

        private List<IFrame> CreateNewFrameList(IEnumerable<IFrame> enumerable)
        {
            List<IFrame> list = CreateNewFrameList();
            list.AddRange(enumerable);
            return list;
        }
        #endregion Frame support

        #region Indices
        private FrameList sideScanFrames,
            downScanFrames,
            primaryScanFrames,
            secondaryScanFrames,
            unknown7ScanFrames,
            unknown8ScanFrames,
            threeDimensionalFrames;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void EnsureIndexAvaliability()
        {
            if (sideScanFrames is null)
                PopulateIndices();
        }
        private FrameList SideScanFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureIndexAvaliability();
                return sideScanFrames;
            }
        }

        private FrameList DownScanFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureIndexAvaliability();
                return downScanFrames;
            }
        }

        private FrameList PrimaryScanFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureIndexAvaliability();
                return primaryScanFrames;
            }
        }

        private FrameList SecondaryScanFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureIndexAvaliability();
                return secondaryScanFrames;
            }
        }

        private FrameList Unknown7ScanFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureIndexAvaliability();
                return unknown7ScanFrames;
            }
        }

        private FrameList Unknown8ScanFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureIndexAvaliability();
                return unknown8ScanFrames;
            }
        }

        private FrameList ThreeDimensionalFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureIndexAvaliability();
                return threeDimensionalFrames;
            }
        }

        private void PopulateIndices()
        {
            IReadOnlyList<IFrame> localFrames = Frames;
            int localCount = localFrames.Count;
            FrameList sideScans = new(localCount / 10, localFrames),
                      downScans = new(localCount / 10, localFrames),
                      primaryScans = new(localCount / 8, localFrames),
                      secondaryScans = new(localCount / 8, localFrames),
                      unknown7Scans = new(localCount / 4, localFrames),
                      unknown8Scans = new(localCount / 4, localFrames),
                      threeDimensionals = new(localCount / 10, localFrames);


            for (int i = 0; i < localCount; i++)
            {
                switch (localFrames[i].SurveyType)
                {
                    case SurveyType.SideScan:
                        {
                            sideScans.Add(i);
                            continue;
                        }
                    case SurveyType.DownScan:
                        {
                            downScans.Add(i);
                            continue;
                        }
                    case SurveyType.Primary:
                        { primaryScans.Add(i); continue; }
                    case SurveyType.Secondary:
                        { secondaryScans.Add(i); continue; }
                    case SurveyType.Unknown7:
                        { unknown7Scans.Add(i); continue; }
                    case SurveyType.Unknown8:
                        { unknown8Scans.Add(i); continue; }
                    case SurveyType.ThreeDimensional:
                        { threeDimensionals.Add(i); continue; }
                }
            }
            sideScanFrames = sideScans;
            downScanFrames = downScans;
            primaryScanFrames = primaryScans;
            secondaryScanFrames = secondaryScans;
            unknown7ScanFrames = unknown7Scans;
            unknown8ScanFrames = unknown8Scans;
            threeDimensionalFrames = threeDimensionals;
        }
        #endregion Indices

        public int Count => Frames.Count; // Can't be readonly hence the Frames could be initialized.

        public IFrame this[int index] => Frames[index]; // Can't be readonly hence the Frames could be initialized.

        public unsafe SL3Reader(string path) :
           base(path, FileMode.Open, FileAccess.Read,
           FileShare.Read, 4096, FileOptions.RandomAccess)
        {
            SLFileHeader* pFileHeader = stackalloc SLFileHeader[1];
            ReadExactly(new(pFileHeader, SLFileHeader.Size));

            if (!pFileHeader->IsValidFormat(LogFileFormat.SL3))
                throw new InvalidDataException("Unsupported file type. Expected type SL3.");
        }

        public unsafe void ExportToCSV(string path, bool reuseFrames = false)
        {
            const string CSVHeader = "SurveyType,WaterDepth,X,Y,GNSSAltitude,GNSSHeading,GNSSSpeed,MagneticHeading,MinRange,MaxRange,WaterTemperature,WaterSpeed,HardwareTime,Frequency,Milliseconds";

            using StreamWriter streamWriter = File.CreateText(path);
            streamWriter.WriteLine(CSVHeader);

            if (reuseFrames && frames is null)
            {
                List<IFrame> localFrames = CreateNewFrameList();
                frames = localFrames;

                foreach (IFrame frame in this)
                {
                    streamWriter.WriteLine(frame.ToString()); // Write & WriteLine calling the same
                    //  "private unsafe void WriteSpan(ReadOnlySpan<char> buffer, bool appendNewLine)"
                    // so ading the '\n' manually has no effect just causing platform dependent issues. 
                    localFrames.Add(frame);
                }
            }
            else
            {
                foreach (IFrame frame in this)
                {
                    // Write & WriteLine calling the same
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
            FrameList sideScanFrames = SideScanFrames;
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
            FrameList localFrames = ThreeDimensionalFrames;
            int length = localFrames.Count;

            ThreeDimensionalFrameHeader* header = stackalloc ThreeDimensionalFrameHeader[1];
            InterferometricMeasuement* measurements = stackalloc InterferometricMeasuement[400];

            for (int i = 0; i < length; i++)
            {
                IFrame _3DFrame = (ExtendedFrame)localFrames[i];
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
        IEnumerator<IFrame> IEnumerable<IFrame>.GetEnumerator() =>
            frames is not null ? frames.GetEnumerator() : new Enumerator(this);

        IEnumerator IEnumerable.GetEnumerator() =>
            frames is not null ? frames.GetEnumerator() : new Enumerator(this);
        public readonly struct Enumerator : IEnumerator<IFrame>, IEnumerator
        {
            private readonly SL3Reader source;
            private unsafe readonly ExtendedFrame* pCurrent;
            private readonly long fileLength;
            readonly unsafe IFrame IEnumerator<IFrame>.Current => GetCurrentFrame();
            readonly unsafe object IEnumerator.Current => GetCurrentFrame();

            private readonly unsafe IFrame GetCurrentFrame()
            {
                ExtendedFrame* frame = pCurrent;
                return frame->SurveyType is SurveyType.Unknown7 or SurveyType.Unknown8
                    ? *(BasicFrame*)pCurrent
                    : *frame;
            }
            public unsafe Enumerator(SL3Reader source)
            {
                bool lockTaken = false;

                Monitor.Enter(source, ref lockTaken);
                if (!lockTaken) throw new IOException("Unable to lock stream for single access use.");

                this.source = source;
                fileLength = source.Length;

                pCurrent = (ExtendedFrame*)AlignedAlloc(ExtendedFrame.Size, (nuint)nuint.Size);

                // The state of the source is unknown, so we have to reset the stream.
                Reset();
                InitTimestamp();
                Reset();
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private unsafe void InitTimestamp()
            {
                if (source.Read(new(pCurrent, ExtendedFrame.Size)) != ExtendedFrame.Size)
                    throw new IOException("Unable to read. It could be due to EOF or IO error.");
                // No need to read the whole frame: will be checked at IEnumerator.MoveNext().
                BasicFrame.InitTimestampBase(pCurrent->HardwareTime);
                ExtendedFrame.InitTimestampBase(pCurrent->HardwareTime);
            }

            unsafe bool IEnumerator.MoveNext()
            {
                SL3Reader stream = source;
                ExtendedFrame* currentFrame = pCurrent;

                return stream.Read(new(currentFrame, ExtendedFrame.Size)) == ExtendedFrame.Size && // If false: unable to read. It could be due to EOF or IO error.
                       stream.Seek(currentFrame->TotalLength - ExtendedFrame.Size, SeekOrigin.Current) < fileLength; // If false: Avoid returning non-complete frame.
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