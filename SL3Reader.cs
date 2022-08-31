using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
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
        private List<IFrame> frames;
        public IReadOnlyList<IFrame> Frames
        {
            get
            {
                frames ??= CreateNewFrameList(this);
                return frames;
            }
        }

        #region Image Export
        private FrameList sideScanFrames,
            downScanFrames,
            primaryScanFrames,
            secondaryScanFrames,
            unknown7ScanFrames;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void EnsureScanIndicesAvaliability()
        {
            if (sideScanFrames is null)
                PopulateImageIndices();
        }
        private FrameList SideScanFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureScanIndicesAvaliability();
                return sideScanFrames;
            }
        }

        private FrameList DownScanFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureScanIndicesAvaliability();
                return downScanFrames;
            }
        }

        private FrameList PrimaryScanFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureScanIndicesAvaliability();
                return primaryScanFrames;
            }
        }

        private FrameList SecondaryScanFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureScanIndicesAvaliability();
                return secondaryScanFrames;
            }
        }

        private FrameList Unknown7ScanFrames
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                EnsureScanIndicesAvaliability();
                return unknown7ScanFrames;
            }
        }

        private void PopulateImageIndices()
        {
            IReadOnlyList<IFrame> localFrames = Frames;
            int localCount = localFrames.Count;
            FrameList sideScans = new(localFrames),
                      downScans = new(localFrames),
                      primaryScans = new(localFrames),
                      secondaryScans = new(localFrames),
                      unknown7Scans = new(localFrames);


            for (int i = 0; i < localCount; i++)
            {
                switch (localFrames[i].SurveyType)
                {
                    case SurveyType.SideScan:
                        {
                            sideScans.Add(i);
                            break;
                        }
                    case SurveyType.DownScan:
                        {
                            downScans.Add(i);
                            break;
                        }
                    case SurveyType.Primary:
                        { primaryScans.Add(i); break; }
                    case SurveyType.Secondary:
                        { secondaryScans.Add(i); break; }
                    case SurveyType.Unknown7:
                        { unknown7Scans.Add(i); break; }
                }
            }
            sideScanFrames = sideScans;
            downScanFrames = downScans;
            primaryScanFrames = primaryScans;
            secondaryScanFrames = secondaryScans;
            unknown7ScanFrames = unknown7Scans;
        }
        #endregion Image Export

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

        public void ExportSideScans(string path)
        {
            const int width = 2800;
            FrameList sideScanFrames = SideScanFrames;
            int frameCount = sideScanFrames.Count;

            // TODO: create slicer!
            byte[] buffer = BitmapHelper.CreateBuffer(frameCount, width);
            BitmapHelper.UpdateBuffer(buffer, frameCount, width,
                out long fileSize, out int fullStride,
                out Span<byte> fullBuffer,
                out Span<byte> dataBuffer);

            for (int i = 0; i < frameCount; i++)
            {
                long dataOffset = sideScanFrames[i].DataOffset;
                if (Seek(dataOffset, SeekOrigin.Begin) != dataOffset)
                {
                    throw new IOException("Unable to seek.");
                }

                ReadExactly(dataBuffer.Slice(i * fullStride, width));
            }

            using FileStream stream = File.OpenWrite(path);
            stream.Write(fullBuffer);
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