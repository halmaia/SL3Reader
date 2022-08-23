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
        FileStream, IEnumerable<Frame>, IEnumerable, IReadOnlyList<Frame>, IReadOnlyCollection<Frame>
    {
        private List<Frame> frames;
        public IReadOnlyList<Frame> Frames
        {
            get
            {
                frames ??= CreateNewFrameList(this);
                return frames;
            }
        }

        private FrameList sideScanFrames, downScanFrames;
        public FrameList SideScanFrames
        {
            get
            {
                if (sideScanFrames is null)
                {
                    PopulateImageIndices();
                }
                return sideScanFrames;
            }
        }

        private void PopulateImageIndices()
        {
            var localFrames = Frames;
            FrameList sideScans = new(localFrames);
            FrameList downScans = new(localFrames);

            for (int i = 0; i < localFrames.Count; i++)
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
                }
            }
            sideScanFrames = sideScans; downScanFrames = downScans;
        }

        public int Count => Frames.Count; // Can't be readonly hence the Frames could be initialized.

        public Frame this[int index] => Frames[index]; // Can't be readonly hence the Frames could be initialized.

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
        private List<Frame> CreateNewFrameList()
        {
            const long averageFrameSize = 2118L; // Empirically set value to avoid frequent resize of the underlying array.
            return new((int)(Length / averageFrameSize));
        }

        private List<Frame> CreateNewFrameList(IEnumerable<Frame> enumerable)
        {
            List<Frame> list = CreateNewFrameList();
            list.AddRange(enumerable);
            return list;
        }

        public void ExportToCSV(string path, bool reuseFrames = false)
        {
            const string CSVHeader = "SurveyType,WaterDepth,X,Y,GNSSAltitude,GNSSHeading,GNSSSpeed,MagneticHeading,MinRange,MaxRange,WaterTemperature,WaterSpeed,HardwareTime,Frequency,Milliseconds";

            using StreamWriter streamWriter = File.CreateText(path);
            streamWriter.WriteLine(CSVHeader);

            if (reuseFrames && frames is null)
            {
                List<Frame> localFrames = CreateNewFrameList();
                frames = localFrames;

                foreach (Frame frame in this)
                {
                    streamWriter.WriteLine(frame.ToString()); // Write & WriteLine calling the same
                    //  "private unsafe void WriteSpan(ReadOnlySpan<char> buffer, bool appendNewLine)"
                    // so ading the '\n' manually has no effect just causing platform dependent issues. 
                    localFrames.Add(frame);
                }
            }
            else
            {
                foreach (Frame frame in this)
                {
                    // Write & WriteLine calling the same
                    //  "private unsafe void WriteSpan(ReadOnlySpan<char> buffer, bool appendNewLine)"
                    // so ading the '\n' manually has no effect just causing platform dependent issues. 
                    streamWriter.WriteLine(frame.ToString());
                }
            }
            streamWriter.Close();
        }

        public void ExportToBmp(IReadOnlyList<uint> frameIndices, string path)
        {
            throw new NotImplementedException();
        }

        #region Enumerator support
        IEnumerator<Frame> IEnumerable<Frame>.GetEnumerator() =>
            frames is null ? new Enumerator(this) : frames.GetEnumerator();

        IEnumerator IEnumerable.GetEnumerator() =>
            frames is null ? new Enumerator(this) : frames.GetEnumerator();
        public readonly struct Enumerator : IEnumerator<Frame>, IEnumerator
        {
            private readonly SL3Reader source;
            private unsafe readonly Frame* pCurrent;
            private readonly long fileLength;
            readonly unsafe Frame IEnumerator<Frame>.Current => *pCurrent;
            readonly unsafe object IEnumerator.Current => *pCurrent;

            public unsafe Enumerator(SL3Reader source)
            {
                bool lockTaken = false;

                Monitor.Enter(source, ref lockTaken);
                if (!lockTaken) throw new IOException("Unable to lock stream for single access use.");

                this.source = source;
                fileLength = source.Length;

                pCurrent = (Frame*)AlignedAlloc(Frame.Size, (nuint)nuint.Size);

                // The state of the source is unknown, so we have to reset the stream.
                Reset();
                InitTimestamp();
                Reset();
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private unsafe void InitTimestamp()
            {
                if (source.Read(new(pCurrent, Frame.Size)) != Frame.Size)
                    throw new IOException("Unable to read. It could be due to EOF or IO error.");
                // No need to read the whole frame: will be checked at IEnumerator.MoveNext().

                Frame.InitTimestampBase(pCurrent->HardwareTime);
            }

            unsafe bool IEnumerator.MoveNext()
            {
                SL3Reader stream = source;
                Frame* currentFrame = pCurrent;

                return stream.Read(new(currentFrame, Frame.Size)) == Frame.Size && // If false: unable to read. It could be due to EOF or IO error.
                       stream.Seek(currentFrame->TotalLength - Frame.Size, SeekOrigin.Current) < fileLength; // If false: Avoid returning non-complete frame.
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