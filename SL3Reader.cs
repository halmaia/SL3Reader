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
                if (frames is null)
                {
                    frames = CreateNewFrameList();
                    frames.AddRange(this);
                }
                return frames;
            }
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
                IEnumerable<Frame> collection = (IEnumerable<Frame>)frames ?? this; // To avoid repeated runs.
                foreach (Frame frame in collection)
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
            new Enumerator(this);

        IEnumerator IEnumerable.GetEnumerator() =>
            new Enumerator(this);
        public readonly struct Enumerator : IEnumerator<Frame>, IEnumerator
        {
            private readonly SL3Reader source;
            private unsafe readonly Frame* pCurrent;
            private readonly long fileLength;
            unsafe Frame IEnumerator<Frame>.Current => *pCurrent;
            unsafe object IEnumerator.Current => *pCurrent;

            public unsafe Enumerator(SL3Reader source)
            {
                bool lockTaken = false;

                Monitor.Enter(source, ref lockTaken);
                if (!lockTaken) throw new IOException("Unable to lock stream for single access use.");

                this.source = source;
                fileLength = source.Length;

                pCurrent = (Frame*)AlignedAlloc(new(Frame.Size), new(sizeof(long)));

                // The state of the source is unknown, so we have to reset the stream.
                ((IEnumerator)this).Reset();

                InitTimestamp();

                ((IEnumerator)this).Reset();
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
            void IEnumerator.Reset()
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