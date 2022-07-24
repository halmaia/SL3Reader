using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading;
using static System.Runtime.InteropServices.NativeMemory;

namespace SL3Reader
{
    [DebuggerDisplay($"{{{nameof(GetDebuggerDisplay)}(),nq}}")]
    public class SL3Reader :
        FileStream, IEnumerable<Frame>, IEnumerable, IReadOnlyList<Frame>, IReadOnlyCollection<Frame>
    {

        private List<Frame> frames = null;
        public IReadOnlyList<Frame> Frames
        {
            get
            {
                if (frames is null)
                {
                    const int factor = 2118; // Empirically set value to avoid frequent resize of the underlying array.
                    frames = new((int)(Length / factor));
                    frames.AddRange(this);
                    if (frames.Count > 0)
                        Frame.Init(frames[0].HardwareTime);
                }
                return frames;
            }
        }

        public int Count => Frames.Count;

        public Frame this[int index] => Frames[index];

        public unsafe SL3Reader(string path) :
           base(path, FileMode.Open, FileAccess.Read,
           FileShare.Read, 4096, FileOptions.RandomAccess)
        {
            SLFileHeader* pFileHeader = stackalloc SLFileHeader[1];
            ReadExactly(new(pFileHeader, SLFileHeader.Size));

            if (!pFileHeader->IsValidSL3File)
                throw new InvalidDataException("Unsupported file type. Expected type SL3.");
        }


        public void ExportToCSV(string path, bool reuseFrames = false)
        {
            const string CSVHeader = "SurveyType,WaterDepth,X,Y,GNSSAltitude,GNSSHeading,GNSSSpeed,MagneticHeading,MinRange,MaxRange,WaterTemperature,WaterSpeed,HardwareTime,Frequency,Milliseconds\n";

            using StreamWriter streamWriter = File.CreateText(path);
            streamWriter.Write(CSVHeader); // Should be updated to C♯ 11 to write Span directly.
     
            if (reuseFrames && frames is null)
            {
                const int factor = 2118; // Empirically set value to avoid frequent resize of the underlying array.
                List<Frame> localFrames = new((int)(Length / factor));
                foreach (Frame frame in this)
                {
                    streamWriter.Write(frame.ToString());
                    localFrames.Add(frame);
                }
                frames = localFrames;
                if(localFrames.Count > 0)
                    Frame.Init(localFrames[0].HardwareTime);
            }
            else
            {
                IEnumerable<Frame> collection = (IEnumerable<Frame>)frames ?? this; // To avoid double runs.
                foreach (Frame frame in collection)
                {
                    streamWriter.Write(frame.ToString());
                }
            }
            streamWriter.Close();
        }

        public void ExportToBmp(IReadOnlyList<uint> frameIndices, string path)
        {
            throw new NotImplementedException();
        }

        private string GetDebuggerDisplay() => Name;

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

                ((IEnumerator)this).Reset();
            }

            unsafe bool IEnumerator.MoveNext()
            {
                Stream stream = source;

                if (stream.Read(new(pCurrent, Frame.Size)) != Frame.Size)
                    return false; // Unable to read. It could be due to EOF or IO error.

                return stream.Seek(pCurrent->TotalLength - Frame.Size, SeekOrigin.Current) < fileLength;
                // Avoid returning non-complete frame.
            }

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