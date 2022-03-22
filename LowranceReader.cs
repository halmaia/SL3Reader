using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.CompilerServices;
using System.Threading;

namespace SL3Reader
{
    public class LowranceReader :
        FileStream, IEnumerable<Frame>, IEnumerable
    {
        public unsafe LowranceReader(string path) :
           base(path, FileMode.Open, FileAccess.Read,
           FileShare.Read, 4096, FileOptions.SequentialScan)
        {
            SLFileHeader* fileHeader = stackalloc SLFileHeader[1];

            if (Read(new(fileHeader, SLFileHeader.Size)) != SLFileHeader.Size)
                throw new EndOfStreamException("The file is too small!");

            if (fileHeader->FileFormat != LogFileFormats.SL3 || fileHeader->BlockSize != 3200)
                throw new InvalidDataException("File type not supported!");
        }

        public void ExportToCSV(string path)
        {
            using StreamWriter stream = File.CreateText(path);
            stream.Write(Frame.CSVHeader);
            foreach (Frame frame in this)
            {
                stream.Write(frame.ToString());
            }
        }

        IEnumerator<Frame> IEnumerable<Frame>.GetEnumerator() =>
            new Enumerator(this);

        IEnumerator IEnumerable.GetEnumerator() =>
            new Enumerator(this);

        public readonly struct Enumerator : IEnumerator<Frame>, IEnumerator
        {
            private readonly LowranceReader source;
            private readonly Frame[] aCurrent;
            private unsafe readonly Frame* pCurrent;

            public unsafe Enumerator(LowranceReader source)
            {
                bool lockTaken = false;
                Monitor.Enter(source, ref lockTaken);
                aCurrent = GC.AllocateArray<Frame>(1, true);
                pCurrent = (Frame*)Unsafe.AsPointer(ref aCurrent[0]);
                this.source = source;
                ((IEnumerator)this).Reset();
            }

            unsafe Frame IEnumerator<Frame>.Current =>
                *pCurrent;
            unsafe object IEnumerator.Current =>
                *pCurrent;

            void IDisposable.Dispose() =>
                Monitor.Exit(source);

            unsafe bool IEnumerator.MoveNext()
            {
                LowranceReader src = source;
                if (src.Read(new(pCurrent, Frame.Size)) != Frame.Size)
                    return false;
                _ = src.Seek(pCurrent->TotalLength - Frame.Size, SeekOrigin.Current);
                return true;
            }

            void IEnumerator.Reset()
            {
                if (source.Seek(SLFileHeader.Size, SeekOrigin.Begin) != SLFileHeader.Size)
                    throw new IOException("Unable to seek.");
            }
        }
    }
}