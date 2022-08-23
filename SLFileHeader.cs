using System.Diagnostics;
using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = Size)]
    [DebuggerDisplay($"{{{nameof(ToString)}(),nq}}")]
    internal readonly ref struct SLFileHeader
    {
        internal const int Size = 8;

        private readonly LogFileFormats fileFormat;
        private readonly short deviceID;
        private readonly short blockSize;
        private readonly short reserved;

        internal LogFileFormats FileFormat => fileFormat;
        internal short DeviceID => deviceID;

        internal short BlockSize => blockSize;

        internal short Reserved => reserved;

        public readonly override string ToString() => fileFormat.ToString() + " (" + blockSize.ToString() + ')';

        public readonly bool IsValidSLFile => fileFormat == LogFileFormats.SLG;
        public readonly bool IsValidSL2File => fileFormat == LogFileFormats.SL2;
        public readonly bool IsValidSL3File => fileFormat == LogFileFormats.SL3;
    }
}