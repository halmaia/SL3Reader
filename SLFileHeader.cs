using System.Diagnostics;
using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = Size)]
    [DebuggerDisplay($"{{{nameof(GetDebuggerDisplay)}(),nq}}")]
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

        public override string ToString() => fileFormat.ToString() + " (" + blockSize.ToString() + ')';

        private string GetDebuggerDisplay() => ToString();

        public bool IsValidSLFile => fileFormat == LogFileFormats.SLG;
        public bool IsValidSL2File => fileFormat == LogFileFormats.SL2;
        public bool IsValidSL3File => fileFormat == LogFileFormats.SL3;
    }
}