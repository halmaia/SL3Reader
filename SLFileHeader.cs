using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = Size)]
    [DebuggerDisplay($"{{{nameof(ToString)}(),nq}}")]
    public readonly ref struct SLFileHeader
    {
        public const int Size = 8;

        private readonly LogFileFormat fileFormat;
        private readonly short deviceID;
        private readonly short blockSize;
        private readonly short reserved;

        public readonly LogFileFormat FileFormat => fileFormat;
        public readonly short DeviceID => deviceID;

        public readonly short BlockSize => blockSize;

        public readonly short Reserved => reserved;

        public readonly override string ToString() => fileFormat.ToString() + " (" + blockSize.ToString() + ')';

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly bool IsValidFormat(LogFileFormat fileFormatToTest) => 
            fileFormat == fileFormatToTest;
    }
}