using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = 8)]
    internal readonly struct SLFileHeader
    {
        internal const int Size = 8;
        internal readonly LogFileFormats FileFormat;
        internal readonly short DeviceID, BlockSize, Reserved;
    }
}