using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = Size, Pack = 2)] // Pack is important!
    public readonly struct BitmapFileHeader
    {
        public const int Size = 14;

        public readonly ushort Type;
        public readonly uint FileSize;
        public readonly ushort Reserved1;
        public readonly ushort Reserved2;
        public readonly uint OffsetOfImageData;

        public BitmapFileHeader(int width, int height)
        {
            Type = 19778; // This is "BM" in ASCII.
            FileSize = (uint)(BitmapHelper.BitmapCombinedHeaderSize + height * width); // (width + (4 - width % 4))
            Reserved1 = 0; Reserved2 = 0;
            OffsetOfImageData = BitmapHelper.BitmapCombinedHeaderSize; 
        }
    }
}