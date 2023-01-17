using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = Size, Pack = 2), // Pack is important!
     SkipLocalsInit] 
    internal struct BitmapFileHeader
    {
        internal const int Size = 14;

        private ushort Type;
        private uint FileSize;
        private ushort Reserved1;
        private ushort Reserved2;
        private uint OffsetOfImageData;

        internal BitmapFileHeader(int width, int height) => 
            Update(width, height);

        internal uint Update(int width, int height)
        {
            Type = 19778; // This is "BM" in ASCII.
            FileSize = (uint)(BitmapHelper.BitmapCombinedHeaderSize + height * width); // (width + (4 - width % 4))
            Reserved1 = 0; Reserved2 = 0;
            OffsetOfImageData = BitmapHelper.BitmapCombinedHeaderSize;
            return FileSize;
        }
    }
}