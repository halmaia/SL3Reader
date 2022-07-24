using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = Size, Pack = 2)]
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
            const int Offset = BitmapFileHeader.Size + BitmapInfoHeader.Size + 256 * Bgra.Size; // 1078; 
            OffsetOfImageData = Offset; 
            Reserved1 = 0; Reserved2 = 0; Type = 19778;
            FileSize = (uint)(Offset + (height * (width + (4 - width % 4))));
        }
    }

}