using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = Size), SkipLocalsInit]
    internal struct BitmapInfoHeader
    {
        internal const int Size = 40;

        private uint StructureSize;
        private int Width;
        private int Height;
        private ushort Planes;
        private ushort BitCount;
        private BitmapCompressionMode Compression;
        private uint ImageSize;
        private int XPixelsPerMeter;
        private int YPixelsPerMeter;
        private uint NumberOfUsedColors;
        private uint NumberOfImportantColors;

        internal BitmapInfoHeader(int width,
                                  int height,
                                  ushort bitCount = 8,
                                  int xPixelsPerMeter = 0,
                                  int yPixelsPerMeter = 0) =>
            Update(width, height, bitCount, xPixelsPerMeter, yPixelsPerMeter);

        internal void Update(int width,
                             int height,
                             ushort bitCount = 8,
                             int xPixelsPerMeter = 0,
                             int yPixelsPerMeter = 0)
        {
            const ushort ByteSize = 8;
            StructureSize = Size;
            Width = width;
            Height = height;
            Planes = 1;
            BitCount = bitCount;
            Compression = BitmapCompressionMode.BI_RGB;
            ImageSize = (uint)(bitCount / ByteSize * height * width); // (width + (4 - width % 4))
            XPixelsPerMeter = xPixelsPerMeter;
            YPixelsPerMeter = yPixelsPerMeter;
            NumberOfUsedColors = 256;
            NumberOfImportantColors = 256;
        }
    }
}