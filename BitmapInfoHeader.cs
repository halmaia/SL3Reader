using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = Size)]
    public readonly struct BitmapInfoHeader
    {
        public const int Size = 40;

        public readonly uint StructureSize;
        public readonly int Width;
        public readonly int Height;
        public readonly ushort Planes;
        public readonly ushort BitCount;
        public readonly BitmapCompressionMode Compression;
        public readonly uint ImageSize;
        public readonly int XPixelsPerMeter;
        public readonly int YPixelsPerMeter;
        public readonly uint NumberOfUsedColors;
        public readonly uint NumberOfImportantColors;

        public BitmapInfoHeader(int width,
                                int height,
                                ushort bitCount = 8,
                                int xPixelsPerMeter = 0,
                                int yPixelsPerMeter = 0)
        {
            StructureSize = Size;
            Width = width;
            Height = height;
            Planes = 1;
            BitCount = bitCount;
            Compression = BitmapCompressionMode.BI_RGB;
            ImageSize = (uint)(bitCount / 8 * height * width); // (width + (4 - width % 4))
            XPixelsPerMeter = xPixelsPerMeter;
            YPixelsPerMeter = yPixelsPerMeter;
            NumberOfUsedColors = 256;
            NumberOfImportantColors = 256;
        }
    }
}