﻿using System;

namespace SL3Reader
{
    public static class BitmapHelper
    {
        private const int bufferPrototypeLength =
                            BitmapFileHeader.Size +
                            BitmapInfoHeader.Size +
                            256 * Bgra.Size;

        private static readonly byte[] bufferPrototype = new byte[bufferPrototypeLength]

        {
            0,0,0,0,0,0,0,0,0,0,
            0,0,0,0, /* BitmapFileHeader part, */

            0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0, /* BitmapInfoHeader part, */

            27,27,27,0,
            37,37,47,0,
            39,39,53,0,
            37,38,57,0,
            36,38,58,0,
            35,36,59,0,
            36,37,59,0,
            34,35,60,0,
            35,36,60,0,
            38,39,60,0,
            32,34,61,0,
            33,35,61,0,
            31,32,62,0,
            32,33,62,0,
            38,39,62,0,
            30,31,63,0,
            31,32,63,0,
            38,39,63,0,
            29,30,64,0,
            29,31,64,0,
            30,31,64,0,
            28,29,65,0,
            28,30,65,0,
            38,39,65,0,
            28,30,66,0,
            28,30,67,0,
            38,39,68,0,
            39,40,68,0,
            29,31,69,0,
            38,39,69,0,
            29,32,70,0,
            29,32,71,0,
            30,33,72,0,
            30,33,74,0,
            30,34,75,0,
            31,35,76,0,
            31,35,77,0,
            31,36,79,0,
            31,37,80,0,
            32,37,81,0,
            32,38,82,0,
            32,38,84,0,
            32,39,85,0,
            33,40,86,0,
            33,40,87,0,
            33,41,89,0,
            33,42,90,0,
            34,42,91,0,
            34,43,92,0,
            34,44,94,0,
            34,44,95,0,
            34,45,96,0,
            34,46,97,0,
            35,47,99,0,
            35,47,100,0,
            35,48,101,0,
            35,49,102,0,
            35,49,104,0,
            35,50,105,0,
            35,51,106,0,
            36,52,107,0,
            36,53,109,0,
            36,53,110,0,
            36,54,111,0,
            36,55,112,0,
            36,56,114,0,
            36,56,115,0,
            36,57,116,0,
            36,58,117,0,
            36,59,119,0,
            36,60,120,0,
            36,61,121,0,
            36,61,122,0,
            36,62,124,0,
            36,63,125,0,
            36,64,126,0,
            36,65,127,0,
            36,66,129,0,
            36,67,130,0,
            36,68,131,0,
            36,68,132,0,
            36,69,134,0,
            36,70,135,0,
            36,71,136,0,
            36,72,137,0,
            36,73,139,0,
            36,74,140,0,
            36,75,141,0,
            36,76,142,0,
            36,77,144,0,
            36,78,145,0,
            36,80,147,0,
            35,81,149,0,
            35,83,151,0,
            35,85,152,0,
            35,87,155,0,
            35,88,156,0,
            34,90,159,0,
            34,91,160,0,
            34,94,163,0,
            33,95,164,0,
            33,97,166,0,
            33,98,168,0,
            33,100,169,0,
            32,101,170,0,
            32,102,171,0,
            32,103,173,0,
            32,105,174,0,
            31,106,175,0,
            31,107,176,0,
            31,109,178,0,
            30,110,179,0,
            30,111,180,0,
            30,113,181,0,
            30,114,183,0,
            29,115,184,0,
            29,117,185,0,
            29,118,186,0,
            28,120,188,0,
            28,121,189,0,
            28,123,190,0,
            27,126,193,0,
            26,127,194,0,
            26,130,196,0,
            25,132,198,0,
            24,135,200,0,
            24,137,201,0,
            24,138,203,0,
            23,140,204,0,
            23,141,205,0,
            22,143,206,0,
            22,145,208,0,
            21,146,209,0,
            21,148,210,0,
            20,150,211,0,
            20,152,213,0,
            20,153,214,0,
            19,157,216,0,
            18,159,218,0,
            17,163,220,0,
            16,164,221,0,
            15,168,224,0,
            15,170,225,0,
            14,172,226,0,
            15,173,227,0,
            16,173,227,0,
            17,173,227,0,
            17,173,228,0,
            18,174,228,0,
            19,174,228,0,
            20,174,228,0,
            20,174,229,0,
            21,175,229,0,
            22,175,229,0,
            23,175,229,0,
            24,176,230,0,
            25,176,230,0,
            26,176,230,0,
            27,176,231,0,
            27,177,231,0,
            28,177,231,0,
            29,177,231,0,
            29,177,232,0,
            30,178,232,0,
            31,178,232,0,
            32,178,232,0,
            32,178,233,0,
            33,179,233,0,
            34,179,233,0,
            36,180,234,0,
            38,180,234,0,
            38,181,234,0,
            39,181,235,0,
            40,181,235,0,
            41,181,235,0,
            42,182,235,0,
            42,182,236,0,
            43,182,236,0,
            44,182,236,0,
            44,183,236,0,
            45,183,237,0,
            46,183,237,0,
            48,184,237,0,
            48,184,238,0,
            50,185,238,0,
            51,185,238,0,
            52,185,239,0,
            52,186,239,0,
            53,186,239,0,
            54,186,239,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0,
            56,187,240,0 /* Palette */
       };

        public static byte[] CreateBuffer(int maxHeight, int maxWidth)
        {
            byte[] buffer = GC.AllocateUninitializedArray<byte>(bufferPrototypeLength +
                                               (maxHeight * (maxWidth + (4 - maxWidth % 4))));
            Buffer.BlockCopy(bufferPrototype, 0, buffer, 0, bufferPrototypeLength);
            return buffer;
        }

        public static unsafe void UpdateBuffer(byte[] buffer,
            int height, int width,
            out long fileSize,
            out int fullStride,
            out Span<byte> fullBuffer,
            out Span<byte> dataBuffer)
        {
            int offset;
            fullStride = width + (4 - (width % 4));

            fixed (byte* pBuffer = buffer)
            {
                BitmapFileHeader* fileHeader = (BitmapFileHeader*)pBuffer;
                *fileHeader = new(width, height);
                fileSize = fileHeader->FileSize;
                offset = (int)fileHeader->OffsetOfImageData;
                *(BitmapInfoHeader*)(pBuffer + BitmapFileHeader.Size) = new(width, height);
            }
            fullBuffer = buffer.AsSpan();
            dataBuffer = fullBuffer[offset..(int)fileSize];
        }
    }
}