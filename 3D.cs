using System;
using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = Size)]
    public readonly struct ThreeDimensionalFrameHeader
    {
        public const int Size = 76;

        public int HeaderSize { get; }
        public int NumberOfLeftBytes { get; }
        public int NumberOfRightBytes { get; }
        public int NumberOfUnreliableBytes { get; }
        public int NumberOfUnreliableRightBytes { get; }
        public int NumberOfUnreliableLeftBytes { get; }
        public int UnknownAt24 { get; }
        public int UnknownAt28 { get; }
        public int UnknownAt32 { get; }
        public int UnknownAt36 { get; }
        public int UnknownAt40 { get; }
        public int UnknownAt44 { get; }
        public int UnknownAt48 { get; }
        public int UnknownAt52 { get; }
        public int UnknownAt56 { get; }
        public int UnknownAt60 { get; }
        public int UnknownAt64 { get; }
        public int UnknownAt68 { get; }
        public int UnknownAt72 { get; }

    }
}
