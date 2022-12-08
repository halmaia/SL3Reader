using System;
using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = Size)]
    public readonly ref struct ThreeDimensionalFrameHeader
    {
        public const int Size = 76;

        public readonly int HeaderSize { get; }
        public readonly int NumberOfLeftBytes { get; }
        public readonly int NumberOfRightBytes { get; }
        public readonly int NumberOfUnreliableBytes { get; }
        public readonly int NumberOfUnreliableRightBytes { get; }
        public readonly int NumberOfUnreliableLeftBytes { get; }
        public readonly int UnknownAt24 { get; }
        public readonly int UnknownAt28 { get; }
        public readonly int UnknownAt32 { get; }
        public readonly int UnknownAt36 { get; }
        public readonly int UnknownAt40 { get; }
        public readonly int UnknownAt44 { get; }
        public readonly int UnknownAt48 { get; }
        public readonly int UnknownAt52 { get; }
        public readonly int UnknownAt56 { get; }
        public readonly int UnknownAt60 { get; }
        public readonly int UnknownAt64 { get; }
        public readonly int UnknownAt68 { get; }
        public readonly int UnknownAt72 { get; }

    }

    [StructLayout(LayoutKind.Sequential, Size = Size)]
    public readonly ref struct InterferometricMeasuement
    {
        public const int Size = 2 * sizeof(float);

        public readonly float Delta { get; }
        public readonly float Depth { get; }

        public override readonly string ToString() => $"{Delta};{Depth}";

    }
}