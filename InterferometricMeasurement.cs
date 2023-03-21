﻿using System.Runtime.InteropServices;

namespace SL3Reader {
    [StructLayout(LayoutKind.Sequential, Size = Size)]
    public readonly ref struct InterferometricMeasurement
    {
        public const int Size = 2 * sizeof(float);
        public readonly float Delta { get; }
        public readonly float Depth { get; }
        public override readonly string ToString() => $"{Delta};{Depth}";
    }
}