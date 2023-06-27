using System.Runtime.InteropServices;

namespace SL3Reader {
    [StructLayout(LayoutKind.Sequential, Size = Size)]
    public readonly ref struct InterferometricMeasurement
    {
        public const int Size = 2 * sizeof(float);
        public readonly float Delta;
        public readonly float Depth;
        public override readonly string ToString() => $"{Delta};{Depth}";

        public readonly bool IsValid() =>
            Delta is not < 0.001f and not> 5000.0f && Depth is not< 1 and not> 250.0f;
    }
}