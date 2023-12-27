using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Sequential, Size = Size)]
    internal readonly ref struct InterferometricMeasurement
    {
        public const int Size = 2 * sizeof(float);
        public readonly float Delta;
        public readonly float Depth;
        public readonly bool IsValid
        {
            [SkipLocalsInit, MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Delta is not < 0.001f and not > 1000.0f && Depth is not < 1f and not > 250.0f;
        }
        public readonly float MetricDistance
        {
            [SkipLocalsInit, MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => .3048f * float.Hypot(Delta, Depth);
        }

        public readonly float AngleInDegrees
        {
            [SkipLocalsInit, MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => 90 - 360 / float.Tau * float.Atan2(Depth, Delta);
        }

        public override readonly string ToString() => $"{Delta};{Depth}";
    }
}