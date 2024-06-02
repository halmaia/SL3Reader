using System.Diagnostics;
using System.Runtime.InteropServices;

namespace SL3Reader;

[StructLayout(LayoutKind.Sequential, Size = Size)]
[DebuggerDisplay($"{{{nameof(ToString)}(),nq}}")]
public readonly struct Bgra
{
    public const int Size = 4;
    public readonly byte B, G, R, A;

    public Bgra(byte b, byte g, byte r, byte a = 255)
    {
        B = b;
        G = g;
        R = r;
        A = a;
    }
    public override readonly string ToString() => string.Join(';', R, G, B, A);
}