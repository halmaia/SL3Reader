﻿using System.Diagnostics;
using System.IO;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SL3Reader;

[StructLayout(LayoutKind.Sequential, Size = Size)]
[DebuggerDisplay($"{{{nameof(ToString)}(),nq}}")]
internal readonly ref struct SLFileHeader
{
    public const int Size = 8;

    public readonly LogFileFormat FileFormat { get; }
    public readonly short DeviceID { get; }

    public readonly short BlockSize { get; }

    public readonly short Reserved { get; }

    [SkipLocalsInit]
    public readonly override string ToString() =>
        FileFormat.ToString() + " (" + BlockSize.ToString() + ')';

    [MethodImpl(MethodImplOptions.AggressiveInlining), SkipLocalsInit]
    public readonly void ThrowIfInvalidFormatDetected()
    {
        if (FileFormat is not LogFileFormat.SL3)
            throw new InvalidDataException("Unsupported file type. Expected type SL3.");
    }
};