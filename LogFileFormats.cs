// Written by Ákos Halmai 2020.
// See LICENCE file for lic. terms.
// Univerity of Pécs, Faculty of Siences, Institute of Geography & Earthsciences
// http://foldrajz.ttk.pte.hu./

namespace LowranceReader2
{
    #region Enums
    public enum SonarLogFileFormat : ushort
    {
        SLG = 1,
        SL2, // Auto-increment
        SL3
    }

    public enum SurveyType : ushort
    {
        Primary, Secondary, DownScan,
        LeftSidescan, RightSidescan,
        SideScan, Unknown7 = 7,
        Unknown8, ThreeDimensional,
        DebugDigital, DebugNoise
    }

    public enum Frequency : ushort
    {
        Fq_200kHz, Fq_50kHz, Fq_83kHz, Fq_455kHz, Fq_800kHz, Fq_38kHz, Fq_28kHz,
        Fq_130kHz_210kHz, Fq_90kHz_150kHz, Fq_40kHz_60kHz, Fq_25kHz_45kHz
    }
    #endregion Enums

    #region Argument checks
    internal static class ArgumentChecks
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void CheckArguments(in byte[] buffer, int offset, int size)
        {
            #region "Constants"
            const string NameOfBuffer = nameof(buffer);
            const string BufferIsNull = "The ‘" + NameOfBuffer + "’ argument can not be null.";
            const string NameOfOffset = nameof(offset);
            const string WrongOffsetParameter = "Wrong offset parameter.";
            string WrongBufferSize = "The length of the ‘" + NameOfBuffer + "’ argument must be equal to or longer than " + size.ToString() + '.';
            #endregion "Constants"

            #region "Argument checks"
            if (offset < 0)
                throw new ArgumentOutOfRangeException(NameOfOffset, NameOfOffset + " should be greater than or equal to 0.");

            if (buffer == null)
                throw new ArgumentNullException(NameOfBuffer, BufferIsNull);

            int bufferLength = buffer.Length;

            if (bufferLength < size)
                throw new ArgumentOutOfRangeException(NameOfBuffer, WrongBufferSize);

            if (bufferLength - offset < size)
                throw new ArgumentException(WrongOffsetParameter, NameOfOffset);
            #endregion "Argument checks" 
        }
    }
    #endregion Argument checks

    #region Structs
    [StructLayout(LayoutKind.Sequential, Size = Size)]
    [DebuggerDisplay("{ToString()}")]
    public readonly struct SonarLogFileHeader
    {
        public const int Size = 8;

        private const SonarLogFileFormat DesiredFormat = SonarLogFileFormat.SL3;
        private const short DesiredBlockSize = 3200;

        public SonarLogFileFormat FileFormat { get; }
        public short DeviceTypeID { get; }
        public short BlockSize { get; }
        public short Unknown { get; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => FileFormat.ToString() + " (" + BlockSize.ToString() + ")";

        [SecuritySafeCritical()]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe SonarLogFileHeader(byte[] buffer, int offset = 0)
        {
            #region "Argument checks"
            ArgumentChecks.CheckArguments(buffer, offset, Size);
            #endregion "Argument checks"

            #region "Unsafe cast of incoming buffer."
            fixed (byte* p = &buffer[offset])
                this = *(SonarLogFileHeader*)p;
            #endregion "Unsafe cast of incoming buffer."
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsValidFormat(
            SonarLogFileFormat desirdFormat = DesiredFormat,
            short desiredBlockSize = DesiredBlockSize)
            =>
            FileFormat == desirdFormat && BlockSize == desiredBlockSize;
    }

    [StructLayout(LayoutKind.Explicit, Size = Size)]
    public readonly struct FrameFlags
    {
        public const int Size = 2;
        [FieldOffset(0)] private readonly ushort ByteStore;

        public bool IsTrackValid => (ByteStore & 0B1U) != 0;
        public bool IsUnknownAt1 => (ByteStore & 0B10) != 0;
        public bool IsUnknownAt2 => (ByteStore & 0B100) != 0;
        public bool IsPositionValid => (ByteStore & 0B1000) != 0;
        public bool IsUnknownAt4 => (ByteStore & 0B10000) != 0;
        public bool IsCourseOrSpeed => (ByteStore & 0B100000) != 0;
        public bool IsSpeedValid => (ByteStore & 0B1000000) != 0;
        public bool IsUnknownAt7 => (ByteStore & 0B10000000) != 0;
        public bool IsUnknownAt8 => (ByteStore & 0B100000000) != 0;
        public bool IsAltitudeOrCourseOrSpeed => (ByteStore & 0B1000000000) != 0;
        public bool IsUnknownAt10 => (ByteStore & 0B10000000000) != 0;
        public bool IsUnknownAt11 => (ByteStore & 0B100000000000) != 0;
        public bool IsUnknownAt12 => (ByteStore & 0B1000000000000) != 0;
        public bool IsUnknownAt13 => (ByteStore & 0B10000000000000) != 0;
        public bool IsAltitudeValid => (ByteStore & 0B100000000000000) != 0;
        public bool IsHeadingValid => (ByteStore & 0B1000000000000000) != 0;
    }

    [StructLayout(LayoutKind.Sequential, Size = Size)]
    [DebuggerDisplay("{ToString()}")]
    public readonly struct SL3Frame
    {
        public const int Size = 168;

        public uint PositionOfFirstByte { get; } // 0 --> 3
        public uint Unknown1 { get; } // 4 --> 7
        public ushort TotalLength { get; } // 8 --> 9
        public ushort PreviousLength { get; } // 10 --> 11
        public SurveyType SurveyType { get; } // 12 --> 13
        public ushort Unknown2 { get; } // 14 --> 15
        public uint NumberOfCampaignInThisType { get; } // 16 --> 19
        public float MinRange { get; }  // 20 --> 23
        public float MaxRange { get; } // 24 --> 27
        public int Unknown3 { get; } // 28 --> 31
        public int Unknown4 { get; } // 32 --> 35
        public int Unknown5 { get; } // 33 --> 39
        public uint HardwareTime { get; } // 40 --> 43
        public uint OriginalLengthOfEchoData { get; } // 44 --> 47
        public float WaterDepth { get; } // 48 --> 51
        public Frequency Frequency { get; } // 52 --> 53
        public int Unknown6 { get; } // 54 --> 57
        public int Unknown7 { get; } // 58 --> 61
        public int Unknown8 { get; } // 62 --> 65
        public int Unknown9 { get; } // 66 --> 69
        public int Unknown10 { get; } // 70 --> 73
        public int Unknown11 { get; } // 74 --> 77
        public int Unknown12 { get; } // 78 --> 81
        public short Unknown13 { get; } // 82 --> 83
        public float GNSSSpeed { get; } // 84 --> 87
        public float WaterTemperature { get; } // 88 --> 91
        public int X { get; } // 92 --> 95
        public int Y { get; } // 96 --> 99
        public float WaterSpeed { get; } // 100 --> 103
        public float GNSSHeading { get; } // 104 --> 107
        public float GNSSAltitude { get; } // 108 --> 111
        public float MagneticHeading { get; } // 112 --> 115
        public FrameFlags Flags { get; } // 116 --> 117
        public int Unknown14 { get; } // 118 --> 121
        public short Unknown15 { get; } // 122 --> 123
        public uint Milliseconds { get; } //(124)
        public uint LastPrimaryChannelFrameOffset { get; }  //(128)
        public uint LastSecondaryChannelFrameOffset { get; }  //(132)
        public uint LastDownScanChannelFrameOffset { get; }  //(136)
        public uint LastSidescanLeftChannelFrameOffset { get; }  //(140)
        public uint LastSidescanRightChannelFrameOffset { get; }  //(144)
        public uint LastSidescanCompositeChannelFrameOffset { get; }  //(148)
        public int Unknown16 { get; } // 152 --> 155
        public int Unknown17 { get; } // 156 --> 159
        public int Unknown18 { get; } // 160 --> 163
        public uint Last3DChannelFrameOffset { get; }  //(164)

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe SL3Frame(in byte[] buffer, int offset = 0, bool strictParameterCheck = true)
        {
            #region "Argument checks"
            if (strictParameterCheck)
                ArgumentChecks.CheckArguments(buffer, offset, Size);
            #endregion "Argument checks"

            fixed (byte* p = &buffer[offset])
                this = *(SL3Frame*)p;
        }
    }
    #endregion Structs
}
