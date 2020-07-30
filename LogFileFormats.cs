// Written by Ákos Halmai 2020.
// See LICENCE file for lic. terms.
// See https://doi.org/10.3390/ijgi9030149 for references.
// Univerity of Pécs, Faculty of Siences, Institute of Geography & Earthsciences
// http://foldrajz.ttk.pte.hu./

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.Contracts;
using System.IO;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Security;

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
        [ContractArgumentValidator]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void CheckArguments(in byte[] buffer, int offset, int size)
        {
            #region "Constants"
            const string NameOfBuffer = nameof(buffer);
            const string BufferIsNull = "The ‘" + NameOfBuffer + "’ argument can not be null.";
            const string NameOfOffset = nameof(offset);
            const string WrongOffsetParameter = "Wrong offset parameter.";
            #endregion "Constants"

            #region "Argument checks"
            if (offset < 0)
                throw new ArgumentOutOfRangeException(NameOfOffset, NameOfOffset + " should be greater than or equal to 0.");

            if (buffer == null)
                throw new ArgumentNullException(NameOfBuffer, BufferIsNull);

            int bufferLength = buffer.Length;

            if (bufferLength < size)
            {
                string WrongBufferSize = "The length of the ‘" + NameOfBuffer + "’ argument must be equal to or longer than " + size.ToString() + '.';
                throw new ArgumentOutOfRangeException(NameOfBuffer, WrongBufferSize);
            }

            if (bufferLength - offset < size)
                throw new ArgumentException(WrongOffsetParameter, NameOfOffset);
            #endregion "Argument checks"
            Contract.EndContractBlock();
        }
    }
    #endregion Argument checks

    #region File reader
    public static class SonarLogFileReader
    {
        public static List<SL3Frame> ReadSL3SonarLogFile(string path, out SonarLogFileHeader header)
        {
            FileStream file = File.OpenRead(path);
            byte[] buffer = new byte[SL3Frame.Size];
            file.Read(buffer, 0, SonarLogFileHeader.Size);
            header = new SonarLogFileHeader(buffer);

            int position = SonarLogFileHeader.Size;
            long length = file.Length;
            List<SL3Frame> frames = new List<SL3Frame>((int)(length / 2000L));

            while (SL3Frame.Size + position < length)
            {
                file.Read(buffer, 0, SL3Frame.Size);
                SL3Frame frame = new SL3Frame(in buffer, 0, false);
                position += frame.TotalLength;
                file.Seek(position, SeekOrigin.Begin);
                frames.Add(frame);
            }

            file.Close();
            return frames;
        }
    }
    #endregion File reader

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

    [StructLayout(LayoutKind.Explicit, Size = Size)]
    [DebuggerDisplay("{ToString()}")]
    public readonly struct SL3Frame
    {
        public const int Size = 168;

        // Field offset attribute makes able us to comment out
        // unused & unknown properties.
        [field: FieldOffset(0)] public uint PositionOfFirstByte { get; } // 0 --> 3
        [field: FieldOffset(4)] public uint Unknown1 { get; } // 4 --> 7
        [field: FieldOffset(8)] public ushort TotalLength { get; } // 8 --> 9
        [field: FieldOffset(10)] public ushort PreviousLength { get; } // 10 --> 11
        [field: FieldOffset(12)] public SurveyType SurveyType { get; } // 12 --> 13
        [field: FieldOffset(14)] public ushort Unknown2 { get; } // 14 --> 15
        [field: FieldOffset(16)] public uint NumberOfCampaignInThisType { get; } // 16 --> 19
        [field: FieldOffset(20)] public float MinRange { get; }  // 20 --> 23
        [field: FieldOffset(24)] public float MaxRange { get; } // 24 --> 27
        [field: FieldOffset(28)] public int Unknown3 { get; } // 28 --> 31
        [field: FieldOffset(32)] public int Unknown4 { get; } // 32 --> 35
        [field: FieldOffset(36)] public int Unknown5 { get; } // 36 --> 39
        [field: FieldOffset(40)] public uint HardwareTime { get; } // 40 --> 43
        [field: FieldOffset(44)] public int OriginalLengthOfEchoData { get; } // 44 --> 47
        [field: FieldOffset(48)] public float WaterDepth { get; } // 48 --> 51
        [field: FieldOffset(52)] public Frequency Frequency { get; } // 52 --> 53
        [field: FieldOffset(54)] public int Unknown6 { get; } // 54 --> 57
        [field: FieldOffset(58)] public int Unknown7 { get; } // 58 --> 61
        [field: FieldOffset(62)] public int Unknown8 { get; } // 62 --> 65
        [field: FieldOffset(66)] public int Unknown9 { get; } // 66 --> 69
        [field: FieldOffset(70)] public int Unknown10 { get; } // 70 --> 73
        [field: FieldOffset(74)] public int Unknown11 { get; } // 74 --> 77
        [field: FieldOffset(78)] public int Unknown12 { get; } // 78 --> 81
        [field: FieldOffset(82)] public short Unknown13 { get; } // 82 --> 83
        [field: FieldOffset(84)] public float GNSSSpeed { get; } // 84 --> 87
        [field: FieldOffset(88)] public float WaterTemperature { get; } // 88 --> 91
        [field: FieldOffset(92)] public int X { get; } // 92 --> 95
        [field: FieldOffset(96)] public int Y { get; } // 96 --> 99
        [field: FieldOffset(100)] public float WaterSpeed { get; } // 100 --> 103
        [field: FieldOffset(104)] public float GNSSHeading { get; } // 104 --> 107
        [field: FieldOffset(108)] public float GNSSAltitude { get; } // 108 --> 111
        [field: FieldOffset(112)] public float MagneticHeading { get; } // 112 --> 115
        [field: FieldOffset(116)] public FrameFlags Flags { get; } // 116 --> 117
        [field: FieldOffset(118)] public int Unknown14 { get; } // 118 --> 121
        [field: FieldOffset(122)] public short Unknown15 { get; } // 122 --> 123
        [field: FieldOffset(124)] public uint Milliseconds { get; } //(124)
        [field: FieldOffset(128)] public uint LastPrimaryChannelFrameOffset { get; }  //(128)
        [field: FieldOffset(132)] public uint LastSecondaryChannelFrameOffset { get; }  //(132)
        [field: FieldOffset(136)] public uint LastDownScanChannelFrameOffset { get; }  //(136)
        [field: FieldOffset(140)] public uint LastSidescanLeftChannelFrameOffset { get; }  //(140)
        [field: FieldOffset(144)] public uint LastSidescanRightChannelFrameOffset { get; }  //(144)
        [field: FieldOffset(148)] public uint LastSidescanCompositeChannelFrameOffset { get; }  //(148)
        [field: FieldOffset(152)] public uint UnknownOffset1 { get; } // 152 --> 155
        [field: FieldOffset(156)] public uint UnknownOffset2 { get; } // 156 --> 159
        [field: FieldOffset(160)] public uint UnknownOffset3 { get; } // 160 --> 163
        [field: FieldOffset(164)] public uint Last3DChannelFrameOffset { get; }  //(164)

        [SecuritySafeCritical()]
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
