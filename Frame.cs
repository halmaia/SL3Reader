using System.Diagnostics;
using static System.Globalization.CultureInfo;
using System.Runtime.InteropServices;
using System;

namespace SL3Reader
{
    public interface IFrame
    {
        uint PositionOfFirstByte { get; }
        uint UnknownAt4 { get; }
        ushort TotalLength { get; }
        ushort PreviousLength { get; }
        SurveyType SurveyType { get; }
        short UnknownAt14 { get; }
        uint NumberOfCampaignInThisType { get; }
        float MinRange { get; }
        float MaxRange { get; }
        float UnknownAt28 { get; }
        float UnknownAt32 { get; }
        float UnknownAt36 { get; }
        uint HardwareTime { get; }
        uint OriginalLengthOfEchoData { get; }
        float WaterDepth { get; }
        Frequency Frequency { get; }
        float UnknownAt54 { get; }
        float UnknownAt58 { get; }
        short UnknownAt62 { get; }
        float UnknownAt64 { get; }
        float UnknownAt68 { get; }
        float UnknownAt72 { get; }
        float UnknownAt76 { get; }
        float UnknownAt80 { get; }
        float GNSSSpeed { get; }
        float WaterTemperature { get; }
        int X { get; }
        int Y { get; }
        float WaterSpeed { get; }
        float GNSSHeading { get; }
        float GNSSAltitude { get; }
        float MagneticHeading { get; }
        DataValidity Flags { get; }
        float UnknownAt118 { get; }
        ushort UnknownAt122 { get; }
        uint Milliseconds { get; }

        // Derived:
        long DataOffset { get; }
    }

    public interface IFrameNavigation
    {
        uint LastPrimaryChannelFrameOffset { get; }
        uint LastSecondaryChannelFrameOffset { get; }
        uint LastDownScanChannelFrameOffset { get; }
        uint LastSidescanLeftChannelFrameOffset { get; }
        uint LastSidescanRightChannelFrameOffset { get; }
        uint LastSidescanCompositeChannelFrameOffset { get; }
        uint UnknownAt152 { get; }
        uint UnknownAt156 { get; }
        uint UnknownAt160 { get; }
        uint Last3DChannelFrameOffset { get; }
    }

    [StructLayout(LayoutKind.Explicit, Size = Size)]
    [DebuggerDisplay($"{{{nameof(GetDebuggerDisplay)}(),nq}}")]
    public readonly struct BasicFrame : IFrame
    {
        public const int Size = 128;

        #region IFrame
        [field: FieldOffset(0)] public readonly uint PositionOfFirstByte { get; } // (0)
        [field: FieldOffset(4)] public readonly uint UnknownAt4 { get; } // (4)
        [field: FieldOffset(8)] public readonly ushort TotalLength { get; } // (8)
        [field: FieldOffset(10)] public readonly ushort PreviousLength { get; } // (10)
        [field: FieldOffset(12)] public readonly SurveyType SurveyType { get; } // (12)
        [field: FieldOffset(14)] public readonly short UnknownAt14 { get; } // (14)
        [field: FieldOffset(16)] public readonly uint NumberOfCampaignInThisType { get; } // (16)
        [field: FieldOffset(20)] public readonly float MinRange { get; } // (20)
        [field: FieldOffset(24)] public readonly float MaxRange { get; } // (24)

        [field: FieldOffset(28)] public readonly float UnknownAt28 { get; } // (28)
        [field: FieldOffset(32)] public readonly float UnknownAt32 { get; } // (32)
        [field: FieldOffset(36)] public readonly float UnknownAt36 { get; } // (36)

        [field: FieldOffset(40)] public readonly uint HardwareTime { get; } // (40)

        [field: FieldOffset(44)] public readonly uint OriginalLengthOfEchoData { get; } // (44)
        [field: FieldOffset(48)] public readonly float WaterDepth { get; } // (48)
        [field: FieldOffset(52)] public readonly Frequency Frequency { get; } // (52)
        [field: FieldOffset(54)] public readonly float UnknownAt54 { get; } // (54)
        [field: FieldOffset(58)] public readonly float UnknownAt58 { get; } // (58)
        [field: FieldOffset(62)] public readonly short UnknownAt62 { get; } // (62)

        [field: FieldOffset(64)] public readonly float UnknownAt64 { get; } // (64)
        [field: FieldOffset(68)] public readonly float UnknownAt68 { get; } // (68)
        [field: FieldOffset(72)] public readonly float UnknownAt72 { get; } // (72)
        [field: FieldOffset(76)] public readonly float UnknownAt76 { get; } // (76)
        [field: FieldOffset(80)] public readonly float UnknownAt80 { get; } // (80)

        [field: FieldOffset(84)] public readonly float GNSSSpeed { get; } // (84)
        [field: FieldOffset(88)] public readonly float WaterTemperature { get; }  // (88)

        [field: FieldOffset(92)] public readonly int X { get; } // (92)
        [field: FieldOffset(96)] public readonly int Y { get; } // (96)

        [field: FieldOffset(100)] public readonly float WaterSpeed { get; } // (100)

        [field: FieldOffset(104)] public readonly float GNSSHeading { get; } // (104)
        [field: FieldOffset(108)] public readonly float GNSSAltitude { get; } // (108)
        [field: FieldOffset(112)] public readonly float MagneticHeading { get; } // (112)

        [field: FieldOffset(116)] public readonly DataValidity Flags { get; } // (116)
        [field: FieldOffset(118)] public readonly float UnknownAt118 { get; } // (118)
        [field: FieldOffset(122)] public readonly ushort UnknownAt122 { get; } // (122)
        [field: FieldOffset(124)] public readonly uint Milliseconds { get; } // (124)
        public readonly long DataOffset => PositionOfFirstByte + Size;
        #endregion IFrame

        #region DateTime support
        private static DateTime timestampBase;
        internal static void InitTimestampBase(uint hardwareTime) =>
                 timestampBase = DateTimeOffset.FromUnixTimeSeconds(hardwareTime)
                 .UtcDateTime;
        public readonly DateTime Timestamp =>
            timestampBase.AddMilliseconds(Milliseconds);
        #endregion

        public readonly override string ToString()
        {
            IFormatProvider culture = InvariantCulture;
            return string.Join(',', new string[]
        {
            SurveyType.ToString(),
            WaterDepth.ToString(culture),
            X.ToString(),
            Y.ToString(),
            GNSSAltitude.ToString(culture),
            GNSSHeading.ToString(culture),
            GNSSSpeed.ToString(culture),
            MagneticHeading.ToString(culture),
            MinRange.ToString(culture),
            MaxRange.ToString(culture),
            WaterTemperature.ToString(culture),
            WaterSpeed.ToString(culture),
            HardwareTime.ToString(),
            Frequency.ToString(),
            Milliseconds.ToString()});
        }

        private readonly string GetDebuggerDisplay() =>
            string.Join("; ", new string[4]
            {
                SurveyType.ToString(),
                WaterDepth.ToString(InvariantCulture) + '′',
                MinRange.ToString(InvariantCulture),
                MaxRange.ToString(InvariantCulture)
            });
    }

    [StructLayout(LayoutKind.Explicit, Size = Size)]
    [DebuggerDisplay($"{{{nameof(GetDebuggerDisplay)}(),nq}}")]
    public readonly struct ExtendedFrame : IFrame, IFrameNavigation
    {
        public const int Size = 168;

        #region IFrame
        [field: FieldOffset(0)] public readonly uint PositionOfFirstByte { get; } // (0)
        [field: FieldOffset(4)] public readonly uint UnknownAt4 { get; } // (4)
        [field: FieldOffset(8)] public readonly ushort TotalLength { get; } // (8)
        [field: FieldOffset(10)] public readonly ushort PreviousLength { get; } // (10)
        [field: FieldOffset(12)] public readonly SurveyType SurveyType { get; } // (12)
        [field: FieldOffset(14)] public readonly short UnknownAt14 { get; } // (14)
        [field: FieldOffset(16)] public readonly uint NumberOfCampaignInThisType { get; } // (16)
        [field: FieldOffset(20)] public readonly float MinRange { get; } // (20)
        [field: FieldOffset(24)] public readonly float MaxRange { get; } // (24)

        [field: FieldOffset(28)] public readonly float UnknownAt28 { get; } // (28)
        [field: FieldOffset(32)] public readonly float UnknownAt32 { get; } // (32)
        [field: FieldOffset(36)] public readonly float UnknownAt36 { get; } // (36)

        [field: FieldOffset(40)] public readonly uint HardwareTime { get; } // (40)

        [field: FieldOffset(44)] public readonly uint OriginalLengthOfEchoData { get; } // (44)
        [field: FieldOffset(48)] public readonly float WaterDepth { get; } // (48)
        [field: FieldOffset(52)] public readonly Frequency Frequency { get; } // (52)
        [field: FieldOffset(54)] public readonly float UnknownAt54 { get; } // (54)
        [field: FieldOffset(58)] public readonly float UnknownAt58 { get; } // (58)
        [field: FieldOffset(62)] public readonly short UnknownAt62 { get; } // (62)

        [field: FieldOffset(64)] public readonly float UnknownAt64 { get; } // (64)
        [field: FieldOffset(68)] public readonly float UnknownAt68 { get; } // (68)
        [field: FieldOffset(72)] public readonly float UnknownAt72 { get; } // (72)
        [field: FieldOffset(76)] public readonly float UnknownAt76 { get; } // (76)
        [field: FieldOffset(80)] public readonly float UnknownAt80 { get; } // (80)

        [field: FieldOffset(84)] public readonly float GNSSSpeed { get; } // (84)
        [field: FieldOffset(88)] public readonly float WaterTemperature { get; }  // (88)

        [field: FieldOffset(92)] public readonly int X { get; } // (92)
        [field: FieldOffset(96)] public readonly int Y { get; } // (96)

        [field: FieldOffset(100)] public readonly float WaterSpeed { get; } // (100)

        [field: FieldOffset(104)] public readonly float GNSSHeading { get; } // (104)
        [field: FieldOffset(108)] public readonly float GNSSAltitude { get; } // (108)
        [field: FieldOffset(112)] public readonly float MagneticHeading { get; } // (112)

        [field: FieldOffset(116)] public readonly DataValidity Flags { get; } // (116)
        [field: FieldOffset(118)] public readonly float UnknownAt118 { get; } // (118)
        [field: FieldOffset(122)] public readonly ushort UnknownAt122 { get; } // (122)
        [field: FieldOffset(124)] public readonly uint Milliseconds { get; } // (124)

        public readonly long DataOffset => PositionOfFirstByte + Size;
        #endregion IFrame

        #region IFrameNavigation
        [field: FieldOffset(128)] public readonly uint LastPrimaryChannelFrameOffset { get; } // (128)
        [field: FieldOffset(132)] public readonly uint LastSecondaryChannelFrameOffset { get; }  // (132)
        [field: FieldOffset(136)] public readonly uint LastDownScanChannelFrameOffset { get; }  // (136)
        [field: FieldOffset(140)] public readonly uint LastSidescanLeftChannelFrameOffset { get; }  // (140)
        [field: FieldOffset(144)] public readonly uint LastSidescanRightChannelFrameOffset { get; }  // (144)
        [field: FieldOffset(148)] public readonly uint LastSidescanCompositeChannelFrameOffset { get; }  // (148)
        [field: FieldOffset(152)] public readonly uint UnknownAt152 { get; }  // (152)
        [field: FieldOffset(156)] public readonly uint UnknownAt156 { get; }  // (156)
        [field: FieldOffset(160)] public readonly uint UnknownAt160 { get; }  // (160)
        [field: FieldOffset(164)] public readonly uint Last3DChannelFrameOffset { get; }  // (164)
        
        #endregion IFrame Navigation

        #region DateTime support
        private static DateTime timestampBase;
        internal static void InitTimestampBase(uint hardwareTime) =>
                 timestampBase = DateTimeOffset.FromUnixTimeSeconds(hardwareTime)
                 .UtcDateTime;
        public readonly DateTime Timestamp => timestampBase.AddMilliseconds(Milliseconds);
        #endregion

        public readonly override string ToString()
        {
            IFormatProvider culture = InvariantCulture;
            return string.Join(',', new string[]
        {
            SurveyType.ToString(),
            WaterDepth.ToString(culture),
            X.ToString(),
            Y.ToString(),
            GNSSAltitude.ToString(culture),
            GNSSHeading.ToString(culture),
            GNSSSpeed.ToString(culture),
            MagneticHeading.ToString(culture),
            MinRange.ToString(culture),
            MaxRange.ToString(culture),
            WaterTemperature.ToString(culture),
            WaterSpeed.ToString(culture),
            HardwareTime.ToString(),
            Frequency.ToString(),
            Milliseconds.ToString()});
        }

        private readonly string GetDebuggerDisplay() =>
            string.Join("; ", new string[4]
            {
                SurveyType.ToString(),
                WaterDepth.ToString(InvariantCulture) + '′',
                MinRange.ToString(InvariantCulture),
                MaxRange.ToString(InvariantCulture)
            });
    }
}