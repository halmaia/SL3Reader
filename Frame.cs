using System.Diagnostics;
using static System.Globalization.CultureInfo;
using System.Runtime.InteropServices;
using System;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Explicit, Size = Size)]
    [DebuggerDisplay($"{{{nameof(GetDebuggerDisplay)}(),nq}}")]
    public readonly struct Frame
    {
        public const int Size = 168;

        [FieldOffset(0)] public readonly uint PositionOfFirstByte; //(0)
        [FieldOffset(4)] public readonly uint UnknownAt4; //(4)
        [FieldOffset(8)] public readonly ushort TotalLength; //(8)
        [FieldOffset(10)] public readonly ushort PreviousLength; //(10)
        [FieldOffset(12)] public readonly SurveyType SurveyType; //(12)
        [FieldOffset(14)] public readonly short UnknownAt14; //(14)
        [FieldOffset(16)] public readonly uint NumberOfCampaignInThisType; //(16)
        [FieldOffset(20)] public readonly float MinRange; //(20)
        [FieldOffset(24)] public readonly float MaxRange; //(24)

        [FieldOffset(28)] public readonly float UnknownAt28; //(28)
        [FieldOffset(32)] public readonly float UnknownAt32; //(32)
        [FieldOffset(36)] public readonly float UnknownAt36; //(36)

        [FieldOffset(40)] public readonly uint HardwareTime; //40

        [FieldOffset(44)] public readonly uint OriginalLengthOfEchoData; //(44)
        [FieldOffset(48)] public readonly float WaterDepth; //(48)
        [FieldOffset(52)] public readonly Frequency Frequency; //(52)
        [FieldOffset(54)] public readonly float UnknownAt54; //(54)
        [FieldOffset(58)] public readonly float UnknownAt58; //(58)
        [FieldOffset(62)] public readonly short UnknownAt62; //(62)

        [FieldOffset(64)] public readonly float UnknownAt64; //(64)
        [FieldOffset(68)] public readonly float UnknownAt68; //(68)
        [FieldOffset(72)] public readonly float UnknownAt72; //(72)
        [FieldOffset(76)] public readonly float UnknownAt76; //(76)
        [FieldOffset(80)] public readonly float UnknownAt80; //(80)

        [FieldOffset(84)] public readonly float GNSSSpeed; //(84)
        [FieldOffset(88)] public readonly float WaterTemperature;  //(88)

        [FieldOffset(92)] public readonly int X; //(92)
        [FieldOffset(96)] public readonly int Y; //(96)

        [FieldOffset(100)] public readonly float WaterSpeed; //(100)

        [FieldOffset(104)] public readonly float GNSSHeading; //(104)
        [FieldOffset(108)] public readonly float GNSSAltitude; //(108)
        [FieldOffset(112)] public readonly float MagneticHeading; //(112)

        [FieldOffset(116)] public readonly DataValidity Flags; //(116)
        [FieldOffset(118)] public readonly float UnknownAt118; //(118)
        [FieldOffset(122)] public readonly ushort UnknownAt122; //(122)
        [FieldOffset(124)] public readonly uint Milliseconds; //(124)
        [FieldOffset(128)] public readonly uint LastPrimaryChannelFrameOffset; //(128)
        [FieldOffset(132)] public readonly uint LastSecondaryChannelFrameOffset;  //(132)
        [FieldOffset(136)] public readonly uint LastDownScanChannelFrameOffset;  //(136)
        [FieldOffset(140)] public readonly uint LastSidescanLeftChannelFrameOffset;  //(140)
        [FieldOffset(144)] public readonly uint LastSidescanRightChannelFrameOffset;  //(144)
        [FieldOffset(148)] public readonly uint LastSidescanCompositeChannelFrameOffset;  //(148)
        [FieldOffset(152)] public readonly uint UnknownAt152;
        [FieldOffset(156)] public readonly uint UnknownAt156;
        [FieldOffset(160)] public readonly uint UnknownAt160;
        [FieldOffset(164)] public readonly uint Last3DChannelFrameOffset;  //(164)

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