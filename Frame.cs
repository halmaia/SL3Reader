using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;
using System.Globalization;
using static System.Globalization.CultureInfo;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Explicit, Size = ExtendedSize)]
    [DebuggerDisplay($"{{{nameof(GetDebuggerDisplay)}(),nq}}")]
    public readonly struct Frame
    {
        public const int ExtendedSize = 168;
        public const int BasicSize = 128;
        public const int MinimumInitSize = 44;

        public const double KnotsToMPS = 1852d / 3600d;
        public const double HalfPI = double.Pi / 2d;
        public const double MillisecondsToSeconds = 1d / 1000d;

        private const double PolarRadius = 6356752.3142d; // Lowrance uses float, causing truncated precision.
        private const double RadToDeg = 360d / double.Tau;

        private const double FootToMeter = .3048;
        private const char FieldSeparator = ',';
        private const string DebugFieldSeparator = "; ";

        private const string DateTimeFormat = "yyyy'-'MM'-'dd HH':'mm':'ss.fff'Z'";

        private static readonly CultureInfo invariantCulture = InvariantCulture;

        #region Basic Properties
        [field: FieldOffset(0)] public readonly uint PositionOfFirstByte { get; } // (0)
        [field: FieldOffset(4)] public readonly uint UnknownAt4 { get; } // (4) In my files it is always 10.
        [field: FieldOffset(8)] public readonly ushort LengthOfFrame { get; } // (8)
        [field: FieldOffset(10)] public readonly ushort PreviousFrameLength { get; } // (10)
        [field: FieldOffset(12)] public readonly SurveyType SurveyType { get; } // (12)
        [field: FieldOffset(14)] public readonly short PackingAt14 { get; } // (14) Always 0 for me.
        [field: FieldOffset(16)] public readonly uint CampaignID { get; } // (16)
        [field: FieldOffset(20)] public readonly float MinRange { get; } // (20)
        [field: FieldOffset(24)] public readonly float MaxRange { get; } // (24)
        [field: FieldOffset(28)] public readonly float UnknownAt28 { get; } // (28)
        [field: FieldOffset(32)] public readonly float UnknownAt32 { get; } // (32)
        [field: FieldOffset(36)] public readonly float UnknownAt36 { get; } // (36)
        [field: FieldOffset(40)] public readonly uint HardwareTime { get; } // (40)
        [field: FieldOffset(44)] public readonly uint LengthOfEchoData { get; } // (44)
        [field: FieldOffset(48)] public readonly float WaterDepth { get; } // (48)
        [field: FieldOffset(52)] public readonly Frequency Frequency { get; } // (52)
        [field: FieldOffset(54)] public readonly float UnknownAt54 { get; } // (54)
        [field: FieldOffset(58)] public readonly float UnknownAt58 { get; } // (58)
        [field: FieldOffset(62)] public readonly ushort UnknownAt62 { get; } // (62)
        [field: FieldOffset(64)] public readonly float UnknownAt64 { get; } // (64) // Fix: -1
        [field: FieldOffset(68)] public readonly float UnknownAt68 { get; } // (68) // Fix: -1
        [field: FieldOffset(72)] public readonly float UnknownAt72 { get; } // (72) // Fix: -1
        [field: FieldOffset(76)] public readonly float UnknownAt76 { get; } // (76) // Fix: -1
        [field: FieldOffset(80)] public readonly float UnknownAt80 { get; } // (80) // Fix: 0
        [field: FieldOffset(84)] public readonly float GNSSSpeed { get; } // (84)
        [field: FieldOffset(88)] public readonly float WaterTemperature { get; }  // (88)
        [field: FieldOffset(92)] public readonly int X { get; } // (92)
        [field: FieldOffset(96)] public readonly int Y { get; } // (96)
        [field: FieldOffset(100)] public readonly float WaterSpeed { get; } // (100)
        /// <summary>
        /// Heading of the vessel, calculation based on GNSS readings only.
        /// (No drift was taken into account.) Unit: radians. Direction: azimuthal (↻⁻;
        /// north is zero or 2·π, east is π/2.)
        /// </summary>
        [field: FieldOffset(104)] public readonly float GNSSHeading { get; } // (104)
        [field: FieldOffset(108)] public readonly float GNSSAltitude { get; } // (108)
        [field: FieldOffset(112)] public readonly float MagneticHeading { get; } // (112)

        [field: FieldOffset(116)] public readonly DataValidity Flags { get; } // (116)
        [field: FieldOffset(118)] public readonly ushort PackingAt118 { get; } // (118)
        [field: FieldOffset(120)] public readonly uint UnknownAt120 { get; } // (120)
        [field: FieldOffset(124)] public readonly uint Milliseconds { get; } // (124)

        public readonly long DataOffset => PositionOfFirstByte + (FrameType is FrameType.Extended ? ExtendedSize : BasicSize);
        #endregion Basic properties

        #region Type support
        // Derived:
        public readonly FrameType FrameType
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => SurveyType is SurveyType.Unknown7 or SurveyType.Unknown8 ? FrameType.Basic : FrameType.Extended;
        }
        #endregion Type support

        #region Extended Properties
        [FieldOffset(128)] private readonly uint lastPrimaryScanFrameOffset;
        public readonly uint LastPrimaryScanFrameOffset => FrameType is FrameType.Extended ? lastPrimaryScanFrameOffset : throw new InvalidFrameTypeException(); // (128)

        [FieldOffset(132)] private readonly uint lastSecondaryScanFrameOffset;
        public readonly uint LastSecondaryScanFrameOffset => FrameType is FrameType.Extended ? lastSecondaryScanFrameOffset : throw new InvalidFrameTypeException();  // (132)
        [FieldOffset(136)] public readonly uint lastDownScanFrameOffset;
        public readonly uint LastDownScanFrameOffset => FrameType is FrameType.Extended ? lastDownScanFrameOffset : throw new InvalidFrameTypeException();  // (136)
        [FieldOffset(140)] private readonly uint lastLeftSidescanFrameOffset;
        public readonly uint LastLeftSidescanFrameOffset => FrameType is FrameType.Extended ? lastLeftSidescanFrameOffset : throw new InvalidFrameTypeException();  // (140)
        [FieldOffset(144)] private readonly uint lastRightSidescanFrameOffset;
        public readonly uint LastRightSidescanFrameOffset => FrameType is FrameType.Extended ? lastRightSidescanFrameOffset : throw new InvalidFrameTypeException();  // (144)

        [FieldOffset(148)] private readonly uint lastSidescanFrameOffset;
        public readonly uint LastSidescanFrameOffset => FrameType is FrameType.Extended ? lastSidescanFrameOffset : throw new InvalidFrameTypeException();  // (148)
        [FieldOffset(152)] private readonly uint unknownAt152;
        public readonly uint UnknownAt152 => FrameType is FrameType.Extended ? unknownAt152 : throw new InvalidFrameTypeException();  // (152)

        [FieldOffset(156)] private readonly uint unknownAt156;
        public readonly uint UnknownAt156 => FrameType is FrameType.Extended ? unknownAt156 : throw new InvalidFrameTypeException();  // (156)

        [FieldOffset(160)] private readonly uint unknown160;
        public readonly uint UnknownAt160 => FrameType is FrameType.Extended ? unknown160 : throw new InvalidFrameTypeException();  // (160)

        [FieldOffset(164)] private readonly uint last3DFrameOffset;
        public readonly uint Last3DFrameOffset => FrameType is FrameType.Extended ? last3DFrameOffset : throw new InvalidFrameTypeException();  // (164)

        #endregion Extended properties

        #region DateTime support
        private static DateTime timestampBase;
        [SkipLocalsInit, MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void InitTimestampBase(uint hardwareTime) =>
                 timestampBase = DateTimeOffset.FromUnixTimeSeconds(hardwareTime)
                 .UtcDateTime;
        public readonly DateTime Timestamp => timestampBase.AddMilliseconds(Milliseconds);
        #endregion

        #region WGS84
        public double Longitude => X / PolarRadius * RadToDeg;
        public double Latitude => RadToDeg * double.Atan(double.Sinh(Y / PolarRadius));
        #endregion WGS84

        #region String generation
        public readonly override string ToString()
        {
            CultureInfo invariantCulture = Frame.invariantCulture;
            return string.Join(FieldSeparator,
        [
            CampaignID.ToString(),
            Timestamp.ToString(DateTimeFormat, invariantCulture),
            SurveyType.ToString(),
            WaterDepth.ToString("0.###", invariantCulture),
            Longitude.ToString("0.#######", invariantCulture),
            Latitude.ToString("0.#######", invariantCulture),
            GNSSAltitude.ToString("0.###", invariantCulture),
            GNSSHeading.ToString("0.###", invariantCulture),
            GNSSSpeed.ToString("0.####", invariantCulture),
            MagneticHeading.ToString("0.###",invariantCulture),
            MinRange.ToString("0.###", invariantCulture),
            MaxRange.ToString("0.###", invariantCulture),
            WaterTemperature.ToString("0.#",invariantCulture),
            WaterSpeed.ToString("0.###", invariantCulture),
            HardwareTime.ToString(),
            Frequency.ToString(),
            Milliseconds.ToString()]);
        }

        public readonly Span<char> Format(Span<char> buffer)
        {
            var invar = invariantCulture;
            CampaignID.TryFormat(buffer, out int pos, provider: invar);
            buffer[pos++] = ',';
            Timestamp.TryFormat(buffer[pos..], out int charWritten, DateTimeFormat, invar);
            pos += charWritten;
            buffer[pos++] = ',';
            Enum.TryFormat(SurveyType, buffer[pos..], out charWritten);
            pos += charWritten;
            buffer[pos++] = ',';
            WaterDepth.TryFormat(buffer[pos..], out charWritten, "0.###", provider: invar);
            pos += charWritten;
            buffer[pos++] = ',';
            Longitude.TryFormat(buffer[pos..], out charWritten, "0.000000", invar);
            pos += charWritten;
            buffer[pos++] = ',';
            Latitude.TryFormat(buffer[pos..], out charWritten, "0.0000000", invar);
            pos += charWritten;
            buffer[pos++] = ',';
            GNSSAltitude.TryFormat(buffer[pos..], out charWritten, "0.###", invar);
            pos += charWritten;
            buffer[pos++] = ',';
            GNSSHeading.TryFormat(buffer[pos..], out charWritten, "0.###", invar);
            pos += charWritten;
            buffer[pos++] = ',';
            GNSSSpeed.TryFormat(buffer[pos..], out charWritten, "0.###", invar);
            pos += charWritten;
            buffer[pos++] = ',';
            MagneticHeading.TryFormat(buffer[pos..], out charWritten, "0.###", invar);
            pos += charWritten;
            buffer[pos++] = ',';
            MinRange.TryFormat(buffer[pos..], out charWritten, "0.###", invar);
            pos += charWritten;
            buffer[pos++] = ',';
            MaxRange.TryFormat(buffer[pos..], out charWritten, "0.###", invar);
            pos += charWritten;
            buffer[pos++] = ',';
            WaterTemperature.TryFormat(buffer[pos..], out charWritten, "0.#", invar);
            pos += charWritten;
            buffer[pos++] = ',';
            WaterSpeed.TryFormat(buffer[pos..], out charWritten, "0.###", invar);
            pos += charWritten;
            buffer[pos++] = ',';
            HardwareTime.TryFormat(buffer[pos..], out charWritten, provider: invar);
            pos += charWritten;
            buffer[pos++] = ',';
            Enum.TryFormat(Frequency, buffer[pos..], out charWritten);
            pos += charWritten;
            buffer[pos++] = ',';
            Milliseconds.TryFormat(buffer[pos..], out charWritten, provider: invar);
            pos += charWritten;
            buffer[pos++] = ',';
            return buffer[..pos];
        }

        private readonly string GetDebuggerDisplay()
        {
            CultureInfo invariantCulture = Frame.invariantCulture;
            return string.Join(DebugFieldSeparator,
            [
                SurveyType.ToString(),
                WaterDepth.ToString(invariantCulture) + '′',
                MinRange.ToString(invariantCulture),
                MaxRange.ToString(invariantCulture),
                FrameType.ToString()
            ]);
        }
        #endregion String generation

        #region Unpack support
        public readonly (double x, double y, double z, double v, double t, double d) QueryMetric() =>
                   (X, Y,
                    FootToMeter * GNSSAltitude,
                    KnotsToMPS * GNSSSpeed,
                    MillisecondsToSeconds * Milliseconds,
                    Math.Tau - GNSSHeading + HalfPI);
        #endregion Unpack support

        #region Navigation

        #endregion End Navigation
    };
}