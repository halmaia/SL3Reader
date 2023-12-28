using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;
using System.Globalization;
using static System.Globalization.CultureInfo;

namespace SL3Reader
{
    // We might need a refactoring. Seemingly it is not a single frame, but one long header,
    // and extra information panel and the data itself.
    [StructLayout(LayoutKind.Explicit, Size = ExtendedSizeV13)]
    [DebuggerDisplay($"{{{nameof(GetDebuggerDisplay)}(),nq}}")]
    public readonly struct Frame
    {
        public const int BasicSize = 128;
        public const int ExtendedSizeV10 = 168;
        public const int ExtendedSizeV13 = 180;

        #region Basic Properties // BasicSize = 128
        [field: FieldOffset(0)] public readonly uint PositionOfFirstByte { get; } // (0)
        [field: FieldOffset(4)] public readonly FrameVersion Version { get; } // (4) 
        [field: FieldOffset(8)] public readonly ushort LengthOfFrame { get; } // (8)
        [field: FieldOffset(10)] public readonly ushort LengthOfPreviousFrame { get; } // (10)
        [field: FieldOffset(12)] public readonly SurveyType SurveyType { get; } // (12)
        [field: FieldOffset(14)] public readonly short UnknownAt14 { get; } // (14) Always 0 for me.
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
        [field: FieldOffset(118)] public readonly ushort UnknownAt118 { get; } // (118)
        [field: FieldOffset(120)] public readonly uint UnknownAt120 { get; } // (120)
        [field: FieldOffset(124)] public readonly uint Milliseconds { get; } // (124)
        #endregion Basic properties

        #region Extended Properties for V10
        [ContextStatic] private readonly static InvalidFrameTypeException invalidFrameTypeException = new();

        [FieldOffset(128)] private readonly uint lastPrimaryScanFrameOffset;
        public readonly uint LastPrimaryScanFrameOffset => FrameType is FrameType.Extended ? lastPrimaryScanFrameOffset : throw invalidFrameTypeException; // (128)
        [FieldOffset(132)] private readonly uint lastSecondaryScanFrameOffset;
        public readonly uint LastSecondaryScanFrameOffset => FrameType is FrameType.Extended ? lastSecondaryScanFrameOffset : throw invalidFrameTypeException;  // (132)
        [FieldOffset(136)] public readonly uint lastDownScanFrameOffset;
        public readonly uint LastDownScanFrameOffset => FrameType is FrameType.Extended ? lastDownScanFrameOffset : throw invalidFrameTypeException;  // (136)
        [FieldOffset(140)] private readonly uint lastLeftSidescanFrameOffset;
        public readonly uint LastLeftSidescanFrameOffset => FrameType is FrameType.Extended ? lastLeftSidescanFrameOffset : throw invalidFrameTypeException;  // (140)
        [FieldOffset(144)] private readonly uint lastRightSidescanFrameOffset;
        public readonly uint LastRightSidescanFrameOffset => FrameType is FrameType.Extended ? lastRightSidescanFrameOffset : throw invalidFrameTypeException;  // (144)

        [FieldOffset(148)] private readonly uint lastSidescanFrameOffset;
        public readonly uint LastSidescanFrameOffset => FrameType is FrameType.Extended ? lastSidescanFrameOffset : throw invalidFrameTypeException;  // (148)
        [FieldOffset(152)] private readonly uint unknownAt152;
        public readonly uint UnknownAt152 => FrameType is FrameType.Extended ? unknownAt152 : throw invalidFrameTypeException;  // (152)

        [FieldOffset(156)] private readonly uint unknownAt156;
        public readonly uint UnknownAt156 => FrameType is FrameType.Extended ? unknownAt156 : throw invalidFrameTypeException;  // (156)

        [FieldOffset(160)] private readonly uint unknown160;
        public readonly uint UnknownAt160 => FrameType is FrameType.Extended ? unknown160 : throw invalidFrameTypeException;  // (160)

        [FieldOffset(164)] private readonly uint last3DFrameOffset;
        public readonly uint Last3DFrameOffset => FrameType is FrameType.Extended ? last3DFrameOffset : throw invalidFrameTypeException;  // (164)
        #endregion Extended properties for V10

        #region Extended properties for V13
        private readonly bool IsV13Extended
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining), SkipLocalsInit]
            get => Version is FrameVersion.V13 &&
                   FrameType is FrameType.Extended;
        }

        [FieldOffset(168)] private readonly uint unknownAt168;
        public readonly uint UnknownAt168 => IsV13Extended ? unknownAt168 : throw invalidFrameTypeException;  // (168)
        [FieldOffset(172)] private readonly uint unknownAt172;
        public readonly uint UnknownAt172 => IsV13Extended ? unknownAt172 : throw invalidFrameTypeException;  // (172)
        [FieldOffset(176)] private readonly uint unknownAt176;
        public readonly uint UnknownAt176 => IsV13Extended ? unknownAt176 : throw invalidFrameTypeException;  // (176)
        #endregion Extended properties for V13

        #region Derived properties
        public readonly FrameType FrameType
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining), SkipLocalsInit]
            get => SurveyType is SurveyType.Unknown7 or SurveyType.Unknown8 ? FrameType.Basic : FrameType.Extended;
        }

        /// <summary>
        /// Provides the data offset relatively to the begining of the frame.
        /// </summary>
        public readonly int HeaderSize
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining), SkipLocalsInit]
            get
            {
                return FrameType is FrameType.Extended
                    ? Version switch
                    {
                        FrameVersion.V10 => ExtendedSizeV10,
                        FrameVersion.V13 => ExtendedSizeV13,
                        _ => UnsupportedFrame()

                    }
                    : BasicSize;

                static int UnsupportedFrame() => throw new NotImplementedException("Unsupported frame version. Contact developer at GitHub!");
            }
        }

        #endregion Derived properties

        #region DateTime support
        private static DateTime timestampBase;
        [SkipLocalsInit, MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void InitTimestampBase(uint hardwareTime) =>
                 timestampBase = DateTimeOffset.FromUnixTimeSeconds(hardwareTime)
                 .UtcDateTime;
        public readonly DateTime Timestamp => timestampBase.AddMilliseconds(Milliseconds);
        #endregion

        #region WGS84
        
        private const double PolarRadius = 6356752.3142d; // Lowrance uses float, causing truncated precision.
        public readonly double Longitude => double.RadiansToDegrees( X / PolarRadius);
        public readonly double Latitude => double.RadiansToDegrees( double.Atan(double.Sinh(Y / PolarRadius)));
        #endregion WGS84

        #region String generation
        private const char FieldSeparator = ',';
        private const string DebugFieldSeparator = "; ";
        private const string DateTimeFormat = "yyyy'-'MM'-'dd HH':'mm':'ss.fff'Z'";

        private static readonly CultureInfo invariantCulture = InvariantCulture;
        public readonly override string ToString()
        {
            CultureInfo invariantCulture = Frame.invariantCulture;
            return string.Join(FieldSeparator,
        [
            CampaignID.ToString(),
            Timestamp.ToString(DateTimeFormat, invariantCulture),
            SurveyType.ToString(),
            WaterDepth.ToString("0.###", invariantCulture),
            Longitude.ToString("0.000000", invariantCulture),
            Latitude.ToString("0.000000", invariantCulture),
            GNSSAltitude.ToString("0.###", invariantCulture),
            GNSSHeading.ToString("0.###", invariantCulture),
            GNSSSpeed.ToString("0.###", invariantCulture),
            MagneticHeading.ToString("0.###",invariantCulture),
            MinRange.ToString("0.###", invariantCulture),
            MaxRange.ToString("0.###", invariantCulture),
            WaterTemperature.ToString("0.#",invariantCulture),
            WaterSpeed.ToString("0.###", invariantCulture),
            HardwareTime.ToString(),
            Frequency.ToString(),
            Milliseconds.ToString()]);
        }

        [SkipLocalsInit]
        public readonly ReadOnlySpan<byte> Format(Span<byte> buffer)
        {
            buffer.Fill(44);//','u8;
            ReadOnlySpan<char> Unit3 = "0.###";
            CultureInfo invariantCulture = Frame.invariantCulture;
            CampaignID.TryFormat(buffer, out int pos, provider: invariantCulture);
            pos++; // Next step: leave the 44 there.
            Timestamp.TryFormat(buffer[pos..], out int charWritten, DateTimeFormat, invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            ReadOnlySpan<byte> surveyType = SurveyTypeTranslator.ToSpan(SurveyType);
            surveyType.CopyTo(buffer[pos..]);
            pos += surveyType.Length;
            pos++; // Next step: leave the 44 there.
            WaterDepth.TryFormat(buffer[pos..], out charWritten, Unit3, invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            Longitude.TryFormat(buffer[pos..], out charWritten, "0.000000", invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            Latitude.TryFormat(buffer[pos..], out charWritten, "0.0000000", invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            GNSSAltitude.TryFormat(buffer[pos..], out charWritten, Unit3, invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            GNSSHeading.TryFormat(buffer[pos..], out charWritten, Unit3, invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            GNSSSpeed.TryFormat(buffer[pos..], out charWritten, Unit3, invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            MagneticHeading.TryFormat(buffer[pos..], out charWritten, Unit3, invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            MinRange.TryFormat(buffer[pos..], out charWritten, Unit3, invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            MaxRange.TryFormat(buffer[pos..], out charWritten, Unit3, invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            WaterTemperature.TryFormat(buffer[pos..], out charWritten, "0.#", invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            WaterSpeed.TryFormat(buffer[pos..], out charWritten, Unit3, invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            HardwareTime.TryFormat(buffer[pos..], out charWritten, provider: invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            ReadOnlySpan<byte> frequency = FrequencyTranslator.ToSpan(Frequency);
            frequency.CopyTo(buffer[pos..]);
            pos += frequency.Length;
            pos++; // Next step: leave the 44 there.
            Milliseconds.TryFormat(buffer[pos..], out charWritten, provider: invariantCulture);
            pos += charWritten;
            pos++; // Next step: leave the 44 there.
            return buffer[..pos];
        }

        [SkipLocalsInit]
        private readonly string GetDebuggerDisplay()
        {
            CultureInfo invariantCulture = Frame.invariantCulture;
            return string.Join(DebugFieldSeparator,
            [
                SurveyType.ToString(),
                WaterDepth.ToString(invariantCulture) + '′',
                MinRange.ToString(invariantCulture),
                MaxRange.ToString(invariantCulture),
                FrameType.ToString(),
                Version.ToString()
            ]);
        }
        #endregion String generation

        #region Coordinate augmentation support
        private const double KnotsToMpS = 1852d / 3600d;
        private const double HalfPI = double.Pi / 2d;
        private const double MillisecondsToSeconds = 1d / 1000d;
        private const double FootToMeter = 0.3048d;
        public readonly (double x, double y, double z, double v, double t, double d) QueryMetric() =>
                   (X, Y,
                    FootToMeter * GNSSAltitude,
                    KnotsToMpS * GNSSSpeed,
                    MillisecondsToSeconds * Milliseconds,
                    Math.Tau - GNSSHeading + HalfPI);
        #endregion Coordinate augmentation support
    }
};