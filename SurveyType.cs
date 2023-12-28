using System;

namespace SL3Reader
{
    public enum SurveyType : ushort
    {
        Primary, Secondary, DownScan,
        LeftSideScan, RightSideScan,
        SideScan, Unknown6, Unknown7,
        Unknown8, ThreeDimensional,
        DebugDigital, DebugNoise,
        All = ushort.MaxValue // For filtering purposes only
    }
    public static class SurveyTypeTranslator
    {
        public static ReadOnlySpan<byte> ToSpan(SurveyType surveyType) => surveyType switch
        {
            SurveyType.Primary => "Primary"u8,
            SurveyType.Secondary => "Secondary"u8,
            SurveyType.DownScan => "DownScan"u8,
            SurveyType.LeftSideScan => "LeftSideScan"u8,
            SurveyType.RightSideScan => "RightSideScan"u8,
            SurveyType.SideScan => "SideScan"u8,
            SurveyType.Unknown6 => "Unknown6"u8,
            SurveyType.Unknown7 => "Unknown7"u8,
            SurveyType.Unknown8 => "Unknown8"u8,
            SurveyType.ThreeDimensional => "ThreeDimensional"u8,
            SurveyType.DebugDigital => "DebugDigital"u8,
            SurveyType.DebugNoise => "DebugNoise"u8,
            SurveyType.All => "All"u8,
            _ => throw new NotImplementedException(),
        };
    }
}