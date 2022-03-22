namespace SL3Reader
{
    internal enum SurveyType : ushort
    {
        Primary, Secondary, DownScan,
        LeftSidescan, RightSidescan,
        SideScan, Unknown6, Unknown7,
        Unknown8, ThreeDimensional,
        DebugDigital, DebugNoise
    }
}