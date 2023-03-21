namespace SL3Reader
{
    public enum SurveyType : ushort
    {
        Primary, Secondary, DownScan,
        LeftSidescan, RightSidescan,
        SideScan, Unknown6, Unknown7,
        Unknown8, ThreeDimensional,
        DebugDigital, DebugNoise,
        All = ushort.MaxValue // For filtering purposes only
    }
}