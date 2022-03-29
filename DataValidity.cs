using System;

namespace SL3Reader
{
    [Flags]
    internal enum DataValidity : ushort
    {
        TrackValid = 1,
        UnknownAt1 = 2,
        UnknownAt2 = 4,
        PositionValid = 8,
        UnknownAt4 = 16,
        CourseOrSpeed = 32,
        SpeedValid = 64,
        UnknownAt7 = 128,
        UnknownAt8 = 256,
        AltitudeOrCourseOrSpeed = 512,
        UnknownAt10 = 1024,
        UnknownAt11 = 2048,
        UnknownAt12 = 4096,
        UnknownAt13 = 8192,
        AltitudeValid = 16384,
        HeadingValid = 32768
    }
}