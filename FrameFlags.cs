using System.Runtime.InteropServices;

namespace SL3Reader
{
    [StructLayout(LayoutKind.Explicit, Size = Size)]
    internal readonly struct FrameFlags
    {
        public const int Size = 2;
        [FieldOffset(0)] private readonly ushort ByteStore;

        public bool IsTrackValid => (ByteStore & 0B1) != 0;
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
}