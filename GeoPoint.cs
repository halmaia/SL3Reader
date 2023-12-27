using System;
using System.Globalization;

namespace SL3Reader
{
    public readonly struct GeoPoint(double x, double y, double heading, double altitude, double distance)
    {
        public readonly double X { get; } = x;
        public readonly double Y { get; } = y;
        public readonly double Altitude { get; } = altitude;
        public readonly double Heading { get; } = heading;
        public readonly double Distance { get; } = distance;

        public override readonly string ToString()
        {
            CultureInfo invariantCulture = CultureInfo.InvariantCulture;
            return X.ToString("0.###", invariantCulture) + ',' +
                   Y.ToString("0.###", invariantCulture);
        }
        private static readonly CultureInfo invariantCulture = CultureInfo.InvariantCulture;
        public readonly Span<char> Format(Span<char> buffer)
        {
            var invar = invariantCulture;
            X.TryFormat(buffer, out int pos, "0.###", invar);
            buffer[pos++] = ',';
            Y.TryFormat(buffer[pos..], out int charWritten, "0.###", invar);
            return buffer[..(pos + charWritten)];
        }
    }
}