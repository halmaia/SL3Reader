using System;
using System.Globalization;
using System.Runtime.Serialization;

namespace SL3Reader
{
    public readonly struct GeoPoint
    {
        public GeoPoint(double x, double y, double heading, double altitude, double distance)
        {
            X = x;
            Y = y;
            Altitude = altitude;
            Heading = heading;
            Distance = distance;
        }

        public readonly double X { get; }
        public readonly double Y { get; }
        public readonly double Altitude { get; }
        public readonly double Heading { get; }

        public readonly double Distance { get; }

        public override readonly string ToString()
        {
            CultureInfo invariantCulture = CultureInfo.InvariantCulture;
            return X.ToString("0.###", invariantCulture) + ',' +
                   Y.ToString("0.###", invariantCulture);
        }
        private static readonly CultureInfo invariantCulture = CultureInfo.InvariantCulture;
        public readonly Span<char> TryFormat(Span<char> buffer)
        {
            var invar = invariantCulture;
            X.TryFormat(buffer, out int pos, "0.###", invar);
            buffer[pos++] = ',';
            Y.TryFormat(buffer[pos..], out int charWritten, "0.###", invar);
            return buffer[..(pos + charWritten)];
        }
    }
}