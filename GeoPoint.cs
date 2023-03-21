using System.Globalization;

namespace SL3Reader
{
    public readonly struct GeoPoint
    {
        public GeoPoint(double x, double y, double heading, double altitude)
        {
            X = x;
            Y = y;
            Altitude = altitude;
            Heading = heading;
        }

        public readonly double X { get; }
        public readonly double Y { get; }
        public readonly double Altitude { get; }
        public readonly double Heading { get; }

        public override readonly string ToString()
        {
            CultureInfo invariantCulture = CultureInfo.InvariantCulture;
            return X.ToString("0.####", invariantCulture) + ',' +
                   Y.ToString("0.####", invariantCulture);
        }
    }
}