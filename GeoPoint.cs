using System.Globalization;

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
        public double Altitude { get; }
        public readonly double Heading { get; }
        public readonly double Distance { get; }


        public override readonly string ToString()
        {
            CultureInfo provider = CultureInfo.InvariantCulture;
            return X.ToString(provider) + ',' +
                   Y.ToString(provider);
        }
    }
}