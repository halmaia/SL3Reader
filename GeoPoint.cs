using System.Globalization;

namespace SL3Reader
{
    public readonly struct GeoPoint
    {
        public GeoPoint(double x, double y)
        {
            X = x;
            Y = y;
        }

        public readonly double X { get; }
        public readonly double Y { get; }

        public override readonly string ToString()
        {
            CultureInfo provider = CultureInfo.InvariantCulture;
            return X.ToString(provider) + ',' +
                   Y.ToString(provider);
        }
    }
}