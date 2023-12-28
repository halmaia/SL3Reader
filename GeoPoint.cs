using System;
using System.Globalization;
using System.Text;

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
        public readonly ReadOnlySpan<byte> Format(Span<byte> buffer, bool addNewLine = true)
        {
            CultureInfo invariantCulture = CultureInfo.InvariantCulture;
            ReadOnlySpan<char> format = "0.###";
            X.TryFormat(buffer, out int pos, format, invariantCulture);
            buffer[pos++] = 44; // ','u8; 
            Y.TryFormat(buffer[pos..], out int charWritten, format, invariantCulture);
            pos += charWritten;

            if (addNewLine)
            {
                "\r\n"u8.CopyTo(buffer[pos..]);
                pos += 2;
            }
            return buffer[..pos];
        }
    }
}