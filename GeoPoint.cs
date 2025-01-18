using System;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace SL3Reader;

public readonly struct GeoPoint(double longitude, double lattitude, double heading, double altitude, double distance)
{
    public readonly double Longitude { get; } = longitude;
    public readonly double Lattitude { get; } = lattitude;
    public readonly double Altitude { get; } = altitude;
    public readonly double Heading { get; } = heading;
    public readonly double Distance { get; } = distance;

    public readonly double X => double.DegreesToRadians(Longitude) * 6356752.3142d;
    public readonly double Y => double.Asinh(double.Tan(double.DegreesToRadians(Lattitude))) * 6356752.3142d;

    public override readonly string ToString()
    {
        CultureInfo invariantCulture = CultureInfo.InvariantCulture;
        return X.ToString("0.###", invariantCulture) + ',' +
               Y.ToString("0.###", invariantCulture);
    }
    [SkipLocalsInit]
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