using System;
using System.IO;
using System.Buffers;

namespace SL3Reader
{
    public static class GeoReferenceHelper
    {
        public static void WriteGeoreferencedPAM(string path, Span<GeoPoint> sourceGCPs, Span<GeoPoint> targetGCPs)
        {
            using FileStream file = File.OpenWrite(path);
            file.Write("""
                <PAMDataset>
                  <Metadata>
                    <MDI key="DataType">Processed</MDI>
                    <MDI key="SensorName">Lowrance StructureScan3D</MDI>
                  </Metadata>
                  <Metadata domain="xml:ESRI" format="xml">
                    <GeodataXform xsi:type="typens:CompositeXform" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xs="http://www.w3.org/2001/XMLSchema" xmlns:typens="http://www.esri.com/schemas/ArcGIS/3.2.0">
                      <XF_0 xsi:type="typens:PolynomialXform">
                        <PolynomialOrder>2</PolynomialOrder>
                        <SpatialReference xsi:type="typens:ProjectedCoordinateSystem">
                          <WKT>PROJCS["Lowrance_Mercator",GEOGCS["Lowrance_Sphere",DATUM["D_Lowrance_Sphere",SPHEROID["Lowrance_Sphere",6356752.31424518,0.0]],PRIMEM["Greenwich",0.0],UNIT["Degree",0.0174532925199433]],PROJECTION["Mercator"],PARAMETER["False_Easting",0.0],PARAMETER["False_Northing",0.0],PARAMETER["Central_Meridian",0.0],PARAMETER["Standard_Parallel_1",0.0],UNIT["Meter",1.0]]</WKT>
                          <XOrigin>-19970500</XOrigin>
                          <YOrigin>-30139700</YOrigin>
                          <XYScale>10000</XYScale>
                          <ZOrigin>-100000</ZOrigin>
                          <ZScale>10000</ZScale>
                          <MOrigin>-100000</MOrigin>
                          <MScale>10000</MScale>
                          <XYTolerance>0.001</XYTolerance>
                          <ZTolerance>0.001</ZTolerance>
                          <MTolerance>0.001</MTolerance>
                          <HighPrecision>true</HighPrecision>
                        </SpatialReference>
                        <SourceGCPs xsi:type="typens:ArrayOfDouble">
                """u8);
            ReadOnlySpan<byte> doubleOpen = "          <Double>"u8;
            ReadOnlySpan<byte> doubleClose = "</Double>\n"u8;
            byte[] buffer = ArrayPool<byte>.Shared.Rent(128);
            Span<byte> utf8Destination = buffer;

            for (int i = 0; i < sourceGCPs.Length; i++)
            {
                file.Write(doubleOpen);
                GeoPoint current = sourceGCPs[0];
                current.X.TryFormat(utf8Destination, out int bytesWritten);
                file.Write(utf8Destination[..bytesWritten]);
                file.Write(doubleClose);
                file.Write(doubleOpen);
                current.Y.TryFormat(utf8Destination, out bytesWritten);
                file.Write(utf8Destination[..bytesWritten]);
                file.Write(doubleClose);
            }

            file.Write("""
                        </SourceGCPs>
                        <TargetGCPs xsi:type="typens:ArrayOfDouble">
                """u8);

            for (int i = 0; i < targetGCPs.Length; i++)
            {
                file.Write(doubleOpen);
                GeoPoint current = targetGCPs[0];
                current.X.TryFormat(utf8Destination, out int bytesWritten);
                file.Write(utf8Destination[..bytesWritten]);
                file.Write(doubleClose);
                file.Write(doubleOpen);
                current.Y.TryFormat(utf8Destination, out bytesWritten);
                file.Write(utf8Destination[..bytesWritten]);
                file.Write(doubleClose);
            }

            ArrayPool<byte>.Shared.Return(buffer);

            file.Write("""
                        </TargetGCPs>
                        <Name />
                      </XF_0>
                      <RequireDEM>false</RequireDEM>
                    </GeodataXform>
                  </Metadata>
                  <PAMRasterBand band="1">
                    <Histograms>
                      <HistItem>
                        <HistMin>-0.5</HistMin>
                        <HistMax>168.5</HistMax>
                        <BucketCount>169</BucketCount>
                        <IncludeOutOfRange>1</IncludeOutOfRange>
                        <Approximate>1</Approximate>
                        <HistCounts>194107|13838|0|0|0|0|0|0|0|0|13204|0|0|0|0|0|0|0|0|0|12328|0|0|0|11492|0|0|10383|0|0|0|9912|0|0|4621|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|4391|0|0|0|8479|0|0|7695|0|0|0|7038|0|0|6490|5879|5554|5039|4656|4477|4055|3763|3511|3250|3025|2814|2592|2443|2320|2267|1980|1861|3424|3258|2877|2640|2440|2289|2162|2879|1758|1709|1500|1480|1446|1320|2954|2051|1884|1743|1506|1425|1265|1123|1017|909|838|775|729|641|1114|938|852|731|632|547|495|441|427|364|363|317|306|292|255|361|509|315|314|314|360|252|241|204|293|178|206|185|262|294|175|271|217|139|212|182|126|155|160|80|124|1993|85|145|208|157|184|190|129|170|104|84|117|157|381|334|658|1195</HistCounts>
                      </HistItem>
                    </Histograms>
                    <Metadata>
                      <MDI key="STATISTICS_COVARIANCES">1748.548089737012</MDI>
                      <MDI key="STATISTICS_MAXIMUM">255</MDI>
                      <MDI key="STATISTICS_MEAN">128</MDI>
                      <MDI key="STATISTICS_MEDIAN">128</MDI>
                      <MDI key="STATISTICS_MINIMUM">0</MDI>
                      <MDI key="STATISTICS_SKIPFACTORX">1</MDI>
                      <MDI key="STATISTICS_SKIPFACTORY">1</MDI>
                      <MDI key="STATISTICS_STDDEV">16</MDI>
                    </Metadata>
                  </PAMRasterBand>
                </PAMDataset>
                """u8);
        }
    }
}
