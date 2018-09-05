// Written by Ákos Halmai ® 2018.
// Univerity of Pécs, Faculty of Siences, Institute of Geography & Earthsciences
// http://foldrajz.ttk.pte.hu./
namespace LowranceReader2
{
    /// <summary>
    /// File format enum for SLG, SL2 &amp; SL3 files.
    /// This is the first <see cref="short"/> field in sonar log files.
    /// </summary> 
    public enum LogFileFormats : short
    {
        /// <summary>
        /// Standard sonar log format for Lowrance® &amp; Simrad® (Navico Inc.) devices.
        /// It supports only primary &amp; secondary sonar readings. Sidescan &amp; 3D is not supported.
        /// The extension is “*.SLG”. Little Endian.
        /// </summary>
        /// <remarks>Check in “HDS Gen3 Operator Manual - Lowrance”
        /// https://ww2.lowrance.com/Root/Lowrance-Documents/HDSGen3/HDS-GEN3_OM_EN_988-10740-005_w.pdf
        /// (09/05/2013)
        /// </remarks>
        SLG = 1,
        /// <summary>
        /// Sonar log format for primary, secondary &amp; StructureScan® (sidescan &amp; DownScan™) readings.
        /// 3D is not supported.
        /// The extension is “*.SL2”. Little Endian.
        /// </summary>
        ///  <remarks>Check in “HDS Gen3 Operator Manual - Lowrance”
        /// https://ww2.lowrance.com/Root/Lowrance-Documents/HDSGen3/HDS-GEN3_OM_EN_988-10740-005_w.pdf
        /// (09/05/2013)
        /// </remarks>
        SL2, // No need to define value: auto-incremented by C♯™ 7.3. (SL2 = 2)
        /// Sonar log format for primary, secondary &amp; StructureScan® 3D readings.
        /// The extension is “*.SL3”. Little Endian.
        /// </summary>
        ///  <remarks>Check in “HDS Gen3 Operator Manual - Lowrance”
        /// https://ww2.lowrance.com/Root/Lowrance-Documents/HDSGen3/HDS-GEN3_OM_EN_988-10740-005_w.pdf
        /// (09/05/2013)
        /// </remarks>
        SL3 // No need to define value: auto-incremented by C♯™ 7.3. (SL3 = 3)
    }
}
