// Written by Ákos Halmai 2020.
// See LICENCE file for lic. terms.
// Univerity of Pécs, Faculty of Siences, Institute of Geography & Earthsciences
// http://foldrajz.ttk.pte.hu./

namespace LowranceReader2
{
       public enum LogFileFormats : short
    {
        SLG = 1,
        SL2, // Auto-increment
        SL3
    }

    [StructLayout(LayoutKind.Sequential)]
    [DebuggerDisplay("{ToString()}")]
    public readonly struct SLFileHeader
    {
        public const int Size = 8;

        private const LogFileFormats DesiredFormat = LogFileFormats.SL3;
        private const short DesiredBlockSize = 3200;

        [DefaultValue(DesiredFormat)] public LogFileFormats FileFormat { get; }
        public short DeviceID { get; }
        [DefaultValue(DesiredBlockSize)] public short BlockSize { get; }
        public short Unknown { get; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => FileFormat.ToString() + " (" + BlockSize.ToString() + ")";

        [SecuritySafeCritical()]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe SLFileHeader(byte[] buffer)
        {
            #region "Constants"
            const string NameOfBuffer = nameof(buffer);
            const string BufferIsNull = "The ‘" + NameOfBuffer + "’ argument can not be null.";
            const string WrongBufferSize = "The length of the ‘" + NameOfBuffer + "’ argument must be equal to or longer than 8.";
            #endregion "Constants"

            #region "Argument checks"
            if (buffer == null)
                throw new ArgumentNullException(NameOfBuffer, BufferIsNull);

            if (buffer.Length < Size)
                throw new ArgumentOutOfRangeException(NameOfBuffer, WrongBufferSize);
            #endregion "Argument checks" 

            #region "Unsafe cast of incoming buffer."
            fixed (byte* p = &buffer[0])
                this = *(SLFileHeader*)p;
            #endregion "Unsafe cast of incoming buffer."
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsValidFormat(
            LogFileFormats desirdFormat = DesiredFormat,
            short desiredBlockSize = DesiredBlockSize)
            =>
            FileFormat == desirdFormat && BlockSize == desiredBlockSize;
    }
}
