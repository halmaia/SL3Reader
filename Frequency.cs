using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace SL3Reader
{
    public enum Frequency : ushort
    {
        Fq_200kHz, Fq_50kHz, Fq_83kHz,
        Fq_455kHz, Fq_800kHz, Fq_38kHz, Fq_28kHz,
        Fq_130kHz_210kHz, Fq_90kHz_150kHz, Fq_40kHz_60kHz, Fq_25kHz_45kHz
    }
    public static class FrequencyTranslator
    {
        [SkipLocalsInit]
        public static ReadOnlySpan<byte> ToSpan(Frequency frequency) => frequency switch
        {
            Frequency.Fq_200kHz => "Fq_200kHz"u8,
            Frequency.Fq_50kHz => "Fq_50kHz"u8,
            Frequency.Fq_83kHz => "Fq_83kHz"u8,
            Frequency.Fq_455kHz => "Fq_455kHz"u8,
            Frequency.Fq_800kHz => "Fq_800kHz"u8,
            Frequency.Fq_38kHz => "Fq_38kHz"u8,
            Frequency.Fq_28kHz => "Fq_28kHz"u8,
            Frequency.Fq_130kHz_210kHz => "Fq_130kHz_210kHz"u8,
            Frequency.Fq_90kHz_150kHz => "Fq_90kHz_150kHz"u8,
            Frequency.Fq_40kHz_60kHz => "Fq_40kHz_60kHz"u8,
            Frequency.Fq_25kHz_45kHz => "Fq_25kHz_45kHz"u8,
            _ => throw new NotImplementedException()
        };
    }
}