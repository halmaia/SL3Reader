using System;
using System.IO;

namespace SL3Reader
{
    public static class Program
    {
        public static unsafe void Main(string[] args)
        {
            string input, output;
            if (args.Length != 2 || !File.Exists(input = Path.GetFullPath(args[0])))
            {
                Console.WriteLine(@"Usage example: SL3Reader.exe C:\input.sl3 D:\output.csv");
                return;
            }
            using SL3Reader sl3reader = new(input);
            // Experiment:
            //sl3reader.ExportSideScans(@"F:\y.bmp");

            // Experiment:
//            System.Collections.Generic.IReadOnlyList<Frame> frames = sl3reader.Frames;
//            Frame frame = frames[31];
//            sl3reader.Seek(Frame.Size + frame.PositionOfFirstByte, SeekOrigin.Begin);
//            var remainingLen = frame.TotalLength - frame.OriginalLengthOfEchoData;

//            var _3DBytes = new byte[ThreeDimensionalFrameHeader.Size];
//fixed( byte* p = _3DBytes)
//            {
//                var spn = new Span<byte>(p, ThreeDimensionalFrameHeader.Size);
//                sl3reader.Read(spn);
//                var header = *(ThreeDimensionalFrameHeader*)p;

//            }

            //
            sl3reader.ExportToCSV(output = Path.GetFullPath(args[1]), false);
        }
    }
}