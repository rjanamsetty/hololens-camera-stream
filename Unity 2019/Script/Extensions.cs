using System.Collections.Generic;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ImgprocModule;

namespace Script
{
    public static class Extensions
    {
        public static T[] SubArray<T>(this T[] array, int offset, int length)
        {
            return new List<T>(array)
                .GetRange(offset, length)
                .ToArray();
        }

        public static T[] JoinArray<T>(this T[] array, T[] toCombine)
        {
            var combined = new T[array.Length + toCombine.Length];
            array.CopyTo(combined, 0);
            toCombine.CopyTo(combined, array.Length);
            return combined;
        }

        public static Point InvalidPoint() => new Point(-1, -1);
        
        public static Mat BGRAtoGrayscale(Mat bgraMat)
        {
            var gray = new Mat();
            Imgproc.cvtColor(bgraMat, gray, Imgproc.COLOR_BGRA2GRAY);
            return gray;
        }

    }
}