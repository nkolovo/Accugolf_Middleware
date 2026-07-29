// ------------------------------------------------------------
// Vision/Calibration/StereoRectifier.cs
// ------------------------------------------------------------
// Precomputes undistort+rectify maps at startup so per-frame
// remapping is just a fast table lookup.
// ------------------------------------------------------------
using System.Drawing;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;

namespace SportSimulator.Vision.Calibration
{
    public class StereoRectifier
    {
        private Mat? _map0X, _map0Y; // remap tables for camera 0
        private Mat? _map1X, _map1Y; // remap tables for camera 1

        public bool IsReady { get; private set; }

        public void Build(StereoCalibrationData cal)
        {
            var imgSize = new Size(cal.ImageWidth, cal.ImageHeight);

            var K0 = ArrayToMat(cal.K0, 3, 3);
            var D0 = ArrayToMat(cal.D0, 1, 5);
            var K1 = ArrayToMat(cal.K1, 3, 3);
            var D1 = ArrayToMat(cal.D1, 1, 5);
            var R  = ArrayToMat(cal.R,  3, 3);
            var T  = ArrayToMat(cal.T,  3, 1);

            // cal.R0/R1/P0/P1/Q are rectification OUTPUTS, not calibrated inputs (see
            // StereoCalibrationData class comment) — StereoCalibrator.Calibrate() fills
            // them via CvInvoke.StereoRectify for a real checkerboard calibration, but
            // that same step was missing here for the CreateDefaults() fallback path.
            // Without it, cal.R0/P0/etc. were still their zero-initialized defaults,
            // which InitUndistortRectifyMap silently accepted and turned into garbage
            // remap tables (and Triangulator, reading the same still-zero P0/P1,
            // computed fx=0 / baseline=NaN) — nothing was ever actually rectified.
            var R0 = new Mat(); var R1 = new Mat();
            var P0 = new Mat(); var P1 = new Mat();
            var Q  = new Mat();
            var roi0 = new Rectangle();
            var roi1 = new Rectangle();
            CvInvoke.StereoRectify(K0, D0, K1, D1, imgSize, R, T,
                R0, R1, P0, P1, Q,
                StereoRectifyType.Default, 0.0, imgSize,
                ref roi0, ref roi1);

            cal.R0 = MatToArray(R0); cal.R1 = MatToArray(R1);
            cal.P0 = MatToArray(P0); cal.P1 = MatToArray(P1);
            cal.Q  = MatToArray(Q);

            _map0X = new Mat(); _map0Y = new Mat();
            _map1X = new Mat(); _map1Y = new Mat();

            // InitUndistortRectifyMap produces fast integer+fractional lookup tables
            // Emgu.CV 4.9: extra int parameter = number of channels for the output maps (1 = grayscale)
            CvInvoke.InitUndistortRectifyMap(K0, D0, R0, P0, imgSize, DepthType.Cv32F, 1, _map0X, _map0Y);
            CvInvoke.InitUndistortRectifyMap(K1, D1, R1, P1, imgSize, DepthType.Cv32F, 1, _map1X, _map1Y);

            IsReady = true;
        }

        private double[] MatToArray(Mat m)
        {
            var arr = new double[m.Rows * m.Cols * m.NumberOfChannels];
            m.CopyTo(arr);
            return arr;
        }

        public Mat Rectify(Mat src, int camIndex)
        {
            var dst = new Mat();
            var (mx, my) = camIndex == 0 ? (_map0X!, _map0Y!) : (_map1X!, _map1Y!);
            CvInvoke.Remap(src, dst, mx, my, Inter.Linear);
            return dst;
        }

        private Mat ArrayToMat(double[] arr, int rows, int cols)
        {
            var m = new Mat(rows, cols, DepthType.Cv64F, 1);
            m.SetTo(arr);
            return m;
        }
    }
}