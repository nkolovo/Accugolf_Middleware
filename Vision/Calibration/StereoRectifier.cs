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
            var R0 = ArrayToMat(cal.R0, 3, 3);
            var P0 = ArrayToMat(cal.P0, 3, 4);

            var K1 = ArrayToMat(cal.K1, 3, 3);
            var D1 = ArrayToMat(cal.D1, 1, 5);
            var R1 = ArrayToMat(cal.R1, 3, 3);
            var P1 = ArrayToMat(cal.P1, 3, 4);

            _map0X = new Mat(); _map0Y = new Mat();
            _map1X = new Mat(); _map1Y = new Mat();

            // InitUndistortRectifyMap produces fast integer+fractional lookup tables
            // Emgu.CV 4.9: extra int parameter = number of channels for the output maps (1 = grayscale)
            CvInvoke.InitUndistortRectifyMap(K0, D0, R0, P0, imgSize, DepthType.Cv32F, 1, _map0X, _map0Y);
            CvInvoke.InitUndistortRectifyMap(K1, D1, R1, P1, imgSize, DepthType.Cv32F, 1, _map1X, _map1Y);

            IsReady = true;
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