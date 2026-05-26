// ------------------------------------------------------------
// Vision/Calibration/StereoCalibrator.cs
// ------------------------------------------------------------
// Run this once with a checkerboard visible to both cameras.
// Produces a calibration JSON you load at runtime.
// ------------------------------------------------------------
using System;
using System.Collections.Generic;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;

namespace SportSimulator.Vision.Calibration
{
    public class StereoCalibrator
    {
        private readonly int _cornersX;   // inner corners wide  (e.g. 9)
        private readonly int _cornersY;   // inner corners tall  (e.g. 6)
        private readonly float _squareMm; // physical square size in mm

        private readonly List<VectorOfPoint3D32F> _objPoints = new();
        private readonly List<VectorOfPointF>     _imgPts0   = new();
        private readonly List<VectorOfPointF>     _imgPts1   = new();

        public int FramePairsCollected => _objPoints.Count;

        public StereoCalibrator(int cornersX = 9, int cornersY = 6, float squareMm = 25f)
        {
            // ⚠️ SETUP TODO — update these to match your printed checkerboard:
            //   cornersX: number of *inner* corners along the wide edge
            //   cornersY: number of *inner* corners along the tall edge
            //   squareMm: physical size of each square in mm
            // A standard A3 print of a 9×6 board with 25mm squares works well.
            // Bigger squares = better results at longer distances.
            _cornersX = cornersX;
            _cornersY = cornersY;
            _squareMm = squareMm;
        }

        /// <summary>
        /// Feed a synchronised frame pair. Returns true if checkerboard found in both.
        /// </summary>
        public bool AddFramePair(byte[] left, byte[] right, int w, int h)
        {
            var matL = BytesToMat(left, w, h);
            var matR = BytesToMat(right, w, h);

            var cornersL = new VectorOfPointF();
            var cornersR = new VectorOfPointF();
            var size = new Size(_cornersX, _cornersY);

            bool foundL = CvInvoke.FindChessboardCorners(matL, size, cornersL);
            bool foundR = CvInvoke.FindChessboardCorners(matR, size, cornersR);

            if (!foundL || !foundR) return false;

            // Sub-pixel refinement
            var criteria = new MCvTermCriteria(30, 0.001);
            CvInvoke.CornerSubPix(matL, cornersL, new Size(11,11), new Size(-1,-1), criteria);
            CvInvoke.CornerSubPix(matR, cornersR, new Size(11,11), new Size(-1,-1), criteria);

            // Build object points (flat checkerboard in Z=0 plane)
            var obj = new Point3D[_cornersX * _cornersY];
            for (int r = 0; r < _cornersY; r++)
                for (int c = 0; c < _cornersX; c++)
                    obj[r * _cornersX + c] = new Point3D(c * _squareMm, r * _squareMm, 0);

            _objPoints.Add(new VectorOfPoint3D32F(Array.ConvertAll(obj,
                p => new MCvPoint3D32f((float)p.X, (float)p.Y, (float)p.Z))));
            _imgPts0.Add(cornersL);
            _imgPts1.Add(cornersR);

            Console.WriteLine($"[Calibrator] Pair {_objPoints.Count} accepted.");
            return true;
        }

        /// <summary>
        /// Run calibration after collecting ≥ 15 frame pairs.
        /// Returns RMS reprojection error (< 1.0 px is good).
        /// </summary>
        public double Calibrate(int w, int h, out StereoCalibrationData result)
        {
            if (_objPoints.Count < 15)
                throw new Exception($"Need ≥15 frame pairs, have {_objPoints.Count}.");

            var imgSize = new Size(w, h);
            var K0 = new Mat(); var D0 = new Mat();
            var K1 = new Mat(); var D1 = new Mat();
            var R  = new Mat(); var T  = new Mat();
            var E  = new Mat(); var F  = new Mat();

            // Stereo calibration — fixes intrinsics from individual calibration then refines
            double rms = CvInvoke.StereoCalibrate(
                _objPoints.ToArray(), _imgPts0.ToArray(), _imgPts1.ToArray(),
                K0, D0, K1, D1, imgSize,
                R, T, E, F,
                CalibType.FixIntrinsic,
                new MCvTermCriteria(100, 1e-5));

            Console.WriteLine($"[Calibrator] RMS reprojection error: {rms:F4} px");

            // Stereo rectification
            var R0 = new Mat(); var R1 = new Mat();
            var P0 = new Mat(); var P1 = new Mat();
            var Q  = new Mat();
            CvInvoke.StereoRectify(K0, D0, K1, D1, imgSize, R, T,
                R0, R1, P0, P1, Q,
                StereoRectifyType.Default, alpha: 0);

            result = new StereoCalibrationData
            {
                K0 = MatToArray(K0), D0 = MatToArray(D0),
                K1 = MatToArray(K1), D1 = MatToArray(D1),
                R  = MatToArray(R),  T  = MatToArray(T),
                R0 = MatToArray(R0), R1 = MatToArray(R1),
                P0 = MatToArray(P0), P1 = MatToArray(P1),
                Q  = MatToArray(Q),
                ImageWidth = w, ImageHeight = h
            };
            return rms;
        }

        private Mat BytesToMat(byte[] data, int w, int h)
        {
            var m = new Mat(h, w, DepthType.Cv8U, 1);
            m.SetTo(data);
            return m;
        }

        private double[] MatToArray(Mat m)
        {
            var arr = new double[m.Rows * m.Cols * m.NumberOfChannels];
            m.CopyTo(arr);
            return arr;
        }

        private record Point3D(double X, double Y, double Z);
    }
}