// ------------------------------------------------------------
// Vision/Calibration/StereoCalibrator.cs
// ------------------------------------------------------------
// Run this once with a checkerboard visible to both cameras.
// Produces a calibration JSON you load at runtime.
// ------------------------------------------------------------
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
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
            var obj = new (double X, double Y, double Z)[_cornersX * _cornersY];
            for (int r = 0; r < _cornersY; r++)
                for (int c = 0; c < _cornersX; c++)
                    obj[r * _cornersX + c] = (c * _squareMm, r * _squareMm, 0);

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

            // Emgu.CV 4.9: StereoCalibrate takes MCvPoint3D32f[][] and PointF[][]
            // (not VectorOf* arrays from older versions)
            var objArr  = _objPoints.Select(v => v.ToArray()).ToArray();
            var pts0Arr = _imgPts0.Select(v => v.ToArray()).ToArray();
            var pts1Arr = _imgPts1.Select(v => v.ToArray()).ToArray();

            // K0/D0/K1/D1 start empty — there's no separate single-camera
            // calibration step feeding them a prior estimate, so CalibType.Default
            // (jointly estimate intrinsics + extrinsics from these frame pairs) is
            // required here. CalibType.FixIntrinsic (previously used) tells OpenCV
            // to treat whatever's already in K0/D0/K1/D1 as correct and skip
            // estimating them — with empty Mats that "fixes" garbage intrinsics
            // and inflates RMS regardless of how much/varied the capture data is.
            double rms = CvInvoke.StereoCalibrate(
                objArr, pts0Arr, pts1Arr,
                K0, D0, K1, D1, imgSize,
                R, T, E, F,
                CalibType.Default,
                new MCvTermCriteria(100, 1e-5));

            Console.WriteLine($"[Calibrator] RMS reprojection error: {rms:F4} px");

            // Stereo rectification
            var R0 = new Mat(); var R1 = new Mat();
            var P0 = new Mat(); var P1 = new Mat();
            var Q  = new Mat();
            // Emgu.CV 4.9: StereoRectify requires newImageSize + ref ROI rectangles
            var roi0 = new System.Drawing.Rectangle();
            var roi1 = new System.Drawing.Rectangle();
            CvInvoke.StereoRectify(K0, D0, K1, D1, imgSize, R, T,
                R0, R1, P0, P1, Q,
                StereoRectifyType.Default, 0.0, imgSize,
                ref roi0, ref roi1);

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

        /// <summary>
        /// Diagnostic helper for --preview: runs the same corner search
        /// AddFramePair does but only to draw the result and save it to disk —
        /// doesn't affect calibration state. Lets you check focus/framing on
        /// the actual sensor image instead of guessing from SpinView separately.
        /// Green corners overlay = found; a saved-but-plain frame = not found
        /// (so you can visually judge whether it's a size/blur/framing issue).
        /// </summary>
        public void SavePreview(byte[] left, byte[] right, int w, int h, string leftPath, string rightPath)
        {
            SavePreviewOne(left, w, h, leftPath);
            SavePreviewOne(right, w, h, rightPath);
        }

        private void SavePreviewOne(byte[] data, int w, int h, string path)
        {
            var mat = BytesToMat(data, w, h);
            var corners = new VectorOfPointF();
            var size = new Size(_cornersX, _cornersY);
            bool found = CvInvoke.FindChessboardCorners(mat, size, corners);

            var color = new Mat();
            CvInvoke.CvtColor(mat, color, ColorConversion.Gray2Bgr);
            if (found)
            {
                var criteria = new MCvTermCriteria(30, 0.001);
                CvInvoke.CornerSubPix(mat, corners, new Size(11, 11), new Size(-1, -1), criteria);
                CvInvoke.DrawChessboardCorners(color, size, corners, found);
            }
            CvInvoke.Imwrite(path, color);
        }

        private double[] MatToArray(Mat m)
        {
            var arr = new double[m.Rows * m.Cols * m.NumberOfChannels];
            m.CopyTo(arr);
            return arr;
        }
    }
}