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

            bool foundL = FindCorners(matL, size, cornersL);
            bool foundR = FindCorners(matR, size, cornersR);

            if (!foundL || !foundR) return false;

            // Sub-pixel refinement
            var criteria = new MCvTermCriteria(30, 0.001);
            CvInvoke.CornerSubPix(matL, cornersL, new Size(11,11), new Size(-1,-1), criteria);
            CvInvoke.CornerSubPix(matR, cornersR, new Size(11,11), new Size(-1,-1), criteria);

            AddCornerPair(cornersL.ToArray(), cornersR.ToArray());

            Console.WriteLine($"[Calibrator] Pair {_objPoints.Count} accepted.");
            return true;
        }

        // Object points in METERS, not millimeters: T, the rectified P1
        // translation, and Q all come out in whatever unit these are in, and
        // Triangulator treats all of them as meters. Building these in mm made a
        // real calibration report a ~500m baseline and every triangulated point
        // ~1000x too far away (then rejected as out of range).
        private void AddCornerPair(PointF[] left, PointF[] right)
        {
            double squareM = _squareMm / 1000.0;
            var obj = new MCvPoint3D32f[_cornersX * _cornersY];
            for (int r = 0; r < _cornersY; r++)
                for (int c = 0; c < _cornersX; c++)
                    obj[r * _cornersX + c] = new MCvPoint3D32f((float)(c * squareM), (float)(r * squareM), 0f);

            _objPoints.Add(new VectorOfPoint3D32F(obj));
            _imgPts0.Add(new VectorOfPointF(left));
            _imgPts1.Add(new VectorOfPointF(right));
        }

        // ── Persisting captured corners ─────────────────────────────────────────
        // Saved after every accepted pair, so changing calibration MATH never
        // again requires redoing a 100-capture session — re-run it offline with
        // `--calibrate-from <file>` instead.

        private class PointsFile
        {
            public int CornersX { get; set; }
            public int CornersY { get; set; }
            public float SquareMm { get; set; }
            public int ImageWidth { get; set; }
            public int ImageHeight { get; set; }
            public List<float[]> Left { get; set; } = new();  // per pair: x0,y0,x1,y1,...
            public List<float[]> Right { get; set; } = new();
        }

        public void SavePoints(string path, int w, int h)
        {
            var file = new PointsFile
            {
                CornersX = _cornersX, CornersY = _cornersY, SquareMm = _squareMm,
                ImageWidth = w, ImageHeight = h,
                Left  = _imgPts0.Select(v => Flatten(v.ToArray())).ToList(),
                Right = _imgPts1.Select(v => Flatten(v.ToArray())).ToList()
            };
            System.IO.File.WriteAllText(path, System.Text.Json.JsonSerializer.Serialize(file));
        }

        public static StereoCalibrator LoadPoints(string path, out int w, out int h)
        {
            var file = System.Text.Json.JsonSerializer.Deserialize<PointsFile>(System.IO.File.ReadAllText(path))
                       ?? throw new Exception($"Could not read calibration points from {path}");

            var cal = new StereoCalibrator(file.CornersX, file.CornersY, file.SquareMm);
            for (int i = 0; i < file.Left.Count; i++)
                cal.AddCornerPair(Unflatten(file.Left[i]), Unflatten(file.Right[i]));

            w = file.ImageWidth;
            h = file.ImageHeight;
            return cal;
        }

        private static float[] Flatten(PointF[] pts)
        {
            var a = new float[pts.Length * 2];
            for (int i = 0; i < pts.Length; i++) { a[2 * i] = pts[i].X; a[2 * i + 1] = pts[i].Y; }
            return a;
        }

        private static PointF[] Unflatten(float[] a)
        {
            var pts = new PointF[a.Length / 2];
            for (int i = 0; i < pts.Length; i++) pts[i] = new PointF(a[2 * i], a[2 * i + 1]);
            return pts;
        }

        /// <summary>
        /// Run calibration after collecting ≥ 15 frame pairs.
        /// Returns RMS reprojection error (< 1.0 px is good).
        /// focalPx is the focal length shared by both cameras (see below); null
        /// means StereoCalibrationData.MeasuredFocalPx.
        /// </summary>
        public double Calibrate(int w, int h, out StereoCalibrationData result, double? focalPx = null)
        {
            if (_objPoints.Count < 15)
                throw new Exception($"Need ≥15 frame pairs, have {_objPoints.Count}.");

            double f = focalPx ?? StereoCalibrationData.MeasuredFocalPx;
            var imgSize = new Size(w, h);
            var R  = new Mat(); var T  = new Mat();
            var E  = new Mat(); var F  = new Mat();

            // Emgu.CV 4.9: StereoCalibrate takes MCvPoint3D32f[][] and PointF[][]
            // (not VectorOf* arrays from older versions)
            var objArr  = _objPoints.Select(v => v.ToArray()).ToArray();
            var pts0Arr = _imgPts0.Select(v => v.ToArray()).ToArray();
            var pts1Arr = _imgPts1.Select(v => v.ToArray()).ToArray();

            // Solve only what the board can actually pin down, using facts about this
            // hardware for the rest. A Letter-size board ~3m from an overhead camera
            // covers too little of the frame to separate focal length, principal
            // point, and distortion from each other, and every freer solve failed
            // live: a free joint solve gave k3=2660 and a rectified fx of 53205px;
            // pinning only k3 put the principal points above the image; per-camera
            // focal lengths then FixIntrinsic gave 1711 vs 1529px for two identical
            // cameras that see the board at the same size, which the stereo solve
            // could only explain with a ~300mm depth offset between two cameras
            // mounted side by side (and a rectified fx of 2268px).
            //   FixFocalLength + SameFocalLength — identical cameras and lenses. The
            //                       VALUE comes from a physical measurement, because
            //                       these images can't determine it: sweeping f from
            //                       1550 to 1850 moved RMS only 0.449 to 0.456px. It
            //                       barely moves the baseline either (488 to 496mm)
            //                       and doesn't touch in-plane X/Y (X = u*B/d); it
            //                       scales absolute depth, about 1.6mm per px at this
            //                       working distance. Correct it with a resting ball's
            //                       reported height: --calibrate-from --focal.
            //   FixAspectRatio    — square-pixel sensor: fx = fy.
            //   FixPrincipalPoint — normally mounted lens: center of the image.
            //   ZeroTangentDist   — tangential terms were soaking up principal-point
            //                       error (p1 = 0.47), not measuring anything real.
            //   FixK3             — not an extreme wide-angle lens.
            // Leaves k1/k2 per camera plus the stereo pose. Each camera alone seeds
            // the distortion, then the joint solve refines it alongside R and T.
            double rms0 = CalibrateSingle(objArr, pts0Arr, imgSize, f, out var K0, out var D0);
            double rms1 = CalibrateSingle(objArr, pts1Arr, imgSize, f, out var K1, out var D1);
            Console.WriteLine($"[Calibrator] Focal length fixed at {f:F1}px for both cameras.");
            Console.WriteLine($"[Calibrator] Camera 0 alone: RMS {rms0:F4}px");
            Console.WriteLine($"[Calibrator] Camera 1 alone: RMS {rms1:F4}px");

            double rms = CvInvoke.StereoCalibrate(
                objArr, pts0Arr, pts1Arr,
                K0, D0, K1, D1, imgSize,
                R, T, E, F,
                PinnedIntrinsics | CalibType.SameFocalLength,
                new MCvTermCriteria(100, 1e-6));

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
                StereoRectifier.Mode, 0.0, imgSize,
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

        private const CalibType PinnedIntrinsics =
            CalibType.UseIntrinsicGuess | CalibType.FixFocalLength | CalibType.FixPrincipalPoint |
            CalibType.FixAspectRatio | CalibType.ZeroTangentDist | CalibType.FixK3;

        // Single-camera calibration with the constraints described in Calibrate().
        // The Fix* flags need a seeded camera matrix (UseIntrinsicGuess): the focal
        // length and principal point stay exactly at the seed, only k1/k2 are solved.
        private static double CalibrateSingle(MCvPoint3D32f[][] obj, PointF[][] img, Size imgSize, double focalPx,
                                              out Mat K, out Mat D)
        {
            using (var k = new Matrix<double>(3, 3))
            {
                k.SetZero();
                k[0, 0] = focalPx; k[0, 2] = imgSize.Width / 2.0;
                k[1, 1] = focalPx; k[1, 2] = imgSize.Height / 2.0;
                k[2, 2] = 1.0;
                K = k.Mat.Clone();
            }
            using (var d = new Matrix<double>(1, 5))
            {
                d.SetZero();
                D = d.Mat.Clone();
            }

            return CvInvoke.CalibrateCamera(obj, img, imgSize, K, D, PinnedIntrinsics,
                new MCvTermCriteria(100, 1e-6), out _, out _);
        }

        // FindChessboardCornersSB is OpenCV's newer "sector-based" detector,
        // built specifically to be more robust than the classic
        // FindChessboardCorners to exactly the conditions this rig struggles
        // with: a board that reads small in frame (only ~15-20px per square at
        // this working distance), uneven/dim lighting, and mild blur. Exhaustive
        // + Accuracy trade a bit more compute for a more thorough search — a
        // non-issue for this one-shot-per-keypress interactive tool.
        //
        // Falls back to the classic detector if SB doesn't find it — different
        // algorithms, different failure modes, so trying both costs little and
        // only increases the chance of accepting a pair that's actually usable.
        private static bool FindCorners(Mat mat, Size size, VectorOfPointF corners)
        {
            if (CvInvoke.FindChessboardCornersSB(mat, size, corners,
                    CalibCbType.Exhaustive | CalibCbType.Accuracy))
                return true;

            corners.Clear();
            return CvInvoke.FindChessboardCorners(mat, size, corners);
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
            bool found = FindCorners(mat, size, corners);

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