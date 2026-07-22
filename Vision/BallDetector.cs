// ------------------------------------------------------------
// Vision/BallDetector.cs
// ------------------------------------------------------------
using System;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System.Drawing;
using SportSimulator.Models;

namespace SportSimulator.Vision
{
    public class DetectionResult
    {
        public bool Found { get; set; }
        public PointF Center { get; set; }
        public float RadiusPx { get; set; }
        public float Confidence { get; set; }
        public long TimestampUs { get; set; }
        public int CameraIndex { get; set; }

        // Square grayscale crop centered on the ball, resized to CropSize×CropSize.
        // Null if the ball was too close to the frame edge to get a clean crop.
        // Consumed by SpinEstimator — NOT used for position/tracking.
        public byte[]? Crop { get; set; }
        public int CropSize { get; set; }
    }

    public class BallDetector
    {
        private Mat? _background;
        private SportProfile _profile = new();

        public void SetProfile(SportProfile p) => _profile = p;

        public DetectionResult Detect(CameraFrame frame)
        {
            using var raw = new Mat(frame.Height, frame.Width, DepthType.Cv8U, 1);
            raw.SetTo(frame.Data);

            // Background subtraction
            if (_background == null)
            {
                _background = raw.Clone();
                return new DetectionResult { Found = false };
            }

            using var diff = new Mat();
            CvInvoke.AbsDiff(raw, _background, diff);

            // Gaussian blur + threshold
            using var blurred = new Mat();
            CvInvoke.GaussianBlur(diff, blurred, new Size(5, 5), 1.5);
            using var thresh = new Mat();
            CvInvoke.Threshold(blurred, thresh, 25, 255, ThresholdType.Binary);

            // Morphological cleanup
            using var kernel = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(5, 5), new Point(-1, -1));
            CvInvoke.MorphologyEx(thresh, thresh, MorphOp.Close, kernel, new Point(-1, -1), 2, BorderType.Default, new MCvScalar());

            // Find contours
            using var contours = new VectorOfVectorOfPoint();
            using var hierarchy = new Mat();
            CvInvoke.FindContours(thresh, contours, hierarchy, RetrType.External, ChainApproxMethod.ChainApproxSimple);

            DetectionResult best = new() { Found = false, TimestampUs = frame.TimestampUs, CameraIndex = frame.CameraIndex };
            float bestScore = 0;

            for (int i = 0; i < contours.Size; i++)
            {
                double area = CvInvoke.ContourArea(contours[i]);
                if (area < _profile.MinContourArea || area > _profile.MaxContourArea) continue;

                var rect = CvInvoke.BoundingRectangle(contours[i]);
                float cx = rect.X + rect.Width / 2f;
                float cy = rect.Y + rect.Height / 2f;
                float r = MathF.Sqrt((float)area / MathF.PI);

                // Circularity score (1.0 = perfect circle; lower for pucks)
                double perim = CvInvoke.ArcLength(contours[i], true);
                float circularity = _profile.IsSphere
                    ? (float)(4 * Math.PI * area / (perim * perim))
                    : 1f; // skip for puck

                float score = (float)area * (circularity * 0.5f + 0.5f);
                if (score > bestScore)
                {
                    bestScore = score;
                    best = new DetectionResult
                    {
                        Found = true,
                        Center = new PointF(cx, cy),
                        RadiusPx = r,
                        Confidence = MathF.Max(0f, MathF.Min(1f, circularity)),
                        TimestampUs = frame.TimestampUs,
                        CameraIndex = frame.CameraIndex
                    };
                }
            }

            // Rolling background update
            CvInvoke.AddWeighted(_background, 0.95, raw, 0.05, 0, _background);

            if (best.Found)
            {
                best.Crop = ExtractCrop(raw, best.Center, best.RadiusPx, out int cropSize);
                best.CropSize = cropSize;
            }

            return best;
        }

        // Fixed output size for spin crops. Kept a multiple of 4 so Emgu.CV's
        // Cv8U row stride never needs padding — avoids stride bugs when reading
        // the Mat back out as a flat byte[].
        private const int SpinCropSize = 48;

        // Pad around the detected radius, clip to frame bounds, and resize to a
        // fixed square so SpinEstimator can compare crops of the same size across
        // frames regardless of how big the ball appeared in each one.
        private static byte[]? ExtractCrop(Mat raw, PointF center, float radiusPx, out int size)
        {
            size = SpinCropSize;
            int pad = Math.Max(8, (int)MathF.Ceiling(radiusPx * 1.6f));
            var wanted = new Rectangle((int)(center.X - pad), (int)(center.Y - pad), pad * 2, pad * 2);
            var bounds = new Rectangle(0, 0, raw.Cols, raw.Rows);
            var clipped = Rectangle.Intersect(wanted, bounds);

            // Ball too close to the frame edge — a lopsided crop puts the ball off
            //-center, which throws off the rotation-about-center search in
            // SpinEstimator. Require at least 75% of the padded box (pad*2) to
            // survive clipping; skip rather than resize a skewed sliver.
            // (Note: because a detected center is always inside [0, raw.Cols) /
            // [0, raw.Rows), single-edge clipping alone can only ever remove up to
            // half the padded box — this threshold is what makes the check reachable.)
            if (clipped.Width < pad * 1.5f || clipped.Height < pad * 1.5f) return null;

            using var roi = new Mat(raw, clipped);
            using var resized = new Mat();
            CvInvoke.Resize(roi, resized, new Size(size, size));

            // Matrix<byte> with indexer — same reliable read pattern documented in
            // KalmanBallTracker.cs / Triangulator.cs for pulling data back out of a Mat.
            using var m = new Matrix<byte>(size, size);
            resized.CopyTo(m.Mat);
            var bytes = new byte[size * size];
            for (int r = 0; r < size; r++)
                for (int c = 0; c < size; c++)
                    bytes[r * size + c] = m[r, c];
            return bytes;
        }
    }
}