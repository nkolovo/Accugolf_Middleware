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
                        Confidence = Math.Clamp(circularity, 0f, 1f),
                        TimestampUs = frame.TimestampUs,
                        CameraIndex = frame.CameraIndex
                    };
                }
            }

            // Rolling background update
            CvInvoke.AddWeighted(_background, 0.95, raw, 0.05, 0, _background);
            return best;
        }
    }
}