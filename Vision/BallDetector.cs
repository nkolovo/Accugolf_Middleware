// ------------------------------------------------------------
// Vision/BallDetector.cs
// ------------------------------------------------------------
using System;
using System.Collections.Generic;
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
        // Keyed by CameraIndex: a single shared background Mat here (the original
        // implementation) diffs whichever camera's frame arrives against a model
        // that's really an alternating blend of BOTH cameras' distinct viewpoints —
        // fine by coincidence for a dead-centered trajectory (both views similar
        // enough that the blend is roughly harmless), but for any shot with real
        // lateral motion the two views diverge enough to produce spurious
        // near-identical "detections" in both cameras — background-subtraction
        // ghosting, not the real ball. Found via a left-azimuth mock shot where
        // triangulation was self-rejecting 100% of the time: the "ball" it was
        // triangulating had a ~3px disparity implying a nonsensical Z of 254m.
        private readonly Dictionary<int, Mat> _backgrounds = new();
        private SportProfile _profile = new();

        public void SetProfile(SportProfile p) => _profile = p;

        public DetectionResult Detect(CameraFrame frame)
        {
            using var raw = new Mat(frame.Height, frame.Width, DepthType.Cv8U, 1);
            raw.SetTo(frame.Data);

            // Background subtraction — per camera, not shared (see class comment).
            if (!_backgrounds.TryGetValue(frame.CameraIndex, out var background))
            {
                _backgrounds[frame.CameraIndex] = raw.Clone();
                return new DetectionResult { Found = false };
            }

            using var diff = new Mat();
            CvInvoke.AbsDiff(raw, background, diff);

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

                // Image-moment centroid, not the bounding rectangle's center: rect.X
                // and rect.Width are integers, so rect.X + rect.Width/2f can only ever
                // land on half-pixel increments — it snaps to a coarse grid rather than
                // measuring the contour's true center. Moments are area-weighted over
                // the actual contour polygon, giving genuine sub-pixel accuracy (~0.1–
                // 0.3px on a clean, well-contrasted blob like this one) instead of being
                // capped at a 0.5px quantization step. This centroid feeds triangulation,
                // the Kalman velocity fit, and Spin3D's point tracking — all three were
                // inheriting this same coarse-quantization noise. Falls back to the old
                // bounding-rect method only in the degenerate case of a zero-area moment
                // (shouldn't happen given the MinContourArea check above already passed).
                var moments = CvInvoke.Moments(contours[i], false);
                float cx, cy;
                if (moments.M00 > 1e-5)
                {
                    cx = (float)(moments.M10 / moments.M00);
                    cy = (float)(moments.M01 / moments.M00);
                }
                else
                {
                    var rect = CvInvoke.BoundingRectangle(contours[i]);
                    cx = rect.X + rect.Width / 2f;
                    cy = rect.Y + rect.Height / 2f;
                }
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

            // Rolling background update (this camera's own model only). Weight was
            // 0.95/0.05 — background only catches up 5% toward the current frame
            // per cycle, taking ~20 frames to "forget" any pixel the ball recently
            // passed through. For a ball moving mostly in depth that ghost trail
            // stays small and localized (found working fine for straight, azimuth=0
            // shots). For a ball also sweeping laterally across the sensor, the
            // trail elongates across many distinct pixel positions — morphological
            // Close merges it into the ball's own contour, and the combined area
            // grows every frame until it exceeds MaxContourArea and gets rejected
            // entirely (found live: contour area climbing 5512->7868->...->10562px²
            // over consecutive frames on a left-azimuth mock shot, well past
            // soccer's 8000px² ceiling). Faster decay (0.7/0.3) keeps the trail short
            // regardless of travel direction — a real, general improvement (any real
            // ball crossing the frame instead of receding straight back would hit
            // the same growth), not just a mock-testing tweak.
            //
            // Selective now, not unconditional: only refresh when NOTHING was found
            // this frame. A ball sitting at address before the shot (always true on
            // real hardware — the golfer takes a stance, sits the ball, THEN swings)
            // would otherwise get slowly absorbed into the background at 30%/frame:
            // ~97% blended in after just 10 identical frames, i.e. under 100ms at
            // this rig's ~200fps. Found live via KalmanBallTracker's new rest-
            // position seeding: with 10 resting frames before a shot, the very first
            // real "moving" detection came back displaced by only ~4cm instead of
            // the true ~36cm a real 18 m/s shot covers in that time — the ball had
            // partially merged into its own background, corrupting the diffed
            // contour's centroid right at the one moment detection quality matters
            // most. Skipping the update on frames where a ball WAS found means the
            // background only ever represents "empty scene", however long the ball
            // sits still first.
            if (!best.Found)
                CvInvoke.AddWeighted(background, 0.7, raw, 0.3, 0, background);

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