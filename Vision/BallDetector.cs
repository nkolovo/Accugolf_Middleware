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

        private const int ForegroundThreshold = 25;
        private const int BackgroundUpdateFrames = 64;       // about 4 brightness levels a second at 262fps
        private const int SceneChangedUpdatesToReseed = 20;  // about 5s
        private readonly Dictionary<int, int> _frameCounts = new();
        private readonly Dictionary<int, int> _sceneChangedUpdates = new();

        // Background model (this camera's own only). It has to keep meaning "empty
        // scene", so nothing in view may ever be learned, however long it sits there.
        // Blending frames in at 30% couldn't do that, per frame or per pixel: a ball
        // sitting at address blended in within 10 frames, and pixels of a person that
        // happened to be close to the floor's brightness were learned as they passed,
        // leaving ghost blobs twice the ball's size stuck to it once they moved (found
        // replaying a real session). The real floor never needs that speed: it held
        // still to within 33 stray pixels across a 48s session with a person in it.
        //
        // So a pixel that looks like background creeps one brightness level toward the
        // current frame per update. That tracks real lighting drift, and because only
        // pixels already within the threshold can move, what gets learned by mistake
        // stays below the threshold and heals by itself.
        private void UpdateBackground(int cameraIndex, Mat background, Mat frame)
        {
            using var difference = new Mat();
            using var unchanged = new Mat();
            CvInvoke.AbsDiff(frame, background, difference);
            CvInvoke.Threshold(difference, unchanged, ForegroundThreshold, 255, ThresholdType.BinaryInv);

            // The one thing that should replace foreground: a lighting change, which
            // would otherwise read as a frame-filling blob forever. A person can cover
            // half the frame too, but not motionlessly for this long in normal use.
            _sceneChangedUpdates.TryGetValue(cameraIndex, out int changedUpdates);
            changedUpdates = CvInvoke.CountNonZero(unchanged) < frame.Rows * frame.Cols / 2 ? changedUpdates + 1 : 0;
            _sceneChangedUpdates[cameraIndex] = changedUpdates;
            if (changedUpdates >= SceneChangedUpdatesToReseed)
            {
                frame.CopyTo(background);
                _sceneChangedUpdates[cameraIndex] = 0;
                Console.WriteLine($"[BallDetector] cam={cameraIndex} most of the frame changed for about " +
                    "5 seconds: treating it as a lighting change, background reset");
                return;
            }

            using var step = new Mat();
            using var one = new ScalarArray(1);
            CvInvoke.Compare(frame, background, step, CmpType.GreaterThan);
            CvInvoke.BitwiseAnd(step, unchanged, step);
            CvInvoke.Add(background, one, background, step);

            CvInvoke.Compare(frame, background, step, CmpType.LessThan);
            CvInvoke.BitwiseAnd(step, unchanged, step);
            CvInvoke.Subtract(background, one, background, step);
        }

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

            // A ball brighter than the floor is told apart from what's darker than it:
            // its own shadow, which otherwise pulls its center a different way in each
            // camera, and a dark foot, which otherwise stays joined to the ball by shadow
            // until the ball has left the frame (a real kick got no detections at all).
            using var diff = new Mat();
            if (_profile.BrighterThanBackground)
                CvInvoke.Subtract(raw, background, diff);
            else
                CvInvoke.AbsDiff(raw, background, diff);

            // Gaussian blur + threshold
            using var blurred = new Mat();
            CvInvoke.GaussianBlur(diff, blurred, new Size(5, 5), 1.5);
            using var thresh = new Mat();
            CvInvoke.Threshold(blurred, thresh, ForegroundThreshold, 255, ThresholdType.Binary);

            // Morphological cleanup
            using var kernel = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(5, 5), new Point(-1, -1));
            CvInvoke.MorphologyEx(thresh, thresh, MorphOp.Close, kernel, new Point(-1, -1), 2, BorderType.Default, new MCvScalar());

            // Find contours
            using var contours = new VectorOfVectorOfPoint();
            using var hierarchy = new Mat();
            CvInvoke.FindContours(thresh, contours, hierarchy, RetrType.External, ChainApproxMethod.ChainApproxSimple);

            DetectionResult best = new() { Found = false, TimestampUs = frame.TimestampUs, CameraIndex = frame.CameraIndex };
            float bestScore = 0;
            double bestArea = 0;

            // Diagnostic only: area and the frame edge are the ONLY rejection criteria
            // in this loop (circularity affects ranking between passing candidates,
            // it doesn't reject on its own) — so tracking the largest contour that
            // failed it tells us, when nothing is found, whether background
            // subtraction is isolating a ball-shaped blob at all (and it's just
            // outside MinContourArea/MaxContourArea) versus not finding any
            // meaningful blob whatsoever (a lighting/contrast/threshold problem
            // instead). One line per empty frame, not per rejected contour.
            double largestRejectedArea = 0;

            for (int i = 0; i < contours.Size; i++)
            {
                double area = CvInvoke.ContourArea(contours[i]);

                // A blob cut off by the frame edge is a ball partly out of view: its
                // centroid is off by up to a radius, and differently in each camera, so
                // it triangulates to a wrong position that moves fast enough to look like
                // a shot (found live-simulating a ball rolled into view: 8.6 m/s, 30cm off).
                var bounds = CvInvoke.BoundingRectangle(contours[i]);
                bool cutOff = bounds.Left <= 0 || bounds.Top <= 0 ||
                              bounds.Right >= thresh.Cols || bounds.Bottom >= thresh.Rows;

                if (cutOff || area < _profile.MinContourArea || area > _profile.MaxContourArea)
                {
                    if (area > largestRejectedArea) largestRejectedArea = area;
                    continue;
                }

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

                // Thresholding on brightness misses a ball's dimmer side and any black
                // panel on its outline, on different sides in each camera, so the centroid
                // of what's left is biased: 10px of false disparity read a resting ball
                // 12cm too high. What IS visible still lies on the ball's true circle.
                if (_profile.BrighterThanBackground && _profile.IsSphere)
                {
                    var circle = CvInvoke.MinEnclosingCircle(contours[i]);
                    cx = circle.Center.X;
                    cy = circle.Center.Y;
                    r = circle.Radius;
                }

                // Circularity score (1.0 = perfect circle; lower for pucks)
                double perim = CvInvoke.ArcLength(contours[i], true);
                float circularity = _profile.IsSphere
                    ? (float)(4 * Math.PI * area / (perim * perim))
                    : 1f; // skip for puck

                float score = (float)area * (circularity * 0.5f + 0.5f);
                if (score > bestScore)
                {
                    bestScore = score;
                    bestArea = area;
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

            _frameCounts.TryGetValue(frame.CameraIndex, out int frameCount);
            _frameCounts[frame.CameraIndex] = ++frameCount;
            if (frameCount % BackgroundUpdateFrames == 0)
                UpdateBackground(frame.CameraIndex, background, raw);

            if (best.Found)
            {
                best.Crop = ExtractCrop(raw, best.Center, best.RadiusPx, out int cropSize);
                best.CropSize = cropSize;

                if (ShouldLog(frame.CameraIndex, LogFound))
                    Console.WriteLine($"[BallDetector] cam={frame.CameraIndex} ball detected: " +
                        $"area={bestArea:F0}px² center=({best.Center.X:F0},{best.Center.Y:F0}) conf={best.Confidence:F2}");
            }
            else if (largestRejectedArea >= _profile.MinContourArea / 4)
            {
                // A blob WAS isolated by background subtraction but every candidate
                // fell outside [MinContourArea, MaxContourArea] — points at the area
                // window being wrong for what the camera's actually seeing, not a
                // lighting/contrast/background problem, or at a ball only partly in view.
                // If this never prints either, background subtraction isn't isolating
                // anything ball-shaped at all. Specks far too small to matter don't count.
                if (ShouldLog(frame.CameraIndex, LogRejected))
                    Console.WriteLine($"[BallDetector] cam={frame.CameraIndex} blob seen but rejected: " +
                        $"{largestRejectedArea:F0}px² (window is {_profile.MinContourArea}-{_profile.MaxContourArea}px², " +
                        "or it touches the frame edge)");
            }
            else
            {
                ShouldLog(frame.CameraIndex, LogNothing);
            }

            return best;
        }

        // Per camera: a repeat every ~10s while nothing changes, and a change at most
        // twice a second, since a person in view flips found/rejected constantly. These
        // lines buried the [Shot] line under hundreds of others, and logging every frame
        // was 500 console writes a second on the thread that has to keep up with the cameras.
        private const int LogNothing = 0, LogFound = 1, LogRejected = 2;
        private const int FramesBetweenRepeats = 2620;
        private const int MinFramesBetweenLogs = 130;
        private readonly Dictionary<int, (int state, int framesSinceLog)> _logStates = new();

        private bool ShouldLog(int cameraIndex, int state)
        {
            if (!_logStates.TryGetValue(cameraIndex, out var s))
                s = (LogNothing, FramesBetweenRepeats);

            int frames = s.framesSinceLog + 1;
            bool log = state != LogNothing &&
                       (frames >= FramesBetweenRepeats || (state != s.state && frames >= MinFramesBetweenLogs));

            _logStates[cameraIndex] = (state, log ? 0 : frames);
            return log;
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
