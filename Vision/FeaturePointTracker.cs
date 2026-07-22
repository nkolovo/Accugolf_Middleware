// ------------------------------------------------------------
// Vision/FeaturePointTracker.cs
// ------------------------------------------------------------
// Detects and tracks distinctive natural-texture points on the ball's
// surface (panel seams, laces, stitching) across consecutive frames of the
// LEFT camera, and finds each point's stereo correspondence in the RIGHT
// (rectified) camera at the same instant. Combined with Triangulator, this
// gives each tracked point's 3D position over time, which RotationFitter
// turns into full 3D spin (axis + magnitude) — unlike SpinEstimator's
// single-camera 2D correlation, which can only see rotation about that one
// camera's own viewing axis.
//
// Natural-texture tracking, not marked/painted balls — reliability depends
// entirely on how much contrast the ball's real surface has at 720×540
// under real lighting. Untested against real footage; validate before
// trusting the numbers, especially for lower-contrast surfaces (a football's
// laces/seams are a weaker signal than a soccer ball's panel edges).
// ------------------------------------------------------------
using System;
using System.Collections.Generic;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Features2D;
using Emgu.CV.Structure;

namespace SportSimulator.Vision
{
    public class TrackedPoint
    {
        public int Id { get; set; }     // persists across frames for the SAME physical feature
        public PointF Left { get; set; } // position in the left (rectified) frame
    }

    public class FeaturePointTracker
    {
        private const int MaxFeatures     = 20;
        private const double QualityLevel = 0.05; // relative to the strongest corner found in the masked region
        private const double MinDistancePx = 4;
        private const int WinSize          = 11;  // Lucas-Kanade search window
        private const int MaxPyramidLevel  = 2;
        private const int MinPointsBeforeRedetect = 4;

        private readonly GFTTDetector _detector = new(MaxFeatures, QualityLevel, MinDistancePx);

        private Mat?     _prevLeftFrame;
        private PointF[]? _prevPoints;
        private int[]?    _prevIds;
        private int       _nextId;

        /// <summary>
        /// Detect (first call, or whenever tracking has degraded) or continue
        /// tracking feature points on the ball in the left camera's rectified
        /// frame, restricted to the region around the detected ball.
        /// </summary>
        public TrackedPoint[] Update(Mat leftFrame, PointF ballCenter, float ballRadiusPx)
        {
            PointF[] points;
            int[] ids;

            if (_prevPoints != null && _prevLeftFrame != null && _prevPoints.Length >= MinPointsBeforeRedetect)
            {
                TrackForward(leftFrame, ballCenter, ballRadiusPx, out points, out ids);
                if (points.Length < MinPointsBeforeRedetect)
                    DetectFresh(leftFrame, ballCenter, ballRadiusPx, out points, out ids);
            }
            else
            {
                DetectFresh(leftFrame, ballCenter, ballRadiusPx, out points, out ids);
            }

            _prevLeftFrame?.Dispose();
            _prevLeftFrame = leftFrame.Clone();
            _prevPoints = points;
            _prevIds    = ids;

            var result = new TrackedPoint[points.Length];
            for (int i = 0; i < points.Length; i++)
                result[i] = new TrackedPoint { Id = ids[i], Left = points[i] };
            return result;
        }

        public void Reset()
        {
            _prevLeftFrame?.Dispose();
            _prevLeftFrame = null;
            _prevPoints = null;
            _prevIds = null;
        }

        // Pyramidal Lucas-Kanade optical flow — this is what preserves point
        // IDENTITY frame-to-frame. A fresh detection every frame would give an
        // unrelated set of points each time, useless for measuring how any one
        // point moved (which is what RotationFitter needs).
        private void TrackForward(Mat leftFrame, PointF ballCenter, float ballRadiusPx,
                                   out PointF[] points, out int[] ids)
        {
            CvInvoke.CalcOpticalFlowPyrLK(
                _prevLeftFrame!, leftFrame, _prevPoints!,
                new Size(WinSize, WinSize), MaxPyramidLevel,
                new MCvTermCriteria(20, 0.03),
                out var nextPoints, out var status, out _,
                LKFlowFlag.Default, 1e-4);

            var keptPoints = new List<PointF>();
            var keptIds = new List<int>();
            float maxDriftSq = (ballRadiusPx * 2f) * (ballRadiusPx * 2f);

            for (int i = 0; i < status.Length; i++)
            {
                if (status[i] == 0) continue; // lost track

                // A point that wandered off the ball entirely is a bad track, not
                // a real feature to keep — discard rather than let it corrupt the
                // rotation fit.
                float dx = nextPoints[i].X - ballCenter.X, dy = nextPoints[i].Y - ballCenter.Y;
                if (dx * dx + dy * dy > maxDriftSq) continue;

                keptPoints.Add(nextPoints[i]);
                keptIds.Add(_prevIds![i]);
            }

            points = keptPoints.ToArray();
            ids    = keptIds.ToArray();
        }

        private void DetectFresh(Mat leftFrame, PointF ballCenter, float ballRadiusPx,
                                  out PointF[] points, out int[] ids)
        {
            using var mask = new Mat(leftFrame.Rows, leftFrame.Cols, DepthType.Cv8U, 1);
            mask.SetTo(new MCvScalar(0));
            int r = (int)MathF.Ceiling(ballRadiusPx * 1.3f);
            CvInvoke.Circle(mask, Point.Round(ballCenter), r, new MCvScalar(255), -1);

            var keypoints = _detector.Detect(leftFrame, mask);

            points = new PointF[keypoints.Length];
            ids    = new int[keypoints.Length];
            for (int i = 0; i < keypoints.Length; i++)
            {
                points[i] = keypoints[i].Point;
                ids[i]    = _nextId++;
            }
        }

        // ── Stereo correspondence ────────────────────────────────────────────────

        // Row-constrained patch search — valid ONLY on rectified images, where
        // corresponding points share the same row and differ only in X
        // (disparity). Searches a horizontal strip via MatchTemplate rather than
        // looping candidate offsets by hand — the sliding search is native/fast,
        // and MinMaxLoc finds the best-scoring offset in one call.
        //
        // Searches AROUND an expected disparity rather than blindly from zero —
        // this rig's real disparity at typical working distance is ~290px (fx≈1758,
        // baseline≈0.4953m, Z≈3.015m — see App/SimulatorEngine.cs), and swings much
        // wider across a shot's depth range (roughly 170–870px between 1m and 5m).
        // A fixed small search window either misses real matches at typical range
        // or has to be so wide it's slow and more prone to false matches. The
        // caller already has a good Z estimate each frame (the ball's own
        // just-triangulated position) — use it.
        public PointF? FindStereoMatch(Mat leftFrame, PointF leftPoint, Mat rightFrame,
                                        float expectedDisparityPx, float disparityMarginPx = 60f)
        {
            const int patchHalf = 5; // 11×11 template
            int lx = (int)leftPoint.X, ly = (int)leftPoint.Y;

            var frameBounds = new Rectangle(0, 0, leftFrame.Cols, leftFrame.Rows);
            var patchRect = new Rectangle(lx - patchHalf, ly - patchHalf, patchHalf * 2 + 1, patchHalf * 2 + 1);
            if (!frameBounds.Contains(patchRect)) return null;

            using var template = new Mat(leftFrame, patchRect);

            // Guard against near-uniform patches: normalized correlation (NCC)
            // divides by the patch's own standard deviation, which is ~0 for a
            // textureless region — that degenerate case can produce a spurious
            // high-confidence "match" anywhere, not a real absence of one.
            MCvScalar mean = default, stdDev = default;
            CvInvoke.MeanStdDev(template, ref mean, ref stdDev);
            if (stdDev.V0 < 5.0) return null; // ~no texture to match against

            // disparity = leftX - rightX must be >= 0 for a real, in-front point
            // (same convention as Triangulator.cs). Center the search on the
            // expected disparity instead of scanning the full [0, lx] range.
            float centerX = lx - expectedDisparityPx;
            int searchX0 = Math.Max(0, (int)(centerX - disparityMarginPx - patchHalf));
            int searchX1 = Math.Min(rightFrame.Cols, (int)(centerX + disparityMarginPx + patchHalf) + 1);
            int searchWidth = searchX1 - searchX0;
            if (searchWidth < template.Cols) return null;

            var stripRect = new Rectangle(searchX0, patchRect.Y, searchWidth, patchRect.Height);
            if (!new Rectangle(0, 0, rightFrame.Cols, rightFrame.Rows).Contains(stripRect)) return null;
            using var strip = new Mat(rightFrame, stripRect);

            using var result = new Mat();
            CvInvoke.MatchTemplate(strip, template, result, TemplateMatchingType.CcoeffNormed);

            double minVal = 0, maxVal = 0;
            Point minLoc = new(), maxLoc = new();
            CvInvoke.MinMaxLoc(result, ref minVal, ref maxVal, ref minLoc, ref maxLoc);

            // Weak match — texture too ambiguous/repetitive at this point to trust.
            if (maxVal < 0.5) return null;

            float matchedX = searchX0 + maxLoc.X + patchHalf;
            if (lx - matchedX <= 0.5f) return null; // non-positive disparity — behind baseline

            return new PointF(matchedX, ly);
        }
    }
}
