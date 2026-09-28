// ------------------------------------------------------------
// Tests/FeaturePointTrackerTests.cs
// ------------------------------------------------------------
// Validates detection, temporal (optical-flow) tracking, and stereo
// correspondence against synthetic textured images with known ground truth.
// This proves the plumbing (GFTT -> optical flow -> row-constrained stereo
// match) works correctly — it does NOT prove real ball surfaces (soccer
// panels, football laces) have enough contrast at 720×540 for this to work
// reliably in practice. That's untested until real footage is available.
// ------------------------------------------------------------
using System;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using FluentAssertions;
using SportSimulator.Vision;
using Xunit;

namespace SportSimulator.Tests
{
    public class FeaturePointTrackerTests
    {
        private const int Size = 120;

        // A disc scattered with small filled squares — squares give sharper,
        // more corner-detector-friendly gradients than circles (a circle's
        // boundary is smooth; GFTT/Harris-style detectors respond to corners).
        private static Mat MakeTexturedBall(PointF center, float radius, int seed = 7)
        {
            var mat = new Mat(Size, Size, DepthType.Cv8U, 1);
            mat.SetTo(new MCvScalar(40));
            CvInvoke.Circle(mat, Point.Round(center), (int)radius, new MCvScalar(120), -1);

            var rng = new Random(seed);
            for (int i = 0; i < 14; i++)
            {
                double angle = rng.NextDouble() * 2 * Math.PI;
                double r = rng.NextDouble() * (radius - 6);
                int cx = (int)(center.X + r * Math.Cos(angle));
                int cy = (int)(center.Y + r * Math.Sin(angle));
                int shade = rng.Next(0, 2) == 0 ? 235 : 5;
                CvInvoke.Rectangle(mat, new Rectangle(cx - 3, cy - 3, 6, 6), new MCvScalar(shade), -1);
            }
            return mat;
        }

        // ── Detection ────────────────────────────────────────────────────────────

        [Fact]
        public void FirstUpdate_DetectsPointsWithinBallRegion()
        {
            var center = new PointF(60, 60);
            float radius = 30;
            using var frame = MakeTexturedBall(center, radius);

            var tracker = new FeaturePointTracker();
            var points = tracker.Update(frame, center, radius);

            points.Should().NotBeEmpty("a textured ball region should yield trackable corners");
            foreach (var p in points)
            {
                float dx = p.Left.X - center.X, dy = p.Left.Y - center.Y;
                MathF.Sqrt(dx * dx + dy * dy).Should().BeLessThan(radius * 1.5f,
                    "detected points should be at or near the ball, not scattered across the whole frame");
            }
        }

        [Fact]
        public void DetectedPoints_HaveUniqueIds()
        {
            var center = new PointF(60, 60);
            float radius = 30;
            using var frame = MakeTexturedBall(center, radius);

            var tracker = new FeaturePointTracker();
            var points = tracker.Update(frame, center, radius);

            var ids = Array.ConvertAll(points, p => p.Id);
            ids.Should().OnlyHaveUniqueItems();
        }

        // ── Temporal tracking (optical flow) ────────────────────────────────────

        [Fact]
        public void TrackForward_MaintainsIdentity_AcrossTranslation()
        {
            var center1 = new PointF(55, 60);
            float radius = 28;
            using var frame1 = MakeTexturedBall(center1, radius);

            var tracker = new FeaturePointTracker();
            var points1 = tracker.Update(frame1, center1, radius);
            points1.Length.Should().BeGreaterThan(3, "need enough points for a meaningful tracking test");

            // Shift the whole textured pattern by a known (dx, dy) — same texture,
            // same relative feature layout, just translated.
            const float dx = 3f, dy = 2f;
            var center2 = new PointF(center1.X + dx, center1.Y + dy);
            using var frame2 = MakeTexturedBall(center2, radius);

            var points2 = tracker.Update(frame2, center2, radius);
            points2.Should().NotBeEmpty();

            // Every surviving tracked point should keep its ID from frame 1, and
            // should have moved by approximately (dx, dy) — not be some unrelated
            // freshly-detected point that happens to share an ID by coincidence.
            var byId1 = new System.Collections.Generic.Dictionary<int, PointF>();
            foreach (var p in points1) byId1[p.Id] = p.Left;

            int matched = 0;
            foreach (var p in points2)
            {
                if (!byId1.TryGetValue(p.Id, out var prev)) continue;
                matched++;
                p.Left.X.Should().BeApproximately(prev.X + dx, 1.5f, "tracked point should move with the pattern");
                p.Left.Y.Should().BeApproximately(prev.Y + dy, 1.5f, "tracked point should move with the pattern");
            }
            matched.Should().BeGreaterThan(0, "at least some points should be tracked forward by identity, not re-detected from scratch");
        }

        [Fact]
        public void Reset_ForcesFreshDetectionInsteadOfTracking()
        {
            var center = new PointF(60, 60);
            float radius = 28;
            using var frame1 = MakeTexturedBall(center, radius);

            var tracker = new FeaturePointTracker();
            var points1 = tracker.Update(frame1, center, radius);
            tracker.Reset();

            using var frame2 = MakeTexturedBall(center, radius, seed: 99); // different pattern
            var act = () => tracker.Update(frame2, center, radius);
            act.Should().NotThrow("a fresh detection after Reset should not try to optical-flow against a disposed previous frame");
        }

        // ── Stereo correspondence ────────────────────────────────────────────────

        [Fact]
        public void FindStereoMatch_RecoversKnownDisparity()
        {
            var leftCenter = new PointF(70, 60);
            float radius = 28;
            const float disparity = 12f; // rightX = leftX - disparity

            using var leftFrame  = MakeTexturedBall(leftCenter, radius, seed: 3);
            using var rightFrame = MakeTexturedBall(new PointF(leftCenter.X - disparity, leftCenter.Y), radius, seed: 3);

            var tracker = new FeaturePointTracker();
            var leftPoint = new PointF(leftCenter.X, leftCenter.Y); // ball center itself is a stable, present feature

            var match = tracker.FindStereoMatch(leftFrame, leftPoint, rightFrame, expectedDisparityPx: disparity);

            match.Should().NotBeNull("matching texture should be found within the search range");
            match!.Value.X.Should().BeApproximately(leftPoint.X - disparity, 1.5f,
                "recovered match should sit at the correct disparity offset");
            match.Value.Y.Should().BeApproximately(leftPoint.Y, 0.1f, "rectified correspondence must stay on the same row");
        }

        [Fact]
        public void FindStereoMatch_NoTexture_ReturnsNull()
        {
            using var blankLeft  = new Mat(Size, Size, DepthType.Cv8U, 1);
            using var blankRight = new Mat(Size, Size, DepthType.Cv8U, 1);
            blankLeft.SetTo(new MCvScalar(100));
            blankRight.SetTo(new MCvScalar(100));

            var tracker = new FeaturePointTracker();
            // A uniform image matches everywhere equally well — NCC score is
            // meaningless, but more importantly there's no reason to trust any
            // single peak. The 0.5 confidence floor should reject this either way
            // once the flat correlation surface fails to clear it convincingly,
            // OR the disparity/edge guards reject it. Either outcome (null) is correct.
            var match = tracker.FindStereoMatch(blankLeft, new PointF(60, 60), blankRight, expectedDisparityPx: 0);
            match.Should().BeNull("a featureless image gives no reliable point to match");
        }
    }
}
