// ------------------------------------------------------------
// Tests/BallDetectorAdditionalTests.cs
// ------------------------------------------------------------
// Supplements BallDetectorTests.cs with: puck profile, multi-blob
// selection, and edge-of-frame detection.
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Models;
using SportSimulator.Vision;
using Xunit;

namespace SportSimulator.Tests
{
    public class BallDetectorAdditionalTests
    {
        private const int W = 320, H = 240;

        private static SportProfile SphereProfile(int minArea = 20, int maxArea = 3000) => new()
        {
            SportId        = "golf",
            IsSphere       = true,
            MinContourArea = minArea,
            MaxContourArea = maxArea
        };

        private static SportProfile PuckProfile() => new()
        {
            SportId        = "hockey",
            IsSphere       = false,  // circularity check skipped
            MinContourArea = 20,
            MaxContourArea = 3000
        };

        private static CameraFrame BlackFrame(long ts = 1000) => new()
        {
            CameraIndex = 0, Data = new byte[W * H],
            Width = W, Height = H, TimestampUs = ts
        };

        /// <summary>Fill a rectangle with white pixels (simulates a non-circular puck shape).</summary>
        private static CameraFrame RectFrame(int cx, int cy, int halfW, int halfH, long ts = 2000)
        {
            var data = new byte[W * H];
            for (int dy = -halfH; dy <= halfH; dy++)
            for (int dx = -halfW; dx <= halfW; dx++)
            {
                int row = cy + dy, col = cx + dx;
                if (row < 0 || row >= H || col < 0 || col >= W) continue;
                data[row * W + col] = 255;
            }
            return new CameraFrame { CameraIndex = 0, Data = data, Width = W, Height = H, TimestampUs = ts };
        }

        /// <summary>Draw two circles at different positions.</summary>
        private static CameraFrame TwoBlobFrame(
            int cx1, int cy1, int r1,
            int cx2, int cy2, int r2,
            long ts = 2000)
        {
            var data = new byte[W * H];
            void DrawCircle(int cx, int cy, int r)
            {
                for (int dy = -r; dy <= r; dy++)
                for (int dx = -r; dx <= r; dx++)
                {
                    if (dx * dx + dy * dy > r * r) continue;
                    int row = cy + dy, col = cx + dx;
                    if (row < 0 || row >= H || col < 0 || col >= W) continue;
                    data[row * W + col] = 255;
                }
            }
            DrawCircle(cx1, cy1, r1);
            DrawCircle(cx2, cy2, r2);
            return new CameraFrame { CameraIndex = 0, Data = data, Width = W, Height = H, TimestampUs = ts };
        }

        private static CameraFrame CircleFrame(int cx, int cy, int radius, long ts = 2000)
        {
            var data = new byte[W * H];
            for (int dy = -radius; dy <= radius; dy++)
            for (int dx = -radius; dx <= radius; dx++)
            {
                if (dx * dx + dy * dy > radius * radius) continue;
                int row = cy + dy, col = cx + dx;
                if (row < 0 || row >= H || col < 0 || col >= W) continue;
                data[row * W + col] = 255;
            }
            return new CameraFrame { CameraIndex = 0, Data = data, Width = W, Height = H, TimestampUs = ts };
        }

        // ── Puck profile ─────────────────────────────────────────────────────────

        [Fact]
        public void PuckProfile_NonCircularBlob_IsDetected()
        {
            // With IsSphere=false, circularity scoring is skipped.
            // A rectangular blob should be found where a sphere profile might score it low.
            var detector = new BallDetector();
            detector.SetProfile(PuckProfile());

            detector.Detect(BlackFrame());
            // A 20×10 rectangle centred at (160,120): area=400px, well within MinContourArea=20
            var result = detector.Detect(RectFrame(160, 120, halfW: 10, halfH: 5));

            result.Found.Should().BeTrue("puck detector should accept non-circular blobs");
        }

        [Fact]
        public void PuckProfile_ConfidenceIsOne_ForNonCircular()
        {
            // When IsSphere=false, circularity is hardcoded to 1.0 per BallDetector —
            // confidence should be at its max (clamped to 1.0f).
            var detector = new BallDetector();
            detector.SetProfile(PuckProfile());

            detector.Detect(BlackFrame());
            var result = detector.Detect(RectFrame(160, 120, halfW: 10, halfH: 5));

            result.Found.Should().BeTrue();
            result.Confidence.Should().BeApproximately(1.0f, 0.01f,
                "puck profile forces circularity=1, so confidence is clamped to 1.0");
        }

        // ── Multi-blob selection ──────────────────────────────────────────────────

        [Fact]
        public void MultipleBlobs_LargerBlobSelected()
        {
            // Two circles: small one at (80, 120), larger one at (240, 120).
            // Detector should pick the one with the highest score (area × circularity).
            var detector = new BallDetector();
            detector.SetProfile(SphereProfile());

            detector.Detect(BlackFrame());
            var result = detector.Detect(TwoBlobFrame(cx1: 80,  cy1: 120, r1: 4,
                                                       cx2: 240, cy2: 120, r2: 10));

            result.Found.Should().BeTrue();
            // Larger blob is at x≈240 — center should be right of frame midpoint
            result.Center.X.Should().BeGreaterThan(160f,
                "detector should select the larger-area blob (rightmost in this frame)");
        }

        [Fact]
        public void MultipleBlobs_BothInRange_OneSelected()
        {
            // Two identically-sized circles: only one result is returned.
            var detector = new BallDetector();
            detector.SetProfile(SphereProfile());

            detector.Detect(BlackFrame());
            var result = detector.Detect(TwoBlobFrame(cx1: 80, cy1: 120, r1: 8,
                                                       cx2: 240, cy2: 120, r2: 8));

            result.Found.Should().BeTrue("at least one blob should be detected");
            // Center X must be one of the two blobs (80 or 240), not somewhere in between
            result.Center.X.Should().Match(x =>
                System.Math.Abs((float)x - 80f) < 15f || System.Math.Abs((float)x - 240f) < 15f,
                "center must correspond to one of the two blobs, not a phantom midpoint");
        }

        // ── Edge-of-frame ────────────────────────────────────────────────────────

        [Fact]
        public void BallAtTopEdge_DoesNotThrow()
        {
            var detector = new BallDetector();
            detector.SetProfile(SphereProfile());

            detector.Detect(BlackFrame());
            var act = () => detector.Detect(CircleFrame(cx: 160, cy: 2, radius: 8));
            act.Should().NotThrow("ball partially off the top edge should not crash");
        }

        [Fact]
        public void BallAtRightEdge_DoesNotThrow()
        {
            var detector = new BallDetector();
            detector.SetProfile(SphereProfile());

            detector.Detect(BlackFrame());
            var act = () => detector.Detect(CircleFrame(cx: W - 2, cy: 120, radius: 8));
            act.Should().NotThrow("ball partially off the right edge should not crash");
        }

        [Fact]
        public void BallAtBottomLeftCorner_DoesNotThrow()
        {
            var detector = new BallDetector();
            detector.SetProfile(SphereProfile());

            detector.Detect(BlackFrame());
            var act = () => detector.Detect(CircleFrame(cx: 2, cy: H - 2, radius: 8));
            act.Should().NotThrow("ball at corner should not crash");
        }

        [Fact]
        public void BallFullyClipped_OffScreen_ReturnsNotFound()
        {
            // Circle whose centre is outside the frame — nothing visible → not found.
            var detector = new BallDetector();
            detector.SetProfile(SphereProfile());

            detector.Detect(BlackFrame());
            // Centre at (-50, 120) — entirely off screen
            var result = detector.Detect(CircleFrame(cx: -50, cy: 120, radius: 8));
            result.Found.Should().BeFalse("a fully off-screen blob should not be detected");
        }
    }
}
